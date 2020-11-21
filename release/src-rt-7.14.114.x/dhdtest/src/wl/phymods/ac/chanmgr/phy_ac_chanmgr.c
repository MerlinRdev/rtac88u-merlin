/*
 * ACPHY Channel Manager module implementation
 *
 * Broadcom Proprietary and Confidential. Copyright (C) 2016,
 * All Rights Reserved.
 * 
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom.
 *
 *
 * <<Broadcom-WL-IPTag/Proprietary:>>
 *
 * $Id$
 */

#include <phy_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>
#include <phy_dbg.h>
#include <phy_mem.h>
#include <qmath.h>
#include "phy_type_chanmgr.h"
#include <phy_ac.h>
#include <phy_ac_chanmgr.h>
#include <phy_ac_noise.h>
#include <phy_rxgcrs_api.h>

/* ************************ */
/* Modules used by this module */
/* ************************ */
#include <wlc_radioreg_20691.h>
#include <wlc_radioreg_20693.h>
#include <wlc_phy_radio.h>
#include <wlc_phy_shim.h>
#include <wlc_phyreg_ac.h>
#include <wlc_phytbl_20691.h>
#include <wlc_phytbl_20693.h>
#include <wlc_phytbl_ac.h>
#include <wlc_phy_ac_gains.h>

#include <hndpmu.h>
#include <sbchipc.h>
#include <phy_utils_reg.h>
#include <phy_utils_channel.h>
#include <phy_utils_math.h>
#include <phy_utils_var.h>
#include <phy_ac_info.h>


/* module private states */
struct phy_ac_chanmgr_info {
	phy_info_t			*pi;
	phy_ac_info_t		*aci;
	phy_chanmgr_info_t	*cmn_info;
};

typedef struct {
	uint16 gi;
	uint16 g21;
	uint16 g32;
	uint16 g43;
	uint16 r12;
	uint16 r34;
	uint16 gff1;
	uint16 gff2;
	uint16 gff3;
	uint16 gff4;
	uint16 g11;
	uint16 ri3;
	uint16 g54;
	uint16 g65;
} tiny_adc_tuning_array_t;

/* 20693 Radio functions */
/* local functions */

/* register phy type specific implementation */
phy_ac_chanmgr_info_t *
BCMATTACHFN(phy_ac_chanmgr_register_impl)(phy_info_t *pi, phy_ac_info_t *aci,
	phy_chanmgr_info_t *cmn_info)
{
	phy_ac_chanmgr_info_t *ac_info;
	phy_type_chanmgr_fns_t fns;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* allocate all storage together */
	if ((ac_info = phy_malloc(pi, sizeof(phy_ac_chanmgr_info_t))) == NULL) {
		PHY_ERROR(("%s: phy_malloc failed\n", __FUNCTION__));
		goto fail;
	}
	ac_info->pi = pi;
	ac_info->aci = aci;
	ac_info->cmn_info = cmn_info;

	/* register PHY type specific implementation */
	bzero(&fns, sizeof(fns));
	fns.ctx = ac_info;

	if (phy_chanmgr_register_impl(cmn_info, &fns) != BCME_OK) {
		PHY_ERROR(("%s: phy_chanmgr_register_impl failed\n", __FUNCTION__));
		goto fail;
	}

	return ac_info;

	/* error handling */
fail:
	if (ac_info != NULL)
		phy_mfree(pi, ac_info, sizeof(phy_ac_chanmgr_info_t));
	return NULL;
}

void
BCMATTACHFN(phy_ac_chanmgr_unregister_impl)(phy_ac_chanmgr_info_t *ac_info)
{
	phy_info_t *pi;
	phy_chanmgr_info_t *cmn_info;

	ASSERT(ac_info);
	pi = ac_info->pi;
	cmn_info = ac_info->cmn_info;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* unregister from common */
	phy_chanmgr_unregister_impl(cmn_info);

	phy_mfree(pi, ac_info, sizeof(phy_ac_chanmgr_info_t));
}

/* ********************************************* */
/*				Internal Definitions					*/
/* ********************************************* */
#define TXMAC_IFHOLDOFF_DEFAULT		0x12	/* 9.0us */
#define TXMAC_MACDELAY_DEFAULT		0x2a8	/* 8.5us */

#define ACPHY_VCO_2P5V	1
#define ACPHY_VCO_1P35V	0

#define WLC_TINY_GI_MULT_P12		4096U
#define WLC_TINY_GI_MULT_TWEAK_P12	4096U
#define WLC_TINY_GI_MULT			WLC_TINY_GI_MULT_P12

typedef enum {
	ACPHY_LP_CHIP_LVL_OPT,
	ACPHY_LP_PHY_LVL_OPT,
	ACPHY_LP_RADIO_LVL_OPT
} acphy_lp_opt_levels_t;

typedef struct _chan_info_common {
	uint16 chan;		/* channel number */
	uint16 freq;		/* in Mhz */
} chan_info_common_t;

static const uint16 qt_rfseq_val1[] = {0x8b5, 0x8b5, 0x8b5};
static const uint16 qt_rfseq_val2[] = {0x0, 0x0, 0x0};
static const uint16 rfseq_reset2rx_dly[] = {12, 2, 2, 4, 4, 6, 1, 4, 1, 2, 1, 1, 1, 1, 1, 1};
static const uint16 rfseq_updl_lpf_hpc_ml[] = {0x0aaa, 0x0aaa};
static const uint16 rfseq_updl_tia_hpc_ml[] = {0x0222, 0x0222};
static const uint16 rfseq_reset2rx_cmd[] = {0x4, 0x3, 0x6, 0x5, 0x2, 0x1, 0x8,
            0x2a, 0x2b, 0xf, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};

uint16 const rfseq_rx2tx_cmd[] =
{0x0, 0x1, 0x2, 0x8, 0x5, 0x0, 0x6, 0x3, 0xf, 0x4, 0x0, 0x35, 0xf, 0x0, 0x36, 0x1f};
static uint16 rfseq_rx2tx_dly_epa1_20[] =
	{0x8, 0x6, 0x4, 0x4, 0x6, 0x2, 0x10, 60, 0x2, 0x5, 0x1, 0x4, 0xe4, 0xfa, 0x2, 0x1};
static uint16 rfseq_rx2tx_dly_epa1_40[] =
	{0x8, 0x6, 0x4, 0x4, 0x6, 0x2, 0x10, 30, 0x2, 0xd, 0x1, 0x4, 0xfa, 0xfa, 0x2, 0x1};
static uint16 rfseq_rx2tx_dly_epa1_80[] =
	{0x8, 0x6, 0x4, 0x4, 0x6, 0x2, 0x10, 20, 0x2, 0x17, 0x1, 0x4, 0xfa, 0xfa, 0x2, 0x1};
static uint16 rfseq_rx2tx_cmd_withtssisleep[] =
{0x0000, 0x0001, 0x0005, 0x0008, 0x0002, 0x0006, 0x0003, 0x000f, 0x0004, 0x0035,
0x000f, 0x0000, 0x0000, 0x0036, 0x0080, 0x001f};
static uint16 rfseq_rx2tx_dly_withtssisleep[] =
{0x0008, 0x0006, 0x0006, 0x0004, 0x0006, 0x0010, 0x0026, 0x0002, 0x0006, 0x0004,
0x00ff, 0x00ff, 0x00a8, 0x0004, 0x0001, 0x0001};
static uint16 rfseq_rx2tx_cmd_rev15_ipa[] =
        {0x0, 0x1, 0x5, 0x8, 0x2, 0x6, 0x35, 0x3, 0xf, 0x4, 0x0f, 0x0, 0x0, 0x36, 0x00, 0x1f};
static uint16 rfseq_rx2tx_cmd_rev15_ipa_withtssisleep[] =
        {0x0, 0x1, 0x5, 0x8, 0x2, 0x6, 0x35, 0x3, 0xf, 0x4, 0x0f, 0x0, 0x0, 0x36, 0x80, 0x1f};
static uint16 rfseq_rx2tx_dly_rev15_ipa20[] =
	{0x8, 0x6, 0x6, 0x4, 0x6, 0x10, 40, 0x26, 0x2, 0x6, 0xff, 0xff, 0x56, 0x4, 0x1, 0x1};
static uint16 rfseq_rx2tx_dly_rev15_ipa40[] =
	{0x8, 0x6, 0x6, 0x4, 0x6, 0x10, 16, 0x26, 0x2, 0x6, 0xff, 0xff, 0x6e, 0x4, 0x1, 0x1};

static const uint16 rfseq_tx2rx_cmd[] =
{0x4, 0x3, 0x6, 0x5, 0x0, 0x2, 0x1, 0x8, 0x2a, 0xf, 0x0, 0xf, 0x2b, 0x1f, 0x1f, 0x1f};

static const uint16 rf_updh_cmd_clamp[] = {0x2a, 0x07, 0x0a, 0x00, 0x08, 0x2b, 0x1f, 0x1f};
static const uint16 rf_updh_dly_clamp[] = {0x01, 0x02, 0x02, 0x02, 0x10, 0x01, 0x01, 0x01};
static const uint16 rf_updl_cmd_clamp[] = {0x2a, 0x07, 0x08, 0x0c, 0x0e, 0x2b, 0x1f, 0x1f};
static const uint16 rf_updl_dly_clamp[] = {0x01, 0x06, 0x12, 0x08, 0x10, 0x01, 0x01, 0x01};
static const uint16 rf_updu_cmd_clamp[] = {0x2a, 0x07, 0x08, 0x0e, 0x2b, 0x1f, 0x1f, 0x1f};
static const uint16 rf_updu_dly_clamp[] = {0x01, 0x06, 0x1e, 0x1c, 0x01, 0x01, 0x01, 0x01};

static const uint16 rf_updh_cmd_adcrst[] = {0x07, 0x0a, 0x00, 0x08, 0xb0, 0xb1, 0x1f, 0x1f};
static const uint16 rf_updh_dly_adcrst[] = {0x02, 0x02, 0x02, 0x01, 0x0a, 0x01, 0x01, 0x01};
static const uint16 rf_updl_cmd_adcrst[] = {0x07, 0x08, 0x0c, 0x0e, 0xb0, 0xb2, 0x1f, 0x1f};
static const uint16 rf_updl_dly_adcrst[] = {0x06, 0x12, 0x08, 0x01, 0x0a, 0x01, 0x01, 0x01};
static const uint16 rf_updu_cmd_adcrst[] = {0x07, 0x08, 0x0e, 0xb0, 0xb1, 0x1f, 0x1f, 0x1f};
static const uint16 rf_updu_dly_adcrst[] = {0x06, 0x1e, 0x1c, 0x0a, 0x01, 0x01, 0x01, 0x01};

/* Coefficients generated by 47xxtcl/rgphy/20691/ */
/* lpf_tx_coefficient_generator/filter_tx_tiny_generate_python_and_tcl.py */
static const uint16 lpf_g10[6][15] = {
	{1188, 1527, 1866, 2206, 2545, 2545, 1188, 1188,
	1188, 1188, 1188, 1188, 1188, 1188, 1188},
	{3300, 4242, 5185, 6128, 7071, 7071, 3300, 3300,
	3300, 3300, 3300, 3300, 3300, 3300, 3300},
	{16059, 16059, 16059, 17294, 18529, 18529, 9882,
	1976, 2470, 3088, 3953, 4941, 6176, 7906, 12353},
	{24088, 24088, 25941, 31500, 37059, 37059, 14823,
	2964, 3705, 4632, 5929, 7411, 9264, 11859, 18529},
	{29647, 32118, 34589, 42001, 49412, 49412, 19765,
	3705, 4941, 6176, 7411, 9882, 12353, 14823, 24706},
	{32941, 36236, 39530, 42824, 46118, 49412, 19765,
	4117, 4941, 6588, 8235, 9882, 13176, 16470, 26353}
};
static const uint16 lpf_g12[6][15] = {
	{1882, 1922, 1866, 1752, 1606, 1275, 2984, 14956,
	11880, 9436, 7495, 5954, 4729, 3756, 2370},
	{5230, 5341, 5185, 4868, 4461, 3544, 8289, 41544,
	33000, 26212, 20821, 16539, 13137, 10435, 6584},
	{24872, 19757, 15693, 13424, 11425, 9075, 24258, 24316,
	24144, 23972, 24374, 24201, 24029, 24432, 24086},
	{37309, 29635, 25351, 24452, 22850, 18151, 36388, 36474,
	36216, 35959, 36561, 36302, 36044, 36648, 36130},
	{44360, 38172, 32654, 31496, 29433, 23379, 46870, 44045,
	46648, 46318, 44150, 46759, 46428, 44254, 46538},
	{49288, 43066, 37319, 32113, 27471, 23379, 46870, 48939,
	46648, 49406, 49055, 46759, 49523, 49172, 49640}
};
static const uint16 lpf_g21[6][15] = {
	{1529, 1497, 1542, 1643, 1793, 2257, 965, 192, 242,
	305, 384, 483, 609, 766, 1215},
	{4249, 4160, 4285, 4565, 4981, 6270, 2681, 534, 673,
	847, 1067, 1343, 1691, 2129, 3375},
	{6135, 7723, 9723, 11367, 13356, 16814, 6290, 6275,
	6320, 6365, 6260, 6305, 6350, 6245, 6335},
	{9202, 11585, 13543, 14041, 15025, 18916, 9435, 9413,
	9480, 9548, 9391, 9458, 9525, 9368, 9503},
	{13760, 15990, 18693, 19380, 20738, 26108, 13023, 13858,
	13085, 13178, 13825, 13054, 13147, 13793, 13116},
	{22016, 25197, 29078, 33791, 39502, 46415, 23152, 22173,
	23262, 21964, 22121, 23207, 21912, 22068, 21860}
};
static const uint16 lpf_g11[6] = {994, 2763, 12353, 18529, 17470, 23293};
static const uint16 g_passive_rc_tx[6] = {62, 172, 772, 1158, 1544, 2058};
static const uint16 biases[6] = {24, 48, 96, 96, 128, 128};
static const int8 g_index1[15] = {0, 1, 2, 3, 4, 5, -2, -9, -8, -7, -6, -5, -4, -3, -1};

static const uint8 avvmid_set[25][5][6] = {{{2, 1, 2,   107, 150, 110},  /* pdet_id = 0 */
			       {2, 2, 1,   157, 153, 160},
			       {2, 2, 1,   157, 153, 161},
			       {2, 2, 0,   157, 153, 186},
			       {2, 2, 0,   157, 153, 187}},
			       {{1, 0, 1,   159, 174, 161},  /* pdet_id = 1 */
			       {1, 0, 1,   160, 185, 156},
			       {1, 0, 1,   163, 185, 162},
			       {1, 0, 1,   169, 187, 167},
			       {1, 0, 1,   152, 188, 160}},
			       {{1, 1, 1,   159, 166, 166},  /* pdet_id = 2 */
			       {2, 2, 4,   140, 151, 100},
			       {2, 2, 3,   143, 153, 116},
			       {2, 2, 2,   143, 153, 140},
			       {2, 2, 2,   145, 160, 154}},
			       {{1, 1, 2,   130, 131, 106},  /* pdet_id = 3 */
			       {1, 1, 2,   130, 131, 106},
			       {1, 1, 2,   128, 127, 97},
			       {0, 1, 3,   159, 137, 75},
			       {0, 0, 3,   164, 162, 76}},
			       {{1, 1, 1,   156, 160, 158},  /* pdet_id = 4 */
			       {1, 1, 1,   156, 160, 158},
			       {1, 1, 1,   156, 160, 158},
			       {1, 1, 1,   156, 160, 158},
			       {1, 1, 1,   156, 160, 158}},
			       {{2, 2, 2,   104, 108, 106},  /* pdet_id = 5 */
			       {2, 2, 2,   104, 108, 106},
			       {2, 2, 2,   104, 108, 106},
			       {2, 2, 2,   104, 108, 106},
			       {2, 2, 2,   104, 108, 106}},
			       {{2, 0, 2,   102, 170, 104},  /* pdet_id = 6 */
			       {3, 4, 3,    82, 102,  82},
			       {1, 3, 1,   134, 122, 136},
			       {1, 3, 1,   134, 124, 136},
			       {2, 3, 2,   104, 122, 108}},
			       {{0, 0, 0,   180, 180, 180},  /* pdet_id = 7 */
			       {0, 0, 0,   180, 180, 180},
			       {0, 0, 0,   180, 180, 180},
			       {0, 0, 0,   180, 180, 180},
			       {0, 0, 0,   180, 180, 180}},
			       {{2, 1, 2,   102, 138, 104},  /* pdet_id = 8 */
			       {3, 5, 3,    82, 100,  82},
			       {1, 4, 1,   134, 116, 136},
			       {1, 3, 1,   134, 136, 136},
			       {2, 3, 2,   104, 136, 108}},
			       {{3, 2, 3,    90, 106,  86},  /* pdet_id = 9 */
			       {3, 1, 3,    90, 158,  90},
			       {2, 1, 2,   114, 158, 112},
			       {2, 1, 1,   116, 158, 142},
			       {2, 1, 1,   116, 158, 142}},
			       {{2, 2, 2,   152, 156, 156},  /* pdet_id = 10 */
			       {2, 2, 2,   152, 156, 156},
			       {2, 2, 2,   152, 156, 156},
			       {2, 2, 2,   152, 156, 156},
			       {2, 2, 2,   152, 156, 156}},
			       {{1, 1, 1,   134, 134, 134},  /* pdet_id = 11 */
			       {1, 1, 1,   136, 136, 136},
			       {1, 1, 1,   136, 136, 136},
			       {1, 1, 1,   136, 136, 136},
			       {1, 1, 1,   136, 136, 136}},
			       {{3, 3, 3,    90,  92,  86},  /* pdet_id = 12 */
			       {3, 3, 3,    90,  86,  90},
			       {2, 3, 2,   114,  86, 112},
			       {2, 2, 1,   116, 109, 142},
			       {2, 2, 1,   116, 110, 142}},
			       {{2, 2, 2,   112, 114, 112},  /* pdet_id = 13 */
			       {2, 2, 2,   114, 114, 114},
			       {2, 2, 2,   114, 114, 114},
			       {2, 2, 2,   113, 114, 112},
			       {2, 2, 2,   113, 114, 112}},
			       {{1, 1, 1,   134, 134, 134},  /* pdet_id = 14 */
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168}},
			       {{0, 0, 0,   172, 172, 172},  /* pdet_id = 15 */
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168}},
			       {{3, 2, 3,    90, 106,  86},  /* pdet_id = 16 */
			       {3, 0, 3,    90, 186,  90},
			       {2, 0, 2,   114, 186, 112},
			       {2, 0, 1,   116, 186, 142},
			       {2, 0, 1,   116, 186, 142}},
			       {{4, 4, 4,   50,  45,  50},  /* pdet_id = 17 */
			       {3, 3, 3,    82,  82, 82},
			       {3, 3, 3,    82,  82, 82},
			       {3, 3, 3,    82,  82, 82},
			       {3, 3, 3,    82,  82, 82}},
			       {{5, 5, 5,   61,  61,  61},  /* pdet_id = 18 */
			       {2, 2, 2,   122, 122, 122},
			       {2, 2, 2,   122, 122, 122},
			       {2, 2, 2,   122, 122, 122},
			       {2, 2, 2,   122, 122, 122}},
			       {{2, 2, 2,  152, 156, 156},  /* pdet_id = 19 */
			       {1, 1, 1,   165, 165, 165},
			       {1, 1, 1,   160, 160, 160},
			       {1, 1, 1,   152, 150, 160},
			       {1, 1, 1,   152, 150, 160}},
                       {{3, 3, 3,  108, 108, 108},  /* pdet_id = 20 */
			       {1, 1, 1,   160, 160, 160},
			       {1, 1, 1,   160, 160, 160},
			       {1, 1, 1,   160, 160, 160},
			       {1, 1, 1,   160, 160, 160}},
                       {{2, 2, 2,  110, 110, 110},  /* pdet_id = 21 */
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168},
			       {0, 0, 0,   168, 168, 168}},
			       {{6, 6, 6,   40,  40,  40},  /* pdet_id = 22 */
			       {2, 2, 1,   115, 115, 142},
			       {1, 2, 1,   142, 115, 142},
			       {1, 1, 1,   142, 142, 142},
			       {1, 1, 1,   142, 142, 142}},
			       {{1, 1, 1,  156, 160, 158},  /* pdet_id = 23 */
			       {6, 6, 6,    47,  45,  48},
			       {1, 1, 1,   147, 146, 148},
			       {1, 1, 1,   146, 146, 152},
			       {1, 1, 1,   146, 146, 152}},
			       {{2, 2, 2,   120, 120, 120}, /* pdet_id =24 */
			       {2, 2, 2,   120, 120, 120},
			       {2, 2, 2,   120, 120, 120},
			       {2, 2, 2,   120, 120, 120},
			       {2, 2, 2,   120, 120, 120}}
};

static const uint8 avvmid_set1[16][5][2] = {
	{{1, 154}, {0, 168}, {0, 168}, {0, 168}, {0, 168}},  /* pdet_id = 0 */
	{{1, 145}, {1, 145}, {1, 145}, {1, 145}, {1, 145}},  /* pdet_id = 1 WLBGA */
	{{6,  76}, {1, 160}, {6,  76}, {6,  76}, {6,  76}},  /* pdet_id = 2 */
	{{1, 156}, {1, 152}, {1, 152}, {1, 152}, {1, 152}},  /* pdet_id = 3 */
	{{1, 152}, {1, 152}, {1, 152}, {1, 152}, {1, 152}},  /* pdet_id = 4 WLCSP */
	{{3, 100}, {3,  75}, {3,  75}, {3,  75}, {3,  75}},  /* pdet_id = 5 WLCSP TM */
	{{1, 152}, {0, 166}, {0, 166}, {0, 166}, {0, 166}},  /* pdet_id = 6 WLCSP HK */
	{{1, 145}, {3, 120}, {3, 120}, {3, 120}, {3, 125}},  /* pdet_id = 7 WLiPA */
	{{1, 145}, {1, 155}, {1, 155}, {1, 155}, {1, 155}},  /* pdet_id = 8 WLBGA C0 */
	{{1, 135}, {1, 165}, {1, 165}, {1, 165}, {1, 165}}   /* pdet_id = 9 WLBGA RR FEM */
};
static const uint8 avvmid_set2[16][5][4] = {
	{
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145}},  /* pdet_id = 0 */
	{
		{3, 3, 100, 100},
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145}},  /* pdet_id = 1 */
	{
		{4, 4,  95,  95},
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145},
		{1, 1, 145, 145}},  /* pdet_id = 2 */
	{
		{1, 1, 145, 145},
		{3, 3,  90,  90},
		{3, 3,  92,  92},
		{2, 3, 110,  90},
		{2, 3, 110,  93}}   /* pdet_id = 3 */
};

static const uint8 avvmid_set3[16][5][2] = {
	{{1, 115}, {2, 90}, {2, 90}, {2, 90}, {2, 90}},  /* pdet_id = 0 4345 TC */
	{{0, 131}, {0, 134}, {0, 134}, {0, 134}, {0, 134}},  /* pdet_id = 1 4345TC FCBGA EPA */
	{{4, 132}, {4, 127}, {4, 127}, {4, 127}, {4, 127}},  /* pdet_id = 2 4345A0 fcbusol */
	{{0, 150}, {2, 97}, {2, 97}, {2, 97}, {2, 97}},  /* pdet_id = 3 4345A0 fcpagb ipa */
};
static const uint8 avvmid_set4[1][5][4] = {
	{
		{2, 2, 130, 130},
		{2, 2, 130, 130},
		{2, 2, 130, 130},
		{2, 2, 130, 130},
		{2, 2, 130, 130}},  /* pdet_id = 0 */
};

uint16 const rfseq_majrev3_reset2rx_dly[] = {12, 2, 2, 4, 4, 6, 1, 4, 1, 2, 1, 1, 1, 1, 1, 1};

uint16 const rfseq_rx2tx_dly[] =
	{0x8, 0x6, 0x6, 0x4, 0x4, 0x2, 0x10, 0x26, 0x2, 0x5, 0x1, 0x4, 0xfa, 0xfa, 0x2, 0x1};
uint16 const tiny_rfseq_rx2tx_cmd[] =
	{0x42, 0x1, 0x2, 0x8, 0x5, 0x6, 0x3, 0xf, 0x4, 0x35, 0xf, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
uint16 const tiny_rfseq_rx2tx_dly[] =
	{0x8, 0x6, 0x6, 0x4, 0x4, 0x10, 0x26, 0x2, 0x5, 0x4, 0xFA, 0xFA, 0x1, 0x1, 0x1, 0x1};
uint16 const tiny_rfseq_rx2tx_tssi_sleep_cmd[] =
	{0x42, 0x1, 0x2, 0x8, 0x5, 0x6, 0x3, 0xf, 0x4, 0x35, 0xf, 0x00, 0x00, 0x36, 0x1f, 0x1f};
uint16 const tiny_rfseq_rx2tx_tssi_sleep_dly[] =
	{0x8, 0x6, 0x6, 0x4, 0x4, 0x10, 0x26, 0x2, 0x5, 0x4, 0xFA, 0xFA, 0x88, 0x1, 0x1, 0x1};
uint16 const tiny_rfseq_tx2rx_cmd[] =
	{0x4, 0x3, 0x6, 0x5, 0x85, 0x2, 0x1, 0x8, 0x2a, 0xf, 0x0, 0xf, 0x2b, 0x43, 0x1F};
uint16 const tiny_rfseq_tx2rx_dly[] =
	{0x8, 0x4, 0x2, 0x2, 0x1, 0x3, 0x4, 0x6, 0x4, 0x1, 0x2, 0x1, 0x1, 0x1, 0x1, 0x1};

/* tiny major rev4 RF Sequences : START */

/* Reset2Rx */
/* changing RF sequencer to add DCC reset */
static const uint16 rfseq_majrev4_reset2rx_cmd[] = {0x84, 0x4, 0x3, 0x6, 0x5, 0x2, 0x1, 0x8,
	0x2a, 0x2b, 0xf, 0x0, 0x0, 0x85, 0x41, 0x1f};
uint16 const rfseq_majrev4_reset2rx_dly[] = {10, 12, 2, 2, 4, 4, 6, 1, 4, 1, 2, 10, 1, 1, 1, 1};

/* Tx2Rx */
uint16 const rfseq_majrev4_tx2rx_cmd[] =
	{0x84, 0x4, 0x3, 0x6, 0x5, 0x85, 0x2, 0x1, 0x8, 0x2a, 0xf, 0x0, 0xf, 0x2b, 0x43, 0x1F};
uint16 const rfseq_majrev4_tx2rx_dly[] =
	{0x8, 0x8, 0x4, 0x2, 0x2, 0x1, 0x3, 0x4, 0x6, 0x4, 0x1, 0x2, 0x1, 0x1, 0x1, 0x1, 0x1};

/* Rx2Tx */
/* Refer to tiny_rfseq_rx2tx_cmd */

/* Rx2Tx -- Cal */
uint16 const rfseq_majrev4_rx2tx_cal_cmd[] =
	{0x84, 0x1, 0x2, 0x8, 0x5, 0x3d, 0x85, 0x6, 0x3, 0xf, 0x4, 0x3e, 0x35, 0xf, 0x36, 0x1f};
uint16 const rfseq_majrev4_rx2tx_cal_dly[] =
	{0x8, 0x6, 0x6, 0x4, 0x4, 0x2, 0x12, 0x10, 0x26, 0x2, 0x5, 0x1, 0x4, 0xfa, 0x2, 0x1};
/* tiny major rev4 RF Sequences : END */

/* Channel smoothing MTE filter image */
#define CHANSMTH_FLTR_LENGTH 64
static CONST uint16 acphy_Smth_tbl_4349[] = {
	0x4a5c, 0xdba7, 0x1672,
	0xb167, 0x742d, 0xa5ca,
	0x4afe, 0x4aa6, 0x14f3,
	0x4176, 0x6f25, 0xa75a,
	0x7aca, 0xeca4, 0x1e94,
	0xf177, 0x4e27, 0xa7fa,
	0x0b46, 0xcead, 0x270c,
	0x3169, 0x4f1d, 0xa70b,
	0xda4e, 0xcb35, 0x1431,
	0xd1d2, 0x572e, 0xae6b,
	0x8a4b, 0x68bc, 0x1f62,
	0x81f6, 0xc826, 0xa4bb,
	0x2add, 0x6b37, 0x1d42,
	0xcaff, 0xdd9e, 0x0c6a,
	0xd0c6, 0xecad, 0xaff9,
	0xbad8, 0xe69d, 0x173a,
	0x20d1, 0xf5b7, 0xa579,
	0x6b71, 0xdb9c, 0x156a,
	0x60d6, 0xf345, 0xa6f9,
	0x0b42, 0xc6a6, 0x1f5a,
	0xb0d4, 0xe22e, 0x9c19,
	0x4bc4, 0x5aaf, 0x1c6b,
	0xc0cc, 0xc326, 0x9c49,
	0x1cf1, 0xddb7, 0x243b,
	0xe17a, 0xe21c, 0xa75a,
	0x6a50, 0xcb35, 0x1441,
	0xb1d3, 0x5d2e, 0xae4b,
	0x8a4b, 0x67bd, 0x1f72,
	0x71f7, 0xd826, 0xa4bb,
	0xfade, 0x6b36, 0x1d42,
	0xe153, 0xcf96, 0x0fc8,
	0xf0fc, 0x6e8c, 0x1539,
	0xd1fd, 0x7d94, 0x0da9,
	0xd047, 0xc08c, 0x1578,
	0x41c9, 0x4c9d, 0x1679,
	0xe043, 0x7696, 0x1459,
	0xf2f7, 0x7faf, 0x1d1a,
	0xd0e4, 0x4c9c, 0x1c49,
	0xe37e, 0xca9c, 0x1782,
	0x31ff, 0x7ba4, 0x2f1a,
	0xd243, 0xe69d, 0x16ba,
	0x616b, 0xddae, 0x2439,
	0xdc69, 0x46ae, 0x1fb2,
	0xf0c9, 0x5a97, 0x0658,
	0xa065, 0x7f85, 0x0c99,
	0xd174, 0x4a95, 0x0508,
	0x2074, 0xce86, 0x0d38,
	0xb152, 0xea9f, 0x0f08,
	0xd078, 0xd785, 0x0d38,
	0x71e3, 0xc29c, 0x0c48,
	0xc06e, 0xd684, 0x0c88,
	0x4262, 0x42a4, 0x1439,
	0x0058, 0xc78e, 0x1658,
	0xb2c4, 0x5cb5, 0x25da,
	0x60f5, 0x5694, 0x1dd9,
	0x02c6, 0xc39d, 0x1792,
	0x61ff, 0x7ba4, 0x2f3a,
	0xf246, 0xee9d, 0x16ca,
	0xe16c, 0xdfae, 0x2469,
	0x2c4d, 0x44af, 0x1fd2,
	0x0bcd, 0x4faf, 0x1c5b,
	0x30cb, 0x7e27, 0x9c6a,
	0xec42, 0xd3b6, 0x243b,
	0x0179, 0xd81d, 0xa77a
 };
static CONST uint16 acphy_Smth_tbl_tiny[] = {
	0x5fd2,	0x16fc,	0x0ce0,
	0x60ce,	0xc501,	0xfd2f,
	0xefe0,	0x09fc,	0x09e0,
	0x90eb,	0xc802,	0xfc5f,
	0xcfed,	0x01fd,	0x0690,
	0xf0ed,	0xd903,	0xfc0f,
	0xcff7,	0xfefe,	0x037f,
	0x30d2,	0xf605,	0xfc7f,
	0xbfd8,	0x4b00,	0x0860,
	0xb052,	0xf501,	0xfe6f,
	0xbfda,	0x33ff,	0x0750,
	0x3075,	0xfb03,	0xfdaf,
	0xefe8,	0x3500,	0x0530,
	0x4fe0,	0xe8f9,	0x119f,
	0x8119,	0x94fe,	0xfe0f,
	0x5fea,	0xe6fa,	0x0e5f,
	0x1142,	0x8aff,	0xfd4f,
	0xaff1,	0xe9fb,	0x0acf,
	0x2156,	0x8d00,	0xfc8f,
	0xfff7,	0xeefc,	0x075f,
	0xa151,	0x9d01,	0xfbef,
	0x2ffb,	0xf4fe,	0x045f,
	0x612f,	0xbd03,	0xfbbf,
	0x1ffe,	0xfaff,	0x021f,
	0xe0f4,	0xe704,	0xfc5f,
	0xafd8,	0x4b00,	0x0880,
	0xa052,	0xf401,	0xfe7f,
	0xafda,	0x33ff,	0x0770,
	0x3077,	0xfa03,	0xfdaf,
	0xdfe8,	0x3500,	0x0540,
	0x30a5,	0xc5f2,	0x1f1e,
	0x51f1,	0x23ec,	0x0a5f,
	0x607c,	0x06f6,	0x167f,
	0xb236,	0xffec,	0x0ade,
	0xc049,	0x67fa,	0x0cdf,
	0x4214,	0x13f2,	0x089f,
	0x001d,	0xc0fe,	0x051f,
	0x2191,	0x68fb,	0x044f,
	0x100f,	0x12fb,	0x0ef0,
	0xe07f,	0xc2fd,	0x01cf,
	0x2021,	0xe6fa,	0x0d5f,
	0x60d5,	0xa2fe,	0x021f,
	0x4ffc,	0x22fe,	0x07b0,
	0x2125,	0x2cf0,	0x32bd,
	0xc32b,	0x02d2,	0x125f,
	0x50e8,	0xb0f4,	0x27ed,
	0xe3a0,	0xc7ce,	0x14be,
	0x40a3,	0x57f8,	0x1bee,
	0x43bd,	0xa3d1,	0x14ce,
	0x8062,	0xf9fb,	0x10ee,
	0x3370,	0xa7da,	0x11fe,
	0xe030,	0x7cfd,	0x085f,
	0x12c1,	0xe2e8,	0x0c9e,
	0x4010,	0xd1ff,	0x02ef,
	0x41d4,	0x54f7,	0x05df,
	0xf011,	0x10fa,	0x0f10,
	0xd07f,	0xc2fd,	0x01cf,
	0x1023,	0xe4fa,	0x0d7f,
	0x40d7,	0xa1fe,	0x023f,
	0x3ffd,	0x22fe,	0x07c0,
	0x3ffb,	0xf6fe,	0x044f,
	0x912a,	0xc103,	0xfbaf,
	0x2ffd,	0xfaff,	0x021f,
	0xf0f1,	0xe904,	0xfc4f,
};

/* China 40M Spur WAR */
static const uint16 resamp_cnwar_5270[] = {0x4bda, 0x0038, 0x10e0, 0x4bda, 0x0038, 0x10e0,
0xed0e, 0x0068, 0xed0e, 0x0068};
static const uint16 resamp_cnwar_5310[] = {0x0000, 0x00d8, 0x0b40, 0x0000, 0x00d8, 0x0b40,
0x6c79, 0x0045, 0x6c79, 0x0045};

/* TIA LUT tables to be used in wlc_tiny_tia_config() */
static const uint8 tiaRC_tiny_8b_20[]= { /* LUT 0--51 (20 MHz) */
	0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff,
	0xb7, 0xb5, 0x97, 0x81, 0xe5, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x14, 0x1d, 0x28, 0x34, 0x34, 0x34,
	0x34, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x40,
	0x5b, 0x6c, 0x80, 0x80
};

static const uint8 tiaRC_tiny_8b_80[]= { /* LUT 0--51 (80 MHz) */
	0xc2, 0x86, 0x5d, 0xff, 0xbe, 0x88, 0x5f, 0x43,
	0x37, 0x2e, 0x26, 0x20, 0x39, 0x00, 0x00, 0x00,
	0x0b, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0d,
	0x0d, 0x07, 0x00, 0x00, 0x00, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00,
	0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x4c,
	0x5b, 0x6c, 0x80, 0x80
};

static const uint16 tiaRC_tiny_16b_80[]= { /* LUT 52--82 (80 MHz) */
	0x0000, 0x0000, 0x0000, 0x016b, 0x0100, 0x00b6, 0x0080, 0x005a,
	0x0040, 0x002d, 0x0020, 0x0017, 0x0000, 0x0100, 0x00b5, 0x0080,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0080, 0x0007, 0x1ff8, 0xf500, 0x00ff
};

static const uint8 tiaRC_tiny_8b_40[]= { /* LUT 0--51 (40 MHz) */
	0xff, 0xe1, 0xb9, 0x81, 0x5d, 0xff, 0xad, 0x82,
	0x6d, 0x5d, 0x4c, 0x41, 0x73, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x16, 0x16, 0x1b, 0x1d, 0x1d, 0x1f,
	0x20, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x4c,
	0x5b, 0x6c, 0x80, 0x80
};

static const uint16 tiaRC_tiny_16b_20[]= { /* LUT 52--82 (20 MHz) */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00b5, 0x0080, 0x005a,
	0x0040, 0x002d, 0x0020, 0x0017, 0x0000, 0x0100, 0x00b5, 0x0080,
	0x005b, 0x0040, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0080, 0x001f, 0x1fe0, 0xf500, 0x00ff
};

static const uint16 *tiaRC_tiny_16b_40 = tiaRC_tiny_16b_20;  /* LUT 52--82 (40 MHz) */
static void wlc_phy_set_noise_var_shaping_acphy(phy_info_t *pi,
	uint8 noise_var[][ACPHY_SPURWAR_NV_NTONES],	int8 *tone_id, uint8 *core_nv);
static void chanspec_setup_papr(phy_info_t *pi,
	int8 papr_final_clipping, int8 papr_final_scaling);
static void wlc_phy_spurwar_nvshp_acphy(phy_info_t *pi, bool bw_chg,
	bool spurwar, bool nvshp);
static void wlc_phy_radio20693_set_channel_bw(phy_info_t *pi,
	radio_20693_adc_modes_t adc_mode, uint8 core);
static void wlc_phy_radio20693_adc_config_overrides(phy_info_t *pi,
	radio_20693_adc_modes_t adc_mode, uint8 core);
static void
wlc_phy_wltx_word_get(phy_info_t *pi, uint8 band, uint32 swctrlmap_wltx,
	uint32 swctrlmap_wltx_ext, uint32 *swctrlword,	uint32 *swctrlwordext);
static void wlc_phy_write_rx_farrow_acphy(phy_info_t *pi, chanspec_t chanspec);
static void wlc_phy_radio20691_4345_vco_opt(phy_info_t *pi, uint8 vco_mode);
static void wlc_phy_enable_pavref_war(phy_info_t *pi);
static void wlc_phy_radio_vco_opt(phy_info_t *pi, uint8 vco_mode);
static int wlc_tiny_sigdel_fast_mult(int raw_p8, int mult_p12, int max_val, int rshift);
static void wlc_tiny_adc_setup_fast(phy_info_t *pi, tiny_adc_tuning_array_t *gvalues, uint8 core);
static void wlc_phy_radio20691_afecal(phy_info_t *pi);
static void wlc_tiny_sigdel_fast_tune(phy_info_t *pi, int g_mult_raw_p12,
	tiny_adc_tuning_array_t *gvalues);

#ifndef WL_FDSS_DISABLED
static void wlc_phy_fdss_init(phy_info_t *pi);
static void wlc_phy_set_fdss_table(phy_info_t *pi);
#endif

static void wlc_phy_papd_set_rfpwrlut_tiny(phy_info_t *pi);
static void wlc_acphy_load_4349_specific_tbls(phy_info_t *pi);
static void wlc_acphy_load_radiocrisscross_phyovr_mode(phy_info_t *pi);
static void wlc_acphy_load_logen_tbl(phy_info_t *pi);
static void wlc_phy_set_regtbl_on_band_change_acphy_20693(phy_info_t *pi);
static void wlc_phy_load_channel_smoothing_tiny(phy_info_t *pi);
static void wlc_phy_set_reg_on_reset_acphy_20693(phy_info_t *pi);
static void wlc_phy_radio20691_xtal_tune(phy_info_t *pi);
static void wlc_phy_set_tbl_on_reset_acphy(phy_info_t *pi);
static void wlc_phy_set_regtbl_on_band_change_acphy(phy_info_t *pi);
static void wlc_phy_set_regtbl_on_bw_change_acphy(phy_info_t *pi);
static void chanspec_setup_regtbl_on_chan_change(phy_info_t *pi);
static void wlc_phy_set_sfo_on_chan_change_acphy(phy_info_t *pi, uint8 ch);
static void wlc_phy_write_sfo_params_acphy(phy_info_t *pi, const uint16 *val_ptr);
static void wlc_phy_write_sfo_80p80_params_acphy(phy_info_t *pi, const uint16 *val_ptr);
static void wlc_phy_radio2069_afecal(phy_info_t *pi);
static void wlc_2069_rfpll_150khz(phy_info_t *pi);
static void wlc_phy_2069_4335_set_ovrds(phy_info_t *pi);
static void wlc_phy_2069_4350_set_ovrds(phy_info_t *pi);
static void acphy_set_lpmode(phy_info_t *pi, acphy_lp_opt_levels_t lp_opt_lvl);
static void wlc_phy_set_reg_on_reset_acphy_20691(phy_info_t *pi);
static void wlc_phy_set_regtbl_on_femctrl(phy_info_t *pi);
static void acphy_load_txv_for_spexp(phy_info_t *pi);
static void wlc_phy_cfg_energydrop_timeout(phy_info_t *pi);
static void wlc_phy_set_regtbl_on_band_change_acphy_20691(phy_info_t *pi);
static void wlc_phy_set_reg_on_bw_change_acphy(phy_info_t *pi);
static void wlc_phy_set_pdet_on_reset_acphy(phy_info_t *pi);
static void wlc_phy_set_tx_iir_coeffs(phy_info_t *pi, bool cck, uint8 filter_type);
static void wlc_phy_radio2069_afecal_invert(phy_info_t *pi);

static void wlc_phy_write_regtbl_fc3_sub0(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc3_sub1(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc3_sub2(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc3_sub3(phy_info_t *pi);
static INLINE void wlc_phy_write_regtbl_fc3(phy_info_t *pi, phy_info_acphy_t *pi_ac);
static void wlc_phy_write_regtbl_fc4_sub0(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc4_sub1(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc4_sub2(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc4_sub34(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc4_sub5(phy_info_t *pi);
static INLINE void wlc_phy_write_regtbl_fc4(phy_info_t *pi, phy_info_acphy_t *pi_ac);
static void wlc_phy_write_regtbl_fc10_sub0(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc10_sub1(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc10_sub2(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc10_sub3(phy_info_t *pi);
static void wlc_phy_write_regtbl_fc10_sub4(phy_info_t *pi);
static INLINE void wlc_phy_write_regtbl_fc10(phy_info_t *pi, phy_info_acphy_t *pi_ac);
static void wlc_phy_tx_gm_gain_boost(phy_info_t *pi);
static void wlc_phy_radio2069_4335C0_vco_opt(phy_info_t *pi, uint8 vco_mode);
static void wlc_tiny_sigdel_slow_tune(phy_info_t *pi, int g_mult_raw_p12,
	tiny_adc_tuning_array_t *gvalues, uint8 bw);
static void wlc_tiny_adc_setup_slow(phy_info_t *pi,
tiny_adc_tuning_array_t *gvalues, uint8 bw, uint8 core);
static void wlc_tiny_tia_config(phy_info_t *pi, uint8 core);
static void wlc_phy_write_rx_farrow_pre_tiny(phy_info_t *pi, chan_info_rx_farrow *rx_farrow,
	chanspec_t chanspec);
static uint16 wlc_phy_femctrlout_get_val(uint32 val_ext, uint32 val, uint32 MASK);
static int wlc_tiny_sigdel_wrap(int prod, int max_val);
static void wlc_phy_set_reg_on_reset_acphy(phy_info_t *pi);
static void wlc_phy_set_analog_tx_lpf(phy_info_t *pi, uint16 mode_mask, int bq0_bw, int bq1_bw,
	int rc_bw, int gmult, int gmult_rc, int core_num);
static void wlc_phy_set_tx_afe_dacbuf_cap(phy_info_t *pi, uint16 mode_mask, int dacbuf_cap,
	int dacbuf_fixed_cap, int core_num);
static void wlc_phy_set_analog_rx_lpf(phy_info_t *pi, uint8 mode_mask, int bq0_bw, int bq1_bw,
	int rc_bw, int gmult, int gmult_rc, int core_num);
#ifndef ACPHY_1X1_ONLY
static void wlc_phy_write_tx_farrow_acphy(phy_info_t *pi, chanspec_t chanspec);
#endif
static void wlc_phy_radio20693_set_reset_table_bits(phy_info_t *pi, uint16 tbl_id, uint16 offset,
	uint16 start, uint16 end, uint16 val, uint8 tblwidth);
static void
wlc_phy_radio20693_adc_dac_setup(phy_info_t *pi, radio_20693_adc_modes_t adc_mode, uint8 core);
static void wlc_phy_radio20693_setup_crisscorss_ovr(phy_info_t *pi, uint8 core);
static void wlc_acphy_dyn_papd_cfg_20693(phy_info_t *pi);
static void wlc_phy_set_bias_ipa_as_epa_acphy_20693(phy_info_t *pi, uint8 core);
static void wlc_phy_set_phyctl_in_phymode_acphy(phy_info_t *pi);


/* chanspec handle */
typedef void (*chanspec_module_t)(phy_info_t *pi);
chanspec_module_t * BCMRAMFN(get_chanspec_module_list)(void);

/* setup */
static void chanspec_setup(phy_info_t *pi);
static void chanspec_setup_phy(phy_info_t *pi);
static void chanspec_setup_cmn(phy_info_t *pi);

/* tune */
static void chanspec_tune_phy(phy_info_t *pi);
static void chanspec_tune_txpath(phy_info_t *pi);
static void chanspec_tune_rxpath(phy_info_t *pi);

/* wars & features */
static void chanspec_fw_enab(phy_info_t *pi);

/* cleanup */
static void chanspec_cleanup(phy_info_t *pi);

/* other helper functions */
static void chanspec_bbpll_parr(phy_info_t *pi, uint32 *bbpll_parr_in, bool state);
static void chanspec_regtbl_fc_from_nvram(phy_info_t *pi);
static void chanspec_clr_olpc_dbg_mode(phy_info_t *pi);
static void chanspec_setup_hirssi_ucode_cap(phy_info_t *pi);
static void chanspec_sparereg_war(phy_info_t *pi);
static void chanspec_prefcbs_init(phy_info_t *pi);
static bool chanspec_papr_enable(phy_info_t *pi);

/* phy setups */
static void chanspec_setup_phy_ACMAJORREV_5(phy_info_t *pi);
static void chanspec_setup_phy_ACMAJORREV_4(phy_info_t *pi);
static void chanspec_setup_phy_ACMAJORREV_3(phy_info_t *pi);
static void chanspec_setup_phy_ACMAJORREV_2(phy_info_t *pi);
static void chanspec_setup_phy_ACMAJORREV_1(phy_info_t *pi);
static void chanspec_setup_phy_ACMAJORREV_0(phy_info_t *pi);

/* phy tunables */
static void chanspec_tune_phy_ACMAJORREV_5(phy_info_t *pi);
static void chanspec_tune_phy_ACMAJORREV_4(phy_info_t *pi);
static void chanspec_tune_phy_ACMAJORREV_3(phy_info_t *pi);
static void chanspec_tune_phy_ACMAJORREV_2(phy_info_t *pi);
static void chanspec_tune_phy_ACMAJORREV_1(phy_info_t *pi);
static void chanspec_tune_phy_ACMAJORREV_0(phy_info_t *pi);

chanspec_module_t chanspec_module_list[] = {
	chanspec_setup,
	chanspec_setup_radio,
	chanspec_setup_phy,
	chanspec_setup_cmn,
	chanspec_setup_rxgcrs,
	chanspec_tune_radio,
	chanspec_tune_phy,
	chanspec_tune_txpath,
	chanspec_tune_rxpath,
	chanspec_fw_enab,
	chanspec_cleanup,
	NULL
};

chanspec_module_t *
BCMRAMFN(get_chanspec_module_list)(void)
{
	return chanspec_module_list;
}

static void
wlc_phy_config_bias_settings_20693(phy_info_t *pi)
{
	uint8 core;

	FOREACH_CORE(pi, core) {

		MOD_RADIO_REG_20693(pi, TRSW2G_CFG1, core, trsw2g_pu, 0);
		MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_trsw2g_pu, 1);
		MOD_RADIO_REG_20693(pi, TRSW2G_CFG1, core, trsw2g_bias_pu, 0);
		MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_trsw2g_bias_pu, 1);
		MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_mx2g_idac_bbdc, 1);
		MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_mx5g_idac_bbdc, 1);
		MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_pad5g_idac_pmos, 1);
		MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_pad5g_idac_gm, 1);
		MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR3, core, ovr_pa5g_bias_filter_main, 1);
	}
}

static void
wlc_phy_set_noise_var_shaping_acphy(phy_info_t *pi, uint8 noise_var[][ACPHY_SPURWAR_NV_NTONES],
                                             int8 *tone_id, uint8 *core_nv)
{
	uint8 i;

	/* Starting offset for nvshp */
	i = ACPHY_NV_NTONES_OFFSET;

	/* 4335C0 */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
		if (!PHY_ILNA(pi)) {
			if (CHSPEC_IS80(pi->radio_chanspec)) {
				static const int8 tone_id_def[] = {-123, -122, -121, -120,
				                                   -119, -118, -117, -116,
				                                   -115, -114, -113, -112,
				                                    112,  113,  114,  115,
				                                    116,  117,  118,  119,
				                                    120,  121,  122,  123};
				static const uint8 noise_var_def[] = {0xF8, 0xF8, 0xF8, 0xF8,
				                                      0xF8, 0xF8, 0xF8, 0xF8,
				                                      0xFA, 0xFA, 0xFC, 0xFE,
				                                      0xFE, 0xFC, 0xFA, 0xFA,
				                                      0xF8, 0xF8, 0xF8, 0xF8,
				                                      0xF8, 0xF8, 0xF8, 0xF8};
				memcpy((tone_id + i), tone_id_def, sizeof(int8)*ACPHY_NV_NTONES);
				memcpy((noise_var[PHY_CORE_0] + i), noise_var_def,
				        sizeof(uint8)*ACPHY_NV_NTONES);
				*core_nv = 1; /* core 0 */
				PHY_INFORM(("wlc_phy_set_noise_var_shaping_acphy:"
				            "applying noise_var shaping for BW 80MHz\n"));
			}
		}
	}
}

/**
 * Whenever the transmit power is less than a certain value, lower PA power consumption can be
 * achieved by selecting lower PA linearity. The VLIN signal towards the FEM is configured to
 * either be driven by the FEM control table or by a chip internal VLIN signal.
 */
void wlc_phy_vlin_en_acphy(phy_info_t *pi)
{
	uint8 band2g_idx, core;
	uint8 stall_val;
	int16 idle_tssi[PHY_CORE_MAX];
	uint16 adj_tssi1[PHY_CORE_MAX];
	uint16 adj_tssi2[PHY_CORE_MAX], adj_tssi3[PHY_CORE_MAX];
	int16 tone_tssi1[PHY_CORE_MAX];
	int16 tone_tssi2[PHY_CORE_MAX], tone_tssi3[PHY_CORE_MAX];
	int16 a1 = 0, b0 = 0, b1 = 0;
	uint8 pwr1, pwr2, pwr3;
	uint8 txidx1 = 40, txidx2 = 90, txidx3;
	struct _orig_reg_vals {
		uint8 core;
		uint16 orig_OVR3;
		uint16 orig_auxpga_cfg1;
		uint16 orig_auxpga_vmid;
		uint16 orig_iqcal_cfg1;
		uint16 orig_tx5g_tssi;
		uint16 orig_pa2g_tssi;
		uint16 orig_RfctrlIntc;
		uint16 orig_RfctrlOverrideRxPus;
		uint16 orig_RfctrlCoreRxPu;
		uint16 orig_RfctrlOverrideAuxTssi;
		uint16 orig_RfctrlCoreAuxTssi1;
		} orig_reg_vals[PHY_CORE_MAX];
	uint core_count = 0;
	txgain_setting_t curr_gain1, curr_gain2, curr_gain3;
	bool init_adc_inside = FALSE;
	uint16 save_afePuCtrl, save_gpio;
	uint16 orig_use_txPwrCtrlCoefs;
	uint16 fval2g_orig, fval5g_orig, fval2g, fval5g;
	uint32 save_chipc = 0;
	uint16 save_gpioHiOutEn;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	txgain_setting_t curr_gain4;
	int16 tone_tssi4[PHY_CORE_MAX];
	uint16 adj_tssi4[PHY_CORE_MAX];
	int bbmultcomp;
	uint16 tempmuxTxVlinOnFemCtrl2;
	uint16 txidxval;
	uint16 txgaintemp1[3], txgaintemp1a[3];
	uint16 tempmuxTxVlinOnFemCtrl, globpusmask;
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));
	/* prevent crs trigger */
	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
	band2g_idx = CHSPEC_IS2G(pi->radio_chanspec);
	if (band2g_idx)	{
		pwr3 = pi_ac->vlinpwr2g_from_nvram;
		}
	else {
		pwr3 = pi_ac->vlinpwr5g_from_nvram;
		}
	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);
	/* Turn off epa/ipa and unused rxrf part to prevent energy go into air */
	orig_use_txPwrCtrlCoefs = READ_PHYREGFLD(pi, TxPwrCtrlCmd,
	use_txPwrCtrlCoefs);
	FOREACH_ACTV_CORE(pi, pi->sh->hw_phyrxchain, core) {
		/* save phy/radio regs going to be touched */
		orig_reg_vals[core_count].orig_RfctrlIntc = READ_PHYREGCE(pi,
		RfctrlIntc, core);
		orig_reg_vals[core_count].orig_RfctrlOverrideRxPus =
			READ_PHYREGCE(pi, RfctrlOverrideRxPus, core);
		orig_reg_vals[core_count].orig_RfctrlCoreRxPu =
			READ_PHYREGCE(pi, RfctrlCoreRxPus, core);
		orig_reg_vals[core_count].orig_RfctrlOverrideAuxTssi =
			READ_PHYREGCE(pi, RfctrlOverrideAuxTssi, core);
		orig_reg_vals[core_count].orig_RfctrlCoreAuxTssi1 =
			READ_PHYREGCE(pi, RfctrlCoreAuxTssi1, core);
		orig_reg_vals[core_count].orig_OVR3 = READ_RADIO_REGC(pi,
			RF, OVR3, core);
		orig_reg_vals[core_count].orig_auxpga_cfg1 =
			READ_RADIO_REGC(pi, RF, AUXPGA_CFG1, core);
		orig_reg_vals[core_count].orig_auxpga_vmid =
			READ_RADIO_REGC(pi, RF, AUXPGA_VMID, core);
		orig_reg_vals[core_count].orig_iqcal_cfg1 =
			READ_RADIO_REGC(pi, RF, IQCAL_CFG1, core);
		orig_reg_vals[core_count].orig_tx5g_tssi = READ_RADIO_REGC(pi,
			RF, TX5G_TSSI, core);
		orig_reg_vals[core_count].orig_pa2g_tssi = READ_RADIO_REGC(pi,
			RF, PA2G_TSSI, core);
		orig_reg_vals[core_count].core = core;
		/* set tssi_range = 0   (it suppose to bypass 10dB attenuation before pdet) */
		MOD_PHYREGCE(pi, RfctrlOverrideAuxTssi,  core, tssi_range, 1);
		MOD_PHYREGCE(pi, RfctrlCoreAuxTssi1,	 core, tssi_range, 0);
		/* turn off lna and other unsed rxrf components */
		WRITE_PHYREGCE(pi, RfctrlOverrideRxPus, core, 0x7CE0);
		WRITE_PHYREGCE(pi, RfctrlCoreRxPus, 	core, 0x0);
		++core_count;
		}
	ACPHY_ENABLE_STALL(pi, stall_val);
	/* tssi loopback setup */
	phy_ac_tssi_loopback_path_setup(pi, LOOPBACK_FOR_TSSICAL);

	if (!init_adc_inside) {
		wlc_phy_init_adc_read(pi, &save_afePuCtrl, &save_gpio,
			&save_chipc, &fval2g_orig, &fval5g_orig,
			&fval2g, &fval5g, &stall_val, &save_gpioHiOutEn);
		}
	wlc_phy_get_paparams_for_band_acphy(pi, &a1, &b0, &b1);
	FOREACH_ACTV_CORE(pi, pi->sh->hw_phyrxchain, core) {
		if (!init_adc_inside)
			wlc_phy_gpiosel_acphy(pi, 16+core, 1);
		/* Measure the Idle TSSI */
		wlc_phy_poll_samps_WAR_acphy(pi, idle_tssi, TRUE, TRUE, NULL,
		FALSE, init_adc_inside, core, 1);
		MOD_PHYREG(pi, TxPwrCtrlCmd, use_txPwrCtrlCoefs, 0);
		wlc_phy_get_txgain_settings_by_index_acphy(pi, &curr_gain1, txidx1);
		wlc_phy_poll_samps_WAR_acphy(pi, tone_tssi1, TRUE, FALSE,
			&curr_gain1, FALSE, init_adc_inside, core, 1);
		adj_tssi1[core] = 1024+idle_tssi[core]-tone_tssi1[core];
		adj_tssi1[core] = adj_tssi1[core] >> 3;
		pwr1 = wlc_phy_tssi2dbm_acphy(pi, adj_tssi1[core], a1, b0, b1);
		wlc_phy_get_txgain_settings_by_index_acphy(pi, &curr_gain2, txidx2);
		wlc_phy_poll_samps_WAR_acphy(pi, tone_tssi2, TRUE, FALSE,
			&curr_gain2, FALSE, init_adc_inside, core, 1);
		adj_tssi2[core] = 1024+idle_tssi[core]-tone_tssi2[core];
		adj_tssi2[core] = adj_tssi2[core] >> 3;
		pwr2 = wlc_phy_tssi2dbm_acphy(pi, adj_tssi2[core], a1, b0, b1);
		txidx3 = txidx1+(4*pwr3-pwr1) *(txidx2-txidx1)/(pwr2-pwr1);
		wlc_phy_get_txgain_settings_by_index_acphy(pi, &curr_gain3, txidx3);
		wlc_phy_poll_samps_WAR_acphy(pi, tone_tssi3, TRUE, FALSE,
			&curr_gain3, FALSE, init_adc_inside, core, 1);
		adj_tssi3[core] = 1024+idle_tssi[core]-tone_tssi3[core];
		adj_tssi3[core] = adj_tssi3[core] >> 3;
		if (band2g_idx)	{
			globpusmask = 1<<(pi_ac->vlinmask2g_from_nvram);
			}
		else {
			globpusmask = 1<<(pi_ac->vlinmask5g_from_nvram);
			}
		tempmuxTxVlinOnFemCtrl = READ_PHYREGFLD(pi, RfctrlCoreGlobalPus,
			muxTxVlinOnFemCtrl);
		tempmuxTxVlinOnFemCtrl2 = (tempmuxTxVlinOnFemCtrl | globpusmask);
		MOD_PHYREG(pi, RfctrlCoreGlobalPus, muxTxVlinOnFemCtrl, tempmuxTxVlinOnFemCtrl2);
		wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_GAINCTRLBBMULTLUTS,
			1, txidx3, 48, &txgaintemp1);
		txgaintemp1a[0] = (txgaintemp1[0]|0x8000);
		txgaintemp1a[1] = txgaintemp1[1];
		txgaintemp1a[2] = txgaintemp1[2];
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_GAINCTRLBBMULTLUTS, 1,
			txidx3, 48, txgaintemp1a);
		wlc_phy_get_txgain_settings_by_index_acphy(pi, &curr_gain4, txidx3);
		wlc_phy_poll_samps_WAR_acphy(pi, tone_tssi4, TRUE, FALSE,
			&curr_gain4, FALSE, init_adc_inside, core, 1);
		adj_tssi4[core] = 1024+idle_tssi[core]-tone_tssi4[core];
		adj_tssi4[core] = adj_tssi4[core] >> 3;
		bbmultcomp = (int)((tone_tssi3[core]-tone_tssi4[core])/6);
		pi_ac->vlin_txidx = txidx3;
		pi_ac->bbmult_comp = bbmultcomp;
		for (txidxval = txidx3; txidxval < 128; txidxval++) {
			wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_GAINCTRLBBMULTLUTS, 1,
				txidxval, 48, &txgaintemp1);
			txgaintemp1a[0] = (txgaintemp1[0]|0x8000)+bbmultcomp;
			txgaintemp1a[1] = txgaintemp1[1];
			txgaintemp1a[2] = txgaintemp1[2];
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_GAINCTRLBBMULTLUTS, 1,
				txidxval, 48, txgaintemp1a);
			}
		if (!init_adc_inside)
			wlc_phy_restore_after_adc_read(pi, &save_afePuCtrl, &save_gpio,
			&save_chipc, &fval2g_orig, &fval5g_orig,
			&fval2g, &fval5g, &stall_val, &save_gpioHiOutEn);
		/* restore phy/radio regs */
		while (core_count > 0) {
			--core_count;
			phy_utils_write_radioreg(pi, RF_2069_OVR3(orig_reg_vals[core_count].core),
				orig_reg_vals[core_count].orig_OVR3);
			phy_utils_write_radioreg(pi,
				RF_2069_AUXPGA_CFG1(orig_reg_vals[core_count].core),
				orig_reg_vals[core_count].orig_auxpga_cfg1);
			phy_utils_write_radioreg(pi,
				RF_2069_AUXPGA_VMID(orig_reg_vals[core_count].core),
				orig_reg_vals[core_count].orig_auxpga_vmid);
			phy_utils_write_radioreg(pi,
				RF_2069_IQCAL_CFG1(orig_reg_vals[core_count].core),
				orig_reg_vals[core_count].orig_iqcal_cfg1);
			phy_utils_write_radioreg(pi,
				RF_2069_TX5G_TSSI(orig_reg_vals[core_count].core),
				orig_reg_vals[core_count].orig_tx5g_tssi);
			phy_utils_write_radioreg(pi,
				RF_2069_PA2G_TSSI(orig_reg_vals[core_count].core),
				orig_reg_vals[core_count].orig_pa2g_tssi);
			WRITE_PHYREGCE(pi, RfctrlIntc, orig_reg_vals[core_count].core,
				orig_reg_vals[core_count].orig_RfctrlIntc);
			WRITE_PHYREGCE(pi, RfctrlOverrideRxPus,
				orig_reg_vals[core_count].core,
				orig_reg_vals[core_count].orig_RfctrlOverrideRxPus);
			WRITE_PHYREGCE(pi, RfctrlCoreRxPus, orig_reg_vals[core_count].core,
				orig_reg_vals[core_count].orig_RfctrlCoreRxPu);
			WRITE_PHYREGCE(pi, RfctrlOverrideAuxTssi,
				orig_reg_vals[core_count].core,
				orig_reg_vals[core_count].orig_RfctrlOverrideAuxTssi);
			WRITE_PHYREGCE(pi, RfctrlCoreAuxTssi1,
				orig_reg_vals[core_count].core,
				orig_reg_vals[core_count].orig_RfctrlCoreAuxTssi1);
			}
		MOD_PHYREG(pi, TxPwrCtrlCmd, use_txPwrCtrlCoefs, orig_use_txPwrCtrlCoefs);
		/* prevent crs trigger */
		wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
		PHY_TRACE(("======= IQLOCAL PreCalGainControl : END =======\n"));
		}
}

/* PAPRR Functions */
static void chanspec_setup_papr(phy_info_t *pi,
	int8 papr_final_clipping, int8 papr_final_scaling)
{
	uint16 lowMcsGamma = 600, highMcsGamma, highMcsGamma_c8_c9 = 1100;
	uint16 gammaOffset[3] = {0, 0, 0};
	uint32 gain = 128, gamma;
	uint8 i, j;

	bool enable = chanspec_papr_enable(pi);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		highMcsGamma = 950;
		lowMcsGamma = 600;
	} else {
		highMcsGamma = 1100;
		lowMcsGamma = 600;
	}

	if (enable) {
		MOD_PHYREG(pi, papr_ctrl, papr_blk_en, enable);
		MOD_PHYREG(pi, papr_ctrl, papr_final_clipping_en, papr_final_clipping);
		MOD_PHYREG(pi, papr_ctrl, papr_final_scaling_en, papr_final_scaling);
		MOD_PHYREG(pi, papr_ctrl, papr_override_enable, 0);
	    for (j = 4; j <= 32; j++) {
			if (j <= 29) {
				/* gain entries for different rates */
				gain = 128;
			} else {
				/* gain offsets */
				gain = 0;
			}
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_PAPR, 1, j, 32, &gain);
	    }
	    for (j = 0x44; j <= 0x5D; j++) {
			if ((j >= 0x44 && j <= 0x47) || (j >= 0x4c && j <= 0x4e) ||
				(j >= 0x54 && j <= 0x56)) {
				gamma = (lowMcsGamma << 13) | lowMcsGamma;
			} else {
				gamma = (highMcsGamma << 13) | highMcsGamma;
				if (!PHY_IPA(pi) && ACMAJORREV_2(pi->pubpi->phy_rev) &&
					ACMINORREV_1(pi) && CHSPEC_IS2G(pi->radio_chanspec)) {
					if (j >= 0x5C && j <= 0x5D) {
				        gamma = (highMcsGamma_c8_c9 << 13) | highMcsGamma_c8_c9;
					}
				}
			}
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_PAPR, 1, j, 32, &gamma);
	    }
	    for (i = 0, j = 0x5E; j <= 0x60; j++, i++) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_PAPR, 1, j, 32, &gammaOffset[i]);
	    }
	} else {
		 MOD_PHYREG(pi, papr_ctrl, papr_blk_en, enable);
	}
}

static void
wlc_phy_spurwar_nvshp_acphy(phy_info_t *pi, bool bw_chg, bool spurwar, bool nvshp)
{
	uint8 i, core;
	uint8 core_nv = 0, core_sp = 0;
	uint8 noise_var[PHY_CORE_MAX][ACPHY_SPURWAR_NV_NTONES];
	int8 tone_id[ACPHY_SPURWAR_NV_NTONES];
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	/* Initialize variables */
	for (i = 0; i < ACPHY_SPURWAR_NV_NTONES; i++) {
		tone_id[i]   = 0;
		FOREACH_CORE(pi, core)
			noise_var[core][i] = 0;
	}

	/* Table reset req or not */
	if (nvshp && !bw_chg && !spurwar)
		nvshp = FALSE;

	if (spurwar || nvshp) {
		/* Reset Table */
		wlc_phy_reset_noise_var_shaping_acphy(pi);

		/* Call nvshp */
		if (nvshp)
			wlc_phy_set_noise_var_shaping_acphy(pi, noise_var, tone_id, &core_nv);

		/* Call spurwar */
		if (spurwar)
			phy_ac_spurwar(pi_ac->rxspuri, noise_var, tone_id, &core_sp);

		/* Write table
		 * If both nvshp and spurwar tries to write same tone
		 * priority lies with spurwar
		 */
		wlc_phy_noise_var_shaping_acphy(pi, core_nv, core_sp, tone_id, noise_var, 0);
	}
}

static void
wlc_phy_radio20693_set_channel_bw(phy_info_t *pi,
	radio_20693_adc_modes_t adc_mode, uint8 core)
{
	const chan_info_radio20693_altclkplan_t *altclkpln = altclkpln_radio20693;
	int row = wlc_phy_radio20693_altclkpln_get_chan_row(pi);
	uint8 dac_rate_mode;
	uint8 dacclkdiv = 0;
	uint16 dac_rate;
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	dac_rate_mode = pi_ac->dac_mode;
	dac_rate = wlc_phy_get_dac_rate_from_mode(pi, dac_rate_mode);
	ASSERT(dac_rate_mode <= 2);

	if (adc_mode == RADIO_20693_FAST_ADC) {
		if (dac_rate_mode == 1) {
			if (dac_rate == 200)
				dacclkdiv = 3;
			else if (dac_rate == 400)
				dacclkdiv = 1;
			else
				dacclkdiv = 0;
		} else if (dac_rate_mode == 2) {
			dacclkdiv = 0;
		}
	} else {
		if (dac_rate_mode == 1) {
			dacclkdiv = 0;
		} else {
			PHY_ERROR(("wl%d: %s: unknown dac rate mode slow\n",
				pi->sh->unit, __FUNCTION__));
		}
	}
	if ((row >= 0) && (adc_mode != RADIO_20693_FAST_ADC)) {
		dacclkdiv = altclkpln[row].dacclkdiv;
	}
	if (PAPD_80MHZ_WAR_4349A0(pi) && CHSPEC_IS80(pi->radio_chanspec)) {
		uint32	fc = wf_channel2mhz(CHSPEC_CHANNEL(pi->radio_chanspec),
			WF_CHAN_FACTOR_5_G);
		dacclkdiv = (fc >= 5500) ? 2 : 1;
	}
	MOD_RADIO_REG_20693(pi, CLK_DIV_CFG1, core, sel_dac_div, dacclkdiv);
}

static void
wlc_phy_radio20693_adc_setup(phy_info_t *pi, uint8 core,
	radio_20693_adc_modes_t adc_mode)
{
	tiny_adc_tuning_array_t gvalues;
	uint8 bw;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	bw = CHSPEC_IS20(pi->radio_chanspec) ? 20 : CHSPEC_IS40(pi->radio_chanspec) ? 40 : 80;

	if ((adc_mode == RADIO_20693_FAST_ADC) || (CHSPEC_IS80(pi->radio_chanspec)) ||
		(CHSPEC_IS8080(pi->radio_chanspec))) {
		/* 20 and 40MHz fast mode and 80MHz channel */
		wlc_tiny_sigdel_fast_tune(pi, pi->u.pi_acphy->rccal_adc_gmult, &gvalues);
		wlc_tiny_adc_setup_fast(pi, &gvalues, core);
	} else {
		/* slow mode for 20 and 40MHz channel */
		wlc_tiny_sigdel_slow_tune(pi, pi->u.pi_acphy->rccal_adc_gmult, &gvalues, bw);
		wlc_tiny_adc_setup_slow(pi, &gvalues, bw, core);
	}
}

static void
wlc_phy_radio20693_adc_config_overrides(phy_info_t *pi,
	radio_20693_adc_modes_t adc_mode, uint8 core)
{
	uint8 is_fast;
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	is_fast = (adc_mode == RADIO_20693_FAST_ADC);
	wlc_phy_radio20693_adc_powerupdown(pi, RADIO_20693_SLOW_ADC, !is_fast, core);
	wlc_phy_radio20693_adc_powerupdown(pi, RADIO_20693_FAST_ADC, is_fast, core);
}


static void
wlc_phy_radio20693_adc_dac_setup(phy_info_t *pi, radio_20693_adc_modes_t adc_mode, uint8 core)
{
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	wlc_phy_radio20693_afeclkpath_setup(pi, core, adc_mode, 0);
	wlc_phy_radio20693_adc_config_overrides(pi, adc_mode, core);
	wlc_phy_radio20693_adc_setup(pi, core, adc_mode);
	wlc_phy_radio20693_set_channel_bw(pi, adc_mode, core);
	wlc_phy_radio20693_config_bf_mode(pi, core);
}

static void
wlc_phy_radio20693_setup_crisscorss_ovr(phy_info_t *pi, uint8 core)
{
	uint8 pupd = 1;
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_tx5g_80p80_cas_pu, pupd);
	MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_tx5g_80p80_gm_pu, pupd);
	MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_tx2g_20p20_cas_pu, pupd);
	MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_tx2g_20p20_gm_pu, pupd);
	MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_rx5g_80p80_src_pu, pupd);
	MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_rx5g_80p80_des_pu, pupd);
	MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_rx5g_80p80_gc, pupd);
	MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core, ovr_rx2g_20p20_src_pu, pupd);
	MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core, ovr_rx2g_20p20_des_pu, pupd);
	MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core, ovr_rx2g_20p20_gc, pupd);
}

int
wlc_phy_radio20693_altclkpln_get_chan_row(phy_info_t *pi)
{
	const chan_info_radio20693_altclkplan_t *altclkpln = altclkpln_radio20693;
	int tbl_len = ARRAYSIZE(altclkpln_radio20693);
	int row;
	int channel = CHSPEC_CHANNEL(pi->radio_chanspec);
	int bw	= CHSPEC_IS20(pi->radio_chanspec) ? 20 : CHSPEC_IS40(pi->radio_chanspec) ? 40 : 80;

	for (row = 0; row < tbl_len; row++) {
		if ((altclkpln[row].channel == channel) && (altclkpln[row].bw == bw)) {
			break;
		}
	}
	return ((row < tbl_len) && ALTCLKPLN_ENABLE) ? row : -1;
}

void
wlc_phy_radio20693_afecal(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	uint8 core;
	radio_20693_adc_modes_t adc_mode;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	if ((pi_ac->fast_adc_en == 1) || (CHSPEC_IS80(pi->radio_chanspec)) ||
		(CHSPEC_IS8080(pi->radio_chanspec))) {
		adc_mode = RADIO_20693_FAST_ADC;
	} else {
		adc_mode = RADIO_20693_SLOW_ADC;
	}

	FOREACH_CORE(pi, core) {
		wlc_phy_radio20693_adc_dac_setup(pi, adc_mode, core);
		wlc_tiny_tia_config(pi, core);
		wlc_phy_radio20693_setup_crisscorss_ovr(pi, core);
	}
}

void
wlc_phy_radio_afecal(phy_info_t *pi)
{
	wlc_phy_resetcca_acphy(pi);
	OSL_DELAY(1);
	if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID))
		wlc_phy_radio20691_afecal(pi);
	else if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID))
		wlc_phy_radio20693_afecal(pi);
	else
		wlc_phy_radio2069_afecal(pi);

}

static void
wlc_phy_radio20693_set_reset_table_bits(phy_info_t *pi, uint16 tbl_id, uint16 offset,
	uint16 start, uint16 end, uint16 val, uint8 tblwidth)
{
	uint16 val_shift, mask;
	uint32 data[2];

	val_shift = val << start;
	mask  = ((1 << (end + 1)) - (1 << start));
	wlc_phy_table_read_acphy(pi, tbl_id, 1, offset, tblwidth, &data);

	data[0] = ((data[0] & mask) | val_shift);
	wlc_phy_table_write_acphy(pi, tbl_id, 1, offset, tblwidth, &data);
}

static void
wlc_phy_wltx_word_get(phy_info_t *pi, uint8 band, uint32 swctrlmap_wltx,
	uint32 swctrlmap_wltx_ext, uint32 *swctrlword,	uint32 *swctrlwordext)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* If linear, use the lower 16 bits */
	if (pi_ac->pa_mode == PAMODE_HI_LIN) {
		*swctrlword = swctrlmap_wltx & PAMODE_HI_LIN_MASK;
		*swctrlwordext = swctrlmap_wltx_ext & PAMODE_HI_LIN_MASK;
	} else {

		/* Otherwise use the upper 16 bits. */
		*swctrlword = (swctrlmap_wltx & PAMODE_HI_EFF_MASK) >> 16;
		*swctrlwordext = (swctrlmap_wltx_ext & PAMODE_HI_EFF_MASK) >> 16;
	}
}

#ifndef ACPHY_1X1_ONLY
static void
wlc_phy_write_tx_farrow_acphy(phy_info_t *pi, chanspec_t chanspec)
{
	uint8	ch = CHSPEC_CHANNEL(chanspec), afe_clk_num, afe_clk_den;
	uint16	a, b, lb_b = 0;
	uint32	fcw, lb_fcw, tmp_low = 0, tmp_high = 0;
	uint32  deltaphase;
	uint16  deltaphase_lo, deltaphase_hi;
	uint16  farrow_downsamp;
	uint32	fc = wf_channel2mhz(ch, CHSPEC_IS2G(pi->radio_chanspec) ? WF_CHAN_FACTOR_2_4_G
	                                                               : WF_CHAN_FACTOR_5_G);

	if (pi->u.pi_acphy->dac_mode == 1) {
		if (CHSPEC_IS20(chanspec)) {
			if (CHSPEC_IS5G(chanspec)) {
				if (((RADIOMAJORREV(pi) == 2) &&
				     ((fc == 5745) || (fc == 5765))) &&
				    !(ISSIM_ENAB(pi->sh->sih))) {
					a = 18;
				} else {
					a = 16;
				}
			} else {
				if ((RADIOMAJORREV(pi) == 2) &&
				    !(ISSIM_ENAB(pi->sh->sih))) {
					if (((fc != 2412) && (fc != 2467)) ||
					    (pi->xtalfreq == 40000000) || (PHY_IPA(pi))) {
						a = 18;
					} else {
						a = 16;
					}
				} else {
					a = 16;
				}
			}
			b = 160;
		} else if (CHSPEC_IS40(chanspec)) {
			if (CHSPEC_IS5G(chanspec)) {
				if (((RADIOMAJORREV(pi) == 2) &&
				     ((fc == 5755) || (fc == 5550 && pi->xtalfreq == 40000000) ||
				      (fc == 5310 && pi->xtalfreq == 37400000))) &&
				    !(ISSIM_ENAB(pi->sh->sih))) {
					a = 9;
				} else {
					a = 8;
				}
			} else {
				if ((RADIOMAJORREV(pi) == 2) &&
				    !(ISSIM_ENAB(pi->sh->sih))) {
					a = 9;
				} else {
					a = 8;
				}
			}
			b = 320;
		} else {
			a = 6;
			b = 640;
		}
	} else if (pi->u.pi_acphy->dac_mode == 2) {
		a = 6;
		b = 640;
		lb_b = 320;
	} else {
		a = 8;
		b = 320;
		lb_b = 320;
	}

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		afe_clk_num = 2;
		afe_clk_den = 3;
	} else {
		afe_clk_num = 3;
		afe_clk_den = 2;
		if (fc == 5290 && ACMAJORREV_2(pi->pubpi->phy_rev) &&
		    ((ACMINORREV_1(pi) && pi->sh->chippkg == 2) ||
		     ACMINORREV_3(pi)) && pi->xtalfreq == 37400000) {
			afe_clk_num = 4;
			afe_clk_den = 3;
		}
	}

	bcm_uint64_multiple_add(&tmp_high, &tmp_low, a * afe_clk_num * b,
		1 << 23, (fc * afe_clk_den) >> 1);
	bcm_uint64_divide(&fcw, tmp_high, tmp_low, fc * afe_clk_den);
	wlc_phy_tx_farrow_mu_setup(pi, fcw & 0xffff, (fcw & 0xff0000) >> 16, fcw & 0xffff,
		(fcw & 0xff0000) >> 16);
	/* DAC MODE 1 lbfarrow setup in rx_farrow_acphy */
	if (pi->u.pi_acphy->dac_mode != 1) {
		bcm_uint64_multiple_add(&tmp_high, &tmp_low, fc * afe_clk_den,
		        1 << 25, 0);
		bcm_uint64_divide(&lb_fcw, tmp_high, tmp_low, a * afe_clk_num * lb_b);
		deltaphase = (lb_fcw - 33554431) >> 1;
		deltaphase_lo = deltaphase & 0xffff;
		deltaphase_hi = (deltaphase >> 16) & 0xff;
		farrow_downsamp = fc * afe_clk_den / (a * afe_clk_num * lb_b);
		WRITE_PHYREG(pi, lbFarrowDeltaPhase_lo, deltaphase_lo);
		WRITE_PHYREG(pi, lbFarrowDeltaPhase_hi, deltaphase_hi);
		WRITE_PHYREG(pi, lbFarrowDriftPeriod, 5120);
		MOD_PHYREG(pi, lbFarrowCtrl, lb_farrow_downsampfactor, farrow_downsamp);
	}
}
#endif /* ACPHY_1X1_ONLY */

static INLINE void
wlc_phy_write_sparse_femctrl_table(phy_info_t *pi)
{
	uint16 fectrl_zeroval[] = {0};
	uint16 fectrl_fourval[] = {4};
	uint16 fectrl_nineval[] = {9};
	uint kk, fem_idx = 0;
	for (kk = 0; kk < pi->u.pi_acphy->fectrl_table_len; kk++) {
		if (fem_idx < pi->u.pi_acphy->fectrl_sparse_table_len &&
			kk == pi->u.pi_acphy->fectrl_idx[fem_idx]) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
			&(pi->u.pi_acphy->fectrl_val[fem_idx]));
			fem_idx++;
		} else if (pi->u.pi_acphy->fectrl_spl_entry_flag) {
			/* 43162: Fix to avoid all zero output from femctrl during */
			/* tx2rx/rx2tx in 5G which causes popping-sound in BT */
			/* tx2rx/rx2tx in 2G also cause zero state on FEM, add lines for safety */
			if (kk & 0x10) {
				/* 5G */
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
					fectrl_fourval);
			} else {
				/* 2G */
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
					fectrl_nineval);
			}
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}

}

static void
wlc_phy_write_rx_farrow_acphy(phy_info_t *pi, chanspec_t chanspec)
{
	uint16 deltaphase_lo, deltaphase_hi;
	uint8 ch = CHSPEC_CHANNEL(chanspec), num, den, bw, M, vco_div;
	uint32 deltaphase, farrow_in_out_ratio, fcw, tmp_low = 0, tmp_high = 0;
	uint16 drift_period, farrow_ctrl;
	uint8 farrow_outsft_reg, dec_outsft_reg, farrow_outscale_reg = 1;
	uint32 fc = wf_channel2mhz(ch, CHSPEC_IS2G(pi->radio_chanspec) ?
	        WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
	if (CHSPEC_IS80(chanspec)) {
		farrow_outsft_reg = 0;
		dec_outsft_reg = 0;
	} else {
		if (((ACMAJORREV_0(pi->pubpi->phy_rev)) && ((ACMINORREV_1(pi)) ||
		    (ACMINORREV_0(pi)))) || ((ACMAJORREV_1(pi->pubpi->phy_rev)) &&
		    (ACMINORREV_1(pi) || ACMINORREV_0(pi)))) {
			farrow_outsft_reg = 2;
		} else {
			farrow_outsft_reg = 0;
		}
		dec_outsft_reg = 3;
	}

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		num = 3;
		den = 2;
	} else {
		num = 2;
		den = 3;
		if (CHSPEC_IS80(chanspec) && fc == 5290 && ACMAJORREV_2(pi->pubpi->phy_rev) &&
		    ((ACMINORREV_1(pi) && pi->sh->chippkg == 2) ||
		    ACMINORREV_3(pi)) && pi->xtalfreq == 37400000) {
			num = 3;
			den = 4;
		}
	}

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		if ((RADIOMAJORREV(pi) == 2) && !(ISSIM_ENAB(pi->sh->sih))) {
			if (CHSPEC_IS40(chanspec)) {
				bw = 40;
				M = 4;
				vco_div = 18;
				drift_period = 1920;
			} else {
				if ((fc != 2412 && fc != 2467) ||(pi->xtalfreq == 40000000) ||
				    PHY_IPA(pi)) {
					bw = 20;
					M = 8;
					vco_div = 18;
					drift_period = 5760;
				} else {
					bw = 20;
					M = 8;
					vco_div = 16;
					drift_period = 5120;
				}
			}
		} else {
			bw = 20;
			M = 8;
			vco_div = 16;
			drift_period = 5120;
		}
	} else {
		if (CHSPEC_IS80(chanspec)) {
			bw = 80;
			M = 4;
			vco_div = 6;
			drift_period = 2880;
			if (fc == 5290 && ACMAJORREV_2(pi->pubpi->phy_rev) &&
			    ((ACMINORREV_1(pi) && pi->sh->chippkg == 2) ||
			    ACMINORREV_3(pi)) && pi->xtalfreq == 37400000) {
				drift_period = 2560;
			}
		} else {
			if (RADIOMAJORREV(pi) == 2) {
				if ((((fc == 5755 || (fc == 5550 && pi->xtalfreq == 40000000) ||
					(fc == 5310 && pi->xtalfreq == 37400000)) &&
					(CHSPEC_IS40(chanspec))) ||
					((fc == 5745 || fc == 5765) && (CHSPEC_IS20(chanspec)))) &&
					!(ISSIM_ENAB(pi->sh->sih))) {
					bw = 20;
					M = 8;
					vco_div = 18;
					drift_period = 4320;
				} else {
					bw = 20;
					M = 8;
					vco_div = 16;
					drift_period = 3840;
				}
			} else {
				bw = 20;
				M = 8;
				vco_div = 16;
				drift_period = 3840;
			}
		}
	}
	bcm_uint64_multiple_add(&tmp_high, &tmp_low, fc * num, 1 << 25, 0);
	bcm_uint64_divide(&fcw, tmp_high, tmp_low, (uint32) (den * vco_div * M * bw));

	farrow_in_out_ratio = (fcw >> 25);
	deltaphase = (fcw - 33554431)>>1;
	deltaphase_lo = deltaphase & 0xffff;
	deltaphase_hi = (deltaphase >> 16) & 0xff;
	farrow_ctrl = (dec_outsft_reg & 0x3) | ((farrow_outscale_reg & 0x3) << 2) |
		((farrow_outsft_reg & 0x7) << 4) | ((farrow_in_out_ratio & 0x3) <<7);

	WRITE_PHYREG(pi, rxFarrowDeltaPhase_lo, deltaphase_lo);
	WRITE_PHYREG(pi, rxFarrowDeltaPhase_hi, deltaphase_hi);
	WRITE_PHYREG(pi, rxFarrowDriftPeriod, drift_period);
	WRITE_PHYREG(pi, rxFarrowCtrl, farrow_ctrl);
	MOD_PHYREG(pi, lbFarrowCtrl, lb_farrow_outShift, farrow_outsft_reg);
	MOD_PHYREG(pi, lbFarrowCtrl, lb_decimator_output_shift, dec_outsft_reg);
	MOD_PHYREG(pi, lbFarrowCtrl, lb_farrow_outScale, farrow_outscale_reg);
	/* Use the same settings for the loopback Farrow */
	if (pi->u.pi_acphy->dac_mode == 1) {
		WRITE_PHYREG(pi, lbFarrowDeltaPhase_lo, deltaphase_lo);
		WRITE_PHYREG(pi, lbFarrowDeltaPhase_hi, deltaphase_hi);
		WRITE_PHYREG(pi, lbFarrowDriftPeriod, drift_period);
		MOD_PHYREG(pi, lbFarrowCtrl, lb_farrow_downsampfactor, farrow_in_out_ratio);
	}
}

static void
wlc_phy_radio20691_4345_vco_opt(phy_info_t *pi, uint8 vco_mode)
{

	if (vco_mode == ACPHY_VCO_2P5V) {
		MOD_RADIO_REG_20691(pi, PLL_VCO3, 0, rfpll_vco_cvar_extra, 0xa);
		MOD_RADIO_REG_20691(pi, PLL_VCO2, 0, rfpll_vco_cvar, 0xf);
		MOD_RADIO_REG_20691(pi, PLL_VCO6, 0, rfpll_vco_bias_mode, 0x0);
		MOD_RADIO_REG_20691(pi, PLL_VCO6, 0, rfpll_vco_ALC_ref_ctrl, 0x0);
		MOD_RADIO_REG_20691(pi, PLL_CP4, 0, rfpll_cp_kpd_scale, 0x21);
		MOD_RADIO_REG_20691(pi, PLL_XTALLDO1, 0, ldo_1p2_xtalldo1p2_lowquiescenten, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO2, 0, ldo_2p5_lowquiescenten_VCO, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO2, 0, ldo_2p5_lowquiescenten_CP, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO4, 0, ldo_2p5_static_load_CP, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO4, 0, ldo_2p5_static_load_VCO, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_CFG3, 0, rfpll_spare1, 0x3);

	} else if (vco_mode == ACPHY_VCO_1P35V) {
		MOD_RADIO_REG_20691(pi, PLL_VCO3, 0, rfpll_vco_cvar_extra, 0xf);
		MOD_RADIO_REG_20691(pi, PLL_VCO2, 0, rfpll_vco_cvar, 0xf);
		MOD_RADIO_REG_20691(pi, PLL_VCO6, 0, rfpll_vco_bias_mode, 0x0);
		MOD_RADIO_REG_20691(pi, PLL_VCO6, 0, rfpll_vco_ALC_ref_ctrl, 0x3);
		MOD_RADIO_REG_20691(pi, PLL_CP4, 0, rfpll_cp_kpd_scale, 0x21);
		MOD_RADIO_REG_20691(pi, PLL_XTALLDO1, 0, ldo_1p2_xtalldo1p2_lowquiescenten, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO2, 0, ldo_2p5_lowquiescenten_VCO, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO2, 0, ldo_2p5_lowquiescenten_CP, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO4, 0, ldo_2p5_static_load_CP, 0x1);
		MOD_RADIO_REG_20691(pi, PLL_HVLDO4, 0, ldo_2p5_static_load_VCO, 0x1);

		MOD_RADIO_REG_20691(pi, PLL_CFG3, 0, rfpll_spare0, 0xb4);

		MOD_RADIO_REG_20691(pi, PLL_CFG3, 0, rfpll_spare1, 0x3);
	}
}

static void
wlc_phy_enable_pavref_war(phy_info_t *pi)
{
	/* 43602a0: power on PARLDO and update RFSeq table */
	const uint16 tx2rx_delay = 0x130;
	const uint16 lna_trsw_timing[] = {0x1, 0x2, 0x2, 0x2, 0x4};
	si_pmu_switch_on_PARLDO(pi->sh->sih, pi->sh->osh);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
		1, 0x80, 16, &tx2rx_delay);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
		ARRAYSIZE(lna_trsw_timing), 0x70, 16, &lna_trsw_timing);

	WRITE_PHYREG(pi, dot11acphycrsTxExtension, 0x1);
	W_REG(pi->sh->osh, &pi->regs->PHYREF_IFS_SIFS_RX_TX_TX, 0x7676);
	W_REG(pi->sh->osh, &pi->regs->PHYREF_IFS_SIFS_NAV_TX, 0x0276);
	W_REG(pi->sh->osh, &pi->regs->psm_int_sel_1, 0x5);
}

static void
wlc_phy_radio_vco_opt(phy_info_t *pi, uint8 vco_mode)
{
	if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
		wlc_phy_radio2069_4335C0_vco_opt(pi, vco_mode);
	} else if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		wlc_phy_radio20691_4345_vco_opt(pi, vco_mode);
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
	}
}

static int
wlc_tiny_sigdel_fast_mult(int raw_p8, int mult_p12, int max_val, int rshift)
{
	int prod;
	/* >> 20 follows from .12 fixed-point for mult, various.8 for raw */
	prod = (mult_p12 * raw_p8) >> rshift;
	return (prod > max_val) ? max_val : prod;
}

static void
wlc_tiny_adc_setup_fast(phy_info_t *pi, tiny_adc_tuning_array_t *gvalues, uint8 core)
{

	/* EXPLICITLY ENABLE/DISABLE ADCs and INTERNAL CLKs?  */
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_adc_fast_pu, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_adc_slow_pu, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_slow_pu, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_adc_clk_fast_pu, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_adc_clk_slow_pu, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG15, core, adc_clk_slow_pu, 0x0);

	/* Setup internal dividers and sipo for 3G2Hz mode. */
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_drive_strength, 0x4);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_div8, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_CFG15, core, adc_clk_slow_div3, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_div8, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG10, core, adc_sipo_sel_fast, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_drive_strength, 0x4);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_div8, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_CFG6, core, adc_biasadj_opamp1, 0x60);
	MOD_RADIO_REG_TINY(pi, ADC_CFG6, core, adc_biasadj_opamp2, 0x60);
	MOD_RADIO_REG_TINY(pi, ADC_CFG7, core, adc_biasadj_opamp3, 0x40);
	MOD_RADIO_REG_TINY(pi, ADC_CFG7, core, adc_biasadj_opamp4, 0x40);
	MOD_RADIO_REG_TINY(pi, ADC_CFG8, core, adc_ff_mult_opamp, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG9, core, adc_cmref_control, 0x40);
	MOD_RADIO_REG_TINY(pi, ADC_CFG9, core, adc_cmref4_control, 0x40);
	MOD_RADIO_REG_TINY(pi, ADC_CFG10, core, adc_sipo_sel_fast, 0x1);

	/* Turn on overload detector */
	MOD_RADIO_REG_TINY(pi, ADC_CFG18, core, adc_od_bias_comp, 0x40);
	MOD_RADIO_REG_TINY(pi, ADC_CFG18, core, adc_od_threshold, 0x3);
	MOD_RADIO_REG_TINY(pi, ADC_CFG18, core, adc_od_reset_duration, 0x3);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_adc_od_pu, 0x1);

	MOD_RADIO_REG_TINY(pi, ADC_CFG18, core, adc_od_pu, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG2, core, adc_gi, gvalues->gi);

	/* typo in spreadsheet for TC only - should be ri3 but got called ri1 */
	MOD_RADIO_REG_TINY(pi, ADC_CFG2, core, adc_ri3, gvalues->ri3);

	MOD_RADIO_REG_TINY(pi, ADC_CFG3, core, adc_g21, gvalues->g21);
	MOD_RADIO_REG_TINY(pi, ADC_CFG3, core, adc_g32, gvalues->g32);
	MOD_RADIO_REG_TINY(pi, ADC_CFG4, core, adc_g43, gvalues->g43);
	MOD_RADIO_REG_TINY(pi, ADC_CFG4, core, adc_g54, gvalues->g54);
	MOD_RADIO_REG_TINY(pi, ADC_CFG5, core, adc_g65, gvalues->g65);
	MOD_RADIO_REG_TINY(pi, ADC_CFG5, core, adc_r12, gvalues->r12);
	MOD_RADIO_REG_TINY(pi, ADC_CFG8, core, adc_r34, gvalues->r34);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_reset_adc, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_adcs_reset, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_adcs_reset, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_reset_adc, 0x0);
}

static void
wlc_phy_radio20691_afecal(phy_info_t *pi)
{
	uint8 core, bw;
	tiny_adc_tuning_array_t gvalues;
	bw = CHSPEC_IS20(pi->radio_chanspec) ? 20 : CHSPEC_IS40(pi->radio_chanspec) ? 40 : 80;

	/* enable ?? wlc_phy_radio20691_rccal(pi); */
	if (CHSPEC_IS80(pi->radio_chanspec)) {
		/* set gvalues [20691_sigdel_fast_tune $def(radio_rccal_adc_gmult)] */
		wlc_tiny_sigdel_fast_tune(pi, pi->u.pi_acphy->rccal_adc_gmult, &gvalues);
		/* 20691_adc_setup_fast $gvalues */
		wlc_tiny_adc_setup_fast(pi, &gvalues, 0);
	} else {
		/* set gvalues [20691_sigdel_slow1g2_tune $def(radio_rccal_adc_gmult)] */
		wlc_tiny_sigdel_slow_tune(pi, pi->u.pi_acphy->rccal_adc_gmult, &gvalues, bw);
		/* 20691_adc_setup_slow1g2 $gvalues */
		wlc_tiny_adc_setup_slow(pi, &gvalues, bw, 0);
	}

	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		wlc_tiny_tia_config(pi, core);
	}
}

static void
wlc_tiny_sigdel_fast_tune(phy_info_t *pi, int g_mult_raw_p12, tiny_adc_tuning_array_t *gvalues)
{
	int g_mult_tweak_p12;
	int g_mult_p12;
	int g_inv_p12;
	int gi_inv_p12;
	int gi_p8;
	int ri3_p8;
	int g21_p8;
	int g32_p8;
	int g43_p8;
	int g54_p8;
	int g65_p8;
	int r12_p8;
	int r34_p8;
	int gi, ri3, g21, g32, g43, g54, g65, r12, r34;

	/* tweak to g_mult to trade off stability over PVT versus performance */
	g_mult_tweak_p12 = 4096;
	g_mult_p12 = (g_mult_tweak_p12 * g_mult_raw_p12) >> 12;

	/* inverse of gmult precomputed to minimise division operations for speed */
	g_inv_p12 = 16777216 / g_mult_p12;
	gi_inv_p12 = 16777216 / WLC_TINY_GI_MULT;

	/* untuned values in p8 fixed-point format, ie. multiplied by 2^8 */
	gi_p8 = 16384;
	ri3_p8 = 1477;
	g21_p8 = 17997;
	g32_p8 = 18341;
	g43_p8 = 15551;
	g54_p8 = 19915;
	g65_p8 = 12369;
	r12_p8 = 1156;
	r34_p8 = 4331;

	/* RC cal */
	gi = wlc_tiny_sigdel_fast_mult(gi_p8, WLC_TINY_GI_MULT, 127, 20);
	ri3 = wlc_tiny_sigdel_fast_mult(ri3_p8, gi_inv_p12, 63, 20);
	g21 = wlc_tiny_sigdel_fast_mult(g21_p8, g_mult_p12, 127, 20);
	g32 = wlc_tiny_sigdel_fast_mult(g32_p8, g_mult_p12, 127, 20);
	g43 = wlc_tiny_sigdel_fast_mult(g43_p8, g_mult_p12, 127, 20);
	g54 = wlc_tiny_sigdel_fast_mult(g54_p8, g_mult_p12, 127, 20);
	g65 = wlc_tiny_sigdel_fast_mult(g65_p8, g_mult_p12, 63, 20);
	r12 = wlc_tiny_sigdel_fast_mult(r12_p8, g_inv_p12, 63, 20);
	r34 = wlc_tiny_sigdel_fast_mult(r34_p8, g_inv_p12, 63, 20);

	gvalues->gi = (uint16) gi;
	gvalues->ri3 = (uint16) ri3;
	gvalues->g21 = (uint16) g21;
	gvalues->g32 = (uint16) g32;
	gvalues->g43 = (uint16) g43;
	gvalues->g54 = (uint16) g54;
	gvalues->g65 = (uint16) g65;
	gvalues->r12 = (uint16) r12;
	gvalues->r34 = (uint16) r34;
}

#ifndef WL_FDSS_DISABLED
static void
wlc_phy_fdss_init(phy_info_t *pi)
{
	uint8 core;
	FOREACH_CORE(pi, core) {
		MOD_PHYREGCEE(pi, txfdss_ctrl, core, txfdss_enable, 1);
		MOD_PHYREGCEE(pi, txfdss_ctrl, core, txfdss_interp_enable, pi->fdss_interp_en);
		MOD_PHYREGCEE(pi, txfdss_cfgtbl, core, txfdss_num_20M_tbl, 2);
		MOD_PHYREGCEE(pi, txfdss_cfgtbl, core, txfdss_num_40M_tbl, 2);
		MOD_PHYREGCEE(pi, txfdss_cfgbrkpt0_, core, txfdss_num_20M_breakpoints, 5);
		MOD_PHYREGCEE(pi, txfdss_cfgbrkpt0_, core, txfdss_num_40M_breakpoints, 5);
		MOD_PHYREGCEE(pi, txfdss_cfgbrkpt1_, core, txfdss_num_80M_breakpoints, 5);
		MOD_PHYREGCEE(pi, txfdss_scaleadj_en_, core, txfdss_scale_adj_enable, 0);
		MOD_PHYREGCEE(pi, txfdss_scaleadj_en_, core, txfdss_scale_adj_enable, 7);
	}
}

static void
wlc_phy_set_fdss_table(phy_info_t *pi)
{
	uint8 mcstable[71] = {16, 16, 16, 16, 17, 17, 17, 17,
		16, 16, 16, 17, 17, 17, 17, 17,
		16, 16, 16, 17, 17, 17, 17, 17, 17, 17,
		16, 16, 16, 16, 17, 17, 17, 17,
		16, 16, 16, 17, 17, 17, 17, 17,
		16, 16, 16, 17, 17, 17, 17, 17, 17, 17,
		17,
		16, 16, 16, 17, 17, 17, 17, 17,
		16, 16, 16, 17, 17, 17, 17, 17, 17, 17,
		};

	uint8 i, fdss_level[2];
	uint8 breakpoint_list_20[5] = {0, 3, 17, 48, 62};
	uint8 breakpoint_list_40[5] = {0, 6, 34, 96, 124};
	uint8 breakpoint_list_80[5] = {0, 12, 68, 192, 248};
	uint8 breakpoint_list_interp_20[2] = {47, 61};
	uint8 breakpoint_list_interp_40[2] = {97, 123};
	uint8 breakpoint_list_interp_80[2] = {191, 247};

	uint8 fdss_scale_level[4][5] = {{128, 128, 128, 128, 128},
		{128, 128, 128, 128, 128},
		{164, 146, 104, 146, 164}, /* Mild, meets older +1, -3 dB flatness limits */
		{180, 128, 72, 128, 180} /* Extreme, meets older +3, -5 dB flatness limits */
		};
	int16 fdss_scale_level_interp_20[4][5] = {{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{-683, -338, 0, 338, 683},
		{-2219, -512, 0, 512, 2219}};
	int16 fdss_scale_level_interp_40[4][5] = {{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{-341, -169, 0, 169, 341},
		{-1109, -256, 0, 256, 1109}};
	int16 fdss_scale_level_interp_80[4][5] = {{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{-171, -86, 0, 86, 171},
		{-555, -128, 0, 128, 555}};
	uint8 fdss_scale_level_adjust_20[4] = {128, 128, 132, 128};
	uint8 fdss_scale_level_adjust_40[4] = {128, 128, 132, 128};
	uint8 fdss_scale_level_adjust_80[4] = {128, 128, 134, 128};
	uint8 fdss_scale_level_adjust_interp_20[4] = {128, 128, 132, 128};
	uint8 fdss_scale_level_adjust_interp_40[4] = {128, 128, 131, 128};
	uint8 fdss_scale_level_adjust_interp_80[4] = {128, 128, 134, 128};

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		fdss_level[0] = pi->fdss_level_2g[0];
		if (pi->fdss_level_2g[1] ==  -1) {
			fdss_level[1] = 0;
		} else {
			fdss_level[1] = pi->fdss_level_2g[1];
		}
	} else {
		fdss_level[0] = pi->fdss_level_5g[0];
		if (pi->fdss_level_5g[1] ==  -1) {
			fdss_level[1] = 0;
		} else {
			fdss_level[1] = pi->fdss_level_5g[1];
		}
	}

	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_MCSINFOTBL0, 71, 0, 8, mcstable);

	/* Populate breakpoint and scale tables with the scale values for each BW */
	for (i = 0; i < 2; i++) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0, 5, 5*i, 8,
			breakpoint_list_20);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEFACTORSTBL0, 5, 5*i, 8,
			fdss_scale_level[fdss_level[i]]);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0, 1, i, 8,
			&fdss_scale_level_adjust_20[fdss_level[i]]);

		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0, 5, 10+5*i, 8,
			breakpoint_list_40);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEFACTORSTBL0, 5, 10+5*i, 8,
			fdss_scale_level[fdss_level[i]]);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0, 1, i+2, 8,
			&fdss_scale_level_adjust_40[fdss_level[i]]);

		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0, 5, 20+5*i, 8,
			breakpoint_list_80);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEFACTORSTBL0, 5, 20+5*i, 8,
			fdss_scale_level[fdss_level[i]]);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0, 1, i+4, 8,
			&fdss_scale_level_adjust_80[fdss_level[i]]);
	}
	/* Edit  breakpoint table for interpolation case */

	if (pi->fdss_interp_en) {
		for (i = 0; i < 2; i++) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0,
				2, 3+5*i, 8, breakpoint_list_interp_20);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEFACTORSDELTATBL0,
				5, 5*i, 16, fdss_scale_level_interp_20[fdss_level[i]]);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0,
				1, i, 8, &fdss_scale_level_adjust_interp_20[fdss_level[i]]);

			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0,
				2, 13+5*i, 8, breakpoint_list_interp_40);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEFACTORSDELTATBL0,
				5, 10+5*i, 16, fdss_scale_level_interp_40[fdss_level[i]]);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0,
				1, i+2, 8, &fdss_scale_level_adjust_interp_40[fdss_level[i]]);

			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0,
				2, 23+5*i, 8, breakpoint_list_interp_80);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEFACTORSDELTATBL0,
				5, 20+5*i, 16, fdss_scale_level_interp_80[fdss_level[i]]);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0,
				1, i+4, 8, &fdss_scale_level_adjust_interp_80[fdss_level[i]]);
		}
	}
}
#endif /* WL_FDSS_DISABLED */

static void
wlc_phy_papd_set_rfpwrlut_tiny(phy_info_t *pi)
{
	int16 radiogainqdb;
	uint8 idx;
	uint16 txgain[1], bbmult;
	int16 temp, temp1, temp2, qQ, qQ1, qQ2, shift;
	uint8 scale_factor = 1;
	int8 papd_rf_pwr_scale = 32; /* Q5 format */
	int32 val = 0;
	uint8 tx_gain_tbl_id, core;
	uint8 rfpwrlut_table_ids[] = { ACPHY_TBL_ID_RFPWRLUTS0,
		ACPHY_TBL_ID_RFPWRLUTS1, ACPHY_TBL_ID_RFPWRLUTS2};

	if (CHSPEC_IS2G(pi->radio_chanspec) && (pi->parfps2g != -1)) {
		papd_rf_pwr_scale = pi->parfps2g;
	} else if (CHSPEC_IS5G(pi->radio_chanspec) && (pi->parfps5g != -1)) {
		papd_rf_pwr_scale = pi->parfps5g;
	}

	/* acphy_beDeaf??? */
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);

	FOREACH_CORE(pi, core) {
		for (idx = 0; idx < 128; idx++)  {
			tx_gain_tbl_id = wlc_phy_get_tbl_id_gainctrlbbmultluts(pi, core);
			wlc_phy_table_read_acphy(pi, tx_gain_tbl_id, 1, idx, 48, &txgain);
			bbmult = (txgain[0] & 0xff);

			qm_log10((int32)(bbmult), 0, &temp1, &qQ1);
			qm_log10((int32)(1<<6), 0, &temp2, &qQ2);

			if (qQ1 < qQ2) {
				temp2 = qm_shr16(temp2, qQ2-qQ1);
				qQ = qQ1;
			} else {
				temp1 = qm_shr16(temp1, qQ1-qQ2);
				qQ = qQ2;
			}
			temp = qm_sub16(temp1, temp2);

			if (qQ >= 4)
				shift = qQ-4;
			else
				shift = 4-qQ;

			val = ((((idx*papd_rf_pwr_scale/32) << shift) + (5*temp) +
				(1<<(scale_factor+shift-3)))>>(scale_factor+shift-2));

			radiogainqdb = -(val)/2;

			/* No need of iteration delays for 4349 family */
			if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
				/* adding 10us of delay as table_write is throwing assert */
				OSL_DELAY(10);
			}
			wlc_phy_table_write_acphy(pi, rfpwrlut_table_ids[core], 1, idx,
				16, &radiogainqdb);
		}
	}
	wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
	wlapi_enable_mac(pi->sh->physhim);
}

static void
wlc_acphy_load_4349_specific_tbls(phy_info_t *pi)
{
		wlc_acphy_load_radiocrisscross_phyovr_mode(pi);
		wlc_acphy_load_logen_tbl(pi);
}

static void
wlc_acphy_load_radiocrisscross_phyovr_mode(phy_info_t *pi)
{
	uint8 core;
	FOREACH_CORE(pi, core) {
		WRITE_PHYREGCE(pi, AfeClkDivOverrideCtrlN, core, 0x0000);
		WRITE_PHYREGCE(pi, RfctrlAntSwLUTIdxN, core, 0x0000);
		WRITE_PHYREGCE(pi, RfctrlCoreTxPus, core,
			(READ_PHYREGCE(pi, RfctrlCoreTxPus, core) & 0x7DFF));
		WRITE_PHYREGCE(pi, RfctrlOverrideTxPus, core,
			(READ_PHYREGCE(pi, RfctrlOverrideTxPus, core) & 0xF3FF));
	}
}


static void wlc_acphy_load_logen_tbl(phy_info_t *pi)
{
	/* 4349BU */
	if (ACMAJORREV_4(pi->pubpi->phy_rev))
		return;

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		if (phy_get_phymode(pi) == PHYMODE_MIMO) {
			/* set logen mimodes pu */
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x14d, 1, 1, 0, 16);
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x15d, 1, 1, 1, 16);
			/* Set logen mimosrc pu */
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x14d, 4, 4, 1, 16);
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x15d, 4, 4, 0, 16);
		} else {
			/* set logen mimodes pu */
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x14d, 1, 1, 0, 16);
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x15d, 1, 1, 0, 16);
			/* Set logen mimosrc pu */
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x14d, 4, 4, 0, 16);
			wlc_phy_radio20693_set_reset_table_bits(pi, ACPHY_TBL_ID_RFSEQ,
				0x15d, 4, 4, 0, 16);
		}
	}
}

static void
wlc_phy_set_regtbl_on_band_change_acphy_20693(phy_info_t *pi)
{

	uint8 core = 0;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20693_ID));

	FOREACH_CORE(pi, core)
	{
		if (CHSPEC_IS2G(pi->radio_chanspec))
		{
			phy_utils_write_radioreg(pi, RADIO_REG_20693(pi,
				TX_TOP_2G_OVR_EAST, core), 0x0);
			phy_utils_write_radioreg(pi, RADIO_REG_20693(pi,
				TX_TOP_2G_OVR1_EAST, core), 0x0);
			phy_utils_write_radioreg(pi, RADIO_REG_20693(pi,
				RX_TOP_2G_OVR_EAST, core), 0x0);
			phy_utils_write_radioreg(pi, RADIO_REG_20693(pi,
				RX_TOP_2G_OVR_EAST2, core), 0x0);

			if (RADIOMAJORREV(pi) == 2) {
				MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR2, core,
					ovr_mix5g_lobuf_en, 0);
				MOD_RADIO_REG_20693(pi, LNA5G_CFG3, core, mix5g_lobuf_en, 0);
			}
			MOD_RADIO_REG_20693(pi, TIA_CFG8, core, tia_offset_dac_biasadj, 1);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core, ovr_lna2g_tr_rx_en, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_CFG1, core, lna2g_tr_rx_en, 1);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_tr_rx_en, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG1, core, lna5g_tr_rx_en, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core, ovr_gm2g_auxgm_pwrup, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_CFG2, core, gm2g_auxgm_pwrup, 0);
			MOD_RADIO_REG_20693(pi, LOGEN_CFG2, core, logencore_5g_pu, 0);
			MOD_RADIO_REG_20693(pi, LOGEN_OVR1, core, ovr_logencore_5g_pu, 1);
			MOD_RADIO_REG_20693(pi, TX5G_CFG1, core, tx5g_bias_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core, ovr_tx5g_bias_pu, 1);
			MOD_RADIO_REG_20693(pi, TXMIX5G_CFG4, core, mx5g_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core, ovr_mx5g_pu, 1);
			MOD_RADIO_REG_20693(pi, TXMIX5G_CFG4, core, mx5g_pu_lodc_loop, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_mx5g_pu_lodc_loop, 1);
			MOD_RADIO_REG_20693(pi, PA5G_CFG1, core, pa5g_bias_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core, ovr_pa5g_bias_pu, 1);
			MOD_RADIO_REG_20693(pi, PA5G_CFG1, core, pa5g_bias_cas_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core, ovr_pa5g_bias_cas_pu, 1);
			MOD_RADIO_REG_20693(pi, PA5G_CFG4, core, pa5g_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core, ovr_pa5g_pu, 1);

			if ((RADIO20693_MAJORREV(pi->pubpi->radiorev) == 1) &&
				(RADIO20693_MINORREV(pi->pubpi->radiorev) == 1)) {
				MOD_RADIO_REG_20693(pi, TRSW5G_CFG1, core, trsw5g_pu, 0);
				MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core, ovr_trsw5g_pu, 1);
			}

			MOD_RADIO_REG_20693(pi, TX_LOGEN5G_CFG1, core, logen5g_tx_enable_5g, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core, ovr_logen5g_tx_enable_5g, 1);
			MOD_RADIO_REG_20693(pi, TX_LOGEN5G_CFG1, core,
				logen5g_tx_enable_5g_low_band, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core,
				ovr_logen5g_tx_enable_5g_low_band, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG1, core, lna5g_lna1_pu, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_lna1_pu, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG2, core, lna5g_pu_lna2, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_gm5g_pwrup, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_RSSI1, core, lna5g_dig_wrssi1_pu, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_dig_wrssi1_pu, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG2, core, lna5g_pu_auxlna2, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_pu_auxlna2, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG3, core, mix5g_en, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_mix5g_en, 1);
			MOD_RADIO_REG_20693(pi, TX_LPF_CFG2, core, lpf_sel_5g_out_gm, 0);
			MOD_RADIO_REG_20693(pi, TX_LPF_CFG3, core, lpf_sel_2g_5g_cmref_gm, 0);
			/* Bimodal settings */
			if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core,
					ovr_rxmix2g_pu, 1);
				MOD_RADIO_REG_20693(pi, RXMIX2G_CFG1, core, rxmix2g_pu, 1);
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core,
					ovr_rxdiv2g_rs, 1);
				MOD_RADIO_REG_20693(pi, RXRF2G_CFG1, core, rxdiv2g_rs, 0);
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core,
					ovr_rxdiv2g_pu_bias, 1);
				MOD_RADIO_REG_20693(pi, RXRF2G_CFG1, core, rxdiv2g_pu_bias, 1);
				/* Turn off 5g overrides */
				MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core,
					ovr_mix5g_en, 0);
				MOD_RADIO_REG_20693(pi, LNA5G_CFG3, core, mix5g_en, 0);
				if (!(PHY_IPA(pi)) && (RADIO20693REV(pi->pubpi->radiorev) == 13)) {
					wlc_phy_set_bias_ipa_as_epa_acphy_20693(pi, core);
				}
			}
		}
		else
		{
			phy_utils_write_radioreg(pi, RADIO_REG_20693(pi, TX_TOP_5G_OVR1, core), 0);
			phy_utils_write_radioreg(pi, RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core), 0);

			if (RADIOMAJORREV(pi) == 2) {
				MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR2, core,
					ovr_mix5g_lobuf_en, 1);
				MOD_RADIO_REG_20693(pi, LNA5G_CFG3, core, mix5g_lobuf_en, 1);
			}
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_lna1_pu, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_gm5g_pwrup, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_dig_wrssi1_pu, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_pu_auxlna2, 1);
			MOD_RADIO_REG_20693(pi, TIA_CFG8, core, tia_offset_dac_biasadj, 1);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core, ovr_lna2g_tr_rx_en, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_CFG1, core, lna2g_tr_rx_en, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_lna5g_tr_rx_en, 1);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG1, core, lna5g_tr_rx_en, 1);
			MOD_RADIO_REG_20693(pi, LOGEN_CFG2, core, logencore_2g_pu, 0);
			MOD_RADIO_REG_20693(pi, LOGEN_OVR1, core, ovr_logencore_2g_pu, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_CFG2, core, gm2g_auxgm_pwrup, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core, ovr_gm2g_auxgm_pwrup, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_CFG2, core, gm2g_pwrup, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core, ovr_gm2g_pwrup, 1);
			MOD_RADIO_REG_20693(pi, TX2G_CFG1, core, tx2g_bias_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core, ovr_tx2g_bias_pu, 1);
			MOD_RADIO_REG_20693(pi, TXMIX2G_CFG2, core, mx2g_bias_en, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core, ovr_mx2g_bias_en, 1);
			MOD_RADIO_REG_20693(pi, PA2G_CFG1, core, pa2g_bias_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core, ovr_pa2g_bias_pu, 1);
			MOD_RADIO_REG_20693(pi, PA2G_CFG1, core, pa2g_2gtx_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core, ovr_pa2g_2gtx_pu, 1);
			MOD_RADIO_REG_20693(pi, PA2G_IDAC2, core, pa2g_bias_cas_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core, ovr_pa2g_bias_cas_pu, 1);
			MOD_RADIO_REG_20693(pi, TX_LOGEN2G_CFG1, core, logen2g_tx_pu_bias, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core,
				ovr_logen2g_tx_pu_bias, 1);
			MOD_RADIO_REG_20693(pi, TX_LOGEN2G_CFG1, core, logen2g_tx_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR_EAST, core, ovr_logen2g_tx_pu, 1);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core, ovr_rxmix2g_pu, 1);
			MOD_RADIO_REG_20693(pi, RXMIX2G_CFG1, core, rxmix2g_pu, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core,
				ovr_lna2g_dig_wrssi1_pu, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_RSSI1, core, lna2g_dig_wrssi1_pu, 0);
			MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core, ovr_lna2g_lna1_pu, 1);
			MOD_RADIO_REG_20693(pi, LNA2G_CFG1, core, lna2g_lna1_pu, 0);
			MOD_RADIO_REG_20693(pi, LNA5G_CFG3, core, mix5g_en, 1);
			MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core, ovr_mix5g_en, 1);
			MOD_RADIO_REG_20693(pi, LOGEN_OVR1, core, ovr_logencore_5g_pu, 0);
			MOD_RADIO_REG_20693(pi, TX_LPF_CFG2, core, lpf_sel_5g_out_gm, 1);
			MOD_RADIO_REG_20693(pi, TX_LPF_CFG3, core, lpf_sel_2g_5g_cmref_gm, 1);
			/* Bimodal settings */
			if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				MOD_RADIO_REG_20693(pi, RX_TOP_5G_OVR, core,
					ovr_mix5g_en, 1);
				MOD_RADIO_REG_20693(pi, LNA5G_CFG3, core, mix5g_en, 1);
				/* Turn off 2G overrides */
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core,
					ovr_rxmix2g_pu, 0);
				MOD_RADIO_REG_20693(pi, RXMIX2G_CFG1, core, rxmix2g_pu, 0);
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core,
					ovr_rxdiv2g_rs, 0);
				MOD_RADIO_REG_20693(pi, RXRF2G_CFG1, core, rxdiv2g_rs, 0);
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST, core,
					ovr_rxdiv2g_pu_bias, 0);
				MOD_RADIO_REG_20693(pi, RXRF2G_CFG1, core,
					rxdiv2g_pu_bias, 0);
			}
		} /* band */
	} /* foreach core */
}

static void
wlc_phy_load_channel_smoothing_tiny(phy_info_t *pi)
{

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
	    /* set 64 48-bit entries */
		wlc_phy_table_write_tiny_chnsmth(pi,
			ACPHY_TBL_ID_CORE0CHANSMTH_FLTR,
			CHANSMTH_FLTR_LENGTH, 0, 48, acphy_Smth_tbl_4349);
		if (phy_get_phymode(pi) == PHYMODE_MIMO) {
			wlc_phy_table_write_tiny_chnsmth(pi,
				ACPHY_TBL_ID_CORE1CHANSMTH_FLTR,
				CHANSMTH_FLTR_LENGTH, 0, 48, acphy_Smth_tbl_4349);
		}
	} else {
	    const uint16 zero_table[3] = { 0, 0, 0 };
	    acphytbl_info_t tbl;
	    tbl.tbl_id = ACPHY_TBL_ID_CHANNELSMOOTHING_1x1;
		tbl.tbl_ptr = zero_table;
		tbl.tbl_len = 1;
		tbl.tbl_offset = 0;
		tbl.tbl_width = 48;
		/* clear 1st 128 48-bit entries */
		for (tbl.tbl_offset = 0; tbl.tbl_offset < 128; tbl.tbl_offset++) {
			wlc_phy_table_write_ext_acphy(pi, &tbl);
		}

		/* set next 64 48-bit entries */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_CHANNELSMOOTHING_1x1,
		                          CHANSMTH_FLTR_LENGTH, 128,
		                          tbl.tbl_width, acphy_Smth_tbl_tiny);

		/* clear next 64 48-bit entries */
		for (tbl.tbl_offset = 128 + (ARRAYSIZE(acphy_Smth_tbl_tiny) / 3);
		     tbl.tbl_offset < 256;
		     tbl.tbl_offset++) {
			wlc_phy_table_write_ext_acphy(pi, &tbl);
		}
	}
}
static void
wlc_phy_set_reg_on_reset_acphy_20693(phy_info_t *pi)
{

}
static void
wlc_phy_radio20691_xtal_tune(phy_info_t *pi)
{
	/* Channel has changed */

	/* Return the following to zero after channel change */
	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_RCCAL, 0x0);
	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_RCCAL1, 0x0);

	/*
	 * pll_xtal2.xtal_pu_caldrv handled differently in driver than in Tcl due to impact on
	 * VCO cal and ucode. SW4345-327
	 */

	/*
	 * These will only work with BT held in reset.
	 * Write the BT CLB register equivalent to 0x0 through backplane to get this to work
	 * in BT+WLAN mode
	 */
	MOD_RADIO_REG_20691(pi, PLL_XTAL_OVR1, 0, ovr_xtal_pu_corebuf_pfd, 0x1);
	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_corebuf_pfd, 0x0);

	/* This will only work if BT is held at reset. CANNOT use it in either BT or BT+WLAN mode */
	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_BT, 0x0);

	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_corebuf_bb, 0x1);

	MOD_RADIO_REG_20691(pi, PLL_XTAL4, 0, xtal_outbufBBstrg, 0x0);

	MOD_RADIO_REG_20691(pi, PLL_XTAL_OVR1, 0, ovr_xtal_coresize_pmos, 0x1);
	MOD_RADIO_REG_20691(pi, PLL_XTAL_OVR1, 0, ovr_xtal_coresize_nmos, 0x1);

	/* pll_xtal3[11] (xtal_core_change) is unwired after 4345A0, don't touch */
	MOD_RADIO_REG_20691(pi, PLL_XTAL3, 0, xtal_refsel, 0x4);
	MOD_RADIO_REG_20691(pi, PLL_XTAL3, 0, xtal_xtal_swcap_in, 0x8);
	MOD_RADIO_REG_20691(pi, PLL_XTAL3, 0, xtal_xtal_swcap_out, 0x8);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		MOD_RADIO_REG_20691(pi, PLL_XTAL1, 0, xtal_coresize_pmos, 0x4);
		MOD_RADIO_REG_20691(pi, PLL_XTAL1, 0, xtal_coresize_nmos, 0x4);

		/* HSIC=On helps in 2G. Keep HSIC ON while reducing buffer strength to minimum */
		MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_HSIC, 0x1);

		/* 0x3 if doubler ON, 0x1 if doubler OFF */
		MOD_RADIO_REG_20691(pi, PLL_XTAL4, 0, xtal_xtbufstrg, 0x3);
		MOD_RADIO_REG_20691(pi, PLL_XTAL4, 0, xtal_outbufstrg, 0x2);

		/* To reduce a few of the output strengths a bit */
		MOD_RADIO_REG_20691(pi, PLL_XTAL6, 0, xtal_bufstrg_HSIC, 0x0);
		MOD_RADIO_REG_20691(pi, PLL_XTAL6, 0, xtal_bufstrg_gci, 0x0);

		/* LDO voltage reduced to 4345B0@~1.12V, pll_xtalldo1 = 0x16e */
		MOD_RADIO_REG_20691(pi, PLL_XTALLDO1, 0, ldo_1p2_xtalldo1p2_ctl, 0xb);
	} else if (CHSPEC_IS5G(pi->radio_chanspec)) {
		MOD_RADIO_REG_20691(pi, PLL_XTAL1, 0, xtal_coresize_pmos, 0x10);
		MOD_RADIO_REG_20691(pi, PLL_XTAL1, 0, xtal_coresize_nmos, 0x10);

		MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_HSIC, 0x0);

		/* doubler no concern in 5G */
		MOD_RADIO_REG_20691(pi, PLL_XTAL4, 0, xtal_xtbufstrg, 0x7);
		MOD_RADIO_REG_20691(pi, PLL_XTAL4, 0, xtal_outbufstrg, 0x4);

		/* To reduce a few of the output strengths a bit */
		MOD_RADIO_REG_20691(pi, PLL_XTAL6, 0, xtal_bufstrg_HSIC, 0x4);
		MOD_RADIO_REG_20691(pi, PLL_XTAL6, 0, xtal_bufstrg_gci, 0x4);

		/* LDO voltage reduced to 4345B0@~1.12V, pll_xtalldo1 = 0x1ce */
		MOD_RADIO_REG_20691(pi, PLL_XTALLDO1, 0, ldo_1p2_xtalldo1p2_ctl, 0xe);
	}
}

static void
wlc_phy_radio20691_xtal_tune_prep(phy_info_t *pi)
{
	/* Enable before changing channel */
	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_RCCAL, 0x1);
	MOD_RADIO_REG_20691(pi, PLL_XTAL2, 0, xtal_pu_RCCAL1, 0x1);
}

void
wlc_phy_chanspec_radio20691_setup(phy_info_t *pi, uint8 ch, uint8 toggle_logen_reset)
{
	const chan_info_radio20691_t *ci20691;
	uint8 itr = 0;
	uint8 ovr_rxdiv2g_rs = 0;
	uint8 ovr_rxdiv2g_pu_bias = 0;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20691_ID));

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	if (wlc_phy_chan2freq_20691(pi, ch, &ci20691) < 0)
		return;

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		/* save the ovr_rxdiv2g radio registers */
		ovr_rxdiv2g_rs = READ_RADIO_REGFLD_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_rxdiv2g_rs);
		ovr_rxdiv2g_pu_bias = READ_RADIO_REGFLD_20691(pi, RX_TOP_2G_OVR_EAST, 0,
		                                              ovr_rxdiv2g_pu_bias);

		/* disable trimodal DC WAR */
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_rxdiv2g_rs, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_rxdiv2g_pu_bias, 0);
	}

	wlc_phy_radio20691_xtal_tune_prep(pi);

	/* logen_reset needs to be toggled whenever bandsel bit if changed */
	/* On a bw change, phy_reset is issued which causes currentBand getting reset to 0 */
	/* So, issue this on both band & bw change */
	if (toggle_logen_reset == 1) {
		wlc_phy_logen_reset(pi, 0);
	}

	/* 20691_radio_tune() */
	/* Write chan specific tuning register */
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_VCOCAL1, 0), ci20691->RF_pll_vcocal1);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_VCOCAL11, 0),
	                         ci20691->RF_pll_vcocal11);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_VCOCAL12, 0),
	                         ci20691->RF_pll_vcocal12);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_FRCT2, 0), ci20691->RF_pll_frct2);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_FRCT3, 0), ci20691->RF_pll_frct3);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, LOGEN_CFG2, 0), ci20691->RF_logen_cfg2);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_LF4, 0), ci20691->RF_pll_lf4);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_LF5, 0), ci20691->RF_pll_lf5);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_LF7, 0), ci20691->RF_pll_lf7);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_LF2, 0), ci20691->RF_pll_lf2);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_LF3, 0), ci20691->RF_pll_lf3);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, LOGEN_CFG1, 0), ci20691->RF_logen_cfg1);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, LNA2G_TUNE, 0), ci20691->RF_lna2g_tune);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, TXMIX2G_CFG5, 0),
	                         ci20691->RF_txmix2g_cfg5);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PA2G_CFG2, 0), ci20691->RF_pa2g_cfg2);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, LNA5G_TUNE, 0), ci20691->RF_lna5g_tune);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, TXMIX5G_CFG6, 0),
	                         ci20691->RF_txmix5g_cfg6);
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PA5G_CFG4, 0), ci20691->RF_pa5g_cfg4);

	if (ci20691->chan > CH_MAX_2G_CHANNEL && ci20691->freq < 5250 &&
	    ci20691->freq != 5190 && ci20691->freq != 5230) {
		MOD_RADIO_REG_20691(pi, PLL_VCO2, 0, rfpll_vco_cap_mode, 1);
	} else {
		MOD_RADIO_REG_20691(pi, PLL_VCO2, 0, rfpll_vco_cap_mode, 0);
	}

	/* # The reset/preferred value for ldo_vco and ldo_cp is 0 which is correct
	 * # The above tuning function writes to the whole hvldo register, instead of
	 * read-modify-write (easy for driver guys) and puts back the ldo_vco and ldo_cp
	 * registers to 0 (i.e. to their preferred values)
	 * # The problem is that PHY is not using direct control to turn on ldo_VCO and
	 * ldo_CP and hence it is important to make sure that the radio jtag value for
	 * these is 1 and not 0
	 */
	phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, PLL_XTAL3, 0), 0x488);

	MOD_RADIO_REG_20691(pi, PLL_HVLDO1, 0, ldo_2p5_pu_ldo_VCO, 1);
	MOD_RADIO_REG_20691(pi, PLL_HVLDO1, 0, ldo_2p5_pu_ldo_CP, 1);

	/* 4345a0: Current optimization */
	/* Value for normal power mode, moved out of wlc_phy_radio_tiny_vcocal(). */
	MOD_RADIO_REG_20691(pi, PLL_VCO6, 0, rfpll_vco_ALC_ref_ctrl, 0xf);
	acphy_set_lpmode(pi, ACPHY_LP_RADIO_LVL_OPT);

	/* Do a VCO cal after writing the tuning table regs */
	do {
		wlc_phy_radio_tiny_vcocal(pi);
		itr++;
		if (itr > 2)
			break;
	} while (READ_RADIO_REGFLD_20691(pi, PLL_DSPR27, 0, rfpll_monitor_need_refresh) == 1);

	wlc_phy_radio20691_xtal_tune(pi);

	/* restore the ovr_rxdiv2g radio registers */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_rxdiv2g_rs, ovr_rxdiv2g_rs);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_rxdiv2g_pu_bias,
		                    ovr_rxdiv2g_pu_bias);
	}
}

void
wlc_phy_chanspec_radio2069_setup(phy_info_t *pi, const void *chan_info, uint8 toggle_logen_reset)
{
	uint8 core;
	uint32 fc = wf_channel2mhz(CHSPEC_CHANNEL(pi->radio_chanspec),
	        CHSPEC_IS2G(pi->radio_chanspec) ? WF_CHAN_FACTOR_2_4_G
	        : WF_CHAN_FACTOR_5_G);
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));
	ASSERT(chan_info != NULL);

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	/* logen_reset needs to be toggled whenever bandsel bit if changed */
	/* On a bw change, phy_reset is issued which causes currentBand getting reset to 0 */
	/* So, issue this on both band & bw change */
	if (toggle_logen_reset == 1) {
		wlc_phy_logen_reset(pi, 0);
	}

	if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) {
		const chan_info_radio2069revGE32_t *ciGE32 = chan_info;

		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5, ciGE32->RFP_pll_vcocal5);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6, ciGE32->RFP_pll_vcocal6);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2, ciGE32->RFP_pll_vcocal2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1, ciGE32->RFP_pll_vcocal1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11, ciGE32->RFP_pll_vcocal11);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12, ciGE32->RFP_pll_vcocal12);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2, ciGE32->RFP_pll_frct2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3, ciGE32->RFP_pll_frct3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10, ciGE32->RFP_pll_vcocal10);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3, ciGE32->RFP_pll_xtal3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2, ciGE32->RFP_pll_vco2);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1, ciGE32->RFP_logen5g_cfg1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8, ciGE32->RFP_pll_vco8);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6, ciGE32->RFP_pll_vco6);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3, ciGE32->RFP_pll_vco3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1, ciGE32->RFP_pll_xtalldo1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1, ciGE32->RFP_pll_hvldo1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2, ciGE32->RFP_pll_hvldo2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5, ciGE32->RFP_pll_vco5);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4, ciGE32->RFP_pll_vco4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, ciGE32->RFP_pll_lf4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, ciGE32->RFP_pll_lf5);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, ciGE32->RFP_pll_lf7);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, ciGE32->RFP_pll_lf2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, ciGE32->RFP_pll_lf3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, ciGE32->RFP_pll_cp4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF6, ciGE32->RFP_pll_lf6);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL4, ciGE32->RFP_pll_xtal4);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN2G_TUNE, ciGE32->RFP_logen2g_tune);
		phy_utils_write_radioreg(pi, RFX_2069_LNA2G_TUNE, ciGE32->RFX_lna2g_tune);
		phy_utils_write_radioreg(pi, RFX_2069_TXMIX2G_CFG1, ciGE32->RFX_txmix2g_cfg1);
		phy_utils_write_radioreg(pi, RFX_2069_PGA2G_CFG2, ciGE32->RFX_pga2g_cfg2);
		phy_utils_write_radioreg(pi, RFX_2069_PAD2G_TUNE, ciGE32->RFX_pad2g_tune);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1, ciGE32->RFP_logen5g_tune1);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE2, ciGE32->RFP_logen5g_tune2);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_IDAC1, ciGE32->RFP_logen5g_idac1);
		phy_utils_write_radioreg(pi, RFX_2069_LNA5G_TUNE, ciGE32->RFX_lna5g_tune);
		phy_utils_write_radioreg(pi, RFX_2069_TXMIX5G_CFG1, ciGE32->RFX_txmix5g_cfg1);
		phy_utils_write_radioreg(pi, RFX_2069_PGA5G_CFG2, ciGE32->RFX_pga5g_cfg2);
		phy_utils_write_radioreg(pi, RFX_2069_PAD5G_TUNE, ciGE32->RFX_pad5g_tune);
		if ((RADIO2069_MINORREV(pi->pubpi->radiorev) == 4) &&
		    pi->sh->chippkg == 2 && pi->xtalfreq == 37400000) {
			if (fc == 5290) {
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5, 0X5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6, 0x1C);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2, 0xA09);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1, 0xF89);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11, 0xD4);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12, 0x2A70);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2, 0x350);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3, 0xA9C1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10, 0x0);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3, 0x488);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2, 0xCE8);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1, 0x40);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8, 0xB);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6, 0x1D6f);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3, 0x1F00);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1, 0x780);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1, 0x0);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2, 0x0);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5, 0x49C);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4, 0x3504);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, 0xB);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, 0xB);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, 0xD6F);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, 0xECBE);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, 0xDDE2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, 0xBC28);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF6, 0x1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL4, 0x36FF);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1, 0x80);
			} else if (fc == 5180) {
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5, 0x5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6, 0x1C);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2, 0xA09);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1, 0xF37);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11, 0xCF);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12, 0xC106);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2, 0x33F);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3, 0x41B);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10, 0x0);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3, 0x488);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2, 0xCE8);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1, 0x40);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8, 0xB);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6, 0x1D6F);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3, 0x1F00);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1, 0x780);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1, 0x0);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2, 0x0);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5, 0x49C);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4, 0x3505);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, 0xB);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, 0xB);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, 0xD6D);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, 0xF1C3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, 0xE2E7);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, 0xBC28);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF6, 0x1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL4, 0x36CF);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1, 0xA0);
			}
		}
		/* Move nbclip by 2dBs to the right */
		FOREACH_CORE(pi, core) {
			MOD_RADIO_REGC(pi, NBRSSI_CONFG, core, nbrssi_ib_Refladder, 7);
			MOD_RADIO_REGC(pi, DAC_CFG1, core, DAC_invclk, 1);
		}

		/* Fix drift/unlock behavior */
		MOD_RADIO_REG(pi, RFP, PLL_CFG3, rfpll_spare1, 0x8);

	} else if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) {
		if ((RADIO2069REV(pi->pubpi->radiorev) != 25) &&
			(RADIO2069REV(pi->pubpi->radiorev) != 26)) {
			const chan_info_radio2069revGE16_t *ciGE16 = chan_info;

			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5, ciGE16->RFP_pll_vcocal5);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6, ciGE16->RFP_pll_vcocal6);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2, ciGE16->RFP_pll_vcocal2);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1, ciGE16->RFP_pll_vcocal1);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11,
			                         ciGE16->RFP_pll_vcocal11);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12,
			                         ciGE16->RFP_pll_vcocal12);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2, ciGE16->RFP_pll_frct2);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3, ciGE16->RFP_pll_frct3);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10,
			                         ciGE16->RFP_pll_vcocal10);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3, ciGE16->RFP_pll_xtal3);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2, ciGE16->RFP_pll_vco2);
			phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1,
			                         ciGE16->RFP_logen5g_cfg1);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8, ciGE16->RFP_pll_vco8);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6, ciGE16->RFP_pll_vco6);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3, ciGE16->RFP_pll_vco3);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1,
			                         ciGE16->RFP_pll_xtalldo1);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1, ciGE16->RFP_pll_hvldo1);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2, ciGE16->RFP_pll_hvldo2);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5, ciGE16->RFP_pll_vco5);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4, ciGE16->RFP_pll_vco4);

			phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, ciGE16->RFP_pll_lf4);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, ciGE16->RFP_pll_lf5);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, ciGE16->RFP_pll_lf7);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, ciGE16->RFP_pll_lf2);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, ciGE16->RFP_pll_lf3);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, ciGE16->RFP_pll_cp4);
			phy_utils_write_radioreg(pi, RFP_2069_PLL_LF6, ciGE16->RFP_pll_lf6);

			phy_utils_write_radioreg(pi, RF0_2069_LOGEN2G_TUNE,
			                         ciGE16->RFP_logen2g_tune);
			phy_utils_write_radioreg(pi, RF0_2069_LNA2G_TUNE, ciGE16->RF0_lna2g_tune);
			phy_utils_write_radioreg(pi, RF0_2069_TXMIX2G_CFG1,
			                         ciGE16->RF0_txmix2g_cfg1);
			phy_utils_write_radioreg(pi, RF0_2069_PGA2G_CFG2, ciGE16->RF0_pga2g_cfg2);
			phy_utils_write_radioreg(pi, RF0_2069_PAD2G_TUNE, ciGE16->RF0_pad2g_tune);
			phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1,
			                         ciGE16->RFP_logen5g_tune1);
			phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE2,
			                         ciGE16->RFP_logen5g_tune2);
			phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_RCCR,
			                         ciGE16->RF0_logen5g_rccr);
			phy_utils_write_radioreg(pi, RF0_2069_LNA5G_TUNE, ciGE16->RF0_lna5g_tune);
			phy_utils_write_radioreg(pi, RF0_2069_TXMIX5G_CFG1,
			                         ciGE16->RF0_txmix5g_cfg1);
			phy_utils_write_radioreg(pi, RF0_2069_PGA5G_CFG2, ciGE16->RF0_pga5g_cfg2);
			phy_utils_write_radioreg(pi, RF0_2069_PAD5G_TUNE, ciGE16->RF0_pad5g_tune);
			/*
			* phy_utils_write_radioreg(pi, RFP_2069_PLL_CP5, ciGE16->RFP_pll_cp5);
			* phy_utils_write_radioreg(pi, RF0_2069_AFEDIV1, ciGE16->RF0_afediv1);
			* phy_utils_write_radioreg(pi, RF0_2069_AFEDIV2, ciGE16->RF0_afediv2);
			* phy_utils_write_radioreg(pi, RF0_2069_ADC_CFG5, ciGE16->RF0_adc_cfg5);
			*/

		} else {
			if (pi->xtalfreq != 52000000) {
				const chan_info_radio2069revGE25_t *ciGE25 = chan_info;
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5,
				                         ciGE25->RFP_pll_vcocal5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6,
				                         ciGE25->RFP_pll_vcocal6);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2,
				                         ciGE25->RFP_pll_vcocal2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1,
				                         ciGE25->RFP_pll_vcocal1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11,
					ciGE25->RFP_pll_vcocal11);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12,
					ciGE25->RFP_pll_vcocal12);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2,
				                         ciGE25->RFP_pll_frct2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3,
				                         ciGE25->RFP_pll_frct3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10,
					ciGE25->RFP_pll_vcocal10);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3,
				                         ciGE25->RFP_pll_xtal3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_CFG3,
				                         ciGE25->RFP_pll_cfg3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2,
				                         ciGE25->RFP_pll_vco2);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1,
					ciGE25->RFP_logen5g_cfg1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8,
				                         ciGE25->RFP_pll_vco8);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6,
				                         ciGE25->RFP_pll_vco6);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3,
				                         ciGE25->RFP_pll_vco3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1,
					ciGE25->RFP_pll_xtalldo1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1,
				                         ciGE25->RFP_pll_hvldo1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2,
				                         ciGE25->RFP_pll_hvldo2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5,
				                         ciGE25->RFP_pll_vco5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4,
				                         ciGE25->RFP_pll_vco4);

				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, ciGE25->RFP_pll_lf4);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, ciGE25->RFP_pll_lf5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, ciGE25->RFP_pll_lf7);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, ciGE25->RFP_pll_lf2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, ciGE25->RFP_pll_lf3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, ciGE25->RFP_pll_cp4);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF6, ciGE25->RFP_pll_lf6);

				phy_utils_write_radioreg(pi, RF0_2069_LOGEN2G_TUNE,
					ciGE25->RFP_logen2g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_LNA2G_TUNE,
				                         ciGE25->RF0_lna2g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_TXMIX2G_CFG1,
					ciGE25->RF0_txmix2g_cfg1);
				phy_utils_write_radioreg(pi, RF0_2069_PGA2G_CFG2,
				                         ciGE25->RF0_pga2g_cfg2);
				phy_utils_write_radioreg(pi, RF0_2069_PAD2G_TUNE,
				                         ciGE25->RF0_pad2g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1,
					ciGE25->RFP_logen5g_tune1);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE2,
					ciGE25->RFP_logen5g_tune2);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_RCCR,
					ciGE25->RF0_logen5g_rccr);
				phy_utils_write_radioreg(pi, RF0_2069_LNA5G_TUNE,
				                         ciGE25->RF0_lna5g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_TXMIX5G_CFG1,
					ciGE25->RF0_txmix5g_cfg1);
				phy_utils_write_radioreg(pi, RF0_2069_PGA5G_CFG2,
				                         ciGE25->RF0_pga5g_cfg2);
				phy_utils_write_radioreg(pi, RF0_2069_PAD5G_TUNE,
				                         ciGE25->RF0_pad5g_tune);

				/*
				 * phy_utils_write_radioreg(pi, RFP_2069_PLL_CP5,
				 *                          ciGE25->RFP_pll_cp5);
				 * phy_utils_write_radioreg(pi, RF0_2069_AFEDIV1,
				 *                          ciGE25->RF0_afediv1);
				 * phy_utils_write_radioreg(pi, RF0_2069_AFEDIV2,
				 *                          ciGE25->RF0_afediv2);
				 * phy_utils_write_radioreg(pi, RF0_2069_ADC_CFG5,
				 *                          ciGE25->RF0_adc_cfg5);
				 */

			} else {
				const chan_info_radio2069revGE25_52MHz_t *ciGE25 = chan_info;

				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5,
				                         ciGE25->RFP_pll_vcocal5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6,
				                         ciGE25->RFP_pll_vcocal6);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2,
				                         ciGE25->RFP_pll_vcocal2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1,
				                         ciGE25->RFP_pll_vcocal1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11,
					ciGE25->RFP_pll_vcocal11);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12,
					ciGE25->RFP_pll_vcocal12);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2,
				                         ciGE25->RFP_pll_frct2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3,
				                         ciGE25->RFP_pll_frct3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10,
					ciGE25->RFP_pll_vcocal10);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3,
				                         ciGE25->RFP_pll_xtal3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2,
				                         ciGE25->RFP_pll_vco2);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1,
					ciGE25->RFP_logen5g_cfg1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8,
				                         ciGE25->RFP_pll_vco8);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6,
				                         ciGE25->RFP_pll_vco6);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3,
				                         ciGE25->RFP_pll_vco3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1,
					ciGE25->RFP_pll_xtalldo1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1,
				                         ciGE25->RFP_pll_hvldo1);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2,
				                         ciGE25->RFP_pll_hvldo2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5,
				                         ciGE25->RFP_pll_vco5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4,
				                         ciGE25->RFP_pll_vco4);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, ciGE25->RFP_pll_lf4);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, ciGE25->RFP_pll_lf5);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, ciGE25->RFP_pll_lf7);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, ciGE25->RFP_pll_lf2);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, ciGE25->RFP_pll_lf3);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, ciGE25->RFP_pll_cp4);
				phy_utils_write_radioreg(pi, RFP_2069_PLL_LF6, ciGE25->RFP_pll_lf6);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN2G_TUNE,
					ciGE25->RFP_logen2g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_LNA2G_TUNE,
				                         ciGE25->RF0_lna2g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_TXMIX2G_CFG1,
					ciGE25->RF0_txmix2g_cfg1);
				phy_utils_write_radioreg(pi, RF0_2069_PGA2G_CFG2,
				                         ciGE25->RF0_pga2g_cfg2);
				phy_utils_write_radioreg(pi, RF0_2069_PAD2G_TUNE,
				                         ciGE25->RF0_pad2g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1,
					ciGE25->RFP_logen5g_tune1);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE2,
					ciGE25->RFP_logen5g_tune2);
				phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_RCCR,
					ciGE25->RF0_logen5g_rccr);
				phy_utils_write_radioreg(pi, RF0_2069_LNA5G_TUNE,
				                         ciGE25->RF0_lna5g_tune);
				phy_utils_write_radioreg(pi, RF0_2069_TXMIX5G_CFG1,
					ciGE25->RF0_txmix5g_cfg1);
				phy_utils_write_radioreg(pi, RF0_2069_PGA5G_CFG2,
				                         ciGE25->RF0_pga5g_cfg2);
				phy_utils_write_radioreg(pi, RF0_2069_PAD5G_TUNE,
				                         ciGE25->RF0_pad5g_tune);
			}

			/* 43162 FCBGA Settings improving Tx EVM */
			/* (1) ch4/ch4m settings to reduce 500k xtal spur */
			/* (2) Rreducing 2440/2480 RX spur */
			if (RADIO2069REV(pi->pubpi->radiorev) == 25 && pi->xtalfreq == 40000000) {
				MOD_RADIO_REG(pi, RFP, GE16_OVR23, ovr_xtal_coresize_nmos, 0x1);
				MOD_RADIO_REG(pi, RFP, PLL_XTAL1, xtal_coresize_nmos, 0x8);
				MOD_RADIO_REG(pi, RFP, GE16_OVR23, ovr_xtal_coresize_pmos, 0x1);
				MOD_RADIO_REG(pi, RFP, PLL_XTAL1, xtal_coresize_pmos, 0x8);

				if (CHSPEC_IS2G(pi->radio_chanspec)) {
				if (CHSPEC_CHANNEL(pi->radio_chanspec) < 12) {
					if (CHSPEC_CHANNEL(pi->radio_chanspec) == 4)
						phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2,
						                         0xce4);

					MOD_RADIO_REG(pi, RFP, PLL_XTAL5, xtal_bufstrg_BT, 0x1);
					MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_xtbufstrg, 0x1);
					MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_xtbufstrg, 0x7);
					MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_xtbufstrg, 0x1);
					MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufstrg, 0x3);
				} else {
					MOD_RADIO_REG(pi, RFP, PLL_XTAL5, xtal_bufstrg_BT, 0x1);
					MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_xtbufstrg, 0x1);
					MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_xtbufstrg, 0x0);
					MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_xtbufstrg, 0x1);
					MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufstrg, 0x1);
				}
				}
			}
		}
	} else {
		const chan_info_radio2069_t *ci = chan_info;

		/* Write chan specific tuning register */
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL5, ci->RFP_pll_vcocal5);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL6, ci->RFP_pll_vcocal6);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL2, ci->RFP_pll_vcocal2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL1, ci->RFP_pll_vcocal1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL11, ci->RFP_pll_vcocal11);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL12, ci->RFP_pll_vcocal12);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT2, ci->RFP_pll_frct2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_FRCT3, ci->RFP_pll_frct3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCOCAL10, ci->RFP_pll_vcocal10);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_XTAL3, ci->RFP_pll_xtal3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2, ci->RFP_pll_vco2);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_CFG1, ci->RF0_logen5g_cfg1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO8, ci->RFP_pll_vco8);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO6, ci->RFP_pll_vco6);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO3, ci->RFP_pll_vco3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_XTALLDO1, ci->RFP_pll_xtalldo1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO1, ci->RFP_pll_hvldo1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_HVLDO2, ci->RFP_pll_hvldo2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO5, ci->RFP_pll_vco5);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO4, ci->RFP_pll_vco4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF4, ci->RFP_pll_lf4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, ci->RFP_pll_lf5);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF7, ci->RFP_pll_lf7);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, ci->RFP_pll_lf2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, ci->RFP_pll_lf3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_CP4, ci->RFP_pll_cp4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP1, ci->RFP_pll_dsp1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP2, ci->RFP_pll_dsp2);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP3, ci->RFP_pll_dsp3);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP4, ci->RFP_pll_dsp4);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP6, ci->RFP_pll_dsp6);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP7, ci->RFP_pll_dsp7);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP8, ci->RFP_pll_dsp8);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_DSP9, ci->RFP_pll_dsp9);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN2G_TUNE, ci->RF0_logen2g_tune);
		phy_utils_write_radioreg(pi, RFX_2069_LNA2G_TUNE, ci->RFX_lna2g_tune);
		phy_utils_write_radioreg(pi, RFX_2069_TXMIX2G_CFG1, ci->RFX_txmix2g_cfg1);
		phy_utils_write_radioreg(pi, RFX_2069_PGA2G_CFG2, ci->RFX_pga2g_cfg2);
		phy_utils_write_radioreg(pi, RFX_2069_PAD2G_TUNE, ci->RFX_pad2g_tune);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE1, ci->RF0_logen5g_tune1);
		phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_TUNE2, ci->RF0_logen5g_tune2);
		phy_utils_write_radioreg(pi, RFX_2069_LOGEN5G_RCCR, ci->RFX_logen5g_rccr);
		phy_utils_write_radioreg(pi, RFX_2069_LNA5G_TUNE, ci->RFX_lna5g_tune);
		phy_utils_write_radioreg(pi, RFX_2069_TXMIX5G_CFG1, ci->RFX_txmix5g_cfg1);
		phy_utils_write_radioreg(pi, RFX_2069_PGA5G_CFG2, ci->RFX_pga5g_cfg2);
		phy_utils_write_radioreg(pi, RFX_2069_PAD5G_TUNE, ci->RFX_pad5g_tune);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_CP5, ci->RFP_pll_cp5);
		phy_utils_write_radioreg(pi, RF0_2069_AFEDIV1, ci->RF0_afediv1);
		phy_utils_write_radioreg(pi, RF0_2069_AFEDIV2, ci->RF0_afediv2);
		phy_utils_write_radioreg(pi, RFX_2069_ADC_CFG5, ci->RFX_adc_cfg5);

		/* We need different values for ADC_CFG5 for cores 1 and 2
		 * in order to get the best reduction of spurs from the AFE clk
		 */
		if (RADIO2069REV(pi->pubpi->radiorev) < 4) {
			phy_utils_write_radioreg(pi, RF1_2069_ADC_CFG5, 0x3e9);
			phy_utils_write_radioreg(pi, RF2_2069_ADC_CFG5, 0x3e9);
			MOD_RADIO_REG(pi, RFP, PLL_CP4, rfpll_cp_ioff, 0xa0);
		}

		/* Reduce 500 KHz spur at fc=2427 MHz for both 4360 A0 and B0 */
		if (CHSPEC_CHANNEL(pi->radio_chanspec) == 4) {
			phy_utils_write_radioreg(pi, RFP_2069_PLL_VCO2, 0xce4);
			MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_xtbufstrg, 0x5);
		}

		/* 43602 XTAL SPUR 2G WAR */
		if (ACMAJORREV_5(pi->pubpi->phy_rev) && CHSPEC_IS2G(pi->radio_chanspec)) {
			MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_xtbufstrg, 0x3);
			MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufstrg, 0x2);
		}

		/* Move nbclip by 2dBs to the right */
		MOD_RADIO_REG(pi, RFX, NBRSSI_CONFG, nbrssi_ib_Refladder, 7);

		/* 5g only: Changing RFPLL bandwidth to be 150MHz */
		if (CHSPEC_IS5G(pi->radio_chanspec))
			wlc_2069_rfpll_150khz(pi);

		if ((RADIO2069_MINORREV(pi->pubpi->radiorev) == 7) &&
		   (BFCTL(pi->u.pi_acphy) == 3)) {
			if ((BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 3 ||
			     BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 5) &&
			    CHSPEC_IS2G(pi->radio_chanspec)) {
				/* 43602 MCH2: offtune to win back linear output power */
				MOD_RADIO_REG(pi, RFX, PAD2G_TUNE, pad2g_tune, 0x1);

				/* increase gain */
				MOD_RADIO_REG(pi, RFX, PGA2G_CFG1, pga2g_gainboost, 0x2);
				phy_utils_write_radioreg(pi, RFX_2069_PAD2G_INCAP, 0x7e7e);
				MOD_RADIO_REG(pi, RFX, PAD2G_IDAC, pad2g_idac_main, 0x38);
				MOD_RADIO_REG(pi, RFX, PGA2G_INCAP, pad2g_idac_aux, 0x38);
				MOD_RADIO_REG(pi, RFX, PAD2G_IDAC, pad2g_idac_cascode, 0xe);
			} else if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 0 &&
			           CHSPEC_IS5G(pi->radio_chanspec)) {
				/* 43602 MCH5: increase gain to win back linear output power */
				MOD_RADIO_REGC(pi, TXMIX5G_CFG1, 2, gainboost, 0x4);
				MOD_RADIO_REGC(pi, PGA5G_CFG1, 2, gainboost, 0x4);
				MOD_RADIO_REG(pi, RFX, PAD5G_IDAC, idac_main, 0x3d);
				MOD_RADIO_REG(pi, RFX, PAD5G_TUNE, idac_aux, 0x3d);
			}
		}
	}

	if (RADIO2069REV(pi->pubpi->radiorev) >= 4) {
		/* Make clamping stronger */
		phy_utils_write_radioreg(pi, RFX_2069_ADC_CFG5, 0x83e0);
	}

	if ((RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) &&
	    (!(PHY_IPA(pi)))) {
	    MOD_RADIO_REG(pi, RFP, PLL_CP4, rfpll_cp_ioff, 0xe0);
	}

	/* increasing pabias to get good evm with pagain3 */
	if ((RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) &&
	    !(ACRADIO_2069_EPA_IS(pi->pubpi->radiorev))) {
		phy_utils_write_radioreg(pi, RF0_2069_PA5G_IDAC2, 0x8484);

		if (PHY_IPA(pi)) {
			phy_utils_write_radioreg(pi, RF0_2069_LOGEN5G_IDAC1, 0x3F37);
			phy_utils_write_radioreg(pi, RF0_2069_PGA5G_IDAC, 0x3838);
			MOD_RADIO_REG(pi, RFP, GE16_OVR2, ovr_bg_pulse, 1);
			MOD_RADIO_REG(pi, RFP, GE16_BG_CFG1, bg_pulse, 1);

			FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
				MOD_RADIO_REGC(pi, PAD5G_IDAC, core, idac_main, 0x20);
				MOD_RADIO_REGC(pi, PAD5G_TUNE, core, idac_aux, 0x20);

				MOD_RADIO_REGC(pi, PA5G_INCAP, core,
					pa5g_idac_incap_compen_main, 0x8);
				MOD_RADIO_REGC(pi, PA5G_INCAP, core,
					pa5g_idac_incap_compen_aux, 0x8);
				MOD_RADIO_REGC(pi, PA2G_INCAP, core,
					pa2g_ptat_slope_incap_compen_main, 0x0);
				MOD_RADIO_REGC(pi, PA2G_INCAP, core,
					pa2g_ptat_slope_incap_compen_aux, 0x0);

				if (pi->sh->chippkg == BCM4335_FCBGA_PKG_ID) {
					MOD_RADIO_REGC(pi, PA2G_CFG2, core,
						pa2g_bias_filter_main, 0x1);
					MOD_RADIO_REGC(pi, PA2G_CFG2, core,
						pa2g_bias_filter_aux, 0x1);
				} else {
					MOD_RADIO_REGC(pi, PA2G_CFG2, core,
						pa2g_bias_filter_main, 0x3);
					MOD_RADIO_REGC(pi, PA2G_CFG2, core,
						pa2g_bias_filter_aux, 0x3);
				}

				MOD_RADIO_REGC(pi, PAD5G_INCAP, core, idac_incap_compen_main, 0xc);
				MOD_RADIO_REGC(pi, PAD5G_INCAP, core, idac_incap_compen_aux, 0xc);
				MOD_RADIO_REGC(pi, PGA5G_INCAP, core, idac_incap_compen, 0x8);
				MOD_RADIO_REGC(pi, PA5G_CFG2, core, pa5g_bias_cas, 0x58);
				MOD_RADIO_REGC(pi, PA5G_IDAC2, core, pa5g_biasa_main, 0x84);
				MOD_RADIO_REGC(pi, PA5G_IDAC2, core, pa5g_biasa_aux, 0x84);
				MOD_RADIO_REGC(pi, GE16_OVR21, core, ovr_mix5g_gainboost, 0x1);
				MOD_RADIO_REGC(pi, TXMIX5G_CFG1, core, gainboost, 0x0);
				MOD_RADIO_REGC(pi, TXGM_CFG1, core, gc_res, 0x0);
				MOD_RADIO_REGC(pi, PA2G_CFG3, core, pa2g_ptat_slope_main, 0x7);
				MOD_RADIO_REGC(pi, PAD2G_SLOPE, core, pad2g_ptat_slope_main, 0x7);
				MOD_RADIO_REGC(pi, PGA2G_CFG2, core, pga2g_ptat_slope_main, 0x7);
				MOD_RADIO_REGC(pi, PGA2G_IDAC, core, pga2g_idac_main, 0x15);
				MOD_RADIO_REGC(pi, PAD2G_TUNE, core, pad2g_idac_tuning_bias, 0xc);
				MOD_RADIO_REGC(pi, TXMIX2G_CFG1, core, lodc, 0x3);
			}
		}
	}
	if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) {
		wlc_phy_2069_4335_set_ovrds(pi);
	} else if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) {
		wlc_phy_2069_4350_set_ovrds(pi);
	}

	/* 4335C0: Current optimization */
	acphy_set_lpmode(pi, ACPHY_LP_RADIO_LVL_OPT);

	/* Do a VCO cal after writing the tuning table regs */
	wlc_phy_radio2069_vcocal(pi);
}

/* Initialize chip regs(RW) that get reset with phy_reset */
static void
wlc_phy_set_reg_on_reset_acphy(phy_info_t *pi)
{
	uint8 core;
	uint16 rxbias, txbias;

	/* IQ Swap (revert swap happening in the radio) */
	if (!(RADIOID_IS(pi->pubpi->radioid, BCM20691_ID))) {
		phy_utils_or_phyreg(pi, ACPHY_RxFeCtrl1(pi->pubpi->phy_rev), 7 <<
			ACPHY_RxFeCtrl1_swap_iq0_SHIFT(pi->pubpi->phy_rev));
	}

	/* kimmer - add change from 0x667 to x668 very slight improvement */
	if (CHSPEC_IS2G(pi->radio_chanspec))
	     WRITE_PHYREG(pi, DsssStep, 0x668);

	/* Avoid underflow trigger for loopback Farrow */
	MOD_PHYREG(pi, RxFeCtrl1, en_txrx_sdfeFifoReset, 1);

	if (ACMAJORREV_1(pi->pubpi->phy_rev) && (ACMINORREV_0(pi) || ACMINORREV_1(pi))) {
		MOD_PHYREG(pi, RxFeCtrl1, rxfe_bilge_cnt, 0);
	} else {
		MOD_PHYREG(pi, RxFeCtrl1, rxfe_bilge_cnt, 4);
	}

	MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 1);
	MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 0);

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		if (ACMINORREV_0(pi) || ACMINORREV_1(pi)) {
			uint16 phymode = phy_get_phymode(pi);
			if (phymode == PHYMODE_MIMO) {
				WRITE_PHYREG(pi, SpareReg, 0xDF);
			}
		}

		if (ACMINORREV_1(pi)) {
			uint16 spare_reg = READ_PHYREG(pi, SpareReg);
			WRITE_PHYREG(pi, SpareReg, (spare_reg | 0x0400));
		}

		if (ACMINORREV_2(pi)) {
			uint16 spare_reg = READ_PHYREG(pi, SpareReg);

			if (phy_get_phymode(pi) == PHYMODE_MIMO) {
				/* The targeted use case is mimo mode coremask 1 case.
				 * Below settings will turn off some of the blocks for core 1
				 * and thus resulting in current savings
				 */
				if (pi->sh->phyrxchain == 1) {
					/* bit #12: Limit hrp access to core0 alone. Should be
					   made 1 before m aking 1 bits 8,9,13 and should
					   be made 0 only after bits 8,9,13 are made 0.
					   Recommended value: 0x1
					 */
					WRITE_PHYREG(pi, SpareReg, (spare_reg & ~(1 << 12)));
					spare_reg = READ_PHYREG(pi, SpareReg);

					/* bit #8: Use core1 clk for second chain like
					   rsdb except div4 clk
					   Recommended value: 0x1
					 */
					spare_reg &= ~(1 << 8);
					/* bit #9: Turn off core1 divider in phy1rx1 */
					/* Recommended value: 0x1 */
					spare_reg &= ~(1 << 9);
					/* bit #13: Use core1 clk for second chain for div4 clk */
					/* Recommended value: 0x1 */
					spare_reg &= ~(1 << 13);
				}
				/* bit #10: Turn off core1 divider in RX2 */
				/* Recommended value: 0x1 */
				spare_reg &= ~(1 << 10);
			}

			/* bit #6: Duration control of Rx2tx reset to some designs. Enable always */
			spare_reg |= (1 << 6);

			/* bit #11: Turn off RX2 during TX */
			spare_reg |= (1 << 11);

			WRITE_PHYREG(pi, SpareReg, spare_reg);
		}
	}

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, overideDigiGain1, cckdigigainEnCntValue, 0x6E);
	}

	if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
		/* Write 0x0 to RfseqMode to turn off both CoreActv_override */
		WRITE_PHYREG(pi, RfseqMode, 0);
	}


	/* Enable 6-bit Carrier Sense Match Filter Mode for 4335C0 and 43602A0 */
	if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
	    (ACMAJORREV_2(pi->pubpi->phy_rev) && !ACMINORREV_0(pi)) ||
	    ACMAJORREV_3(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, CRSMiscellaneousParam, crsMfMode, 1);
	}

	/* Turn on TxCRS extension.
	 * (Need to eventually make the 1.0 be native TxCRSOff (1.0us))
	 */
	WRITE_PHYREG(pi, dot11acphycrsTxExtension, 200);

	/* Currently PA turns on 1us before first DAC sample. Decrease that gap to 0.5us */
	if ((ACMAJORREV_0(pi->pubpi->phy_rev)) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
	        WRITE_PHYREG(pi, TxRealFrameDelay, 146);
	}

	/* This number combined with MAC RIFS results in 2.0us RIFS air time */
	WRITE_PHYREG(pi, TxRifsFrameDelay, 48);

	si_core_cflags(pi->sh->sih, SICF_MPCLKE, SICF_MPCLKE);
	if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
		wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_RESET2RX);
	}
	/* allow TSSI loopback path to turn off */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) || (ACMAJORREV_2(pi->pubpi->phy_rev) && PHY_IPA(pi))) {
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			if (((CHSPEC_IS20(pi->radio_chanspec)) &&
			  (pi->u.pi_acphy->srom_tssisleep_en & 0x1)) ||
			  ((CHSPEC_IS40(pi->radio_chanspec)) &&
			  (pi->u.pi_acphy->srom_tssisleep_en & 0x2))) {
				MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 1);
			} else {
				MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 0);
			}
		} else {
			if (((CHSPEC_IS20(pi->radio_chanspec)) &&
			  (pi->u.pi_acphy->srom_tssisleep_en & 0x4)) ||
			  ((CHSPEC_IS40(pi->radio_chanspec)) &&
			  (pi->u.pi_acphy->srom_tssisleep_en & 0x8)) ||
			  ((CHSPEC_IS80(pi->radio_chanspec)) &&
			  (pi->u.pi_acphy->srom_tssisleep_en & 0x10))) {
				MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 1);
			} else {
				MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 0);
			}
		}
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 1);
	} else {
		MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 0);
	}

	/* In event of high power spurs/interference that causes crs-glitches,
	   stay in WAIT_ENERGY_DROP for 1 clk20 instead of default 1 ms.
	   This way, we get back to CARRIER_SEARCH quickly and will less likely to miss
	   actual packets. PS: this is actually one settings for ACI
	*/
	/* WRITE_PHYREG(pi, ACPHY_energydroptimeoutLen, 0x2); */

	/* Upon Reception of a High Tone/Tx Spur, the default 40MHz MF settings causes ton of
	   glitches. Set the MF settings similar to 20MHz uniformly. Provides Robustness for
	   tones (on-chip, on-platform, accidential loft coming from other devices)
	*/
	if (ACREV_GE(pi->pubpi->phy_rev, 32)) {
		MOD_PHYREG(pi, crsControll0, mfLessAve, 0);
		MOD_PHYREG(pi, crsControlu0, mfLessAve, 0);
		MOD_PHYREG(pi, crsControllSub10, mfLessAve, 0);
		MOD_PHYREG(pi, crsControluSub10, mfLessAve, 0);

		MOD_PHYREG(pi, crsControll1, mfLessAve, 0);
		MOD_PHYREG(pi, crsControlu1, mfLessAve, 0);
		MOD_PHYREG(pi, crsControllSub11, mfLessAve, 0);
		MOD_PHYREG(pi, crsControluSub11, mfLessAve, 0);

		MOD_PHYREG(pi, crsControll2, mfLessAve, 0);
		MOD_PHYREG(pi, crsControlu2, mfLessAve, 0);
		MOD_PHYREG(pi, crsControllSub12, mfLessAve, 0);
		MOD_PHYREG(pi, crsControluSub12, mfLessAve, 0);

		MOD_PHYREG(pi, crsControll3, mfLessAve, 0);
		MOD_PHYREG(pi, crsControlu3, mfLessAve, 0);
		MOD_PHYREG(pi, crsControllSub13, mfLessAve, 0);
		MOD_PHYREG(pi, crsControluSub13, mfLessAve, 0);
	} else {
		MOD_PHYREG(pi, crsControll, mfLessAve, 0);
		MOD_PHYREG(pi, crsControlu, mfLessAve, 0);
		MOD_PHYREG(pi, crsControllSub1, mfLessAve, 0);
		MOD_PHYREG(pi, crsControluSub1, mfLessAve, 0);
	}
	if (!(ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev))) {
		if (ACREV_GE(pi->pubpi->phy_rev, 32)) {
			MOD_PHYREG(pi, crsThreshold2l0, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2u0, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2lSub10, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2uSub10, peakThresh, 85);

			MOD_PHYREG(pi, crsThreshold2l1, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2u1, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2lSub11, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2uSub11, peakThresh, 85);

			MOD_PHYREG(pi, crsThreshold2l2, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2u2, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2lSub12, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2uSub12, peakThresh, 85);

			MOD_PHYREG(pi, crsThreshold2l3, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2u3, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2lSub13, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2uSub13, peakThresh, 85);
		} else {
			MOD_PHYREG(pi, crsThreshold2l, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2u, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2lSub1, peakThresh, 85);
			MOD_PHYREG(pi, crsThreshold2uSub1, peakThresh, 85);
		}
	} else {
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			WRITE_PHYREG(pi, crsThreshold2u, 0x2055);
			WRITE_PHYREG(pi, crsThreshold2l, 0x2055);
		} else {
			WRITE_PHYREG(pi, crsThreshold2u, 0x204d);
			WRITE_PHYREG(pi, crsThreshold2l, 0x204d);
		}
		WRITE_PHYREG(pi, crsThreshold2lSub1, 0x204d);
		WRITE_PHYREG(pi, crsThreshold2uSub1, 0x204d);
	}

	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {

		MOD_PHYREG(pi, crsThreshold2l, peakThresh, 77);
		MOD_PHYREG(pi, crsThreshold2u, peakThresh, 77);
		MOD_PHYREG(pi, crsThreshold2lSub1, peakThresh, 77);
		MOD_PHYREG(pi, crsThreshold2uSub1, peakThresh, 77);

		MOD_PHYREG(pi, crsacidetectThreshl, acidetectThresh, 0x80);
		MOD_PHYREG(pi, crsacidetectThreshlSub1, acidetectThresh, 0x80);
		MOD_PHYREG(pi, crsacidetectThreshu, acidetectThresh, 0x80);
		MOD_PHYREG(pi, crsacidetectThreshuSub1, acidetectThresh, 0x80);
		WRITE_PHYREG(pi, initcarrierDetLen,  0x40);
		WRITE_PHYREG(pi, clip1carrierDetLen, 0x5c);

		if ((CHSPEC_IS2G(pi->radio_chanspec) && BF3_AGC_CFG_2G(pi->u.pi_acphy)) ||
			(CHSPEC_IS5G(pi->radio_chanspec) && BF3_AGC_CFG_5G(pi->u.pi_acphy))) {
			WRITE_PHYREG(pi, clip2carrierDetLen, 0x3a);
			WRITE_PHYREG(pi, defer_setClip1_CtrLen, 20);
		} else {
		        WRITE_PHYREG(pi, clip2carrierDetLen, 0x48);
			WRITE_PHYREG(pi, defer_setClip1_CtrLen, 24);
		}
		MOD_PHYREG(pi, clip_detect_normpwr_var_mux, use_norm_var_for_clip_detect, 0);
		MOD_PHYREG(pi, norm_var_hyst_th_pt8us, cck_gain_pt8us_en, 1);
		MOD_PHYREG(pi, CRSMiscellaneousParam, mf_crs_initgain_only, 1);
		/* disable bphyacidetEn as it is causing random rxper humps */
		MOD_PHYREG(pi, RxControl, bphyacidetEn, 0);

		WRITE_PHYREG(pi, RfseqCoreActv2059, 0x7717);

		if (!ACMAJORREV_3(pi->pubpi->phy_rev) &&
			(ACMINORREV_0(pi) || ACMINORREV_1(pi))) {
			WRITE_PHYREG(pi, HTSigTones, 0x9ee1);
			MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 0);
			MOD_PHYREG(pi, bOverAGParams, bOverAGlog2RhoSqrth, 0);
			MOD_PHYREG(pi, CRSMiscellaneousParam, b_over_ag_falsedet_en, 1);
		} else {
			WRITE_PHYREG(pi, HTSigTones, 0x9ee9);
			MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en,
			           (ACMAJORREV_3(pi->pubpi->phy_rev)) ? 0 : 0);
			MOD_PHYREG(pi, dot11acConfig, bphyPreDetTmOutEn, 0);
			/* digigain is not proper for low power bphy signals */
			/* causes kink near sensitivity region of 11mbps */
			/* fix is to increase cckshiftbitsRefVar by 1.5dB */
			/* WRITE_PHYREG(pi, cckshiftbitsRefVar, 46422); */
		}
		MOD_PHYREG(pi, FSTRCtrl, fineStrSgiVldCntVal,  0xb);
		MOD_PHYREG(pi, FSTRCtrl, fineStrVldCntVal, 0xa);

		MOD_PHYREG(pi, musigb2, mu_sigbmcs9, 0x7);
		MOD_PHYREG(pi, musigb2, mu_sigbmcs8, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs7, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs6, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs5, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs4, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs3, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs2, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs1, 0x3);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs0, 0x2);

	}

	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		wlc_phy_set_lowpwr_phy_reg_rev3(pi);

		if (ACMAJORREV_5(pi->pubpi->phy_rev)) {
			MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 0);
		} else {
			/* Enable BPHY pre-detect */
			MOD_PHYREG(pi, RxControl, preDetOnlyinCS, 1);
			MOD_PHYREG(pi, dot11acConfig, bphyPreDetTmOutEn, 1);
			MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 1);
			MOD_PHYREG(pi, bphyPreDetectThreshold0, ac_det_1us_min_pwr_0, 350);
			WRITE_PHYREG(pi, cckshiftbitsRefVar, 46422);
		}

		MOD_PHYREG(pi, musigb1, mu_sigbmcs6, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs5, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs4, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs3, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs2, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs1, 0x3);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs0, 0x2);
	}

	WRITE_PHYREG(pi, RfseqMode, 0);

	/* Disable Viterbi cache-hit low power featre for 4360
	 * since it is hard to meet 320 MHz timing
	 */
	MOD_PHYREG(pi, ViterbiControl0, CacheHitEn, ACMAJORREV_0(pi->pubpi->phy_rev) ? 0 : 1);

	/* Reset pktproc state and force RESET2RX sequence */
	wlc_phy_resetcca_acphy(pi);

	/* Try to fix the Tx2RX turnaround issue */
	if (0) {
		MOD_PHYREG(pi, RxFeStatus, sdfeFifoResetCntVal, 0xF);
		MOD_PHYREG(pi, RxFeCtrl1, resetsdFeInNonActvSt, 0x1);
	}

	/* Make TSSI to select Q-rail */
	if (ACREV_IS(pi->pubpi->phy_rev, 4) && CHSPEC_IS2G(pi->radio_chanspec))
		MOD_PHYREG(pi, TSSIMode, tssiADCSel, 0);
	else
		MOD_PHYREG(pi, TSSIMode, tssiADCSel, 1);

	/* Increase this by 10 ticks helps in getting rid of humps at high SNR, single core runs */
	if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
		WRITE_PHYREG(pi, defer_setClip2_CtrLen, 16);
	}

	MOD_PHYREG(pi, HTSigTones, support_gf, 0);

	/* JIRA-CRDOT11ACPHY-273: SIG errror check For number of VHT symbols calculated */
	MOD_PHYREG(pi, partialAIDCountDown, check_vht_siga_length, 1);

	MOD_PHYREG(pi, DmdCtrlConfig, check_vhtsiga_rsvd_bit, 0);

	FOREACH_CORE(pi, core) {
		MOD_PHYREGCE(pi, forceFront, core, freqCor, 1);
		MOD_PHYREGCE(pi, forceFront, core, freqEst, 1);
	}

	WRITE_PHYREG(pi, pktgainSettleLen, 48);

	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
		WRITE_PHYREG(pi, CoreConfig, 0x29);
		WRITE_PHYREG(pi, RfseqCoreActv2059, 0x1111);

		if (ACMAJORREV_1(pi->pubpi->phy_rev))
			wlc_phy_set_lowpwr_phy_reg(pi);
	}

	if (!ACMAJORREV_0(pi->pubpi->phy_rev)) {
		/* 4335:tkip macdelay & mac holdoff */
		WRITE_PHYREG(pi, TxMacIfHoldOff, TXMAC_IFHOLDOFF_DEFAULT);
		WRITE_PHYREG(pi, TxMacDelay, TXMAC_MACDELAY_DEFAULT);
	}

	/* tiny radio specific processing */
	if (TINY_RADIO(pi)) {
		if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID))
			wlc_phy_set_reg_on_reset_acphy_20691(pi);
		else if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID))
			wlc_phy_set_reg_on_reset_acphy_20693(pi);
	}

	wlc_phy_mlua_adjust_acphy(pi, pi->bt_active);
#ifndef WLC_DISABLE_ACI
	/* Setup HW_ACI block */
	if (!ACPHY_ENABLE_FCBS_HWACI(pi)) {
		if (((pi->sh->interference_mode_2G & ACPHY_ACI_HWACI_PKTGAINLMT) != 0) ||
		    ((pi->sh->interference_mode_5G & ACPHY_ACI_HWACI_PKTGAINLMT) != 0))
			wlc_phy_hwaci_setup_acphy(pi, FALSE, TRUE);
		else
			wlc_phy_hwaci_setup_acphy(pi, FALSE, FALSE);
	}
#endif /* !WLC_DISABLE_ACI */

	/* 4335C0: Current optimization */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
		WRITE_PHYREG(pi, FFTSoftReset, 0x2);
		WRITE_PHYREG(pi, fineclockgatecontrol, 0x0);
		WRITE_PHYREG(pi, RxFeTesMmuxCtrl, 0x60);
		MOD_PHYREG(pi, forceFront0, freqEst, 0);
		MOD_PHYREG(pi, forceFront0, freqCor, 0);
		MOD_PHYREG(pi, fineRxclockgatecontrol, forcedigigaingatedClksOn, 0);
	}

	/* 43602: C-Model Parameters setting */
	if (ACMAJORREV_5(pi->pubpi->phy_rev)) {

		/* Turn ON 11n 256 QAM in 2.4G */
		WRITE_PHYREG(pi, miscSigCtrl, 0x203);
		WRITE_PHYREG(pi, HTAGCWaitCounters, 0x1028);

		/* WRITE_PHYREG(pi, bfeConfigReg1, 0x8) */

		WRITE_PHYREG(pi, crsThreshold2lSub1, 0x204d);
		WRITE_PHYREG(pi, crsThreshold2uSub1, 0x204d);

		/* Fine timing optimization for linear filter */
		WRITE_PHYREG(pi, FSTRCtrl, 0x7aa);
	}

	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* Low_power settings */
		WRITE_PHYREG(pi, RxFeTesMmuxCtrl, 0x60);
		/* Commenting out this low-power feature. Seen performance hit because of it.  */
		/* FOREACH_CORE(pi, core) { */
		/* 	MOD_PHYREGCE(pi, forceFront, core, freqCor, 0); */
		/* 	MOD_PHYREGCE(pi, forceFront, core, freqEst, 0); */
		/* } */
	}

	/* enable fix for bphy loft calibration issue CRDOT11ACPHY-378 */
	if (ACREV_GE(pi->pubpi->phy_rev, 6))
		MOD_PHYREG(pi, bphyTest, bphyTxfiltTrunc, 0);

	/* for: http://jira.broadcom.com/browse/SWWFA-10  */
	WRITE_PHYREG(pi, drop20sCtrl1, 0xc07f);

	/* phyrcs20S drop threshold -110 dBm */
	WRITE_PHYREG(pi, drop20sCtrl2, 0x64);

	/* phyrcs40S drop threshold -110 dBm */
	WRITE_PHYREG(pi, drop20sCtrl3, 0x64);

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {

		uint16 phymode = phy_get_phymode(pi);

		WRITE_PHYREG(pi, miscSigCtrl, 0x003);
		MOD_PHYREG(pi, CRSMiscellaneousParam, crsInpHold, 1);
		MOD_PHYREG(pi, crsThreshold2l, peakThresh, 77);
		MOD_PHYREG(pi, crsThreshold2u, peakThresh, 77);
		MOD_PHYREG(pi, crsThreshold2lSub1, peakThresh, 77);
		MOD_PHYREG(pi, crsThreshold2uSub1, peakThresh, 77);

		MOD_PHYREG(pi, crshighlowpowThresholdl, low2highpowThresh, 69);
		MOD_PHYREG(pi, crshighlowpowThresholdu, low2highpowThresh, 69);
		MOD_PHYREG(pi, crshighlowpowThresholdlSub1, low2highpowThresh, 69);
		MOD_PHYREG(pi, crshighlowpowThresholduSub1, low2highpowThresh, 69);

		MOD_PHYREG(pi, dot11acConfig, bphyPreDetTmOutEn, 0);
		MOD_PHYREG(pi, bOverAGParams, bOverAGlog2RhoSqrth, 0);
		MOD_PHYREG(pi, CRSMiscellaneousParam, b_over_ag_falsedet_en, 1);
		WRITE_PHYREG(pi, cckshiftbitsRefVar, 46422);
		MOD_PHYREG(pi, RxStatPwrOffset0, use_gainVar_for_rssi0, 1);
		if (ACMINORREV_0(pi) || ACMINORREV_1(pi))
			WRITE_PHYREG(pi, HTAGCWaitCounters, 0x2220);
		else
			WRITE_PHYREG(pi, HTAGCWaitCounters, 0x1020);
		WRITE_PHYREG(pi, FSTRCtrl, 0x7aa);
		MOD_PHYREG(pi, FFTSoftReset, lbsdadc_clken_ovr, 0);
		MOD_PHYREG(pi, RxSdFeConfig5, rx_farow_scale_value, 7);
		MOD_PHYREG(pi, RxSdFeConfig5, tiny_bphy20_ADC10_sel, 0);

		txbias = 0x2b;
		rxbias = 0x28;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0xe8, 16,
		                          &txbias);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0xe7, 16,
		                          &rxbias);

		MOD_PHYREG(pi, bphyTest, dccomp, 0);

		MOD_PHYREG(pi, RfseqCoreActv2059, DisTx, pi->pubpi->phy_coremask);
		MOD_PHYREG(pi, RfseqCoreActv2059, DisRx, pi->pubpi->phy_coremask);
		MOD_PHYREG(pi, RfseqCoreActv2059, EnTx, pi->pubpi->phy_coremask);
		MOD_PHYREG(pi, RfseqCoreActv2059, EnRx, pi->pubpi->phy_coremask);
		MOD_PHYREG(pi, CoreConfig, CoreMask, pi->pubpi->phy_coremask);
		wlc_phy_set_phyctl_in_phymode_acphy(pi);
		MOD_PHYREG(pi, CoreConfig, NumRxCores, pi->pubpi->phy_corenum);
		WRITE_PHYREG(pi, HTSigTones, 0x9ee9);
		MOD_PHYREG(pi, HTSigTones, support_max_nss, pi->pubpi->phy_corenum);
		MOD_PHYREG(pi, bphyFiltBypass, bphy_tap_20in20path_from_DVGA_en, 1);
		WRITE_PHYREG(pi, femctrl_override_control_reg, 0x0);

		FOREACH_CORE(pi, core) {
			MOD_RADIO_REG_20693(pi, RX_BB_2G_OVR_EAST, core,
			                  ovr_tia_offset_comp_pwrup, 1);
			MOD_RADIO_REG_20693(pi, TIA_CFG15, core, tia_offset_comp_pwrup, 1);
		}
		MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 1);

		if (phymode == PHYMODE_MIMO) {
			WRITE_PHYREG(pi, BfeConfigReg1, 0x8);
			MOD_PHYREG(pi, dot11acConfig, bphyPreDetTmOutEn, 1);
			MOD_PHYREG(pi, CRSMiscellaneousParam, crsMfMode, 1);
			WRITE_PHYREG(pi, fineclockgatecontrol, 0x4000);
			/* disableML if QT and MIMO mode */
			if (ISSIM_ENAB(pi->sh->sih)) {
				MOD_PHYREG(pi, RxControl, MLenable, 0);
			}
		} else if (phymode == PHYMODE_80P80) {
			MOD_PHYREG(pi, crshighlowpowThresholdl1, low2highpowThresh, 69);
			MOD_PHYREG(pi, crshighlowpowThresholdu1, low2highpowThresh, 69);
			MOD_PHYREG(pi, crshighlowpowThresholdlSub11, low2highpowThresh, 69);
			MOD_PHYREG(pi, crshighlowpowThresholduSub11, low2highpowThresh, 69);
			MOD_PHYREG(pi, crsThreshold2u1, peakThresh, 77);

			MOD_PHYREG(pi, crsControll1, mfLessAve, 0);
			MOD_PHYREG(pi, crsControlu1, mfLessAve, 0);
			MOD_PHYREG(pi, crsControllSub11, mfLessAve, 0);
			MOD_PHYREG(pi, crsControluSub11, mfLessAve, 0);

			WRITE_PHYREG(pi, fineclockgatecontrol, 0x4000);
			MOD_PHYREG(pi, HTSigTones, support_max_nss, 0x1);
		} else {
			WRITE_PHYREG(pi, fineclockgatecontrol, 0x0);
		}

		MOD_PHYREG(pi, RxFeCtrl1, swap_iq1, 1);
		MOD_PHYREG(pi, RxFeCtrl1, swap_iq2, 0);

		/* RfseqMode mixer_1st_dis is set to 1, so mixer 1st is not enabled */
		MOD_PHYREG(pi, RfseqMode, mixer_first_mask_dis, 1);
		FOREACH_CORE(pi, core) {
			MOD_PHYREGCE(pi, forceFront, core, freqCor, 0);
			MOD_PHYREGCE(pi, forceFront, core, freqEst, 0);
		}
		/* Doppler related fix in channel update block */
		MOD_PHYREG(pi, ChanestCDDshift, dmd_chupd_use_mod_depend_mu, 1);
		WRITE_PHYREG(pi, chanupsym2, 0x050);
		WRITE_PHYREG(pi, mu_a_mod_ml_4, 0x4400);
		WRITE_PHYREG(pi, mu_a_mod_ml_5, 0x4444);
	}
}


/* Initialize chip tbls(reg-based) that get reset with phy_reset */
static void
wlc_phy_set_tbl_on_reset_acphy(phy_info_t *pi)
{
	uint8 stall_val;
	phy_info_acphy_t *pi_ac;
	uint16 adc_war_val = 0x20, pablowup_war_val = 120;
	uint8 core;
	uint16 gmult20, gmult40, gmult80;
	uint16 rfseq_bundle_tssisleep48[3];
	uint16 rfseq_bundle_48[3];
	const void *data, *dly;
	/* uint16 AFEdiv_read_val = 0x0000; */

	bool ext_pa_ana_2g =  ((BOARDFLAGS2(GENERIC_PHY_INFO(pi)->boardflags2) &
		BFL2_SROM11_ANAPACTRL_2G) != 0);
	bool ext_pa_ana_5g =  ((BOARDFLAGS2(GENERIC_PHY_INFO(pi)->boardflags2) &
		BFL2_SROM11_ANAPACTRL_5G) != 0);

	/* DEBUG: TEST CODE FOR SS PTW70 DEBUG */
	uint32 war_val = 0x7ffffff;
	uint8 offset;

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);
	pi_ac = pi->u.pi_acphy;

	/* Load settings related to antswctrl if not on QT */
	if (!ISSIM_ENAB(pi->sh->sih)) {
		wlc_phy_set_regtbl_on_femctrl(pi);
	}

	/* Quickturn only init */
	if (ISSIM_ENAB(pi->sh->sih)) {
		uint8 ncore_idx;
		uint16 val;

		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		   val = 64;
		} else {
		   /* changing to TCL value */
		   val = 50;
		   if (CHSPEC_IS20(pi->radio_chanspec)) {
			  MOD_PHYREG(pi, DcFiltAddress, dcBypass, 1);
		   }

		}

		FOREACH_CORE(pi, ncore_idx) {
			wlc_phy_set_tx_bbmult_acphy(pi, &val, ncore_idx);
		}

		/* dummy call to satisfy compiler */
		wlc_phy_get_tx_bbmult_acphy(pi, &val, 0);

		/* on QT: force the init gain to allow noise_var not limiting 256QAM performance */
		ACPHYREG_BCAST(pi, Core0InitGainCodeA, 0x16a);
		ACPHYREG_BCAST(pi, Core0InitGainCodeB, 0x24);

		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
				0xf9 + core, 16, &qt_rfseq_val1[core]);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
				0xf6 + core, 16, &qt_rfseq_val2[core]);
		}
	}

	/* Update gmult, dacbuf after radio init */
	/* Tx Filters */
	wlc_phy_set_analog_tx_lpf(pi, 0x1ff, -1, -1, -1, pi_ac->rccal_gmult,
	                          pi_ac->rccal_gmult_rc, -1);
	wlc_phy_set_tx_afe_dacbuf_cap(pi, 0x1ff, pi_ac->rccal_dacbuf, -1, -1);

	/* Rx Filters */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* 4360 (tighten rx analog filters). Note than 80mhz filter cutoff
		   was speced at 39mhz (should have been 38.5)
		   C-model desired bw : {9, 18.5, 38.5}  @ 3dB cutoff
		   lab-desired (freq offset + 5%PVT): {9.5, 20, 41}
		   with gmult = 193 (in 2069_procs.tcl), we get {11, 23.9, 48.857}
		   Reduce bw by factor : {9.5/11, 20/23.9, 41/48.857} = {0.863, 0.837, 0.839}
		*/
		gmult20 = (pi_ac->rccal_gmult * 221) >> 8;     /* gmult * 0.863 */
		gmult40 = (pi_ac->rccal_gmult * 215) >> 8;     /* gmult * 0.839 (~ 0.837) */
		gmult80 = (pi_ac->rccal_gmult * 215) >> 8;     /* gmult * 0.839 */
	} else if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
		/* 4335C0 (tighten rx analog filter for 80mhz only).
		   This is needed to take away
		   hump which comes because of ACI causing false clip_hi
		*/
		gmult20 = pi_ac->rccal_gmult;
		gmult40 = pi_ac->rccal_gmult;
		if (!(PHY_ILNA(pi))) {
			gmult80 = (pi_ac->rccal_gmult * 225) >> 8;     /* gmult * 0.879 */
		} else {
			gmult80 = pi_ac->rccal_gmult;
		}
	} else {
		gmult20 = pi_ac->rccal_gmult;
		gmult40 = pi_ac->rccal_gmult;
		gmult80 = pi_ac->rccal_gmult;
	}
	wlc_phy_set_analog_rx_lpf(pi, 1, -1, -1, -1, gmult20, pi_ac->rccal_gmult_rc, -1);
	wlc_phy_set_analog_rx_lpf(pi, 2, -1, -1, -1, gmult40, pi_ac->rccal_gmult_rc, -1);
	wlc_phy_set_analog_rx_lpf(pi, 4, -1, -1, -1, gmult80, pi_ac->rccal_gmult_rc, -1);

	/* Reset2rx sequence */
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		data = rfseq_majrev4_reset2rx_cmd;
		dly = rfseq_majrev4_reset2rx_dly;
	} else if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		data = rfseq_reset2rx_cmd;
		dly = rfseq_majrev3_reset2rx_dly;
	} else {
		data = rfseq_reset2rx_cmd;
		dly = rfseq_reset2rx_dly;
	}

	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x20, 16, data);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x90, 16, dly);

	/* during updateGainL make sure the lpf/tia hpc corner is set properly to optimum setting */
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 2, 0x121, 16, rfseq_updl_lpf_hpc_ml);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 2, 0x131, 16, rfseq_updl_lpf_hpc_ml);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 2, 0x124, 16, rfseq_updl_tia_hpc_ml);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 2, 0x137, 16, rfseq_updl_tia_hpc_ml);

	/* tx2rx/rx2tx: Remove SELECT_RFPLL_AFE_CLKDIV/RESUME as we are not in boost mode */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) || (ACMAJORREV_2(pi->pubpi->phy_rev) && PHY_IPA(pi))) {
		if ((CHSPEC_IS2G(pi->radio_chanspec) &&
			(((CHSPEC_IS20(pi->radio_chanspec)) &&
			(pi->u.pi_acphy->srom_tssisleep_en & 0x1)) ||
			((CHSPEC_IS40(pi->radio_chanspec)) &&
			(pi->u.pi_acphy->srom_tssisleep_en & 0x2)))) ||
			(CHSPEC_IS5G(pi->radio_chanspec) &&
			(((CHSPEC_IS20(pi->radio_chanspec)) &&
			(pi->u.pi_acphy->srom_tssisleep_en & 0x4)) ||
			((CHSPEC_IS40(pi->radio_chanspec)) &&
			(pi->u.pi_acphy->srom_tssisleep_en & 0x8)) ||
			((CHSPEC_IS80(pi->radio_chanspec)) &&
			(pi->u.pi_acphy->srom_tssisleep_en & 0x10))))) {
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
					16, rfseq_rx2tx_cmd_withtssisleep);
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x70,
					16, rfseq_rx2tx_dly_withtssisleep);
				MOD_PHYREG(pi, RfBiasControl, tssi_sleep_bg_pulse_val, 1);
				MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 1);
				rfseq_bundle_tssisleep48[0] = 0x0000;
				rfseq_bundle_tssisleep48[1] = 0x20;
				rfseq_bundle_tssisleep48[2] = 0x0;
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 0, 48,
					rfseq_bundle_tssisleep48);
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
				16, rfseq_rx2tx_cmd);
		}
	} else if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00, 16,
		                          tiny_rfseq_rx2tx_cmd);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 112, 16,
		                          tiny_rfseq_rx2tx_dly);
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00, 16,
		                          tiny_rfseq_rx2tx_tssi_sleep_cmd);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 112, 16,
		                          tiny_rfseq_rx2tx_tssi_sleep_dly);
		MOD_PHYREG(pi, AfePuCtrl, tssiSleepEn, 1);
	} else {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
			16, rfseq_rx2tx_cmd);
	}

	if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x10, 16,
		                          tiny_rfseq_tx2rx_cmd);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 128, 16,
		                          tiny_rfseq_tx2rx_dly);
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x10, 16,
		                          rfseq_majrev4_tx2rx_cmd);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 128, 16,
		                          rfseq_majrev4_tx2rx_dly);
	} else {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x10, 16, rfseq_tx2rx_cmd);
	}

	/* This was to keep the adc-clock buffer powered up even if adc is powered down
	   for non-tiny radio. But for tiny radio this is not required.
	*/
	if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x3c6, 16, &adc_war_val);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x3c7, 16, &adc_war_val);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x3d6, 16, &adc_war_val);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x3d7, 16, &adc_war_val);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x3e6, 16, &adc_war_val);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x3e7, 16, &adc_war_val);
	}

	/* do this during fem table load for 43602a0 */
	if (((CHSPEC_IS2G(pi->radio_chanspec) && ext_pa_ana_2g) ||
	    (CHSPEC_IS5G(pi->radio_chanspec) && ext_pa_ana_5g)) &&
	    !(ACMAJORREV_5(pi->pubpi->phy_rev) && ACMINORREV_0(pi))) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x80, 16, &pablowup_war_val);
	}

	/* 4360 and 43602 */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* load the txv for spatial expansion */
		acphy_load_txv_for_spexp(pi);
	}

	if ((RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) &&
	    RADIO2069_MAJORREV(pi->pubpi->radiorev) > 0) {
		/* 11n_20 */
		wlc_phy_set_analog_tx_lpf(pi, 0x2, -1, 5, 5, -1, -1, -1);
		/* 11ag_11ac_20 */
		wlc_phy_set_analog_tx_lpf(pi, 0x4, -1, 5, 5, -1, -1, -1);
		/* 11n_40 */
		wlc_phy_set_analog_tx_lpf(pi, 0x10, -1, 5, 5, -1, -1, -1);
		/* 11ag_11ac_40 */
		wlc_phy_set_analog_tx_lpf(pi, 0x20, -1, 5, 5, -1, -1, -1);
		/* 11n_11ag_11ac_80 */
		wlc_phy_set_analog_tx_lpf(pi, 0x80, -1, 6, 6, -1, -1, -1);
	}

	/* tiny radio specific processing */
	if (TINY_RADIO(pi)) {
		uint16 regval;
		const uint32 NvAdjTbl[64] = { 0x000000, 0x400844, 0x300633, 0x200422,
			0x100211, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
			0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
			0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000100,
			0x000200, 0x000311, 0x000422, 0x100533, 0x200644, 0x300700,
			0x400800, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
			0x000000, 0x000000, 0x400800, 0x300700, 0x200644, 0x100533,
			0x000422, 0x000311, 0x000200, 0x000100, 0x000000, 0x000000,
			0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
			0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
			0x000000, 0x000000, 0x100211, 0x200422, 0x300633, 0x400844};

		const uint32 phasetracktbl[22] = { 0x06af56cd, 0x059acc7b,
			0x04ce6652, 0x02b15819, 0x02b15819, 0x02b15819, 0x02b15819,
			0x02b15819, 0x02b15819, 0x02b15819, 0x02b15819, 0x06af56cd,
			0x059acc7b, 0x04ce6652, 0x02b15819, 0x02b15819, 0x02b15819,
			0x02b15819, 0x02b15819, 0x02b15819, 0x02b15819, 0x02b15819};

		/* Tiny NvAdjTbl */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_NVADJTBL, 64, 0, 32, NvAdjTbl);

		if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
			/* Tiny phasetrack */
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_PHASETRACKTBL_1X1, 22, 0, 32,
			phasetracktbl);
		}

		/* Channels Smoothing */
		if (!ACMINORREV_0(pi) || ACMAJORREV_4(pi->pubpi->phy_rev))
			wlc_phy_load_channel_smoothing_tiny(pi);

		/* program tx, rx bias reset to avoid clock stalls */
		regval = 0x2b;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0xe8, 16, &regval);
		regval = 0x28;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0xe7, 16, &regval);

		/* #Keep lpf_pu @ 0 for rx since lpf_pu controls tx lpf exclusively */
		regval = 0x82c0;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x14b, 16, &regval);

		/* Magic rfseqbundle writes to make TX->Rx turnaround work */
		/* set rfseq_bundle_tbl {0x4000 0x0000 } */
		/* acphy_write_table RfseqBundle $rfseq_bundle_tbl 4 */
		rfseq_bundle_48[0] = 0x4000;
		rfseq_bundle_48[1] = 0x0;
		rfseq_bundle_48[2] = 0x0;

		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 4, 48,
		                          rfseq_bundle_48);
		rfseq_bundle_48[0] = 0x0000;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 5, 48,
		                          rfseq_bundle_48);

		/* set rfseq_bundle_tbl {0x3000C 0x20000 0x30034 0x20000} */
		/* acphy_write_table RfseqBundle $rfseq_bundle_tbl 0 */
		rfseq_bundle_48[0] = 0x0000;
		rfseq_bundle_48[1] = 0x2;
		rfseq_bundle_48[2] = 0x0;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 0, 48,
		                          rfseq_bundle_48);
		rfseq_bundle_48[0] = 0x0034;
		rfseq_bundle_48[1] = 0x3;
		rfseq_bundle_48[2] = 0x0;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 1, 48,
		                          rfseq_bundle_48);
		rfseq_bundle_48[0] = 0x0000;
		rfseq_bundle_48[1] = 0x2;
		rfseq_bundle_48[2] = 0x0;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 2, 48,
		                          rfseq_bundle_48);
		rfseq_bundle_48[0] = 0x000c;
		rfseq_bundle_48[1] = 0x3;
		rfseq_bundle_48[2] = 0x0;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 3, 48,
		                          rfseq_bundle_48);
	}

	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
		uint8 txevmtbl[40] = {0x09, 0x0E, 0x11, 0x14, 0x17, 0x1A, 0x1D, 0x20, 0x09,
			0x0E, 0x11, 0x14, 0x17, 0x1A, 0x1D, 0x20, 0x22, 0x24, 0x09, 0x0E,
			0x11, 0x14, 0x17, 0x1A, 0x1D, 0x20, 0x22, 0x24, 0x09, 0x0E, 0x11,
			0x14, 0x17, 0x1A, 0x1D, 0x20, 0x22, 0x24, 0x0, 0x0};
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_TXEVMTBL, 40, 0, 8, txevmtbl);
	}

	/* 4335: Running phase track loop faster */
	/* Fix for ping issue caused by high phase imbalance */
	if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
		uint32 phasetracktbl_1x1[22] = { 0x6AF5700, 0x59ACC9A,
			0x4CE6666, 0x4422064, 0x4422064, 0x4422064,	0x4422064,
			0x4422064, 0x4422064, 0x4422064, 0x4422064, 0x6AF5700,
			0x59ACC9A, 0x4CE6666, 0x4422064, 0x4422064, 0x4422064,
			0x4422064, 0x4422064, 0x4422064, 0x4422064, 0x4422064};
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_PHASETRACKTBL_1X1, 22, 0, 32,
		                          phasetracktbl_1x1);
	}
	/* Increase phase track loop BW to improve PER floor, */
	/*   Phase noise  seems higher. Needs further investigation */
	if (ACMAJORREV_2(pi->pubpi->phy_rev)) {
		uint32 phasetracktbl[22] = { 0x6AF5700, 0x59ACC9A,
			0x4CE6666, 0x4422064, 0x4422064, 0x4422064,	0x4422064,
			0x4422064, 0x4422064, 0x4422064, 0x4422064, 0x6AF5700,
			0x59ACC9A, 0x4CE6666, 0x4422064, 0x4422064, 0x4422064,
			0x4422064, 0x4422064, 0x4422064, 0x4422064, 0x4422064};
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_PHASETRACKTBL, 22, 0, 32,
		                          phasetracktbl);
	}
	/* DEBUG: TEST CODE FOR SS PTW70 DEBUG */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) && BF3_PHASETRACK_MAX_ALPHABETA(pi_ac)) {
		for (offset = 0; offset < 22; offset++) {
			wlc_phy_table_write_acphy(pi, 0x1a, 1, offset, 32, &war_val);
		}
	}

	/* To save current, turn off AFEDiv for the unused core, */
	/* Below forces AFEDiv_pu_repeater2_disRX to be 0 when doing TX2RX || reset2RX */
	/* if (ACMAJORREV_2(pi->pubpi->phy_rev)) { */
	/* 	wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0xe9, 16, &AFEdiv_read_val); */
	/* 	AFEdiv_read_val = (AFEdiv_read_val & 0xfdff); */
	/* 	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0xe9, 16, &AFEdiv_read_val); */
	/* } */

	ACPHY_ENABLE_STALL(pi, stall_val);

}

static void
wlc_phy_set_regtbl_on_band_change_acphy(phy_info_t *pi)
{
	uint8 stall_val;
	uint16 bq1_gain_core1 = 0x49;
	uint8 pdet_range_id;
#ifndef WLC_DISABLE_ACI
	bool hwaci_on;
#endif /* !WLC_DISABLE_ACI */
	bool w2_on;
	txcal_coeffs_t txcal_cache[PHY_CORE_MAX];
#ifdef PHYCAL_CACHING
	ch_calcache_t *ctx;
	bool ctx_valid;
#endif /* PHYCAL_CACHING */
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	wlc_phy_cfg_energydrop_timeout(pi);

	if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			MOD_PHYREG(pi, fineRxclockgatecontrol, forcedigigaingatedClksOn, 1);
		} else {
			MOD_PHYREG(pi, fineRxclockgatecontrol, forcedigigaingatedClksOn, 0);
		}

		/* 4335C0: Current optimization */
		if (ACMINORREV_2(pi)) {
			MOD_PHYREG(pi, fineRxclockgatecontrol, forcedigigaingatedClksOn, 0);
		}
	}

	if (ACMAJORREV_5(pi->pubpi->phy_rev) || ACMAJORREV_2(pi->pubpi->phy_rev) ||
	    (ACMAJORREV_1(pi->pubpi->phy_rev) &&
	     !(ACMINORREV_0(pi) || ACMINORREV_1(pi)))) {
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			MOD_PHYREG(pi, bOverAGParams, bOverAGlog2RhoSqrth, 120);
			MOD_PHYREG(pi, CRSMiscellaneousParam, b_over_ag_falsedet_en, 1);
		} else {
			MOD_PHYREG(pi, bOverAGParams, bOverAGlog2RhoSqrth, 255);
			MOD_PHYREG(pi, CRSMiscellaneousParam, b_over_ag_falsedet_en, 0);
		}
	}

	if (ACMAJORREV_3(pi->pubpi->phy_rev) || ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, clip_detect_normpwr_var_mux, use_norm_var_for_clip_detect, 1);
	}

	/* Load tx gain table */
	wlc_phy_ac_gains_load(pi);

	if (ACREV_IS(pi->pubpi->phy_rev, 0)) {
		wlc_phy_tx_gm_gain_boost(pi);
	}

	pdet_range_id = pi->u.pi_acphy->srom_5g_pdrange_id;
	if (pdet_range_id == 9 || pdet_range_id == 16) {
		bq1_gain_core1 = (CHSPEC_IS5G(pi->radio_chanspec))? 0x49 : 0;
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, 0x18e, 16, &bq1_gain_core1);
	}

	if (!ACMAJORREV_0(pi->pubpi->phy_rev)) {
		/* When WLAN is in 5G, WLAN table should control the FEM lines */
		/* and BT should not have any access permissions */
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			/* disable BT Fem control table accesses */
			MOD_PHYREG(pi, FemCtrl, enBtSignalsToFEMLut, 0x0);
			if (!ACPHY_FEMCTRL_ACTIVE(pi)) {
				MOD_PHYREG(pi, FemCtrl, femCtrlMask,
				           pi_ac->sromi->femctrlmask_5g);
			} else {
				if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
					if (BFCTL(pi_ac) == 4) {
						if (BF3_FEMCTRL_SUB(pi_ac) == 1) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x23c);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 2) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x297);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 3) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x058);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 4) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x058);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 6) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0xe);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 7) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x2e);
						} else {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x3ff);
						}
					}
				} else if (ACMAJORREV_2(pi->pubpi->phy_rev) ||
				           ACMAJORREV_5(pi->pubpi->phy_rev)) {
					if (BFCTL(pi_ac) == 10) {
						if (BF3_FEMCTRL_SUB(pi_ac) == 0) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x317);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 1) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x347);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 2) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x303);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 3) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x307);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 4) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x309);
						} else if (BF3_FEMCTRL_SUB(pi_ac) == 5) {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x3c7);
						} else {
							MOD_PHYREG(pi, FemCtrl, femCtrlMask,
							           0x3ff);
						}
					} else {
						MOD_PHYREG(pi, FemCtrl, femCtrlMask, 0x3ff);
					}
				} else if (TINY_RADIO(pi)) {
					MOD_PHYREG(pi, FemCtrl, femCtrlMask, 0x3ff);
				}
			}
			if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
				phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_SCALE(0), 0x0);
				if (BF2_DAC_SPUR_IMPROVEMENT(pi_ac) == 1) {
					phy_utils_write_radioreg(pi, RFX_2069_ADC_CFG5, 0x83e3);
				}
			}
		} else { /* When WLAN is in 2G, BT controls should be allowed to go through */
			/* BT should also be able to control FEM Control Table */
			if ((!BCM43602_CHIP(pi->sh->chip)) ||
				BF_SROM11_BTCOEX(pi_ac)) {
				MOD_PHYREG(pi, FemCtrl, enBtSignalsToFEMLut, 0x1);
			}
			MOD_PHYREG(pi, FemCtrl, femCtrlMask, 0x3ff);

			if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				uint8 DLNA_BTFLAG;
				DLNA_BTFLAG = (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) &
					0x00400000) >> 22;
				if (DLNA_BTFLAG == 0) {
					MOD_PHYREG(pi, FemCtrl, enBtSignalsToFEMLut, 0x0);
					MOD_PHYREG(pi, FemCtrl, femCtrlMask,
						pi_ac->sromi->femctrlmask_2g);
				} else {
					if (BF3_RSDB_1x1_BOARD(pi_ac)) {
						MOD_PHYREG(pi, FemCtrl, enBtSignalsToFEMLut, 0x1);
					} else {
						if (phy_get_phymode(pi) == PHYMODE_MIMO) {
						/* writes to both cores */
						MOD_PHYREG(pi, FemCtrl, enBtSignalsToFEMLut, 0x0);
						wlapi_exclusive_reg_access_core0(
							pi->sh->physhim, 1);
						/* writes to only core0 */
						MOD_PHYREG(pi, FemCtrl, enBtSignalsToFEMLut, 0x1);
						wlapi_exclusive_reg_access_core0(
							pi->sh->physhim, 0);
						} else if (phy_get_phymode(pi) == PHYMODE_RSDB)  {
							if (phy_get_current_core(pi) == 0) {
								MOD_PHYREG(pi, FemCtrl,
									enBtSignalsToFEMLut, 0x1);
							} else {
								MOD_PHYREG(pi, FemCtrl,
									enBtSignalsToFEMLut, 0x0);
							}
						}
					}
				}
			}

			if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
				phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_SCALE(0), 0xa);
				if (BF2_DAC_SPUR_IMPROVEMENT(pi_ac) == 1) {
					phy_utils_write_radioreg(pi, RFX_2069_ADC_CFG5, 0x83e0);
				}
			}
		}
	}

	/* 20691 specific processing, if needed */
	if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID))
		wlc_phy_set_regtbl_on_band_change_acphy_20691(pi);
	else if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID))
		wlc_phy_set_regtbl_on_band_change_acphy_20693(pi);

	/* 2g/5g band can have different aci modes */
	if (!ACPHY_ENABLE_FCBS_HWACI(pi)) {
#ifndef WLC_DISABLE_ACI
		hwaci_on = (pi->sh->interference_mode & ACPHY_ACI_HWACI_PKTGAINLMT) != 0;
		wlc_phy_hwaci_setup_acphy(pi, hwaci_on, FALSE);
#endif /* !WLC_DISABLE_ACI */
		w2_on = (pi->sh->interference_mode & ACPHY_ACI_W2NB_PKTGAINLMT) != 0;
		wlc_phy_aci_w2nb_setup_acphy(pi, w2_on);
	}

	if (PHY_PAPDEN(pi)) {

		if (!ACMAJORREV_4(pi->pubpi->phy_rev))
			OSL_DELAY(100);

		if (TINY_RADIO(pi))
		{
#ifdef PHYCAL_CACHING
			ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
			ctx_valid = (ctx != NULL) ? ctx->valid : FALSE;

			/* allow reprogramming rfpwrlut if ctx is not available or
			 * ctx is available but invalid
			 */
			if (!ctx_valid)
#endif /* PHYCAL_CACHING */
				wlc_phy_papd_set_rfpwrlut_tiny(pi);
		}
		else {
			wlc_phy_papd_set_rfpwrlut(pi);
		}
	}

	/* For 4350C0, bphy is turned off when in 5G. Need to disable the predetector. */
	if (ACMAJORREV_2(pi->pubpi->phy_rev) && (ACMINORREV_1(pi) || ACMINORREV_3(pi))) {
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 0);
		} else {
			MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 1);
		}
	}

	/* Turn ON 11n 256 QAM in 2.4G */
	if (ACMAJORREV_4(pi->pubpi->phy_rev))
	{
		bool enable = (CHSPEC_IS2G(pi->radio_chanspec) && CHSPEC_IS20(pi->radio_chanspec));

		WRITE_PHYREG(pi, miscSigCtrl, enable ? 0x203 : 0x3);

		wlapi_11n_proprietary_rates_enable(pi->sh->physhim, enable);

		PHY_INFORM(("wl%d %s: 11n turbo QAM %s\n",
			PI_INSTANCE(pi), __FUNCTION__,
			enable ? "enabled" : "disabled"));
	}

	/* need to zero out cal coeffs on band change */
	bzero(txcal_cache, sizeof(txcal_cache));
	wlc_phy_cal_coeffs_upd(pi, txcal_cache);

	ACPHY_ENABLE_STALL(pi, stall_val);
}

static void
wlc_phy_set_regtbl_on_bw_change_acphy(phy_info_t *pi)
{
	int sp_tx_bw = 0;
	uint8 stall_val, core, nbclip_cnt_4360 = 15;
	uint16 rfseq_bundle_adcrst48[3];
	uint16 rfseq_bundle_adcrst49[3];
	uint16 rfseq_bundle_adcrst50[3];
	uint8 rxevm20p[] = {8, 6, 4}, rxevm20n[] = {4, 6, 8};
	uint8 rxevm0[] = {0, 0, 0}, rxevm_len = 3;
	uint32 epa_turnon_time;

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	if (BW_RESET == 1)
		wlc_phy_set_reg_on_bw_change_acphy(pi);

	if (CHIPID(pi->sh->chip) == BCM4335_CHIP_ID &&
		pi->sh->chippkg == BCM4335_FCBGA_PKG_ID)
		nbclip_cnt_4360 = 12;

	if (CHSPEC_IS80(pi->radio_chanspec)) {
		/* 80mhz */
		if (ACMAJORREV_0(pi->pubpi->phy_rev))
			sp_tx_bw = 5;
		else
			sp_tx_bw = 6;

		nbclip_cnt_4360 *= 4;
	} else if (CHSPEC_IS40(pi->radio_chanspec)) {
		/* 40mhz */
		if (ACMAJORREV_0(pi->pubpi->phy_rev))
			sp_tx_bw = 4;
		else
			sp_tx_bw = 5;

		nbclip_cnt_4360 *= 2;
	} else if (CHSPEC_IS20(pi->radio_chanspec)) {
		/* 20mhz */
		if (ACMAJORREV_0(pi->pubpi->phy_rev))
			sp_tx_bw = 3;
		else
			sp_tx_bw = 5;
	} else {
		PHY_ERROR(("%s: No primary channel settings for bw=%d\n",
		           __FUNCTION__, CHSPEC_BW(pi->radio_chanspec)));
	}

	/* reduce NB clip CNT thresholds */
	FOREACH_CORE(pi, core) {
		if (!ACMAJORREV_1(pi->pubpi->phy_rev) ||
			(CHSPEC_IS2G(pi->radio_chanspec) && BF3_AGC_CFG_2G(pi->u.pi_acphy)) ||
			(CHSPEC_IS5G(pi->radio_chanspec) && BF3_AGC_CFG_5G(pi->u.pi_acphy))) {
			MOD_PHYREGC(pi, FastAgcClipCntTh, core, fastAgcNbClipCntTh,
				nbclip_cnt_4360);
		} else {
			MOD_PHYREGC(pi, FastAgcClipCntTh, core, fastAgcNbClipCntTh, 23);
		}
	}

	wlc_phy_set_analog_tx_lpf(pi, 0x100, -1, sp_tx_bw, sp_tx_bw, -1, -1, -1);
	/* change the barelyclipgainbackoff to 6 for 80Mhz due to some PER issue for 4360A0 CHIP */
	if (ACREV_IS(pi->pubpi->phy_rev, 0)) {
	  if (CHSPEC_IS80(pi->radio_chanspec)) {
	      ACPHYREG_BCAST(pi, Core0computeGainInfo, 0xcc0);
	  } else {
	      ACPHYREG_BCAST(pi, Core0computeGainInfo, 0xc60);
	  }
	}

#ifndef WL_FDSS_DISABLED
	/* Enable FDSS */
	if (TINY_RADIO(pi) && ((CHSPEC_IS2G(pi->radio_chanspec) && (pi->fdss_level_2g[0] != -1)) ||
		(CHSPEC_IS5G(pi->radio_chanspec) && (pi->fdss_level_5g[0] != -1))))  {
		wlc_phy_fdss_init(pi);
		wlc_phy_set_fdss_table(pi);
	}
#endif /* WL_FDSS_DISABLED */

	/* SWWLAN-28943 */
	if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
		MOD_PHYREGC(pi, computeGainInfo, 0, gainBackoffValue, 1);
	}

	if (ACMAJORREV_2(pi->pubpi->phy_rev) && (ACMINORREV_1(pi) || ACMINORREV_3(pi))) {
		FOREACH_CORE(pi, core) {
			/* Reduces 54Mbps humps */
			MOD_PHYREGC(pi, computeGainInfo, core, gainBackoffValue, 1);
			/* Reduces 20in80 humps */
			WRITE_PHYREGC(pi, Clip2Threshold, core, 0xa04e);
		}
	}

	/* Shape rxevm table due to hit on near DC_tones */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_2(pi->pubpi->phy_rev) ||
	    ACMAJORREV_5(pi->pubpi->phy_rev)) {
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			/* Freq Bins {1 2 3} = {8 6 4} dB */
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_NVRXEVMSHAPINGTBL,
			                          rxevm_len, 1, 8, rxevm20p);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_NVRXEVMSHAPINGTBL,
			                          rxevm_len, 64 - rxevm_len, 8, rxevm20n);
		} else {
			/* Reset the 20mhz entries to 0 */
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_NVRXEVMSHAPINGTBL,
			                          rxevm_len, 1, 8, rxevm0);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_NVRXEVMSHAPINGTBL,
			                          rxevm_len, 64 - rxevm_len, 8, rxevm0);
		}
	}

	/* JIRA (HW11ACRADIO-30) - clamp_en needs to be high for ~1us for clipped pkts (80mhz) */
	if (CHSPEC_IS80(pi->radio_chanspec) && !TINY_RADIO(pi)) {
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_clamp_en, 1);
			MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_iqadc_clamp_en, 1);}

		rfseq_bundle_adcrst48[2]  = 0;
		rfseq_bundle_adcrst49[2]  = 0;
		rfseq_bundle_adcrst50[2]  = 0;
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			rfseq_bundle_adcrst48[0] = 0xef52;
			rfseq_bundle_adcrst48[1] = 0x94;
			rfseq_bundle_adcrst49[0] = 0xef42;
			rfseq_bundle_adcrst49[1] = 0x84;
			rfseq_bundle_adcrst50[0] = 0xef52;
			rfseq_bundle_adcrst50[1] = 0x84;
		} else if (CHSPEC_IS40(pi->radio_chanspec)) {
			rfseq_bundle_adcrst48[0] = 0x4f52;
			rfseq_bundle_adcrst48[1] = 0x94;
			rfseq_bundle_adcrst49[0] = 0x4f42;
			rfseq_bundle_adcrst49[1] = 0x84;
			rfseq_bundle_adcrst50[0] = 0x4f52;
			rfseq_bundle_adcrst50[1] = 0x84;
		} else {
			rfseq_bundle_adcrst48[0] = 0x0fd2;
			rfseq_bundle_adcrst48[1] = 0x96;
			rfseq_bundle_adcrst49[0] = 0x0fc2;
			rfseq_bundle_adcrst49[1] = 0x86;
			rfseq_bundle_adcrst50[0] = 0x0fd2;
			rfseq_bundle_adcrst50[1] = 0x86;
		}
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 48, 48,
		                          rfseq_bundle_adcrst48);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 49, 48,
		                          rfseq_bundle_adcrst49);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE, 1, 50, 48,
		                          rfseq_bundle_adcrst50);

		/* updategainH : issue adc reset for 250ns */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0x30, 16, rf_updh_cmd_adcrst);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0xa0, 16, rf_updh_dly_adcrst);

		/* updategainL : issue adc reset for 250ns */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0x40, 16, rf_updl_cmd_adcrst);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0xb0, 16, rf_updl_dly_adcrst);

		/* updategainU : issue adc reset for 250n */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0x50, 16, rf_updu_cmd_adcrst);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0xc0, 16, rf_updu_dly_adcrst);
	} else {
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			/* 4360A0 : SD-ADC was not monotonic for 1st revision, but is fixed now */
			if (ACREV_IS(pi->pubpi->phy_rev, 0)) {
				MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_clamp_en, 0);
			} else {
				MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_clamp_en, 1);
			}
			MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_iqadc_clamp_en, 1);
		}

		/* updategainH : increase clamp_en off delay to 16 */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0x30, 16, rf_updh_cmd_clamp);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0xa0, 16, rf_updh_dly_clamp);

		/* updategainL : increase clamp_en off delay to 16 */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0x40, 16, rf_updl_cmd_clamp);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0xb0, 16, rf_updl_dly_clamp);

		/* updategainU : increase clamp_en off delay to 16 */
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0x50, 16, rf_updu_cmd_clamp);
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 8, 0xc0, 16, rf_updu_dly_clamp);
	}

	if (ACMAJORREV_1(pi->pubpi->phy_rev) || TINY_RADIO(pi)) {
			if (CHSPEC_IS20(pi->radio_chanspec)) {
				WRITE_PHYREG(pi, nonpaydecodetimeoutlen, 1);
				MOD_PHYREG(pi, timeoutEn, resetCCAontimeout, 1);
				MOD_PHYREG(pi, timeoutEn, nonpaydecodetimeoutEn, 1);
			} else {
				WRITE_PHYREG(pi, nonpaydecodetimeoutlen, 32);
				MOD_PHYREG(pi, timeoutEn, resetCCAontimeout, 0);
				MOD_PHYREG(pi, timeoutEn, nonpaydecodetimeoutEn, 0);
			}
	}

	/* 4360, 4350. 4335 does its own stuff */
	if (!ACMAJORREV_1(pi->pubpi->phy_rev)) {
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			/* reduce clip2 len, helps with humps due to late clip2 */
			WRITE_PHYREG(pi, defer_setClip1_CtrLen, 20);
			WRITE_PHYREG(pi, defer_setClip2_CtrLen, 16);
		} else {
			/* increase clip1 len. Needed for 20in80, 40in80 cases */
			WRITE_PHYREG(pi, defer_setClip1_CtrLen, 30);
			WRITE_PHYREG(pi, defer_setClip2_CtrLen, 20);
		}
	} else if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi) &&
		(!(PHY_ILNA(pi))) && pi->sh->chippkg != BCM4335_FCBGA_PKG_ID) {
		if (CHSPEC_IS80(pi->radio_chanspec)) {
			/* increase clip1 defer  len to make clip gain more accurate */
			/* decrease clip1 carrier blanking length to speedup crs */
			/* this is okay fror 80MHz as the settling is very fast for wider BW */
			WRITE_PHYREG(pi, defer_setClip1_CtrLen, 36);
			WRITE_PHYREG(pi, defer_setClip2_CtrLen, 16);
			WRITE_PHYREG(pi, clip1carrierDetLen, 77);
			WRITE_PHYREG(pi, clip2carrierDetLen, 72);
		} else {
		  /* increase defer setclip Gain by 0.1usec */
		  /* reduce clip1 carrier detect blanking by same amount */
		  /* reduce clip2 carrier detect blanking to speedup carrier detect */
		  /* this helps in cleaning the small floor in 4335C0 epa boards */
			WRITE_PHYREG(pi, defer_setClip1_CtrLen, 28);
			WRITE_PHYREG(pi, defer_setClip2_CtrLen, 16);
			WRITE_PHYREG(pi, clip1carrierDetLen, 87);
			WRITE_PHYREG(pi, clip2carrierDetLen, 62);
		}
	}

	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* CRS. 6bit MF */
		/* BPHY pre-detect is disabled by default. No writes here. */
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			WRITE_PHYREG(pi, crsThreshold2u, 0x2055);
			WRITE_PHYREG(pi, crsThreshold2l, 0x2055);
		} else {
			WRITE_PHYREG(pi, crsThreshold2u, 0x204d);
			WRITE_PHYREG(pi, crsThreshold2l, 0x204d);
		}
		WRITE_PHYREG(pi, crsThreshold2lSub1, 0x204d);
		WRITE_PHYREG(pi, crsThreshold2uSub1, 0x204d);
	}

	if (ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* Spur canceller */
		if (CHSPEC_IS20(pi->radio_chanspec))
			WRITE_PHYREG(pi, spur_can_phy_bw_mhz, 0x14);
		else if (CHSPEC_IS40(pi->radio_chanspec))
			WRITE_PHYREG(pi, spur_can_phy_bw_mhz, 0x280);
		else
			WRITE_PHYREG(pi, spur_can_phy_bw_mhz, 0x50);
	}

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {

		uint16 phymode = phy_get_phymode(pi);

		if (phymode == PHYMODE_MIMO) {
			if (CHSPEC_IS20(pi->radio_chanspec)) {
				MOD_PHYREG(pi, CRSMiscellaneousParam, crsMfFlipCoef, 0);
				WRITE_PHYREG(pi, crsThreshold2u, 0x2055);
				WRITE_PHYREG(pi, crsThreshold2l, 0x2055);
			} else {
				MOD_PHYREG(pi, CRSMiscellaneousParam, crsMfFlipCoef, 1);
			}
		}
	}

	if (PHY_IPA(pi) && ACMAJORREV_2(pi->pubpi->phy_rev) &&
	    (ACMINORREV_3(pi) || ACMINORREV_5(pi))) {
		/* 4354a1_ipa, to decrease LOFT, move TSSI_CONFIG & extra delay before IPA_PU. Need
		   to move in TSSI_CONFIG, otherwise only delaying IPA_PU would delay TSSI_CONFIG
		   ;80MHz alone this change is backed out..
		*/
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			if (((CHSPEC_IS2G(pi->radio_chanspec)) &&
				(pi->u.pi_acphy->srom_tssisleep_en & 0x1)) ||
				((CHSPEC_IS5G(pi->radio_chanspec)) &&
				(pi->u.pi_acphy->srom_tssisleep_en & 0x4))) {
					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
						16, rfseq_rx2tx_cmd_rev15_ipa_withtssisleep);
			} else {
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
				                 16, rfseq_rx2tx_cmd_rev15_ipa);
			}
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x70,
			                         16, rfseq_rx2tx_dly_rev15_ipa20);
		} else if (CHSPEC_IS40(pi->radio_chanspec)) {
			if (((CHSPEC_IS2G(pi->radio_chanspec)) &&
				(pi->u.pi_acphy->srom_tssisleep_en & 0x2)) ||
				((CHSPEC_IS5G(pi->radio_chanspec)) &&
				(pi->u.pi_acphy->srom_tssisleep_en & 0x8))) {
					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
						16, rfseq_rx2tx_cmd_rev15_ipa_withtssisleep);
			} else {
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
				                 16, rfseq_rx2tx_cmd_rev15_ipa);
			}
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x70,
			                          16, rfseq_rx2tx_dly_rev15_ipa40);
		}
	}

	/* R8000 - atlas has different PA turn on timing */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		epa_turnon_time = (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) &
		                   BFL_SROM11_EPA_TURNON_TIME) >> BFL_SROM11_EPA_TURNON_TIME_SHIFT;
		if (epa_turnon_time == 1) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
			                          16, rfseq_rx2tx_cmd);
			if (CHSPEC_IS20(pi->radio_chanspec)) {
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x70,
				                          16, rfseq_rx2tx_dly_epa1_20);
			} else if (CHSPEC_IS40(pi->radio_chanspec)) {
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x70,
				                          16, rfseq_rx2tx_dly_epa1_40);
			} else {
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x70,
				                          16, rfseq_rx2tx_dly_epa1_80);
			}
		}
	}


	ACPHY_ENABLE_STALL(pi, stall_val);
}
static void
wlc_phy_set_sfo_on_chan_change_acphy(phy_info_t *pi, uint8 ch)
{
	const uint16 *val_ptr = NULL;

	if (!TINY_RADIO(pi)) {
		const void *chan_info;

		if (wlc_phy_chan2freq_acphy(pi, ch, &chan_info) < 0)
			return;

		if (ACMAJORREV_2(pi->pubpi->phy_rev)) {
			const chan_info_radio2069revGE32_t *ciGE32 = chan_info;
			val_ptr = &(ciGE32->PHY_BW1a);
		} else if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
			if ((RADIO2069REV(pi->pubpi->radiorev) != 25) &&
				(RADIO2069REV(pi->pubpi->radiorev) != 26)) {
				const chan_info_radio2069revGE16_t *ciGE16 = chan_info;
				val_ptr = &(ciGE16->PHY_BW1a);
			} else {
				if (pi->xtalfreq != 52000000) {
					const chan_info_radio2069revGE25_t *ciGE25 = chan_info;
					val_ptr = &(ciGE25->PHY_BW1a);
				} else {
					const chan_info_radio2069revGE25_52MHz_t *ciGE25
						= chan_info;
					val_ptr = &(ciGE25->PHY_BW1a);
				}
			}
		} else {
			const chan_info_radio2069_t *ci = chan_info;
			val_ptr = &(ci->PHY_BW1a);
		}
		wlc_phy_write_sfo_params_acphy(pi, val_ptr);
	} else {
		if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
			const chan_info_radio20691_t *ci20691;

			if (wlc_phy_chan2freq_20691(pi, ch, &ci20691) >= 0) {
				WRITE_PHYREG(pi, BW2, ci20691->PHY_BW2);
				WRITE_PHYREG(pi, BW5, ci20691->PHY_BW5);
			}
		} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			const chan_info_radio20693_pll_t *pll_tbl;
			const chan_info_radio20693_rffe_t *rffe_tbl;

			if (phy_get_phymode(pi) != PHYMODE_80P80) {
				if (wlc_phy_chan2freq_20693(pi, ch, &pll_tbl, &rffe_tbl) >= 0) {
					val_ptr = &(pll_tbl->PHY_BW1a);
					wlc_phy_write_sfo_params_acphy(pi, val_ptr);
				}
			} else {
				/* For First freq segment */
				ch = wf_chspec_primary80_channel(pi->radio_chanspec);
				if (wlc_phy_chan2freq_20693(pi, ch, &pll_tbl, &rffe_tbl) >= 0) {
					val_ptr = &(pll_tbl->PHY_BW1a);
					wlc_phy_write_sfo_params_acphy(pi, val_ptr);
				}

				/* For second freq segment */
				ch = wf_chspec_secondary80_channel(pi->radio_chanspec);
				if (wlc_phy_chan2freq_20693(pi, ch, &pll_tbl, &rffe_tbl) >= 0) {
					val_ptr = &(pll_tbl->PHY_BW1a);
					wlc_phy_write_sfo_80p80_params_acphy(pi, val_ptr);
				}
			}
		}
	}

}

static void
wlc_phy_write_sfo_params_acphy(phy_info_t *pi, const uint16 *val_ptr)
{
	ASSERT(val_ptr != NULL);
	if (val_ptr != NULL) {
		/* set SFO parameters */
		/* For 4349, BW3 register bit[1] is used for Lower BW cases(5 and 2.5MHz)
		to provide high resoultion
		*/
		if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
			WRITE_PHYREG(pi, BW1a, val_ptr[0]);
			WRITE_PHYREG(pi, BW3, val_ptr[2]);
		}
		WRITE_PHYREG(pi, BW2, val_ptr[1]);
		/* Set sfo_chan_center_factor */
		WRITE_PHYREG(pi, BW4, val_ptr[3]);
		WRITE_PHYREG(pi, BW5, val_ptr[4]);
		WRITE_PHYREG(pi, BW6, val_ptr[5]);
	}
}
static void
wlc_phy_write_sfo_80p80_params_acphy(phy_info_t *pi, const uint16 *val_ptr)
{
	ASSERT(val_ptr != NULL);
	if (val_ptr != NULL) {
		/* set SFO parameters */
		WRITE_PHYREG(pi, BW1a1, val_ptr[0]);
		WRITE_PHYREG(pi, BW21, val_ptr[1]);
		WRITE_PHYREG(pi, BW31, val_ptr[2]);
		/* Set sfo_chan_center_factor */
		WRITE_PHYREG(pi, BW41, val_ptr[3]);
		WRITE_PHYREG(pi, BW51, val_ptr[4]);
		WRITE_PHYREG(pi, BW61, val_ptr[5]);
	}
}

static void
chanspec_setup_regtbl_on_chan_change(phy_info_t *pi)
{
	uint32 rx_afediv_sel, tx_afediv_sel;
	uint32 read_val[2], write_val[2];
	bool suspend;
	uint8 stall_val, orig_rxfectrl1;
	uint8 bphy_testmode_val;
	uint8 ch[NUM_CHANS_IN_CHAN_BONDING];

	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* get the center freq */
	int fc = pi_ac->fc;

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	/* get the operating channels */
	chanspec_get_operating_channels(pi, ch);

	/* -ve freq means channel not found in tuning table */
	if (fc < 0)
		return;

	PHY_CHANLOG(pi, __FUNCTION__, TS_ENTER, 0);

	/* Setup the Tx/Rx Farrow resampler */
	if (TINY_RADIO(pi))
		wlc_phy_farrow_setup_tiny(pi, pi->radio_chanspec);
	else
		wlc_phy_farrow_setup_acphy(pi, pi->radio_chanspec);

	/* Load Pdet related settings */
	wlc_phy_set_pdet_on_reset_acphy(pi);

	/* 4350A0 radio */
	if ((RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) &&
	    (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) &&
	    !(ISSIM_ENAB(pi->sh->sih))) {
		suspend = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
		if (!suspend)
			wlapi_suspend_mac_and_wait(pi->sh->physhim);

		/* Disable stalls and hold FIFOs in reset */
		stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
		orig_rxfectrl1 = READ_PHYREGFLD(pi, RxFeCtrl1, soft_sdfeFifoReset);

		ACPHY_DISABLE_STALL(pi);
		MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 1);

		/* AFE clk and Harmonic of 40 MHz crystal causes a spur at 417 Khz */
		if (CHSPEC_IS20(pi->radio_chanspec)) {
			/* if AFE divider of 8 is used for 20 MHz channel 149,153,
			 * or any channel in 2GHz when xtalfreq=40MHz,
			 * or any 2Ghz channel except 2467 when xtalfreq=37.4MHz
			 * so change divider ratio to 9
			 */
			if ((CHSPEC_IS2G(pi->radio_chanspec) &&
			        ((fc != 2412 && fc != 2467) || (pi->xtalfreq == 40000000) ||
			        PHY_IPA(pi))) || (fc == 5745) || (fc == 5765) || (fc == 5180 &&
			        ACMAJORREV_2(pi->pubpi->phy_rev) &&
				((ACMINORREV_1(pi) && pi->sh->chippkg == 2) ||
			        ACMINORREV_3(pi)) && pi->xtalfreq == 37400000)) {
					wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT,
						1, 6, 60, &read_val);
				rx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x4 << 14);
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 0, 60,
				                         &read_val);
				tx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x4 << 14);
			} else {
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 6, 60,
				                         &read_val);
				rx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x3 << 14);
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 0, 60,
				                         &read_val);
				tx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x3 << 14);
			}
			/* RX_SD_ADC_PU_VAL bw20 */
			write_val[0] = ((rx_afediv_sel & 0xfff) << 20) | rx_afediv_sel;
			write_val[1] = (rx_afediv_sel << 8) | (rx_afediv_sel >> 12);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 6, 60,
			                          write_val);
			/* bw20_HighspeedMode1 */
			write_val[0] = ((tx_afediv_sel & 0xfff) << 20) | tx_afediv_sel;
			write_val[1] = (tx_afediv_sel << 8) | (tx_afediv_sel >> 12);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 0, 60,
			                          write_val);
			wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_RX2TX);
			wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_TX2RX);
		} else if (CHSPEC_IS40(pi->radio_chanspec)) {
			/* if AFE divider of 4 is used for 40 MHz channel 151m,
			 * so change divider ratio to 4.5
			 */
			if ((CHSPEC_IS2G(pi->radio_chanspec)) || (fc == 5755) ||
			    (fc == 5550 && pi->xtalfreq == 40000000) ||
			    (fc == 5310 && pi->xtalfreq == 37400000)) {
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 7, 60,
				                         &read_val);
				rx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x2 << 14);
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 1, 60,
				                         &read_val);
				tx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x2 << 14);
			} else {
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 7, 60,
				                         &read_val);
				rx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x1 << 14);
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 1, 60,
				                         &read_val);
				tx_afediv_sel = (read_val[0] & ~(0x7 << 14) & 0xfffff) |
				        (0x1 << 14);
			}
			/* RX_SD_ADC_PU_VAL bw40 */
			write_val[0] = ((rx_afediv_sel & 0xfff) << 20) | rx_afediv_sel;
			write_val[1] = (rx_afediv_sel << 8) | (rx_afediv_sel >> 12);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 7, 60,
			                          write_val);
			/* bw40_HighspeedMode1 */
			write_val[0] = ((tx_afediv_sel & 0xfff) << 20) | tx_afediv_sel;
			write_val[1] = (tx_afediv_sel << 8) | (tx_afediv_sel >> 12);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQEXT, 1, 1, 60,
			                          write_val);
			wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_RX2TX);
			wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_TX2RX);
		}
		if (!suspend)
			wlapi_enable_mac(pi->sh->physhim);

		/* Restore FIFO reset and Stalls */
		ACPHY_ENABLE_STALL(pi, stall_val);
		MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, orig_rxfectrl1);
	}

	wlc_phy_set_sfo_on_chan_change_acphy(pi, ch[0]);

	/* Set the correct primary channel */
	if (CHSPEC_IS8080(pi->radio_chanspec)) {
		/* 80P80 */
		if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_LLL) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 0);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 0);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_LLU) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 0);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 1);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_LUL) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 0);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 2);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_LUU) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 0);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 3);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_ULL) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 1);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 0);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_ULU) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 1);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 1);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_UUL) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 1);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 2);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_UUU) {
			MOD_PHYREG(pi, ClassifierCtrl_80p80, prim_sel_hi, 1);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 3);
		} else {
			PHY_ERROR(("%s: No primary channel settings for CTL_SB=%d\n",
			           __FUNCTION__, CHSPEC_CTL_SB(pi->radio_chanspec)));
		}
	} else if (CHSPEC_IS80(pi->radio_chanspec)) {
		/* 80mhz */
		if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_LL) {
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 0);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_LU) {
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 1);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_UL) {
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 2);
		} else if (CHSPEC_CTL_SB(pi->radio_chanspec) == WL_CHANSPEC_CTL_SB_UU) {
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 3);
		} else {
			PHY_ERROR(("%s: No primary channel settings for CTL_SB=%d\n",
			           __FUNCTION__, CHSPEC_CTL_SB(pi->radio_chanspec)));
		}
	} else if (CHSPEC_IS40(pi->radio_chanspec)) {
		/* 40mhz */
		if (CHSPEC_SB_UPPER(pi->radio_chanspec)) {
			MOD_PHYREG(pi, RxControl, bphy_band_sel, 1);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 1);
		} else {
			MOD_PHYREG(pi, RxControl, bphy_band_sel, 0);
			MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 0);
		}
	} else if (CHSPEC_IS20(pi->radio_chanspec)) {
		/* 20mhz */
		MOD_PHYREG(pi, RxControl, bphy_band_sel, 0);
		MOD_PHYREG(pi, ClassifierCtrl2, prim_sel, 0);
	} else {
		PHY_ERROR(("%s: No primary channel settings for bw=%d\n",
		           __FUNCTION__, CHSPEC_BW(pi->radio_chanspec)));
	}

	/* set aci thresholds */
	wlc_phy_set_aci_regs_acphy(pi);

	bzero((uint8 *)pi->u.pi_acphy->phy_noise_all_core,
	      sizeof(pi->u.pi_acphy->phy_noise_all_core));
	bzero((uint8 *)pi->u.pi_acphy->phy_noise_in_crs_min,
	      sizeof(pi->u.pi_acphy->phy_noise_in_crs_min));
	bzero((uint8 *)pi->u.pi_acphy->phy_noise_pwr_array,
	      sizeof(pi->u.pi_acphy->phy_noise_pwr_array));

	/* Debug parameters: printed by 'wl dump phycal' */
	pi->u.pi_acphy->phy_debug_crscal_counter = 0;
	pi->u.pi_acphy->phy_noise_counter = 0;

	/* set the crsmin_th from cache at chan_change */
	wlc_phy_crs_min_pwr_cal_acphy(pi, PHY_CRS_SET_FROM_CACHE);

	/* making IIR filter gaussian like for BPHY to improve ACPR */

	/* set RRC filter alpha
	 FiltSel2 is 11 bit which msb, bphyTest's 6th bit is lsb
	 These 2 bits control alpha
	 bits 11 & 6    Resulting filter
	  -----------    ----------------
	      00         alpha=0.35 - default
	      01         alpha=0.75 - alternate
	      10         alpha=0.2  - for use in Japan on channel 14
	      11         no TX filter
	*/
	if ((fc == 2484) && (!CHSPEC_IS8080(pi->radio_chanspec))) {
		bphy_testmode_val = (0x3F & READ_PHYREGFLD(pi, bphyTest, testMode));
		MOD_PHYREG(pi, bphyTest, testMode, bphy_testmode_val);
		MOD_PHYREG(pi, bphyTest, FiltSel2, 1);
		/* Load default filter */
		wlc_phy_set_tx_iir_coeffs(pi, 1, 0);
	} else {
		if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
			MOD_PHYREG(pi, bphyTest, FiltSel2, 0);
			if (PHY_IPA(pi)) {
				wlc_phy_set_tx_iir_coeffs(pi, 1, 2);
			} else {
				wlc_phy_set_tx_iir_coeffs(pi, 1, 1);
			}
		} else if ACMAJORREV_3(pi->pubpi->phy_rev) {
			MOD_PHYREG(pi, bphyTest, testMode, 0);
			MOD_PHYREG(pi, bphyTest, FiltSel2, 0);
		} else if ACMAJORREV_4(pi->pubpi->phy_rev) {
				MOD_PHYREG(pi, bphyTest, testMode, 0);
				MOD_PHYREG(pi, bphyTest, FiltSel2, 1);
				wlc_phy_set_tx_iir_coeffs(pi, 1, pi->sromi->cckfilttype);
				wlc_phy_set_tx_iir_coeffs(pi, 0, 0); /* default setting for ofdm */
				MOD_PHYREG(pi, BphyControl3, bphyScale20MHz, 0x3b);
				if (((fc == 2412) || (fc == 2462) || (fc == 2467) ||
					(fc == 2472)) &&
					(pi->sromi->ofdmfilttype_2g != 127)) {
					wlc_phy_set_tx_iir_coeffs(pi, 0,
						pi->sromi->ofdmfilttype_2g);
				} else if (((fc == 5240) || (fc == 5260) || (fc == 5580) ||
					(fc == 5660)) &&
					(pi->sromi->ofdmfilttype != 127)) {
					wlc_phy_set_tx_iir_coeffs(pi, 0, pi->sromi->ofdmfilttype);
				}
		} else {
			bphy_testmode_val = (0x3F & READ_PHYREGFLD(pi, bphyTest, testMode));
			bphy_testmode_val = bphy_testmode_val |
				((pi->sromi->cckfilttype & 0x2)  << 5);
			MOD_PHYREG(pi, bphyTest, testMode, bphy_testmode_val);
			MOD_PHYREG(pi, bphyTest, FiltSel2,
				((pi->sromi->cckfilttype & 0x4) >> 2));
			/* Load filter with Gaussian shaping */
			wlc_phy_set_tx_iir_coeffs(pi, 1, (pi->sromi->cckfilttype & 0x1));
		}
		if (ACMAJORREV_1(pi->pubpi->phy_rev) && PHY_IPA(pi)) {
			MOD_PHYREG(pi, bphyTest, testMode, 0);
			MOD_PHYREG(pi, bphyTest, FiltSel2, 0);
			wlc_phy_set_tx_iir_coeffs(pi, 1, (pi->sromi->cckfilttype & 0xF));
		}
	}

	/* if it's 2x2 or 3x3 design, populate the reciprocity compensation coeff */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_2(pi->pubpi->phy_rev) ||
	    ACMAJORREV_5(pi->pubpi->phy_rev)) {
		wlc_phy_populate_recipcoeffs_acphy(pi);
	}

	/* 4335c0 wlipa 2GHz xtal spur war */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi) && PHY_ILNA(pi)) {
		MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_outbufBBstrg, 1);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufBBstrg, 0);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufcalstrg, 0);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL5, xtal_bufstrg_BT, 1);
		MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_xtbufstrg, 1);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_xtbufstrg, 7);
		MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_outbufstrg, 1);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufstrg, 3);
	}

	/* 4354 wlipa 2GHz xtal spur war */
	if (ACMAJORREV_2(pi->pubpi->phy_rev) && (ACMINORREV_1(pi) ||
		ACMINORREV_3(pi)) && PHY_ILNA(pi)) {
		MOD_RADIO_REG(pi, RFP, PLL_XTAL2, xtal_pu_RCCAL1, 0);
		MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_outbufBBstrg, 1);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufBBstrg, 0);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufcalstrg, 0);
		MOD_RADIO_REG(pi, RFP, GE16_OVR27, ovr_xtal_outbufstrg, 1);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL4, xtal_outbufstrg, 2);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL5, xtal_sel_BT, 1);
		MOD_RADIO_REG(pi, RFP, PLL_XTAL5, xtal_bufstrg_BT, 2);
	}

	if (BFCTL(pi->u.pi_acphy) == 3) {
		if (fc == 5180 || fc == 5190 || fc == 5310 ||
		    fc == 5320 || fc == 5500 || fc == 5510) {
			MOD_RADIO_REG(pi, RFP, PLL_CP4, rfpll_cp_ioff, 0xA0);
		}
	}
	PHY_CHANLOG(pi, __FUNCTION__, TS_EXIT, 0);
}

#define MAX_2069_AFECAL_WAITLOOPS 10
static void
wlc_phy_radio2069_afecal(phy_info_t *pi)
{
	uint8 core, itr, done_i, done_q;
	uint16 adc_cfg4, *afe_cfg_arr;

	if (ISSIM_ENAB(pi->sh->sih))
		return;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	/* Used to latch (clk register) rcal, rccal, ADC cal code */
	MOD_RADIO_REG(pi, RFP, PLL_XTAL2, xtal_pu_RCCAL, 1);

	/* Allocate storage to save current config registers */
	/* 3 registers per AFE core: RfctrlCoreAfeCfg1, RfctrlCoreAfeCfg2, RfctrlOverrideAfeCfg */
	afe_cfg_arr =
	MALLOC(pi->sh->osh, 3 * PHYCORENUM((pi)->pubpi->phy_corenum) * sizeof(uint16));

	/* Proceed only if allocation successful */
	if (afe_cfg_arr == NULL) {
		PHY_ERROR(("wl%d: %s: MALLOC failure\n", pi->sh->unit, __FUNCTION__));
		return;
	}
	/* Save config registers and issue reset */
	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		/* Cfg1  in 0 to PHYCORENUM-1 */
		afe_cfg_arr[core] = READ_PHYREGCE(pi, RfctrlCoreAfeCfg1, core);
		/* Cfg2 in PHYCORENUM to 2*PHYCORENUM -1 */
		afe_cfg_arr[core + PHYCORENUM(pi->pubpi->phy_corenum)] =
		  READ_PHYREGCE(pi, RfctrlCoreAfeCfg2, core);
		/* Overrides in 2*PHYCORENUM to 3*PHYCORENUM - 1 */
		afe_cfg_arr[core + 2*PHYCORENUM(pi->pubpi->phy_corenum)] =
		  READ_PHYREGCE(pi, RfctrlOverrideAfeCfg, core);
		MOD_PHYREGCE(pi, RfctrlCoreAfeCfg1, core, afe_iqadc_reset, 1);
		MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_iqadc_reset, 1);
		MOD_RADIO_REGC(pi, ADC_CFG3, core, flash_calrstb, 0); /* reset */
	}

	OSL_DELAY(100);

	/* Bring each AFE core back from reset and perform cal */
	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		MOD_RADIO_REGC(pi, ADC_CFG3, core, flash_calrstb, 1);
		adc_cfg4 = READ_RADIO_REGC(pi, RF, ADC_CFG4, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CFG4(core), adc_cfg4 | 0xf);

		done_i = 0; done_q = 0;
		for (itr = 0; itr < MAX_2069_AFECAL_WAITLOOPS; itr++) {
			OSL_DELAY(10);
			done_i = READ_RADIO_REGFLDC(pi, RF_2069_ADC_STATUS(core), ADC_STATUS,
				i_wrf_jtag_afe_iqadc_Ich_cal_state);
			done_q = READ_RADIO_REGFLDC(pi, RF_2069_ADC_STATUS(core), ADC_STATUS,
				i_wrf_jtag_afe_iqadc_Qch_cal_state);
			if ((done_i == 1) && (done_q == 1)) {
				PHY_INFORM(("wl%d: %s afecal(%d) done\n",
					pi->sh->unit, __FUNCTION__, core));
				break;
			}
		}
		/* Don't assert for QT */
		if (!ISSIM_ENAB(pi->sh->sih)) {
			ASSERT((done_i == 1) && (done_q == 1));
		}
		/* calMode = 0 */
		phy_utils_write_radioreg(pi, RF_2069_ADC_CFG4(core), (adc_cfg4 & 0xfff0));
		/* Restore AFE config registers for that core with saved values */
		WRITE_PHYREGCE(pi, RfctrlCoreAfeCfg1, core, afe_cfg_arr[core]);
		WRITE_PHYREGCE(pi, RfctrlCoreAfeCfg2, core,
			afe_cfg_arr[core + PHYCORENUM(pi->pubpi->phy_corenum)]);
		WRITE_PHYREGCE(pi, RfctrlOverrideAfeCfg, core,
			afe_cfg_arr[core + 2 * PHYCORENUM(pi->pubpi->phy_corenum)]);
	}

	/* Turn off clock */
	MOD_RADIO_REG(pi, RFP, PLL_XTAL2, xtal_pu_RCCAL, 0);
	if (RADIO2069REV(pi->pubpi->radiorev) < 4) {
	  /* JIRA (CRDOT11ACPHY-153) calCodes are inverted for 4360a0 */
	  wlc_phy_radio2069_afecal_invert(pi);
	}
	/* Free allocated memory */
	MFREE(pi->sh->osh, afe_cfg_arr, 3 * sizeof(uint16) * PHYCORENUM(pi->pubpi->phy_corenum));

}


static void
wlc_2069_rfpll_150khz(phy_info_t *pi)
{
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	MOD_RADIO_REG(pi, RFP, PLL_LF4, rfpll_lf_lf_r1, 0);
	MOD_RADIO_REG(pi, RFP, PLL_LF4, rfpll_lf_lf_r2, 2);
	phy_utils_write_radioreg(pi, RFP_2069_PLL_LF5, 2);
	MOD_RADIO_REG(pi, RFP, PLL_LF7, rfpll_lf_lf_rs_cm, 2);
	MOD_RADIO_REG(pi, RFP, PLL_LF7, rfpll_lf_lf_rf_cm, 0xff);
	phy_utils_write_radioreg(pi, RFP_2069_PLL_LF2, 0xffff);
	phy_utils_write_radioreg(pi, RFP_2069_PLL_LF3, 0xffff);
}

static void
wlc_phy_2069_4335_set_ovrds(phy_info_t *pi)
{
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR30, 0x1df3);
	phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR31, 0x1ffc);
	phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR32, 0x0078);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		phy_utils_write_radioreg(pi, RF0_2069_GE16_OVR28, 0x0);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR29, 0x0);
	} else {
		phy_utils_write_radioreg(pi, RF0_2069_GE16_OVR28, 0xffff);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR29, 0xffff);
		if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1)
			if (PHY_IPA(pi))
			    phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR29, 0x6900);
	}


}

static void
wlc_phy_2069_4350_set_ovrds(phy_info_t *pi)
{
	uint8 core;
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR30, 0x1df3);
	phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR31, 0x1ffc);
	phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR32, 0x0078);

	MOD_RADIO_REG(pi, RFP, GE16_PLL_HVLDO4, ldo_2p5_static_load_CP, 0x1);
	MOD_RADIO_REG(pi, RFP, GE16_PLL_HVLDO4, ldo_2p5_static_load_VCO, 0x1);
	if (PHY_IPA(pi)&&(pi->xtalfreq == 37400000)&&CHSPEC_IS2G(pi->radio_chanspec)) {
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			MOD_RADIO_REGC(pi, PA2G_CFG1, core, pa2g_bias_reset, 1);
			MOD_RADIO_REGC(pi, GE16_OVR13, core, ovr_pa2g_bias_reset, 1);
			}
	}
	if ((RADIO2069REV(pi->pubpi->radiorev) == 36) ||
		(RADIO2069REV(pi->pubpi->radiorev) >= 39)) {
		MOD_PHYREG(pi, radio_logen2g, idac_qb, 0x2);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_itx, 0x3);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_irx, 0x3);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_qrx, 0x3);
		MOD_PHYREG(pi, radio_logen2g, idac_qtx, 0x3);
		MOD_RADIO_REG(pi, RFP, PLL_CFG4, rfpll_spare2, 0x6);
		MOD_RADIO_REG(pi, RFP, PLL_CFG4, rfpll_spare3, 0x34);

		phy_utils_write_radioreg(pi, RFP_2069_TOP_SPARE7, 0x1);
		phy_utils_write_radioreg(pi, RFP_2069_PLL_LF1, 0x48);
	}
}

static void
acphy_set_lpmode(phy_info_t *pi, acphy_lp_opt_levels_t lp_opt_lvl)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
	    ACMAJORREV_3(pi->pubpi->phy_rev)) {
		switch (lp_opt_lvl) {
		case ACPHY_LP_RADIO_LVL_OPT:
			if (CHSPEC_IS2G(pi->radio_chanspec)) {
				if (pi_ac->lpmode_2g == ACPHY_LPMODE_LOW_PWR_SETTINGS_1) {
					wlc_phy_radio_vco_opt(pi, ACPHY_VCO_2P5V);
				} else if (pi_ac->lpmode_2g == ACPHY_LPMODE_LOW_PWR_SETTINGS_2) {
					wlc_phy_radio_vco_opt(pi, ACPHY_VCO_1P35V);
				}
			}
			break;
		case ACPHY_LP_CHIP_LVL_OPT:
		case ACPHY_LP_PHY_LVL_OPT:
		default:
			break;
		}
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* 4349 related power optimizations */
		bool for_2g = ((pi_ac->lpmode_2g != ACPHY_LPMODE_NONE) &&
			(CHSPEC_IS2G(pi->radio_chanspec)));
		bool for_5g = ((pi_ac->lpmode_5g != ACPHY_LPMODE_NONE) &&
			(CHSPEC_IS5G(pi->radio_chanspec)));

		switch (lp_opt_lvl) {
		case ACPHY_LP_RADIO_LVL_OPT:
		{
			if ((phy_get_phymode(pi) == PHYMODE_MIMO) &&
				((for_2g == TRUE) || (for_5g == TRUE))) {
				if ((pi->sh->phyrxchain == 1) && (pi_ac->phyrxchain_old == 3)) {
					wlc_phy_radio20693_mimo_core1_pmu_off(pi);
				} else if ((pi->sh->phyrxchain == 3) &&
					(pi_ac->phyrxchain_old == 1)) {
					wlc_phy_radio20693_mimo_core1_pmu_on(pi);
				}
			}
		}
			break;
		case ACPHY_LP_CHIP_LVL_OPT:
			break;
		case ACPHY_LP_PHY_LVL_OPT:
			if ((for_2g == TRUE) || (for_5g == TRUE)) {
				MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 1);
				MOD_PHYREG(pi, CRSMiscellaneousParam,
					b_over_ag_falsedet_en, 0);
			} else {
				MOD_PHYREG(pi, CRSMiscellaneousParam, bphy_pre_det_en, 0);
				MOD_PHYREG(pi, CRSMiscellaneousParam,
					b_over_ag_falsedet_en, 1);
			}
			if (!((phy_get_phymode(pi) == PHYMODE_RSDB) &&
				(phy_get_current_core(pi) == 1))) {
				uint16 val;
				wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_BFECONFIG2X2TBL,
					1, 0, 16, &val);
				BCM_REFERENCE(val);
			}
			/*
			if {[hwaccess] == $def(hw_jtag)} {
				jtag w 0x18004492 2 0x0
				jtag w 0x18001492 2 0x2
			}
			*/

			WRITE_PHYREG(pi, FFTSoftReset, 0x2);
			break;
		default:
			break;
		}

	}
}

static void
wlc_phy_set_reg_on_reset_acphy_20691(phy_info_t *pi)
{
	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20691_ID));

	MOD_RADIO_REG_20691(pi, SPARE_CFG1, 0, spare_0, 0xfc00);
	MOD_RADIO_REG_20691(pi, SPARE_CFG2, 0, spare_1, 0x003f);

	if (ACPHY_ENABLE_FCBS_HWACI(pi))
		MOD_PHYREG(pi, FastChanSW_PLLVCOARBITR, arbitrdisable, 1);

	/* CRDOT11ACPHY-566: rx fix for dac rate mode 2 & 3 for >= rev1
	 * i.e. clear the top bit of the work_around_ctrl ACPHY register
	 */
	_PHY_REG_MOD(pi, ACPHY_work_around_ctrl(pi->pubpi->phy_rev), 0x8000, 0);

	MOD_PHYREG(pi, RxStatPwrOffset0, use_gainVar_for_rssi0, 1);
	MOD_PHYREG(pi, ForcePktAbort, dcblk_hpf_bw_en, 1);
	MOD_PHYREG(pi, HTAGCWaitCounters, HTAgcPktgainWait, 34);

	/* CRDOT11ACPHY-601: BPHY-20in20 Tapping via Datapath DC Filter */
	MOD_PHYREG(pi, RxSdFeConfig5, tiny_bphy20_ADC10_sel, 0);
	MOD_PHYREG(pi, RxFeCtrl1, swap_iq1, 1);
	MOD_PHYREG(pi, RxFeCtrl1, swap_iq2, 0);
	MOD_PHYREG(pi, bphyTest, dccomp, 0);

	/* maximum drive strength */
	MOD_RADIO_REG_20691(pi, TIA_CFG8, 0, tia_offset_comp_drive_strength, 1);

	/* DCC FSM Defaults */
	MOD_PHYREG(pi, BBConfig, dcc_wakeup_restart_en, 0);
	MOD_PHYREG(pi, BBConfig, dcc_wakeup_restart_delay, 10);
	MOD_PHYREG(pi, dcc_ctrl_restart_length_grp, dcc_ctrl_restart_length, 0xffff);

	/* Set DCC FSM to run and then stop - i.e  do not idle, */
	MOD_PHYREG(pi, rx_tia_dc_loop_0, en_lock, 1);

	/* Correct sign of loop gain */
	MOD_PHYREG(pi, rx_tia_dc_loop_0, dac_sign, 1);

	/* disable DVG2 to avoid bphy resampler saturation */
	MOD_PHYREG(pi, RxSdFeConfig5, tiny_bphy20_ADC10_sel, 0);

	/* digital-packet gain only */
	MOD_PHYREG(pi, singleShotAgcCtrl, singleShotPktGainElement, 96);

	MOD_PHYREG(pi, overideDigiGain1, cckdigigainEnCntValue, 119);
}

static void
wlc_phy_set_regtbl_on_femctrl(phy_info_t *pi)
{
	uint8 stall_val;
	uint8 bt_fem;	/* bitfield in PHY register BT_FemControl */
	uint8 gpio_en;	/* set which gpio pins are controlled by the PHY and which by ucode */
	bool bt_on_gpio4;
	uint32 chipcontrol_mask; /* chipcommon core chipcontrol register */
	uint32 chipcontrol_val;

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	if (!ACPHY_FEMCTRL_ACTIVE(pi)) {
		wlc_phy_write_regtbl_fc_from_nvram(pi);
	} else {
		switch (BFCTL(pi->u.pi_acphy)) {
		case 0:
			/* Chip default, do nothing */
			break;
		case 1:
			/* chip_bandsel = bandsel */
			MOD_PHYREG(pi, BT_SwControl, bt_sharing_en, 1);
			wlc_phy_write_femctrl_table(pi->u.pi_acphy->anai);
			break;
		case 2:
			/*	X29c & 4352hmb(wiht B0)
				Cores {0, 2} have 5516 fem. Core 1 has separate 2g/5g fems
			*/
			bt_fem = 0; bt_on_gpio4 = FALSE;
			wlc_phy_write_femctrl_table(pi->u.pi_acphy->anai);
			gpio_en = 0xa0;
			chipcontrol_mask = CCTRL4360_SECI_MODE | CCTRL4360_SECI_ON_GPIO01 |
				CCTRL4360_BTSWCTRL_MODE;
			chipcontrol_val = 0;

			if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 0) {
				bt_on_gpio4 = TRUE;  /* fem_bt = gpio4 */
				gpio_en = 0xe0;
			} else if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 3) {
				/*
				 * For 5517. In 43602 turn off VLIN override mux, and always keep
				 * VLIN high through FEM CTRL * table.
				 * bcm943602bu : 3 antenna board, no BT support
				 * bcm943602cd (X238) : 3 Wifi + 1 BT antenna board
				 */
				MOD_PHYREG(pi, RfctrlCoreGlobalPus,
					muxTxVlinOnFemCtrl, 0x0);
				chipcontrol_val = (CCTRL4360_SECI_MODE | CCTRL4360_SECI_ON_GPIO01);
				bt_fem = 4;      /* fem_bt = bt_fem[2] */
				gpio_en = 0xa0;  /* bt on gpio6 */
			} else if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 4) {
				/*
				 * For 5517. In 43602 turn off VLIN override mux, and
				 * always keep VLIN high through FEM CTRL * table.
				 * bcm943602cs (X87) : 3 antenna, middle antenna is shared BT/Wifi.
				 * gpio7 flows towards FEM BT_EN pin.
				 */
				MOD_PHYREG(pi, RfctrlCoreGlobalPus, muxTxVlinOnFemCtrl, 0x0);
				chipcontrol_val = (CCTRL4360_SECI_MODE | CCTRL4360_SECI_ON_GPIO01 |
					CCTRL4360_BTSWCTRL_MODE);
				bt_fem = 2; /* fem_bt = bt_fem[1] */
				gpio_en = 0x60; /* d[7]=0 -> allows ucode to control gpio7 */
			} else {
				bt_fem = 4; /* fem_bt = bt_fem[2] */
				gpio_en = 0xa0;  /* bt on gpio6 */
			}

			if (chipcontrol_val != 0) {
				si_corereg(pi->sh->sih, SI_CC_IDX,
					OFFSETOF(chipcregs_t, chipcontrol),
					chipcontrol_mask, chipcontrol_val);
			}

			/* Setup middle core for BT */
			wlc_phy_set_bt_on_core1_acphy(pi, bt_fem, gpio_en);

			/* Release control of gpio4 if required */
			if (bt_on_gpio4)
				wlc_phy_bt_on_gpio4_acphy(pi);
			break;
		case 3:
			/*	Routers (MCH5, J28) */
			MOD_PHYREG(pi, BT_SwControl, bt_sharing_en, 0);
			if (ACMAJORREV_5(pi->pubpi->phy_rev)) {
				/* all 43602 chips */
				if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 0 ||
				    BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 3) {
					if (ACMINORREV_0(pi)) {
						/* 43602a0: enable PAVREF WAR for boards
						 * that enable PA with PAVREF
						 */
						wlc_phy_enable_pavref_war(pi);
					} else {
						/* 43602a1 and later: power on PAVREF LDO
						 * for boards that enable PA with PAVREF
						 */
						si_pmu_switch_on_PARLDO(pi->sh->sih, pi->sh->osh);
					}
				}
			} else {
				si_pmu_regcontrol(pi->sh->sih, 0, 0x4, 4); /* pwron pavref ldo */
			}
			wlc_phy_write_femctrl_table(pi->u.pi_acphy->anai);

			if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) == 5) {
				/* MCH2 with digital PA control */
				si_corereg(pi->sh->sih, SI_CC_IDX,
					OFFSETOF(chipcregs_t, chipcontrol), 0xfffffd,
					CCTRL4360_DISCRETE_FEMCTRL_MODE |
					CCTRL4360_DIGITAL_PACTRL_MODE);
			} else if (BF3_FEMCTRL_SUB(pi->u.pi_acphy) < 2 ||
			           ACMAJORREV_5(pi->pubpi->phy_rev)) {
				/* MCH5 and 43602MHC2: leave bit1 untouched for uart */
				si_corereg(pi->sh->sih, SI_CC_IDX,
					OFFSETOF(chipcregs_t, chipcontrol),
					0xfffffd, CCTRL4360_DISCRETE_FEMCTRL_MODE);
			}
			/* STB (USBH5) */
			if ((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags2) &
				BFL2_SROM11_ANAPACTRL_5G) &&
				(BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardtype) ==
				BCM94360USBH5_D11AC5G)) {
				/* power on PA(bit 2) & RF(bit 1) LDO */
				wlapi_bmac_write_shm(pi->sh->physhim, M_RFLDO_ON_L, 0x4);
				wlapi_bmac_write_shm(pi->sh->physhim, M_RFLDO_ON_H, 0x20);
			}
			break;
		case 5:
			wlc_phy_write_femctrl_table(pi->u.pi_acphy->anai);
			/* Setup middle core for BT */
			wlc_phy_set_bt_on_core1_acphy(pi, 8, 0xc0);
			break;
		case 6:
			wlc_phy_write_femctrl_table(pi->u.pi_acphy->anai);
			break;
		case 4:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
			wlc_phy_write_sparse_femctrl_table(pi);
			break;
			/* LOOK: when adding new cases, follow above pattern to
			 * minimize stack/memory usage!
			 */
		default:
			/* 5516 on all cores */
			/* chip_bandsel = bandsel */
			MOD_PHYREG(pi, BT_SwControl, bt_sharing_en, 1);
			wlc_phy_write_femctrl_table(pi->u.pi_acphy->anai);
			break;
		}
	}

	if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
		if (BF_SROM11_BTCOEX(pi->u.pi_acphy)) {
			if (ACMAJORREV_0(pi->pubpi->phy_rev)) {
				if (ACMINORREV_0(pi)) {
					si_corereg(pi->sh->sih,
						SI_CC_IDX, OFFSETOF(chipcregs_t, chipcontrol),
						CCTRL4360_SECI_MODE, CCTRL4360_SECI_MODE);
				} else if (ACMINORREV_1(pi)) {
					si_corereg(pi->sh->sih,
						SI_CC_IDX, OFFSETOF(chipcregs_t, chipcontrol),
						CCTRL4360_SECI_ON_GPIO01, CCTRL4360_SECI_ON_GPIO01);
				} else {
					ASSERT(0);
				}
			} else if (ACMAJORREV_1(pi->pubpi->phy_rev) ||
				ACMAJORREV_2(pi->pubpi->phy_rev) ||
				ACMAJORREV_3(pi->pubpi->phy_rev) ||
				ACMAJORREV_5(pi->pubpi->phy_rev)) {
				PHY_ERROR(("wl%d: %s: FIXME bt_coex\n",
					pi->sh->unit, __FUNCTION__));
			} else {
				ASSERT(0);
			}
		}
	}

	ACPHY_ENABLE_STALL(pi, stall_val);
}


/*
gmult_rc (24:17), gmult(16:9), bq1_bw(8:6), rc_bw(5:3), bq0_bw(2:0)
LO: (15:0), HI (24:16)
mode_mask = bits[0:8] = 11b_20, 11n_20, 11ag_11ac_20, 11b_40, 11n_40, 11ag_11ac_40, 11b_80,
11n_11ag_11ac_80, samp_play
*/
static void
wlc_phy_set_analog_tx_lpf(phy_info_t *pi, uint16 mode_mask, int bq0_bw, int bq1_bw,
                       int rc_bw, int gmult, int gmult_rc, int core_num)
{
	uint8 ctr, core, max_modes = 9;
	uint16 addr_lo_offs[] = {0x142, 0x152, 0x162};
	uint16 addr_hi_offs[] = {0x362, 0x372, 0x382};
	uint16 addr_lo_base, addr_hi_base, addr_lo, addr_hi;
	uint16 val_lo, val_hi;
	uint32 val;
	uint8 stall_val;
	/* This proc does not impact 4349, so return without doing anything */
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		return;
	}
	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);
	/* core_num = -1 ==> all cores */
	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		if ((core_num == -1) || (core_num == core)) {
			addr_lo_base = addr_lo_offs[core];
			addr_hi_base = addr_hi_offs[core];
			for (ctr = 0; ctr < max_modes; ctr++) {
				if ((mode_mask >> ctr) & 1) {
					addr_lo = addr_lo_base + ctr;
					addr_hi = addr_hi_base + ctr;
					wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                         1, addr_lo, 16, &val_lo);
					wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                         1, addr_hi, 16, &val_hi);
					val = (val_hi << 16) | val_lo;

					if (bq0_bw >= 0) {
						val = (val & 0x1fffff8) | (bq0_bw << 0);
						}
					if (rc_bw >= 0) {
						val = (val & 0x1ffffc7) | (rc_bw << 3);
					}
					if (bq1_bw >= 0) {
						val = (val & 0x1fffe3f) | (bq1_bw << 6);
					}
					if (gmult >= 0) {
						val = (val & 0x1fe01ff) | (gmult << 9);
					}
					if (gmult_rc >= 0) {
						val = (val & 0x001ffff) | (gmult_rc << 17);
					}

					val_lo = val & 0xffff;
					val_hi = (val >> 16) & 0x1ff;
					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                          1, addr_lo, 16, &val_lo);
					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                          1, addr_hi, 16, &val_hi);
				}
			}
		}
	}
	ACPHY_ENABLE_STALL(pi, stall_val);
}

/*
dacbuf_fixed_cap[5], dacbuf_cap[4:0]
mode_mask = bits[0:8] = 11b_20, 11n_20, 11ag_11ac_20, 11b_40, 11n_40, 11ag_11ac_40, 11b_80,
11n_11ag_11ac_80, samp_play
*/
static void
wlc_phy_set_tx_afe_dacbuf_cap(phy_info_t *pi, uint16 mode_mask, int dacbuf_cap,
                           int dacbuf_fixed_cap, int core_num)
{
	uint8 ctr, core, max_modes = 9;
	uint16 core_base[] = {0x3f0, 0x60, 0xd0};
	uint8 offset[] = {0xb, 0xb, 0xc, 0xc, 0xe, 0xe, 0xf, 0xf, 0xa};
	uint8 shift[] = {0, 6, 0, 6, 0, 6, 0, 6, 0};
	uint16 addr, read_val, val;
	uint8 stall_val;
	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);
	/* core_num = -1 ==> all cores */
	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		if ((core_num == -1) || (core_num == core)) {
			for (ctr = 0; ctr < max_modes; ctr++) {
				if ((mode_mask >> ctr) & 1) {
					addr = core_base[core] + offset[ctr];
					wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                         1, addr, 16, &read_val);
					val = (read_val >> shift[ctr]) & 0x3f;

					if (dacbuf_cap >= 0) {
							val = (val & 0x20) | dacbuf_cap;
					}
					if (dacbuf_fixed_cap >= 0) {
						val = (val & 0x1f) |
						        (dacbuf_fixed_cap << 5);
					}

					if (shift[ctr] == 0) {
						val = (read_val & 0xfc0) | val;
					} else {
						val = (read_val & 0x3f) | (val << 6);
					}

					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                          1, addr, 16, &val);
				}
			}
		}
	}
	ACPHY_ENABLE_STALL(pi, stall_val);
}

/*
gmult_rc (24:17), rc_bw(16:14), gmult(13:6), bq1_bw(5:3), bq0_bw(2:0)
LO: (15:0), HI (24:16)
mode_mask = bits[0:2] = 20, 40, 80
*/
static void
wlc_phy_set_analog_rx_lpf(phy_info_t *pi, uint8 mode_mask, int bq0_bw, int bq1_bw,
                  int rc_bw, int gmult, int gmult_rc, int core_num)
{
	uint8 ctr, core, max_modes = 3;
	uint16 addr20_lo_offs[] = {0x140, 0x150, 0x160};
	uint16 addr20_hi_offs[] = {0x360, 0x370, 0x380};
	uint16 addr40_lo_offs[] = {0x141, 0x151, 0x161};
	uint16 addr40_hi_offs[] = {0x361, 0x371, 0x381};
	uint16 addr80_lo_offs[] = {0x441, 0x443, 0x445};
	uint16 addr80_hi_offs[] = {0x440, 0x442, 0x444};
	uint16 addr_lo, addr_hi;
	uint16 val_lo, val_hi;
	uint32 val;
	uint8 stall_val;
	/* This proc does not impact 4349, so return without doing anything */
	if (ACMAJORREV_4(pi->pubpi->phy_rev))
		return;

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);
	/* core_num = -1 ==> all cores */
	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		if ((core_num == -1) || (core_num == core)) {
			for (ctr = 0; ctr < max_modes; ctr++) {
				if ((mode_mask >> ctr) & 1) {
					if (ctr == 0) {
						addr_lo = addr20_lo_offs[core];
						addr_hi = addr20_hi_offs[core];
					}
					else if (ctr == 1) {
						addr_lo = addr40_lo_offs[core];
						addr_hi = addr40_hi_offs[core];
					} else {
						addr_lo = addr80_lo_offs[core];
						addr_hi = addr80_hi_offs[core];
					}

					wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                         1, addr_lo, 16, &val_lo);
					wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ,
					                         1, addr_hi, 16, &val_hi);
					val = (val_hi << 16) | val_lo;

					if (bq0_bw >= 0) {
						val = (val & 0x1fffff8) | (bq0_bw << 0);
					}
					if (bq1_bw >= 0) {
						val = (val & 0x1ffffc7) | (bq1_bw << 3);
					}
					if (gmult >= 0) {
						val = (val & 0x1ffc03f) | (gmult << 6);
					}
					if (rc_bw >= 0) {
						val = (val & 0x1fe3fff) | (rc_bw << 14);
					}
					if (gmult_rc >= 0) {
						val = (val & 0x001ffff) | (gmult_rc << 17);
					}

					val_lo = val & 0xffff;
					val_hi = (val >> 16) & 0x1ff;
					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
					                          addr_lo, 16, &val_lo);
					wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
					                          addr_hi, 16, &val_hi);
				}
			}
		}
	}
	ACPHY_ENABLE_STALL(pi, stall_val);
}

static void
acphy_load_txv_for_spexp(phy_info_t *pi)
{
	uint32 len = 243, offset = 1220;

	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_BFMUSERINDEX,
	                          len, offset, 32, acphy_txv_for_spexp);
}

static void
wlc_phy_cfg_energydrop_timeout(phy_info_t *pi)
{
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		/* Fine timing mod to have more overlap(~10dB) between low and high SNR regimes
		 * change to 0x8 to prevent the radar to trigger the fine timing
		 */
		MOD_PHYREG(pi, FSTRMetricTh, hiPwr_min_metric_th, 0x8);
		/* change it to 40000 for radar detection */
		WRITE_PHYREG(pi, energydroptimeoutLen, 0x9c40);
	} else {
		/* Fine timing mod to have more overlap(~10dB) between low and high SNR regimes */
		MOD_PHYREG(pi, FSTRMetricTh, hiPwr_min_metric_th, 0xf);
		/* In event of high power spurs/interference that causes crs-glitches,
		 * stay in WAIT_ENERGY_DROP for 1 clk20 instead of default 1 ms.
		 * This way, we get back to CARRIER_SEARCH quickly and will less likely to miss
		 * actual packets. PS: this is actually one settings for ACI
		 */
		WRITE_PHYREG(pi, energydroptimeoutLen, 0x2);
	}
}

static void
wlc_phy_set_regtbl_on_band_change_acphy_20691(phy_info_t *pi)
{
	uint8 core, bw;
	tiny_adc_tuning_array_t gvalues;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20691_ID));
	bw = CHSPEC_IS20(pi->radio_chanspec) ? 20 : CHSPEC_IS40(pi->radio_chanspec) ? 40 : 80;

	/* ### 20691_band_set(pi); */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		/* # Restore PHY control for Gband blocks which may have been switched
		 * off in Aband
		 */
		MOD_RADIO_REG_20691(pi, TIA_CFG8, 0, tia_offset_dac_biasadj, 4);
		MOD_RADIO_REG_20691(pi, LOGEN_OVR1, 0, ovr_logencore_2g_pu, 0);
		phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, TX_TOP_2G_OVR_EAST, 0), 0);
		phy_utils_write_radioreg(pi,
			RADIO_REG_20691(pi, TX_TOP_2G_OVR_NORTH, 0), 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_gm2g_auxgm_pwrup, 1);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_gm2g_pwrup, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_EAST, 0, ovr_lna2g_dig_wrssi1_pu, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_NORTH, 0, ovr_rxmix2g_pu, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_2G_OVR_NORTH, 0, ovr_lna2g_lna1_pu, 0);

		/* lna5g_pu_lna2 seems to get switched on during 5G band switch */
		MOD_RADIO_REG_20691(pi, LNA5G_CFG2, 0, lna5g_pu_lna2, 0);

		/* # Misc */
		MOD_RADIO_REG_20691(pi, TX_LPF_CFG2, 0, lpf_sel_5g_out_gm, 0);
		MOD_RADIO_REG_20691(pi, TX_LPF_CFG3, 0, lpf_sel_2g_5g_cmref_gm, 0);

		if (!PHY_IPA(pi)) {
			MOD_RADIO_REG_20691(pi, TXMIX2G_CFG6, 0, mx2g_idac_bbdc, 0x20);
		}

	} else {
		/* # clear 5g overrides */
		MOD_RADIO_REG_20691(pi, TIA_CFG8, 0, tia_offset_dac_biasadj, 12);
		MOD_RADIO_REG_20691(pi, LOGEN_OVR1, 0, ovr_logencore_5g_pu, 0);
		phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, TX_TOP_5G_OVR1, 0), 0);
		phy_utils_write_radioreg(pi, RADIO_REG_20691(pi, TX_TOP_5G_OVR2, 0), 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_5G_OVR, 0, ovr_lna5g_lna1_pu, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_5G_OVR, 0, ovr_gm5g_pwrup, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_5G_OVR, 0, ovr_lna5g_dig_wrssi1_pu, 0);
		MOD_RADIO_REG_20691(pi, RX_TOP_5G_OVR, 0, ovr_lna5g_pu_auxlna2, 1);
		MOD_RADIO_REG_20691(pi, LNA5G_CFG2, 0, lna5g_pu_auxlna2, 0);

		/* to power up/down logen appropriately */
		MOD_RADIO_REG_20691(pi, TX_LOGEN5G_CFG1, 0, logen5g_tx_enable_5g_low_band, 0);
		MOD_RADIO_REG_20691(pi, TX_TOP_5G_OVR2, 0, ovr_logen5g_tx_enable_5g_low_band, 1);

		/* # Restore PHY control for Gband blocks which may have been switched
		 * off in Aband
		 */
		MOD_RADIO_REG_20691(pi, LOGEN_OVR1, 0, ovr_logencore_5g_pu, 0);

		/* # Misc */
		/* # There is no direct control for this */
		MOD_RADIO_REG_20691(pi, TX_LPF_CFG2, 0, lpf_sel_5g_out_gm, 1);
		/* # There is no direct control for this */
		MOD_RADIO_REG_20691(pi, TX_LPF_CFG3, 0, lpf_sel_2g_5g_cmref_gm,
		                    (PHY_IPA(pi)) ? 1 : 0);

		if ((RADIO20691_MAJORREV(pi->pubpi->radiorev) != 0) && !(PHY_IPA(pi))) {
			MOD_RADIO_REG_20691(pi, TXMIX2G_CFG6, 0, mx2g_idac_bbdc, 0xb);
			if (ACPHY_IBOARD(pi)) {
				MOD_RADIO_REG_20691(pi, TXMIX5G_CFG8, 0, pad5g_idac_gm, 0x18);
				MOD_RADIO_REG_20691(pi, PA5G_INCAP, 0, pad5g_idac_pmos, 0x34);
				MOD_RADIO_REG_20691(pi, TX5G_CFG1, 0, pad5g_slope_gm, 0x0);
				MOD_RADIO_REG_20691(pi, TXMIX5G_CFG6, 0, mx5g_ptat_slope_lodc, 0x0);
				MOD_RADIO_REG_20691(pi, PA5G_INCAP, 0, pa5g_idac_incap_compen_main,
					0x2f);
			}
		}
	}
	if (CHSPEC_IS80(pi->radio_chanspec)) {
		/* set gvalues [20691_sigdel_fast_tune $def(radio_rccal_adc_gmult)] */
		wlc_tiny_sigdel_fast_tune(pi, pi->u.pi_acphy->rccal_adc_gmult, &gvalues);
		/* 20691_adc_setup_fast $gvalues */
		wlc_tiny_adc_setup_fast(pi, &gvalues, 0);
	} else {
		/* set gvalues [20691_sigdel_slow0g6_tune $def(radio_rccal_adc_gmult)] */
		wlc_tiny_sigdel_slow_tune(pi, pi->u.pi_acphy->rccal_adc_gmult, &gvalues, bw);
		/* 20691_adc_setup_slow0g6 $gvalues */
		wlc_tiny_adc_setup_slow(pi, &gvalues, bw, 0);
	}

	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
	    wlc_tiny_tia_config(pi, core);
	}
}

static void
wlc_phy_set_reg_on_bw_change_acphy(phy_info_t *pi)
{
	uint8 core;

	if (TINY_RADIO(pi))
		MOD_PHYREG(pi, TssiEnRate, StrobeRateOverride, 1);
	MOD_PHYREG(pi, TssiEnRate, StrobeRate, CHSPEC_IS20(pi->radio_chanspec) ?
	0x1 : CHSPEC_IS40(pi->radio_chanspec) ? 0x2 : 0x3);
	MOD_PHYREG(pi, ClassifierCtrl, mac_bphy_band_sel, CHSPEC_IS20(pi->radio_chanspec) ?
	0x1 : CHSPEC_IS40(pi->radio_chanspec) ? 0x0  : 0x0);
	MOD_PHYREG(pi, RxControl, bphy_band_sel, CHSPEC_IS20(pi->radio_chanspec) ?
	0x1 : CHSPEC_IS40(pi->radio_chanspec) ? 0x0 : 0x0);
	MOD_PHYREG(pi, DcFiltAddress, dcCoef0, CHSPEC_IS20(pi->radio_chanspec) ?
	0x15 : CHSPEC_IS40(pi->radio_chanspec) ? 0xb : 0x5);
	if (ACMAJORREV_4(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, iqest_input_control, dc_accum_wait_vht,
			CHSPEC_IS20(pi->radio_chanspec) ? 0xc :
			CHSPEC_IS40(pi->radio_chanspec) ? 0x1d : 0x3b);
		MOD_PHYREG(pi, iqest_input_control, dc_accum_wait_mm,
			CHSPEC_IS20(pi->radio_chanspec) ? 0xb :
			CHSPEC_IS40(pi->radio_chanspec) ? 0x1b : 0x37);
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			MOD_PHYREG(pi, IqestWaitTime, waitTime,
				CHSPEC_IS20(pi->radio_chanspec) ? 0x14  : 0x28);
			if (ACMINORREV_0(pi) || ACMINORREV_1(pi)) {
				MOD_PHYREG(pi, FSTRCtrl, fineStrSgiVldCntVal,
					CHSPEC_IS20(pi->radio_chanspec) ? 0x9 : 0xa);
				MOD_PHYREG(pi, FSTRCtrl, fineStrVldCntVal,
					CHSPEC_IS20(pi->radio_chanspec) ? 0x9 : 0xa);
			}
		}
	}

	if (!TINY_RADIO(pi)) {
		MOD_PHYREG(pi, RxFilt40Num00, RxFilt40Num00, CHSPEC_IS20(pi->radio_chanspec) ?
			0x146 : CHSPEC_IS40(pi->radio_chanspec) ? 0x181 : 0x17a);
		MOD_PHYREG(pi, RxFilt40Num01, RxFilt40Num01, CHSPEC_IS20(pi->radio_chanspec) ?
			0x88 : CHSPEC_IS40(pi->radio_chanspec) ? 0x5a : 0x9e);
		MOD_PHYREG(pi, RxFilt40Num02, RxFilt40Num02, CHSPEC_IS20(pi->radio_chanspec) ?
			0x146 : CHSPEC_IS40(pi->radio_chanspec) ? 0x181 : 0x17a);
		MOD_PHYREG(pi, RxFilt40Den00, RxFilt40Den00, CHSPEC_IS20(pi->radio_chanspec) ?
			0x76e : CHSPEC_IS40(pi->radio_chanspec) ? 0x793 : 0x7ca);
		MOD_PHYREG(pi, RxFilt40Den01, RxFilt40Den01, CHSPEC_IS20(pi->radio_chanspec) ?
			0x1a8 : CHSPEC_IS40(pi->radio_chanspec) ? 0x1b7 : 0x1b2);
		MOD_PHYREG(pi, RxFilt40Num10, RxFilt40Num10, CHSPEC_IS20(pi->radio_chanspec) ?
			0xa3 : CHSPEC_IS40(pi->radio_chanspec) ? 0xc1 : 0xbd);
		MOD_PHYREG(pi, RxFilt40Num11, RxFilt40Num11, CHSPEC_IS20(pi->radio_chanspec) ?
			0xf4 : CHSPEC_IS40(pi->radio_chanspec) ? 0x102 : 0x114);
		MOD_PHYREG(pi, RxFilt40Num12, RxFilt40Num12, CHSPEC_IS20(pi->radio_chanspec) ?
			0xa3 : CHSPEC_IS40(pi->radio_chanspec) ? 0xc1 : 0xbd);
		MOD_PHYREG(pi, RxFilt40Den10, RxFilt40Den10, CHSPEC_IS20(pi->radio_chanspec) ?
			0x684 : CHSPEC_IS40(pi->radio_chanspec) ? 0x6c0 : 0x6d6);
		MOD_PHYREG(pi, RxFilt40Den11, RxFilt40Den11, CHSPEC_IS20(pi->radio_chanspec) ?
			0xad : CHSPEC_IS40(pi->radio_chanspec) ? 0xa9 : 0xa2);
		MOD_PHYREG(pi, RxStrnFilt40Num00, RxStrnFilt40Num00,
			CHSPEC_IS20(pi->radio_chanspec) ? 0xe5 : CHSPEC_IS40(pi->radio_chanspec) ?
			0x162 : 0x16c);
		MOD_PHYREG(pi, RxStrnFilt40Num01, RxStrnFilt40Num01,
			CHSPEC_IS20(pi->radio_chanspec) ? 0x68 : CHSPEC_IS40(pi->radio_chanspec) ?
			0x42 : 0x6f);
		MOD_PHYREG(pi, RxStrnFilt40Num02, RxStrnFilt40Num02,
			CHSPEC_IS20(pi->radio_chanspec) ? 0xe5 : CHSPEC_IS40(pi->radio_chanspec) ?
			0x162 : 0x16c);
		MOD_PHYREG(pi, RxStrnFilt40Den00, RxStrnFilt40Den00,
			CHSPEC_IS20(pi->radio_chanspec) ? 0x6be : CHSPEC_IS40(pi->radio_chanspec) ?
			0x75c : 0x793);
		MOD_PHYREG(pi, RxStrnFilt40Den01, RxStrnFilt40Den01,
			CHSPEC_IS20(pi->radio_chanspec) ? 0x19e : CHSPEC_IS40(pi->radio_chanspec) ?
			0x1b3 : 0x1b2);
		MOD_PHYREG(pi, RxStrnFilt40Num10, RxStrnFilt40Num10,
			CHSPEC_IS20(pi->radio_chanspec) ? 0x73 : CHSPEC_IS40(pi->radio_chanspec) ?
			0xb1 : 0xb6);
		MOD_PHYREG(pi, RxStrnFilt40Num11, RxStrnFilt40Num11,
			CHSPEC_IS20(pi->radio_chanspec) ? 0xb2 : CHSPEC_IS40(pi->radio_chanspec) ?
			0xed : 0xff);
		MOD_PHYREG(pi, RxStrnFilt40Num12, RxStrnFilt40Num12,
			CHSPEC_IS20(pi->radio_chanspec) ? 0x73 : CHSPEC_IS40(pi->radio_chanspec) ?
			0xb1 : 0xb6);
		MOD_PHYREG(pi, RxStrnFilt40Den10, RxStrnFilt40Den10,
			CHSPEC_IS20(pi->radio_chanspec) ? 0x5fe : CHSPEC_IS40(pi->radio_chanspec) ?
			0x692 : 0x6b4);
		MOD_PHYREG(pi, RxStrnFilt40Den11, RxStrnFilt40Den11,
			CHSPEC_IS20(pi->radio_chanspec) ? 0xcc : CHSPEC_IS40(pi->radio_chanspec) ?
			0xaf : 0xa8);
	}
	MOD_PHYREG(pi, nvcfg3, noisevar_rxevm_lim_qdb, CHSPEC_IS20(pi->radio_chanspec) ?
	0x97 : CHSPEC_IS40(pi->radio_chanspec) ? 0x8b : 0x97);
	if (CHSPEC_IS5G(pi->radio_chanspec)) {
	    MOD_PHYREG(pi, RadarBlankCtrl, radarBlankingInterval,
	    CHSPEC_IS20(pi->radio_chanspec) ? 0x19 :
	    CHSPEC_IS40(pi->radio_chanspec) ? 0x32 : 0x32);
	    MOD_PHYREG(pi, RadarT3BelowMin, Count, CHSPEC_IS20(pi->radio_chanspec) ?
	    0x14 : CHSPEC_IS40(pi->radio_chanspec) ? 0x28 : 0x28);
	    MOD_PHYREG(pi, RadarT3Timeout, Timeout, CHSPEC_IS20(pi->radio_chanspec) ?
	    0xc8 : CHSPEC_IS40(pi->radio_chanspec) ? 0x190 : 0x190);
	    MOD_PHYREG(pi, RadarResetBlankingDelay, Count, CHSPEC_IS20(pi->radio_chanspec) ?
	    0x19 : CHSPEC_IS40(pi->radio_chanspec) ? 0x32 : 0x32);
	}
	MOD_PHYREG(pi, ClassifierCtrl6, logACDelta2, CHSPEC_IS20(pi->radio_chanspec) ?
	0x13 : CHSPEC_IS40(pi->radio_chanspec) ? 0x13 : 0x9);
	MOD_PHYREG(pi, ClassifierLogAC1, logACDelta1, CHSPEC_IS20(pi->radio_chanspec) ?
	0x13 : CHSPEC_IS40(pi->radio_chanspec) ? 0x13 : 0x9);
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
	  if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
	    MOD_PHYREG(pi, bphyPreDetectThreshold6, ac_det_1us_aci_th,
	    CHSPEC_IS20(pi->radio_chanspec) ?
	    0x80 : CHSPEC_IS40(pi->radio_chanspec) ? 0x200 : 0x200);
	  }
	}
	FOREACH_CORE(pi, core) {
	  MOD_PHYREGC(pi, Adcclip, core, adc_clip_cnt_th, CHSPEC_IS20(pi->radio_chanspec) ?
	  0xa : CHSPEC_IS40(pi->radio_chanspec) ? 0x14 : 0x14);
	  MOD_PHYREGC(pi, FastAgcClipCntTh, core, fastAgcNbClipCntTh,
	  CHSPEC_IS20(pi->radio_chanspec) ?
	  0x17 : CHSPEC_IS40(pi->radio_chanspec) ? 0x2a : 0x54);
	  MOD_PHYREGC(pi, FastAgcClipCntTh, core, fastAgcW1ClipCntTh,
	  CHSPEC_IS20(pi->radio_chanspec) ?
	  0xe : CHSPEC_IS40(pi->radio_chanspec) ? 0x16 : 0x2c);
	}
	if (!ACMAJORREV_0(pi->pubpi->phy_rev) &&
	    !(ACMAJORREV_2(pi->pubpi->phy_rev) && ACMINORREV_0(pi))) {
	  MOD_PHYREG(pi, CRSMiscellaneousParam, crsMfFlipCoef, CHSPEC_IS20(pi->radio_chanspec) ?
	    0x0 : 0x1);
	}
	/* FIX ME : Currently setting only for 4350, Other phy revs should
	 * check with RTL folks and set accordingly
	 */
	if (ACMAJORREV_2(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, FSTRCtrl, fineStrSgiVldCntVal, CHSPEC_IS20(pi->radio_chanspec) ?
			0x9 : 0xa);
		MOD_PHYREG(pi, FSTRCtrl, fineStrVldCntVal, CHSPEC_IS20(pi->radio_chanspec) ?
			0x9 : 0xa);
	}
}

extern uint8 avvmid_set_from_nvram[3][5][2];

/* Load pdet related Rfseq on reset */
static void
wlc_phy_set_pdet_on_reset_acphy(phy_info_t *pi)
{
	uint8 core, pdet_range_id, subband_idx, ant, core_freq_segment_map;
	uint16 offset, tmp_val, val_av, val_vmid;
	uint8 av[3] = {0, 0, 0};
	uint8 vmid[3] = {0, 0, 0};
	uint8 stall_val;

	bool flag2rangeon =
		((CHSPEC_IS2G(pi->radio_chanspec) && pi->u.pi_acphy->srom_tworangetssi2g) ||
		(CHSPEC_IS5G(pi->radio_chanspec) && pi->u.pi_acphy->srom_tworangetssi5g)) &&
		PHY_IPA(pi);

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		pdet_range_id = pi->u.pi_acphy->srom_2g_pdrange_id;
	} else {
		pdet_range_id = pi->u.pi_acphy->srom_5g_pdrange_id;
	}

	FOREACH_CORE(pi, core) {
		/* core_freq_segment_map is only required for 80P80 mode
		 For other modes, it is ignored
		*/
		core_freq_segment_map = pi->u.pi_acphy->core_freq_mapping[core];
		subband_idx = wlc_phy_get_chan_freq_range_acphy(pi, 0, core_freq_segment_map);
		ant = phy_get_rsdbbrd_corenum(pi, core);
		if (BF3_AVVMID_FROM_NVRAM(pi->u.pi_acphy)) {
			av[core] = avvmid_set_from_nvram[ant][subband_idx][0];
			vmid[core] = avvmid_set_from_nvram[ant][subband_idx][1];
		} else {
			if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
				/* 4360 and 43602 */
				av[core] = avvmid_set[pdet_range_id][subband_idx][ant];
				vmid[core] = avvmid_set[pdet_range_id][subband_idx][ant+3];
			} else if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
				if (core == 0) {
					av[core] = avvmid_set1[pdet_range_id][subband_idx][ant];
					vmid[core] = avvmid_set1[pdet_range_id][subband_idx][ant+1];
				}
			} else if (ACMAJORREV_2(pi->pubpi->phy_rev)) {
				av[core] = avvmid_set2[pdet_range_id][subband_idx][ant];
				vmid[core] = avvmid_set2[pdet_range_id][subband_idx][ant+2];
			} else if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
				if (core == 0) {
					av[core] = avvmid_set3[pdet_range_id][subband_idx][ant];
					vmid[core] =
					        avvmid_set3[pdet_range_id][subband_idx][ant+1];
				}
			} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				av[core] = avvmid_set4[pdet_range_id][subband_idx][ant];
				vmid[core] =
				        avvmid_set4[pdet_range_id][subband_idx][ant+2];
			}
		}
	}
	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	FOREACH_ACTV_CORE(pi, pi->sh->hw_phyrxchain, core) {
		if ((ACMAJORREV_1(pi->pubpi->phy_rev) && (core == 0)) ||
		    !(ACMAJORREV_1(pi->pubpi->phy_rev))) {
			offset = 0x3c0 + 0xd + core*0x10;
			wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ,
			                         1, offset, 16, &tmp_val);
			val_av = (tmp_val & 0x1ff8) | (av[core]&0x7);
			val_vmid = (val_av & 0x7) | ((vmid[core]&0x3ff)<<3);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
			                          1, offset, 16, &val_vmid);

			if (((ACMAJORREV_1(pi->pubpi->phy_rev) ||
				ACMAJORREV_2(pi->pubpi->phy_rev) ||
				ACMAJORREV_4(pi->pubpi->phy_rev)) &&
				BF3_TSSI_DIV_WAR(pi->u.pi_acphy)) ||
				flag2rangeon) {
				offset = 0x3c0 + 0xe + core*0x10;
				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ,
				                          1, offset, 16, &val_vmid);
			}
		}
	}
	ACPHY_ENABLE_STALL(pi, stall_val);
}

static void
wlc_phy_set_tx_iir_coeffs(phy_info_t *pi, bool cck, uint8 filter_type)
{
	if (cck == FALSE) {
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			/* Default filters */
			if (filter_type == 0) {
				/* Default Chebyshev ~10.5MHz cutoff */
				WRITE_PHYREG(pi, txfilt20in20st0a1, 0x0056);
				WRITE_PHYREG(pi, txfilt20in20st0a2, 0x02fb);
				WRITE_PHYREG(pi, txfilt20in20st0n, 0x0003);
				WRITE_PHYREG(pi, txfilt20in20st1a1, 0x0f3d);
				WRITE_PHYREG(pi, txfilt20in20st1a2, 0x0169);
				WRITE_PHYREG(pi, txfilt20in20st1n, 0x0003);
				WRITE_PHYREG(pi, txfilt20in20st2a1, 0x0e23);
				WRITE_PHYREG(pi, txfilt20in20st2a2, 0x0068);
				WRITE_PHYREG(pi, txfilt20in20st2n, 0x0002);
				WRITE_PHYREG(pi, txfilt20in20finescale, 0x00a6);
			} else if (filter_type == 1) {
				 /* Chebyshev ~8.8MHz cutoff (FCC -26dBr BW) */
				WRITE_PHYREG(pi, txfilt20in20st0a1, 0x0e73);
				WRITE_PHYREG(pi, txfilt20in20st0a2, 0x033d);
				WRITE_PHYREG(pi, txfilt20in20st0n, 0x0002);
				WRITE_PHYREG(pi, txfilt20in20st1a1, 0x0d5f);
				WRITE_PHYREG(pi, txfilt20in20st1a2, 0x0205);
				WRITE_PHYREG(pi, txfilt20in20st1n, 0x0003);
				WRITE_PHYREG(pi, txfilt20in20st2a1, 0x0c39);
				WRITE_PHYREG(pi, txfilt20in20st2a2, 0x011e);
				WRITE_PHYREG(pi, txfilt20in20st2n, 0x0002);
				WRITE_PHYREG(pi, txfilt20in20finescale, 0x001a);
			}
		}
	} else {
		/* Tx filters in PHY REV 3, PHY REV 6 and later operate at 1/2 the sampling
		 * rate of previous revs
		 */
		if ((ACMAJORREV_0(pi->pubpi->phy_rev) && (ACMINORREV_0(pi) || ACMINORREV_1(pi))) ||
		    (ACMAJORREV_1(pi->pubpi->phy_rev) && (ACMINORREV_0(pi) || ACMINORREV_1(pi))) ||
		    (ACMAJORREV_3(pi->pubpi->phy_rev)) || (ACMAJORREV_4(pi->pubpi->phy_rev))) {
	    if (filter_type == 0) {
	        /* Default filter */
	        WRITE_PHYREG(pi, txfiltbphy20in20st0a1, 0x0a94);
	        WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 0x0373);
	        WRITE_PHYREG(pi, txfiltbphy20in20st0n, 0x0005);
	        WRITE_PHYREG(pi, txfiltbphy20in20st1a1, 0x0a93);
	        WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 0x0298);
	        WRITE_PHYREG(pi, txfiltbphy20in20st1n, 0x0004);
	        WRITE_PHYREG(pi, txfiltbphy20in20st2a1, 0x0a52);
	        WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 0x021d);
	        WRITE_PHYREG(pi, txfiltbphy20in20st2n, 0x0004);
	        WRITE_PHYREG(pi, txfiltbphy20in20finescale, 0x0080);
	    } else if (filter_type == 1) {
	        /* Gaussian  shaping filter */
	        WRITE_PHYREG(pi, txfiltbphy20in20st0a1, 0x0b54);
	        WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 0x0290);
	        WRITE_PHYREG(pi, txfiltbphy20in20st0n, 0x0004);
	        WRITE_PHYREG(pi, txfiltbphy20in20st1a1, 0x0a40);
	        WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 0x0290);
	        WRITE_PHYREG(pi, txfiltbphy20in20st1n, 0x0005);
	        WRITE_PHYREG(pi, txfiltbphy20in20st2a1, 0x0a06);
	        WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 0x0240);
	        WRITE_PHYREG(pi, txfiltbphy20in20st2n, 0x0005);
	        WRITE_PHYREG(pi, txfiltbphy20in20finescale, 0x0080);
		} else if (filter_type == 4) {
			if (pi->u.pi_acphy->dac_mode == 1) {
				/* Gaussian shaping filter for TINY_A0, dac_rate_mode 1 */
				WRITE_PHYREG(pi, txfiltbphy20in20st0a1, -80);
				WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 369);
				WRITE_PHYREG(pi, txfiltbphy20in20st0n, 3);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a1, -757);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 369);
				WRITE_PHYREG(pi, txfiltbphy20in20st1n, 3);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a1, -1007);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 256);
				WRITE_PHYREG(pi, txfiltbphy20in20st2n, 3);
				WRITE_PHYREG(pi, txfiltbphy20in20finescale, 120);
			} else if (pi->u.pi_acphy->dac_mode == 2) {
				/* Gaussian shaping filter for TINY_A0, dac_rate_mode 2 */
				WRITE_PHYREG(pi, txfiltbphy20in80st0a1, -1852);
				WRITE_PHYREG(pi, txfiltbphy20in80st0a2, 892);
				WRITE_PHYREG(pi, txfiltbphy20in80st0n, 7);
				WRITE_PHYREG(pi, txfiltbphy20in80st1a1, -1890);
				WRITE_PHYREG(pi, txfiltbphy20in80st1a2, 892);
				WRITE_PHYREG(pi, txfiltbphy20in80st1n, 7);
				WRITE_PHYREG(pi, txfiltbphy20in80st2a1, -1877);
				WRITE_PHYREG(pi, txfiltbphy20in80st2a2, 860);
				WRITE_PHYREG(pi, txfiltbphy20in80st2n, 7);
				WRITE_PHYREG(pi, txfiltbphy20in80finescale, 65);
			} else {
				/* Gaussian shaping filter for TINY_A0, dac_rate_mode 3 */
				WRITE_PHYREG(pi, txfiltbphy20in40st0a1, -1714);
				WRITE_PHYREG(pi, txfiltbphy20in40st0a2, 829);
				WRITE_PHYREG(pi, txfiltbphy20in40st0n, 6);
				WRITE_PHYREG(pi, txfiltbphy20in40st1a1, -1796);
				WRITE_PHYREG(pi, txfiltbphy20in40st1a2, 829);
				WRITE_PHYREG(pi, txfiltbphy20in40st1n, 6);
				WRITE_PHYREG(pi, txfiltbphy20in40st2a1, -1790);
				WRITE_PHYREG(pi, txfiltbphy20in40st2a2, 784);
				WRITE_PHYREG(pi, txfiltbphy20in40st2n, 6);
				WRITE_PHYREG(pi, txfiltbphy20in40finescale, 54);
			}
	    } else if (filter_type == 5) {
				WRITE_PHYREG(pi, txfiltbphy20in20st0a1, -48);
				WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 1);
				WRITE_PHYREG(pi, txfiltbphy20in20st0n, 3);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a1, -75);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 23);
				WRITE_PHYREG(pi, txfiltbphy20in20st1n, 3);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a1, -504);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 64);
				WRITE_PHYREG(pi, txfiltbphy20in20st2n, 3);
				WRITE_PHYREG(pi, txfiltbphy20in20finescale, 175);
		}
	} else if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
	           ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
			if (filter_type == 0) {
				/* Default filter */
				WRITE_PHYREG(pi, txfiltbphy20in20st0a1, 0x0f6b);
				WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 0x0339);
				WRITE_PHYREG(pi, txfiltbphy20in20st0n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a1, 0x0e29);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 0x01e5);
				WRITE_PHYREG(pi, txfiltbphy20in20st1n, 0x0002);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a1, 0x0cb2);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 0x00f0);
				WRITE_PHYREG(pi, txfiltbphy20in20st2n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20finescale, 0x00b3);
			} else if (filter_type == 1) {
				/* Gaussian shaping filter (-0.5 dB Tx Power) */
				WRITE_PHYREG(pi, txfiltbphy20in20st0a1, 0x0edb);
				WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 0x01cb);
				WRITE_PHYREG(pi, txfiltbphy20in20st0n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a1, 0x0d1d);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 0x0192);
				WRITE_PHYREG(pi, txfiltbphy20in20st1n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a1, 0x0c33);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 0x00f3);
				WRITE_PHYREG(pi, txfiltbphy20in20st2n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20finescale, 0x0076);
			} else if (filter_type == 2) {
				/* Tweaked Gaussian for 4335 iPA CCk margin */
				WRITE_PHYREG(pi, txfiltbphy20in20st0a1, 0x0edb);
				WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 0x01ab);
				WRITE_PHYREG(pi, txfiltbphy20in20st0n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a1, 0x0d1d);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 0x0172);
				WRITE_PHYREG(pi, txfiltbphy20in20st1n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a1, 0x0c77);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 0x00a9);
				WRITE_PHYREG(pi, txfiltbphy20in20st2n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20finescale, 0x0082);
			}
		} else {
			if (filter_type == 0) {
				/* Default filter */
				WRITE_PHYREG(pi, txfiltbphy20in20st0a1, 0x0f6b);
				WRITE_PHYREG(pi, txfiltbphy20in20st0a2, 0x0339);
				WRITE_PHYREG(pi, txfiltbphy20in20st0n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a1, 0x0e29);
				WRITE_PHYREG(pi, txfiltbphy20in20st1a2, 0x01e5);
				WRITE_PHYREG(pi, txfiltbphy20in20st1n, 0x0002);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a1, 0x0cb2);
				WRITE_PHYREG(pi, txfiltbphy20in20st2a2, 0x00f0);
				WRITE_PHYREG(pi, txfiltbphy20in20st2n, 0x0003);
				WRITE_PHYREG(pi, txfiltbphy20in20finescale, 0x00b3);
			} else if (filter_type == 1) {
				/* TBD */
			}
		}
	}
}

static void
wlc_phy_radio2069_afecal_invert(phy_info_t *pi)
{
	uint8 core;
	uint16 calcode;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	/* Switch on the clk */
	MOD_RADIO_REG(pi, RFP, PLL_XTAL2, xtal_pu_RCCAL, 1);

	/* Output calCode = 1:14, latched = 15:28 */

	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		/* Use calCodes 1:14 instead of 15:28 */
		MOD_RADIO_REGC(pi, OVR3, core, ovr_afe_iqadc_flash_calcode_Ich, 1);
		MOD_RADIO_REGC(pi, OVR3, core, ovr_afe_iqadc_flash_calcode_Qch, 1);

		/* Invert the CalCodes */
		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE28, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE14(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE27, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE13(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE26, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE12(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE25, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE11(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE24, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE10(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE23, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE9(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE22, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE8(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE21, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE7(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE20, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE6(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE19, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE5(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE18, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE4(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE17, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE3(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE16, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE2(core), ~calcode & 0xffff);

		calcode = READ_RADIO_REGC(pi, RF, ADC_CALCODE15, core);
		phy_utils_write_radioreg(pi, RF_2069_ADC_CALCODE1(core), ~calcode & 0xffff);
	}

	/* Turn off the clk */
	MOD_RADIO_REG(pi, RFP, PLL_XTAL2, xtal_pu_RCCAL, 0);
}

static void
wlc_phy_write_regtbl_fc3_sub0(phy_info_t *pi)
{
	uint8 fectrl_mch5_c0_p200_p400[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		2, 4, 3, 11, 2, 4, 3, 11, 0x02, 0x24, 0x03, 0x2d, 0x02, 0x24, 0x03, 0x2d};
	uint8 fectrl_mch5_c1_p200_p400[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		2, 1, 6, 14, 2, 1, 6, 14, 0x02, 0x21, 0x06, 0x2d, 0x02, 0x21, 0x06, 0x2d};
	uint8 fectrl_mch5_c2_p200_p400[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		4, 1, 6, 14, 4, 1, 6, 14, 0x04, 0x21, 0x06, 0x2b, 0x04, 0x21, 0x06, 0x2b};

	si_corereg(pi->sh->sih, SI_CC_IDX, OFFSETOF(chipcregs_t, chipcontrol),
		0xffffff, CCTRL4360_DISCRETE_FEMCTRL_MODE);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32,	0, 8,
		fectrl_mch5_c0_p200_p400);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 32, 8,
		fectrl_mch5_c1_p200_p400);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 64, 8,
		fectrl_mch5_c2_p200_p400);

}

static void
wlc_phy_write_regtbl_fc3_sub1(phy_info_t *pi)
{
	uint8 fectrl_mch5_c0[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		8, 4, 3, 8, 8, 4, 3, 8, 0x08, 0x24, 0x03, 0x25, 0x08, 0x24, 0x03, 0x25};
	uint8 fectrl_mch5_c1[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		8, 1, 6, 8, 8, 1, 6, 8, 0x08, 0x21, 0x06, 0x25, 0x08, 0x21, 0x06, 0x25};
	uint8 fectrl_mch5_c2[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		8, 1, 6, 8, 8, 1, 6, 8, 0x08, 0x21, 0x06, 0x23, 0x08, 0x21, 0x06, 0x23};

	/* P500+ */
	si_corereg(pi->sh->sih, SI_CC_IDX, OFFSETOF(chipcregs_t, chipcontrol),
		0xffffff, CCTRL4360_DISCRETE_FEMCTRL_MODE);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32,	0, 8,
		fectrl_mch5_c0);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 32, 8,
		fectrl_mch5_c1);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 64, 8,
		fectrl_mch5_c2);
}

static void
wlc_phy_write_regtbl_fc3_sub2(phy_info_t *pi)
{
	uint8 fectrl_j28[] =  {2, 4, 3, 2, 2, 4, 3, 2, 0x22, 0x24, 0x23, 0x25, 0x22, 0x24, 0x23,
		0x25, 2, 4, 3, 2, 2, 4, 3, 2, 0x22, 0x24, 0x23, 0x25, 0x22, 0x24, 0x23, 0x25};

	/* J28 */
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32,	0, 8,
		fectrl_j28);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 32, 8,
		fectrl_j28);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 64, 8,
		fectrl_j28);
}

static void
wlc_phy_write_regtbl_fc3_sub3(phy_info_t *pi)
{
	uint8 fectrl3_sub3_c0[] = {2, 4, 3, 2, 2, 4, 3, 2, 0x22, 0x24, 0x23, 0x25, 0x22, 0x24, 0x23,
		0x25, 2, 4, 3, 2, 2, 4, 3, 2, 0x22, 0x24, 0x23, 0x25, 0x22, 0x24, 0x23, 0x25};
	uint8 fectrl3_sub3_c1[] = {2, 1, 6, 2, 2, 1, 6, 2, 0x22, 0x21, 0x26, 0x25, 0x22, 0x21, 0x26,
		0x25, 2, 1, 6, 2, 2, 1, 6, 2, 0x22, 0x21, 0x26, 0x25, 0x22, 0x21, 0x26, 0x25};
	uint8 fectrl3_sub3_c2[] = {4, 1, 6, 4, 4, 1, 6, 4, 0x24, 0x21, 0x26, 0x23, 0x24, 0x21, 0x26,
		0x23, 4, 1, 6, 4, 4, 1, 6, 4, 0x24, 0x21, 0x26, 0x23, 0x24, 0x21, 0x26, 0x23};

	/* MCH2 */
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32,	0, 8,
		fectrl3_sub3_c0);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 32, 8,
		fectrl3_sub3_c1);
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 32, 64, 8,
		fectrl3_sub3_c2);
}

static INLINE void
wlc_phy_write_regtbl_fc3(phy_info_t *pi, phy_info_acphy_t *pi_ac)
{
	switch (BF3_FEMCTRL_SUB(pi_ac)) {
		case 0:
			wlc_phy_write_regtbl_fc3_sub0(pi);
		break;
		case 1:
			wlc_phy_write_regtbl_fc3_sub1(pi);
		break;
		case 2:
			wlc_phy_write_regtbl_fc3_sub2(pi);
		break;
		case 3:
			wlc_phy_write_regtbl_fc3_sub3(pi);
		break;
	}
}

static void
wlc_phy_write_regtbl_fc4_sub0(phy_info_t *pi)
{
	uint16 fectrl_zeroval[] = {0};
	uint16 kk, fem_idx = 0;
	sparse_array_entry_t fectrl_fcbga_epa_elna[] =
		{{2, 264}, {3, 8}, {9, 32}, {18, 5}, {19, 4}, {25, 128}, {130, 64}, {192, 64}};

	for (kk = 0; kk < 256; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_fcbga_epa_elna) &&
			kk == fectrl_fcbga_epa_elna[fem_idx].idx) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				&(fectrl_fcbga_epa_elna[fem_idx].val));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc4_sub1(phy_info_t *pi)
{
	uint16 fectrl_zeroval[] = {0};
	uint16 kk, fem_idx = 0;
	sparse_array_entry_t fectrl_wlbga_epa_elna[] =
	{{2, 3}, {3, 1}, {9, 256}, {18, 20}, {19, 16}, {25, 8}, {66, 3}, {67, 1},
	{73, 256}, {82, 20}, {83, 16}, {89, 8}, {128, 3}, {129, 1}, {130, 3}, {131, 1},
	{132, 1}, {133, 1}, {134, 1}, {135, 1}, {136, 3}, {137, 1}, {138, 3}, {139, 1},
	{140, 1}, {141, 1}, {142, 1}, {143, 1}, {160, 3}, {161, 1}, {162, 3}, {163, 1},
	{164, 1}, {165, 1}, {166, 1}, {167, 1}, {168, 3}, {169, 1}, {170, 3}, {171, 1},
	{172, 1}, {173, 1}, {174, 1}, {175, 1}, {192, 128}, {193, 128}, {196, 128}, {197, 128},
	{200, 128}, {201, 128}, {204, 128}, {205, 128}, {224, 128}, {225, 128}, {228, 128},
	{229, 128}, {232, 128}, {233, 128}, {236, 128}, {237, 128} };
	for (kk = 0; kk < 256; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_wlbga_epa_elna) &&
			kk == fectrl_wlbga_epa_elna[fem_idx].idx) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				&(fectrl_wlbga_epa_elna[fem_idx].val));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc4_sub2(phy_info_t *pi)
{
	uint16 fectrl_zeroval[] = {0};
	uint16 kk, fem_idx = 0;
	sparse_array_entry_t fectrl_fchm_epa_elna[] =
	{{2, 280}, {3, 24}, {9, 48}, {18, 21}, {19, 20}, {25, 144}, {34, 776}, {35, 520},
	{41, 544}, {50, 517}, {51, 516}, {57, 640}, {66, 280}, {67, 24}, {73, 48}, {82, 21},
	{83, 20}, {89, 144}, {98, 776}, {99, 520}, {105, 544}, {114, 517}, {115, 516}, {121, 640},
	{128, 280}, {129, 24}, {130, 280}, {131, 24}, {132, 24}, {133, 24}, {134, 24}, {135, 24},
	{136, 280}, {137, 24}, {138, 280}, {139, 24}, {140, 24}, {141, 24}, {142, 24}, {143, 24},
	{160, 776}, {161, 520}, {162, 776}, {163, 520}, {164, 520}, {165, 520}, {166, 520},
	{167, 520}, {168, 776}, {169, 520}, {170, 776}, {171, 520}, {172, 520}, {173, 520},
	{174, 520}, {175, 520},	{192, 16}, {193, 16}, {196, 16}, {197, 16}, {200, 16}, {201, 16},
	{204, 16}, {205, 16}, {224, 512}, {225, 512}, {228, 512}, {229, 512}, {232, 512},
	{233, 512}, {236, 512}, {237, 512}};
	for (kk = 0; kk < 256; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_fchm_epa_elna) &&
			kk == fectrl_fchm_epa_elna[fem_idx].idx) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				&(fectrl_fchm_epa_elna[fem_idx].val));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc4_sub34(phy_info_t *pi)
{
	uint16 fectrl_zeroval[] = {0};
	uint16 kk, fem_idx = 0;

	sparse_array_entry_t fectrl_wlcsp_epa_elna[] =
		{{2, 34}, {3, 2}, {9, 1}, {18, 80}, {19, 16}, {25, 8}, {66, 34}, {67, 2},
		{73, 1}, {82, 80}, {83, 16}, {89, 8}, {128, 34}, {129, 2}, {130, 34}, {131, 2},
		{132, 2}, {133, 2}, {134, 2}, {135, 2}, {136, 34}, {137, 2}, {138, 34}, {139, 2},
		{140, 2}, {141, 2}, {142, 2}, {143, 2}, {160, 34}, {161, 2}, {162, 34}, {163, 2},
		{164, 2}, {165, 2}, {166, 2}, {167, 2}, {168, 34}, {169, 2}, {170, 34}, {171, 2},
		{172, 2}, {173, 2}, {174, 2}, {175, 2}, {192, 4}, {193, 4}, {196, 4}, {197, 4},
		{200, 4}, {201, 4}, {204, 4}, {205, 4}, {224, 4}, {225, 4}, {228, 4}, {229, 4},
		{232, 4}, {233, 4}, {236, 4}, {237, 4} };
	for (kk = 0; kk < 256; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_wlcsp_epa_elna) &&
			kk == fectrl_wlcsp_epa_elna[fem_idx].idx) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				&(fectrl_wlcsp_epa_elna[fem_idx].val));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc4_sub5(phy_info_t *pi)
{
	uint16 fectrl_zeroval[] = {0};
	uint16 kk, fem_idx = 0;

	sparse_array_entry_t fectrl_fp_dpdt_epa_elna[] =
		{{2, 280}, {3, 24}, {9, 48}, {18, 21}, {19, 20}, {25, 144}, {34, 776},
		{35, 520}, {41, 544}, {50, 517}, {51, 516}, {57, 640}, {130, 80},
		{192, 80}};

	for (kk = 0; kk < 256; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_fp_dpdt_epa_elna) &&
			kk == fectrl_fp_dpdt_epa_elna[fem_idx].idx) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				&(fectrl_fp_dpdt_epa_elna[fem_idx].val));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static INLINE void
wlc_phy_write_regtbl_fc4(phy_info_t *pi, phy_info_acphy_t *pi_ac)
{
	switch (BF3_FEMCTRL_SUB(pi_ac)) {
		case 0:
			wlc_phy_write_regtbl_fc4_sub0(pi);
		break;
		case 1:
			wlc_phy_write_regtbl_fc4_sub1(pi);
		break;
		case 2:
			wlc_phy_write_regtbl_fc4_sub2(pi);
		break;
		case 3:
		case 4:
			wlc_phy_write_regtbl_fc4_sub34(pi);
		break;
		case 5:
			wlc_phy_write_regtbl_fc4_sub5(pi);
		break;
	}
}

static void
wlc_phy_write_regtbl_fc10_sub0(phy_info_t *pi)
{
	uint16 fectrl_fcbga_epa_elna_idx[] = {2, 3, 9, 18, 19, 25, 66, 67, 73, 82, 83, 128,
		129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
		192, 193, 196, 197, 200, 201, 204, 205, 210, 211, 258, 259, 265, 274, 275, 281};
	uint16 fectrl_fcbga_epa_elna_val[] = {96, 32, 8, 6, 2, 1, 96, 32, 8, 6, 2, 96, 32,
		96, 32, 32, 32, 32, 32, 96, 32, 96, 32, 32, 32, 32, 32, 128, 128, 128, 128,
		128, 128, 128, 128, 134, 130, 5, 4, 8, 48, 32, 64 };
	uint16 fectrl_zeroval[] = {0};
	uint kk, fem_idx = 0;
	for (kk = 0; kk < 320; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_fcbga_epa_elna_idx) &&
			kk == fectrl_fcbga_epa_elna_idx[fem_idx]) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
			&(fectrl_fcbga_epa_elna_val[fem_idx]));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc10_sub1(phy_info_t *pi)
{
	uint16 fectrl_wlbga_epa_elna_idx[] = {2, 3, 9, 18, 19, 25, 66, 67, 73, 82, 83, 128,
		129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
		192, 193, 196, 197, 200, 201, 204, 205, 210, 211, 258, 259, 265, 274, 275, 281};
	uint16 fectrl_wlbga_epa_elna_val[] = {48, 32, 8, 6, 2, 1, 48, 32, 8, 6, 2, 48, 32,
	        48, 32, 32, 32, 32, 32, 48, 32, 48, 32, 32, 32, 32, 32, 128, 128, 128, 128,
		128, 128, 128, 128, 134, 130, 48, 32, 8, 6, 2, 1};
	uint16 fectrl_zeroval[] = {0};
	uint kk, fem_idx = 0;
	for (kk = 0; kk < 320; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_wlbga_epa_elna_idx) &&
			kk == fectrl_wlbga_epa_elna_idx[fem_idx]) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
			&(fectrl_wlbga_epa_elna_val[fem_idx]));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc10_sub2(phy_info_t *pi)
{
	uint16 fectrl_wlbga_ipa_ilna_idx[] = {2, 3, 9, 18, 19, 25, 66, 67, 73, 82, 83, 128,
		129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
		192, 193, 196, 197, 200, 201, 204, 205, 210, 211, 258, 259, 265, 274, 275, 281};
	uint16 fectrl_wlbga_ipa_ilna_val[] = {48, 32, 8, 6, 2, 1, 48, 32, 8, 6, 2, 48, 32,
	        48, 32, 32, 32, 32, 32, 48, 32, 48, 32, 32, 32, 32, 32, 128, 128, 128, 128,
		128, 128, 128, 128, 134, 130, 48, 32, 8, 6, 2, 1};
	uint16 fectrl_zeroval[] = {0};
	uint kk, fem_idx = 0;
	for (kk = 0; kk < 320; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_wlbga_ipa_ilna_idx) &&
			kk == fectrl_wlbga_ipa_ilna_idx[fem_idx]) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
			&(fectrl_wlbga_ipa_ilna_val[fem_idx]));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc10_sub3(phy_info_t *pi)
{
	uint16 fectrl_43556usb_epa_elna_idx[] = {2, 3, 9, 18, 19, 25, 66, 67, 73, 82, 83, 128,
		129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 192,
		193, 196, 197, 200, 201, 204, 205, 210, 211, 258, 259, 265, 274, 275, 281};
	uint16 fectrl_43556usb_epa_elna_val[] = {96, 32, 8, 6, 2, 1, 96, 32, 8, 6, 2, 96, 32, 96,
		32, 32, 32, 32, 32, 96, 32, 96, 32, 32, 32, 32, 32, 128, 128, 128, 128, 128, 128,
		128, 128, 134, 130, 5, 4, 8, 48, 32, 64};
	uint16 fectrl_zeroval[] = {0};
	uint kk, fem_idx = 0;
	for (kk = 0; kk < 320; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_43556usb_epa_elna_idx) &&
			kk == fectrl_43556usb_epa_elna_idx[fem_idx]) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
			&(fectrl_43556usb_epa_elna_val[fem_idx]));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static void
wlc_phy_write_regtbl_fc10_sub4(phy_info_t *pi)
{
	uint16 fectrl_fcbga_ipa_ilna_idx[] = {2, 3, 9, 18, 19, 25, 66, 67, 73, 82, 83, 128, 129,
		130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 192, 193,
		196, 197, 200, 201, 204, 205, 210, 211, 258, 259, 265, 265, 274, 275, 281, 281};
	uint16 fectrl_fcbga_ipa_ilna_val[] = {128, 32, 32, 8, 1, 1, 128, 32, 32, 8, 8, 128, 32,
		128, 32, 128, 32, 128, 32, 128, 32, 128, 32, 128, 32, 128, 32, 64, 64, 64, 64,
		64, 64, 64, 64, 72, 72, 4, 8, 8, 8, 64, 16, 16, 16};
	uint16 fectrl_zeroval[] = {0};
	uint kk, fem_idx = 0;
	for (kk = 0; kk < 320; kk++) {
		if (fem_idx < ARRAYSIZE(fectrl_fcbga_ipa_ilna_idx) &&
			kk == fectrl_fcbga_ipa_ilna_idx[fem_idx]) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
			&(fectrl_fcbga_ipa_ilna_val[fem_idx]));
			fem_idx++;
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, kk, 16,
				fectrl_zeroval);
		}
	}
}

static INLINE void
wlc_phy_write_regtbl_fc10(phy_info_t *pi, phy_info_acphy_t *pi_ac)
{
	switch (BF3_FEMCTRL_SUB(pi_ac)) {
	case 0:
	        wlc_phy_write_regtbl_fc10_sub0(pi);
	        break;
	case 1:
	        wlc_phy_write_regtbl_fc10_sub1(pi);
	        break;
	case 2:
	        wlc_phy_write_regtbl_fc10_sub2(pi);
	        break;
	case 3:
	        wlc_phy_write_regtbl_fc10_sub3(pi);
	        break;
	case 4:
	        wlc_phy_write_regtbl_fc10_sub4(pi);
	        break;
	}
}

static void
wlc_phy_tx_gm_gain_boost(phy_info_t *pi)
{
	uint8 core;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			MOD_RADIO_REGC(pi, TXGM_CFG1, core, gc_res, 0x1);
		}
	} else {
		if (BF_SROM11_GAINBOOSTA01(pi->u.pi_acphy)) {
			/* Boost A0/1 radio gain */
			FOREACH_CORE(pi, core) {
				MOD_RADIO_REGC(pi, TXMIX5G_CFG1, core, gainboost, 0x6);
				MOD_RADIO_REGC(pi, PGA5G_CFG1, core, gainboost, 0x6);
			}
		}
		if (RADIO2069REV(pi->pubpi->radiorev) <= 3) {
			/* Boost A2 radio gain */
			core = 2;
			MOD_RADIO_REGC(pi, TXMIX5G_CFG1, core, gainboost, 0x6);
			MOD_RADIO_REGC(pi, PGA5G_CFG1, core, gainboost, 0x6);
		}
	}
}

static void
wlc_phy_radio2069_4335C0_vco_opt(phy_info_t *pi, uint8 vco_mode)
{
	uint16 temp_reg;

	if (vco_mode == ACPHY_VCO_2P5V) {
		phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR27, 0xfff8);
		MOD_RADIO_REG(pi, RFP, PLL_VCO8, rfpll_vco_vctrl_buf_ical, 0x0);
		MOD_RADIO_REG(pi, RFP, PLL_VCO6, rfpll_vco_bypass_vctrl_buf, 0x1);
		MOD_RADIO_REG(pi, RFP, PLL_VCO3, rfpll_vco_cvar_extra, 0xa);
		MOD_RADIO_REG(pi, RFP, PLL_VCO2, rfpll_vco_cvar, 0xf);
		MOD_RADIO_REG(pi, RFP, PLL_VCO6, rfpll_vco_bias_mode, 0x0);
		MOD_RADIO_REG(pi, RFP, PLL_VCO6, rfpll_vco_ALC_ref_ctrl, 0x0);
		MOD_RADIO_REG(pi, RFP, PLL_CP4, rfpll_cp_kpd_scale, 0x21);
		MOD_RADIO_REG(pi, RFP, PLL_HVLDO2, ldo_2p5_lowquiescenten_VCO, 0x1);
		MOD_RADIO_REG(pi, RFP, PLL_HVLDO2, ldo_2p5_lowquiescenten_CP, 0x1);
		MOD_RADIO_REG(pi, RFP, GE16_PLL_HVLDO4, ldo_2p5_static_load_CP, 0x1);
		MOD_RADIO_REG(pi, RFP, GE16_PLL_HVLDO4, ldo_2p5_static_load_VCO, 0x1);

		temp_reg = phy_utils_read_radioreg(pi, RFP_2069_GE16_PLL_CFG3);
		temp_reg |= (0x3 << 10);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_CFG3, temp_reg);

		temp_reg = phy_utils_read_radioreg(pi, RFP_2069_GE16_TOP_SPARE3);
		temp_reg |= (0x3 << 11);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_TOP_SPARE3, temp_reg);

	} else if (vco_mode == ACPHY_VCO_1P35V) {
		phy_utils_write_radioreg(pi, RFP_2069_GE16_OVR27, 0xfff8);

		MOD_RADIO_REG(pi, RFP, PLL_VCO3, rfpll_vco_cvar_extra, 0xf);
		MOD_RADIO_REG(pi, RFP, PLL_VCO2, rfpll_vco_cvar, 0xf);
		MOD_RADIO_REG(pi, RFP, PLL_VCO6, rfpll_vco_bias_mode, 0x0);
		MOD_RADIO_REG(pi, RFP, PLL_VCO6, rfpll_vco_ALC_ref_ctrl, 0x0);
		MOD_RADIO_REG(pi, RFP, PLL_CP4, rfpll_cp_kpd_scale, 0x21);
		MOD_RADIO_REG(pi, RFP, PLL_XTALLDO1, ldo_1p2_xtalldo1p2_lowquiescenten, 0x1);
		MOD_RADIO_REG(pi, RFP, PLL_HVLDO2, ldo_2p5_lowquiescenten_VCO, 0x1);
		MOD_RADIO_REG(pi, RFP, PLL_HVLDO2, ldo_2p5_lowquiescenten_CP, 0x1);
		MOD_RADIO_REG(pi, RFP, GE16_PLL_HVLDO4, ldo_2p5_static_load_CP, 0x1);
		MOD_RADIO_REG(pi, RFP, GE16_PLL_HVLDO4, ldo_2p5_static_load_VCO, 0x1);

		temp_reg = phy_utils_read_radioreg(pi, RFP_2069_GE16_PLL_CFG4);
		temp_reg |= (0x3 << 12);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_CFG4, temp_reg);

		MOD_RADIO_REG(pi, RFP, PLL_VCO2, rfpll_vco_USE_2p5V, 0x0);

		temp_reg = phy_utils_read_radioreg(pi, RFP_2069_GE16_PLL_CFG3);
		temp_reg |= (0x3 << 10);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_CFG3, temp_reg);

		temp_reg = phy_utils_read_radioreg(pi, RFP_2069_GE16_TOP_SPARE3);
		temp_reg |= (0x3 << 11);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_TOP_SPARE3, temp_reg);

		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_LF2, 0x5555);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_LF3, 0x5555);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_LF4, 0xe);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_LF5, 0xe);
		phy_utils_write_radioreg(pi, RFP_2069_GE16_PLL_LF7, 0x1085);
	}

}

#define WLC_TINY_GI_MULT_P12		4096U
#define WLC_TINY_GI_MULT_TWEAK_P12	4096U
#define WLC_TINY_GI_MULT		WLC_TINY_GI_MULT_P12

static void
wlc_tiny_sigdel_slow_tune(phy_info_t *pi, int g_mult_raw_p12,
	tiny_adc_tuning_array_t *gvalues, uint8 bw)
{
	int g_mult_p12;
	int ri = 0;
	int r21;
	int r32;
	int r43;
	int rff1;
	int rff2;
	int rff3;
	int rff4;
	int r12v;
	int r34v;
	int r11v;
	int g21;
	int g32;
	int g43;
	int r12;
	int r34;
	int g11;
	int temp;
	int gff3_val = 0;
	uint8 shift_val = 0;

	/* RC cals the slow ADC IN 40MHz channels or 20MHz bandwidth, based on g_mult */
	/* input signal scaling, changes ADC gain, 4096 <=> 1.0 for g_mult and gi_mult */
	/* Function is 32 bit (signed) integer arithmetic and a/b division rounding  */
	/* is performed in integers from: (a-1)/b+1 */

	/* ERR! 20691_rc_cal "adc" returns the RC value, so correction is 1/rccal! */
	/* so invert it */
	/* This is a nice way of inverting the number... jnh */
	/* inverse of gmult precomputed to minimise division operations for speed */
	/* 4.12 fixed point so scale reciprocal by 2^24 */
	g_mult_p12 = g_mult_raw_p12 > 0 ? g_mult_raw_p12 : 1;
	/* tweak to g_mult */
	g_mult_p12 = (WLC_TINY_GI_MULT_TWEAK_P12 * g_mult_p12) >> 12;

	/* to avoid divide by zeros and negative values */
	if (g_mult_p12 <= 0)
		g_mult_p12 = 1;

	if (bw == 20) {
	    shift_val = 1;
		ri = 10176;
		gff3_val = 32000;
	} else if (bw == 40) {
		shift_val = 0;
		ri = 10670;
		gff3_val = 12000;
	}

	/* RC cal in slow ADC is mostly of the form Runit/(Rval/(g_mult/2**12)-Roff). */
	/* For integer manipulation do Runit/({Rval*2**12}/gmult-Roff).  */
	/* where Rval*2**12 are res design values in matlab script {x kint234 , kr12, kr34} */
	/* but right shifted a number of times */
	/* All but r11 and rff4 resistances are x2 for 20MHz. */

	/* Rvals from matlab already scaled by kint234, kr12 */
	/* or kr34 (due to amplifier finite GBW) */
	/* x2 for half the BW and half the sampling frequency. */
	ri = ri << shift_val;
	r21 = 8323 << shift_val;
	r32 = 6390 << shift_val;
	r43 = 6827 << shift_val;
	rff1 = 19768 << shift_val;
	rff2 = 16916 << shift_val;
	rff3 = 29113  << shift_val;
	/* rff4 does not double with 20MHz channels, 10MHz BW. */
	rff4 = 100000;
	r12v = 83205 << shift_val;
	r34v = 243530 << shift_val;
	/* rff4 does not double with 20MHz channels, 10MHz BW. */
	r11v = 8000;

	/* saturate correctly when you get negative numbers and round divisions */
	/* subject to gmult twice so scale gmult back to 12b so it only divides with 12b+ r21 */
	g21 = (g_mult_p12 * g_mult_p12) >> 12;
	if (g21 <= 0)
		g21 = 1;
	g21 = ((r21 << 12) - 1) / g21 + 1;
	g21 = (512000 - 1) / g21 + 1;
	g21 = wlc_tiny_sigdel_wrap(g21, 127);
	g32 = (256000 - 1) / (((r32 << 12) - 1) / g_mult_p12 + 1) + 1;
	g32 = wlc_tiny_sigdel_wrap(g32, 127);
	g43 = (256000 - 1) / (((r43 << 12) - 1) / g_mult_p12 + 1) + 1;
	g43  = wlc_tiny_sigdel_wrap(g43, 127);

	/* gff1234 subject to gmult and gimult; step operations so range */
	/* is not exceeded and scale is correct */
	/* gff1234 will overflow if $g_mult_p12*$gi_mult < 1023*2, eq. to 0.25, */
	/* assuming rff1234 {<<2} < 131072 */

	/* gi */
	temp = (((ri << 12) - 1) / WLC_TINY_GI_MULT) - 4000 + 1;
	ASSERT(temp > 0);	/* should be a positive value */
	temp = (256000 - 1) / temp + 1;
	temp = wlc_tiny_sigdel_wrap(temp, 127);
	gvalues->gi = (uint16) temp;

	/* gff1 */
	temp = ((((((rff1 << 12) - 1) / g_mult_p12 + 1) << 12) - 1) / WLC_TINY_GI_MULT) - 8000 + 1;
	if (temp <= 0)
		temp = 1;
	temp = (256000 - 1) / temp + 1;
	temp = wlc_tiny_sigdel_wrap(temp, 127);
	gvalues->gff1 = (uint16) temp;

	/* gff2 */
	temp = ((((((rff2 << 12) - 1) / g_mult_p12 + 1) << 12) - 1) / WLC_TINY_GI_MULT) - 4000 + 1;
	if (temp <= 0)
		temp = 1;
	temp = (256000 - 1) / temp + 1;
	temp = wlc_tiny_sigdel_wrap(temp, 127);
	gvalues->gff2 = (uint16) temp;

	/* gff3 */
	temp = ((((((rff3 << 12) - 1) / g_mult_p12 + 1) << 12) - 1) / WLC_TINY_GI_MULT) - gff3_val
	    + 1;

	if (temp <= 0)
		temp = 1;
	temp = (256000 - 1) / temp + 1;
	temp = wlc_tiny_sigdel_wrap(temp, 127);
	gvalues->gff3 = (uint16) temp;

	/* gff4 */
	temp = (((rff4 << 12) - 1) / WLC_TINY_GI_MULT) - 72000 + 1;	/* subject to gimult only */
	ASSERT(temp > 0);	/* should be a positive value */
	temp = (256000 - 1) / temp + 1;
	temp  = wlc_tiny_sigdel_wrap(temp, 255);
	gvalues->gff4 = (uint16) temp;

	/* stays constant to RC shifts, g21 shifts twice for it. */
	r12 = (r12v - 1) / 4000 + 1;
	r12 = wlc_tiny_sigdel_wrap(r12, 127);

	if (bw == 20)
		r34 = ((((r34v << 12) - 1) / g_mult_p12 +1) - 128000 - 1) / 4000 + 1;
	else
		r34 = ((r34v << 12) - 1) / g_mult_p12 / 4000 + 1;

	if (r34 <= 0)
		r34 = 1;
	r34 = wlc_tiny_sigdel_wrap(r34, 127);
	g11 = (((r11v << 12) - 1) / g_mult_p12 +1) - 2000;
	if (g11 <= 0)
		g11 = 1;
	g11 = (128000 - 1) / g11 + 1;
	g11 = wlc_tiny_sigdel_wrap(g11, 127);

	gvalues->g21 = (uint16) g21;
	gvalues->g32 = (uint16) g32;
	gvalues->g43 = (uint16) g43;
	gvalues->r12 = (uint16) r12;
	gvalues->r34 = (uint16) r34;
	gvalues->g11 = (uint16) g11;
	PHY_TRACE(("gi   = %i\n", gvalues->gi));
	PHY_TRACE(("g21  = %i\n", gvalues->g21));
	PHY_TRACE(("g32  = %i\n", gvalues->g32));
	PHY_TRACE(("g43  = %i\n", gvalues->g43));
	PHY_TRACE(("r12  = %i\n", gvalues->r12));
	PHY_TRACE(("r34  = %i\n", gvalues->r34));
	PHY_TRACE(("gff1 = %i\n", gvalues->gff1));
	PHY_TRACE(("gff2 = %i\n", gvalues->gff2));
	PHY_TRACE(("gff3 = %i\n", gvalues->gff3));
	PHY_TRACE(("gff4 = %i\n", gvalues->gff4));
	PHY_TRACE(("g11  = %i\n", gvalues->g11));
}

static void
wlc_tiny_adc_setup_slow(phy_info_t *pi, tiny_adc_tuning_array_t *gvalues, uint8 bw, uint8 core)
{
	const chan_info_radio20693_altclkplan_t *altclkpln = altclkpln_radio20693;
	int row = wlc_phy_radio20693_altclkpln_get_chan_row(pi);
	uint8 adcclkdiv = 0x1;
	uint8 sipodiv = 0x1;

	ASSERT(TINY_RADIO(pi));

	if (row >= 0) {
		adcclkdiv = altclkpln[row].adcclkdiv;
		sipodiv = altclkpln[row].sipodiv;
	}
	/* SETS UP THE slow ADC IN 20MHz channels or 10MHz bandwidth */
	/* Function should be 32 bit (signed) arithmetic */
	if ((RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) && (bw == 20)) {
		MOD_RADIO_REG_20693(pi, SPARE_CFG2, core, adc_clk_slow_div, adcclkdiv);
	} else if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID)) {
		/* EXPLICITELY ENABLE/DISABLE ADCs and INTERNAL CLKs? */
		/* Only changes between fast/slow ADC, not 20/40MHz */
		MOD_RADIO_REG_20691(pi, ADC_OVR1, core, ovr_adc_fast_pu, 0x1);
		MOD_RADIO_REG_20691(pi, ADC_CFG1, core, adc_fast_pu, 0x0);
		MOD_RADIO_REG_20691(pi, ADC_OVR1, core, ovr_adc_slow_pu, 0x0);
		MOD_RADIO_REG_20691(pi, ADC_OVR1, core, ovr_adc_clk_fast_pu, 0x1);
		MOD_RADIO_REG_20691(pi, ADC_CFG15, core, adc_clk_fast_pu, 0x0);
		MOD_RADIO_REG_20691(pi, ADC_OVR1, core, ovr_adc_clk_slow_pu, 0x0);
	}
	/* Setup internal dividers and sipo for 1G2Hz mode */
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_drive_strength, 0x4);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_div8, sipodiv);
	/* set adc_clk_slow_div3 to 0x0 in 20MHz mode, 0x1 in 40MHz mode */
	if (bw == 20)
		MOD_RADIO_REG_TINY(pi, ADC_CFG15, core, adc_clk_slow_div3, 0x0);
	else if (bw == 40)
		MOD_RADIO_REG_TINY(pi, ADC_CFG15, core, adc_clk_slow_div3, 0x1);
	if ((RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) && (bw == 40))
		MOD_RADIO_REG_20693(pi, SPARE_CFG2, core, adc_clk_slow_div, adcclkdiv);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_sipo_div8, sipodiv);
	MOD_RADIO_REG_TINY(pi, ADC_CFG10, core, adc_sipo_sel_fast, 0x0);

	MOD_RADIO_REG_TINY(pi, ADC_CFG18, core, adc_od_pu, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_adc_od_pu, 0x1);

	/* Setup biases */
	/* Slow adc halves opamp current from 20MHz to 40MHz channels */
	/* Opamp1 is 26u/0, 4u/2 other 3 are 26u/0, 4u/4,	*/
	/* so opamp1 (40M, 20M) = (0x20, 0x10), opamp234 = (0x10, 0x8) */
	MOD_RADIO_REG_TINY(pi, ADC_CFG6, core, adc_biasadj_opamp1, (0x10 * (bw/20)));
	MOD_RADIO_REG_TINY(pi, ADC_CFG6, core, adc_biasadj_opamp2, (0x8 * (bw/20)));
	MOD_RADIO_REG_TINY(pi, ADC_CFG7, core, adc_biasadj_opamp3, (0x8 * (bw/20)));
	MOD_RADIO_REG_TINY(pi, ADC_CFG7, core, adc_biasadj_opamp4, (0x8 * (bw/20)));
	MOD_RADIO_REG_TINY(pi, ADC_CFG8, core, adc_ff_mult_opamp, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG9, core, adc_cmref_control, 0x40);
	MOD_RADIO_REG_TINY(pi, ADC_CFG9, core, adc_cmref4_control, 0x40);

	/* Setup transconductances. These are tuned with gmult(RC) and/or gimult(input gain) */
	/* rnm */
	MOD_RADIO_REG_TINY(pi, ADC_CFG2, core, adc_gi, gvalues->gi);
	MOD_RADIO_REG_TINY(pi, ADC_CFG3, core, adc_g21, gvalues->g21);
	MOD_RADIO_REG_TINY(pi, ADC_CFG3, core, adc_g32, gvalues->g32);
	MOD_RADIO_REG_TINY(pi, ADC_CFG4, core, adc_g43, gvalues->g43);
	/* rff */
	MOD_RADIO_REG_TINY(pi, ADC_CFG16, core, adc_gff1, gvalues->gff1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG16, core, adc_gff2, gvalues->gff2);
	MOD_RADIO_REG_TINY(pi, ADC_CFG17, core, adc_gff3, gvalues->gff3);
	MOD_RADIO_REG_TINY(pi, ADC_CFG17, core, adc_gff4, gvalues->gff4);
	/* resonator and r11 */
	MOD_RADIO_REG_TINY(pi, ADC_CFG5, core, adc_r12, gvalues->r12);
	MOD_RADIO_REG_TINY(pi, ADC_CFG8, core, adc_r34, gvalues->r34);
	MOD_RADIO_REG_TINY(pi, ADC_CFG4, core, adc_g54, gvalues->g11);

	/* Setup feedback DAC and tweak delay compensation */
	if (bw == 20) {
		/* In slow 40MHz ADC rt is 0x0, in 20MHz ADC rt is 0x2 */
	    MOD_RADIO_REG_TINY(pi, ADC_CFG19, core, adc_rt, 0x2);
		/* In slow 40MHz ADC slow_dacs is 0x2 in 20MHz ADC rt is 0x1 */
	    MOD_RADIO_REG_TINY(pi, ADC_CFG19, core, adc_slow_dacs, 0x1);
	} else if (bw == 40) {
		/* In slow 40MHz ADC rt is 0x0, in 20MHz ADC rt is 0x2 */
		MOD_RADIO_REG_TINY(pi, ADC_CFG19, core, adc_rt, 0x0);
		/* In slow 40MHz ADC slow_dacs is 0x2 in 20MHz ADC rt is 0x1 */
		MOD_RADIO_REG_TINY(pi, ADC_CFG19, core, adc_slow_dacs, 0x2);
	}
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_reset_adc, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_adcs_reset, 0x1);
	MOD_RADIO_REG_TINY(pi, ADC_CFG1, core, adc_adcs_reset, 0x0);
	MOD_RADIO_REG_TINY(pi, ADC_OVR1, core, ovr_reset_adc, 0x0);
}

/*
 *  The TIA has 13 distinct gain steps.
 *  Each of the tia_* scalers are packed with the
 *  tia settings for each gain step.
 *  Mapping for each gain step is:
 *  pwrup_amp2, amp2_bypass, R1, R2, R3, R4, C1, C2, enable_st1
 */
static void
wlc_tiny_tia_config(phy_info_t *pi, uint8 core)
{
/*
 *  The TIA has 13 distinct gain steps.
 *  Each of the tia_* scalers are packed with the
 *  tia settings for each gain step.
 *  Mapping for each gain step is:
 *  pwrup_amp2, amp2_bypass, R1, R2, R3, R4, C1, C2, enable_st1
 */
	const uint8  *p8;
	const uint16 *p16;
	uint16 lut;

	ASSERT(TINY_RADIO(pi));

	if (CHSPEC_IS80(pi->radio_chanspec)) {
		STATIC_ASSERT(ARRAYSIZE(tiaRC_tiny_8b_80) + ARRAYSIZE(tiaRC_tiny_16b_80) == 82);
		p8 = tiaRC_tiny_8b_80;
		p16 = tiaRC_tiny_16b_80;
	} else if (CHSPEC_IS40(pi->radio_chanspec)) {
		STATIC_ASSERT(ARRAYSIZE(tiaRC_tiny_8b_40) + ARRAYSIZE(tiaRC_tiny_16b_20) == 82);
		p8 = tiaRC_tiny_8b_40;
		p16 = tiaRC_tiny_16b_40;
	} else {
		STATIC_ASSERT(ARRAYSIZE(tiaRC_tiny_8b_20) + ARRAYSIZE(tiaRC_tiny_16b_20) == 82);
		p8 = tiaRC_tiny_8b_20;
		p16 = tiaRC_tiny_16b_20;
	}

	lut = RADIO_REG(pi, TIA_LUT_0, core);

	/* the assumption is that all the TIA LUT registers are in sequence */
	ASSERT(RADIO_REG(pi, TIA_LUT_82, core) - lut == 81);

	do {
		phy_utils_write_radioreg(pi, lut++, *p8++);
	} while (lut <= RADIO_REG(pi, TIA_LUT_51, core));

	do {
		phy_utils_write_radioreg(pi, lut++, *p16++);
	} while (lut <= RADIO_REG(pi, TIA_LUT_82, core));
}

static void
wlc_phy_write_rx_farrow_pre_tiny(phy_info_t *pi, chan_info_rx_farrow *rx_farrow,
	chanspec_t chanspec)
{
	uint16 deltaphase_lo, deltaphase_hi;
	uint16 drift_period, farrow_ctrl;

#ifdef ACPHY_1X1_ONLY
	uint8 channel = CHSPEC_CHANNEL(chanspec);
	uint32 deltaphase;

	if (channel <= 14) {
		if (CHSPEC_IS20(chanspec))
			drift_period = 5120; /* 40x32x4 */
		else if (CHSPEC_IS40(chanspec))
			drift_period = 5120; /* 40x32x4 */
		else
			drift_period = 1280; /* 160x4x2 */
	} else {
		if (CHSPEC_IS20(chanspec))
			drift_period = 3840; /* 40x24x4 */
		else if (CHSPEC_IS40(chanspec))
			drift_period = 3840; /* 40x24x4 */
		else
			drift_period = 2880; /* 160x9x2 */
	}

	if (CHSPEC_IS80(chanspec)) {
		deltaphase = rx_farrow->deltaphase_80;
		farrow_ctrl = rx_farrow->farrow_ctrl_80;
	} else {
		deltaphase = rx_farrow->deltaphase_20_40;
		farrow_ctrl = rx_farrow->farrow_ctrl_20_40;
	}
	if (ACMAJORREV_1(pi->pubpi->phy_rev) && !(ACMINORREV_0(pi) || ACMINORREV_1(pi))) {
		farrow_ctrl = (farrow_ctrl &
			~ACPHY_rxFarrowCtrl_rx_farrow_outShift_MASK(pi->pubpi->phy_rev));
	}
	deltaphase_lo = deltaphase & 0xffff;
	deltaphase_hi = (deltaphase >> 16) & 0xff;


#else  /* ACPHY_1X1_ONLY */
	UNUSED_PARAMETER(chanspec);

	/* Setup the Rx Farrow */
	deltaphase_lo = rx_farrow->deltaphase_lo;
	deltaphase_hi = rx_farrow->deltaphase_hi;
	drift_period = rx_farrow->drift_period;
	farrow_ctrl = rx_farrow->farrow_ctrl;


#endif  /* ACPHY_1X1_ONLY */
	/* Setup the Rx Farrow */
	WRITE_PHYREG(pi, rxFarrowDeltaPhase_lo, deltaphase_lo);
	WRITE_PHYREG(pi, rxFarrowDeltaPhase_hi, deltaphase_hi);
	WRITE_PHYREG(pi, rxFarrowDriftPeriod, drift_period);
	WRITE_PHYREG(pi, rxFarrowCtrl, farrow_ctrl);

	/* Use the same settings for the loopback Farrow */
	WRITE_PHYREG(pi, lbFarrowDeltaPhase_lo, deltaphase_lo);
	WRITE_PHYREG(pi, lbFarrowDeltaPhase_hi, deltaphase_hi);
	WRITE_PHYREG(pi, lbFarrowDriftPeriod, drift_period);
	WRITE_PHYREG(pi, lbFarrowCtrl, farrow_ctrl);
}

static uint16 wlc_phy_femctrlout_get_val(uint32 val_ext, uint32 val, uint32 MASK)
{
	uint32 value = 0;
	value =  ((val_ext>>(MASK)) & 0x3)<<8 | ((val>>(MASK)) & 0xff);
	return (uint16) value;
}

static int
wlc_tiny_sigdel_wrap(int prod, int max_val)
{
	/* to make sure you hit the maximum number of bits in word allocated  */
	return (prod > max_val) ? max_val : prod;
}

/* ********************************************* */
/*				External Definitions					*/
/* ********************************************* */

void
wlc_phy_write_rx_farrow_tiny(phy_info_t *pi, chanspec_t chanspec)
{
	uint8 ch, num, den, bw, M, vco_div, core;
	uint32 fcw, tmp_low = 0, tmp_high = 0;
	uint32 fc;
	bool vco_12GHz = pi->u.pi_acphy->vco_12GHz;
	bw = CHSPEC_IS20(chanspec) ? PHYBW_20: CHSPEC_IS40(chanspec) ? PHYBW_40 : PHYBW_80;

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		if (!vco_12GHz) {
			num = 3;
			den = 2;
		} else {
			num = 4;
			den = 1;
		}
	} else {
		if (!vco_12GHz) {
			num = 2;
			den = 3;
		} else {
			num = 2;
			den = 1;
		}
	}

	if (vco_12GHz) {
		if ((pi->u.pi_acphy->fast_adc_en) ||
			(ACMAJORREV_4(pi->pubpi->phy_rev) && CHSPEC_IS8080(chanspec))) {
			M = SIPO_DIV_FAST * PHYBW_80 / bw;
			vco_div = AFE_DIV_FAST * ADC_DIV_FAST;
		} else {
			M = SIPO_DIV_SLOW;
			vco_div = AFE_DIV_BW(bw) * ADC_DIV_SLOW;
		}
	} else {
		if (CHSPEC_IS20(chanspec)) {
			M = SIPO_DIV_SLOW;
			vco_div = 6;
		} else if (CHSPEC_IS40(chanspec)) {
			M = SIPO_DIV_SLOW;
			vco_div = 3;
		} else {
			M = SIPO_DIV_FAST;
			vco_div = 1;
		}
	}

	if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
		const uint8 afeclkdiv_arr[] = {2, 16, 4, 8, 3, 24, 6, 12};
		const uint8 adcclkdiv_arr[] = {1, 2, 3, 6};
		const uint8 sipodiv_arr[] = {12, 8};
		const chan_info_radio20693_altclkplan_t *altclkpln = altclkpln_radio20693;
		int row = wlc_phy_radio20693_altclkpln_get_chan_row(pi);
		if ((row >= 0) && (pi->u.pi_acphy->fast_adc_en == 0)) {
			num = CHSPEC_IS2G(pi->radio_chanspec) ? 4 : 2;
			M = sipodiv_arr[altclkpln[row].sipodiv];
			den = 1;
			vco_div = afeclkdiv_arr[altclkpln[row].afeclkdiv] *
				adcclkdiv_arr[altclkpln[row].adcclkdiv];
		}
	}
	/* bits_in_mu = 24 */
	/*
	fcw = (num * phy_utils_channel2freq(ch) * (((uint32)(1<<31))/
		(den * vco_div * 2 * M * bw)))>> 7;
	*/
	if (CHSPEC_IS8080(chanspec)) {
		FOREACH_CORE(pi, core) {
			if (core == 0) {
				ch = wf_chspec_primary80_channel(chanspec);
				fc = wf_channel2mhz(ch, WF_CHAN_FACTOR_5_G);

				bcm_uint64_multiple_add(&tmp_high, &tmp_low, fc * num, 1 << 24, 0);
				bcm_uint64_divide(&fcw, tmp_high, tmp_low,
					(uint32) (den * vco_div * 2 * M * bw));

				PHY_INFORM(("%s: fcw 0x%0x ch %d freq %d vco_div %d bw %d\n",
					__FUNCTION__, fcw, ch, phy_utils_channel2freq(ch),
					vco_div, bw));

				MOD_PHYREG(pi, RxSdFeConfig20, fcw_value_lo, fcw & 0xffff);
				MOD_PHYREG(pi, RxSdFeConfig30, fcw_value_hi,
					(fcw >> 16) & 0xffff);
				MOD_PHYREG(pi, RxSdFeConfig30, fast_ADC_en,
					(pi->u.pi_acphy->fast_adc_en & 0x1));
			} else if (core == 1) {
				ch = wf_chspec_secondary80_channel(chanspec);
				fc = wf_channel2mhz(ch, WF_CHAN_FACTOR_5_G);

				bcm_uint64_multiple_add(&tmp_high, &tmp_low, fc * num, 1 << 24, 0);
				bcm_uint64_divide(&fcw, tmp_high, tmp_low,
					(uint32) (den * vco_div * 2 * M * bw));

				PHY_INFORM(("%s: fcw 0x%0x ch %d freq %d vco_div %d bw %d\n",
					__FUNCTION__, fcw, ch, phy_utils_channel2freq(ch),
					vco_div, bw));

				MOD_PHYREG(pi, RxSdFeConfig21, fcw_value_lo, fcw & 0xffff);
				MOD_PHYREG(pi, RxSdFeConfig31, fcw_value_hi,
					(fcw >> 16) & 0xffff);
				MOD_PHYREG(pi, RxSdFeConfig31, fast_ADC_en,
					(pi->u.pi_acphy->fast_adc_en & 0x1));
			}
		}
	} else {
		ch = CHSPEC_CHANNEL(chanspec);
		fc = wf_channel2mhz(ch, CHSPEC_IS2G(pi->radio_chanspec) ? WF_CHAN_FACTOR_2_4_G
			: WF_CHAN_FACTOR_5_G);

		bcm_uint64_multiple_add(&tmp_high, &tmp_low, fc * num, 1 << 24, 0);
		bcm_uint64_divide(&fcw, tmp_high, tmp_low, (uint32) (den * vco_div * 2 * M * bw));

		PHY_INFORM(("%s: fcw 0x%0x ch %d freq %d vco_div %d bw %d\n",
			__FUNCTION__, fcw, ch, phy_utils_channel2freq(ch), vco_div, bw));

		MOD_PHYREG(pi, RxSdFeConfig2, fcw_value_lo, fcw & 0xffff);
		MOD_PHYREG(pi, RxSdFeConfig3, fcw_value_hi, (fcw >> 16) & 0xffff);
		MOD_PHYREG(pi, RxSdFeConfig3, fast_ADC_en, (pi->u.pi_acphy->fast_adc_en & 0x1));
	}
}

#define ACPHY_MASK_TDM	                0x100
#define ACPHY_MASK_OVR_EN	        0x200
#define ACPHY_MASK_OVR_ANT              0x400
#define ACPHY_MAP_BT_TX	                0xc00
#define ACPHY_MASK_BT_TX	        0xc90
#define ACPHY_MAP_BT_RX	                0x800
#define ACPHY_MASK_BT_RX                0x810
#define ACPHY_MAP_WLAN_RX	        0x002
#define ACPHY_MASK_WLAN_RX	        0xbcf
#define ACPHY_MAP_WLAN_LOW_GAIN_RX	0x003
#define ACPHY_MASK_WLAN_LOW_GAIN_RX	0xbcf
#define ACPHY_MAP_WLAN_TX	        0x009
#define ACPHY_MASK_WLAN_TX	        0xbcf
#define ACPHY_MASK_ANT                  0x020
#define ACPHY_MASK_BAND                 0x010
void wlc_phy_write_regtbl_fc_from_nvram(phy_info_t *pi)
{
	uint8 band = 0, ovr_en = 0, ovr_ant = 0, elna = 0, ant = 0;
	uint8 inv_btcx_prisel = 0, muxErcxPriSel = 0;
	/* uint8 tdm, ercx_prisel; */
	uint8 ABAND = 0, BT_priority = 0, BT_TX = 0, MUX_CTRL = 0;
	uint8 WL_ANT_SEL = 0, BT_AoA = 0, WL_ePA_PU = 0;
	uint8 WL_eLNA_Gain = 0, BT_eLNA_Gain = 0, WL_eLNA_PU = 0;
	uint8 BT_RX = 0, WL_TRSW = 0, BT_rx_attn = 0;
	uint16 index = 0, indx = 0, femctrlout = 0;
	uint32 *swctrlmap = NULL, *swctrlmapext = NULL, decoded_address = 0;
	uint8 core = 0;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* BT_prisel and ErcxPriSel polarity info */
	/* No Need of forcing inv_btcx_prisel below */
	/* should be already taken care of in reg_on_init function */
	/*
	   MOD_PHYREG(pi, BT_SwControl, inv_btcx_prisel, 0x1);
	 */
	inv_btcx_prisel = READ_PHYREGFLD(pi, BT_SwControl, inv_btcx_prisel);
	muxErcxPriSel = READ_PHYREGFLD(pi, FemCtrl, muxErcxPriSel);

	for (band = 0; band <= 1; band++) {
		swctrlmap = ((band == 0) ? &(pi_ac->sromi->nvram_femctrl.swctrlmap_2g[0]) :
			&(pi_ac->sromi->nvram_femctrl.swctrlmap_5g[0]));
		swctrlmapext = ((band == 0) ? &(pi_ac->sromi->nvram_femctrl.swctrlmapext_2g[0]) :
			&(pi_ac->sromi->nvram_femctrl.swctrlmapext_5g[0]));
		if (band) {
			pi_ac->sromi->femctrlmask_2g =
			        ((pi_ac->sromi->nvram_femctrl.swctrlmapext_2g[4] & 0x3)<<8
			         | (pi_ac->sromi->nvram_femctrl.swctrlmap_2g[4] & 0xff));
		} else {
			pi_ac->sromi->femctrlmask_5g =
				((pi_ac->sromi->nvram_femctrl.swctrlmapext_5g[4] & 0x3)<<8
			         | (pi_ac->sromi->nvram_femctrl.swctrlmap_5g[4] & 0xff));
		}

		elna =  ((band == 0) ? BF_ELNA_2G(pi_ac) : BF_ELNA_5G(pi_ac));
		/* tdm = ((swctrlmap[4] & ACPHY_MASK_TDM) == ACPHY_MASK_TDM); */
		ovr_en = ((swctrlmap[4] & ACPHY_MASK_OVR_EN) == ACPHY_MASK_OVR_EN);
		ovr_ant = ((swctrlmap[4] & ACPHY_MASK_OVR_ANT) == ACPHY_MASK_OVR_ANT);

		FOREACH_CORE(pi, core) {
			/* Core 0 and 1 respectively have 8 and 6 inputs to the FEM ctrl LUT */
			for (index = 0; index < ((core == 0) ? 128 : 32); index++) {

				/* split the index into appropriate controls */
				femctrlout = 0;
				ABAND = band;

				/* generate a femctrl index which includes band bit as well */
				indx = (index & 0xf) | (ABAND<<4) | ((index>>4) <<5);

				/* BT_priority and BT_TX */
				BT_priority =  (indx & (1 << 7)) >> 7;
				BT_TX = (indx & (1 << 6)) >> 6;

				if (inv_btcx_prisel == 1) {
					MUX_CTRL  =  BT_priority & (ABAND == 0);
				} else {
					MUX_CTRL  =  (BT_priority == 0) & (ABAND == 0);
				}

				/* BT_AoA and ANT_SEL */
				if ((MUX_CTRL == 0) || (core == 1)) {
					WL_ANT_SEL = (indx & (1 << 5)) >> 5;
					BT_AoA  =   0;
				} else {
					WL_ANT_SEL =  0;
					BT_AoA     =  (indx & (1 << 5)) >> 5;
				}

				/* ercx_prisel and WL_ePA_PU */
				if ((muxErcxPriSel == 0) || (core == 1)) {
					WL_ePA_PU = (indx & (1 << 3)) >> 3;
					/* ercx_prisel = 0; */
				} else {
					WL_ePA_PU = 0;
					/* ercx_prisel = (indx & (1 << 3)) >> 3; */
				}

				/* eLNA_gain */
				if ((MUX_CTRL == 0) || (core == 1)) {
					WL_eLNA_Gain = (indx & (1 << 2)) >> 2;
					BT_eLNA_Gain =   0;
				} else {
					WL_eLNA_Gain = 0;
					BT_eLNA_Gain = (indx & (1 << 2)) >> 2;
				}

				/* RX_PU */
				if ((MUX_CTRL == 0) || (core == 1)) {
					WL_eLNA_PU  = (indx & (1 << 1)) >> 1;
					BT_RX   =  0;
				} else {
					WL_eLNA_PU = 0;
					BT_RX  = (indx & (1 << 1)) >> 1;
				}

				/* TR switch related */
				if ((MUX_CTRL == 0) || (core == 1)) {
					WL_TRSW = (indx & (1 << 0)) >> 0;
					BT_rx_attn   =  0;
				} else {
					WL_TRSW = 0;
					BT_rx_attn = (indx & (1 << 0)) >> 0;
				}

				/* now classify which case this address corresponds to */
				decoded_address = WL_TRSW+2*WL_eLNA_PU+4*WL_eLNA_Gain+8*WL_ePA_PU
					+16*ABAND+32*WL_ANT_SEL+64*BT_rx_attn+128*BT_RX
					+256*BT_eLNA_Gain+512*BT_AoA+1024*BT_TX+2048*BT_priority;

				/* depending on the case decoded and if elna and ant overrdide */
				/* read appropriate byte from swctrlmap */
				ant =  ((ovr_en == 1) ? ovr_ant :
					((decoded_address & ACPHY_MASK_ANT) == ACPHY_MASK_ANT));

				/* No 2o3 antenna selection for 2x2 for now */
				if (pi->pubpi->phy_corenum == 2)
					ant = core;

				if ((decoded_address & ACPHY_MASK_WLAN_TX) ==
						ACPHY_MAP_WLAN_TX) {
					uint32 swctrlwordext, swctrlword;
					wlc_phy_wltx_word_get(pi, band, swctrlmap[0],
						swctrlmapext[0], &swctrlword, &swctrlwordext);
					femctrlout = wlc_phy_femctrlout_get_val(
						swctrlwordext, swctrlword, 8*ant);
				} else if ((decoded_address & ACPHY_MASK_WLAN_RX) ==
						ACPHY_MAP_WLAN_RX) {
					femctrlout = wlc_phy_femctrlout_get_val(
						swctrlmapext[1], swctrlmap[1], 8*ant+16*elna);
				} else if ((decoded_address & ACPHY_MASK_WLAN_LOW_GAIN_RX) ==
						ACPHY_MAP_WLAN_LOW_GAIN_RX) {
					femctrlout = wlc_phy_femctrlout_get_val(
						swctrlmapext[2], swctrlmap[2], 8*ant+16*elna);
				} else if ((decoded_address & ACPHY_MASK_BT_TX) ==
						ACPHY_MAP_BT_TX) {
					femctrlout = wlc_phy_femctrlout_get_val(
						swctrlmapext[3], swctrlmap[3], 16);
				} else if ((decoded_address & ACPHY_MASK_BT_RX) ==
						ACPHY_MAP_BT_RX) {
					femctrlout = wlc_phy_femctrlout_get_val(
						swctrlmapext[3], swctrlmap[3],
						8*elna*!BT_eLNA_Gain);
				}

				if (core == 1)
					indx += 256;

				/* antdiv_rfswctrlpin_aX is the rfswctrl bit position to override */
				/* For stella this is USI ES2.0 it is 7 and 9 respectively */
				if (pi_ac->ant_swOvr_state_core0 != 2 && core == 0) {
					if (pi_ac->ant_swOvr_state_core0 == 1) {
						femctrlout = ((femctrlout &
							(~(1<<pi_ac->antdiv_rfswctrlpin_a0))) |
							(1<<pi_ac->antdiv_rfswctrlpin_a0));
					} else {
						femctrlout = (femctrlout &
							(~(1<<pi_ac->antdiv_rfswctrlpin_a0)));
					}
				}

				/* antdiv_rfswctrlpin_a1  - 8 maps to the LUT bit position */
				if (pi_ac->ant_swOvr_state_core1 != 2 && core == 1) {
					if (pi_ac->ant_swOvr_state_core1 == 1) {
						femctrlout = ((femctrlout &
							(~(1<<(pi_ac->antdiv_rfswctrlpin_a1-8)))) |
							(1<<(pi_ac->antdiv_rfswctrlpin_a1-8)));
					} else {
						femctrlout = (femctrlout &
							(~(1<<(pi_ac->antdiv_rfswctrlpin_a1-8))));
					}
				}

				wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_FEMCTRLLUT, 1, indx,
					16, &femctrlout);
			}
		}
	}
}

void
wlc_phy_farrow_setup_acphy(phy_info_t *pi, chanspec_t chanspec)
{
#ifdef ACPHY_1X1_ONLY
	uint32 dac_resamp_fcw;
	uint16 MuDelta_l, MuDelta_u;
	uint16 MuDeltaInit_l, MuDeltaInit_u;
#endif
	uint16 channel = CHSPEC_CHANNEL(chanspec);
	const uint16 *resamp_set = NULL;
	chan_info_tx_farrow *tx_farrow = NULL;
	chan_info_rx_farrow *rx_farrow;
	uint16 regval;
	int bw_idx = 0;
	int tbl_idx = 0;

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));
	if (ISSIM_ENAB(pi->sh->sih)) {
		/* Use channel 7(2g)/151(5g) settings for Quickturn */
		if (CHSPEC_IS2G(chanspec)) {
			channel = 7;
		} else {
			channel = 155;
		}
	}

	/* China 40M Spur WAR */
	if (ACMAJORREV_0(pi->pubpi->phy_rev)) {
		uint8 core;
		/* Cleanup Overrides */
		MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div_ovr, 0);
		MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div, 0x0);
		pi->sdadc_config_override = FALSE;

		FOREACH_CORE(pi, core) {
			MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_flashhspd, 0);
			MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_iqadc_flashhspd, 0);
			MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_ctrl_flash17lvl, 0);
			MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_ctrl_flash17lvl, 0);
			MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_mode, 0);
			MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_iqadc_mode, 0);
		}
	}

#ifdef ACPHY_1X1_ONLY
	bw_idx = 0;
#else /* ACPHY_1X1_ONLY */
	bw_idx = CHSPEC_IS20(chanspec)? 0 : (CHSPEC_IS40(chanspec)? 1 : 2);
#endif /* ACPHY_1X1_ONLY */
	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* Compute rx farrow setup */
		wlc_phy_write_rx_farrow_acphy(pi, chanspec);
	} else {
		/* Find the Rx Farrow settings in the table for the specific b/w and channel */
		for (tbl_idx = 0; tbl_idx < ACPHY_NUM_CHANS; tbl_idx++) {
			rx_farrow = &pi->u.pi_acphy->rx_farrow[bw_idx][tbl_idx];
			if (rx_farrow->chan == channel) {
				wlc_phy_write_rx_farrow_pre_tiny(pi, rx_farrow, chanspec);
				break;
			}
		}

		/*
		 * No need to iterate through the Tx Farrow table, since the channels have the same
		 * order as the Rx Farrow table.
		 */

		if (tbl_idx == ACPHY_NUM_CHANS) {
			PHY_ERROR(("wl%d: %s: Failed to find Farrow settings"
				   " for bw=%d, channel=%d\n",
				   pi->sh->unit, __FUNCTION__, CHSPEC_BW(chanspec), channel));
			return;
		}
	}

#ifdef ACPHY_1X1_ONLY
	ASSERT(((phy_info_acphy_t *)pi->u.pi_acphy)->dac_mode == 1);
	tx_farrow = &pi->u.pi_acphy->tx_farrow[0][tbl_idx];
	dac_resamp_fcw = tx_farrow->dac_resamp_fcw;

	if (CHSPEC_IS80(chanspec))
	{
		dac_resamp_fcw += (dac_resamp_fcw >> 1);
	}

	dac_resamp_fcw = (dac_resamp_fcw + 32) >> 6;

	MuDelta_l = (dac_resamp_fcw & 0xFFFF);
	MuDelta_u = (dac_resamp_fcw & 0xFF0000) >> 16;
	MuDeltaInit_l = (dac_resamp_fcw & 0xFFFF);
	MuDeltaInit_u = (dac_resamp_fcw & 0xFF0000) >> 16;

	wlc_phy_tx_farrow_mu_setup(pi, MuDelta_l, MuDelta_u, MuDeltaInit_l, MuDeltaInit_u);
#else /* ACPHY_1X1_ONLY */
	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* Compute tx farrow setup */
		wlc_phy_write_tx_farrow_acphy(pi, chanspec);
	} else {
		tx_farrow = &pi->u.pi_acphy->tx_farrow[bw_idx][tbl_idx];
		wlc_phy_tx_farrow_mu_setup(pi, tx_farrow->MuDelta_l, tx_farrow->MuDelta_u,
			tx_farrow->MuDeltaInit_l, tx_farrow->MuDeltaInit_u);
	}
#endif /* ACPHY_1X1_ONLY */

	/* China 40M Spur WAR */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) &&
	    (pi->afe_override) && CHSPEC_IS40(pi->radio_chanspec)) {
		uint16 fc;
		if (CHSPEC_CHANNEL(pi->radio_chanspec) > 14)
			fc = CHAN5G_FREQ(CHSPEC_CHANNEL(pi->radio_chanspec));
		else
			fc = CHAN2G_FREQ(CHSPEC_CHANNEL(pi->radio_chanspec));

		/* AFE Settings */
		if (fc == 5310) {
			uint8 core;
			MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div_ovr, 0x1);
			MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div, 0x0);

			FOREACH_CORE(pi, core) {
				MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_flashhspd, 1);
				MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core,
				             afe_iqadc_flashhspd, 1);
				MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_ctrl_flash17lvl, 0);
				MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core,
				             afe_ctrl_flash17lvl, 1);
				MOD_PHYREGCE(pi, RfctrlCoreAfeCfg2, core, afe_iqadc_mode, 1);
				MOD_PHYREGCE(pi, RfctrlOverrideAfeCfg, core, afe_iqadc_mode, 1);
			}

			MOD_RADIO_REG(pi, RF0, AFEDIV1, afediv_main_driver_size, 8);
			MOD_RADIO_REG(pi, RF0, AFEDIV2, afediv_repeater1_dsize, 8);
			MOD_RADIO_REG(pi, RF0, AFEDIV2, afediv_repeater2_dsize, 8);

			/* Set Override variable to pick up correct settings during cals */
			pi->sdadc_config_override = TRUE;
		} else if (fc == 5270) {
			MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div_ovr, 0x1);
			MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div, 0x2);
		}

		/* Resampler Settings */
		if (fc == 5270)
			resamp_set = resamp_cnwar_5270;
		else if (fc == 5310)
			resamp_set = resamp_cnwar_5310;

		if (resamp_set != NULL) {
			WRITE_PHYREG(pi, rxFarrowDeltaPhase_lo, resamp_set[0]);
			WRITE_PHYREG(pi, rxFarrowDeltaPhase_hi, resamp_set[1]);
			WRITE_PHYREG(pi, rxFarrowDriftPeriod, resamp_set[2]);
			WRITE_PHYREG(pi, lbFarrowDeltaPhase_lo, resamp_set[3]);
			WRITE_PHYREG(pi, lbFarrowDeltaPhase_hi, resamp_set[4]);
			WRITE_PHYREG(pi, lbFarrowDriftPeriod, resamp_set[5]);
			ACPHYREG_BCAST(pi, TxResamplerMuDelta0l, resamp_set[6]);
			ACPHYREG_BCAST(pi, TxResamplerMuDelta0u, resamp_set[7]);
			ACPHYREG_BCAST(pi, TxResamplerMuDeltaInit0l, resamp_set[8]);
			ACPHYREG_BCAST(pi, TxResamplerMuDeltaInit0u, resamp_set[9]);
		}
	}

	/* Enable the Tx resampler on all cores */
	regval = READ_PHYREG(pi, TxResamplerEnable0);
	regval |= (1 < ACPHY_TxResamplerEnable0_enable_tx_SHIFT(pi->pubpi->phy_rev));
	ACPHYREG_BCAST(pi, TxResamplerEnable0,  regval);
	if (ACMAJORREV_4(pi->pubpi->phy_rev))
		MOD_PHYREG(pi, AfeClkDivOverrideCtrl, afediv_sel_div_ovr, 0x1);
}


void
wlc_phy_enable_lna_dcc_comp_20691(phy_info_t *pi, bool on)
{
	uint16 sparereg = READ_PHYREG(pi, SpareReg);

	if (on)
		sparereg &= 0xfffe;
	else
		sparereg |= 0x0001;

	WRITE_PHYREG(pi, SpareReg, sparereg);
}

/*  lookup radio-chip-specific channel code */
int
wlc_phy_chan2freq_acphy(phy_info_t *pi, uint8 channel, const void **chan_info)
{
	uint i;
	chan_info_radio2069_t *chan_info_tbl = NULL;
	chan_info_radio2069revGE16_t *chan_info_tbl_GE16 = NULL;
	chan_info_radio2069revGE25_t *chan_info_tbl_GE25 = NULL;
	chan_info_radio2069revGE32_t *chan_info_tbl_GE32 = NULL;
	chan_info_radio2069revGE25_52MHz_t *chan_info_tbl_GE25_52MHz = NULL;

	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	uint32 tbl_len = 0;
	int freq;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));
	switch (RADIO2069_MAJORREV(pi->pubpi->radiorev)) {
	case 0:
		switch (RADIO2069REV(pi->pubpi->radiorev)) {
		case 3:
			chan_info_tbl = chan_tuning_2069rev3;
			tbl_len = ARRAYSIZE(chan_tuning_2069rev3);
		break;

		case 4:
		case 8:
			chan_info_tbl = chan_tuning_2069rev4;
			tbl_len = ARRAYSIZE(chan_tuning_2069rev4);
			break;
		case 7: /* e.g. 43602a0 */
			chan_info_tbl = chan_tuning_2069rev7;
			tbl_len = ARRAYSIZE(chan_tuning_2069rev7);
			break;
		case 64: /* e.g. 4364 */
			chan_info_tbl = chan_tuning_2069rev64;
			tbl_len = ARRAYSIZE(chan_tuning_2069rev64);
			break;
		default:

			PHY_ERROR(("wl%d: %s: Unsupported radio revision %d\n",
			           pi->sh->unit, __FUNCTION__, RADIO2069REV(pi->pubpi->radiorev)));
			ASSERT(0);
		}
		break;

	case 1:
		switch (RADIO2069REV(pi->pubpi->radiorev)) {
			case 16:
				if (pi->xtalfreq == 40000000) {
#ifndef ACPHY_1X1_37P4
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						/* In this configure the LP mode settings */
						/* For Rev16/17/18 using the same LP setting TBD */
						chan_info_tbl_GE16 = chan_tuning_2069rev_GE16_40_lp;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_GE16_40_lp);
					} else {
						chan_info_tbl_GE16 = chan_tuning_2069rev_16_17_40;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_16_17_40);
					}
#else
					ASSERT(0);
#endif /* ACPHY_1X1_37P4 */
				} else {
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						/* In this configure the LP mode settings */
						/* For Rev16/17/18 using the same LP setting TBD */
						chan_info_tbl_GE16 = chan_tuning_2069rev_GE16_lp;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_GE16_lp);
					} else {
						chan_info_tbl_GE16 = chan_tuning_2069rev_16_17;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_16_17);
					}
				}
				pi_ac->acphy_prev_lp_mode = pi_ac->acphy_lp_mode;
				break;
			case 17:
			case 23:
				if (pi->xtalfreq == 40000000) {
#ifndef ACPHY_1X1_37P4
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						/* In this configure the LP mode settings */
						/* For Rev16/17/18 using the same LP setting TBD */
						chan_info_tbl_GE16 =
						       chan_tuning_2069rev_GE16_40_lp;
						tbl_len =
						       ARRAYSIZE(chan_tuning_2069rev_GE16_40_lp);
					} else {
						chan_info_tbl_GE16 = chan_tuning_2069rev_16_17_40;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_16_17_40);
					}
#else
					ASSERT(0);
#endif /* ACPHY_1X1_37P4 */
				} else {
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
#ifndef ACPHY_1X1_37P4
					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						/* In this configure the LP mode settings */
						/* For Rev16/17/18 using the same LP setting TBD */
						chan_info_tbl_GE16 = chan_tuning_2069rev_GE16_lp;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_GE16_lp);
					} else {
						chan_info_tbl_GE16 = chan_tuning_2069rev_16_17;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_16_17);
					}
#else
					if ((RADIO2069REV(pi->pubpi->radiorev)) == 23) {
						chan_info_tbl_GE16 =
						 chan_tuning_2069rev_23_2Glp_5Gnonlp;
						tbl_len =
						 ARRAYSIZE(chan_tuning_2069rev_23_2Glp_5Gnonlp);

					} else {
						chan_info_tbl_GE16 =
						 chan_tuning_2069rev_GE16_2Glp_5Gnonlp;
						tbl_len =
						 ARRAYSIZE(chan_tuning_2069rev_GE16_2Glp_5Gnonlp);
					}
#endif /* ACPHY_1X1_37P4 */
				}
				pi_ac->acphy_prev_lp_mode = pi_ac->acphy_lp_mode;
				break;
			case 18:
			case 24:
				if (pi->xtalfreq == 40000000) {
#ifndef ACPHY_1X1_37P4
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						/* In this configure the LP mode settings */
						/* For Rev16/17/18 using the same LP setting TBD */
						chan_info_tbl_GE16 =
						    chan_tuning_2069rev_GE16_40_lp;
						tbl_len =
						    ARRAYSIZE(chan_tuning_2069rev_GE16_40_lp);
					} else {
						chan_info_tbl_GE16 = chan_tuning_2069rev_18_40;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_18_40);
					}
#else
					ASSERT(0);
#endif /* ACPHY_1X1_37P4 */
				} else {
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
					if ((pi_ac->acphy_lp_mode == 2) ||
					        (pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						/* In this configure LP mode settings */
						/* For Rev16/17/18 using same LP setting TBD */
						chan_info_tbl_GE16 =
						       chan_tuning_2069rev_GE16_lp;
						tbl_len =
						       ARRAYSIZE(chan_tuning_2069rev_GE16_lp);
					} else {
						chan_info_tbl_GE16 = chan_tuning_2069rev_18;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_18);
					}
				}
				pi_ac->acphy_prev_lp_mode = pi_ac->acphy_lp_mode;
				break;
			case 25:
			case 26:
				if (pi->xtalfreq == 40000000) {
#ifndef ACPHY_1X1_37P4
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;

					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						chan_info_tbl_GE25 =
							chan_tuning_2069rev_GE_25_40MHz_lp;
						tbl_len =
						ARRAYSIZE(chan_tuning_2069rev_GE_25_40MHz_lp);
					} else {
						chan_info_tbl_GE25 =
						     chan_tuning_2069rev_GE_25_40MHz;
						tbl_len =
						     ARRAYSIZE(chan_tuning_2069rev_GE_25_40MHz);
					}
#else
					ASSERT(0);
#endif /* ACPHY_1X1_37P4 */
				} else if (pi->xtalfreq == 52000000) {
					chan_info_tbl_GE25_52MHz = pi->u.pi_acphy->chan_tuning;
					tbl_len = pi->u.pi_acphy->chan_tuning_tbl_len;
				} else {
					pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
					if ((pi_ac->acphy_lp_mode == 2) ||
						(pi_ac->acphy_lp_mode == 3) ||
						(pi_ac->acphy_force_lpvco_2G == 1 &&
						CHSPEC_IS2G(pi->radio_chanspec))) {
						chan_info_tbl_GE25 = chan_tuning_2069rev_GE_25_lp;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_GE_25_lp);
					} else {
						chan_info_tbl_GE25 = chan_tuning_2069rev_GE_25;
						tbl_len = ARRAYSIZE(chan_tuning_2069rev_GE_25);
					}
				}
				pi_ac->acphy_prev_lp_mode = pi_ac->acphy_lp_mode;
				break;
			default:
				PHY_ERROR(("wl%d: %s: Unsupported radio revision %d\n",
				   pi->sh->unit, __FUNCTION__, RADIO2069REV(pi->pubpi->radiorev)));
				ASSERT(0);
		}

		break;

	case 2:
		switch (RADIO2069REV(pi->pubpi->radiorev)) {
		case 32:
		case 33:
		case 34:
		case 35:
		case 36:
		case 37:
		case 38:
		case 39:
		case 40:
		case 44:
			/* can have more conditions based on different radio revs */
			/*  RADIOREV(pi->pubpi->radiorev) =32/33/34 */
			/* currently tuning tbls for these are all same */
			chan_info_tbl_GE32 = pi->u.pi_acphy->chan_tuning;
			tbl_len = pi->u.pi_acphy->chan_tuning_tbl_len;
			break;

		default:

			PHY_ERROR(("wl%d: %s: Unsupported radio revision %d\n",
			           pi->sh->unit, __FUNCTION__, RADIO2069REV(pi->pubpi->radiorev)));
			ASSERT(0);
		}
		break;
	default:
		PHY_ERROR(("wl%d: %s: Unsupported radio major revision %d\n",
		           pi->sh->unit, __FUNCTION__, RADIO2069_MAJORREV(pi->pubpi->radiorev)));
		ASSERT(0);
	}

	for (i = 0; i < tbl_len; i++) {

		if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) {
			if (chan_info_tbl_GE32[i].chan == channel)
				break;
		} else if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) {
			if ((RADIO2069REV(pi->pubpi->radiorev) == 25) ||
			   (RADIO2069REV(pi->pubpi->radiorev) == 26))  {
			    if (pi->xtalfreq != 52000000) {
					if (chan_info_tbl_GE25[i].chan == channel)
						break;
				} else {
					if (chan_info_tbl_GE25_52MHz[i].chan == channel)
						break;
				}
			}
			else if (chan_info_tbl_GE16[i].chan == channel)
				break;
		} else {
			if (chan_info_tbl[i].chan == channel)
				break;
		}
	}

	if (i >= tbl_len) {
		PHY_ERROR(("wl%d: %s: channel %d not found in channel table\n",
		           pi->sh->unit, __FUNCTION__, channel));
		ASSERT(i < tbl_len);

		return -1;
	}

	if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) {
		*chan_info = &chan_info_tbl_GE32[i];
		freq = chan_info_tbl_GE32[i].freq;
	} else if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) {
		if ((RADIO2069REV(pi->pubpi->radiorev) == 25) ||
			(RADIO2069REV(pi->pubpi->radiorev) == 26)) {
				if (pi->xtalfreq != 52000000) {
					*chan_info = &chan_info_tbl_GE25[i];
					freq = chan_info_tbl_GE25[i].freq;
				} else {
					*chan_info = &chan_info_tbl_GE25_52MHz[i];
					freq = chan_info_tbl_GE25_52MHz[i].freq;
				}
		} else {
			*chan_info = &chan_info_tbl_GE16[i];
			freq = chan_info_tbl_GE16[i].freq;
		}
	} else {
		*chan_info = &chan_info_tbl[i];
		freq = chan_info_tbl[i].freq;
	}

	return freq;
}

int
wlc_phy_chan2freq_20691(phy_info_t *pi, uint8 channel, const chan_info_radio20691_t **chan_info)
{
	uint i;
	chan_info_radio20691_t *chan_info_tbl;
	uint32 tbl_len;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM20691_ID));

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	/* Choose the right table to use */
	switch (RADIO20691REV(pi->pubpi->radiorev)) {
	case 60:
	case 68:
		chan_info_tbl = chan_tuning_20691_rev68;
		tbl_len = ARRAYSIZE(chan_tuning_20691_rev68);
		break;
	case 75:
		chan_info_tbl = chan_tuning_20691_rev75;
		tbl_len = ARRAYSIZE(chan_tuning_20691_rev75);
		break;
	case 79:
		chan_info_tbl = chan_tuning_20691_rev79;
		tbl_len = ARRAYSIZE(chan_tuning_20691_rev79);
		break;
	case 74:
	case 82:
		chan_info_tbl = chan_tuning_20691_rev82;
		tbl_len = ARRAYSIZE(chan_tuning_20691_rev82);
		break;
	case 85:
	case 86:
	case 87:
	case 88:
		chan_info_tbl = chan_tuning_20691_rev88;
		tbl_len = 0;
		break;
	case 129:
		chan_info_tbl = chan_tuning_20691_rev129;
		tbl_len = ARRAYSIZE(chan_tuning_20691_rev129);
		break;
	default:
		PHY_ERROR(("wl%d: %s: Unsupported radio revision %d\n",
			pi->sh->unit, __FUNCTION__, RADIO20691REV(pi->pubpi->radiorev)));
		ASSERT(FALSE);
		return -1;
	}

	for (i = 0; i < tbl_len && chan_info_tbl[i].chan != channel; i++);

	if (i >= tbl_len) {
		PHY_ERROR(("wl%d: %s: channel %d not found in channel table\n",
		           pi->sh->unit, __FUNCTION__, channel));
		ASSERT(tbl_len == 0 || i < tbl_len);
		return -1;
	}

	*chan_info = &chan_info_tbl[i];

	return chan_info_tbl[i].freq;
}

void
wlc_phy_set_lowpwr_phy_reg_rev3(phy_info_t *pi)
{
	MOD_PHYREG(pi, radio_pll_vcoSet1, vco_en_alc, 0x0);
	MOD_PHYREG(pi, radio_rxrf_lna5g, lna5g_lna1_bias_idac, 0x8);
	MOD_PHYREG(pi, radio_pll_vcoSet4, vco_tempco_dcadj_1p2, 0x9);
	MOD_PHYREG(pi, radio_pll_vcoSet2, vco_vctrl_buf_ical, 0x3);
	MOD_PHYREG(pi, radio_pll_vcoSet4, vco_ib_bias_opamp, 0x6);
	MOD_PHYREG(pi, radio_pll_vcoSet4, vco_ib_bias_opamp_fastsettle, 0xf);
	MOD_PHYREG(pi, radio_pll_vcoSet1, vco_bypass_vctrl_buf, 0x0);
	MOD_PHYREG(pi, radio_pll_vcoSet3, vco_HDRM_CAL, 0x2);
	MOD_PHYREG(pi, radio_pll_vcoSet2, vco_ICAL, 0x16);
	MOD_PHYREG(pi, radio_pll_vcoSet3, vco_ICAL_1p2, 0xc);
	MOD_PHYREG(pi, radio_pll_vcoSet1, vco_USE_2p5V, 0x1);
	if (ACMAJORREV_2(pi->pubpi->phy_rev) && ACMINORREV_1(pi)) {
		MOD_PHYREG(pi, radio_logen2gN5g, idac_mix, 0x4);
	}
}

void
wlc_phy_set_lowpwr_phy_reg(phy_info_t *pi)
{
	/* These guys not required for tiny based phys */
	if (!TINY_RADIO(pi)) {
		MOD_PHYREG(pi, radio_logen2g, idac_gm, 0x3);
		MOD_PHYREG(pi, radio_logen2g, idac_gm_2nd, 0x3);
		MOD_PHYREG(pi, radio_logen2g, idac_qb, 0x3);
		MOD_PHYREG(pi, radio_logen2g, idac_qb_2nd, 0x3);
		MOD_PHYREG(pi, radio_logen2g, idac_qtx, 0x4);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_itx, 0x4);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_qrx, 0x4);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_irx, 0x4);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_buf, 0x3);
		MOD_PHYREG(pi, radio_logen2gN5g, idac_mix, 0x3);
		MOD_PHYREG(pi, radio_logen5g, idac_div, 0x3);
		MOD_PHYREG(pi, radio_logen5g, idac_vcob, 0x3);
		MOD_PHYREG(pi, radio_logen5gbufs, idac_bufb, 0x3);
		MOD_PHYREG(pi, radio_logen5g, idac_mixb, 0x3);
		MOD_PHYREG(pi, radio_logen5g, idac_load, 0x3);
		MOD_PHYREG(pi, radio_logen5gbufs, idac_buf2, 0x3);
		MOD_PHYREG(pi, radio_logen5gbufs, idac_bufb2, 0x3);
		MOD_PHYREG(pi, radio_logen5gbufs, idac_buf1, 0x3);
		MOD_PHYREG(pi, radio_logen5gbufs, idac_bufb1, 0x3);
		MOD_PHYREG(pi, radio_logen5gQI, idac_qtx, 0x4);
		MOD_PHYREG(pi, radio_logen5gQI, idac_itx, 0x4);
		MOD_PHYREG(pi, radio_logen5gQI, idac_qrx, 0x4);
		MOD_PHYREG(pi, radio_logen5gQI, idac_irx, 0x4);
		MOD_PHYREG(pi, radio_pll_vcocal, vcocal_rstn, 0x1);
		MOD_PHYREG(pi, radio_pll_vcocal, vcocal_force_caps, 0x0);
		MOD_PHYREG(pi, radio_pll_vcocal, vcocal_force_caps_val, 0x40);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_ALC_ref_ctrl, 0xd);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_bias_mode, 0x1);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_cvar_extra, 0xb);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_cvar, 0xf);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_en_alc, 0x0);
		MOD_PHYREG(pi, radio_pll_vcoSet2, vco_tempco_dcadj, 0xe);
		MOD_PHYREG(pi, radio_pll_vcoSet2, vco_tempco, 0xb);
		MOD_PHYREG(pi, radio_pll_vcoSet3, vco_cal_en, 0x1);
		MOD_PHYREG(pi, radio_pll_vcoSet3, vco_cal_en_empco, 0x1);
		MOD_PHYREG(pi, radio_pll_vcoSet3, vco_cap_mode, 0x0);
		MOD_PHYREG(pi, radio_pll_vcoSet4, vco_ib_ctrl, 0x0);
		MOD_PHYREG(pi, radio_pll_vcoSet3, vco_por, 0x0);
		MOD_PHYREG(pi, radio_pll_lf_r1, lf_r1, 0x0);
		MOD_PHYREG(pi, radio_pll_lf_r2r3, lf_r2, 0xc);
		MOD_PHYREG(pi, radio_pll_lf_r2r3, lf_r3, 0xc);
		MOD_PHYREG(pi, radio_pll_lf_cm, lf_rs_cm, 0xff);
		MOD_PHYREG(pi, radio_pll_lf_cm, lf_rf_cm, 0xc);
		MOD_PHYREG(pi, radio_pll_lf_cSet1, lf_c1, 0x99);
		MOD_PHYREG(pi, radio_pll_lf_cSet1, lf_c2, 0x8b);
		MOD_PHYREG(pi, radio_pll_lf_cSet2, lf_c3, 0x8b);
		MOD_PHYREG(pi, radio_pll_lf_cSet2, lf_c4, 0x8f);
		MOD_PHYREG(pi, radio_pll_cp, cp_kpd_scale, 0x34);
		MOD_PHYREG(pi, radio_pll_cp, cp_ioff, 0x60);
		MOD_PHYREG(pi, radio_ldo, ldo_1p2_xtalldo1p2_lowquiescenten, 0x0);
		MOD_PHYREG(pi, radio_ldo, ldo_2p5_lowpwren_VCO, 0x0);
		MOD_PHYREG(pi, radio_ldo, ldo_2p5_lowquiescenten_VCO_aux, 0x0);
		MOD_PHYREG(pi, radio_ldo, ldo_2p5_lowpwren_VCO_aux, 0x0);
		MOD_PHYREG(pi, radio_ldo, ldo_2p5_lowquiescenten_CP, 0x0);
		MOD_PHYREG(pi, radio_ldo, ldo_2p5_lowquiescenten_VCO, 0x0);
		MOD_PHYREG(pi, radio_rxrf_lna2g, lna2g_lna1_bias_idac, 0x2);
		MOD_PHYREG(pi, radio_rxrf_lna2g, lna2g_lna2_aux_bias_idac, 0x8);
		MOD_PHYREG(pi, radio_rxrf_lna2g, lna2g_lna2_main_bias_idac, 0x8);
		MOD_PHYREG(pi, radio_rxrf_lna5g, lna5g_lna1_bias_idac, 0x8);
		MOD_PHYREG(pi, radio_rxrf_lna5g, lna5g_lna2_aux_bias_idac, 0x7);
		MOD_PHYREG(pi, radio_rxrf_lna5g, lna5g_lna2_main_bias_idac, 0x4);
		MOD_PHYREG(pi, radio_rxrf_rxmix, rxmix2g_aux_bias_idac, 0x8);
		MOD_PHYREG(pi, radio_rxrf_rxmix, rxmix2g_main_bias_idac, 0x8);
		MOD_PHYREG(pi, radio_rxrf_rxmix, rxmix5g_gm_aux_bias_idac_i, 0x8);
		MOD_PHYREG(pi, radio_rxrf_rxmix, rxmix5g_gm_main_bias_idac_i, 0x8);
		MOD_PHYREG(pi, radio_rxbb_tia, tia_DC_Ib1, 0x6);
		MOD_PHYREG(pi, radio_rxbb_tia, tia_DC_Ib2, 0x6);
		MOD_PHYREG(pi, radio_rxbb_tia, tia_Ib_I, 0x6);
		MOD_PHYREG(pi, radio_rxbb_tia, tia_Ib_Q, 0x6);
		MOD_PHYREG(pi, radio_rxbb_bias12, lpf_bias_level1, 0x4);
		MOD_PHYREG(pi, radio_rxbb_bias12, lpf_bias_level2, 0x8);
		MOD_PHYREG(pi, radio_rxbb_bias34, lpf_bias_level3, 0x10);
		MOD_PHYREG(pi, radio_rxbb_bias34, lpf_bias_level4, 0x20);
		MOD_PHYREG(pi, radio_pll_vcoSet4, vco_tempco_dcadj_1p2, 0x9);
		MOD_PHYREG(pi, radio_pll_vcoSet2, vco_vctrl_buf_ical, 0x3);
		MOD_PHYREG(pi, radio_pll_vcoSet4, vco_ib_bias_opamp, 0x6);
		MOD_PHYREG(pi, radio_pll_vcoSet4, vco_ib_bias_opamp_fastsettle, 0xf);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_bypass_vctrl_buf, 0x0);
		MOD_PHYREG(pi, radio_pll_vcoSet3, vco_HDRM_CAL, 0x2);
		MOD_PHYREG(pi, radio_pll_vcoSet2, vco_ICAL, 0x16);
		MOD_PHYREG(pi, radio_pll_vcoSet3, vco_ICAL_1p2, 0xc);
		MOD_PHYREG(pi, radio_pll_vcoSet1, vco_USE_2p5V, 0x1);
	}
}

/** Tx implicit beamforming. Ingress and outgress channels are assumed to have reprocity. */
void
wlc_phy_populate_recipcoeffs_acphy(phy_info_t *pi)
{
	int16 sin_tbl[] = {
	0x000, 0x00d, 0x019, 0x026, 0x032, 0x03f, 0x04b, 0x058,
	0x064, 0x070, 0x07c, 0x089, 0x095, 0x0a1, 0x0ac, 0x0b8,
	0x0c4, 0x0cf, 0x0db, 0x0e6, 0x0f1, 0x0fc, 0x107, 0x112,
	0x11c, 0x127, 0x131, 0x13b, 0x145, 0x14e, 0x158, 0x161,
	0x16a, 0x173, 0x17b, 0x184, 0x18c, 0x194, 0x19b, 0x1a3,
	0x1aa, 0x1b1, 0x1b7, 0x1bd, 0x1c4, 0x1c9, 0x1cf, 0x1d4,
	0x1d9, 0x1de, 0x1e2, 0x1e6, 0x1ea, 0x1ed, 0x1f1, 0x1f4,
	0x1f6, 0x1f8, 0x1fa, 0x1fc, 0x1fe, 0x1ff, 0x1ff, 0x200};

	uint16 start_words[][3] = {
		{0x005B, 0x0000, 0x0000},
		{0x8250, 0x0000, 0x0000},
		{0xC338, 0x0000, 0x0000},
		{0x4527, 0x0001, 0x0000},
		{0xA6A1, 0x0001, 0x0000},
		{0x081B, 0x0002, 0x0000},
		{0x8A18, 0x0002, 0x0000},
		{0x2C96, 0x0003, 0x0000},
		{0x8E17, 0x0003, 0x0000},
		{0x101B, 0x0004, 0x0000},
		{0x0020, 0x0000, 0x0000},
		{0x0020, 0x0000, 0x0000}};

	uint16 packed_word[3];
	uint16 zero_word[3] = {0, 0, 0};

	uint16 ang_tmp, recip_coef_nfrac = 11;
	uint16 subband_idx, k, num_angles = 2;
	uint16 theta[2];
	int16  re = 0, im = 0, exp, quad;
	int16  sin_idx, cos_idx;
	int16  sin_out, cos_out;
	uint32 packed;
	uint16 nwords_start = 12, nwords_pad = 4, nwords_recip;
	uint8  stall_val;

	bool is_caled = wlc_phy_is_txbfcal((wlc_phy_t *)pi);

	if (pi->sh->hw_phytxchain <= 1 || !(is_caled)) {
		return;
	}

	/* 1. obtain angles from SROM */
	subband_idx = wlc_phy_get_chan_freq_range_acphy(pi, 0, PRIMARY_FREQ_SEGMENT);
	switch (subband_idx) {
	case WL_CHAN_FREQ_RANGE_2G:
#ifdef WLTXBF_2G_DISABLED
		ang_tmp = 0;
#else
		ang_tmp = pi->sromi->rpcal2g;
#endif /* WLTXBF_2G_DISABLED */
		break;
	case WL_CHAN_FREQ_RANGE_5G_BAND0:
		ang_tmp = pi->sromi->rpcal5gb0;
		break;
	case WL_CHAN_FREQ_RANGE_5G_BAND1:
		ang_tmp = pi->sromi->rpcal5gb1;
		break;
	case WL_CHAN_FREQ_RANGE_5G_BAND2:
		ang_tmp = pi->sromi->rpcal5gb2;
		break;
	case WL_CHAN_FREQ_RANGE_5G_BAND3:
		ang_tmp = pi->sromi->rpcal5gb3;
		break;
	default:
		ang_tmp = pi->sromi->rpcal2g;
		break;
	}
	theta[0] = (uint8) (ang_tmp & 0xFF);
	theta[1] = (uint8) ((ang_tmp >> 8) & 0xFF);

	/* printf("---- theta1 = %d, theta2 = %d\n", theta[0], theta[1]); */

	/* 2. generate packed word */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {

		for (k = 0, packed = 0; k < num_angles; k++) {

			/* 6 LSBs for 1st quadrant */
			sin_idx = (theta[k] & 0x3F);
			cos_idx = 63 - sin_idx;

			sin_out = sin_tbl[sin_idx];
			cos_out = sin_tbl[cos_idx];

			/* 2MSBs for quadrant */
			quad = ((theta[k] >> 6) & 0x3);

			if (quad == 0) {
				re =  cos_out; im = -sin_out;
			} else if (quad == 1) {
				re = -sin_out; im = -cos_out;
			} else if (quad == 2) {
				re = -cos_out; im =  sin_out;
			} else if (quad == 3) {
				re =  sin_out; im =  cos_out;
			}

			re += (re < 0) ? (1 << recip_coef_nfrac) : 0;
			im += (im < 0) ? (1 << recip_coef_nfrac) : 0;
			exp = 1;

			packed = (uint32) ((exp << (2*recip_coef_nfrac)) |
			(im << recip_coef_nfrac) | re);

			if (k == 0) {
				packed_word[0] = (packed & 0xFFFF);
				packed_word[1] = (packed >> 16) & 0xFF;
			} else if (k == 1) {
				packed_word[1] |= ((packed & 0xFF) << 8);
				packed_word[2] = (packed >> 8) & 0xFFFF;
			}
		}
		/* printf("reciprocity packed_word: %x%x%x\n",
		packed_word[2], packed_word[1], packed_word[0]);
		*/

	} else if ((ACMAJORREV_2(pi->pubpi->phy_rev)) || (ACMAJORREV_4(pi->pubpi->phy_rev))) {

		/* every 4 tones are packed into 1 word */
		packed = (theta[0] | (theta[0] << 8) | (theta[0] << 16) | (theta[0] << 24));

		/* printf("reciprocity packedWideWord: %x\n", packed); */
	}

	/* Disable stalls and hold FIFOs in reset */
	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	/* 3. write to table */
	/* 4360 and 43602 */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		nwords_recip = 64 + 128 + 256;

		for (k = 0; k < nwords_start; k++) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_BFECONFIG,
			1, k, 48, start_words[k]);
		}

		for (k = 0; k < nwords_recip; k++) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_BFECONFIG,
			1, nwords_start + k, 48, packed_word);
		}

		for (k = 0; k < nwords_pad; k++) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_BFECONFIG,
			1, nwords_start + nwords_recip + k, 48, zero_word);
		}
	} else if ((ACMAJORREV_2(pi->pubpi->phy_rev)) || (ACMAJORREV_4(pi->pubpi->phy_rev))) {

		/* 4 tones are packed into one word */
		nwords_recip = (256 >> 2);

		for (k = 0; k < nwords_recip; k++) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_BFECONFIG2X2TBL,
			1, k, 32, &packed);
		}
	}

	ACPHY_ENABLE_STALL(pi, stall_val);
}

/* get the complex freq. if chan==0, use default radio channel */
uint8
wlc_phy_get_chan_freq_range_acphy(phy_info_t *pi, chanspec_t chanspec, uint8 core_segment_mapping)
{
	int freq = WL_CHAN_FREQ_RANGE_2G;
	uint8 channel = CHSPEC_CHANNEL(chanspec);

	if (phy_get_phymode(pi) != PHYMODE_80P80) {
		if (channel == 0)
			channel = CHSPEC_CHANNEL(pi->radio_chanspec);
	} else {
		if (channel == 0)
			chanspec = pi->radio_chanspec;

		if (CHSPEC_BW(chanspec) == WL_CHANSPEC_BW_8080) {
			if (PRIMARY_FREQ_SEGMENT == core_segment_mapping)
				channel = wf_chspec_primary80_channel(chanspec);

			if (SECONDARY_FREQ_SEGMENT == core_segment_mapping)
				channel = wf_chspec_secondary80_channel(chanspec);
		} else {
			channel = CHSPEC_CHANNEL(chanspec);
		}
	}
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	if (RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) {
		const void *chan_info;

		freq = wlc_phy_chan2freq_acphy(pi, channel, &chan_info);
	} else if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
		const chan_info_radio20693_pll_t *chan_info_pll;
		const chan_info_radio20693_rffe_t *chan_info_rffe;

		freq = wlc_phy_chan2freq_20693(pi, channel, &chan_info_pll, &chan_info_rffe);
	} else {
		const chan_info_radio20691_t *chan_info;

		freq = wlc_phy_chan2freq_20691(pi, channel, &chan_info);
	}

	if (channel <= CH_MAX_2G_CHANNEL || freq < 0)
		return WL_CHAN_FREQ_RANGE_2G;
	if ((pi->sromi->subband5Gver == PHY_MAXNUM_5GSUBBANDS) ||
	    (pi->sromi->subband5Gver == PHY_SUBBAND_4BAND)) {
			if ((freq >= PHY_SUBBAND_4BAND_BAND0) &&
				(freq < PHY_SUBBAND_4BAND_BAND1))
				return WL_CHAN_FREQ_RANGE_5G_BAND0;
			else if ((freq >= PHY_SUBBAND_4BAND_BAND1) &&
				(freq < PHY_SUBBAND_4BAND_BAND2))
				return WL_CHAN_FREQ_RANGE_5G_BAND1;
			else if ((freq >= PHY_SUBBAND_4BAND_BAND2) &&
				(freq < PHY_SUBBAND_4BAND_BAND3))
				return WL_CHAN_FREQ_RANGE_5G_BAND2;
			else
				return WL_CHAN_FREQ_RANGE_5G_BAND3;
		} else if (pi->sromi->subband5Gver == PHY_SUBBAND_3BAND_EMBDDED) {
			if ((freq >= EMBEDDED_LOW_5G_CHAN) && (freq < EMBEDDED_MID_5G_CHAN)) {
				return WL_CHAN_FREQ_RANGE_5GL;
			} else if ((freq >= EMBEDDED_MID_5G_CHAN) &&
			           (freq < EMBEDDED_HIGH_5G_CHAN)) {
				return WL_CHAN_FREQ_RANGE_5GM;
			} else {
				return WL_CHAN_FREQ_RANGE_5GH;
			}
		} else if (pi->sromi->subband5Gver == PHY_SUBBAND_3BAND_HIGHPWR) {
			if ((freq >= HIGHPWR_LOW_5G_CHAN) && (freq < HIGHPWR_MID_5G_CHAN)) {
				return WL_CHAN_FREQ_RANGE_5GL;
			} else if ((freq >= HIGHPWR_MID_5G_CHAN) && (freq < HIGHPWR_HIGH_5G_CHAN)) {
				return WL_CHAN_FREQ_RANGE_5GM;
			} else {
				return WL_CHAN_FREQ_RANGE_5GH;
			}
	} else { /* Default PPR Subband subband5Gver = 7 */
			if ((freq >= JAPAN_LOW_5G_CHAN) && (freq < JAPAN_MID_5G_CHAN)) {
				return WL_CHAN_FREQ_RANGE_5GL;
			} else if ((freq >= JAPAN_MID_5G_CHAN) && (freq < JAPAN_HIGH_5G_CHAN)) {
				return WL_CHAN_FREQ_RANGE_5GM;
			} else {
				return WL_CHAN_FREQ_RANGE_5GH;
		}
	}
}

/* get the complex freq. if chan==0, use default radio channel */
uint8
wlc_phy_get_chan_freq_range_srom12_acphy(phy_info_t *pi, chanspec_t chanspec)
{
	if (!(SROMREV(pi->sh->sromrev) < 12)) {
	    int freq = WL_CHAN_FREQ_RANGE_2G;
	    uint8 channel = CHSPEC_CHANNEL(chanspec);
	    PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	    if (channel == 0)
		channel = CHSPEC_CHANNEL(pi->radio_chanspec);

	    if (RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) {
		const void *chan_info;
		freq = wlc_phy_chan2freq_acphy(pi, channel, &chan_info);
	    }

	    if (channel <= CH_MAX_2G_CHANNEL || freq < 0) {
		if (CHSPEC_IS40(chanspec))
		    return WL_CHAN_FREQ_RANGE_2G_40;
		else
		    return WL_CHAN_FREQ_RANGE_2G;
	    }

	    if (pi->sromi->subband5Gver == PHY_MAXNUM_5GSUBBANDS) {
		if ((freq >= PHY_MAXNUM_5GSUBBANDS_BAND0) &&
		    (freq < PHY_MAXNUM_5GSUBBANDS_BAND1)) {
		    if (CHSPEC_IS40(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND0_40;
		    else if (CHSPEC_IS80(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND0_80;
		    else
			return WL_CHAN_FREQ_RANGE_5G_BAND0;
		} else if ((freq >= PHY_MAXNUM_5GSUBBANDS_BAND1) &&
			(freq < PHY_MAXNUM_5GSUBBANDS_BAND2)) {
		    if (CHSPEC_IS40(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND1_40;
		    else if (CHSPEC_IS80(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND1_80;
		    else
			return WL_CHAN_FREQ_RANGE_5G_BAND1;
		} else if ((freq >= PHY_MAXNUM_5GSUBBANDS_BAND2) &&
			(freq < PHY_MAXNUM_5GSUBBANDS_BAND3)) {
		    if (CHSPEC_IS40(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND2_40;
		    else if (CHSPEC_IS80(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND2_80;
		    else
			return WL_CHAN_FREQ_RANGE_5G_BAND2;
		} else if ((freq >= PHY_MAXNUM_5GSUBBANDS_BAND3) &&
		        (freq < PHY_MAXNUM_5GSUBBANDS_BAND4)) {
		    if (CHSPEC_IS40(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND3_40;
		    else if (CHSPEC_IS80(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND3_80;
		    else
			return WL_CHAN_FREQ_RANGE_5G_BAND3;
		} else {
		    if (CHSPEC_IS40(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND4_40;
		    else if (CHSPEC_IS80(chanspec))
			return WL_CHAN_FREQ_RANGE_5G_BAND4_80;
		    else
			return WL_CHAN_FREQ_RANGE_5G_BAND4;
		}
	    }
	}
	return WL_CHAN_FREQ_RANGE_2G;
}

void
wlc_phy_smth(phy_info_t *pi, int8 enable_smth, int8 dump_mode)
{
#ifdef WL_PROXDETECT
	if (pi->u.pi_acphy->tof_smth_forced)
		return;
#endif

	if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
	    ACMAJORREV_5(pi->pubpi->phy_rev) || ACMAJORREV_4(pi->pubpi->phy_rev) ||
	    ACMAJORREV_3(pi->pubpi->phy_rev)) {
		phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
		uint16 SmthReg0, SmthReg1;

		/* Set the SigB to the default values */
		MOD_PHYREG(pi, musigb2, mu_sigbmcs9, 0x7);
		MOD_PHYREG(pi, musigb2, mu_sigbmcs8, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs7, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs6, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs5, 0x7);
		MOD_PHYREG(pi, musigb1, mu_sigbmcs4, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs3, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs2, 0x7);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs1, 0x3);
		MOD_PHYREG(pi, musigb0, mu_sigbmcs0, 0x2);

		pi_ac->acphy_smth_dump_mode = SMTH_NODUMP;

		switch (enable_smth) {
		case SMTH_DISABLE:
			/* Disable Smoothing and Enable SigB */
			SmthReg0 = READ_PHYREG(pi, chnsmCtrl0) & 0xFFFE;
			SmthReg1 = READ_PHYREG(pi, chnsmCtrl1);
			break;
		case SMTH_ENABLE:
			/* Enable Smoothing With all modes ON */
			/* This is the default config in which Smth is enabled for */
			/* SISO pkts and HT TxBF case. Use SigB for VHT-TxBF */
			SmthReg0 = 0x33F;
			SmthReg1 = 0x2C0;
			if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				SmthReg0 |=
					ACPHY_chnsmCtrl0_mte_pilot_enable_MASK(pi->pubpi->phy_rev);
			}
			pi_ac->acphy_smth_dump_mode = dump_mode;
			switch (dump_mode)
			{
			case SMTH_FREQDUMP:
			/* Enable Freq-domain dumping (Raw Channel Estimates) */
			SmthReg0 &= ~(
				ACPHY_chnsmCtrl0_nw_whiten_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_group_delay_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_mte_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_window_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_fft_enable_MASK(pi->pubpi->phy_rev));
			SmthReg1 &= ~(
				ACPHY_chnsmCtrl1_ifft_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev));
			break;

			case SMTH_FREQDUMP_AFTER_NW:
			/* Enable Freq-domain dumping (After NW Filtering) */
			SmthReg0 &= ~(
				ACPHY_chnsmCtrl0_group_delay_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_mte_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_window_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_fft_enable_MASK(pi->pubpi->phy_rev));
			SmthReg1 &= ~(
				ACPHY_chnsmCtrl1_ifft_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev));
			break;

			case SMTH_FREQDUMP_AFTER_GD:
			/* Enable Freq-domain dumping (After GD Compensation) */
			SmthReg0 &= ~(
				ACPHY_chnsmCtrl0_mte_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_window_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_fft_enable_MASK(pi->pubpi->phy_rev));
			SmthReg1 &= ~(
				ACPHY_chnsmCtrl1_ifft_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev));
			break;

			case SMTH_FREQDUMP_AFTER_MTE:
			/* Enable Freq-domain dumping (After MTE) */
			SmthReg0 &= ~(
				ACPHY_chnsmCtrl0_window_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_fft_enable_MASK(pi->pubpi->phy_rev));
			SmthReg1 &= ~(
				ACPHY_chnsmCtrl1_ifft_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev));
			break;

			case SMTH_TIMEDUMP_AFTER_IFFT:
			/* Enable Time-domain dumping (After IFFT) */
			SmthReg0 &= ~(
				ACPHY_chnsmCtrl0_window_enable_MASK(pi->pubpi->phy_rev) |
				ACPHY_chnsmCtrl0_fft_enable_MASK(pi->pubpi->phy_rev));
			SmthReg1 &= ~ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev);
			break;

			case SMTH_TIMEDUMP_AFTER_WIN:
				/* Enable Time-domain dumping (After Windowing) */
			SmthReg0 &= ~ACPHY_chnsmCtrl0_fft_enable_MASK(pi->pubpi->phy_rev);
			SmthReg1 &= ~ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev);
			break;

			case SMTH_FREQDUMP_AFTER_FFT:
			/* Enable Freq-domain dumping (After FFT) */
			SmthReg1 &= ~ACPHY_chnsmCtrl1_output_enable_MASK(pi->pubpi->phy_rev);
			break;
			}
			break;
		case SMTH_ENABLE_NO_NW:
			/* Enable Smoothing With all modes ON Except NW Filter */
			SmthReg0 = 0x337;
			SmthReg1 = 0x2C0;
			break;
		case SMTH_ENABLE_NO_NW_GD:
			/* Enable Smoothing With all modes ON Except NW and GD  */
			SmthReg0 = 0x327;
			SmthReg1 = 0x2C0;
			break;
		case SMTH_ENABLE_NO_NW_GD_MTE:
			/* Enable Smoothing With all modes ON Except NW, GD and  MTE */
			SmthReg0 = 0x307;
			SmthReg1 = 0x2C0;
			break;
		case DISABLE_SIGB_AND_SMTH:
			/* Disable Smoothing and SigB */
			SmthReg0 = 0x33E;
			SmthReg1 = 0x0C0;
			MOD_PHYREG(pi, musigb2, mu_sigbmcs9, 0x0);
			MOD_PHYREG(pi, musigb2, mu_sigbmcs8, 0x0);
			MOD_PHYREG(pi, musigb1, mu_sigbmcs7, 0x0);
			MOD_PHYREG(pi, musigb1, mu_sigbmcs6, 0x0);
			MOD_PHYREG(pi, musigb1, mu_sigbmcs5, 0x0);
			MOD_PHYREG(pi, musigb1, mu_sigbmcs4, 0x0);
			MOD_PHYREG(pi, musigb0, mu_sigbmcs3, 0x0);
			MOD_PHYREG(pi, musigb0, mu_sigbmcs2, 0x0);
			MOD_PHYREG(pi, musigb0, mu_sigbmcs1, 0x0);
			MOD_PHYREG(pi, musigb0, mu_sigbmcs0, 0x0);
			break;
		case SMTH_FOR_TXBF:
			/* Enable Smoothing for TxBF using Smth for HT and VHT */
			SmthReg0 = 0x33F;
			SmthReg1 = 0x6C0;
			break;
		default:
			PHY_ERROR(("wl%d: %s: Unrecognized smoothing mode: %d\n",
			          pi->sh->unit, __FUNCTION__, enable_smth));
			return;
		}
		WRITE_PHYREG(pi, chnsmCtrl0, SmthReg0);
		WRITE_PHYREG(pi, chnsmCtrl1, SmthReg1);
		pi_ac->acphy_enable_smth = enable_smth;

		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			/* 4349 specific setting */
			if (enable_smth == SMTH_ENABLE) {
				/* output_enable_new = 0x0 no output
				 * output_enable_new = 0x1 only legacy channel is smoothed
				 * output_enable_new = 0x2 only HT/VHT channel is smoothed
				 * output_enable_new = 0x3 both legacy and HT/VHT are smoothed
				 */
				/* 0x2 since TXBF doesn't work if legacy smoothing is enabled */
				MOD_PHYREG(pi, chnsmCtrl1, output_enable_new, 0x2);
			} else {
			    MOD_PHYREG(pi, chnsmCtrl1, output_enable_new, 0x0);
			}
			if ((phy_get_phymode(pi) == PHYMODE_MIMO) && (pi->sh->phyrxchain == 0x3)) {
				MOD_PHYREG(pi, chnsmCtrl1, disable_2rx_nvar_calc, 0x0);
			} else {
				MOD_PHYREG(pi, chnsmCtrl1, disable_2rx_nvar_calc, 0x1);
			}
			MOD_PHYREG(pi, nvcfg3, unity_gain_for_2x2_coremask2, 0x1);
		}

		/* set the Tiny specific filter slopes for channel smoothing */
		if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
			MOD_PHYREG(pi, chnsmCtrl5, filter_slope_20MHz, 0x2);
			MOD_PHYREG(pi, chnsmCtrl6, filter_slope_40MHz, 0x2);
			MOD_PHYREG(pi, chnsmCtrl6, filter_slope_80MHz, 0x1);
		}
	}
}

void
wlc_phy_preempt(phy_info_t *pi, bool enable_preempt)
{
	uint8 core;
	if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
	    ACMAJORREV_3(pi->pubpi->phy_rev)) {
		if (enable_preempt) {
			if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
				/* disable clip condition for norm power when preemption on */
				_PHY_REG_MOD(pi, ACPHY_MLDisableMcs(pi->pubpi->phy_rev),
				             0x0001, 0x0001);
				MOD_PHYREG(pi, Core0_BPHY_TargetVar_log2_pt8us,
				           bphy_targetVar_log2_pt8us, 479);
				WRITE_PHYREG(pi, PktAbortSupportedStates,
				             (ACREV_GE(pi->pubpi->phy_rev, 13)) ? 0x2bbf : 0x2bb7);
			} else {
				WRITE_PHYREG(pi, PktAbortSupportedStates, 0x2bb7);
				WRITE_PHYREG(pi, SpareReg, 0x3f);
			}
			WRITE_PHYREG(pi, PREMPT_per_pkt_en0, 0x1);
			WRITE_PHYREG(pi, PktAbortCtrl, 0x1841);
			WRITE_PHYREG(pi, BphyAbortExitCtrl, 0x3840);
			MOD_PHYREG(pi, RxMacifMode, AbortStatusEn, 1);
			WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_th0, 0xffff);
			WRITE_PHYREG(pi, PREMPT_ofdm_large_gain_mismatch_th0, 0x1f);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_th0, 0xffff);
			WRITE_PHYREG(pi, PREMPT_cck_large_gain_mismatch_th0, 0x1f);
			if (CHSPEC_IS5G(pi->radio_chanspec)) {
				if (CHSPEC_IS80(pi->radio_chanspec)) {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0xb0);
				} else if (CHSPEC_IS40(pi->radio_chanspec)) {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x50);
				} else {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x28);
				}
			} else {
				WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x28);
				WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_cnt_th0, 0x38);
			}
		} else {
		  /* disable Preempt */
		  MOD_PHYREG(pi, RxMacifMode, AbortStatusEn, 0);
		  MOD_PHYREG(pi, PktAbortCtrl, PktAbortEn, 0);
		  WRITE_PHYREG(pi, PREMPT_per_pkt_en0, 0x0);
		}
	} else if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* Enable Preemption */
		if (enable_preempt) {
			WRITE_PHYREG(pi, PktAbortCtrl, 0x1041);
			WRITE_PHYREG(pi, RxMacifMode, 0x0a00);

			WRITE_PHYREG(pi, PktAbortSupportedStates, 0x283f);
			WRITE_PHYREG(pi, BphyAbortExitCtrl, 0x3840);
			WRITE_PHYREG(pi, PREMPT_per_pkt_en0, 0x1);
			WRITE_PHYREG(pi, PREMPT_per_pkt_en1, 0x1);

			if (ACMINORREV_2(pi))
				WRITE_PHYREG(pi, PREMPT_per_pkt_en2, 0x1);

			WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_th0, 0xffff);
			WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_th1, 0xffff);
			WRITE_PHYREG(pi, PREMPT_ofdm_large_gain_mismatch_th0, 0x1f);
			WRITE_PHYREG(pi, PREMPT_ofdm_large_gain_mismatch_th1, 0x1f);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_th0, 0xffff);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_th1, 0xffff);
			WRITE_PHYREG(pi, PREMPT_cck_large_gain_mismatch_th0, 0x1f);
			WRITE_PHYREG(pi, PREMPT_cck_large_gain_mismatch_th1, 0x1f);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_cnt_th0, 0x30);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_cnt_th1, 0x30);
			if (CHSPEC_IS5G(pi->radio_chanspec)) {
				if (CHSPEC_IS20(pi->radio_chanspec)) {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x24);
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0x24);
				} else if (CHSPEC_IS40(pi->radio_chanspec)) {
				        WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x48);
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0x48);
				} else {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0xa0);
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0xa0);
				}
			} else {
				if (CHSPEC_IS20(pi->radio_chanspec)) {
				        WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x24);
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0x24);
				} else {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x48);
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0x48);
				}
			}
			/* Enable Preemption */
		} else {
		  /* disable Preempt */
			MOD_PHYREG(pi, RxMacifMode, AbortStatusEn, 0);
			MOD_PHYREG(pi, PktAbortCtrl, PktAbortEn, 0);
		    WRITE_PHYREG(pi, PREMPT_per_pkt_en0, 0x0);
		    WRITE_PHYREG(pi, PREMPT_per_pkt_en1, 0x0);

		    if (ACMINORREV_2(pi))
			    WRITE_PHYREG(pi, PREMPT_per_pkt_en2, 0);
		}
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* 4349 preemption settings */
		if (enable_preempt) {
			if (phy_get_phymode(pi) != PHYMODE_RSDB) {
				WRITE_PHYREG(pi, PktAbortCtrl, 0x1041);
			} else {
				WRITE_PHYREG(pi, PktAbortCtrl, 0x1841);
			}
			FOREACH_CORE(pi, core) {
				MOD_PHYREGC(pi, _BPHY_TargetVar_log2_pt8us,
					core, bphy_targetVar_log2_pt8us, 479);
			}
			WRITE_PHYREG(pi, RxMacifMode, 0x0a00);
			WRITE_PHYREG(pi, PktAbortSupportedStates, 0x2bbf);
			WRITE_PHYREG(pi, BphyAbortExitCtrl, 0x3840);
			WRITE_PHYREG(pi, PREMPT_per_pkt_en0, 0x21);
			WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_th0, 0xffff);
			WRITE_PHYREG(pi, PREMPT_ofdm_large_gain_mismatch_th0, 0x1f);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_th0, 0xffff);
			WRITE_PHYREG(pi, PREMPT_cck_large_gain_mismatch_th0, 0x1f);
			WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_cnt_th0, 0x38);
			if (phy_get_phymode(pi) != PHYMODE_RSDB) {
				WRITE_PHYREG(pi, PREMPT_per_pkt_en1, 0x21);
				WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_th1, 0xffff);
				WRITE_PHYREG(pi, PREMPT_ofdm_large_gain_mismatch_th1, 0x1f);
				WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_th1, 0xffff);
				WRITE_PHYREG(pi, PREMPT_cck_large_gain_mismatch_th1, 0x1f);
				WRITE_PHYREG(pi, PREMPT_cck_nominal_clip_cnt_th1, 0x38);
			}

			if (CHSPEC_IS5G(pi->radio_chanspec)) {
				if (CHSPEC_IS20(pi->radio_chanspec)) {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x28);
			        if (phy_get_phymode(pi) != PHYMODE_RSDB)
						WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1,
							0x28);
				} else if (CHSPEC_IS40(pi->radio_chanspec)) {
				    WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x50);
			        if (phy_get_phymode(pi) != PHYMODE_RSDB)
						WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1,
							0x50);
				} else {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0xb0);
			        if (phy_get_phymode(pi) != PHYMODE_RSDB)
						WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1,
							0xb0);
				}
			} else {
				WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x28);
			    if (phy_get_phymode(pi) != PHYMODE_RSDB)
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0x28);
				if ((CHSPEC_IS40(pi->radio_chanspec)) &&
				    (phy_get_phymode(pi) != PHYMODE_RSDB)) {
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th0, 0x50);
					WRITE_PHYREG(pi, PREMPT_ofdm_nominal_clip_cnt_th1, 0x50);
			    }
			}
			/* 4349B0: Disable SSAGC after pktabort & Power post RxFilter */
			if (ACMINORREV_2(pi)) {
				MOD_PHYREG(pi, PktAbortCounterClr, ssagc_pktabrt_enable, 0);
				MOD_PHYREG(pi, PktAbortCounterClr,
					mux_post_rxfilt_power_for_abrt, 0);
			}
		} else {
			/* Disable Preempt */
			MOD_PHYREG(pi, RxMacifMode, AbortStatusEn, 0);
		    MOD_PHYREG(pi, PktAbortCtrl, PktAbortEn, 0);
		    WRITE_PHYREG(pi, PREMPT_per_pkt_en0, 0x0);
			if (phy_get_phymode(pi) != PHYMODE_RSDB)
				WRITE_PHYREG(pi, PREMPT_per_pkt_en1, 0x0);
		}
	}
}

void
wlc_phy_lp_mode(phy_info_t *pi, int8 lp_mode)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev))	{
		if (lp_mode == 1) {
			/* AC MODE (Full Pwr mode) */
			pi_ac->acphy_lp_mode = 1;
		} else if (lp_mode == 2) {
			/* 11n MODE (VCO in 11n Mode) */
			pi_ac->acphy_lp_mode = 2;
		} else if (lp_mode == 3) {
			/* Low pwr MODE */
			pi_ac->acphy_lp_mode = 3;
		} else {
			return;
		}
	} else {
		return;
	}
}

void
wlc_phy_force_lpvco_2G(phy_info_t *pi, int8 force_lpvco_2G)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	if ACMAJORREV_1(pi->pubpi->phy_rev)	{
		pi_ac->acphy_force_lpvco_2G = force_lpvco_2G;
	} else {
		return;
	}
}

void
wlc_phy_rxcore_setstate_acphy(wlc_phy_t *pih, uint8 rxcore_bitmask)
{
	phy_info_t *pi = (phy_info_t*)pih;
	uint16 rfseqCoreActv_DisRx_save;
	uint16 rfseqMode_save;
	uint8 stall_val = 0;
	uint8 orig_rxfectrl1 = 0;
	uint16 classifier_state = 0;

	ASSERT((rxcore_bitmask > 0) && (rxcore_bitmask <= 7));
	pi->sh->phyrxchain = rxcore_bitmask;

	if (!pi->sh->clk)
		return;

	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	pi->u.pi_acphy->phyrxchain_old = READ_PHYREGFLD(pi, CoreConfig, CoreMask);

	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev) ||
		ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* Disable classifier */
		classifier_state = READ_PHYREG(pi, ClassifierCtrl);
		wlc_phy_classifier_acphy(pi, ACPHY_ClassifierCtrl_classifierSel_MASK, 4);

		/* Disable stalls and hold FIFOs in reset */
		stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
		orig_rxfectrl1 = READ_PHYREGFLD(pi, RxFeCtrl1, soft_sdfeFifoReset);
		ACPHY_DISABLE_STALL(pi);
		MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 1);
	}

	/* Save Registers */
	rfseqCoreActv_DisRx_save = READ_PHYREGFLD(pi, RfseqCoreActv2059, DisRx);
	rfseqMode_save = READ_PHYREG(pi, RfseqMode);

	/* Indicate to PHY of the Inactive Core */
	MOD_PHYREG(pi, CoreConfig, CoreMask, rxcore_bitmask);
	/* Indicate to RFSeq of the Inactive Core */
	MOD_PHYREG(pi, RfseqCoreActv2059, EnRx, rxcore_bitmask);
	/* Make sure Rx Chain gets shut off in Rx2Tx Sequence */
	MOD_PHYREG(pi, RfseqCoreActv2059, DisRx, 7);
	/* Make sure Tx Chain doesn't get turned off during this function */
	MOD_PHYREG(pi, RfseqCoreActv2059, EnTx, 0);
	MOD_PHYREG(pi, RfseqMode, CoreActv_override, 1);
	wlc_phy_set_phyctl_in_phymode_acphy(pi);
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		acphy_set_lpmode(pi, ACPHY_LP_RADIO_LVL_OPT);
	}

	wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_RX2TX);
	wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_TX2RX);

	/* Make TxEn chains point to hwphytxchain & should be subset of rxchain */
	MOD_PHYREG(pi, RfseqCoreActv2059, EnTx, pi->sh->hw_phytxchain & rxcore_bitmask);

	/*  Restore Register */
	MOD_PHYREG(pi, RfseqCoreActv2059, DisRx, rfseqCoreActv_DisRx_save);
	WRITE_PHYREG(pi, RfseqMode, rfseqMode_save);

	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev) ||
		ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* Restore FIFO reset and Stalls */
		ACPHY_ENABLE_STALL(pi, stall_val);
		MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, orig_rxfectrl1);
		OSL_DELAY(1);

		/* Restore classifier */
		WRITE_PHYREG(pi, ClassifierCtrl, classifier_state);
		OSL_DELAY(1);

		/* Reset PHY */
		wlc_phy_resetcca_acphy(pi);
	}

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* 4349 Channel Smoothing related changes */
		if ((phy_get_phymode(pi) == PHYMODE_MIMO) && (pi->sh->phyrxchain == 0x3)) {
			MOD_PHYREG(pi, chnsmCtrl1, disable_2rx_nvar_calc, 0x0);
		} else {
			MOD_PHYREG(pi, chnsmCtrl1, disable_2rx_nvar_calc, 0x1);
		}
	}

	wlapi_enable_mac(pi->sh->physhim);
}

void
wlc_phy_update_rxchains(wlc_phy_t *pih, uint8 *rxcore_bitmask, uint8 *txcore_bitmask)
{
	phy_info_t *pi = (phy_info_t*)pih;
	/* Local copy of phyrxchains before overwrite */
	*rxcore_bitmask = 0;
	/* Local copy of EnTx bits from RfseqCoreActv.EnTx */
	*txcore_bitmask = 0;
	/* Save and overwrite Rx chains */
	if (ACMAJORREV_4(pi->pubpi->phy_rev) && (phy_get_phymode(pi) != PHYMODE_RSDB)) {
		*rxcore_bitmask = pi->sh->phyrxchain;
		*txcore_bitmask = READ_PHYREGFLD(pi, RfseqCoreActv2059, EnTx);
		pi->sh->phyrxchain = pi->sh->hw_phyrxchain;
		wlc_phy_rxcore_setstate_acphy((wlc_phy_t *)pi, pi->sh->hw_phyrxchain);
	}
}

void
wlc_phy_restore_rxchains(wlc_phy_t *pih, uint8 enRx, uint8 enTx)
{
	phy_info_t *pi = (phy_info_t*)pih;
	/* Restore Rx chains */
	if (ACMAJORREV_4(pi->pubpi->phy_rev) && (phy_get_phymode(pi) != PHYMODE_RSDB)) {
		wlc_phy_rxcore_setstate_acphy((wlc_phy_t *)pi, enRx);
		MOD_PHYREG(pi, RfseqCoreActv2059, EnTx, enTx);
	}
}

uint8
wlc_phy_rxcore_getstate_acphy(wlc_phy_t *pih)
{
	uint16 rxen_bits;
	phy_info_t *pi = (phy_info_t*)pih;

	rxen_bits = READ_PHYREGFLD(pi, RfseqCoreActv2059, EnRx);

	ASSERT(pi->sh->phyrxchain == rxen_bits);

	return ((uint8) rxen_bits);
}

bool
wlc_phy_is_scan_chan_acphy(phy_info_t *pi)
{
	return (SCAN_RM_IN_PROGRESS(pi) &&
	        (pi->interf->curr_home_channel != CHSPEC_CHANNEL(pi->radio_chanspec)));
}

void
wlc_phy_resetcca_acphy(phy_info_t *pi)
{
	uint32 phy_ctl_reg_val = 0;
	/* SAVE PHY_CTL value */
	phy_ctl_reg_val = R_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param);
	/* MAC should be suspended before calling this function */
	ASSERT((R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC) == 0);

	/* bilge count sequence fix */
	if ((ACMAJORREV_1(pi->pubpi->phy_rev) &&
	     (ACMINORREV_0(pi) || ACMINORREV_1(pi))) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
		wlapi_bmac_phyclk_fgc(pi->sh->physhim, ON);

		MOD_PHYREG(pi, BBConfig, resetCCA, 1);
		OSL_DELAY(1);
		if (!TINY_RADIO(pi)) {
			MOD_PHYREG(pi, RxFeCtrl1, rxfe_bilge_cnt, 0);
			OSL_DELAY(1);
		}
		MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 1);
		OSL_DELAY(1);
		wlapi_bmac_phyclk_fgc(pi->sh->physhim, OFF);
		OSL_DELAY(1);
		MOD_PHYREG(pi, BBConfig, resetCCA, 0);
		OSL_DELAY(1);
		MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 0);
	} else {
		wlapi_bmac_phyclk_fgc(pi->sh->physhim, ON);

		/* # force gated clock on */
		W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, 0x6); /* set reg(PHY_CTL) 0x6 */
		MOD_PHYREG(pi, BBConfig, resetCCA, 1);
		OSL_DELAY(1);
		MOD_PHYREG(pi, BBConfig, resetCCA, 0);
		W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, 0x2); /* set reg(PHY_CTL) 0x2 */

		wlapi_bmac_phyclk_fgc(pi->sh->physhim, OFF);
	}

	/* wait for reset2rx finish, which is triggered by resetcca in hw */
	OSL_DELAY(2);

	/* Restore PHY_CTL register */
	W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, phy_ctl_reg_val);

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, RfseqMode, Trigger_override, 0);
	}
}

void
BCMATTACHFN(acphy_get_lpmode)(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	pi_ac->lpmode_2g = ACPHY_LPMODE_NONE;
	pi_ac->lpmode_5g = ACPHY_LPMODE_NONE;

	if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
	    ACMAJORREV_3(pi->pubpi->phy_rev)) {
		switch (BF3_ACPHY_LPMODE_2G(pi_ac)) {
			case 1:
				pi_ac->lpmode_2g = ACPHY_LPMODE_LOW_PWR_SETTINGS_1;
				break;
			case 2:
				pi_ac->lpmode_2g = ACPHY_LPMODE_LOW_PWR_SETTINGS_2;
				break;
			case 3:
				pi_ac->lpmode_2g = ACPHY_LPMODE_NORMAL_SETTINGS;
				break;
			case 0:
			default:
				pi_ac->lpmode_2g = ACPHY_LPMODE_NONE;
		}

		switch (BF3_ACPHY_LPMODE_5G(pi_ac)) {
			case 1:
				pi_ac->lpmode_5g = ACPHY_LPMODE_LOW_PWR_SETTINGS_1;
				break;
			case 2:
				pi_ac->lpmode_5g = ACPHY_LPMODE_LOW_PWR_SETTINGS_2;
				break;
			case 3:
				pi_ac->lpmode_5g = ACPHY_LPMODE_NORMAL_SETTINGS;
				break;
			case 0:
			default:
				pi_ac->lpmode_5g = ACPHY_LPMODE_NONE;
		}
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		switch (BF3_ACPHY_LPMODE_2G(pi_ac)) {
			case 1:
				pi_ac->lpmode_2g = ACPHY_LPMODE_LOW_PWR_SETTINGS_1;
				break;
			case 0:
			default:
				pi_ac->lpmode_2g = ACPHY_LPMODE_NONE;
		}

		switch (BF3_ACPHY_LPMODE_5G(pi_ac)) {
			case 1:
				pi_ac->lpmode_5g = ACPHY_LPMODE_LOW_PWR_SETTINGS_1;
				break;
			case 0:
			default:
				pi_ac->lpmode_5g = ACPHY_LPMODE_NONE;
		}

	}
}

/* 20691_lpf_tx_set is the top Tx LPF function and should be the usual */
/* function called from acphyprocs or used from the REPL in the lab */
void
wlc_phy_radio_tiny_lpf_tx_set(phy_info_t *pi, int8 bq_bw, int8 bq_gain,
	int8 rc_bw_ofdm, int8 rc_bw_cck)
{
	uint8 i, core;
	uint16 gmult;
	uint16 gmult_rc;
	uint16 g10_tuned, g11_tuned, g12_tuned, g21_tuned, bias;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	gmult = pi_ac->rccal_gmult;
	gmult_rc = pi_ac->rccal_gmult_rc;

	/* search for given bq_gain */
	for (i = 0; i < ARRAYSIZE(g_index1); i++) {
		if (bq_gain == g_index1[i])
			break;
	}

	if (i < ARRAYSIZE(g_index1)) {
		uint16 g_passive_rc_tx_tuned_ofdm, g_passive_rc_tx_tuned_cck;
		g10_tuned = (lpf_g10[bq_bw][i] * gmult) >> 15;
		g11_tuned = (lpf_g11[bq_bw] * gmult) >> 15;
		g12_tuned = (lpf_g12[bq_bw][i] * gmult) >> 15;
		g21_tuned = (lpf_g21[bq_bw][i] * gmult) >> 15;
		g_passive_rc_tx_tuned_ofdm = (g_passive_rc_tx[rc_bw_ofdm] * gmult_rc) >> 15;
		g_passive_rc_tx_tuned_cck = (g_passive_rc_tx[rc_bw_cck] * gmult_rc) >> 15;
		bias = biases[bq_bw];
		FOREACH_CORE(pi, core) {
			MOD_RADIO_REG_TINY(pi, TX_LPF_CFG3, core, lpf_g10, g10_tuned);
			MOD_RADIO_REG_TINY(pi, TX_LPF_CFG7, core, lpf_g11, g11_tuned);
			MOD_RADIO_REG_TINY(pi, TX_LPF_CFG4, core, lpf_g12, g12_tuned);
			MOD_RADIO_REG_TINY(pi, TX_LPF_CFG5, core, lpf_g21, g21_tuned);
			MOD_RADIO_REG_TINY(pi, TX_LPF_CFG6, core, lpf_g_passive_rc_tx,
				g_passive_rc_tx_tuned_ofdm);
			MOD_RADIO_REG_TINY(pi, TX_LPF_CFG8, core, lpf_bias_bq, bias);
		}

		/* Note down the values of the passive_rc for OFDM and CCK in Shmem */
		wlapi_bmac_write_shm(pi->sh->physhim, M_LPF_PASSIVE_RC_OFDM,
			g_passive_rc_tx_tuned_ofdm);
		wlapi_bmac_write_shm(pi->sh->physhim, M_LPF_PASSIVE_RC_CCK,
			g_passive_rc_tx_tuned_cck);
	} else {
		PHY_ERROR(("wl%d: %s: Invalid bq_gain %d\n", pi->sh->unit, __FUNCTION__, bq_gain));
	}
}

/* Proc to power on dac_clocks though override */
bool wlc_phy_poweron_dac_clocks(phy_info_t *pi, uint8 core, uint16 *orig_dac_clk_pu,
	uint16 *orig_ovr_dac_clk_pu)
{
	uint16 dacpwr = READ_RADIO_REGFLD_TINY(pi, TX_DAC_CFG1, core, DAC_pwrup);

	if (dacpwr == 0) {
		*orig_dac_clk_pu = READ_RADIO_REGFLD_TINY(pi, CLK_DIV_CFG1, core,
			dac_clk_pu);
		*orig_ovr_dac_clk_pu = READ_RADIO_REGFLD_TINY(pi, CLK_DIV_OVR1, core,
			ovr_dac_clk_pu);
		MOD_RADIO_REG_TINY(pi, CLK_DIV_CFG1, core, dac_clk_pu, 1);
		MOD_RADIO_REG_TINY(pi, CLK_DIV_OVR1, core, ovr_dac_clk_pu, 1);
	}

	return (dacpwr == 0);
}

/* Proc to resotre dac_clock_pu and the corresponding ovrride registers */
void wlc_phy_restore_dac_clocks(phy_info_t *pi, uint8 core, uint16 orig_dac_clk_pu,
	uint16 orig_ovr_dac_clk_pu)
{
	MOD_RADIO_REG_TINY(pi, CLK_DIV_CFG1, core, dac_clk_pu, orig_dac_clk_pu);
	MOD_RADIO_REG_TINY(pi, CLK_DIV_OVR1, core, ovr_dac_clk_pu, orig_ovr_dac_clk_pu);
}

/* 20693_dyn_papd_cfg */
static void
wlc_acphy_dyn_papd_cfg_20693(phy_info_t *pi)
{
	uint8 core;
	FOREACH_CORE(pi, core) {
		if (core == 0) {
			MOD_PHYREG(pi, dyn_radioa0, dyn_radio_ovr0, 0);
		} else {
			MOD_PHYREG(pi, dyn_radioa1, dyn_radio_ovr1, 0);
		}
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
				ovr_pa2g_idac_cas, 1);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
				ovr_pa2g_idac_incap_compen_main, 1);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
				ovr_pa2g_idac_main, 1);
		} else {
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR3, core,
				ovr_pa5g_idac_cas, 1);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core,
				ovr_pa5g_idac_incap_compen_main, 1);
			MOD_RADIO_REG_20693(pi, TX_TOP_5G_OVR2, core,
				ovr_pa5g_idac_main, 1);
		}
	}
}

static void
wlc_phy_set_bias_ipa_as_epa_acphy_20693(phy_info_t *pi, uint8 core)
{
	MOD_RADIO_REG_20693(pi, SPARE_CFG2, core,
		pa2g_bias_bw_main, 0);
	MOD_RADIO_REG_20693(pi, SPARE_CFG2, core,
		pa2g_bias_bw_cas, 0);
	MOD_RADIO_REG_20693(pi, SPARE_CFG2, core,
		pa2g_bias_bw_pmos, 0);
	MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
		ovr_pa2g_idac_main, 1);
	MOD_RADIO_REG_20693(pi, PA2G_IDAC1, core,
		pa2g_idac_main, 0x24);
	MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
		ovr_pa2g_idac_cas, 1);
	MOD_RADIO_REG_20693(pi, PA2G_IDAC1, core,
		pa2g_idac_cas, 0x22);
	MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
		ovr_pa2g_idac_incap_compen_main, 1);
	MOD_RADIO_REG_20693(pi, PA2G_INCAP, core,
		pa2g_idac_incap_compen_main, 0x2d);
	MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core,
		ovr_mx2g_idac_bbdc, 1);
	MOD_RADIO_REG_20693(pi, TXMIX2G_CFG6, core,
		mx2g_idac_bbdc, 0x1c);
	MOD_RADIO_REG_20693(pi, TXMIX2G_CFG2, core,
		mx2g_idac_cascode, 0x13);
}
static void
wlc_phy_set_phyctl_in_phymode_acphy(phy_info_t *pi)
{
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));
	if (ACMAJORREV_4(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
		si_t *sih = pi->sh->sih;
		uint phymode = (si_core_cflags(sih, 0, 0) & SICF_PHYMODE)
			>> SICF_PHYMODE_SHIFT;
		uint sicoreunit, mask = 0x6, shift = 0x1;

		if ((phy_get_phymode(pi) == PHYMODE_MIMO) ||
			(phy_get_phymode(pi) == PHYMODE_80P80)) {
			if (pi->sh->phyrxchain == 2) {
				phymode |= (0x2 << shift);
			}
		} else {
			phymode &= (~mask);
		}

		sicoreunit = si_coreunit(sih);
		si_d11_switch_addrbase(sih, 0);
		si_core_cflags(sih, SICF_PHYMODE, phymode);
		si_d11_switch_addrbase(sih, sicoreunit);
	}
}

/* Clean up chanspec */
void
chanspec_get_operating_channels(phy_info_t *pi, uint8 *ch)
{
	bool is_80p80 = FALSE;
	uint8 core;
	uint16 phymode = phy_get_phymode(pi);

	BCM_REFERENCE(is_80p80);
	for (core = 0; core < PHY_CORE_MAX; core++) {
		pi->u.pi_acphy->core_freq_mapping[core] = PRIMARY_FREQ_SEGMENT;
	}

	/* RSDB family has 80p80, need to handle carefully */
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		if (phymode == PHYMODE_80P80) {
			ch[0] = wf_chspec_primary80_channel(pi->radio_chanspec);
			ch[1] = wf_chspec_secondary80_channel(pi->radio_chanspec);
			pi->u.pi_acphy->core_freq_mapping[0] = PRIMARY_FREQ_SEGMENT;
			pi->u.pi_acphy->core_freq_mapping[1] = SECONDARY_FREQ_SEGMENT;
			is_80p80 = TRUE;
		} else {
			ch[0] = CHSPEC_CHANNEL(pi->radio_chanspec);
			ch[1] = ch[0];
		}
	} else {
		ch[0] = CHSPEC_CHANNEL(pi->radio_chanspec);
		ch[1] = 0;
	}

	PHY_INFORM(("wl%d: %s channels (%d, %d) | %s\n", PI_INSTANCE(pi), __FUNCTION__, ch[0],
		is_80p80 ? ch[1] : ch [0], is_80p80 ? "chan bonded" : "not 80p80, single chan"));
}

static void
chanspec_tune_phy_ACMAJORREV_5(phy_info_t *pi)
{
}

static void
chanspec_tune_phy_ACMAJORREV_4(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	uint8 core = 0;

	bool elna_present = (CHSPEC_IS2G(pi->radio_chanspec)) ? BF_ELNA_2G(pi_ac) :
		BF_ELNA_5G(pi_ac);

	/* setup DCC parameters */
	if (CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac) || CCT_BAND_CHG(pi_ac))
		wlc_tiny_setup_coarse_dcc(pi);

	phy_ac_spurcan(pi_ac->rxspuri, !elna_present);

	if (CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac) || CCT_BAND_CHG(pi_ac))
		wlc_phy_smth(pi, pi_ac->acphy_enable_smth, pi_ac->acphy_smth_dump_mode);


	/* 4349A0: in quickturn, disable stalls and swap iq */
	if (ISSIM_ENAB(pi->sh->sih)) {
		ACPHY_DISABLE_STALL(pi);

		MOD_PHYREG(pi, RxFeCtrl1, swap_iq0, 0x0);
		MOD_PHYREG(pi, RxFeCtrl1, swap_iq1, 0x0);
		MOD_PHYREG(pi, Core1TxControl, iqSwapEnable, 0x0);

		if (phy_get_phymode(pi) != PHYMODE_RSDB)
			MOD_PHYREG(pi, Core2TxControl, iqSwapEnable, 0x0);
	}

	wlc_dcc_fsm_reset(pi);

	if (ACMINORREV_1(pi) && (PHY_IPA(pi)) && (CHSPEC_IS2G(pi->radio_chanspec))) {
		FOREACH_CORE(pi, core) {
			MOD_RADIO_REG_20693(pi, TXMIX2G_CFG3, core, mx2g_ptat_slope_lodc, 0x0);
			MOD_RADIO_REG_20693(pi, TXMIX2G_CFG6, core, mx2g_idac_lodc, 0x22);
			MOD_RADIO_REG_20693(pi, TXMIX2G_CFG6, core, mx2g_ptat_slope_bbdc, 0x0);
			MOD_RADIO_REG_20693(pi, TXMIX2G_CFG6, core, mx2g_idac_bbdc, 0x18);
			MOD_RADIO_REG_20693(pi, PA2G_CFG3, core, pa2g_ptat_slope_main, 0x0);
			MOD_RADIO_REG_20693(pi, TX_LOGEN2G_CFG1, core, logen2g_tx_xover, 0x4);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_pa2g_idac_main, 1);
			MOD_RADIO_REG_20693(pi, PLL_CP4, core, rfpll_cp_kpd_scale, 0x5b);
			MOD_RADIO_REG_20693(pi, PLL_VCO4, core, rfpll_vco_tempco, 0x4);
			MOD_RADIO_REG_20693(pi, SPARE_CFG4, core, swcap_pri_pd_2g, 0x0);
			MOD_RADIO_REG_20693(pi, PA2G_IDAC1, core, pa2g_idac_main, 0x12);
			MOD_RADIO_REG_20693(pi, TX_TOP_2G_OVR1_EAST, core, ovr_pa2g_idac_cas, 1);
			MOD_RADIO_REG_20693(pi, PA2G_IDAC1, core, pa2g_idac_cas, 0x21);
		}
	}
}

static void
chanspec_tune_phy_ACMAJORREV_3(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* setup DCC parameters */
	if (CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac) || CCT_BAND_CHG(pi_ac))
		wlc_tiny_setup_coarse_dcc(pi);

	/* Spur war for 4345ilna */
	if (PHY_ILNA(pi))
		wlc_phy_spurwar_nvshp_acphy(pi, CCT_BW_CHG(pi_ac), TRUE, FALSE);

	if (CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac) || CCT_BAND_CHG(pi_ac))
		wlc_phy_smth(pi, pi_ac->acphy_enable_smth, pi_ac->acphy_smth_dump_mode);
}

static void
chanspec_tune_phy_ACMAJORREV_2(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if ((ACMINORREV_1(pi) || ACMINORREV_3(pi)) &&
		CHSPEC_IS2G(pi->radio_chanspec) && (BF2_2G_SPUR_WAR(pi_ac) == 1)) {
		phy_ac_dssfB(pi_ac->rxspuri, TRUE);
	}

	/* Spur war for 4350 */
	if (BF2_2G_SPUR_WAR(pi_ac) == 1)
		wlc_phy_spurwar_nvshp_acphy(pi, CCT_BW_CHG(pi_ac), TRUE, FALSE);
}

static void
chanspec_tune_phy_ACMAJORREV_1(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* Spur war for 4335 Ax/Bx IPA */
	if (PHY_ILNA(pi) && (ACMINORREV_0(pi) || ACMINORREV_1(pi))) {
		if ((BF2_2G_SPUR_WAR(pi_ac) == 1) &&
			CHSPEC_IS2G(pi->radio_chanspec)) {
			wlc_phy_spurwar_nvshp_acphy(pi, CCT_BW_CHG(pi_ac), TRUE, FALSE);
			MOD_RADIO_REG(pi, RFP, PLL_XTAL5, xtal_bufstrg_BT, 3);
			PHY_TRACE(("BT buffer 3 for Spur WAR; %s \n", __FUNCTION__));
		}
		if ((BF3_5G_SPUR_WAR(pi_ac) == 1) &&
				CHSPEC_IS5G(pi->radio_chanspec)) {
			wlc_phy_spurwar_nvshp_acphy(pi, CCT_BW_CHG(pi_ac), TRUE, FALSE);
		}
	}

	/* Spur war for 4339iLNA */
	if (PHY_ILNA(pi) && ACMINORREV_2(pi))
		wlc_phy_spurwar_nvshp_acphy(pi, CCT_BW_CHG(pi_ac), TRUE, FALSE);

	/* Nvshp for 4335 C0 ELNA, 80 MHz since tight filter is being used */
	if (ACMINORREV_2(pi) && (!(PHY_ILNA(pi)))) {
		if (CHSPEC_IS80(pi->radio_chanspec)) {
			wlc_phy_spurwar_nvshp_acphy(pi, CCT_BW_CHG(pi_ac), FALSE, TRUE);
		} else {
		/* Restoring default for 20/40 mhz by reseting it */
			if (CCT_BW_CHG(pi_ac))
				wlc_phy_reset_noise_var_shaping_acphy(pi);
		}
	}

	MOD_PHYREG(pi, RfseqMode, CoreActv_override, 0);

	if ((CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac) || CCT_BAND_CHG(pi_ac)) && ACMINORREV_2(pi))
		wlc_phy_smth(pi, pi_ac->acphy_enable_smth, pi_ac->acphy_smth_dump_mode);
}

static void
chanspec_tune_phy_ACMAJORREV_0(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* Update ucode settings based on current band/bw */
	if (CCT_INIT(pi_ac) || CCT_BAND_CHG(pi_ac) || CCT_BW_CHG(pi_ac))
		wlc_phy_hirssi_elnabypass_set_ucode_params_acphy(pi);
}

static void
chanspec_setup_phy_ACMAJORREV_5(phy_info_t *pi)
{
	MOD_PHYREG(pi, BT_SwControl, inv_btcx_prisel, 0x1);
}

static void
chanspec_setup_phy_ACMAJORREV_4(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	MOD_PHYREG(pi, BT_SwControl, inv_btcx_prisel, 0x1);

	/* Reset the TxPwrCtrl HW during the setup */
	MOD_PHYREG(pi, TxPwrCtrlCmd, txpwrctrlReset, 1);
	OSL_DELAY(10);
	MOD_PHYREG(pi, TxPwrCtrlCmd, txpwrctrlReset, 0);

	/* 4349 specific chspec initializations */
	if (CCT_INIT(pi_ac) || CCT_BAND_CHG(pi_ac) || CCT_BW_CHG(pi_ac)) {
		wlc_acphy_load_4349_specific_tbls(pi);
		wlc_acphy_dyn_papd_cfg_20693(pi);
		wlc_phy_config_bias_settings_20693(pi);
		acphy_set_lpmode(pi, ACPHY_LP_PHY_LVL_OPT);
	}
}

static void
chanspec_setup_phy_ACMAJORREV_3(phy_info_t *pi)
{
	MOD_PHYREG(pi, BT_SwControl, inv_btcx_prisel, 0x1);
}

static void
chanspec_setup_phy_ACMAJORREV_2(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	MOD_PHYREG(pi, BT_SwControl, inv_btcx_prisel, 0x1);

	if (CCT_INIT(pi_ac) && (ACMINORREV_1(pi) || ACMINORREV_3(pi)) && PHY_ILNA(pi)) {
		si_gci_chipcontrol(pi->sh->sih, CC_GCI_CHIPCTRL_06, CC_GCI_XTAL_BUFSTRG_NFC,
			(0x1 << 12));
	}
}

static void
chanspec_setup_phy_ACMAJORREV_1(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	MOD_PHYREG(pi, BT_SwControl, inv_btcx_prisel, 0x1);

	if (CCT_INIT(pi_ac) && ACMINORREV_2(pi) && PHY_ILNA(pi)) {
		si_gci_chipcontrol(pi->sh->sih, CC_GCI_CHIPCTRL_06, CC_GCI_XTAL_BUFSTRG_NFC,
			(0x1 << 12));
	}
}

static void
chanspec_setup_phy_ACMAJORREV_0(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* store/clear the hirssi(shmem) info of previous channel */
	if (wlc_phy_hirssi_elnabypass_shmem_read_clear_acphy(pi)) {
		/* Check for previous channel */
		if (pi_ac->curr_band2g) {
			if (pi_ac->hirssi_elnabyp2g_en)
				pi_ac->hirssi_timer2g = pi_ac->hirssi_period;
		} else {
			if (pi_ac->hirssi_elnabyp5g_en)
				pi_ac->hirssi_timer5g = pi_ac->hirssi_period;
		}
	}
}

static void
chanspec_setup_phy(phy_info_t *pi)
{
	if (ACMAJORREV_5(pi->pubpi->phy_rev))
		chanspec_setup_phy_ACMAJORREV_5(pi);
	else if (ACMAJORREV_4(pi->pubpi->phy_rev))
		chanspec_setup_phy_ACMAJORREV_4(pi);
	else if (ACMAJORREV_3(pi->pubpi->phy_rev))
		chanspec_setup_phy_ACMAJORREV_3(pi);
	else if (ACMAJORREV_2(pi->pubpi->phy_rev))
		chanspec_setup_phy_ACMAJORREV_2(pi);
	else if (ACMAJORREV_1(pi->pubpi->phy_rev))
		chanspec_setup_phy_ACMAJORREV_1(pi);
	else if (ACMAJORREV_0(pi->pubpi->phy_rev))
		chanspec_setup_phy_ACMAJORREV_0(pi);
	else {
		PHY_ERROR(("wl%d %s: Invalid ACMAJORREV!\n", PI_INSTANCE(pi), __FUNCTION__));
		ASSERT(0);
	}
}

static void
chanspec_setup_cmn(phy_info_t *pi)
{
	uint8 max_rxchain;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (CCT_INIT(pi_ac)) {
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			if (pi_ac->sromi->nvram_femctrl.txswctrlmap_2g) {
				pi_ac->pa_mode = (pi_ac->sromi->nvram_femctrl.txswctrlmap_2g_mask >>
					(CHSPEC_CHANNEL(pi->radio_chanspec) - 1)) & 1;
			} else {
				pi_ac->pa_mode = 0;
			}
		} else {
			pi_ac->pa_mode = pi_ac->sromi->nvram_femctrl.txswctrlmap_5g;
		}

		wlc_phy_set_reg_on_reset_acphy(pi);
		wlc_phy_set_tbl_on_reset_acphy(pi);

		/* If any rx cores were disabled before phy_init,
		 * disable them again since phy_init enables all rx cores
		 * Also make RfseqCoreActv2059.EnTx = hw_txchain & rxchain
		 */
		max_rxchain =  (1 << pi->pubpi->phy_corenum) - 1;

		if ((pi->sh->phyrxchain != max_rxchain) || (pi->sh->hw_phytxchain != max_rxchain)) {
			wlc_phy_rxcore_setstate_acphy((wlc_phy_t *)pi, pi->sh->phyrxchain);
		}
	}

	/* Set up ED thresholds */
	if (CCT_BAND_CHG(pi_ac)) {
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			if (pi_ac->sromi->ed_thresh5g) {
				wlc_phy_adjust_ed_thres_acphy(pi, &pi_ac->sromi->ed_thresh5g, TRUE);
			} else {
				wlc_phy_adjust_ed_thres_acphy(pi,
					&pi_ac->sromi->ed_thresh_default, TRUE);
			}
		} else {
			if (pi_ac->sromi->ed_thresh2g) {
				wlc_phy_adjust_ed_thres_acphy(pi, &pi_ac->sromi->ed_thresh2g, TRUE);
			} else {
				wlc_phy_adjust_ed_thres_acphy(pi,
					&pi_ac->sromi->ed_thresh_default, TRUE);
			}
		}

		/* special handling for EU region */
		if (wlc_phy_get_locale(pi->rxgcrsi) ==  REGION_EU)
			wlc_phy_set_srom_eu_edthresh_acphy(pi);
	}

	if (CCT_INIT(pi_ac) || CCT_BAND_CHG(pi_ac))
		wlc_phy_set_regtbl_on_band_change_acphy(pi);

	if (CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac))
		wlc_phy_set_regtbl_on_bw_change_acphy(pi);

	chanspec_setup_regtbl_on_chan_change(pi);
	chanspec_regtbl_fc_from_nvram(pi);
	chanspec_prefcbs_init(pi);
}

static void
chanspec_cleanup(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* Restore FIFO reset and Stalls */
	MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, pi_ac->FifoReset);

	/* reset RX */
	wlc_phy_resetcca_acphy(pi);

	/* return from Deaf */
	wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);

	/* clear Chspec Call Trace */
	CCT_CLR(pi_ac);
}

/* see chanspec_cleanup which restores some of the setup params */
static void
chanspec_setup(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* JIRA(CRDOT11ACPHY-143) - Turn off receiver during channel change */
	pi_ac->deaf_count = 0;
	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);

	/* Hold FIFOs in reset before changing channels */
	pi_ac->FifoReset = READ_PHYREGFLD(pi, RxFeCtrl1, soft_sdfeFifoReset);
	MOD_PHYREG(pi, RxFeCtrl1, soft_sdfeFifoReset, 1);

	/* update corenum and coremask state variables */
	if (ACMAJORREV_4(pi->pubpi->phy_rev))
		phy_ac_update_phycorestate(pi);

	/* BAND CHANGED ? */
	if (CCT_INIT(pi_ac) || (pi_ac->curr_band2g != CHSPEC_IS2G(pi->radio_chanspec))) {

		chanspec_setup_hirssi_ucode_cap(pi);

		pi_ac->curr_band2g = CHSPEC_IS2G(pi->radio_chanspec);

		/* indicate band change to control flow */
		mboolset(pi_ac->CCTrace, CALLED_ON_BAND_CHG);
	}

	/* BW CHANGED ? */
	if (CCT_INIT(pi_ac) || (pi_ac->curr_bw != CHSPEC_BW(pi->radio_chanspec))) {
		pi_ac->curr_bw = CHSPEC_BW(pi->radio_chanspec);

		/* If called from init, don't call this, as this is called before init */
		if (!CCT_INIT(pi_ac)) {

			/* Set the phy BW as dictated by the chspec (also calls phy_reset) */
			wlapi_bmac_bw_set(pi->sh->physhim, CHSPEC_BW(pi->radio_chanspec));

			/* bw change  do not need a phy_reset when BW_RESET == 1 */
			if (BW_RESET == 0) {
				/* indicate phy reset, follow init path to control flow */
				mboolset(pi_ac->CCTrace, CALLED_ON_INIT);
			} else {
				chanspec_sparereg_war(pi);
			}
		}

		OSL_DELAY(2);

		/* indicate bw change to control flow */
		mboolset(pi_ac->CCTrace, CALLED_ON_BW_CHG);
	}

	/* Change the band bit. Do this after phy_reset */
	if (CHSPEC_IS2G(pi->radio_chanspec))
		MOD_PHYREG(pi, ChannelControl, currentBand, 0);
	else
		MOD_PHYREG(pi, ChannelControl, currentBand, 1);
}

static void
chanspec_tune_phy(phy_info_t *pi)
{
	if (ACMAJORREV_5(pi->pubpi->phy_rev))
		chanspec_tune_phy_ACMAJORREV_5(pi);
	else if (ACMAJORREV_4(pi->pubpi->phy_rev))
		chanspec_tune_phy_ACMAJORREV_4(pi);
	else if (ACMAJORREV_3(pi->pubpi->phy_rev))
		chanspec_tune_phy_ACMAJORREV_3(pi);
	else if (ACMAJORREV_2(pi->pubpi->phy_rev))
		chanspec_tune_phy_ACMAJORREV_2(pi);
	else if (ACMAJORREV_1(pi->pubpi->phy_rev))
		chanspec_tune_phy_ACMAJORREV_1(pi);
	else if (ACMAJORREV_0(pi->pubpi->phy_rev))
		chanspec_tune_phy_ACMAJORREV_0(pi);
	else {
		PHY_ERROR(("wl%d %s: Invalid ACMAJORREV!\n", PI_INSTANCE(pi), __FUNCTION__));
		ASSERT(0);
	}
}

/* ******************** WARs ********************* */

static void
chanspec_setup_hirssi_ucode_cap(phy_info_t *pi)
{
}

static void
chanspec_sparereg_war(phy_info_t *pi)
{
	if (CHIPID(pi->sh->chip) == BCM4335_CHIP_ID &&
		CHSPEC_IS80(pi->radio_chanspec)) {

		WRITE_PHYREG(pi, SpareReg, 0xfe);
		wlc_phy_resetcca_acphy(pi);
		WRITE_PHYREG(pi, SpareReg, 0xff);

	}
}

static void
chanspec_regtbl_fc_from_nvram(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (!CCT_INIT(pi_ac) && CHSPEC_IS2G(pi->radio_chanspec) &&
		pi_ac->sromi->nvram_femctrl.txswctrlmap_2g &&
		(pi_ac->pa_mode ^ ((pi_ac->sromi->nvram_femctrl.txswctrlmap_2g_mask >>
		(CHSPEC_CHANNEL(pi->radio_chanspec) - 1)) & 1)) &&
		!ACPHY_FEMCTRL_ACTIVE(pi)) {

		pi_ac->pa_mode = (pi_ac->sromi->nvram_femctrl.txswctrlmap_2g_mask >>
			(CHSPEC_CHANNEL(pi->radio_chanspec) - 1)) & 1;

		wlc_phy_write_regtbl_fc_from_nvram(pi);
	}
}

static bool
chanspec_papr_enable(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	bool enable = FALSE;

	if (!pi_ac->srom_paprdis) {
		if (PHY_IPA(pi) && (ACMAJORREV_1(pi->pubpi->phy_rev) ||
			(ACMAJORREV_2(pi->pubpi->phy_rev) &&
			(ACMINORREV_1(pi) || ACMINORREV_3(pi))))) {
			enable = TRUE;
		} else if (!PHY_IPA(pi) && ACMAJORREV_2(pi->pubpi->phy_rev) && (ACMINORREV_1(pi))) {
			enable = TRUE;
		}
	}
	return enable;
}

static void
chanspec_tune_rxpath(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* DSSF for 4335C0 & 4345 */
	phy_ac_dssf(pi_ac->rxspuri, TRUE);

	phy_ac_rssi_init_gain_err(pi_ac->rssii);
}

static void
chanspec_tune_txpath(phy_info_t *pi)
{
	uint16 tssi_limit;
	uint8 tx_pwr_ctrl_state = PHY_TPC_HW_OFF;

	/* set txgain in case txpwrctrl is disabled */
	wlc_phy_txpwr_fixpower_acphy(pi);

	/* Disable TxPwrCtrl */
	tx_pwr_ctrl_state = pi->txpwrctrl;
	wlc_phy_txpwrctrl_enable_acphy(pi, PHY_TPC_HW_OFF);

	/* Set the TSSI visibility limits for 4360 A0/B0 */
	tssi_limit = (127 << 8) + (wlc_phy_tssivisible_thresh_acphy(pi) & 0xFF);
	ACPHYREG_BCAST(pi, TxPwrCtrlCore0TSSISensLmt, tssi_limit);

	/* Enable TxPwrCtrl */
	wlc_phy_txpwrctrl_enable_acphy(pi, tx_pwr_ctrl_state);

	chanspec_setup_papr(pi, 0, 0);
}

static void
chanspec_prefcbs_init(phy_info_t *pi)
{
#ifdef ENABLE_FCBS
	int chanidx, chanidx_current;
	chanidx = 0;
	chanidx_current = 0;

	if (IS_FCBS(pi)) {

		chanidx_current = wlc_phy_channelindicator_obtain_acphy(pi);

		for (chanidx = 0; chanidx < MAX_FCBS_CHANS; chanidx++) {
			if ((chanidx != chanidx_current) &&
			(!(pi->phy_fcbs.initialized[chanidx]))) {

				wlc_phy_prefcbsinit_acphy(pi, chanidx);

				if (CCT_INIT(pi_ac)) {
					wlc_phy_set_reg_on_reset_acphy(pi);
					wlc_phy_set_tbl_on_reset_acphy(pi);
				}

				if (CCT_BAND_CHG(pi_ac))
					wlc_phy_set_regtbl_on_band_change_acphy(pi);

				if (CCT_BW_CHG(pi_ac))
					wlc_phy_set_regtbl_on_bw_change_acphy(pi);
			}
		}

		wlc_phy_prefcbsinit_acphy(pi, chanidx_current);
	}
#endif /* ENABLE_FCBS */
}

static bool
chanspec_bbpll_parr_enable(phy_info_t *pi)
{
	return ((pi->sh->chippkg == BCM4335_WLBGA_PKG_ID &&
		((CHIPID(pi->sh->chip) == BCM4345_CHIP_ID &&
		!CST4345_CHIPMODE_USB20D(pi->sh->sih->chipst)))) ||
		(CHIPID(pi->sh->chip) == BCM4335_CHIP_ID &&
		pi->sh->chippkg == BCM4335_FCBGA_PKG_ID));
}

static void
chanspec_bbpll_parr(phy_info_t *pi, uint32 *bbpll_parr_in, bool state)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* input : bbpll_parr_in
	 * 0 : min_res_mask
	 * 1 : max_res_mask
	 * 2 : clk_ctl_st
	 */
	uint32 min_res_mask = 0, max_res_mask = 0, clk_ctl_st = 0;

	BCM_REFERENCE(min_res_mask);
	BCM_REFERENCE(max_res_mask);
	BCM_REFERENCE(clk_ctl_st);
	BCM_REFERENCE(pi_ac);

	if (!chanspec_bbpll_parr_enable(pi))
		return;

#ifdef BBPLL_PARR
	min_res_mask = bbpll_parr_in[0];
	max_res_mask = bbpll_parr_in[1];
	clk_ctl_st = bbpll_parr_in[2];

	if (state == OFF) {
		/* power down BBPLL */
		phy_ac_get_spurmode(pi_ac->rxspuri, (uint16)pi_ac->fc);
		if ((pi_ac->curr_spurmode != pi->acphy_spuravoid_mode)) {
			si_pmu_pll_off_PARR(pi->sh->sih, pi->sh->osh,
				&min_res_mask, &max_res_mask, &clk_ctl_st);
		}
	} else {
		/* update and power up BBPLL */
		if (pi_ac->curr_spurmode != pi->acphy_spuravoid_mode) {
			pi_ac->curr_spurmode =  pi->acphy_spuravoid_mode;
			si_pmu_spuravoid_isdone(pi->sh->sih, pi->sh->osh, min_res_mask,
				max_res_mask, clk_ctl_st, pi->acphy_spuravoid_mode);
			wlapi_switch_macfreq(pi->sh->physhim, pi->acphy_spuravoid_mode);
		}
	}

	bbpll_parr_in[0] = min_res_mask;
	bbpll_parr_in[1] = max_res_mask;
	bbpll_parr_in[2] = clk_ctl_st;
#endif /* BBPLL_PARR */
	return;
}

static void
chanspec_clr_olpc_dbg_mode(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	BCM_REFERENCE(pi_ac);

	/* Clearing the olpc cal done only during dbg mode */
#if defined(WLOLPC) || defined(BCMDBG) || defined(WLTEST)
	pi_ac->olpc_dbg_mode = FALSE;
#endif /* WLOLPC || BCMDBG || WLTEST */
}

/* features and WARs enable */
static void
chanspec_fw_enab(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* min_res_mask = 0, max_res_mask = 0, clk_ctl_st = 0 */
	uint32 bbpll_parr_in[3] = {0, 0, 0};

	wlapi_bmac_write_shm(pi->sh->physhim, M_PAPDOFF_MCS, pi_ac->srom_papdwar);

	/* Toggle */
	chanspec_bbpll_parr(pi, bbpll_parr_in, OFF);
	chanspec_bbpll_parr(pi, bbpll_parr_in, ON);

	chanspec_clr_olpc_dbg_mode(pi);

	/* Enable antenna diversity */
	if (wlc_phy_check_antdiv_enable_acphy(pi) &&
		(CCT_INIT(pi_ac) || CCT_BW_CHG(pi_ac) || CCT_BAND_CHG(pi_ac)) &&
		pi->sh->rx_antdiv) {
		wlc_phy_antdiv_acphy(pi, pi->sh->rx_antdiv);
	}
}

void
wlc_phy_chanspec_set_acphy(phy_info_t *pi, chanspec_t chanspec)
{
	chanspec_module_t *module = get_chanspec_module_list();

	PHY_CHANLOG(pi, __FUNCTION__, TS_ENTER, 0);

	/* sync pi->radio_chanspec with incoming chanspec */
	wlc_phy_chanspec_radio_set((wlc_phy_t *)pi, chanspec);

	/* CHANSPEC DISPATCH */
	do {
		(*module)(pi);
		++module;
	} while (*module != NULL);

	PHY_CHANLOG(pi, __FUNCTION__, TS_EXIT, 0);
}
