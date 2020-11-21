/*
 * PHY and RADIO specific portion of Broadcom BCM43XX 802.11abgn
 * Networking Device Driver.
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
 * $Id: wlc_phy_ac.c 546222 2015-04-02 18:44:46Z $
 */

#include <wlc_cfg.h>
#if (ACCONF != 0) || (ACCONF2 != 0)
#include <typedefs.h>
#include <qmath.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <bcmendian.h>
#include <wlioctl.h>
#include <wlc_phy_radio.h>
#include <bitfuncs.h>
#include <bcmdevs.h>
#include <bcmnvram.h>
#include <proto/802.11.h>
#include <hndpmu.h>
#include <bcmsrom_fmt.h>
#include <sbsprom.h>
#include <wlc_phy_hal.h>
#include <wlc_phy_int.h>
#include <wlc_phy_ac.h>
#include <sbchipc.h>
#include <phy_mem.h>
#include <bcmotp.h>
#include <phy_utils_math.h>
#include <phy_utils_status.h>
#include <phy_utils_channel.h>
#include <phy_utils_var.h>
#include <phy_utils_reg.h>
#include <phy_ac_rssi.h>
#include "wlc_phyreg_ac.h"
#include "wlc_phytbl_ac.h"
#include "wlc_phytbl_20691.h"
#include "wlc_phytbl_20693.h"
#include "wlc_radioreg_20691.h"
#include "wlc_radioreg_20693.h"
#include "wlc_phy_ac_gains.h"
#ifdef ATE_BUILD
#include <wl_ate.h>
#endif

#include <phy_ac.h>
#include <phy_ac_info.h>

#ifndef ACPHY_PAPD_EPS_TBL_SIZE
#define ACPHY_PAPD_EPS_TBL_SIZE 64
#endif

#ifndef ACPHY_PAPD_RFPWRLUT_TBL_SIZE
#define ACPHY_PAPD_RFPWRLUT_TBL_SIZE 128
#endif

#define WLC_PHY_PRECAL_TRACE(tx_idx, target_gains) \
	PHY_TRACE(("Index was found to be %d\n", tx_idx)); \
	PHY_TRACE(("Gain Code was found to be : \n")); \
	PHY_TRACE(("radio gain = 0x%x%x%x, bbm=%d, dacgn = %d  \n", \
		target_gains->rad_gain_hi, \
		target_gains->rad_gain_mi, \
		target_gains->rad_gain, \
		target_gains->bbmult, \
		target_gains->dac_gain))


typedef struct {
	acphy_txgains_t gains;
	bool useindex;
	uint8 index;
} acphy_ipa_txcalgains_t;

typedef struct acphy_papd_restore_state_t {
	uint16 fbmix[2];
	uint16 vga_master[2];
	uint16 intpa_master[2];
	uint16 afectrl[2];
	uint16 afeoverride[2];
	uint16 pwrup[2];
	uint16 atten[2];
	uint16 mm;
	uint16 tr2g_config1;
	uint16 tr2g_config1_core[2];
	uint16 tr2g_config4_core[2];
	uint16 reg10;
	uint16 reg20;
	uint16 reg21;
	uint16 reg29;
} acphy_papd_restore_state;


typedef struct _acphy_ipa_txrxgain {
	uint16 hpvga;
	uint16 lpf_biq1;
	uint16 lpf_biq0;
	uint16 lna2;
	uint16 lna1;
	int8 txpwrindex;
} acphy_ipa_txrxgain_t;

typedef struct {
	uint8 percent;
	uint8 g_env;
} acphy_txiqcal_ladder_t;

typedef struct {
	uint8 nwords;
	uint8 offs;
	uint8 boffs;
} acphy_coeff_access_t;

typedef struct {
	int32 i_accum;
	int32 q_accum;
} phy_hpf_dc_est_t;

#define TXFILT_SHAPING_OFDM20   0
#define TXFILT_SHAPING_OFDM40   1
#define TXFILT_SHAPING_CCK      2
#define TXFILT_DEFAULT_OFDM20   3
#define TXFILT_DEFAULT_OFDM40   4

uint16 papd_gainctrl_pga[PHY_CORE_MAX];

/*
 * ATE want this global variable so they can check the 4345 sample capture timeouts.
 * And we also want to limit the number of retires.
 */
uint32 sample_capture_pointer_timeouts;

#ifdef WL_PROXDETECT
#define TOF_INITIATOR_K_4345_80M	34434 /* initiator K value for 80M */
#define TOF_TARGET_K_4345_80M		34474 /* target K value for 80M */
#define TOF_INITIATOR_K_4345_40M	35214 /* initiator K value for 40M */
#define TOF_TARGET_K_4345_40M		35214 /* target K value for 40M */
#define TOF_INITIATOR_K_4345_20M	36553 /* initiator K value for 20M */
#define TOF_TARGET_K_4345_20M		36553 /* target K value for 20M */
#define TOF_INITIATOR_K_4345_2G		37169 /* initiator K value for 2G */
#define TOF_TARGET_K_4345_2G		37169 /* target K value for 2G */

static const uint16 proxd_4345_80m_k_values[] =
{0x0, 0xee12, 0xe201, 0xe4fc, 0xe6f8, 0xe6f7 /* 42, 58, 106, 122, 138, 155 */};

static const uint16 proxd_4345_40m_k_values[] =
{0x7b7b, 0x757c, 0x7378, 0x7074, 0x9a9a, 0x9898, /* 38, 46, 54, 62, 102,110 */
0x9898, 0x9898, 0x9393, 0x9494, 0x9191, 0x8484 /* 118, 126, 134, 142,151,159 */};

static const uint16 proxd_4345_20m_k_values[] =
{0x0f0f, 0x0101, 0x0101, 0x1313, 0x0f0f, 0x0101, 0x0f0f, 0x0505, /* 36 -64 */
0xe9e9, 0xe8e8, 0xe6e6, 0xe4e4, 0xcbcb, 0xcbcb, 0xcbcb, 0xcbcb, /* 100 -128 */
0xcbcb, 0xd5d5, 0xdada, 0xcbcb, /* 132 -144 */
0xcbcb, 0xbfbf, 0xd5d5, 0xbfbf, 0xcbcb /* 149 - 165 */
};

static const uint16 proxd_4345_2g_k_values[] =
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, /* 1 -7 */
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 /* 8 -14 */
};

#define TOF_INITIATOR_K_4350_80M	36546 /* initiator K value for 80M */
#define TOF_TARGET_K_4350_80M		36569 /* target K value for 80M */
#define TOF_INITIATOR_K_4350_40M	35713 /* initiator K value for 40M */
#define TOF_TARGET_K_4350_40M		35713 /* target K value for 40M */
#define TOF_INITIATOR_K_4350_20M	37733 /* initiator K value for 20M */
#define TOF_TARGET_K_4350_20M		37733 /* target K value for 20M */
#define TOF_INITIATOR_K_4350_2G		37733 /* initiator K value for 2G */
#define TOF_TARGET_K_4350_2G		37733 /* target K value for 2G */

static const uint16 proxd_4350_80m_k_values[] =
{0x0, 0xef02, 0xf404, 0xf704, 0xfc04, 0xf3f9 /* 42, 58, 106, 122, 138, 155 */};

static const uint16 proxd_4350_40m_k_values[] =
{0x0, 0xfdfd, 0xf6f6, 0x1414, 0xebeb, 0xebeb, /* 38, 46, 54, 62, 102,110 */
0xeeee, 0xeeee, 0xe2e2, 0xe5e5, 0xfdfa, 0xe5e5 /* 118, 126, 134, 142,151,159 */};

static const uint16 proxd_4350_20m_k_values[] =
{0x0, 0xfdfd, 0xfdfd, 0xf8f8, 0xf8f8, 0xf5f5, 0xf5f5, 0xf5f5, /* 36 -64 */
0xe9e9, 0xe6e6, 0xe3e3, 0xe3e3, 0xe6e6, 0xe6e6, 0xe6e6, 0xe6e6, /* 100 - 128 */
0xe6e6, 0xe6e6, 0xd6d6, 0xe9e9, /* 132 -144 */
0xe9e9, 0x0808, 0xd4d4, 0xd4d4, 0xd4d4 /* 149 - 165 */
};

static const uint16 proxd_4350_2g_k_values[] =
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, /* 1 -7 */
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 /* 8 -14 */
};

#define TOF_INITIATOR_K_4354_80M	36553 /* initiator K value for 80M */
#define TOF_TARGET_K_4354_80M		36559 /* target K value for 80M */
#define TOF_INITIATOR_K_4354_40M	35699 /* initiator K value for 40M */
#define TOF_TARGET_K_4354_40M		35713 /* target K value for 40M */
#define TOF_INITIATOR_K_4354_20M	37723 /* initiator K value for 20M */
#define TOF_TARGET_K_4354_20M		37728 /* target K value for 20M */
#define TOF_INITIATOR_K_4354_2G		37816 /* initiator K value for 2G */
#define TOF_TARGET_K_4354_2G		37816 /* target K value for 2G */

static const uint16 proxd_4354_80m_k_values[] =
{0, 0xF0F3, 0xFCFE, 0, 0xFEFE, 0xF0F4 /* 42, 58, 106, 122, 138, 155 */};

static const uint16 proxd_4354_40m_k_values[] =
{0, 0xFC04, 0xFA00, 0x0d1D, 0xF1FB, 0xF0FB, /* 38, 46, 54, 62, 102 */
0, 0, 0xE9F5, 0xE5F4, 0xFE0B, 0xE0EF /* 110, 118, 126, 134, 142,151,159 */};

static const uint16 proxd_4354_20m_k_values[] =
{0, 0xFFFE, 0xFAFB, 0xFBFA, 0xF9F8, 0xFAF7, 0xF4F4, 0xFDEC, /* 36 -64 */
0xEEE1, 0xE3E3, 0xE2E0, 0xE2E2, 0, 0, 0, 0, 0xD9DB, 0xDEDD, 0xD7D7, 0xD9D6, /* 100 -144 */
0x0BFD, 0x0209, 0xD6D1, 0xE1C6, 0xE1C6 /* 149 - 165 */
};

static const uint16 proxd_4354_2g_k_values[] =
{0xbbbb, 0xf1f1, 0xebeb, 0xe5e5, 0xe6e6, 0xe6e6, 0xe3e3, /* 1 -7 */
0xe6e6, 0xe3e3, 0xe3e3, 0xe6e6, 0xb5b5, 0xf0f0, 0x0c0c /* 8 -14 */
};

#define TOF_INITIATOR_K_4349_80M	35989 /* initiator K value for 80M */
#define TOF_TARGET_K_4349_80M		35991 /* target K value for 80M */
#define TOF_INITIATOR_K_4349_40M	35527 /* initiator K value for 40M */
#define TOF_TARGET_K_4349_40M		35529 /* target K value for 40M */
#define TOF_INITIATOR_K_4349_20M	37466 /* initiator K value for 20M */
#define TOF_TARGET_K_4349_20M		37470 /* target K value for 20M */
#define TOF_INITIATOR_K_4349_2G		37550 /* initiator K value for 2G */
#define TOF_TARGET_K_4349_2G		37550 /* initiator K value for 2G */

static const uint16 proxd_4349_80m_k_values[] =
{0, 0xF7F8, 0xFAE5, 0xFAE5, 0xEBEC, 0xE3E5 /* 42, 58, 106, 122, 138, 155 */};

static const uint16 proxd_4349_40m_k_values[] =
{0, 0xFFFF, 0x0002, 0xFBFB, 0xFDD5, 0xEDED, /* 38, 46, 54, 62, 102 110 */
0xEDED, 0xEDED, 0xE1E1, 0xF8D0, 0xF0D2, 0xDEDF /* 118, 126, 134, 142,151,159 */};

static const uint16 proxd_4349_20m_k_values[] =
{0, 0x0, 0x0300, 0x0300, 0xFCFF, 0xFAFC, 0xF9F9, 0xF9F7, /* 36 -64 */
0xEFEE, 0xE8EC, 0xE9E9, 0xE6E8, 0xCDCE, 0xCDCE, 0xCDCE, 0xCDCE, 0xCCCF, 0xCCCF,  /* 100 -136 */
0xC8CC, 0xC6CA, 0xC6CA, 0xC5C8, 0xC6C7, 0xC5C6, 0xC5C6 /* 140 - 165 */
};

static const uint16 proxd_4349_2g_k_values[] =
{0x2222, 0x1e1e, 0x1919, 0x1313, 0x0f0f, 0x0e0e, 0x0f0f, /* 1 -7 */
0x1111, 0x1313, 0x1313, 0x1515, 0x1818, 0x1d1d, 0x3939 /* 8 -14 */
};

/* ratespec related k offset table <legacy 6M, legacy non-6M, HT-MCS0, HT-MCS(1-7)> */
static const int16 proxd_rate_offset_2g[] = { 2333, 8034, 1197, 4038 };
static const int16 proxd_rate_offset_20m[] = { 2360, 8064, 1298, 4068 };
static const int16 proxd_rate_offset_40m[] = { 2099, 8439, 1053, 4216 };
static const int16 proxd_rate_offset_80m[] = { 2804, 8102, 0, 0 };

/* different bandwidth  k offset table <VHT legacy 6M, legacy non-6M, HT-MCS0, HT-MCS(1-7)> */
static const int16 proxd_subbw_offset [3][5] = {
	/* 80M-40M */
	{1036, 325, 1366, 2094, 5273},
	/* 80M -20M */
	{1470, -170, 1400, 1412, 5507},
	/* 40M - 20M */
	{200, -714, -216, -800, 20}
};
#endif /* WL_PROXDETECT */

#define ACPHY_IQCAL_TONEFREQ_80MHz 8000
#define ACPHY_IQCAL_TONEFREQ_40MHz 4000
#define ACPHY_IQCAL_TONEFREQ_20MHz 2000

#define CAL_TYPE_IQ                 0
#define CAL_TYPE_LOFT_DIG           2
#define CAL_TYPE_LOFT_ANA_FINE      3
#define CAL_TYPE_LOFT_ANA_COARSE    4

#define TXMAC_IFHOLDOFF_DEFAULT		0x12	/* 9.0us */
#define TXMAC_MACDELAY_DEFAULT		0x2a8	/* 8.5us */


#define MAX_PAD_GAIN				0xFF
#define MAX_TX_IDX				127

/* %%%%%% function declaration */

/* wd handle */
typedef void (*wd_module_t)(phy_info_t *pi);

/* wd accessor fn() */
wd_module_t * BCMRAMFN(get_wd_module_list)(void);
wd_module_t * BCMRAMFN(get_wd_feature_list)(void);

/* helper functions */
static void phy_ac_wd_btcx_desense(phy_info_t *pi);
static void phy_ac_wd_wfd_ll(phy_info_t *pi);
static void phy_ac_wd_hirssi_elnabypass_engine(phy_info_t *pi);
static void phy_ac_wd_update_features(phy_info_t *pi);
static void phy_ac_wd_periodic_calibs(phy_info_t *pi);
static void phy_ac_wd_report_rssi(phy_info_t *pi);
static void phy_ac_wd_wars(phy_info_t *pi);

/* wd dispatch */
wd_module_t wd_feature_list[] = {
	phy_ac_wd_btcx_desense,
	phy_ac_wd_wfd_ll,
	phy_ac_wd_hirssi_elnabypass_engine,
	NULL
};

wd_module_t wd_module_list[] = {
	phy_ac_wd_update_features,
	phy_ac_wd_periodic_calibs,
	phy_ac_wd_report_rssi,
	phy_ac_wd_wars,
	NULL
};

static void wlc_phy_watchdog_acphy(phy_info_t *pi);

static void wlc_phy_cals_mac_susp_en_other_cr(phy_info_t *pi, bool suspend);

/* 2069 related Radio Functions */

/* 20691  Radio Functions */
static void wlc_phy_cal_init_acphy(phy_info_t *pi);
static void wlc_phy_write_tx_farrow_tiny(phy_info_t *pi, chanspec_t chanspec);
static uint16 wlc_phy_gen_load_samples_acphy(phy_info_t *pi, int32 f_kHz, uint16 max_val,
                                             uint8 mac_based);
int wlc_phy_tx_tone_acphy(phy_info_t *pi, int32 f_kHz, uint16 max_val, uint8 iqmode,
                                     uint8 mac_based, bool modify_bbmult);
void wlc_phy_stopplayback_acphy(phy_info_t *pi);

/* function to read femctrl params from nvram */
static void wlc_phy_nvram_femctrl_read(phy_info_t *pi);
static void wlc_phy_nvram_rssioffset_read_sub(phy_info_t *pi);
static void wlc_phy_nvram_rssioffset_read(phy_info_t *pi);
static void wlc_phy_nvram_avvmid_read(phy_info_t *pi);
static void  wlc_phy_nvram_vlin_params_read(phy_info_t *pi);

#ifdef WL_SAR_SIMPLE_CONTROL
static void wlc_phy_nvram_dynamicsarctrl_read(phy_info_t *pi);
#endif /* WL_SAR_SIMPLE_CONTROL */

void wlc_phy_init_test_acphy(phy_info_t *pi);

#if defined(PHYCAL_CACHING)
#ifdef WLOLPC
static int8 wlc_phy_olpcthresh(void);
#endif /* WLOLPC */
#endif /* PHYCAL_CACHING */
void wlc_phy_get_rxgain_acphy(phy_info_t *pi, rxgain_t rxgain[], int16 *tot_gain,
                              uint8 force_gain_type);
static bool wlc_phy_srom_read_acphy(phy_info_t *pi);
static void wlc_phy_srom_read_gainctrl_acphy(phy_info_t *pi);
uint8 wlc_phy_calc_extra_init_gain_acphy(phy_info_t *pi, uint8 extra_gain_3dB, rxgain_t rxgain[]);
static void wlc_phy_susp2tx_cts2self(phy_info_t *pi, uint16 duration);

#if defined(PHYCAL_CACHING) && defined(BCMDBG)
static void wlc_phy_cal_cache_dbg_acphy(wlc_phy_t *pih, ch_calcache_t *ctx);
#endif /* PHYCAL_CACHING && BCMDBG */
static int wlc_phy_tiny_static_dc_offset_cal(phy_info_t *pi);
static void wlc_rx_digi_dccomp_set(phy_info_t *pi, int16 i, int16 q, uint8 core);
static void wlc_dcc_fsm_restart(phy_info_t *pi);

#if (defined(WLTEST) || defined(WLPKTENG))
static bool wlc_phy_isperratedpden_acphy(phy_info_t *pi);
static void wlc_phy_perratedpdset_acphy(phy_info_t *pi, bool enable);
#endif

#if defined(WL_PROXDETECT)
static int wlc_phy_tof_seq_params_acphy(phy_info_t *pi);
#endif /* defined(WL_PROXDETECT) */

/* ============= attach submodules ================ */
static bool
BCMATTACHFN(wlc_phy_attach_farrow)(phy_info_t *pi);
static bool
BCMATTACHFN(wlc_phy_attach_chan_tuning_tbl)(phy_info_t *pi);

static void BCMATTACHFN(wlc_phy_fptr_attach_acphy)(phy_info_t *pi);
static bool BCMATTACHFN(wlc_phy_nvram_attach_acphy)(phy_info_t *pi);
static void BCMATTACHFN(wlc_phy_std_params_attach_acphy)(phy_info_t *pi);

/* reclaim strings that are only used in attach functions */
static const char BCMATTACHDATA(rstr_pagc2g)[] = "pagc2g";
static const char BCMATTACHDATA(rstr_pagc5g)[] = "pagc5g";
static const char BCMATTACHDATA(rstr_rpcal2g)[] = "rpcal2g";
static const char BCMATTACHDATA(rstr_femctrl)[] = "femctrl";
static const char BCMATTACHDATA(rstr_papdmode)[] = "papdmode";
static const char BCMATTACHDATA(rstr_pdgain2g)[] = "pdgain2g";
static const char BCMATTACHDATA(rstr_pdgain5g)[] = "pdgain5g";
static const char BCMATTACHDATA(rstr_epacal2g)[] = "epacal2g";
static const char BCMATTACHDATA(rstr_epacal5g)[] = "epacal5g";
static const char BCMATTACHDATA(rstr_offtgpwr)[] = "offtgpwr";
static const char BCMATTACHDATA(rstr_epagain2g)[] = "epagain2g";
static const char BCMATTACHDATA(rstr_epagain5g)[] = "epagain5g";
static const char BCMATTACHDATA(rstr_rpcal5gb0)[] = "rpcal5gb0";
static const char BCMATTACHDATA(rstr_rpcal5gb1)[] = "rpcal5gb1";
static const char BCMATTACHDATA(rstr_rpcal5gb2)[] = "rpcal5gb2";
static const char BCMATTACHDATA(rstr_rpcal5gb3)[] = "rpcal5gb3";
static const char BCMATTACHDATA(rstr_txidxcap2g)[] = "txidxcap2g";
static const char BCMATTACHDATA(rstr_txidxcap5g)[] = "txidxcap5g";
static const char BCMATTACHDATA(rstr_extpagain2g)[] = "extpagain2g";
static const char BCMATTACHDATA(rstr_extpagain5g)[] = "extpagain5g";
static const char BCMATTACHDATA(rstr_boardflags3)[] = "boardflags3";
static const char BCMATTACHDATA(rstr_pacalshift2g)[] = "pacalshift2g";
static const char BCMATTACHDATA(rstr_pacalshift5g)[] = "pacalshift5g";
static const char BCMATTACHDATA(rstr_pacalindex2g)[] = "pacalindex2g";
static const char BCMATTACHDATA(rstr_pacalindex5g)[] = "pacalindex5g";
static const char BCMATTACHDATA(rstr_txiqcalidx2g)[] = "txiqcalidx2g";
static const char BCMATTACHDATA(rstr_txiqcalidx5g)[] = "txiqcalidx5g";
static const char BCMATTACHDATA(rstr_pacalpwr2g)[] = "pacalpwr2g";
static const char BCMATTACHDATA(rstr_pacalpwr5g)[] = "pacalpwr5g";
static const char BCMATTACHDATA(rstr_txgaintbl5g)[] = "txgaintbl5g";
static const char BCMATTACHDATA(rstr_pacalpwr5g40)[] = "pacalpwr5g40";
static const char BCMATTACHDATA(rstr_pacalpwr5g80)[] = "pacalpwr5g80";
static const char BCMATTACHDATA(rstr_parfps2g)[] = "parfps2g";
static const char BCMATTACHDATA(rstr_parfps5g)[] = "parfps5g";
static const char BCMATTACHDATA(rstr_papdbbmult2g)[] = "papdbbmult2g";
static const char BCMATTACHDATA(rstr_papdbbmult5g)[] = "papdbbmult5g";
static const char BCMATTACHDATA(rstr_pacalmode)[] = "pacalmode";
static const char BCMATTACHDATA(rstr_pacalopt)[] = "pacalopt";
static const char BCMATTACHDATA(rstr_patoneidx2g)[] = "patoneidx2g";
static const char BCMATTACHDATA(rstr_patoneidx5g)[] = "patoneidx5g";
static const char BCMATTACHDATA(rstr_subband5gver)[] = "subband5gver";
static const char BCMATTACHDATA(rstr_dacratemode2g)[] = "dacratemode2g";
static const char BCMATTACHDATA(rstr_dacratemode5g)[] = "dacratemode5g";
static const char BCMATTACHDATA(rstr_vcodivmode)[] = "vcodivmode";
static const char BCMATTACHDATA(rstr_fdss_interp_en)[] = "fdss_interp_en";
static const char BCMATTACHDATA(rstr_fdss_level_2g)[] = "fdss_level_2g";
static const char BCMATTACHDATA(rstr_fdss_level_5g)[] = "fdss_level_5g";
static const char BCMATTACHDATA(rstr_epacal2g_mask)[] = "epacal2g_mask";
static const char BCMATTACHDATA(rstr_cckdigfilttype)[] = "cckdigfilttype";
static const char BCMATTACHDATA(rstr_ofdmfilttype_5gbe)[] = "ofdmfilttype_5gbe";
static const char BCMATTACHDATA(rstr_ofdmfilttype_2gbe)[] = "ofdmfilttype_2gbe";
static const char BCMATTACHDATA(rstr_tworangetssi2g)[] = "tworangetssi2g";
static const char BCMATTACHDATA(rstr_tworangetssi5g)[] = "tworangetssi5g";
static const char BCMATTACHDATA(rstr_lowpowerrange2g)[] = "lowpowerrange2g";
static const char BCMATTACHDATA(rstr_lowpowerrange5g)[] = "lowpowerrange5g";
static const char BCMATTACHDATA(rstr_paprdis)[] = "paprdis";
static const char BCMATTACHDATA(rstr_papdwar)[] = "papdwar";
static const char BCMATTACHDATA(rstr_tssisleep_en)[] = "tssisleep_en";
static const char BCMATTACHDATA(ed_thresh2g)[] = "ed_thresh2g";
static const char BCMATTACHDATA(ed_thresh5g)[] = "ed_thresh5g";
static const char BCMATTACHDATA(rstr_bphyscale)[] = "bphyscale";
static const char BCMATTACHDATA(rstr_antdiv_rfswctrlpin_a0)[]         = "antdiv_rfswctrlpin_a0";
static const char BCMATTACHDATA(rstr_antdiv_rfswctrlpin_a1)[]         = "antdiv_rfswctrlpin_a1";
static const char BCMATTACHDATA(rstr_rxgains2gelnagainaD)[]           = "rxgains2gelnagaina%d";
static const char BCMATTACHDATA(rstr_rxgains2gtrelnabypaD)[]          = "rxgains2gtrelnabypa%d";
static const char BCMATTACHDATA(rstr_rxgains2gtrisoaD)[]              = "rxgains2gtrisoa%d";
static const char BCMATTACHDATA(rstr_rxgains5gelnagainaD)[]           = "rxgains5gelnagaina%d";
static const char BCMATTACHDATA(rstr_rxgains5gtrelnabypaD)[]          = "rxgains5gtrelnabypa%d";
static const char BCMATTACHDATA(rstr_rxgains5gtrisoaD)[]              = "rxgains5gtrisoa%d";
static const char BCMATTACHDATA(rstr_rxgains5gmelnagainaD)[]          = "rxgains5gmelnagaina%d";
static const char BCMATTACHDATA(rstr_rxgains5gmtrelnabypaD)[]         = "rxgains5gmtrelnabypa%d";
static const char BCMATTACHDATA(rstr_rxgains5gmtrisoaD)[]             = "rxgains5gmtrisoa%d";
static const char BCMATTACHDATA(rstr_rxgains5ghelnagainaD)[]          = "rxgains5ghelnagaina%d";
static const char BCMATTACHDATA(rstr_rxgains5ghtrelnabypaD)[]         = "rxgains5ghtrelnabypa%d";
static const char BCMATTACHDATA(rstr_rxgains5ghtrisoaD)[]             = "rxgains5ghtrisoa%d";
static const char BCMATTACHDATA(rstr_VlinPwr2g_cD)[]                  = "VlinPwr2g_c%d";
static const char BCMATTACHDATA(rstr_VlinPwr5g_cD)[]                  = "VlinPwr5g_c%d";
static const char BCMATTACHDATA(rstr_Vlinmask2g_cD)[]                 = "Vlinmask2g_c%d";
static const char BCMATTACHDATA(rstr_Vlinmask5g_cD)[]                 = "Vlinmask5g_c%d";
static const char BCMATTACHDATA(rstr_rawtempsense)[]                  = "rawtempsense";
static const char BCMATTACHDATA(rstr_rxgainerr2ga0)[]                 = "rxgainerr2ga0";
static const char BCMATTACHDATA(rstr_rxgainerr2ga1)[]                 = "rxgainerr2ga1";
static const char BCMATTACHDATA(rstr_rxgainerr2ga2)[]                 = "rxgainerr2ga2";
static const char BCMATTACHDATA(rstr_rxgainerr5ga0)[]                 = "rxgainerr5ga0";
static const char BCMATTACHDATA(rstr_rxgainerr5ga1)[]                 = "rxgainerr5ga1";
static const char BCMATTACHDATA(rstr_rxgainerr5ga2)[]                 = "rxgainerr5ga2";
static const char BCMATTACHDATA(rstr_noiselvl2gaD)[]                  = "noiselvl2ga%d";
static const char BCMATTACHDATA(rstr_noiselvl5gaD)[]                  = "noiselvl5ga%d";
static const char BCMATTACHDATA(rstr_swctrlmap_2g)[]                  = "swctrlmap_2g";
static const char BCMATTACHDATA(rstr_swctrlmap_5g)[]                  = "swctrlmap_5g";
static const char BCMATTACHDATA(rstr_swctrlmapext_2g)[]               = "swctrlmapext_2g";
static const char BCMATTACHDATA(rstr_swctrlmapext_5g)[]               = "swctrlmapext_5g";
static const char BCMATTACHDATA(rstr_txswctrlmap_2g)[]                = "txswctrlmap_2g";
static const char BCMATTACHDATA(rstr_txswctrlmap_2g_mask)[]           = "txswctrlmap_2g_mask";
static const char BCMATTACHDATA(rstr_txswctrlmap_5g)[]                = "txswctrlmap_5g";
static const char BCMATTACHDATA(rstr_rxgaintempcoeff2g)[]             = "rstr_rxgaintempcoeff2g";
static const char BCMATTACHDATA(rstr_rxgaintempcoeff5gl)[]            = "rstr_rxgaintempcoeff5gl";
static const char BCMATTACHDATA(rstr_rxgaintempcoeff5gml)[]           = "rstr_rxgaintempcoeff5gml";
static const char BCMATTACHDATA(rstr_rxgaintempcoeff5gmu)[]           = "rstr_rxgaintempcoeff5gmu";
static const char BCMATTACHDATA(rstr_rxgaintempcoeff5gh)[]            = "rstr_rxgaintempcoeff5gh";
static const char BCMATTACHDATA(rstr_rssicorrnorm_cD)[]               = "rssicorrnorm_c%d";
static const char BCMATTACHDATA(rstr_rssicorrnorm5g_cD)[]             = "rssicorrnorm5g_c%d";
static const char BCMATTACHDATA(rstr_rssi_delta_2g_cD)[]              = "rssi_delta_2g_c%d";
static const char BCMATTACHDATA(rstr_rssi_delta_5gS_cD)[]             = "rssi_delta_5g%s_c%d";
static const char BCMATTACHDATA(rstr_gain_cal_temp)[]                 = "gain_cal_temp";
static const char BCMATTACHDATA(rstr_rssi_cal_rev)[]                  = "rssi_cal_rev";
static const char BCMATTACHDATA(rstr_rxgaincal_rssical)[]             = "rxgaincal_rssical";
static const char BCMATTACHDATA(rstr_rud_agc_enable)[]                = "rud_agc_enable";
static const char BCMATTACHDATA(rstr_rssi_delta_2gS)[]                = "rssi_delta_2g%s";
static const char BCMATTACHDATA(rstr_rssi_delta_5gS)[]                = "rssi_delta_5g%s";
static const char BCMATTACHDATA(rstr_rssi_cal_freq_grp_2g)[]          = "rssi_cal_freq_grp_2g";
static const char BCMATTACHDATA(rstr_AvVmid_cD)[]                     = "AvVmid_c%d";

#if (defined(WLTEST) || defined(WLPKTENG))
static const char BCMATTACHDATA(rstr_perratedpd2g)[] = "perratedpd2g";
static const char BCMATTACHDATA(rstr_perratedpd5g)[] = "perratedpd5g";
#endif

#ifdef ATE_BUILD
static void wlc_phy_gpaio_acphy(phy_info_t *pi, wl_gpaio_option_t option, int core);
#endif

/* PA Mode related configuration functions */
static void wlc_txswctrlmap_set_acphy(phy_info_t *pi, int8 pamode_requested);
static int8 wlc_txswctrlmap_get_acphy(phy_info_t *pi);

#if defined(WLTEST) || defined(BCMDBG)
static void wlc_phy_epa_dpd_set_acphy(phy_info_t *pi, uint8 enab_epa_dpd, bool in_2g_band);
#endif

/* ============= Function Definitions ============= */
bool
BCMATTACHFN(wlc_phy_attach_acphy)(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac;

	acphy_txcal_radioregs_t *ac_txcal_radioregs_orig = NULL;
	acphy_rxcal_phyregs_t   *ac_rxcal_phyregs_orig = NULL;
	acphy_desense_values_t *curr_desense = NULL, *zero_desense = NULL, *total_desense = NULL;
	acphy_hwaci_setup_t *hwaci_args = NULL;
	acphy_desense_values_t *bt_desense = NULL;
	acphy_rx_fdiqi_ctl_t *fdiqi = NULL;
	phy_pwr_ctrl_s *pwr_ctrl_save = NULL;
	pi->u.pi_acphy = (phy_info_acphy_t*)MALLOC(pi->sh->osh, sizeof(phy_info_acphy_t));

	if (pi->u.pi_acphy == NULL) {
		PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n", pi->sh->unit,
		           __FUNCTION__, MALLOCED(pi->sh->osh)));
		return FALSE;
	}
	bzero((char *)pi->u.pi_acphy, sizeof(phy_info_acphy_t));
	pi_ac = pi->u.pi_acphy;
	if ((ac_txcal_radioregs_orig = phy_malloc(pi, sizeof(acphy_txcal_radioregs_t))) == NULL) {
		PHY_ERROR(("%s: ac_txcal_radioregs_orig malloc failed\n", __FUNCTION__));
		return FALSE;
	}

	if ((ac_rxcal_phyregs_orig = phy_malloc(pi, sizeof(acphy_rxcal_phyregs_t))) == NULL) {
		PHY_ERROR(("%s: ac_rxcal_phyregs_orig malloc failed\n", __FUNCTION__));
		return FALSE;
	}

	if ((curr_desense = phy_malloc(pi, sizeof(acphy_desense_values_t))) == NULL) {
		PHY_ERROR(("%s: curr_desense malloc failed\n", __FUNCTION__));
		return FALSE;
	}

	if ((zero_desense = phy_malloc(pi, sizeof(acphy_desense_values_t))) == NULL) {
		PHY_ERROR(("%s: zero_desense malloc failed\n", __FUNCTION__));
		return FALSE;
	}

	if ((total_desense = phy_malloc(pi, sizeof(acphy_desense_values_t))) == NULL) {
		PHY_ERROR(("%s: total_desense malloc failed\n", __FUNCTION__));
		return FALSE;
	}

#ifndef WLC_DISABLE_ACI
	if ((hwaci_args = phy_malloc(pi, sizeof(acphy_hwaci_setup_t))) == NULL) {
		PHY_ERROR(("%s: hwaci_args malloc failed\n", __FUNCTION__));
		return FALSE;
	}
#endif
	if ((bt_desense = phy_malloc(pi, sizeof(acphy_desense_values_t))) == NULL) {
		PHY_ERROR(("%s: bt_desense malloc failed\n", __FUNCTION__));
		return FALSE;
	}
	if ((fdiqi = phy_malloc(pi, sizeof(acphy_rx_fdiqi_ctl_t))) == NULL) {
		PHY_ERROR(("%s: fdiqi malloc failed\n", __FUNCTION__));
		return FALSE;
	}
#ifdef PREASSOC_PWRCTRL
	if ((pwr_ctrl_save = phy_malloc(pi, sizeof(phy_pwr_ctrl_s))) == NULL) {
		PHY_ERROR(("%s: pwr_ctrl_save malloc failed\n", __FUNCTION__));
		return FALSE;
	}
#endif

	pi_ac->ac_txcal_radioregs_orig = ac_txcal_radioregs_orig;
	pi_ac->ac_rxcal_phyregs_orig = ac_rxcal_phyregs_orig;
	pi_ac->curr_desense = curr_desense;
	pi_ac->zero_desense = zero_desense;
	pi_ac->total_desense = total_desense;
	pi_ac->bt_desense = bt_desense;
	pi_ac->fdiqi = fdiqi;
	pi_ac->pwr_ctrl_save = pwr_ctrl_save;
	pi_ac->hwaci_args = hwaci_args;
	/* Find out the number of cores contained in this ACPHY */
	pi->pubpi->phy_corenum = READ_PHYREGFLD(pi, PhyCapability0, NumberOfStreams);
	pi_ac->phy_caps |= READ_PHYREGFLD(pi, PhyCapability1, SupportMU_MIMO_AP) ?
		PHY_CAP_MU_BFR : 0;
	pi_ac->phy_caps |= READ_PHYREGFLD(pi, PhyCapability1, SupportMU_MIMO_STA) ?
		PHY_CAP_MU_BFE : 0;

	if (ACMAJORREV_32(pi->pubpi->phy_rev)) {
		pi_ac->phy_caps |= READ_PHYREGFLD(pi, PhyCapability1, SupportQAM1024) ?
			PHY_CAP_1024QAM : 0;

	}

	/* setup srom cfg */
	if (!wlc_phy_nvram_attach_acphy(pi))
		return FALSE;

	/* set the gain tables for tiny radio */
	if (TINY_RADIO(pi))
		wlc_phy_set_txgain_tbls(pi);

	/* setup default config params */
	wlc_phy_std_params_attach_acphy(pi);

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* update corenum and coremask state variables */
		phy_ac_update_phycorestate(pi);
		phy_regaccess_war_acphy(pi);
	}

	wlc_phy_txpwrctrl_config_acphy(pi);

	if (!wlc_phy_srom_read_acphy(pi))
		return FALSE;

	acphy_get_lpmode(pi);

	/* Read RFLDO from OTP */
	wlc_phy_rfldo_trim_value(pi);

	/* setup function pointers */
	wlc_phy_fptr_attach_acphy(pi);

	if (BF3_RCAL_OTP_VAL_EN(pi_ac) == 1) {
		if (!otp_read_word(pi->sh->sih, ACPHY_RCAL_OFFSET, &pi->sromi->rcal_otp_val)) {
			pi->sromi->rcal_otp_val &= 0xf;
		} else {
			if (RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) {
				if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) {
					pi->sromi->rcal_otp_val = ACPHY_RCAL_VAL_2X2;
				} else if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) {
					pi->sromi->rcal_otp_val = ACPHY_RCAL_VAL_1X1;
				}
			} else if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID)) {
				pi->sromi->rcal_otp_val = ACPHY_RCAL_VAL_1X1;
			}
		}
	}

	if (CHIPID(pi->sh->chip) == BCM4364_CHIP_ID) {
		uint16 rcal_value = 0;
		if (!otp_read_word(pi->sh->sih, ACPHY_RCAL_OFFSET, &pi->sromi->rcal_otp_val)) {
			rcal_value = (pi->sromi->rcal_otp_val & 0x300) >> 8;
			otp_read_word(pi->sh->sih, 0x14, &pi->sromi->rcal_otp_val);
			pi->sromi->rcal_otp_val = ((pi->sromi->rcal_otp_val &
				0x3) << 2) | rcal_value;
		}
	} else if ((CHIPID(pi->sh->chip) == BCM4345_CHIP_ID) &&
		(RADIO20691_MAJORREV(pi->pubpi->radiorev) == 1) &&
		(RADIO20691_MINORREV(pi->pubpi->radiorev) >= 5)) {
		if (!otp_read_word(pi->sh->sih, ACPHY_RCAL_OFFSET, &pi->sromi->rcal_otp_val))
			pi->sromi->rcal_otp_val = (pi->sromi->rcal_otp_val & 0xf00) >> 8;
	}

	/* PA Mode is set so that NVRAM values are used by default */
	pi_ac->pa_mode = AUTO;
	if (!TINY_RADIO(pi) && !wlc_phy_attach_farrow(pi))
		return FALSE;
	if (!wlc_phy_attach_chan_tuning_tbl(pi))
		return FALSE;
	return TRUE;
}

/*
************************   PHY procs **************************
*/
void wlc_phy_adjust_ed_thres_acphy(phy_info_t *pi, int32 *assert_thresh_dbm, bool set_threshold)
{
	/* Set the EDCRS Assert and De-assert Threshold
	The de-assert threshold is set to 6dB lower then the assert threshold
	Accurate Formula:64*log2(round((10.^((THRESHOLD_dBm +65-30)./10).*50).*(2^9./0.4).^2))
	Simplified Accurate Formula: 64*(THRESHOLD_dBm + 75)/(10*log10(2)) + 832;
	Implemented Approximate Formula: 640000*(THRESHOLD_dBm + 75)/30103 + 832;
	*/
	int32 assert_thres_val, de_assert_thresh_val;

	if (set_threshold == TRUE) {
		assert_thres_val = (640000*(*assert_thresh_dbm + 75) + 25045696)/30103;
		de_assert_thresh_val = (640000*(*assert_thresh_dbm + 69) + 25045696)/30103;
		if (ACREV_GE(pi->pubpi->phy_rev, 32)) {
			/* Set the EDCRS Assert Threshold for core0 */
			WRITE_PHYREG(pi, ed_crs20LAssertThresh00, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20LAssertThresh10, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh00, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh10, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh00, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh10, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh00, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh10, (uint16)assert_thres_val);

			/* Set the EDCRS De-assert Threshold for core0 */
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh00, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh10, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh00, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh10, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh00,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh10,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh00,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh10,
				(uint16)de_assert_thresh_val);

			/* Set the EDCRS Assert Threshold for core1 */
			WRITE_PHYREG(pi, ed_crs20LAssertThresh01, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20LAssertThresh11, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh01, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh11, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh01, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh11, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh01, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh11, (uint16)assert_thres_val);

			/* Set the EDCRS De-assert Threshold for core1 */
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh01, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh11, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh01, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh11, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh01,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh11,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh01,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh11,
				(uint16)de_assert_thresh_val);

			/* Set the EDCRS Assert Threshold for core2 */
			WRITE_PHYREG(pi, ed_crs20LAssertThresh02, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20LAssertThresh12, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh02, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh12, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh02, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh12, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh02, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh12, (uint16)assert_thres_val);

			/* Set the EDCRS De-assert Threshold for core2 */
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh02, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh12, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh02, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh12, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh02,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh12,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh02,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh12,
				(uint16)de_assert_thresh_val);

			/* Set the EDCRS Assert Threshold for core3 */
			WRITE_PHYREG(pi, ed_crs20LAssertThresh03, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20LAssertThresh13, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh03, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh13, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh03, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh13, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh03, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh13, (uint16)assert_thres_val);

			/* Set the EDCRS De-assert Threshold for core3 */
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh03, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh13, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh03, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh13, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh03,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh13,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh03,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh13,
				(uint16)de_assert_thresh_val);
		} else {
			/* Set the EDCRS Assert Threshold */
			WRITE_PHYREG(pi, ed_crs20LAssertThresh0, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20LAssertThresh1, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh0, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20UAssertThresh1, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh0, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1AssertThresh1, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh0, (uint16)assert_thres_val);
			WRITE_PHYREG(pi, ed_crs20Usub1AssertThresh1, (uint16)assert_thres_val);

			/* Set the EDCRS De-assert Threshold */
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh0, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20LDeassertThresh1, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh0, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20UDeassertThresh1, (uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh0,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Lsub1DeassertThresh1,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh0,
				(uint16)de_assert_thresh_val);
			WRITE_PHYREG(pi, ed_crs20Usub1DeassertThresh1,
				(uint16)de_assert_thresh_val);
		}
	}
	else {
		if (ACREV_GE(pi->pubpi->phy_rev, 32))
			assert_thres_val = READ_PHYREG(pi, ed_crs20LAssertThresh00);
		else
			assert_thres_val = READ_PHYREG(pi, ed_crs20LAssertThresh0);
		*assert_thresh_dbm = ((((assert_thres_val - 832)*30103)) - 48000000)/640000;
	}
}

static void
WLBANDINITFN(wlc_phy_cal_init_acphy)(phy_info_t *pi)
{
	PHY_TRACE(("%s: NOT Implemented\n", __FUNCTION__));
}

static bool
BCMATTACHFN(wlc_phy_attach_chan_tuning_tbl)(phy_info_t *pi)
{
	chan_info_radio2069revGE32_t *chan_info_tbl_GE32 = NULL;
	chan_info_radio2069revGE25_52MHz_t *chan_info_tbl_GE25_52MHz = NULL;

	uint32 tbl_len = 0;
	pi->u.pi_acphy->chan_tuning = NULL;
	pi->u.pi_acphy->chan_tuning_tbl_len = 0;

	if (RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) {
		if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 1) {
			if (pi->xtalfreq == 52000000) {
				if ((pi->u.pi_acphy->chan_tuning =
				     MALLOC(pi->sh->osh,
				            NUM_ROWS_CHAN_TUNING *
				            sizeof(chan_info_radio2069revGE25_52MHz_t))) == NULL) {
					PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes",
						pi->sh->unit, __FUNCTION__, MALLOCED(pi->sh->osh)));
					return FALSE;
				}
				switch (RADIO2069REV(pi->pubpi->radiorev)) {
				case 25:
				case 26:
					chan_info_tbl_GE25_52MHz =
					     chan_tuning_2069rev_GE_25_52MHz;
					tbl_len =
					ARRAYSIZE(chan_tuning_2069rev_GE_25_52MHz);
					break;
				default:

					PHY_ERROR(("wl%d: %s: Unsupported radio revision %d\n",
						pi->sh->unit,
						__FUNCTION__, RADIO2069REV(pi->pubpi->radiorev)));
					ASSERT(0);
					return FALSE;
				}
				pi->u.pi_acphy->chan_tuning_tbl_len = tbl_len;
				memcpy(pi->u.pi_acphy->chan_tuning, chan_info_tbl_GE25_52MHz,
					NUM_ROWS_CHAN_TUNING *
					sizeof(chan_info_radio2069revGE25_52MHz_t));
			}

		} else if (RADIO2069_MAJORREV(pi->pubpi->radiorev) == 2) {
			/* malloc chan tuning */
			if ((pi->u.pi_acphy->chan_tuning =
			     MALLOC(pi->sh->osh,
			            NUM_ROWS_CHAN_TUNING *
			            sizeof(chan_info_radio2069revGE32_t))) == NULL) {
				PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n",
				           pi->sh->unit, __FUNCTION__, MALLOCED(pi->sh->osh)));
				return FALSE;
			}
			switch (RADIO2069REV(pi->pubpi->radiorev)) {
			case 32:
			case 33:
			case 34:
			case 35:
			case 37:
			case 38:
				/* can have more conditions based on different radio revs */
				/*  RADIOREV(pi->pubpi->radiorev) =32/33/34 */
				/* currently tuning tbls for these are all same */
				if (pi->xtalfreq == 40000000) {
					chan_info_tbl_GE32 = chan_tuning_2069_rev33_37_40;
					tbl_len = ARRAYSIZE(chan_tuning_2069_rev33_37_40);
				} else {
					chan_info_tbl_GE32 = chan_tuning_2069_rev33_37;
					tbl_len = ARRAYSIZE(chan_tuning_2069_rev33_37);
				}
				break;
			case 39:
			case 40:
			case 44:
				if (pi->xtalfreq == 40000000) {
					chan_info_tbl_GE32 = chan_tuning_2069_rev33_37_40;
					tbl_len = ARRAYSIZE(chan_tuning_2069_rev33_37_40);
				} else {
					chan_info_tbl_GE32 = chan_tuning_2069_rev39;
					tbl_len = ARRAYSIZE(chan_tuning_2069_rev39);
				}
				break;
			case 36:
				if (pi->xtalfreq == 40000000) {
					chan_info_tbl_GE32 = chan_tuning_2069_rev36_40;
					tbl_len = ARRAYSIZE(chan_tuning_2069_rev36_40);
				} else {
					chan_info_tbl_GE32 = chan_tuning_2069_rev36;
					tbl_len = ARRAYSIZE(chan_tuning_2069_rev36);
				}
				break;
			default:

				PHY_ERROR(("wl%d: %s: Unsupported radio revision %d\n",
				           pi->sh->unit,
				           __FUNCTION__, RADIO2069REV(pi->pubpi->radiorev)));
				ASSERT(0);
				return FALSE;
			}
			pi->u.pi_acphy->chan_tuning_tbl_len = tbl_len;
			memcpy(pi->u.pi_acphy->chan_tuning, chan_info_tbl_GE32,
			        NUM_ROWS_CHAN_TUNING * sizeof(chan_info_radio2069revGE32_t));
		}
	}
	return TRUE;
}

void
wlc_phy_tx_farrow_mu_setup(phy_info_t *pi, uint16 MuDelta_l, uint16 MuDelta_u, uint16 MuDeltaInit_l,
	uint16 MuDeltaInit_u)
{
	ACPHYREG_BCAST(pi, TxResamplerMuDelta0l, MuDelta_l);
	ACPHYREG_BCAST(pi, TxResamplerMuDelta0u, MuDelta_u);
	ACPHYREG_BCAST(pi, TxResamplerMuDeltaInit0l, MuDeltaInit_l);
	ACPHYREG_BCAST(pi, TxResamplerMuDeltaInit0u, MuDeltaInit_u);
}

static void
wlc_phy_write_tx_farrow_tiny(phy_info_t *pi, chanspec_t chanspec)
{
	uint8	ch, afe_clk_num, afe_clk_den, core;
	uint16	a, b;
	uint32	fcw, tmp_low = 0, tmp_high = 0;
	uint32	fc;
	bool vco_12GHz_in5G = (pi->u.pi_acphy->vco_12GHz && CHSPEC_IS5G(pi->radio_chanspec));

	if (pi->u.pi_acphy->dac_mode == 1) {
		if (CHSPEC_IS20(chanspec)) {
			if ((pi->vcodivmode & 0x1) || vco_12GHz_in5G)
				a = 16;
			else
				a = 18;
			b = 160;
		} else if (CHSPEC_IS40(chanspec)) {
			if ((pi->vcodivmode & 0x2) || vco_12GHz_in5G)
				a = 8;
			else
				a = 9;
			b = 320;
		} else {
			a = 6;
			b = 640;
		}
	} else if (pi->u.pi_acphy->dac_mode == 2) {
		a = 6;
		b = 640;
	} else {
		if (CHSPEC_IS80(chanspec) || CHSPEC_IS8080(chanspec)) {
			a = 6;
			b = 640;
		} else {
			a = 8;
			b = 320;
		}
	}
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		afe_clk_num = 2;
		afe_clk_den = 3;
	} else {
		if (PAPD_80MHZ_WAR_4349A0(pi) && CHSPEC_IS80(pi->radio_chanspec)) {
			fc = wf_channel2mhz(CHSPEC_CHANNEL(chanspec), WF_CHAN_FACTOR_5_G);
			afe_clk_num = (fc >= 5500) ? 9 : 4;
			afe_clk_den = (fc >= 5500) ? 4 : 2;
		} else {
			afe_clk_num = 3;
			afe_clk_den = 2;
		}
	}
	if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
		const uint8 afeclkdiv_arr[] = {2, 16, 4, 8, 3, 24, 6, 12};
		const uint8 dacclkdiv_arr[] = {6, 8, 9, 16, 18, 32, 64, 10};
		const uint8 dacdiv_arr[] = {2, 4};
		const chan_info_radio20693_altclkplan_t *altclkpln = altclkpln_radio20693;
		int row = wlc_phy_radio20693_altclkpln_get_chan_row(pi);
		if ((row >= 0) && (pi->u.pi_acphy->fast_adc_en == 0)) {
			a = 1;
			afe_clk_num = afeclkdiv_arr[altclkpln[row].afeclkdiv] *
				dacclkdiv_arr[altclkpln[row].dacclkdiv] *
				dacdiv_arr[altclkpln[row].dacdiv];
			afe_clk_den = CHSPEC_IS2G(pi->radio_chanspec) ? 8 : 4;
		}
	}
	/* bits_in_mu = 23 */
	if (CHSPEC_IS8080(chanspec)) {
		FOREACH_CORE(pi, core) {
			if (core == 0) {
				ch = wf_chspec_primary80_channel(chanspec);
				fc = wf_channel2mhz(ch, WF_CHAN_FACTOR_5_G);

				bcm_uint64_multiple_add(&tmp_high, &tmp_low, a * afe_clk_num * b,
					1 << 23, (fc * afe_clk_den) >> 1);
				bcm_uint64_divide(&fcw, tmp_high, tmp_low, fc * afe_clk_den);

				WRITE_PHYREG(pi, TxResamplerMuDelta0l, fcw & 0xffff);
				WRITE_PHYREG(pi, TxResamplerMuDelta0u, (fcw & 0xff0000) >> 16);
				WRITE_PHYREG(pi, TxResamplerMuDeltaInit0l, fcw & 0xffff);
				WRITE_PHYREG(pi, TxResamplerMuDeltaInit0u, (fcw & 0xff0000) >> 16);
			} else if (core == 1) {
				ch = wf_chspec_secondary80_channel(chanspec);
				fc = wf_channel2mhz(ch, WF_CHAN_FACTOR_5_G);

				bcm_uint64_multiple_add(&tmp_high, &tmp_low, a * afe_clk_num * b,
					1 << 23, (fc * afe_clk_den) >> 1);
				bcm_uint64_divide(&fcw, tmp_high, tmp_low, fc * afe_clk_den);

				WRITE_PHYREG(pi, TxResamplerMuDelta1l, fcw & 0xffff);
				WRITE_PHYREG(pi, TxResamplerMuDelta1u, (fcw & 0xff0000) >> 16);
				WRITE_PHYREG(pi, TxResamplerMuDeltaInit1l, fcw & 0xffff);
				WRITE_PHYREG(pi, TxResamplerMuDeltaInit1u, (fcw & 0xff0000) >> 16);
			}
		}
	} else {
		ch = CHSPEC_CHANNEL(chanspec);
		fc = wf_channel2mhz(ch, CHSPEC_IS2G(pi->radio_chanspec) ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		bcm_uint64_multiple_add(&tmp_high, &tmp_low, a * afe_clk_num * b,
			1 << 23, (fc * afe_clk_den) >> 1);
		bcm_uint64_divide(&fcw, tmp_high, tmp_low, fc * afe_clk_den);

		wlc_phy_tx_farrow_mu_setup(pi, fcw & 0xffff, (fcw & 0xff0000) >> 16, fcw & 0xffff,
			(fcw & 0xff0000) >> 16);
	}
}

static bool
BCMATTACHFN(wlc_phy_attach_farrow)(phy_info_t *pi)
{
	int num_bw;
#ifndef ACPHY_1X1_ONLY
	phy_info_acphy_t *pi_ht = (phy_info_acphy_t *)pi->u.pi_acphy;
#endif
	chan_info_tx_farrow(*tx_farrow) [ACPHY_NUM_CHANS];
	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));
	pi->u.pi_acphy->tx_farrow = NULL;
	pi->u.pi_acphy->rx_farrow = NULL;
#ifdef ACPHY_1X1_ONLY
	num_bw = 1;
#else
	num_bw = ACPHY_NUM_BW;
#endif

	if ((pi->u.pi_acphy->tx_farrow =
	     MALLOC(pi->sh->osh,
	            num_bw * sizeof(chan_info_tx_farrow[ACPHY_NUM_CHANS]))) == NULL) {
		PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n", pi->sh->unit,
		           __FUNCTION__, MALLOCED(pi->sh->osh)));
		return FALSE;
	}

	if (!TINY_RADIO(pi)) {
		chan_info_rx_farrow(*rx_farrow) [ACPHY_NUM_CHANS];
		/* TINY RADIO does not have an rx farrow table */
		if ((pi->u.pi_acphy->rx_farrow =
		     MALLOC(pi->sh->osh,
		            num_bw * sizeof(chan_info_rx_farrow[ACPHY_NUM_CHANS]))) == NULL) {
			PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n", pi->sh->unit,
			           __FUNCTION__, MALLOCED(pi->sh->osh)));
			return FALSE;
		}

		rx_farrow = rx_farrow_tbl;
		memcpy(pi->u.pi_acphy->rx_farrow, rx_farrow,
		       ACPHY_NUM_CHANS * num_bw * sizeof(chan_info_rx_farrow));
	}


#ifdef ACPHY_1X1_ONLY
	ASSERT(((phy_info_acphy_t *)pi->u.pi_acphy)->dac_mode == 1);
	tx_farrow = tx_farrow_dac1_tbl;
#else /* ACPHY_1X1_ONLY */
	switch (pi_ht->dac_mode) {
	case 2:
		tx_farrow = tx_farrow_dac2_tbl;
		break;
	case 3:
		tx_farrow = tx_farrow_dac3_tbl;
		break;
	case 1:
	default:
		/* default to dac_mode 1 */
		tx_farrow = tx_farrow_dac1_tbl;
		break;
	}
#endif /* ACPHY_1X1_ONLY */
	memcpy(pi->u.pi_acphy->tx_farrow, tx_farrow,
	       ACPHY_NUM_CHANS * num_bw * sizeof(chan_info_tx_farrow));
	return TRUE;
}

void
wlc_phy_farrow_setup_tiny(phy_info_t *pi, chanspec_t chanspec)
{
	/* Setup adc mode based on BW */
	pi->u.pi_acphy->fast_adc_en = TINY_GET_ADC_MODE(pi, chanspec);

	wlc_phy_write_tx_farrow_tiny(pi, chanspec);
	wlc_phy_write_rx_farrow_tiny(pi, chanspec);

	/* Enable the Tx resampler on all cores */
	MOD_PHYREG(pi, TxResamplerEnable0, enable_tx, 1);
}

void
wlc_phy_loadsampletable_acphy(phy_info_t *pi, math_cint32 *tone_buf, uint16 num_samps,
        bool alloc, bool conj)
{
	uint16 t;
	uint32* data_buf = NULL;
	int32 sgn = 1;

	if (alloc) {
	/* allocate buffer */
	if ((data_buf = (uint32 *)MALLOC(pi->sh->osh, sizeof(uint32) * num_samps)) == NULL) {
		PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n", pi->sh->unit,
		           __FUNCTION__, MALLOCED(pi->sh->osh)));
		return;
	}
	} else {
	  data_buf = (uint32*)tone_buf;
	}

	if (conj)
	  sgn = -1;

	/* load samples into sample play buffer */
	for (t = 0; t < num_samps; t++) {
		data_buf[t] = ((((unsigned int)tone_buf[t].i) & 0x3ff) << 10) |
		               (((unsigned int)(sgn * tone_buf[t].q)) & 0x3ff);
	}
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_SAMPLEPLAY, num_samps, 0, 32, data_buf);

	if (alloc && (data_buf != NULL))
		MFREE(pi->sh->osh, data_buf, sizeof(uint32) * num_samps);
}

static uint16
wlc_phy_gen_load_samples_acphy(phy_info_t *pi, int32 f_kHz, uint16 max_val, uint8 mac_based)
{
	uint8 fs_spb;
	uint16 spb_depth = 512;
	uint16 num_samps, t, max_periods, num_periods = 1;
	math_fixed theta = 0, rot = 0;
	uint16 tbl_len;
	math_cint32* tone_buf = NULL;

	if (ACMAJORREV_3(pi->pubpi->phy_rev) || ACMAJORREV_4(pi->pubpi->phy_rev)) {
		spb_depth = 256;
	}

	/* check phy_bw */
	if (pi->u.pi_acphy->dac_mode == 1) {
		if ((CHSPEC_IS80(pi->radio_chanspec)) || (CHSPEC_IS8080(pi->radio_chanspec)))
			fs_spb = 160;
		else if (CHSPEC_IS40(pi->radio_chanspec))
			fs_spb = 80;
		else
			fs_spb = 40;
	} else if (pi->u.pi_acphy->dac_mode == 2) {
		fs_spb = 160;
	} else { /* dac mode 3 */
		fs_spb = 80;
	}

	if (f_kHz == 0) {
		tbl_len = fs_spb;
	} else {

		max_periods = (spb_depth * ABS(f_kHz)) / (fs_spb * 1000);
		for (t = 1; t <= max_periods; t++) {
			if (((fs_spb * 1000 * t) % ABS(f_kHz)) == 0) {
				num_periods = t;
				break;
			}
		}
		if (((fs_spb * 1000 * num_periods) % ABS(f_kHz)) != 0) {
			PHY_ERROR(("%s ERROR: Unable to fit tone period within table boundary\n",
			__FUNCTION__));
			PHY_ERROR(("sample play freq = %d inum_period=%d Tone Freq=%d\n",
			fs_spb, num_periods, f_kHz));
			return 0;
		}

		tbl_len = (fs_spb * 1000 * num_periods) / ABS(f_kHz);
	}

	/* allocate buffer */
	if ((tone_buf = MALLOC(pi->sh->osh, sizeof(math_cint32) * tbl_len)) == NULL) {
		PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n", pi->sh->unit,
		          __FUNCTION__, MALLOCED(pi->sh->osh)));
		return 0;
	}

	/* set up params to generate tone */
	num_samps  = (uint16)tbl_len;
	rot = FIXED((f_kHz * 36)/fs_spb) / 100; /* 2*pi*f/bw/1000  Note: f in KHz */
	theta = 0; /* start angle 0 */

	/* tone freq = f_c MHz ; phy_bw = phy_bw MHz ; # samples = phy_bw (1us) */
	for (t = 0; t < num_samps; t++) {
		/* compute phasor */
		phy_utils_cordic(theta, &tone_buf[t]);
		/* update rotation angle */
		theta += rot;
		/* produce sample values for play buffer */
		tone_buf[t].q = (int32)FLOAT(tone_buf[t].q * max_val);
		tone_buf[t].i = (int32)FLOAT(tone_buf[t].i * max_val);
	}

	/* load sample table */
	wlc_phy_loadsampletable_acphy(pi, tone_buf, num_samps, TRUE, FALSE);

	if (tone_buf != NULL)
		MFREE(pi->sh->osh, tone_buf, sizeof(math_cint32) * tbl_len);

	return num_samps;
}

int
wlc_phy_tx_tone_acphy(phy_info_t *pi, int32 f_kHz, uint16 max_val, uint8 iqmode,
                      uint8 mac_based, bool modify_bbmult)
{
	uint8 core;
	uint16 num_samps;
	uint16 bb_mult;
	uint16 loops = 0xffff;
	uint16 wait = 0;
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	PHY_TRACE(("wl%d: %s\n", pi->sh->unit, __FUNCTION__));

	if (max_val == 0) {
		num_samps = 1;
	} else if ((num_samps = wlc_phy_gen_load_samples_acphy(pi, f_kHz, max_val, mac_based))
	           == 0) {
		return BCME_ERROR;
	}

	if (pi_ac->bb_mult_save_valid == 0) {
		FOREACH_CORE(pi, core) {
			wlc_phy_get_tx_bbmult_acphy(pi, &pi_ac->bb_mult_save[core], core);
		}
		pi_ac->bb_mult_save_valid = 1;
	}

	if (max_val == 0 || modify_bbmult) {
		if (max_val == 0) {
			bb_mult = 0;
		} else {
			if (CHSPEC_IS80(pi->radio_chanspec))
				bb_mult = 64;
			else if (CHSPEC_IS40(pi->radio_chanspec))
				bb_mult = 64;
			else
				bb_mult = 64;
		}
		FOREACH_CORE(pi, core) {
			wlc_phy_set_tx_bbmult_acphy(pi, &bb_mult, core);
		}
	}

	if (ACMAJORREV_5(pi->pubpi->phy_rev) && ACMINORREV_0(pi) &&
	    (BFCTL(pi_ac) == 3) &&
	    (BF3_FEMCTRL_SUB(pi_ac) == 0 || BF3_FEMCTRL_SUB(pi_ac) == 3)) {
		/* 43602a0 router boards with PAVREF WAR: turn on PA */
		si_pmu_regcontrol(pi->sh->sih, 0, 0x7, 7);
	}

	wlc_phy_runsamples_acphy(pi, num_samps, loops, wait, iqmode, mac_based);

	return BCME_OK;
}

void
wlc_phy_stopplayback_acphy(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
	uint16 playback_status, phy_ctl, SampleCollectPlayCtrl;
	uint8 mac_sample_play_on = 0;
	uint16 mask;
	uint8 stall_val;

	if (ACMAJORREV_5(pi->pubpi->phy_rev) && ACMINORREV_0(pi) &&
		(BFCTL(pi_ac) == 3) &&
		(BF3_FEMCTRL_SUB(pi_ac) == 0 || BF3_FEMCTRL_SUB(pi_ac) == 3)) {
		/* 43602a0 router boards with PAVREF WAR: turn off PA */
		si_pmu_regcontrol(pi->sh->sih, 0, 0x7, 0);
	}

	/* Find out if its a mac based sample play or phy based sample play */
	/* If its mac based sample play, unset the appropriate bits based on d11rev */
	if (D11REV_IS(pi->sh->corerev, 50) || D11REV_GE(pi->sh->corerev, 53)) {
		SampleCollectPlayCtrl =
			R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectPlayCtrl);
		mac_sample_play_on = (SampleCollectPlayCtrl >>
			SAMPLE_COLLECT_PLAY_CTRL_PLAY_START_SHIFT) & 1;
		if (mac_sample_play_on == 1) {
			mask = ~(1 << SAMPLE_COLLECT_PLAY_CTRL_PLAY_START_SHIFT);
			SampleCollectPlayCtrl &=  mask;
			W_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectPlayCtrl,
				SampleCollectPlayCtrl);
		}
	} else {
		phy_ctl = R_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param);
		mac_sample_play_on = (phy_ctl >> PHYCTRL_SAMPLEPLAYSTART_SHIFT) & 1;
		if (mac_sample_play_on == 1) {
			mask = ~(1 << PHYCTRL_SAMPLEPLAYSTART_SHIFT);
			phy_ctl &= mask;
			W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, phy_ctl);
		}
	}

	if (mac_sample_play_on == 0) {
		/* check status register */
		playback_status = READ_PHYREG(pi, sampleStatus);
		if (playback_status & 0x1) {
			stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
			ACPHY_DISABLE_STALL(pi);
			phy_utils_or_phyreg(pi, ACPHY_sampleCmd(pi->pubpi->phy_rev),
				ACPHY_sampleCmd_stop_MASK(pi->pubpi->phy_rev));
			ACPHY_ENABLE_STALL(pi, stall_val);
		} else if (playback_status & 0x2) {
			phy_utils_and_phyreg(pi, ACPHY_iqloCalCmdGctl(pi->pubpi->phy_rev),
				(uint16)~ACPHY_iqloCalCmdGctl_iqlo_cal_en_MASK(pi->pubpi->phy_rev));
		} else {
			PHY_CAL(("wlc_phy_stopplayback_acphy: already disabled\n"));
		}
	}
	/* disable the dac_test mode */
	phy_utils_and_phyreg(pi, ACPHY_sampleCmd(pi->pubpi->phy_rev),
		~ACPHY_sampleCmd_DacTestMode_MASK(pi->pubpi->phy_rev));

	/* if bb_mult_save does exist, restore bb_mult and undef bb_mult_save */
	if (pi_ac->bb_mult_save_valid != 0) {
		uint8 core;

		FOREACH_CORE(pi, core) {
			wlc_phy_set_tx_bbmult_acphy(pi, &pi_ac->bb_mult_save[core], core);
		}
		pi_ac->bb_mult_save_valid = 0;
	}

	wlc_phy_resetcca_acphy(pi);
}

void
wlc_phy_runsamples_acphy(phy_info_t *pi, uint16 num_samps, uint16 loops, uint16 wait, uint8 iqmode,
                         uint8 mac_based)
{
	uint8  sample_cmd;
	uint16 orig_RfseqCoreActv;
	uint8  dac_test_mode = 0;
	const uint phy_rev = pi->pubpi->phy_rev;
	uint8 stall_val;

	/* The phy_rev parameter is unused in embedded builds as the compiler optimises it away.
	 * Mark the param as unused to avoid compiler warnings.
	 */
	UNUSED_PARAMETER(phy_rev);

	if (!(iqmode))
		wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
	/* Delay for proper RX2TX in sample play ow spurious emissions,radar FD */
	OSL_DELAY(15);
	if (mac_based == 1) {
		phy_utils_or_phyreg(pi, ACPHY_macbasedDACPlay(phy_rev),
			ACPHY_macbasedDACPlay_macBasedDACPlayEn_MASK(phy_rev));

		if (CHSPEC_IS80(pi->radio_chanspec)) {
			phy_utils_or_phyreg(pi, ACPHY_macbasedDACPlay(phy_rev),
				ACPHY_macbasedDACPlay_macBasedDACPlayMode_MASK(phy_rev) & (0x3 <<
				ACPHY_macbasedDACPlay_macBasedDACPlayMode_SHIFT(phy_rev)));
		} else if (CHSPEC_IS40(pi->radio_chanspec)) {
			phy_utils_or_phyreg(pi, ACPHY_macbasedDACPlay(phy_rev),
				ACPHY_macbasedDACPlay_macBasedDACPlayMode_MASK(phy_rev) & (0x2 <<
				ACPHY_macbasedDACPlay_macBasedDACPlayMode_SHIFT(phy_rev)));
		} else {
			phy_utils_or_phyreg(pi, ACPHY_macbasedDACPlay(phy_rev),
				ACPHY_macbasedDACPlay_macBasedDACPlayMode_MASK(phy_rev) & (0x1 <<
				ACPHY_macbasedDACPlay_macBasedDACPlayMode_SHIFT(phy_rev)));
		}

		PHY_TRACE(("Starting MAC based Sample Play"));
		wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_RX2TX);

		if (D11REV_IS(pi->sh->corerev, 50) || D11REV_GE(pi->sh->corerev, 53)) {
			uint16 SampleCollectPlayCtrl =
				R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectPlayCtrl);
			SampleCollectPlayCtrl |= (1 << SAMPLE_COLLECT_PLAY_CTRL_PLAY_START_SHIFT);
			W_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectPlayCtrl,
				SampleCollectPlayCtrl);
		} else {
			uint16 phy_ctl;
			phy_ctl = (1 << PHYCTRL_SAMPLEPLAYSTART_SHIFT)
				| (1 << PHYCTRL_MACPHYFORCEGATEDCLKSON_SHIFT);
			W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, phy_ctl);
		}
	} else {
		phy_utils_and_phyreg(pi, ACPHY_macbasedDACPlay(phy_rev),
			~ACPHY_macbasedDACPlay_macBasedDACPlayEn_MASK(phy_rev));

		/* configure sample play buffer */
		WRITE_PHYREG(pi, sampleDepthCount, num_samps-1);

		if (loops != 0xffff) { /* 0xffff means: keep looping forever */
			WRITE_PHYREG(pi, sampleLoopCount, loops - 1);
		} else {
			WRITE_PHYREG(pi, sampleLoopCount, loops);
		}

		/* Wait time should be atleast 60 for farrow FIFO depth to settle
		 * 60 is to support 80mhz mode.
		 * Though 20 is even for 20mhz mode, and 40 for 80mhz mode,
		 * but just giving some extra wait time
		 */
		WRITE_PHYREG(pi, sampleInitWaitCount, (wait > 60) ? wait : 60);

		/* start sample play buffer (in regular mode or iqcal mode) */
		orig_RfseqCoreActv = READ_PHYREG(pi, RfseqMode);
		phy_utils_or_phyreg(pi, ACPHY_RfseqMode(phy_rev),
			ACPHY_RfseqMode_CoreActv_override_MASK(phy_rev));
		phy_utils_and_phyreg(pi, ACPHY_sampleCmd(phy_rev),
			~ACPHY_sampleCmd_DacTestMode_MASK(phy_rev));
		phy_utils_and_phyreg(pi, ACPHY_sampleCmd(phy_rev),
		                     ~ACPHY_sampleCmd_start_MASK(phy_rev));
		phy_utils_and_phyreg(pi, ACPHY_iqloCalCmdGctl(phy_rev), 0x3FFF);
		if (iqmode) {
			phy_utils_or_phyreg(pi, ACPHY_iqloCalCmdGctl(phy_rev), 0x8000);
		} else {
			sample_cmd = ACPHY_sampleCmd_start_MASK(phy_rev);
			sample_cmd |= (dac_test_mode == 1 ?
				ACPHY_sampleCmd_DacTestMode_MASK(phy_rev) : 0);
			/* Disable stall before issue the sample play start
			as the stall can cause it to miss the start
			*/
			stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
			ACPHY_DISABLE_STALL(pi);
			phy_utils_or_phyreg(pi, ACPHY_sampleCmd(phy_rev), sample_cmd);
			ACPHY_ENABLE_STALL(pi, stall_val);
		}

		/* Wait till the Rx2Tx sequencing is done */
		SPINWAIT(((READ_PHYREG(pi, RfseqStatus0) & 0x1) == 1),
		         ACPHY_SPINWAIT_RUNSAMPLE);

		/* restore mimophyreg(RfseqMode.CoreActv_override) */
		WRITE_PHYREG(pi, RfseqMode, orig_RfseqCoreActv);
	}

	if (!(iqmode))
		wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
}

void wlc_ant_div_sw_control(phy_info_t *pi, int8 divOvrride, int core)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	/* pin position 255 implies the NVRAM does not have antdiv_rfswctrlpin_aX entry */
	/* Diversity override can be 0 or 1 to select the antenna */
	/*  and 2 to restore original FEMCTRL table */
	if (core == 0 && (pi_ac->antdiv_rfswctrlpin_a0 != 255)) {
	  pi_ac->ant_swOvr_state_core0 = divOvrride;
	} else if (core == 1 && (pi_ac->antdiv_rfswctrlpin_a1 != 255)) {
	  pi_ac->ant_swOvr_state_core1 = divOvrride;
	} else {
	  return;
	}
	/* compute/write the whole femctrl table again for now */
	/* Should be optimized later */
	wlc_phy_write_regtbl_fc_from_nvram(pi);
}

/* done with papd cal */

uint8
wlc_phy_rssi_get_chan_freq_range_acphy(phy_info_t *pi, uint8 core_segment_mapping)
{
	uint8 channel = CHSPEC_CHANNEL(pi->radio_chanspec);

	if (phy_get_phymode(pi) == PHYMODE_80P80) {

		if (PRIMARY_FREQ_SEGMENT == core_segment_mapping)
			channel = wf_chspec_primary80_channel(pi->radio_chanspec);

		if (SECONDARY_FREQ_SEGMENT == core_segment_mapping)
			channel = wf_chspec_secondary80_channel(pi->radio_chanspec);
	}

	PHY_TRACE(("wl%d: %s | channel = %d \n", pi->sh->unit, __FUNCTION__, channel));

	if (channel <= CH_MAX_2G_CHANNEL) {
		phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

		return pi_ac->sromi->rssi_cal_freq_grp[channel-1] & 0x7;
	} else {
		int freq;

		if (RADIOID_IS(pi->pubpi->radioid, BCM2069_ID)) {
			const void *chan_info;

			freq = wlc_phy_chan2freq_acphy(pi, channel, &chan_info);
		} else if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
			const chan_info_radio20693_pll_t *chan_info_20693_pll;
			const chan_info_radio20693_rffe_t *chan_info_20693_rffe;

			freq = wlc_phy_chan2freq_20693(pi, channel,
			                               &chan_info_20693_pll, &chan_info_20693_rffe);
		} else {
			const chan_info_radio20691_t *chan_info_20691;

			freq = wlc_phy_chan2freq_20691(pi, channel, &chan_info_20691);
		}

		if ((freq >= PHY_RSSI_SUBBAND_4BAND_BAND0) &&
			(freq < PHY_RSSI_SUBBAND_4BAND_BAND1))
			return WL_CHAN_FREQ_RANGE_5G_BAND0 - 1;
		else if ((freq >= PHY_RSSI_SUBBAND_4BAND_BAND1) &&
			(freq < PHY_RSSI_SUBBAND_4BAND_BAND2))
			return WL_CHAN_FREQ_RANGE_5G_BAND1 - 1;
		else if ((freq >= PHY_RSSI_SUBBAND_4BAND_BAND2) &&
			(freq < PHY_RSSI_SUBBAND_4BAND_BAND3))
			return WL_CHAN_FREQ_RANGE_5G_BAND2 - 1;
		else
			return WL_CHAN_FREQ_RANGE_5G_BAND3 - 1;
	}
}

#ifdef WL_SAR_SIMPLE_CONTROL
void wlc_phy_dynamic_sarctrl_set(wlc_phy_t *pi, bool isctrlon)
{
	phy_info_t *piinfo = (phy_info_t*)pi;
	uint32 sarctrlmap = 0;

	if (isctrlon) {
		switch (wlc_phy_chanspec_bandrange_get(piinfo, piinfo->radio_chanspec)) {
			case WL_CHAN_FREQ_RANGE_2G:
				sarctrlmap = piinfo->dynamic_sarctrl_2g;
				break;
		#ifdef BAND5G
			case WL_CHAN_FREQ_RANGE_5GL:
			case WL_CHAN_FREQ_RANGE_5GM:
			case WL_CHAN_FREQ_RANGE_5GH:
				sarctrlmap = piinfo->dynamic_sarctrl_5g;
				break;
		#endif /* BAND5G */
			default:
				break;
		}
	} else {
		sarctrlmap = 0;
	}
	wlc_phy_sar_limit_set_percore(pi, sarctrlmap);
}
#endif /* WL_SAR_SIMPLE_CONTROL */


#define ACPHY_RXCAL_NUMRXGAINS 16

typedef struct _acphy_rxcal_rxgain {
	int8 lna;
	uint8 tia;
	uint8 far;
	uint8 dvga;
} acphy_rxcal_rxgain_t;

/* see also: proc acphy_rx_iq_cal_txrxgain_control_tiny { } */
void
wlc_phy_rxcal_txrx_gainctrl_acphy_tiny(phy_info_t *pi)
{
	/* table for leakage path 5G : lna,tia,far,dvga */
	acphy_rxcal_rxgain_t gaintbl_5G[ACPHY_RXCAL_NUMRXGAINS] = {
		{ -4, 0, 2, 0 },
		{ -4, 0, 1, 0 },
		{ -4, 0, 0, 0 },
		{ -3, 0, 0, 0 },
		{ -2, 0, 0, 0 },
		{ -1, 0, 0, 0 },
		{ 0, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 2, 0, 0 },
		{ 0, 3, 0, 0 },
		{ 0, 4, 0, 0 },
		{ 0, 5, 0, 0 },
		{ 0, 6, 0, 0 },
		{ 0, 7, 0, 0 },
		{ 0, 8, 0, 0 },
		{ 0, 9, 0, 0 }
	};

	/* table for papd loopback path 2G : lna,tia,far,dvga */
	acphy_rxcal_rxgain_t gaintbl_2G[ACPHY_RXCAL_NUMRXGAINS] = {
		{ 0, 1, 1, 0 },
		{ 0, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 2, 0, 0 },
		{ 0, 3, 0, 0 },
		{ 0, 4, 0, 0 },
		{ 0, 5, 0, 0 },
		{ 0, 6, 0, 0 },
		{ 0, 7, 0, 0 },
		{ 0, 8, 0, 0 },
		{ 0, 9, 0, 0 },
		{ 0, 10, 0, 0 },
		{ 0, 10, 0, 1 },
		{ 0, 10, 0, 2 },
		{ 0, 10, 0, 3 },
		{ 0, 10, 0, 4 }
	};

	acphy_rxcal_rxgain_t *gaintbl;
	uint8 core;
	uint8 g_index, done, found_ideal, wn, txindex;
	bool txdone;
	uint8 do_max = 10;	/* >= ACPHY_RXCAL_NUMRXGAINS / 2 */
	uint8 tia, far, lna_idx;
	int8 lna;
	uint16 num_samps = 1024;
	uint32 meansq_max = (PHY_ILNA(pi)) ? 4000 : 7000; /* As dictated by iqest / dc offsets */

	/*
	 * Set min power more than max gain step below max power to prevent AGC hunting
	 * set 8 dB below max setting
	 */
	uint32 meansq_min = (PHY_ILNA(pi)) ?  535 : 1111; /* -8dB on pwr_max / 6.3  */
	uint32 i_meansq = 0;
	uint32 q_meansq = 0;
	uint8 lna2 = 0;
	uint8 lna2_rout = 0;
	uint8 dvga = 0;
	const uint8 txindx_start = 104;
	const uint8 txindx_stop  = 56;
	const uint8 txindx_step  = 12;
	int p;
	uint8 clipDet;
	uint8 lna_gain_code, lna_rout_code;

	phy_iq_est_t est[PHY_CORE_MAX];

	(void)memset(est, 0, sizeof(est));

	if (CHSPEC_IS5G(pi->radio_chanspec)) {
		gaintbl = gaintbl_5G;
	} else {
		gaintbl = gaintbl_2G;
	}

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* reusing PAPD path for 5G as well in 4349A0,
		 * so using the same gain sets as for 2G PAPD
		 * loopback path
		 */
		gaintbl = gaintbl_2G;
		/* use maximum LNA2 gain index */
		lna2 = 3;
	}

	FOREACH_CORE(pi, core) {
		g_index = 8;
		done = 0;
		found_ideal = 0;
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			txdone = 1;
		} else {
			txdone = CHSPEC_IS2G(pi->radio_chanspec);	/* only adapt tx in 5G */
		}
		wn = 0;
		txindex = txindx_start;
		lna_idx = READ_PHYREGFLDC(pi, InitGainCodeA, core, initLnaIndex);

		while ((done != do_max) && (g_index != 0) && (g_index != ACPHY_RXCAL_NUMRXGAINS)) {
			if (CHSPEC_IS2G(pi->radio_chanspec) ||
				(CHSPEC_IS5G(pi->radio_chanspec) &&
				ACMAJORREV_4(pi->pubpi->phy_rev))) {
				/* papd loopback path */
				 lna = lna_idx;
				 tia = gaintbl[g_index].tia;
				 far = gaintbl[g_index].far;
				 dvga = gaintbl[g_index].dvga;
			} else { /* leakage path for 5G */
				lna = lna_idx + gaintbl[g_index].lna;
				tia = gaintbl[g_index].tia;
				far = gaintbl[g_index].far;
			}

			MOD_PHYREG(pi, RxSdFeConfig6, rx_farrow_rshift_0, far);
			MOD_PHYREGCE(pi, RfctrlOverrideGains, core, rxgain, 1);

			lna_gain_code = wlc_phy_get_lna_gain_rout(pi, lna, GET_LNA_GAINCODE);
			lna_rout_code = wlc_phy_get_lna_gain_rout(pi, lna, GET_LNA_ROUT);

			WRITE_PHYREGCE(pi, RfctrlCoreRXGAIN1, core,
			               (dvga << 10) | (tia << 6) | (lna2 << 3) |
			               lna_gain_code);
			WRITE_PHYREGCE(pi, RfctrlCoreRXGAIN2, core,
			               ((lna2_rout << 4) | (lna_rout_code & 0xf)));

			MOD_PHYREG(pi, RfseqCoreActv2059, EnTx, 0x7);

			if (!txdone)
				wlc_phy_set_txpwr_by_index_acphy(pi, (1 << core), txindex);

			/* turn on testtone */
			wlc_phy_tx_tone_acphy(pi, (((CHSPEC_IS80(pi->radio_chanspec))
			                          ? ACPHY_IQCAL_TONEFREQ_80MHz
			                          : (CHSPEC_IS40(pi->radio_chanspec))
			                              ? ACPHY_IQCAL_TONEFREQ_40MHz
			                              : ACPHY_IQCAL_TONEFREQ_20MHz) >> 1),
			                              ACPHY_RXCAL_TONEAMP, 0, 0, FALSE);

			/*
			 * Check for RF saturation by (1) power detect or (2) bb power.
			 * See txdone condition.
			 */
			wn = 0;
			for (p = 0; p < 8; p++) {
				wn +=  READ_RADIO_REGFLD_TINY(pi, LNA5G_RSSI2, core,
				                               lna5g_dig_wrssi1_out_low);
				wn +=  READ_RADIO_REGFLD_TINY(pi, TIA_CFG14, core, nbrssi_Ich_low);
				wn +=  READ_RADIO_REGFLD_TINY(pi, TIA_CFG14, core, nbrssi_Qch_low);
			}

			/* estimate digital power using rx_iq_est */
			wlc_phy_rx_iq_est_acphy(pi, est, num_samps, 32, 0, FALSE);

			/* Turn off the tone */
			wlc_phy_stopplayback_acphy(pi);

			i_meansq = (est[core].i_pwr + num_samps / 2) / num_samps;
			q_meansq = (est[core].q_pwr + num_samps / 2) / num_samps;

			clipDet = READ_PHYREGFLDCXE(pi, IqestCmd, clipDet, core);
#if defined(BCMDBG_RXCAL)
	printf("RxIQCAL[%d]: txindx=%d g_index=%d lna=%d tia=%d far=%d dvga=%d\n",
	       core, txindex, g_index, lna, tia, far, dvga);
	printf("RxIQCAL[%d]: i_meansq=%d q_meansq=%d meansq_max=%d meansq_min=%d clipDet=%d\n",
		core, i_meansq, q_meansq, meansq_max, meansq_min, clipDet);
#endif
			txdone = txdone ||
				(txindex < txindx_stop) || (wn > 0) || (i_meansq > meansq_min) ||
				(q_meansq > meansq_min);

			if (!txdone) {
				txindex -= txindx_step;
				continue;
			}

			if ((i_meansq > meansq_max) || (q_meansq > meansq_max) || (clipDet == 1)) {
				g_index--;
				done++;
			} else if ((i_meansq < meansq_max) && (q_meansq < meansq_min)) {
				g_index++;
				done++;
			} else {
				done = do_max;
				found_ideal = 1;
			}
		}
		if (found_ideal == 0) {
			PHY_ERROR(("%s: Too much or too little power? "
				"[core: %d, pwr: (%d, %d), gain_index=%d]\n",
				__FUNCTION__, core, i_meansq, q_meansq, g_index));
		}
	}
}

static void
wlc_idac_preload_20691(phy_info_t *pi, int16 i, int16 q)
{
	uint16 cnt;
	ASSERT(ACREV_GE(pi->pubpi->phy_rev, 11));

	if (i < 0)
		i += 512;
	if (q < 0)
		q += 512;

	i &= 0x1ff;
	q &= 0x1ff;

	/* WAR for negative values -ensure fsm is running */
	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 0x0);
	cnt = READ_PHYREG(pi, rx_tia_dc_loop_gain_5);
	WRITE_PHYREG(pi, rx_tia_dc_loop_gain_5, 15); /* restart gain is 0, ie NOP */
	MOD_PHYREG(pi, rx_tia_dc_loop_0, en_lock, 0); /* always run */
	OSL_DELAY(1);

	/* overide offset comp PU; phyctrl logic not reliable for preload WAR,
	   dac value fails to update occassionally
	*/
	MOD_RADIO_REG_20691(pi, TIA_CFG15, 0, tia_offset_comp_pwrup, 1);
	MOD_RADIO_REG_20691(pi, RX_BB_2G_OVR_NORTH, 0, ovr_tia_offset_comp_pwrup, 1);

	/* restart to enable preload WAR for negative value (CRDOT11ACPHY-871) */
	wlc_dcc_fsm_restart(pi);

	/* write the values across */
	WRITE_PHYREG(pi, BfmConfig3, 0x0200 | i);
	WRITE_PHYREG(pi, BfmConfig3, 0x0600 | q);
	WRITE_PHYREG(pi, BfmConfig3, 0x0000);

	/* remove comp pwrup force */
	MOD_RADIO_REG_20691(pi, RX_BB_2G_OVR_NORTH, 0, ovr_tia_offset_comp_pwrup, 0);

	/* stop loop running and restore config */
	MOD_PHYREG(pi, rx_tia_dc_loop_0, en_lock, 1);

	OSL_DELAY(1);

	WRITE_PHYREG(pi, rx_tia_dc_loop_gain_5, cnt);

	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 0x1);
}

static void
wlc_idac_read_20691(phy_info_t *pi, int16 *i, int16 *q)
{
	int16 i1, q1;

	ASSERT(ACREV_GE(pi->pubpi->phy_rev, 11));

	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 0x0);

	wlc_phy_enable_lna_dcc_comp_20691(pi, 0);
	OSL_DELAY(2);

	i1 = READ_PHYREG(pi, TIA_offset_DAC_I);

	if (i1 > 255)
		i1 = 256 - i1;

	q1 = READ_PHYREG(pi, TIA_offset_DAC_Q);

	if (q1 > 255)
		q1 = 256 - q1;

	*i = i1;
	*q = q1;

	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 0x1);

	wlc_phy_enable_lna_dcc_comp_20691(pi, PHY_ILNA(pi));
}
void
wlc_phy_cals_mac_susp_en_other_cr(phy_info_t *pi, bool suspend)
{
	phy_info_t *other_pi = phy_get_other_pi(pi);
	/* WAR:  Simultaneous CAL + Tx in RSDB mode results in
	   Chip hang due to excess current consumption. SUSPEND MAC
	   for the other core during cal on current core and enable it
	   after the cal is complete
	 */
	if (ACMAJORREV_4(pi->pubpi->phy_rev) &&
		(phy_get_phymode(pi) == PHYMODE_RSDB) &&
		!PUB_NOT_ASSOC(other_pi)) {
		if (suspend == TRUE) {
			wlapi_suspend_mac_and_wait(other_pi->sh->physhim);
		} else {
			wlapi_enable_mac(other_pi->sh->physhim);
		}
	}
}
void
wlc_phy_cals_acphy(phy_info_t *pi, uint8 searchmode)
{

	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	uint8 core;
	uint8 tx_pwr_ctrl_state;
	uint8 phase_id = pi->cal_info->cal_phase_id;
	acphy_cal_result_t *accal = &pi->cal_info->u.accal;
	uint16 tbl_cookie = TXCAL_CACHE_VALID;
	int8 tx_idx, tx_idx_pwr;
	uint16 cal_exec_time, papd_cal_time;
	uint8 papdmode = pi->u.pi_acphy->papdmode;
	uint8 band;

	uint16 coremask = 0;
	/* Local copy of phyrxchains & EnTx bits before overwrite */
	uint8 enRx = 0;
	uint8 enTx = 0;

	uint8 iqlocal_tbl_id = wlc_phy_get_tbl_id_iqlocal(pi, 0);
	int16 idac_i = 0, idac_q = 0;
	uint32 cal_start_time;

#if defined(PHYCAL_CACHING)
	ch_calcache_t *ctx = NULL;
	ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
#endif

	BCM_REFERENCE(coremask);
	BCM_REFERENCE(enRx);
	BCM_REFERENCE(enTx);

	if (NORADIO_ENAB(pi->pubpi))
		return;

	PHY_CAL(("wl%d: Running ACPHY periodic calibration: Searchmode: %d. phymode: 0x%x \n",
	         pi->sh->unit, searchmode, phy_get_phymode(pi)));

	/* -----------------
	 *  Initializations
	 * -----------------
	 */

	/* Exit immediately if we are running on Quickturn */
	if (ISSIM_ENAB(pi->sh->sih)) {
		wlc_phy_cal_perical_mphase_reset(pi);
		return;
	}

	/* skip cal if phy is muted */
	if (PHY_MUTED(pi) && !TINY_RADIO(pi)) {
		return;
	}

	cal_start_time = OSL_SYSUPTIME();

	/* Get current subband information */
	band = wlc_phy_get_chan_freq_range_acphy(pi, 0, PRIMARY_FREQ_SEGMENT);

	if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		wlc_phy_enable_lna_dcc_comp_20691(pi, 0);

		if (PHY_ILNA(pi)) {
			wlc_idac_read_20691(pi, &idac_i, &idac_q);
		}
	}

	if (TINY_RADIO(pi)) {
		/* switch back to original rx2tx seq and dly for tiny cal */
		wlc_phy_tiny_rfseq_mode_set(pi, 1);
		if (pi->u.pi_acphy->dac_mode != 1) {
			pi->u.pi_acphy->dac_mode = 1;
			wlc_phy_dac_rate_mode_acphy(pi, pi->u.pi_acphy->dac_mode);
		}
	}
	/* Save and overwrite Rx chains */
	wlc_phy_update_rxchains((wlc_phy_t *)pi, &enRx, &enTx);

	if ((phase_id == MPHASE_CAL_STATE_IDLE) || (phase_id == ACPHY_CAL_PHASE_RXCAL)) {
	  if ((pi->u.pi_acphy->crsmincal_enable)) {
			PHY_CAL(("%s : crsminpwr cal\n", __FUNCTION__));
			pi->u.pi_acphy->force_crsmincal = TRUE;
			wlc_phy_noise_sample_request_crsmincal((wlc_phy_t*)pi);
		}
	}

	/*
	 * Search-Mode Sanity Check for Tx-iqlo-Cal
	 *
	 * Notes: - "RESTART" means: start with 0-coeffs and use large search radius
	 *        - "REFINE"  means: start with latest coeffs and only search
	 *                    around that (faster)
	 *        - here, if channel has changed or no previous valid coefficients
	 *          are available, enforce RESTART search mode (this shouldn't happen
	 *          unless cal driver code is work-in-progress, so this is merely a safety net)
	 */
	if ((pi->radio_chanspec != accal->chanspec) ||
	    (accal->txiqlocal_coeffsvalid == 0)) {
		searchmode = PHY_CAL_SEARCHMODE_RESTART;
	}

	/*
	 * If previous phase of multiphase cal was on different channel,
	 * then restart multiphase cal on current channel (again, safety net)
	 */
	if ((phase_id > MPHASE_CAL_STATE_INIT)) {
		if (accal->chanspec != pi->radio_chanspec) {
			wlc_phy_cal_perical_mphase_restart(pi);
		}
	}

#ifdef WFD_PHY_LL_DEBUG
	uint32 cal_phase_id = pi->cal_info->cal_phase_id;
	uint32 start_time = hnd_time_us();
#endif

	/* Make the ucode send a CTS-to-self packet with duration set to 10ms. This
	 *  prevents packets from other STAs/AP from interfering with Rx IQcal
	 */
	/* Disable Power control */
	tx_pwr_ctrl_state = pi->txpwrctrl;

	/* If single phase cal send out CTS to self to ensure assoc/join */

	cal_exec_time = 29000;
	if (papdmode == PAPD_ANALYTIC) {
		papd_cal_time = 8000;
	} else if (papdmode == PAPD_ANALYTIC_WO_YREF) {
		papd_cal_time = 5000;
	} else {
		papd_cal_time = 12000;
	}

	if (phase_id == MPHASE_CAL_STATE_IDLE) {
		if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
			if PHY_PAPDEN(pi) {
				cal_exec_time = cal_exec_time + papd_cal_time;
			}
		}
		wlc_phy_susp2tx_cts2self(pi, cal_exec_time);
	}

	pi->u.pi_acphy->radar_cal_active = TRUE;

	/* -------------------
	 *  Calibration Calls
	 * -------------------
	 */

	PHY_CAL(("wlc_phy_cals_acphy: Time=%d, LastTi=%d, SrchMd=%d, PhIdx=%d,"
		" Chan=%d, LastCh=%d, First=%d, vld=%d\n",
		pi->sh->now, pi->cal_info->last_cal_time, searchmode, phase_id,
		pi->radio_chanspec, accal->chanspec,
		pi->first_cal_after_assoc, accal->txiqlocal_coeffsvalid));

	if (phase_id == MPHASE_CAL_STATE_IDLE) {
		/*
		 * SINGLE-SHOT Calibrations
		 *
		 *    Call all Cals one after another
		 *
		 *    Notes:
		 *    - if this proc is called with the phase state in IDLE,
		 *      we know that this proc was called directly rather
		 *      than via the mphase scheduler (the latter puts us into
		 *      INIT state); under those circumstances, perform immediate
		 *      execution over all cal tasks
		 *    - for better code structure, we would use the below mphase code for
		 *      sphase case, too, by utilizing an appropriate outer for-loop
		 */

		/* TO-DO: Ensure that all inits and cleanups happen here */


		/* carry out all phases "en bloc", for comments see the various phases below */
		pi->cal_info->last_cal_time     = pi->sh->now;
		accal->chanspec = pi->radio_chanspec;

		if (TINY_RADIO(pi) && (READ_RADIO_REGFLD_20691(pi, PLL_DSPR27, 0,
			rfpll_monitor_need_refresh) == 1)) {
			wlc_phy_radio_tiny_vcocal(pi);
		}

		wlc_phy_precal_txgain_acphy(pi, accal->txcal_txgain);
		wlc_phy_cal_txiqlo_acphy(pi, searchmode, FALSE, 0); /* request "Sphase" */


		if (TINY_RADIO(pi)) {
			wlc_phy_tiny_static_dc_offset_cal(pi);
		} else {
		  wlc_phy_precal_txgain_acphy(pi, accal->txcal_txgain);
			wlc_phy_cal_txiqlo_acphy(pi, searchmode, FALSE, 1); /* request "Sphase" */
		}
		wlc_phy_txpwrctrl_idle_tssi_meas_acphy(pi);
		wlc_phy_cals_mac_susp_en_other_cr(pi, TRUE);
		wlc_phy_cal_rx_fdiqi_acphy(pi);
		wlc_phy_cals_mac_susp_en_other_cr(pi, FALSE);

		phy_ac_dssf(pi_ac->rxspuri, TRUE);

		wlc_phy_table_write_acphy(pi, iqlocal_tbl_id, 1,
		                          IQTBL_CACHE_COOKIE_OFFSET, 16, &tbl_cookie);

		/* Do PAPD cal */
		if (PHY_PAPDEN(pi)) {
			PHY_PAPD(("PAPD : PHY_IPA(pi) = %d", PHY_IPA(pi)));
			if (TINY_RADIO(pi)) {
				if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
					/* use for phy_pacalidx0 and phy_pacalidx1 iovar */
					pi->u.pi_acphy->papd_lut0_cal_idx = -1;
					pi->u.pi_acphy->papd_lut1_cal_idx = -1;

					/* 4th priority: default cal index */
					if (CHSPEC_IS2G(pi->radio_chanspec)) {
						tx_idx = 26;
					} else {
						tx_idx = 30;
					}
					if (PHY_EPAPD(pi)) {
						if (CHSPEC_IS2G(pi->radio_chanspec)) {
							tx_idx = 48;
						} else {
							if (band <= WL_CHAN_FREQ_RANGE_5G_BAND1) {
								tx_idx = 44;
							} else {
								tx_idx = 52;
							}
						}
					}

					/* 3rd priority: pacalindex from nvram */
					if (CHSPEC_IS2G(pi->radio_chanspec) &&
						(pi->pacalindex2g != -1)) {
						tx_idx = pi->pacalindex2g;
					} else {
						if ((band <= WL_CHAN_FREQ_RANGE_5G_BAND1) &&
							(pi->pacalindex5g[0] != -1)) {
							tx_idx = pi->pacalindex5g[0];
						} else if ((band == WL_CHAN_FREQ_RANGE_5G_BAND2) &&
							(pi->pacalindex5g[1] != -1)) {
							tx_idx = pi->pacalindex5g[1];
						} else if ((band == WL_CHAN_FREQ_RANGE_5G_BAND3) &&
							(pi->pacalindex5g[2] != -1)) {
							tx_idx = pi->pacalindex5g[2];
						}
					}

					/* 2nd priority: pacalpwr from nvram */
					tx_idx_pwr = wlc_phy_tone_pwrctrl(pi, 96, 0);
					if (tx_idx_pwr != -1)
						tx_idx = tx_idx_pwr;

					 /* 1st priority: force cal index through iovar */
					if (pi->u.pi_acphy->pacalidx_iovar != -1) {
						tx_idx = pi->u.pi_acphy->pacalidx_iovar;
					}

					pi->u.pi_acphy->papd_lut0_cal_idx = tx_idx;
					wlc_phy_txpwr_by_index_acphy(pi, 1, tx_idx);
				}

				wlc_phy_cals_mac_susp_en_other_cr(pi, TRUE);

				wlc_phy_tiny_papd_cal_run_acphy(pi, tx_pwr_ctrl_state);

				wlc_phy_cals_mac_susp_en_other_cr(pi, FALSE);

			} else {
				wlc_phy_txpwr_papd_cal_run_acphy(pi, tx_pwr_ctrl_state);
			}
		} else {
			FOREACH_CORE(pi, core) {
				MOD_PHYREGCEE(pi, PapdEnable, core, papd_compEnb, 0);
			}
		}
		pi->first_cal_after_assoc = FALSE;

#if !defined(PHYCAL_CACHING)
		pi->u.pi_acphy->txcal_cache_cookie = 0;
		/* cache cals for restore on return to home channel */
		wlc_phy_scanroam_cache_cal_acphy(pi_ac->calmgri, 1);
#endif /* !defined(PHYCAL_CACHING) */
#if defined(PHYCAL_CACHING)
		if (ctx)
			wlc_phy_cal_cache_acphy((wlc_phy_t *)pi);
#endif

	} else {
		/*
		 * MULTI-PHASE CAL
		 *
		 *   Carry out next step in multi-phase execution of cal tasks
		 *
		 */

		switch (phase_id) {
		case ACPHY_CAL_PHASE_INIT:

			/*
			 *   Housekeeping & Pre-Txcal Tx Gain Adjustment
			 */

#ifdef WFD_PHY_LL
			/* Single-core on 20MHz channel */
			wlc_phy_susp2tx_cts2self(pi, 2500);
#else
			wlc_phy_susp2tx_cts2self(pi, 4000);
#endif

			/* remember time and channel of this cal event */
			pi->cal_info->last_cal_time     = pi->sh->now;
			accal->chanspec = pi->radio_chanspec;

			wlc_phy_precal_txgain_acphy(pi, accal->txcal_txgain);

			/* move on */
			pi->cal_info->cal_phase_id++;
			break;

		case ACPHY_CAL_PHASE_TX0:
		case ACPHY_CAL_PHASE_TX1:
		case ACPHY_CAL_PHASE_TX2:
		case ACPHY_CAL_PHASE_TX3:
		case ACPHY_CAL_PHASE_TX4:
		case ACPHY_CAL_PHASE_TX5:
		case ACPHY_CAL_PHASE_TX6:
		case ACPHY_CAL_PHASE_TX7:
		case ACPHY_CAL_PHASE_TX8:
		case ACPHY_CAL_PHASE_TX9:
		case ACPHY_CAL_PHASE_TX_LAST:

			/*
			 *   Tx-IQLO-Cal
			 */
		  /* Relevant changes must be ported to ACPHY_CAL_PHASE_TXPRERXCALx as well */
			if (!ACREV_IS(pi->pubpi->phy_rev, 1) && (phase_id > ACPHY_CAL_PHASE_TX8)) {
				wlc_phy_susp2tx_cts2self(pi, 0);
				pi->cal_info->cal_phase_id++;
				break;
			}

#ifdef WFD_PHY_LL
			/* Single-core on 20MHz channel */
			wlc_phy_susp2tx_cts2self(pi, 3000);
#else
			wlc_phy_susp2tx_cts2self(pi, 4400);
#endif

			/* to ensure radar detect is skipped during cals */
			if ((pi->radar_percal_mask & 0x10) != 0) {
				pi->u.pi_acphy->radar_cal_active = TRUE;
			}

			if (wlc_phy_cal_txiqlo_acphy(pi, searchmode, TRUE, 0) != BCME_OK) {
				/* rare case, just reset */
				PHY_ERROR(("wlc_phy_cal_txiqlo_acphy failed\n"));
				wlc_phy_cal_perical_mphase_reset(pi);
				break;
			}

			/* move on */
			pi->cal_info->cal_phase_id++;
			break;

		case ACPHY_CAL_PHASE_PAPDCAL:
			wlc_phy_cals_mac_susp_en_other_cr(pi, TRUE);
			if (ACMAJORREV_1(pi->pubpi->phy_rev) &&
				ACMINORREV_2(pi) && PHY_PAPDEN(pi)) {
				if ((pi->radar_percal_mask & 0x20) != 0) {
					pi->u.pi_acphy->radar_cal_active = TRUE;
				}
			}

#ifdef WFD_PHY_LL
			if (pi->wfd_ll_enable) {
				/* skip the PAPD calibration */
				wlc_phy_susp2tx_cts2self(pi, 0);
				pi->cal_info->cal_phase_id++;
				break;
			}
#endif
			if (PHY_PAPDEN(pi)) {
			        wlc_phy_susp2tx_cts2self(pi, papd_cal_time);
				if (TINY_RADIO(pi)) {
					if (CHSPEC_IS2G(pi->radio_chanspec)) {
						tx_idx = 26;
					} else {
						tx_idx = 22;
					}
					if (PHY_EPAPD(pi)) {
						if (CHSPEC_IS2G(pi->radio_chanspec)) {
							tx_idx = 48;
						} else {
							if (band <= WL_CHAN_FREQ_RANGE_5G_BAND1) {
								tx_idx = 20;
							} else {
								tx_idx = 28;
							}
						}
					}
					wlc_phy_txpwr_by_index_acphy(pi, 1, tx_idx);
					wlc_phy_tiny_papd_cal_run_acphy(pi, tx_pwr_ctrl_state);
				} else {
					wlc_phy_txpwr_papd_cal_run_acphy(pi, tx_pwr_ctrl_state);
				}
			} else {
			  /* To make phyreg_enter & mac_suspend in sync for PAPD_EN =0 */
			       wlc_phy_susp2tx_cts2self(pi, 0);
			}
			wlc_phy_cals_mac_susp_en_other_cr(pi, FALSE);

			/* move on */
			pi->cal_info->cal_phase_id++;
			break;

		case ACPHY_CAL_PHASE_TXPRERXCAL0:
		case ACPHY_CAL_PHASE_TXPRERXCAL1:
		case ACPHY_CAL_PHASE_TXPRERXCAL2:

#ifdef WFD_PHY_LL
			/* Single-core on 20MHz channel */
			wlc_phy_susp2tx_cts2self(pi, 3000);
#else
			wlc_phy_susp2tx_cts2self(pi, 4400);
#endif

			/* to ensure radar detect is skipped during cals */
			if ((pi->radar_percal_mask & 0x10) != 0) {
				pi->u.pi_acphy->radar_cal_active = TRUE;
			}
			if (wlc_phy_cal_txiqlo_acphy(pi, searchmode, TRUE, 1) != BCME_OK) {
				/* rare case, just reset */
				PHY_ERROR(("wlc_phy_cal_txiqlo_acphy failed\n"));
				wlc_phy_cal_perical_mphase_reset(pi);
				break;
			}

			/* move on */
			pi->cal_info->cal_phase_id++;
			break;
		case ACPHY_CAL_PHASE_RXCAL:
			/*
			 *   Rx IQ Cal
			 */

#ifdef WFD_PHY_LL
			/* Single-core on 20MHz channel */
			wlc_phy_susp2tx_cts2self(pi, 3000);
#else
			wlc_phy_susp2tx_cts2self(pi, 9500);
#endif

			if ((pi->radar_percal_mask & 0x1) != 0) {
				pi->u.pi_acphy->radar_cal_active = TRUE;
			}

			wlc_phy_cals_mac_susp_en_other_cr(pi, TRUE);
			wlc_phy_cal_rx_fdiqi_acphy(pi);
			wlc_phy_cals_mac_susp_en_other_cr(pi, FALSE);

#if !defined(PHYCAL_CACHING)
			pi->u.pi_acphy->txcal_cache_cookie = 0;
			/* cache cals for restore on return to home channel */
			wlc_phy_scanroam_cache_cal_acphy(pi_ac->calmgri, 1);
#endif /* !defined(PHYCAL_CACHING) */

			/* move on */
			pi->cal_info->cal_phase_id++;
			break;

		case ACPHY_CAL_PHASE_RSSICAL:

			/*
			 *     RSSI Cal & VCO Cal
			 */

#ifdef WFD_PHY_LL
			/* Single-core on 20MHz channel */
			wlc_phy_susp2tx_cts2self(pi, 600);
#else
			wlc_phy_susp2tx_cts2self(pi, 300);
#endif

			if ((pi->radar_percal_mask & 0x4) != 0) {
			    pi->u.pi_acphy->radar_cal_active = TRUE;
			}

			/* RSSI & VCO cal (prevents VCO/PLL from losing lock with temp delta) */

			if (TINY_RADIO(pi))
				wlc_phy_radio_tiny_vcocal(pi);
			else
				wlc_phy_radio2069_vcocal(pi);

			wlc_phy_radio2069x_vcocal_isdone(pi, TRUE);

			pi->cal_info->last_cal_time = pi->sh->now;
			accal->chanspec = pi->radio_chanspec;

			/* If this is the first calibration after association then we
			 * still have to do calibrate the idle-tssi, otherrwise done
			 */
			if (pi->first_cal_after_assoc) {
				pi->cal_info->cal_phase_id++;
			} else {
				wlc_phy_table_write_acphy(pi, iqlocal_tbl_id, 1,
				                          IQTBL_CACHE_COOKIE_OFFSET, 16,
				                          &tbl_cookie);

#if defined(PHYCAL_CACHING)
				if (ctx)
					wlc_phy_cal_cache_acphy((wlc_phy_t *)pi);
#endif

				wlc_phy_cal_perical_mphase_reset(pi);
			}
			break;

		case ACPHY_CAL_PHASE_IDLETSSI:

			/*
			 *     Idle TSSI & TSSI-to-dBm Mapping Setup
			 */


			wlc_phy_susp2tx_cts2self(pi, 1550);
			if ((pi->radar_percal_mask & 0x8) != 0)
				pi->u.pi_acphy->radar_cal_active = TRUE;

			/* Idle TSSI determination once right after join/up/assoc */
			wlc_phy_txpwrctrl_idle_tssi_meas_acphy(pi);
			/* done with multi-phase cal, reset phase */
			pi->first_cal_after_assoc = FALSE;

			wlc_phy_table_write_acphy(pi, iqlocal_tbl_id, 1,
			  IQTBL_CACHE_COOKIE_OFFSET, 16, &tbl_cookie);

#if defined(PHYCAL_CACHING)
			if (ctx)
				wlc_phy_cal_cache_acphy((wlc_phy_t *)pi);
#endif

			wlc_phy_cal_perical_mphase_reset(pi);
			break;

		default:
			PHY_ERROR(("%s: Invalid calibration phase %d\n", __FUNCTION__, phase_id));
			ASSERT(0);
			wlc_phy_cal_perical_mphase_reset(pi);
			break;
		}
	}

	/* ----------
	 *  Cleanups
	 * ----------
	 */
	if ((PHY_IPA(pi)) && (tx_pwr_ctrl_state == PHY_TPC_HW_ON) && (!TINY_RADIO(pi))) {
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			MOD_PHYREGCEE(pi, EpsilonTableAdjust, core, epsilonOffset, 0);
		}
	}
	wlc_phy_txpwrctrl_enable_acphy(pi, tx_pwr_ctrl_state);
	/* Restore Rx chains */
	wlc_phy_restore_rxchains((wlc_phy_t *)pi, enRx, enTx);

	if (TINY_RADIO(pi)) {
	/* switch to normal rx2tx seq and dly after tiny cal */
		wlc_phy_tiny_rfseq_mode_set(pi, 0);
		if (pi->u.pi_acphy->dac_mode != (CHSPEC_IS2G(pi->radio_chanspec)
			? pi->dacratemode2g : pi->dacratemode5g)) {
			pi->u.pi_acphy->dac_mode = CHSPEC_IS2G(pi->radio_chanspec)
				? pi->dacratemode2g : pi->dacratemode5g;
			wlc_phy_dac_rate_mode_acphy(pi, pi->u.pi_acphy->dac_mode);
		}

		if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
			if (PHY_ILNA(pi)) {
				wlc_idac_preload_20691(pi, idac_i, idac_q);
				wlc_phy_enable_lna_dcc_comp_20691(pi, PHY_ILNA(pi));
			} else {
				wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
				wlc_dcc_fsm_reset(pi);
				wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
			}
		}
	}

	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);

#ifdef WFD_PHY_LL_DEBUG
	printf("phase_id:%2d usec:%d\n", cal_phase_id, hnd_time_us() - start_time);
#endif
	pi->cal_dur += OSL_SYSUPTIME() - cal_start_time;
}

#if defined(BCMDBG)
/* dump calibration regs/info */
void
wlc_phy_cal_dump_acphy(phy_info_t *pi, struct bcmstrbuf *b)
{
	uint8 core;
	int8  ac_reg, mf_reg, off1 = 0, off2 = 0;
	int16  a_reg, b_reg, a_int, b_int;
	int32 slope;
	uint16 ab_int[2], d_reg;
	uint16 coremask;

	if (!pi->sh->up) {
		return;
	}

	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);

	if (ACMAJORREV_4(pi->pubpi->phy_rev) && (phy_get_phymode(pi) != PHYMODE_RSDB)) {
		coremask = pi->sh->hw_phyrxchain;
	} else {
		coremask = pi->sh->phyrxchain;
	}

	bcm_bprintf(b, "Tx-IQ/LOFT-Cal:\n");
	FOREACH_ACTV_CORE(pi, coremask, core) {
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ, ab_int,
			TB_OFDM_COEFFS_AB, core);
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ, &d_reg,
			TB_OFDM_COEFFS_D, core);
		if (TINY_RADIO(pi)) {
			bcm_bprintf(b, "   core-%d: a/b: (%4d,%4d), d: (%3d,%3d)\n",
				core, (int16) ab_int[0], (int16) ab_int[1],
				(int8)((d_reg & 0xFF00) >> 8), /* di */
				(int8)((d_reg & 0x00FF)));     /* dq */
		} else {
			uint16 eir, eqr, fir, fqr;

			eir = READ_RADIO_REGC(pi, RF, TXGM_LOFT_FINE_I, core);
			eqr = READ_RADIO_REGC(pi, RF, TXGM_LOFT_FINE_Q, core);
			fir = READ_RADIO_REGC(pi, RF, TXGM_LOFT_COARSE_I, core);
			fqr = READ_RADIO_REGC(pi, RF, TXGM_LOFT_COARSE_Q, core);
			bcm_bprintf(b, "   core-%d: a/b: (%4d,%4d), d: (%3d,%3d),"
				" e: (%3d,%3d), f: (%3d,%3d)\n",
				core, (int16) ab_int[0], (int16) ab_int[1],
				(int8)((d_reg & 0xFF00) >> 8), /* di */
				(int8)((d_reg & 0x00FF)),      /* dq */
				(int8)(-((eir & 0xF0) >> 4) + ((eir & 0xF))), /* ei */
				(int8)(-((eqr & 0xF0) >> 4) + ((eqr & 0xF))), /* eq */
				(int8)(-((fir & 0xF0) >> 4) + ((fir & 0xF))), /* fi */
				(int8)(-((fqr & 0xF0) >> 4) + ((fqr & 0xF))));  /* fq */
		}
	}
	bcm_bprintf(b, "Rx-IQ-Cal:\n");
	FOREACH_ACTV_CORE(pi, coremask, core) {
		a_reg = READ_PHYREGCE(pi, Core1RxIQCompA, core);
		b_reg = READ_PHYREGCE(pi, Core1RxIQCompB, core);
		a_int = (a_reg >= 512) ? a_reg - 1024 : a_reg; /* s0.9 format */
		b_int = (b_reg >= 512) ? b_reg - 1024 : b_reg;
		if (pi->u.pi_acphy->fdiqi->enabled) {
			slope = pi->u.pi_acphy->fdiqi->slope[core];
			bcm_bprintf(b, "   core-%d: a/b = (%4d,%4d), S = %2d (%1d)\n",
				core, a_int, b_int, slope,
				READ_PHYREGFLD(pi, rxfdiqImbCompCtrl, rxfdiqImbCompEnable));
		} else {
			bcm_bprintf(b, "   core-%d: a/b = (%4d,%4d), S = OFF (%1d)\n",
				core, a_int, b_int,
				READ_PHYREGFLD(pi, rxfdiqImbCompCtrl, rxfdiqImbCompEnable));
		}
	}

	ac_reg =  READ_PHYREGFLD(pi, crsminpoweru0, crsminpower0);
	mf_reg =  READ_PHYREGFLD(pi, crsmfminpoweru0, crsmfminpower0);

	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		if (core == 1) {
			off1 =    READ_PHYREGFLD(pi, crsminpoweroffset1, crsminpowerOffsetu);
		} else if (core == 2) {
			off2 =    READ_PHYREGFLD(pi, crsminpoweroffset2, crsminpowerOffsetu);
		}
	}
	bcm_bprintf(b, "crs_min_pwr cal:\n");
	if (pi->u.pi_acphy->crsmincal_run != 1) {
		bcm_bprintf(b, "  ACI desense is on:  crs_min_pwr cal DID NOT run\n");
	} else {
		bcm_bprintf(b, "   crsmin_cal ran %d times for channel %d:\n",
		            pi->u.pi_acphy->phy_debug_crscal_counter,
		            pi->u.pi_acphy->phy_debug_crscal_channel);
	}
	bcm_bprintf(b, "   Noise power used for setting crs_min thresholds : ");
	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		bcm_bprintf(b, "Core-%d : %d, ", core, pi->u.pi_acphy->phy_noise_in_crs_min[core]);
	}

	bcm_bprintf(b, "\n");
	bcm_bprintf(b, "   AC-CRS = %d,", ac_reg);
	bcm_bprintf(b, "   MF-CRS = %d,", mf_reg);
	bcm_bprintf(b, "   Offset 1 = %d,", off1);
	bcm_bprintf(b, "   Offset 2 = %d\n", off2);
#if defined(BCMDBG)
	if (PHY_PAPDEN(pi)) {
		/* Make dump available for both iPA and ePA */
		wlc_phy_papd_dump_eps_trace_acphy(pi, b);
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
				bcm_bprintf(b, "papdcalidx%d %d\n", core, papd_gainctrl_pga[core]);
			} else if (ACMAJORREV_2(pi->pubpi->phy_rev) ||
			           ACMAJORREV_5(pi->pubpi->phy_rev)) {
				bcm_bprintf(b, "papdcalidx%d %d\n", core, papd_gainctrl_pga[core]);
			}
		}
	}
#endif  
	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);

	return;
}
#endif	

static void
BCMATTACHFN(wlc_phy_nvram_rssioffset_read)(phy_info_t *pi)
{
	uint8 i, j, ant;
	uint8 core;
	char phy_var_name[40];
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	uint8 subband_idx;
	const char *subband_key[4] = {"l", "ml", "mu", "h"};

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rxgaintempcoeff2g);
	pi_ac->sromi->rxgain_tempadj_2g =
		(int16)PHY_GETINTVAR(pi, phy_var_name);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rxgaintempcoeff5gl);
	pi_ac->sromi->rxgain_tempadj_5gl =
		(int16)PHY_GETINTVAR(pi, phy_var_name);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rxgaintempcoeff5gml);
	pi_ac->sromi->rxgain_tempadj_5gml =
		(int16)PHY_GETINTVAR(pi, phy_var_name);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rxgaintempcoeff5gmu);
	pi_ac->sromi->rxgain_tempadj_5gmu =
		(int16)PHY_GETINTVAR(pi, phy_var_name);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rxgaintempcoeff5gh);
	pi_ac->sromi->rxgain_tempadj_5gh =
		(int16)PHY_GETINTVAR(pi, phy_var_name);

	FOREACH_CORE(pi, core) {
		ant = phy_get_rsdbbrd_corenum(pi, core);
		(void)snprintf(phy_var_name, sizeof(phy_var_name), rstr_rssicorrnorm_cD, ant);
		if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
			for (i = 0; i < ACPHY_NUM_BW_2G; i++) {
				pi_ac->sromi->rssioffset.rssi_corr_normal[ant][i] =
				        (int8)PHY_GETINTVAR_ARRAY(pi, phy_var_name, i);
			}
		} else {
			for (i = 0; i < ACPHY_NUM_BW_2G; i++) {
				pi_ac->sromi->rssioffset.rssi_corr_normal[ant][i] = 0;
			}
		}

		(void)snprintf(phy_var_name, sizeof(phy_var_name), rstr_rssicorrnorm5g_cD, ant);
		if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
			for (i = 0; i < ACPHY_RSSIOFFSET_NVRAM_PARAMS; i++) {
				for (j = 0; j < ACPHY_NUM_BW; j++) {
					pi_ac->sromi->rssioffset.rssi_corr_normal_5g[ant][i][j]
					        = (int8)PHY_GETINTVAR_ARRAY(pi, phy_var_name,
					                                    (3*i+j));
				}
			}
		} else {
			for (i = 0; i < ACPHY_RSSIOFFSET_NVRAM_PARAMS; i++) {
				for (j = 0; j < ACPHY_NUM_BW; j++) {
					pi_ac->sromi->rssioffset.rssi_corr_normal_5g[ant][i][j]
					        = 0;
				}
			}
		}

		(void)snprintf(phy_var_name, sizeof(phy_var_name), rstr_rssi_delta_2g_cD, ant);
		if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
			for (j = 0; j < ACPHY_NUM_BW_2G; j++) {
				for (i = 0; i < ACPHY_GAIN_DELTA_2G_PARAMS; i++) {
				pi_ac->sromi->rssioffset.rssi_corr_gain_delta_2g[ant][i][j] =
				        (int8)PHY_GETINTVAR_ARRAY(pi, phy_var_name, (i+2*j));
				}
			}
		} else {
			for (j = 0; j < ACPHY_NUM_BW_2G; j++) {
				for (i = 0; i < ACPHY_GAIN_DELTA_2G_PARAMS; i++) {
					pi_ac->sromi->rssioffset.
					        rssi_corr_gain_delta_2g[ant][i][j] = 0;
				}
			}
		}

		for (subband_idx = 0; subband_idx < CH_5G_4BAND; subband_idx++) {
			(void)snprintf(phy_var_name, sizeof(phy_var_name),
			               rstr_rssi_delta_5gS_cD, subband_key[subband_idx], ant);

			if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
				for (j = 0; j < ACPHY_NUM_BW; j++) {
					for (i = 0; i < ACPHY_GAIN_DELTA_5G_PARAMS; i++) {
						pi_ac->sromi->rssioffset.rssi_corr_gain_delta_5g
						        [ant][i][j][subband_idx] = (int8)
						        PHY_GETINTVAR_ARRAY(pi, phy_var_name,
						                            (i+2*j));
					}
				}
			} else {
				for (j = 0; j < ACPHY_NUM_BW; j++) {
					for (i = 0; i < ACPHY_GAIN_DELTA_5G_PARAMS; i++) {
						pi_ac->sromi->rssioffset.rssi_corr_gain_delta_5g
							[ant][i][j][subband_idx] = 0;
					}
				}
			}
		}

		for (j = 0; j < ACPHY_NUM_BW_2G; j++) {
			for (i = 0; i < ACPHY_GAIN_DELTA_2G_PARAMS; i++) {
				PHY_INFORM(("%d ", pi_ac->sromi->rssioffset.rssi_corr_gain_delta_2g
				            [ant][i][j]));
			}
		}
		for (subband_idx = 0; subband_idx < CH_5G_4BAND; subband_idx++) {
			for (j = 0; j < ACPHY_NUM_BW; j++) {
				for (i = 0; i < ACPHY_GAIN_DELTA_5G_PARAMS; i++) {
				PHY_INFORM(("%d ", pi_ac->sromi->rssioffset.rssi_corr_gain_delta_5g
					            [ant][i][j][subband_idx]));
				}
			}
			PHY_INFORM(("\n"));
		}
	}
}


static void
BCMATTACHFN(wlc_phy_nvram_rssioffset_read_sub)(phy_info_t *pi)
{
	uint8 i, j, k;
	uint8 core, ant;
	char phy_var_name[40];
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	uint8 subband_idx;
	const char *subband_key[4] = {"l", "ml", "mu", "h"};
	const char *subband_key_2g[5] = {"b0", "b1", "b2", "b3", "b4"};

	/* if gain_cal_temp is not set in nvram, set it to 255 by default
	 * to disable temperature correction for rssi
	 */
	pi->srom_gain_cal_temp = (int16)PHY_GETINTVAR_DEFAULT(pi, "gain_cal_temp", 255);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rssi_cal_rev);
	pi->u.pi_acphy->rssi_cal_rev =
		(bool)PHY_GETINTVAR(pi, phy_var_name);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rxgaincal_rssical);
	pi->u.pi_acphy->rxgaincal_rssical =
		(bool)PHY_GETINTVAR(pi, phy_var_name);

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rud_agc_enable);
	pi->u.pi_acphy->rud_agc_enable =
		(bool)PHY_GETINTVAR(pi, phy_var_name);

	FOREACH_CORE(pi, core) {
		ant = phy_get_rsdbbrd_corenum(pi, core);

		for (subband_idx = 0; subband_idx < CH_2G_GROUP_NEW; subband_idx++) {
			(void)snprintf(phy_var_name, sizeof(phy_var_name),
			               rstr_rssi_delta_2gS, subband_key_2g[subband_idx]);

			if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
			  for (j = 0; j < ACPHY_NUM_BW_2G; j++) {
			    for (i = 0; i < ACPHY_GAIN_DELTA_2G_PARAMS_EXT; i++) {
			      k = ant * ACPHY_NUM_BW_2G * ACPHY_GAIN_DELTA_2G_PARAMS_EXT;
			      pi_ac->sromi->rssioffset.rssi_corr_gain_delta_2g_sub[ant]
			      [i][j][subband_idx] =
			        (int8)PHY_GETINTVAR_ARRAY(pi, phy_var_name, (i+4*j+k));
			    }
			  }
			} else {
			  for (j = 0; j < ACPHY_NUM_BW_2G; j++) {
			    for (i = 0; i < ACPHY_GAIN_DELTA_2G_PARAMS_EXT; i++) {
			      pi_ac->sromi->rssioffset.rssi_corr_gain_delta_2g_sub[ant]
				  [i][j][subband_idx] = 0;
			    }
			  }
			}
		}
		for (subband_idx = 0; subband_idx < CH_5G_4BAND; subband_idx++) {
			(void)snprintf(phy_var_name, sizeof(phy_var_name),
			               rstr_rssi_delta_5gS, subband_key[subband_idx]);

			if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
			  for (j = 0; j < ACPHY_NUM_BW; j++) {
			    for (i = 0; i < ACPHY_GAIN_DELTA_5G_PARAMS_EXT; i++) {
			      k = ant * ACPHY_NUM_BW * ACPHY_GAIN_DELTA_5G_PARAMS_EXT;
			      pi_ac->sromi->rssioffset.rssi_corr_gain_delta_5g_sub
				[ant][i][j][subband_idx] = (int8)
				PHY_GETINTVAR_ARRAY(pi, phy_var_name, (i+4*j+k));
			    }
			  }
			} else {
			  for (j = 0; j < ACPHY_NUM_BW; j++) {
			    for (i = 0; i < ACPHY_GAIN_DELTA_5G_PARAMS_EXT; i++) {
			      pi_ac->sromi->rssioffset.rssi_corr_gain_delta_5g_sub
				[ant][i][j][subband_idx] = 0;
			    }
			  }
			}
		}
		for (subband_idx = 0; subband_idx < CH_2G_GROUP_NEW; subband_idx++) {
		  for (j = 0; j < ACPHY_NUM_BW_2G; j++) {
		    for (i = 0; i < ACPHY_GAIN_DELTA_2G_PARAMS_EXT; i++) {
		      PHY_INFORM(("%d ", pi_ac->sromi->rssioffset.rssi_corr_gain_delta_2g_sub
				  [ant][i][j][subband_idx]));
		    }
		  }
		}
		for (subband_idx = 0; subband_idx < CH_5G_4BAND; subband_idx++) {
		  for (j = 0; j < ACPHY_NUM_BW; j++) {
		    for (i = 0; i < ACPHY_GAIN_DELTA_5G_PARAMS_EXT; i++) {
		      PHY_INFORM(("%d ", pi_ac->sromi->rssioffset.rssi_corr_gain_delta_5g_sub
				  [ant][i][j][subband_idx]));
		    }
		  }
		  PHY_INFORM(("\n"));
		}
	}

	(void)snprintf(phy_var_name, sizeof(phy_var_name),
	               rstr_rssi_cal_freq_grp_2g);
	j = 0;
	for (i = 0; i < 7; i++) {
		k = (uint8)
		        PHY_GETINTVAR_ARRAY(pi, phy_var_name, i);
		pi_ac->sromi->rssi_cal_freq_grp[j] = (k >> 4) & 0xf;
		j++;
		pi_ac->sromi->rssi_cal_freq_grp[j] = k & 0xf;
		j++;
	}

}

uint8 avvmid_set_from_nvram[3][5][2];

static void
BCMATTACHFN(wlc_phy_nvram_avvmid_read)(phy_info_t *pi)
{
	uint8 i, j, ant;
	uint8 core;
	char phy_var_name[20];
	/*	phy_info_acphy_t *pi_ac = pi->u.pi_acphy; */
	FOREACH_CORE(pi, core) {
		ant = phy_get_rsdbbrd_corenum(pi, core);
		(void)snprintf(phy_var_name, sizeof(phy_var_name), rstr_AvVmid_cD, ant);
		if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
			for (i = 0; i < ACPHY_NUM_BANDS; i++) {
				for (j = 0; j < ACPHY_AVVMID_NVRAM_PARAMS; j++) {
					avvmid_set_from_nvram[ant][i][j] =
						(uint8) PHY_GETINTVAR_ARRAY(pi, phy_var_name,
						(ACPHY_AVVMID_NVRAM_PARAMS*i +j));
				}
			}
		}
	}
}

static void BCMATTACHFN(wlc_phy_nvram_vlin_params_read)(phy_info_t *pi)
{

	char phy_var_name2[20], phy_var_name3[20];
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	uint8 core, ant;
	FOREACH_CORE(pi, core) {
		ant = phy_get_rsdbbrd_corenum(pi, core);
		if ((!TINY_RADIO(pi)) && BF3_VLIN_EN_FROM_NVRAM(pi_ac)) {
			(void)snprintf(phy_var_name2, sizeof(phy_var_name2),
				rstr_VlinPwr2g_cD, ant);
			if ((PHY_GETVAR(pi, phy_var_name2)) != NULL) {
				pi_ac->vlinpwr2g_from_nvram =
					(uint8) PHY_GETINTVAR(pi, phy_var_name2);
				}
			(void)snprintf(phy_var_name2, sizeof(phy_var_name2),
				rstr_VlinPwr5g_cD, ant);
			if ((PHY_GETVAR(pi, phy_var_name2)) != NULL) {
				pi_ac->vlinpwr5g_from_nvram =
					(uint8) PHY_GETINTVAR(pi, phy_var_name2);
				}
			(void)snprintf(phy_var_name3, sizeof(phy_var_name3),
				rstr_Vlinmask2g_cD, ant);
			if ((PHY_GETVAR(pi, phy_var_name3)) != NULL) {
				pi_ac->vlinmask2g_from_nvram =
					(uint8) PHY_GETINTVAR(pi, phy_var_name3);
				}
			(void)snprintf(phy_var_name3, sizeof(phy_var_name3),
				rstr_Vlinmask5g_cD, ant);
			if ((PHY_GETVAR(pi, phy_var_name3)) != NULL) {
				pi_ac->vlinmask5g_from_nvram =
					(uint8) PHY_GETINTVAR(pi, phy_var_name3);
				}
			}
		}
}

static void
BCMATTACHFN(wlc_phy_srom_read_rxgainerr_acphy)(phy_info_t *pi)
{
	/* read and uncompress gain-error values for rx power reporting */

	int8 tmp[PHY_CORE_NUM_3];
	uint8 coreidx[3] = {0, 1, 2};
	int16 tmp2;

	if (phy_get_phymode(pi) == PHYMODE_RSDB) {
		if (phy_get_current_core(pi) == PHY_RSBD_PI_IDX_CORE0) {
			/* update pi[0] to hold pwrdet params for all cores */
			/* This is required for mimo operation */
			pi->pubpi->phy_corenum <<= 1;
		} else {
			coreidx[1] = 0;
		}
	}

	(void)memset(tmp, -1, sizeof(tmp));

	/* read in temperature at calibration time */
	tmp2 = (int16) (((int16)PHY_GETINTVAR(pi, rstr_rawtempsense))  << 7) >> 7;
	if (tmp2 == -1) {
		/* set to some bogus value, since nothing was written to SROM */
		pi->srom_rawtempsense = 255;
	} else {
		pi->srom_rawtempsense = tmp2;
	}
	pi->u.pi_acphy->current_temperature = pi->srom_rawtempsense;

	/* 2G: */
	/* read and sign-extend */
	tmp[0] = (int8)(((int8)PHY_GETINTVAR(pi, rstr_rxgainerr2ga0)) << 2) >> 2;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		tmp[1] = (int8)(((int8)PHY_GETINTVAR(pi, rstr_rxgainerr2ga1)) << 3) >> 3;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		tmp[2] = (int8)(((int8)PHY_GETINTVAR(pi, rstr_rxgainerr2ga2)) << 3) >> 3;

	if ((tmp[0] == -1) && (tmp[1] == -1) && (tmp[2] == -1) && (tmp2 == -1)) {
		/* If all srom values are -1, then possibly
		 * no gainerror info was written to srom
		 */
		tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
		pi->rxgainerr2g_isempty = TRUE;
	} else {
		pi->rxgainerr2g_isempty = FALSE;
	}
	/* gain errors for cores 1 and 2 are stored in srom as deltas relative to core 0: */
	pi->rxgainerr_2g[coreidx[0]] = tmp[0];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		pi->rxgainerr_2g[coreidx[1]] = tmp[0] + tmp[1];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		pi->rxgainerr_2g[coreidx[2]] = tmp[0] + tmp[2];

	/* 5G low: */
	/* read and sign-extend */
	tmp[0] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga0, 0)) << 2) >> 2;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		tmp[1] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga1, 0)) << 3) >> 3;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		tmp[2] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga2, 0)) << 3) >> 3;

	if ((tmp[0] == -1) && (tmp[1] == -1) && (tmp[2] == -1) && (tmp2 == -1)) {
		/* If all srom values are -1, then possibly
		 * no gainerror info was written to srom
		 */
		tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
		pi->rxgainerr5gl_isempty = TRUE;
	} else {
		pi->rxgainerr5gl_isempty = FALSE;
	}
	/* gain errors for cores 1 and 2 are stored in srom as deltas relative to core 0: */
	pi->rxgainerr_5gl[coreidx[0]] = tmp[0];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		pi->rxgainerr_5gl[coreidx[1]] = tmp[0] + tmp[1];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		pi->rxgainerr_5gl[coreidx[2]] = tmp[0] + tmp[2];

	/* 5G mid: */
	/* read and sign-extend */
	tmp[0] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga0, 1)) << 2) >> 2;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		tmp[1] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga1, 1)) << 3) >> 3;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		tmp[2] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga2, 1)) << 3) >> 3;

	if ((tmp[0] == -1) && (tmp[1] == -1) && (tmp[2] == -1) && (tmp2 == -1)) {
		/* If all srom values are -1, then possibly
		 * no gainerror info was written to srom
		 */
		tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
		pi->rxgainerr5gm_isempty = TRUE;
	} else {
		pi->rxgainerr5gm_isempty = FALSE;
	}
	/* gain errors for cores 1 and 2 are stored in srom as deltas relative to core 0: */
	pi->rxgainerr_5gm[coreidx[0]] = tmp[0];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		pi->rxgainerr_5gm[coreidx[1]] = tmp[0] + tmp[1];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		pi->rxgainerr_5gm[coreidx[2]] = tmp[0] + tmp[2];

	/* 5G high: */
	/* read and sign-extend */
	tmp[0] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga0, 2)) << 2) >> 2;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		tmp[1] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga1, 2)) << 3) >> 3;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		tmp[2] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga2, 2)) << 3) >> 3;

	if ((tmp[0] == -1) && (tmp[1] == -1) && (tmp[2] == -1) && (tmp2 == -1)) {
		/* If all srom values are -1, then possibly
		 * no gainerror info was written to srom
		 */
		tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
		pi->rxgainerr5gh_isempty = TRUE;
	} else {
		pi->rxgainerr5gh_isempty = FALSE;
	}
	/* gain errors for cores 1 and 2 are stored in srom as deltas relative to core 0: */
	pi->rxgainerr_5gh[coreidx[0]] = tmp[0];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		pi->rxgainerr_5gh[coreidx[1]] = tmp[0] + tmp[1];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		pi->rxgainerr_5gh[coreidx[2]] = tmp[0] + tmp[2];

	/* 5G upper: */
	/* read and sign-extend */
	tmp[0] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga0, 3)) << 2) >> 2;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		tmp[1] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga1, 3)) << 3) >> 3;
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		tmp[2] = (int8)(((int8)getintvararray(pi->vars, rstr_rxgainerr5ga2, 3)) << 3) >> 3;

	if ((tmp[0] == -1) && (tmp[1] == -1) && (tmp[2] == -1) && (tmp2 == -1)) {
		/* If all srom values are -1, then possibly
		 * no gainerror info was written to srom
		 */
		tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
		pi->rxgainerr5gu_isempty = TRUE;
	} else {
		pi->rxgainerr5gu_isempty = FALSE;
	}
	/* gain errors for cores 1 and 2 are stored in srom as deltas relative to core 0: */
	pi->rxgainerr_5gu[coreidx[0]] = tmp[0];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 1)
		pi->rxgainerr_5gu[coreidx[1]] = tmp[0] + tmp[1];
	if (PHYCORENUM(pi->pubpi->phy_corenum) > 2)
		pi->rxgainerr_5gu[coreidx[2]] = tmp[0] + tmp[2];

	if ((phy_get_phymode(pi) == PHYMODE_RSDB) &&
		(phy_get_current_core(pi) == PHY_RSBD_PI_IDX_CORE0))
	{
		/* update pi[0] to hold pwrdet params for all cores */
		/* This is required for mimo operation */
		pi->pubpi->phy_corenum >>= 1;
	}

}

#define ACPHY_SROM_NOISELVL_OFFSET (-70)

static void
BCMATTACHFN(wlc_phy_srom_read_noiselvl_acphy)(phy_info_t *pi)
{
	/* read noise levels from SROM */
	uint8 core, ant;
	char phy_var_name[20];

	if (phy_get_phymode(pi) == PHYMODE_RSDB &&
		(phy_get_current_core(pi) == PHY_RSBD_PI_IDX_CORE0))
	{
		/* update pi[0] to hold pwrdet params for all cores */
		/* This is required for mimo operation */
		pi->pubpi->phy_corenum <<= 1;
	}

	FOREACH_CORE(pi, core) {
		/* 2G: */
		ant = phy_get_rsdbbrd_corenum(pi, core);
		(void)snprintf(phy_var_name, sizeof(phy_var_name), rstr_noiselvl2gaD, ant);
		pi->noiselvl_2g[core] = ACPHY_SROM_NOISELVL_OFFSET -
		                             ((uint8)PHY_GETINTVAR(pi, phy_var_name));

		/* 5G low: */
		(void)snprintf(phy_var_name, sizeof(phy_var_name), rstr_noiselvl5gaD, ant);
		pi->noiselvl_5gl[core] = ACPHY_SROM_NOISELVL_OFFSET -
		                              ((uint8)getintvararray(pi->vars, phy_var_name, 0));

		/* 5G mid: */
		pi->noiselvl_5gm[core] = ACPHY_SROM_NOISELVL_OFFSET -
		                              ((uint8)getintvararray(pi->vars, phy_var_name, 1));

		/* 5G high: */
		pi->noiselvl_5gh[core] = ACPHY_SROM_NOISELVL_OFFSET -
		                              ((uint8)getintvararray(pi->vars, phy_var_name, 2));

		/* 5G upper: */
		pi->noiselvl_5gu[core] = ACPHY_SROM_NOISELVL_OFFSET -
		                              ((uint8)getintvararray(pi->vars, phy_var_name, 3));
	}
	if ((phy_get_phymode(pi) == PHYMODE_RSDB) &&
		(phy_get_current_core(pi) == PHY_RSBD_PI_IDX_CORE0))
	{
		/* update pi[0] to hold pwrdet params for all cores */
		/* This is required for mimo operation */
		pi->pubpi->phy_corenum >>= 1;
	}
}

static bool
BCMATTACHFN(wlc_phy_srom_read_acphy)(phy_info_t *pi)
{
	/* Read rxgainctrl srom entries - elna gain, trloss */
	wlc_phy_srom_read_gainctrl_acphy(pi);
	if (!((SROMREV(pi->sh->sromrev) >= 12) ? wlc_phy_txpwr_srom12_read(pi) :
	      wlc_phy_txpwr_srom11_read(pi)))
	  return FALSE;

	wlc_phy_srom_read_rxgainerr_acphy(pi);

	wlc_phy_srom_read_noiselvl_acphy(pi);

	wlc_phy_nvram_femctrl_read(pi);

	wlc_phy_nvram_rssioffset_read(pi);

	wlc_phy_nvram_rssioffset_read_sub(pi);

	if (!TINY_RADIO(pi))
		wlc_phy_nvram_vlin_params_read(pi);

	wlc_phy_nvram_avvmid_read(pi);

#ifdef WL_SAR_SIMPLE_CONTROL
	wlc_phy_nvram_dynamicsarctrl_read(pi);
#endif

	return TRUE;
}

void
wlc_phy_init_test_acphy(phy_info_t *pi)
{

	/* Force WLAN antenna */
	wlc_btcx_override_enable(pi);
	/* Disable tx power control */
	wlc_phy_txpwrctrl_enable_acphy(pi, PHY_TPC_HW_OFF);
	/* Recalibrate for this channel */
	wlc_phy_cals_acphy(pi, PHY_CAL_SEARCHMODE_RESTART);
	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
}

static void
BCMATTACHFN(wlc_phy_srom_read_gainctrl_acphy)(phy_info_t *pi)
{
	uint8 core, srom_rx, ant;
	char srom_name[30];
	phy_info_acphy_t *pi_ac;
	uint8 raw_elna, raw_trloss, raw_bypass;

	pi_ac = pi->u.pi_acphy;
	BCM_REFERENCE(pi_ac);

#ifndef BOARD_FLAGS
	BF_ELNA_2G(pi_ac) = (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) &
	                              BFL_SROM11_EXTLNA) != 0;
	BF_ELNA_5G(pi_ac) = (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) &
	                              BFL_SROM11_EXTLNA_5GHz) != 0;
#endif /* BOARD_FLAGS3 */

	FOREACH_CORE(pi, core) {
		ant = phy_get_rsdbbrd_corenum(pi, core);
		pi_ac->sromi->femrx_2g[ant].elna = 0;
		pi_ac->sromi->femrx_2g[ant].trloss = 0;
		pi_ac->sromi->femrx_2g[ant].elna_bypass_tr = 0;

		pi_ac->sromi->femrx_5g[ant].elna = 0;
		pi_ac->sromi->femrx_5g[ant].trloss = 0;
		pi_ac->sromi->femrx_5g[ant].elna_bypass_tr = 0;

		pi_ac->sromi->femrx_5gm[ant].elna = 0;
		pi_ac->sromi->femrx_5gm[ant].trloss = 0;
		pi_ac->sromi->femrx_5gm[ant].elna_bypass_tr = 0;

		pi_ac->sromi->femrx_5gh[ant].elna = 0;
		pi_ac->sromi->femrx_5gh[ant].trloss = 0;
		pi_ac->sromi->femrx_5gh[ant].elna_bypass_tr = 0;

		/*  -------  2G -------  */
		if (BF_ELNA_2G(pi_ac)) {
			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains2gelnagainaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL) {
				srom_rx = (uint8)PHY_GETINTVAR(pi, srom_name);
				pi_ac->sromi->femrx_2g[ant].elna = (2 * srom_rx) + 6;
			}

			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains2gtrelnabypaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL) {
				pi_ac->sromi->femrx_2g[ant].elna_bypass_tr =
				        (uint8)PHY_GETINTVAR(pi, srom_name);
			}
		}

		snprintf(srom_name, sizeof(srom_name),  rstr_rxgains2gtrisoaD, ant);
		if (PHY_GETVAR(pi, srom_name) != NULL) {
			srom_rx = (uint8)PHY_GETINTVAR(pi, srom_name);
			pi_ac->sromi->femrx_2g[ant].trloss = (2 * srom_rx) + 8;
		}


		/*  -------  5G -------  */
		if (BF_ELNA_5G(pi_ac)) {
			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5gelnagainaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL) {
				srom_rx = (uint8)PHY_GETINTVAR(pi, srom_name);
				pi_ac->sromi->femrx_5g[ant].elna = (2 * srom_rx) + 6;
			}

			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5gtrelnabypaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL) {
				pi_ac->sromi->femrx_5g[ant].elna_bypass_tr =
				        (uint8)PHY_GETINTVAR(pi, srom_name);
			}
		}

		snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5gtrisoaD, ant);
		if (PHY_GETVAR(pi, srom_name) != NULL) {
			srom_rx = (uint8)PHY_GETINTVAR(pi, srom_name);
			pi_ac->sromi->femrx_5g[ant].trloss = (2 * srom_rx) + 8;
		}

		/*  -------  5G (mid) -------  */
		raw_elna = 0; raw_trloss = 0; raw_bypass = 0;
		if (BF_ELNA_5G(pi_ac)) {
			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5gmelnagainaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL)
				raw_elna = (uint8)PHY_GETINTVAR(pi, srom_name);

			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5gmtrelnabypaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL)
				raw_bypass = (uint8)PHY_GETINTVAR(pi, srom_name);
		}
		snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5gmtrisoaD, ant);
		if (PHY_GETVAR(pi, srom_name) != NULL)
			raw_trloss = (uint8)PHY_GETINTVAR(pi, srom_name);

		if (((raw_elna == 0) && (raw_trloss == 0) && (raw_bypass == 0)) ||
		    ((raw_elna == 7) && (raw_trloss == 0xf) && (raw_bypass == 1))) {
			/* No entry in SROM, use generic 5g ones */
			pi_ac->sromi->femrx_5gm[ant].elna = pi_ac->sromi->femrx_5g[ant].elna;
			pi_ac->sromi->femrx_5gm[ant].elna_bypass_tr =
			        pi_ac->sromi->femrx_5g[ant].elna_bypass_tr;
			pi_ac->sromi->femrx_5gm[ant].trloss = pi_ac->sromi->femrx_5g[ant].trloss;
		} else {
			if (BF_ELNA_5G(pi_ac)) {
				pi_ac->sromi->femrx_5gm[ant].elna = (2 * raw_elna) + 6;
				pi_ac->sromi->femrx_5gm[ant].elna_bypass_tr = raw_bypass;
			}
			pi_ac->sromi->femrx_5gm[ant].trloss = (2 * raw_trloss) + 8;
		}

		/*  -------  5G (high) -------  */
		raw_elna = 0; raw_trloss = 0; raw_bypass = 0;
		if (BF_ELNA_5G(pi_ac)) {
			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5ghelnagainaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL)
				raw_elna = (uint8)PHY_GETINTVAR(pi, srom_name);

			snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5ghtrelnabypaD, ant);
			if (PHY_GETVAR(pi, srom_name) != NULL)
				raw_bypass = (uint8)PHY_GETINTVAR(pi, srom_name);
		}
		snprintf(srom_name, sizeof(srom_name),  rstr_rxgains5ghtrisoaD, ant);
		if (PHY_GETVAR(pi, srom_name) != NULL)
			raw_trloss = (uint8)PHY_GETINTVAR(pi, srom_name);

		if (((raw_elna == 0) && (raw_trloss == 0) && (raw_bypass == 0)) ||
		    ((raw_elna == 7) && (raw_trloss == 0xf) && (raw_bypass == 1))) {
			/* No entry in SROM, use generic 5g ones */
			pi_ac->sromi->femrx_5gh[ant].elna = pi_ac->sromi->femrx_5gm[ant].elna;
			pi_ac->sromi->femrx_5gh[ant].elna_bypass_tr =
			        pi_ac->sromi->femrx_5gm[ant].elna_bypass_tr;
			pi_ac->sromi->femrx_5gh[ant].trloss = pi_ac->sromi->femrx_5gm[ant].trloss;
		} else {
			if (BF_ELNA_5G(pi_ac)) {
				pi_ac->sromi->femrx_5gh[ant].elna = (2 * raw_elna) + 6;
				pi_ac->sromi->femrx_5gh[ant].elna_bypass_tr = raw_bypass;
			}
			pi_ac->sromi->femrx_5gh[ant].trloss = (2 * raw_trloss) + 8;
		}
	}
}

void wlc_phy_get_rxgain_acphy(phy_info_t *pi, rxgain_t rxgain[], int16 *tot_gain,
                              uint8 force_gain_type)
{
	uint8  core, bw_idx, ant, core_freq_segment_map;
	uint16 code_A, code_B, gain_tblid, stall_val;
	int8   gain_dvga, gain_bq0, gain_bq1, gain_lna1, gain_lna2, gain_mix, tr_loss;
	int8   elna_gain[PHY_CORE_MAX];
	bool   elna_present;
	int8   subband_idx;

	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	bzero(elna_gain, sizeof(elna_gain));

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		elna_present = BF_ELNA_2G(pi_ac);
		bw_idx = (CHSPEC_IS40(pi->radio_chanspec)) ? 1 : 0;
	} else {
		elna_present = BF_ELNA_5G(pi_ac);
		bw_idx = (CHSPEC_IS80(pi->radio_chanspec)) ? 2 :
		        (CHSPEC_IS40(pi->radio_chanspec)) ? 1 : 0;
	}

	/* 43602 China Spur WAR Gain Boosting for PAD */
	if (CHSPEC_IS5G(pi->radio_chanspec) && ACMAJORREV_5(pi->pubpi->phy_rev) &&
		(pi->sromi->dBpad)) {
		/* Boost Core 2 radio gain */
		MOD_RADIO_REGC(pi, TXMIX5G_CFG1, 2, gainboost, 0x9);
		MOD_RADIO_REGC(pi, PGA5G_CFG1, 2, gainboost, 0x7);
	}


	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {

		if (force_gain_type == 4) {
			code_A  =  READ_PHYREGC(pi, cliploGainCodeA, core);
			code_B   = READ_PHYREGC(pi, cliploGainCodeB, core);
			MOD_PHYREGCE(pi, RfctrlIntc, core, tr_sw_tx_pu, 1);
			MOD_PHYREGCE(pi, RfctrlIntc, core, tr_sw_rx_pu, 0);
			MOD_PHYREGCE(pi, RfctrlIntc, core, override_tr_sw, 1);
		} else if (force_gain_type == 3) {
			code_A  =  READ_PHYREGC(pi, clipmdGainCodeA, core);
			code_B   = READ_PHYREGC(pi, clipmdGainCodeB, core);
			if (pi_ac->mdgain_trtx_allowed) {
				MOD_PHYREGCE(pi, RfctrlIntc, core, tr_sw_tx_pu, 1);
				MOD_PHYREGCE(pi, RfctrlIntc, core, tr_sw_rx_pu, 0);
				MOD_PHYREGCE(pi, RfctrlIntc, core, override_tr_sw, 1);
			}
		} else if (force_gain_type == 2) {
			code_A  =  READ_PHYREGC(pi, clipHiGainCodeA, core);
			code_B   = READ_PHYREGC(pi, clipHiGainCodeB, core);
		} else if (force_gain_type == 1) {
			/* Change limited to 4350, Olympic program
			 * When we issue iqest with -i 1 option, INIT gain is applied.
			 * But because of Interference_code, the INIT gain can change
			 * So, for 4350, we have forced the fixed init gain by hardcoding it.
			 */
			if (ACMAJORREV_2(pi->pubpi->phy_rev) && ACMINORREV_1(pi)) {
				code_A	=  pi_ac->initGain_codeA;
				code_B	 = pi_ac->initGain_codeB;
			} else {
				code_A  =  READ_PHYREGC(pi, InitGainCodeA, core);
				code_B   = READ_PHYREGC(pi, InitGainCodeB, core);
			}
		} else if (force_gain_type == 7) {
			code_A	=  pi_ac->initGain_codeA;
			code_B	 = pi_ac->initGain_codeB;
		} else if (force_gain_type == 8) {
			code_A	=  pi_ac->initGain_codeA;
			code_B	 = pi_ac->initGain_codeB;
		} else if (force_gain_type == 9) {
			code_A  =  0x16a;
			code_B   = 0x554;
		} else if (force_gain_type == 6) {
			MOD_PHYREGCE(pi, RfctrlIntc, core, tr_sw_tx_pu, 0);
			MOD_PHYREGCE(pi, RfctrlIntc, core, tr_sw_rx_pu, 0);
			MOD_PHYREGCE(pi, RfctrlIntc, core, override_tr_sw, 0);
			continue;
		} else {
			return;
		}

		rxgain[core].lna1 = (code_A >> 1) & 0x7;
		rxgain[core].lna2 = (code_A >> 4) & 0x7;
		rxgain[core].mix  = (code_A >> 7) & 0xf;
		rxgain[core].lpf0 = (code_B >> 4) & 0x7;
		rxgain[core].lpf1 = (code_B >> 8) & 0x7;
		rxgain[core].dvga = (code_B >> 12) & 0xf;

		if (core == 0) {
			gain_tblid =  ACPHY_TBL_ID_GAIN0;
		} else if (core == 1) {
			gain_tblid =  ACPHY_TBL_ID_GAIN1;
		} else {
			gain_tblid =  ACPHY_TBL_ID_GAIN2;
		}

		stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
		ACPHY_DISABLE_STALL(pi);
		/* ELNA */
		if (elna_present == 1) {
			wlc_phy_table_read_acphy(pi, gain_tblid, 1, (0x0 + (code_A & 0x1)),
			                         8, &elna_gain[core]);
		}

		/* lna1, lna2 and mixer */
		wlc_phy_table_read_acphy(pi, gain_tblid, 1, (0x8 + rxgain[core].lna1), 8,
		                         &gain_lna1);
		gain_lna2 = pi_ac->lna2_complete_gaintbl[rxgain[core].lna2];
		wlc_phy_table_read_acphy(pi, gain_tblid, 1, (0x20 + rxgain[core].mix), 8,
		                         &gain_mix);
		ACPHY_ENABLE_STALL(pi, stall_val);

		gain_bq0 = 3 * rxgain[core].lpf0;
		gain_bq1 = 3 * rxgain[core].lpf1;
		gain_dvga = 3 * rxgain[core].dvga;

		if ((force_gain_type == 4) ||
			((force_gain_type == 3) && (pi_ac->mdgain_trtx_allowed))) {
			if (core == 0) {
				tr_loss = READ_PHYREGFLD(pi, Core0_TRLossValue, freqGainTLoss0);
			} else if (core == 1) {
				tr_loss = READ_PHYREGFLD(pi, Core1_TRLossValue, freqGainTLoss1);
			} else {
				tr_loss = READ_PHYREGFLD(pi, Core2_TRLossValue, freqGainTLoss2);
			}
		} else {
			tr_loss =  READ_PHYREG(pi, TRLossValue) & 0x7f;
		}

		/* Total gain: */
		tot_gain[core] = elna_gain[core] + gain_lna1 + gain_lna2 + gain_mix + gain_bq0
		        +  gain_bq1 - tr_loss + gain_dvga;

		/* core_freq_segment_map is only required for 80P80 mode.
		For other modes, it is ignored
		*/
		core_freq_segment_map = pi->u.pi_acphy->core_freq_mapping[core];

		/* adjust total gain based on common rssi correction factor: */
		ant = phy_get_rsdbbrd_corenum(pi, core);
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			tot_gain[core] -=
			        pi_ac->sromi->rssioffset.rssi_corr_normal[ant][bw_idx];
		} else {
			subband_idx = wlc_phy_get_chan_freq_range_acphy(pi,
				pi->radio_chanspec, core_freq_segment_map)-1;
			tot_gain[core] -=
			        pi_ac->sromi->rssioffset.rssi_corr_normal_5g[ant][subband_idx]
			        [bw_idx];
		}
		PHY_RXIQ(("In %s: | Mode = %d | Code_A = %X | Code_B = %X |"
			  "\n", __FUNCTION__, force_gain_type, code_A, code_B));
	}
}

#ifndef WLC_DISABLE_ACI

void
wlc_phy_aci_updsts_acphy(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	acphy_aci_params_t *aci;
	uint32 phy_mode = 0;

	if (pi_ac->aci != NULL) {
		aci = pi_ac->aci;
		if (aci->desense.on || aci->hwaci_desense_state > 0)
			phy_mode = PHY_MODE_ACI;
	}

	wlapi_high_update_phy_mode(pi->sh->physhim, phy_mode);
}

/**********  DESENSE : ACI, NOISE, BT (end)  ********** */
#endif /* !WLC_DISABLE_ACI */

#if defined(PHYCAL_CACHING)
int8 wlc_phy_get_thresh_acphy(phy_info_t *pi)
{
#ifdef WLOLPC
#if defined(BCMDBG) || defined(WLTEST)
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
#endif /* BCMDBG || WLTEST */
	acphy_calcache_t *cache;
	ch_calcache_t *ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
	if (ctx) {
		if (ctx->valid) {
			cache = &ctx->u.acphy_cache;
			if (cache->olpc_caldone)
				return wlc_phy_olpcthresh();
		}
	}
#if defined(BCMDBG) || defined(WLTEST)
	else {
		if (pi_ac->olpc_dbg_mode)
			return wlc_phy_olpcthresh();
	}
#endif /* BCMDBG || WLTEST */
#endif /* WLOLPC */
	return wlc_phy_tssivisible_thresh_acphy(pi);
}

#ifdef WLOLPC
static int8
wlc_phy_olpcthresh()
{
	/* Threshold = 1dBm (in quarter dB units) above which olpc will kick in */
	int8 olpc_thresh = 1*WLC_TXPWR_DB_FACTOR;
	return olpc_thresh;
}

void
wlc_phy_update_olpc_cal(wlc_phy_t *ppi, bool set, bool dbg)
{
	phy_info_t *pi = (phy_info_t *)ppi;
#if defined(BCMDBG) || defined(WLTEST)
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
#endif /* BCMDBG || WLTEST */

	ch_calcache_t *ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
	acphy_calcache_t *cache;
	if (ctx) {
		cache = &ctx->u.acphy_cache;
		cache->olpc_caldone = set;
	}
#if defined(BCMDBG) || defined(WLTEST)
	else {
		if (dbg) {
			pi_ac->olpc_dbg_mode = dbg;
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			/* Toggle Power Control to save off base index */
			wlc_phy_txpwrctrl_enable_acphy(pi, 0);
			wlc_phy_txpwrctrl_enable_acphy(pi, 1);
			wlapi_enable_mac(pi->sh->physhim);
		}
	}
#endif /* BCMDBG || WLTEST */
}

void
wlc_phy_pi_update_olpc_cal(phy_info_t *pi, bool set, bool dbg)
{
	wlc_phy_update_olpc_cal((wlc_phy_t *)pi, set, dbg);
}
#endif /* WLOLPC */

void
wlc_phy_cal_cache_acphy(wlc_phy_t *pih)
{
	phy_info_t *pi = (phy_info_t *) pih;
	ch_calcache_t *ctx = NULL;
	acphy_calcache_t *cache;

	uint8 core;
	uint16 tbl_cookie;

	uint16 *epstbl_offset_cache;
	uint32 *epsilon_cache;
	uint32 epsilon_table_ids[] =
		{ACPHY_TBL_ID_EPSILON0, ACPHY_TBL_ID_EPSILON1, ACPHY_TBL_ID_EPSILON2};
	uint32 rfpwrlut_table_ids[] =
		{ACPHY_TBL_ID_RFPWRLUTS0, ACPHY_TBL_ID_RFPWRLUTS1, ACPHY_TBL_ID_RFPWRLUTS2};

	ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);

#ifndef WLOLPC
	/* A context must have been created before reaching here */
	ASSERT(ctx != NULL);
#endif /* WLOLPC */

	if (ctx == NULL) {
		PHY_ERROR(("wl%d: %s call with null ctx\n",
			pi->sh->unit, __FUNCTION__));
		return;
	}

	/* Ensure that the Callibration Results are valid */
	wlc_phy_table_read_acphy(pi, wlc_phy_get_tbl_id_iqlocal(pi, 0), 1,
		IQTBL_CACHE_COOKIE_OFFSET, 16, &tbl_cookie);

	if (tbl_cookie != TXCAL_CACHE_VALID) {
		PHY_ERROR(("wl%d: %s tbl_cookie != TXCAL_CACHE_VALID\n",
			pi->sh->unit, __FUNCTION__));
		return;
	}

	ctx->valid = TRUE;

	cache = &ctx->u.acphy_cache;

	epsilon_cache = cache->papd_eps;
	epstbl_offset_cache = cache->eps_offset_cache;

	/* save the callibration to cache */
	FOREACH_CORE(pi, core) {
		uint16 ab_int[2];
		/* Save OFDM Tx IQ Imb Coeffs A,B and Digital Loft Comp Coeffs */
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ,
		                                ab_int, TB_OFDM_COEFFS_AB, core);
		cache->ofdm_txa[core] = ab_int[0];
		cache->ofdm_txb[core] = ab_int[1];
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ,
		                                &cache->ofdm_txd[core], TB_OFDM_COEFFS_D, core);
		/* Save OFDM Tx IQ Imb Coeffs A,B and Digital Loft Comp Coeffs */
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ,
		                                ab_int, TB_BPHY_COEFFS_AB, core);
		cache->bphy_txa[core] = ab_int[0];
		cache->bphy_txb[core] = ab_int[1];
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ,
		                                &cache->bphy_txd[core], TB_BPHY_COEFFS_D, core);

		if (!TINY_RADIO(pi)) {
			/* Save Analog Tx Loft Comp Coeffs */
			cache->txei[core] = (uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_FINE_I, core);
			cache->txeq[core] = (uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_FINE_Q, core);
			cache->txfi[core] = (uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_COARSE_I,
			                                           core);
			cache->txfq[core] = (uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_COARSE_Q,
			                                           core);
		}

		/* Save Rx IQ Imb Coeffs */
		cache->rxa[core] = READ_PHYREGCE(pi, Core1RxIQCompA, core);
		cache->rxb[core] = READ_PHYREGCE(pi, Core1RxIQCompB, core);
		cache->rxs[core] = pi->u.pi_acphy->fdiqi->slope[core];
		cache->rxe =  pi->u.pi_acphy->fdiqi->enabled;
		/* Save base index */
		cache->baseidx[core] = READ_PHYREGFLDCE(pi, TxPwrCtrlStatus_path, core, baseIndex);
		/* save idle TSSI */
		cache->idle_tssi[core] = READ_PHYREGCE(pi, TxPwrCtrlIdleTssi_path, core);

		/* save PAPD epsilon offsets */
		if (PHY_PAPDEN(pi)) {
			wlc_phy_table_read_acphy_dac_war(pi, epsilon_table_ids[core],
				ACPHY_PAPD_EPS_TBL_SIZE, 0, 32, epsilon_cache, core);
			epsilon_cache += ACPHY_PAPD_EPS_TBL_SIZE;
			wlc_phy_table_read_acphy(pi, rfpwrlut_table_ids[core],
				ACPHY_PAPD_RFPWRLUT_TBL_SIZE, 0, 16, epstbl_offset_cache);
			epstbl_offset_cache += ACPHY_PAPD_RFPWRLUT_TBL_SIZE;
		}
	}

#ifdef BCMDBG
	PHY_CAL(("wl%d: %s: Cached cal values for chanspec 0x%x are:\n",
		pi->sh->unit, __FUNCTION__,  ctx->chanspec));
	wlc_phy_cal_cache_dbg_acphy(pih, ctx);
#endif
}

#ifdef BCMDBG
static void
wlc_phy_cal_cache_dbg_acphy(wlc_phy_t *pih, ch_calcache_t *ctx)
{
	phy_info_t *pi = (phy_info_t *) pih;

	if (ISACPHY(pi)) {
		uint8 i, j;
		acphy_calcache_t *cache = &ctx->u.acphy_cache;

		FOREACH_CORE(pi, i) {
			PHY_CAL(("CORE %d:\n", i));
			PHY_CAL(("\tofdm_txa:0x%x  ofdm_txb:0x%x  ofdm_txd:0x%x\n",
				cache->ofdm_txa[i], cache->ofdm_txb[i], cache->ofdm_txd[i]));
			PHY_CAL(("\tbphy_txa:0x%x  bphy_txb:0x%x  bphy_txd:0x%x\n",
				cache->bphy_txa[i], cache->bphy_txb[i], cache->bphy_txd[i]));
			PHY_CAL(("\ttxei:0x%x  txeq:0x%x\n", cache->txei[i], cache->txeq[i]));
			PHY_CAL(("\ttxfi:0x%x  txfq:0x%x\n", cache->txfi[i], cache->txfq[i]));
			PHY_CAL(("\trxa:0x%x  rxb:0x%x\n", cache->rxa[i], cache->rxb[i]));
			PHY_CAL(("\trxs:0x%x  rxe:0x%x\n", cache->rxs[i], cache->rxe));
			PHY_CAL(("\tidletssi:0x%x\n", cache->idle_tssi[i]));
			PHY_CAL(("\tbasedindex:0x%x\n", cache->baseidx[i]));

			if (PHY_PAPDEN(pi)) {
				PHY_CAL(("\tPAPD eps table\n"));
				for (j = 0; j < ACPHY_PAPD_EPS_TBL_SIZE; j += 16) {
					PHY_CAL(("\t%d : 0x%x\n",
						j, cache->papd_eps[(i*
						ACPHY_PAPD_EPS_TBL_SIZE)+j]));
				}
				PHY_CAL(("\tPAPD rfpwrlut\n"));
				for (j = 0; j < ACPHY_PAPD_RFPWRLUT_TBL_SIZE; j += 32) {
					PHY_CAL(("\t%d : 0x%x\n",
						j,
						cache->eps_offset_cache[(i*
						ACPHY_PAPD_RFPWRLUT_TBL_SIZE)+j]));
				}
			}
		}
	}
}

void
wlc_phydump_cal_cache_acphy(phy_info_t *pi, ch_calcache_t *ctx, struct bcmstrbuf *b)
{
	if (ISACPHY(pi)) {
		uint8 i;
		acphy_calcache_t *cache = &ctx->u.acphy_cache;

		FOREACH_CORE(pi, i) {
			bcm_bprintf(b, "CORE %d:\n", i);
			bcm_bprintf(b, "\tofdm_txa:0x%x  ofdm_txb:0x%x  ofdm_txd:0x%x\n",
				cache->ofdm_txa[i], cache->ofdm_txb[i], cache->ofdm_txd[i]);
			bcm_bprintf(b, "\tbphy_txa:0x%x  bphy_txb:0x%x  bphy_txd:0x%x\n",
				cache->bphy_txa[i], cache->bphy_txb[i], cache->bphy_txd[i]);
			bcm_bprintf(b, "\ttxei:0x%x  txeq:0x%x\n", cache->txei[i], cache->txeq[i]);
			bcm_bprintf(b, "\ttxfi:0x%x  txfq:0x%x\n", cache->txfi[i], cache->txfq[i]);
			bcm_bprintf(b, "\trxa:0x%x  rxb:0x%x\n", cache->rxa[i], cache->rxb[i]);
			bcm_bprintf(b, "\tidletssi:0x%x\n", cache->idle_tssi[i]);
			bcm_bprintf(b, "\tbasedindex:0x%x\n", cache->baseidx[i]);
		}
	}
}
#endif /* BCMDBG */

int
wlc_phy_cal_cache_restore_acphy(phy_info_t *pi)
{
	ch_calcache_t *ctx;
	acphy_calcache_t *cache = NULL;
	bool suspend;
	uint8 core;
	phy_iq_comp_t coeffs[PHY_CORE_MAX];

	uint16 *epstbl_offset_cache;
	uint32 *epsilon_cache;
	uint32 epsilon_table_ids[] =
		{ACPHY_TBL_ID_EPSILON0, ACPHY_TBL_ID_EPSILON1, ACPHY_TBL_ID_EPSILON2};
	uint32 rfpwrlut_table_ids[] =
		{ACPHY_TBL_ID_RFPWRLUTS0, ACPHY_TBL_ID_RFPWRLUTS1, ACPHY_TBL_ID_RFPWRLUTS2};

	uint16 tbl_cookie = TXCAL_CACHE_VALID;

	ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);

	if (!ctx) {
		PHY_CAL(("wl%d: %s: Chanspec 0x%x not found in calibration cache\n",
		           pi->sh->unit, __FUNCTION__, pi->radio_chanspec));
		return BCME_ERROR;
	}

	if (!ctx->valid) {
		PHY_CAL(("wl%d: %s: Chanspec 0x%x found, but not valid in phycal cache\n",
		           pi->sh->unit, __FUNCTION__, ctx->chanspec));
		return BCME_ERROR;
	}

	PHY_CAL(("wl%d: %s: Restoring all cal coeffs from calibration cache for chanspec 0x%x\n",
	           pi->sh->unit, __FUNCTION__, pi->radio_chanspec));

	cache = &ctx->u.acphy_cache;

	epsilon_cache = cache->papd_eps;
	epstbl_offset_cache = cache->eps_offset_cache;

	suspend = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
	if (!suspend) {
		/* suspend mac */
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
	}
	phy_utils_phyreg_enter(pi);

	/* restore the txcal from cache */
	FOREACH_CORE(pi, core) {
		uint16 ab_int[2];
		/* Restore OFDM Tx IQ Imb Coeffs A,B and Digital Loft Comp Coeffs */
		ab_int[0] = cache->ofdm_txa[core];
		ab_int[1] = cache->ofdm_txb[core];
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_WRITE,
		                                ab_int, TB_OFDM_COEFFS_AB, core);
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_WRITE,
		                                &cache->ofdm_txd[core], TB_OFDM_COEFFS_D, core);
		/* Restore BPHY Tx IQ Imb Coeffs A,B and Digital Loft Comp Coeffs */
		ab_int[0] = cache->bphy_txa[core];
		ab_int[1] = cache->bphy_txb[core];
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_WRITE,
		                                ab_int, TB_BPHY_COEFFS_AB, core);
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_WRITE,
		                                &cache->bphy_txd[core], TB_BPHY_COEFFS_D, core);

		if (!TINY_RADIO(pi)) {
			/* Restore Analog Tx Loft Comp Coeffs */
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_FINE_I(core),
			                         cache->txei[core]);
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_FINE_Q(core),
			                         cache->txeq[core]);
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_COARSE_I(core),
			                         cache->txfi[core]);
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_COARSE_Q(core),
			                         cache->txfq[core]);
		}

		/* Restore Rx IQ Imb Coeffs */
		coeffs[core].a = cache->rxa[core] & 0x3ff;
		coeffs[core].b = cache->rxb[core] & 0x3ff;
		wlc_phy_rx_iq_comp_acphy(pi, 1, &(coeffs[core]), core);

		if (cache->rxe) {
		  pi->u.pi_acphy->fdiqi->slope[core] = cache->rxs[core];
		}
		/* Restore base index */
		MOD_PHYREGCEE(pi, TxPwrCtrlInit_path, core,
		              pwrIndex_init_path, cache->baseidx[core]);

		/* Restore Idle TSSI & Vmid values */
		wlc_phy_txpwrctrl_set_idle_tssi_acphy(pi, cache->idle_tssi[core], core);

		/* Restore PAPD epsilon offsets */
		if (PHY_PAPDEN(pi)) {
			wlc_phy_table_write_acphy_dac_war(pi, epsilon_table_ids[core],
				ACPHY_PAPD_EPS_TBL_SIZE, 0, 32, epsilon_cache, core);
			epsilon_cache += ACPHY_PAPD_EPS_TBL_SIZE;
			wlc_phy_table_write_acphy(pi, rfpwrlut_table_ids[core],
				ACPHY_PAPD_RFPWRLUT_TBL_SIZE, 0, 16, epstbl_offset_cache);
			epstbl_offset_cache += ACPHY_PAPD_RFPWRLUT_TBL_SIZE;
		}
	}

	if ((!TINY_RADIO(pi) || ACMAJORREV_4(pi->pubpi->phy_rev)) && cache->rxe) {
	        wlc_phy_rx_fdiqi_comp_acphy(pi, TRUE);
	}

	/* Validate the Calibration Results */
	wlc_phy_table_write_acphy(pi, wlc_phy_get_tbl_id_iqlocal(pi, 0), 1,
	                          IQTBL_CACHE_COOKIE_OFFSET, 16, &tbl_cookie);

	phy_utils_phyreg_exit(pi);

	/* unsuspend mac */
	if (!suspend) {
		wlapi_enable_mac(pi->sh->physhim);
	}

#ifdef BCMDBG
	PHY_CAL(("wl%d: %s: Restored values for chanspec 0x%x are:\n", pi->sh->unit,
	           __FUNCTION__, ctx->chanspec));
	wlc_phy_cal_cache_dbg_acphy((wlc_phy_t *)pi, ctx);
#endif
	return BCME_OK;
}
#endif /* PHYCAL_CACHING */

#ifdef WL_SAR_SIMPLE_CONTROL
static void
BCMATTACHFN(wlc_phy_nvram_dynamicsarctrl_read)(phy_info_t *pi)
{
/* Nvram parameter to get sarlimits customized by user
 * Value interpetation:
 *  dynamicsarctrl_2g = 0x[core3][core2][core1][core0]
 * each core# has the bitmask followings:
 * 8th bit : 0 - sarlimit enable / 1 - sarlimit disable
 * 0 ~ 7 bits : qdbm power val (0x7f as a maxumum)
 */
	char phy_var_name[20];

	(void)snprintf(phy_var_name, sizeof(phy_var_name), "dynamicsarctrl_2g");
	if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
		pi->dynamic_sarctrl_2g = (uint32)PHY_GETINTVAR(pi, phy_var_name);
	}

	(void)snprintf(phy_var_name, sizeof(phy_var_name), "dynamicsarctrl_5g");
	if ((PHY_GETVAR(pi, phy_var_name)) != NULL) {
		pi->dynamic_sarctrl_5g = (uint32)PHY_GETINTVAR(pi, phy_var_name);
	}
}
#endif /* WL_SAR_SIMPLE_CONTROL */

static void
BCMATTACHFN(wlc_phy_nvram_femctrl_read)(phy_info_t *pi)
{
	uint8 i;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (ACPHY_FEMCTRL_ACTIVE(pi)) {
		return;
	}

	if (PHY_GETVAR(pi, rstr_swctrlmap_2g)) {
		for (i = 0; i < ACPHY_SWCTRL_NVRAM_PARAMS; i++) {
			pi_ac->sromi->nvram_femctrl.swctrlmap_2g[i] =
				(uint32) PHY_GETINTVAR_ARRAY(pi, rstr_swctrlmap_2g, i);
		}
	} else {
		PHY_ERROR(("%s: Switch control map(%s) is NOT found\n",
		           __FUNCTION__, rstr_swctrlmap_2g));
	}

	if (PHY_GETVAR(pi, rstr_swctrlmapext_2g)) {
			for (i = 0; i < ACPHY_SWCTRL_NVRAM_PARAMS; i++) {
				pi_ac->sromi->nvram_femctrl.swctrlmapext_2g[i] =
					(uint32) PHY_GETINTVAR_ARRAY(pi, rstr_swctrlmapext_2g, i);
			}
	}

	if (PHY_GETVAR(pi, rstr_swctrlmap_5g)) {
			for (i = 0; i < ACPHY_SWCTRL_NVRAM_PARAMS; i++) {
				pi_ac->sromi->nvram_femctrl.swctrlmap_5g[i] =
					(uint32) PHY_GETINTVAR_ARRAY(pi, rstr_swctrlmap_5g, i);
			}
	} else {
		PHY_ERROR(("%s: Switch control map(%s) is NOT found\n",
		           __FUNCTION__, rstr_swctrlmap_5g));
	}

	if (PHY_GETVAR(pi, rstr_swctrlmapext_5g)) {
			for (i = 0; i < ACPHY_SWCTRL_NVRAM_PARAMS; i++) {
				pi_ac->sromi->nvram_femctrl.swctrlmapext_5g[i] =
					(uint32) PHY_GETINTVAR_ARRAY(pi, "swctrlmapext_5g", i);
			}
	}

	pi_ac->sromi->nvram_femctrl.txswctrlmap_2g =
		(uint32) PHY_GETINTVAR_DEFAULT(pi, rstr_txswctrlmap_2g, PAMODE_HI_LIN);

	pi_ac->sromi->nvram_femctrl.txswctrlmap_2g_mask =
		(uint16) PHY_GETINTVAR_DEFAULT(pi, rstr_txswctrlmap_2g_mask, 0x3fff);

	pi_ac->sromi->nvram_femctrl.txswctrlmap_5g =
		(uint32) PHY_GETINTVAR_DEFAULT(pi, rstr_txswctrlmap_5g, PAMODE_HI_LIN);
}

void
wlc_phy_aci_w2nb_setup_acphy(phy_info_t *pi, bool on)
{
	uint8 core, on_off;

	on_off = on ? 1 : 0;
	FOREACH_CORE(pi, core) {
		MOD_PHYREGCE(pi, RfctrlCoreRxPus, core, rxrf_lna2_wrssi2_pwrup, on_off);
		MOD_PHYREGCE(pi, RfctrlOverrideRxPus, core, rxrf_lna2_wrssi2_pwrup, 0x1);

#ifndef WLC_DISABLE_ACI
		if (TINY_RADIO(pi))
			PHY_INFORM(("%s: HWACI not yet implemented for Tiny Radio chips\n",
				__FUNCTION__));
		else if (on) {
			MOD_RADIO_REGC(pi, LNA5G_RSSI, core,
			               dig_wrssi2_threshold, pi->u.pi_acphy->hwaci_args->w2);
		}
#endif /* !WLC_DISABLE_ACI */
	}
}

void
wlc_tiny_setup_coarse_dcc(phy_info_t *pi)
{
	uint8 phybw;
	uint8 core;

	/*
	 * Settings required to use the RFSeq to trigger the coarse DCC
	 * 4345TC Not used. 20691_coarse_dcc used
	 * 4345A0 offset comparator has hysteresis and dc offset but is adequate for 5G
	 */

	if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
		wlc_tiny_dc_static_WAR(pi);
	}

	/* DCC FSM Defaults */
	MOD_PHYREG(pi, BBConfig, dcc_wakeup_restart_en, 0);
	MOD_PHYREG(pi, BBConfig, dcc_wakeup_restart_delay, 10);

	/* Control via pktproc, instead of RFSEQ */
	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl,  1);

	FOREACH_CORE(pi, core) {

		/* Disable overrides that may have been set during 2G cal */
		MOD_RADIO_REG_TINY(pi, RX_BB_2G_OVR_EAST, core, ovr_tia_offset_dac_pwrup, 0);
		MOD_RADIO_REG_TINY(pi, RX_BB_2G_OVR_EAST, core, ovr_tia_offset_dac, 0);
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			MOD_RADIO_REG_20693(pi, RX_BB_2G_OVR_EAST, core,
				ovr_tia_offset_comp_pwrup, 0);
		} else {
			MOD_RADIO_REG_TINY(pi, RX_BB_2G_OVR_NORTH, core,
				ovr_tia_offset_comp_pwrup, 0);
		}
		MOD_RADIO_REG_TINY(pi, RX_BB_2G_OVR_EAST, core, ovr_tia_offset_dac, 0);
		MOD_RADIO_REG_TINY(pi, TIA_CFG8, core, tia_offset_comp_drive_strength, 1);

		/* Set idac LSB to (50nA * 4) ~ 0.2uA for 2G, (50nA * 12) ~ 0.6 uA for 5G */
		/* changed biasadj to 1 as coupled d.c. in loop is very less. */
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			MOD_RADIO_REG_TINY(pi, TIA_CFG8, core, tia_offset_dac_biasadj,
			(CHSPEC_IS2G(pi->radio_chanspec)) ? 1 : 1);
		} else {
			MOD_RADIO_REG_TINY(pi, TIA_CFG8, core, tia_offset_dac_biasadj,
			(CHSPEC_IS2G(pi->radio_chanspec)) ? 4 : 12);
		}
	}
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dac_sign, 1);
		MOD_PHYREG(pi, rx_tia_dc_loop_0, en_lock, 1);
		if (ACMINORREV_2(pi)) {
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dac_sign, 1);
			MOD_PHYREG(pi, rx_tia_dc_loop_1, en_lock, 1);
			MOD_PHYREG(pi, rx_tia_dc_loop_1, restart_gear, 6);
		}
	}
	/* Set minimum idle gain incase of restart */
	MOD_PHYREG(pi, rx_tia_dc_loop_0, restart_gear, 6);

	if (IS20MHZ(pi))
		phybw = 0;
	else if (IS40MHZ(pi))
		phybw = 1;
	else
		phybw = 2;

	/*
	 * Because FSM clock is PHY_BW dependant scale gear gain and loop count.
	 *
	 * Settings below assume:
	 *	9 DCC FSM clock cycle latency and single pole  RC filter >=2MHz ala 4345B0.
	 * (Valid also for 4345A0).
	 */
	MOD_PHYREG(pi, rx_tia_dc_loop_gain_0, loop_gain_0, 15); /* disable */
	MOD_PHYREG(pi, rx_tia_dc_loop_gain_1, loop_gain_1, 2 + phybw);
	MOD_PHYREG(pi, rx_tia_dc_loop_gain_2, loop_gain_2, 4 + phybw);
	MOD_PHYREG(pi, rx_tia_dc_loop_gain_3, loop_gain_3, 5 + phybw);
	MOD_PHYREG(pi, rx_tia_dc_loop_gain_4, loop_gain_4, 6 + phybw);
	MOD_PHYREG(pi, rx_tia_dc_loop_gain_5, loop_gain_5, 8 + phybw);

	MOD_PHYREG(pi, rx_tia_dc_loop_count_0, loop_count_0, 0); /* disable */
	MOD_PHYREG(pi, rx_tia_dc_loop_count_1, loop_count_1, (phybw > 1) ? 255 : (80 << phybw));
	MOD_PHYREG(pi, rx_tia_dc_loop_count_2, loop_count_2, (phybw > 1) ? 255 : (80 << phybw));
	MOD_PHYREG(pi, rx_tia_dc_loop_count_3, loop_count_3, (phybw > 1) ? 255 : (80 << phybw));
	MOD_PHYREG(pi, rx_tia_dc_loop_count_4, loop_count_4, (phybw > 1) ? 255 : (80 << phybw));
	MOD_PHYREG(pi, rx_tia_dc_loop_count_5, loop_count_5, (20 << phybw));

	if (ACMAJORREV_3(pi->pubpi->phy_rev))
		wlc_phy_enable_lna_dcc_comp_20691(pi, PHY_ILNA(pi));
}


/* NB returns SUM not AVERAGE of hpf dc est */
/* Based on: proc acphy_get_hpf_dc_est { {averages 10} } */

static void
wlc_phy_get_rx_hpf_dc_est_acphy(phy_info_t *pi, phy_hpf_dc_est_t *hpf_iqdc, uint16 nump_samps)
{
	uint8 core;
	uint16 i;

	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		hpf_iqdc[core].i_accum = 0;
		hpf_iqdc[core].q_accum = 0;
	}
	for (i = 0; i < nump_samps; i++) {
		FOREACH_CORE(pi, core) {
			hpf_iqdc[core].i_accum += (int16)READ_PHYREGCE(pi, DCestimateI, core);
			hpf_iqdc[core].q_accum += (int16)READ_PHYREGCE(pi, DCestimateQ, core);
		}
	}
}

/*
 * Based on:
 *  proc acphy_digi_dccomp_minsearch { {DDCC_MIN -32} {DDCC_MAX 31} {DDCC_STEP 1} {use_hpf_est 1} }
 *  proc acphy_tiny_static_dc_offset_cal { }
 */
static int
wlc_phy_tiny_static_dc_offset_cal(phy_info_t *pi)
{
	/* Use HPF DC estimate to find optimum static DC offset setting */

	int16 inext[PHY_CORE_MAX];
	int16 qnext[PHY_CORE_MAX];
	uint16 sparereg = 0;

	phy_hpf_dc_est_t hpf_iqdc[PHY_CORE_MAX];

	uint8 core;
	uint8 count;
	uint8 swap_saved[PHY_CORE_MAX] = {0};
	uint8 ovr[PHY_CORE_MAX] = {0};
	uint8 pu[PHY_CORE_MAX] = {0};
	uint8 idacbias[PHY_CORE_MAX] = {0};
	uint8 force_rshift;
	uint8 rx_farrow_rshift;

	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);

	FOREACH_CORE(pi, core) {
		/* set LNA off */
		/* save config */
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			ovr[core] = READ_RADIO_REGFLD_TINY(pi, RX_TOP_5G_OVR, core,
				ovr_lna5g_lna1_pu);
			pu[core] = READ_RADIO_REGFLD_TINY(pi, LNA5G_CFG1, core, lna5g_lna1_pu);

			if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
				MOD_RADIO_REG_TINY(pi, RX_TOP_5G_OVR, core,
					ovr_lna5g_lna1_out_short_pu, 1);
				MOD_RADIO_REG_TINY(pi, LNA5G_CFG1, core,
					lna5g_lna1_out_short_pu, 1);
			}
			MOD_RADIO_REG_TINY(pi, RX_TOP_5G_OVR, core, ovr_lna5g_lna1_pu, 1);
			MOD_RADIO_REG_TINY(pi, LNA5G_CFG1, core, lna5g_lna1_pu, 0);
		} else {
			pu[core] = READ_RADIO_REGFLD_TINY(pi, LNA2G_CFG1, core, lna2g_lna1_pu);
			if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				ovr[core] = READ_RADIO_REGFLD_20693(pi, RX_TOP_2G_OVR_EAST2, core,
					ovr_lna2g_lna1_pu);
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core,
					ovr_lna2g_lna1_pu, 1);
			} else {
				ovr[core] = READ_RADIO_REGFLD_TINY(pi, RX_TOP_2G_OVR_NORTH, core,
					ovr_lna2g_lna1_pu);

				MOD_RADIO_REG_TINY(pi, RX_TOP_2G_OVR_NORTH, core,
					ovr_lna2g_lna1_out_short_pu, 1);
				MOD_RADIO_REG_TINY(pi, LNA2G_CFG1, core,
					lna2g_lna1_out_short_pu, 1);
				MOD_RADIO_REG_TINY(pi, RX_TOP_2G_OVR_NORTH, core,
					ovr_lna2g_lna1_pu, 1);
			}
			MOD_RADIO_REG_TINY(pi, LNA2G_CFG1, core, lna2g_lna1_pu, 0);
		}

		idacbias[core] = READ_RADIO_REGFLD_TINY(pi, TIA_CFG8, core,
			tia_offset_dac_biasadj);

		/* Minimise idac step size so that it does not contribute to DC offsets */
		MOD_RADIO_REG_TINY(pi, TIA_CFG8, core, tia_offset_dac_biasadj, 1);
	}

	/* Saving Farrow settings */
	force_rshift = READ_PHYREGFLD(pi, RxSdFeConfig1, farrow_rshift_force);
	rx_farrow_rshift = READ_PHYREGFLD(pi, RxSdFeConfig6, rx_farrow_rshift_0);

	/* Changing Farrow settings */
	MOD_PHYREG(pi, RxSdFeConfig1, farrow_rshift_force, 1);
	MOD_PHYREG(pi, RxSdFeConfig6, rx_farrow_rshift_0, 0);

	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 0);

	OSL_DELAY(1);	/* allow radio to settle */

	wlc_dcc_fsm_reset(pi);	/* Redo Coarse cal */

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_hold, 1);
		if (ACMINORREV_2(pi))
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_hold, 1);
	} else {
		sparereg = READ_PHYREG(pi, SpareReg);
		WRITE_PHYREG(pi, SpareReg,  sparereg | 0x4000);	/* assert dcc fsm hold */
	}

	FOREACH_CORE(pi, core) {
		swap_saved[core] = READ_PHYREGFLDCXE(pi, RxFeCtrl1, swap_iq, core);
		MOD_PHYREGCXE(pi, RxFeCtrl1, swap_iq, core, 0);

		inext[core] = 0;
		qnext[core] = 0;
	}

	for (count = 0; count < 3; count++) {

		FOREACH_CORE(pi, core) {
			uint8 core_i;
			if (ACMAJORREV_4(pi->pubpi->phy_rev) &&
				(phy_get_phymode(pi) != PHYMODE_RSDB)) {
			/* In MIMO mode, RxSdFeConfig7 is a common register instead of being a
			 * path register in 4349A0. There are however two copies due to the
			 * RSDB support, which can be exploited to write to the shadow copy (RSDB)
			 * of each of the cores. This is done by writing to the MAC
			 * PHYMODE register => Turning on a bit in PHYMODE results the
			 * register write writing only to the core0 shadow copy. The solution
			 * then is to write to core1 first (which writes both core1 and core0
			 * copies), and then write only to core0 using the hack.
			 * Hence the following code: The register RxSdFeConfig7 is written by the
			 * function wlc_rx_digi_dccomp_set.
			 * So first writing to core1, then to core0
			 * -> function wlc_rx_digi_dccomp_set then enables the HACK when
			 *  writing to core 0
			 */
				core_i = !core;
			} else {
				core_i = core;
			}
			wlc_rx_digi_dccomp_set(pi, inext[core_i], qnext[core_i], core_i);
		}

		wlc_phy_get_rx_hpf_dc_est_acphy(pi, hpf_iqdc, 117);

		FOREACH_CORE(pi, core) {
			hpf_iqdc[core].i_accum += (1 << 13);
			hpf_iqdc[core].q_accum += (1 << 13);
			inext[core] = inext[core] - (int16)(hpf_iqdc[core].i_accum >> 14);
			qnext[core] = qnext[core] - (int16)(hpf_iqdc[core].q_accum >> 14);
		}
	}

	FOREACH_CORE(pi, core) {
		uint8 core_i;
		if (ACMAJORREV_4(pi->pubpi->phy_rev) &&
			(phy_get_phymode(pi) != PHYMODE_RSDB)) {
			/* see comment above */
			core_i = !core;
		} else {
			core_i = core;
		}

		/*
		 * Limit adjustment to maximum expected. This includes net offset due to:
		 * TIA, comparator and ADC offsets;
		if (inext[core_i] < -15)  inext[core_i] = -15;
		if (qnext[core_i] < -15)  qnext[core_i] = -15;
		if (inext[core_i] > 15)  inext[core_i] = 15;
		if (qnext[core_i] > 15)  qnext[core_i] = 15;
		*/

		wlc_rx_digi_dccomp_set(pi, inext[core_i], qnext[core_i], core_i);
		MOD_PHYREGCXE(pi, RxFeCtrl1, swap_iq, core_i, swap_saved[core_i]);
#if defined(BCMDBG_RXCAL)
		printf("core[%d]: static dc offset cal (%i, %i)\n",
			core_i, inext[core_i], qnext[core_i]);
#endif
	}

	/* Restore settings */
	FOREACH_CORE(pi, core) {
		MOD_RADIO_REG_TINY(pi, TIA_CFG8, core, tia_offset_dac_biasadj, idacbias[core]);
		if (CHSPEC_IS5G(pi->radio_chanspec)) {
			MOD_RADIO_REG_TINY(pi, RX_TOP_5G_OVR, core, ovr_lna5g_lna1_pu, ovr[core]);
			if (!ACMAJORREV_4(pi->pubpi->phy_rev)) {
				MOD_RADIO_REG_TINY(pi, LNA5G_CFG1, core,
					lna5g_lna1_out_short_pu, 0);
				MOD_RADIO_REG_TINY(pi, RX_TOP_5G_OVR, core,
					ovr_lna5g_lna1_out_short_pu, 0);
			}
			MOD_RADIO_REG_TINY(pi, LNA5G_CFG1, core, lna5g_lna1_pu, pu[core]);
		} else {
			if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
				MOD_RADIO_REG_20693(pi, RX_TOP_2G_OVR_EAST2, core,
					ovr_lna2g_lna1_pu, ovr[core]);
			} else {
				MOD_RADIO_REG_TINY(pi, RX_TOP_2G_OVR_NORTH, core,
					ovr_lna2g_lna1_pu, ovr[core]);
				MOD_RADIO_REG_TINY(pi, RX_TOP_2G_OVR_NORTH, core,
					ovr_lna2g_lna1_out_short_pu, 0);
				MOD_RADIO_REG_TINY(pi, LNA2G_CFG1, core,
					lna2g_lna1_out_short_pu, 0);
			}
			MOD_RADIO_REG_TINY(pi, LNA2G_CFG1, core, lna2g_lna1_pu, pu[core]);
		}
	}

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_hold, 0);
		if (ACMINORREV_2(pi)) {
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_hold, 0);
		}
	} else {
		WRITE_PHYREG(pi, SpareReg, sparereg & 0xbfff);	/* Release dcc fsm hold */
	}
	MOD_PHYREG(pi, RfseqTrigger, en_pkt_proc_dcc_ctrl, 1);

	/* Restoring Farrow settings */
	MOD_PHYREG(pi, RxSdFeConfig1, farrow_rshift_force, force_rshift);
	MOD_PHYREG(pi, RxSdFeConfig6, rx_farrow_rshift_0, rx_farrow_rshift);

	wlc_dcc_fsm_reset(pi);	/* Redo Coarse cal */
	wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);

	return BCME_OK;
}

/*
 * Based on: proc tiny_rx_digi_dccomp_set {{value 0} { channel "i" }}
 */
static void
wlc_rx_digi_dccomp_set(phy_info_t *pi, int16 i, int16 q, uint8 core)
{
	if (i < -32)  i = -32;
	if (q < -32)  q = -32;
	if (i > 31)  i = 31;
	if (q > 31)  q = 31;

	/* Convert to 2's complement 6 bits representation */
	if (i < 0)
		i += 64;

	if (q < 0)
		q += 64;

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		if (ACMINORREV_2(pi)) {
			WRITE_PHYREGCE(pi, RxSdFeConfig7, core,
				((q & 0x3f) << 6) | (i & 0x3f));
		} else {
			if ((phy_get_phymode(pi) != PHYMODE_RSDB) && (core == 0)) {
				wlapi_exclusive_reg_access_core0(pi->sh->physhim, 1);
				WRITE_PHYREG(pi, RxSdFeConfig7, ((q & 0x3f) << 6) | (i & 0x3f));
				wlapi_exclusive_reg_access_core0(pi->sh->physhim, 0);
			} else {
				WRITE_PHYREG(pi, RxSdFeConfig7, ((q & 0x3f) << 6) | (i & 0x3f));
			}
			PHY_NONE(("%d  %d  0x%x\n", i, q, (READ_PHYREG(pi, RxSdFeConfig7)
					& 0x0fff)));
		}
	} else {
		uint16 war = READ_PHYREG(pi, work_around_ctrl);
		WRITE_PHYREG(pi, work_around_ctrl,
			(war & 0x8181) | ((q & 0x3f) << 9) | ((i & 0x3f) << 1));
		PHY_NONE(("%d  %d  0x%x\n", i, q, (READ_PHYREG(pi, work_around_ctrl) & 0x7e7e)));
	}
}

/* Based on: proc 20691_dcc_reset {  } */
void
wlc_dcc_fsm_reset(phy_info_t *pi)
{
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_hold, 0);
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_reset, 1);
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_reset, 0);
		if (ACMINORREV_2(pi)) {
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_hold, 0);
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_reset, 1);
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_reset, 0);
			}
	} else {
		uint16 sparereg;

		sparereg = READ_PHYREG(pi, SpareReg);
		ASSERT(ACREV_GE(pi->pubpi->phy_rev, 11));
		WRITE_PHYREG(pi, SpareReg,  sparereg | 0x8000);
		WRITE_PHYREG(pi, SpareReg,  sparereg & 0x7fff);
	}
	/* Wait for FSM to finish. NB:depends on total dcc_loop_count_XX */
	OSL_DELAY(10);
}

/* Based on: proc 20691_dcc_restart {  } */
static void
wlc_dcc_fsm_restart(phy_info_t *pi)
{
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_hold, 1);
		MOD_PHYREG(pi, rx_tia_dc_loop_0, dc_loop_hold, 0);
		if (ACMINORREV_2(pi)) {
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_hold, 1);
			MOD_PHYREG(pi, rx_tia_dc_loop_1, dc_loop_hold, 0);
			}
	} else {
		uint16 sparereg;

		sparereg = READ_PHYREG(pi, SpareReg);
		ASSERT(ACREV_GE(pi->pubpi->phy_rev, 11));
		WRITE_PHYREG(pi, SpareReg,  sparereg | 0x4000);
		WRITE_PHYREG(pi, SpareReg,  sparereg & 0xbfff);
	}
}

extern const uint16 tiny_rfseq_rx2tx_dly[];
extern const uint16 tiny_rfseq_rx2tx_cmd[];
extern const uint16 tiny_rfseq_rx2tx_tssi_sleep_cmd[];
extern const uint16 tiny_rfseq_rx2tx_tssi_sleep_dly[];
extern const uint16 rfseq_rx2tx_dly[];
extern const uint16 rfseq_rx2tx_cmd[];
extern const uint16 rfseq_majrev4_rx2tx_cal_cmd[];
extern const uint16 rfseq_majrev4_rx2tx_cal_dly[];

/* Set up rx2tx rfseq tables differently for cal vs. packets for tiny */
/* to avoid problems with AGC lock-up */
void
wlc_phy_tiny_rfseq_mode_set(phy_info_t *pi, bool cal_mode)
{
	if (cal_mode) {
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
				16, rfseq_majrev4_rx2tx_cal_cmd);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 112, 16,
				rfseq_majrev4_rx2tx_cal_dly);
		} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00,
				16, rfseq_rx2tx_cmd);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 112, 16,
				rfseq_rx2tx_dly);
		}
	} else {
		if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00, 16,
				tiny_rfseq_rx2tx_tssi_sleep_cmd);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 112, 16,
				tiny_rfseq_rx2tx_tssi_sleep_dly);
	} else {
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 0x00, 16,
				tiny_rfseq_rx2tx_cmd);
			wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, 112, 16,
				tiny_rfseq_rx2tx_dly);
		}
	}
}

#if defined(WLTEST)
void
wlc_phy_force_vcocal_acphy(phy_info_t *pi)
{
	/* IOVAR call */
	if (TINY_RADIO(pi))
		wlc_phy_radio_tiny_vcocal(pi);
	else
		wlc_phy_radio2069_vcocal(pi);
}
#endif  

uint16
wlc_phy_get_dac_rate_from_mode(phy_info_t *pi, uint8 dac_rate_mode)
{
	uint16 dac_rate = 200;

	if (CHSPEC_IS20(pi->radio_chanspec)) {
		switch (dac_rate_mode) {
			case 2:
				dac_rate = 600;
				break;
			case 3:
				dac_rate = 400;
				break;
			default: /* dac rate mode 1 */
				dac_rate = 200;
				break;
		}
	} else if (CHSPEC_IS40(pi->radio_chanspec)) {
		switch (dac_rate_mode) {
			case 2:
				dac_rate = 600;
				break;
			default: /* dac rate mode 1 and 3 */
				dac_rate = 400;
				break;
		}
	} else {
		dac_rate = 600;
	}

	return (dac_rate);
}

void wlc_phy_dac_rate_mode_acphy(phy_info_t *pi, uint8 dac_rate_mode)
{
	uint16 dac_rate;
	uint8 bw_idx = 0;

	bw_idx = CHSPEC_IS20(pi->radio_chanspec)? 0 : (CHSPEC_IS40(pi->radio_chanspec)? 1 : 2);
	dac_rate = wlc_phy_get_dac_rate_from_mode(pi, dac_rate_mode);

	if (dac_rate_mode == 2) {
		si_core_cflags(pi->sh->sih, 0x300, 0x200);
	} else if (dac_rate_mode == 3) {
		si_core_cflags(pi->sh->sih, 0x300, 0x300);
	} else {
		si_core_cflags(pi->sh->sih, 0x300, 0x100);
	}

	switch (dac_rate) {
		case 200:
			MOD_RADIO_REG_20691(pi, CLK_DIV_CFG1, 0, sel_dac_div,
				(pi->vcodivmode & 0x1) ? 3 : 4);
			break;
		case 400:
			MOD_RADIO_REG_20691(pi, CLK_DIV_CFG1, 0, sel_dac_div,
				(pi->vcodivmode & 0x2) ? 1 : 2);
			break;
		case 600:
			MOD_RADIO_REG_20691(pi, CLK_DIV_CFG1, 0, sel_dac_div, 0);
			break;
		default:
			PHY_ERROR(("Unsupported dac_rate %d\n", dac_rate));
			ASSERT(0);
			break;
	}

	if ((dac_rate_mode == 1) || (bw_idx == 2)) {
		MOD_PHYREG(pi, sdfeClkGatingCtrl, txlbclkmode_ovr, 0);
		MOD_PHYREG(pi, sdfeClkGatingCtrl, txlbclkmode_ovr_value, 0);
	} else {
		MOD_PHYREG(pi, sdfeClkGatingCtrl, txlbclkmode_ovr, 1);
		if (bw_idx == 1)
			MOD_PHYREG(pi, sdfeClkGatingCtrl, txlbclkmode_ovr_value, 2);
		else
			MOD_PHYREG(pi, sdfeClkGatingCtrl, txlbclkmode_ovr_value, 1);
	}

	if (TINY_RADIO(pi))
		wlc_phy_farrow_setup_tiny(pi, pi->radio_chanspec);
	else
		wlc_phy_farrow_setup_acphy(pi, pi->radio_chanspec);
}


/* ******************  HIRSSI ELNABYPASS (uCode supported). Begin ****************** */
void
wlc_phy_hirssi_elnabypass_init_acphy(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	pi_ac->hirssi_timer2g = PHY_SW_HIRSSI_OFF;
	pi_ac->hirssi_timer5g = PHY_SW_HIRSSI_OFF;

	if (PHY_SW_HIRSSI_UCODE_CAP(pi)) {
		pi_ac->hirssi_elnabyp2g_en = pi_ac->hirssi_en;
		pi_ac->hirssi_elnabyp5g_en = pi_ac->hirssi_en;
		if (pi->sh->clk) {
			wlc_phy_hirssi_elnabypass_set_ucode_params_acphy(pi);
			wlapi_bmac_write_shm(pi->sh->physhim, M_HIRSSI_FLAG, 0);
		}
	} else {
		pi_ac->hirssi_elnabyp2g_en = FALSE;
		pi_ac->hirssi_elnabyp5g_en = FALSE;
	}
}

void
wlc_phy_hirssi_elnabypass_engine(phy_info_t *pi)
{
	int16 timer;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	bool ucode_hirssi, upd = FALSE;

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		timer = pi_ac->hirssi_timer2g;
		pi_ac->hirssi_timer5g = MAX(pi_ac->hirssi_timer5g - 1, PHY_SW_HIRSSI_OFF);
		if (!pi_ac->hirssi_elnabyp2g_en) return;
	} else {
		timer = pi_ac->hirssi_timer5g;
		pi_ac->hirssi_timer2g = MAX(pi_ac->hirssi_timer2g - 1, PHY_SW_HIRSSI_OFF);
		if (!pi_ac->hirssi_elnabyp5g_en) return;
	}

	/* Logic */
	if (timer > PHY_SW_HIRSSI_OFF) {
		timer--;
		if (timer == PHY_SW_HIRSSI_OFF) {
			ucode_hirssi = wlc_phy_hirssi_elnabypass_shmem_read_clear_acphy(pi);
			if (ucode_hirssi) {
				PHY_ERROR(("wl%d: %s state:Already ON\n", pi->sh->unit,
				           __FUNCTION__));
				timer = pi_ac->hirssi_period;
			} else  {
				PHY_ERROR(("wl%d: %s state:OFF\n", pi->sh->unit, __FUNCTION__));
				timer = PHY_SW_HIRSSI_OFF;
				upd = TRUE;

			}
		}
	} else {
		ucode_hirssi = wlc_phy_hirssi_elnabypass_shmem_read_clear_acphy(pi);
		if (ucode_hirssi) {
			PHY_ERROR(("wl%d: %s state:ON\n", pi->sh->unit, __FUNCTION__));
			timer = pi_ac->hirssi_period;
			upd = TRUE;
		}
	}

	/* Update Timer */
	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		pi_ac->hirssi_timer2g = timer;
	} else {
		pi_ac->hirssi_timer5g = timer;
	}

	/* Update uCode & apply gainctrl changes */
	if (upd) {
		wlc_phy_hirssi_elnabypass_set_ucode_params_acphy(pi);
		wlc_phy_hirssi_elnabypass_apply_acphy(pi);
	}
}

void
wlc_phy_hirssi_elnabypass_set_ucode_params_acphy(phy_info_t *pi)
{
	int16 hirssi_rssi = 50;
	uint16 hirssi_w1_reg =  PHY_SW_HIRSSI_W1_BYP_REG;
	uint16 hirssi_w1_cnt = 500;
	bool res, en;
	uint8 factor;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (!PHY_SW_HIRSSI_UCODE_CAP(pi)) return;

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		en = pi_ac->hirssi_elnabyp2g_en;
		res = pi_ac->hirssi_timer2g > PHY_SW_HIRSSI_OFF;
	} else {
		en = pi_ac->hirssi_elnabyp5g_en;
		res = pi_ac->hirssi_timer5g > PHY_SW_HIRSSI_OFF;
	}

	if (en) {
		hirssi_rssi = (res) ? pi_ac->hirssi_res_rssi : pi_ac->hirssi_byp_rssi;
		hirssi_w1_reg = (res) ? PHY_SW_HIRSSI_W1_RES_REG : PHY_SW_HIRSSI_W1_BYP_REG;
		hirssi_w1_cnt = (res) ? pi_ac->hirssi_res_cnt : pi_ac->hirssi_byp_cnt;

		factor = CHSPEC_IS20(pi->radio_chanspec) ? 1 :
		        (CHSPEC_IS40(pi->radio_chanspec) ? 2 : 4);
		hirssi_w1_cnt *= factor;
	}

	wlapi_bmac_write_shm(pi->sh->physhim, M_HIRSSI_THR, hirssi_rssi);
	wlapi_bmac_write_shm(pi->sh->physhim, M_PHYREG_WRSSI, hirssi_w1_reg);
	wlapi_bmac_write_shm(pi->sh->physhim, M_WRSSI_THR, hirssi_w1_cnt);
}

void
wlc_phy_hirssi_elnabypass_apply_acphy(phy_info_t *pi)
{
	if (!(pi->sh->clk))
		return;

	wlapi_suspend_mac_and_wait(pi->sh->physhim);

#ifndef WLC_DISABLE_ACI
	if (!ACPHY_ENABLE_FCBS_HWACI(pi)) {
		/* ACI - reset aci for current band & restore defaults */
		wlc_phy_desense_aci_reset_params_acphy(pi, FALSE, CHSPEC_IS2G(pi->radio_chanspec),
		                                       CHSPEC_IS5G(pi->radio_chanspec));
		wlc_phy_desense_calc_total_acphy(pi);
		wlc_phy_desense_apply_acphy(pi, FALSE);
	}
#endif /* !WLC_DISABLE_ACI */

	/* Set new gainctrl with current aci_off/elna_bypass settings */
	wlc_phy_rxgainctrl_set_gaintbls_acphy(pi, TRUE, TRUE, TRUE);
	wlc_phy_rxgainctrl_gainctrl_acphy(pi);

	wlc_phy_resetcca_acphy(pi);
	wlc_phy_force_rfseq_acphy(pi, ACPHY_RFSEQ_RESET2RX);

	wlapi_enable_mac(pi->sh->physhim);
}


bool
wlc_phy_hirssi_elnabypass_shmem_read_clear_acphy(phy_info_t *pi)
{
	bool hirssi = FALSE;

	if (PHY_SW_HIRSSI_UCODE_CAP(pi)) {
		hirssi = (wlapi_bmac_read_shm(pi->sh->physhim, M_HIRSSI_FLAG) == 0xdead);
		if (hirssi)
			wlapi_bmac_write_shm(pi->sh->physhim, M_HIRSSI_FLAG, 0);
	}

	return hirssi;
}

bool
wlc_phy_hirssi_elnabypass_status_acphy(phy_info_t *pi)
{
	bool status;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (CHSPEC_IS2G(pi->radio_chanspec))
		status = pi_ac->hirssi_timer2g > PHY_SW_HIRSSI_OFF;
	else
		status = pi_ac->hirssi_timer5g > PHY_SW_HIRSSI_OFF;
	if (!pi->sh->clk)
		return status;
	else
		return (status || (wlapi_bmac_read_shm(pi->sh->physhim, M_HIRSSI_FLAG) == 0xdead));
}
/* ******************  HIRSSI ELNABYPASS (uCode supported). End  ****************** */

static void
wlc_txswctrlmap_set_acphy(phy_info_t *pi, int8 pamode_requested)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/*
	 * Populate the right swctrlmap only if the pa_mode requested is different
	 * from the current setting
	 */
	if (pi_ac->pa_mode != pamode_requested) {
		/* Note the new state */
		pi_ac->pa_mode = pamode_requested;

		/* Call this function again to repopulate the switch control table. */
		wlc_phy_write_regtbl_fc_from_nvram(pi);
	}
}

static int8
wlc_txswctrlmap_get_acphy(phy_info_t *pi)
{
	return pi->u.pi_acphy->pa_mode;
}

void
wlc_phy_btc_adjust_acphy(phy_info_t *pi, bool btactive)
{
	if (ACMAJORREV_0(pi->pubpi->phy_rev)) {
	  wlapi_suspend_mac_and_wait(pi->sh->physhim);
	  wlc_phy_mlua_adjust_acphy(pi, btactive);
	  wlapi_enable_mac(pi->sh->physhim);
	}
}

void
wlc_phy_mlua_adjust_acphy(phy_info_t *pi, bool btactive)
{
	uint8 zfuA1, zfuA1_log2, zfuA2, zfuA2_log2;
	uint8 mluA1, mluA1_log2, mluA2, mluA2_log2;

	/* Disable this for now, there is some issue with BTcoex */
	if (btactive) {
		mluA1 = 2; mluA1_log2 = 1; mluA2 = 0; mluA2_log2 = 0;
		zfuA1 = 2; zfuA1_log2 = 1; zfuA2 = 2; zfuA2_log2 = 1;
	} else {
		mluA1 = 4; mluA1_log2 = 2; mluA2 = 4; mluA2_log2 = 2;
		zfuA1 = 4; zfuA1_log2 = 2; zfuA2 = 4; zfuA2_log2 = 2;
	}

	/* Increase Channel Update ML mu */
	if (ACMAJORREV_0(pi->pubpi->phy_rev) && (ACMINORREV_0(pi) || ACMINORREV_1(pi))) {
		/* 4360 a0,b0 */
		MOD_PHYREG(pi, mluA, mluA1, mluA1);
		MOD_PHYREG(pi, mluA, mluA2, mluA2);
		/* zfuA register used to update channel for 256 QAM */
		MOD_PHYREG(pi, zfuA, zfuA1, zfuA1);
		MOD_PHYREG(pi, zfuA, zfuA2, zfuA2);
	} else if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* 4350 a0,b0 (log domain) */
		MOD_PHYREG(pi, mluA, mluA1, mluA1_log2);
		MOD_PHYREG(pi, mluA, mluA2, mluA2_log2);
		/* zfuA register used to update channel for 256 QAM */
		MOD_PHYREG(pi, zfuA, zfuA1, zfuA1_log2);
		MOD_PHYREG(pi, zfuA, zfuA2, zfuA2_log2);
	} else {
	}
}

void
wlc_phy_stop_bt_toggle_acphy(phy_info_t *pi)
{
	uint8 phyrxchain = pi->sh->phyrxchain;
	uint8 phytxchain = pi->sh->phytxchain;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	int8 shared_ant_mask;
	if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL_FEM_BT) {
		if (BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) & BFL2_BT_SHARE_ANT0)
			shared_ant_mask = 1; /* 2 chain devices with first core shared */
		else
			shared_ant_mask = 2; /* 3 chain devices with middle core shared */
	} else
		return;

	if (pi_ac->bt_sw_state == AUTO) {
		phy_utils_phyreg_enter(pi);
		if (((phytxchain & shared_ant_mask) == 0) &&
			((shared_ant_mask & phyrxchain) == 0)) {
			wlc_phy_set_femctrl_bt_wlan_ovrd_acphy(pi, 1);
			/* forced bt switch to BT side instead of toggling */
		} else
			wlc_phy_set_femctrl_bt_wlan_ovrd_acphy(pi, AUTO);
		phy_utils_phyreg_exit(pi);
	}
}

#ifdef ATE_BUILD
void
wlc_phy_gpaio_acphy(phy_info_t *pi, wl_gpaio_option_t option, int core)
{
	ASSERT(TINY_RADIO(pi));

	/* powerup gpaio block */
	MOD_RADIO_REG_TINY(pi, GPAIO_SEL2, core, gpaio_pu, 1);
	/* powerdown rcal, otherwise it conflicts */
	MOD_RADIO_REG_TINY(pi, RCAL_CFG_NORTH, core, rcal_pu, 0);

	/* To bring out various radio test signals on gpaio. */
	if (option == GPAIO_PMU_CLEAR)
	    MOD_RADIO_REG_TINY(pi, GPAIO_SEL0, core, gpaio_sel_0to15_port, (0x1 << 0));
	else if (option == GPAIO_ICTAT_CAL) {
	    MOD_RADIO_REG_TINY(pi, GPAIO_SEL0, core, gpaio_sel_0to15_port, 0x0);
	    MOD_RADIO_REG_TINY(pi, GPAIO_SEL1, core, gpaio_sel_16to31_port, (0x1 << 11));
	}
	else
	    MOD_RADIO_REG_TINY(pi, GPAIO_SEL0, core, gpaio_sel_0to15_port, (0x1 << 14));

	if (option != GPAIO_ICTAT_CAL)
		MOD_RADIO_REG_TINY(pi, GPAIO_SEL1, core, gpaio_sel_16to31_port, 0x0);
	switch (option) {
		case (GPAIO_PMU_AFELDO): {
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_tsten, 0x01);
			MOD_RADIO_REG_TINY(pi, PMU_CFG1, core, wlpmu_ana_mux, 0x00);
			break;
		}
		case (GPAIO_PMU_TXLDO): {
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_tsten, 0x01);
			MOD_RADIO_REG_TINY(pi, PMU_CFG1, core, wlpmu_ana_mux, 0x01);
			break;
		}
		case (GPAIO_PMU_VCOLDO): {
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_tsten, 0x01);
			MOD_RADIO_REG_TINY(pi, PMU_CFG1, core, wlpmu_ana_mux, 0x02);
			break;
		}
		case GPAIO_PMU_LNALDO: {
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_tsten, 0x01);
			MOD_RADIO_REG_TINY(pi, PMU_CFG1, core, wlpmu_ana_mux, 0x03);
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_ana_mux_high, 0x00);
			break;
		}
		case GPAIO_PMU_ADCLDO: {
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_tsten, 0x01);
			MOD_RADIO_REG_TINY(pi, PMU_CFG1, core, wlpmu_ana_mux, 0x03);
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_ana_mux_high, 0x01);
			break;
		}
		case GPAIO_PMU_CLEAR: {
			MOD_RADIO_REG_TINY(pi, PMU_CFG3, core, wlpmu_tsten, 0x00);
			break;
		}
		case GPAIO_OFF: {
			MOD_RADIO_REG_TINY(pi, GPAIO_SEL2, core, gpaio_pu, 0);
			break;
		}
		default:
			break;
	}
}
#endif /* ATE_BUILD */

static void
BCMATTACHFN(wlc_phy_std_params_attach_acphy)(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	uint8 i, core, core_num;
	uint8 gain_len[] = {2, 6, 7, 10, 8, 8, 11}; /* elna, lna1, lna2, mix, bq0, bq1, dvga */

	if (TINY_RADIO(pi)) {
		gain_len[3] = 12; /* tia */
		gain_len[5] = 3;  /* farrow */
	}

	pi->phy_cal_mode = PHY_PERICAL_MPHASE;
	pi->phy_cal_delay = PHY_PERICAL_DELAY_DEFAULT;
	pi->phy_scraminit = AUTO;
	pi->cal_period = PHY_PERICAL_MAXINTRVL;

#ifdef WL_SAR_SIMPLE_CONTROL
	/* user specified sarlimit by nvram. off as a default */
	pi->dynamic_sarctrl_2g = 0;
	pi->dynamic_sarctrl_5g = 0;
#endif /* WL_SAR_CONTROL_LIMIT */

	pi_ac->init = FALSE;
#if defined(BCMDBG)
	pi_ac->fdiqi->forced = FALSE;
	pi_ac->fdiqi->forced_val = 0;
#endif
	pi_ac->curr_band2g = CHSPEC_IS2G(pi->radio_chanspec);
	pi_ac->band2g_init_done = FALSE;
	pi_ac->band5g_init_done = FALSE;
	pi_ac->prev_subband = 15;
	pi_ac->curr_bw = CHSPEC_BW(pi->radio_chanspec);
	pi_ac->curr_spurmode = 0;
	pi_ac->fast_adc_en = 0;
	pi_ac->use_fast_adc_20_40 = 0;

	if (CHIPID(pi->sh->chip) == BCM4335_CHIP_ID) {
		if (pi->sh->chippkg == BCM4335_WLBGA_PKG_ID) {
			pi_ac->curr_spurmode = 8;
			pi->acphy_spuravoid_mode = 8;
		} else {
			/* for WLCSP packages */
			if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
				/* for 4335 Cx Chips */
				pi_ac->curr_spurmode = 8;
				pi->acphy_spuravoid_mode = 8;
			} else {
				/* for 4335 Ax/Bx Chips */
				pi_ac->curr_spurmode = 2;
				pi->acphy_spuravoid_mode = 2;
			}
		}
		if (pi->xtalfreq == 52000000) {
			pi_ac->curr_spurmode = 0;
			pi->acphy_spuravoid_mode = 0;
		}
	}
	pi->acphy_spuravoid_mode_override = 0;

	pi_ac->dac_mode = 1;
	pi_ac->rccal_gmult = 128;
	pi_ac->rccal_gmult_rc = 128;
	pi_ac->rccal_dacbuf = 12;
	pi_ac->txcal_cache_cookie = 0;
	pi_ac->poll_adc_WAR = FALSE;
	pi_ac->crsmincal_enable = TRUE;
	pi_ac->force_crsmincal  = FALSE;
	pi_ac->crsmincal_run = 0;
#ifndef BOARD_FLAGS
	BF_ELNA_2G(pi_ac) = FALSE;
	BF_ELNA_5G(pi_ac) = FALSE;
#endif /* BOARD_FLAGS */
	pi_ac->acphy_lp_mode = 1;
	pi_ac->acphy_prev_lp_mode = pi_ac->acphy_lp_mode;
	pi_ac->acphy_lp_status = pi_ac->acphy_lp_mode;
	if (ACMAJORREV_4(pi->pubpi->phy_rev) && ACMINORREV_1(pi)) {
		/* Keeping smoothing disabled on 4349A2 since 80 MHz c9s2 has issues */
		pi_ac->acphy_enable_smth = SMTH_DISABLE;
	} else {
		pi_ac->acphy_enable_smth = SMTH_ENABLE;
	}
	pi_ac->ant_swOvr_state_core0 = 2;
	pi_ac->ant_swOvr_state_core1 = 2;
	pi_ac->acphy_smth_dump_mode = SMTH_NODUMP;

	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* 4349A0 uses 12Ghz VCO */
		pi_ac->vco_12GHz = TRUE;
	} else {
		/* Disable 12Ghz VCO for all other chips */
		pi_ac->vco_12GHz = FALSE;
	}

	pi_ac->acphy_4335_radio_pd_status = 0;
	pi_ac->phy_crs_th_from_crs_cal = ACPHY_CRSMIN_DEFAULT;
	/* AFE */
	pi_ac->afeRfctrlCoreAfeCfg10 = READ_PHYREG(pi, RfctrlCoreAfeCfg10);
	pi_ac->afeRfctrlCoreAfeCfg20 = READ_PHYREG(pi, RfctrlCoreAfeCfg20);
	pi_ac->afeRfctrlOverrideAfeCfg0 = READ_PHYREG(pi, RfctrlOverrideAfeCfg0);
	/* Radio RX */
	pi_ac->rxRfctrlCoreRxPus0 = READ_PHYREG(pi, RfctrlCoreRxPus0);
	pi_ac->rxRfctrlOverrideRxPus0 = READ_PHYREG(pi, RfctrlOverrideRxPus0);
	/* Radio TX */
	pi_ac->txRfctrlCoreTxPus0 = READ_PHYREG(pi, RfctrlCoreTxPus0);
	pi_ac->txRfctrlOverrideTxPus0 = READ_PHYREG(pi, RfctrlOverrideTxPus0);
	/* {radio, rfpll, pllldo}_pu = 0 */
	pi_ac->radioRfctrlCmd = READ_PHYREG(pi, RfctrlCmd);
	pi_ac->radioRfctrlCoreGlobalPus = READ_PHYREG(pi, RfctrlCoreGlobalPus);
	pi_ac->radioRfctrlOverrideGlobalPus = READ_PHYREG(pi, RfctrlOverrideGlobalPus);
	/* read chipid */
#ifndef BCMCHIPID
	/* Used to select correct radio offsets based on chipid */
	acphychipid = pi->sh->chip;
#endif
	/* pre_init to ON, register POR default setting */
	pi_ac->ac_rxldpc_override = ON;
	/* RSSI reg reporintg only happens for one core at a time */
	pi_ac->rssi_coresel = 0;
	/* ucode hirssi detect - bypass lna1 to save it */
	pi_ac->hirssi_en = 0; /* ACMAJORREV_0(pi->pubpi->phy_rev); */
	pi_ac->hirssi_period = PHY_SW_HIRSSI_PERIOD;
	pi_ac->hirssi_byp_rssi = PHY_SW_HIRSSI_BYP_THR;
	pi_ac->hirssi_res_rssi = PHY_SW_HIRSSI_RES_THR;
	pi_ac->hirssi_byp_cnt = PHY_SW_HIRSSI_W1_BYP_CNT;
	pi_ac->hirssi_res_cnt = PHY_SW_HIRSSI_W1_RES_CNT;

	/* J28 have attenuators, so don't use hirssi feature there */
	if (pi->sh->boardvendor == VENDOR_APPLE &&
	    ((pi->sh->boardtype == BCM94360J28_D11AC2G) ||
	     (pi->sh->boardtype == BCM94360J28_D11AC5G))) {
		pi_ac->hirssi_en = FALSE;
	}
	if (!ACMAJORREV_3(pi->pubpi->phy_rev)) {
		/* Only supported in ucode for mac revid 40 and 42 */
		wlc_phy_hirssi_elnabypass_init_acphy(pi);
	}

	pi->n_preamble_override = WLC_N_PREAMBLE_MIXEDMODE;

	/* RX-IQ-CAL per core */
	pi_ac->rxiqcal_percore_2g = FALSE;
	pi_ac->rxiqcal_percore_5g = TRUE;
	/* default clip1_th & edcrs_en */
	pi_ac->clip1_th = 0x404e;
	pi_ac->edcrs_en = 0xfff;

	/* Get xtal frequency from PMU */
#if !defined(XTAL_FREQ)
	pi->xtalfreq = si_alp_clock(pi->sh->sih);
#endif
	ASSERT((PHY_XTALFREQ(pi->xtalfreq) % 1000) == 0);
	PHY_INFORM(("wl%d: %s: using %d.%d MHz xtalfreq for RF PLL\n",
		pi->sh->unit, __FUNCTION__,
		PHY_XTALFREQ(pi->xtalfreq) / 1000000, PHY_XTALFREQ(pi->xtalfreq) % 1000000));

	FOREACH_CORE(pi, core)
		pi_ac->txpwrindex_hw_save[core] = 128;

	for (i = 0; i < ACPHY_MAX_RX_GAIN_STAGES; i++)
		pi_ac->rxgainctrl_stage_len[i] = gain_len[i];

	bzero((uint8 *)pi_ac->phy_noise_pwr_array, sizeof(pi_ac->phy_noise_pwr_array));
	bzero((uint8 *)pi_ac->phy_noise_in_crs_min, sizeof(pi_ac->phy_noise_in_crs_min));

	pi_ac->phy_debug_crscal_counter = 0;
	pi_ac->phy_debug_crscal_channel = 0;
	pi_ac->phy_noise_counter = 0;

	if (ACMAJORREV_0(pi->pubpi->phy_rev)) {
		uint8 subband_num;
		for (subband_num = 0; subband_num <= 4; subband_num++)
			FOREACH_CORE(pi, core_num)
				pi_ac->phy_noise_cache_crsmin[subband_num][core_num] = -30;
	} else {
		FOREACH_CORE(pi, core_num) {
			pi_ac->phy_noise_cache_crsmin[0][core_num] = -30;
			if (!(BF_ELNA_5G(pi_ac)) &&
			(ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_4(pi->pubpi->phy_rev))) {
				pi_ac->phy_noise_cache_crsmin[1][core_num] = -32;
				pi_ac->phy_noise_cache_crsmin[2][core_num] = -32;
				pi_ac->phy_noise_cache_crsmin[3][core_num] = -31;
				pi_ac->phy_noise_cache_crsmin[4][core_num] = -31;
			} else {
				pi_ac->phy_noise_cache_crsmin[1][core_num] = -28;
				pi_ac->phy_noise_cache_crsmin[2][core_num] = -28;
				pi_ac->phy_noise_cache_crsmin[3][core_num] = -26;
				pi_ac->phy_noise_cache_crsmin[4][core_num] = -25;
			}
		}
	}

	if (ACMAJORREV_1(pi->pubpi->phy_rev))
		pi_ac->acphy_force_lpvco_2G = 1; /* Enable 2G LP mode */
	else
		pi_ac->acphy_force_lpvco_2G = 0;

	/* 4349A0: priority flags for 80p80 and RSDB modes
	   these flags indicate which core's tuning settings takes
	   precedence in conflict scenarios
	 */
	pi_ac->crisscross_priority_core_80p80 = 0;
	pi_ac->crisscross_priority_core_rsdb = 0;

	pi_ac->is_crisscross_actv = 0;
}

static bool
BCMATTACHFN(wlc_phy_nvram_attach_acphy)(phy_info_t *pi)
{
	uint8 i;
#ifndef BOARD_FLAGS3
	uint32 bfl3; /* boardflags3 */
#endif
	int ref_count = 0;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	/* OBJECT REGISTRY: check if shared key has value already stored */
	pi_ac->sromi = (acphy_srom_info_t *)wlapi_obj_registry_get(pi->sh->physhim,
		OBJR_ACPHY_SROM_INFO);

	if (pi_ac->sromi == NULL) {
		if ((pi_ac->sromi = (acphy_srom_info_t *)MALLOC(pi->sh->osh,
			sizeof(acphy_srom_info_t))) == NULL) {

			PHY_ERROR(("wl%d: %s: out of memory, malloced %d bytes\n",
				pi->sh->unit, __FUNCTION__, MALLOCED(pi->sh->osh)));
			return FALSE;
		}
		bzero((char*)pi_ac->sromi, sizeof(acphy_srom_info_t));

		/* OBJECT REGISTRY: We are the first instance, store value for key */
		wlapi_obj_registry_set(pi->sh->physhim, OBJR_ACPHY_SROM_INFO, pi_ac->sromi);
	}
	/* OBJECT REGISTRY: Reference the stored value in both instances */
	ref_count = wlapi_obj_registry_ref(pi->sh->physhim, OBJR_ACPHY_SROM_INFO);
	ASSERT(ref_count <= MAX_RSDB_MAC_NUM);
	BCM_REFERENCE(ref_count);

	pi->sromi->rpcal2g = (uint16)PHY_GETINTVAR_DEFAULT(pi, rstr_rpcal2g, 0);
	pi->sromi->rpcal5gb0 = (uint16)PHY_GETINTVAR_DEFAULT(pi, rstr_rpcal5gb0, 0);
	pi->sromi->rpcal5gb1 = (uint16)PHY_GETINTVAR_DEFAULT(pi, rstr_rpcal5gb1, 0);
	pi->sromi->rpcal5gb2 = (uint16)PHY_GETINTVAR_DEFAULT(pi, rstr_rpcal5gb2, 0);
	pi->sromi->rpcal5gb3 = (uint16)PHY_GETINTVAR_DEFAULT(pi, rstr_rpcal5gb3, 0);
	pi->sromi->txidxcap2g = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_txidxcap2g, 0);
	pi->sromi->txidxcap5g = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_txidxcap5g, 0);
	pi->sromi->epagain2g = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_epagain2g, 0);
	pi->sromi->epagain5g = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_epagain5g, 0);
	pi->sromi->extpagain2g = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_extpagain2g, 0);
	pi->sromi->extpagain5g = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_extpagain5g, 0);

	pi->sromi->subband5Gver =
		(uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_subband5gver, PHY_SUBBAND_4BAND);

	pi->ipa2g_on = (pi->sromi->extpagain2g == 2);
	pi->ipa5g_on = (pi->sromi->extpagain5g == 2);

	/* update txpwr settings */
	wlc_phy_txpower_ipa_upd(pi);

	pi->bphy_scale = (uint16) (PHY_GETINTVAR_DEFAULT(pi, rstr_bphyscale, 0));
	pi->dacratemode2g = (uint8) (PHY_GETINTVAR_DEFAULT(pi, rstr_dacratemode2g, 1));
	pi->dacratemode5g = (uint8) (PHY_GETINTVAR_DEFAULT(pi, rstr_dacratemode5g, 1));
#ifndef WL_FDSS_DISABLED
	pi->fdss_interp_en = (uint8) (PHY_GETINTVAR_DEFAULT(pi, rstr_fdss_interp_en, 1));
	for (i = 0; i < 2; i++) {
		pi->fdss_level_2g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT(pi,
			rstr_fdss_level_2g, i, -1));
		pi->fdss_level_5g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT(pi,
			rstr_fdss_level_5g, i, -1));
	}
#endif /* !WL_FDSS_DISABLED */
	pi->vcodivmode = (uint8) (PHY_GETINTVAR_DEFAULT(pi, rstr_vcodivmode, 0));
	pi->epacal2g = (uint8) (PHY_GETINTVAR_DEFAULT(pi, rstr_epacal2g, 0));
	pi->epacal5g = (uint8) (PHY_GETINTVAR_DEFAULT(pi, rstr_epacal5g, 0));
	pi->epacal2g_mask = (uint16) (PHY_GETINTVAR_DEFAULT(pi, rstr_epacal2g_mask, 0x3fff));

	for (i = 0; i < 3; i++) {
		pi->pacalshift2g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			(pi, rstr_pacalshift2g, i, 0));
		pi->pacalshift5g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			(pi, rstr_pacalshift5g, i, 0));
	}

	pi->pacalindex2g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_pacalindex2g, -1));
	for (i = 0; i < 3; i++) {
		pi->pacalindex5g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			(pi, rstr_pacalindex5g, i, -1));
	}

	pi->txiqcalidx2g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_txiqcalidx2g, -1));
	pi->txiqcalidx5g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_txiqcalidx5g, -1));

	pi->pacalpwr2g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_pacalpwr2g, -99));
	if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		/* For 4350, pacalpwr5g = lo, mi, hi, x1, lo, mi, hi, x1 */
		/*                       |    core 0     |     core 1    | */
		for (i = 0; i < 8; i++) {
			pi->pacalpwr5g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			        (pi, rstr_pacalpwr5g, i, -99));
			pi->pacalpwr5g40[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			        (pi, rstr_pacalpwr5g40, i, -99));
			pi->pacalpwr5g80[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			        (pi, rstr_pacalpwr5g80, i, -99));
		}
	} else {
		/* For 4345 and others */
		for (i = 0; i < 4; i++) {
			pi->pacalpwr5g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			        (pi, rstr_pacalpwr5g, i, -99));
		}
	}

	pi->txgaintbl5g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_txgaintbl5g, -1));

	pi->parfps2g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_parfps2g, -1));
	pi->parfps5g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_parfps5g, -1));

	pi->papdbbmult2g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_papdbbmult2g, -1));
	pi->papdbbmult5g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_papdbbmult5g, -1));

	pi->pacalmode = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_pacalmode, -1));
	pi->pacalopt = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_pacalopt, -1));

	pi->patoneidx2g = (int8) (PHY_GETINTVAR_DEFAULT(pi, rstr_patoneidx2g, -1));
	for (i = 0; i < 4; i++) {
		pi->patoneidx5g[i] = (int8) (PHY_GETINTVAR_ARRAY_DEFAULT
			(pi, rstr_patoneidx5g, i, -1));
	}

	/* Default value for forced papd cal index */
	pi_ac->pacalidx_iovar = -1;

	pi_ac->papdmode = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_papdmode, PAPD_LMS);
	pi_ac->srom_tworangetssi2g = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_tworangetssi2g, FALSE);
	pi_ac->srom_tworangetssi5g = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_tworangetssi5g, FALSE);
	pi_ac->srom_lowpowerrange2g = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_lowpowerrange2g, FALSE);
	pi_ac->srom_lowpowerrange5g = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_lowpowerrange5g, FALSE);
	pi_ac->srom_2g_pdrange_id = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_pdgain2g, 0);
	pi_ac->srom_5g_pdrange_id = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_pdgain5g, 0);
	pi_ac->srom_paprdis = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_paprdis, FALSE);
	pi_ac->srom_papdwar = (int8)PHY_GETINTVAR_DEFAULT(pi, rstr_papdwar, -1);


	if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
		pi_ac->srom_tssisleep_en = (uint)PHY_GETINTVAR_DEFAULT(pi, rstr_tssisleep_en, 7);
	} else {
		pi_ac->srom_tssisleep_en = (uint)PHY_GETINTVAR_DEFAULT(pi, rstr_tssisleep_en, 0);
	}

#ifndef FEMCTRL
	BFCTL(pi_ac) = (uint8)PHY_GETINTVAR_DEFAULT(pi, rstr_femctrl, 0);
#endif /* FEMCTRL */

#ifndef BOARD_FLAGS
	BF_SROM11_BTCOEX(pi_ac) = ((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) &
		BFL_SROM11_BTCOEX) != 0);
	BF_SROM11_GAINBOOSTA01(pi_ac) = ((BOARDFLAGS(GENERIC_PHY_INFO(pi)->boardflags) &
		BFL_SROM11_GAINBOOSTA01) != 0);
#endif /* BOARD_FLAGS */

#ifndef BOARD_FLAGS2
	BF2_SROM11_APLL_WAR(pi_ac) = ((BOARDFLAGS2(GENERIC_PHY_INFO(pi)->boardflags2) &
		BFL2_SROM11_APLL_WAR) != 0);
	BF2_2G_SPUR_WAR(pi_ac) = ((BOARDFLAGS2(GENERIC_PHY_INFO(pi)->boardflags2) &
		BFL2_2G_SPUR_WAR) != 0);
	BF2_DAC_SPUR_IMPROVEMENT(pi_ac) = (BOARDFLAGS2(GENERIC_PHY_INFO(pi)->boardflags2) &
		BFL2_DAC_SPUR_IMPROVEMENT) != 0;
#endif /* BOARD_FLAGS2 */

#ifndef BOARD_FLAGS3
	if ((PHY_GETVAR(pi, rstr_boardflags3)) != NULL) {
		bfl3 = (uint32)PHY_GETINTVAR(pi, rstr_boardflags3);
		BF3_FEMCTRL_SUB(pi_ac) = bfl3 & BFL3_FEMCTRL_SUB;
		BF3_AGC_CFG_2G(pi_ac) = ((bfl3 & BFL3_AGC_CFG_2G) != 0);
		BF3_AGC_CFG_5G(pi_ac) = ((bfl3 & BFL3_AGC_CFG_5G) != 0);
		BF3_5G_SPUR_WAR(pi_ac) = ((bfl3 & BFL3_5G_SPUR_WAR) != 0);
		BF3_FEMTBL_FROM_NVRAM(pi_ac) = (bfl3 & BFL3_FEMTBL_FROM_NVRAM)
			>> BFL3_FEMTBL_FROM_NVRAM_SHIFT;
		BF3_AVVMID_FROM_NVRAM(pi_ac) = (bfl3 & BFL3_AVVMID_FROM_NVRAM)
			>> BFL3_AVVMID_FROM_NVRAM_SHIFT;
		/* BF3_VLIN_EN_FROM_NVRAM(pi_ac) = (bfl3 & BFL3_VLIN_EN_FROM_NVRAM)
			>> BFL3_VLIN_EN_FROM_NVRAM_SHIFT;
		*/
		BF3_TXGAINTBLID(pi_ac) = (bfl3 & BFL3_TXGAINTBLID) >> BFL3_TXGAINTBLID_SHIFT;
		BF3_TSSI_DIV_WAR(pi_ac) = (bfl3 & BFL3_TSSI_DIV_WAR) >> BFL3_TSSI_DIV_WAR_SHIFT;
		BF3_RCAL_WAR(pi_ac) = ((bfl3 & BFL3_RCAL_WAR) != 0);
		BF3_RCAL_OTP_VAL_EN(pi_ac) = ((bfl3 & BFL3_RCAL_OTP_VAL_EN) != 0);
		BF3_PPR_BIT_EXT(pi_ac) = (bfl3 & BFL3_PPR_BIT_EXT) >> BFL3_PPR_BIT_EXT_SHIFT;
		BF3_BBPLL_SPR_MODE_DIS(pi_ac) = ((bfl3 & BFL3_BBPLL_SPR_MODE_DIS) != 0);
		BF3_2GTXGAINTBL_BLANK(pi_ac) = (bfl3 & BFL3_2GTXGAINTBL_BLANK) >>
			BFL3_2GTXGAINTBL_BLANK_SHIFT;
		BF3_5GTXGAINTBL_BLANK(pi_ac) = (bfl3 & BFL3_5GTXGAINTBL_BLANK) >>
			BFL3_5GTXGAINTBL_BLANK_SHIFT;
		BF3_PHASETRACK_MAX_ALPHABETA(pi_ac) = (bfl3 & BFL3_PHASETRACK_MAX_ALPHABETA) >>
			BFL3_PHASETRACK_MAX_ALPHABETA_SHIFT;
		BF3_LTECOEX_GAINTBL_EN(pi_ac) = (bfl3 & BFL3_LTECOEX_GAINTBL_EN) >>
			BFL3_LTECOEX_GAINTBL_EN_SHIFT;
		BF3_ACPHY_LPMODE_2G(pi_ac) = (bfl3 & BFL3_ACPHY_LPMODE_2G) >>
			BFL3_ACPHY_LPMODE_2G_SHIFT;
		BF3_ACPHY_LPMODE_5G(pi_ac) = (bfl3 & BFL3_ACPHY_LPMODE_5G) >>
			BFL3_ACPHY_LPMODE_5G_SHIFT;
		BF3_RSDB_1x1_BOARD(pi_ac) = (bfl3 & BFL3_1X1_RSDB_ANT) >>
			BFL3_1X1_RSDB_ANT_SHIFT;
	} else {
		BF3_FEMCTRL_SUB(pi_ac) = 0;
		BF3_AGC_CFG_2G(pi_ac) = 0;
		BF3_AGC_CFG_5G(pi_ac) = 0;
		BF3_5G_SPUR_WAR(pi_ac) = 0;
		BF3_FEMTBL_FROM_NVRAM(pi_ac) = 0;
		BF3_TXGAINTBLID(pi_ac) = 0;
		BF3_TSSI_DIV_WAR(pi_ac) = 0;
		BF3_RCAL_WAR(pi_ac) = 0;
		BF3_PPR_BIT_EXT(pi_ac) = 0;
		BF3_RCAL_OTP_VAL_EN(pi_ac) = 0;
		BF3_BBPLL_SPR_MODE_DIS(pi_ac) = 0;
		BF3_2GTXGAINTBL_BLANK(pi_ac) = 0;
		BF3_5GTXGAINTBL_BLANK(pi_ac) = 0;
		BF3_PHASETRACK_MAX_ALPHABETA(pi_ac) = 0;
		BF3_LTECOEX_GAINTBL_EN(pi_ac) = 0;
		BF3_ACPHY_LPMODE_2G(pi_ac) = 0;
		BF3_ACPHY_LPMODE_5G(pi_ac) = 0;
		BF3_RSDB_1x1_BOARD(pi_ac) = 0;
	}
#endif /* BOARD_FLAGS3 */

	pi->sromi->dBpad = pi->sh->boardflags4 & BFL4_SROM12_4dBPAD;
	pi->sromi->precal_tx_idx = ((pi->sh->boardflags & BFL_SROM11_PRECAL_TX_IDX) != 0);
	pi->sromi->epa_on_during_txiqlocal = ((pi->sh->boardflags2 &
	    BFL2_SROM11_EPA_ON_DURING_TXIQLOCAL) != 0);
	if ((PHY_GETVAR(pi, rstr_cckdigfilttype)) != NULL) {
		pi->sromi->cckfilttype = (uint8)PHY_GETINTVAR(pi, rstr_cckdigfilttype);
	} else {
		if (ACMAJORREV_1(pi->pubpi->phy_rev) &&
			ACMINORREV_2(pi) &&
			((pi->sromi->epagain2g == 2) || (pi->sromi->extpagain2g == 2)) &&
			((pi->sromi->epagain5g == 2) || (pi->sromi->extpagain5g == 2)) &&
			pi->xtalfreq == 40000000) {
			/* 43162yp improving ACPR */
			pi->sromi->cckfilttype = 0x02;
		} else {
			/* bit0 is gaussian shaping and bit1 & 2 are for RRC alpha */
			pi->sromi->cckfilttype = 0x01;
		}
	}

	pi->sromi->ofdmfilttype = (uint8)PHY_GETINTVAR_DEFAULT(pi,
		rstr_ofdmfilttype_5gbe, 127);
	pi->sromi->ofdmfilttype_2g = (uint8)PHY_GETINTVAR_DEFAULT(pi,
		rstr_ofdmfilttype_2gbe, 127);

	if ((PHY_GETVAR(pi, rstr_pagc2g)) != NULL) {
		pi_ac->srom_pagc2g = (uint8)PHY_GETINTVAR(pi, rstr_pagc2g);
		pi_ac->srom_pagc2g_ovr = 0x1;
	} else {
		pi_ac->srom_pagc2g = 0xff;
		pi_ac->srom_pagc2g_ovr = 0x0;
	}

	if ((PHY_GETVAR(pi, rstr_pagc5g)) != NULL) {
		pi_ac->srom_pagc5g = (uint8)PHY_GETINTVAR(pi, rstr_pagc5g);
		pi_ac->srom_pagc5g_ovr = 0x1;
	} else {
		pi_ac->srom_pagc5g = 0xff;
		pi_ac->srom_pagc5g_ovr = 0x0;
	}

	if ((PHY_GETVAR(pi, ed_thresh2g)) != NULL) {
		pi_ac->sromi->ed_thresh2g = (int32)PHY_GETINTVAR(pi, ed_thresh2g);
	} else {
		pi_ac->sromi->ed_thresh2g = 0;
	}

	if ((PHY_GETVAR(pi, ed_thresh5g)) != NULL) {
		pi_ac->sromi->ed_thresh5g = (int32)PHY_GETINTVAR(pi, ed_thresh5g);
	} else {
		pi_ac->sromi->ed_thresh5g = 0;
	}

	if ((PHY_GETVAR(pi, rstr_antdiv_rfswctrlpin_a0)) != NULL) {
		pi_ac->antdiv_rfswctrlpin_a0 = (uint8)PHY_GETINTVAR(pi, rstr_antdiv_rfswctrlpin_a0);
	} else {
		pi_ac->antdiv_rfswctrlpin_a0 = (uint8)255;
	}
	if ((PHY_GETVAR(pi, rstr_antdiv_rfswctrlpin_a1)) != NULL) {
		pi_ac->antdiv_rfswctrlpin_a1 = (uint8)PHY_GETINTVAR(pi, rstr_antdiv_rfswctrlpin_a1);
	} else {
		pi_ac->antdiv_rfswctrlpin_a1 = (uint8)255;
	}

	/* Read the offset target power var */
	pi_ac->offset_targetpwr = (uint16)PHY_GETINTVAR_DEFAULT(pi, rstr_offtgpwr, 0);

#if (defined(WLTEST) || defined(WLPKTENG))
	/* Read the per rate dpd enable param */
	pi_ac->perratedpd2g = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_perratedpd2g, 0);
	pi_ac->perratedpd5g = (bool)PHY_GETINTVAR_DEFAULT(pi, rstr_perratedpd5g, 0);
#endif

	return TRUE;
}

static void
BCMATTACHFN(wlc_phy_fptr_attach_acphy)(phy_info_t *pi)
{
	pi->pi_fptr->calinit = wlc_phy_cal_init_acphy;
	pi->pi_fptr->txiqccset = wlc_acphy_set_tx_iqcc;
	pi->pi_fptr->txiqccget = wlc_acphy_get_tx_iqcc;
	pi->pi_fptr->txloccget = wlc_acphy_get_tx_locc;
	pi->pi_fptr->txloccset = wlc_acphy_set_tx_locc;
	pi->pi_fptr->chanset = wlc_phy_chanspec_set_acphy;
	pi->pi_fptr->phywatchdog = wlc_phy_watchdog_acphy;
	pi->pi_fptr->radioloftget = wlc_acphy_get_radio_loft;
	pi->pi_fptr->radioloftset = wlc_acphy_set_radio_loft;
	pi->pi_fptr->phybtcadjust = wlc_phy_btc_adjust_acphy;
	pi->pi_fptr->txcorepwroffsetset = wlc_phy_txpower_core_offset_set_acphy;
	pi->pi_fptr->txcorepwroffsetget = wlc_phy_txpower_core_offset_get_acphy;
#ifdef ENABLE_FCBS
	if (CHIPID(pi->sh->chip) == BCM4360_CHIP_ID) {
		pi->pi_fptr->fcbs = wlc_phy_fcbs_acphy;
		pi->pi_fptr->prefcbs = wlc_phy_prefcbs_acphy;
		pi->pi_fptr->postfcbs = wlc_phy_postfcbs_acphy;
		pi->pi_fptr->fcbsinit = wlc_phy_fcbsinit_acphy;
		pi->pi_fptr->prefcbsinit = wlc_phy_prefcbsinit_acphy;
		pi->pi_fptr->postfcbsinit = wlc_phy_postfcbsinit_acphy;
		pi->HW_FCBS = TRUE;
		pi->FCBS = TRUE;
		if (ACMAJORREV_0(pi->pubpi->phy_rev)) {
			pi->phy_fcbs.FCBS_ucode = TRUE;
		} else {
			pi->phy_fcbs.FCBS_ucode = FALSE;
		}
	}
#else
	pi->HW_FCBS = FALSE;
#endif /* ENABLE_FCBS */
#ifdef WL_LPC
	pi->pi_fptr->lpcsetmode = NULL;
	pi->pi_fptr->lpcgettxcpwrval = NULL;
	pi->pi_fptr->lpcsettxcpwrval = NULL;
	pi->pi_fptr->lpcgetpwros = wlc_acphy_lpc_getoffset;
	pi->pi_fptr->lpcgetminidx = wlc_acphy_lpc_getminidx;
#ifdef WL_LPC_DEBUG
	pi->pi_fptr->lpcgetpwrlevelptr = wlc_acphy_lpc_get_pwrlevelptr;
#endif
#endif /* WL_LPC */
#ifdef ATE_BUILD
	pi->pi_fptr->gpaioconfigptr = wlc_phy_gpaio_acphy;
#endif
	pi->pi_fptr->txswctrlmapsetptr = wlc_txswctrlmap_set_acphy;
	pi->pi_fptr->txswctrlmapgetptr = wlc_txswctrlmap_get_acphy;

#if defined(WLTEST) || defined(BCMDBG)
	pi->pi_fptr->epadpdsetptr = wlc_phy_epa_dpd_set_acphy;
#endif

#if (defined(WLTEST) || defined(WLPKTENG))
	pi->pi_fptr->isperratedpdenptr = wlc_phy_isperratedpden_acphy;
	pi->pi_fptr->perratedpdsetptr = wlc_phy_perratedpdset_acphy;
#endif

}

#if defined(WLTEST) || defined(BCMDBG)
static void
wlc_phy_epa_dpd_set_acphy(phy_info_t *pi, uint8 enab_epa_dpd, bool in_2g_band)
{
	bool turn_papd_on = FALSE;
	bool iovar_in_band;
	uint8 core = 0;

	if (in_2g_band) {
		pi->epacal2g = enab_epa_dpd;
		turn_papd_on = (pi->epacal2g == 1);
	} else {
		pi->epacal5g = enab_epa_dpd;
		turn_papd_on = (pi->epacal5g == 1);
	}
	iovar_in_band = ((in_2g_band &&
		(CHSPEC_IS2G(pi->radio_chanspec))) ||
		(!in_2g_band && (CHSPEC_IS5G(pi->radio_chanspec))));
	if (iovar_in_band) {
		if (!PHY_PAPDEN(pi) && !PHY_IPA(pi) && in_2g_band) {
			if (CHSPEC_IS20(pi->radio_chanspec)) {
				/* WAR for FDIQI when bq_bw = 9, 25 MHz */
				wlc_phy_radio_tiny_lpf_tx_set(pi, 2, 2, 1, 1);
			} else {
				wlc_phy_radio_tiny_lpf_tx_set(pi, 2, 2, 2, 1);
			}
		}
		if (turn_papd_on) {
			wlc_phy_cals_acphy(pi, 0);
		} else {
			MOD_PHYREGCEE(pi, PapdEnable, core, papd_compEnb, 0);
		}
	}
}
#endif /* defined(WLTEST) || defined(BCMDBG) */

static void
wlc_phy_susp2tx_cts2self(phy_info_t *pi, uint16 duration)
{
	int mac_depth = 0;

	while ((mac_depth < 100) && !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC)) {
	  /* Unsuspend mac */
	  wlapi_enable_mac(pi->sh->physhim);
	  mac_depth++;
	}
	ASSERT((R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC) != 0);
	if (duration > 0)
	  wlapi_bmac_write_shm(pi->sh->physhim, M_CTS_DURATION, duration);
	while (mac_depth) {
	  /* Leave the mac in its original state */
	  wlapi_suspend_mac_and_wait(pi->sh->physhim);
	  mac_depth--;
	}
	/* Prepare Mac and Phregs */
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);
	/* Disable Power control */
	wlc_phy_txpwrctrl_enable_acphy(pi, PHY_TPC_HW_OFF);
}

#ifdef WL_PROXDETECT
#undef TOF_TEST_TONE

#ifdef WL_PROXD_SEQ
static void wlc_phy_tof_sc_acphy(phy_info_t *pi, bool setup, int sc_start, int sc_stop, uint16 cfg)
{
	uint16 phy_ctl;
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	phy_ctl = R_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param) & ~((1<<4) | (1<<5));
	W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, phy_ctl);
	WRITE_PHYREG(pi, RxFeTesMmuxCtrl,
	             (0x40 | (pi_ac->tof_core <<
	                      ACPHY_RxFeTesMmuxCtrl_samp_coll_core_sel_SHIFT(0))));
	WRITE_PHYREG(pi, AdcDataCollect, 0);
	if (setup) {
	  acphy_set_sc_startptr(pi, (uint32)sc_start);
	  acphy_set_sc_stopptr(pi, (uint32)sc_stop);
	  if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
	    uint32 pmu_chipctReg5 = si_pmu_chipcontrol(pi->sh->sih,
	                                               PMU_CHIPCTL5, 0, 0) & 0xcfe0ffff;
	    pmu_chipctReg5 |= (0x1036 << 16);
	    si_pmu_chipcontrol(pi->sh->sih, PMU_CHIPCTL5, 0xFFFFFFFF, pmu_chipctReg5);
	    pmu_chipctReg5 |= (1 << 19);
	    si_pmu_chipcontrol(pi->sh->sih, PMU_CHIPCTL5, 0xFFFFFFFF, pmu_chipctReg5);
	  }
	}
	if (cfg) {
	  W_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param, phy_ctl | ((1<<4) | (1<<5)));
	  WRITE_PHYREG(pi, AdcDataCollect, cfg);
	}
}

static int wlc_phy_tof_sc_read_acphy(phy_info_t *pi, bool iq, int n, cint32* pIn,
                                     int16 sc_ptr, int16 sc_base_ptr, int16* p_sc_start_ptr)
{
	d11regs_t *regs = pi->regs;
	uint32 dataL = 0, dataH, data;
	cint32* pEnd = pIn + n;
	int nbits = 0, n_out = 0;
	int32* pOut = (int32*)pIn;
	int16 sc_end_ptr;

	if (sc_ptr <= 0) {
	  /* Offset from sc_base_ptr */
	  sc_ptr = (-sc_ptr >> 2);
	  *p_sc_start_ptr = (sc_ptr << 2);
	  if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev))
	    sc_ptr = 3*sc_ptr;
	  else
	    sc_ptr = (sc_ptr << 2);
	  sc_ptr += sc_base_ptr;
	} else {
	  /* Actual address */
	  if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
	    sc_ptr = (sc_ptr - sc_base_ptr)/3;
	    *p_sc_start_ptr = 4*sc_ptr + sc_base_ptr;
	    sc_ptr = 3*sc_ptr + sc_base_ptr;
	  } else {
	    *p_sc_start_ptr = sc_ptr;
	  }
	}
	sc_end_ptr = R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectCurPtr);
	W_REG(pi->sh->osh, &pi->regs->tplatewrptr, ((uint32)sc_ptr << 2));
	while ((pIn < pEnd) && (sc_ptr < sc_end_ptr)) {
	  if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
	    dataH = dataL;
	    dataL = (uint32)R_REG(pi->sh->osh, &regs->tplatewrdata);
	    nbits += 32;
	    do {
	      nbits -= 12;
	      data = (dataL >> nbits);
	      if (nbits)
		data |= (dataH << (32 - nbits));
	      if (nbits & 4) {
		pIn->q = (int32)(data) & 0xfff;
	      } else {
		pIn->i = (int32)(data) & 0xfff;
		pIn++;
		n_out++;
	      }
	    } while (nbits >= 12);
	  } else {
	    dataL = (uint32)R_REG(pi->sh->osh, &regs->tplatewrdata);
	    pIn->i = (int32)(dataL & 0xfff);
	    pIn->q = (int32)((dataL >> 16) & 0xfff);
	    pIn++;
	    n_out++;
	  }
	  sc_ptr++;
	}
	if (iq) {
	  int32 datum;

	  n = 2*n_out;
	  while (n-- > 0) {
	    datum = *pOut;
	    if (datum > 2047)
	      datum -= 4096;
	    *pOut++ = datum;
	  }
	}
	return n_out;
}

#define k_tof_filt_1_mask   0x1
#define k_tof_filt_neg_mask 0xc
#define k_tof_filt_non_zero_mask 0x3

static int
wlc_tof_rfseq_event_offset(phy_info_t *pi, uint16 event, uint16* rfseq_events)
{
	int i;

	wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 16, (uint32)0, 16, (void*)rfseq_events);

	for (i = 0; i < 16; i++) {
	  if (rfseq_events[i] == event) {
	    break;
	  }
	}
	return i;
}

static void wlc_phy_tof_mf(phy_info_t *pi, int n, cint32* pIn, bool seq, int a,
                           int b, int cfo, int s1, int k2, int s2)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	int i, k;
	cint32 *pTmp;
	int32 tmp;
	int nF = pi_ac->tof_seq_spb_len;
	const uint32* pF = pi_ac->tof_seq_spb;

	pTmp = pIn;
	for (i = 0; i < n; i++) {
	  if (seq) {
	    pTmp->q = 0;
	    pTmp->i = (int32)s1;
	  } else {
	    pTmp->q = (pTmp->q + ((pTmp->i*(int32)a)>>10)) << s1;
	    pTmp->i = (pTmp->i + ((pTmp->i*(int32)b)>>10)) << s1;
	  }
	  pTmp++;
	}

	if (!seq) {
	  if (cfo) {
	  }
	  wlapi_fft(pi->sh->physhim, n, (void*)pIn, (void*)pIn, 2);
	}

	pTmp = pIn;
	for (k = 0; k < 2; k++) {
	  for (i = 0; i < nF; i++) {
	    uint32 f;
	    int j;

	    f = *pF++;
	    for (j = 0; j < 32; j += 4) {
	      if (f & k_tof_filt_non_zero_mask) {
		if (!(f & k_tof_filt_1_mask)) {
		  tmp = pTmp->q;
		  pTmp->q = pTmp->i;
		  pTmp->i = -tmp;
		}
		if (f & k_tof_filt_neg_mask) {
		  pTmp->i = -pTmp->i;
		  pTmp->q = -pTmp->q;
		}
	      } else {
		pTmp->i = 0;
		pTmp->q = 0;
	      }
	      pTmp++;
	      f = f >> 4;
	    }
	  }
	  if (!k) {
	    for (i = 0; i < (n - 2*8*nF); i++) {
	      pTmp->i = 0;
	      pTmp->q = 0;
	      pTmp++;
	    }
	  }
	}

	wlapi_fft(pi->sh->physhim, n, (void*)pIn, (void*)pIn, 2);

	if (s2) {
	  int32 *pTmpIQ, m2 = (int32)(1 << (s2 - 1));
	  pTmpIQ = (int32*)pIn;
	  for (i = 0; i < 2*n; i++) {
	    tmp = ((k2*(*pTmpIQ) + m2) >> s2);
	    *pTmpIQ++ = tmp;
	  }
	}
}

#define k_tof_rfseq_tiny_bundle_base 8
#define k_tof_seq_tiny_rx_fem_gain_offset 0x29
#define k_tof_seq_tiny_tx_fem_gain_offset 0x2b
const uint16 k_tof_seq_tiny_tbls[] = {
	ACPHY_TBL_ID_RFSEQ,
	0x260,
	15,
	0x42, 0x10, 0x8b, 0x9b, 0xaa, 0xb9, 0x8c, 0x9c, 0x9d,
	0xab, 0x8d, 0x9e, 0x43, 0x42, 0x1f,
	ACPHY_TBL_ID_RFSEQ,
	0x290,
	14,
	0x42, 0x10, 0x88, 0x98, 0xa8, 0xb8, 0x89, 0x99, 0xa9,
	0x8a, 0x9a, 0x43, 0x42, 0x1f,
	ACPHY_TBL_ID_RFSEQ,
	0x2f0,
	15,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x01, 0x04, 0x04,
	0x04, 0x01, 0x04, 0x1e, 0x01, 0x01,
	ACPHY_TBL_ID_RFSEQ,
	0x320,
	14,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x01, 0x04, 0x04,
	0x01, 0x04, 0x1e, 0x01, 0x01,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x008,
	18,
	0x0000, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000,
	0x0000, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x018,
	21,
	0x0009, 0x8000, 0x0007, 0xf009, 0x8066, 0x0007, 0xf009, 0x8066, 0x0004,
	0x00c9, 0x8060, 0x0007, 0xf7d9, 0x8066, 0x0007, 0xf7f9, 0x8066, 0x0007,
	0xf739, 0x8066, 0x0004,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x028,
	12,
	0x0009, 0x0000, 0x0000, 0x0229, 0x0000, 0x0000, 0x0019, 0x0000, 0x0000,
	0x0099, 0x0000, 0x0000,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x038,
	6,
	0x0fc9, 0x00a6, 0x0000, 0x0fc9, 0x00a6, 0x0000,
};
#define k_tof_seq_rx_gain_tiny ((0 << 13) | (0 << 10) | (5 << 6) | (0 << 3) | (4 << 0))
#define k_tof_seq_rx_loopback_gain_tiny ((0 << 13) | (0 << 10) | (0 << 6) | (0 << 3) | (0 << 0))

#define k_tof_rfseq_bundle_base 8
#define k_tof_seq_rx_fem_gain_offset 0x29
#define k_tof_seq_tx_fem_gain_offset 0x2b
const uint16 k_tof_seq_tbls[] = {
	ACPHY_TBL_ID_RFSEQ,
	0x260,
	14,
	0x10, 0x8b, 0x9b, 0xaa, 0xb9, 0xc9, 0xd9, 0x8c, 0x9c,
	0x9d, 0xab, 0x8d, 0x9e, 0x1f,
	ACPHY_TBL_ID_RFSEQ,
	0x290,
	13,
	0x10, 0x88, 0x98, 0xa8, 0xb8, 0xc8, 0xd8, 0x89, 0x99,
	0xa9, 0x8a, 0x9a, 0x1f,
	ACPHY_TBL_ID_RFSEQ,
	0x2f0,
	14,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x01, 0x04,
	0x04, 0x04, 0x01, 0x04, 0x01,
	ACPHY_TBL_ID_RFSEQ,
	0x320,
	13,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x01, 0x04,
	0x04, 0x01, 0x04, 0x01,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x008,
	18,
	0x0000, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000,
	0x0000, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x018,
	21,
	0x0009, 0x8000, 0x000f, 0xff09, 0x8066, 0x000f, 0xff09, 0x8066, 0x000c,
	0x00c9, 0x8060, 0x000f, 0xffd9, 0x8066, 0x000f, 0xfff9, 0x8066, 0x000f,
	0xff39, 0x8066, 0x000c,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x028,
	12,
	0x0009, 0x0000, 0x0000, 0x0229, 0x0000, 0x0000, 0x0019, 0x0000, 0x0000,
	0x0099, 0x0000, 0x0000,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x038,
	6,
	0x0fc9, 0x00a6, 0x0000, 0x0fc9, 0x00a6, 0x0000,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x048,
	6,
	0x2099, 0x586e, 0x0000, 0x1519, 0x586e, 0x0000,
	ACPHY_TBL_ID_RFSEQBUNDLE,
	0x058,
	6,
	0x2409, 0xc040, 0x000c, 0x6b89, 0xc040, 0x000c,
};
#define k_tof_seq_rx_gain ((0 << 13) | (2 << 10) | (2 << 6) | (6 << 3) | (4 << 0))
#define k_tof_seq_rx_loopback_gain ((2 << 13) | (4 << 10) | (3 << 6) | (3 << 3) | (3 << 0))

#define k_tof_seq_rfseq_gain_base 0x1d0
#define k_tof_seq_rfseq_rx_gain_offset 7
#define k_tof_seq_rfseq_loopback_gain_offset  4

#define k_tof_seq_shm_offset 4
#define k_tof_seq_shm_setup_regs_offset 2*(0 + k_tof_seq_shm_offset)
#define k_tof_seq_shm_setup_regs_len    15
#define k_tof_seq_shm_setup_vals_offset 2*(15 + k_tof_seq_shm_offset)
#define k_tof_seq_shm_setup_vals_len    16
#define k_tof_set_shm_restr_regs_offset 2*(7 + k_tof_seq_shm_offset)
#define k_tof_set_shm_restr_vals_offset 2*(31 + k_tof_seq_shm_offset)
#define k_tof_set_shm_restr_vals_len    8
#define k_tof_seq_shm_fem_radio_hi_gain_offset 2*(18 + k_tof_seq_shm_offset)
#define k_tof_seq_shm_fem_radio_lo_gain_offset 2*(39 + k_tof_seq_shm_offset)
#define k_tof_seq_shm_dly_offset 2*(40 + k_tof_seq_shm_offset)
#define k_tof_seq_shm_dly_len    (3*2)

#define k_tof_seq_sc_start 1024
#define k_tof_seq_sc_stop  (k_tof_seq_sc_start + 2730)

#define k_tof_rfseq_dc_run_event 0x43
#define k_tof_rfseq_epa_event    0x4
#define k_tof_rfseq_end_event    0x1f
#define k_tof_rfseq_tssi_event   0x35
#define k_tof_rfseq_ipa_event    0x3
#define k_tof_rfseq_tx_gain_event 0x6

const uint16 k_tof_seq_ucode_regs[k_tof_seq_shm_setup_regs_len] = {
	ACPHY_RxControl(0),
	ACPHY_TableID(0),
	ACPHY_TableOffset(0),
	(ACPHY_TableDataWide(0) | (7 << 12)),
	ACPHY_TableID(0),
	ACPHY_TableOffset(0),
	ACPHY_TableDataLo(0),
	ACPHY_TableOffset(0),
	ACPHY_TableDataLo(0),
	ACPHY_RfseqMode(0),
	ACPHY_sampleCmd(0),
	ACPHY_RfseqMode(0),
	ACPHY_SlnaControl(0),
	ACPHY_AdcDataCollect(0),
	ACPHY_RxControl(0),
};

const uint16 k_tof_seq_fem_gains[] = {
	(8 | (1 << 5) | (1 << 9)), /* fem hi */
	(8 | (1 << 5)), /* fem lo */
};

#define k_tof_seq_in_scale (1<<12)
#define k_tof_seq_out_scale 11
#define k_tof_seq_out_shift 8
#define k_tof_mf_in_shift  0
#define k_tof_mf_out_scale 0
#define k_tof_mf_out_shift 0

static void
wlc_tof_seq_write_shm_acphy(phy_info_t *pi, int len, uint16 offset, uint16* p)
{
	uint16 p_shm = pi->u.pi_acphy->tof_shm_ptr;

	while (len-- > 0) {
	  wlapi_bmac_write_shm(pi->sh->physhim, (p_shm + offset), *p);
	  p++;
	  offset += 2;
	}
}


static int
wlc_phy_tof_seq_setup_acphy(phy_info_t *pi, bool enter, bool tx, uint8 core)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
	uint8 stall_val;
	uint16 *pSrc, *pEnd;
	uint16 shm[18];
	uint16 tof_rfseq_bundle_offset, rx_ctrl, rfseq_mode, rfseq_trigger, rfseq_offset, mask;
	uint16 wrds_per_us;
	int i;
	bool  suspend = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
	cint32* pSeq;
	uint16 tof_seq_n;
	uint16 tof_rfseq_event;

	if (!enter)
	  return BCME_OK;

	if (((pi_ac->tof_rfseq_bundle_offset >= k_tof_rfseq_bundle_base) && !TINY_RADIO(pi)) ||
	    ((pi_ac->tof_rfseq_bundle_offset >= k_tof_rfseq_tiny_bundle_base) && TINY_RADIO(pi)) ||
	    (READ_PHYREGFLD(pi, OCLControl1, ocl_mode_enable)))
	  return BCME_ERROR;

	if (wlc_phy_tof_seq_params_acphy(pi) != BCME_OK)
		return BCME_ERROR;

	tof_seq_n = (1 << pi_ac->tof_seq_log2_n);

	WRITE_PHYREG(pi, sampleLoopCount, 0xffff);
	WRITE_PHYREG(pi, sampleDepthCount, (tof_seq_n - 1));
	if (pi_ac->tof_setup_done)
	  return BCME_OK;

	pSeq = (cint32*)MALLOC(pi->sh->osh, tof_seq_n*sizeof(cint32));
	if (!pSeq) {
	  return BCME_ERROR;
	}

	pi_ac->tof_tx = tx;
	pi_ac->tof_core = core;
	mask = (1 << pi_ac->tof_core);

	if (!suspend)
	  wlapi_suspend_mac_and_wait(pi->sh->physhim);
	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	/* Setup rfcontrol sequence for tx_on / tx_off events */
	tof_rfseq_bundle_offset = pi_ac->tof_rfseq_bundle_offset;
	if (TINY_RADIO(pi)) {
	  pSrc = (uint16*)k_tof_seq_tiny_tbls;
	  pEnd = pSrc + sizeof(k_tof_seq_tiny_tbls)/sizeof(uint16);
	} else {
	  pSrc = (uint16*)k_tof_seq_tbls;
	  pEnd = pSrc + sizeof(k_tof_seq_tbls)/sizeof(uint16);
	}
	while (pSrc != pEnd) {
	  uint32 id, width, len, tbl_len, offset;
	  uint16* pTblData;

	  id = (uint32)*pSrc++;
	  offset = (uint32)*pSrc++;
	  len = (uint32)*pSrc++;
	  if (id == ACPHY_TBL_ID_RFSEQBUNDLE) {
	    width = 48;
	    tbl_len = len / 3;
	    for (i = 0; i < len; i++) {
	      shm[i] = pSrc[i];
	      if ((offset >= 0x10) && ((i % 3) == 0)) {
		shm[i] = (shm[i] & ~7) | mask;
	      }
	    }
	    pTblData = shm;
	  } else {
	    width = 16;
	    tbl_len = len;
	    pTblData = pSrc;
	  }
	  wlc_phy_table_write_acphy(pi, id, tbl_len, offset, width, (void*)pTblData);
	  pSrc += len;
	}
	pi_ac->tof_rfseq_bundle_offset = tof_rfseq_bundle_offset;
	/* Set rx gain during loopback -- cant use rf bundles due to hw bug in 4350 */
	if (TINY_RADIO(pi))
	  shm[0] = k_tof_seq_rx_loopback_gain_tiny;
	else
	  shm[0] = k_tof_seq_rx_loopback_gain;
	wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
	                          (k_tof_seq_rfseq_gain_base + core*0x10
	                           + k_tof_seq_rfseq_loopback_gain_offset),
	                           16, (void*)&shm[0]);
	/* Setup shm which tells ucode sequence of phy reg writes */
	/* before/after triggering sequence */
	rx_ctrl = READ_PHYREG(pi, RxControl);
	rfseq_mode = (READ_PHYREG(pi, RfseqMode) &
	              ~(ACPHY_RfseqMode_CoreActv_override_MASK(0) |
	                ACPHY_RfseqMode_Trigger_override_MASK(0)));
	rfseq_offset = wlc_tof_rfseq_event_offset(pi, k_tof_rfseq_tx_gain_event, shm);
	rfseq_offset += 1;
	tof_rfseq_event = shm[rfseq_offset];
	wlc_tof_seq_write_shm_acphy(pi,
	                            k_tof_seq_shm_setup_regs_len,
	                            k_tof_seq_shm_setup_regs_offset,
	                            (uint16*)k_tof_seq_ucode_regs);
	bzero((void*)shm, sizeof(shm));
	shm[0] = rx_ctrl | ACPHY_RxControl_dbgpktprocReset_MASK(0); /* first setup */
	shm[1] = ACPHY_TBL_ID_RFSEQBUNDLE;
	if (TINY_RADIO(pi))
	  shm[2] = k_tof_seq_tiny_rx_fem_gain_offset;
	else
	  shm[2] = k_tof_seq_rx_fem_gain_offset;
	shm[3] = k_tof_seq_fem_gains[0] | mask;
	shm[6] = ACPHY_TBL_ID_RFSEQ; /* first restore */
	shm[7] = k_tof_seq_rfseq_gain_base + core*0x10 + k_tof_seq_rfseq_rx_gain_offset;
#ifdef TOF_DBG
	if (TINY_RADIO(pi))
	  shm[8] = k_tof_seq_rx_gain_tiny;
	else
	  shm[8] = k_tof_seq_rx_gain;
#endif
	shm[9] = rfseq_offset;
	shm[10] = k_tof_rfseq_end_event;
	shm[11] = (rfseq_mode | ACPHY_RfseqMode_CoreActv_override_MASK(0));
	shm[12] = ACPHY_sampleCmd_start_MASK(0);
	shm[13] = (rfseq_mode |
	           ACPHY_RfseqMode_Trigger_override_MASK(0) |
	           ACPHY_RfseqMode_CoreActv_override_MASK(0));
	shm[14] = ((~core) & 3) << ACPHY_SlnaControl_SlnaCore_SHIFT(0);
	shm[15] = ACPHY_AdcDataCollect_adcDataCollectEn_MASK(0); /* last setup */
	wlc_tof_seq_write_shm_acphy(pi,
	                            k_tof_seq_shm_setup_vals_len,
	                            k_tof_seq_shm_setup_vals_offset,
	                            shm);
	shm[10] = tof_rfseq_event;
	shm[11] = rfseq_mode;
	shm[12] = ACPHY_sampleCmd_stop_MASK(0);
	shm[13] = rfseq_mode;
	shm[14] = READ_PHYREG(pi, SlnaControl);
	shm[15] = 0;
	shm[16] = rx_ctrl; /* last restore */
	wlc_tof_seq_write_shm_acphy(pi,
	                            k_tof_set_shm_restr_vals_len,
	                            k_tof_set_shm_restr_vals_offset,
	                            &shm[9]);

	/* Setup delays and triggers */
	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev))
	  wrds_per_us = ((3*pi_ac->tof_sc_FS) >> 2);
	else
	  wrds_per_us = pi_ac->tof_sc_FS;
	pSrc = shm;
	rfseq_trigger = READ_PHYREG(pi, RfseqTrigger) &
	        ACPHY_RfseqTrigger_en_pkt_proc_dcc_ctrl_MASK(0);
	for (i = 0; i < 3; i++) {
	  *pSrc++ = pi_ac->tof_ucode_dlys_us[(tx ? 1 : 0)][i] * wrds_per_us;
	  if (i ^ tx)
	    *pSrc = rfseq_trigger | ACPHY_RfseqTrigger_ocl_shut_off_MASK(0);
	  else
	    *pSrc = rfseq_trigger | ACPHY_RfseqTrigger_ocl_reset2rx_MASK(0);
	  pSrc++;
	}
	shm[k_tof_seq_shm_dly_len-1] = rfseq_trigger; /* Restore value */
	shm[k_tof_seq_shm_dly_len] = ACPHY_PhyStatsGainInfo0(0) + 0x200*core;
	wlc_tof_seq_write_shm_acphy(pi,
	                            (k_tof_seq_shm_dly_len + 1),
	                            k_tof_seq_shm_dly_offset,
	                            shm);
	/* Setup sample play */
	wlc_phy_tof_mf(pi, tof_seq_n, pSeq, TRUE,
	               0, 0, 0,
	               k_tof_seq_in_scale, k_tof_seq_out_scale, k_tof_seq_out_shift);
#ifdef TOF_TEST_TONE
	for (i = 0; i < tof_seq_n; i++) {
	  const int32 v[16] = {96, 89, 68, 37, 0, -37, -68, -89, -96, -89, -68, -37, 0, 37, 68, 89};
	  pSeq[i].i = v[i & 0xf];
	  pSeq[i].q = v[(i-4) & 0xf];
	}
#endif
	wlc_phy_loadsampletable_acphy(pi, pSeq, tof_seq_n, FALSE, TRUE);

#ifdef TOF_DBG
	printf("SETUP CORE %d\n", core);
#endif
	MFREE(pi->sh->osh, pSeq, tof_seq_n*sizeof(cint32));
	ACPHY_ENABLE_STALL(pi, stall_val);
	wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
	if (!suspend)
	  wlapi_enable_mac(pi->sh->physhim);

	pi_ac->tof_setup_done = TRUE;
	return BCME_OK;
}

#if defined(WL_PROXDETECT) || defined(WL_PROXDETECT_OL)
int
wlc_phy_seq_ts_acphy(phy_info_t *pi, int n, cint32* p_buffer, int tx,
                     int cfo, int adj, void* pparams, int32* p_ts, int32* p_seq_len, uint32* p_raw)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
	int16 sc_ptr;
	int32 ts[2], dT;
	int i, n_out, a, b;
	uint16 tof_seq_n;
	int32 tof_seq_M;

	tof_seq_n = (1 <<  pi_ac->tof_seq_log2_n);

	if (n < tof_seq_n)
	  return BCME_ERROR;
	a = READ_PHYREGCE(pi, Core1RxIQCompA, pi_ac->tof_core);
	b = READ_PHYREGCE(pi, Core1RxIQCompB, pi_ac->tof_core);
	if (a > 511)
	  a -= 1024;
	if (b > 511)
	  b -= 1024;

	for (i = 0; i < 2; i++) {
	  n_out = wlc_phy_tof_sc_read_acphy(pi, TRUE, n, p_buffer,
	                                    -(pi_ac->tof_ucode_dlys_us[tx][3+i]),
	                                    k_tof_seq_sc_start, &sc_ptr);
	  if (n_out != n)
	    return BCME_ERROR;
#ifdef TOF_DBG
	  if (p_raw && (2*(n_out + 1) <= k_tof_collect_Hraw_size)) {
	    int j;
	    for (j = 0; j < n_out; j++) {
	      *p_raw++ = (uint32)(p_buffer[j].i & 0xffff) |
	              ((uint32)(p_buffer[j].q & 0xffff) << 16);
	    }
	    *p_raw++ = (uint32)((int32)a & 0xffff) | (uint32)(((int32)b & 0xffff) << 16);
	  }
#endif
#if defined(TOF_TEST_TONE)
	  ts[i] = 0;
#else
	  wlc_phy_tof_mf(pi, tof_seq_n, p_buffer, FALSE,
	                 a, b, (i)?cfo:0,
	                 k_tof_mf_in_shift,
	                 k_tof_mf_out_scale,
	                 k_tof_mf_out_shift);
	  if (wlapi_tof_pdp_ts(pi_ac->tof_seq_log2_n, (void*)p_buffer, pi_ac->tof_sc_FS,
	                       i, pparams, &ts[i], NULL) != BCME_OK)
	    return BCME_ERROR;
#endif
	}

	tof_seq_M = ((10000 * tof_seq_n)/  pi_ac->tof_sc_FS);

	ts[0] += adj;
	dT = (ts[tx] - ts[tx ^ 1]);
	if (dT < 0)
	  dT += tof_seq_M;
	*p_ts = dT;
	*p_seq_len = tof_seq_M;
	return BCME_OK;
}
#endif /* defined (WL_PROXDTECT) || defined (WL_PROXDETECT_OL) */

#if defined(TOF_DBG)
#define k_tof_dbg_sc_delta 32
int
wlc_phy_tof_dbg_acphy(wlc_phy_t *ppi, int arg)
{
	phy_info_t *pi = (phy_info_t*)ppi;
	cint32 buf[k_tof_dbg_sc_delta];
	int16  p, p_start;
	int i = 0, n = 0;
	bool  suspend = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
	uint8 stall_val;

	if (!suspend)
	  wlapi_suspend_mac_and_wait(pi->sh->physhim);
	wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	ACPHY_DISABLE_STALL(pi);

	p = k_tof_seq_sc_start;
	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev))
	  p += ((k_tof_dbg_sc_delta>>2)*arg*3);
	else
	  p += (k_tof_dbg_sc_delta*arg);
	if (arg >= 255) {
	  int j;
	  uint16 v, offset = 0;
	  uint16 bundle[3*16];
	  const uint16 bundle_addr[] = {0x00, 0x10, 0x20, 0x30, 0x40, 0x50};
	  for (i = 0; i < sizeof(bundle_addr)/sizeof(uint16); i++) {
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQBUNDLE,
	                             8, bundle_addr[i]+8, 48, (void*)bundle);
	    for (j = 0; j < 16; j++) {
	      printf("RFBUNDLE 0x%x : 0x%04x%04x%04x\n",
	             (bundle_addr[i]+j),
	             bundle[3*j+2], bundle[3*j+1], bundle[3*j+0]);
	    }
	  }
	  for (i = 0; i < 16; i++) {
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x290 + i), 16, (void*)&bundle[0]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x320 + i), 16, (void*)&bundle[1]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x260 + i), 16, (void*)&bundle[2]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x2f0 + i), 16, (void*)&bundle[3]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x000 + i), 16, (void*)&bundle[4]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x070 + i), 16, (void*)&bundle[5]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x010 + i), 16, (void*)&bundle[6]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1, (0x080 + i), 16, (void*)&bundle[7]);
	    printf("RFSEQ RXs 0x%04x 0x%04x TXs 0x%04x 0x%04x TX 0x%04x 0x%04x RX 0x%04x 0x%04x\n",
	           bundle[0], bundle[1], bundle[2], bundle[3], bundle[4], bundle[5], bundle[6],
	           bundle[7]);
	  }
	  for (i = 0; i < 3; i++) {
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
	                             (k_tof_seq_rfseq_gain_base + i*0x10 +
	                              k_tof_seq_rfseq_loopback_gain_offset), 16, (void*)&bundle[0]);
	    wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, 1,
	                             (k_tof_seq_rfseq_gain_base + i*0x10 +
	                              k_tof_seq_rfseq_rx_gain_offset), 16, (void*)&bundle[1]);
	    printf("GC%d  LBK 0x%04x RX 0x%04x\n", i, bundle[0], bundle[1]);
	  }
	  offset = k_tof_seq_shm_setup_regs_offset;
	  n = 46;
	  while (n-- > 0) {
	    v = wlapi_bmac_read_shm(pi->sh->physhim, (pi->u.pi_acphy->tof_shm_ptr + offset));
	    printf("SHM %d 0x%04x\n", ((offset - k_tof_seq_shm_setup_regs_offset) >> 1), v);
	    offset += 2;
	  }
	  n = 1;
	} else if (arg == 252) {
	  printf("MAC 0x%x STRT %d STP %d CUR %d\n",
	         R_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param),
	         R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectStartPtr),
	         R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectStopPtr),
	         R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectCurPtr));
	  n = 1;
	} else {
	  if (arg == 0) {
	    printf("MAC 0x%x STRT %d STP %d CUR %d ",
	           R_REG(pi->sh->osh, &pi->regs->psm_phy_hdr_param),
	           R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectStartPtr),
	           R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectStopPtr),
	           R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectCurPtr));
	  }
	  if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev))
	    n = (((int)R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectCurPtr) - p)/3) << 2;
	  else
	    n = ((int)R_REG(pi->sh->osh, &pi->regs->PHYREF_SampleCollectCurPtr) - p);
	  if (n > k_tof_dbg_sc_delta)
	    n = k_tof_dbg_sc_delta;
	  if (n > 0) {
	    arg = wlc_phy_tof_sc_read_acphy(pi, TRUE, n, buf, p, k_tof_seq_sc_start, &p_start);
	    i = 0;
	    while (i < arg) {
	      if (buf[i].i > 2047)
		buf[i].i -= 4096;
	      if (buf[i].q > 2047)
		buf[i].q -= 4096;
	      printf("SD %4d %d %d\n", p_start, buf[i].i, buf[i].q);
	      i++;
	      p_start++;
	    }
	  } else {
	    printf("\n");
	  }
	}

	ACPHY_ENABLE_STALL(pi, stall_val);
	wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
	if (!suspend)
	  wlapi_enable_mac(pi->sh->physhim);

	return (n > 0) ? 1 : 0;
}
#endif /* defined(TOF_DBG) */

#endif /* WL_PROXD_SEQ */

#if defined(WL_PROXDETECT)
int wlc_phy_tof_acphy(phy_info_t *pi, bool enter, bool tx, bool hw_adj, bool seq_en, int core)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
	bool change = pi_ac->tof_active != enter;
	bool  suspend = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
	int retval = BCME_OK;

	if (change) {
#ifdef WL_PROXD_SEQ
	  if (seq_en) {
	    retval = wlc_phy_tof_seq_setup_acphy(pi, enter, tx, core);

	  } else
#endif
	  {
		  pi_ac->tof_setup_done = FALSE;
	    if (!suspend)
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
	    if (enter) {
		/* Save state fdiqcomp and disable */
		pi_ac->tof_rx_fdiqcomp_enable = (uint8)READ_PHYREGFLD(pi, rxfdiqImbCompCtrl,
			rxfdiqImbCompEnable);
		pi_ac->tof_tx_fdiqcomp_enable = (uint8)READ_PHYREGFLD(pi, fdiqImbCompEnable,
			txfdiqImbCompEnable);
		MOD_PHYREG(pi, rxfdiqImbCompCtrl, rxfdiqImbCompEnable, 0);
		MOD_PHYREG(pi, fdiqImbCompEnable, txfdiqImbCompEnable, 0);
		if (hw_adj) {
			/* Save channel smoothing state and enable special  mode */
			pi_ac->tof_smth_enable = pi_ac->acphy_enable_smth;
			pi_ac->tof_smth_dump_mode = pi_ac->acphy_smth_dump_mode;
			wlc_phy_smth(pi, SMTH_ENABLE, SMTH_TIMEDUMP_AFTER_IFFT);
			pi_ac->tof_smth_forced = TRUE;
		}
	    } else {
		/* Restore fdiqcomp state */
		MOD_PHYREG(pi, rxfdiqImbCompCtrl, rxfdiqImbCompEnable,
			pi_ac->tof_rx_fdiqcomp_enable);
		MOD_PHYREG(pi, fdiqImbCompEnable, txfdiqImbCompEnable,
			pi_ac->tof_tx_fdiqcomp_enable);
		if (pi_ac->tof_smth_forced) {
			/* Restore channel smoothing state */
			pi_ac->tof_smth_forced = FALSE;
			wlc_phy_smth(pi, pi_ac->tof_smth_enable, pi_ac->tof_smth_dump_mode);
		}
	}

		wlc_phy_resetcca_acphy(pi);
		if (!suspend)
			wlapi_enable_mac(pi->sh->physhim);
	}
}
	pi_ac->tof_active = enter;
	return retval;
}
#endif /* defined (WL_PROXDTECT) || defined (WL_PROXDETECT_OL) */

/* Unpacks floating point to fixed point for further processing */
/* Fixed point format:

	A.fmt = TRUE
			sign      real         sign       image             exp
			|-|--------------------||-|--------------------||----------|
	size:            nman                   nman              nexp

	B.fmt = FALSE
	         exp     sign		 image                  real
			|----------||-|-||-------------------||--------------------|
	size:		 nexp	 1 1          nman					nman

	When Hi is NULL, we return "real * real + image * image" in Hr array, otherwise
	real and image is save in Hr and Hi.

	When autoscale is TRUE, calculate the max shift to save fixed point value into uint32
*/


/*
	H holds upacked 32 bit data when the function is called.
	H and Hout could partially overlap.
	H and h can not overlap
*/

#define k_tof_unpack_sgn_mask (1<<31)
static int
wlc_unpack_float_acphy(int nbits, int autoscale, int shft,
	int fmt, int nman, int nexp, int nfft, uint32* H, cint32* Hout, int32* h)
{
	int e_p, maxbit, e, i, pwr_shft = 0, e_zero, sgn;
	int n_out, e_shift;
	int8 He[256];
	int32 vi, vq, *pOut;
	uint32 x, iq_mask, e_mask, sgnr_mask, sgni_mask;

	/* when fmt is TRUE, the size nman include its sign bit */
	/* so need to minus one to get value mask */
	if (fmt)
		iq_mask = (1<<(nman-1))- 1;
	else
		iq_mask = (1<<nman)- 1;

	e_mask = (1<<nexp)-1;	/* exp part mask */
	e_p = (1<<(nexp-1));	/* max abs value of exp */

	if (h) {
		/* Set pwr_shft to make sure that square sum can be hold by uint32 */
		pwr_shft = (2*nman + 1 - 31);
		if (pwr_shft < 0)
			pwr_shft = 0;
		pwr_shft = (pwr_shft + 1)>>1;
		sgnr_mask = 0;	/* don't care sign for square sum */
		sgni_mask = 0;
		e_zero = -(2*(nman - pwr_shft) + 1);
		pOut = (int32*)h;
		n_out = nfft;
		e_shift = 0;
	} else {
		/* Set the location of sign bit */
		if (fmt) {
			sgnr_mask = (1 << (nexp + 2*nman - 1));
			sgni_mask = (sgnr_mask >> nman);
		} else {
			sgnr_mask = (1 << 2*nman);
			sgni_mask = (sgnr_mask << 1);
		}
		e_zero = -nman;
		pOut = (int32*)Hout;
		n_out = (nfft << 1);
		e_shift = 1;
	}

	maxbit = -e_p;
	for (i = 0; i < nfft; i++) {
		/* get the real, image and exponent value */
		if (fmt) {
			vi = (int32)((H[i] >> (nexp + nman)) & iq_mask);
			vq = (int32)((H[i] >> nexp) & iq_mask);
			e =   (int)(H[i] & e_mask);
		} else {
			vi = (int32)(H[i] & iq_mask);
			vq = (int32)((H[i] >> nman) & iq_mask);
			e = (int32)((H[i] >> (2*nman + 2)) & e_mask);
		}

		/* adjust exponent */
		if (e >= e_p)
			e -= (e_p << 1);

		if (h) {
			/* calculate square sum of real and image data */
			vi = (vi >> pwr_shft);
			vq = (vq >> pwr_shft);
			h[i] = vi*vi + vq*vq;
			vq = 0;
			e = 2*(e + pwr_shft);
		}

		He[i] = (int8)e;

		/* auto scale need to find the maximus exp bits */
		x = (uint32)vi | (uint32)vq;
		if (autoscale && x) {
			uint32 m = 0xffff0000, b = 0xffff;
			int s = 16;

			while (s > 0) {
				if (x & m) {
					e += s;
					x >>= s;
				}
				s >>= 1;
				m = (m >> s) & b;
				b >>= s;
			}
			if (e > maxbit)
				maxbit = e;
		}

		if (!h) {
			if (H[i] & sgnr_mask)
				vi |= k_tof_unpack_sgn_mask;
			if (H[i] & sgni_mask)
				vq |= k_tof_unpack_sgn_mask;
			Hout[i].i = vi;
			Hout[i].q = vq;
		}
	}

	/* shift bits */
	if (autoscale)
		shft = nbits - maxbit;

	/* scal and sign */
	for (i = 0; i < n_out; i++) {
		e = He[(i >> e_shift)] + shft;
		vi = *pOut;
		sgn = 1;
		if (!h && (vi & k_tof_unpack_sgn_mask)) {
			sgn = -1;
			vi &= ~k_tof_unpack_sgn_mask;
		}
		/* trap the zero case */
		if (e < e_zero) {
			vi = 0;
		} else if (e < 0) {
			e = -e;
			vi = (vi >> e);
		} else {
			vi = (vi << e);
		}
		*pOut++ = (int32)sgn*vi;
	}

	return shft;
}

#if defined(WL_PROXDETECT) || defined(WL_PROXDETECT_OL)
/* Get channel frequency response for deriving 11v rx timestamp */
int
wlc_phy_chan_freq_response_acphy(phy_info_t *pi,
	int len, int nbits, cint32* H, uint32* Hraw)
{
	uint32 *pTmp, *pIn;
	int i, i_l, i_r, n1, n2, n3, nfft, nfft_over_2;


	if ((len != TOF_NFFT_20MHZ) && (len != TOF_NFFT_40MHZ) && (len != TOF_NFFT_80MHZ))
		return BCME_ERROR;

	ASSERT(sizeof(cint32) == 2*sizeof(int32));

	pTmp = (uint32*)H;
	pIn = (uint32*)H + len;
	wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_CORE0CHANESTTBL, len, 0,
		CORE0CHANESTTBL_TABLE_WIDTH, pTmp);
#ifdef TOF_DBG
#if defined(k_tof_collect_Hraw_size)
	if (Hraw && (len <= k_tof_collect_Hraw_size))
	  bcopy((void*)pTmp, (void*)Hraw, len*sizeof(uint32));
#else
	if (Hraw) {
		bcopy((void*)pTmp, (void*)Hraw, len*sizeof(uint32));
	}

#endif
/* store raw data for log collection */
#endif /* TOF_DBG */
	/* reorder tones */
	nfft = len;
	nfft_over_2 = (len >> 1);
	if (CHSPEC_IS80(pi->radio_chanspec)) {
		i_l = 122;
		i_r = 2;
	} else if (CHSPEC_IS40(pi->radio_chanspec)) {
		i_l = 58;
		i_r = 2;
	} else {
		/* for legacy this is 26, for vht-20 this is 28 */
		i_l = 28;
		i_r = 1;
	}
	memset((void *)pIn, 0, len * sizeof(int32));
	for (i = i_l; i >= i_r; i--) {
		n1 = nfft_over_2 - i;
		n2 = nfft_over_2 + i;
		n3 = nfft - i;
		pIn[n1] = pTmp[n3];
		pIn[n2] = pTmp[i];
	}

	if (ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_3(pi->pubpi->phy_rev)) {
		int32 chi, chq;

		for (i = 0; i < len; i++) {
			chi = ((int32)pIn[i] >> CORE0CHANESTTBL_INTEGER_DATA_SIZE) &
				CORE0CHANESTTBL_INTEGER_DATA_MASK;
			chq = (int32)pIn[i] & CORE0CHANESTTBL_INTEGER_DATA_MASK;
			if (chi >= CORE0CHANESTTBL_INTEGER_MAXVALUE)
				chi = chi - (CORE0CHANESTTBL_INTEGER_MAXVALUE << 1);
			if (chq >= CORE0CHANESTTBL_INTEGER_MAXVALUE)
				chq = chq - (CORE0CHANESTTBL_INTEGER_MAXVALUE << 1);
			H[i].i = chi;
			H[i].q = chq;
		}
	} else if (ACMAJORREV_2(pi->pubpi->phy_rev) || ACMAJORREV_5(pi->pubpi->phy_rev)) {
		wlc_unpack_float_acphy(nbits, UNPACK_FLOAT_AUTO_SCALE, 0,
			CORE0CHANESTTBL_FLOAT_FORMAT, CORE0CHANESTTBL_REV2_DATA_SIZE,
			CORE0CHANESTTBL_REV2_EXP_SIZE, len, pIn, H, NULL);
	} else if (ACMAJORREV_0(pi->pubpi->phy_rev)) {
		wlc_unpack_float_acphy(nbits, UNPACK_FLOAT_AUTO_SCALE, 0,
			CORE0CHANESTTBL_FLOAT_FORMAT, CORE0CHANESTTBL_REV0_DATA_SIZE,
			CORE0CHANESTTBL_REV0_EXP_SIZE, len, pIn, H, NULL);
	} else {
		return BCME_UNSUPPORTED;
	}

	return BCME_OK;
}

/* Get mag sqrd channel impulse response(from channel smoothing hw) to derive 11v rx timestamp */
int
wlc_phy_chan_mag_sqr_impulse_response_acphy(phy_info_t *pi, int frame_type,
	int len, int offset, int nbits, int32* h, int* pgd, uint32* hraw, uint16 tof_shm_ptr)
{
	uint8 stall_val;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;
	int N, s0, s1, m_mask, idx, i, j, l, m, i32x4, i4x2, tbl_offset, gd, n;
	uint32 *pdata, *ptmp, data_l, data_h;
	uint16 chnsm_status0;

	idx = -offset;

	if (ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) {
		/* Only 11n/acphy frames */
		if (frame_type < 2)
			return BCME_UNSUPPORTED;
	} else if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		/* 11n/acphy frames + 20MHz legacy frames */
		if ((frame_type < 2) && !((frame_type == 1) && (pi_ac->curr_bw == 0)))
			return BCME_UNSUPPORTED;
	} else {
		return BCME_UNSUPPORTED;
	}

	if (CHSPEC_IS80(pi->radio_chanspec)) {
		N = TOF_NFFT_80MHZ;
		s0 = 2 + TOF_BW_80MHZ_INDEX;
	} else if (CHSPEC_IS40(pi->radio_chanspec)) {
		N = TOF_NFFT_40MHZ;
		s0 = 2 + TOF_BW_40MHZ_INDEX;
	} else if (CHSPEC_IS20(pi->radio_chanspec)) {
		N = TOF_NFFT_20MHZ;
		s0 = 2 + TOF_BW_20MHZ_INDEX;
	} else
		return BCME_ERROR;

	chnsm_status0 = wlapi_bmac_read_shm(pi->sh->physhim, tof_shm_ptr + M_TOF_CHNSM_0);
	gd = (int)((chnsm_status0 >>
		ACPHY_chnsmStatus0_group_delay_SHIFT(pi->pubpi->phy_rev)) & 0xff);
	if (gd > 127)
		gd -= 256;

	if (ACMAJORREV_3(pi->pubpi->phy_rev))
		idx += gd; /* Impulse response is not shited, 0 is @ gd */

	*pgd = Q1_NS * gd; /* gd in Q1 ns */

	if (len > N)
		len = N;

	s1 = s0 + 2;
	m_mask = (1 << s1) - 1;

	stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);
	if (stall_val == 0)
		ACPHY_DISABLE_STALL(pi);

	phy_utils_write_phyreg(pi, ACPHY_TableID(pi->pubpi->phy_rev),
		(uint16)ACPHY_TBL_ID_CHANNELSMOOTHING_1x1);

	n = len;
	ptmp = (uint32*)h + len;
	pdata = ptmp;
	while (n > 0) {
		idx = idx & (N - 1);
		i = (idx >> 2);
		j = (idx & 3);
		l = i + j;
		i32x4 = (i & 0xc);
		if (N == TOF_NFFT_80MHZ) {
			m  = (i32x4 + (i << 2) + (i32x4 >> 2)) & 0xf;
			m += (i ^ ((i << 1) & 0x20)) & 0x30;
			l += (i >> 4) + (i >> 2);
		} else if (N == TOF_NFFT_40MHZ) {
			i4x2 = (i >> 3) & 2;
			m  = ((0x1320 >> i32x4) & 0xf) << 3;
			m += ((((i >> 2) & 1) + i + i4x2) & 3) << 1;
			m += (i4x2 >> 1);
			l += (0x130 >> i32x4) + i4x2;
		} else {
			m  = ((i & 3) + (i32x4 ^ (i32x4 << 1))) & 0xf;
			l += (i >> 2);
		}
		l = (l & 3);
		tbl_offset = ((m + (l << s0)) & m_mask) + (l << s1) + CHANNELSMOOTHING_DATA_OFFSET;
		phy_utils_write_phyreg(pi, ACPHY_TableOffset(pi->pubpi->phy_rev),
			(uint16) tbl_offset);
		data_l = (uint32)phy_utils_read_phyreg(pi, ACPHY_TableDataWide(pi->pubpi->phy_rev));
		data_h = (uint32)phy_utils_read_phyreg_wide(pi);
		*pdata++ = (data_h << 16) | (data_l & 0xffff);
		idx++;
		n--;
	};

	if (stall_val == 0)
		ACPHY_ENABLE_STALL(pi, stall_val);
#ifdef TOF_DBG
#if defined(k_tof_collect_Hraw_size)
	if (hraw && ((len + 1) <= k_tof_collect_Hraw_size)) {
		bcopy((void*)ptmp, (void*)hraw, len*sizeof(uint32));
		/* store raw data for log collection */
		hraw[len] = (uint32)chnsm_status0;
	}
#else
	if (hraw) {
		bcopy((void*)ptmp, (void*)hraw, len*sizeof(uint32));
		hraw[len] = (uint32)chnsm_status0;
	}
#endif
#endif /* TOF_DBG */
	wlc_unpack_float_acphy(nbits, UNPACK_FLOAT_AUTO_SCALE, 0, CHANNELSMOOTHING_FLOAT_FORMAT,
		CHANNELSMOOTHING_FLOAT_DATA_SIZE, CHANNELSMOOTHING_FLOAT_EXP_SIZE,
		len, ptmp, NULL, h);

	return BCME_OK;
}

/* get rxed frame type, bandwidth and rssi value */
int
wlc_phy_tof_info_acphy(phy_info_t *pi, int* p_frame_type, int* p_frame_bw, int* p_cfo, int8* p_rssi)
{
	uint16 status0, status1, status2, status5;
	int frame_type, frame_bw, rssi, cfo;

	status0 = (int)READ_PHYREG(pi, RxStatus0) & 0xffff;
	status1 = (int)READ_PHYREG(pi, RxStatus1);
	status2 = (int)READ_PHYREG(pi, RxStatus2);
	status5 = (int)READ_PHYREG(pi, RxStatus5);

	frame_type = status0 & PRXS0_FT_MASK;
	if (ACMAJORREV_GE32(pi->pubpi.phy_rev)) {
		frame_bw = (status1 & PRXS1_ACPHY2_SUBBAND_MASK) >> PRXS1_ACPHY2_SUBBAND_SHIFT;
	} else {
		frame_bw = (status0 & PRXS0_ACPHY_SUBBAND_MASK) >> PRXS0_ACPHY_SUBBAND_SHIFT;
	}
	if (frame_bw == PRXS_SUBBAND_80)
	  frame_bw = TOF_BW_80MHZ_INDEX | (frame_bw << 16);
	else if ((frame_bw == PRXS_SUBBAND_40L) || (frame_bw == PRXS_SUBBAND_40U))
	  frame_bw = TOF_BW_40MHZ_INDEX | (frame_bw << 16);
	else
	  frame_bw = TOF_BW_20MHZ_INDEX | (frame_bw << 16);
	rssi = ((int)status2 >> 8) & 0xff;
	if (rssi > 127)
		rssi -= 256;

	cfo = ((int)status5 & 0xff);
	if (cfo > 127)
		cfo -= 256;
	cfo = cfo * 2298;

	*p_frame_type = frame_type;
	*p_frame_bw = frame_bw;
	*p_rssi = rssi;
	*p_cfo = cfo;

	return BCME_OK;
}

/* turn on classification to receive frames */
void
wlc_phy_tof_cmd_acphy(phy_info_t *pi, bool seq)
{
	uint16 class_mask;
#ifdef WL_PROXD_SEQ
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	if (seq) {
		uint16 tof_seq_fem_gains[2],  mask = (1 << pi_ac->tof_core);

		MOD_PHYREG(pi, RfseqCoreActv2059, EnTx, mask);
		tof_seq_fem_gains[0] = k_tof_seq_fem_gains[0] | mask;
		tof_seq_fem_gains[1] = k_tof_seq_fem_gains[1] | mask;
		wlc_tof_seq_write_shm_acphy(pi, 1, k_tof_seq_shm_fem_radio_hi_gain_offset,
			(uint16*)tof_seq_fem_gains);
		wlc_tof_seq_write_shm_acphy(pi, 1, k_tof_seq_shm_fem_radio_lo_gain_offset,
			(uint16*)tof_seq_fem_gains+1);
		wlc_phy_tof_sc_acphy(pi, TRUE, k_tof_seq_sc_start, k_tof_seq_sc_stop, 0);
	}
#endif /* WL_PROXD_SEQ */

	class_mask = CHSPEC_IS2G(pi->radio_chanspec) ? 7 : 6; /* No bphy in 5g */
	wlc_phy_classifier_acphy(pi, ACPHY_ClassifierCtrl_classifierSel_MASK, class_mask);
}

/* get K value for initiator or target  */
#define k_tof_k_rtt_adj_Q 2
int
wlc_phy_tof_kvalue_acphy(phy_info_t *pi, chanspec_t chanspec, ratespec_t rspec,
	uint32 *kip, uint32 *ktp, uint8 flag)
{
	uint16 const *kvaluep = NULL;
	int idx = 0, channel = CHSPEC_CHANNEL(chanspec);
	int32 ki = 0, kt = 0, kseq = 0;
	int rtt_adj = 0, rtt_adj_ts, rate_adj = 0;
	int rtt_adj_papd = 0, papd_en = 0;
	uint8 bwidx;

	if (ACMAJORREV_3(pi->pubpi->phy_rev)) {
		/* For 4345 B0/B1 */
		if (CHSPEC_IS80(chanspec)) {
			kvaluep = proxd_4345_80m_k_values;
			ki = TOF_INITIATOR_K_4345_80M;
			kt = TOF_TARGET_K_4345_80M;
		}
		else if (CHSPEC_IS40(chanspec)) {
			kvaluep = proxd_4345_40m_k_values;
			ki = TOF_INITIATOR_K_4345_40M;
			kt = TOF_TARGET_K_4345_40M;
		}
		else if (channel >= 36) {
			kvaluep = proxd_4345_20m_k_values;
			ki = TOF_INITIATOR_K_4345_20M;
			kt = TOF_TARGET_K_4345_20M;
		}
		else {
			kvaluep = proxd_4345_2g_k_values;
			ki = TOF_INITIATOR_K_4345_2G;
			kt = TOF_TARGET_K_4345_2G;
		}
	} else if (ACMAJORREV_2(pi->pubpi->phy_rev) && (ACMINORREV_1(pi) || ACMINORREV_4(pi))) {
		/* For 4350 C0/C1/C2 */
		if (CHSPEC_IS80(chanspec)) {
			kvaluep = proxd_4350_80m_k_values;
			ki = TOF_INITIATOR_K_4350_80M;
			kt = TOF_TARGET_K_4350_80M;
			kseq = 90;
		}
		else if (CHSPEC_IS40(chanspec)) {
			kvaluep = proxd_4350_40m_k_values;
			ki = TOF_INITIATOR_K_4350_40M;
			kt = TOF_TARGET_K_4350_40M;
		}
		else if (channel >= 36) {
			kvaluep = proxd_4350_20m_k_values;
			ki = TOF_INITIATOR_K_4350_20M;
			kt = TOF_TARGET_K_4350_20M;
		}
		else {
			kvaluep = proxd_4350_2g_k_values;
			ki = TOF_INITIATOR_K_4350_2G;
			kt = TOF_TARGET_K_4350_2G;
		}
	} else if (ACMAJORREV_2(pi->pubpi->phy_rev) && (ACMINORREV_3(pi) || ACMINORREV_5(pi))) {
		/* For 4354A1/4345A2(4356) */
		if (CHSPEC_IS80(chanspec)) {
			kvaluep = proxd_4354_80m_k_values;
			ki = TOF_INITIATOR_K_4354_80M;
			kt = TOF_TARGET_K_4354_80M;
			kseq = 90;
		}
		else if (CHSPEC_IS40(chanspec)) {
			kvaluep = proxd_4354_40m_k_values;
			ki = TOF_INITIATOR_K_4354_40M;
			kt = TOF_TARGET_K_4354_40M;
		}
		else if (channel >= 36) {
			kvaluep = proxd_4354_20m_k_values;
			ki = TOF_INITIATOR_K_4354_20M;
			kt = TOF_TARGET_K_4354_20M;
		}
		else {
			kvaluep = proxd_4354_2g_k_values;
			ki = TOF_INITIATOR_K_4354_2G;
			kt = TOF_TARGET_K_4354_2G;
		}
	} else if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		/* For 4349/4355/4359 */
		if (CHSPEC_IS80(chanspec)) {
			kvaluep = proxd_4349_80m_k_values;
			ki = TOF_INITIATOR_K_4349_80M;
			kt = TOF_TARGET_K_4349_80M;
		}
		else if (CHSPEC_IS40(chanspec)) {
			kvaluep = proxd_4349_40m_k_values;
			ki = TOF_INITIATOR_K_4349_40M;
			kt = TOF_TARGET_K_4349_40M;
		}
		else if (channel >= 36) {
			kvaluep = proxd_4349_20m_k_values;
			ki = TOF_INITIATOR_K_4349_20M;
			kt = TOF_TARGET_K_4349_20M;
		}
		else {
			kvaluep = proxd_4349_2g_k_values;
			ki = TOF_INITIATOR_K_4349_2G;
			kt = TOF_TARGET_K_4349_2G;
		}
	}

	if (flag & WL_PROXD_SEQEN) {
		if (kip)
			*kip = kseq;
		if (ktp)
			*ktp = kseq;
		return BCME_OK;
	}
	bwidx = flag & WL_PROXD_BW_MASK;

	if (kvaluep) {
		int8 rateidx = -1; /* VHT = -1, legacy6M = 0, legacy = 1, mcs0 = 2, mcs = 3 */
		if (RSPEC_ISLEGACY(rspec)) {
			if (RSPEC2RATE(rspec) == WLC_RATE_6M)
				rateidx = 0;
			else
				rateidx = 1;
		} else if (RSPEC_ISHT(rspec)) {
			if (wlc_ratespec_mcs(rspec))
				rateidx = 3;
			else
				rateidx = 2;
		}
		rtt_adj = (4 - READ_PHYREGFLD(pi, RxFeCtrl1, rxfe_bilge_cnt));
		rtt_adj_ts = 80;
		if (READ_PHYREGFLD(pi, PapdEnable0, papd_compEnb0))
			papd_en = 1;
		if (CHSPEC_IS80(chanspec)) {
			if (channel <= 58)
				idx = (channel - 42) >> 4;
			else if (channel <= 138)
				idx = ((channel - 106) >> 4) + 2;
			else
				idx = 5;
			rtt_adj_ts = 25;
			if (papd_en)
				rtt_adj_papd = 25;
			if (rateidx != -1)
				rate_adj = proxd_rate_offset_80m[rateidx];
		} else if (CHSPEC_IS40(chanspec)) {
			if (channel <= 62)
				idx = (channel - 38) >> 3;
			else if (channel <= 142)
				idx = ((channel - 102) >> 3) + 4;
			else
				idx = ((channel - 151) >> 3) + 10;
			rtt_adj_ts = 40;
			if (papd_en)
				rtt_adj_papd = 30;
			if (rateidx != -1)
				rate_adj = proxd_rate_offset_40m[rateidx];
		} else if (channel >= 36) {
			/* 5G 20M Hz channels */
			if (channel <= 64)
				idx = (channel - 36) >> 2;
			else if (channel <= 144)
				idx = ((channel - 100) >> 2) + 8;
			else
				idx = ((channel - 149) >> 2) + 20;
			if (papd_en)
				rtt_adj_papd = 66;
			if (rateidx != -1)
				rate_adj = proxd_rate_offset_20m[rateidx];
		} else if (channel >= 1 && channel <= 14) {
			/* 2G channels */
			idx = channel - 1;
			if (papd_en)
				rtt_adj_papd = 70;
			if (rateidx != -1)
				rate_adj = proxd_rate_offset_2g[rateidx];
		}
		rtt_adj = (rtt_adj_ts * rtt_adj) >> k_tof_k_rtt_adj_Q;
		ki += ((int32)rtt_adj + (int32)rtt_adj_papd - rate_adj);
		kt += ((int32)rtt_adj + (int32)rtt_adj_papd - rate_adj);
		if (bwidx) {
			kt -= proxd_subbw_offset[bwidx-1][rateidx+1];
		}
		if (kip)
		{
			*kip = (uint32)(ki + (int8)(kvaluep[idx] & 0xff));
		}
		if (ktp)
		{
			*ktp = (uint32)(kt + (int8)(kvaluep[idx] >> 8));
		}
		return BCME_OK;
	}

	return BCME_ERROR;
}
#endif /* defined (WL_PROXDTECT) || defined (WL_PROXDETECT_OL) */

#define tof_w(x, n, log2_n) (x - ((1<<log2_n)/2) + (n*(1<<log2_n)))

#ifdef TOF_SEQ_20MHz_BW

#define k_tof_seq_log2_n_20MHz 7

/* 20MHz case */
const uint16 k_tof_ucode_dlys_us_20MHz[2][5] = {
	{1, 8, 8, tof_w(200, 2, k_tof_seq_log2_n_20MHz), tof_w(
			200,0, k_tof_seq_log2_n_20MHz)}, /* RX -> TX */
	{2, 6, 8, tof_w(480, -2, k_tof_seq_log2_n_20MHz), tof_w(
			480, 0, k_tof_seq_log2_n_20MHz)}, /* TX -> RX */
};

#endif

#ifdef TOF_SEQ_40MHz_BW

#define k_tof_seq_log2_n_40MHz 8

/* For 20 in 40MHz */
const uint16 k_tof_ucode_dlys_us_40MHz[2][5] = {
	{1, 8, 8, tof_w(338, 3, k_tof_seq_log2_n_40MHz), tof_w(
			338, 0, k_tof_seq_log2_n_40MHz)},  /* RX -> TX */
	{2, 6, 8, tof_w(1188, -3, k_tof_seq_log2_n_40MHz), tof_w(
			1188, 0, k_tof_seq_log2_n_40MHz)}, /* TX -> RX */
};

#endif

#ifdef TOF_SEQ_40_IN_40MHz

#define k_tof_seq_log2_n_40MHz 8

/* For 40 in 40MHz */
const uint16 k_tof_ucode_dlys_us_40MHz_40[2][5] = {
	{1, 8, 8, tof_w(338, 3, k_tof_seq_log2_n_40MHz), tof_w(
			338, 0, k_tof_seq_log2_n_40MHz)},  /* RX -> TX */
	{2, 6, 8, tof_w(1188, -3, k_tof_seq_log2_n_40MHz), tof_w(
			1188, 0, k_tof_seq_log2_n_40MHz)}, /* TX -> RX */
};

#endif

#ifdef TOF_SEQ_20_IN_80MHz

#define k_tof_seq_log2_n_80MHz_20 9

/* For 20 in 80MHz */
const uint16 k_tof_ucode_dlys_us_80MHz_20[2][5] = {
	{1, 8, 8, tof_w(698, 3, k_tof_seq_log2_n_80MHz_20), tof_w(
			698, 0, k_tof_seq_log2_n_80MHz_20)}, /* RX -> TX */
	{2, 6, 8, tof_w(2368, -3, k_tof_seq_log2_n_80MHz_20), tof_w(
			2368, 0, k_tof_seq_log2_n_80MHz_20)}, /* TX -> RX */
};

#endif

#if !defined(TOF_SEQ_20_IN_80MHz)

#define k_tof_seq_log2_n_80MHz 8

/* Original 80MHz case */
const uint16 k_tof_ucode_dlys_us_80MHz[2][5] = {
	{1, 6, 6, tof_w(750, 4, k_tof_seq_log2_n_80MHz), tof_w(
			750, 0, k_tof_seq_log2_n_80MHz)}, /* RX -> TX */
	{2, 6, 6, tof_w(1850, -4, k_tof_seq_log2_n_80MHz), tof_w(
			1850, 0, k_tof_seq_log2_n_80MHz)}, /* TX -> RX */
};

#endif


#if (defined(TOF_SEQ_20_IN_80MHz) || defined(TOF_SEQ_20MHz_BW) || \
	defined(TOF_SEQ_40MHz_BW))

#define k_tof_seq_spb_len_20MHz 4
const uint32 k_tof_seq_spb_20MHz[2*k_tof_seq_spb_len_20MHz] = {
	0x1f11ff10,
	0x1fffff1f,
	0x1f1f1ff1,
	0x000ff111,
	0x11110000,
	0x1f1f11ff,
	0x1ff11111,
	0x1111f1f1,
};

#endif

#ifdef TOF_SEQ_40_IN_40MHz

#define k_tof_seq_spb_len_40MHz 8
const uint32 k_tof_seq_spb_40MHz[2*k_tof_seq_spb_len_40MHz] = {
	0xf11ff110,
	0x111111f1,
	0x1f1f11ff,
	0x1ff11111,
	0xfff1f1f1,
	0xf1ff11ff,
	0xff1111f1,
	0x0000001f,
	0xf0000000,
	0x1ff11f11,
	0x1111f1f1,
	0x1f11ff11,
	0xf111111f,
	0xf1f1f11f,
	0xff11ffff,
	0x1111f1f1,
};

#endif /* TOF_SEQ_40_IN_40MHz */

#if !defined(TOF_SEQ_20_IN_80MHz)

#define k_tof_seq_spb_len_80MHz 8
const uint32 k_tof_seq_spb_80MHz[2*k_tof_seq_spb_len_80MHz] = {
	0xee2ee200,
	0xe2e2ee22,
	0xe22eeeee,
	0xeeee2e2e,
	0xe2ee22ee,
	0xe22222e2,
	0xe2e2e22e,
	0x00000eee,
	0x11000000,
	0x1f1f11ff,
	0x1ff11111,
	0x1111f1f1,
	0x1f11ff11,
	0x1fffff1f,
	0x1f1f1ff1,
	0x01fff111,
};

#endif /* !defined(TOF_SEQ_20_IN_80MHz) */


int
wlc_phy_tof_seq_params_acphy(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	if (CHSPEC_IS2G(pi->radio_chanspec))
		return BCME_ERROR;

	if (CHSPEC_IS80(pi->radio_chanspec)) {

		pi_ac->tof_sc_FS = 160;
#ifdef TOF_SEQ_20_IN_80MHz
		pi_ac->tof_seq_log2_n = k_tof_seq_log2_n_80MHz_20;
		memcpy(pi_ac->tof_ucode_dlys_us, k_tof_ucode_dlys_us_80MHz_20, sizeof(uint16)*10);
		pi_ac->tof_seq_spb_len = k_tof_seq_spb_len_20MHz;
		pi_ac->tof_seq_spb = (uint32 *) k_tof_seq_spb_20MHz;
#else
		pi_ac->tof_seq_log2_n = k_tof_seq_log2_n_80MHz;
		memcpy(pi_ac->tof_ucode_dlys_us, k_tof_ucode_dlys_us_80MHz, sizeof(uint16)*10);
		pi_ac->tof_seq_spb_len = k_tof_seq_spb_len_80MHz;
		pi_ac->tof_seq_spb = (uint32 *) k_tof_seq_spb_80MHz;
#endif

		return BCME_OK;

	}

#if defined(TOF_SEQ_40MHz_BW) || defined(TOF_SEQ_40_IN_40MHz)
	else if (CHSPEC_IS40(pi->radio_chanspec)) {
		pi_ac->tof_sc_FS = 80;
		pi_ac->tof_seq_log2_n = k_tof_seq_log2_n_40MHz;
#ifdef TOF_SEQ_40_IN_40MHz
		memcpy(pi_ac->tof_ucode_dlys_us, k_tof_ucode_dlys_us_40MHz_40, sizeof(uint16)*10);
		pi_ac->tof_seq_spb_len = k_tof_seq_spb_len_40MHz;
		pi_ac->tof_seq_spb = (uint32 *) k_tof_seq_spb_40MHz;
#else
		memcpy(pi_ac->tof_ucode_dlys_us, k_tof_ucode_dlys_us_40MHz, sizeof(uint16)*10);
		pi_ac->tof_seq_spb_len = k_tof_seq_spb_len_20MHz;
		pi_ac->tof_seq_spb = (uint32 *) k_tof_seq_spb_20MHz;
#endif
		return BCME_OK;

	}
#endif /* defined(TOF_SEQ_40MHz_BW) || defined (TOF_SEQ_40_IN_40MHz) */

#ifdef TOF_SEQ_20MHz_BW
else {
		pi_ac->tof_sc_FS = 40;
		pi_ac->tof_seq_log2_n = k_tof_seq_log2_n_20MHz;
		memcpy(pi_ac->tof_ucode_dlys_us, k_tof_ucode_dlys_us_20MHz, sizeof(uint16)*10);
		pi_ac->tof_seq_spb_len = k_tof_seq_spb_len_20MHz;
		pi_ac->tof_seq_spb = (uint32 *) k_tof_seq_spb_20MHz;

		return BCME_OK;

	}
#endif
	return BCME_ERROR;
}

#endif /* defined(WL_PROXDETECT) || defined(WL_PROXDETECT_OL) */

#if (defined(WLTEST) || defined(WLPKTENG))
static bool
wlc_phy_isperratedpden_acphy(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (CHSPEC_IS2G(pi->radio_chanspec))
		return (pi->epacal2g && pi_ac->perratedpd2g);
	else
		return (pi->epacal5g && pi_ac->perratedpd5g);
}

static void
wlc_phy_perratedpdset_acphy(phy_info_t *pi, bool enable)
{
	/* Set the new value */
	MOD_PHYREG(pi, PapdEnable0, papd_compEnb0, enable);
}
#endif 

uint8 wlc_phy_get_tbl_id_gainctrlbbmultluts(phy_info_t *pi, uint8 core)
{
	uint8 tbl_id = ACPHY_TBL_ID_GAINCTRLBBMULTLUTS;
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		tbl_id = ACPHY_TBL_ID_GAINCTRLBBMULTLUTS0;

		if ((phy_get_phymode(pi) != PHYMODE_RSDB) && (core == 1))  {
			tbl_id = ACPHY_TBL_ID_GAINCTRLBBMULTLUTS1;
		}
	} else if (ACMAJORREV_5(pi->pubpi->phy_rev)&& ACMINORREV_2(pi)) {
	   switch (core) {
		   case 0:
			   tbl_id = ACPHY_TBL_ID_TXGAINCTRLBBMULTLUTS0;
			   break;
		   case 1:
			   tbl_id = ACPHY_TBL_ID_TXGAINCTRLBBMULTLUTS1;
			   break;
		   case 2:
			   tbl_id = ACPHY_TBL_ID_TXGAINCTRLBBMULTLUTS2;
			   break;
	   }
	} else {
		tbl_id = ACPHY_TBL_ID_GAINCTRLBBMULTLUTS;
	}
	return (tbl_id);
}
uint8 wlc_phy_get_tbl_id_iqlocal(phy_info_t *pi, uint16 core)
{
	uint8 tbl_id;
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		tbl_id = ACPHY_TBL_ID_IQLOCAL0;

		if ((phy_get_phymode(pi) != PHYMODE_RSDB) && (core == 1))  {
			tbl_id = ACPHY_TBL_ID_IQLOCAL1;
		}
	} else {
		tbl_id = ACPHY_TBL_ID_IQLOCAL;
	}
	return (tbl_id);
}
uint8 wlc_phy_get_tbl_id_estpwrshftluts(phy_info_t *pi, uint8 core)
{
	uint8 tbl_id;
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		tbl_id = ACPHY_TBL_ID_ESTPWRSHFTLUTS0;

		if ((phy_get_phymode(pi) != PHYMODE_RSDB) && (core == 1))  {
			tbl_id = ACPHY_TBL_ID_ESTPWRSHFTLUTS1;
		}
	} else {
		tbl_id = ACPHY_TBL_ID_ESTPWRSHFTLUTS;
	}
	return (tbl_id);
}

#if defined(DNG_DBGDUMP)
void wlc_acphy_print_phydbg_regs(phy_info_t *pi)
{
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {
		uint16 phymode = phy_get_phymode(pi);
		PHY_ERROR(("*** [PHY_DBG] *** : wl%d:: PHYMODE: 0x%x\n", pi->sh->unit, phymode));

		if ((phymode == PHYMODE_RSDB) &&
			(phy_get_current_core(pi) == PHY_RSBD_PI_IDX_CORE0)) {
			PHY_ERROR(("*** [PHY_DBG] *** : RSDB Cr 0: wl isup: %d\n", pi->sh->up));
		} else if ((phymode == PHYMODE_RSDB) &&
			(phy_get_current_core(pi) == PHY_RSBD_PI_IDX_CORE1)) {
			PHY_ERROR(("*** [PHY_DBG] *** : RSDB Cr 1: wl isup: %d\n", pi->sh->up));
		} else {
			PHY_ERROR(("*** [PHY_DBG] *** : MIMO : wl isup: %d\n", pi->sh->up));
		}

		PHY_ERROR(("*** [PHY_DBG] *** : RxFeStatus: 0x%x\n",
			READ_PHYREG(pi, RxFeStatus)));
		PHY_ERROR(("*** [PHY_DBG] *** : TxFIFOStatus0: 0x%x\n",
			READ_PHYREG(pi, TxFIFOStatus0)));
		PHY_ERROR(("*** [PHY_DBG] *** : TxFIFOStatus1: 0x%x\n",
			READ_PHYREG(pi, TxFIFOStatus1)));
		PHY_ERROR(("*** [PHY_DBG] *** : RfseqMode: 0x%x\n",
			READ_PHYREG(pi, RfseqMode)));
		PHY_ERROR(("*** [PHY_DBG] *** : RfseqStatus0: 0x%x\n",
			READ_PHYREG(pi, RfseqStatus0)));
		PHY_ERROR(("*** [PHY_DBG] *** : RfseqStatus1: 0x%x\n",
			READ_PHYREG(pi, RfseqStatus1)));
		PHY_ERROR(("*** [PHY_DBG] *** : bphyTxError: 0x%x\n",
			READ_PHYREG(pi, bphyTxError)));
		PHY_ERROR(("*** [PHY_DBG] *** : TxCCKError: 0x%x\n",
			READ_PHYREG(pi, TxCCKError)));
		PHY_ERROR(("*** [PHY_DBG] *** : TxError: 0x%x\n",
			READ_PHYREG(pi, TxError)));
	}
}
#endif /* DNG_DBGDUMP */


/* **************************** */
/* ACPhy Watchdog Functionality */
/* **************************** */
wd_module_t *
BCMRAMFN(get_wd_feature_list)(void)
{
	return wd_feature_list;
}

wd_module_t *
BCMRAMFN(get_wd_module_list)(void)
{
	return wd_module_list;
}

/* Check to see if a cal needs to be run */
static bool
phy_ac_wd_perical_schedule(phy_info_t *pi)
{
	if (pi->disable_percal)
		return FALSE;

	if (IS_WFD_PHY_LL_ENABLE(pi) && DCS_INPROG_PHY(pi))
		return FALSE;

	if ((pi->phy_cal_mode == PHY_PERICAL_DISABLE) ||
		(pi->phy_cal_mode == PHY_PERICAL_MANUAL) ||
		(pi->cal_info->cal_suppress_count != 0))
		return FALSE;

	if (GLACIAL_TIMEOUT(pi)) {

		/* RSDB designs need special handling */
		if (RSDB_FAMILY(pi)) {

			uint8 curr_core = phy_get_current_core(pi);
			uint16 phymode = phy_get_phymode(pi);

			if ((phymode == PHYMODE_RSDB) &&
				(curr_core == PHY_RSBD_PI_IDX_CORE1) &&
				!PUB_NOT_ASSOC(phy_get_other_pi(pi))) {

				uint referred_time, available_cal_slot;

				referred_time = PHYTIMER_NOW(phy_get_other_pi(pi));
				available_cal_slot = LAST_CAL_TIME(phy_get_other_pi(pi)) +
					(GLACIAL_TIMER(pi) >> 1);

				if (referred_time <= available_cal_slot)
					pi->sh->scheduled_cal_time = available_cal_slot;
				else
					pi->sh->scheduled_cal_time = available_cal_slot +
						GLACIAL_TIMER(pi);

				PHY_CAL(("wl%d: %s : Rt %d Acs %d | Sct %d\n",
					PI_INSTANCE(pi), __FUNCTION__,
					referred_time, available_cal_slot,
					pi->sh->scheduled_cal_time));

				if (referred_time >= pi->sh->scheduled_cal_time)
					return TRUE;
				else
					return FALSE;
			} /* instance : pi(1) */
			else
				return TRUE;
		} /* chip : rsdb family */
		else
			return TRUE;
	} /* event : galcial timeout */
	else
		return FALSE;
}

static void
phy_ac_wd_btcx_desense(phy_info_t *pi)
{
#if !defined(WLC_DISABLE_ACI) && defined(BCMLTECOEX)
	if (wlapi_ltecx_get_lte_map(pi->sh->physhim)) {
		int32 ltecx_ed_thresh_val = LTECX_ED_THRESH;
		wlc_phy_adjust_ed_thres_acphy(pi, &ltecx_ed_thresh_val, TRUE);
		if (wlapi_ltecx_chk_elna_bypass_mode(pi->sh->physhim) &&
			!pi->u.pi_acphy->ltecx_elna_bypass_status) {
			wlc_phy_desense_ltecx_acphy(pi, 1);
		} else if (!wlapi_ltecx_chk_elna_bypass_mode(pi->sh->physhim) &&
			pi->u.pi_acphy->ltecx_elna_bypass_status) {
			wlc_phy_desense_ltecx_acphy(pi, 0);
		}
	} else {
		int32 ed_thresh_default_val = LTECX_DEFAULT_ED_THRESH;
		wlc_phy_adjust_ed_thres_acphy(pi, &ed_thresh_default_val, TRUE);
		if (pi->u.pi_acphy->ltecx_elna_bypass_status) {
			wlc_phy_desense_ltecx_acphy(pi, 0);
		}
	}
#endif /* BCMLTECOEX && WLC_DISABLE_ACI */
}

static void
phy_ac_wd_wfd_ll(phy_info_t *pi)
{
#ifdef WFD_PHY_LL
	/* Be sure there is no cal in progress to enable/disable optimization */
	if (!PHY_PERICAL_MPHASE_PENDING(pi)) {
		if (pi->wfd_ll_enable != pi->wfd_ll_enable_pending) {
			pi->wfd_ll_enable = pi->wfd_ll_enable_pending;
			if (!pi->wfd_ll_enable) {
				/* Force a watchdog CAL when disabling WFD optimization
				 * As PADP CAL has not been executed since a long time
				 * a PADP CAL is executed at the next watchdog timeout
				 */
				pi->cal_info->last_cal_time = 0;
			}
		}
	}
#endif /* WFD_PHY_LL */
}

static void
phy_ac_wd_hirssi_elnabypass_engine(phy_info_t *pi)
{
	/* Bypass elna if hirssi is detected to protect lna1.
	 * Applicable only for 4360 A0/B0 designs
	 */
	if (PHY_SW_HIRSSI_UCODE_CAP(pi))
		wlc_phy_hirssi_elnabypass_engine(pi);
}

static void
phy_ac_wd_periodic_calibs(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	/* Local copy of phyrxchains & EnTx bits before overwrite */
	uint8 enRx = 0;
	uint8 enTx = 0;

	BCM_REFERENCE(enTx);
	BCM_REFERENCE(enRx);

	/* Check to see if a cal needs to be run */
	if (phy_ac_wd_perical_schedule(pi)) {

		PHY_CAL(("wl%d: %s: wd triggered cal\n"
			"(cal mode=%d, now=%d,"
			"prev_time=%d, gt=%d)\n",
			PI_INSTANCE(pi), __FUNCTION__,
			pi->phy_cal_mode, PHYTIMER_NOW(pi),
			LAST_CAL_TIME(pi), GLACIAL_TIMER(pi)));

		wlc_phy_cal_perical((wlc_phy_t *)pi, PHY_PERICAL_WATCHDOG);
	}

	/* Enabling crsmin_cal from watchdog. crsmin phyregs are only updated if the
	 * (current noise power - prev value from cache) is above certain threshold
	 */
	if (pi_ac->crsmincal_enable) {
		/* Save and overwrite Rx chains */
		wlc_phy_update_rxchains((wlc_phy_t *)pi, &enRx, &enTx);
		wlc_phy_noise_sample_request_crsmincal((wlc_phy_t*)pi);
		/* Restore Rx chains */
		wlc_phy_restore_rxchains((wlc_phy_t *)pi, enRx, enTx);
	}
}

static void
phy_ac_wd_report_rssi(phy_info_t *pi)
{
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;

	/* Change cores for RSSI reporting every sec to get an idea of all cores */
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	MOD_PHYREG(pi, RssiStatusControl, coreSel, pi_ac->rssi_coresel);
	pi_ac->rssi_coresel = (pi_ac->rssi_coresel + 1) % PHYCORENUM(pi->pubpi->phy_corenum);
	wlapi_enable_mac(pi->sh->physhim);
}

static void
phy_ac_wd_wars(phy_info_t *pi)
{
	if (CHIPID(pi->sh->chip) == BCM4360_CHIP_ID)
		wlc_phy_stop_bt_toggle_acphy(pi);
}

static void
phy_ac_wd_update_features(phy_info_t *pi)
{
	wd_module_t *wdf = get_wd_feature_list();

	while (*wdf != NULL) {
		(*wdf)(pi);
		wdf++;
	}
}

static void
wlc_phy_watchdog_acphy(phy_info_t *pi)
{
	wd_module_t *wdm = get_wd_module_list();

	while (*wdm != NULL) {
		(*wdm)(pi);
		wdm++;
	}
}
#endif /* (ACCONF != 0) || (ACCONF2 != 0) */
