/*
 * ACPHY DeepSleepInit module
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
 * $Id: phy_ac_dsi.c 527374 2015-01-17 05:11:33Z $
 */

#include <phy_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>

/* PHY common dependencies */
#include <phy_dbg.h>
#include <phy_mem.h>
#include <phy_utils_reg.h>
#include <phy_utils_radio.h>

/* PHY type dependencies */
#include <phy_ac.h>
#include <phy_ac_info.h>
#include <wlc_phyreg_ac.h>
#include <wlc_phy_radio.h>
#include <wlc_phytbl_ac.h>

/* DSI module dependencies */
#include <phy_ac_dsi.h>
#include "phy_type_dsi.h"

/* module private states */
struct phy_ac_dsi_info {
	phy_info_t *pi;
	phy_ac_info_t *aci;
	phy_dsi_info_t *di;
};

/* dispatch handle */
typedef void (*dsi_fn_t)(phy_info_t *pi);

/* macros & data structures */
#define ADDR_F	0
#define DATA_F	1

#define NUM_RFSEQ		22
#define NUM_PHYTBLS		13
#define NUM_PHYREGS		76
#define NUM_RADIOREGS		55
#define NUM_RFSEQ_VALS		172 /* add up column 2 of addr_RFSeq */

/* tbl lengths */
#define TL_TXEVMTBL		40
#define TL_GAINLIMIT		105
#define TL_ESTPWRSHFTLUTS	24
#define TL_NVRXEVMSHAPINGTBL	256
#define TL_IQLOCAL		160
#define TL_FEMCTRLLUT		256
#define TL_ESTPWRLUTS0		128
#define TL_PAPR			68
#define TL_EPSILON0		64
#define TL_RSSICLIPGAIN0	22
#define TL_PHASETRACKTBL_1X1	22
#define TL_NVNOISESHAPINGTBL	256
#define TL_RFSEQBUNDLE		128
#define TL_CHANNELSMOOTHING_1x1	512
#define TL_RFSEQEXT		9

typedef struct {
	uint16	offset;
	uint8	len;
} rfseq_t;

typedef struct {
	uint16 id;
	uint16 len;
	uint16 offset;
	uint8 width;
	void *data;
} tbl_t;

/* Address Data Pairs : adp vectors */
static rfseq_t addr_RFSeq[NUM_RFSEQ];

static uint16 DECLSPEC_ALIGN(4) phyregs_adp[NUM_PHYREGS][2];
static uint16 DECLSPEC_ALIGN(4) radioregs_adp[NUM_RADIOREGS][2];

static uint8  DECLSPEC_ALIGN(4) data_TXEVMTBL[TL_TXEVMTBL];
static uint8  DECLSPEC_ALIGN(4) data_GAINLIMIT[TL_GAINLIMIT];
static uint8  DECLSPEC_ALIGN(4) data_ESTPWRSHFTLUTS[TL_ESTPWRSHFTLUTS];
static uint8  DECLSPEC_ALIGN(4) data_NVRXEVMSHAPINGTBL[TL_NVRXEVMSHAPINGTBL];
static uint16 DECLSPEC_ALIGN(4) data_IQLOCAL[TL_IQLOCAL];
static uint16 DECLSPEC_ALIGN(4) data_FEMCTRLLUT[TL_FEMCTRLLUT];
static uint16 DECLSPEC_ALIGN(4) data_ESTPWRLUTS0[TL_ESTPWRSHFTLUTS];
static uint16 DECLSPEC_ALIGN(4) data_RFSeq[NUM_RFSEQ_VALS];
static uint16 DECLSPEC_ALIGN(4) data_RFSEQBUNDLE[TL_RFSEQBUNDLE * 3];
static uint16 DECLSPEC_ALIGN(4) data_CHANNELSMOOTHING[TL_CHANNELSMOOTHING_1x1 * 3];
static uint32 DECLSPEC_ALIGN(4) data_PAPR[TL_PAPR];
static uint32 DECLSPEC_ALIGN(4) data_EPSILON0[TL_EPSILON0];
static uint32 DECLSPEC_ALIGN(4) data_RSSICLIPGAIN0[TL_RSSICLIPGAIN0];
static uint32 DECLSPEC_ALIGN(4) data_PHASETRACKTBL[TL_PHASETRACKTBL_1X1];
static uint32 DECLSPEC_ALIGN(4) data_NVNOISESHAPINGTBL[TL_NVNOISESHAPINGTBL];
static uint32 DECLSPEC_ALIGN(4) data_RFSEQEXT[TL_RFSEQEXT * 2];

/* size vectors */
static uint16 sz_phyregs;
static uint16 sz_radioregs;

static uint16 sz_TXEVMTBL;
static uint16 sz_GAINLIMIT;
static uint16 sz_ESTPWRSHFTLUTS;
static uint16 sz_NVRXEVMSHAPINGTBL;
static uint16 sz_IQLOCAL;
static uint16 sz_FEMCTRLLUT;
static uint16 sz_ESTPWRLUTS0;
static uint16 sz_RFSEQBUNDLE;
static uint16 sz_CHANNELSMOOTHING;
static uint16 sz_PAPR;
static uint16 sz_EPSILON0;
static uint16 sz_RSSICLIPGAIN0;
static uint16 sz_PHASETRACKTBL;
static uint16 sz_NVNOISESHAPINGTBL;
static uint16 sz_RFSEQEXT;
static uint16 sz_RFSeq;

tbl_t tbl_info[NUM_PHYTBLS] =
{
	{ACPHY_TBL_ID_TXEVMTBL, TL_TXEVMTBL, 0, 8, data_TXEVMTBL},
	{ACPHY_TBL_ID_GAINLIMIT, TL_GAINLIMIT, 0, 8, data_GAINLIMIT},
	{ACPHY_TBL_ID_ESTPWRSHFTLUTS, TL_ESTPWRSHFTLUTS, 0, 8, data_ESTPWRSHFTLUTS},
	{ACPHY_TBL_ID_NVRXEVMSHAPINGTBL, TL_NVRXEVMSHAPINGTBL, 0, 8, data_NVRXEVMSHAPINGTBL},
	{ACPHY_TBL_ID_IQLOCAL, TL_IQLOCAL, 0, 16, data_IQLOCAL},
	/* {ACPHY_TBL_ID_FEMCTRLLUT, TL_FEMCTRLLUT, 0, 16, data_FEMCTRLLUT}, */
	{ACPHY_TBL_ID_ESTPWRLUTS0, TL_ESTPWRLUTS0, 0, 16, data_ESTPWRLUTS0},
	{ACPHY_TBL_ID_RFSEQBUNDLE, TL_RFSEQBUNDLE, 0, 48, data_RFSEQBUNDLE},
	{ACPHY_TBL_ID_CHANNELSMOOTHING_1x1, TL_CHANNELSMOOTHING_1x1, 0, 48, data_CHANNELSMOOTHING},
	{ACPHY_TBL_ID_PAPR, TL_PAPR, 0, 32, data_PAPR},
	{ACPHY_TBL_ID_EPSILON0, TL_EPSILON0, 0, 32, data_EPSILON0},
	{ACPHY_TBL_ID_RSSICLIPGAIN0, TL_RSSICLIPGAIN0, 0, 32, data_RSSICLIPGAIN0},
	{ACPHY_TBL_ID_PHASETRACKTBL_1X1, TL_PHASETRACKTBL_1X1, 0, 32, data_PHASETRACKTBL},
	{ACPHY_TBL_ID_NVNOISESHAPINGTBL, TL_NVNOISESHAPINGTBL, 0, 32, data_NVNOISESHAPINGTBL}
	/* {ACPHY_TBL_ID_RFSEQEXT, TL_RFSEQEXT, 0, 60, data_RFSEQEXT} */
};

/* local functions */
static void dsi_populate_addr(phy_info_t *pi);

static void dsi_restore(phy_type_dsi_ctx_t *ctx);
static int  dsi_save(phy_type_dsi_ctx_t *ctx);

static void dsi_restore_wars(phy_info_t *pi);
static void dsi_restore_tables(phy_info_t *pi);
static void dsi_restore_phyregs(phy_info_t *pi);
static void dsi_restore_prefvals(phy_info_t *pi);
static void dsi_restore_radioregs(phy_info_t *pi);

static void dsi_save_tables(phy_info_t *pi);
static void dsi_save_phyregs(phy_info_t *pi);
static void dsi_save_radioregs(phy_info_t *pi);

/* Programs PhyInit RegSpace to PreferredVals. These will go as chip defaults for 43012
 * Spliting prefvals into 3 functions : Small stack size, (Max stack frame sz = 1024 bytes)
 */
static void dsi_prefvals_SetA(phy_info_t *pi);
static void dsi_prefvals_SetB(phy_info_t *pi);
static void dsi_prefvals_SetC(phy_info_t *pi);

/* register phy type specific implementation */
phy_ac_dsi_info_t *
BCMATTACHFN(phy_ac_dsi_register_impl)(phy_info_t *pi, phy_ac_info_t *aci, phy_dsi_info_t *di)
{
	phy_ac_dsi_info_t *info;
	phy_type_dsi_fns_t fns;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* allocate all storage together */
	if ((info = phy_malloc(pi, sizeof(phy_ac_dsi_info_t))) == NULL) {
		PHY_ERROR(("%s: phy_malloc failed\n", __FUNCTION__));
		goto fail;
	}
	info->pi = pi;
	info->aci = aci;
	info->di = di;

	/* register PHY type specific implementation */
	bzero(&fns, sizeof(fns));
	fns.restore = dsi_restore;
	fns.save = dsi_save;
	fns.ctx = info;

	dsi_populate_addr(pi);

	phy_dsi_register_impl(di, &fns);

	return info;

	/* error handling */
fail:
	if (info != NULL)
		phy_mfree(pi, info, sizeof(phy_ac_dsi_info_t));

	return NULL;
}

void
BCMATTACHFN(phy_ac_dsi_unregister_impl)(phy_ac_dsi_info_t *info)
{
	phy_info_t *pi = info->pi;
	phy_dsi_info_t *di = info->di;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* unregister from common */
	phy_dsi_unregister_impl(di);

	phy_mfree(pi, info, sizeof(phy_ac_dsi_info_t));
}

static int
dsi_save(phy_type_dsi_ctx_t *ctx)
{
	phy_ac_dsi_info_t *di = (phy_ac_dsi_info_t *)ctx;
	phy_info_t *pi = di->pi;

	uint16 stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);

	dsi_fn_t save_fn_list[] = {
		dsi_save_tables,
		dsi_save_phyregs,
		dsi_save_radioregs,
		NULL
	};
	dsi_fn_t *fn = save_fn_list;

	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);
	ACPHY_DISABLE_STALL(pi);

	do {
		(*fn)(pi);
		++fn;
	} while (*fn != NULL);

	ACPHY_ENABLE_STALL(pi, stall_val);
	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);

	return BCME_OK;
}

static void
dsi_restore(phy_type_dsi_ctx_t *ctx)
{
	phy_ac_dsi_info_t *di = (phy_ac_dsi_info_t *)ctx;
	phy_info_t *pi = di->pi;

	uint32 tstamp = R_REG(pi->sh->osh, &pi->regs->tsf_timerlow);
	uint16 stall_val = READ_PHYREGFLD(pi, RxFeCtrl1, disable_stalls);

	dsi_fn_t restore_fn_list[] = {
		dsi_restore_prefvals,
		dsi_restore_tables,
		dsi_restore_phyregs,
		dsi_restore_radioregs,
		dsi_restore_wars,
		wlc_phy_resetcca_acphy,
		NULL
	};
	dsi_fn_t *fn = restore_fn_list;

	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);
	ACPHY_DISABLE_STALL(pi);

	do {
		(*fn)(pi);
		++fn;
	} while (*fn != NULL);


	ACPHY_ENABLE_STALL(pi, stall_val);
	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);

	tstamp = R_REG(pi->sh->osh, &pi->regs->tsf_timerlow) - tstamp;

	PHY_TRACE(("*** wl%d: %s | DeepSleepInit took %d us\n",
		PI_INSTANCE(pi), __FUNCTION__, tstamp));
}

static void
dsi_populate_addr(phy_info_t *pi)
{
	int i = 0;

	uint16 map_PA[] =
	{
		ACPHY_REG(pi, Core0_TRLossValue),
		ACPHY_REG(pi, Core0clip2GainCodeA),
		ACPHY_REG(pi, Core0clip2GainCodeB),
		ACPHY_REG(pi, Core0clipHiGainCodeA),
		ACPHY_REG(pi, Core0clipHiGainCodeB),
		ACPHY_REG(pi, Core0cliploGainCodeA),
		ACPHY_REG(pi, Core0cliploGainCodeB),
		ACPHY_REG(pi, Core0clipmdGainCodeA),
		ACPHY_REG(pi, Core0clipmdGainCodeB),
		ACPHY_REG(pi, Core0computeGainInfo),
		ACPHY_REG(pi, Core0InitGainCodeA),
		ACPHY_REG(pi, Core0InitGainCodeB),
		ACPHY_REG(pi, CoreConfig),
		ACPHY_REG(pi, timeoutEn),
		ACPHY_REG(pi, TxMacIfHoldOff),
		ACPHY_REG(pi, TxRifsFrameDelay),
		ACPHY_REG(pi, TxMacDelay),
		ACPHY_REG(pi, TSSIMode),
		ACPHY_REG(pi, TxPwrCtrlCmd),
		ACPHY_REG(pi, TxPwrCtrlIdleTssi_path0),
		ACPHY_REG(pi, TxPwrCtrlIdleTssi_second_path0),
		ACPHY_REG(pi, TxPwrCtrlInit_path0),
		ACPHY_REG(pi, TxPwrCtrlNnum),
		ACPHY_REG(pi, RfctrlOverrideAfeCfg0),
		ACPHY_REG(pi, RfctrlOverrideGains0),
		ACPHY_REG(pi, RfctrlCoreLowPwr0),
		ACPHY_REG(pi, RfctrlOverrideLpfCT0),
		ACPHY_REG(pi, RfctrlOverrideLpfSwtch0),
		ACPHY_REG(pi, RfctrlCoreTxPus0),
		ACPHY_REG(pi, RfctrlOverrideTxPus0),
		ACPHY_REG(pi, RfctrlCoreRxPus0),
		ACPHY_REG(pi, RfctrlOverrideRxPus0),
		ACPHY_REG(pi, RfseqMode),
		ACPHY_REG(pi, RxFeCtrl1),
		ACPHY_REG(pi, FemCtrl),
		ACPHY_REG(pi, radio_logen2g),
		ACPHY_REG(pi, radio_logen2gN5g),
		ACPHY_REG(pi, radio_logen5g),
		ACPHY_REG(pi, radio_logen5gbufs),
		ACPHY_REG(pi, radio_logen5gQI),
		ACPHY_REG(pi, radio_pll_vcocal),
		ACPHY_REG(pi, radio_pll_vcoSet1),
		ACPHY_REG(pi, radio_pll_vcoSet2),
		ACPHY_REG(pi, radio_pll_vcoSet3),
		ACPHY_REG(pi, radio_pll_vcoSet4),
		ACPHY_REG(pi, radio_pll_lf_r1),
		ACPHY_REG(pi, radio_pll_lf_r2r3),
		ACPHY_REG(pi, radio_pll_lf_cm),
		ACPHY_REG(pi, radio_pll_lf_cSet1),
		ACPHY_REG(pi, radio_pll_lf_cSet2),
		ACPHY_REG(pi, radio_pll_cp),
		ACPHY_REG(pi, radio_ldo),
		ACPHY_REG(pi, radio_rxrf_lna2g),
		ACPHY_REG(pi, radio_rxrf_lna5g),
		ACPHY_REG(pi, radio_rxrf_rxmix),
		ACPHY_REG(pi, radio_rxbb_tia),
		ACPHY_REG(pi, radio_rxbb_bias12),
		ACPHY_REG(pi, radio_rxbb_bias34),
		ACPHY_REG(pi, radio_pll_vcoSet1),
		ACPHY_REG(pi, radio_pll_vcoSet2),
		ACPHY_REG(pi, radio_pll_vcoSet3),
		ACPHY_REG(pi, radio_pll_vcoSet4),
		ACPHY_REG(pi, lbFarrowDeltaPhase_hi),
		ACPHY_REG(pi, lbFarrowDeltaPhase_lo),
		ACPHY_REG(pi, lbFarrowDriftPeriod),
		ACPHY_REG(pi, lbFarrowCtrl),
		ACPHY_REG(pi, rxFarrowDeltaPhase_hi),
		ACPHY_REG(pi, rxFarrowDeltaPhase_lo),
		ACPHY_REG(pi, rxFarrowDriftPeriod),
		ACPHY_REG(pi, TxResamplerMuDelta0l),
		ACPHY_REG(pi, TxResamplerMuDelta0u),
		ACPHY_REG(pi, TxResamplerMuDeltaInit0l),
		ACPHY_REG(pi, TxResamplerMuDeltaInit0u),
		ACPHY_REG(pi, Core1RxIQCompA0),
		ACPHY_REG(pi, Core1RxIQCompB0),
		ACPHY_REG(pi, TableOffset)
	};

	uint16 map_RA[] =
	{
		RF0_2069_OVR21,
		RF0_2069_LNA2G_TUNE,
		RF0_2069_LNA5G_TUNE,
		RF0_2069_TXGM_CFG1,
		RF0_2069_TXMIX2G_CFG1,
		RF0_2069_TXMIX5G_CFG1,
		RF0_2069_PGA2G_CFG1,
		RF0_2069_PGA2G_CFG2,
		RF0_2069_PGA5G_CFG2,
		RF0_2069_PGA5G_IDAC,
		RF0_2069_PGA5G_INCAP,
		RF0_2069_PAD2G_SLOPE,
		RF0_2069_PAD2G_TUNE,
		RF0_2069_PAD5G_IDAC,
		RF0_2069_PAD5G_TUNE,
		RF0_2069_PAD5G_INCAP,
		RF0_2069_PA2G_CFG2,
		RF0_2069_PA2G_CFG3,
		RF0_2069_PA2G_INCAP,
		RF0_2069_PA5G_CFG2,
		RF0_2069_PA5G_IDAC2,
		RF0_2069_LOGEN5G_RCCR,
		RFP_2069_GE16_BG_CFG1,
		RFP_2069_GE16_PLL_XTALLDO1,
		RFP_2069_GE16_PLL_HVLDO1,
		RFP_2069_GE16_PLL_HVLDO2,
		RFP_2069_GE16_PLL_VCOCAL1,
		RFP_2069_GE16_PLL_VCOCAL6,
		RFP_2069_GE16_PLL_VCOCAL11,
		RFP_2069_GE16_PLL_VCOCAL12,
		RFP_2069_GE16_PLL_XTAL4,
		RFP_2069_GE16_PLL_XTAL5,
		RFP_2069_GE16_PLL_CP4,
		RFP_2069_GE16_PLL_VCO3,
		RFP_2069_GE16_PLL_VCO4,
		RFP_2069_GE16_PLL_VCO5,
		RFP_2069_GE16_PLL_VCO6,
		RFP_2069_GE16_PLL_VCO8,
		RFP_2069_GE16_LOGEN2G_TUNE,
		RFP_2069_GE16_LOGEN5G_TUNE1,
		RFP_2069_GE16_LOGEN5G_TUNE2,
		RFP_2069_GE16_OVR2,
		RFP_2069_GE16_OVR27,
		RFP_2069_GE16_OVR30,
		RFP_2069_GE16_OVR31,
		RFP_2069_GE16_OVR32,
		RFP_2069_GE16_PLL_FRCT2,
		RFP_2069_GE16_PLL_FRCT3,
		RFP_2069_GE16_PLL_LF2,
		RFP_2069_GE16_PLL_LF3,
		RFP_2069_GE16_PLL_LF4,
		RFP_2069_GE16_PLL_LF5,
		RFP_2069_GE16_PLL_LF6,
		RFP_2069_GE16_PLL_LF7,
		RFP_2069_GE16_PLL_XTAL2
	};

	rfseq_t map_RFSeq[] =
	{
		{0x000, 0x10},
		{0x010, 0x10},
		{0x020, 0x10},
		{0x030, 0x8},
		{0x040, 0x8},
		{0x050, 0x8},
		{0x070, 0x10},
		{0x090, 0x10},
		{0x0a0, 0x8},
		{0x0b0, 0x8},
		{0x0c0, 0x8},
		{0x121, 0x5},
		{0x131, 0x2},
		{0x137, 0x2},
		{0x140, 0xb},
		{0x360, 0xb},
		{0x3fa, 0x3},
		{0x3fe, 0x2},
		{0x440, 0x2},
		{0x3c6, 0x2},
		{0x3d6, 0x2},
		{0x3e6, 0x2}
	};

	/* initialize phyregs address array */
	for (i = 0; i < NUM_PHYREGS; i++)
		phyregs_adp[i][ADDR_F] = map_PA[i];

	/* initialize radioregs address array */
	for (i = 0; i < NUM_RADIOREGS; i++)
		radioregs_adp[i][ADDR_F] = map_RA[i];

	/* initialize RF Sequencer struct */
	for (i = 0; i < NUM_RFSEQ; i++) {
		addr_RFSeq[i].offset = map_RFSeq[i].offset;
		addr_RFSeq[i].len = map_RFSeq[i].len;
	}
}

static void
dsi_prefvals_SetA(phy_info_t *pi)
{
	uint16 i = 0;

	uint16 reglist_SetA[][2] = {
		{ACPHY_REG(pi, radio_logen2g), 0x46db},
		{ACPHY_REG(pi, radio_logen2gN5g), 0x3724},
		{ACPHY_REG(pi, radio_logen5g), 0x6db},
		{ACPHY_REG(pi, radio_logen5gbufs), 0x36db},
		{ACPHY_REG(pi, radio_logen5gQI), 0x924},
		{ACPHY_REG(pi, radio_pll_vcocal), 0x101},
		{ACPHY_REG(pi, radio_pll_vcoSet1), 0x3edd},
		{ACPHY_REG(pi, radio_pll_vcoSet2), 0xbe},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x600},
		{ACPHY_REG(pi, radio_pll_vcoSet4), 0x0},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x600},
		{ACPHY_REG(pi, radio_pll_lf_r1), 0x0},
		{ACPHY_REG(pi, radio_pll_lf_r2r3), 0xc0c},
		{ACPHY_REG(pi, radio_pll_lf_cm), 0xcff},
		{ACPHY_REG(pi, radio_pll_lf_cSet1), 0x8b99},
		{ACPHY_REG(pi, radio_pll_lf_cSet2), 0x8f8b},
		{ACPHY_REG(pi, radio_pll_cp), 0x3034},
		{ACPHY_REG(pi, radio_ldo), 0x0},
		{ACPHY_REG(pi, radio_rxrf_lna2g), 0x882},
		{ACPHY_REG(pi, radio_rxrf_lna5g), 0x478},
		{ACPHY_REG(pi, radio_rxrf_rxmix), 0x8888},
		{ACPHY_REG(pi, radio_rxbb_tia), 0x6666},
		{ACPHY_REG(pi, radio_rxbb_bias12), 0x804},
		{ACPHY_REG(pi, radio_rxbb_bias34), 0x2010},
		{ACPHY_REG(pi, radio_pll_vcoSet4), 0x9},
		{ACPHY_REG(pi, radio_pll_vcoSet2), 0x3be},
		{ACPHY_REG(pi, radio_pll_vcoSet4), 0xf69},
		{ACPHY_REG(pi, radio_pll_vcoSet1), 0x3edd},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x640},
		{ACPHY_REG(pi, radio_pll_vcoSet2), 0xb3be},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x64c},
		{ACPHY_REG(pi, radio_pll_vcoSet1), 0xbedd},
		{ACPHY_REG(pi, Pllldo_resetCtrl), 0x0},
		{ACPHY_REG(pi, Rfpll_resetCtrl), 0x0},
		{ACPHY_REG(pi, Logen_AfeDiv_reset), 0x2000},
		{ACPHY_REG(pi, RfctrlCmd), 0xc00},
		{ACPHY_REG(pi, RfctrlCmd), 0xc06},
		{ACPHY_REG(pi, RfctrlCmd), 0xc02},
		{ACPHY_REG(pi, AfeClkDivOverrideCtrlN0), 0x3},
		{ACPHY_REG(pi, AfeClkDivOverrideCtrl), 0x77},
		{ACPHY_REG(pi, AfectrlOverride0), 0x0},
		{ACPHY_REG(pi, crsControlu), 0xc},
		{ACPHY_REG(pi, crsControll), 0x2c},
		{ACPHY_REG(pi, crsControlu), 0xc},
		{ACPHY_REG(pi, crsControll), 0x2c},
		{ACPHY_REG(pi, crsControluSub1), 0xc},
		{ACPHY_REG(pi, crsControllSub1), 0x2c},
		{ACPHY_REG(pi, ed_crsEn), 0x0},
		{ACPHY_REG(pi, DsssStep), 0x668},
		{ACPHY_REG(pi, dot11acphycrsTxExtension), 0xc8},
		{ACPHY_REG(pi, TxRifsFrameDelay), 0x30},
		{ACPHY_REG(pi, crsControll), 0xc},
		{ACPHY_REG(pi, crsControlu), 0xc},
		{ACPHY_REG(pi, crsControllSub1), 0xc},
		{ACPHY_REG(pi, crsControluSub1), 0xc},
		{ACPHY_REG(pi, crsThreshold2l), 0x2055},
		{ACPHY_REG(pi, crsThreshold2u), 0x2055},
		{ACPHY_REG(pi, crsThreshold2lSub1), 0x2055},
		{ACPHY_REG(pi, crsThreshold2uSub1), 0x2055},
		{ACPHY_REG(pi, crsThreshold2l), 0x204d},
		{ACPHY_REG(pi, crsThreshold2u), 0x204d},
		{ACPHY_REG(pi, crsThreshold2lSub1), 0x204d},
		{ACPHY_REG(pi, crsThreshold2uSub1), 0x204d},
		{ACPHY_REG(pi, crsacidetectThreshl), 0x80},
		{ACPHY_REG(pi, crsacidetectThreshlSub1), 0x80},
		{ACPHY_REG(pi, crsacidetectThreshu), 0x80},
		{ACPHY_REG(pi, crsacidetectThreshuSub1), 0x80},
		{ACPHY_REG(pi, initcarrierDetLen), 0x40},
		{ACPHY_REG(pi, clip1carrierDetLen), 0x5c},
		{ACPHY_REG(pi, clip_detect_normpwr_var_mux), 0x0},
		{ACPHY_REG(pi, norm_var_hyst_th_pt8us), 0x4c},
		{ACPHY_REG(pi, CRSMiscellaneousParam), 0x6},
		{ACPHY_REG(pi, CRSMiscellaneousParam), 0x6},
		{ACPHY_REG(pi, HTSigTones), 0x9ee9},
		{ACPHY_REG(pi, FSTRCtrl), 0x7ba},
		{ACPHY_REG(pi, musigb1), 0x7777},
	};

	for (i = 0; i < ARRAYSIZE(reglist_SetA); i++)
		phy_utils_write_phyreg(pi, reglist_SetA[i][0], reglist_SetA[i][1]);
}

static void
dsi_prefvals_SetB(phy_info_t *pi)
{
	uint16 i = 0;

	uint16 reglist_SetB[][2] = {
		{ACPHY_REG(pi, musigb0), 0x7732},
		{ACPHY_REG(pi, ViterbiControl0), 0x3000},
		{ACPHY_REG(pi, defer_setClip2_CtrLen), 0x10},
		{ACPHY_REG(pi, partialAIDCountDown), 0x5c00},
		{ACPHY_REG(pi, DmdCtrlConfig), 0x443b},
		{ACPHY_REG(pi, CoreConfig), 0x29},
		{ACPHY_REG(pi, radio_pll_vcoSet1), 0x3edd},
		{ACPHY_REG(pi, radio_pll_vcoSet2), 0xbe},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x600},
		{ACPHY_REG(pi, radio_pll_vcoSet4), 0x0},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x600},
		{ACPHY_REG(pi, radio_pll_lf_cm), 0xcff},
		{ACPHY_REG(pi, radio_pll_lf_cSet1), 0x8b99},
		{ACPHY_REG(pi, radio_pll_lf_cSet2), 0x8f8b},
		{ACPHY_REG(pi, radio_pll_cp), 0x3034},
		{ACPHY_REG(pi, radio_rxbb_bias12), 0x804},
		{ACPHY_REG(pi, radio_rxbb_bias34), 0x2010},
		{ACPHY_REG(pi, radio_pll_vcoSet4), 0x9},
		{ACPHY_REG(pi, radio_pll_vcoSet2), 0x3be},
		{ACPHY_REG(pi, radio_pll_vcoSet4), 0xf69},
		{ACPHY_REG(pi, radio_pll_vcoSet1), 0x3edd},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x640},
		{ACPHY_REG(pi, radio_pll_vcoSet2), 0xb3be},
		{ACPHY_REG(pi, radio_pll_vcoSet3), 0x64c},
		{ACPHY_REG(pi, radio_pll_vcoSet1), 0xbedd},
		{ACPHY_REG(pi, pktgainSettleLen), 0x30},
		{ACPHY_REG(pi, TxMacIfHoldOff), 0x12},
		{ACPHY_REG(pi, TxMacDelay), 0x2a8},
		{ACPHY_REG(pi, BT_SwControl), 0x290},
		{ACPHY_REG(pi, FemCtrl), 0xffc},
		{ACPHY_REG(pi, FFTSoftReset), 0x2},
		{ACPHY_REG(pi, fineclockgatecontrol), 0x0},
		{ACPHY_REG(pi, RxFeTesMmuxCtrl), 0x60},
		{ACPHY_REG(pi, ed_crs20LAssertThresh0), 0x415},
		{ACPHY_REG(pi, ed_crs20LDeassertThresh0), 0x395},
		{ACPHY_REG(pi, ed_crs20UAssertThresh0), 0x415},
		{ACPHY_REG(pi, ed_crs20UDeassertThresh0), 0x395},
		{ACPHY_REG(pi, ed_crs20Lsub1AssertThresh0), 0x415},
		{ACPHY_REG(pi, ed_crs20Lsub1DeassertThresh0), 0x395},
		{ACPHY_REG(pi, ed_crs20Usub1AssertThresh0), 0x415},
		{ACPHY_REG(pi, ed_crs20Usub1DeassertThresh0), 0x395},
		{ACPHY_REG(pi, bphyTest), 0x100},
		{ACPHY_REG(pi, RfBiasControl), 0xe19b},
		{ACPHY_REG(pi, fineRxclockgatecontrol), 0x0},
		{ACPHY_REG(pi, EpsilonTableAdjust0), 0x8},
		{ACPHY_REG(pi, CRSMiscellaneousParam), 0x6},
		{ACPHY_REG(pi, FSTRMetricTh), 0xf20},
		{ACPHY_REG(pi, energydroptimeoutLen), 0x2},
		{ACPHY_REG(pi, FemCtrl), 0xffd},
		{ACPHY_REG(pi, nonpaydecodetimeoutlen), 0x1},
		{ACPHY_REG(pi, TxResamplerEnable0), 0x49},
		{ACPHY_REG(pi, rxFarrowCtrl), 0x87},
		{ACPHY_REG(pi, lbFarrowCtrl), 0x87},
		{ACPHY_REG(pi, clip2carrierDetLen), 0x48},
		{ACPHY_REG(pi, defer_setClip1_CtrLen), 0x18},
		{ACPHY_REG(pi, Core0FastAgcClipCntTh), 0xe17},
		{ACPHY_REG(pi, BW3), 0x3c1},
		{ACPHY_REG(pi, BW2), 0x3c5},
		{ACPHY_REG(pi, BW1a), 0x3c9},
		{ACPHY_REG(pi, BW6), 0x443},
		{ACPHY_REG(pi, BW5), 0x43f},
		{ACPHY_REG(pi, BW4), 0x43a},
		{ACPHY_REG(pi, crsacidetectThreshl), 0x80},
		{ACPHY_REG(pi, crsacidetectThreshlSub1), 0x80},
		{ACPHY_REG(pi, crsacidetectThreshu), 0x80},
		{ACPHY_REG(pi, crsacidetectThreshuSub1), 0x80},
		{ACPHY_REG(pi, bphyaciPwrThresh0), 0x0},
		{ACPHY_REG(pi, bphyaciPwrThresh1), 0x0},
		{ACPHY_REG(pi, bphyaciPwrThresh2), 0x0},
		{ACPHY_REG(pi, bphyaciThresh0), 0x0},
		{ACPHY_REG(pi, bphyaciThresh1), 0x0},
		{ACPHY_REG(pi, bphyaciThresh2), 0x0},
		{ACPHY_REG(pi, bphyaciThresh3), 0x9f},
		{ACPHY_REG(pi, bphyTest), 0x100},
		{ACPHY_REG(pi, txfiltbphy20in20st0a1), 0xedb},
		{ACPHY_REG(pi, txfiltbphy20in20st0a2), 0x1ab},
	};

	for (i = 0; i < ARRAYSIZE(reglist_SetB); i++)
		phy_utils_write_phyreg(pi, reglist_SetB[i][0], reglist_SetB[i][1]);
}

static void
dsi_prefvals_SetC(phy_info_t *pi)
{
	uint16 i = 0;

	uint16 reglist_SetC[][2] = {
		{ACPHY_REG(pi, txfiltbphy20in20st0n), 0x3},
		{ACPHY_REG(pi, txfiltbphy20in20st1a1), 0xd1d},
		{ACPHY_REG(pi, txfiltbphy20in20st1a2), 0x172},
		{ACPHY_REG(pi, txfiltbphy20in20st1n), 0x3},
		{ACPHY_REG(pi, txfiltbphy20in20st2a1), 0xc77},
		{ACPHY_REG(pi, txfiltbphy20in20st2a2), 0xa9},
		{ACPHY_REG(pi, txfiltbphy20in20st2n), 0x3},
		{ACPHY_REG(pi, txfiltbphy20in20finescale), 0x82},
		{ACPHY_REG(pi, RxControl), 0x4887},
		{ACPHY_REG(pi, PREMPT_cck_nominal_clip_cnt_th0), 0x38},
		{ACPHY_REG(pi, PREMPT_ofdm_nominal_clip_cnt_th0), 0x28},
		{ACPHY_REG(pi, PREMPT_ofdm_large_gain_mismatch_th0), 0x1f},
		{ACPHY_REG(pi, PREMPT_cck_large_gain_mismatch_th0), 0x1f},
		{ACPHY_REG(pi, PREMPT_ofdm_nominal_clip_th0), 0xffff},
		{ACPHY_REG(pi, PREMPT_cck_nominal_clip_th0), 0xffff},
		{ACPHY_REG(pi, SpareReg), 0x3f},
		{ACPHY_REG(pi, PktAbortSupportedStates), 0x2bb7},
		{ACPHY_REG(pi, PktAbortCtrl), 0x1841},
		{ACPHY_REG(pi, BphyAbortExitCtrl), 0x3840},
		{ACPHY_REG(pi, fineRxclockgatecontrol), 0x40},
		{ACPHY_REG(pi, Core0_TRLossValue), 0x1602},
		{ACPHY_REG(pi, fineRxclockgatecontrol), 0x0},
		{ACPHY_REG(pi, Core0InitGainCodeA), 0x16a},
		{ACPHY_REG(pi, Core0InitGainCodeB), 0x624},
		{ACPHY_REG(pi, Core0clipHiGainCodeA), 0x16a},
		{ACPHY_REG(pi, Core0clipHiGainCodeB), 0x14},
		{ACPHY_REG(pi, Core0clipmdGainCodeA), 0x13a},
		{ACPHY_REG(pi, Core0clipmdGainCodeB), 0x4},
		{ACPHY_REG(pi, Core0clip2GainCodeA), 0x128},
		{ACPHY_REG(pi, Core0clip2GainCodeB), 0x14},
		{ACPHY_REG(pi, Core0cliploGainCodeA), 0x15a},
		{ACPHY_REG(pi, Core0cliploGainCodeB), 0x8},
		{ACPHY_REG(pi, Core0RssiClipMuxSel), 0x1},
		{ACPHY_REG(pi, TxPwrCtrlNnum), 0x4aa},
		{ACPHY_REG(pi, sampleLoopCount), 0xffff},
		{ACPHY_REG(pi, sampleInitWaitCount), 0x3c},
		{ACPHY_REG(pi, TxPwrCtrlIdleTssi_path0), 0x2c5},
		{ACPHY_REG(pi, TxPwrCtrlIdleTssi_second_path0), 0x2c5},
		{ACPHY_REG(pi, TxPwrCtrlInit_path0), 0x19},
		{ACPHY_REG(pi, gpioLoOutEn), 0xffff},
		{ACPHY_REG(pi, gpioHiOutEn), 0xffff},
		{ACPHY_REG(pi, IqestSampleCount), 0x200},
		{ACPHY_REG(pi, IqestWaitTime), 0x20},
		{ACPHY_REG(pi, Core1RxIQCompA0), 0x6b},
		{ACPHY_REG(pi, Core1RxIQCompB0), 0x3ed},
		{ACPHY_REG(pi, CRSMiscellaneousParam), 0x16},
		{ACPHY_REG(pi, RfseqCoreActv2059), 0x1111},
		{ACPHY_REG(pi, RfctrlCoreGlobalPus), 0x4},
		{ACPHY_REG(pi, RfctrlOverrideTxPus0), 0x180},
		{ACPHY_REG(pi, RfctrlOverrideRxPus0), 0x5000},
		{ACPHY_REG(pi, RfctrlCoreRxPus0), 0x0},
		{ACPHY_REG(pi, RfctrlOverrideLpfCT0), 0x0},
		{ACPHY_REG(pi, RfctrlOverrideLowPwrCfg0), 0xc},
		{ACPHY_REG(pi, RfctrlCoreLowPwr0), 0x2c},
		{ACPHY_REG(pi, RfctrlCoreAuxTssi10), 0x30},
		{ACPHY_REG(pi, TxPwrCtrlCmd), 0x0},
		{ACPHY_REG(pi, PapdEnable0), 0x15ad},
		{ACPHY_REG(pi, DSSF_gain_th0_s20), 0x44},
		{ACPHY_REG(pi, DSSF_gain_th1_s20), 0x4e},
		{ACPHY_REG(pi, DSSF_gain_th2_s20), 0x54},
		{ACPHY_REG(pi, RfseqMode), 0x0},
		{ACPHY_REG(pi, RfctrlOverrideAfeCfg0), 0x600},
		{ACPHY_REG(pi, RfctrlCoreAfeCfg20), 0x180},
		{ACPHY_REG(pi, chnsmCtrl0), 0x33f},
		{ACPHY_REG(pi, chnsmCtrl1), 0x6c0},
		{ACPHY_REG(pi, papr_ctrl), 0x8},
		{ACPHY_REG(pi, Core0computeGainInfo), 0xc61},
		{ACPHY_REG(pi, crsControlu), 0x1c},
		{ACPHY_REG(pi, crsControll), 0x1c},
		{ACPHY_REG(pi, crsControluSub1), 0x1c},
		{ACPHY_REG(pi, crsControllSub1), 0x1c},
		{ACPHY_REG(pi, ed_crsEn), 0xfff},
		{ACPHY_REG(pi, timeoutEn), 0x1f},
		{ACPHY_REG(pi, ofdmpaydecodetimeoutlen), 0x7d0},
		{ACPHY_REG(pi, cckpaydecodetimeoutlen), 0x7d0},
		{ACPHY_REG(pi, RxFeCtrl1), 0x250}
	};

	for (i = 0; i < ARRAYSIZE(reglist_SetC); i++)
		phy_utils_write_phyreg(pi, reglist_SetC[i][0], reglist_SetC[i][1]);
}

static void
dsi_restore_prefvals(phy_info_t *pi)
{
	dsi_prefvals_SetA(pi);
	dsi_prefvals_SetB(pi);
	dsi_prefvals_SetC(pi);
}

static void
dsi_restore_wars(phy_info_t *pi)
{
	/* Enable the uCode TSSI_DIV WAR */
	if ((ACMAJORREV_1(pi->pubpi->phy_rev) || ACMAJORREV_2(pi->pubpi->phy_rev)) &&
	    BF3_TSSI_DIV_WAR(pi->u.pi_acphy)) {
		wlapi_bmac_mhf(pi->sh->physhim, MHF2, MHF2_PPR_HWPWRCTL, MHF2_PPR_HWPWRCTL,
		               WLC_BAND_ALL);
	}
}

static void
dsi_restore_tables(phy_info_t *pi)
{
	uint8 i = 0;
	uint16 *RFSeq_val = data_RFSeq;

	rfseq_t *RFSeq = addr_RFSeq;
	tbl_t *tbl = tbl_info;

	for (i = 0; i < NUM_PHYTBLS; i++) {
		wlc_phy_table_write_acphy(pi, tbl->id, tbl->len, tbl->offset, tbl->width,
			tbl->data);
		tbl++;
	}

	for (i = 0; i < NUM_RFSEQ; i++) {
		wlc_phy_table_write_acphy(pi, ACPHY_TBL_ID_RFSEQ, RFSeq->len, RFSeq->offset, 16,
			RFSeq_val);
		RFSeq_val += RFSeq->len;
		RFSeq++;
	}
}

static void
dsi_restore_phyregs(phy_info_t *pi)
{
	uint16 i = 0;

	for (i = 0; i < ARRAYSIZE(phyregs_adp); i++) {
		phy_utils_write_phyreg(pi, phyregs_adp[i][ADDR_F], phyregs_adp[i][DATA_F]);
	}
}

static void
dsi_restore_radioregs(phy_info_t *pi)
{
	uint16 i = 0;
	for (i = 0; i < ARRAYSIZE(radioregs_adp); i++) {
		phy_utils_write_radioreg(pi, radioregs_adp[i][ADDR_F], radioregs_adp[i][DATA_F]);
	}
	phy_ac_dsi_radio_fns(pi);
}

static void
dsi_save_tables(phy_info_t *pi)
{
	uint16 i = 0;
	uint16 *RFSeq_val = data_RFSeq;

	tbl_t *tbl = tbl_info;
	rfseq_t *RFSeq = addr_RFSeq;

	sz_TXEVMTBL          = ARRAYSIZE(data_TXEVMTBL);
	sz_GAINLIMIT         = ARRAYSIZE(data_GAINLIMIT);
	sz_ESTPWRSHFTLUTS    = ARRAYSIZE(data_ESTPWRSHFTLUTS);
	sz_NVRXEVMSHAPINGTBL = ARRAYSIZE(data_NVRXEVMSHAPINGTBL);
	sz_IQLOCAL           = ARRAYSIZE(data_IQLOCAL);
	sz_FEMCTRLLUT        = ARRAYSIZE(data_FEMCTRLLUT);
	sz_ESTPWRLUTS0       = ARRAYSIZE(data_ESTPWRLUTS0);
	sz_RFSEQBUNDLE       = ARRAYSIZE(data_RFSEQBUNDLE);
	sz_CHANNELSMOOTHING  = ARRAYSIZE(data_CHANNELSMOOTHING);
	sz_PAPR              = ARRAYSIZE(data_PAPR);
	sz_EPSILON0          = ARRAYSIZE(data_EPSILON0);
	sz_RSSICLIPGAIN0     = ARRAYSIZE(data_RSSICLIPGAIN0);
	sz_PHASETRACKTBL     = ARRAYSIZE(data_PHASETRACKTBL);
	sz_NVNOISESHAPINGTBL = ARRAYSIZE(data_NVNOISESHAPINGTBL);
	sz_RFSEQEXT          = ARRAYSIZE(data_RFSEQEXT);
	sz_RFSeq             = ARRAYSIZE(data_RFSeq);

	for (i = 0; i < NUM_PHYTBLS; i++) {
		wlc_phy_table_read_acphy(pi, tbl->id, tbl->len, tbl->offset, tbl->width, tbl->data);
		tbl++;
	}

	/* Save RFSeq */
	for (i = 0; i < NUM_RFSEQ; i++) {
		wlc_phy_table_read_acphy(pi, ACPHY_TBL_ID_RFSEQ, RFSeq->len, RFSeq->offset, 16,
			RFSeq_val);
		RFSeq_val += RFSeq->len;
		RFSeq++;
	}
}

static void
dsi_save_phyregs(phy_info_t *pi)
{
	uint16 i = 0;
	sz_phyregs = ARRAYSIZE(phyregs_adp);

	for (i = 0; i < sz_phyregs; i++) {
		phyregs_adp[i][DATA_F] = phy_utils_read_phyreg(pi, phyregs_adp[i][ADDR_F]);
	}
}

static void
dsi_save_radioregs(phy_info_t *pi)
{
	uint16 i = 0;
	sz_radioregs = ARRAYSIZE(radioregs_adp);

	for (i = 0; i < sz_radioregs; i++) {
		radioregs_adp[i][DATA_F] = phy_utils_read_radioreg(pi, radioregs_adp[i][ADDR_F]);
	}
}
