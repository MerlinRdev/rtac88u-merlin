/*
 * PHY and RADIO specific portion of Broadcom BCM43XX 802.11abg
 * PHY iovar processing of Broadcom BCM43XX 802.11abg
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
 * $Id: wlc_phy_iovar.c 528118 2015-01-21 08:03:32Z $
 */

/*
 * This file contains high portion PHY iovar processing and table.
 */

#include <wlc_cfg.h>

#ifdef WLC_HIGH
#include <typedefs.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <wlioctl.h>
#include <d11.h>
#include <wlc_rate.h>
#include <wlc_pub.h>
#include <wl_dbg.h>
#include <wlc.h>
#include <bcmwifi_channels.h>
#include <phy_dbg.h>


const bcm_iovar_t phy_iovars[] = {
	/* OLD, PHYTYPE specific iovars, to phase out, use unified ones at the end of this array */
#if NCONF
#if defined(BCMDBG)
	{"nphy_initgain", IOV_NPHY_INITGAIN,
	IOVF_SET_UP, IOVT_UINT16, 0
	},
	{"nphy_hpv1gain", IOV_NPHY_HPVGA1GAIN,
	IOVF_SET_UP, IOVT_INT8, 0
	},
	{"nphy_tx_temp_tone", IOV_NPHY_TX_TEMP_TONE,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
	{"nphy_cal_reset", IOV_NPHY_CAL_RESET,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
	{"nphy_est_tonepwr", IOV_NPHY_EST_TONEPWR,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"phy_est_tonepwr", IOV_PHY_EST_TONEPWR,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"nphy_rfseq_txgain", IOV_NPHY_RFSEQ_TXGAIN,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"phy_spuravoid", IOV_PHY_SPURAVOID,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_INT8, 0
	},
#endif /* defined(BCMDBG) */

#if defined(WLTEST)
	{"nphy_cckpwr_offset", IOV_NPHY_CCK_PWR_OFFSET,
	(IOVF_SET_UP | IOVF_MFG), IOVT_INT8, 0
	},
	{"nphy_cal_sanity", IOV_NPHY_CAL_SANITY,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
	{"nphy_txiqlocal", IOV_NPHY_TXIQLOCAL,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_rxiqcal", IOV_NPHY_RXIQCAL,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_rxcalparams", IOV_NPHY_RXCALPARAMS,
	(0), IOVT_UINT32, 0
	},
	{"nphy_rssisel", IOV_NPHY_RSSISEL,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_rssical", IOV_NPHY_RSSICAL,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_gpiosel", IOV_PHY_GPIOSEL,
	(IOVF_MFG), IOVT_UINT16, 0
	},
	{"nphy_gain_boost", IOV_NPHY_GAIN_BOOST,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_elna_gain_config", IOV_NPHY_ELNA_GAIN_CONFIG,
	(IOVF_SET_DOWN), IOVT_UINT8, 0
	},
	{"nphy_aci_scan", IOV_NPHY_ACI_SCAN,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_pacaltype", IOV_NPHY_PAPDCALTYPE,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_papdcal", IOV_NPHY_PAPDCAL,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_skippapd", IOV_NPHY_SKIPPAPD,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_UINT8, 0
	},
	{"nphy_pacalindex", IOV_NPHY_PAPDCALINDEX,
	(IOVF_MFG), IOVT_UINT16, 0
	},
	{"nphy_caltxgain", IOV_NPHY_CALTXGAIN,
	(IOVF_MFG), IOVT_INT8, 0
	},
	{"nphy_tbldump_minidx", IOV_NPHY_TBLDUMP_MINIDX,
	(IOVF_MFG), IOVT_INT8, 0
	},
	{"nphy_tbldump_maxidx", IOV_NPHY_TBLDUMP_MAXIDX,
	(IOVF_MFG), IOVT_INT8, 0
	},
	{"nphy_phyreg_skipdump", IOV_NPHY_PHYREG_SKIPDUMP,
	(IOVF_MFG), IOVT_UINT16, 0
	},
	{"nphy_phyreg_skipcount", IOV_NPHY_PHYREG_SKIPCNT,
	(IOVF_MFG), IOVT_INT8, 0
	},
#endif 
#endif /* NCONF */

#if defined(WLTEST)

#if LCNCONF
	{"lcnphy_papdepstbl", IOV_LCNPHY_PAPDEPSTBL,
	(IOVF_GET_UP | IOVF_MFG), IOVT_BUFFER, 0
	},
	{"lcnphy_ldovolt", IOV_LCNPHY_LDOVOLT,
	(IOVF_SET_UP), IOVT_UINT32, 0
	},

#endif /* LCNCONF */
#endif 
	/* terminating element, only add new before this */
	{NULL, 0, 0, 0, 0 }
};

const bcm_iovar_t phy_iovars_generic[] = {
#if defined(BCMDBG) || defined(WLTEST)
	{"fast_timer", IOV_FAST_TIMER,
	(IOVF_NTRL | IOVF_MFG), IOVT_UINT32, 0
	},
	{"slow_timer", IOV_SLOW_TIMER,
	(IOVF_NTRL | IOVF_MFG), IOVT_UINT32, 0
	},
#endif /* BCMDBG || WLTEST */
#if defined(BCMDBG) || defined(WLTEST) || defined(PHYCAL_CHNG_CS)
	{"glacial_timer", IOV_GLACIAL_TIMER,
	IOVF_NTRL, IOVT_UINT32, 0
	},
#endif
#if defined(WLTEST) || defined(MACOSX) || defined(DBG_PHY_IOV)
	{"phy_watchdog", IOV_PHY_WATCHDOG,
	(IOVF_MFG), IOVT_UINT8, 0
	},
#endif
	{"cal_period", IOV_CAL_PERIOD,
	0, IOVT_UINT32, 0
	},
#if defined(WLTEST)
#ifdef BAND5G
	{"phy_cga_5g", IOV_PHY_CGA_5G,
	IOVF_SET_UP, IOVT_BUFFER, 24*sizeof(int8)
	},
#endif /* BAND5G */
	{"phy_cga_2g", IOV_PHY_CGA_2G,
	IOVF_SET_UP, IOVT_BUFFER, 14*sizeof(int8)
	},
	{"phymsglevel", IOV_PHYHAL_MSG,
	(0), IOVT_UINT32, 0
	},
	{"phy_fixed_noise", IOV_PHY_FIXED_NOISE,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"phynoise_polling", IOV_PHYNOISE_POLL,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"carrier_suppress", IOV_CARRIER_SUPPRESS,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"pkteng_stats", IOV_PKTENG_STATS,
	(IOVF_GET_UP | IOVF_MFG), IOVT_BUFFER, sizeof(wl_pkteng_stats_t)
	},
#ifdef BAND5G
	{"subband5gver", IOV_PHY_SUBBAND5GVER,
	0, IOVT_INT8, 0
	},
#endif /* BAND5G */
	{"phy_txrx_chain", IOV_PHY_TXRX_CHAIN,
	(0), IOVT_INT8, 0
	},
	{"phy_bphy_evm", IOV_PHY_BPHY_EVM,
	(IOVF_SET_DOWN | IOVF_SET_BAND | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_bphy_rfcs", IOV_PHY_BPHY_RFCS,
	(IOVF_SET_DOWN | IOVF_SET_BAND | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_scraminit", IOV_PHY_SCRAMINIT,
	(IOVF_SET_UP | IOVF_MFG), IOVT_INT8, 0
	},
	{"phy_rfseq", IOV_PHY_RFSEQ,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_tx_tone", IOV_PHY_TX_TONE,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phy_tx_tone_hz", IOV_PHY_TX_TONE_HZ,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phy_tx_tone_stop", IOV_PHY_TX_TONE_STOP,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_test_tssi", IOV_PHY_TEST_TSSI,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT8, 0
	},
	{"phy_test_tssi_offs", IOV_PHY_TEST_TSSI_OFFS,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT8, 0
	},
	{"phy_test_idletssi", IOV_PHY_TEST_IDLETSSI,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT8, 0
	},
	{"phy_setrptbl", IOV_PHY_SETRPTBL,
	(IOVF_SET_UP | IOVF_MFG), IOVT_VOID, 0
	},
	{"phy_forceimpbf", IOV_PHY_FORCEIMPBF,
	(IOVF_SET_UP | IOVF_MFG), IOVT_VOID, 0
	},
	{"phy_forcesteer", IOV_PHY_FORCESTEER,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
#ifdef BAND5G
	{"phy_5g_pwrgain", IOV_PHY_5G_PWRGAIN,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_UINT8, 0
	},
#endif /* BAND5G */
	{"phy_enrxcore", IOV_PHY_ENABLERXCORE,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_activecal", IOV_PHY_ACTIVECAL,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
	{"phy_bbmult", IOV_PHY_BBMULT,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 0
	},
	{"phy_lowpower_beacon_mode", IOV_PHY_LOWPOWER_BEACON_MODE,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
#endif 
#if defined(WLTEST)
	{"pkteng_gainindex", IOV_PKTENG_GAININDEX,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
#endif  
#if defined(WLTEST) || defined(DBG_PHY_IOV) || defined(WFD_PHY_LL_DEBUG) || \
	defined(ATE_BUILD)
	{"phy_forcecal", IOV_PHY_FORCECAL,
	IOVF_SET_UP, IOVT_UINT8, 0
	},
	{"phy_papd_en_war", IOV_PAPD_EN_WAR,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_tx_tone", IOV_PHY_TX_TONE,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phy_txlo_tone", IOV_PHY_TXLO_TONE,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
#ifndef ATE_BUILD
	{"phy_skippapd", IOV_PHY_SKIPPAPD,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_forcecal_obt", IOV_PHY_FORCECAL_OBT,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_forcecal_noise", IOV_PHY_FORCECAL_NOISE,
	(IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, sizeof(uint16)
	},
#endif /* !ATE_BUILD */
#endif 
#if defined(WLTEST) || defined(MACOSX)
	{"phy_deaf", IOV_PHY_DEAF,
	(IOVF_GET_UP | IOVF_SET_UP), IOVT_UINT8, 0
	},
#endif 
#ifdef WLTEST
	{"fem2g", IOV_PHY_FEM2G,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, 0
	},
#ifdef BAND5G
	{"fem5g", IOV_PHY_FEM5G,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, 0
	},
#endif /* BAND5G */
#endif /* WLTEST */
	{"phy_rxiqest", IOV_PHY_RXIQ_EST,
	IOVF_SET_UP, IOVT_UINT32, IOVT_UINT32
	},
	{"phynoise_srom", IOV_PHYNOISE_SROM,
	IOVF_GET_UP, IOVT_UINT32, 0
	},
	{"num_stream", IOV_NUM_STREAM,
	(0), IOVT_INT32, 0
	},
	{"band_range", IOV_BAND_RANGE,
	0, IOVT_INT8, 0
	},
	{"subband_idx", IOV_BAND_RANGE_SUB,
	0, IOVT_INT8, 0
	},
	{"min_txpower", IOV_MIN_TXPOWER,
	0, IOVT_UINT32, 0
	},
	{"ant_diversity_sw_core0", IOV_ANT_DIV_SW_CORE0,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
	{"ant_diversity_sw_core1", IOV_ANT_DIV_SW_CORE1,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
#if defined(WLTEST)
	{"tssivisi_thresh", IOV_TSSIVISI_THRESH,
	0, IOVT_UINT32, 0
	},
#endif 
#if defined(MACOSX)
	{"phywreg_limit", IOV_PHYWREG_LIMIT,
	0, IOVT_UINT32, IOVT_UINT32
	},
#endif 
	{"phy_muted", IOV_PHY_MUTED,
	0, IOVT_UINT8, 0
	},
#if defined(RXDESENS_EN)
	{"phy_rxdesens", IOV_PHY_RXDESENS,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* defined(RXDESENS_EN) */
#if defined(WLMEDIA_N2DEV) || defined(WLMEDIA_N2DBG)
	{"ntd_gds_lowtxpwr", IOV_NTD_GDS_LOWTXPWR,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
	{"papdcal_indexdelta", IOV_PAPDCAL_INDEXDELTA,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
#endif /* defined(WLMEDIA_N2DEV) || defined(WLMEDIA_N2DBG) */
	{"phy_rxantsel", IOV_PHY_RXANTSEL,
	(0), IOVT_UINT8, 0
	},
	{"phy_fcbs_init", IOV_PHY_FCBSINIT,
	(IOVF_SET_UP), IOVT_INT8, 0
	},
	{"phy_fcbs", IOV_PHY_FCBS,
	(IOVF_SET_UP | IOVF_GET_UP), IOVT_UINT8, 0
	},
	{"phy_fcbs_arm", IOV_PHY_FCBSARM,
	(IOVF_SET_UP), IOVT_UINT8, 0
	},
	{"phy_fcbs_exit", IOV_PHY_FCBSEXIT,
	(IOVF_SET_UP), IOVT_UINT8, 0
	},
#if ((ACCONF != 0) || (ACCONF2 != 0) || (NCONF != 0) || (HTCONF != 0) || (LCN40CONF != \
	0))
	{"phy_ed_thresh", IOV_ED_THRESH,
	(IOVF_SET_UP | IOVF_GET_UP), IOVT_INT32, 0
	},
#endif /* ACCONF || ACCONF2 || NCONF || HTCONF || LCN40CONF */
	{"phy_btc_restage_rxgain", IOV_PHY_BTC_RESTAGE_RXGAIN,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
	{"phy_dssf", IOV_PHY_DSSF,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
	{"phy_sarlimit", IOV_PHY_SAR_LIMIT,
	0, IOVT_UINT32, 0
	},
#if defined(WLTEST) || defined(BCMDBG)
	{"phy_txpwr_core", IOV_PHY_TXPWR_CORE,
	0, IOVT_UINT32, 0
	},
#endif /* WLTEST || BCMDBG */
	{"phy_txswctrlmap", IOV_PHY_TXSWCTRLMAP,
	0, IOVT_INT8, 0
	},
	{"phy_wfd_ll_enable", IOV_PHY_WFD_LL_ENABLE,
	0, IOVT_UINT8, 0
	},
#if defined(WLTEST) || defined(BCMDBG)
	{"phy_enable_epa_dpd_2g", IOV_PHY_ENABLE_EPA_DPD_2G,
	IOVF_SET_UP, IOVT_INT8, 0
	},
	{"phy_enable_epa_dpd_5g", IOV_PHY_ENABLE_EPA_DPD_5G,
	IOVF_SET_UP, IOVT_INT8, 0
	},
	{"phy_epacal2gmask", IOV_PHY_EPACAL2GMASK,
	0, IOVT_INT16, 0
	},
#endif /* defined(WLTEST) || defined(BCMDBG) */
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_calib[] = {
#if defined(WLTEST) || defined(ATE_BUILD)
	{"phy_txiqcc", IOV_PHY_TXIQCC,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER,  2*sizeof(int32)
	},
	{"phy_txlocc", IOV_PHY_TXLOCC,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 6
	},
#endif 

//#if defined(BCMDBG) || defined(WLTEST) || defined(MACOSX) || defined(ATE_BUILD)
	{"phy_tempsense", IOV_PHY_TEMPSENSE,
	IOVF_GET_UP, IOVT_INT16, 0
	},
//#endif /* BCMDBG || WLTEST || MACOSX || ATE_BUILD */
#if defined(WLTEST)
	{"phy_cal_disable", IOV_PHY_CAL_DISABLE,
	(IOVF_MFG), IOVT_UINT8, 0
	},
#endif  
#if defined(WLTEST)
	{"phy_vbatsense", IOV_PHY_VBATSENSE,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"phy_idletssi", IOV_PHY_IDLETSSI_REG,
	(IOVF_GET_UP | IOVF_SET_UP), IOVT_BUFFER, 0
	},
	{"phy_tssi", IOV_PHY_AVGTSSI_REG,
	(IOVF_GET_UP), IOVT_BUFFER, 0
	},
	{"phy_resetcca", IOV_PHY_RESETCCA,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_pacalidx0", IOV_PHY_PACALIDX0,
	(IOVF_GET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phy_pacalidx1", IOV_PHY_PACALIDX1,
	(IOVF_GET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phy_iqlocalidx", IOV_PHY_IQLOCALIDX,
	(IOVF_GET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phy_pacalidx", IOV_PHY_PACALIDX,
	(IOVF_GET_UP | IOVF_MFG), IOVT_UINT32, 0
	},
	{"phycal_tempdelta", IOV_PHYCAL_TEMPDELTA,
	(IOVF_MFG), IOVT_UINT8, 0
	},
#endif 
	{"phy_sromtempsense", IOV_PHY_SROM_TEMPSENSE,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT16, 0
	},
	{"rxg_rssi", IOV_PHY_RXGAIN_RSSI,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT16, 0
	},
	{"gain_cal_temp", IOV_PHY_GAIN_CAL_TEMP,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT16, 0
	},
	{"rssi_cal_rev", IOV_PHY_RSSI_CAL_REV,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT16, 0
	},
	{"rud_agc_enable", IOV_PHY_RUD_AGC_ENABLE,
	(IOVF_SET_UP | IOVF_GET_UP | IOVF_MFG), IOVT_INT16, 0
	},
#ifdef PHYMON
	{"phycal_state", IOV_PHYCAL_STATE,
	IOVF_GET_UP, IOVT_UINT32, 0,
	},
#endif /* PHYMON */
#if defined(WLTEST) || defined(AP)
	{"phy_percal", IOV_PHY_PERICAL,
	(IOVF_MFG), IOVT_UINT8, 0
	},
#endif 
	{"phy_percal_delay", IOV_PHY_PERICAL_DELAY,
	(0), IOVT_UINT16, 0
	},
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_aci[] = {
#if defined(WLTEST)
	{"aci_exit_check_period", IOV_ACI_EXIT_CHECK_PERIOD,
	(IOVF_MFG), IOVT_UINT32, 0
	},
#endif 
#if defined(WLTEST)
	{"phy_glitchthrsh", IOV_PHY_GLITCHK,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_noise_up", IOV_PHY_NOISE_UP,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_noise_dwn", IOV_PHY_NOISE_DWN,
	(IOVF_MFG), IOVT_UINT8, 0
	},
#endif /* #if defined(WLTEST) */
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_acphy[] = {
#if defined(BCMDBG) || defined(WLTEST) || defined(PHYCAL_CHNG_CS)
	{"hirssi_period", IOV_HIRSSI_PERIOD,
	(IOVF_MFG), IOVT_INT16, 0
	},
	{"hirssi_en", IOV_HIRSSI_EN,
	0, IOVT_UINT8, 0
	},
	{"hirssi_byp_rssi", IOV_HIRSSI_BYP_RSSI,
	(IOVF_MFG), IOVT_INT8, 0
	},
	{"hirssi_res_rssi", IOV_HIRSSI_RES_RSSI,
	(IOVF_MFG), IOVT_INT8, 0
	},
	{"hirssi_byp_w1cnt", IOV_HIRSSI_BYP_CNT,
	(IOVF_NTRL | IOVF_MFG), IOVT_UINT16, 0
	},
	{"hirssi_res_w1cnt", IOV_HIRSSI_RES_CNT,
	(IOVF_NTRL | IOVF_MFG), IOVT_UINT16, 0
	},
	{"hirssi_status", IOV_HIRSSI_STATUS,
	0, IOVT_UINT8, 0
	},
#endif /*  BCMDBG || WLTEST || PHYCAL_CHNG_CS */
#if defined(WLTEST)
	{"rpcalvars", IOV_RPCALVARS,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, WL_NUM_RPCALVARS * sizeof(wl_rpcal_t)
	},
	{"phy_vcocal", IOV_PHY_VCOCAL,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
#endif 
#if defined(BCMDBG)
	{"phy_force_gainlevel", IOV_PHY_FORCE_GAINLEVEL,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
#endif /* BCMDBG */
#if defined(WLTEST)
	{"phy_force_spurmode", IOV_PHY_FORCE_SPURMODE,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
#endif /* WLTEST */
#if defined(BCMDBG)
	{"phy_force_fdiqi", IOV_PHY_FORCE_FDIQI,
	IOVF_SET_UP, IOVT_UINT32, 0
	},
#endif /* BCMDBG */
	{"phy_force_crsmin", IOV_PHY_FORCE_CRSMIN,
	IOVF_SET_UP, IOVT_BUFFER, 4*sizeof(int8)
	},
#if defined(BCMDBG)
	{"phy_btcoex_desense", IOV_PHY_BTCOEX_DESENSE,
	IOVF_SET_UP, IOVT_INT32, 0
	},
#endif /* BCMDBG */
	{"edcrs", IOV_EDCRS,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
	{"lp_mode", IOV_LP_MODE,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
	{"lp_vco_2g", IOV_LP_VCO_2G,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
#if defined(WLTEST)
	{"smth_enable", IOV_SMTH,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
	{"radio_pd", IOV_RADIO_PD,
	(IOVF_SET_UP|IOVF_GET_UP), IOVT_UINT8, 0
	},
#endif /* WLTEST */
	{"phy_afeoverride", IOV_PHY_AFE_OVERRIDE,
	(0), IOVT_UINT8, 0
	},
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_rssi[] = {
#if defined(WLTEST)
	{"unmod_rssi", IOV_UNMOD_RSSI,
	(IOVF_MFG), IOVT_INT32, 0
	},
#endif 
#if (NCONF || LCN40CONF || ACCONF || ACCONF2) && defined(WLTEST)
	{"phy_rssi_gain_cal_temp", IOV_PHY_RSSI_GAIN_CAL_TEMP,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_INT32, 0
	},
	{"rssi_cal_freq_grp_2g", IOV_PHY_RSSI_CAL_FREQ_GRP_2G,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 7*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2gb0", IOV_PHY_RSSI_GAIN_DELTA_2GB0,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 8*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2gb1", IOV_PHY_RSSI_GAIN_DELTA_2GB1,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 8*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2gb2", IOV_PHY_RSSI_GAIN_DELTA_2GB2,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 8*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2gb3", IOV_PHY_RSSI_GAIN_DELTA_2GB3,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 8*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2gb4", IOV_PHY_RSSI_GAIN_DELTA_2GB4,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 8*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2g", IOV_PHY_RSSI_GAIN_DELTA_2G,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 18*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2gh", IOV_PHY_RSSI_GAIN_DELTA_2GH,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 18*sizeof(int8)
	},
	{"phy_rssi_gain_delta_2ghh", IOV_PHY_RSSI_GAIN_DELTA_2GHH,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 18*sizeof(int8)
	},
	{"phy_rssi_gain_delta_5gl", IOV_PHY_RSSI_GAIN_DELTA_5GL,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 6*sizeof(int8)
	},
	{"phy_rssi_gain_delta_5gml", IOV_PHY_RSSI_GAIN_DELTA_5GML,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 6*sizeof(int8)
	},
	{"phy_rssi_gain_delta_5gmu", IOV_PHY_RSSI_GAIN_DELTA_5GMU,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 6*sizeof(int8)
	},
	{"phy_rssi_gain_delta_5gh", IOV_PHY_RSSI_GAIN_DELTA_5GH,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 6*sizeof(int8)
	},
	{"phy_rxgainerr_2g", IOV_PHY_RXGAINERR_2G,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 2*sizeof(int8)
	},
	{"phy_rxgainerr_5gl", IOV_PHY_RXGAINERR_5GL,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 2*sizeof(int8)
	},
	{"phy_rxgainerr_5gm", IOV_PHY_RXGAINERR_5GM,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 2*sizeof(int8)
	},
	{"phy_rxgainerr_5gh", IOV_PHY_RXGAINERR_5GH,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 2*sizeof(int8)
	},
	{"phy_rxgainerr_5gu", IOV_PHY_RXGAINERR_5GU,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 2*sizeof(int8)
	},
#endif /* (NCONF || LCN40CONF || ACCONF || ACCONF2) && WLTEST */
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_nphy[] = {
	{"phy_oclscdenable", IOV_PHY_OCLSCDENABLE,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"lnldo2", IOV_LNLDO2,
	(IOVF_MFG), IOVT_UINT8, 0
	},
#if defined(WLTEST) || defined(DBG_PHY_IOV)
	{"phy_dynamic_ml", IOV_PHY_DYN_ML,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
	{"aci_nams", IOV_PHY_ACI_NAMS,
	(IOVF_SET_UP | IOVF_MFG), IOVT_UINT8, 0
	},
#endif 
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_lcncmnphy[] = {
#if defined(WLTEST)
	{"phy_auxpga", IOV_PHY_AUXPGA,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, 6*sizeof(uint8)
	},
#endif 
#if defined(WLTEST)
#if defined(LCNCONF) || defined(LCN40CONF)
	{"lcnphy_rxiqgain", IOV_LCNPHY_RXIQGAIN,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"lcnphy_rxiqgspower", IOV_LCNPHY_RXIQGSPOWER,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"lcnphy_rxiqpower", IOV_LCNPHY_RXIQPOWER,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"lcnphy_rxiqstatus", IOV_LCNPHY_RXIQSTATUS,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"lcnphy_rxiqsteps", IOV_LCNPHY_RXIQSTEPS,
	IOVF_SET_UP, IOVT_UINT8, 0
	},
	{"lcnphy_tssimaxpwr", IOV_LCNPHY_TSSI_MAXPWR,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"lcnphy_tssiminpwr", IOV_LCNPHY_TSSI_MINPWR,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* #if defined(LCNCONF) || defined(LCN40CONF) */
#if LCN40CONF
	{"lcnphy_txclampdis", IOV_LCNPHY_TXPWRCLAMP_DIS,
	IOVF_GET_UP, IOVT_UINT8, 0
	},
	{"lcnphy_txclampofdm", IOV_LCNPHY_TXPWRCLAMP_OFDM,
	IOVF_GET_UP, IOVT_INT32, 0
	},
#endif /* LCN40CONF */
#endif /* #if defined(WLTEST) */
#if defined(BCMDBG)
	{"lcnphy_txclampcck", IOV_LCNPHY_TXPWRCLAMP_CCK,
	IOVF_GET_UP, IOVT_INT32, 0
	},
	{"lcnphy_cwtxpwrctrl", IOV_LCNPHY_CWTXPWRCTRL,
	IOVF_MFG, IOVT_UINT8, 0
	},
#endif
	{"phy_crs_war", IOV_PHY_CRS_WAR,
	(0), IOVT_INT8, 0
	},
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_txpwrctl[] = {
#if defined(BCMDBG) || defined(WLTEST)
	{"txinstpwr", IOV_TXINSTPWR,
	(IOVF_GET_CLK | IOVF_GET_BAND | IOVF_MFG), IOVT_BUFFER, sizeof(tx_inst_power_t)
	},
#endif 
#if defined(BCMDBG) || defined(WLTEST)
	{"tssical_start_idx", IOV_TSSICAL_START_IDX,
	(IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, sizeof(int)
	},
	{"tssical_start", IOV_TSSICAL_START,
	(IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, sizeof(int)
	},
	{"tssical_power", IOV_TSSICAL_POWER,
	(IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, sizeof(int)
	},
	{"tssical_params", IOV_TSSICAL_PARAMS,
	(IOVF_GET_UP | IOVF_MFG), IOVT_BUFFER, 4*sizeof(int64)
	},
	{"tssical_txdelay", IOV_PHY_TSSITXDELAY,
	(IOVF_SET_UP | IOVF_GET_UP), IOVT_UINT32, 0
	},
#endif /* BCMDBG || WLTEST */

#if defined(WLTEST) || defined(ATE_BUILD)
	{"phy_txpwrctrl", IOV_PHY_TXPWRCTRL,
	(IOVF_MFG), IOVT_UINT8, 0
	},
	{"phy_txpwrindex", IOV_PHY_TXPWRINDEX,
	(IOVF_GET_UP | IOVF_SET_UP | IOVF_MFG), IOVT_BUFFER, 0
	},
	{"phy_tone_txpwr", IOV_PHY_TONE_TXPWR,
	(IOVF_SET_UP), IOVT_INT8, 0
	},
#endif 
#if defined(WLTEST)
	{"patrim", IOV_PATRIM,
	(IOVF_MFG), IOVT_INT32, 0
	},
	{"pavars", IOV_PAVARS,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, WL_PHY_PAVARS_LEN * sizeof(uint16)
	},
	{"pavars2", IOV_PAVARS2,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, sizeof(wl_pavars2_t)
	},
	{"povars", IOV_POVARS,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, sizeof(wl_po_t)
	},
#endif 
	{"sromrev", IOV_SROM_REV,
	(IOVF_SET_DOWN), IOVT_UINT8, 0
	},
#ifdef WLTEST
	{"maxpower", IOV_PHY_MAXP,
	(IOVF_SET_DOWN | IOVF_MFG), IOVT_BUFFER, 0
	},
#endif /* WLTEST */
	{NULL, 0, 0, 0, 0 }
};
const bcm_iovar_t phy_iovars_sc[] = {
#ifdef SAMPLE_COLLECT
	{"sample_collect", IOV_PHY_SAMPLE_COLLECT,
	(IOVF_GET_CLK), IOVT_BUFFER, WLC_SAMPLECOLLECT_MAXLEN
	},
	{"sample_data", IOV_PHY_SAMPLE_DATA,
	(IOVF_GET_CLK), IOVT_BUFFER, WLC_SAMPLECOLLECT_MAXLEN
	},
	{"sample_collect_gainadj", IOV_PHY_SAMPLE_COLLECT_GAIN_ADJUST,
	0, IOVT_INT8, 0
	},
	{"mac_triggered_sample_collect", IOV_PHY_MAC_TRIGGERED_SAMPLE_COLLECT,
	0, IOVT_BUFFER, WLC_SAMPLECOLLECT_MAXLEN
	},
	{"mac_triggered_sample_data", IOV_PHY_MAC_TRIGGERED_SAMPLE_DATA,
	0, IOVT_BUFFER, WLC_SAMPLECOLLECT_MAXLEN
	},
	{"sample_collect_gainidx", IOV_PHY_SAMPLE_COLLECT_GAIN_INDEX,
	0, IOVT_UINT8, 0
	},
	{"iq_metric_data", IOV_IQ_IMBALANCE_METRIC_DATA,
	(IOVF_GET_DOWN | IOVF_GET_CLK), IOVT_BUFFER, WLC_SAMPLECOLLECT_MAXLEN
	},
	{"iq_metric", IOV_IQ_IMBALANCE_METRIC,
	(IOVF_GET_DOWN | IOVF_GET_CLK), IOVT_BUFFER, 0
	},
	{"iq_metric_pass", IOV_IQ_IMBALANCE_METRIC_PASS,
	(IOVF_GET_DOWN | IOVF_GET_CLK), IOVT_BUFFER, 0
	},
#endif /* sample_collect */
	{NULL, 0, 0, 0, 0 }
};
#endif /* WLC_HIGH */

#ifdef WLC_LOW
#include <typedefs.h>
/* *********************************************** */
#include <phy_dbg.h>
/* *********************************************** */
#include <bcmdefs.h>
#include <wlc_phy_hal.h>
#include <wlc_phy_int.h>
#include <wlc_phyreg_n.h>
#include <wlc_phyreg_ht.h>
#include <wlc_phyreg_ac.h>
#include <wlc_phyreg_lcn.h>
#include <wlc_phyreg_lcn40.h>
#include <wlc_phytbl_n.h>
#include <wlc_phytbl_ht.h>
#include <wlc_phytbl_ac.h>
#include <wlc_phytbl_20691.h>
#include <wlc_phytbl_20693.h>
#include <wlc_phy_radio.h>
#include <wlc_phy_lcn.h>
#include <wlc_phy_lcn40.h>
#include <wlc_phy_n.h>
#include <wlc_phy_ht.h>
#if (ACCONF != 0) || (ACCONF2 != 0)
#include <wlc_phy_ac.h>
#endif /* (ACCONF != 0) || (ACCONF2 != 0) */
#include <bcmwifi_channels.h>
#include <bcmotp.h>
#ifdef WLSRVSDB
#include <saverestore.h>
#endif
#include <phy_utils_math.h>
#include <phy_utils_var.h>
#include <phy_utils_status.h>
#include <phy_utils_reg.h>
#include <phy_utils_channel.h>
#include <phy_utils_api.h>

#define RSSI_IQEST_DEBUG 0
#define RSSI_CORR_EN 1

#if defined(WLTEST)
#define BFECONFIGREF_FORCEVAL    0x9
#define BFMCON_FORCEVAL          0x8c03
#define BFMCON_RELEASEVAL        0x8c1d
#define REFRESH_THR_FORCEVAL     0xffff
#define REFRESH_THR_RELEASEVAL   0x186a
#define BFRIDX_POS_FORCEVAL      0x100
#define BFRIDX_POS_RELEASEVAL    0x0
#endif 

#define LCN40_RX_GAIN_INDEX_MASK	0x7F00
#define LCN40_RX_GAIN_INDEX_SHIFT	8
#define LCN40_QDB_MASK	0x3
#define LCN40_QDB_SHIFT	2

int phy_legacy_register_iovt(phy_info_t *pi, wlc_iocv_info_t *ii);
#if !defined(EFI)
static void ratmodel_paparams_fix64(ratmodel_paparams_t* rsd, int m);
#if PHY_TSSI_CAL_DBG_EN
static void print_int64(int64 *a);
#endif
static uint16 tssi_cal_sweep(phy_info_t *pi);
int wlc_phy_tssi_cal(phy_info_t *pi);
#else
#define wlc_phy_tssi_cal(a)	do {} while (0)
#endif /* #if !defined(EFI) */
/* Modularise and clean up attach functions */
#if ((ACCONF != 0) || (ACCONF2 != 0) || (NCONF != 0) || (HTCONF != 0) || (LCN40CONF != \
	0))
static int
wlc_phy_adjust_ed_thres(phy_info_t *pi, int32 *assert_thresh_dbm, bool set_threshold);
#endif /* ((ACCONF != 0) || (ACCONF2 != 0) || (NCONF != 0) || (HTCONF != 0) || (LCN40CONF != 0)) */
static void wlc_phy_cal_perical_mphase_schedule(phy_info_t *pi, uint delay);
static int wlc_phy_iovar_dispatch_old(phy_info_t *pi, uint32 actionid, void *p, void *a, int vsize,
	int32 int_val, bool bool_val);
static int wlc_phy_iovars_phy_specific(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_nphy(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_lcncmnphy(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_acphy(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_calib(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_generic(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_aci(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_rssi(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);
static int wlc_phy_iovars_txpwrctl(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);


#if defined(WLTEST) || defined(AP)
static int wlc_phy_iovar_perical_config(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr,
		bool set);
#endif
static int wlc_phy_iovar_set_btc_restage_rxgain(phy_info_t *pi, int32 set_val);
static int wlc_phy_iovar_get_btc_restage_rxgain(phy_info_t *pi, int32 *ret_val);
static int wlc_phy_iovar_set_dssf(phy_info_t *pi, int32 set_val);
static int wlc_phy_iovar_get_dssf(phy_info_t *pi, int32 *ret_val);
#if defined(BCMDBG) || defined(WLTEST) || defined(MACOSX) || defined(ATE_BUILD)
static int
wlc_phy_iovar_tempsense_paldosense(phy_info_t *pi, int32 *ret_int_ptr, uint8 tempsense_paldosense);
#endif
#if defined(WLTEST)
static int wlc_phy_iovar_idletssi(phy_info_t *pi, int32 *ret_int_ptr, bool type);
static int
wlc_phy_iovar_bbmult_get(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr);
static int wlc_phy_iovar_bbmult_set(phy_info_t *pi, void *p);
static int wlc_phy_iovar_vbatsense(phy_info_t *pi, int32 *ret_int_ptr);
#endif 
#if defined(WLTEST)
static int wlc_phy_iovar_idletssi_reg(phy_info_t *pi, int32 *ret_int_ptr, int32 int_val, bool set);
static int wlc_phy_iovar_avgtssi_reg(phy_info_t *pi, int32 *ret_int_ptr);
#endif 
#if defined(WLTEST)
static int wlc_phy_iovar_txrx_chain(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, bool set);
static void wlc_phy_iovar_bphy_testpattern(phy_info_t *pi, uint8 testpattern, bool enable);
static void wlc_phy_iovar_scraminit(phy_info_t *pi, int8 scraminit);
static void wlc_phy_iovar_force_rfseq(phy_info_t *pi, uint8 int_val);
static void wlc_phy_iovar_tx_tone_hz(phy_info_t *pi, int32 int_val);
static void wlc_phy_iovar_tx_tone_stop(phy_info_t *pi);
static int16 wlc_phy_iovar_test_tssi(phy_info_t *pi, uint8 val, uint8 pwroffset);
static int16 wlc_phy_iovar_test_idletssi(phy_info_t *pi, uint8 val);
static int16 wlc_phy_iovar_setrptbl(phy_info_t *pi);
static int16 wlc_phy_iovar_forceimpbf(phy_info_t *pi);
static int16 wlc_phy_iovar_forcesteer(phy_info_t *pi, uint8 enable);
static void
wlc_phy_iovar_rxcore_enable(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr,
	bool set);
#endif 
#if defined(WLTEST) || defined(MACOSX)
static void wlc_phy_iovar_set_deaf(phy_info_t *pi, int32 int_val);
static int wlc_phy_iovar_get_deaf(phy_info_t *pi, int32 *ret_int_ptr);
#endif 
#if defined(WLTEST) || defined(DBG_PHY_IOV) || defined(WFD_PHY_LL_DEBUG) || \
	defined(ATE_BUILD)
static int
wlc_phy_iovar_forcecal(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, int vsize, bool set);
#ifndef ATE_BUILD
static int
wlc_phy_iovar_forcecal_obt(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, int vsize, bool set);
static int
wlc_phy_iovar_forcecal_noise(phy_info_t *pi, int32 int_val, void *a, int vsize, bool set);
#endif /* !ATE_BUILD */
#endif 
#if defined(WLTEST) || defined(ATE_BUILD)
static int
wlc_phy_iovar_txpwrctrl(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr,
	bool set);
static int
wlc_phy_iovar_txpwrindex_get(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr);
static int wlc_phy_iovar_txpwrindex_set(phy_info_t *pi, void *p);
static void wlc_phy_iovar_tx_tone(phy_info_t *pi, int32 int_val);
static void wlc_phy_iovar_txlo_tone(phy_info_t *pi);
#endif 
#if defined(WLTEST)
static int wlc_phy_pkteng_get_gainindex(phy_info_t *pi, int32 *gain_idx);
void wlc_phy_pkteng_rxstats_update(wlc_phy_t *ppi, uint8 statidx);
static int wlc_phy_pkteng_stats_get(phy_info_t *pi, void *a, int alen);
#endif 


#include <wlc_patch.h>


static int
phy_legacy_doiovar(void *ctx, uint32 aid,
	void *p, uint plen, void *a, uint alen, uint vsize)
{
	return wlc_phy_iovar_dispatch((phy_info_t *)ctx, aid, p, plen, a, (int)alen, (int)vsize);
}

#if defined(BCMDBG) || defined(WLTEST)
/* Return the current instantaneous est. power
 * For swpwr ctrl it's based on current TSSI value (as opposed to average)
 * Mainly used by mfg.
 */
static void
wlc_phy_txpower_get_instant(phy_info_t *pi, void *pwr)
{
	tx_inst_power_t *power = (tx_inst_power_t *)pwr;
	/* If sw power control, grab the instant value based on current TSSI Only
	 * If hw based, read the hw based estimate in realtime
	 */
	if (ISLCNPHY(pi)) {
		if (!pi->hwpwrctrl)
			return;

		wlc_lcnphy_get_tssi(pi, (int8*)&power->txpwr_est_Pout_gofdm,
			(int8*)&power->txpwr_est_Pout[0]);
		power->txpwr_est_Pout[1] = power->txpwr_est_Pout_gofdm;

	} else if (ISLCN40PHY(pi)) {
		if (!pi->hwpwrctrl)
			return;

		wlc_lcn40phy_get_tssi(pi, (int8*)&power->txpwr_est_Pout_gofdm,
			(int8*)&power->txpwr_est_Pout[0]);
		power->txpwr_est_Pout[1] = power->txpwr_est_Pout_gofdm;

	}

}
#endif 

#if defined(WLTEST)
static int
wlc_phy_pkteng_get_gainindex(phy_info_t *pi, int32 *gain_idx)
{
	int i;

	if (D11REV_LT(pi->sh->corerev, 11))
		return BCME_UNSUPPORTED;

	if (!pi->sh->up) {
		return BCME_NOTUP;
	}

	PHY_INFORM(("wlc_phy_pkteng_get_gainindex Called\n"));

	if (ISLCNCOMMONPHY(pi)) {
		uint8 gidx[4];
		uint16 rssi_addr[4];

		uint16 lcnphyregs_shm_addr =
			2 * wlapi_bmac_read_shm(pi->sh->physhim, M_LCN40PHYREGS_PTR);

		rssi_addr[0] = lcnphyregs_shm_addr + M_LCN_RSSI_0;
		rssi_addr[1] = lcnphyregs_shm_addr + M_LCN_RSSI_1;
		rssi_addr[2] = lcnphyregs_shm_addr + M_LCN_RSSI_2;
		rssi_addr[3] = lcnphyregs_shm_addr + M_LCN_RSSI_3;

		for (i = 0; i < 4; i++) {
			gidx[i] =
				(wlapi_bmac_read_shm(pi->sh->physhim, rssi_addr[i])
				& LCN40_RX_GAIN_INDEX_MASK) >> LCN40_RX_GAIN_INDEX_SHIFT;

		}
		*gain_idx = (int32)gidx[0];
	}

	return BCME_OK;
}

void
wlc_phy_pkteng_rxstats_update(wlc_phy_t *ppi, uint8 statidx)
{
	phy_info_t *pi;
	pi = (phy_info_t*)ppi;

	if (ISACPHY(pi) && ((pi->measure_hold & PHY_HOLD_FOR_PKT_ENG)))
		pi->u.pi_acphy->rxstats[statidx] += 1;
}

static int
wlc_phy_pkteng_stats_get(phy_info_t *pi, void *a, int alen)
{
	wl_pkteng_stats_t stats;
	uint16 rxstats_base;
	uint16 hi, lo;
	int i;

	if (D11REV_LT(pi->sh->corerev, 11))
		return BCME_UNSUPPORTED;

	if (!pi->sh->up) {
		return BCME_NOTUP;
	}

	PHY_INFORM(("Pkteng Stats Called\n"));

	/* Read with guard against carry */
	do {
		hi = wlapi_bmac_read_shm(pi->sh->physhim, M_PKTENG_FRMCNT_HI);
		lo = wlapi_bmac_read_shm(pi->sh->physhim, M_PKTENG_FRMCNT_LO);
	} while (hi != wlapi_bmac_read_shm(pi->sh->physhim, M_PKTENG_FRMCNT_HI));

	stats.lostfrmcnt = (hi << 16) | lo;
	stats.rssi_qdb = 0;

	if (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi)) {
		if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 4) &&
			(!CHIPID_4324X_MEDIA_FAMILY(pi))) {
			int16 rxpwr0, rxpwr1;
			rxpwr0 = R_REG(pi->sh->osh, &pi->regs->rssi) & 0xff;
			rxpwr1 = (R_REG(pi->sh->osh, &pi->regs->rssi) >> 8) & 0xff;
			/* Sign extend */
			if (rxpwr0 > 127)
				rxpwr0 -= 256;
			if (rxpwr1 > 127)
				rxpwr1 -= 256;
			stats.rssi = wlc_phy_swrssi_compute_nphy(pi, &rxpwr0, &rxpwr1);
		} else if (ISACPHY(pi)) {
		  #ifdef WL11AC
			 stats.rssi = (R_REG(pi->sh->osh,
			 &pi->regs->u_rcv.d11regs.rxe_phyrs_2) >> 8) & 0xff;
			if (stats.rssi > 127)
				stats.rssi -= 256;
		  #endif

			if (ACMAJORREV_2(pi->pubpi->phy_rev) &&
				(ACMINORREV_1(pi) || ACMINORREV_3(pi))) {
				stats.rssi = pi->u.pi_acphy->last_rssi;
			}
		} else {
			stats.rssi = R_REG(pi->sh->osh, &pi->regs->rssi) & 0xff;
			if (stats.rssi > 127)
				stats.rssi -= 256;
		}
		stats.snr = stats.rssi - PHY_NOISE_FIXED_VAL_NPHY;
	} else if (ISLCNCOMMONPHY(pi)) {
		int16 rssi_lcn[4];
		int16 snr_a_lcn[4];
		int16 snr_b_lcn[4];
		uint8 gidx[4];
		int8 snr[4];
		int8 snr_lcn[4];
		phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
		phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
		uint16 rssi_addr[4], snr_a_addr[4], snr_b_addr[4];
		uint16 lcnphyregs_shm_addr =
			2 * wlapi_bmac_read_shm(pi->sh->physhim, M_LCN40PHYREGS_PTR);
		int16 rxpath_gain;
		uint16 board_atten = 0;
		int16 rssi_lcn40_qdb[4];
		uint16 rssi_qdb_addr[4];
		uint8 gain_resp[4];
		uint8 gidx_calc[4];

#if RSSI_IQEST_DEBUG
		uint16 rssi_iqpwr;
		int16 rssi_iqpwr_dB;
		int wait_count = 0;
		uint16 board_atten_dbg;

		while (wlapi_bmac_read_shm(pi->sh->physhim, lcnphyregs_shm_addr + M_RSSI_LOCK)) {
			/* Check for timeout */
			if (wait_count > 100) { /* 1 ms */
				PHY_ERROR(("wl%d: %s: Unable to get RSSI_LOCK\n",
					pi->sh->unit, __FUNCTION__));
				goto iqest_rssi_skip;
			}
			OSL_DELAY(10);
			wait_count++;
		}
		wlapi_bmac_write_shm(pi->sh->physhim, lcnphyregs_shm_addr + M_RSSI_LOCK, 1);
		rssi_iqpwr = wlapi_bmac_read_shm(pi->sh->physhim,
			lcnphyregs_shm_addr + M_RSSI_IQPWR_DBG);
		rssi_iqpwr_dB = wlapi_bmac_read_shm(pi->sh->physhim,
			lcnphyregs_shm_addr + M_RSSI_IQPWR_DB_DBG);
		board_atten_dbg = wlapi_bmac_read_shm(pi->sh->physhim,
			lcnphyregs_shm_addr + M_RSSI_BOARDATTEN_DBG);
		wlapi_bmac_write_shm(pi->sh->physhim, lcnphyregs_shm_addr + M_RSSI_LOCK, 0);
		printf("iqpwr = %d, iqpwr_dB = %d, board_atten = %d\n",
			rssi_iqpwr, rssi_iqpwr_dB, board_atten_dbg);
iqest_rssi_skip:
#endif /* RSSI_IQEST_DEBUG */

		rssi_addr[0] = lcnphyregs_shm_addr + M_LCN_RSSI_0;
		rssi_addr[1] = lcnphyregs_shm_addr + M_LCN_RSSI_1;
		rssi_addr[2] = lcnphyregs_shm_addr + M_LCN_RSSI_2;
		rssi_addr[3] = lcnphyregs_shm_addr + M_LCN_RSSI_3;

		rssi_qdb_addr[0] = lcnphyregs_shm_addr + M_LCN40_RSSI_QDB_0;
		rssi_qdb_addr[1] = lcnphyregs_shm_addr + M_LCN40_RSSI_QDB_1;
		rssi_qdb_addr[2] = lcnphyregs_shm_addr + M_LCN40_RSSI_QDB_2;
		rssi_qdb_addr[3] = lcnphyregs_shm_addr + M_LCN40_RSSI_QDB_3;

		stats.rssi = 0;

		for (i = 0; i < 4; i++) {
			rssi_lcn[i] = (int8)(wlapi_bmac_read_shm(pi->sh->physhim,
				rssi_addr[i]) & 0xFF);
			snr_lcn[i] = (int8)((wlapi_bmac_read_shm(pi->sh->physhim,
				rssi_addr[i]) >> 8) & 0xFF);
			BCM_REFERENCE(snr_lcn[i]);
			gidx[i] =
				(wlapi_bmac_read_shm(pi->sh->physhim, rssi_addr[i])
				& LCN40_RX_GAIN_INDEX_MASK) >> LCN40_RX_GAIN_INDEX_SHIFT;

			if (ISLCNPHY(pi)) {
				rssi_lcn[i] +=
				        phy_lcn_get_pkt_rssi_gain_index_offset(gidx[i]);
				if ((wlapi_bmac_read_shm(pi->sh->physhim, rssi_addr[i])) & 0x8000) {
					rssi_lcn[i] = rssi_lcn[i] +
						pi->rssi_corr_boardatten;
				} else
					rssi_lcn[i] = rssi_lcn[i] + pi->rssi_corr_normal;

				rssi_lcn[i] = rssi_lcn[i] << 2;
			} else {
				int8 *corr2g;
#ifdef BAND5G
				int8 *corr5g;
#endif /* BAND5G */
				int8 *corrperrg;
				int8 po_reg;
				int16 po_nom;
				uint8 tr_iso = 0;


				if (pi_lcn40->rssi_iqest_en) {
					board_atten = wlapi_bmac_read_shm(pi->sh->physhim,
					rssi_addr[i]) >> 15;

					gain_resp[i] =
						wlapi_bmac_read_shm(pi->sh->physhim,
						rssi_qdb_addr[i]) >> 2;
					gidx_calc[i] = ((gain_resp[i] + 18)* 85) >> 8;
					if (gidx[i] > 37)
						gidx_calc[i] = gidx_calc[i] + 38;
					if (CHSPEC_IS2G(pi->radio_chanspec))
						tr_iso = pi_lcn->lcnphy_tr_isolation_mid;
#ifdef BAND5G
					else
						tr_iso = pi_lcn->triso5g[0];
#endif
					if (board_atten) {
						gidx_calc[i] = gidx_calc[i] + tr_iso;
					}
					gidx[i] = gidx_calc[i];

					rxpath_gain =
						wlc_lcn40phy_get_rxpath_gain_by_index(pi,
						gidx[i], board_atten);
					PHY_INFORM(("iqdB= %d, boardattn= %d, rxpath_gain= %d, "
						"gidx = %d, rssi_iqest_gain_adj = %d\n",
						rssi_lcn[i], board_atten, rxpath_gain, gidx[i],
						pi_lcn40->rssi_iqest_gain_adj));
					rssi_lcn40_qdb[i] =
						wlapi_bmac_read_shm(pi->sh->physhim,
						rssi_qdb_addr[i]) & 0x3;
					rssi_lcn[i] = (rssi_lcn[i] << LCN40_QDB_SHIFT)
						+ rssi_lcn40_qdb[i] - rxpath_gain +
						(pi_lcn40->rssi_iqest_gain_adj
						<< LCN40_QDB_SHIFT);
				}

#if RSSI_CORR_EN
			if (!pi_lcn40->rssi_iqest_en) {
				/* JSSI adjustment wrt power offset */
				if (CHSPEC_IS20(pi->radio_chanspec))
					po_reg =
					PHY_REG_READ(pi, LCN40PHY,
					SignalBlockConfigTable6_new,
					crssignalblk_input_pwr_offset_db);
				else
					po_reg =
					PHY_REG_READ(pi, LCN40PHY,
					SignalBlockConfigTable5_new,
					crssignalblk_input_pwr_offset_db_40mhz);

				switch (wlc_phy_chanspec_bandrange_get(pi, pi->radio_chanspec)) {
				case WL_CHAN_FREQ_RANGE_2G:
					if (CHSPEC_IS20(pi->radio_chanspec))
						po_nom = pi_lcn->noise.nvram_input_pwr_offset_2g;
					else
						po_nom = pi_lcn->noise.nvram_input_pwr_offset_40_2g;
					break;
			#ifdef BAND5G
				case WL_CHAN_FREQ_RANGE_5GL:
					/* 5 GHz low */
					if (CHSPEC_IS20(pi->radio_chanspec))
						po_nom = pi_lcn->noise.nvram_input_pwr_offset_5g[0];
					else
						po_nom =
						pi_lcn->noise.nvram_input_pwr_offset_40_5g[0];
					break;
				case WL_CHAN_FREQ_RANGE_5GM:
					/* 5 GHz middle */
					if (CHSPEC_IS20(pi->radio_chanspec))
						po_nom = pi_lcn->noise.nvram_input_pwr_offset_5g[1];
					else
						po_nom =
						pi_lcn->noise.nvram_input_pwr_offset_40_5g[1];
					break;
				case WL_CHAN_FREQ_RANGE_5GH:
					/* 5 GHz high */
					if (CHSPEC_IS20(pi->radio_chanspec))
						po_nom = pi_lcn->noise.nvram_input_pwr_offset_5g[2];
					else
						po_nom =
						pi_lcn->noise.nvram_input_pwr_offset_40_5g[2];
					break;
			#endif /* BAND5G */
				default:
					po_nom = po_reg;
					break;
				}

				rssi_lcn[i] += ((po_nom - po_reg) << LCN40_QDB_SHIFT);

				/* RSSI adjustment and Adding the JSSI range specific corrections */
				#ifdef BAND5G
				if (wlc_phy_chanspec_bandrange_get(pi, pi->radio_chanspec) !=
					WL_CHAN_FREQ_RANGE_2G) {
						if (((rssi_lcn[i] >> LCN40_QDB_SHIFT) < -60) &&
							((gidx[i] > 0) && (gidx[i] < 38)))
					    rssi_lcn[i] +=
					      (phy_lcn40_get_pkt_rssi_gain_index_offset_5g(gidx[i])
					            << LCN40_QDB_SHIFT);
						corrperrg = pi->rssi_corr_perrg_5g;
					} else
				#endif /* BAND5G */
					{
						if (((rssi_lcn[i] >> LCN40_QDB_SHIFT) < -60) &&
							((gidx[i] > 0) && (gidx[i] < 38)))
					    rssi_lcn[i] +=
					      (phy_lcn40_get_pkt_rssi_gain_index_offset_2g(gidx[i])
					            << LCN40_QDB_SHIFT);
						corrperrg = pi->rssi_corr_perrg_2g;
					}

					if ((rssi_lcn[i] << LCN40_QDB_SHIFT) <= corrperrg[0])
						rssi_lcn[i] += (corrperrg[2] << LCN40_QDB_SHIFT);
					else if ((rssi_lcn[i] << LCN40_QDB_SHIFT) <= corrperrg[1])
						rssi_lcn[i] += (corrperrg[3] << LCN40_QDB_SHIFT);
					else
						rssi_lcn[i] += (corrperrg[4] << LCN40_QDB_SHIFT);

					corr2g = &(pi->rssi_corr_normal);
#ifdef BAND5G
					corr5g = &(pi->rssi_corr_normal_5g[0]);
#endif /* BAND5G */

				switch (wlc_phy_chanspec_bandrange_get(pi, pi->radio_chanspec)) {
						case WL_CHAN_FREQ_RANGE_2G:
							rssi_lcn[i] = rssi_lcn[i] +
								(*corr2g  << LCN40_QDB_SHIFT);
							break;
					#ifdef BAND5G
						case WL_CHAN_FREQ_RANGE_5GL:
							/* 5 GHz low */
							rssi_lcn[i] = rssi_lcn[i] +
							(corr5g[0] << LCN40_QDB_SHIFT);
							break;

						case WL_CHAN_FREQ_RANGE_5GM:
							/* 5 GHz middle */
							rssi_lcn[i] = rssi_lcn[i] +
							(corr5g[1] << LCN40_QDB_SHIFT);
							break;

						case WL_CHAN_FREQ_RANGE_5GH:
							/* 5 GHz high */
							rssi_lcn[i] = rssi_lcn[i] +
							(corr5g[2] << LCN40_QDB_SHIFT);
							break;
					#endif /* BAND5G */
						default:
							rssi_lcn[i] = rssi_lcn[i] + 0;
							break;
					}
				}

				/* Temp sense based correction */
				rssi_lcn[i] += wlc_lcn40phy_rssi_tempcorr(pi, 0);
				if (pi_lcn40->rssi_iqest_en)
					rssi_lcn[i] +=
					wlc_lcn40phy_iqest_rssi_tempcorr(pi, 0, board_atten);

#endif /* RSSI_CORR_EN */
			}
			stats.rssi += rssi_lcn[i];
		}

		stats.rssi = stats.rssi >> 2;

		/* temperature compensation */
		stats.rssi = stats.rssi + (pi_lcn->lcnphy_pkteng_rssi_slope << LCN40_QDB_SHIFT);

		/* convert into dB and save qdB portion */
		stats.rssi_qdb = stats.rssi & LCN40_QDB_MASK;
		stats.rssi = stats.rssi >> LCN40_QDB_SHIFT;

		/* SNR */
		snr_a_addr[0] = lcnphyregs_shm_addr + M_LCN_SNR_A_0;
		snr_a_addr[1] = lcnphyregs_shm_addr + M_LCN_SNR_A_1;
		snr_a_addr[2] = lcnphyregs_shm_addr + M_LCN_SNR_A_2;
		snr_a_addr[3] = lcnphyregs_shm_addr + M_LCN_SNR_A_3;

		snr_b_addr[0] = lcnphyregs_shm_addr + M_LCN_SNR_B_0;
		snr_b_addr[1] = lcnphyregs_shm_addr + M_LCN_SNR_B_1;
		snr_b_addr[2] = lcnphyregs_shm_addr + M_LCN_SNR_B_2;
		snr_b_addr[3] = lcnphyregs_shm_addr + M_LCN_SNR_B_3;

		stats.snr = 0;
		for (i = 0; i < 4; i++) {
			snr_a_lcn[i] = wlapi_bmac_read_shm(pi->sh->physhim, snr_a_addr[i]);
			snr_b_lcn[i] = wlapi_bmac_read_shm(pi->sh->physhim, snr_b_addr[i]);
			snr[i] = ((snr_a_lcn[i] - snr_b_lcn[i])* 3) >> 5;
			if (snr[i] > 31)
				snr[i] = 31;
			stats.snr += snr[i];
			PHY_INFORM(("i = %d, gidx = %d, snr = %d, snr_lcn = %d\n",
				i, phy_lcn_get_pkt_rssi_gain_index_offset(gidx[i]),
				snr[i], snr_lcn[i]));
		}
		stats.snr = stats.snr >> 2;

#if RSSI_IQEST_DEBUG
	stats.rssi = rssi_iqpwr_dB;
	stats.lostfrmcnt = rssi_iqpwr;
	stats.snr = board_atten_dbg;
#endif
	} else {
		/* Not available */
		stats.rssi = stats.snr = 0;
	}

	/* rx pkt stats */
	if (ISACPHY(pi)) {
		for (i = 0; i <= NUM_80211_RATES; i++)
			stats.rxpktcnt[i] = pi->u.pi_acphy->rxstats[i];
	} else {
		rxstats_base = wlapi_bmac_read_shm(pi->sh->physhim, M_RXSTATS_BLK_PTR);
		for (i = 0; i <= NUM_80211_RATES; i++) {
			stats.rxpktcnt[i] =
				wlapi_bmac_read_shm(pi->sh->physhim, 2*(rxstats_base+i));
		}
	}
	bcopy(&stats, a,
		(sizeof(wl_pkteng_stats_t) < (uint)alen) ? sizeof(wl_pkteng_stats_t) : (uint)alen);

	return BCME_OK;
}
#endif 

#if !defined(EFI)

#if PHY_TSSI_CAL_DBG_EN
static void
print_int64(int64 *a)
{
	void *llp = a;
	uint32 *lp_low = (uint32 *)llp;
	uint32 *lp_high = lp_low + 1;
	printf("0x%08x%08x ", *lp_high, *lp_low);
}
#endif


#if PHY_TSSI_CAL_DBG_EN
/*
 * matrix print
 * dimensions a (m x n)
 * name - matrix name
 */
static void
mat_print(int64 *a, int m, int n, const char *name)
{
	int i, j;

	printf("\n%s\n", name);
	for (i = 0; i < m; i++) {
		for (j = 0; j < n; j++)
			print_int64(a + (i * n) + j);
		printf("\n");
	}
}
#else /* PHY_TSSI_CAL_DBG_EN */
static void
mat_print(int64 *a, int m, int n, const char *name)
{
}
#endif /* PHY_TSSI_CAL_DBG_EN */


/* ================================================================
function [b0 b1 a1] = ratmodel_paparams_fix64(n, P)

%This is the algorithm used for curve fitting to get PA Params
%n: Adjusted TSSI values
%P: Power in qdBm

q1 = 4;
n = reshape(n, length(n), 1);
P = (reshape(P, length(P), 1)*q1);
P = round(P);

rho = ones(length(n), 3)*q1;
rho(:,2) = n*q1;
rho(:,3) = -n.*P;
rho = (rho./q1);
rho = round(rho);

C1 = rho' * rho;

a11 = C1(1,1); a12 = C1(1,2); a13 = C1(1,3);
a21 = C1(2,1); a22 = C1(2,2); a23 = C1(2,3);
a31 = C1(3,1); a32 = C1(3,2); a33 = C1(3,3);

C2_calc = [a22*a33 - a32*a23  a13*a32 - a12*a33  a12*a23 - a13*a22
		a23*a31 - a21*a33  a11*a33 - a13*a31  a13*a21 - a11*a23
		a21*a32 - a31*a22  a12*a31 - a11*a32  a11*a22 - a12*a21];

det_C1 = a11*a22*a33 + a12*a23*a31 + a13*a21*a32
		- a11*a23*a32 - a12*a21*a33 - a13*a22*a31;

C3 = C2_calc * rho';

C4 = C3 * P

C4 = round(C4./q1)

C = C4./det_C1;

b0=round(2^8*C(1)) ;
b1=round(2^12*C(2));
a1=round(2^15*C(3));

return;
*/
static void
ratmodel_paparams_fix64(ratmodel_paparams_t* rsd, int m)
{
	int i, j, n, q;
	int64 *a, temp;

	phy_utils_mat_rho((int64 *)(&rsd->n), (int64 *)(&rsd->p),
		(int64 *)(&rsd->rho), m);
	mat_print((int64 *)(&rsd->rho), m, 3, "rho");

	phy_utils_mat_transpose((int64 *)(&rsd->rho),
		(int64 *)(&rsd->rho_t), m, 3);
	mat_print((int64 *)(&rsd->rho_t), 3, m, "rho_t");

	phy_utils_mat_mult((int64*)(&rsd->rho_t), (int64 *)(&rsd->rho),
		(int64*)(&rsd->c1), 3, m, 3);
	mat_print((int64 *)(&rsd->c1), 3, 3, "c1");

	phy_utils_mat_inv_prod_det((int64 *)(&rsd->c1),
		(int64 *)(&rsd->c2_calc));
	mat_print((int64 *)(&rsd->c2_calc), 3, 3, "c2_calc");

	phy_utils_mat_det((int64 *)(&rsd->c1), (int64 *)(&rsd->det_c1));

#if PHY_TSSI_CAL_DBG_EN
	printf("\ndet_c1 = ");
	print_int64(&rsd->det_c1);
	printf("\n");
#endif

	phy_utils_mat_mult((int64*)(&rsd->c2_calc), (int64 *)(&rsd->rho_t),
		(int64*)(&rsd->c3), 3, 3, m);
	mat_print((int64 *)(&rsd->c3), 3, m, "c3");

	phy_utils_mat_mult((int64*)(&rsd->c3), (int64 *)(&rsd->p),
		(int64*)(&rsd->c4), 3, m, 1);

	m = 3; n = 1; q = 2;
	a = (int64*)(&rsd->c4);
	for (i = 0; i < m; i++)
		for (j = 0; j < n; j++) {
			temp = *(a + (i * n) + j);
			temp = (temp + (int64)(1 << (q-1)));
			temp = temp >> q;
			*(a + (i * n) + j) = temp;
		}

	mat_print((int64 *)(&rsd->c4), 3, 1, "c4");
}

int
wlc_phy_tssi_cal(phy_info_t *pi)
{
	uint16 count;
	count = tssi_cal_sweep(pi);
	ratmodel_paparams_fix64(&pi->ptssi_cal->rsd, count);
	return 0;
}

static uint16
tssi_cal_sweep(phy_info_t *pi)
{

	uint16 i, k = 0;

	uint16 count = 0;
	int8 *des_pwr = NULL;
	uint8 *adj_tssi = NULL;
	int *sort_pwr = NULL, avg_pwr;
	uint8 *sort_pwr_cnt = NULL;
	int16 MIN_PWR = 32; /* 8dBm */
	int16 MAX_PWR = 72; /* 18dBm */

	int64* tssi = pi->ptssi_cal->rsd.n;
	int64* pwr = pi->ptssi_cal->rsd.p;

	des_pwr = (int8*)MALLOC(pi->sh->osh, sizeof(int8)* 80 * MAX_NUM_ANCHORS);
	if (des_pwr == NULL)
		goto cleanup;

	adj_tssi = (uint8*)MALLOC(pi->sh->osh, sizeof(uint8) * 80 * MAX_NUM_ANCHORS);
	if (adj_tssi == NULL)
		goto cleanup;

	sort_pwr = (int*)MALLOC(pi->sh->osh, sizeof(int)*128);
	if (sort_pwr == NULL)
		goto cleanup;

	sort_pwr_cnt = (uint8*)MALLOC(pi->sh->osh, sizeof(uint8)*128);
	if (sort_pwr_cnt == NULL)
		goto cleanup;

	if (pi->pi_fptr->tssicalsweep)
		count = (*pi->pi_fptr->tssicalsweep)(pi, des_pwr, adj_tssi);
	else
		goto cleanup;

	for (i = 0; i < 128; i++) {
		sort_pwr[i] = 0xffffffff;
		sort_pwr_cnt[i] = 0;
	}

	for (i = 0; i < count; i++) {
		if (sort_pwr[adj_tssi[i]] == 0xffffffff)
			sort_pwr[adj_tssi[i]] = des_pwr[i];
		else {
			sort_pwr[adj_tssi[i]] += des_pwr[i];
		}
		sort_pwr_cnt[adj_tssi[i]]++;
	}

	k = 0;
	for (i = 0; i < 128; i++) {
		if (sort_pwr[i] != 0xffffffff) {
			avg_pwr =  sort_pwr[i]/sort_pwr_cnt[i];
			if ((avg_pwr >= MIN_PWR) && (avg_pwr <= MAX_PWR)) {
				tssi[k] = (int64) i;
				pwr[k] =  (int64) avg_pwr;
				k++;
			}
		}
	}

#if PHY_TSSI_CAL_DBG_EN
	printf("TSSI\tPWR, k = %d\n", k);
	for (i = 0; i < k; i++) {
		print_int64(&tssi[i]);
		printf("\t\t");
		print_int64(&pwr[i]);
		printf("\n");
	}
#endif

cleanup:
	if (des_pwr)
		MFREE(pi->sh->osh, des_pwr, sizeof(int8) * 80 * MAX_NUM_ANCHORS);
	if (adj_tssi)
		MFREE(pi->sh->osh, adj_tssi, sizeof(uint8) * 80 * MAX_NUM_ANCHORS);
	if (sort_pwr)
		MFREE(pi->sh->osh, sort_pwr, sizeof(int)*128);
	if (sort_pwr_cnt)
		MFREE(pi->sh->osh, sort_pwr_cnt, sizeof(uint8)*128);

	return k;
}

#endif /* #if !defined(EFI) */

#if defined(WLTEST) || defined(DBG_PHY_IOV)
static int
wlc_phy_dynamic_ml(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, int vsize, bool set)
{
	int err = BCME_OK;

	if (!pi->sh->clk)
		return BCME_NOCLK;

	if (!ISNPHY(pi))
		return BCME_UNSUPPORTED;
	else if (NREV_LT(pi->pubpi->phy_rev, LCNXN_BASEREV + 2))
		return BCME_UNSUPPORTED;

	if (!set) {
		if (ISNPHY(pi)) {
			wlc_phy_dynamic_ml_get(pi);
			*ret_int_ptr = pi->nphy_ml_type;
		}

	} else {
		if ((int_val > 4) || (int_val < 0))
			return BCME_RANGE;
		wlc_phy_dynamic_ml_set(pi, int_val);
	}
	return err;
}

/* aci_nams : ACI Non Assoc Mode Sanity */
static int
wlc_phy_aci_nams(phy_info_t *pi, int32 int_val,	int32 *ret_int_ptr, int vsize, bool set)
{
	int err = BCME_OK;

	if (!pi->sh->clk)
		return BCME_NOCLK;

	if (!ISNPHY(pi))
		return BCME_UNSUPPORTED;
	else if NREV_LT(pi->pubpi->phy_rev, LCNXN_BASEREV + 2)
		return BCME_UNSUPPORTED;

	if (!set) {
		if (ISNPHY(pi)) {
			*ret_int_ptr = pi->aci_nams;
		}

	} else {
		if ((int_val > 1) || (int_val < 0))
			return BCME_RANGE;
		pi->aci_nams = (uint8)int_val;
	}
	return err;
}
#endif 

#if ((ACCONF != 0) || (ACCONF2 != 0) || (NCONF != 0) || (HTCONF != 0) || (LCN40CONF != \
	0))
static int
wlc_phy_adjust_ed_thres(phy_info_t *pi, int32 *assert_thresh_dbm, bool set_threshold)
{
	if (ISACPHY(pi))
		wlc_phy_adjust_ed_thres_acphy(pi, assert_thresh_dbm, set_threshold);
	else if (ISNPHY(pi))
		wlc_phy_adjust_ed_thres_nphy(pi, assert_thresh_dbm, set_threshold);
	else if (ISHTPHY(pi))
		wlc_phy_adjust_ed_thres_htphy(pi, assert_thresh_dbm, set_threshold);
	else if (ISLCN40PHY(pi))
		wlc_phy_adjust_ed_thres_lcn40phy(pi, assert_thresh_dbm, set_threshold);
	else
		return BCME_UNSUPPORTED;

	return BCME_OK;
}
#endif /* ((ACCONF != 0) || (ACCONF2 != 0) || (NCONF != 0) || (HTCONF != 0) || (LCN40CONF != 0)) */

/* wrapper function for multi phase perical schedule */
void
wlc_phy_mphase_cal_schedule(wlc_phy_t *pih, uint delay_val)
{
	phy_info_t *pi = (phy_info_t*)pih;

	if (!ISNPHY(pi) && !ISHTPHY(pi) && !ISACPHY(pi))
		return;

	if (PHY_PERICAL_MPHASE_PENDING(pi)) {
		wlc_phy_cal_perical_mphase_reset(pi);
	}

	pi->cal_info->cal_searchmode = PHY_CAL_SEARCHMODE_RESTART;
	/* schedule mphase cal */
	wlc_phy_cal_perical_mphase_schedule(pi, delay_val);
}

static void
wlc_phy_cal_perical_mphase_schedule(phy_info_t *pi, uint delay_val)
{
	/* for manual mode, let it run */
	if ((pi->phy_cal_mode != PHY_PERICAL_MPHASE) &&
	    (pi->phy_cal_mode != PHY_PERICAL_MANUAL))
		return;

	PHY_CAL(("wlc_phy_cal_perical_mphase_schedule\n"));

	/* use timer to wait for clean context since this
	 * may be called in the middle of nphy_init
	 */
	wlapi_del_timer(pi->sh->physhim, pi->phycal_timer);

	pi->cal_info->cal_phase_id = MPHASE_CAL_STATE_INIT;
	wlapi_add_timer(pi->sh->physhim, pi->phycal_timer, delay_val, 0);
}

/* policy entry */
void
wlc_phy_cal_perical(wlc_phy_t *pih, uint8 reason)
{
	int16 current_temp = 0, delta_temp = 0;
	uint delta_time = 0;
	uint8 do_cals = 0;
	bool  suppress_cal = FALSE;

	phy_info_t *pi = (phy_info_t*)pih;

#if defined(PHYCAL_CACHING)
	ch_calcache_t *ctx = NULL;
	ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
#endif

	/* reset to default */
	pi->cal_info->cal_suppress_count = 0;

	/* do only init noisecal or trigger noisecal when STA
	 * joins to an AP (also trigger noisecal if AP roams)
	 */
	pi->trigger_noisecal = TRUE;

	/* reset periodic noise poll flag to avoid
	 * race-around condition with cal triggers
	 * between watchdog and others which could
	 * potentially cause sw corruption.
	 */
	pi->capture_periodic_noisestats = FALSE;

	if (!ISNPHY(pi) && !ISHTPHY(pi) && !ISACPHY(pi))
		return;

	if ((pi->phy_cal_mode == PHY_PERICAL_DISABLE) ||
	    (pi->phy_cal_mode == PHY_PERICAL_MANUAL))
		return;

	/* NPHY_IPA : disable PAPD cal for following calibration at least 4322A1? */

	PHY_CAL(("wlc_phy_cal_perical: reason %d chanspec 0x%x\n", reason,
	         pi->radio_chanspec));

	/* Update the Tx power per channel on ACPHY for 2GHz channels */
#ifdef POWPERCHANNL
		if (ISACPHY(pi) && !(ACMAJORREV_4(pi->pubpi->phy_rev)))
			wlc_phy_tx_target_pwr_per_channel_decide_run_acphy(pi);
#endif /* POWPERCHANNL */

	/* perical is enabled : Either single phase only, or mphase is allowed
	 * Dispatch to s-phase or m-phase based on reasons
	 */
	switch (reason) {

	case PHY_PERICAL_DRIVERUP:	/* always single phase ? */
		break;

	case PHY_PERICAL_PHYINIT:	/* always multi phase */
		if (pi->phy_cal_mode == PHY_PERICAL_MPHASE) {
#if defined(PHYCAL_CACHING)
			if (ctx)
			{
				/* Switched context so restart a pending MPHASE cal or
				 * restore stored calibration
				 */
				ASSERT(ctx->chanspec == pi->radio_chanspec);

				/* If it was pending last time, just restart it */
				if (PHY_PERICAL_MPHASE_PENDING(pi)) {
					/* Delete any existing timer just in case */
					PHY_CAL(("%s: Restarting calibration for 0x%x phase %d\n",
						__FUNCTION__, ctx->chanspec,
						pi->cal_info->cal_phase_id));
					wlapi_del_timer(pi->sh->physhim, pi->phycal_timer);
					wlapi_add_timer(pi->sh->physhim, pi->phycal_timer, 0, 0);
				} else if (wlc_phy_cal_cache_restore(pi) != BCME_ERROR) {
					break;
				}
			}
			else
#endif /* PHYCAL_CACHING */
			{
				if (PHY_PERICAL_MPHASE_PENDING(pi))
					wlc_phy_cal_perical_mphase_reset(pi);

				pi->cal_info->cal_searchmode = PHY_CAL_SEARCHMODE_RESTART;

				/* schedule mphase cal */
				wlc_phy_cal_perical_mphase_schedule(pi, PHY_PERICAL_INIT_DELAY);
			}
		}
		break;

	case PHY_PERICAL_JOIN_BSS:
	case PHY_PERICAL_START_IBSS:
	case PHY_PERICAL_UP_BSS:
	case PHY_PERICAL_PHYMODE_SWITCH:

		/* These must run in single phase to ensure clean Tx/Rx
		 * performance so the auto-rate fast-start is promising
		 */

		if ((pi->phy_cal_mode == PHY_PERICAL_MPHASE) && PHY_PERICAL_MPHASE_PENDING(pi))
			wlc_phy_cal_perical_mphase_reset(pi);

		/* Always do idle TSSI measurement at the end of NPHY cal
		 * while starting/joining a BSS/IBSS
		 */
		pi->first_cal_after_assoc = TRUE;

		if (ISNPHY(pi))
			pi->u.pi_nphy->cal_type_override = PHY_PERICAL_FULL; /* not used in htphy */


		/* Update last cal temp to current tempsense reading */
		if (pi->phycal_tempdelta) {
			if (ISNPHY(pi))
				pi->cal_info->last_cal_temp = wlc_phy_tempsense_nphy(pi);
			else if (ISHTPHY(pi))
				pi->cal_info->last_cal_temp = wlc_phy_tempsense_htphy(pi);
			else if (ISACPHY(pi))
				pi->cal_info->last_cal_temp =
					wlc_phy_tempsense_acphy(pi);
		}

		/* Attempt cal cache restore if ctx is valid */
#if defined(PHYCAL_CACHING)
		if (ctx)
		{
			PHY_CAL(("wl%d: %s: Attempting to restore cals on JOIN...\n",
				pi->sh->unit, __FUNCTION__));

			if (wlc_phy_cal_cache_restore(pi) == BCME_ERROR) {
				if (ISNPHY(pi))
					wlc_phy_cal_perical_nphy_run(pi, PHY_PERICAL_FULL);
				else if (ISHTPHY(pi))
					wlc_phy_cals_htphy(pi, PHY_CAL_SEARCHMODE_RESTART);
				else if (ISACPHY(pi))
					wlc_phy_cals_acphy(pi, PHY_CAL_SEARCHMODE_RESTART);
			}
		}
		else
#endif /* PHYCAL_CACHING */
		{
			if (ISNPHY(pi))
				wlc_phy_cal_perical_nphy_run(pi, PHY_PERICAL_FULL);
			else if (ISHTPHY(pi))
				wlc_phy_cals_htphy(pi, PHY_CAL_SEARCHMODE_RESTART);
			else if (ISACPHY(pi))
				wlc_phy_cals_acphy(pi, PHY_CAL_SEARCHMODE_RESTART);
		}
		break;

	case PHY_PERICAL_WATCHDOG:

		if (PUB_NOT_ASSOC(pi) && ISACPHY(pi))
			return;

		if (ACMAJORREV_4(pi->pubpi->phy_rev) && (RADIOREV(pi->pubpi->radiorev) == 13))
			do_cals = wlc_phy_vbat_monitoring_algorithm_decision(pi);

		/* Disable periodic noisecal trigger */
		pi->trigger_noisecal = FALSE;

		if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
			pi->capture_periodic_noisestats = TRUE;
		else
			pi->capture_periodic_noisestats = FALSE;

		PHY_CAL(("%s: %sPHY phycal_tempdelta=%d\n", __FUNCTION__,
			(ISNPHY(pi)) ? "N": (ISHTPHY(pi) ? "HT" : (ISACPHY(pi) ? "AC" : "some")),
			pi->phycal_tempdelta));

		if (pi->phycal_tempdelta && (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi))) {

			int cal_chanspec = 0;

			if (ISNPHY(pi)) {
				current_temp = wlc_phy_tempsense_nphy(pi);
				cal_chanspec = pi->cal_info->u.ncal.txiqlocal_chanspec;
			} else if (ISHTPHY(pi)) {
				current_temp = wlc_phy_tempsense_htphy(pi);
				cal_chanspec = pi->cal_info->u.htcal.chanspec;
			} else if (ISACPHY(pi)) {
				if (pi->sh->now - pi->cal_info->last_temp_cal_time >=
					pi->sh->glacial_timer) {
					pi->cal_info->last_temp_cal_time = pi->sh->now;
					current_temp = wlc_phy_tempsense_acphy(pi);
				} else {
					current_temp =  pi->cal_info->last_cal_temp;
				}
				cal_chanspec = pi->cal_info->u.accal.chanspec;
			}

			delta_temp = ((current_temp > pi->cal_info->last_cal_temp) ?
				(current_temp - pi->cal_info->last_cal_temp) :
				(pi->cal_info->last_cal_temp - current_temp));

			/* Only do WATCHDOG triggered (periodic) calibration if
			 * the channel hasn't changed and if the temperature delta
			 * is above the specified threshold
			 */
			PHY_CAL(("%sPHY temp is %d, delta %d, cal_delta %d, chanspec %04x/%04x\n",
				(ISNPHY(pi)) ? "N": (ISHTPHY(pi) ? "HT" :
				(ISACPHY(pi) ? "AC" : "some")),
				current_temp, delta_temp, pi->phycal_tempdelta,
				cal_chanspec, pi->radio_chanspec));

			delta_time = pi->sh->now - pi->cal_info->last_cal_time;

			/* cal_period = 0 implies only temperature based cals */
			if (((delta_temp < pi->phycal_tempdelta) &&
				(((delta_time < pi->cal_period) &&
				(pi->cal_period > 0)) || (pi->cal_period == 0)) &&
				(cal_chanspec == pi->radio_chanspec)) && do_cals == 0) {

				suppress_cal = TRUE;
				pi->cal_info->cal_suppress_count = pi->sh->glacial_timer;
				PHY_CAL(("Suppressing calibration.\n"));

			} else {
				pi->cal_info->last_cal_temp = current_temp;
			}
		}

		if (!suppress_cal) {
			/* if mphase is allowed, do it, otherwise, fall back to single phase */
			if (pi->phy_cal_mode == PHY_PERICAL_MPHASE) {
				/* only schedule if it's not in progress */
				if (!PHY_PERICAL_MPHASE_PENDING(pi)) {
					pi->cal_info->cal_searchmode = PHY_CAL_SEARCHMODE_REFINE;
					wlc_phy_cal_perical_mphase_schedule(pi,
						PHY_PERICAL_WDOG_DELAY);
				}
			} else if (pi->phy_cal_mode == PHY_PERICAL_SPHASE) {
				if (ISNPHY(pi))
					wlc_phy_cal_perical_nphy_run(pi, PHY_PERICAL_AUTO);
				else if (ISHTPHY(pi))
					wlc_phy_cals_htphy(pi, PHY_CAL_SEARCHMODE_RESTART);
				else if (ISACPHY(pi))
					wlc_phy_cals_acphy(pi, PHY_CAL_SEARCHMODE_RESTART);
			} else {
				ASSERT(0);
			}
		}
		break;

	case PHY_PERICAL_DCS:

		/* Only applicable for NPHYs */
		ASSERT(ISNPHY(pi));

		if (ISNPHY(pi)) {
			if (PHY_PERICAL_MPHASE_PENDING(pi)) {
				wlc_phy_cal_perical_mphase_reset(pi);

				if (pi->phycal_tempdelta) {
					current_temp = wlc_phy_tempsense_nphy(pi);
					pi->cal_info->last_cal_temp = current_temp;
				}
			} else if (pi->phycal_tempdelta) {

				current_temp = wlc_phy_tempsense_nphy(pi);

				delta_temp = ((current_temp > pi->cal_info->last_cal_temp) ?
					(current_temp - pi->cal_info->last_cal_temp) :
					(pi->cal_info->last_cal_temp - current_temp));

				if ((delta_temp < (int16)pi->phycal_tempdelta)) {
					suppress_cal = TRUE;
				} else {
					pi->cal_info->last_cal_temp = current_temp;
				}
			}

			if (suppress_cal) {
				wlc_phy_txpwr_papd_cal_nphy_dcs(pi);
			} else {
				/* only mphase is allowed */
				if (pi->phy_cal_mode == PHY_PERICAL_MPHASE) {
					pi->cal_info->cal_searchmode = PHY_CAL_SEARCHMODE_REFINE;
					wlc_phy_cal_perical_mphase_schedule(pi,
						PHY_PERICAL_WDOG_DELAY);
				} else {
					ASSERT(0);
				}
			}
		}
		break;

	default:
		ASSERT(0);
		break;
	}
}

void
wlc_phy_trigger_cals_for_btc_adjust(phy_info_t *pi)
{
	wlc_phy_cal_perical_mphase_reset(pi);
	if (ISNPHY(pi)) {
		pi->u.pi_nphy->cal_type_override = PHY_PERICAL_FULL;
	}
	wlc_phy_cal_perical_mphase_schedule(pi, PHY_PERICAL_NODELAY);
}

#if defined(WLTEST)

static int
wlc_phy_iovar_idletssi(phy_info_t *pi, int32 *ret_int_ptr, bool type)
{
	/* no argument or type = 1 will do full tx_pwr_ctrl_init */
	/* type = 0 will do just idle_tssi_est */
	int err = BCME_OK;
	if (ISLCNPHY(pi))
		*ret_int_ptr = wlc_lcnphy_idle_tssi_est_iovar(pi, type);
	else if (ISLCN40PHY(pi))
		*ret_int_ptr = wlc_lcn40phy_idle_tssi_est_iovar(pi, type);
	else
		*ret_int_ptr = 0;

	return err;
}

static int
wlc_phy_iovar_bbmult_get(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr)
{
	int err = BCME_OK;

	if (ISNPHY(pi))
		wlc_phy_get_bbmult_nphy(pi, ret_int_ptr);
	else
		err = BCME_UNSUPPORTED;

	return err;
}

static int
wlc_phy_iovar_bbmult_set(phy_info_t *pi, void *p)
{
	int err = BCME_OK;
	uint16 bbmult[PHY_CORE_NUM_2] = { 0 };
	uint8 m0, m1;

	bcopy(p, bbmult, PHY_CORE_NUM_2 * sizeof(uint16));

	if (ISNPHY(pi)) {
		m0 = (uint8)(bbmult[0] & 0xff);
		m1 = (uint8)(bbmult[1] & 0xff);
		wlc_phy_set_bbmult_nphy(pi, m0, m1);
	} else
		err = BCME_UNSUPPORTED;

	return err;
}

static int
wlc_phy_iovar_vbatsense(phy_info_t *pi, int32 *ret_int_ptr)
{
	int err = BCME_OK;
	int32 int_val;

	if (ISLCNPHY(pi)) {
		int_val = wlc_lcnphy_vbatsense(pi, 1);
		bcopy(&int_val, ret_int_ptr, sizeof(int_val));
	} else if (ISLCN40PHY(pi)) {
		int_val = wlc_lcn40phy_vbatsense(pi, TEMPER_VBAT_TRIGGER_NEW_MEAS);
		bcopy(&int_val, ret_int_ptr, sizeof(int_val));
	} else if (ISNPHY(pi)) {
		int_val = (int32)(wlc_phy_vbat_from_statusbyte_nphy_rev19(pi));
		bcopy(&int_val, ret_int_ptr, sizeof(int_val));
	} else
		err = BCME_UNSUPPORTED;

	return err;
}
#endif 

#if defined(WLTEST) || defined(AP)
static int
wlc_phy_iovar_perical_config(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr,	bool set)
{
	int err = BCME_OK;

	if (!set) {
		if (!ISNPHY(pi) && !ISHTPHY(pi) && !ISACPHY(pi) && !ISLCNPHY(pi))
			/* supported for n, ht, ac and lcn phy only */
			return BCME_UNSUPPORTED;

		*ret_int_ptr =  pi->phy_cal_mode;
	} else {
		if (!ISNPHY(pi) && !ISHTPHY(pi) && !ISACPHY(pi) && !ISLCNPHY(pi))
			/* supported for n, ht, ac and lcn phy only */
			return BCME_UNSUPPORTED;

		if (int_val == 0) {
			pi->phy_cal_mode = PHY_PERICAL_DISABLE;
		} else if (int_val == 1) {
			pi->phy_cal_mode = PHY_PERICAL_SPHASE;
		} else if (int_val == 2) {
			if (ISACPHY(pi) && ACREV_IS(pi->pubpi->phy_rev, 1)) {
				pi->phy_cal_mode = PHY_PERICAL_MPHASE;
				/* enabling MPHASE only for 4360 B0 */
			} else {
				pi->phy_cal_mode = PHY_PERICAL_SPHASE;
			}
		} else if (int_val == 3) {
			/* this mode is to disable normal periodic cal paths
			 *  only manual trigger(nphy_forcecal) can run it
			 */
			pi->phy_cal_mode = PHY_PERICAL_MANUAL;
		} else {
			err = BCME_RANGE;
			goto end;
		}
		wlc_phy_cal_perical_mphase_reset(pi);
	}
end:
	return err;
}
#endif	

#if defined(BCMDBG) || defined(WLTEST) || defined(MACOSX) || defined(ATE_BUILD)
static int
wlc_phy_iovar_tempsense_paldosense(phy_info_t *pi, int32 *ret_int_ptr, uint8 tempsense_paldosense)
{
	int err = BCME_OK;
	int32 int_val;

	*ret_int_ptr = 0;
	if (ISNPHY(pi)) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		*ret_int_ptr = (int32)wlc_phy_tempsense_nphy(pi);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
	} else if (ISHTPHY(pi)) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		*ret_int_ptr = (int32)wlc_phy_tempsense_htphy(pi);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
	} else if (ISACPHY(pi)) {
		/* No need to call suspend_mac and phyreg_enter since it
		* is done inside wlc_phy_tempsense_acphy
		*/
	  if (pi->radio_is_on)
	    *ret_int_ptr = (int32)wlc_phy_tempsense_acphy(pi);
	  else
	    err = BCME_RADIOOFF;
	} else if (ISLCNPHY(pi)) {
		if (CHIPID(pi->sh->chip) == BCM4313_CHIP_ID)
			int_val = (int32)wlc_lcnphy_tempsense(pi, 1);
		else
			int_val = wlc_lcnphy_tempsense_degree(pi, 1);
		bcopy(&int_val, ret_int_ptr, sizeof(int_val));
	} else if (ISLCN40PHY(pi)) {
		int_val = wlc_lcn40phy_tempsense(pi, TEMPER_VBAT_TRIGGER_NEW_MEAS);
		bcopy(&int_val, ret_int_ptr, sizeof(int_val));
	} else
		err = BCME_UNSUPPORTED;

	return err;
}
#endif	/* defined(BCMDBG) || defined(WLTEST) || defined(MACOSX) || defined(ATE_BUILD) */

#if defined(WLTEST)
static int
wlc_phy_iovar_idletssi_reg(phy_info_t *pi, int32 *ret_int_ptr, int32 int_val, bool set)
{
	int err = BCME_OK;
	uint32 tmp;
	uint16 idle_tssi[NPHY_CORE_NUM];
	phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;

	if (ISLCN40PHY(pi))
		*ret_int_ptr = wlc_lcn40phy_idle_tssi_reg_iovar(pi, int_val, set, &err);
	else if (ISHTPHY(pi))
		*ret_int_ptr = wlc_phy_idletssi_get_htphy(pi);
	else if (ISNPHY(pi)) {
		if (!(CHIP_4324_B0(pi) || CHIP_4324_B4(pi))) {
			wlc_phy_lcnxn_rx2tx_stallwindow_nphy(pi, 1);
			wlc_phy_txpwrctrl_idle_tssi_nphy(pi);
			wlc_phy_lcnxn_rx2tx_stallwindow_nphy(pi, 0);
		}
		if (CHSPEC_IS2G(pi->radio_chanspec)) {
			idle_tssi[0] = (uint16)pi_nphy->nphy_pwrctrl_info[0].idle_tssi_2g;
			idle_tssi[1] = (uint16)pi_nphy->nphy_pwrctrl_info[1].idle_tssi_2g;
		} else {
			idle_tssi[0] = (uint16)pi_nphy->nphy_pwrctrl_info[0].idle_tssi_5g;
			idle_tssi[1] = (uint16)pi_nphy->nphy_pwrctrl_info[1].idle_tssi_5g;
		}
		tmp = (idle_tssi[1] << 16) | idle_tssi[0];
		*ret_int_ptr = tmp;
	}

	return err;
}

static int
wlc_phy_iovar_avgtssi_reg(phy_info_t *pi, int32 *ret_int_ptr)
{
	int err = BCME_OK;
	if (ISLCN40PHY(pi))
		*ret_int_ptr = wlc_lcn40phy_avg_tssi_reg_iovar(pi);
	else if (ISNPHY(pi)) {
		*ret_int_ptr = wlc_nphy_tssi_read_iovar(pi);
	}
	return err;
}
#endif 

#if defined(WLTEST) || defined(DBG_PHY_IOV) || defined(WFD_PHY_LL_DEBUG) || \
	defined(ATE_BUILD)
static int
wlc_phy_iovar_forcecal(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, int vsize, bool set)
{
	int err = BCME_OK;
	void *a;
#if defined(PHYCAL_CACHING)
	ch_calcache_t *ctx;
#endif /* PHYCAL_CACHING */

	a = (int32*)ret_int_ptr;

	if (!pi->sh->up)
		return BCME_NOTUP;

	if (ISHTPHY(pi)) {
		uint8 mphase = 0, searchmode = 0;

		/* for get with no argument, assume 0x00 */
		if (!set)
			int_val = 0x00;

		/* upper nibble: 0 = sphase,  1 = mphase */
		mphase = (((uint8) int_val) & 0xf0) >> 4;

		/* 0 = RESTART, 1 = REFINE, for Tx-iq/lo-cal */
		searchmode = ((uint8) int_val) & 0xf;

		PHY_CAL(("wlc_phy_iovar_forcecal (mphase = %d, refine = %d)\n",
			mphase, searchmode));

		/* call sphase or schedule mphase cal */
		wlc_phy_cal_perical_mphase_reset(pi);
		if (mphase) {
			pi->cal_info->cal_searchmode = searchmode;
			wlc_phy_cal_perical_mphase_schedule(pi, PHY_PERICAL_NODELAY);
		} else {
			wlc_phy_cals_htphy(pi, searchmode);
		}
	} else if (ISACPHY(pi)) {
		uint8 mphase = FALSE;
		uint8 searchmode = PHY_CAL_SEARCHMODE_RESTART;

		/* for get with no argument, assume 0x00 */
		if (!set)
			int_val = 0x00;

		/* only values in range [0-3] are valids */
		if (int_val > 3)
			return BCME_BADARG;

		/* 3 is mphase, anything else is single phase */
		if (int_val == 3) {
			mphase = TRUE;
		}
		else {
			/* Single phase, using 2 means sphase partial */
			if (int_val == 2)
				searchmode = PHY_CAL_SEARCHMODE_REFINE;
		}

		PHY_CAL(("wlc_phy_iovar_forcecal (mphase = %d, refine = %d)\n",
			mphase, searchmode));

		/* call sphase or schedule mphase cal */
		wlc_phy_cal_perical_mphase_reset(pi);
		if (mphase) {
			pi->cal_info->cal_searchmode = searchmode;
			wlc_phy_cal_perical_mphase_schedule(pi, PHY_PERICAL_NODELAY);
		} else {
			wlc_phy_cals_acphy(pi, searchmode);
		}
	} else if (ISNPHY(pi)) {
		/* for get with no argument, assume 0x00 */
		if (!set)
			int_val = PHY_PERICAL_AUTO;

		if ((int_val == PHY_PERICAL_PARTIAL) ||
		    (int_val == PHY_PERICAL_AUTO) ||
		    (int_val == PHY_PERICAL_FULL)) {
			wlc_phy_cal_perical_mphase_reset(pi);
			pi->u.pi_nphy->cal_type_override = (uint8)int_val;
			wlc_phy_cal_perical_mphase_schedule(pi, PHY_PERICAL_NODELAY);
#ifdef WLOTA_EN
		} else if (int_val == PHY_FULLCAL_SPHASE) {
			wlc_phy_cal_perical((wlc_phy_t *)pi, PHY_FULLCAL_SPHASE);
#endif /* WLOTA_EN */
		} else
			err = BCME_RANGE;

		/* phy_forcecal will trigger noisecal */
		pi->trigger_noisecal = TRUE;

	} else if (ISLCNCOMMONPHY(pi) && pi->pi_fptr->calibmodes) {
#if defined(PHYCAL_CACHING)
		ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
		if (ctx) {
			ctx->valid = FALSE;
		}
		/* null ctx is invalid by definition */
#else
		phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
		pi_lcn->lcnphy_full_cal_channel = 0;
#endif /* PHYCAL_CACHING */
		if (!set)
			*ret_int_ptr = 0;

		pi->pi_fptr->calibmodes(pi, PHY_FULLCAL);
		int_val = 0;
		bcopy(&int_val, a, vsize);
	}

	return err;
}

#ifndef ATE_BUILD
static int
wlc_phy_iovar_forcecal_obt(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, int vsize, bool set)
{
	int err = BCME_OK;
	uint8 wait_ctr = 0;
	int val = 1;
	if (ISNPHY(pi)) {
			wlc_phy_cal_perical_mphase_reset(pi);

			pi->cal_info->cal_phase_id = MPHASE_CAL_STATE_INIT;
			pi->trigger_noisecal = TRUE;

			while (wait_ctr < 50) {
				val = ((pi->cal_info->cal_phase_id !=
					MPHASE_CAL_STATE_IDLE)? 1 : 0);
				if (val == 0) {
					err = BCME_OK;
					break;
				}
				else {
					wlc_phy_cal_perical_nphy_run(pi, PHY_PERICAL_FULL);
					wait_ctr++;
				}
			}
			if (wait_ctr >= 50) {
				return BCME_ERROR;
			}

		}

	return err;
}

static int
wlc_phy_iovar_forcecal_noise(phy_info_t *pi, int32 int_val, void *a, int vsize, bool set)
{
	int err = BCME_OK;
	uint8 wait_ctr = 0;
	int val = 1;
	uint16 crsmin[4];

	if (ISNPHY(pi)) {
		if (!set) {
			crsmin[0] = phy_utils_read_phyreg(pi, NPHY_crsminpowerl0);
			crsmin[1] = phy_utils_read_phyreg(pi, NPHY_crsminpoweru0);
			crsmin[2] = phy_utils_read_phyreg(pi, NPHY_crsminpowerl0_core1);
			crsmin[3] = phy_utils_read_phyreg(pi, NPHY_crsminpoweru0_core1);
			bcopy(crsmin, a, sizeof(uint16)*4);
		}
		else {
			wlc_phy_cal_perical_mphase_reset(pi);

			pi->cal_info->cal_phase_id = MPHASE_CAL_STATE_NOISECAL;
			pi->trigger_noisecal = TRUE;

			while (wait_ctr < 50) {
				val = ((pi->cal_info->cal_phase_id !=
				MPHASE_CAL_STATE_IDLE)? 1 : 0);

				if (val == 0) {
					err = BCME_OK;
					break;
				}
				else {
					wlc_phy_cal_perical_nphy_run(pi, PHY_PERICAL_PARTIAL);
					wait_ctr++;
				}
			}

			if (wait_ctr >= 50) {
				return BCME_ERROR;
			}
		}
	}
	return err;
}
#endif /* !ATE_BUILD */
#endif 

#if defined(WLTEST) || defined(MACOSX)
static void
wlc_phy_iovar_set_deaf(phy_info_t *pi, int32 int_val)
{
	if (int_val) {
		wlc_phy_set_deaf((wlc_phy_t *) pi, TRUE);
	} else {
		wlc_phy_clear_deaf((wlc_phy_t *) pi, TRUE);
	}
}

static int
wlc_phy_iovar_get_deaf(phy_info_t *pi, int32 *ret_int_ptr)
{
	if (ISHTPHY(pi)) {
		*ret_int_ptr = (int32)wlc_phy_get_deaf_htphy(pi);
		return BCME_OK;
	} else if (ISNPHY(pi)) {
		*ret_int_ptr = (int32)wlc_phy_get_deaf_nphy(pi);
		return BCME_OK;
	} else if (ISACPHY(pi)) {
	        *ret_int_ptr = (int32)wlc_phy_get_deaf_acphy(pi);
		return BCME_OK;
	} else {
		return BCME_UNSUPPORTED;
	}
}
#endif 
#if defined(WLTEST) || defined(ATE_BUILD)
static int
wlc_phy_iovar_txpwrctrl(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr,
	bool set)
{
	int err = BCME_OK;

	if (!set) {
		if (ISACPHY(pi) || ISHTPHY(pi)) {
			*ret_int_ptr = pi->txpwrctrl;
		} else if (ISNPHY(pi)) {
			*ret_int_ptr = pi->nphy_txpwrctrl;
		} else if (ISLCNPHY(pi)) {
			*ret_int_ptr = wlc_phy_tpc_iovar_isenabled_lcnphy(pi);
		} else if (pi->pi_fptr->ishwtxpwrctrl) {
			*ret_int_ptr = pi->pi_fptr->ishwtxpwrctrl(pi);
		}

	} else {
		if (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi)) {
			if ((int_val != PHY_TPC_HW_OFF) && (int_val != PHY_TPC_HW_ON)) {
				err = BCME_RANGE;
				goto end;
			}

			pi->nphy_txpwrctrl = (uint8)int_val;
			pi->txpwrctrl = (uint8)int_val;

			/* if not up, we are done */
			if (!pi->sh->up)
				goto end;

			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			phy_utils_phyreg_enter(pi);
			if (ISNPHY(pi))
				wlc_phy_txpwrctrl_enable_nphy(pi, (uint8) int_val);
			else if (ISHTPHY(pi))
				wlc_phy_txpwrctrl_enable_htphy(pi, (uint8) int_val);
			else if (ISACPHY(pi))
				wlc_phy_txpwrctrl_enable_acphy(pi, (uint8) int_val);
			phy_utils_phyreg_exit(pi);
			wlapi_enable_mac(pi->sh->physhim);

		} else if (ISLCNPHY(pi)) {
			wlc_lcnphy_iovar_txpwrctrl(pi, int_val, ret_int_ptr);
		} else if (ISLCN40PHY(pi)) {
			wlc_lcn40phy_iovar_txpwrctrl(pi, int_val, ret_int_ptr);
		}
	}

end:
	return err;
}

static int
wlc_phy_iovar_txpwrindex_get(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr)
{
	int err = BCME_OK;

	if (ISNPHY(pi)) {

		if (D11REV_IS(pi->sh->corerev, 11) || D11REV_IS(pi->sh->corerev, 12)) {
			wlapi_bmac_mctrl(pi->sh->physhim, MCTL_PHYLOCK,  MCTL_PHYLOCK);
			(void)R_REG(pi->sh->osh, &pi->regs->maccontrol);
			OSL_DELAY(1);
		}

		*ret_int_ptr = wlc_phy_txpwr_idx_get_nphy(pi);

		if (D11REV_IS(pi->sh->corerev, 11) || D11REV_IS(pi->sh->corerev, 12))
			wlapi_bmac_mctrl(pi->sh->physhim, MCTL_PHYLOCK,  0);

	} else if (ISHTPHY(pi)) {
		*ret_int_ptr = wlc_phy_txpwr_idx_get_htphy(pi);
	} else if (ISACPHY(pi)) {
		*ret_int_ptr = wlc_phy_txpwr_idx_get_acphy(pi);
	} else if (ISLCNPHY(pi))
		*ret_int_ptr = wlc_lcnphy_get_current_tx_pwr_idx(pi);
	else if (ISLCN40PHY(pi))
		*ret_int_ptr = wlc_lcn40phy_get_current_tx_pwr_idx(pi);

	return err;
}


static int
wlc_phy_iovar_txpwrindex_set(phy_info_t *pi, void *p)
{
	int err = BCME_OK;
	uint32 txpwridx[PHY_CORE_MAX] = { 0x30 };
	int8 idx, core;
	int8 siso_int_val;
	phy_info_nphy_t *pi_nphy = NULL;
#if defined(PHYCAL_CACHING)
	ch_calcache_t *ctx = wlc_phy_get_chanctx(pi, pi->radio_chanspec);
#endif

	if (ISNPHY(pi))
		pi_nphy = pi->u.pi_nphy;

	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);

	bcopy(p, txpwridx, PHY_CORE_MAX * sizeof(uint32));
	siso_int_val = (int8)(txpwridx[0] & 0xff);

	if (ISNPHY(pi)) {
		FOREACH_CORE(pi, core) {
			idx = (int8)(txpwridx[core] & 0xff);
			pi_nphy->nphy_txpwrindex[core].index_internal = idx;
			wlc_phy_store_txindex_nphy(pi);
			wlc_phy_txpwr_index_nphy(pi, (1 << core), idx, TRUE);
		}
	} else if (ISHTPHY(pi)) {
		FOREACH_CORE(pi, core) {
			idx = (int8)(txpwridx[core] & 0xff);
			wlc_phy_txpwr_by_index_htphy(pi, (1 << core), idx);
		}
	} else if (ISACPHY(pi)) {
		FOREACH_CORE(pi, core) {
			idx = (int8)(txpwridx[core] & 0xff);
			wlc_phy_txpwrctrl_enable_acphy(pi, PHY_TPC_HW_OFF);
			wlc_phy_txpwr_by_index_acphy(pi, (1 << core), idx);
		}
	} else if (ISLCNCOMMONPHY(pi)) {
#if defined(PHYCAL_CACHING)
		err = wlc_iovar_txpwrindex_set_lcncommon(pi, siso_int_val, ctx);
#else
		err = wlc_iovar_txpwrindex_set_lcncommon(pi, siso_int_val);
#endif
	}

	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);

	return err;
}

static void
wlc_phy_iovar_tx_tone(phy_info_t *pi, int32 int_val)
{
	pi->phy_tx_tone_freq = (int32) int_val;

	if (pi->phy_tx_tone_freq == 0) {
		if (ISNPHY(pi)) {
			/* Restore back PAPD settings after stopping the tone */
			if (NREV_IS(pi->pubpi->phy_rev, LCNXN_BASEREV))
				wlc_phy_papd_enable_nphy(pi, TRUE);
			/* FIXME4324. Dont know why only 4324 hangs if mac is not suspended */
			if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
				wlapi_suspend_mac_and_wait(pi->sh->physhim);
			wlc_phy_stopplayback_nphy(pi);
			wlc_phy_stay_in_carriersearch_nphy(pi, FALSE);
			if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
				wlapi_enable_mac(pi->sh->physhim);
		} else if (ISACPHY(pi)) {
			wlc_phy_stopplayback_acphy(pi);
			wlc_phy_stay_in_carriersearch_acphy(pi, FALSE);
			wlapi_enable_mac(pi->sh->physhim);
		} else if (ISHTPHY(pi)) {
			wlc_phy_stopplayback_htphy(pi);
			wlc_phy_stay_in_carriersearch_htphy(pi, FALSE);
		} else if (ISLCNPHY(pi)) {
			wlc_lcnphy_stop_tx_tone(pi);
		} else if (ISLCN40PHY(pi)) {
			wlc_lcn40phy_stop_tx_tone(pi);
			wlapi_enable_mac(pi->sh->physhim);
		}
	} else {
		if (ISNPHY(pi)) {
			/* use 151 since that should correspond to nominal tx output power */
			/* Can not play tone with papd bit enabled */
			if (NREV_IS(pi->pubpi->phy_rev, LCNXN_BASEREV))
				wlc_phy_papd_enable_nphy(pi, FALSE);
			if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
				wlapi_suspend_mac_and_wait(pi->sh->physhim);
			wlc_phy_stay_in_carriersearch_nphy(pi, TRUE);
			wlc_phy_tx_tone_nphy(pi, (uint32)int_val, 151, 0, 0, TRUE); /* play tone */
			if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
				wlapi_enable_mac(pi->sh->physhim);
		} else if (ISACPHY(pi)) {
			pi->phy_tx_tone_freq = pi->phy_tx_tone_freq * 1000; /* Covert to Hz */
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
			wlc_phy_tx_tone_acphy(pi, (int32)int_val, 151, 0, 0, TRUE);
		} else if (ISHTPHY(pi)) {
			wlc_phy_stay_in_carriersearch_htphy(pi, TRUE);
			wlc_phy_tx_tone_htphy(pi, (uint32)int_val, 151, 0, 0, TRUE); /* play tone */
		} else if (ISLCNPHY(pi)) {
			pi->phy_tx_tone_freq = pi->phy_tx_tone_freq * 1000; /* Covert to Hz */
			wlc_lcnphy_set_tx_tone_and_gain_idx(pi);
		} else if (ISLCN40PHY(pi)) {
			pi->phy_tx_tone_freq = pi->phy_tx_tone_freq * 1000; /* Covert to Hz */
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			wlc_lcn40phy_set_tx_tone_and_gain_idx(pi);
		}
	}
}


static void
wlc_phy_iovar_txlo_tone(phy_info_t *pi)
{
	pi->phy_tx_tone_freq = 0;
	if (ISNPHY(pi)) {
		/* use 151 since that should correspond to nominal tx output power */
		/* Can not play tone with papd bit enabled */
		if (NREV_IS(pi->pubpi->phy_rev, LCNXN_BASEREV))
			wlc_phy_papd_enable_nphy(pi, FALSE);
		if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
		wlc_phy_stay_in_carriersearch_nphy(pi, TRUE);
		wlc_phy_tx_tone_nphy(pi, 0, 151, 0, 0, TRUE); /* play tone */
		if (NREV_GE(pi->pubpi->phy_rev, LCNXN_BASEREV + 3))
			wlapi_enable_mac(pi->sh->physhim);
	} else if (ISACPHY(pi)) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		wlc_phy_stay_in_carriersearch_acphy(pi, TRUE);
		wlc_phy_tx_tone_acphy(pi, 0, 151, 0, 0, TRUE);
	} else if (ISHTPHY(pi)) {
		wlc_phy_stay_in_carriersearch_htphy(pi, TRUE);
		wlc_phy_tx_tone_htphy(pi, 0, 151, 0, 0, TRUE); /* play tone */
	} else if (ISLCNPHY(pi)) {
		wlc_lcnphy_set_tx_tone_and_gain_idx(pi);
	} else if (ISLCN40PHY(pi)) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		wlc_lcn40phy_set_tx_tone_and_gain_idx(pi);
	}
}

#endif 

#if defined(WLTEST)
static int
wlc_phy_iovar_txrx_chain(phy_info_t *pi, int32 int_val, int32 *ret_int_ptr, bool set)
{
	int err = BCME_OK;

	if (ISHTPHY(pi))
		return BCME_UNSUPPORTED;

	if (!set) {
		if (ISNPHY(pi)) {
			*ret_int_ptr = (int)pi->nphy_txrx_chain;
		}
	} else {
		if (ISNPHY(pi)) {
			if ((int_val != AUTO) && (int_val != WLC_N_TXRX_CHAIN0) &&
				(int_val != WLC_N_TXRX_CHAIN1)) {
				err = BCME_RANGE;
				goto end;
			}

			if (pi->nphy_txrx_chain != (int8)int_val) {
				pi->nphy_txrx_chain = (int8)int_val;
				if (pi->sh->up) {
					wlapi_suspend_mac_and_wait(pi->sh->physhim);
					phy_utils_phyreg_enter(pi);
					wlc_phy_stf_chain_upd_nphy(pi);
					wlc_phy_force_rfseq_nphy(pi, NPHY_RFSEQ_RESET2RX);
					phy_utils_phyreg_exit(pi);
					wlapi_enable_mac(pi->sh->physhim);
				}
			}
		}
	}
end:
	return err;
}

static void
wlc_phy_iovar_bphy_testpattern(phy_info_t *pi, uint8 testpattern, bool enable)
{
	bool existing_enable = FALSE;

	/* WL out check */
	if (pi->sh->up) {
		PHY_ERROR(("wl%d: %s: This function needs to be called after 'wl out'\n",
		          pi->sh->unit, __FUNCTION__));
		return;
	}

	/* confirm band is locked to 2G */
	if (!CHSPEC_IS2G(pi->radio_chanspec)) {
		PHY_ERROR(("wl%d: %s: Band needs to be locked to 2G (b)\n",
		          pi->sh->unit, __FUNCTION__));
		return;
	}

	if (NREV_LT(pi->pubpi->phy_rev, 2) || ISHTPHY(pi)) {
		PHY_ERROR(("wl%d: %s: This function is supported only for NPHY PHY_REV > 1\n",
		          pi->sh->unit, __FUNCTION__));
		return;
	}

	if (testpattern == NPHY_TESTPATTERN_BPHY_EVM) {    /* CW CCK for EVM testing */
		existing_enable = (bool) pi->phy_bphy_evm;
	} else if (testpattern == NPHY_TESTPATTERN_BPHY_RFCS) { /* RFCS testpattern */
		existing_enable = (bool) pi->phy_bphy_rfcs;
	} else {
		PHY_ERROR(("Testpattern needs to be between [0 (BPHY_EVM), 1 (BPHY_RFCS)]\n"));
		ASSERT(0);
	}

	if (ISNPHY(pi)) {
		wlc_phy_bphy_testpattern_nphy(pi, testpattern, enable, existing_enable);
	} else {
		PHY_ERROR(("support yet to be added\n"));
		ASSERT(0);
	}

	/* Return state of testpattern enables */
	if (testpattern == NPHY_TESTPATTERN_BPHY_EVM) {    /* CW CCK for EVM testing */
		pi->phy_bphy_evm = enable;
	} else if (testpattern == NPHY_TESTPATTERN_BPHY_RFCS) { /* RFCS testpattern */
		pi->phy_bphy_rfcs = enable;
	}
}

static void
wlc_phy_iovar_scraminit(phy_info_t *pi, int8 scraminit)
{
	pi->phy_scraminit = (int8)scraminit;
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);

	if (ISNPHY(pi)) {
		wlc_phy_test_scraminit_nphy(pi, scraminit);
	} else if (ISHTPHY(pi)) {
		wlc_phy_test_scraminit_htphy(pi, scraminit);
	} else if (ISACPHY(pi)) {
		wlc_phy_test_scraminit_acphy(pi, scraminit);
	} else {
		PHY_ERROR(("support yet to be added\n"));
		ASSERT(0);
	}

	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);
}

static void
wlc_phy_iovar_force_rfseq(phy_info_t *pi, uint8 int_val)
{
	phy_utils_phyreg_enter(pi);
	if (ISNPHY(pi)) {
		wlc_phy_force_rfseq_nphy(pi, int_val);
	} else if (ISHTPHY(pi)) {
		wlc_phy_force_rfseq_htphy(pi, int_val);
	}
	phy_utils_phyreg_exit(pi);
}

static void
wlc_phy_iovar_tx_tone_hz(phy_info_t *pi, int32 int_val)
{
	pi->phy_tx_tone_freq = (int32) int_val;

	if (ISLCNPHY(pi)) {
		wlc_lcnphy_set_tx_tone_and_gain_idx(pi);
	} else if (ISLCN40PHY(pi)) {
		wlc_lcn40phy_set_tx_tone_and_gain_idx(pi);
	}
}

static void
wlc_phy_iovar_tx_tone_stop(phy_info_t *pi)
{
	if (ISLCNPHY(pi)) {
		wlc_lcnphy_stop_tx_tone(pi);
	} else if (ISLCN40PHY(pi)) {
		wlc_lcn40phy_stop_tx_tone(pi);
	}
}

static int16
wlc_phy_iovar_test_tssi(phy_info_t *pi, uint8 val, uint8 pwroffset)
{
	int16 tssi = 0;
	if (ISNPHY(pi)) {
		tssi = (int16) wlc_phy_test_tssi_nphy(pi, val, pwroffset);
	} else if (ISHTPHY(pi)) {
		tssi = (int16) wlc_phy_test_tssi_htphy(pi, val, pwroffset);
	} else if (ISACPHY(pi)) {
		tssi = (int16) wlc_phy_test_tssi_acphy(pi, val, pwroffset);
	}
	return tssi;
}

static int16
wlc_phy_iovar_test_idletssi(phy_info_t *pi, uint8 val)
{
	int16 idletssi = INVALID_IDLETSSI_VAL;
	if (ISACPHY(pi)) {
		idletssi = (int16) wlc_phy_test_idletssi_acphy(pi, val);
	}
	return idletssi;
}

static int16
wlc_phy_iovar_setrptbl(phy_info_t *pi)
{
	if (ISACPHY(pi) && (!ACMAJORREV_1(pi->pubpi->phy_rev))) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		wlc_phy_populate_recipcoeffs_acphy(pi);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		return 0;
	}

	return BCME_UNSUPPORTED;
}

static int16
wlc_phy_iovar_forceimpbf(phy_info_t *pi)
{
	if (ISACPHY(pi) && (!ACMAJORREV_1(pi->pubpi->phy_rev))) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		phy_utils_write_phyreg(pi, ACPHY_BfeConfigReg0(pi->pubpi->phy_rev),
		                       BFECONFIGREF_FORCEVAL);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		return 0;
	}

	return BCME_UNSUPPORTED;
}

static int16
wlc_phy_iovar_forcesteer(phy_info_t *pi, uint8 enable)
{
#if (ACCONF || ACCONF2) && defined(WL_BEAMFORMING)
	uint16 bfmcon_val      = 0;
	uint16 bfridx_pos_val  = 0;
	uint16 refresh_thr_val = 0;
	uint16 shm_base, addr1, addr2;

	if (ISACPHY(pi) && (!ACMAJORREV_1(pi->pubpi->phy_rev))) {
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);

		bfmcon_val      = enable ? BFMCON_FORCEVAL      : BFMCON_RELEASEVAL;
		bfridx_pos_val  = enable ? BFRIDX_POS_FORCEVAL  : BFRIDX_POS_RELEASEVAL;
		refresh_thr_val = enable ? REFRESH_THR_FORCEVAL : REFRESH_THR_RELEASEVAL;

		shm_base = wlapi_bmac_read_shm(pi->sh->physhim, M_BFI_BLK_PTR);
		addr1 = (shm_base + M_BFI_REFRESH_THR_OFFSET) * 2;
		addr2 = (shm_base + C_BFI_BFRIDX_POS)* 2;

		phy_utils_write_phyreg(pi, ACPHY_BfmCon(pi->pubpi->phy_rev), bfmcon_val);
		wlapi_bmac_write_shm(pi->sh->physhim, addr1, refresh_thr_val);
		wlapi_bmac_write_shm(pi->sh->physhim, addr2, bfridx_pos_val);

		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		return 0;
	}
#endif /* (ACCONF || ACCONF2) && WL_BEAMFORMING */

	return BCME_UNSUPPORTED;
}

static void
wlc_phy_iovar_rxcore_enable(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr,
	bool set)
{
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);

	if (set) {
		if (ISNPHY(pi)) {
			wlc_phy_rxcore_setstate_nphy((wlc_phy_t *)pi, (uint8) int_val, 0);
		} else if (ISHTPHY(pi)) {
			wlc_phy_rxcore_setstate_htphy((wlc_phy_t *)pi, (uint8) int_val);
		} else if (ISACPHY(pi)) {
			wlc_phy_rxcore_setstate_acphy((wlc_phy_t *)pi, (uint8) int_val);
		}
	} else {
		if (ISNPHY(pi)) {
			*ret_int_ptr =  (uint32)wlc_phy_rxcore_getstate_nphy((wlc_phy_t *)pi);
		} else if (ISHTPHY(pi)) {
			*ret_int_ptr =  (uint32)wlc_phy_rxcore_getstate_htphy((wlc_phy_t *)pi);
		} else if (ISACPHY(pi)) {
			*ret_int_ptr =  (uint32)wlc_phy_rxcore_getstate_acphy((wlc_phy_t *)pi);
		}
	}

	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);
}

#endif 

static int
wlc_phy_iovar_set_btc_restage_rxgain(phy_info_t *pi, int32 set_val)
{
	int err = BCME_OK;

	if (ISHTPHY(pi) && (IS_X29B_BOARDTYPE(pi) ||
	                    IS_X29D_BOARDTYPE(pi) || IS_X33_BOARDTYPE(pi))) {
		phy_info_htphy_t *pi_ht = (phy_info_htphy_t *)pi->u.pi_htphy;

		if ((set_val < 0) || (set_val > 1)) {
			return BCME_RANGE;
		}
		if (SCAN_RM_IN_PROGRESS(pi)) {
			return BCME_NOTREADY;
		}
		if (IS_X29B_BOARDTYPE(pi)) {
			if ((pi->sh->interference_mode != INTERFERE_NONE) &&
			     (pi->sh->interference_mode != NON_WLAN)) {
				return BCME_UNSUPPORTED;
			}
		}
		if ((IS_X29D_BOARDTYPE(pi) || IS_X33_BOARDTYPE(pi)) &&
		    !CHSPEC_IS2G(pi->radio_chanspec)) {
			return BCME_BADBAND;
		}

		if (((set_val == 0) && pi_ht->btc_restage_rxgain) ||
		    ((set_val == 1) && !pi_ht->btc_restage_rxgain)) {
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			wlc_phy_btc_restage_rxgain_htphy(pi, (bool)set_val);
			wlapi_enable_mac(pi->sh->physhim);
		}
	} else if (ISACPHY(pi)) {
#ifndef WLC_DISABLE_ACI
		phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
#endif /* !WLC_DISABLE_ACI */
		if ((set_val < 0) || (set_val > 7)) {
			return BCME_RANGE;
		}
		if (SCAN_RM_IN_PROGRESS(pi)) {
			return BCME_NOTREADY;
		}
#ifndef WLC_DISABLE_ACI
		if (((set_val == 0) && (pi_ac->btc_mode != 0)) ||
		    ((set_val != 0) && (pi_ac->btc_mode != set_val))) {
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			wlc_phy_desense_btcoex_acphy(pi, set_val);
			wlapi_enable_mac(pi->sh->physhim);
		}
#endif /* !WLC_DISABLE_ACI */
	} else if (CHIPID_4324X_MEDIA_FAMILY(pi)) {

		const uint8 num_dev_thres = 2;	/* Make configurable later */
		uint16 num_bt_devices;

		/* pi->bt_shm_addr is set by phy_init */
		if (pi->bt_shm_addr == 0) {
			return BCME_NOTUP;
		}

		/* Read number of BT task and desense configuration for SHM */
		num_bt_devices  = wlapi_bmac_read_shm(pi->sh->physhim,
			pi->bt_shm_addr + M_BTCX_NUM_TASKS);

		/* If more than num_dev_thres (two) tasks, no interference override
		 * and not already in manual ACI override mode
		 */
		if ((num_dev_thres > 0) && (num_bt_devices >= num_dev_thres) &&
			(!pi->sh->interference_mode_override) &&
			(pi->sh->interference_mode != WLAN_MANUAL)) {

			wlc_phy_set_interference_override_mode(pi, WLAN_MANUAL);

		} else if ((num_bt_devices < num_dev_thres) &&
			(pi->sh->interference_mode_override)) {

			wlc_phy_set_interference_override_mode(pi, INTERFERE_OVRRIDE_OFF);
		}
		/* Return "not BCME_OK" so that this function will be called every time
		 * wlc_btcx_watchdog is called
		 */
		err = BCME_BUSY;

	} else {
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovar_get_btc_restage_rxgain(phy_info_t *pi, int32 *ret_val)
{
	if (ISHTPHY(pi) && (IS_X29B_BOARDTYPE(pi) || IS_X29D_BOARDTYPE(pi) ||
	                    IS_X33_BOARDTYPE(pi))) {
		if ((IS_X29D_BOARDTYPE(pi) || IS_X33_BOARDTYPE(pi)) &&
		    !CHSPEC_IS2G(pi->radio_chanspec)) {
			return BCME_BADBAND;
		}
		*ret_val = (int32)pi->u.pi_htphy->btc_restage_rxgain;
		return BCME_OK;
	} else if (ISACPHY(pi)) {
	  *ret_val = (int32)pi->u.pi_acphy->btc_mode;
		return BCME_OK;
	} else
		return BCME_UNSUPPORTED;
}

static int
wlc_phy_iovar_set_dssf(phy_info_t *pi, int32 set_val)
{
	if (ISACPHY(pi) && PHY_ILNA(pi)) {
	  phy_utils_write_phyreg(pi, ACPHY_DSSF_C_CTRL(pi->pubpi->phy_rev), (uint16) set_val);

		return BCME_OK;
	}

	return BCME_UNSUPPORTED;
}

static int
wlc_phy_iovar_get_dssf(phy_info_t *pi, int32 *ret_val)
{
	if (ISACPHY(pi) && PHY_ILNA(pi)) {
		*ret_val = (int32) phy_utils_read_phyreg(pi, ACPHY_DSSF_C_CTRL(pi->pubpi->phy_rev));

		return BCME_OK;
	}

	return BCME_UNSUPPORTED;
}

static int
wlc_phy_iovar_oclscd(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr,
	bool set)
{
	int err = BCME_OK;
	uint8 coremask;

	if (!pi->sh->clk)
		return BCME_NOCLK;

	coremask = ((phy_utils_read_phyreg(pi, NPHY_CoreConfig) & NPHY_CoreConfig_CoreMask_MASK)
		>> NPHY_CoreConfig_CoreMask_SHIFT);

	if (!ISNPHY(pi))
		return BCME_UNSUPPORTED;
	else if (NREV_LT(pi->pubpi->phy_rev, LCNXN_BASEREV + 2))
		return BCME_UNSUPPORTED;

	if (!set) {
		if (ISNPHY(pi)) {
			*ret_int_ptr = pi->nphy_oclscd;
		}

	} else {
		if (ISNPHY(pi)) {

			if ((int_val > 3) || (int_val < 0))
				return BCME_RANGE;

			if (int_val == 2)
				return BCME_BADARG;

			if ((coremask < 3) && (int_val != 0))
				return BCME_NORESOURCE;

			pi->nphy_oclscd = (uint8)int_val;
			/* suspend mac */
			wlapi_suspend_mac_and_wait(pi->sh->physhim);

			wlc_phy_set_oclscd_nphy(pi);

			/* resume mac */
			wlapi_enable_mac(pi->sh->physhim);
		}
	}
	return err;
}

/* Debug functionality. Is called via an iovar. */
static int
wlc_phy_iovar_prog_lnldo2(phy_info_t *pi, int32 int_val, bool bool_val, int32 *ret_int_ptr,
	bool set)
{
	int err = BCME_OK;
	uint8 lnldo2_val = 0;
	uint32 reg_value = 0;

	if (!ISNPHY(pi))
		return BCME_UNSUPPORTED;
	else if (!CHIPID_4324X_IPA_FAMILY(pi))
		return BCME_UNSUPPORTED;

	if (!set) {
		/* READ */
		wlc_si_pmu_regcontrol_access(pi, 5, &reg_value, 0);
		*ret_int_ptr = (int32)((reg_value & 0xff) >> 1);
	} else {
		/* WRITE */
		lnldo2_val = (uint8)(int_val & 0xff);
		*ret_int_ptr = wlc_phy_lnldo2_war_nphy(pi, 1, lnldo2_val);
	}
	return err;
}

/* handler for iovar modules */
typedef int (*iovar_module_t)(phy_info_t *pi, uint32 actionid, uint16 type, void *p,
	uint plen, void *a, int alen, int vsize);

/* Dispatch phy iovars */
int
wlc_phy_iovar_dispatch(phy_info_t *pi, uint32 actionid, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	bool bool_val;
	int err = BCME_OK;

	iovar_module_t iovar_module_list[] = {
		wlc_phy_iovars_generic,
		wlc_phy_iovars_aci,
		wlc_phy_iovars_rssi,
		wlc_phy_iovars_calib,
		wlc_phy_iovars_txpwrctl,
		wlc_phy_iovars_phy_specific,
#ifdef SAMPLE_COLLECT
		phy_iovars_sample_collect,
#endif
		NULL
	};

	iovar_module_t *module;
	module = iovar_module_list;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	BCM_REFERENCE(bool_val);

	do {
		err = (*module)(pi, actionid, -1, p, plen, a, alen, vsize);
		++module;
	} while ((*module != NULL) && (err == BCME_UNSUPPORTED));

	if (err == BCME_UNSUPPORTED)
		err = wlc_phy_iovar_dispatch_old(pi, actionid, p, a, vsize, int_val, bool_val);

	return err;
}

static int
wlc_phy_iovar_dispatch_old(phy_info_t *pi, uint32 actionid, void *p, void *a, int vsize,
	int32 int_val, bool bool_val)
{
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	phy_info_nphy_t *pi_nphy;

	pi_nphy = pi->u.pi_nphy;
	BCM_REFERENCE(pi_nphy);
	BCM_REFERENCE(ret_int_ptr);

	switch (actionid) {
#if NCONF
#if defined(BCMDBG)
	case IOV_SVAL(IOV_NPHY_INITGAIN):
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		wlc_phy_setinitgain_nphy(pi, (uint16) int_val);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		break;

	case IOV_SVAL(IOV_NPHY_HPVGA1GAIN):
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		wlc_phy_sethpf1gaintbl_nphy(pi, (int8) int_val);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		break;

	case IOV_SVAL(IOV_NPHY_TX_TEMP_TONE): {
		uint16 orig_BBConfig;
		uint16 m0m1;
		nphy_txgains_t target_gain;

		if ((uint32)int_val > 0) {
			pi->phy_tx_tone_freq = (uint32) int_val;
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			phy_utils_phyreg_enter(pi);
			wlc_phy_stay_in_carriersearch_nphy(pi, TRUE);

			/* Save the bbmult values,since it gets overwritten by mimophy_tx_tone() */
			wlc_phy_table_read_nphy(pi, 15, 1, 87, 16, &m0m1);

			/* Disable the re-sampler (in case we are in spur avoidance mode) */
			orig_BBConfig = phy_utils_read_phyreg(pi, NPHY_BBConfig);
			phy_utils_mod_phyreg(pi, NPHY_BBConfig,
			                     NPHY_BBConfig_resample_clk160_MASK, 0);

			/* read current tx gain and use as target_gain */
			wlc_phy_get_tx_gain_nphy(pi, &target_gain);

			PHY_ERROR(("Tx gain core 0: target gain: ipa = %d,"
			         " pad = %d, pga = %d, txgm = %d, txlpf = %d\n",
			         target_gain.ipa[0], target_gain.pad[0], target_gain.pga[0],
			         target_gain.txgm[0], target_gain.txlpf[0]));

			PHY_ERROR(("Tx gain core 1: target gain: ipa = %d,"
			         " pad = %d, pga = %d, txgm = %d, txlpf = %d\n",
			         target_gain.ipa[0], target_gain.pad[1], target_gain.pga[1],
			         target_gain.txgm[1], target_gain.txlpf[1]));

			/* play a tone for 10 secs and then stop it and return */
			wlc_phy_tx_tone_nphy(pi, (uint32)int_val, 250, 0, 0, FALSE);

			/* Now restore the original bbmult values */
			wlc_phy_table_write_nphy(pi, 15, 1, 87, 16, &m0m1);
			wlc_phy_table_write_nphy(pi, 15, 1, 95, 16, &m0m1);

			OSL_DELAY(10000000);
			wlc_phy_stopplayback_nphy(pi);

			/* Restore the state of the re-sampler
			   (in case we are in spur avoidance mode)
			*/
			phy_utils_write_phyreg(pi, NPHY_BBConfig, orig_BBConfig);

			wlc_phy_stay_in_carriersearch_nphy(pi, FALSE);
			phy_utils_phyreg_exit(pi);
			wlapi_enable_mac(pi->sh->physhim);
		}
		break;
	}
	case IOV_SVAL(IOV_NPHY_CAL_RESET):
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		wlc_phy_cal_reset_nphy(pi, (uint32) int_val);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		break;

	case IOV_GVAL(IOV_NPHY_EST_TONEPWR):
	case IOV_GVAL(IOV_PHY_EST_TONEPWR): {
		int32 dBm_power[2];
		uint16 orig_BBConfig;
		uint16 m0m1;

		if (ISNPHY(pi)) {
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			phy_utils_phyreg_enter(pi);

			/* Save the bbmult values, since it gets overwritten
			   by mimophy_tx_tone()
			*/
			wlc_phy_table_read_nphy(pi, 15, 1, 87, 16, &m0m1);

			/* Disable the re-sampler (in case we are in spur avoidance mode) */
			orig_BBConfig = phy_utils_read_phyreg(pi, NPHY_BBConfig);
			phy_utils_mod_phyreg(pi, NPHY_BBConfig,
			                     NPHY_BBConfig_resample_clk160_MASK, 0);
			pi->phy_tx_tone_freq = (uint32) 4000;

			/* play a tone for 10 secs */
			wlc_phy_tx_tone_nphy(pi, (uint32)4000, 250, 0, 0, FALSE);

			/* Now restore the original bbmult values */
			wlc_phy_table_write_nphy(pi, 15, 1, 87, 16, &m0m1);
			wlc_phy_table_write_nphy(pi, 15, 1, 95, 16, &m0m1);

			OSL_DELAY(10000000);
			wlc_phy_est_tonepwr_nphy(pi, dBm_power, 128);
			wlc_phy_stopplayback_nphy(pi);

			/* Restore the state of the re-sampler
			   (in case we are in spur avoidance mode)
			*/
			phy_utils_write_phyreg(pi, NPHY_BBConfig, orig_BBConfig);

			phy_utils_phyreg_exit(pi);
			wlapi_enable_mac(pi->sh->physhim);

			int_val = dBm_power[0]/4;
			bcopy(&int_val, a, vsize);
			break;
		} else {
			err = BCME_UNSUPPORTED;
			break;
		}
	}

	case IOV_GVAL(IOV_NPHY_RFSEQ_TXGAIN): {
		uint16 rfseq_tx_gain[2];
		wlc_phy_table_read_nphy(pi, NPHY_TBL_ID_RFSEQ, 2, 0x110, 16, rfseq_tx_gain);
		int_val = (((uint32) rfseq_tx_gain[1] << 16) | ((uint32) rfseq_tx_gain[0]));
		bcopy(&int_val, a, vsize);
		break;
	}

	case IOV_SVAL(IOV_PHY_SPURAVOID):
		if ((int_val != SPURAVOID_DISABLE) && (int_val != SPURAVOID_AUTO) &&
		    (int_val != SPURAVOID_FORCEON) && (int_val != SPURAVOID_FORCEON2)) {
			err = BCME_RANGE;
			break;
		}

		pi->phy_spuravoid = (int8)int_val;
		break;

	case IOV_GVAL(IOV_PHY_SPURAVOID):
		int_val = pi->phy_spuravoid;
		bcopy(&int_val, a, vsize);
		break;
#endif /* defined(BCMDBG) */

#if defined(WLTEST)
	case IOV_GVAL(IOV_NPHY_CCK_PWR_OFFSET):
		if (ISNPHY(pi)) {
			int_val =  pi_nphy->nphy_cck_pwr_err_adjust;
			bcopy(&int_val, a, vsize);
		}
		break;
	case IOV_GVAL(IOV_NPHY_CAL_SANITY):
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		*ret_int_ptr = (uint32)wlc_phy_cal_sanity_nphy(pi);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		break;

	case IOV_GVAL(IOV_NPHY_BPHY_EVM):
		*ret_int_ptr = pi->phy_bphy_evm;
		break;


	case IOV_SVAL(IOV_NPHY_BPHY_EVM):
		wlc_phy_iovar_bphy_testpattern(pi, NPHY_TESTPATTERN_BPHY_EVM, (bool) int_val);
		break;

	case IOV_GVAL(IOV_NPHY_BPHY_RFCS):
		*ret_int_ptr = pi->phy_bphy_rfcs;
		break;

	case IOV_SVAL(IOV_NPHY_BPHY_RFCS):
		wlc_phy_iovar_bphy_testpattern(pi, NPHY_TESTPATTERN_BPHY_RFCS, (bool) int_val);
		break;

	case IOV_GVAL(IOV_NPHY_SCRAMINIT):
		*ret_int_ptr = pi->phy_scraminit;
		break;

	case IOV_SVAL(IOV_NPHY_SCRAMINIT):
		wlc_phy_iovar_scraminit(pi, pi->phy_scraminit);
		break;

	case IOV_SVAL(IOV_NPHY_RFSEQ):
		wlc_phy_iovar_force_rfseq(pi, (uint8)int_val);
		break;

	case IOV_GVAL(IOV_NPHY_TXIQLOCAL): {
		nphy_txgains_t target_gain;
		uint8 tx_pwr_ctrl_state;
		if (ISNPHY(pi)) {

			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			phy_utils_phyreg_enter(pi);

			/* read current tx gain and use as target_gain */
			wlc_phy_get_tx_gain_nphy(pi, &target_gain);
			tx_pwr_ctrl_state = pi->nphy_txpwrctrl;
			wlc_phy_txpwrctrl_enable_nphy(pi, PHY_TPC_HW_OFF);

			/* want outer (0,1) ants so T/R works properly for CB2 2x3 switch, */
			if (pi->antsel_type == ANTSEL_2x3) {
				wlc_phy_antsel_init_nphy((wlc_phy_t *)pi, TRUE);
			}

			err = wlc_phy_cal_txiqlo_nphy(pi, target_gain, TRUE, FALSE);
			if (err)
				break;
			wlc_phy_txpwrctrl_enable_nphy(pi, tx_pwr_ctrl_state);
			phy_utils_phyreg_exit(pi);
			wlapi_enable_mac(pi->sh->physhim);
		}
		*ret_int_ptr = 0;
		break;
	}
	case IOV_SVAL(IOV_NPHY_RXIQCAL): {
		nphy_txgains_t target_gain;
		uint8 tx_pwr_ctrl_state;


		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);

		/* read current tx gain and use as target_gain */
		wlc_phy_get_tx_gain_nphy(pi, &target_gain);
		tx_pwr_ctrl_state = pi->nphy_txpwrctrl;
		wlc_phy_txpwrctrl_enable_nphy(pi, PHY_TPC_HW_OFF);
#ifdef RXIQCAL_FW_WAR
		if (wlc_phy_cal_rxiq_nphy_fw_war(pi, target_gain, 0, (bool)int_val, 0x3) != BCME_OK)
#else
		if (wlc_phy_cal_rxiq_nphy(pi, target_gain, 0, (bool)int_val, 0x3) != BCME_OK)
#endif
		{
			break;
		}
		wlc_phy_txpwrctrl_enable_nphy(pi, tx_pwr_ctrl_state);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		int_val = 0;
		bcopy(&int_val, a, vsize);
		break;
	}
	case IOV_GVAL(IOV_NPHY_RXCALPARAMS):
		if (ISNPHY(pi)) {
			*ret_int_ptr = pi_nphy->nphy_rxcalparams;
		}
		break;

	case IOV_SVAL(IOV_NPHY_RXCALPARAMS):
		if (ISNPHY(pi)) {
			pi_nphy->nphy_rxcalparams = (uint32)int_val;
		}
		break;

	case IOV_GVAL(IOV_NPHY_TXPWRCTRL):
		wlc_phy_iovar_txpwrctrl(pi, int_val, bool_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_NPHY_TXPWRCTRL):
		err = wlc_phy_iovar_txpwrctrl(pi, int_val, bool_val, ret_int_ptr, TRUE);
		break;

	case IOV_GVAL(IOV_NPHY_RSSISEL):
		*ret_int_ptr = pi->nphy_rssisel;
		break;

	case IOV_SVAL(IOV_NPHY_RSSISEL):
		pi->nphy_rssisel = (uint8)int_val;

		if (!pi->sh->up)
			break;

		if (pi->nphy_rssisel < 0) {
			phy_utils_phyreg_enter(pi);
			wlc_phy_rssisel_nphy(pi, RADIO_MIMO_CORESEL_OFF, 0);
			phy_utils_phyreg_exit(pi);
		} else {
			int32 rssi_buf[4];
			phy_utils_phyreg_enter(pi);
			wlc_phy_poll_rssi_nphy(pi, (uint8)int_val, rssi_buf, 1);
			phy_utils_phyreg_exit(pi);
		}
		break;

	case IOV_GVAL(IOV_NPHY_RSSICAL): {
		/* if down, return the value, if up, run the cal */
		if (!pi->sh->up) {
			int_val = pi->nphy_rssical;
			bcopy(&int_val, a, vsize);
			break;
		}

		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		/* run rssi cal */
		wlc_phy_rssi_cal_nphy(pi);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		int_val = pi->nphy_rssical;
		bcopy(&int_val, a, vsize);
		break;
	}

	case IOV_SVAL(IOV_NPHY_RSSICAL): {
		pi->nphy_rssical = bool_val;
		break;
	}

	case IOV_GVAL(IOV_NPHY_GPIOSEL):
	case IOV_GVAL(IOV_PHY_GPIOSEL):
		*ret_int_ptr = pi->phy_gpiosel;
		break;

	case IOV_SVAL(IOV_NPHY_GPIOSEL):
	case IOV_SVAL(IOV_PHY_GPIOSEL):
		pi->phy_gpiosel = (uint16) int_val;

		if (!pi->sh->up)
			break;

		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);
		if (ISNPHY(pi))
			wlc_phy_gpiosel_nphy(pi, (uint16)int_val);
		else if (ISHTPHY(pi))
			wlc_phy_gpiosel_htphy(pi, (uint16)int_val);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);
		break;

	case IOV_GVAL(IOV_NPHY_TX_TONE):
		*ret_int_ptr = pi->phy_tx_tone_freq;
		break;

	case IOV_SVAL(IOV_NPHY_TX_TONE):
		wlc_phy_iovar_tx_tone(pi, (uint32)int_val);
		break;

	case IOV_SVAL(IOV_NPHY_GAIN_BOOST):
		pi->nphy_gain_boost = bool_val;
		break;

	case IOV_GVAL(IOV_NPHY_GAIN_BOOST):
		*ret_int_ptr = (int32)pi->nphy_gain_boost;
		break;

	case IOV_SVAL(IOV_NPHY_ELNA_GAIN_CONFIG):
		pi->nphy_elna_gain_config = (int_val != 0) ? TRUE : FALSE;
		break;

	case IOV_GVAL(IOV_NPHY_ELNA_GAIN_CONFIG):
		*ret_int_ptr = (int32)pi->nphy_elna_gain_config;
		break;

	case IOV_GVAL(IOV_NPHY_TEST_TSSI):
		*((uint*)a) = wlc_phy_iovar_test_tssi(pi, (uint8)int_val, 0);
		break;

	case IOV_GVAL(IOV_NPHY_TEST_TSSI_OFFS):
		*((uint*)a) = wlc_phy_iovar_test_tssi(pi, (uint8)int_val, 12);
		break;

#ifdef BAND5G
	case IOV_SVAL(IOV_NPHY_5G_PWRGAIN):
		pi->phy_5g_pwrgain = bool_val;
		break;

	case IOV_GVAL(IOV_NPHY_5G_PWRGAIN):
		*ret_int_ptr = (int32)pi->phy_5g_pwrgain;
		break;
#endif /* BAND5G */

	case IOV_GVAL(IOV_NPHY_PERICAL):
		wlc_phy_iovar_perical_config(pi, int_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_NPHY_PERICAL):
		wlc_phy_iovar_perical_config(pi, int_val, ret_int_ptr, TRUE);
		break;

	case IOV_SVAL(IOV_NPHY_FORCECAL):
		err = wlc_phy_iovar_forcecal(pi, int_val, ret_int_ptr, vsize, TRUE);
		break;

#ifndef WLC_DISABLE_ACI
	case IOV_GVAL(IOV_NPHY_ACI_SCAN):
		if (SCAN_INPROG_PHY(pi)) {
			PHY_ERROR(("Scan in Progress, can execute %s\n", __FUNCTION__));
			*ret_int_ptr = -1;
		} else {
			if (pi->cur_interference_mode == INTERFERE_NONE) {
				PHY_ERROR(("interference mode is off\n"));
				*ret_int_ptr = -1;
				break;
			}

			wlapi_suspend_mac_and_wait(pi->sh->physhim);
			*ret_int_ptr = wlc_phy_aci_scan_nphy(pi);
			wlapi_enable_mac(pi->sh->physhim);
		}
		break;
#endif /* Compiling out ACI code for 4324 */
	case IOV_SVAL(IOV_NPHY_ENABLERXCORE):
		wlc_phy_iovar_rxcore_enable(pi, int_val, bool_val, ret_int_ptr, TRUE);
		break;

	case IOV_GVAL(IOV_NPHY_ENABLERXCORE):
		wlc_phy_iovar_rxcore_enable(pi, int_val, bool_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_NPHY_PAPDCALTYPE):
		if (ISNPHY(pi))
			pi_nphy->nphy_papd_cal_type = (int8) int_val;
		break;

	case IOV_GVAL(IOV_NPHY_PAPDCAL):
		if (ISNPHY(pi))
			pi_nphy->nphy_force_papd_cal = TRUE;
		int_val = 0;
		bcopy(&int_val, a, vsize);
		break;

	case IOV_SVAL(IOV_NPHY_SKIPPAPD):
		if ((int_val != 0) && (int_val != 1)) {
			err = BCME_RANGE;
			break;
		}
		if (ISNPHY(pi))
			pi_nphy->nphy_papd_skip = (uint8)int_val;
		break;

	case IOV_GVAL(IOV_NPHY_PAPDCALINDEX):
		if (ISNPHY(pi)) {
			*ret_int_ptr = (pi_nphy->nphy_papd_cal_gain_index[0] << 8) |
				pi_nphy->nphy_papd_cal_gain_index[1];
		}
		break;

	case IOV_SVAL(IOV_NPHY_CALTXGAIN): {
		uint8 tx_pwr_ctrl_state;

		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		phy_utils_phyreg_enter(pi);

		if (ISNPHY(pi)) {
			pi_nphy->nphy_cal_orig_pwr_idx[0] =
			        (uint8) ((phy_utils_read_phyreg(pi,
			                  NPHY_Core0TxPwrCtrlStatus) >> 8) & 0x7f);
			pi_nphy->nphy_cal_orig_pwr_idx[1] =
				(uint8) ((phy_utils_read_phyreg(pi,
			                  NPHY_Core1TxPwrCtrlStatus) >> 8) & 0x7f);
		}

		tx_pwr_ctrl_state = pi->nphy_txpwrctrl;
		wlc_phy_txpwrctrl_enable_nphy(pi, PHY_TPC_HW_OFF);

		wlc_phy_cal_txgainctrl_nphy(pi, int_val, TRUE);

		wlc_phy_txpwrctrl_enable_nphy(pi, tx_pwr_ctrl_state);
		phy_utils_phyreg_exit(pi);
		wlapi_enable_mac(pi->sh->physhim);

		break;
	}

	case IOV_GVAL(IOV_NPHY_VCOCAL):
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		wlc_phy_radio205x_vcocal_nphy(pi);
		wlapi_enable_mac(pi->sh->physhim);
		*ret_int_ptr = 0;
		break;

	case IOV_GVAL(IOV_NPHY_TBLDUMP_MINIDX):
		*ret_int_ptr = (int32)pi->nphy_tbldump_minidx;
		break;

	case IOV_SVAL(IOV_NPHY_TBLDUMP_MINIDX):
		pi->nphy_tbldump_minidx = (int8) int_val;
		break;

	case IOV_GVAL(IOV_NPHY_TBLDUMP_MAXIDX):
		*ret_int_ptr = (int32)pi->nphy_tbldump_maxidx;
		break;

	case IOV_SVAL(IOV_NPHY_TBLDUMP_MAXIDX):
		pi->nphy_tbldump_maxidx = (int8) int_val;
		break;

	case IOV_SVAL(IOV_NPHY_PHYREG_SKIPDUMP):
		if (pi->nphy_phyreg_skipcnt < 127) {
			pi->nphy_phyreg_skipaddr[pi->nphy_phyreg_skipcnt++] = (uint) int_val;
		}
		break;

	case IOV_GVAL(IOV_NPHY_PHYREG_SKIPDUMP):
		*ret_int_ptr = (pi->nphy_phyreg_skipcnt > 0) ?
			(int32) pi->nphy_phyreg_skipaddr[pi->nphy_phyreg_skipcnt-1] : 0;
		break;

	case IOV_SVAL(IOV_NPHY_PHYREG_SKIPCNT):
		pi->nphy_phyreg_skipcnt = (int8) int_val;
		break;

	case IOV_GVAL(IOV_NPHY_PHYREG_SKIPCNT):
		*ret_int_ptr = (int32)pi->nphy_phyreg_skipcnt;
		break;
#endif 
#endif /* NCONF */

#if defined(WLTEST)

#if LCNCONF
	case IOV_GVAL(IOV_LCNPHY_PAPDEPSTBL):
	{
		lcnphytbl_info_t tab;
		uint32 papdepstbl[PHY_PAPD_EPS_TBL_SIZE_LCNPHY];

		/* Preset PAPD eps table */
		tab.tbl_len = PHY_PAPD_EPS_TBL_SIZE_LCNPHY;
		tab.tbl_id = LCNPHY_TBL_ID_PAPDCOMPDELTATBL;
		tab.tbl_offset = 0;
		tab.tbl_width = 32;
		tab.tbl_phywidth = 32;
		tab.tbl_ptr = &papdepstbl[0];

		/* read the table */
		wlc_phy_table_read_lcnphy(pi, &tab);
		bcopy(&papdepstbl[0], a, PHY_PAPD_EPS_TBL_SIZE_LCNPHY*sizeof(uint32));
	}
	break;
#endif /* LCNCONF */
#endif 

	default:
		err = BCME_UNSUPPORTED;
	}

	return err;
}

static int
wlc_phy_iovars_txpwrctl(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	bool bool_val;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	BCM_REFERENCE(*ret_int_ptr);
	BCM_REFERENCE(bool_val);

	switch (actionid) {
#if defined(BCMDBG) || defined(WLTEST)
	case IOV_GVAL(IOV_TXINSTPWR):
		phy_utils_phyreg_enter(pi);
		/* Return the current instantaneous est. power
		 * For swpwr ctrl it's based on current TSSI value (as opposed to average)
		 */
		wlc_phy_txpower_get_instant(pi, a);
		phy_utils_phyreg_exit(pi);
		break;
#endif 
#if defined(BCMDBG) || defined(WLTEST)
	case IOV_SVAL(IOV_TSSICAL_START_IDX):
	case IOV_SVAL(IOV_TSSICAL_START):
		if (ISLCNPHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = pi->u.pi_lcnphy;
			uint16 curr_anchor;
			if (!pi->ptssi_cal) {
				pi->ptssi_cal = (tssi_cal_info_t *)MALLOC(pi->sh->osh,
					sizeof(tssi_cal_info_t));
				if (pi->ptssi_cal == NULL) {
					PHY_ERROR(("wl%d: %s: MALLOC failure\n",
						pi->sh->unit, __FUNCTION__));
					err = BCME_UNSUPPORTED;
					break;
				}
				bzero((char *)pi->ptssi_cal, sizeof(tssi_cal_info_t));
			}
			curr_anchor = pi->ptssi_cal->curr_anchor;
			wlc_lcnphy_clear_tx_power_offsets(pi);
			PHY_REG_MOD(pi, LCNPHY, TxPwrCtrlRangeCmd, cckPwrOffset,
				pi_lcn->cckPwrOffset);
			PHY_REG_MOD(pi, LCNPHY, TxPwrCtrlCmd, txPwrCtrl_en, 1);

			if (actionid == IOV_SVAL(IOV_TSSICAL_START_IDX))
				bcopy(p, &(pi->ptssi_cal->anchor_txidx[curr_anchor]),
					sizeof(uint16));
			else
				bcopy(p, &(pi->ptssi_cal->target_pwr_qdBm[curr_anchor]),
					sizeof(int));

			pi->ptssi_cal->paparams_calc_done = 0;
		} else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_SVAL(IOV_TSSICAL_POWER):
		if (ISLCNPHY(pi)) {
			uint16 curr_anchor;
			if (!pi->ptssi_cal) {
				err = BCME_UNSUPPORTED;
				break;
			}
			curr_anchor = pi->ptssi_cal->curr_anchor;
			bcopy(p, &(pi->ptssi_cal->measured_pwr_qdBm[curr_anchor]), sizeof(int));
		} else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_GVAL(IOV_TSSICAL_POWER):
		if (ISLCNPHY(pi)) {
			uint16 anchor_var[3];
			uint16 curr_anchor;
			if (!pi->ptssi_cal) {
				err = BCME_UNSUPPORTED;
				break;
			}
			curr_anchor = pi->ptssi_cal->curr_anchor;
			pi->ptssi_cal->anchor_txidx[curr_anchor] =
				wlc_lcnphy_get_current_tx_pwr_idx(pi);
			pi->ptssi_cal->anchor_tssi[curr_anchor] =
				PHY_REG_READ(pi, LCNPHY, TxPwrCtrlStatusNew4, avgTssi);
			pi->ptssi_cal->anchor_bbmult[curr_anchor] =
				wlc_lcnphy_get_bbmult_from_index(pi,
				pi->ptssi_cal->anchor_txidx[curr_anchor]);

			anchor_var[0] = pi->ptssi_cal->anchor_bbmult[curr_anchor];
			anchor_var[1] = pi->ptssi_cal->anchor_txidx[curr_anchor];
			anchor_var[2] = pi->ptssi_cal->anchor_tssi[curr_anchor];

			bcopy(anchor_var, a, 3*sizeof(uint16));

			pi->ptssi_cal->curr_anchor++;
			if (pi->ptssi_cal->curr_anchor >= MAX_NUM_ANCHORS)
				pi->ptssi_cal->curr_anchor = 0;

		} else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_GVAL(IOV_TSSICAL_PARAMS):
		if (ISLCNPHY(pi)) {
			uint16 num_anchor;
			if (!pi->ptssi_cal) {
				err = BCME_UNSUPPORTED;
				break;
			}
			if ((!pi->ptssi_cal->paparams_calc_done) &&
				(!pi->ptssi_cal->paparams_calc_in_progress)) {

				if (pi->ptssi_cal->anchor_bbmult[0]) { /* Atleast One Anchor */

					pi->ptssi_cal->paparams_calc_in_progress = 1;
					wlc_phy_tssi_cal(pi);

					pi->ptssi_cal->paparams_new[0] =
						pi->ptssi_cal->rsd.c4[0][0];
					pi->ptssi_cal->paparams_new[1] =
						pi->ptssi_cal->rsd.c4[1][0];
					pi->ptssi_cal->paparams_new[2] =
						pi->ptssi_cal->rsd.c4[2][0];
					pi->ptssi_cal->paparams_new[3] =
						pi->ptssi_cal->rsd.det_c1;
				}
				else {
					pi->ptssi_cal->paparams_new[0] = 1;
					pi->ptssi_cal->paparams_new[1] = 1;
					pi->ptssi_cal->paparams_new[2] = 1;
					pi->ptssi_cal->paparams_new[3] = 1;
				}

				pi->ptssi_cal->paparams_calc_done = 1;
			}

			if (pi->ptssi_cal->paparams_calc_done) {
				bcopy(pi->ptssi_cal->paparams_new, a, 4*sizeof(int64));
				pi->ptssi_cal->paparams_calc_in_progress = 0;
				pi->ptssi_cal->curr_anchor = 0;
				for (num_anchor = 0; num_anchor < MAX_NUM_ANCHORS; num_anchor++)
					pi->ptssi_cal->anchor_bbmult[num_anchor] = 0;
			}
		} else
			err = BCME_UNSUPPORTED;
		break;
	case IOV_SVAL(IOV_PHY_TSSITXDELAY):
		if (ISLCNPHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = pi->u.pi_lcnphy;
			pi_lcn->lcnphy_tssical_txdelay = (uint32)int_val;
		}
		break;

	case IOV_GVAL(IOV_PHY_TSSITXDELAY):
		if (ISLCNPHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = pi->u.pi_lcnphy;
			int_val = pi_lcn->lcnphy_tssical_txdelay;
			bcopy(&int_val, a, sizeof(int_val));
		}
		break;
#endif /* BCMDBG || WLTEST */

#if defined(WLTEST) || defined(ATE_BUILD)
	case IOV_GVAL(IOV_PHY_TXPWRCTRL):
		wlc_phy_iovar_txpwrctrl(pi, int_val, bool_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_TXPWRCTRL):
		err = wlc_phy_iovar_txpwrctrl(pi, int_val, bool_val, ret_int_ptr, TRUE);
		break;

	case IOV_SVAL(IOV_PHY_TXPWRINDEX):
		if (!pi->sh->clk) {
			err = BCME_NOCLK;
			break;
		}
		err = wlc_phy_iovar_txpwrindex_set(pi, p);
		break;

	case IOV_GVAL(IOV_PHY_TXPWRINDEX):
		if (!pi->sh->clk) {
			err = BCME_NOCLK;
			break;
		}
		wlc_phy_iovar_txpwrindex_get(pi, int_val, bool_val, ret_int_ptr);
		break;
	case IOV_SVAL(IOV_PHY_TONE_TXPWR):
		if (!pi->sh->clk) {
		   err = BCME_NOCLK;
		   break;
		}
		if (ISACPHY(pi)) {
		   wlc_phy_tone_pwrctrl_loop(pi, (int8)int_val);
		}
		break;
#endif 
#if defined(WLTEST)
	case IOV_GVAL(IOV_PATRIM):
		if (ISACPHY(pi))
			wlc_phy_iovar_patrim_acphy(pi, ret_int_ptr);
		else
			*ret_int_ptr = 0;
	break;

	case IOV_GVAL(IOV_PAVARS): {
		uint16 *outpa = a;
		uint16 inpa[WL_PHY_PAVARS_LEN];
		uint j = 3;	/* PA parameters start from offset 3 */

		bcopy(p, inpa, sizeof(inpa));

		outpa[0] = inpa[0]; /* Phy type */
		outpa[1] = inpa[1]; /* Band range */
		outpa[2] = inpa[2]; /* Chain */

		if (ISHTPHY(pi)) {
			if (inpa[0] != PHY_TYPE_HT) {
				outpa[0] = PHY_TYPE_NULL;
				break;
			}

			if (inpa[2] >= PHYCORENUM(pi->pubpi->phy_corenum))
				return BCME_BADARG;

			switch (inpa[1]) {
			case WL_CHAN_FREQ_RANGE_2G:
			case WL_CHAN_FREQ_RANGE_5G_BAND0:
			case WL_CHAN_FREQ_RANGE_5G_BAND1:
			case WL_CHAN_FREQ_RANGE_5G_BAND2:
			case WL_CHAN_FREQ_RANGE_5G_BAND3:
				wlc_phy_pavars_get_htphy(pi, &outpa[j], inpa[1], inpa[2]);
				break;
			default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				break;
			}
		} else if (ISNPHY(pi)) {
			srom_pwrdet_t	*pwrdet  = pi->pwrdet;

			if (inpa[0] != PHY_TYPE_N) {
				outpa[0] = PHY_TYPE_NULL;
				break;
			}
			outpa[j++] = pwrdet->pwrdet_a1[inpa[2]][inpa[1]];	/* a1 */
			outpa[j++] = pwrdet->pwrdet_b0[inpa[2]][inpa[1]];	/* b0 */
			outpa[j++] = pwrdet->pwrdet_b1[inpa[2]][inpa[1]];	/* b1 */
		} else if (ISLCNCOMMONPHY(pi)) {
			if (((inpa[0] != PHY_TYPE_LCN) && (inpa[0] != PHY_TYPE_LCN40))) {
				outpa[0] = PHY_TYPE_NULL;
				break;
			}

			if (inpa[2] != 0)
				return BCME_BADARG;
#if defined(LCNCONF)
			switch (inpa[1]) {
			case WL_CHAN_FREQ_RANGE_2G:
				outpa[j++] = pi->txpa_2g[0];		/* b0 */
				outpa[j++] = pi->txpa_2g[1];		/* b1 */
				outpa[j++] = pi->txpa_2g[2];		/* a1 */
				break;
#ifdef BAND5G
			case WL_CHAN_FREQ_RANGE_5GL:
				outpa[j++] = pi->txpa_5g_low[0];	/* b0 */
				outpa[j++] = pi->txpa_5g_low[1];	/* b1 */
				outpa[j++] = pi->txpa_5g_low[2];	/* a1 */
				break;

			case WL_CHAN_FREQ_RANGE_5GM:
				outpa[j++] = pi->txpa_5g_mid[0];	/* b0 */
				outpa[j++] = pi->txpa_5g_mid[1];	/* b1 */
				outpa[j++] = pi->txpa_5g_mid[2];	/* a1 */
				break;

			case WL_CHAN_FREQ_RANGE_5GH:
				outpa[j++] = pi->txpa_5g_hi[0];	/* b0 */
				outpa[j++] = pi->txpa_5g_hi[1];	/* b1 */
				outpa[j++] = pi->txpa_5g_hi[2];	/* a1 */
				break;
#endif /* BAND5G */
			default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[0]));
				break;
			}
#else
			return BCME_UNSUPPORTED;
#endif /* older PHYs */
		} else if (ISACPHY(pi)) {
			int chain = inpa[2];
			int freq_range;
			int num_paparams = PHY_CORE_MAX;

#ifdef WL_CHAN_FREQ_RANGE_5G_4BAND
			int n;
#endif
			freq_range = inpa[1];

			if ((BF3_TSSI_DIV_WAR(pi->u.pi_acphy)) &&
				(ACMAJORREV_1(pi->pubpi->phy_rev))) {
				num_paparams = 3;
			} else if ((ACMAJORREV_2(pi->pubpi->phy_rev) ||
				(ACMAJORREV_4(pi->pubpi->phy_rev))) &&
				BF3_TSSI_DIV_WAR(pi->u.pi_acphy)) {
				num_paparams = 4;
			} else if ((pi->u.pi_acphy->srom_tworangetssi2g) &&
			 (inpa[1] == WL_CHAN_FREQ_RANGE_2G) && pi->ipa2g_on &&
				(ACMAJORREV_1(pi->pubpi->phy_rev))) {
				num_paparams = 2;
			} else if ((pi->u.pi_acphy->srom_tworangetssi5g) &&
			 (inpa[1] != WL_CHAN_FREQ_RANGE_2G) && pi->ipa5g_on &&
				(ACMAJORREV_1(pi->pubpi->phy_rev))) {
				num_paparams = 2;
			}
			if (inpa[0] != PHY_TYPE_AC) {
				PHY_ERROR(("Wrong phy type %d\n", inpa[0]));
				outpa[0] = PHY_TYPE_NULL;
				break;
			}
			if (chain > (num_paparams - 1)) {
				PHY_ERROR(("Wrong chain number %d\n", chain));
				outpa[0] = PHY_TYPE_NULL;
				break;
			}

			if (SROMREV(pi->sh->sromrev) >= 12) {
			    srom12_pwrdet_t *pwrdet = pi->pwrdet_ac;
			    switch (freq_range) {
			    case WL_CHAN_FREQ_RANGE_2G:
				outpa[j++] = pwrdet->pwrdet_a[chain][freq_range];
				outpa[j++] = pwrdet->pwrdet_b[chain][freq_range];
				outpa[j++] = pwrdet->pwrdet_c[chain][freq_range];
				outpa[j++] = pwrdet->pwrdet_d[chain][freq_range];
				break;
			    case WL_CHAN_FREQ_RANGE_2G_40:
				outpa[j++] = pwrdet->pwrdet_a_40[chain][freq_range-6];
				outpa[j++] = pwrdet->pwrdet_b_40[chain][freq_range-6];
				outpa[j++] = pwrdet->pwrdet_c_40[chain][freq_range-6];
				outpa[j++] = pwrdet->pwrdet_d_40[chain][freq_range-6];
				break;
				/* allow compile in branches without 4BAND definition */
#ifdef WL_CHAN_FREQ_RANGE_5G_4BAND
			    case WL_CHAN_FREQ_RANGE_5G_BAND4:
				outpa[j++] = pwrdet->pwrdet_a[chain][freq_range];
				outpa[j++] = pwrdet->pwrdet_b[chain][freq_range];
				outpa[j++] = pwrdet->pwrdet_c[chain][freq_range];
				outpa[j++] = pwrdet->pwrdet_d[chain][freq_range];
				break;
			    case WL_CHAN_FREQ_RANGE_5G_BAND0:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3:
				if (ACMAJORREV_2(pi->pubpi->phy_rev) ||
				    ACMAJORREV_5(pi->pubpi->phy_rev)) {
				    outpa[j++] = pwrdet->pwrdet_a[chain][freq_range];
				    outpa[j++] = pwrdet->pwrdet_b[chain][freq_range];
				    outpa[j++] = pwrdet->pwrdet_c[chain][freq_range];
				    outpa[j++] = pwrdet->pwrdet_d[chain][freq_range];
				} else {
				    PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_BAND0_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND4_40:
				outpa[j++] = pwrdet->pwrdet_a_40[chain][freq_range-6];
				outpa[j++] = pwrdet->pwrdet_b_40[chain][freq_range-6];
				outpa[j++] = pwrdet->pwrdet_c_40[chain][freq_range-6];
				outpa[j++] = pwrdet->pwrdet_d_40[chain][freq_range-6];
				break;
			    case WL_CHAN_FREQ_RANGE_5G_BAND0_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND4_80:
				outpa[j++] = pwrdet->pwrdet_a_80[chain][freq_range-12];
				outpa[j++] = pwrdet->pwrdet_b_80[chain][freq_range-12];
				outpa[j++] = pwrdet->pwrdet_c_80[chain][freq_range-12];
				outpa[j++] = pwrdet->pwrdet_d_80[chain][freq_range-12];
				break;
			    case WL_CHAN_FREQ_RANGE_5G_5BAND:
				for (n = 1; n <= 5; n++) {
				    outpa[j++] = pwrdet->pwrdet_a[chain][n];
				    outpa[j++] = pwrdet->pwrdet_b[chain][n];
				    outpa[j++] = pwrdet->pwrdet_c[chain][n];
				    outpa[j++] = pwrdet->pwrdet_d[chain][n];
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_5BAND_40:
				for (n = 1; n <= 5; n++) {
				    outpa[j++] = pwrdet->pwrdet_a_40[chain][n];
				    outpa[j++] = pwrdet->pwrdet_b_40[chain][n];
				    outpa[j++] = pwrdet->pwrdet_c_40[chain][n];
				    outpa[j++] = pwrdet->pwrdet_d_40[chain][n];
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_5BAND_80:
				for (n = 0; n <= 4; n++) {
				    outpa[j++] = pwrdet->pwrdet_a_80[chain][n];
				    outpa[j++] = pwrdet->pwrdet_b_80[chain][n];
				    outpa[j++] = pwrdet->pwrdet_c_80[chain][n];
				    outpa[j++] = pwrdet->pwrdet_d_80[chain][n];
				}
				break;
#endif /* WL_CHAN_FREQ_RANGE_5G_4BAND */
			    default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				break;
			    }
			} else {
			    srom11_pwrdet_t *pwrdet11 = pi->pwrdet_ac;
			    switch (freq_range) {
			    case WL_CHAN_FREQ_RANGE_2G:
				outpa[j++] = pwrdet11->pwrdet_a1[chain][freq_range];
				outpa[j++] = pwrdet11->pwrdet_b0[chain][freq_range];
				outpa[j++] = pwrdet11->pwrdet_b1[chain][freq_range];
				break;
				/* allow compile in branches without 4BAND definition */
#ifdef WL_CHAN_FREQ_RANGE_5G_4BAND
			    case WL_CHAN_FREQ_RANGE_5G_4BAND:
				for (n = 1; n <= 4; n ++) {
				    outpa[j++] = pwrdet11->pwrdet_a1[chain][n];
				    outpa[j++] = pwrdet11->pwrdet_b0[chain][n];
				    outpa[j++] = pwrdet11->pwrdet_b1[chain][n];
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_BAND0:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3:
				if (ACMAJORREV_2(pi->pubpi->phy_rev) ||
				    ACMAJORREV_5(pi->pubpi->phy_rev)) {
				    outpa[j++] = pwrdet11->pwrdet_a1[chain][freq_range];
				    outpa[j++] = pwrdet11->pwrdet_b0[chain][freq_range];
				    outpa[j++] = pwrdet11->pwrdet_b1[chain][freq_range];
				} else {
				    PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				}
				break;
			    default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				break;
			    }
#endif /* WL_CHAN_FREQ_RANGE_5G_4BAND */

			}
		} else {
		    PHY_ERROR(("Unsupported PHY type!\n"));
		    err = BCME_UNSUPPORTED;
		}
	}
	    break;

	case IOV_SVAL(IOV_PAVARS): {
		uint16 inpa[WL_PHY_PAVARS_LEN];
		uint j = 3;	/* PA parameters start from offset 3 */
		int chain;
		int freq_range;
		int num_paparams;
		bcopy(p, inpa, sizeof(inpa));
		if (ISHTPHY(pi)) {
			if (inpa[0] != PHY_TYPE_HT) {
				break;
			}

			if (inpa[2] >= PHYCORENUM(pi->pubpi->phy_corenum))
				return BCME_BADARG;

			switch (inpa[1]) {
			case WL_CHAN_FREQ_RANGE_2G:
			case WL_CHAN_FREQ_RANGE_5G_BAND0:
			case WL_CHAN_FREQ_RANGE_5G_BAND1:
			case WL_CHAN_FREQ_RANGE_5G_BAND2:
			case WL_CHAN_FREQ_RANGE_5G_BAND3:
				wlc_phy_pavars_set_htphy(pi, &inpa[j], inpa[1], inpa[2]);
				break;
			default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				break;
			}
		} else if (ISNPHY(pi)) {
			srom_pwrdet_t	*pwrdet  = pi->pwrdet;

			if (inpa[0] != PHY_TYPE_N)
				break;

			if (inpa[2] > 1)
				return BCME_BADARG;

			pwrdet->pwrdet_a1[inpa[2]][inpa[1]] = inpa[j++];
			pwrdet->pwrdet_b0[inpa[2]][inpa[1]] = inpa[j++];
			pwrdet->pwrdet_b1[inpa[2]][inpa[1]] = inpa[j++];

		} else if (ISLCNCOMMONPHY(pi)) {
			if ((inpa[0] != PHY_TYPE_LCN) && (inpa[0] != PHY_TYPE_LCN40))
				break;

			if (inpa[2] != 0)
				return BCME_BADARG;
#if defined(LCNCONF)
			switch (inpa[1]) {
			case WL_CHAN_FREQ_RANGE_2G:
				pi->txpa_2g[0] = inpa[j++];	/* b0 */
				pi->txpa_2g[1] = inpa[j++];	/* b1 */
				pi->txpa_2g[2] = inpa[j++];	/* a1 */
				break;
#ifdef BAND5G
			case WL_CHAN_FREQ_RANGE_5GL:
				pi->txpa_5g_low[0] = inpa[j++];	/* b0 */
				pi->txpa_5g_low[1] = inpa[j++];	/* b1 */
				pi->txpa_5g_low[2] = inpa[j++];	/* a1 */
				break;

			case WL_CHAN_FREQ_RANGE_5GM:
				pi->txpa_5g_mid[0] = inpa[j++];	/* b0 */
				pi->txpa_5g_mid[1] = inpa[j++];	/* b1 */
				pi->txpa_5g_mid[2] = inpa[j++];	/* a1 */
				break;

			case WL_CHAN_FREQ_RANGE_5GH:
				pi->txpa_5g_hi[0] = inpa[j++];	/* b0 */
				pi->txpa_5g_hi[1] = inpa[j++];	/* b1 */
				pi->txpa_5g_hi[2] = inpa[j++];	/* a1 */
				break;
#endif /* BAND5G */
			default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[0]));
				break;
			}
#else
			return BCME_UNSUPPORTED;
#endif /* Older PHYs */
		} else if (ISACPHY(pi)) {
#ifdef WL_CHAN_FREQ_RANGE_5G_4BAND
			int n;
#endif
			chain = inpa[2];
			freq_range = inpa[1];
			num_paparams = PHY_CORE_MAX;

			if ((BF3_TSSI_DIV_WAR(pi->u.pi_acphy)) &&
				(ACMAJORREV_1(pi->pubpi->phy_rev))) {
				num_paparams = 3;
			} else if ((ACMAJORREV_2(pi->pubpi->phy_rev) ||
				ACMAJORREV_4(pi->pubpi->phy_rev)) &&
				BF3_TSSI_DIV_WAR(pi->u.pi_acphy)) {
				num_paparams = 4;
			} else if ((pi->u.pi_acphy->srom_tworangetssi2g) &&
			 (inpa[1] == WL_CHAN_FREQ_RANGE_2G) && pi->ipa2g_on &&
				(ACMAJORREV_1(pi->pubpi->phy_rev))) {
				num_paparams = 2;
			} else if ((pi->u.pi_acphy->srom_tworangetssi5g) &&
			 (inpa[1] != WL_CHAN_FREQ_RANGE_2G) && pi->ipa5g_on &&
				(ACMAJORREV_1(pi->pubpi->phy_rev))) {
				num_paparams = 2;
			}

			if (inpa[0] != PHY_TYPE_AC) {
				PHY_ERROR(("Wrong phy type %d\n", inpa[0]));
				break;
			}

			if (chain > (num_paparams - 1)) {
				PHY_ERROR(("Wrong chain number %d\n", chain));
				break;
			}

			if (SROMREV(pi->sh->sromrev) >= 12) {
			    srom12_pwrdet_t *pwrdet = pi->pwrdet_ac;
			    switch (freq_range) {
			    case WL_CHAN_FREQ_RANGE_2G:
				pwrdet->pwrdet_a[chain][freq_range] = inpa[j++];
				pwrdet->pwrdet_b[chain][freq_range] = inpa[j++];
				pwrdet->pwrdet_c[chain][freq_range] = inpa[j++];
				pwrdet->pwrdet_d[chain][freq_range] = inpa[j++];
				break;
			    case WL_CHAN_FREQ_RANGE_2G_40:
				pwrdet->pwrdet_a_40[chain][freq_range-6] = inpa[j++];
				pwrdet->pwrdet_b_40[chain][freq_range-6] = inpa[j++];
				pwrdet->pwrdet_c_40[chain][freq_range-6] = inpa[j++];
				pwrdet->pwrdet_d_40[chain][freq_range-6] = inpa[j++];
				break;
				/* allow compile in branches without 4BAND definition */
#ifdef WL_CHAN_FREQ_RANGE_5G_4BAND
			    case WL_CHAN_FREQ_RANGE_5G_BAND4:
				pwrdet->pwrdet_a[chain][freq_range] = inpa[j++];
				pwrdet->pwrdet_b[chain][freq_range] = inpa[j++];
				pwrdet->pwrdet_c[chain][freq_range] = inpa[j++];
				pwrdet->pwrdet_d[chain][freq_range] = inpa[j++];
				break;

			    case WL_CHAN_FREQ_RANGE_5G_BAND0:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3:
				if (ACMAJORREV_2(pi->pubpi->phy_rev) ||
				    ACMAJORREV_5(pi->pubpi->phy_rev)) {
				    pwrdet->pwrdet_a[chain][freq_range] = inpa[j++];
				    pwrdet->pwrdet_b[chain][freq_range] = inpa[j++];
				    pwrdet->pwrdet_c[chain][freq_range] = inpa[j++];
				    pwrdet->pwrdet_d[chain][freq_range] = inpa[j++];
				} else {
				    PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_BAND0_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3_40:
			    case WL_CHAN_FREQ_RANGE_5G_BAND4_40:
				pwrdet->pwrdet_a_40[chain][freq_range-6] = inpa[j++];
				pwrdet->pwrdet_b_40[chain][freq_range-6] = inpa[j++];
				pwrdet->pwrdet_c_40[chain][freq_range-6] = inpa[j++];
				pwrdet->pwrdet_d_40[chain][freq_range-6] = inpa[j++];
				break;
			    case WL_CHAN_FREQ_RANGE_5G_BAND0_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3_80:
			    case WL_CHAN_FREQ_RANGE_5G_BAND4_80:
				pwrdet->pwrdet_a_80[chain][freq_range-12] = inpa[j++];
				pwrdet->pwrdet_b_80[chain][freq_range-12] = inpa[j++];
				pwrdet->pwrdet_c_80[chain][freq_range-12] = inpa[j++];
				pwrdet->pwrdet_d_80[chain][freq_range-12] = inpa[j++];
				break;
			    case WL_CHAN_FREQ_RANGE_5G_5BAND:
				for (n = 1; n <= 5; n++) {
				    pwrdet->pwrdet_a[chain][n] = inpa[j++];
				    pwrdet->pwrdet_b[chain][n] = inpa[j++];
				    pwrdet->pwrdet_c[chain][n] = inpa[j++];
				    pwrdet->pwrdet_d[chain][n] = inpa[j++];
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_5BAND_40:
				for (n = 1; n <= 5; n++) {
				    pwrdet->pwrdet_a_40[chain][n] = inpa[j++];
				    pwrdet->pwrdet_b_40[chain][n] = inpa[j++];
				    pwrdet->pwrdet_c_40[chain][n] = inpa[j++];
				    pwrdet->pwrdet_d_40[chain][n] = inpa[j++];
				}
				break;
			    case WL_CHAN_FREQ_RANGE_5G_5BAND_80:
				for (n = 0; n <= 4; n++) {
				    pwrdet->pwrdet_a_80[chain][n] = inpa[j++];
				    pwrdet->pwrdet_b_80[chain][n] = inpa[j++];
				    pwrdet->pwrdet_c_80[chain][n] = inpa[j++];
				    pwrdet->pwrdet_d_80[chain][n] = inpa[j++];
				}
				break;
#endif /* WL_CHAN_FREQ_RANGE_5G_4BAND */
			    default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				break;
			    }
			} else {
			    srom11_pwrdet_t *pwrdet11 = pi->pwrdet_ac;
			    switch (freq_range) {
			    case WL_CHAN_FREQ_RANGE_2G:
				pwrdet11->pwrdet_a1[chain][freq_range] = inpa[j++];
				pwrdet11->pwrdet_b0[chain][freq_range] = inpa[j++];
				pwrdet11->pwrdet_b1[chain][freq_range] = inpa[j++];
				break;
				/* allow compile in branches without 4BAND definition */
#ifdef WL_CHAN_FREQ_RANGE_5G_4BAND
			    case WL_CHAN_FREQ_RANGE_5G_4BAND:
				for (n = 1; n <= 4; n ++) {
				    pwrdet11->pwrdet_a1[chain][n] = inpa[j++];
				    pwrdet11->pwrdet_b0[chain][n] = inpa[j++];
				    pwrdet11->pwrdet_b1[chain][n] = inpa[j++];
				}
				break;

			    case WL_CHAN_FREQ_RANGE_5G_BAND0:
			    case WL_CHAN_FREQ_RANGE_5G_BAND1:
			    case WL_CHAN_FREQ_RANGE_5G_BAND2:
			    case WL_CHAN_FREQ_RANGE_5G_BAND3:
				if (ACMAJORREV_2(pi->pubpi->phy_rev) ||
				    ACMAJORREV_5(pi->pubpi->phy_rev)) {
				    pwrdet11->pwrdet_a1[chain][freq_range] = inpa[j++];
				    pwrdet11->pwrdet_b0[chain][freq_range] = inpa[j++];
				    pwrdet11->pwrdet_b1[chain][freq_range] = inpa[j++];
				} else {
				    PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				}
				break;
#endif /* WL_CHAN_FREQ_RANGE_5G_4BAND */
			    default:
				PHY_ERROR(("bandrange %d is out of scope\n", inpa[1]));
				break;
			    }
			}
		} else {
		    PHY_ERROR(("Unsupported PHY type!\n"));
		    err = BCME_UNSUPPORTED;
		}
	}
	    break;
	case IOV_GVAL(IOV_PAVARS2): {
			wl_pavars2_t *invar = (wl_pavars2_t *)p;
			wl_pavars2_t *outvar = (wl_pavars2_t *)a;
			uint16 *outpa = outvar->inpa;
			uint j = 0; /* PA parameters start from offset */

			if (invar->ver	!= WL_PHY_PAVAR_VER) {
				PHY_ERROR(("Incompatible version; use %d expected version %d\n",
					invar->ver, WL_PHY_PAVAR_VER));
				return BCME_BADARG;
			}

			outvar->ver = WL_PHY_PAVAR_VER;
			outvar->len = sizeof(wl_pavars2_t);
			if (wlc_phy_chanspec_bandrange_get(pi, pi->radio_chanspec)
				== invar->bandrange)
				outvar->inuse = 1;
			else
				outvar->inuse = 0;

#ifdef BAND5G
			if (pi->sromi->subband5Gver == PHY_SUBBAND_5BAND) {
				if ((invar->bandrange == WL_CHAN_FREQ_RANGE_5GL) ||
					(invar->bandrange == WL_CHAN_FREQ_RANGE_5GM) ||
					(invar->bandrange == WL_CHAN_FREQ_RANGE_5GH)) {
					outvar->phy_type = PHY_TYPE_NULL;
					break;
				}
			}

			if (pi->sromi->subband5Gver == PHY_SUBBAND_3BAND_JAPAN) {
				if ((invar->bandrange == WL_CHAN_FREQ_RANGE_5GLL_5BAND) ||
					(invar->bandrange == WL_CHAN_FREQ_RANGE_5GLH_5BAND) ||
					(invar->bandrange == WL_CHAN_FREQ_RANGE_5GML_5BAND) ||
					(invar->bandrange == WL_CHAN_FREQ_RANGE_5GMH_5BAND) ||
					(invar->bandrange == WL_CHAN_FREQ_RANGE_5GH_5BAND)) {
					outvar->phy_type = PHY_TYPE_NULL;
					break;
				}
			}
#endif /* BAND5G */

			if (ISHTPHY(pi)) {
				if (invar->phy_type != PHY_TYPE_HT) {
					outvar->phy_type = PHY_TYPE_NULL;
					break;
				}

				if (invar->chain >= PHYCORENUM(pi->pubpi->phy_corenum))
					return BCME_BADARG;

				switch (invar->bandrange) {
				case WL_CHAN_FREQ_RANGE_2G:
				case WL_CHAN_FREQ_RANGE_5GL:
				case WL_CHAN_FREQ_RANGE_5GM:
				case WL_CHAN_FREQ_RANGE_5GH:
					wlc_phy_pavars_get_htphy(pi, &outpa[j], invar->bandrange,
						invar->chain);
					break;
				default:
					PHY_ERROR(("bandrange %d is out of scope\n",
						invar->bandrange));
					break;
				}
			} else if (ISLCNCOMMONPHY(pi)) {
				if ((invar->phy_type != PHY_TYPE_LCN) &&
					(invar->phy_type != PHY_TYPE_LCN40)) {
					outvar->phy_type = PHY_TYPE_NULL;
					break;
				}

				if (invar->chain != 0)
					return BCME_BADARG;

				switch (invar->bandrange) {
				case WL_CHAN_FREQ_RANGE_2G:
					outpa[j++] = pi->txpa_2g[0];		/* b0 */
					outpa[j++] = pi->txpa_2g[1];		/* b1 */
					outpa[j++] = pi->txpa_2g[2];		/* a1 */
					break;
#ifdef BAND5G
				case WL_CHAN_FREQ_RANGE_5GL:
					outpa[j++] = pi->txpa_5g_low[0];	/* b0 */
					outpa[j++] = pi->txpa_5g_low[1];	/* b1 */
					outpa[j++] = pi->txpa_5g_low[2];	/* a1 */
					break;

				case WL_CHAN_FREQ_RANGE_5GM:
					outpa[j++] = pi->txpa_5g_mid[0];	/* b0 */
					outpa[j++] = pi->txpa_5g_mid[1];	/* b1 */
					outpa[j++] = pi->txpa_5g_mid[2];	/* a1 */
					break;

				case WL_CHAN_FREQ_RANGE_5GH:
					outpa[j++] = pi->txpa_5g_hi[0]; /* b0 */
					outpa[j++] = pi->txpa_5g_hi[1]; /* b1 */
					outpa[j++] = pi->txpa_5g_hi[2]; /* a1 */
					break;
#endif /* BAND5G */
				default:
					PHY_ERROR(("bandrange %d is out of scope\n",
						invar->bandrange));
					break;
				}
			} else {
				PHY_ERROR(("Unsupported PHY type!\n"));
				err = BCME_UNSUPPORTED;
			}
		}
		break;

		case IOV_SVAL(IOV_PAVARS2): {
			wl_pavars2_t *invar = (wl_pavars2_t *)p;
			uint16 *inpa = invar->inpa;
			uint j = 0; /* PA parameters start from offset */

			if (invar->ver	!= WL_PHY_PAVAR_VER) {
				PHY_ERROR(("Incompatible version; use %d expected version %d\n",
					invar->ver, WL_PHY_PAVAR_VER));
				return BCME_BADARG;
			}

			if (ISHTPHY(pi)) {
				if (invar->phy_type != PHY_TYPE_HT) {
					break;
				}

				if (invar->chain >= PHYCORENUM(pi->pubpi->phy_corenum))
					return BCME_BADARG;

				switch (invar->bandrange) {
				case WL_CHAN_FREQ_RANGE_2G:
				case WL_CHAN_FREQ_RANGE_5GL:
				case WL_CHAN_FREQ_RANGE_5GM:
				case WL_CHAN_FREQ_RANGE_5GH:
				case WL_CHAN_FREQ_RANGE_5GLL_5BAND:
				case WL_CHAN_FREQ_RANGE_5GLH_5BAND:
				case WL_CHAN_FREQ_RANGE_5GML_5BAND:
				case WL_CHAN_FREQ_RANGE_5GMH_5BAND:
				case WL_CHAN_FREQ_RANGE_5GH_5BAND:
					if (invar->bandrange < (CH_2G_GROUP + CH_5G_4BAND)) {
						wlc_phy_pavars_set_htphy(pi, &inpa[j],
							invar->bandrange, invar->chain);
					} else {
						err = BCME_RANGE;
						PHY_ERROR(("bandrange %d is out of scope\n",
							invar->bandrange));
					}
					break;
				default:
					err = BCME_RANGE;
					PHY_ERROR(("bandrange %d is out of scope\n",
						invar->bandrange));
					break;
				}
			} else if (ISLCNCOMMONPHY(pi)) {
				if ((invar->phy_type != PHY_TYPE_LCN) &&
					(invar->phy_type != PHY_TYPE_LCN40))
					break;

				if (invar->chain != 0)
					return BCME_BADARG;

				switch (invar->bandrange) {
				case WL_CHAN_FREQ_RANGE_2G:
					pi->txpa_2g[0] = inpa[j++]; /* b0 */
					pi->txpa_2g[1] = inpa[j++]; /* b1 */
					pi->txpa_2g[2] = inpa[j++]; /* a1 */
					break;
#ifdef BAND5G
				case WL_CHAN_FREQ_RANGE_5GL:
					pi->txpa_5g_low[0] = inpa[j++]; /* b0 */
					pi->txpa_5g_low[1] = inpa[j++]; /* b1 */
					pi->txpa_5g_low[2] = inpa[j++]; /* a1 */
					break;

				case WL_CHAN_FREQ_RANGE_5GM:
					pi->txpa_5g_mid[0] = inpa[j++]; /* b0 */
					pi->txpa_5g_mid[1] = inpa[j++]; /* b1 */
					pi->txpa_5g_mid[2] = inpa[j++]; /* a1 */
					break;

				case WL_CHAN_FREQ_RANGE_5GH:
					pi->txpa_5g_hi[0] = inpa[j++];	/* b0 */
					pi->txpa_5g_hi[1] = inpa[j++];	/* b1 */
					pi->txpa_5g_hi[2] = inpa[j++];	/* a1 */
					break;
#endif /* BAND5G */
				default:
					PHY_ERROR(("bandrange %d is out of scope\n",
						invar->bandrange));
					break;
				}
			} else {
				PHY_ERROR(("Unsupported PHY type!\n"));
				err = BCME_UNSUPPORTED;
			}
		}
		break;

	case IOV_GVAL(IOV_POVARS): {
		wl_po_t tmppo;

		/* tmppo has the input phy_type and band */
		bcopy(p, &tmppo, sizeof(wl_po_t));
		if (ISHTPHY(pi)) {
			if ((tmppo.phy_type != PHY_TYPE_HT) && (tmppo.phy_type != PHY_TYPE_N))  {
				tmppo.phy_type = PHY_TYPE_NULL;
				break;
			}

			err = wlc_phy_get_po_htphy(pi, &tmppo);
			if (!err)
				bcopy(&tmppo, a, sizeof(wl_po_t));
			break;
		} else if (ISNPHY(pi)) {
			if (tmppo.phy_type != PHY_TYPE_N)  {
				tmppo.phy_type = PHY_TYPE_NULL;
				break;
			}

			/* Power offsets variables depend on the SROM revision */
			if (NREV_GE(pi->pubpi->phy_rev, 8) && (pi->sh->sromrev >= 9)) {
				err = wlc_phy_get_po_htphy(pi, &tmppo);

			} else {
				switch (tmppo.band) {
				case WL_CHAN_FREQ_RANGE_2G:
					tmppo.cckpo = pi->ppr->u.sr8.cck2gpo;
					tmppo.ofdmpo = pi->ppr->u.sr8.ofdm[tmppo.band];
					bcopy(&pi->ppr->u.sr8.mcs[tmppo.band][0], &tmppo.mcspo,
						8*sizeof(uint16));
					break;
#ifdef BAND5G
				case WL_CHAN_FREQ_RANGE_5G_BAND0:
					tmppo.ofdmpo = pi->ppr->u.sr8.ofdm[tmppo.band];
					bcopy(&pi->ppr->u.sr8.mcs[tmppo.band], &tmppo.mcspo,
						8*sizeof(uint16));
					break;

				case WL_CHAN_FREQ_RANGE_5G_BAND1:
					tmppo.ofdmpo = pi->ppr->u.sr8.ofdm[tmppo.band];
					bcopy(&pi->ppr->u.sr8.mcs[tmppo.band], &tmppo.mcspo,
						8*sizeof(uint16));
					break;

				case WL_CHAN_FREQ_RANGE_5G_BAND2:
					tmppo.ofdmpo = pi->ppr->u.sr8.ofdm[tmppo.band];
					bcopy(&pi->ppr->u.sr8.mcs[tmppo.band], &tmppo.mcspo,
						8*sizeof(uint16));
					break;

				case WL_CHAN_FREQ_RANGE_5G_BAND3:
					tmppo.ofdmpo = pi->ppr->u.sr8.ofdm[tmppo.band];
					bcopy(&pi->ppr->u.sr8.mcs[tmppo.band], &tmppo.mcspo,
						8*sizeof(uint16));
					break;
#endif /* BAND5G */
				default:
					PHY_ERROR(("bandrange %d is out of scope\n", tmppo.band));
					err = BCME_BADARG;
					break;
				}
			}

			if (!err)
				bcopy(&tmppo, a, sizeof(wl_po_t));
		} else if (ISLCNCOMMONPHY(pi)) {
			if ((tmppo.phy_type != PHY_TYPE_LCN) &&
				(tmppo.phy_type != PHY_TYPE_LCN40)) {
				tmppo.phy_type = PHY_TYPE_NULL;
				break;
			}

			switch (tmppo.band) {
			case WL_CHAN_FREQ_RANGE_2G:
				tmppo.cckpo = pi->ppr->u.sr8.cck2gpo;
				tmppo.ofdmpo = pi->ppr->u.sr8.ofdm[tmppo.band];
				bcopy(&pi->ppr->u.sr8.mcs[tmppo.band], &tmppo.mcspo,
					8*sizeof(uint16));

				break;

			default:
				PHY_ERROR(("bandrange %d is out of scope\n", tmppo.band));
				err = BCME_BADARG;
				break;
			}

			if (!err)
				bcopy(&tmppo, a, sizeof(wl_po_t));
		} else {
			PHY_ERROR(("Unsupported PHY type!\n"));
			err = BCME_UNSUPPORTED;
		}
	}
	break;

	case IOV_SVAL(IOV_POVARS): {
		wl_po_t inpo;

		bcopy(p, &inpo, sizeof(wl_po_t));

		if (ISHTPHY(pi)) {
			if ((inpo.phy_type == PHY_TYPE_HT) || (inpo.phy_type == PHY_TYPE_N))
				err = wlc_phy_set_po_htphy(pi, &inpo);
			break;
		} else if (ISNPHY(pi)) {
			if (inpo.phy_type != PHY_TYPE_N)
				break;

			/* Power offsets variables depend on the SROM revision */
			if (NREV_GE(pi->pubpi->phy_rev, 8) && (pi->sh->sromrev >= 9)) {
				err = wlc_phy_set_po_htphy(pi, &inpo);

			} else {

				switch (inpo.band) {
				case WL_CHAN_FREQ_RANGE_2G:
					pi->ppr->u.sr8.cck2gpo = inpo.cckpo;
					pi->ppr->u.sr8.ofdm[inpo.band]  = inpo.ofdmpo;
					bcopy(inpo.mcspo, &(pi->ppr->u.sr8.mcs[inpo.band][0]),
						8*sizeof(uint16));
					break;
#ifdef BAND5G
				case WL_CHAN_FREQ_RANGE_5G_BAND0:
					pi->ppr->u.sr8.ofdm[inpo.band] = inpo.ofdmpo;
					bcopy(inpo.mcspo, &(pi->ppr->u.sr8.mcs[inpo.band][0]),
						8*sizeof(uint16));
					break;

				case WL_CHAN_FREQ_RANGE_5G_BAND1:
					pi->ppr->u.sr8.ofdm[inpo.band] = inpo.ofdmpo;
					bcopy(inpo.mcspo, &(pi->ppr->u.sr8.mcs[inpo.band][0]),
						8*sizeof(uint16));
					break;

				case WL_CHAN_FREQ_RANGE_5G_BAND2:
					pi->ppr->u.sr8.ofdm[inpo.band] = inpo.ofdmpo;
					bcopy(inpo.mcspo, &(pi->ppr->u.sr8.mcs[inpo.band][0]),
						8*sizeof(uint16));
					break;

				case WL_CHAN_FREQ_RANGE_5G_BAND3:
					pi->ppr->u.sr8.ofdm[inpo.band] = inpo.ofdmpo;
					bcopy(inpo.mcspo, &(pi->ppr->u.sr8.mcs[inpo.band][0]),
						8*sizeof(uint16));
					break;
#endif /* BAND5G */
				default:
					PHY_ERROR(("bandrange %d is out of scope\n", inpo.band));
					err = BCME_BADARG;
					break;
				}

			}

		} else {
			PHY_ERROR(("Unsupported PHY type!\n"));
			err = BCME_UNSUPPORTED;
		}
	}
	break;
#endif 

	case IOV_GVAL(IOV_SROM_REV): {
			*ret_int_ptr = pi->sh->sromrev;
	}
	break;

#ifdef WLTEST
	case IOV_GVAL(IOV_PHY_MAXP): {
		if (ISNPHY(pi)) {
			srom_pwrdet_t	*pwrdet  = pi->pwrdet;
			uint8*	maxp = (uint8*)a;

			maxp[0] = pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_2G];
			maxp[1] = pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_2G];
			maxp[2] = pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND0];
			maxp[3] = pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND0];
			maxp[4] = pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND1];
			maxp[5] = pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND1];
			maxp[6] = pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND2];
			maxp[7] = pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND2];
			if (pi->sromi->subband5Gver == PHY_SUBBAND_4BAND)
			{
				maxp[8] = pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND3];
				maxp[9] = pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND3];
			}
		}
		break;
	}
	case IOV_SVAL(IOV_PHY_MAXP): {
		if (ISNPHY(pi)) {
			uint8*	maxp = (uint8*)p;
			srom_pwrdet_t	*pwrdet  = pi->pwrdet;

			pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_2G] = maxp[0];
			pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_2G] = maxp[1];
			pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND0] = maxp[2];
			pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND0] = maxp[3];
			pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND1] = maxp[4];
			pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND1] = maxp[5];
			pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND2] = maxp[6];
			pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND2] = maxp[7];
			if (pi->sromi->subband5Gver == PHY_SUBBAND_4BAND)
			{
				pwrdet->max_pwr[PHY_CORE_0][WL_CHAN_FREQ_RANGE_5G_BAND3] = maxp[8];
				pwrdet->max_pwr[PHY_CORE_1][WL_CHAN_FREQ_RANGE_5G_BAND3] = maxp[9];
			}
		}
		break;
	}
#endif /* WLTEST */
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_rssi(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	BCM_REFERENCE(*ret_int_ptr);
	BCM_REFERENCE(*pi_ac);

	switch (actionid) {
#if defined(WLTEST)
	case IOV_GVAL(IOV_UNMOD_RSSI):
	{
		int32 input_power_db = 0;
		rxsigpwrfn_t rx_sig_pwr_fn = pi->pi_fptr->rxsigpwr;

		PHY_INFORM(("UNMOD RSSI Called\n"));

		if (!rx_sig_pwr_fn)
			return BCME_UNSUPPORTED;	/* lpphy and sslnphy support only for now */

		if (!pi->sh->up) {
			err = BCME_NOTUP;
			break;
		}

		input_power_db = (*rx_sig_pwr_fn)(pi, -1);

		*ret_int_ptr = input_power_db;
		break;
	}
#endif 
#if (NCONF || LCN40CONF || ACCONF || ACCONF2) && defined(WLTEST)
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_CAL_TEMP):
		if (ISLCN40PHY(pi)) {
			phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
			*ret_int_ptr = (int32)pi_lcn40->gain_cal_temp;
		} else if (CHIPID_4324X_EPA_FAMILY(pi)) {
			phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
			*ret_int_ptr = (int32)pi_nphy->gain_cal_temp;
		}
		break;

	case IOV_SVAL(IOV_PHY_RSSI_GAIN_CAL_TEMP):
		if (ISLCN40PHY(pi)) {
			phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
			pi_lcn40->gain_cal_temp = (int8)int_val;
		} else if (CHIPID_4324X_EPA_FAMILY(pi)) {
			phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
			pi_nphy->gain_cal_temp = (int8)int_val;
		}
		break;
	case IOV_SVAL(IOV_PHY_RSSI_CAL_FREQ_GRP_2G):
	{
		int8 i;
		uint8 *nvramValues = p;

		for (i = 0; i < 14; i++) {
			pi_ac->sromi->rssi_cal_freq_grp[i] =
				nvramValues[i];
		}
		break;
	}
	case IOV_GVAL(IOV_PHY_RSSI_CAL_FREQ_GRP_2G):
	{
		int8 i;
		uint8 *nvramValues = a;

		for (i = 0; i < 14; i++) {
		 nvramValues[i] =
		   pi_ac->sromi->rssi_cal_freq_grp[i];
		}

		break;
	}
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB0):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB1):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB2):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB3):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB4):
		if (ISACPHY(pi)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
			  &pi_ac->sromi->rssioffset;
			int8 *deltaValues = p;
			uint8 core = deltaValues[0];
			uint8 gain_idx, bw_idx, subband_idx;

			if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB0)) {
				subband_idx = 0;
			} else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB1)) {
				subband_idx = 1;
			} else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB2)) {
				subband_idx = 2;
			} else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB3)) {
				subband_idx = 3;
			} else {
				subband_idx = 4;
			}

			for (bw_idx = 0; bw_idx < ACPHY_NUM_BW_2G; bw_idx++) {
				for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_2G_PARAMS_EXT;
					 gain_idx++) {
					pi_ac_rssioffset->rssi_corr_gain_delta_2g_sub[core]
							[gain_idx][bw_idx][subband_idx]
							= deltaValues[gain_idx + 4*bw_idx + 1];
				}
			}
			wlc_phy_set_trloss_reg_acphy(pi, core);


		}
		break;

	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB0):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB1):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB2):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB3):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB4):
		if (ISACPHY(pi)) {
		acphy_rssioffset_t *pi_ac_rssioffset =
		&pi_ac->sromi->rssioffset;
		int8 *deltaValues = a;
		uint8 core, ant;
		uint8 gain_idx, bw_idx, core_idx = 0, subband_idx;

		if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB0)) {
			subband_idx = 0;
		} else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB1)) {
			subband_idx = 1;
		} else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB2)) {
			subband_idx = 2;
		} else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GB3)) {
			subband_idx = 3;
		} else {
			subband_idx = 4;
		}

		FOREACH_CORE(pi, core) {
			ant = phy_get_rsdbbrd_corenum(pi, core);
			deltaValues[9*core_idx] = ant;
			for (bw_idx = 0; bw_idx < ACPHY_NUM_BW_2G; bw_idx++) {
				for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_2G_PARAMS_EXT;
					gain_idx++) {
					deltaValues[gain_idx + 4*bw_idx + 9*core_idx +1 ]=
					  pi_ac_rssioffset->rssi_corr_gain_delta_2g_sub[ant]
					  [gain_idx][bw_idx][subband_idx];

				}
			}
			core_idx++;
		}

		deltaValues[9*core_idx] = -1;
		}
		break;

	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2G):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GH):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GHH):
		if (ISLCN40PHY(pi)) {
			phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
			int8 *deltaValues = p;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2G))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_2g;
			else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GH))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_2gh;
			else
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_2ghh;

			for (i = 0; i < LCN40PHY_GAIN_DELTA_2G_PARAMS; i++)
				rssi_gain_delta[i] = deltaValues[i];
		} else if (CHIPID_4324X_EPA_FAMILY(pi)) {
			phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
			int8 *deltaValues = p;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2G))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_2g;
			else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2GH))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_2gh;
			else
				rssi_gain_delta = pi_nphy->rssi_gain_delta_2ghh;

			for (i = 0; i < NPHY_GAIN_DELTA_2G_PARAMS; i++)
				rssi_gain_delta[i] = deltaValues[i];
		} else if (ISACPHY(pi)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
			        &pi_ac->sromi->rssioffset;
			int8 *deltaValues = p;
			uint8 core = deltaValues[0];
			uint8 gain_idx, bw_idx;

			if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_2G)) {
				for (bw_idx = 0; bw_idx < ACPHY_NUM_BW_2G; bw_idx++) {
					for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_2G_PARAMS;
					     gain_idx++) {
						pi_ac_rssioffset->rssi_corr_gain_delta_2g[core]
						        [gain_idx][bw_idx]
						        = deltaValues[gain_idx + 2*bw_idx + 1];
					}
				}
				wlc_phy_set_trloss_reg_acphy(pi, core);

			} else {
				PHY_ERROR(("Unsupported RSSI_GAIN_DELTA_2G type!\n"));
				err = BCME_UNSUPPORTED;
			}
		}
		break;

	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2G):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GH):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GHH):
		if (ISLCN40PHY(pi)) {
			phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
			int8 *deltaValues = a;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2G))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_2g;
			else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GH))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_2gh;
			else
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_2ghh;

			for (i = 0; i < LCN40PHY_GAIN_DELTA_2G_PARAMS; i++)
				deltaValues[i] = rssi_gain_delta[i];
		} else if (CHIPID_4324X_EPA_FAMILY(pi)) {
			phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
			int8 *deltaValues = a;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2G))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_2g;
			else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2GH))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_2gh;
			else
				rssi_gain_delta = pi_nphy->rssi_gain_delta_2ghh;

			for (i = 0; i < NPHY_GAIN_DELTA_2G_PARAMS; i++)
				deltaValues[i] = rssi_gain_delta[i];
		} else if (ISACPHY(pi)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
				&pi_ac->sromi->rssioffset;
			int8 *deltaValues = a;
			uint8 core, ant;
			uint8 gain_idx, bw_idx, core_idx = 0;

			if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_2G)) {
			   FOREACH_CORE(pi, core) {
			      ant = phy_get_rsdbbrd_corenum(pi, core);
			      deltaValues[5*core_idx] = ant;
			      for (bw_idx = 0; bw_idx < ACPHY_NUM_BW_2G; bw_idx++) {
			         for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_2G_PARAMS;
				      gain_idx++) {
			            deltaValues[gain_idx + 2*bw_idx + 5*core_idx +1 ]=
			              pi_ac_rssioffset->rssi_corr_gain_delta_2g[ant]
				            [gain_idx][bw_idx];
			         }
			      }
			      core_idx++;
			   }
			   /* set core to -1 after the last valid entry */
			   deltaValues[core_idx*5] = -1;

			   for (bw_idx = 0; bw_idx < core_idx*5; bw_idx++) {
				printf("%d ", deltaValues[bw_idx]);
			   }
			   printf("\n");
			} else {
				PHY_ERROR(("Unsupported RSSI_GAIN_DELTA_2G type!\n"));
				err = BCME_UNSUPPORTED;
			}
		}
		break;

	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU):
	case IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GH):
		if (ISLCN40PHY(pi)) {
			phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
			int8 *deltaValues = p;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gl;
			else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gml;
			else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gmu;
			else
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gh;

			for (i = 0; i < LCN40PHY_GAIN_DELTA_5G_PARAMS; i++)
				rssi_gain_delta[i] = deltaValues[i];
		} else if (CHIPID_4324X_EPA_FAMILY(pi)) {
			phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
			int8 *deltaValues = p;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gl;
			else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gml;
			else if (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gmu;
			else
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gh;

			for (i = 0; i < NPHY_GAIN_DELTA_5G_PARAMS; i++)
				rssi_gain_delta[i] = deltaValues[i];
		} else if (ISACPHY(pi) && (pi->u.pi_acphy->rssi_cal_rev == FALSE)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
				&pi_ac->sromi->rssioffset;
			int8 *deltaValues = p;
			uint8 core = deltaValues[0];
			uint8 gain_idx, bw_idx, subband_idx;

			subband_idx = (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL)) ? 0:
			        (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML)) ? 1:
			        (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU)) ? 2:3;
			for (bw_idx = 0; bw_idx < ACPHY_NUM_BW; bw_idx++) {
				for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_5G_PARAMS;
				     gain_idx++) {
					pi_ac_rssioffset->rssi_corr_gain_delta_5g
						[core][gain_idx][bw_idx][subband_idx]
						= deltaValues[gain_idx + 2*bw_idx + 1];
				}
			}
			wlc_phy_set_trloss_reg_acphy(pi, core);

			for (bw_idx = 0; bw_idx < ACPHY_NUM_BW; bw_idx++) {
				for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_5G_PARAMS;
				     gain_idx++) {
					printf("%d ", pi_ac_rssioffset->rssi_corr_gain_delta_5g
						[core][gain_idx][bw_idx][subband_idx]);
				}
			}
			printf("\n");
		} else if (ISACPHY(pi) && (pi->u.pi_acphy->rssi_cal_rev == TRUE)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
				&pi_ac->sromi->rssioffset;
			int8 *deltaValues = p;
			uint8 core = deltaValues[0];
			uint8 gain_idx, bw_idx, subband_idx;

			subband_idx = (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL)) ? 0:
			        (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML)) ? 1:
			        (actionid == IOV_SVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU)) ? 2:3;
			for (bw_idx = 0; bw_idx < ACPHY_NUM_BW; bw_idx++) {
				for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_5G_PARAMS_EXT;
				     gain_idx++) {
					pi_ac_rssioffset->rssi_corr_gain_delta_5g_sub
						[core][gain_idx][bw_idx][subband_idx]
						= deltaValues[gain_idx + 4*bw_idx + 1];
				}
			}
			wlc_phy_set_trloss_reg_acphy(pi, core);

			for (bw_idx = 0; bw_idx < ACPHY_NUM_BW; bw_idx++) {
				for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_5G_PARAMS_EXT;
				     gain_idx++) {
					printf("%d ", pi_ac_rssioffset->rssi_corr_gain_delta_5g_sub
						[core][gain_idx][bw_idx][subband_idx]);
				}
			}
			printf("\n");
		}

		break;

	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU):
	case IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GH):
		if (ISLCN40PHY(pi)) {
			phy_info_lcn40phy_t *pi_lcn40 = pi->u.pi_lcn40phy;
			int8 *deltaValues = a;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gl;
			else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gml;
			else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU))
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gmu;
			else
				rssi_gain_delta = pi_lcn40->rssi_gain_delta_5gh;

			for (i = 0; i < LCN40PHY_GAIN_DELTA_5G_PARAMS; i++)
				deltaValues[i] = rssi_gain_delta[i];
		} else if (CHIPID_4324X_EPA_FAMILY(pi)) {
			phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
			int8 *deltaValues = a;
			uint8 i;
			int8 *rssi_gain_delta;
			if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gl;
			else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gml;
			else if (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU))
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gmu;
			else
				rssi_gain_delta = pi_nphy->rssi_gain_delta_5gh;

			for (i = 0; i < NPHY_GAIN_DELTA_5G_PARAMS; i++)
				deltaValues[i] = rssi_gain_delta[i];
		} else if (ISACPHY(pi) && (pi->u.pi_acphy->rssi_cal_rev == FALSE)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
				&pi_ac->sromi->rssioffset;
			int8 *deltaValues = a;
			uint8 core, ant, core_idx = 0;
			uint8 gain_idx, bw_idx, subband_idx;

			subband_idx = (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL)) ? 0:
			        (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML)) ? 1:
			        (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU)) ? 2:3;

			FOREACH_CORE(pi, core) {
				ant = phy_get_rsdbbrd_corenum(pi, core);
				deltaValues[7*core_idx] =  ant;
				for (bw_idx = 0; bw_idx < ACPHY_NUM_BW; bw_idx++) {
					for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_5G_PARAMS;
					     gain_idx++) {
						deltaValues[gain_idx + 2*bw_idx + 7*core_idx +1]=
							pi_ac_rssioffset->rssi_corr_gain_delta_5g
							[ant][gain_idx][bw_idx][subband_idx];
					}
				}
				core_idx++;
			}
			deltaValues[core_idx*7] = -1;
		} else if (ISACPHY(pi) && (pi->u.pi_acphy->rssi_cal_rev == TRUE)) {
			acphy_rssioffset_t *pi_ac_rssioffset =
				&pi_ac->sromi->rssioffset;
			int8 *deltaValues = a;
			uint8 core, core_idx = 0;
			uint8 gain_idx, bw_idx, subband_idx;

			subband_idx = (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GL)) ? 0:
			        (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GML)) ? 1:
			        (actionid == IOV_GVAL(IOV_PHY_RSSI_GAIN_DELTA_5GMU)) ? 2:3;

			FOREACH_CORE(pi, core) {
				deltaValues[13*core_idx] = core;
				for (bw_idx = 0; bw_idx < ACPHY_NUM_BW; bw_idx++) {
				  for (gain_idx = 0; gain_idx < ACPHY_GAIN_DELTA_5G_PARAMS_EXT;
				       gain_idx++) {
				    deltaValues[gain_idx + 4*bw_idx + 13*core_idx +1 ]=
				      pi_ac_rssioffset->rssi_corr_gain_delta_5g_sub
				      [core][gain_idx][bw_idx][subband_idx];
				  }
				}
				core_idx++;
			}
			/* set core to -1 after the last valid entry */
			deltaValues[core_idx*13] = -1;

		}
		break;
	case IOV_SVAL(IOV_PHY_RXGAINERR_2G):
		if (ISACPHY(pi)) {
			uint8 core;
			int8 *deltaValues = p;
			FOREACH_CORE(pi, core) {
				pi->rxgainerr_2g[core] = deltaValues[core];
			}
		}
		break;

	case IOV_GVAL(IOV_PHY_RXGAINERR_2G):
		if (ISACPHY(pi)) {
			int8 *deltaValues = a;
			uint8 core;
			FOREACH_CORE(pi, core) {
			  deltaValues[core] = pi->rxgainerr_2g[core];
			}
		}
		break;
	case IOV_SVAL(IOV_PHY_RXGAINERR_5GL):
		if (ISACPHY(pi)) {
			uint8 core;
			int8 *deltaValues = p;
			FOREACH_CORE(pi, core) {
				pi->rxgainerr_5gl[core] = deltaValues[core];
			}
		}
		break;

	case IOV_GVAL(IOV_PHY_RXGAINERR_5GL):
		if (ISACPHY(pi)) {
			int8 *deltaValues = a;
			uint8 core;
			FOREACH_CORE(pi, core) {
			  deltaValues[core] = pi->rxgainerr_5gl[core];
			}
		}
		break;

	case IOV_SVAL(IOV_PHY_RXGAINERR_5GM):
		if (ISACPHY(pi)) {
			uint8 core;
			int8 *deltaValues = p;
			FOREACH_CORE(pi, core) {
				pi->rxgainerr_5gm[core] = deltaValues[core];
			}
		}
		break;

	case IOV_GVAL(IOV_PHY_RXGAINERR_5GM):
		if (ISACPHY(pi)) {
			int8 *deltaValues = a;
			uint8 core;
			FOREACH_CORE(pi, core) {
			  deltaValues[core] = pi->rxgainerr_5gm[core];
			}
		}
		break;

	case IOV_SVAL(IOV_PHY_RXGAINERR_5GH):
		if (ISACPHY(pi)) {
			uint8 core;
			int8 *deltaValues = p;
			FOREACH_CORE(pi, core) {
				pi->rxgainerr_5gh[core] = deltaValues[core];
			}
		}
		break;

	case IOV_GVAL(IOV_PHY_RXGAINERR_5GH):
		if (ISACPHY(pi)) {
			int8 *deltaValues = a;
			uint8 core;
			FOREACH_CORE(pi, core) {
			  deltaValues[core] = pi->rxgainerr_5gh[core];
			}
		}
		break;

	case IOV_SVAL(IOV_PHY_RXGAINERR_5GU):
		if (ISACPHY(pi)) {
			uint8 core;
			int8 *deltaValues = p;
			FOREACH_CORE(pi, core) {
				pi->rxgainerr_5gu[core] = deltaValues[core];
			}
		}
		break;

	case IOV_GVAL(IOV_PHY_RXGAINERR_5GU):
		if (ISACPHY(pi)) {
			int8 *deltaValues = a;
			uint8 core;

			FOREACH_CORE(pi, core) {
			  deltaValues[core] = pi->rxgainerr_5gu[core];
			}
		}
		break;

#endif /* (NCONF || LCN40CONF || ACCONF || ACCONF2) && WLTEST */
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_aci(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	int err = BCME_OK;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	switch (actionid) {
#if defined(WLTEST)
	case IOV_SVAL(IOV_ACI_EXIT_CHECK_PERIOD):
		if (int_val == 0)
			err = BCME_RANGE;
		else
			pi->aci_exit_check_period = int_val;
		break;

	case IOV_GVAL(IOV_ACI_EXIT_CHECK_PERIOD):
		int_val = pi->aci_exit_check_period;
		bcopy(&int_val, a, vsize);
		break;

#endif 
#if defined(WLTEST)
	case IOV_SVAL(IOV_PHY_GLITCHK):
		pi->tunings[0] = (uint16)int_val;
		break;

	case IOV_SVAL(IOV_PHY_NOISE_UP):
		pi->tunings[1] = (uint16)int_val;
		break;

	case IOV_SVAL(IOV_PHY_NOISE_DWN):
		pi->tunings[2] = (uint16)int_val;
		break;
#endif /* #if defined(WLTEST) */
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_phy_specific(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen,
	void *a, int alen, int vsize)
{
	int err = BCME_OK;

	if (ISACPHY(pi))
		err = wlc_phy_iovars_acphy(pi, actionid, type, p, plen, a, alen, vsize);
	else if (ISLCNCOMMONPHY(pi))
		err = wlc_phy_iovars_lcncmnphy(pi, actionid, type, p, plen, a, alen, vsize);
	else if (ISNPHY(pi))
		err = wlc_phy_iovars_nphy(pi, actionid, type, p, plen, a, alen, vsize);
	else
		err = BCME_UNSUPPORTED;

	return err;
}

static int
wlc_phy_iovars_nphy(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	bool bool_val;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	BCM_REFERENCE(*ret_int_ptr);
	BCM_REFERENCE(bool_val);

	switch (actionid) {
	case IOV_GVAL(IOV_PHY_OCLSCDENABLE):
		err = wlc_phy_iovar_oclscd(pi, int_val, bool_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_OCLSCDENABLE):
		err = wlc_phy_iovar_oclscd(pi, int_val, bool_val, ret_int_ptr, TRUE);
		break;

	case IOV_GVAL(IOV_LNLDO2):
		wlc_phy_iovar_prog_lnldo2(pi, int_val, bool_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_LNLDO2):
		err = wlc_phy_iovar_prog_lnldo2(pi, int_val, bool_val, ret_int_ptr, TRUE);
		break;

#if defined(WLTEST) || defined(DBG_PHY_IOV)
	case IOV_GVAL(IOV_PHY_DYN_ML):
		err =  wlc_phy_dynamic_ml(pi, int_val, ret_int_ptr, vsize, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_DYN_ML):
		err =  wlc_phy_dynamic_ml(pi, int_val, ret_int_ptr, vsize, TRUE);
		break;

	case IOV_GVAL(IOV_PHY_ACI_NAMS):
		err =  wlc_phy_aci_nams(pi, int_val, ret_int_ptr, vsize, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_ACI_NAMS):
		err =  wlc_phy_aci_nams(pi, int_val, ret_int_ptr, vsize, TRUE);
		break;
#endif 
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_lcncmnphy(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	BCM_REFERENCE(*ret_int_ptr);

	switch (actionid) {
#if defined(WLTEST)
	case IOV_SVAL(IOV_PHY_AUXPGA):
			if (ISLCNCOMMONPHY(pi)) {
				phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
				uint8 *aug_pga_val = (uint8*)p;
				pi_lcn->lcnphy_rssi_vf = aug_pga_val[0];
				pi_lcn->lcnphy_rssi_vc = aug_pga_val[1];
				pi_lcn->lcnphy_rssi_gs = aug_pga_val[2];
#ifdef BAND5G
				pi_lcn->rssismf5g = aug_pga_val[3];
				pi_lcn->rssismc5g = aug_pga_val[4];
				pi_lcn->rssisav5g = aug_pga_val[5];
#endif
			}
		break;

	case IOV_GVAL(IOV_PHY_AUXPGA):
			if (ISLCNCOMMONPHY(pi)) {
				phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
				uint8 *aug_pga_val = (uint8*)a;
				aug_pga_val[0] = pi_lcn->lcnphy_rssi_vf;
				aug_pga_val[1] = pi_lcn->lcnphy_rssi_vc;
				aug_pga_val[2] = pi_lcn->lcnphy_rssi_gs;
#ifdef BAND5G
				aug_pga_val[3] = (uint8) pi_lcn->rssismf5g;
				aug_pga_val[4] = (uint8) pi_lcn->rssismc5g;
				aug_pga_val[5] = (uint8) pi_lcn->rssisav5g;
#endif
			}
		break;
#endif 
#if defined(WLTEST)
#if defined(LCNCONF) || defined(LCN40CONF)
	case IOV_GVAL(IOV_LCNPHY_RXIQGAIN):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->rxpath_gain;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_RXIQGSPOWER):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->rxpath_gainselect_power;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_RXIQPOWER):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->rxpath_final_power;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_RXIQSTATUS):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->rxpath_status;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_RXIQSTEPS):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->rxpath_steps;
		}
		break;
	case IOV_SVAL(IOV_LCNPHY_RXIQSTEPS):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			pi_lcn->rxpath_steps = (uint8)int_val;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_TSSI_MAXPWR):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->tssi_maxpwr_limit;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_TSSI_MINPWR):
		{
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->tssi_minpwr_limit;
		}
		break;
#endif /* #if defined(LCNCONF) || defined(LCN40CONF) */
#if LCN40CONF
	case IOV_GVAL(IOV_LCNPHY_TXPWRCLAMP_DIS):
		if (ISLCNPHY(pi) || ISLCN40PHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->txpwr_clamp_dis;
		}
		break;
	case IOV_SVAL(IOV_LCNPHY_TXPWRCLAMP_DIS):
		if (ISLCNPHY(pi) || ISLCN40PHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			pi_lcn->txpwr_clamp_dis = (uint8)int_val;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_TXPWRCLAMP_OFDM):
		if (ISLCNPHY(pi) || ISLCN40PHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->target_pwr_ofdm_max;
		}
		break;
	case IOV_GVAL(IOV_LCNPHY_TXPWRCLAMP_CCK):
		if (ISLCNPHY(pi) || ISLCN40PHY(pi)) {
			phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
			*ret_int_ptr = (int32)pi_lcn->target_pwr_cck_max;
		}
		break;
#endif /* #if LCN40CONF */
#endif /* #if defined(WLTEST) */
#if defined(BCMDBG)
	case IOV_GVAL(IOV_LCNPHY_CWTXPWRCTRL):
		if (ISLCNPHY(pi))
			wlc_lcnphy_iovar_cw_tx_pwr_ctrl(pi, int_val, ret_int_ptr, FALSE);
		break;
	case IOV_SVAL(IOV_LCNPHY_CWTXPWRCTRL):
		if (ISLCNPHY(pi))
			wlc_lcnphy_iovar_cw_tx_pwr_ctrl(pi, int_val, ret_int_ptr, TRUE);
		break;
#endif
	case IOV_SVAL(IOV_PHY_CRS_WAR):
		if (ISLCN40PHY(pi)) {
			pi->u.pi_lcn40phy->phycrs_war_en = (bool)int_val;
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
	case IOV_GVAL(IOV_PHY_CRS_WAR):
		if (ISLCN40PHY(pi)) {
			*ret_int_ptr = (int32) pi->u.pi_lcn40phy->phycrs_war_en;
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_acphy(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	BCM_REFERENCE(*ret_int_ptr);

	switch (actionid) {
#if defined(BCMDBG) || defined(WLTEST) || defined(PHYCAL_CHNG_CS)
	case IOV_GVAL(IOV_HIRSSI_PERIOD):
		*ret_int_ptr = (int32)pi->u.pi_acphy->hirssi_period;
		break;
	case IOV_SVAL(IOV_HIRSSI_PERIOD):
		pi->u.pi_acphy->hirssi_period = (int_val <= 0) ? PHY_SW_HIRSSI_PERIOD  :
		(uint16)int_val;
		break;
	case IOV_GVAL(IOV_HIRSSI_EN):
		*ret_int_ptr = (int32)pi->u.pi_acphy->hirssi_en;
		break;
	case IOV_SVAL(IOV_HIRSSI_EN):
		pi->u.pi_acphy->hirssi_en = (int_val == 0) ? FALSE : TRUE;
		if (ISACPHY(pi) && PHY_SW_HIRSSI_UCODE_CAP(pi)) {
			wlc_phy_hirssi_elnabypass_init_acphy(pi);
			if (!pi->u.pi_acphy->hirssi_en)
				wlc_phy_hirssi_elnabypass_apply_acphy(pi);
		}
		break;
	case IOV_GVAL(IOV_HIRSSI_BYP_RSSI):
		*ret_int_ptr = (int32)pi->u.pi_acphy->hirssi_byp_rssi;
		break;
	case IOV_SVAL(IOV_HIRSSI_BYP_RSSI):
		pi->u.pi_acphy->hirssi_byp_rssi = (int8)int_val;
		if (ISACPHY(pi) && PHY_SW_HIRSSI_UCODE_CAP(pi))
			wlc_phy_hirssi_elnabypass_init_acphy(pi);
		break;
	case IOV_GVAL(IOV_HIRSSI_RES_RSSI):
		*ret_int_ptr = (int32)pi->u.pi_acphy->hirssi_res_rssi;
		break;
	case IOV_SVAL(IOV_HIRSSI_RES_RSSI):
		pi->u.pi_acphy->hirssi_res_rssi = (int8)int_val;
		if (ISACPHY(pi) && PHY_SW_HIRSSI_UCODE_CAP(pi))
			wlc_phy_hirssi_elnabypass_init_acphy(pi);
		break;
	case IOV_GVAL(IOV_HIRSSI_BYP_CNT):
		*ret_int_ptr = (int32)pi->u.pi_acphy->hirssi_byp_cnt;
		break;
	case IOV_SVAL(IOV_HIRSSI_BYP_CNT):
		pi->u.pi_acphy->hirssi_byp_cnt = (int_val == -1) ? PHY_SW_HIRSSI_W1_BYP_CNT :
		(uint16)int_val;
		if (ISACPHY(pi) && PHY_SW_HIRSSI_UCODE_CAP(pi))
			wlc_phy_hirssi_elnabypass_init_acphy(pi);
		break;
	case IOV_GVAL(IOV_HIRSSI_RES_CNT):
	        *ret_int_ptr = (int32)pi->u.pi_acphy->hirssi_res_cnt;
		break;
	case IOV_SVAL(IOV_HIRSSI_RES_CNT):
		pi->u.pi_acphy->hirssi_res_cnt = (int_val == -1) ? PHY_SW_HIRSSI_W1_RES_CNT :
		(uint16)int_val;
		if (ISACPHY(pi) && PHY_SW_HIRSSI_UCODE_CAP(pi))
			wlc_phy_hirssi_elnabypass_init_acphy(pi);
		break;
	case IOV_GVAL(IOV_HIRSSI_STATUS):
		*ret_int_ptr = 0;
		if (ISACPHY(pi) && PHY_SW_HIRSSI_UCODE_CAP(pi))
			*ret_int_ptr = (int32) wlc_phy_hirssi_elnabypass_status_acphy(pi);
		break;
#endif /*  BCMDBG || WLTEST || PHYCAL_CHNG_CS */
#if defined(WLTEST)
	case IOV_SVAL(IOV_RPCALVARS): {
		const wl_rpcal_t *rpcal = p;
		if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
			PHY_ERROR(("Number of TX Chain has to be > 1!\n"));
			err = BCME_UNSUPPORTED;
		} else {
			pi->sromi->rpcal2g = rpcal[WL_CHAN_FREQ_RANGE_2G].update ?
			rpcal[WL_CHAN_FREQ_RANGE_2G].value: pi->sromi->rpcal2g;

			pi->sromi->rpcal5gb0 = rpcal[WL_CHAN_FREQ_RANGE_5G_BAND0].update ?
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND0].value : pi->sromi->rpcal5gb0;

			pi->sromi->rpcal5gb1 = rpcal[WL_CHAN_FREQ_RANGE_5G_BAND1].update ?
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND1].value : pi->sromi->rpcal5gb1;

			pi->sromi->rpcal5gb2 = rpcal[WL_CHAN_FREQ_RANGE_5G_BAND2].update ?
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND2].value : pi->sromi->rpcal5gb2;

			pi->sromi->rpcal5gb3 = rpcal[WL_CHAN_FREQ_RANGE_5G_BAND3].update ?
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND3].value : pi->sromi->rpcal5gb3;
		}
		break;
	}

	case IOV_GVAL(IOV_RPCALVARS): {
		wl_rpcal_t *rpcal = a;
		if (ACMAJORREV_1(pi->pubpi->phy_rev)) {
			PHY_ERROR(("Number of TX Chain has to be > 1!\n"));
			err = BCME_UNSUPPORTED;
		} else {
			rpcal[WL_CHAN_FREQ_RANGE_2G].value = pi->sromi->rpcal2g;
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND0].value = pi->sromi->rpcal5gb0;
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND1].value = pi->sromi->rpcal5gb1;
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND2].value = pi->sromi->rpcal5gb2;
			rpcal[WL_CHAN_FREQ_RANGE_5G_BAND3].value = pi->sromi->rpcal5gb3;
		}
		break;
	}

	case IOV_GVAL(IOV_PHY_VCOCAL):
	case IOV_SVAL(IOV_PHY_VCOCAL):
		wlapi_suspend_mac_and_wait(pi->sh->physhim);
		wlc_phy_force_vcocal_acphy(pi);
		wlapi_enable_mac(pi->sh->physhim);
		break;
#endif 
#if defined(BCMDBG)
	case IOV_SVAL(IOV_PHY_FORCE_GAINLEVEL):
	{
		wlc_phy_force_gainlevel_acphy(pi, (int16) int_val);
		break;
	}
#endif /* BCMDBG */
#if defined(WLTEST)
	case IOV_SVAL(IOV_PHY_FORCE_SPURMODE):
	{
		if (int_val == -1)
			pi->acphy_spuravoid_mode_override = 0;
		else
			pi->acphy_spuravoid_mode_override = 1;

		phy_ac_force_spurmode(pi->u.pi_acphy->rxspuri, (int16)int_val);
		break;
	}
	case IOV_GVAL(IOV_PHY_FORCE_SPURMODE):
	{
	  if (pi->acphy_spuravoid_mode_override == 1)
			*ret_int_ptr = pi->acphy_spuravoid_mode;
		else
			*ret_int_ptr = -1;
		break;
	}
#endif /* WLTEST */
#if defined(BCMDBG)
	case IOV_SVAL(IOV_PHY_FORCE_FDIQI):
	{

		wlc_phy_force_fdiqi_acphy(pi, (uint16) int_val);

		break;
	}
#endif /* BCMDBG */

	case IOV_SVAL(IOV_PHY_FORCE_CRSMIN):
	{
	    wlc_phy_force_crsmin_acphy(pi, p);
		break;
	}
#if defined(BCMDBG)
	case IOV_SVAL(IOV_PHY_BTCOEX_DESENSE):
	{
	  wlc_phy_desense_btcoex_acphy(pi, int_val);
		break;
	}
#endif /* BCMDBG */
#ifndef WLC_DISABLE_ACI
#endif /* WLC_DISABLE_ACI */
	case IOV_SVAL(IOV_EDCRS):
	{

		if (int_val == 0)
		{
			W_REG(pi->sh->osh, &pi->regs->PHYREF_IFS_CTL_SEL_PRICRS, 0x000F);
		}
		else
		{
			W_REG(pi->sh->osh, &pi->regs->PHYREF_IFS_CTL_SEL_PRICRS, 0x0F0F);
		}
		break;
	}
	case IOV_SVAL(IOV_LP_MODE):
	{
		if ((int_val > 3) || (int_val < 1)) {
			PHY_ERROR(("LP MODE %d is not supported \n", (uint16)int_val));
		} else {
			wlc_phy_lp_mode(pi, (int8) int_val);
		}
		break;
	}
	case IOV_GVAL(IOV_LP_MODE):
	{
		if (ISACPHY(pi))
			*ret_int_ptr = pi->u.pi_acphy->acphy_lp_status;
		break;

	}
	case IOV_SVAL(IOV_LP_VCO_2G):
	{
		if ((int_val != 0) && (int_val != 1)) {
			PHY_ERROR(("LP MODE %d is not supported \n", (uint16)int_val));
		} else {
			wlc_phy_force_lpvco_2G(pi, (int8) int_val);
		}
		break;
	}
	case IOV_GVAL(IOV_LP_VCO_2G):
	{
		if (ISACPHY(pi))
			*ret_int_ptr = pi->u.pi_acphy->acphy_force_lpvco_2G;
		break;
	}
#if defined(WLTEST)
	case IOV_SVAL(IOV_SMTH):
	{
		if ((ACMAJORREV_1(pi->pubpi->phy_rev) &&
			ACMINORREV_2(pi)) ||
		    ACMAJORREV_3(pi->pubpi->phy_rev) || ACMAJORREV_4(pi->pubpi->phy_rev)) {
			if ((int_val > SMTH_FOR_TXBF) || (int_val < SMTH_DISABLE)) {
				PHY_ERROR(("Smth %d is not supported \n", (uint16)int_val));
			} else {
				wlc_phy_smth(pi, (int8) int_val, SMTH_NODUMP);
			}
		} else {
			PHY_ERROR(("Smth is not supported for this chip \n"));
		}
		break;
	}
	case IOV_GVAL(IOV_SMTH):
	{
		if ((ACMAJORREV_1(pi->pubpi->phy_rev) && ACMINORREV_2(pi)) ||
		    ACMAJORREV_3(pi->pubpi->phy_rev) || ACMAJORREV_4(pi->pubpi->phy_rev)) {
			if (ISACPHY(pi))
				*ret_int_ptr = pi->u.pi_acphy->acphy_enable_smth;
		} else {
			PHY_ERROR(("Smth is not supported for this chip \n"));
		}
		break;
	}
	case IOV_SVAL(IOV_RADIO_PD):
	{
		if (CHIPID(pi->sh->chip) == BCM4335_CHIP_ID) {
			if (int_val == 1) {
				pi->u.pi_acphy->acphy_4335_radio_pd_status = 1;
				wlc_phy_radio2069_pwrdwn_seq(pi);
			} else if (int_val == 0) {
				wlc_phy_radio2069_pwrup_seq(pi);
				pi->u.pi_acphy->acphy_4335_radio_pd_status = 0;
			} else {
				PHY_ERROR(("RADIO PD %d is not supported \n", (uint16)int_val));
			}
		} else {
			PHY_ERROR(("RADIO PD is not supported for this chip \n"));
		}
		break;
	}
	case IOV_GVAL(IOV_RADIO_PD):
	{
	    *ret_int_ptr = pi->u.pi_acphy->acphy_4335_radio_pd_status;
		break;
	}
#endif /* WLTEST */
	case IOV_GVAL(IOV_EDCRS):
	{
		if (R_REG(pi->sh->osh, &pi->regs->PHYREF_IFS_CTL_SEL_PRICRS) == 0x000F)
		{
			*ret_int_ptr = 0;
		}
		else if (R_REG(pi->sh->osh, &pi->regs->PHYREF_IFS_CTL_SEL_PRICRS) == 0x0F0F)
		{
			*ret_int_ptr = 1;
		}
		break;
	}
	case IOV_GVAL(IOV_PHY_AFE_OVERRIDE):
		*ret_int_ptr = (int32)pi->afe_override;
		break;
	case IOV_SVAL(IOV_PHY_AFE_OVERRIDE):
		pi->afe_override = (uint8)int_val;
		break;
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_calib(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	bool bool_val;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	BCM_REFERENCE(bool_val);
	BCM_REFERENCE(*ret_int_ptr);

	switch (actionid) {
#if defined(WLTEST) || defined(ATE_BUILD)
	case IOV_GVAL(IOV_PHY_TXIQCC):
	{
		int32 iqccValues[4];
		uint16 valuea = 0;
		uint16 valueb = 0;
		uint16 valuea1 = 0;
		uint16 valueb1 = 0;
		if (!(ISNPHY(pi))) {
			txiqccgetfn_t txiqcc_fn = pi->pi_fptr->txiqccget;
			if (txiqcc_fn) {
				(*txiqcc_fn)(pi, &valuea, &valueb);
				iqccValues[0] = valuea;
				iqccValues[1] = valueb;
				bcopy(iqccValues, a, 2*sizeof(int32));
			}
		} else {
			txiqccmimogetfn_t txiqcc_fn = pi->pi_fptr->txiqccmimoget;
			if (txiqcc_fn) {
				(*txiqcc_fn)(pi, &valuea, &valueb, &valuea1, &valueb1);
				iqccValues[0] = valuea;
				iqccValues[1] = valueb;
				iqccValues[2] = valuea1;
				iqccValues[3] = valueb1;
				bcopy(iqccValues, a, 4*sizeof(int32));
			}
		}
		break;
	}
	case IOV_SVAL(IOV_PHY_TXIQCC):
	{
		int32 iqccValues[4];
		uint16 valuea, valueb, valuea1, valueb1;
		if (!(ISNPHY(pi))) {
			txiqccsetfn_t txiqcc_fn = pi->pi_fptr->txiqccset;
			if (txiqcc_fn) {
				bcopy(p, iqccValues, 2*sizeof(int32));
				valuea = (uint16)(iqccValues[0]);
				valueb = (uint16)(iqccValues[1]);
				(*txiqcc_fn)(pi, valuea, valueb);
			}
		} else {
			txiqccmimosetfn_t txiqcc_fn = pi->pi_fptr->txiqccmimoset;
			if (txiqcc_fn) {
				bcopy(p, iqccValues, 4*sizeof(int32));
				valuea = (uint16)(iqccValues[0]);
				valueb = (uint16)(iqccValues[1]);
				valuea1 = (uint16)(iqccValues[2]);
				valueb1 = (uint16)(iqccValues[3]);
				(*txiqcc_fn)(pi, valuea, valueb, valuea1, valueb1);
			}
		}
		break;
	}
	case IOV_GVAL(IOV_PHY_TXLOCC):
	{
		uint16 di0dq0;
		uint16 di1dq1;
		uint8 *loccValues = a;

		if (!(ISNPHY(pi))) {
			txloccgetfn_t txlocc_fn = pi->pi_fptr->txloccget;
			radioloftgetfn_t radio_loft_fn = pi->pi_fptr->radioloftget;
			if ((txlocc_fn) && (radio_loft_fn))
			{
				/* copy the 6 bytes to a */
				di0dq0 = (*txlocc_fn)(pi);
				loccValues[0] = (uint8)(di0dq0 >> 8);
				loccValues[1] = (uint8)(di0dq0 & 0xff);
				(*radio_loft_fn)(pi, &loccValues[2], &loccValues[3],
					&loccValues[4], &loccValues[5]);
			}
		} else {
			txloccmimogetfn_t txlocc_fn = pi->pi_fptr->txloccmimoget;
			radioloftmimogetfn_t radio_loft_fn = pi->pi_fptr->radioloftmimoget;

			if ((txlocc_fn) && (radio_loft_fn))
			{
				/* copy the 6 bytes to a */
				(*txlocc_fn)(pi, &di0dq0, &di1dq1);
				loccValues[0] = (uint8)(di0dq0 >> 8);
				loccValues[1] = (uint8)(di0dq0 & 0xff);
				loccValues[6] = (uint8)(di1dq1 >> 8);
				loccValues[7] = (uint8)(di1dq1 & 0xff);
				(*radio_loft_fn)(pi, &loccValues[2], &loccValues[3],
					&loccValues[4], &loccValues[5], &loccValues[8],
					&loccValues[9], &loccValues[10], &loccValues[11]);
			}
		}
		break;
	}
	case IOV_SVAL(IOV_PHY_TXLOCC):
	{
		/* copy 6 bytes from a to radio */
		uint16 di0dq0, di1dq1;
		uint8 *loccValues = p;

		if (!(ISNPHY(pi))) {
			di0dq0 = ((uint16)loccValues[0] << 8) | loccValues[1];
			if (pi->pi_fptr->txloccset && pi->pi_fptr->radioloftset) {
				pi->pi_fptr->txloccset(pi, di0dq0);
				pi->pi_fptr->radioloftset(pi, loccValues[2],
					loccValues[3], loccValues[4], loccValues[5]);
			} else
			return BCME_UNSUPPORTED;
		} else {
			di0dq0 = ((uint16)loccValues[0] << 8) | loccValues[1];
			di1dq1 = ((uint16)loccValues[6] << 8) | loccValues[7];
			if (pi->pi_fptr->txloccmimoset && pi->pi_fptr->radioloftmimoset) {
				pi->pi_fptr->txloccmimoset(pi, di0dq0, di1dq1);
				pi->pi_fptr->radioloftmimoset(pi, loccValues[2],
					loccValues[3], loccValues[4], loccValues[5],
					loccValues[8], loccValues[9], loccValues[10],
					loccValues[11]);
			}
		}
		break;
	}
#endif 

#if defined(BCMDBG) || defined(WLTEST) || defined(MACOSX) || defined(ATE_BUILD)
	case IOV_GVAL(IOV_PHY_TEMPSENSE):
		err = wlc_phy_iovar_tempsense_paldosense(pi, ret_int_ptr, 0);
		break;
#endif /* BCMDBG || WLTEST || MACOSX || ATE_BUILD */
#if defined(WLTEST)
	case IOV_GVAL(IOV_PHY_CAL_DISABLE):
		*ret_int_ptr = (int32)pi->disable_percal;
		break;

	case IOV_SVAL(IOV_PHY_CAL_DISABLE):
		pi->disable_percal = bool_val;
		break;
#endif  

#if defined(WLTEST)
	case IOV_GVAL(IOV_PHY_IDLETSSI):
		wlc_phy_iovar_idletssi(pi, ret_int_ptr, TRUE);
		break;

	case IOV_SVAL(IOV_PHY_IDLETSSI):
		wlc_phy_iovar_idletssi(pi, ret_int_ptr, bool_val);
		break;

	case IOV_GVAL(IOV_PHY_VBATSENSE):
		wlc_phy_iovar_vbatsense(pi, ret_int_ptr);
		break;

	case IOV_GVAL(IOV_PHY_IDLETSSI_REG):
		if (!pi->sh->clk)
			err = BCME_NOCLK;
		else
			err = wlc_phy_iovar_idletssi_reg(pi, ret_int_ptr, int_val, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_IDLETSSI_REG):
		if (!pi->sh->clk)
			err = BCME_NOCLK;
		else
			err = wlc_phy_iovar_idletssi_reg(pi, ret_int_ptr, int_val, TRUE);
		break;

	case IOV_GVAL(IOV_PHY_AVGTSSI_REG):
		if (!pi->sh->clk)
			err = BCME_NOCLK;
		else
			wlc_phy_iovar_avgtssi_reg(pi, ret_int_ptr);
		break;

	case IOV_SVAL(IOV_PHY_RESETCCA):
		if (ISNPHY(pi)) {
			wlc_phy_resetcca_nphy(pi);
		}
		else if (ISACPHY(pi)) {
			bool macSuspended;
			/* check if MAC already suspended */
			macSuspended = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
			if (!macSuspended) {
				wlapi_suspend_mac_and_wait(pi->sh->physhim);
			}
			wlc_phy_resetcca_acphy(pi);
			if (!macSuspended)
				wlapi_enable_mac(pi->sh->physhim);
		}
		break;
	case IOV_GVAL(IOV_PHY_PACALIDX0):

		if (ISLCNPHY(pi)) {
			uint32 papd_cal_idx;
			papd_cal_idx = 0;
			papd_cal_idx = (uint32) (pi->u.pi_lcnphy)->papd_lut0_cal_idx;
			bcopy(&papd_cal_idx, a, sizeof(uint32));
		} else if (ISACPHY(pi)) {
			int32 papd_cal_idx;
			papd_cal_idx = (int32) (pi->u.pi_acphy)->papd_lut0_cal_idx;
			bcopy(&papd_cal_idx, a, sizeof(int32));
		}

		break;
	case IOV_GVAL(IOV_PHY_PACALIDX1):

		if (ISLCNPHY(pi)) {
			uint32 papd_cal_idx;
			papd_cal_idx = 0;
			papd_cal_idx = (uint32) (pi->u.pi_lcnphy)->papd_lut1_cal_idx;
			bcopy(&papd_cal_idx, a, sizeof(uint32));
		} else if (ISACPHY(pi)) {
			uint32 papd_cal_idx;
			papd_cal_idx = (int32) (pi->u.pi_acphy)->papd_lut1_cal_idx;
			bcopy(&papd_cal_idx, a, sizeof(int32));
		}

		break;

	case IOV_SVAL(IOV_PHY_IQLOCALIDX):
		if (ISLCNPHY(pi)) {
			if (CHSPEC_IS2G(pi->radio_chanspec)) {
				OSL_DELAY(1000);
				(pi->u.pi_lcnphy)->iqlocalidx_2g = (int8) *ret_int_ptr;
				(pi->u.pi_lcnphy)->iqlocalidx2goffs = 0;
			}
#ifdef BAND5G
			else {
				OSL_DELAY(1000);
				(pi->u.pi_lcnphy)->iqlocalidx_5g = (int8) *ret_int_ptr;
				(pi->u.pi_lcnphy)->iqlocalidx5goffs = 0;
			}
#endif /* BAND5G */
		}
		break;

	case IOV_SVAL(IOV_PHY_PACALIDX):
		if (ISLCNPHY(pi)) {
				OSL_DELAY(1000);
				(pi->u.pi_lcnphy)->pacalidx = (int8) *ret_int_ptr;
		} else if (ISACPHY(pi)) {
				(pi->u.pi_acphy)->pacalidx_iovar = (int8) *ret_int_ptr;
		}
		break;

	case IOV_GVAL(IOV_PHYCAL_TEMPDELTA):
		*ret_int_ptr = (int32)pi->phycal_tempdelta;
		break;

	case IOV_SVAL(IOV_PHYCAL_TEMPDELTA):
		if (int_val == -1)
			pi->phycal_tempdelta = pi->phycal_tempdelta_default;
		else
			pi->phycal_tempdelta = (uint8)int_val;
		break;
#endif 
	case IOV_SVAL(IOV_PHY_SROM_TEMPSENSE):
	{
		pi->srom_rawtempsense = (int16)int_val;
		break;
	}

	case IOV_GVAL(IOV_PHY_SROM_TEMPSENSE):
	{
		*ret_int_ptr = pi->srom_rawtempsense;
		break;
	}
	case IOV_SVAL(IOV_PHY_RXGAIN_RSSI):
	{
		pi->u.pi_acphy->rxgaincal_rssical = (bool)int_val;
		break;
	}

	case IOV_GVAL(IOV_PHY_RXGAIN_RSSI):
	{
		*ret_int_ptr = pi->u.pi_acphy->rxgaincal_rssical;
		break;
	}
	case IOV_SVAL(IOV_PHY_GAIN_CAL_TEMP):
	{
		pi->srom_gain_cal_temp  = (int16)int_val;
		break;
	}
	case IOV_GVAL(IOV_PHY_GAIN_CAL_TEMP):
	{
		*ret_int_ptr = pi->srom_gain_cal_temp;
		break;
	}
	case IOV_SVAL(IOV_PHY_RSSI_CAL_REV):
	{
		pi->u.pi_acphy->rssi_cal_rev = (bool)int_val;
		break;
	}

	case IOV_GVAL(IOV_PHY_RSSI_CAL_REV):
	{
		*ret_int_ptr = pi->u.pi_acphy->rssi_cal_rev;
		break;
	}
	case IOV_SVAL(IOV_PHY_RUD_AGC_ENABLE):
	{
		pi->u.pi_acphy->rud_agc_enable = (bool)int_val;
		break;
	}

	case IOV_GVAL(IOV_PHY_RUD_AGC_ENABLE):
	{
		*ret_int_ptr = pi->u.pi_acphy->rud_agc_enable;
		break;
	}

#ifdef PHYMON
	case IOV_GVAL(IOV_PHYCAL_STATE): {
		if (alen < (int)sizeof(wl_phycal_state_t)) {
			err = BCME_BUFTOOSHORT;
			break;
		}
		if (ISNPHY(pi))
			err = wlc_phycal_state_nphy(pi, a, alen);
		else
			err = BCME_UNSUPPORTED;

		break;
	}
#endif /* PHYMON */
#if defined(WLTEST) || defined(AP)
	case IOV_GVAL(IOV_PHY_PERICAL):
		wlc_phy_iovar_perical_config(pi, int_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_PERICAL):
		wlc_phy_iovar_perical_config(pi, int_val, ret_int_ptr, TRUE);
		break;
#endif 

	case IOV_GVAL(IOV_PHY_PERICAL_DELAY):
		*ret_int_ptr = (int32)pi->phy_cal_delay;
		break;

	case IOV_SVAL(IOV_PHY_PERICAL_DELAY):
		if ((int_val >= PHY_PERICAL_DELAY_MIN) && (int_val <= PHY_PERICAL_DELAY_MAX))
			pi->phy_cal_delay = (uint16)int_val;
		else
			err = BCME_RANGE;
		break;

	case IOV_GVAL(IOV_PHY_PAPD_DEBUG):
		break;

	case IOV_GVAL(IOV_NOISE_MEASURE):
#if LCNCONF
		if (ISLCNPHY(pi))
		  wlc_lcnphy_noise_measure_start(pi, TRUE);
#endif
		int_val = 0;
		bcopy(&int_val, a, sizeof(int_val));
		break;
	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

static int
wlc_phy_iovars_generic(phy_info_t *pi, uint32 actionid, uint16 type, void *p, uint plen, void *a,
	int alen, int vsize)
{
	int32 int_val = 0;
	bool bool_val;
	int err = BCME_OK;
	int32 *ret_int_ptr = (int32 *)a;

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	BCM_REFERENCE(*ret_int_ptr);
	BCM_REFERENCE(bool_val);

	switch (actionid) {

#if defined(BCMDBG) || defined(WLTEST)
	case IOV_GVAL(IOV_FAST_TIMER):
		*ret_int_ptr = (int32)pi->sh->fast_timer;
		break;

	case IOV_SVAL(IOV_FAST_TIMER):
		pi->sh->fast_timer = (uint32)int_val;
		break;

	case IOV_GVAL(IOV_SLOW_TIMER):
		*ret_int_ptr = (int32)pi->sh->slow_timer;
		break;

	case IOV_SVAL(IOV_SLOW_TIMER):
		pi->sh->slow_timer = (uint32)int_val;
		break;

#endif /* BCMDBG || WLTEST */
#if defined(BCMDBG) || defined(WLTEST) || defined(PHYCAL_CHNG_CS)
	case IOV_GVAL(IOV_GLACIAL_TIMER):
		*ret_int_ptr = (int32)pi->sh->glacial_timer;
		break;

	case IOV_SVAL(IOV_GLACIAL_TIMER):
		pi->sh->glacial_timer = (uint32)int_val;
		break;
#endif
#if defined(WLTEST) || defined(MACOSX) || defined(DBG_PHY_IOV)
	case IOV_GVAL(IOV_PHY_WATCHDOG):
		*ret_int_ptr = (int32)pi->phywatchdog_override;
		break;

	case IOV_SVAL(IOV_PHY_WATCHDOG):
		pi->phywatchdog_override = bool_val;
		break;
#endif
	case IOV_GVAL(IOV_CAL_PERIOD):
	        *ret_int_ptr = (int32)pi->cal_period;
	        break;

	case IOV_SVAL(IOV_CAL_PERIOD):
	        pi->cal_period = (uint32)int_val;
	        break;
#if defined(WLTEST)
#ifdef BAND5G
	case IOV_GVAL(IOV_PHY_CGA_5G):
		/* Pass on existing channel based offset into wl */
		bcopy(pi->phy_cga_5g, a, 24*sizeof(int8));
		break;
#endif /* BAND5G */
	case IOV_GVAL(IOV_PHY_CGA_2G):
		/* Pass on existing channel based offset into wl */
		bcopy(pi->phy_cga_2g, a, 14*sizeof(int8));
		break;

	case IOV_GVAL(IOV_PHYHAL_MSG):
		*ret_int_ptr = (int32)phyhal_msg_level;
		break;

	case IOV_SVAL(IOV_PHYHAL_MSG):
		phyhal_msg_level = (uint32)int_val;
		break;

	case IOV_SVAL(IOV_PHY_FIXED_NOISE):
		pi->phy_fixed_noise = bool_val;
		break;

	case IOV_GVAL(IOV_PHY_FIXED_NOISE):
		int_val = (int32)pi->phy_fixed_noise;
		bcopy(&int_val, a, vsize);
		break;

	case IOV_GVAL(IOV_PHYNOISE_POLL):
		*ret_int_ptr = (int32)pi->phynoise_polling;
		break;

	case IOV_SVAL(IOV_PHYNOISE_POLL):
		pi->phynoise_polling = bool_val;
		break;

	case IOV_GVAL(IOV_CARRIER_SUPPRESS):
		if (!ISLCNPHY(pi))
			err = BCME_UNSUPPORTED; /* lcnphy for now */
		*ret_int_ptr = (pi->carrier_suppr_disable == 0);
		break;

	case IOV_SVAL(IOV_CARRIER_SUPPRESS):
	{
		initfn_t carr_suppr_fn = pi->pi_fptr->carrsuppr;
		if (carr_suppr_fn) {
			pi->carrier_suppr_disable = bool_val;
			if (pi->carrier_suppr_disable) {
				(*carr_suppr_fn)(pi);
			}
			PHY_INFORM(("Carrier Suppress Called\n"));
		} else
			err = BCME_UNSUPPORTED;
		break;
	}

	case IOV_GVAL(IOV_PKTENG_STATS):
	  wlc_phy_pkteng_stats_get(pi, a, alen);
		break;
#ifdef BAND5G
	case IOV_GVAL(IOV_PHY_SUBBAND5GVER):
		/* Retrieve 5G subband version */
		int_val = (uint8)(pi->sromi->subband5Gver);
		bcopy(&int_val, a, vsize);
		break;
#endif /* BAND5G */
	case IOV_GVAL(IOV_PHY_TXRX_CHAIN):
		wlc_phy_iovar_txrx_chain(pi, int_val, ret_int_ptr, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_TXRX_CHAIN):
		err = wlc_phy_iovar_txrx_chain(pi, int_val, ret_int_ptr, TRUE);
		break;

	case IOV_GVAL(IOV_PHY_BPHY_EVM):
		*ret_int_ptr = pi->phy_bphy_evm;
		break;

	case IOV_SVAL(IOV_PHY_BPHY_EVM):
		wlc_phy_iovar_bphy_testpattern(pi, NPHY_TESTPATTERN_BPHY_EVM, (bool) int_val);
		break;

	case IOV_GVAL(IOV_PHY_BPHY_RFCS):
		*ret_int_ptr = pi->phy_bphy_rfcs;
		break;

	case IOV_SVAL(IOV_PHY_BPHY_RFCS):
		wlc_phy_iovar_bphy_testpattern(pi, NPHY_TESTPATTERN_BPHY_RFCS, (bool) int_val);
		break;

	case IOV_GVAL(IOV_PHY_SCRAMINIT):
		*ret_int_ptr = pi->phy_scraminit;
		break;

	case IOV_SVAL(IOV_PHY_SCRAMINIT):
		wlc_phy_iovar_scraminit(pi, (uint8)int_val);
		break;

	case IOV_SVAL(IOV_PHY_RFSEQ):
		wlc_phy_iovar_force_rfseq(pi, (uint8)int_val);
		break;

	case IOV_GVAL(IOV_PHY_TX_TONE_HZ):
		*ret_int_ptr = pi->phy_tx_tone_freq;
		break;

	case IOV_SVAL(IOV_PHY_TX_TONE_STOP):
		wlc_phy_iovar_tx_tone_stop(pi);
		break;

	case IOV_SVAL(IOV_PHY_TX_TONE_HZ):
		wlc_phy_iovar_tx_tone_hz(pi, (int32)int_val);
		break;

	case IOV_GVAL(IOV_PHY_TEST_TSSI):
		*((uint*)a) = wlc_phy_iovar_test_tssi(pi, (uint8)int_val, 0);
		break;

	case IOV_GVAL(IOV_PHY_TEST_TSSI_OFFS):
		*((uint*)a) = wlc_phy_iovar_test_tssi(pi, (uint8)int_val, 12);
		break;

	case IOV_GVAL(IOV_PHY_TEST_IDLETSSI):
		*((uint*)a) = wlc_phy_iovar_test_idletssi(pi, (uint8)int_val);
		break;

	case IOV_SVAL(IOV_PHY_SETRPTBL):
		wlc_phy_iovar_setrptbl(pi);
		break;

	case IOV_SVAL(IOV_PHY_FORCEIMPBF):
		wlc_phy_iovar_forceimpbf(pi);
		break;

	case IOV_SVAL(IOV_PHY_FORCESTEER):
		wlc_phy_iovar_forcesteer(pi, (uint8)int_val);
		break;
#ifdef BAND5G
	case IOV_SVAL(IOV_PHY_5G_PWRGAIN):
		pi->phy_5g_pwrgain = bool_val;
		break;

	case IOV_GVAL(IOV_PHY_5G_PWRGAIN):
		*ret_int_ptr = (int32)pi->phy_5g_pwrgain;
		break;
#endif /* BAND5G */

	case IOV_SVAL(IOV_PHY_ENABLERXCORE):
		wlc_phy_iovar_rxcore_enable(pi, int_val, bool_val, ret_int_ptr, TRUE);
		break;

	case IOV_GVAL(IOV_PHY_ENABLERXCORE):
		wlc_phy_iovar_rxcore_enable(pi, int_val, bool_val, ret_int_ptr, FALSE);
		break;

	case IOV_GVAL(IOV_PHY_ACTIVECAL):
		*ret_int_ptr = (int32)((pi->cal_info->cal_phase_id !=
			MPHASE_CAL_STATE_IDLE)? 1 : 0);
		break;

	case IOV_SVAL(IOV_PHY_BBMULT):
		if (!pi->sh->clk) {
			err = BCME_NOCLK;
			break;
		}
		err = wlc_phy_iovar_bbmult_set(pi, p);
		break;

	case IOV_GVAL(IOV_PHY_BBMULT):
		if (!pi->sh->clk) {
			err = BCME_NOCLK;
			break;
		}
		wlc_phy_iovar_bbmult_get(pi, int_val, bool_val, ret_int_ptr);
		break;

#if defined(WLC_LOWPOWER_BEACON_MODE)
	case IOV_GVAL(IOV_PHY_LOWPOWER_BEACON_MODE):
		if (ISLCN40PHY(pi)) {
			*ret_int_ptr = (pi->u.pi_lcn40phy)->lowpower_beacon_mode;
		}
		break;

	case IOV_SVAL(IOV_PHY_LOWPOWER_BEACON_MODE):
		wlc_phy_lowpower_beacon_mode(pih, int_val);
		break;
#endif /* WLC_LOWPOWER_BEACON_MODE */
#endif 
#if defined(WLTEST)
	case IOV_GVAL(IOV_PKTENG_GAININDEX):
		if (!pi->sh->clk) {
			err = BCME_NOCLK;
			break;
		}
		err = wlc_phy_pkteng_get_gainindex(pi, ret_int_ptr);
		break;

#endif  
#if defined(WLTEST) || defined(DBG_PHY_IOV) || defined(WFD_PHY_LL_DEBUG) || \
	defined(ATE_BUILD)
	case IOV_GVAL(IOV_PHY_FORCECAL):
		err = wlc_phy_iovar_forcecal(pi, int_val, ret_int_ptr, vsize, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_FORCECAL):
		err = wlc_phy_iovar_forcecal(pi, int_val, ret_int_ptr, vsize, TRUE);
		break;

	case IOV_SVAL(IOV_PAPD_EN_WAR):
		wlapi_bmac_write_shm(pi->sh->physhim, M_PAPDOFF_MCS, (uint16)int_val);
		break;

	case IOV_GVAL(IOV_PAPD_EN_WAR):
		*ret_int_ptr = wlapi_bmac_read_shm(pi->sh->physhim, M_PAPDOFF_MCS);
		break;

	case IOV_GVAL(IOV_PHY_TX_TONE):
	case IOV_GVAL(IOV_PHY_TXLO_TONE):
		*ret_int_ptr = pi->phy_tx_tone_freq;
		break;

	case IOV_SVAL(IOV_PHY_TX_TONE):
		wlc_phy_iovar_tx_tone(pi, (int32)int_val);
		break;

	case IOV_SVAL(IOV_PHY_TXLO_TONE):
		wlc_phy_iovar_txlo_tone(pi);
		break;

#ifndef ATE_BUILD
	case IOV_SVAL(IOV_PHY_SKIPPAPD):
		if ((int_val != 0) && (int_val != 1)) {
			err = BCME_RANGE;
			break;
		}
		if (ISACPHY(pi))
			pi->u.pi_acphy->acphy_papd_skip = (uint8)int_val;
		break;

	case IOV_GVAL(IOV_PHY_SKIPPAPD):
		if (ISACPHY(pi))
		        *ret_int_ptr = pi->u.pi_acphy->acphy_papd_skip;
		break;

	case IOV_GVAL(IOV_PHY_FORCECAL_OBT):
		err = wlc_phy_iovar_forcecal_obt(pi, int_val, ret_int_ptr, vsize, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_FORCECAL_OBT):
		err = wlc_phy_iovar_forcecal_obt(pi, int_val, ret_int_ptr, vsize, FALSE);
		break;

	case IOV_GVAL(IOV_PHY_FORCECAL_NOISE): /* Get crsminpwr for core 0 & core 1 */
		err = wlc_phy_iovar_forcecal_noise(pi, int_val, a, vsize, FALSE);
		break;

	case IOV_SVAL(IOV_PHY_FORCECAL_NOISE): /* do only Noise Cal */
		err = wlc_phy_iovar_forcecal_noise(pi, int_val, a, vsize, TRUE);
		break;
#endif /* !ATE_BUILD */
#endif 
#if defined(WLTEST) || defined(MACOSX)
	case IOV_SVAL(IOV_PHY_DEAF):
		wlc_phy_iovar_set_deaf(pi, int_val);
		break;
	case IOV_GVAL(IOV_PHY_DEAF):
		err = wlc_phy_iovar_get_deaf(pi, ret_int_ptr);
		break;
#endif 
#ifdef WLTEST
	case IOV_GVAL(IOV_PHY_FEM2G): {
		bcopy(pi->fem2g, a, sizeof(srom_fem_t));
		break;
	}

	case IOV_SVAL(IOV_PHY_FEM2G): {
		bcopy(p, pi->fem2g, sizeof(srom_fem_t));
		/* srom_fem2g.extpagain changed after attach time */
		wlc_phy_txpower_ipa_upd(pi);
		break;
	}

#ifdef BAND5G
	case IOV_GVAL(IOV_PHY_FEM5G): {
		bcopy(pi->fem5g, a, sizeof(srom_fem_t));
		break;
	}

	case IOV_SVAL(IOV_PHY_FEM5G): {
		bcopy(p, pi->fem5g, sizeof(srom_fem_t));
		/* srom_fem5g.extpagain changed after attach time */
		wlc_phy_txpower_ipa_upd(pi);
		break;
	}
#endif /* BAND5G */
#endif /* WLTEST */
	case IOV_GVAL(IOV_PHY_RXIQ_EST):
	{
		bool suspend;
		bool low_pwr = FALSE;
		uint16 r;
		int temp_dBm;

		if (!pi->sh->up) {
			err = BCME_NOTUP;
			break;
		}

		/* make sure bt-prisel is on WLAN side */
		wlc_phy_btcx_wlan_critical_enter(pi);

		suspend = !(R_REG(pi->sh->osh, &pi->regs->maccontrol) & MCTL_EN_MAC);
		if (!suspend) {
			wlapi_suspend_mac_and_wait(pi->sh->physhim);
		}

		phy_utils_phyreg_enter(pi);

		/* For 4350 Olympic program, -i 0 should behave exactly same as -i 1
		 * So, if there is force gain type is 0, then make it 1 for 4350
		 */
		if ((pi->u.pi_acphy->rud_agc_enable == TRUE) &&
			(pi->phy_rxiq_force_gain_type == 0) &&
			(pi->phy_rxiq_extra_gain_3dB == 0)) {
			pi->phy_rxiq_force_gain_type = 1;
		}
		/* get IQ power measurements */
		*ret_int_ptr = wlc_phy_rx_iq_est(pi, pi->phy_rxiq_samps, pi->phy_rxiq_antsel,
		                                 pi->phy_rxiq_resln, pi->phy_rxiq_lpfhpc,
		                                 pi->phy_rxiq_diglpf,
		                                 pi->phy_rxiq_gain_correct,
		                                 pi->phy_rxiq_extra_gain_3dB, 0,
		                                 pi->phy_rxiq_force_gain_type);

		if ((pi->u.pi_acphy->rud_agc_enable == TRUE) &&
			(pi->phy_rxiq_force_gain_type == 1) && (pi->phy_rxiq_resln == 1)) {
			FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, r) {
				temp_dBm = *ret_int_ptr;
				temp_dBm = (temp_dBm >> (10*r)) & 0x3ff;
				temp_dBm = ((int16)(temp_dBm << 6)) >> 6; /* sign extension */
				if ((temp_dBm >> 2) < -82) {
					low_pwr = TRUE;
				}
				PHY_RXIQ(("In %s: | For core %d | iqest_dBm = %d"
					  " \n", __FUNCTION__, r, (temp_dBm >> 2)));
			}
			if (low_pwr) {
				pi->phy_rxiq_force_gain_type = 9;
				*ret_int_ptr = wlc_phy_rx_iq_est(pi, pi->phy_rxiq_samps,
					pi->phy_rxiq_antsel,
					pi->phy_rxiq_resln, pi->phy_rxiq_lpfhpc,
					pi->phy_rxiq_diglpf, pi->phy_rxiq_gain_correct,
					pi->phy_rxiq_extra_gain_3dB, 0,
					pi->phy_rxiq_force_gain_type);
			}
		}

		phy_utils_phyreg_exit(pi);

		if (!suspend)
			wlapi_enable_mac(pi->sh->physhim);
		wlc_phy_btcx_wlan_critical_exit(pi);
		break;
	}

	case IOV_SVAL(IOV_PHY_RXIQ_EST):

		{
			uint8 samples, antenna, resolution, lpf_hpc, dig_lpf;
			uint8 gain_correct, extra_gain_3dB, force_gain_type;

			extra_gain_3dB = (int_val >> 28) & 0xf;
			gain_correct = (int_val >> 24) & 0xf;
			lpf_hpc = (int_val >> 20) & 0x3;
			dig_lpf = (int_val >> 22) & 0x3;
			resolution = (int_val >> 16) & 0xf;
			samples = (int_val >> 8) & 0xff;
			antenna = int_val & 0xf;
			force_gain_type = (int_val >> 4) & 0xf;
#if defined(WLTEST)
			if (ISLCNCOMMONPHY(pi)) {
				uint8 index, elna, index_valid;
				phy_info_lcnphy_t *pi_lcn = wlc_phy_getlcnphy_common(pi);
				antenna = int_val & 0x7f;
				elna =  (int_val >> 20) & 0x1;
				index = (((int_val >> 28) & 0xF) << 3) | ((int_val >> 21) & 0x7);
				index_valid = (int_val >> 7) & 0x1;
				if (index_valid)
					pi_lcn->rxpath_index = index;
				else
					pi_lcn->rxpath_index = 0xFF;
				pi_lcn->rxpath_elna = elna;
			}
#endif
		       if (CHIPID_4324X_EPA_FAMILY(pi)) {
				uint8 index;
				phy_info_nphy_t *pi_nphy = pi->u.pi_nphy;
				index = (((int_val >> 28) & 0xF) << 3) | ((int_val >> 21) & 0x7);
				pi_nphy->gainindex = index;
			}

			if (gain_correct > 4) {
				err = BCME_RANGE;
				break;
			}

			if (!ISLCNCOMMONPHY(pi) && !(CHIPID_4324X_EPA_FAMILY(pi))) {
				if ((lpf_hpc != 0) && (lpf_hpc != 1)) {
					err = BCME_RANGE;
					break;
				}
				if (dig_lpf > 2) {
					err = BCME_RANGE;
					break;
					}
			}

			if ((resolution != 0) && (resolution != 1)) {
				err = BCME_RANGE;
				break;
			}

			if (samples < 10 || samples > 15) {
				err = BCME_RANGE;
				break;
			}


			/* Limit max number of samples to 2^14 since Lcnphy RXIQ Estimator
			 * takes too much and variable time for more than that.
			*/
			if (ISLCNCOMMONPHY(pi)) {
				samples = MIN(14, samples);
			}
			if (!(CHIPID_4324X_EPA_FAMILY(pi))) {
				if ((antenna != ANT_RX_DIV_FORCE_0) &&
					(antenna != ANT_RX_DIV_FORCE_1) &&
					(antenna != ANT_RX_DIV_DEF)) {
						err = BCME_RANGE;
						break;
				}
			}
			pi->phy_rxiq_samps = samples;
			pi->phy_rxiq_antsel = antenna;
			pi->phy_rxiq_resln = resolution;
			pi->phy_rxiq_lpfhpc = lpf_hpc;
			pi->phy_rxiq_diglpf = dig_lpf;
			pi->phy_rxiq_gain_correct = gain_correct;
			pi->phy_rxiq_extra_gain_3dB = extra_gain_3dB;
			pi->phy_rxiq_force_gain_type = force_gain_type;
		}
		break;

	case IOV_GVAL(IOV_PHYNOISE_SROM):
		if (ISHTPHY(pi) || ISACPHY(pi) ||
		(ISNPHY(pi) && NREV_GE(pi->pubpi->phy_rev, 3))) {

			int8 noiselvl[PHY_CORE_MAX];
			uint8 core;
			uint32 pkd_noise = 0;
			if (!pi->sh->up) {
				err = BCME_NOTUP;
				break;
			}
			wlc_phy_get_SROMnoiselvl_phy(pi, noiselvl);
			for (core = PHYCORENUM(pi->pubpi->phy_corenum); core >= 1; core--) {
				pkd_noise = (pkd_noise << 8) | (uint8)(noiselvl[core-1]);
			}
			*ret_int_ptr = pkd_noise;
		} else if (ISLCNPHY(pi)) {
#if defined(WLTEST)
			int8 noiselvl;
			uint32 pkd_noise = 0;
			if (!pi->sh->up) {
				err = BCME_NOTUP;
				break;
			}
			wlc_phy_get_SROMnoiselvl_lcnphy(pi, &noiselvl);
			pkd_noise  = (uint8)noiselvl;
			*ret_int_ptr = pkd_noise;
#else
			return BCME_UNSUPPORTED;
#endif
		} else {
			return BCME_UNSUPPORTED;        /* only htphy supported for now */
		}
		break;

	case IOV_GVAL(IOV_NUM_STREAM):
		if (ISNPHY(pi)) {
			int_val = 2;
		} else if (ISHTPHY(pi)) {
			int_val = 3;
		} else if (ISLCNPHY(pi)) {
			int_val = 1;
		} else {
			int_val = -1;
		}
		bcopy(&int_val, a, vsize);
		break;

	case IOV_GVAL(IOV_BAND_RANGE_SUB):
		int_val = wlc_phy_chanspec_bandrange_get(pi, pi->radio_chanspec);
		bcopy(&int_val, a, vsize);
		break;

	case IOV_GVAL(IOV_BAND_RANGE):
		int_val = wlc_phy_chanspec_bandrange_get(pi, pi->radio_chanspec);
		bcopy(&int_val, a, vsize);
		break;

	case IOV_SVAL(IOV_MIN_TXPOWER):
		pi->min_txpower = (uint8)int_val;
		break;

	case IOV_GVAL(IOV_MIN_TXPOWER):
		int_val = pi->min_txpower;
		bcopy(&int_val, a, sizeof(int_val));
		break;

	case IOV_SVAL(IOV_ANT_DIV_SW_CORE0):
	{
		if (ISACPHY(pi)) {
			if ((int_val > 2) || (int_val < 0)) {
				PHY_ERROR(("Value %d is not supported \n", (uint16)int_val));
			} else {
				wlc_ant_div_sw_control(pi, (int8) int_val, 0);
			}
		} else {
			PHY_ERROR(("IOVAR is not supported for this chip \n"));
		}
		break;
	}

	case IOV_GVAL(IOV_ANT_DIV_SW_CORE0):
	{
		if (ISACPHY(pi)) {
				*ret_int_ptr = pi->u.pi_acphy->ant_swOvr_state_core0;
		} else {
			PHY_ERROR(("IOVAR is not supported for this chip \n"));
		}
		break;
	}
	case IOV_SVAL(IOV_ANT_DIV_SW_CORE1):
	{
		if (ISACPHY(pi)) {
			if ((int_val > 2) || (int_val < 0)) {
				PHY_ERROR(("Value %d is not supported \n", (uint16)int_val));
			} else {
				wlc_ant_div_sw_control(pi, (int8) int_val, 1);
			}
		} else {
			PHY_ERROR(("IOVAR is not supported for this chip \n"));
		}
		break;
	}

	case IOV_GVAL(IOV_ANT_DIV_SW_CORE1):
	{
		if (ISACPHY(pi)) {
			*ret_int_ptr = pi->u.pi_acphy->ant_swOvr_state_core1;
		} else {
			PHY_ERROR(("IOVAR is not supported for this chip \n"));
		}
		break;
	}

#if defined(WLTEST)
	case IOV_GVAL(IOV_TSSIVISI_THRESH):
		int_val = wlc_phy_tssivisible_thresh((wlc_phy_t *)pi);
		bcopy(&int_val, a, sizeof(int_val));
		break;
#endif 

#if defined(MACOSX)
	case IOV_GVAL(IOV_PHYWREG_LIMIT):
		int_val = pi->phy_wreg_limit;
		bcopy(&int_val, a, vsize);
		break;

	case IOV_SVAL(IOV_PHYWREG_LIMIT):
		pi->phy_wreg_limit = (uint8)int_val;
		break;
#endif 
	case IOV_GVAL(IOV_PHY_MUTED):
		*ret_int_ptr = PHY_MUTED(pi) ? 1 : 0;
		break;

#if defined(RXDESENS_EN)
	case IOV_GVAL(IOV_PHY_RXDESENS):
		if (ISNPHY(pi))
			err = wlc_nphy_get_rxdesens((wlc_phy_t *)pi, ret_int_ptr);
		else if (ISACPHY(pi) && !ACPHY_ENABLE_FCBS_HWACI(pi) &&
		         pi->u.pi_acphy->total_desense.forced) {
			*ret_int_ptr = (int32)pi->u.pi_acphy->total_desense.ofdm_desense;
		} else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_SVAL(IOV_PHY_RXDESENS):
		if (ISNPHY(pi) && (pi->sh->interference_mode == INTERFERE_NONE))
			err = wlc_nphy_set_rxdesens((wlc_phy_t *)pi, int_val);
		else
#if ACCONF || ACCONF2
			if (ISACPHY(pi) && !(ACPHY_ENABLE_FCBS_HWACI(pi)) {
				wlapi_suspend_mac_and_wait(pi->sh->physhim);
				if (int_val <= 0) {
					/* disable phy_rxdesens and restore
					 * default interference mode
					 */
					pi->u.pi_acphy->total_desense.forced = FALSE;

					pi->sh->interference_mode_override = FALSE;
					if (CHSPEC_IS2G(pi->radio_chanspec)) {
						pi->sh->interference_mode =
							pi->sh->interference_mode_2G;
					} else {
						pi->sh->interference_mode =
							pi->sh->interference_mode_5G;
					}

#ifndef WLC_DISABLE_ACI
					/* turn off interference mode
					 * before entering another mode
					 */
					if (pi->sh->interference_mode != INTERFERE_NONE)
						wlc_phy_interference(pi, INTERFERE_NONE, TRUE);

					if (!wlc_phy_interference
						(pi, pi->sh->interference_mode, TRUE))
						err = BCME_BADOPTION;
#endif /* !defined(WLC_DISABLE_ACI) */

					/* restore crsmincal automode, and force crsmincal */
					int8 negative = -1;
					wlc_phy_force_crsmin_acphy(pi, &negative);

				} else {
					/* enable phy_rxdesens and disable interference mode
					* through override mode
					*/
					pi->sh->interference_mode_override = TRUE;
					pi->sh->interference_mode_2G_override = INTERFERE_NONE;
					pi->sh->interference_mode_5G_override = INTERFERE_NONE;
					if (CHSPEC_IS2G(pi->radio_chanspec)) {
						pi->sh->interference_mode =
							pi->sh->interference_mode_2G_override;
					} else {
						pi->sh->interference_mode =
							pi->sh->interference_mode_5G_override;
					}
					wlc_phy_interference(pi, INTERFERE_NONE, TRUE);

					/* disable crsmincal */
					pi->u.pi_acphy->crsmincal_enable = FALSE;

					/* apply desense */
					pi->u.pi_acphy->total_desense.forced = TRUE;
					pi->u.pi_acphy->total_desense.ofdm_desense = (uint8)int_val;
					pi->u.pi_acphy->total_desense.bphy_desense = (uint8)int_val;
					wlc_phy_desense_apply_acphy(pi, TRUE);

				}
				wlapi_enable_mac(pi->sh->physhim);

			} else
#endif /* ACCONF || ACCONF2 */
				err = BCME_UNSUPPORTED;
		break;
#endif /* defined(RXDESENS_EN) */
	case IOV_GVAL(IOV_PHY_RXANTSEL):
		if (ISNPHY(pi) && NREV_GE(pi->pubpi->phy_rev, 7))
			*ret_int_ptr = pi->nphy_enable_hw_antsel ? 1 : 0;
		break;

	case IOV_SVAL(IOV_PHY_RXANTSEL):
		if (ISNPHY(pi) && NREV_GE(pi->pubpi->phy_rev, 7)) {
			pi->nphy_enable_hw_antsel = bool_val;
			/* make sure driver is up (so clks are on) before writing to PHY regs */
			if (pi->sh->up) {
				wlc_phy_init_hw_antsel(pi);
			}
		}
		break;
#ifdef ENABLE_FCBS
	case IOV_SVAL(IOV_PHY_FCBSINIT):
		if (ISHTPHY(pi)) {
			if ((int_val >= FCBS_CHAN_A) && (int_val <= FCBS_CHAN_B)) {
				wlc_phy_fcbs_init((wlc_phy_t*)pi, int_val);
			} else {
				err = BCME_RANGE;
			}
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
	case IOV_SVAL(IOV_PHY_FCBS):
		if (ISACPHY(pi)) {
			pi->FCBS = (bool)int_val;
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
	case IOV_GVAL(IOV_PHY_FCBS):
		if (ISACPHY(pi)) {
			*ret_int_ptr = pi->FCBS;
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
	case IOV_GVAL(IOV_PHY_FCBSARM):
		if (ISACPHY(pi)) {
			wlc_phy_fcbs_arm((wlc_phy_t*)pi, 0xFFFF, 0);
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
	case IOV_GVAL(IOV_PHY_FCBSEXIT):
		if (ISACPHY(pi)) {
			 wlc_phy_fcbs_exit((wlc_phy_t*)pi);
		} else {
			err = BCME_UNSUPPORTED;
		}
		break;
#endif /* ENABLE_FCBS */
#if ((ACCONF != 0) || (ACCONF2 != 0) || (NCONF != 0) || (HTCONF != 0) || (LCN40CONF != \
	0))
	case IOV_SVAL(IOV_ED_THRESH):
		err = wlc_phy_adjust_ed_thres(pi, &int_val, TRUE);
		break;
	case IOV_GVAL(IOV_ED_THRESH):
		err = wlc_phy_adjust_ed_thres(pi, ret_int_ptr, FALSE);
		break;
#endif /* ACCONF || ACCONF2 || NCONF || HTCONF || LCN40CONF */
	case IOV_GVAL(IOV_PHY_BTC_RESTAGE_RXGAIN):
		err = wlc_phy_iovar_get_btc_restage_rxgain(pi, ret_int_ptr);
		break;
	case IOV_SVAL(IOV_PHY_BTC_RESTAGE_RXGAIN):
		err = wlc_phy_iovar_set_btc_restage_rxgain(pi, int_val);
		break;
	case IOV_GVAL(IOV_PHY_DSSF):
	        err = wlc_phy_iovar_get_dssf(pi, ret_int_ptr);
		break;
	case IOV_SVAL(IOV_PHY_DSSF):
		err = wlc_phy_iovar_set_dssf(pi, int_val);
		break;
#ifdef WL_SARLIMIT
	case IOV_SVAL(IOV_PHY_SAR_LIMIT):
	{
		wlc_phy_sar_limit_set((wlc_phy_t*)pi, (uint32)int_val);
		break;
	}

	case IOV_GVAL(IOV_PHY_TXPWR_CORE):
	{
		uint core;
		uint32 sar = 0;
		uint8 tmp;

		FOREACH_CORE(pi, core) {
			if (pi->txpwr_max_percore_override[core] != 0)
				tmp = pi->txpwr_max_percore_override[core];
			else
				tmp = pi->txpwr_max_percore[core];

			sar |= (tmp << (core * 8));
		}
		*ret_int_ptr = (int32)sar;
		break;
	}
#if defined(WLTEST) || defined(BCMDBG)
	case IOV_SVAL(IOV_PHY_TXPWR_CORE):
	{
		uint core;

		if (!pi->sh->up) {
			err = BCME_NOTUP;
			break;
		} else if (!ISHTPHY(pi)) {
			err = BCME_UNSUPPORTED;
			break;
		}

		FOREACH_CORE(pi, core) {
			pi->txpwr_max_percore_override[core] =
				(uint8)(((uint32)int_val) >> (core * 8) & 0x7f);
			if (pi->txpwr_max_percore_override[core] != 0) {
				pi->txpwr_max_percore_override[core] =
					MIN(WLC_TXPWR_MAX,
					MAX(pi->txpwr_max_percore_override[core],
					pi->min_txpower));
			}
		}
		phy_tpc_recalc_tgt(pi->tpci);
		break;
	}
#endif /* WLTEST || BCMDBG */
#endif /* WL_SARLIMIT */

	case IOV_GVAL(IOV_PHY_TXSWCTRLMAP): {
		/* Getter mode, return the previously set value. */
		if (pi->pi_fptr->txswctrlmapgetptr) {
			*ret_int_ptr = (int32) pi->pi_fptr->txswctrlmapgetptr(pi);
		} else {
			/* Not implemented for this phy. */
			err = BCME_UNSUPPORTED;
			PHY_ERROR(("Command not supported for this phy\n"));
		}
		break;
	}
	case IOV_SVAL(IOV_PHY_TXSWCTRLMAP): {
		if (pi->pi_fptr->txswctrlmapsetptr) {
			if (!((int_val >= AUTO) && (int_val <= PAMODE_HI_EFF))) {
				PHY_ERROR(("Value out of range\n"));
				err = BCME_RANGE;
				break;
			}
			/* Setter mode, sets the value. */
			pi->pi_fptr->txswctrlmapsetptr(pi, (int8)int_val);
		} else {
			/* Not implemented for this phy. */
			err = BCME_UNSUPPORTED;
			PHY_ERROR(("Command not supported for this phy\n"));
		}
		break;
	}

#if defined(WFD_PHY_LL)
	case IOV_SVAL(IOV_PHY_WFD_LL_ENABLE):
		if ((int_val < 0) || (int_val > 2)) {
			err = BCME_RANGE;
			break;
		}
		if (ISNPHY(pi) || ISACPHY(pi)) {
			/* Force the channel to be active */
			pi->wfd_ll_chan_active_force =  (int_val == 2) ?TRUE : FALSE;

			pi->wfd_ll_enable_pending = (uint8)int_val;
			if (!PHY_PERICAL_MPHASE_PENDING(pi)) {
				/* Apply it since there is no CAL in progress */
				pi->wfd_ll_enable = (uint8)int_val;
				if (!int_val) {
					/* Force a watchdog CAL when disabling WFD optimization
					 * As PADP CAL has not been executed since a long time
					 * a PADP CAL is executed at the next watchdog timeout
					 */
					 pi->cal_info->last_cal_time = 0;
				}
			}
		}
		else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_GVAL(IOV_PHY_WFD_LL_ENABLE):
		if (ISNPHY(pi) || ISACPHY(pi)) {
			*ret_int_ptr = pi->wfd_ll_enable;
		}
		else err = BCME_UNSUPPORTED;
		break;
#endif /* WFD_PHY_LL */

#if defined(WLTEST) || defined(BCMDBG)

		case IOV_SVAL(IOV_PHY_ENABLE_EPA_DPD_2G):
		case IOV_SVAL(IOV_PHY_ENABLE_EPA_DPD_5G):
		{
			if (pi->pi_fptr->epadpdsetptr) {
				if ((int_val < 0) || (int_val > 1)) {
					err = BCME_RANGE;
					PHY_ERROR(("Value out of range\n"));
					break;
				}
				pi->pi_fptr->epadpdsetptr(pi, (uint8)int_val,
					(actionid == IOV_SVAL(IOV_PHY_ENABLE_EPA_DPD_2G)));
			} else {
				/* Not implemented for this phy. */
				err = BCME_UNSUPPORTED;
				PHY_ERROR(("Command not supported for this phy\n"));
			}
			break;
		}

		case IOV_GVAL(IOV_PHY_EPACAL2GMASK): {
			*ret_int_ptr = (uint32)pi->epacal2g_mask;
			break;
		}

		case IOV_SVAL(IOV_PHY_EPACAL2GMASK): {
			pi->epacal2g_mask = (uint16)int_val;
			break;
		}
#endif /* defined(WLTEST) || defined(BCMDBG) */

	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}


#endif /* WLC_LOW */

#ifdef WLC_HIGH_ONLY
#include <bcm_xdr.h>

static bool
phy_legacy_pack_iov(wlc_info_t *wlc, uint32 aid, void *p, uint p_len, bcm_xdr_buf_t *b)
{
	int err;

	BCM_REFERENCE(err);

	/* Decide the buffer is 16-bit or 32-bit buffer */
	switch (IOV_ID(aid)) {
	case IOV_PKTENG_STATS:
	case IOV_POVARS:
		p_len &= ~3;
		err = bcm_xdr_pack_uint32(b, p_len);
		ASSERT(!err);
		err = bcm_xdr_pack_uint32_vec(b, p_len, p);
		ASSERT(!err);
		return TRUE;
	case IOV_PAVARS:
		p_len &= ~1;
		err = bcm_xdr_pack_uint32(b, p_len);
		ASSERT(!err);
		err = bcm_xdr_pack_uint16_vec(b, p_len, p);
		ASSERT(!err);
		return TRUE;
	}
	return FALSE;
}

static bool
phy_legacy_unpack_iov(wlc_info_t *wlc, uint32 aid, bcm_xdr_buf_t *b, void *a, uint a_len)
{
	/* Dealing with all the structures/special cases */
	switch (aid) {
	case IOV_GVAL(IOV_PHY_SAMPLE_COLLECT):
		WL_ERROR(("%s: nphy_sample_collect need endianess conversion code\n",
		          __FUNCTION__));
		break;
	case IOV_GVAL(IOV_PHY_SAMPLE_DATA):
		WL_ERROR(("%s: nphy_sample_data need endianess conversion code\n",
		          __FUNCTION__));
		break;
	}
	return FALSE;
}
#endif /* WLC_HIGH_ONLY */

/* register iovar table to the system */
#include <phy_api.h>

#include <wlc_iocv_types.h>
#include <wlc_iocv_reg.h>

int phy_legacy_register_iovt(phy_info_t *pi, wlc_iocv_info_t *ii);

int
BCMATTACHFN(phy_legacy_register_iovt)(phy_info_t *pi, wlc_iocv_info_t *ii)
{
	wlc_iovt_desc_t iovd;

	ASSERT(ii != NULL);

	wlc_iocv_init_iovd(phy_iovars,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_old_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_generic,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_generic_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_calib,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_calib_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_acphy,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_acphy_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_nphy,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_nphy_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_lcncmnphy,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_lcncmnphy_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_txpwrctl,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_txpwrctl_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
	wlc_iocv_init_iovd(phy_iovars_rssi,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_rssi_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
#ifdef SAMPLE_COLLECT
	wlc_iocv_init_iovd(phy_iovars_sc,
	                   phy_legacy_pack_iov, phy_legacy_unpack_iov,
	                   phy_legacy_doiovar, pi,
	                   &iovd);
	if (wlc_iocv_register_iovt(ii, &iovd) != BCME_OK) {
		PHY_ERROR(("%s: phy_sc_register_iovt failed\n", __FUNCTION__));
		goto fail;
	}
#endif

	return BCME_OK;
fail :
	return BCME_ERROR;
}
