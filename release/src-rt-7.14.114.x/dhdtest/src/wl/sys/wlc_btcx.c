/*
 * BT Coex module
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
 * $Id: wlc_btcx.c 530011 2015-01-29 00:27:13Z $
 */


#include <wlc_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <sbchipc.h>
#include <bcmendian.h>
#include <proto/802.11.h>
#include <wlioctl.h>
#include <bcmwpa.h>
#include <bcmwifi_channels.h>
#include <bcmdevs.h>
#include <d11.h>
#include <wlc_rate.h>
#include <wlc_pub.h>
#include <wlc_key.h>
#include <wlc_channel.h>
#include <wlc_bsscfg.h>
#include <wlc.h>
#include <wlc_scb.h>
#include <wlc_btcx.h>
#include <wlc_scan.h>
#include <wlc_assoc.h>
#include <wlc_bmac.h>
#include <wlc_ap.h>
#include <wlc_stf.h>
#include <wlc_ampdu.h>
#include <wlc_ampdu_rx.h>
#include <wlc_ampdu_cmn.h>
#ifdef WLMCNX
#include <wlc_mcnx.h>
#endif
#include <wlc_hw_priv.h>
#ifdef BCMLTECOEX
#include <wlc_ltecx.h>
#endif /* BCMLTECOEX */
#ifdef WLRSDB
#include <wlc_rsdb.h>
#endif /* WLRSDB */

static int wlc_btc_mode_set(wlc_info_t *wlc, int int_val);
static int wlc_btc_wire_set(wlc_info_t *wlc, int int_val);
static int wlc_btc_flags_idx_set(wlc_info_t *wlc, int int_val, int int_val2);
static int wlc_btc_flags_idx_get(wlc_info_t *wlc, int int_val);
static int wlc_btc_params_set(wlc_info_t *wlc, int int_val, int int_val2);
static int wlc_btc_params_get(wlc_info_t *wlc, int int_val);
static void wlc_btc_stuck_war50943(wlc_info_t *wlc, bool enable);
static void wlc_btc_rssi_threshold_get(wlc_info_t *wlc);
static int16 wlc_btc_siso_ack_get(wlc_info_t *wlc);
static int wlc_btc_wlc_up(void *ctx);
static int wlc_btc_wlc_down(void *ctx);
int wlc_btcx_desense(wlc_btc_info_t *btc, int band);
#ifdef WL_BTCDYN
static int wlc_btc_dynctl_profile_set(wlc_info_t *wlc, void *parambuf);
static int wlc_btc_dynctl_profile_get(wlc_info_t *wlc, void *resbuf);
static int wlc_btc_dynctl_status_get(wlc_info_t *wlc, void *resbuf);
static int wlc_btc_dynctl_sim_get(wlc_info_t *wlc, void *resbuf);
static int wlc_btc_dynctl_sim_set(wlc_info_t *wlc, void *parambuf);
#endif /* WL_BTCDYN */
static int8 wlc_btc_get_btrssi(wlc_btc_info_t *btc);
static void wlc_btc_reset_btrssi(wlc_btc_info_t *btc);

#if defined(STA) && defined(BTCX_PM0_IDLE_WAR)
static void wlc_btc_pm_adjust(wlc_info_t *wlc,  bool bt_active);
#endif
static int wlc_btc_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid, const char *name,
        void *params, uint p_len, void *arg, int len, int val_size, struct wlc_if *wlcif);
static void wlc_btcx_watchdog(void *arg);

#if defined(BCMDBG)
static int wlc_dump_btcx(wlc_info_t *wlc, struct bcmstrbuf *b);
static int wlc_clr_btcxdump(wlc_info_t *wlc);
#endif 

enum {
	IOV_BTC_MODE,           /* BT Coexistence mode */
	IOV_BTC_WIRE,           /* BTC number of wires */
	IOV_BTC_STUCK_WAR,       /* BTC stuck WAR */
	IOV_BTC_FLAGS,          /* BT Coex ucode flags */
	IOV_BTC_PARAMS,         /* BT Coex shared memory parameters */
	IOV_BTCX_CLEAR_DUMP,    /* clear btcx stats */
	IOV_BTC_SISO_ACK,       /* Specify SISO ACK antenna, disabled when 0 */
	IOV_BTC_RXGAIN_THRESH,  /* Specify restage rxgain thresholds */
	IOV_BTC_RXGAIN_FORCE,   /* Force for BTC restage rxgain */
	IOV_BTC_RXGAIN_LEVEL,   /* Set the BTC restage rxgain level */
#ifdef WL_BTCDYN
	IOV_BTC_DYNCTL,			/* get/set coex dynctl profile */
	IOV_BTC_DYNCTL_STATUS,  /* get dynctl status: dsns, btpwr, wlrssi, btc_mode, etc */
	IOV_BTC_DYNCTL_SIM,		/* en/dis & config dynctl algo simulation mode */
#endif
};

const bcm_iovar_t btc_iovars[] = {
	{"btc_mode", IOV_BTC_MODE, 0, IOVT_UINT32, 0},
	{"btc_stuck_war", IOV_BTC_STUCK_WAR, 0, IOVT_BOOL, 0 },
	{"btc_flags", IOV_BTC_FLAGS, (IOVF_SET_UP | IOVF_GET_UP), IOVT_BUFFER, 0 },
	{"btc_params", IOV_BTC_PARAMS, (IOVF_SET_UP | IOVF_GET_UP), IOVT_BUFFER, 0 },
#if defined(BCMDBG)
	{"btcx_clear_dump", IOV_BTCX_CLEAR_DUMP, (IOVF_SET_CLK), IOVT_VOID, 0 },
#endif
	{"btc_siso_ack", IOV_BTC_SISO_ACK, 0, IOVT_INT16, 0
	},
	{"btc_rxgain_thresh", IOV_BTC_RXGAIN_THRESH, 0, IOVT_UINT32, 0
	},
	{"btc_rxgain_force", IOV_BTC_RXGAIN_FORCE, 0, IOVT_UINT32, 0
	},
	{"btc_rxgain_level", IOV_BTC_RXGAIN_LEVEL, 0, IOVT_UINT32, 0
	},
#ifdef WL_BTCDYN
	/* set dynctl profile, get status , etc */
	{"btc_dynctl", IOV_BTC_DYNCTL, 0, IOVT_BUFFER, 0
	},
	/* set dynctl status */
	{"btc_dynctl_status", IOV_BTC_DYNCTL_STATUS, 0, IOVT_BUFFER, 0
	},
	/* enable & configure dynctl simulation mode (aka dryrun) */
	{"btc_dynctl_sim", IOV_BTC_DYNCTL_SIM, 0, IOVT_BUFFER, 0
	},
#endif
	{NULL, 0, 0, 0, 0}
};
#ifdef WL_BTCDYN
/*  btcdyn nvram variables to initialize the profile  */
static const char BCMATTACHDATA(rstr_btcdyn_flags)[] = "btcdyn_flags";
static const char BCMATTACHDATA(rstr_btcdyn_dflt_dsns_level)[] = "btcdyn_dflt_dsns_level";
static const char BCMATTACHDATA(rstr_btcdyn_low_dsns_level)[] = "btcdyn_low_dsns_level";
static const char BCMATTACHDATA(rstr_btcdyn_mid_dsns_level)[] = "btcdyn_mid_dsns_level";
static const char BCMATTACHDATA(rstr_btcdyn_high_dsns_level)[] = "btcdyn_high_dsns_level";
static const char BCMATTACHDATA(rstr_btcdyn_default_btc_mode)[] = "btcdyn_default_btc_mode";
static const char BCMATTACHDATA(rstr_btcdyn_msw_rows)[] = "btcdyn_msw_rows";
static const char BCMATTACHDATA(rstr_btcdyn_dsns_rows)[] = "btcdyn_dsns_rows";
static const char BCMATTACHDATA(rstr_btcdyn_msw_row0)[] = "btcdyn_msw_row0";
static const char BCMATTACHDATA(rstr_btcdyn_msw_row1)[] = "btcdyn_msw_row1";
static const char BCMATTACHDATA(rstr_btcdyn_msw_row2)[] = "btcdyn_msw_row2";
static const char BCMATTACHDATA(rstr_btcdyn_msw_row3)[] = "btcdyn_msw_row3";
static const char BCMATTACHDATA(rstr_btcdyn_dsns_row0)[] = "btcdyn_dsns_row0";
static const char BCMATTACHDATA(rstr_btcdyn_dsns_row1)[] = "btcdyn_dsns_row1";
static const char BCMATTACHDATA(rstr_btcdyn_dsns_row2)[] = "btcdyn_dsns_row2";
static const char BCMATTACHDATA(rstr_btcdyn_dsns_row3)[] = "btcdyn_dsns_row3";
static const char BCMATTACHDATA(rstr_btcdyn_btrssi_hyster)[] = "btcdyn_btrssi_hyster";
#endif /* WL_BTCDYN */

/* BT RSSI threshold for implict mode switching */
static const char BCMATTACHDATA(rstr_prot_btrssi_thresh)[] = "prot_btrssi_thresh";

#define BTC_BTRSSI_THRESH	-70 /* btrssi thresh for implibit mode switch */
/* actual btrssi = -1 * (btrssi_from_ucode * btrssi_step + btrssi_offset) */
#define BTC_BTRSSI_STEP		5
#define BTC_BTRSSI_OFFSET	10
#define BTC_BTRSSI_INVALID	0   /* invalid btrssi */
#define BTC_BTRSSI_SIZE		16  /* max number of btrssi samples for moving avg. */
#define BTC_BTRSSI_MIN_NUM	4   /* min number of btrssi samples for moving avg. */

#ifdef WL_BTCDYN
/* dynamic BTCOEX wl densense & coex mode switching */
/*
	MAC --> notify BTCOEX c band & chanspec has changed
	note: the code in this function needs to be as small as possible
	it is on realtime switching band/channel path
*/
void wlc_btcx_chspec_change_notify(wlc_info_t *wlc, chanspec_t chanspec, bool switchband)
{
#ifdef	DYNCTL_DBG
	chanspec_t old_chanspec = wlc->chanspec;
#endif /* DYNCTL_DBG */
	wlc_btc_info_t *btc = wlc->btch;
	bool bt_active;

	BTCDBG(("%s:wl%d: old chspec:0x%x new chspec:0x%x, bandsw:%d\n",
		__FUNCTION__, wlc->pub->unit,
		old_chanspec, chanspec, switchband));

	/*
	  note new bt activity detection uses wlc_btcx_get_btpwr() func call,
	  but to keep thE code small we'll use original Karthik's detection
	*/
	bt_active = btc->bth_active;

	if (btc->coex_two_ant && bt_active) {
		/*  BT is active & the board is 2 ant coex design *
		    WAR# we need to force wl to enforce spatioal policy for wl
		    to operate on core0 if wl is sitching to 5g channel
		*/
		 if (CHSPEC_IS5G(chanspec)) {
			/*	use coremask to override spatioal policy
				for ofdm & b rates -> single stream, core0 operation only
			*/
			BTCDBG(("BT is active, WL _/-> 5g, "
				"current txcore_mask[cck,ofdm,nsts1,nsts2\n]"
				":%0x,%0x,%0x,%0x\n",
				wlc->stf->txcore[0][1],
				wlc->stf->txcore[1][1],
				wlc->stf->txcore[2][1],
				wlc->stf->txcore[3][1]));

			wlc->stf->txcore_override[NSTS2_IDX] = 0x01;
			wlc->stf->txcore_override[NSTS1_IDX] = 0x01;
			wlc->stf->txcore_override[OFDM_IDX]  = 0x1;
			wlc->stf->txcore_override[CCK_IDX] = 0x1;

		 } else {
			/* new chan is 2g, reset txcore override */
			uint8 i;
			BTCDBG(("BT is active wl_/-> 2G, remove coremask override\n"));
			for (i = 0; i < MAX_CORE_IDX-1; i++) {
				wlc->stf->txcore_override[i] = 0;
			}
		 }
	}
}

/* extract bf, apply mask check if invalid */
static void btcx_extract_pwr(int16 btpwr, uint8 shft, int8 *pwr8)
{
	int8 tmp;
	*pwr8 = BT_INVALID_TX_PWR;

	if ((tmp = (btpwr >> shft) & SHM_BTC_MASK_TXPWR)) {
		*pwr8 = (tmp * BT_TX_PWR_STEP) + BT_TX_PWR_OFFSET;
	}
}


#ifdef DBG_BTPWR_HOLES
typedef struct pwrs {
	int8 pwr;
	uint32 ts;
} pwrs_t;
static void btcdyn_detect_btpwrhole(wlc_btc_info_t *btc, int8 cur_pwr)
{
	static pwrs_t pwr_smpl[4] = {{-127, 0}, {-127, 0}, {-127, 0}, {-127, 0}};
	static uint8 pwr_idx = 0;
	int32 cur_ts;

	cur_ts =  OSL_SYSUPTIME();
	pwr_smpl[pwr_idx].pwr = cur_pwr;
	pwr_smpl[pwr_idx].ts = cur_ts;

	/* detect a hole (an abnormality) in PWR sampling sequence */
	if ((pwr_smpl[pwr_idx].pwr != BT_INVALID_TX_PWR) &&
		(pwr_smpl[(pwr_idx-1) & 0x3].pwr == BT_INVALID_TX_PWR) &&
		(pwr_smpl[(pwr_idx-2) & 0x3].pwr != BT_INVALID_TX_PWR)) {

		DYNCTL_ERROR(("BTPWR hole at T-1:%d, delta from T-2:%d\n"
			" btpwr:[t, t-1, t-2]:%d,%d,%d\n", pwr_smpl[(pwr_idx-1) & 0x3].ts,
			(pwr_smpl[(pwr_idx-1) & 0x3].ts - pwr_smpl[(pwr_idx-2) & 0x3].ts),
			pwr_smpl[pwr_idx].pwr, pwr_smpl[(pwr_idx-1) & 0x3].pwr,
			pwr_smpl[(pwr_idx-2) & 0x3].pwr));

	}
	pwr_idx = (pwr_idx + 1) & 0x3;
}
#endif /* DBG_BTPWR_HOLES */
/*
	checks for BT power of each active task, converts to dBm and
	returns the highest power level if there is > 1 task detected.
*/
static int8 wlc_btcx_get_btpwr(wlc_btc_info_t *btc)
{
	wlc_info_t *wlc = btc->wlc;

	int8 pwr_sco, pwr_a2dp, pwr_sniff, pwr_acl;
	int8 result_pwr = BT_INVALID_TX_PWR;
	uint16 txpwr_shm;
	uint16 btcx_blk_ptr = wlc->hw->btc->bt_shm_addr;
	int8 pwr_tmp;

	/* read btpwr  */
	txpwr_shm = wlc_read_shm(wlc, btcx_blk_ptr + M_BTCX_BT_TXPWR);
	btc->btcdyn->bt_pwr_shm = txpwr_shm; /* keep a copy for dbg & status */

	/* clear the shm after read, ucode will refresh with a new value  */
	wlc_write_shm(wlc,  btcx_blk_ptr + M_BTCX_BT_TXPWR, 0);

	btcx_extract_pwr(txpwr_shm, SHM_BTC_SHFT_TXPWR_SCO, &pwr_sco);
	btcx_extract_pwr(txpwr_shm, SHM_BTC_SHFT_TXPWR_A2DP, &pwr_a2dp);
	btcx_extract_pwr(txpwr_shm, SHM_BTC_SHFT_TXPWR_SNIFF, &pwr_sniff);
	btcx_extract_pwr(txpwr_shm, SHM_BTC_SHFT_TXPWR_ACL, &pwr_acl);

	/*
	  although rare, both a2dp & sco may be active,
	  pick the highest one. if both are invalid, check sniff
	*/
	if (pwr_sco != BT_INVALID_TX_PWR ||
		pwr_a2dp != BT_INVALID_TX_PWR) {

		BTCDBG(("%s: shmem_val:%x, BT tasks pwr: SCO:%d, A2DP:%d, SNIFF:%d\n",
			__FUNCTION__, txpwr_shm, pwr_sco, pwr_a2dp, pwr_sniff));

		result_pwr = pwr_sco;
		if (pwr_a2dp > pwr_sco)
			result_pwr = pwr_a2dp;

	} else if (pwr_acl != BT_INVALID_TX_PWR) {
		result_pwr = pwr_acl;
	} else if (pwr_sniff != BT_INVALID_TX_PWR) {
		result_pwr = pwr_sniff;
	}

#ifdef DBG_BTPWR_HOLES
	btcdyn_detect_btpwrhole(btc, result_pwr);
#endif

	/* protect from single invalid pwr reading ("pwr hole") */
	if (result_pwr == BT_INVALID_TX_PWR) {
		BTCDBG(("cur btpwr invalid, use prev value:%d\n",
			btc->prev_btpwr));
		pwr_tmp = btc->btcdyn->prev_btpwr;
	} else {
		pwr_tmp = result_pwr;
	}

	btc->btcdyn->prev_btpwr = result_pwr;
	result_pwr = pwr_tmp;
	return result_pwr;
}

/*
	At a given BT TX PWR level (typically > 7dbm)
	there is a certain WL RSSI level range at which WL performance in Hybrid
	COEX mode is actually lower than in Full TDM.
	The algorithm below selects the right mode based on tabulated data points.
	The profile table is specific for each board and needs to be calibrated
	for every new board + BT+WIFI antenna design.
*/
static uint8 btcx_dflt_mode_switch(wlc_info_t *wlc, int8 wl_rssi, int8 bt_pwr, int8 bt_rssi)
{
	wlc_btc_info_t *btc = wlc->btch;
	dctl_prof_t *profile = btc->btcdyn->dprof;
	uint8 row, new_mode;

	new_mode = profile->default_btc_mode;

	/* no active BT task when bt_pwr is invalid */
	if	(bt_pwr == BT_INVALID_TX_PWR) {
		return new_mode;
	}

	/* keep current coex mode  if: */
	if ((btc->bt_rssi == BTC_BTRSSI_INVALID) ||
		(btc->btcdyn->wl_rssi == WLC_RSSI_INVALID)) {
		return btc->mode;
	}

	for (row = 0; row < profile->msw_rows; row++) {
		if ((bt_pwr >= profile->msw_data[row].bt_pwr) &&
			(bt_rssi < profile->msw_data[row].bt_rssi +
			btc->btcdyn->msw_btrssi_hyster) &&
			(wl_rssi > profile->msw_data[row].wl_rssi_low) &&
			(wl_rssi < profile->msw_data[row].wl_rssi_high)) {
			/* new1: fallback mode is now per {btpwr + bt_rssi + wl_rssi range} */
				new_mode = profile->msw_data[row].mode;
			break;
		}
	}

	if (new_mode != profile->default_btc_mode) {
		/* the new mode is a downgrade from the default one,
		 for wl & bt signal conditions have deteriorated.
		 Apply hysteresis to stay in this mode until
		 the conditions get better by >= hyster values
		*/
		/*  positive for bt rssi  */
		btc->btcdyn->msw_btrssi_hyster = profile->msw_btrssi_hyster;
	} else {
		/* in or has switched to default, turn off h offsets */
		btc->btcdyn->msw_btrssi_hyster = 0;
	}

	return new_mode;
}

/*
* calculates new desense level using
* current btcmode, bt_pwr, wl_rssi,* and board profile data points
*/
static uint8 btcx_dflt_get_desense_level(wlc_info_t *wlc, int8 wl_rssi, int8 bt_pwr, int8 bt_rssi)
{
	wlc_btc_info_t *btc = wlc->btch;
	dctl_prof_t *profile = btc->btcdyn->dprof;
	uint8 row, new_level;

	new_level = profile->dflt_dsns_level;

	/* BT "No tasks" -> use default desense */
	if	(bt_pwr == BT_INVALID_TX_PWR) {
		return new_level;
	}

	/*  keep current desense level if: */
	if ((btc->bt_rssi == BTC_BTRSSI_INVALID) ||
		(btc->btcdyn->wl_rssi == WLC_RSSI_INVALID)) {
		return btc->btcdyn->cur_dsns;
	}

	for (row = 0; row < profile->dsns_rows; row++) {
		if (btc->mode == profile->dsns_data[row].mode &&
			bt_pwr >= profile->dsns_data[row].bt_pwr) {

			if (wl_rssi > profile->dsns_data[row].wl_rssi_high) {
				new_level = profile->high_dsns_level;
			}
			else if (wl_rssi > profile->dsns_data[row].wl_rssi_low) {

				new_level = profile->mid_dsns_level;
			} else {
				new_level = profile->low_dsns_level;
			}
			break;
		}
	}
	return  new_level;
}

/* set external desense handler */
int btcx_set_ext_desense_calc(wlc_info_t *wlc, btcx_dynctl_calc_t fn)
{
	wlc_btcdyn_info_t *btcdyn = wlc->btch->btcdyn;

	ASSERT(fn);
	btcdyn->desense_fn = fn;
	return BCME_OK;
}

/* set external mode switch handler */
int btcx_set_ext_mswitch_calc(wlc_info_t *wlc, btcx_dynctl_calc_t fn)
{
	wlc_btcdyn_info_t *btcdyn = wlc->btch->btcdyn;

	ASSERT(fn);
	btcdyn->mswitch_fn = fn;
	return BCME_OK;
}

/*  wrapper for btcx code portability */
static int8 btcx_get_wl_rssi(wlc_btc_info_t *btc)
{
	return btc->wlc->cfg->link->rssi;
}
/*
	dynamic COEX CTL (desense & switching) called from btc_wtacdog()
*/
static void wlc_btcx_dynctl(wlc_btc_info_t *btc)
{
	wlc_info_t *wlc = btc->wlc;
	uint8 btc_mode = wlc->btch->mode;
	dctl_prof_t *ctl_prof = btc->btcdyn->dprof;
	uint16 btcx_blk_ptr = wlc->hw->btc->bt_shm_addr;
	uint32 cur_ts = OSL_SYSUPTIME();

	/* protection against too frequent calls from btc watchdog context */
	if ((cur_ts - btc->btcdyn->prev_btpwr_ts) < DYNCTL_MIN_PERIOD) {
		btc->btcdyn->prev_btpwr_ts = cur_ts;
		return;
	}
	btc->btcdyn->prev_btpwr_ts = cur_ts;

	btc->btcdyn->bt_pwr = wlc_btcx_get_btpwr(btc);
	btc->btcdyn->wl_rssi = btcx_get_wl_rssi(btc);
	btc->bt_rssi = wlc_btc_get_btrssi(btc);

	if (btc->dynctl_sim_on) {
	/* simulation mode is on  */
		btc->bt_pwr = btc->sim_btpwr;
		btc->wl_rssi = btc->sim_wlrssi;
		btc->bt_rssi = btc->sim_btrssi;
	}

#ifdef BCMLTECOEX
	/* No mode switch is required if ltecx is ON */
	if (IS_MSWITCH_ON(ctl_prof) && !wlc_ltecx_get_lte_status(wlc)) {
	/* 1st check if we need to switch the btc_mode */
#else
	if (IS_MSWITCH_ON(ctl_prof)) {
#endif /* BCMLTECOEX */
		uint8 new_mode;

		ASSERT(btc->btcdyn->mswitch_fn);
		new_mode = btc->btcdyn->mswitch_fn(wlc, btc->btcdyn->wl_rssi,
			btc->btcdyn->bt_pwr, btc->bt_rssi);

		if (new_mode != btc_mode) {
			wlc_btc_mode_set(wlc, new_mode);

			BTCDBG(("%s mswitch mode:%d -> mode:%d,"
				" bt_pwr:%d, wl_rssi:%d, cur_dsns:%d, BT hstr[rssi:%d]\n",
				__FUNCTION__, btc_mode, new_mode, btc->btcdyn->bt_pwr,
				btc->btcdyn->wl_rssi, btc->btcdyn->cur_dsns,
				ctl_prof->msw_btrssi_hyster));

			if ((wlc->hw->boardflags & BFL_FEM_BT) == 0 && /* dLNA chip */
				btcx_blk_ptr != 0) {
				/* update btcx_host_flags  based on btc_mode */
				if (new_mode == WL_BTC_FULLTDM) {
					wlc_bmac_update_shm(wlc->hw,
						btcx_blk_ptr + M_BTCX_HOST_FLAGS,
						BTCX_HFLG_DLNA_TDM_VAL, BTCX_HFLG_DLNA_MASK);
				} else {
					/* mainly for hybrid and parallel */
					wlc_bmac_update_shm(wlc->hw,
						btcx_blk_ptr + M_BTCX_HOST_FLAGS,
						BTCX_HFLG_DLNA_DFLT_VAL, BTCX_HFLG_DLNA_MASK);
				}
			}
		}
	}

	/* enable protection after mode switching */
	wlc_btc_set_ps_protection(wlc, wlc->cfg); /* enable */

	/* check if we need to switch the desense level */
	if (IS_DESENSE_ON(ctl_prof)) {
		uint8 new_level;

		ASSERT(btc->btcdyn->desense_fn);
		new_level = btc->btcdyn->desense_fn(wlc, btc->btcdyn->wl_rssi,
			btc->btcdyn->bt_pwr, btc->bt_rssi);

		if (new_level != btc->cur_dsns) {
			/* apply new desense level */
			if ((wlc_iovar_setint(wlc, "phy_btc_restage_rxgain",
				new_level)) == BCME_OK) {

				BTCDBG(("%s: set new desense:%d, prev was:%d btcmode:%d,"
					" bt_pwr:%d, wl_rssi:%d\n",
					__FUNCTION__, new_level, btc->btcdyn->cur_dsns, btc->mode,
					btc->btcdyn->bt_pwr, btc->btcdyn->wl_rssi));

				btc->btcdyn->cur_dsns = new_level;
			} else
				WL_ERROR(("%s desense apply error\n",
					__FUNCTION__));
		}
	}
}

static int wlc_btc_dynctl_profile_set(wlc_info_t *wlc, void *parambuf)
{
	wlc_btcdyn_info_t *btcdyn = wlc->btch->btcdyn;
	bcopy(parambuf, btcdyn->dprof, sizeof(dctl_prof_t));
	return BCME_OK;
}

static int wlc_btc_dynctl_profile_get(wlc_info_t *wlc, void *resbuf)
{
	wlc_btcdyn_info_t *btcdyn = wlc->btch->btcdyn;
	bcopy(btcdyn->dprof, resbuf, sizeof(dctl_prof_t));
	return BCME_OK;
}

/* get dynctl status iovar handler */
static int wlc_btc_dynctl_status_get(wlc_info_t *wlc, void *resbuf)
{
	wlc_btc_info_t *btc = wlc->btch;
	dynctl_status_t dynctl_status;

	/* agg. stats into local stats var */
	dynctl_status.sim_on = btc->btcdyn->dynctl_sim_on;
	dynctl_status.bt_pwr_shm  = btc->btcdyn->bt_pwr_shm;
	dynctl_status.bt_pwr = btc->btcdyn->bt_pwr;
	dynctl_status.bt_rssi = btc->bt_rssi;
	dynctl_status.wl_rssi = btc->btcdyn->wl_rssi;
	dynctl_status.dsns_level = btc->btcdyn->cur_dsns;
	dynctl_status.btc_mode = btc->mode;

	/* return it */
	bcopy(&dynctl_status, resbuf, sizeof(dynctl_status_t));
	return BCME_OK;
}

/*   get dynctl sim parameters */
static int wlc_btc_dynctl_sim_get(wlc_info_t *wlc, void *resbuf)
{
	dynctl_sim_t sim;
	wlc_btcdyn_info_t *btcdyn = wlc->btch->btcdyn;

	sim.sim_on = btcdyn->dynctl_sim_on;
	sim.btpwr = btcdyn->sim_btpwr;
	sim.btrssi = btcdyn->sim_btrssi;
	sim.wlrssi = btcdyn->sim_wlrssi;

	bcopy(&sim, resbuf, sizeof(dynctl_sim_t));
	return BCME_OK;
}

/*   set dynctl sim parameters */
static int wlc_btc_dynctl_sim_set(wlc_info_t *wlc, void *parambuf)
{
	dynctl_sim_t sim;

	wlc_btcdyn_info_t *btcdyn = wlc->btch->btcdyn;
	bcopy(parambuf, &sim, sizeof(dynctl_sim_t));

	btcdyn->dynctl_sim_on = sim.sim_on;
	btcdyn->sim_btpwr = sim.btpwr;
	btcdyn->sim_btrssi = sim.btrssi;
	btcdyn->sim_wlrssi = sim.wlrssi;

	return BCME_OK;
}

/*
* initialize one row btc_thr_data_t * from a named nvram var
*/
static int
BCMATTACHFN(wlc_btc_dynctl_init_trow)(wlc_btc_info_t *btc,
	btc_thr_data_t *trow, const char *varname, uint16 xpected_sz)
{
	wlc_info_t *wlc = btc->wlc;
	wlc_hw_info_t *wlc_hw = wlc->hw;
	uint16 j = 0;

	/* read mode switching table */
	int array_sz = getintvararraysize(wlc_hw->vars, varname);
	if (!array_sz)
		return 0; /* var is not present */

	/* mk sure num of items in the var is OK */
	if (array_sz != xpected_sz)
		return -1;

	trow->mode = getintvararray(wlc_hw->vars, varname, j++);
	trow->bt_pwr = getintvararray(wlc_hw->vars, varname, j++);
	trow->bt_rssi = getintvararray(wlc_hw->vars, varname, j++);
	trow->wl_rssi_high = getintvararray(wlc_hw->vars, varname, j++);
	trow->wl_rssi_low = getintvararray(wlc_hw->vars, varname, j++);
	return 1;
}
#endif /* WL_BTCDYN */

/* Read bt rssi from shm and do moving average */
static int8
wlc_btc_get_btrssi(wlc_btc_info_t *btc)
{
	wlc_info_t *wlc = btc->wlc;
	uint16 btrssi_shm, btcx_blk_ptr;
	int16 btrssi_avg = 0;

	/* get btc block base addr in shm  */
	btcx_blk_ptr = btc->bt_shm_addr;
	if (!btc->bth_active || btcx_blk_ptr == 0) {
		return BTC_BTRSSI_INVALID;
	}

	/* read btrssi idx from shm */
	btrssi_shm = wlc_bmac_read_shm(wlc->hw, btcx_blk_ptr + M_BTCX_RSSI);

	if (btrssi_shm) {
		int8 cur_rssi, old_rssi;
		/* clear shm because ucode keeps max btrssi idx */
		wlc_bmac_write_shm(wlc->hw, btcx_blk_ptr + M_BTCX_RSSI, 0);

		/* actual bt rssi = -1 x (btrssi_shm x 5 + 10) */
		cur_rssi = (-1) * (int8)(btrssi_shm * BTC_BTRSSI_STEP + BTC_BTRSSI_OFFSET);

		/* after BT on let accumulate > BTC_BTRSSI_SIZE */
		if (btc->btrssi_cnt < BTC_BTRSSI_SIZE)
			btc->btrssi_cnt++;

		/* accumulate & calc moving average */
		old_rssi = btc->btrssi[btc->btrssi_idx];
		btc->btrssi[btc->btrssi_idx] = cur_rssi;
		/* sum = -old one, +new  */
		btc->btrssi_sum = btc->btrssi_sum - old_rssi + cur_rssi;
		ASSERT(btc->btrssi_cnt);
		btrssi_avg = btc->btrssi_sum / btc->btrssi_cnt;

		btc->btrssi_idx = MODINC_POW2(btc->btrssi_idx, BTC_BTRSSI_SIZE);
	}

	if (btc->btrssi_cnt < BTC_BTRSSI_MIN_NUM) {
		return BTC_BTRSSI_INVALID;
	}

	return (int8)btrssi_avg;
}

static void
wlc_btc_reset_btrssi(wlc_btc_info_t *btc)
{
	memset(btc->btrssi, 0, sizeof(int8)*BTC_BTRSSI_SIZE);
	btc->btrssi_cnt = 0;
	btc->btrssi_idx = 0;
	btc->btrssi_sum = 0;
}

/* This includes the auto generated ROM IOCTL/IOVAR patch handler C source file (if auto patching is
 * enabled). It must be included after the prototypes and declarations above (since the generated
 * source file may reference private constants, types, variables, and functions).
 */
#include <wlc_patch.h>


wlc_btc_info_t *
BCMATTACHFN(wlc_btc_attach)(wlc_info_t *wlc)
{
	wlc_btc_info_t *btc;
#ifdef WLBTCPROF
	wlc_btc_profile_t *select_profile;
#endif /* WLBTCPROF */
#if defined(WLC_LOW) || defined(WL_BTCDYN)
	wlc_hw_info_t *wlc_hw = wlc->hw;
#endif

	if ((btc = (wlc_btc_info_t*)
		MALLOCZ(wlc->osh, sizeof(wlc_btc_info_t))) == NULL) {
		WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
			wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		goto fail;
	}
	btc->wlc = wlc;

	/* register module */
	if (wlc_module_register(wlc->pub, btc_iovars, "btc", btc, wlc_btc_doiovar,
		wlc_btcx_watchdog, wlc_btc_wlc_up, wlc_btc_wlc_down) != BCME_OK) {
		WL_ERROR(("wl%d: %s: btc register err\n", wlc->pub->unit, __FUNCTION__));
		goto fail;
	}

#if defined(BCMDBG)
	/* register dump stats for btcx */
	wlc_dump_register(wlc->pub, "btcx", (dump_fn_t)wlc_dump_btcx, (void *)wlc);
#endif 

#ifdef WLBTCPROF
	if ((btc->btc_prev_connect = (wlc_btc_prev_connect_t *)
		MALLOCZ(wlc->osh, sizeof(wlc_btc_prev_connect_t))) == NULL) {
		WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
			wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		goto fail;
	}

	if ((btc->btc_profile = (wlc_btc_select_profile_t *)
		MALLOCZ(wlc->osh, sizeof(wlc_btc_select_profile_t) * BTC_SUPPORT_BANDS)) == NULL) {
		WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
			wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		goto fail;
	}

	bzero(btc->btc_prev_connect, sizeof(wlc_btc_prev_connect_t));
	bzero(btc->btc_profile, sizeof(wlc_btc_profile_t) * BTC_SUPPORT_BANDS);

	memset(&btc->btc_prev_connect->prev_2G_profile, 0, sizeof(struct wlc_btc_profile));
	memset(&btc->btc_prev_connect->prev_5G_profile, 0, sizeof(struct wlc_btc_profile));

	btc->btc_prev_connect->prev_band = WLC_BAND_ALL;

	select_profile = &btc->btc_profile[BTC_PROFILE_2G].select_profile;
	select_profile->btc_wlrssi_thresh = BTC_WL_RSSI_DEFAULT;
	select_profile->btc_btrssi_thresh = BTC_BT_RSSI_DEFAULT;
	if (CHIPID(wlc->pub->sih->chip) == BCM4350_CHIP_ID) {
		select_profile->btc_num_desense_levels = MAX_BT_DESENSE_LEVELS_4350;
		select_profile->btc_wlrssi_hyst = BTC_WL_RSSI_HYST_DEFAULT_4350;
		select_profile->btc_btrssi_hyst = BTC_BT_RSSI_HYST_DEFAULT_4350;
		select_profile->btc_max_siso_resp_power[0] =
			BTC_WL_MAX_SISO_RESP_POWER_TDD_DEFAULT_4350;
		select_profile->btc_max_siso_resp_power[1] =
			BTC_WL_MAX_SISO_RESP_POWER_HYBRID_DEFAULT_4350;
	}
#endif /* WLBTCPROF */

	btc->siso_ack_ovr = FALSE;

#ifdef WL_BTCDYN
	if ((btc->btcdyn = (wlc_btcdyn_info_t *)
			MALLOCZ(wlc->osh, sizeof(wlc_btcdyn_info_t))) == NULL) {
			WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
				wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
			goto fail;
	}
	if ((btc->btcdyn->dprof = MALLOCZ(wlc->osh, sizeof(dctl_prof_t))) != NULL) {

		/*  default desnense & mode switching tables (2 rows each) */
		btc_thr_data_t dflt_msw_data[2] = {
			{1, 12, -73, -30, -90},
			{1, 8, -73, -30, -60},
		};
		/*  default desense datatable  */
		btc_thr_data_t dflt_dsns_data[2] = {
			{5, 4, 0, -55, -65},
			{5, -16, 0, -50, -65}
		};

		/* dynctl profile struct ver */
		btc->btcdyn->dprof->version = DCTL_PROFILE_VER;
		/* default WL desense & mode switch handlers  */
		btc->btcdyn->desense_fn = btcx_dflt_get_desense_level;
		btc->btcdyn->mswitch_fn = btcx_dflt_mode_switch;
		btc->bt_rssi = BTC_BTRSSI_INVALID;
		btc->btcdyn->dprof->msw_btrssi_hyster = BTCDYN_DFLT_BTRSSI_HYSTER;

		/*
		 * try loading btcdyn profile from nvram,
		 * use "btcdyn_flags" var as a presense indication
		 */
		if (getvar(wlc_hw->vars, rstr_btcdyn_flags) != NULL) {

			uint16 i;

			/* read int params 1st */
			btc->btcdyn->dprof->flags =	getintvar(wlc_hw->vars, rstr_btcdyn_flags);
			btc->btcdyn->dprof->dflt_dsns_level =
				getintvar(wlc_hw->vars, rstr_btcdyn_dflt_dsns_level);
			btc->btcdyn->dprof->low_dsns_level =
				getintvar(wlc_hw->vars, rstr_btcdyn_low_dsns_level);
			btc->btcdyn->dprof->mid_dsns_level =
				getintvar(wlc_hw->vars, rstr_btcdyn_mid_dsns_level);
			btc->btcdyn->dprof->high_dsns_level =
				getintvar(wlc_hw->vars, rstr_btcdyn_high_dsns_level);
			btc->btcdyn->dprof->default_btc_mode =
				getintvar(wlc_hw->vars, rstr_btcdyn_default_btc_mode);
			btc->btcdyn->dprof->msw_btrssi_hyster =
				getintvar(wlc_hw->vars, rstr_btcdyn_btrssi_hyster);

			/* these two are used for data array sz check */
			btc->btcdyn->dprof->msw_rows =
				getintvar(wlc_hw->vars, rstr_btcdyn_msw_rows);
			btc->btcdyn->dprof->dsns_rows =
				getintvar(wlc_hw->vars, rstr_btcdyn_dsns_rows);

			/* sanity check on btcdyn nvram table sz */
			if ((btc->btcdyn->dprof->msw_rows > DCTL_TROWS_MAX) ||
				(((btc->btcdyn->dprof->flags & DCTL_FLAGS_MSWITCH) == 0) !=
				(btc->btcdyn->dprof->msw_rows == 0))) {
				BTCDBG(("btcdyn invalid mode switch config\n"));
				goto rst2_dflt;
			}
			if ((btc->btcdyn->dprof->dsns_rows > DCTL_TROWS_MAX) ||
				(((btc->btcdyn->dprof->flags & DCTL_FLAGS_DESENSE) == 0) !=
				(btc->btcdyn->dprof->dsns_rows == 0))) {
				BTCDBG(("btcdyn invalid dynamic desense config\n"));
				goto rst2_dflt;
			}

			/*  initialize up to 4 rows in msw table */
			i = wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->msw_data[0],
				rstr_btcdyn_msw_row0, sizeof(btc_thr_data_t));
			i += wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->msw_data[1],
				rstr_btcdyn_msw_row1, sizeof(btc_thr_data_t));
			i += wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->msw_data[2],
				rstr_btcdyn_msw_row2, sizeof(btc_thr_data_t));
			i += wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->msw_data[3],
				rstr_btcdyn_msw_row3, sizeof(btc_thr_data_t));

			/* number of initialized table rows must match to specified in nvram */
			if (i != btc->btcdyn->dprof->msw_rows)
				BTCDBG(("btcdyn incorrect nr of mode switch rows (%d)\n", i));
				goto rst2_dflt;
			}

			/*  initialize up to 4 rows in desense sw table */
			i = wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->dsns_data[0],
				rstr_btcdyn_dsns_row0, sizeof(btc_thr_data_t));
			i += wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->dsns_data[1],
				rstr_btcdyn_dsns_row1, sizeof(btc_thr_data_t));
			i += wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->dsns_data[2],
				rstr_btcdyn_dsns_row2, sizeof(btc_thr_data_t));
			i += wlc_btc_dynctl_init_trow(btc, &btc->btcdyn->dprof->dsns_data[3],
				rstr_btcdyn_dsns_row3, sizeof(btc_thr_data_t));

			/* number of initialized table rows must match to specified in nvram */
			if (i != btc->btcdyn->dprof->dsns_rows)
				BTCDBG(("btcdyn incorrect nr of dynamic desense rows (%d)\n", i));
				goto rst2_dflt;
			}

			BTCDBG(("btcdyn profile has been loaded from nvram - Ok\n"));
		} else {
			rst2_dflt:
			WL_ERROR(("wl%d: %s: nvram.txt: missing or bad btcdyn profile vars."
				" do init from default\n", wlc->pub->unit, __FUNCTION__));

			/* enable both dyn desense & mode switching */
			btc->btcdyn->dprof->flags = (DCTL_FLAGS_DYNCTL |
				DCTL_FLAGS_DESENSE |
				DCTL_FLAGS_MSWITCH);
			/* initialize default profile */
			btc->btcdyn->dprof->dflt_dsns_level = DESENSE_OFF;
			btc->btcdyn->dprof->low_dsns_level = DESENSE_OFF;
			btc->btcdyn->dprof->mid_dsns_level = DFLT_DESENSE_MID;
			btc->btcdyn->dprof->high_dsns_level = DFLT_DESENSE_HIGH;
			btc->btcdyn->dprof->default_btc_mode = WL_BTC_HYBRID;
			btc->btcdyn->dprof->msw_rows = DCTL_TROWS;
			btc->btcdyn->dprof->dsns_rows = DCTL_TROWS;
			btc->btcdyn->sim_btpwr = BT_INVALID_TX_PWR;
			btc->btcdyn->sim_wlrssi = WLC_RSSI_INVALID;

			/*  sanity check for the table sizes */
			ASSERT(sizeof(dflt_msw_data) <=
				(DCTL_TROWS_MAX * sizeof(btc_thr_data_t)));
			bcopy(dflt_msw_data, btc->btcdyn->dprof->msw_data, sizeof(dflt_msw_data));
			ASSERT(sizeof(dflt_dsns_data) <=
				(DCTL_TROWS_MAX * sizeof(btc_thr_data_t)));
			bcopy(dflt_dsns_data,
				btc->btcdyn->dprof->dsns_data, sizeof(dflt_dsns_data));
		}
		/* set btc_mode to default value */
		wlc_hw->btc->mode = btc->dprof->default_btc_mode;
	} else {
		WL_ERROR(("wl%d: %s: MALLOC for btc->btcdyn->dprof failed, %d bytes\n",
			wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
			goto fail;
	}
#endif /* WL_BTCDYN */

	if ((btc->btrssi = (int8*)MALLOCZ(wlc->osh, sizeof(int8) * BTC_BTRSSI_SIZE)) == NULL) {
		WL_ERROR(("wl%d: %s: MALLOC for btrssi failed, %d bytes\n",
		    wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		goto fail;
	}
	btc->btrssi_cnt = 0;
	btc->btrssi_idx = 0;
	btc->btrssi_sum = 0;
#ifdef WLC_LOW /* avoid WIN32 NIC driver compile error */
	if (getvar(wlc_hw->vars, rstr_prot_btrssi_thresh) != NULL) {
		btc->prot_btrssi_thresh =
			(int8)getintvar(wlc_hw->vars, rstr_prot_btrssi_thresh);
	} else {
		btc->prot_btrssi_thresh = BTC_BTRSSI_THRESH;
	}
#else
	btc->prot_btrssi_thresh = BTC_BTRSSI_THRESH;
#endif

	btc->dyagg = AUTO;
	return btc;

	/* error handling */
fail:
	wlc_btc_detach(btc);
	return NULL;
}

void
BCMATTACHFN(wlc_btc_detach)(wlc_btc_info_t *btc)
{
	wlc_info_t *wlc;

	if (btc == NULL)
		return;

	wlc = btc->wlc;
	wlc_module_unregister(wlc->pub, "btc", btc);

#ifdef WL_BTCDYN
	if (btc->btcdyn->dprof)
		MFREE(wlc->osh, btc->btcdyn->dprof, sizeof(dctl_prof_t));
	btc->btcdyn->dprof = NULL;
	MFREE(wlc->osh, btc->btcdyn, sizeof(wlc_btcdyn_info_t));
#endif /* WL_BTCDYN */

#ifdef WLBTCPROF
	MFREE(wlc->osh, btc->btc_prev_connect, sizeof(wlc_btc_prev_connect_t));
	MFREE(wlc->osh, btc->btc_profile, sizeof(wlc_btc_select_profile_t));
#endif /* WLBTCPROF */
	MFREE(wlc->osh, btc->btrssi, sizeof(int8) * BTC_BTRSSI_SIZE);
	MFREE(wlc->osh, btc, sizeof(wlc_btc_info_t));
}

/* BTCX Wl up callback */
static int
wlc_btc_wlc_up(void *ctx)
{
	wlc_btc_info_t *btc = (wlc_btc_info_t *)ctx;
	wlc_info_t *wlc = btc->wlc;

	if (!btc->bt_shm_addr)
		btc->bt_shm_addr = 2 * wlc_bmac_read_shm(wlc->hw, M_BTCX_BLK_PTR);

	if (AIBSS_ENAB(wlc->pub)) {
		/* Enable dynamic Tx aggregation in AIBSS mode */
		wlc_btc_hflg(wlc, ON, BTCX_HFLG_DYAGG);
	}
	return BCME_OK;
}

/* BTCX Wl down callback */
static int
wlc_btc_wlc_down(void *ctx)
{
	wlc_btc_info_t *btc = (wlc_btc_info_t *)ctx;
	wlc_info_t *wlc = btc->wlc;

	btc->bt_shm_addr = 0;
	if (AIBSS_ENAB(wlc->pub)) {
		/* Disable dynamic Tx aggregation if AIBSS goes down */
		wlc_btc_hflg(wlc, OFF, BTCX_HFLG_DYAGG);
	}
	return BCME_OK;
}

static int
wlc_btc_mode_set(wlc_info_t *wlc, int int_val)
{
	int err = wlc_bmac_btc_mode_set(wlc->hw, int_val);
	wlc->btch->mode = wlc_bmac_btc_mode_get(wlc->hw);
	return err;
}

int
wlc_btc_mode_get(wlc_info_t *wlc)
{
	return wlc->btch->mode;
}

int
wlc_btcx_desense(wlc_btc_info_t *btc, int band)
{
	int i;
	wlc_info_t *wlc = (wlc_info_t *)btc->wlc;
	int btc_mode = wlc_btc_mode_get(wlc);

	/* Dynamic restaging of rxgain for BTCoex */
	if (!SCAN_IN_PROGRESS(wlc->scan) &&
	    btc->bth_active &&
	    (wlc->cfg->link->rssi != WLC_RSSI_INVALID)) {
		if (!btc->restage_rxgain_active &&
		    ((BAND_5G(band) &&
		      ((btc->restage_rxgain_force &
			BTC_RXGAIN_FORCE_5G_MASK) == BTC_RXGAIN_FORCE_5G_ON)) ||
		     (BAND_2G(band) &&
		      ((btc->restage_rxgain_force &
			BTC_RXGAIN_FORCE_2G_MASK) == BTC_RXGAIN_FORCE_2G_ON) &&
		      (!btc->restage_rxgain_on_rssi_thresh ||
		       (btc_mode == WL_BTC_DISABLE) ||
		       (btc->restage_rxgain_on_rssi_thresh &&
		       ((btc_mode == WL_BTC_HYBRID) || (btc_mode == WL_BTC_FULLTDM)) &&
			(-wlc->cfg->link->rssi < btc->restage_rxgain_on_rssi_thresh)))))) {
			if ((i = wlc_iovar_setint(wlc, "phy_btc_restage_rxgain",
				btc->restage_rxgain_level)) == BCME_OK) {
				btc->restage_rxgain_active = 1;
			}
			WL_BTCPROF(("wl%d: BTC restage rxgain (%x) ON: RSSI %d "
				"Thresh -%d, bt %d, (err %d)\n",
				wlc->pub->unit, wlc->stf->rxchain, wlc->cfg->link->rssi,
				btc->restage_rxgain_on_rssi_thresh,
				btc->bth_active, i));
		}
		else if (btc->restage_rxgain_active &&
			((BAND_5G(band) &&
			((btc->restage_rxgain_force &
			BTC_RXGAIN_FORCE_5G_MASK) == BTC_RXGAIN_FORCE_OFF)) ||
			(BAND_2G(band) &&
			(((btc->restage_rxgain_force &
			BTC_RXGAIN_FORCE_2G_MASK) == BTC_RXGAIN_FORCE_OFF) ||
			(btc->restage_rxgain_off_rssi_thresh &&
			((btc_mode == WL_BTC_HYBRID) || (btc_mode == WL_BTC_FULLTDM)) &&
			(-wlc->cfg->link->rssi > btc->restage_rxgain_off_rssi_thresh)))))) {
			  if ((i = wlc_iovar_setint(wlc, "phy_btc_restage_rxgain", 0)) == BCME_OK) {
				btc->restage_rxgain_active = 0;
			  }
			  WL_BTCPROF(("wl%d: BTC restage rxgain (%x) OFF: RSSI %d "
				"Thresh -%d, bt %d, (err %d)\n",
				wlc->pub->unit, wlc->stf->rxchain, wlc->cfg->link->rssi,
				btc->restage_rxgain_off_rssi_thresh,
				btc->bth_active, i));
		}
	} else if (btc->restage_rxgain_active) {
		if ((i = wlc_iovar_setint(wlc, "phy_btc_restage_rxgain", 0)) == BCME_OK) {
			btc->restage_rxgain_active = 0;
		}
		WL_BTCPROF(("wl%d: BTC restage rxgain (%x) OFF: RSSI %d bt %d (err %d)\n",
			wlc->pub->unit, wlc->stf->rxchain, wlc->cfg->link->rssi,
			btc->bth_active, i));
	}

	return BCME_OK;
}

#ifdef WLBTCPROF
#ifdef WLBTCPROF_EXT
int
wlc_btcx_set_ext_profile_param(wlc_info_t *wlc)
{
	wlc_btc_info_t *btch = wlc->btch;
	int err = BCME_ERROR;
	wlc_btc_profile_t *select_profile;

	select_profile = &btch->btc_profile[BTC_PROFILE_2G].select_profile;
	/* program bt and wlan threshold and hysteresis data */
	btch->prot_btrssi_thresh = -1 * select_profile->btc_btrssi_thresh;
	btch->btrssi_hyst = select_profile->btc_btrssi_hyst;
	btch->prot_rssi_thresh = -1 * select_profile->btc_wlrssi_thresh;
	btch->wlrssi_hyst = select_profile->btc_wlrssi_hyst;
	if (wlc->clk) {
		wlc_btc_set_ps_protection(wlc, wlc->cfg); /* enable */

		if (select_profile->btc_num_desense_levels == MAX_BT_DESENSE_LEVELS_4350) {
			err = wlc_phy_btc_set_max_siso_resp_pwr(WLC_PI(wlc),
				&select_profile->btc_max_siso_resp_power[0],
				MAX_BT_DESENSE_LEVELS_4350);
		}
	}

	return err;
}
#endif /* WLBTCPROF_EXT */
int
wlc_btcx_set_btc_profile_param(struct wlc_info *wlc, chanspec_t chanspec, bool force)
{
	int err = BCME_OK;
	wlc_btc_profile_t *btc_cur_profile, *btc_prev_profile;
	wlc_btc_info_t *btch = wlc->btch;
	int btc_inactive_offset[WL_NUM_TXCHAIN_MAX] = {0, 0, 0, 0};

	int band = CHSPEC_IS2G(chanspec) ? WLC_BAND_2G : WLC_BAND_5G;

	if (!btch)
	{
		WL_INFORM(("%s invalid btch\n", __FUNCTION__));
		err = BCME_ERROR;
		goto finish;
	}

	if (!btch->btc_prev_connect)
	{
		WL_INFORM(("%s invalid btc_prev_connect\n", __FUNCTION__));
		err = BCME_ERROR;
		goto finish;
	}

	if (!btch->btc_profile)
	{
		WL_INFORM(("%s invalid btc_profile\n", __FUNCTION__));
		err = BCME_ERROR;
		goto finish;
	}

	if (band == WLC_BAND_2G)
	{
		if (btch->btc_profile[BTC_PROFILE_2G].enable == BTC_PROFILE_OFF)
			goto finish;

		if (!force && btch->btc_prev_connect->prev_band == WLC_BAND_2G)
			goto finish;

		if ((btch->btc_prev_connect->prev_2G_mode == BTC_PROFILE_DISABLE) &&
			(btch->btc_profile[BTC_PROFILE_2G].enable == BTC_PROFILE_DISABLE))
			goto finish;

		btc_cur_profile = &btch->btc_profile[BTC_PROFILE_2G].select_profile;
		btc_prev_profile = &btch->btc_prev_connect->prev_2G_profile;
		if (btch->btc_profile[BTC_PROFILE_2G].enable == BTC_PROFILE_DISABLE)
		{
			btch->btc_prev_connect->prev_2G_mode = BTC_PROFILE_DISABLE;
			memset(btc_cur_profile, 0, sizeof(wlc_btc_profile_t));
			btc_cur_profile->btc_wlrssi_thresh = BTC_WL_RSSI_DEFAULT;
			btc_cur_profile->btc_btrssi_thresh = BTC_BT_RSSI_DEFAULT;
			if (CHIPID(wlc->pub->sih->chip) == BCM4350_CHIP_ID) {
				btc_cur_profile->btc_wlrssi_hyst = BTC_WL_RSSI_HYST_DEFAULT_4350;
				btc_cur_profile->btc_btrssi_hyst = BTC_BT_RSSI_HYST_DEFAULT_4350;
				btc_cur_profile->btc_max_siso_resp_power[0] =
					BTC_WL_MAX_SISO_RESP_POWER_TDD_DEFAULT_4350;
				btc_cur_profile->btc_max_siso_resp_power[1] =
					BTC_WL_MAX_SISO_RESP_POWER_HYBRID_DEFAULT_4350;
			}
		}
		else
		{
			btch->btc_prev_connect->prev_2G_mode = BTC_PROFILE_ENABLE;
		}

		btch->btc_prev_connect->prev_band = WLC_BAND_2G;
	}
	else
	{
		if (btch->btc_profile[BTC_PROFILE_5G].enable == BTC_PROFILE_OFF)
			goto finish;

		if (!force && btch->btc_prev_connect->prev_band == WLC_BAND_5G)
			goto finish;

		if ((btch->btc_prev_connect->prev_5G_mode == BTC_PROFILE_DISABLE) &&
			(btch->btc_profile[BTC_PROFILE_5G].enable == BTC_PROFILE_DISABLE))
			goto finish;

		if (btch->btc_profile[BTC_PROFILE_5G].enable == BTC_PROFILE_DISABLE)
		{
			btch->btc_prev_connect->prev_5G_mode = BTC_PROFILE_DISABLE;
			memset(&btch->btc_profile[BTC_PROFILE_5G].select_profile,
				0, sizeof(wlc_btc_profile_t));
		}
		else
		{
			btch->btc_prev_connect->prev_5G_mode = BTC_PROFILE_ENABLE;
		}

		btch->btc_prev_connect->prev_band = WLC_BAND_5G;
		btc_cur_profile = &btch->btc_profile[BTC_PROFILE_5G].select_profile;
		btc_prev_profile = &btch->btc_prev_connect->prev_5G_profile;
	}

	WL_BTCPROF(("%s chanspec 0x%x\n", __FUNCTION__, chanspec));

	/* setBTCOEX_MODE */
	err = wlc_btc_mode_set(wlc, btc_cur_profile->mode);
	WL_BTCPROF(("btc mode %d\n", btc_cur_profile->mode));
	if (err)
	{
		err = BCME_ERROR;
		goto finish;
	}

	/* setDESENSE_LEVEL */
	btch->restage_rxgain_level = btc_cur_profile->desense_level;
	WL_BTCPROF(("btc desense level %d\n", btc_cur_profile->desense_level));

	/* setDESENSE */
	btch->restage_rxgain_force =
		(btch->btc_profile[BTC_PROFILE_2G].select_profile.desense)?
		BTC_RXGAIN_FORCE_2G_ON : 0;
	btch->restage_rxgain_force |=
		(btch->btc_profile[BTC_PROFILE_5G].select_profile.desense)?
		BTC_RXGAIN_FORCE_5G_ON : 0;
	WL_BTCPROF(("btc rxgain force 0x%x\n", btch->restage_rxgain_force));

	/* setting 2G thresholds, 5G thresholds are not used */
	btch->restage_rxgain_on_rssi_thresh =
		(uint8)((btch->btc_profile[BTC_PROFILE_2G].select_profile.desense_thresh_high *
		-1) & 0xFF);
	btch->restage_rxgain_off_rssi_thresh =
		(uint8)((btch->btc_profile[BTC_PROFILE_2G].select_profile.desense_thresh_low *
		-1) & 0xFF);
	WL_BTCPROF(("btc rxgain on 0x%x rxgain off 0x%x\n",
		btch->restage_rxgain_on_rssi_thresh,
		btch->restage_rxgain_off_rssi_thresh));

	/* check the state of bt_active signal */
		if (BT3P_HW_COEX(wlc) && wlc->clk) {
			wlc_bmac_btc_period_get(wlc->hw, &btch->bth_period,
				&btch->bth_active, &btch->agg_off_bm);
		}

	/* apply desense settings */
	wlc_btcx_desense(btch, band);

	/* setChain_Ack */
	wlc_btc_siso_ack_set(wlc, (int16)btc_cur_profile->chain_ack[0], TRUE);
	WL_BTCPROF(("btc chain ack 0x%x num chains %d\n",
		btc_cur_profile->chain_ack[0],
		btc_cur_profile->num_chains));

	/* setTX_CHAIN_POWER */
	if (btch->bth_active) {
	wlc_channel_set_tx_power(wlc, band, btc_cur_profile->num_chains,
		&btc_cur_profile->chain_power_offset[0],
		&btc_prev_profile->chain_power_offset[0]);
	*btc_prev_profile = *btc_cur_profile;
	} else {
		wlc_channel_set_tx_power(wlc, band, btc_cur_profile->num_chains,
			&btc_inactive_offset[0], &btc_prev_profile->chain_power_offset[0]);
	}

#ifdef WLBTCPROF_EXT
	/* set hybrid params */
	if ((band == WLC_BAND_2G) && (CHIPID(wlc->pub->sih->chip) == BCM4350_CHIP_ID)) {
		if ((err = wlc_btcx_set_ext_profile_param(wlc)) == BCME_ERROR) {
			WL_INFORM(("%s Unable to program siso ack powers\n", __FUNCTION__));
		}
	} else {
		for (i = 0; i < MAX_BT_DESENSE_LEVELS; i++) {
			btc_cur_profile->btc_max_siso_resp_power[i] =
				BTC_WL_MAX_SISO_RESP_POWER_TDD_DEFAULT;
		}

		if ((btc_cur_profile->btc_num_desense_levels == MAX_BT_DESENSE_LEVELS_4350) &&
			wlc->clk) {
			err = wlc_phy_btc_set_max_siso_resp_pwr(WLC_PI(wlc),
				&btc_cur_profile->btc_max_siso_resp_power[0],
				MAX_BT_DESENSE_LEVELS_4350);
		}
	}
#endif /* WLBTCPROF_EXT */

finish:
	return err;
}

int
wlc_btcx_select_profile_set(wlc_info_t *wlc, uint8 *pref, int len)
{
	wlc_btc_info_t *btch;

	btch = wlc->btch;
	if (!btch)
	{
		WL_INFORM(("%s invalid btch\n", __FUNCTION__));
		return BCME_ERROR;
	}

	if (pref)
	{
		if (!bcmp(pref, btch->btc_profile, len))
			return BCME_OK;

		bcopy(pref, btch->btc_profile, len);

		if (wlc_btcx_set_btc_profile_param(wlc, wlc->chanspec, TRUE))
		{
			WL_ERROR(("wl%d: %s: setting btc profile first time error: chspec %d!\n",
				wlc->pub->unit, __FUNCTION__, CHSPEC_CHANNEL(wlc->chanspec)));
		}

		return BCME_OK;
	}

	return BCME_ERROR;
}

int
wlc_btcx_select_profile_get(wlc_info_t *wlc, uint8 *pref, int len)
{
	wlc_btc_info_t *btch = wlc->btch;

	if (!btch)
	{
		WL_INFORM(("%s invalid btch\n", __FUNCTION__));
		return BCME_ERROR;
	}

	if (pref)
	{
		bcopy(btch->btc_profile, pref, len);
		return BCME_OK;
	}

	return BCME_ERROR;
}
#endif /* WLBTCPROF */

int
wlc_btc_siso_ack_set(wlc_info_t *wlc, int16 siso_ack, bool force)
{
	wlc_btc_info_t *btch = wlc->btch;

	if (!btch)
		return BCME_ERROR;

	if (force) {
		if (siso_ack == AUTO)
			btch->siso_ack_ovr = FALSE;
		else {
			/* sanity check forced value */
			if (!(siso_ack & TXCOREMASK))
				return BCME_BADARG;
			btch->siso_ack = siso_ack;
			btch->siso_ack_ovr = TRUE;
		}
	}

	if (!btch->siso_ack_ovr) {
		/* no override, set siso_ack according to btc_mode/chipids/boardflag etc. */
		if (siso_ack == AUTO) {
			siso_ack = 0;
#ifdef WLC_LOW
			if (wlc->pub->boardflags & BFL_FEM_BT) {
				/* check boardflag: antenna shared w BT */
				/* futher check srom, nvram */
				if (wlc->hw->btc->btcx_aa == 0x3) { /* two antenna */
					if (wlc->pub->boardflags2 &
					    BFL2_BT_SHARE_ANT0) { /* core 0 shared */
						siso_ack = 0x2;
					} else {
						siso_ack = 0x1;
					}
				} else if (wlc->hw->btc->btcx_aa == 0x7) { /* three antenna */
					; /* not supported yet */
				}
			}
#endif /* WLC_LOW */
		}
		btch->siso_ack = siso_ack;
	}

	if (wlc->clk) {
		if (D11REV_GE(wlc->pub->corerev, 40)) {
			wlc_bmac_write_shm(wlc->hw, M_COREMASK_BTRESP, (uint16)siso_ack);
		} else {
			wlc_bmac_write_shm(wlc->hw, M_COREMASK_BTRESP_PRE40, (uint16)siso_ack);
		}
	}

	return BCME_OK;
}

int16
wlc_btc_siso_ack_get(wlc_info_t *wlc)
{
	return wlc->btch->siso_ack;
}

static int
wlc_btc_wire_set(wlc_info_t *wlc, int int_val)
{
	int err;
	err = wlc_bmac_btc_wire_set(wlc->hw, int_val);
	wlc->btch->wire = wlc_bmac_btc_wire_get(wlc->hw);
	return err;
}

int
wlc_btc_wire_get(wlc_info_t *wlc)
{
	return wlc->btch->wire;
}

void wlc_btc_mode_sync(wlc_info_t *wlc)
{
	wlc->btch->mode = wlc_bmac_btc_mode_get(wlc->hw);
	wlc->btch->wire = wlc_bmac_btc_wire_get(wlc->hw);
}

uint8 wlc_btc_save_host_requested_pm(wlc_info_t *wlc, uint8 val)
{
	return (wlc->btch->host_requested_pm = val);
}

bool wlc_btc_get_bth_active(wlc_info_t *wlc)
{
	return wlc->btch->bth_active;
}

uint16 wlc_btc_get_bth_period(wlc_info_t *wlc)
{
	return wlc->btch->bth_period;
}

static int
wlc_btc_flags_idx_set(wlc_info_t *wlc, int int_val, int int_val2)
{
	return wlc_bmac_btc_flags_idx_set(wlc->hw, int_val, int_val2);
}

static int
wlc_btc_flags_idx_get(wlc_info_t *wlc, int int_val)
{
	return wlc_bmac_btc_flags_idx_get(wlc->hw, int_val);
}

static int
wlc_btc_params_set(wlc_info_t *wlc, int int_val, int int_val2)
{
	return wlc_bmac_btc_params_set(wlc->hw, int_val, int_val2);
}

static int
wlc_btc_params_get(wlc_info_t *wlc, int int_val)
{
	return wlc_bmac_btc_params_get(wlc->hw, int_val);
}

#ifdef WLBTCPROF_EXT
int wlc_btc_profile_set(wlc_info_t *wlc, int int_val, int iovar_id)
{
	wlc_btc_info_t *btch = wlc->btch;
	wlc_btc_profile_t *select_profile;

	select_profile = &btch->btc_profile[BTC_PROFILE_2G].select_profile;
	switch (iovar_id)
	{
	case IOV_BTC_WLRSSI_THRESH:
		select_profile->btc_wlrssi_thresh = (int8) int_val;
		break;
	case IOV_BTC_WLRSSI_HYST:
		select_profile->btc_wlrssi_hyst = (uint8) int_val;
		break;
	case IOV_BTC_BTRSSI_THRESH:
		select_profile->btc_btrssi_thresh = (int8) int_val;
		break;
	case IOV_BTC_BTRSSI_HYST:
		select_profile->btc_btrssi_hyst = (uint8) int_val;
		break;
	default:
		break;
	}
	return 1;
}

int wlc_btc_profile_get(wlc_info_t *wlc, int iovar_id)
{
	wlc_btc_info_t *btch = wlc->btch;
	wlc_btc_profile_t *select_profile;

	select_profile = &btch->btc_profile[BTC_PROFILE_2G].select_profile;
	switch (iovar_id)
	{
	case IOV_BTC_WLRSSI_THRESH:
		return select_profile->btc_wlrssi_thresh;
		break;
	case IOV_BTC_WLRSSI_HYST:
		return select_profile->btc_wlrssi_hyst;
		break;
	case IOV_BTC_BTRSSI_THRESH:
		return select_profile->btc_btrssi_thresh;
		break;
	case IOV_BTC_BTRSSI_HYST:
		return select_profile->btc_btrssi_hyst;
		break;
	default:
		break;
	}
	return 1;
}
#endif /* WLBTCPROF_EXT */

static void
wlc_btc_stuck_war50943(wlc_info_t *wlc, bool enable)
{
	wlc_bmac_btc_stuck_war50943(wlc->hw, enable);
}

static void
wlc_btc_rssi_threshold_get(wlc_info_t *wlc)
{
	wlc_bmac_btc_rssi_threshold_get(wlc->hw,
		&wlc->btch->prot_rssi_thresh,
		&wlc->btch->high_threshold,
		&wlc->btch->low_threshold);
}

#ifdef WLAMPDU
static void
wlc_ampdu_agg_state_update_rx_all_but_AWDL(wlc_info_t *wlc, bool aggr)
{
	int idx;
	wlc_bsscfg_t *cfg;

	FOREACH_BSS(wlc, idx, cfg) {
		if (BSSCFG_AWDL(wlc, cfg)) {
			continue;
		}
		wlc_ampdu_agg_state_update_rx_all(wlc, aggr);
	}
}
#endif /* WLAMPDU */

void  wlc_btc_4313_gpioctrl_init(wlc_info_t *wlc)
{
	if (CHIPID(wlc->pub->sih->chip) == BCM4313_CHIP_ID) {
	/* ePA 4313 brds */
		if (wlc->pub->boardflags & BFL_FEM) {
			if (wlc->pub->boardrev >= 0x1250 && (wlc->pub->boardflags & BFL_FEM_BT)) {
				wlc_mhf(wlc, MHF5, MHF5_4313_BTCX_GPIOCTRL, MHF5_4313_BTCX_GPIOCTRL,
					WLC_BAND_ALL);
			} else
				wlc_mhf(wlc, MHF4, MHF4_EXTPA_ENABLE,
				MHF4_EXTPA_ENABLE, WLC_BAND_ALL);
			/* iPA 4313 brds */
			} else {
				if (wlc->pub->boardflags & BFL_FEM_BT)
					wlc_mhf(wlc, MHF5, MHF5_4313_BTCX_GPIOCTRL,
						MHF5_4313_BTCX_GPIOCTRL, WLC_BAND_ALL);
			}
	}
}

uint
wlc_btc_frag_threshold(wlc_info_t *wlc, struct scb *scb)
{
	ratespec_t rspec;
	uint rate, thresh;
	wlc_bsscfg_t *cfg;

	/* Make sure period is known */
	if (wlc->btch->bth_period == 0)
		return 0;

	ASSERT(scb != NULL);

	cfg = SCB_BSSCFG(scb);
	ASSERT(cfg != NULL);

	/* if BT SCO is ongoing, packet length should not exceed 1/2 of SCO period */
	rspec = wlc_get_rspec_history(cfg);
	rate = RSPEC2KBPS(rspec);

	/*  use one half of the duration as threshold.  convert from usec to bytes */
	/* thresh = (bt_period * rate) / 1000 / 8 / 2  */
	thresh = (wlc->btch->bth_period * rate) >> 14;

	if (thresh < DOT11_MIN_FRAG_LEN)
		thresh = DOT11_MIN_FRAG_LEN;
	return thresh;
}

#if defined(STA) && defined(BTCX_PM0_IDLE_WAR)
static void
wlc_btc_pm_adjust(wlc_info_t *wlc,  bool bt_active)
{
	wlc_bsscfg_t *cfg = wlc->cfg;
	/* only bt is not active, set PM to host requested mode */
	if (wlc->btch->host_requested_pm != PM_FORCE_OFF) {
		if (bt_active) {
				if (PM_OFF == wlc->btch->host_requested_pm &&
				cfg->pm->PM != PM_FAST)
				wlc_set_pm_mode(wlc, PM_FAST, cfg);
		} else {
			if (wlc->btch->host_requested_pm != cfg->pm->PM)
				wlc_set_pm_mode(wlc, wlc->btch->host_requested_pm, cfg);
		}
	}
}
#endif /* STA */

void
wlc_btc_set_ps_protection(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg)
{
	if (MCHAN_ACTIVE(wlc->pub))
		return;

	if ((wlc_btc_wire_get(wlc) >= WL_BTC_3WIRE) &&
		wlc_btc_mode_get(wlc)) {
		wlc_bsscfg_t *pc;
		int btc_flags = wlc_bmac_btc_flags_get(wlc->hw);
		uint16 protections;
		uint16 active = 0;
		uint16 ps;

		pc = wlc->cfg;
		BCM_REFERENCE(pc);

		/* if radio is disable, driver may be down, quit here */
		if (wlc->pub->radio_disabled || !wlc->pub->up)
			return;

#if defined(STA)
		/* ??? if ismpc, driver should be in down state if up/down is allowed */
		if (wlc->mpc && wlc_ismpc(wlc))
			return;
#endif

#ifdef WL_BTCDYN
		if (!IS_DYNCTL_ON(wlc->btch->dprof)) {
#endif
			/* enable protection for hybrid mode when rssi below certain threshold */
			/* the implicit switching does not change btc_mode or PS protection */
			if (wlc->btch->prot_rssi_thresh &&
				-wlc->cfg->link->rssi > wlc->btch->prot_rssi_thresh) {
				active = MHF3_BTCX_ACTIVE_PROT;
			}

			/* enable protection if bt rssi < threshold */
			if (D11REV_IS(wlc->pub->corerev, 48) &&
				wlc->btch->prot_btrssi_thresh &&
				wlc_btc_get_btrssi(wlc->btch) <
				wlc->btch->prot_btrssi_thresh) {
				active = MHF3_BTCX_ACTIVE_PROT;
			}
#ifdef WL_BTCDYN
		}
#endif

		if (btc_flags & WL_BTC_FLAG_ACTIVE_PROT) {
			active = MHF3_BTCX_ACTIVE_PROT;
		}

		ps = (btc_flags & WL_BTC_FLAG_PS_PROTECT) ? MHF3_BTCX_PS_PROTECT : 0;
		BCM_REFERENCE(ps);

#ifdef STA
		/* Enable PS protection when the primary bsscfg is associated as
		 * an infra STA and is the only connection
		 */
		if (BSSCFG_STA(pc) && pc->current_bss->infra &&
		    WLC_BSS_CONNECTED(pc) && wlc->stas_connected == 1 &&
		    (wlc->aps_associated == 0 || wlc_ap_stas_associated(wlc->ap) == 0)) {
			/* when WPA/PSK security is enabled wait until WLC_PORTOPEN() is TRUE */
			if (pc->WPA_auth == WPA_AUTH_DISABLED || !WSEC_ENABLED(pc->wsec) ||
			    WLC_PORTOPEN(pc))
				protections = active | ps;
			else
				protections = 0;
		}
		/*
		Enable PS protection if there is only one BSS
		associated as STA and there are no APs. All else
		enable CTS2SELF.
		*/
		else if (wlc->stas_connected > 0)
		{
			if ((wlc->stas_connected == 1) && (wlc->aps_associated == 0))
				protections = active | ps;
			else
				protections = active;
		}
		/* Enable CTS-to-self protection when AP(s) are up and there are
		 * STAs associated
		 */
		else
#endif /* STA */
#ifdef AP
		if (wlc->aps_associated > 0 && wlc_ap_stas_associated(wlc->ap) > 0)
			protections = active;
		/* No protection */
		else
#endif /* AP */
			protections = 0;

		wlc_mhf(wlc, MHF3, MHF3_BTCX_ACTIVE_PROT | MHF3_BTCX_PS_PROTECT,
		        protections, WLC_BAND_2G);
#ifdef WLMCNX
		/*
		For non-VSDB the only time we turn on PS protection is when there is only
		one STA associated - primary or GC. In this case, set the BSS index in
		designated SHM location as well.
		*/
		if ((MCNX_ENAB(wlc->pub)) && (protections & ps)) {
			uint idx;
			wlc_bsscfg_t *cfg;
			int bss_idx;

			FOREACH_AS_STA(wlc, idx, cfg) {
				if (!cfg->BSS)
					continue;
				bss_idx = wlc_mcnx_BSS_idx(wlc->mcnx, cfg);
				wlc_mcnx_shm_bss_idx_set(wlc->mcnx, bss_idx);
				break;
			}
		}
#endif /* WLMCNX */
	}
}

#if defined(BCMDBG)
static int wlc_dump_btcx(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	uint8 idx, offset;
	uint16 hi, lo;
	uint32 buff[C_BTCX_DBGBLK_SZ/2];
	uint16 base = D11REV_LT(wlc->pub->corerev, 40) ?
	M_BTCX_DBGBLK: M_BTCX_DBGBLK_11AC;

	if (!wlc->clk) {
		return BCME_NOCLK;
	}

	for (idx = 0; idx < C_BTCX_DBGBLK_SZ; idx += 2) {
		offset = idx*2;
		lo = wlc_bmac_read_shm(wlc->hw, base+offset);
		hi = wlc_bmac_read_shm(wlc->hw, base+offset+2);
		buff[idx>>1] = (hi<<16) | lo;
	}

	bcm_bprintf(b, "nrfact: %u, ntxconf: %u (%u%%), txconf_durn(us): %u\n",
		buff[0], buff[1], buff[0] ? (buff[1]*100)/buff[0]: 0, buff[2]);
	return 0;
}

static int wlc_clr_btcxdump(wlc_info_t *wlc)
{
	uint16 base = D11REV_LT(wlc->pub->corerev, 40) ?
	M_BTCX_DBGBLK: M_BTCX_DBGBLK_11AC;

	if (!wlc->clk) {
	return BCME_NOCLK;
	}

	wlc_bmac_set_shm(wlc->hw, base, 0, C_BTCX_DBGBLK_SZ*2);
	return 0;
}
#endif 

/* Read relevant BTC params to determine if aggregation has to be enabled/disabled */
void
wlc_btcx_read_btc_params(wlc_info_t *wlc)
{
	wlc_btc_info_t *btc = wlc->btch;
	if (BT3P_HW_COEX(wlc) && wlc->clk) {
		wlc_bmac_btc_period_get(wlc->hw, &btc->bth_period,
			&btc->bth_active, &btc->agg_off_bm);
		if (!btc->bth_active && btc->btrssi_cnt) {
			wlc_btc_reset_btrssi(btc);
		}
	}
}

#ifdef WLRSDB
void
wlc_btcx_update_coex_iomask(wlc_info_t *wlc)
{
#if defined(BCMECICOEX) && defined(WLC_LOW)

	wlc_hw_info_t *wlc_hw = wlc->hw;

	/* Should come here only for RSDB capable devices */
	ASSERT(wlc_bmac_rsdb_cap(wlc_hw));

	if (!RSDB_ENAB(wlc->pub) ||
		(wlc_rsdb_mode(wlc) == PHYMODE_MIMO) ||
		(wlc_rsdb_mode(wlc) == PHYMODE_80P80)) {

		/* In the MIMO/80p80 mode set coex_io_mask to 0x3 on core 1
		 * (i.e) mask both Txconf and Prisel on Core 1.
		 * Leave coex_io_mask on core 0 to its default value (0x0)
		 */
		if (!si_coreunit(wlc_hw->sih)) {
			d11regs_t *sregs;

			/* Unmask coex_io_mask on core0 to 0 */
			AND_REG(wlc_hw->osh, &wlc_hw->regs->u.d11regs.coex_io_mask,
				~((1 << COEX_IOMASK_PRISEL_POS) | (1 << COEX_IOMASK_TXCONF_POS)));

			/* Set: core 1 */
			sregs = si_d11_switch_addrbase(wlc_hw->sih, 1);

			/* Enable MAC control IHR access on core 1 */
			OR_REG(wlc_hw->osh, &sregs->maccontrol, MCTL_IHR_EN);

			/* Mask Txconf and Prisel on core 1 */
			OR_REG(wlc_hw->osh, &sregs->u.d11regs.coex_io_mask,
				((1 << COEX_IOMASK_PRISEL_POS) | (1 << COEX_IOMASK_TXCONF_POS)));

			/* Disable MAC control IHR access on core 1 */
			/* OR_REG(wlc_hw->osh, &sregs->maccontrol, ~MCTL_IHR_EN); */

			/* Restore: core 0 */
			si_d11_switch_addrbase(wlc_hw->sih, 0);
		}
	} else {
		wlc_cmn_info_t* wlc_cmn = wlc->cmn;
		wlc_info_t *other_wlc;
		int idx;
		int coex_io_mask, coex_io_mask_ch;

		if (si_coreunit(wlc_hw->sih)) {
			/* Enable MAC control IHR access for core1 */
			OR_REG(wlc_hw->osh, &wlc_hw->regs->maccontrol, MCTL_IHR_EN);
		}
		/* read present core iomask */
		coex_io_mask = R_REG(wlc_hw->osh, &wlc_hw->regs->u.d11regs.coex_io_mask);

		/* by default let's not mask txconf and prisel from this core */
		coex_io_mask_ch = coex_io_mask &
			~((1 << COEX_IOMASK_TXCONF_POS)
			| (1 << COEX_IOMASK_PRISEL_POS));
		FOREACH_WLC(wlc_cmn, idx, other_wlc) {
			if (wlc != other_wlc) {
				if (CHSPEC_IS5G(wlc_hw->chanspec) &&
					CHSPEC_IS2G(other_wlc->hw->chanspec)) {
					/* mask txconf and prisel from this core */
					coex_io_mask_ch = coex_io_mask |
						(1 << COEX_IOMASK_TXCONF_POS) |
						(1 << COEX_IOMASK_PRISEL_POS);
				}
			}
		}

		/* update coex_io_mask if there is a change */
		if (coex_io_mask_ch != coex_io_mask) {
			W_REG(wlc_hw->osh, &wlc_hw->regs->u.d11regs.coex_io_mask, coex_io_mask_ch);
		}
	}
#endif /* BCMECICOEX && WLC_LOW */
}
#endif /* WLRSDB */

static void
wlc_btcx_watchdog(void *arg)
{
	wlc_btc_info_t *btc = (wlc_btc_info_t *)arg;
	wlc_info_t *wlc = (wlc_info_t *)btc->wlc;
	int btc_mode = wlc_btc_mode_get(wlc);
	struct scb *scb;
	struct scb_iter scbiter;
	int32 idx;
	wlc_bsscfg_t *bsscfg;
	ASSERT(wlc->pub->tunables->maxscb <= 255);

	/* update critical BT state, only for 2G band */
	if (btc_mode && BAND_2G(wlc->band->bandtype)) {
#if defined(STA) && defined(BTCX_PM0_IDLE_WAR)
		wlc_btc_pm_adjust(wlc, wlc->btch->bth_active);
#endif /* STA */

#ifdef WL_BTCDYN
		if (IS_DYNCTL_ON(btc->dprof)) {
			/* new dynamic btcoex algo */
			wlc_btcx_dynctl(btc);
		} else {
#endif
			/* legacy mode switching */
			wlc_btc_rssi_threshold_get(wlc);

			/* if rssi too low, switch to TDM */
			if (wlc->btch->low_threshold &&
				-wlc->cfg->link->rssi >
				wlc->btch->low_threshold) {
				if (!IS_BTCX_FULLTDM(btc_mode)) {
					wlc->btch->mode_overridden = (uint8)btc_mode;
					wlc_btc_mode_set(wlc, WL_BTC_FULLTDM);
				}
			} else if (wlc->btch->high_threshold &&
				-wlc->cfg->link->rssi <
				wlc->btch->high_threshold) {
				if (btc_mode != WL_BTC_PARALLEL) {
					wlc->btch->mode_overridden = (uint8)btc_mode;
					wlc_btc_mode_set(wlc, WL_BTC_PARALLEL);
				}
			} else {
				if (wlc->btch->mode_overridden) {
					wlc_btc_mode_set(wlc, wlc->btch->mode_overridden);
					wlc->btch->mode_overridden = 0;
				}
			}

			/* enable protection in ucode */
			wlc_btc_set_ps_protection(wlc, wlc->cfg); /* enable */
#ifdef WL_BTCDYN
		}
#endif
	}

#if !defined(BCMDONGLEHOST) && defined(BCMECICOEX) && defined(WL11N)
	if (CHIPID(wlc->pub->sih->chip) == BCM4331_CHIP_ID &&
		(wlc->pub->boardflags & BFL_FEM_BT)) {
		/* for X28, middle chain has to be disabeld when bt is active */
		/* to assure smooth rate policy */
		if ((btc_mode == WL_BTC_LITE || btc_mode == WL_BTC_HYBRID) &&
			BAND_2G(wlc->band->bandtype) && wlc->btch->bth_active) {
			if (btc_mode == WL_BTC_LITE)
				wlc_stf_txchain_set(wlc, 0x5, TRUE,
					WLC_TXCHAIN_ID_BTCOEX);
		} else {
			wlc_stf_txchain_set(wlc, wlc->stf->hw_txchain,
				FALSE, WLC_TXCHAIN_ID_BTCOEX);
		}
	}
#endif /* !defined(BCMDONGLEHOST) && defined(BCMECICOEX) && defined (WL11N) */

#ifdef WLAMPDU
	if (BT3P_HW_COEX(wlc) && BAND_2G(wlc->band->bandtype)) {
		if (wlc_btc_mode_not_parallel(btc_mode)) {
			FOREACH_BSS(wlc, idx, bsscfg) {
			/* Check if Ampdu need to be clamped for every scb */
				FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb)
					wlc_btc_ampdu_aggr(wlc, scb);
			}
		} else {
#ifdef BCMLTECOEX
			/* If LTECX is enabled,
			  * Aggregation is resumed in LTECX Watchdog
			  */
			if (!BCMLTECOEX_ENAB(wlc->pub))
#endif /* BCMLTECOEX */
			{
				/* Dynamic BTC mode requires this */
				wlc_ampdu_agg_state_update_tx_all(wlc, ON);
			}
		}
	}
#endif /* WLAMPDU */

#ifdef WL_BTCDYN
	if (!IS_DYNCTL_ON(btc->dprof)) {
		/* legacy desnse coex  */
		wlc_btcx_desense(btc, wlc->band->bandtype);
	}
#else
	/* Dynamic restaging of rxgain for BTCoex */
	wlc_btcx_desense(btc, wlc->band->bandtype);
#endif /* WL_BTCDYN */

	if (wlc->clk && (wlc->pub->sih->boardvendor == VENDOR_APPLE) &&
	    ((CHIPID(wlc->pub->sih->chip) == BCM4331_CHIP_ID) ||
	     (CHIPID(wlc->pub->sih->chip) == BCM4360_CHIP_ID))) {
		wlc_write_shm(wlc, M_COREMASK_BTRESP, (uint16)btc->siso_ack);
	}
}


/* handle BTC related iovars */

static int
wlc_btc_doiovar(void *ctx, const bcm_iovar_t *vi, uint32 actionid, const char *name,
        void *params, uint p_len, void *arg, int len, int val_size, struct wlc_if *wlcif)
{
	wlc_btc_info_t *btc = (wlc_btc_info_t *)ctx;
	wlc_info_t *wlc = (wlc_info_t *)btc->wlc;
	int32 int_val = 0;
	int32 int_val2 = 0;
	int32 *ret_int_ptr;
	bool bool_val;
	int err = 0;

	if (p_len >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	if (p_len >= (int)sizeof(int_val) * 2)
		bcopy((void*)((uintptr)params + sizeof(int_val)), &int_val2, sizeof(int_val));

	/* convenience int ptr for 4-byte gets (requires int aligned arg) */
	ret_int_ptr = (int32 *)arg;

	bool_val = (int_val != 0) ? TRUE : FALSE;

	switch (actionid) {

	case IOV_SVAL(IOV_BTC_FLAGS):
		err = wlc_btc_flags_idx_set(wlc, int_val, int_val2);
		break;

	case IOV_GVAL(IOV_BTC_FLAGS): {
		*ret_int_ptr = wlc_btc_flags_idx_get(wlc, int_val);
		break;
		}

	case IOV_SVAL(IOV_BTC_PARAMS):
		err = wlc_btc_params_set(wlc, int_val, int_val2);
		break;

	case IOV_GVAL(IOV_BTC_PARAMS):
		*ret_int_ptr = wlc_btc_params_get(wlc, int_val);
		break;

	case IOV_SVAL(IOV_BTC_MODE):
		err = wlc_btc_mode_set(wlc, int_val);
		break;

	case IOV_GVAL(IOV_BTC_MODE):
		*ret_int_ptr = wlc_btc_mode_get(wlc);
		break;

	case IOV_SVAL(IOV_BTC_WIRE):
		err = wlc_btc_wire_set(wlc, int_val);
		break;

	case IOV_GVAL(IOV_BTC_WIRE):
		*ret_int_ptr = wlc_btc_wire_get(wlc);
		break;

	case IOV_SVAL(IOV_BTC_STUCK_WAR):
		wlc_btc_stuck_war50943(wlc, bool_val);
		break;


#if defined(BCMDBG)
	case IOV_SVAL(IOV_BTCX_CLEAR_DUMP):
		err = wlc_clr_btcxdump(wlc);
		break;
#endif 

	case IOV_GVAL(IOV_BTC_SISO_ACK):
		*ret_int_ptr = wlc_btc_siso_ack_get(wlc);
		break;

	case IOV_SVAL(IOV_BTC_SISO_ACK):
		wlc_btc_siso_ack_set(wlc, (int16)int_val, TRUE);
		break;

	case IOV_GVAL(IOV_BTC_RXGAIN_THRESH):
		*ret_int_ptr = ((uint32)btc->restage_rxgain_on_rssi_thresh |
			((uint32)btc->restage_rxgain_off_rssi_thresh << 8));
		break;

	case IOV_SVAL(IOV_BTC_RXGAIN_THRESH):
		if (int_val == 0) {
			err = wlc_iovar_setint(wlc, "phy_btc_restage_rxgain", 0);
			if (err == BCME_OK) {
				btc->restage_rxgain_on_rssi_thresh = 0;
				btc->restage_rxgain_off_rssi_thresh = 0;
				btc->restage_rxgain_active = 0;
				WL_BTCPROF(("wl%d: BTC restage rxgain disabled\n", wlc->pub->unit));
			} else {
				err = BCME_NOTREADY;
			}
		} else {
			btc->restage_rxgain_on_rssi_thresh = (uint8)(int_val & 0xFF);
			btc->restage_rxgain_off_rssi_thresh = (uint8)((int_val >> 8) & 0xFF);
			WL_BTCPROF(("wl%d: BTC restage rxgain enabled\n", wlc->pub->unit));
		}
		WL_BTCPROF(("wl%d: BTC restage rxgain thresh ON: -%d, OFF -%d\n",
			wlc->pub->unit,
			btc->restage_rxgain_on_rssi_thresh,
			btc->restage_rxgain_off_rssi_thresh));
		break;

	case IOV_GVAL(IOV_BTC_RXGAIN_FORCE):
		*ret_int_ptr = btc->restage_rxgain_force;
		break;

	case IOV_SVAL(IOV_BTC_RXGAIN_FORCE):
		btc->restage_rxgain_force = int_val;
		break;

	case IOV_GVAL(IOV_BTC_RXGAIN_LEVEL):
		*ret_int_ptr = btc->restage_rxgain_level;
		break;

	case IOV_SVAL(IOV_BTC_RXGAIN_LEVEL):
		btc->restage_rxgain_level = int_val;
		if (btc->restage_rxgain_active) {
			if ((err = wlc_iovar_setint(wlc, "phy_btc_restage_rxgain",
				btc->restage_rxgain_level)) != BCME_OK) {
				/* Need to apply new level on next update */
				btc->restage_rxgain_active = 0;
				err = BCME_NOTREADY;
			}
			WL_BTCPROF(("wl%d: set BTC rxgain level %d (active %d)\n",
				wlc->pub->unit,
				btc->restage_rxgain_level,
				btc->restage_rxgain_active));
		}
		break;

#ifdef WLBTCPROF_EXT
	case IOV_SVAL(IOV_BTC_BTRSSI_THRESH):
#ifdef WLBTCPROF
		wlc_btc_profile_set(wlc, int_val, IOV_BTC_BTRSSI_THRESH);
#endif /* WLBTCPROF */
		btc->prot_btrssi_thresh = (uint8)int_val;
		break;

	case IOV_SVAL(IOV_BTC_BTRSSI_HYST):
#ifdef WLBTCPROF
		wlc_btc_profile_set(wlc, int_val, IOV_BTC_BTRSSI_HYST);
#endif /* WLBTCPROF */
		btc->btrssi_hyst = (uint8)int_val;
		break;

	case IOV_SVAL(IOV_BTC_WLRSSI_HYST):
#ifdef WLBTCPROF
		wlc_btc_profile_set(wlc, int_val, IOV_BTC_WLRSSI_HYST);
#endif /* WLBTCPROF */
		btc->wlrssi_hyst = (uint8)int_val;
		break;

	case IOV_SVAL(IOV_BTC_WLRSSI_THRESH):
#ifdef WLBTCPROF
		wlc_btc_profile_set(wlc, int_val, IOV_BTC_WLRSSI_THRESH);
#endif /* WLBTCPROF */
		err = wlc_btc_params_set(wlc, (M_BTCX_PROT_RSSI_THRESH >> 1), int_val);
		break;
#endif /* WLBTCPROF_EXT */

	default:
		err = BCME_UNSUPPORTED;
	}
	return err;
}

/* E.g., To set BTCX_HFLG_SKIPLMP, wlc_btc_hflg(wlc, 1, BTCX_HFLG_SKIPLMP) */
void
wlc_btc_hflg(wlc_info_t *wlc, bool set, uint16 val)
{
	uint16 btc_blk_ptr, btc_hflg;

	if (!wlc->clk)
		return;

	btc_blk_ptr = 2 * wlc_bmac_read_shm(wlc->hw, M_BTCX_BLK_PTR);
	btc_hflg = wlc_bmac_read_shm(wlc->hw, btc_blk_ptr + M_BTCX_HOST_FLAGS);

	if (set)
		btc_hflg |= val;
	else
		btc_hflg &= ~val;

	wlc_bmac_write_shm(wlc->hw, btc_blk_ptr + M_BTCX_HOST_FLAGS, btc_hflg);
}

/* Returns true if Aggregation needs to be turned off for BTCX */
bool
wlc_btc_turnoff_aggr(wlc_info_t *wlc)
{
	if ((wlc == NULL) || (wlc->btch == NULL)) {
		return FALSE;
	}
	return (wlc->btch->bth_period && (wlc->btch->bth_period < BT_AMPDU_THRESH));
}

bool
wlc_btc_mode_not_parallel(int btc_mode)
{
	return (btc_mode && (btc_mode != WL_BTC_PARALLEL));
}

bool
wlc_btc_active(wlc_info_t *wlc)
{
	return (wlc->btch->bth_active);
}

void wlc_btc_ampdu_aggr(wlc_info_t *wlc, struct scb *scb)
{
	ampdu_tx_info_t *ampdu_tx = wlc ? wlc->ampdu_tx : NULL;
	int btc_mode = wlc_btc_mode_get(wlc);
#ifdef WLAMPDU
	uint16 btc_reagg_en;
#endif /* WLAMPDU */

/* Make sure STA is on the home channel to avoid changing AMPDU
				 * state during scanning
				 */
/* Does not take multi-bsscfg into account */
	if (AMPDU_ENAB(wlc->pub) && !SCAN_IN_PROGRESS(wlc->scan) &&
		wlc->pub->associated) {
		bool dyagg = FALSE; /* dyn tx agg turned off by default */
		/* process all bt related disabling/enabling here */
		if (wlc_btc_turnoff_aggr(wlc) ||
			wlc->btch->agg_off_bm) {
			/* shutoff one rxchain to avoid steep rate drop */
			if ((btc_mode == WL_BTC_HYBRID) &&
				(CHIPID(wlc->pub->sih->chip) == BCM43225_CHIP_ID)) {
				wlc_stf_rxchain_set(wlc, 1, TRUE);
				wlc->stf->rxchain_restore_delay = 0;
			}
			if (IS_BTCX_FULLTDM(btc_mode)) {
				/* Configure AMPDU-TX based on scenario */
				if (wlc_btcx_pri_activity(wlc)) {
					/* Note: For A2DP or BLE, STA won't DELBA
					 * TX aggregation. Instead, aggregation will
					 * be clamped to2.5ms duration.
					 * This is needed for AWDL Jira 49554.
					 * This also includes Jira 37710 SoftAP COEX
					 * scenario, which leaves aggregation
					 * enabled in the TX direction for A2DP.
					 */
					wlc_ampdu_agg_state_update_tx_all(wlc, ON);
					/* clamping the aggregation to 2.5 ms */
					wlc_ampdu_tx_max_dur(ampdu_tx, scb,
						BTCX_MODULE, BTCX_AMPDU_MAX_DUR);
					/* For RX direction, don't send DELBA to
					 * AWDL peers, as we need to keep RX
					 * aggregation enabled.  The AWDL peers will
					 * be updated to limit their AMPDU duration
					 * to us shortly via host interaction.
					 */
					wlc_ampdu_agg_state_update_rx_all_but_AWDL(
						wlc, OFF);
				} else {
				/* Turn off Agg for TX & RX for SCO COEX */
				wlc_ampdu_agg_state_update_tx_all(wlc, OFF);
				wlc_ampdu_agg_state_update_rx_all(wlc, OFF);
				}
			} else {
				wlc_ampdu_agg_state_update_tx_all(wlc, ON);
				wlc_ampdu_agg_state_update_rx_all(wlc, ON);
				if (btc_mode == WL_BTC_HYBRID &&
					D11REV_GE(wlc->pub->corerev, 48)) {
					dyagg = TRUE; /* enable dyn tx agg */
				}
			}
			} else {
#ifdef BCMLTECOEX
			/* If LTECX is enabled,
			  * Aggregation is resumed in LTECX Watchdog
			  */
			if (!BCMLTECOEX_ENAB(wlc->pub))
#endif   /* BCMLTECOEX */
			{
				btc_reagg_en = (uint16)wlc_btc_params_get(wlc,
					BTC_PARAMS_FW_START_IDX +
					BTC_FW_RX_REAGG_AFTER_SCO);
				if (btc_reagg_en) {
					wlc_ampdu_agg_state_update_rx_all(wlc, ON);
				}
				wlc_ampdu_agg_state_update_tx_all(wlc, ON);
			}
			if ((btc_mode == WL_BTC_HYBRID) &&
				(CHIPID(wlc->pub->sih->chip) == BCM43225_CHIP_ID) &&
				(++wlc->stf->rxchain_restore_delay > 5)) {
				/* restore rxchain. */
				wlc_stf_rxchain_set(wlc,
					wlc->stf->hw_rxchain, TRUE);
				wlc->stf->rxchain_restore_delay = 0;
			}
		}
		if (wlc->btch->dyagg != AUTO) {
			dyagg = wlc->btch->dyagg; /* IOVAR override */
		}
		wlc_btc_hflg(wlc, dyagg, BTCX_HFLG_DYAGG);
	}
}

/* Returns TRUE for medium intensity COEX activities */
bool
wlc_btcx_pri_activity(wlc_info_t *wlc)
{
	if (wlc->stf->hw_txchain == 1) { /* Is this a SISO chip ? */
		if (wlc->btch->mode &&
			(wlc->btch->agg_off_bm & (C_BTCX_AGGOFF_A2DP|C_BTCX_AGGOFF_BLE))) {
			return TRUE;
		} else {
			return FALSE;
		}
	}
	return FALSE;
}
