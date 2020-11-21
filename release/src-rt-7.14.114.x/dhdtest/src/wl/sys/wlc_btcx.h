/*
 * BT Coex module interface
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
 * $Id: wlc_btcx.h 529245 2015-01-27 00:00:29Z $
 */


#ifndef _wlc_btcx_h_
#define _wlc_btcx_h_

#define BT_AMPDU_THRESH		10000	/* if BT period < this threshold, turn off ampdu */

#define BTC_RXGAIN_FORCE_OFF            0
#define BTC_RXGAIN_FORCE_2G_MASK        0x1
#define BTC_RXGAIN_FORCE_2G_ON          0x1
#define BTC_RXGAIN_FORCE_5G_MASK        0x2
#define BTC_RXGAIN_FORCE_5G_ON          0x2

/*  ----------- dynamic desense mode switching ---------------- */
#ifdef WL_BTCDYN
/* #define DYNCTL_DBG */
#ifdef	DYNCTL_DBG
	#define BTCDBG(x) printf x
	#define DBG_BTPWR_HOLES
#else
	#define BTCDBG(x)
#endif /* DYNCTL_DBG */
#define DYNCTL_ERROR(x) printf x

/* simulation of BT activity */
#define IS_DRYRUN_ON(prof)  ((prof->flags & DCTL_FLAGS_DRYRUN) != 0)

/* shm that contains current bt power for each task */
#define M_BTCX_BT_TXPWR	(118 * 2)
#define SHM_BTC_MASK_TXPWR			0X7
/*  bit-field position in shm word */
#define SHM_BTC_SHFT_TXPWR_SCO		0
#define SHM_BTC_SHFT_TXPWR_A2DP		3
#define SHM_BTC_SHFT_TXPWR_SNIFF	6
#define SHM_BTC_SHFT_TXPWR_ACL		9
/*
	BT uses 3 bits to report current Tx power
	0 = invalid (no connection), 1 <= -12dBm, 2 = -8dBm,..., 6 = 8dBm, 7 >= 12dBm
	TxPwr = 3bitValue * TxPwrStep + TxPwrOffset, where TxPwrStep = 4 and TxPwrOffset = -16
*/
#define BT_TX_PWR_STEP				4
#define BT_TX_PWR_OFFSET			(-16)
#define BT_INVALID_TX_PWR			-127
#define BTCDYN_DFLT_BTRSSI_HYSTER	1
/* time threshold to avoid incorrect btpwr readings */
#define DYNCTL_MIN_PERIOD 950

/* override built-in  dynctl_calc function  with an external one */
extern int btcx_set_ext_desense_calc(wlc_info_t *wlc, btcx_dynctl_calc_t cbfn);
/* override built-in  dynctl mode swithcing with an external one */
extern int btcx_set_ext_mswitch_calc(wlc_info_t *wlc, btcx_dynctl_calc_t cbfn);
/* initialize dynctl profile data with user's provided data (like from nvram.txt */
extern int btcx_init_dynctl_profile(wlc_info_t *wlc,  void *profile_data);
/*  WLC MAC -> notify BTCOEX about chanspec change   */
extern void wlc_btcx_chspec_change_notify(wlc_info_t *wlc, chanspec_t chanspec, bool switchband);
#endif /* WL_BTCDYN */
#ifdef WLRSDB
/* COEX IO_MASK block */
typedef enum {
	COEX_IOMASK_TXCONF_POS = 0,
	COEX_IOMASK_PRISEL_POS = 1,
	COEX_IOMASK_WLPRIO_POS = 4,
	COEX_IOMASK_WLTXON_POS = 5
} coex_io_mask_t;
#endif /* WLRSDB */

extern wlc_btc_info_t *wlc_btc_attach(wlc_info_t *wlc);
extern void wlc_btc_detach(wlc_btc_info_t *btc);
extern int wlc_btc_wire_get(wlc_info_t *wlc);
extern int wlc_btc_mode_get(wlc_info_t *wlc);
extern void wlc_btc_set_ps_protection(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg);
extern uint wlc_btc_frag_threshold(wlc_info_t *wlc, struct scb *scb);
extern void wlc_btc_mode_sync(wlc_info_t *wlc);
extern uint8 wlc_btc_save_host_requested_pm(wlc_info_t *wlc, uint8 val);
extern bool wlc_btc_get_bth_active(wlc_info_t *wlc);
extern uint16 wlc_btc_get_bth_period(wlc_info_t *wlc);
extern void wlc_btc_4313_gpioctrl_init(wlc_info_t *wlc);
extern void wlc_btcx_read_btc_params(wlc_info_t *wlc);
extern bool wlc_btc_turnoff_aggr(wlc_info_t *wlc);
extern bool wlc_btc_mode_not_parallel(int btc_mode);
extern bool wlc_btc_active(wlc_info_t *wlc);
extern bool wlc_btcx_pri_activity(wlc_info_t *wlc);
#ifdef WLRSDB
extern void wlc_btcx_update_coex_iomask(wlc_info_t *wlc);
#endif /* WLRSDB */
extern int wlc_btc_siso_ack_set(wlc_info_t *wlc, int16 int_val, bool force);
extern void wlc_btc_hflg(wlc_info_t *wlc, bool set, uint16 val);
extern void wlc_btc_ampdu_aggr(wlc_info_t *wlc, struct scb *scb);

/* BTCOEX profile data structures */
#define BTC_SUPPORT_BANDS	2

#define BTC_PROFILE_2G		0
#define BTC_PROFILE_5G		1

#define BTC_PROFILE_OFF		0
#define BTC_PROFILE_DISABLE 1
#define BTC_PROFILE_ENABLE	2

#define MAX_BT_DESENSE_LEVELS	8
#define BTC_WL_RSSI_DEFAULT -70
#define BTC_BT_RSSI_DEFAULT -70
#define BTC_WL_RSSI_HYST_DEFAULT_4350 5
#define BTC_BT_RSSI_HYST_DEFAULT_4350 5
#define MAX_BT_DESENSE_LEVELS_4350 2
#define BTC_WL_MAX_SISO_RESP_POWER_TDD_DEFAULT 127
#define BTC_WL_MAX_SISO_RESP_POWER_HYBRID_DEFAULT 127
#define BTC_WL_MAX_SISO_RESP_POWER_TDD_DEFAULT_4350 8
#define BTC_WL_MAX_SISO_RESP_POWER_HYBRID_DEFAULT_4350 16

/* WLBTCPROF info */
struct wlc_btc_profile {
	uint32 mode;
	uint32 desense;
	int desense_level;
	int desense_thresh_high;
	int desense_thresh_low;
	uint32 num_chains;
	uint32 chain_ack[WL_NUM_TXCHAIN_MAX];
	int chain_power_offset[WL_NUM_TXCHAIN_MAX];
	int8 btc_wlrssi_thresh;
	uint8 btc_wlrssi_hyst;
	int8 btc_btrssi_thresh;
	uint8 btc_btrssi_hyst;
	uint8 btc_num_desense_levels;
	uint8 btc_siso_resp_en[MAX_BT_DESENSE_LEVELS];
	int8 btc_max_siso_resp_power[MAX_BT_DESENSE_LEVELS];
};
typedef struct wlc_btc_profile wlc_btc_profile_t;

struct wlc_btc_prev_connect {
	int prev_band;
	int prev_2G_mode;
	int prev_5G_mode;
	struct wlc_btc_profile prev_2G_profile;
	struct wlc_btc_profile prev_5G_profile;
};
typedef struct wlc_btc_prev_connect wlc_btc_prev_connect_t;

struct wlc_btc_select_profile {
	int enable;
	struct wlc_btc_profile select_profile;
};
typedef struct wlc_btc_select_profile wlc_btc_select_profile_t;

/* BTCDYN info */

/*  central desense & switching decision function type */
typedef uint8 (* btcx_dynctl_calc_t)(wlc_info_t *wlc, int8 wl_rssi, int8 bt_pwr, int8 bt_rssi);

struct wlc_btcdyn_info {
	/* BTCOEX extension: adds dynamic desense & modes witching feature */
	uint16	bt_pwr_shm;	/* last raw/per task bt_pwr read from ucode */
	int8	bt_pwr;		/* current bt power  */
	int8	wl_rssi;	/* last wl rssi */
	uint8	cur_dsns; /* current desense level */
	dctl_prof_t *dprof;	/* current board dynctl profile  */
	btcx_dynctl_calc_t desense_fn;  /* calculate desense level  */
	btcx_dynctl_calc_t mswitch_fn;  /* calculate mode switch */
	/*  stimuli for dynctl dry runs(fake BT & WL activity) */
	int8	sim_btpwr;
	int8	sim_wlrssi;
	int8	sim_btrssi;
	/* mode switching hysteresis */
	int8 msw_btrssi_hyster;	/* from bt rssi */
	bool	dynctl_sim_on;  /* enable/disable simulation mode */
	uint32	prev_btpwr_ts;	/* timestamp of last call to btc_dynctl() */
	int8	prev_btpwr;		/* prev btpwr reading to filter out false invalids */
};
typedef struct wlc_btcdyn_info wlc_btcdyn_info_t;


/* BTC stuff */
struct wlc_btc_info {
	wlc_info_t *wlc;
	uint16  bth_period;             /* bt coex period. read from shm. */
	bool    bth_active;             /* bt active session */
	uint8   prot_rssi_thresh;       /* rssi threshold for forcing protection */
	uint8   ampdutx_rssi_thresh;    /* rssi threshold to turn off ampdutx */
	uint8   ampdurx_rssi_thresh;    /* rssi threshold to turn off ampdurx */
	uint8   high_threshold;         /* threshold to switch to btc_mode 4 */
	uint8   low_threshold;          /* threshold to switch to btc_mode 1 */
	uint8   host_requested_pm;      /* saved pm state specified by host */
	uint8   mode_overridden;        /* override btc_mode for long range */
	/* cached value for btc in high driver to avoid frequent RPC calls */
	int     mode;
	int     wire;
	int16   siso_ack;               /* txcoremask for siso ack (e.g., 1: use core 1 for ack) */
	int     restage_rxgain_level;
	int     restage_rxgain_force;
	int     restage_rxgain_active;
	uint8   restage_rxgain_on_rssi_thresh;  /* rssi threshold to turn on rxgain restaging */
	uint8   restage_rxgain_off_rssi_thresh; /* rssi threshold to turn off rxgain restaging */
	uint16	agg_off_bm;
	bool    siso_ack_ovr;           /* siso_ack set 0: automatically 1: by iovar */
	wlc_btc_prev_connect_t *btc_prev_connect; /* btc previous connection info */
	wlc_btc_select_profile_t *btc_profile; /* User selected profile for 2G and 5G params */
	wlc_btcdyn_info_t *btcdyn; /* btcdyn info */
	int8	*btrssi;        /* array of recent BT RSSI values */
	int16	btrssi_sum;	/* bt rssi MA sum */
	uint8   btrssi_cnt;     /* number of btrssi samples */
	uint8	btrssi_idx;	/* index to bt_rssi sample array */
	int8	bt_rssi;	/* averaged bt rssi */
	uint16	bt_shm_addr;
	uint8	run_cnt;
	int8	prot_btrssi_thresh; /* used by implicit mode switching */
	int8    dyagg;                   /* dynamic tx agg (1: on, 0: off, -1: auto) */
};

#ifdef WLBTCPROF

#define WL_BTCPROF WL_INFORM

extern int wlc_btc_profile_set(wlc_info_t *wlc, int int_val, int iovar_id);
extern int wlc_btc_profile_get(wlc_info_t *wlc, int iovar_id);
extern int wlc_btcx_select_profile_set(wlc_info_t *wlc, uint8 *pref, int len);
extern int wlc_btcx_select_profile_get(wlc_info_t *wlc, uint8 *pref, int len);
extern int wlc_btcx_set_btc_profile_param(struct wlc_info *wlc, chanspec_t chanspec, bool force);
extern int wlc_btcx_set_ext_profile_param(wlc_info_t *wlc);
#else
#define WL_BTCPROF WL_NONE
#endif /* WLBTCPROF */

#endif /* _wlc_btcx_h_ */
