/*
 * 802.11k definitions for
 * Broadcom 802.11abgn Networking Device Driver
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
 * $Id: wlc_rrm.h 523117 2014-12-26 18:32:49Z $
 */


#ifndef _wlc_rrm_h_
#define _wlc_rrm_h_

extern wlc_rrm_info_t *wlc_rrm_attach(wlc_info_t *wlc);
extern void wlc_rrm_detach(wlc_rrm_info_t *rrm_info);
extern void wlc_frameaction_rrm(wlc_rrm_info_t *rrm_info, wlc_bsscfg_t *cfg, struct scb *scb,
	uint action_id, uint8 *body, int body_len, int8 rssi, ratespec_t rspec);
extern void wlc_rrm_pm_pending_complete(wlc_rrm_info_t *rrm_info);
extern void wlc_rrm_terminate(wlc_rrm_info_t *rrm_info);
extern bool wlc_rrm_inprog(wlc_info_t *wlc);
extern void wlc_rrm_stop(wlc_info_t *wlc);
extern bool wlc_rrm_wait_tx_suspend(wlc_info_t *wlc);
extern void wlc_rrm_start_timer(wlc_info_t *wlc);
extern bool wlc_rrm_enabled(wlc_rrm_info_t *rrm_info, wlc_bsscfg_t *cfg);
#ifdef WLSCANCACHE
extern void wlc_rrm_update_cap(wlc_rrm_info_t *rrm_info, wlc_bsscfg_t *bsscfg);
#endif /* WLSCANCACHE */
extern bool wlc_rrm_in_progress(wlc_info_t *wlc);
extern void wlc_rrm_upd_data_activity_ts(wlc_rrm_info_t *ri);
#endif	/* _wlc_rrm_h_ */
