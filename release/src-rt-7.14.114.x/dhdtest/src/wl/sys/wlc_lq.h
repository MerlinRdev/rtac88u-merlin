/*
 * Code that controls the link quality
 * Broadcom 802.11bang Networking Device Driver
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
 * $Id: wlc_lq.h 530263 2015-01-29 18:21:14Z $
 */

#ifndef _wlc_lq_h_
#define _wlc_lq_h_

/*
 * For multi-channel (VSDB), chanim supports 2 interfaces.
 */
#define CHANIM_NUM_INTERFACES_MCHAN		2
#define CHANIM_NUM_INTERFACES_SINGLECHAN	1

extern int wlc_lq_attach(wlc_info_t* wlc);
extern void wlc_lq_detach(wlc_info_t* wlc);

#ifdef WLCHANIM
extern int wlc_lq_chanim_update(wlc_info_t *wlc, chanspec_t chanspec, uint32 flags);
extern void wlc_lq_chanim_acc_reset(wlc_info_t *wlc);
extern bool wlc_lq_chanim_interfered(wlc_info_t *wlc, chanspec_t chanspec);
extern void wlc_lq_chanim_upd_act(wlc_info_t *wlc);
extern void wlc_lq_chanim_upd_acs_record(chanim_info_t *c_info, chanspec_t home_chspc,
	chanspec_t selected, uint8 trigger);
extern void wlc_lq_chanim_action(wlc_info_t *wlc);
#if defined(WLMCHAN) && !defined(WLMCHAN_DISABLED)
extern void wlc_lq_chanim_create_bss_chan_context(wlc_info_t *wlc, chanspec_t chanspec,
	chanspec_t prev_chanspec);
extern void wlc_lq_chanim_delete_bss_chan_context(wlc_info_t *wlc, chanspec_t chanspec);
extern int wlc_lq_chanim_adopt_bss_chan_context(wlc_info_t *wlc, chanspec_t chanspec,
	chanspec_t prev_chanspec);
#else /* !WLMCHAN || WLMCHAN_DISABLED */
#define wlc_lq_chanim_create_bss_chan_context(a, b, c)	do {} while (0)
#define wlc_lq_chanim_delete_bss_chan_context(a, b)	do {} while (0)
#define wlc_lq_chanim_adopt_bss_chan_context(a, b, c)	do {} while (0)
#endif /* !WLMCHAN || WLMCHAN_DISABLED */
#else
#define wlc_lq_chanim_update(a, b, c)	BCME_OK
#define wlc_lq_chanim_acc_reset(a)	do {} while (0)
#define wlc_lq_chanim_interfered(a, b)	0
#define wlc_lq_chanim_upd_act(a)	do {} while (0)
#define wlc_lq_chanim_upd_acs_record(a, b, c, d) do {} while (0)
#define wlc_lq_chanim_action(a)		do {} while (0)
#ifdef WLMCHAN
#define wlc_lq_chanim_create_bss_chan_context(a, b, c)	do {} while (0)
#define wlc_lq_chanim_delete_bss_chan_context(a, b)	do {} while (0)
#define wlc_lq_chanim_adopt_bss_chan_context(a, b, c)	BCME_OK
#endif /* WLMCHAN */
#endif /* WLCHANIM */
extern wlc_chanim_stats_t* wlc_lq_chanim_chanspec_to_stats(chanim_info_t *c_info, chanspec_t);


#ifdef WLCQ
extern int wlc_lq_channel_qa_start(wlc_info_t *wlc);
#else
#define wlc_lq_channel_qa_start(a)	(0)
#endif

typedef int (*stats_cb)(wlc_info_t *wlc, void *ctx, uint32 elapsed_time, void *vstats);

/* register a callbk [cb] to return wlc_bmac_obss_counts_t stats after
* req_time millisecs
*/
int wlc_lq_register_obss_stats_cb(wlc_info_t *wlc, uint32 req_time_ms, stats_cb cb,
	uint16 connID, void *arg);

extern void wlc_lq_rssi_init(wlc_info_t *wlc, int rssi);
extern int8 wlc_lq_rssi_pktrxh_cal(wlc_info_t *wlc, wlc_d11rxhdr_t *wrxh);
extern int  wlc_lq_recv_snr_compute(wlc_info_t *wlc, wlc_d11rxhdr_t *wrxh, int noise_lvl);
extern void wlc_lq_noise_sample_request(wlc_info_t *wlc, uint8 request, uint8 channel);
extern void wlc_lq_noise_cb(wlc_info_t *wlc, uint8 channel, int8 noise_dbm);
extern void wlc_lq_channel_qa_sample_req(wlc_info_t *wlc);

extern void wlc_lq_rssi_snr_noise_reset_ma(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int nval,
	int snr, int noise);
extern void wlc_lq_rssi_reset_ma(wlc_bsscfg_t *cfg, int nval);
extern int  wlc_lq_rssi_update_ma(wlc_bsscfg_t *cfg, int nval, int qdb, bool unicast);
extern void wlc_lq_rssi_event_update(wlc_bsscfg_t *cfg);
extern int  wlc_lq_snr_update_ma(wlc_bsscfg_t *cfg, int nval, bool unicast);
extern int  wlc_lq_noise_update_ma(wlc_info_t *wlc, int nval);
extern int  wlc_lq_noise_lte_update_ma(wlc_info_t *wlc, int nval);
#ifdef CCA_STATS
extern bool wlc_lq_cca_chan_qual_event_update(wlc_info_t *wlc, uint8 id, int v);
#endif

#endif	/* _wlc_lq_h */
