/*
 * A-MPDU Tx (with extended Block Ack) related header file
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
 * $Id: wlc_ampdu.h 529245 2015-01-27 00:00:29Z $
*/


#ifndef _wlc_ampdu_tx_h_
#define _wlc_ampdu_tx_h_

typedef enum {
				BTCX_MODULE,
				AWDL_MODULE,
				P2P_MODULE,
				NUM_MODULES
} scb_ampdu_module_t;

extern ampdu_tx_info_t *wlc_ampdu_tx_attach(wlc_info_t *wlc);
extern void wlc_ampdu_tx_detach(ampdu_tx_info_t *ampdu_tx);

/**
 * @brief Enable A-MPDU aggregation for the specified SCB
 *
 */
extern void wlc_ampdu_tx_scb_enable(ampdu_tx_info_t *ampdu_tx, struct scb *scb);

/**
 * @brief Disable A-MPDU aggregation for the specified SCB
 *
 */
extern void wlc_ampdu_tx_scb_disable(ampdu_tx_info_t *ampdu_tx, struct scb *scb);

#ifdef NEW_TXQ
extern int wlc_sendampdu(ampdu_tx_info_t *ampdu_tx, wlc_txq_info_t *qi, void **aggp, int prec,
	struct spktq *output_q, int *supplied_time);
#else
extern int wlc_sendampdu(ampdu_tx_info_t *ampdu_tx, wlc_txq_info_t *qi, void **aggp, int prec);
#endif /* NEW_TXQ */
extern bool wlc_ampdu_dotxstatus(ampdu_tx_info_t *ampdu_tx, struct scb *scb, void *p,
	tx_status_t *txs, wlc_txh_info_t *txh_info);
extern void wlc_ampdu_dotxstatus_regmpdu(ampdu_tx_info_t *ampdu_tx, struct scb *scb, void *p,
	tx_status_t *txs, wlc_txh_info_t *txh_info);
extern void wlc_ampdu_tx_reset(ampdu_tx_info_t *ampdu_tx);

extern void wlc_ampdu_macaddr_upd(wlc_info_t *wlc);
#if defined(BCMPCIEDEV) && defined(PROP_TXSTATUS)
extern bool wlc_ampdu_if_freechain(ampdu_tx_info_t *ampdu_tx, void *p, struct scb *scb);
#endif
extern uint8 wlc_ampdu_null_delim_cnt(ampdu_tx_info_t *ampdu_tx, struct scb *scb,
	ratespec_t rspec, int phylen, uint16* minbytes);
extern bool wlc_ampdu_frameburst_override(ampdu_tx_info_t *ampdu_tx);
#ifdef WLC_HIGH_ONLY
extern void wlc_ampdu_txstatus_complete(ampdu_tx_info_t *ampdu_tx, uint32 s1, uint32 s2);
#endif
extern void wlc_ampdu_tx_set_bsscfg_aggr_override(ampdu_tx_info_t *ampdu_tx,
	wlc_bsscfg_t *bsscfg, int8 txaggr);
extern uint16 wlc_ampdu_tx_get_bsscfg_aggr(ampdu_tx_info_t *ampdu_tx, wlc_bsscfg_t *bsscfg);
extern void wlc_ampdu_tx_set_bsscfg_aggr(ampdu_tx_info_t *ampdu_tx, wlc_bsscfg_t *bsscfg,
	bool txaggr, uint16 conf_TID_bmap);

extern bool wlc_ampdu_tx_cap(ampdu_tx_info_t *ampdu_tx);
extern int wlc_ampdumac_set(ampdu_tx_info_t *ampdu_tx, uint8 on);
extern  void wlc_ampdu_tx_max_dur(ampdu_tx_info_t *ampdu_tx, struct scb *scb,
	scb_ampdu_module_t module_id, uint16 dur);

#ifdef WLAMPDU_MAC
#define AMU_EPOCH_NO_CHANGE		-1	/* no-epoch change but change params */
#define AMU_EPOCH_CHG_PLCP		0	/* epoch change due to plcp */
#define AMU_EPOCH_CHG_FID		1	/* epoch change due to rate flag in frameid */
#define AMU_EPOCH_CHG_NAGG		2	/* epoch change due to ampdu off */
#define AMU_EPOCH_CHG_MPDU		3	/* epoch change due to mpdu */
#define AMU_EPOCH_CHG_DSTTID		4	/* epoch change due to dst/tid */
#define AMU_EPOCH_CHG_SEQ		5	/* epoch change due to discontinuous seq no */
#define AMU_EPOCH_CHG_TXC_UPD  6    /* Epoch change due to txcache update  */
#define AMU_EPOCH_CHG_TXHDR    7    /* Epoch change due to Long hdr to short hdr transition */


extern void wlc_ampdu_change_epoch(ampdu_tx_info_t *ampdu_tx, int fifo, int reason_code);
extern uint8 wlc_ampdu_chgnsav_epoch(ampdu_tx_info_t *, int fifo,
	int reason_code, struct scb *, uint8 tid, wlc_txh_info_t*);
extern bool wlc_ampdu_was_ampdu(ampdu_tx_info_t *, int fifo);
extern int wlc_dump_aggfifo(wlc_info_t *wlc, struct bcmstrbuf *b);
extern void wlc_ampdu_fill_percache_info(ampdu_tx_info_t *ampdu_tx, struct scb *scb, uint8 tid,
	d11actxh_t *txh);
#endif /* WLAMPDU_MAC */

#define TXFS_WSZ_AC_BE	32
#define TXFS_WSZ_AC_BK	10
#define TXFS_WSZ_AC_VI	4
#define TXFS_WSZ_AC_VO	4

#define AMPDU_MAXDUR_INVALID_VAL 0xffff
#define AMPDU_MAXDUR_INVALID_IDX 0xff

extern void wlc_sidechannel_init(ampdu_tx_info_t *ampdu_tx);


extern void ampdu_cleanup_tid_ini(ampdu_tx_info_t *ampdu_tx, struct scb *scb,
	uint8 tid, bool force);

extern void scb_ampdu_tx_flush(ampdu_tx_info_t *ampdu_tx, struct scb *scb);

extern void wlc_ampdu_clear_tx_dump(ampdu_tx_info_t *ampdu_tx);

extern void wlc_ampdu_recv_ba(ampdu_tx_info_t *ampdu_tx, struct scb *scb, uint8 *body,
	int body_len);
extern void wlc_ampdu_recv_addba_req_ini(ampdu_tx_info_t *ampdu_tx, struct scb *scb,
	dot11_addba_req_t *addba_req, int body_len);
extern void wlc_ampdu_recv_addba_resp(ampdu_tx_info_t *ampdu_tx, struct scb *scb,
	uint8 *body, int body_len);

#if defined(BCMDBG) || defined(WLTEST) || defined(BCMDBG_AMPDU)
extern int wlc_ampdu_tx_dump(ampdu_tx_info_t *ampdu_tx, struct bcmstrbuf *b);
#endif /* BCMDBG || WLTEST */

extern void wlc_ampdu_tx_set_mpdu_density(ampdu_tx_info_t *ampdu_tx, uint8 mpdu_density);
extern void wlc_ampdu_tx_set_ba_tx_wsize(ampdu_tx_info_t *ampdu_tx, uint8 wsize);
extern uint8 wlc_ampdu_tx_get_ba_tx_wsize(ampdu_tx_info_t *ampdu_tx);
extern uint8 wlc_ampdu_tx_get_ba_max_tx_wsize(ampdu_tx_info_t *ampdu_tx);
extern void wlc_ampdu_tx_recv_delba(ampdu_tx_info_t *ampdu_tx, struct scb *scb, uint8 tid,
	uint8 category, uint16 initiator, uint16 reason);
extern void wlc_ampdu_tx_send_delba(ampdu_tx_info_t *ampdu_tx, struct scb *scb, uint8 tid,
	uint16 initiator, uint16 reason);

extern int wlc_ampdu_tx_set(ampdu_tx_info_t *ampdu_tx, bool on);

extern uint wlc_ampdu_tx_get_tcp_ack_ratio(ampdu_tx_info_t *ampdu_tx);

extern uint8 wlc_ampdu_get_txpkt_weight(ampdu_tx_info_t *ampdu_tx);

#if defined(MACOSX) && !defined(TXQ_MUX)
extern bool wlc_ampdu_tid_enabled(wlc_info_t *wlc, uint tid);
extern uint wlc_ampdu_txpktcnt_tid(ampdu_tx_info_t *ampdu, uint tid);
extern uint wlc_ampdu_pmax_tid(ampdu_tx_info_t *ampdu, uint tid);
#endif /* MACOSX */

#if defined(WLNAR)
extern uint8 BCMFASTPATH wlc_ampdu_ba_on_tidmask(const struct scb *scb);
#endif

extern struct pktq* wlc_ampdu_txq(ampdu_tx_info_t *ampdu, struct scb *scb);
#ifdef PROP_TXSTATUS
extern void wlc_ampdu_send_bar_cfg(ampdu_tx_info_t * ampdu, struct scb *scb);
extern void wlc_ampdu_flush_ampdu_q(ampdu_tx_info_t *ampdu, wlc_bsscfg_t *cfg);
extern void wlc_ampdu_flush_pkts(wlc_info_t *wlc, struct scb *scb, uint8 tid);
#endif /* PROP_TXSTATUS */

extern void wlc_check_ampdu_fc(ampdu_tx_info_t *ampdu, struct scb *scb);

extern void wlc_ampdu_txeval_all(wlc_info_t *wlc);

extern void wlc_ampdu_agg_state_update_tx_all(wlc_info_t *wlc, bool aggr);

#if defined(WL_LINKSTAT)
extern void wlc_ampdu_txrates_get(ampdu_tx_info_t *ampdu_tx, wifi_rate_stat_t *rate,
	int i, bool vht);
#endif

#endif /* _wlc_ampdu_tx_h_ */
