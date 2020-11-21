/*
 * MBSS (Multi BSS) related declarations and exported functions for
 * Broadcom 802.11 Networking Device Driver
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


#ifndef _WLC_MBSS_H_
#define _WLC_MBSS_H_

#include "wlc_cfg.h"

#include <typedefs.h>
#include <proto/ethernet.h>

#include "wlc_types.h"

/* max ap bss supported by driver */
#define WLC_MAX_AP_BSS(d11corerev) \
	((D11REV_IS(d11corerev, 16) || D11REV_IS(d11corerev, 17) || D11REV_IS(d11corerev, 22)) \
	 ? 8 : \
	 (D11REV_IS(d11corerev, 24) || D11REV_IS(d11corerev, 25)) ? 4 : WLC_MAX_UCODE_BSS)

/* calculate the number of template mem blks needed for beacons
 * and probe responses of all the BSSs. add one additional block
 * to account for 104 bytes of space reserved (?) at the start of
 * template memory.
 */
#define TPLBLKS_PER_BCN(r)	(D11REV_GE(r, 40) ?		\
				 TPLBLKS_AC_PER_BCN_NUM : TPLBLKS_PER_BCN_NUM)
#define TPLBLKS_PER_PRS(r)	(D11REV_GE(r, 40) ?		\
				 TPLBLKS_AC_PER_PRS_NUM : TPLBLKS_PER_PRS_NUM)

#define	MBSS_TPLBLKS(r, n)		(1 + ((n) * (TPLBLKS_PER_BCN(r) + TPLBLKS_PER_PRS(r))))
#define	MBSS_TXFIFO_START_BLK(r, n)	MBSS_TPLBLKS(r, n)

/*
 * Under MBSS, a pre-TBTT interrupt is generated.  The driver puts beacons in
 * the ATIM fifo at that time and tells uCode about pending BC/MC packets.
 * The delay is settable thru uCode.  MBSS_PRE_TBTT_DEFAULT_us is the default
 * setting for this value.
 * If the driver experiences significant latency, it must avoid setting up
 * beacons or changing the SHM FID registers.  The "max latency" setting
 * indicates the maximum permissible time between the TBTT interrupt and the
 * DPC to respond to the interrupt before the driver must abort the TBTT
 * beacon operations.
 */
#define MBSS_PRE_TBTT_DEFAULT_us 5000		/* 5 milliseconds! */
#define MBSS_PRE_TBTT_MAX_LATENCY_us 4000
#define MBSS_PRE_TBTT_MIN_THRESH_us 1000	/* 1 msec threshold before actual TBTT */


/* external function prototypes */
extern wlc_mbss_info_t *wlc_mbss_attach(wlc_info_t *wlc);
extern void wlc_mbss_detach(wlc_mbss_info_t *mbss);

extern int wlc_mbss_bsscfg_up(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern void wlc_mbss_bsscfg_down(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern void wlc_mbss_update_beacon(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern void wlc_mbss_update_probe_resp(wlc_info_t *wlc, wlc_bsscfg_t *cfg, bool suspend);
extern bool wlc_prq_process(wlc_info_t *wlc, bool bounded);
extern void wlc_mbss_dotxstatus(wlc_info_t *wlc, tx_status_t *txs, void *pkt, uint16 fc,
                                wlc_pkttag_t *pkttag, uint supr_status);
extern void wlc_mbss_dotxstatus_mcmx(wlc_info_t *wlc, wlc_bsscfg_t *cfg, tx_status_t *txs);
extern void wlc_mbss_shm_ssid_upd(wlc_info_t *wlc, wlc_bsscfg_t *cfg, uint16 *base);
extern void wlc_mbss_txq_update_bcmc_counters(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern void wlc_mbss_increment_ps_trans_cnt(wlc_info_t *wlc, wlc_bsscfg_t *cfg);

#ifdef BCMDBG
extern void wlc_mbss_dump_spt_pkt_state(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int i);
#else
#define wlc_mbss_dump_spt_pkt_state(wlc, cfg, i)
#endif /* BCMDBG || BCMDBG_ERR */

extern wlc_pkt_t wlc_mbss_get_probe_template(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern wlc_pkt_t wlc_mbss_get_bcn_template(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern wlc_spt_t *wlc_mbss_get_bcn_spt(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
extern void wlc_mbss_set_bcn_tim_ie(wlc_info_t *wlc, wlc_bsscfg_t *cfg, uint8 *ie);
extern uint32 wlc_mbss_get_bcmc_pkts_sent(wlc_info_t *wlc, wlc_bsscfg_t *cfg);

int wlc_mbss_validate_mac(wlc_info_t *wlc, wlc_bsscfg_t *cfg, struct ether_addr *addr);
void wlc_mbss_reset_mac(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
void wlc_mbss_set_mac(wlc_info_t *wlc, wlc_bsscfg_t *cfg);
void wlc_mbss_record_time(wlc_info_t *wlc);
void wlc_mbss_reset_prq(wlc_info_t *wlc);

int wlc_mbss4_tbtt(wlc_info_t *wlc, uint32 macintstatus);
int wlc_mbss16_tbtt(wlc_info_t *wlc, uint32 macintstatus);

#endif /* _WLC_MBSS_H_ */
