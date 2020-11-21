/*
 * Wireless Multicast Forwarding
 *
 *   Broadcom Proprietary and Confidential. Copyright (C) 2016,
 *   All Rights Reserved.
 *   
 *   This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom;
 *   the contents of this file may not be disclosed to third parties, copied
 *   or duplicated in any form, in whole or in part, without the prior
 *   written permission of Broadcom.
 *
 *
 *   <<Broadcom-WL-IPTag/Proprietary:>>
 *
 *  $Id: wlc_wmf.h 523117 2014-12-26 18:32:49Z $
 */


#ifndef _wlc_wmf_h_
#define _wlc_wmf_h_

/* Packet handling decision code */
#define WMF_DROP 0
#define WMF_NOP 1
#define WMF_TAKEN 2

/* Module attach and detach functions */
extern wmf_info_t *wlc_wmf_attach(wlc_info_t *wlc);
extern void wlc_wmf_detach(wmf_info_t *wmfi);

/* WMF packet handler */
extern int wlc_wmf_packets_handle(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg, struct scb *scb,
	void *p, bool frombss);

/* Enable/Disable feature to send multicast packets to host */
extern int wlc_wmf_mcast_data_sendup(wmf_info_t *wmf, wlc_bsscfg_t *cfg, bool set, bool enable);

#endif	/* _wlc_wmf_h_ */
