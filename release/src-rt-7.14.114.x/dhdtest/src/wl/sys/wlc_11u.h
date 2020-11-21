/*
 * 802.11u module header file (interworking protocol)
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
 * $Id: wlc_11u.h 524513 2015-01-07 06:00:57Z $
*/


#ifndef _wlc_11u_h_
#define _wlc_11u_h_

/* APIs */
#ifdef WL11U
/* 802.11u Interworking(IW) IE size */
#define IW_IE_MAX_SIZE 				11
/* 802.11u IW Advertisement Protocol IE size */
#define IWAP_IE_MAX_SIZE 			4
/* Access network type offset in IW IE */
#define IW_ANT_OFFSET				0
#define IW_ANT_WILDCARD			0xF
#define IW_LEN						9
#define IW_HESSID_OFFSET			3
#define SHORT_IW_HESSID_OFFSET	1
#define IWAP_QUERY_INFO_SIZE		1

/* module */
extern bool wlc_11u_check_probe_req_iw(void *handle, wlc_bsscfg_t *cfg,
	wlc_d11rxhdr_t *wrxh, uint8 *plcp, struct dot11_management_header *hdr,
	uint8 *body, int body_len, bool *psendProbeResp);

extern wlc_11u_info_t *wlc_11u_attach(wlc_info_t *wlc);
extern void wlc_11u_detach(wlc_11u_info_t *m11u);

extern int wlc_11u_set_rx_qos_map_ie(wlc_11u_info_t *m11u, wlc_bsscfg_t *cfg,
	bcm_tlv_t *ie, int ie_len);
extern void wlc_11u_set_pkt_prio(wlc_11u_info_t *m11u, wlc_bsscfg_t *cfg, void *pkt);

#else /* !WL11U */
#define wlc_11u_set_pkt_prio(m11u, cfg, pkt)
#endif /* !WL11U */

#endif /* _wlc_11u_h_ */
