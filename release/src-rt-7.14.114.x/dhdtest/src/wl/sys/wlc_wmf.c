/**
 * @file
 * @brief
 * Wireless Multicast Forwarding (WMF)
 *
 * WMF is forwarding multicast packets as unicast packets to
 * multicast group members in a BSS
 *
 * Supported protocol families: IPV4
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
 * $Id: wlc_wmf.c 529218 2015-01-26 21:13:23Z $
 */

/**
 * @file
 * @brief
 * The objective of Wireless Multicast Forwarding (WMF) module is to support multicast streaming
 * from Access Point to each participating multicast group members that can be a STA, WET or WDS end
 * point. Streaming between WET/STA devices is also supported.
 */


#include <wlc_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <wlioctl.h>
#include <proto/802.11.h>
#include <proto/ethernet.h>
#include <proto/vlan.h>
#include <proto/802.3.h>
#include <proto/bcmip.h>
#include <proto/bcmarp.h>
#include <proto/bcmudp.h>
#include <proto/bcmdhcp.h>
#include <bcmendian.h>

#include <d11.h>
#include <wlc_rate.h>
#include <wlc_pub.h>
#include <wlc_bsscfg.h>
#include <wlc.h>
#include <wlc_scb.h>
#include <wl_export.h>
#include <wlc_wmf.h>
#include <emf/emf/emf_cfg.h>
#include <emf/emf/emfc_export.h>
#include <emf/igs/igs_cfg.h>
#include <emf/igs/igsc_export.h>


struct wmf_info {
	wlc_info_t *wlc;
	int	scb_handle;	/* scb cubby handle */
	int	cfg_handle;	/* bsscfg cubby handle */
};

/* bsscfg specific data */
typedef struct {
	bool	wmf_ucast_igmp_query;	/* 1 to enable, 0 by default */
	bool	wmf_ucast_upnp;		/* 1 to enable, 0 by default */
	bool	wmf_psta_disable;	/* shall unicats to psta (default) or not */
	bool wmf_instance;	/* WMF instance instantiated */
	void *emfci;	/* Pointer to emfc instance */
	void *igsci;	/* Pointer to igsc instance */
} bss_wmf_info_t;

/* bsscfg cubby access macro */
#define BSS_WMF_INFO(wmf, cfg)	(bss_wmf_info_t *)BSSCFG_CUBBY(cfg, (wmf)->cfg_handle)

/* forward declarations */
static int wlc_wmf_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid, const char *name,
        void *p, uint plen, void *a, int alen, int vsize, struct wlc_if *wlcif);
static void wlc_wmf_scb_deinit(void *context, struct scb *scb);
static void wlc_wmf_bss_deinit(void *ctx, wlc_bsscfg_t *cfg);
static int32 wmf_forward(void *wrapper, void *p, uint32 mgrp_ip, void *txif, bool rt_port);
static void wmf_sendup(void *wrapper, void *p);
static int32 wmf_hooks_register(void *wrapper);
static int32 wmf_hooks_unregister(void *wrapper);
static int32 wmf_igs_broadcast(void *wrapper, uint8 *ip, uint32 length, uint32 mgrp_ip);
static int wlc_wmf_down(void *hdl);

#if defined(BCMDBG) || defined(WLTEST)
static int wlc_wmf_dump(void *context, struct bcmstrbuf *b);
#endif /* BCMDBG || WLTEST */

/* Add wmf instance to a bsscfg */
static int32 wlc_wmf_instance_add(wmf_info_t *wmf, struct wlc_bsscfg *bsscfg);
/* Delete wmf instance from bsscfg */
static void wlc_wmf_instance_del(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg);
/* Start WMF on the bsscfg */
static int wlc_wmf_start(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg);
/* Stop WMF on the bsscfg */
static void wlc_wmf_stop(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg);
/* Delete a station from the WMF interface list */
static int wlc_wmf_sta_del(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg, struct scb *scb);

/* iovar table */
enum {
	IOV_WMF_BSS_ENABLE,
	IOV_WMF_UCAST_IGMP,
	IOV_WMF_MCAST_DATA_SENDUP,
	IOV_WMF_PSTA_DISABLE,
#ifdef WL_IGMP_UCQUERY
	IOV_WMF_UCAST_IGMP_QUERY,
#endif
#ifdef WL_UCAST_UPNP
	IOV_WMF_UCAST_UPNP,
#endif
	IOV_LAST 		/* In case of a need to check max ID number */
};

static const bcm_iovar_t wmf_iovars[] = {
	{"wmf_bss_enable", IOV_WMF_BSS_ENABLE,
	(0), IOVT_BOOL, 0
	},
	{"wmf_ucast_igmp", IOV_WMF_UCAST_IGMP,
	(0), IOVT_BOOL, 0
	},
	{"wmf_mcast_data_sendup", IOV_WMF_MCAST_DATA_SENDUP,
	(0), IOVT_BOOL, 0
	},
	{"wmf_psta_disable", IOV_WMF_PSTA_DISABLE,
	(IOVF_SET_DOWN), IOVT_BOOL, 0
	},
#ifdef WL_IGMP_UCQUERY
	{"wmf_ucast_igmp_query", IOV_WMF_UCAST_IGMP_QUERY,
	(0), IOVT_BOOL, 0
	},
#endif
#ifdef WL_UCAST_UPNP
	{"wmf_ucast_upnp", IOV_WMF_UCAST_UPNP,
	(0), IOVT_BOOL, 0
	},
#endif
	{NULL, 0, 0, 0, 0 }
};

#define IGMPV2_HOST_MEMBERSHIP_QUERY	0x11
#define MCAST_ADDR_UPNP_SSDP(addr) ((addr) == 0xeffffffa)

/*
 * WMF module attach function
 */
wmf_info_t *
BCMATTACHFN(wlc_wmf_attach)(wlc_info_t *wlc)
{
	wmf_info_t *wmf;

	if (!(wmf = (wmf_info_t *)MALLOCZ(wlc->osh, sizeof(wmf_info_t)))) {
		WL_ERROR(("wl%d: wlc_wmf_attach: out of mem, malloced %d bytes\n",
		          wlc->pub->unit, MALLOCED(wlc->osh)));
		return NULL;
	}

	wmf->wlc = wlc;

	/* reserve cubby in the scb container for per-scb private data */
	wmf->scb_handle = wlc_scb_cubby_reserve(wlc, 0, NULL, wlc_wmf_scb_deinit,
		NULL, (void *)wmf);
	if (wmf->scb_handle < 0) {
		WL_ERROR(("wl%d: wlc_wmf_attach: wlc_scb_cubby_reserve failed\n", wlc->pub->unit));
		MFREE(wlc->osh, wmf, sizeof(wmf_info_t));
		return NULL;
	}

	/* reserve cubby in the cfg container for per-cfg private data */
	wmf->cfg_handle = wlc_bsscfg_cubby_reserve(wlc, sizeof(bss_wmf_info_t),
	                                           NULL, wlc_wmf_bss_deinit, NULL, wmf);
	if (wmf->cfg_handle < 0) {
		WL_ERROR(("wl%d: wlc_wmf_attach: wlc_bsscfg_cubby_reserve failed\n",
		          wlc->pub->unit));
		MFREE(wlc->osh, wmf, sizeof(wmf_info_t));
		return NULL;
	}

	/* register module */
	if (wlc_module_register(wlc->pub, wmf_iovars, "wmf", wmf, wlc_wmf_doiovar,
		NULL, NULL, wlc_wmf_down)) {
		WL_ERROR(("wl%d: %s wlc_module_register() failed\n",
		          wlc->pub->unit, __FUNCTION__));
		return NULL;
	}

	return wmf;
}

/*
 * WMF module detach function
 */
void
BCMATTACHFN(wlc_wmf_detach)(wmf_info_t *wmf)
{
	wlc_info_t *wlc;

	if (!wmf)
		return;

	wlc = wmf->wlc;
	wlc_module_unregister(wlc->pub, "wmf", wmf);
	MFREE(wlc->osh, wmf, sizeof(wmf_info_t));
}
/*
 * WMF module iovar handler function
 */
static int
wlc_wmf_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid, const char *name,
        void *p, uint plen, void *a, int alen, int vsize, struct wlc_if *wlcif)
{
	wmf_info_t *wmf = (wmf_info_t *)hdl;
	wlc_info_t *wlc;
	wlc_bsscfg_t *bsscfg;
	int32 *ret_int_ptr = (int32 *) a;
	bool bool_val;
	int err = 0;
	int32 int_val = 0;
	bss_wmf_info_t *bssinfo;

	ASSERT(wmf != NULL);
	wlc = wmf->wlc;

	bsscfg = wlc_bsscfg_find_by_wlcif(wlc, wlcif);
	ASSERT(bsscfg != NULL);

	if (plen >= (int)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	bool_val = (int_val != 0) ? TRUE : FALSE;

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	switch (actionid) {
	case IOV_GVAL(IOV_WMF_BSS_ENABLE):
		*ret_int_ptr = bsscfg->wmf_enable;
		break;

	case IOV_SVAL(IOV_WMF_BSS_ENABLE):
		/* Duplicate request */
		if (bsscfg->wmf_enable == bool_val)
			break;

		if (bool_val) {
			if ((err = wlc_wmf_instance_add(wmf, bsscfg)) != BCME_OK) {
				WL_ERROR(("wl%d: %s: Error in creating WMF instance\n",
					wlc->pub->unit, __FUNCTION__));
				break;
			}

			/* Start WMF if it is enabled for this bsscfg */
			wlc_wmf_start(wmf, bsscfg);

			bsscfg->wmf_enable = TRUE;
		} else {
			bsscfg->wmf_enable = FALSE;

			/* Stop WMF if it is enabled for this BSS */
			wlc_wmf_stop(wmf, bsscfg);

			/* Delete WMF instance for this bsscfg */
			wlc_wmf_instance_del(wmf, bsscfg);
		}
		break;

	case IOV_SVAL(IOV_WMF_UCAST_IGMP):
		if (int_val >= OFF && int_val <= ON)
			bsscfg->wmf_ucast_igmp = int_val;
		else
			err = BCME_RANGE;
		break;

	case IOV_GVAL(IOV_WMF_UCAST_IGMP):
		*ret_int_ptr = bsscfg->wmf_ucast_igmp;
		break;

	case IOV_SVAL(IOV_WMF_MCAST_DATA_SENDUP):
		wlc_wmf_mcast_data_sendup(wmf, bsscfg, TRUE, bool_val);
		break;

	case IOV_GVAL(IOV_WMF_MCAST_DATA_SENDUP):
		*ret_int_ptr = wlc_wmf_mcast_data_sendup(wmf, bsscfg, FALSE, FALSE);
		break;

	case IOV_SVAL(IOV_WMF_PSTA_DISABLE):
		bssinfo->wmf_psta_disable = int_val;
		break;

	case IOV_GVAL(IOV_WMF_PSTA_DISABLE):
		*ret_int_ptr = bssinfo->wmf_psta_disable;
		break;

#ifdef WL_IGMP_UCQUERY
	case IOV_SVAL(IOV_WMF_UCAST_IGMP_QUERY):
		if (int_val >= OFF && int_val <= ON)
			bssinfo->wmf_ucast_igmp_query = int_val;
		else
			err = BCME_RANGE;
		break;

	case IOV_GVAL(IOV_WMF_UCAST_IGMP_QUERY):
		*ret_int_ptr = bssinfo->wmf_ucast_igmp_query;
		break;
#endif
#ifdef WL_UCAST_UPNP
	case IOV_SVAL(IOV_WMF_UCAST_UPNP):
		if (int_val >= OFF && int_val <= ON)
			bssinfo->wmf_ucast_upnp = int_val;
		else
			err = BCME_RANGE;
		break;

	case IOV_GVAL(IOV_WMF_UCAST_UPNP):
		*ret_int_ptr = bssinfo->wmf_ucast_upnp;
		break;
#endif
	default:
		err = BCME_UNSUPPORTED;
	}

	return err;
}

#if defined(BCMDBG) || defined(WLTEST)
static int
wlc_wmf_dump(void *context, struct bcmstrbuf *b)
{
	wlc_bsscfg_t *bsscfg = (wlc_bsscfg_t *)context;
	wlc_info_t *wlc = bsscfg->wlc;
	wmf_info_t *wmf = wlc->wmfi;
	emf_cfg_request_t req;
	emf_cfg_mfdb_list_t *list;
	emf_stats_t *emfs;
	struct scb *scb;
	int32 i;
	bss_wmf_info_t *bssinfo;

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	if (!bssinfo->wmf_instance)
		return BCME_ERROR;

	bcm_bprintf(b, "WMF instance wl%d.%d:\n", wlc->pub->unit, WLC_BSSCFG_IDX(bsscfg));

	/*
	 * Dump some counters and statistics
	 */
	(void)snprintf((char *)req.inst_id, sizeof(req.inst_id), "wmf%d", bsscfg->_idx);
	req.command_id = EMFCFG_CMD_EMF_STATS;
	req.size = sizeof(emf_stats_t);
	req.oper_type = EMFCFG_OPER_TYPE_GET;

	emfc_cfg_request_process(bssinfo->emfci, &req);
	if (req.status != EMFCFG_STATUS_SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	emfs = (emf_stats_t *)req.arg;
	bcm_bprintf(b, "McastDataPkts   McastDataFwd    McastFlooded    "
		    "McastDataSentUp McastDataDropped\n");
	bcm_bprintf(b, "%-15d %-15d %-15d %-15d %d\n",
	            emfs->mcast_data_frames, emfs->mcast_data_fwd,
	            emfs->mcast_data_flooded, emfs->mcast_data_sentup,
	            emfs->mcast_data_dropped);
	bcm_bprintf(b, "IgmpPkts        IgmpPktsFwd     "
		    "IgmpPktsSentUp  MFDBCacheHits   MFDBCacheMisses\n");
	bcm_bprintf(b, "%-15d %-15d %-15d %-15d %d\n",
	            emfs->igmp_frames, emfs->igmp_frames_fwd,
	            emfs->igmp_frames_sentup, emfs->mfdb_cache_hits,
	            emfs->mfdb_cache_misses);

	/*
	 * Dump the learned table of scb attached to a group
	 */
	bzero((char *)&req, sizeof(emf_cfg_request_t));
	(void)snprintf((char *)req.inst_id, sizeof(req.inst_id), "wmf%d", bsscfg->_idx);
	req.command_id = EMFCFG_CMD_MFDB_LIST;
	req.size = sizeof(emf_cfg_mfdb_list_t);
	req.oper_type = EMFCFG_OPER_TYPE_GET;

	emfc_cfg_request_process(bssinfo->emfci, &req);
	if (req.status != EMFCFG_STATUS_SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	bcm_bprintf(b, "\nGroup \t\tSCB\t\t\tPkts\n");

	list = (emf_cfg_mfdb_list_t *)req.arg;
	for (i = 0; i < list->num_entries; i++)
	{
		bcm_bprintf(b, "%d.%d.%d.%d \t",
			((list->mfdb_entry[i].mgrp_ip) >> 24) & 0xff,
			((list->mfdb_entry[i].mgrp_ip) >> 16) & 0xff,
			((list->mfdb_entry[i].mgrp_ip) >> 8) & 0xff,
			((list->mfdb_entry[i].mgrp_ip) & 0xff));

		scb = (struct scb *)list->mfdb_entry[i].if_ptr;
		bcm_bprintf(b, MACF, ETHER_TO_MACF(scb->ea));

		bcm_bprintf(b, "\t%d\n", list->mfdb_entry[i].pkts_fwd);
	}
	bcm_bprintf(b, "\n");

	return 0;
}
#endif /* BCMDBG || WLTEST */

/*
 * SCB free notification
 * We call the WMF specific interface delete function
 */

static void
wlc_wmf_scb_deinit(void *context, struct scb *scb)
{
	wmf_info_t *wmf = (wmf_info_t *)context;
	bss_wmf_info_t *bssinfo;
	wlc_bsscfg_t *bsscfg = SCB_BSSCFG(scb);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	if (!bssinfo->wmf_instance)
		return;

	/* Delete the station from  WMF list */
	wlc_wmf_sta_del(wmf, bsscfg, scb);
}

/*
 * Description: This function is called to instantiate emf
 *		and igs instance on enabling WMF on a
 *              bsscfg.
 *
 * Input:       wlc - pointer to the wlc_info_t structure
 *              bsscfg - pointer to the bss configuration
 */
int32
wlc_wmf_instance_add(wmf_info_t *wmf, struct wlc_bsscfg *bsscfg)
{
	wlc_info_t *wlc = wmf->wlc;
	emfc_wrapper_t wmf_emfc;
	igsc_wrapper_t wmf_igsc;
	char inst_id[10];
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	/* Fill in the wmf efmc wrapper functions */
	wmf_emfc.forward_fn = wmf_forward;
	wmf_emfc.sendup_fn = wmf_sendup;
	wmf_emfc.hooks_register_fn = wmf_hooks_register;
	wmf_emfc.hooks_unregister_fn = wmf_hooks_unregister;

	/* Create Instance ID */
	(void)snprintf(inst_id, sizeof(inst_id), "wmf%d", bsscfg->_idx);

	/* Create EMFC instance for WMF */
	bssinfo->emfci = emfc_init((int8 *)inst_id, (void *)bsscfg, wlc->osh, &wmf_emfc);
	if (bssinfo->emfci == NULL) {
		WL_ERROR(("wl%d: %s: WMF EMFC init failed\n",
			wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	WL_WMF(("wl%d: %s: Created EMFC instance\n",
		wlc->pub->unit, __FUNCTION__));

	/* Fill in the wmf igsc wrapper functions */
	wmf_igsc.igs_broadcast = wmf_igs_broadcast;

	/* Create IGSC instance */
	bssinfo->igsci = igsc_init((int8 *)inst_id, (void *)bsscfg, wlc->osh, &wmf_igsc);
	if (bssinfo->igsci == NULL) {
		WL_ERROR(("wl%d: %s: WMF IGSC init failed\n",
			wlc->pub->unit, __FUNCTION__));
		/* Free the earlier allocated resources */
		emfc_exit(bssinfo->emfci);
		return BCME_ERROR;
	}

	WL_WMF(("wl%d: %s: Created IGSC instance\n",
		wlc->pub->unit, __FUNCTION__));

	/* Set the wmf instance pointer inside the bsscfg */
	bssinfo->wmf_instance = TRUE;

	WL_WMF(("wl%d: %s: Addding WLC wmf instance\n",
		wlc->pub->unit, __FUNCTION__));

#if defined(BCMDBG) || defined(WLTEST)
	wlc_dump_register(wlc->pub, "wmf", (dump_fn_t)wlc_wmf_dump, (void *)bsscfg);
#endif /*  defined(BCMDBG) || defined(WLTEST) */

	return BCME_OK;
}

/*
 * Description: This function is called to destroy emf
 *		and igs instances on disabling WMF on
 *              a bsscfg.
 *
 * Input:       bsscfg - pointer to the bss configuration
 */
void
wlc_wmf_instance_del(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg)
{
	wlc_info_t *wlc = wmf->wlc;
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	if (!bssinfo->wmf_instance)
		return;

	WL_WMF(("wl%d: %s\n", wlc->pub->unit, __FUNCTION__));

	/* Free the EMFC instance */
	emfc_exit(bssinfo->emfci);

	/* Free the IGSC instance */
	igsc_exit(bssinfo->igsci);

	/* Make the pointer NULL */
	bssinfo->wmf_instance = FALSE;

	return;
}

/*
 * Description: This function is called to start wmf operation
 *              when a bsscfg is up
 *
 * Input:       bsscfg - pointer to the bss configuration
 */
int
wlc_wmf_start(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg)
{
	wlc_info_t *wlc = wmf->wlc;
	emf_cfg_request_t req;
	char inst_id[10];
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	WL_WMF(("wl%d: %s\n", wlc->pub->unit, __FUNCTION__));

	bzero((char *)&req, sizeof(emf_cfg_request_t));

	(void)snprintf(inst_id, sizeof(inst_id), "wmf%d", bsscfg->_idx);
	strncpy((char *)req.inst_id, inst_id, sizeof(req.inst_id));
	req.inst_id[sizeof(req.inst_id) - 1] = '\0';
	req.command_id = EMFCFG_CMD_EMF_ENABLE;
	req.size = sizeof(bool);
	req.oper_type = EMFCFG_OPER_TYPE_SET;
	*(bool *)req.arg = TRUE;

	emfc_cfg_request_process(bssinfo->emfci, &req);
	if (req.status != EMFCFG_STATUS_SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	return BCME_OK;
}

/*
 * Description: This function is called to stop wmf
 *		operation when bsscfg is down
 *
 * Input:       bsscfg - pointer to the bss configuration
 */
void
wlc_wmf_stop(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg)
{
	wlc_info_t *wlc = wmf->wlc;
	emf_cfg_request_t req;
	char inst_id[10];
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	if (!bssinfo->wmf_instance) {
		return;
	}

	WL_WMF(("wl%d: %s\n", wlc->pub->unit, __FUNCTION__));
	bzero((char *)&req, sizeof(emf_cfg_request_t));

	(void)snprintf(inst_id, sizeof(inst_id), "wmf%d", bsscfg->_idx);
	strncpy((char *)req.inst_id, inst_id, sizeof(req.inst_id));
	req.inst_id[sizeof(req.inst_id) - 1] = '\0';
	req.command_id = EMFCFG_CMD_EMF_ENABLE;
	req.size = sizeof(bool);
	req.oper_type = EMFCFG_OPER_TYPE_SET;
	*(bool *)req.arg = FALSE;

	emfc_cfg_request_process(bssinfo->emfci, &req);
	if (req.status != EMFCFG_STATUS_SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return;
	}
}

/*
 * Description: This function is called to delete a station
 *		to emfc interface list when it is
 *              disassociated to the BSS
 *
 * Input:       bsscfg - pointer to the bss configuration
 *              scb - pointer to the scb
 */
int
wlc_wmf_sta_del(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg, struct scb *scb)
{
	wlc_info_t *wlc = wmf->wlc;
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	WL_WMF(("wl%d: %s\n", wlc->pub->unit, __FUNCTION__));

	if (igsc_sdb_interface_del(bssinfo->igsci, scb) != SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	if (igsc_interface_rtport_del(bssinfo->igsci, scb) != SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	return BCME_OK;
}

/*
 * Description: This function is called by EMFC layer to
 *		forward a frame on an interface
 *
 * Input:       wrapper - pointer to the bss configuration
 *              p     - Pointer to the packet buffer.
 *              mgrp_ip - Multicast destination address.
 *              txif    - Interface to send the frame on.
 *              rt_port    - router port or not
 */
static int32
wmf_forward(void *wrapper, void *p, uint32 mgrp_ip,
            void *txif, bool rt_port)
{
	struct ether_header *eh;
	wlc_bsscfg_t *bsscfg = (wlc_bsscfg_t *)wrapper;
	wlc_info_t *wlc = bsscfg->wlc;
	struct scb *scb;
	osl_t *osh;

	osh = wlc->osh;
	scb = (struct scb *)txif;

	if (!scb || !SCB_ASSOCIATED(scb)) {
		if (p != NULL)
			PKTFREE(osh, p, FALSE);
		WL_WMF(("wl%d: %s: unknown scb %p associated %d\n",
			wlc->pub->unit, __FUNCTION__,
			scb, scb ? SCB_ASSOCIATED(scb) : 0));
		return FAILURE;
	}

	WL_WMF(("wl%d: %s: scb "MACF" is associated\n",
		wlc->pub->unit, __FUNCTION__,
		ETHER_TO_MACF(scb->ea)));

	/* Since we are going to modify the header below and the
	 * packet may be shared we allocate a header buffer and
	 * prepend it to the original sdu.
	 */
	if (PKTSHARED(p)) {
		void *pkt;

		if ((pkt = PKTGET(osh, TXOFF + ETHER_HDR_LEN, TRUE)) == NULL) {
			WL_ERROR(("wl%d: %s: PKTGET headroom %d failed\n",
			          wlc->pub->unit, __FUNCTION__, TXOFF));
			WLCNTINCR(wlc->pub->_cnt->txnobuf);
			WLCIFCNTINCR(scb, txnobuf);
			WLCNTSCBINCR(scb->scb_stats.tx_failures);
			return FAILURE;
		}
		PKTPULL(osh, pkt, TXOFF);

		wlc_pkttag_info_move(bsscfg->wlc, p, pkt);
		PKTSETPRIO(pkt, PKTPRIO(p));

		/* Copy ether header from data buffer to header buffer */
		memcpy(PKTDATA(osh, pkt), PKTDATA(osh, p), ETHER_HDR_LEN);
		PKTPULL(osh, p, ETHER_HDR_LEN);

		/* Chain original sdu onto newly allocated header */
		PKTSETNEXT(osh, pkt, p);
		p = pkt;
	}

	/* Fill in the unicast address of the station into the ether dest */
	eh = (struct ether_header *) PKTDATA(osh, p);
	memcpy(eh->ether_dhost, &scb->ea, ETHER_ADDR_LEN);

	/* Send the packet using bsscfg wlcif */
	wlc_sendpkt(wlc, p, bsscfg->wlcif);

	return SUCCESS;
}

/*
 * Description: This function is called by EMFC layer to
 *		send up a frame
 *
 * Input:       wrapper - pointer to the bss configuration
 *              p     - Pointer to the packet buffer.
 */
static void
wmf_sendup(void *wrapper, void *p)
{
	wlc_bsscfg_t *bsscfg = (wlc_bsscfg_t *)wrapper;
	wlc_info_t *wlc = bsscfg->wlc;
	wmf_info_t *wmf = wlc->wmfi;
	bss_wmf_info_t *bssinfo;

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	WL_WMF(("wl%d: %s\n", wlc->pub->unit, __FUNCTION__));

	if (!bssinfo->wmf_instance) {
		if (p != NULL)
			PKTFREE(wlc->osh, p, FALSE);
		WL_ERROR(("wl%d: %s: Cannot send packet up because WMF instance does not exist\n",
		          wlc->pub->unit, __FUNCTION__));
		return;
	}


	/* Send the packet up */
	wlc_sendup(wlc, bsscfg, NULL, p);
}

/*
 * Description: This function is called to broadcast an IGMP query
 *		to the BSS
 *
 * Input:       wrapper - pointer to the bss configuration
 *		ip      - pointer to the ip header
 *		length  - length of the packet
 *		mgrp_ip - multicast group ip
 */
static int32
wmf_igs_broadcast(void *wrapper, uint8 *ip, uint32 length, uint32 mgrp_ip)
{
	void *pkt;
	wlc_bsscfg_t *bsscfg = (wlc_bsscfg_t *)wrapper;
	wlc_info_t *wlc = bsscfg->wlc;
	wmf_info_t *wmf = wlc->wmfi;
	struct ether_header *eh;
	bss_wmf_info_t *bssinfo;

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	WL_WMF(("wl%d: %s\n", wlc->pub->unit, __FUNCTION__));

	if (!bssinfo->wmf_instance) {
		WL_ERROR(("wl%d: %s: Cannot send IGMP query because WMF instance does not exist\n",
		          wlc->pub->unit, __FUNCTION__));
		return FAILURE;
	}

	/* Allocate the packet, copy the ip part */
	pkt = PKTGET(wlc->osh, length + ETHER_HDR_LEN, TRUE);
	if (pkt == NULL) {
		WL_ERROR(("wl%d: %s: Out of memory allocating IGMP Query packet\n",
			wlc->pub->unit, __FUNCTION__));
		return FAILURE;
	}

	/* Add the ethernet header */
	eh = (struct ether_header *)PKTDATA(wlc->osh, pkt);
	eh->ether_type = hton16(ETHER_TYPE_IP);
	ETHER_FILL_MCAST_ADDR_FROM_IP(eh->ether_dhost, mgrp_ip);

	/* Add bsscfg address as the source ether address */
	memcpy(eh->ether_shost, &bsscfg->cur_etheraddr, ETHER_ADDR_LEN);

	/* Copy the IP part */
	memcpy((uint8 *)eh + ETHER_HDR_LEN, ip, length);

	/* Send the frame on to the bss */
	wlc_sendpkt(wlc, pkt, bsscfg->wlcif);

	return SUCCESS;
}

/*
 * Description: This function is called to register hooks
 *		into wl for packet reception
 *
 * Input:       wrapper  - pointer to the bsscfg
 */
static int32
wmf_hooks_register(void *wrapper)
{
	WL_WMF(("Calling WMF hooks register\n"));

	/*
	 * We dont need to do anything here. WMF enable status will be checked
	 * in the wl before handing off packets to WMF
	 */

	return BCME_OK;
}

/*
 * Description: This function is called to unregister hooks
 *		into wl for packet reception
 *
 * Input:       wrapper  - pointer to the bsscfg
 */
static int32
wmf_hooks_unregister(void *wrapper)
{
	WL_WMF(("Calling WMF hooks unregister\n"));

	/*
	 *  We dont need to do anything here. WMF enable status will be checked
	 * in the wl before handing off packets to WMF
	 */

	return BCME_OK;
}

/*
 * Description: This function is called to do the packet handling by
 *		WMF
 *
 * Input:       bsscfg - pointer to the bss configuration
 *              scb - pointer to the station scb
 *              p - pointer to the packet
 *		frombss - packet from BSS or DS
 */
int
wlc_wmf_packets_handle(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg, struct scb *scb,
	void *p, bool frombss)
{
	wlc_info_t *wlc = wmf->wlc;
	uint8 *iph;
	struct ether_header *eh;
#if defined(WL_IGMP_UCQUERY) || defined(WL_UCAST_UPNP)
	struct scb *scb_ptr;
	struct scb_iter scbiter;
	void *sdu_clone;
	bool ucast_convert = FALSE;
	uint32 dest_ip;
#endif /* WL_IGMP_UCQUERY */
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	/* If the WMF instance is not yet created return */
	if (!bssinfo->wmf_instance)
		return WMF_NOP;

	eh = (struct ether_header *)PKTDATA(wlc->osh, p);
	iph = (uint8 *)eh + ETHER_HDR_LEN;

	/* Only IP packets are handled */
	if (ntoh16(eh->ether_type) != ETHER_TYPE_IP)
		return WMF_NOP;

	/* Change interface to primary interface of proxySTA */
	if (frombss && scb && scb->psta_prim && (!bssinfo->wmf_psta_disable)) {
		scb = scb->psta_prim;
	}

	if (WL_WMF_ON() && scb)
		WL_WMF(("wl%d: %s: to "MACF"\n",
			wlc->pub->unit, __FUNCTION__,
			ETHER_TO_MACF(scb->ea)));

#if defined(WL_IGMP_UCQUERY) || defined(WL_UCAST_UPNP)
	dest_ip = ntoh32(*((uint32 *)(iph + IPV4_DEST_IP_OFFSET)));

	if ((!frombss) && (IP_VER(iph) == IP_VER_4)) {
#ifdef WL_UCAST_UPNP
		ucast_convert = bssinfo->wmf_ucast_upnp && MCAST_ADDR_UPNP_SSDP(dest_ip);
#endif /* WL_UCAST_UPNP */
#ifdef WL_IGMP_UCQUERY
		ucast_convert |= bssinfo->wmf_ucast_igmp_query &&
		        (IPV4_PROT(iph) == IP_PROT_IGMP) &&
			(*(iph + IPV4_HLEN(iph)) == IGMPV2_HOST_MEMBERSHIP_QUERY);
#endif /* WL_IGMP_UCQUERY */
		if (ucast_convert) {
			/* Convert upnp/igmp query to unicast for each assoc STA */
			FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb_ptr) {
				/* Skip sending to proxy interfaces of proxySTA */
				if (scb_ptr->psta_prim != NULL && (!bssinfo->wmf_psta_disable)) {
					continue;
				}
				WL_WMF(("wl%d: %s: send Query to "MACF"\n",
					wlc->pub->unit, __FUNCTION__,
					ETHER_TO_MACF(scb_ptr->ea)));
				if ((sdu_clone = PKTDUP(wlc->osh, p)) == NULL)
					return (WMF_NOP);
				wmf_forward(bsscfg, sdu_clone, 0, scb_ptr, !frombss);
			}
			PKTFREE(wlc->osh, p, TRUE);
			return WMF_TAKEN;
		}
	}
#endif /* defined(WL_IGMP_UCQUERY) || defined(WL_UCAST_UPNP) */

	return (emfc_input(bssinfo->emfci, p, scb, iph, !frombss));
}

/*
 * WMF module down function
 * Does not do anything now
 */
static
int wlc_wmf_down(void *hdl)
{
	return 0;
}

/* Enable/Disable  sending multicast packets to host  for WMF instance */
int
wlc_wmf_mcast_data_sendup(wmf_info_t *wmf, wlc_bsscfg_t *bsscfg, bool set, bool enable)
{
	wlc_info_t *wlc = wmf->wlc;
	emf_cfg_request_t req;
	bss_wmf_info_t *bssinfo;

	BCM_REFERENCE(wlc);

	bssinfo = BSS_WMF_INFO(wmf, bsscfg);
	ASSERT(bssinfo != NULL);

	if (!bssinfo->wmf_instance) {
		WL_ERROR(("wl%d: %s failed: WMF not enabled\n",
			wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}

	bzero((char *)&req, sizeof(emf_cfg_request_t));

	(void)snprintf((char *)req.inst_id, sizeof(req.inst_id), "wmf%d", bsscfg->_idx);
	req.command_id = EMFCFG_CMD_MC_DATA_IND;
	req.size = sizeof(bool);
	req.oper_type = EMFCFG_OPER_TYPE_GET;
	if (set) {
		req.oper_type = EMFCFG_OPER_TYPE_SET;
		*(bool *)req.arg = enable;
	}

	emfc_cfg_request_process(bssinfo->emfci, &req);
	if (req.status != EMFCFG_STATUS_SUCCESS) {
		WL_ERROR(("wl%d: %s failed\n", wlc->pub->unit, __FUNCTION__));
		return BCME_ERROR;
	}
	if (set) {
		return BCME_OK;
	}
	return ((int)(*((bool *)req.arg)));
}

static void
wlc_wmf_bss_deinit(void *ctx, wlc_bsscfg_t *cfg)
{
	wmf_info_t *wmf = (wmf_info_t *)ctx;

	/* Delete WMF instance if it created for this bsscfg */
	wlc_wmf_instance_del(wmf, cfg);
}
