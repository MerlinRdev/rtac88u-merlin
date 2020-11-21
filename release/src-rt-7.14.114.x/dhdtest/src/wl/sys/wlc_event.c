/*
 * Event mechanism
 *
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
 * $Id: wlc_event.c 525520 2015-01-09 23:21:47Z $
 */

/**
 * @file
 * @brief
 * The WLAN driver currently has tight coupling between different components. In particular,
 * components know about each other, and call each others functions, access data, and invoke
 * callbacks. This means that maintenance and new features require changing these
 * relationships. This is fundamentally a tightly coupled system where everything touches
 * many other things.
 *
 * @brief
 * We can reduce the coupling between our features by reducing their need to directly call
 * each others functions, and access each others data. An mechanism for accomplishing this is
 * a generic event signaling mechanism. The event infrastructure enables modules to communicate
 * indirectly through events, rather than directly by calling each others routines and
 * callbacks.
 */


#include <wlc_cfg.h>
#include <typedefs.h>
#include <bcmutils.h>
#include <siutils.h>
#include <bcmendian.h>
#include <wlioctl.h>
#include <wl_dbg.h>
#include <wlc_pub.h>
#include <wl_export.h>
#include <wlc_event.h>
#include <bcm_mpool_pub.h>
#include <d11.h>
#include <wlc_bsscfg.h>
#include <wlc_rate.h>
#include <wlc.h>
#ifdef MSGTRACE
#include <msgtrace.h>
#endif
#ifdef LOGTRACE
#include <logtrace.h>
#endif
#include <wlc_rx.h>

/* Local prototypes */
static void wlc_eventq_timer_cb(void *arg);
static void wlc_eventq_process(void *arg);
static int wlc_eventq_down(void *ctx);
static int wlc_eventq_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid, const char *name,
        void *params, uint p_len, void *arg, int len, int val_size, struct wlc_if *wlcif);

static void wlc_eventq_bss_deinit(void *ctx, wlc_bsscfg_t *cfg);
static void wlc_eventq_bss_updown(void *ctx, bsscfg_up_down_event_data_t *evt);

static void wlc_eventq_enq(wlc_eventq_t *eq, wlc_event_t *e);
static wlc_event_t *wlc_eventq_deq(wlc_eventq_t *eq);
static bool wlc_eventq_avail(wlc_eventq_t *eq);

#ifndef WLNOEIND
static void wlc_event_sendup(wlc_eventq_t *eq, const wlc_event_t *e,
	struct ether_addr *da, struct ether_addr *sa, uint8 *data, uint32 len);
static int wlc_eventq_query_ind_ext(wlc_eventq_t *eq, eventmsgs_ext_t* in_iovar_msg,
	eventmsgs_ext_t* out_iovar_msg, uint8 *mask);
static int wlc_eventq_handle_ind(wlc_eventq_t *eq, wlc_event_t *e);
static int wlc_eventq_register_ind_ext(wlc_eventq_t *eq, eventmsgs_ext_t* iovar_msg, uint8 *mask);
#else
#define wlc_eventq_query_ind_ext(a, b, c, d) 0
#define wlc_eventq_handle_ind(a, b) do {} while (0)
#define wlc_eventq_register_ind_ext(a, b, c) 0
#endif /* WLNOEIND */

enum {
	IOV_EVENT_MSGS = 1,
	IOV_EVENT_MSGS_EXT = 2
};

static const bcm_iovar_t eventq_iovars[] = {
	{"event_msgs", IOV_EVENT_MSGS,
	(IOVF_OPEN_ALLOW|IOVF_RSDB_SET), IOVT_BUFFER, WL_EVENTING_MASK_LEN
	},
	{"event_msgs_ext", IOV_EVENT_MSGS_EXT,
	(IOVF_OPEN_ALLOW|IOVF_RSDB_SET), IOVT_BUFFER, EVENTMSGS_EXT_STRUCT_SIZE
	},
	{NULL, 0, 0, 0, 0}
};

#define WL_EVENTING_MASK_EXT_LEN \
	MAX(WL_EVENTING_MASK_LEN, (ROUNDUP(WLC_E_LAST, NBBY)/NBBY))

typedef void (*wlc_eventq_cb_t)(void *arg);

/* Private data structures */
struct wlc_eventq
{
	wlc_event_t		*head;
	wlc_event_t		*tail;
	wlc_info_t		*wlc;
	void			*wl;
	bool			tpending;
	bool			workpending;
	struct wl_timer		*timer;
	wlc_eventq_cb_t		cb;
	void			*cb_ctx;
	bcm_mp_pool_h		mpool_h;
	uint8			event_inds_mask_len;
	uint8			*event_inds_mask;
};

/*
 * Export functions
 */
wlc_eventq_t*
BCMATTACHFN(wlc_eventq_attach)(wlc_info_t *wlc)
{
	wlc_eventq_t *eq;
	uint eventqsize;

	eventqsize = sizeof(wlc_eventq_t) + WL_EVENTING_MASK_EXT_LEN;

	eq = (wlc_eventq_t*)MALLOCZ(wlc->osh, eventqsize);
	if (eq == NULL) {
		WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
		          wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		goto exit;
	}

	eq->event_inds_mask_len = WL_EVENTING_MASK_EXT_LEN;

	eq->event_inds_mask = (uint8*)((uintptr)eq + sizeof(wlc_eventq_t));

	/* make a reference to get rid of non-debug build error */
	(void)wlc_eventq_avail(eq);

	/* Create memory pool for 'wlc_event_t' data structs. */
	if (bcm_mpm_create_heap_pool(wlc->mem_pool_mgr, sizeof(wlc_event_t),
	                             "event", &eq->mpool_h) != BCME_OK) {
		WL_ERROR(("wl%d: %s: bcm_mpm_create_heap_pool failed\n",
			wlc->pub->unit, __FUNCTION__));
		goto exit;
	}
	eq->cb = wlc_eventq_process;
	eq->cb_ctx = eq;
	eq->wlc = wlc;
	eq->wl = wlc->wl;

	/* utilize the bsscfg cubby machanism to register deinit callback */
	if (wlc_bsscfg_cubby_reserve(wlc, 0, NULL, wlc_eventq_bss_deinit, NULL, eq) < 0) {
		WL_ERROR(("wl%d: %s: wlc_bsscfg_cubby_reserve() failed\n",
		          wlc->pub->unit, __FUNCTION__));
		goto exit;
	}

	if (wlc_bsscfg_updown_register(wlc, wlc_eventq_bss_updown, eq) != BCME_OK) {
		WL_ERROR(("wl%d: %s: wlc_bsscfg_updown_register failed\n",
		          wlc->pub->unit, __FUNCTION__));
		goto exit;
	}

	/* register event module */
	if (wlc_module_register(wlc->pub, eventq_iovars, "eventq", eq, wlc_eventq_doiovar,
	                        NULL, NULL, wlc_eventq_down)) {
		WL_ERROR(("wl%d: %s: event wlc_module_register() failed",
			wlc->pub->unit, __FUNCTION__));
		goto exit;
	}
	if (!(eq->timer = wl_init_timer(eq->wl, wlc_eventq_timer_cb, eq, "eventq"))) {
		WL_ERROR(("wl%d: %s: timer failed\n", wlc->pub->unit, __FUNCTION__));
		goto exit;
	}
	return eq;

exit:
	wlc_eventq_detach(eq);
	return NULL;
}

void
BCMATTACHFN(wlc_eventq_detach)(wlc_eventq_t *eq)
{
	uint eventqsize;
	wlc_info_t *wlc;

	if (eq == NULL)
		return;

	wlc = eq->wlc;

	if (eq->timer) {
		if (eq->tpending) {
			wl_del_timer(eq->wl, eq->timer);
			eq->tpending = FALSE;
		}
		wl_free_timer(eq->wl, eq->timer);
		eq->timer = NULL;
	}

	wlc_module_unregister(wlc->pub, "eventq", eq);

	(void)wlc_bsscfg_updown_unregister(wlc, wlc_eventq_bss_updown, eq);

	ASSERT(wlc_eventq_avail(eq) == FALSE);

	bcm_mpm_delete_heap_pool(wlc->mem_pool_mgr, &eq->mpool_h);

	eventqsize = sizeof(wlc_eventq_t) + WL_EVENTING_MASK_EXT_LEN;

	MFREE(wlc->osh, eq, eventqsize);
}

static int
wlc_eventq_doiovar(void *hdl, const bcm_iovar_t *vi, uint32 actionid, const char *name,
        void *params, uint p_len, void *arg, int len, int val_size, struct wlc_if *wlcif)
{
	wlc_eventq_t *eq = (wlc_eventq_t *)hdl;
	int err = BCME_OK;

	switch (actionid) {
		case IOV_GVAL(IOV_EVENT_MSGS): {
			eventmsgs_ext_t in_iovar_msg, out_iovar_msg;
			bzero(arg, WL_EVENTING_MASK_LEN);
			in_iovar_msg.len = WL_EVENTING_MASK_LEN;
			out_iovar_msg.len = 0;
			err = wlc_eventq_query_ind_ext(eq, &in_iovar_msg, &out_iovar_msg, arg);
			break;
		}

		case IOV_SVAL(IOV_EVENT_MSGS): {
			eventmsgs_ext_t iovar_msg;
			iovar_msg.len = WL_EVENTING_MASK_LEN;
			iovar_msg.command = EVENTMSGS_SET_MASK;
			err = wlc_eventq_register_ind_ext(eq, &iovar_msg, arg);
			break;
		}

		case IOV_GVAL(IOV_EVENT_MSGS_EXT): {
			if (((eventmsgs_ext_t*)params)->ver != EVENTMSGS_VER) {
				err = BCME_VERSION;
				break;
			}
			if (len < (int)((EVENTMSGS_EXT_STRUCT_SIZE +
				((eventmsgs_ext_t*)params)->len))) {
					err = BCME_BUFTOOSHORT;
					break;
			}
			err = wlc_eventq_query_ind_ext(eq, (eventmsgs_ext_t*)params,
				(eventmsgs_ext_t*)arg, ((eventmsgs_ext_t*)arg)->mask);
			break;
		}

		case IOV_SVAL(IOV_EVENT_MSGS_EXT):
			if (((eventmsgs_ext_t*)arg)->ver != EVENTMSGS_VER) {
				err = BCME_VERSION;
				break;
			}
			if (len < (int)((EVENTMSGS_EXT_STRUCT_SIZE +
				((eventmsgs_ext_t*)arg)->len))) {
					err = BCME_BUFTOOSHORT;
					break;
			}
			err = wlc_eventq_register_ind_ext(eq, (eventmsgs_ext_t*)arg,
				((eventmsgs_ext_t*)arg)->mask);
			break;

		default: {
			err = BCME_UNSUPPORTED;
			break;
		}
	}
	return err;
}

static void
_wlc_eventq_process(wlc_eventq_t *eq)
{
	ASSERT(wlc_eventq_avail(eq) == TRUE);
	ASSERT(eq->workpending == FALSE);
	eq->workpending = TRUE;

	if (eq->cb)
		eq->cb(eq->cb_ctx);

	ASSERT(wlc_eventq_avail(eq) == FALSE);
	ASSERT(eq->workpending == TRUE);
	eq->workpending = FALSE;

	eq->tpending = FALSE;
}

static int
BCMUNINITFN(wlc_eventq_down)(void *ctx)
{
	wlc_eventq_t *eq = (wlc_eventq_t *)ctx;
	int callbacks = 0;

	if (eq->tpending && !eq->workpending) {
		if (!wl_del_timer(eq->wl, eq->timer))
			callbacks++;

		_wlc_eventq_process(eq);
	}
	else {
		ASSERT(eq->workpending || wlc_eventq_avail(eq) == FALSE);
	}
	return callbacks;
}

wlc_event_t*
wlc_event_alloc(wlc_eventq_t *eq)
{
	wlc_event_t *e;

	e = (wlc_event_t *) bcm_mp_alloc(eq->mpool_h);
	if (e == NULL)
		return NULL;

	bzero(e, sizeof(wlc_event_t));
	return e;
}

void
wlc_event_free(wlc_eventq_t *eq, wlc_event_t *e)
{
	if (e->data != NULL) {
		wlc_info_t *wlc = eq->wlc;
		BCM_REFERENCE(wlc);
		MFREE(wlc->osh, e->data, e->event.datalen);
	}

	ASSERT(e->next == NULL);

	bcm_mp_free(eq->mpool_h, e);
}

static void
wlc_eventq_enq(wlc_eventq_t *eq, wlc_event_t *e)
{
	ASSERT(e->next == NULL);
	e->next = NULL;

	if (eq->tail) {
		eq->tail->next = e;
		eq->tail = e;
	}
	else
		eq->head = eq->tail = e;

	if (!eq->tpending) {
		eq->tpending = TRUE;
		/* Use a zero-delay timer to trigger
		 * delayed processing of the event.
		 */
		wl_add_timer(eq->wl, eq->timer, 0, 0);
	}
}

static wlc_event_t*
wlc_eventq_deq(wlc_eventq_t *eq)
{
	wlc_event_t *e;

	e = eq->head;
	if (e) {
		eq->head = e->next;
		e->next = NULL;

		if (eq->head == NULL)
			eq->tail = eq->head;
	}
	else if (eq->tpending) {
		/* Timer might have been started within event/timeout handlers,
		 * but, since all the events are processed and event queue
		 * is empty delete the pending timer
		 */
		wl_del_timer(eq->wl, eq->timer);
		eq->tpending = FALSE;
	}

	return e;
}

static bool
wlc_eventq_avail(wlc_eventq_t *eq)
{
	return (eq->head != NULL);
}

#if defined(BCMDBG) || defined(BCMDBG_ERR)
static void
wlc_event_print(wlc_eventq_t *eq, wlc_event_t *e)
{
	wlc_info_t *wlc = eq->wlc;
	uint msg = e->event.event_type;
	struct ether_addr *addr = e->addr;
	uint result = e->event.status;
	char eabuf[ETHER_ADDR_STR_LEN];
#if defined(BCMDBG) || defined(WLMSG_INFORM)
	uint auth_type = e->event.auth_type;
	const char *auth_str;
	const char *event_name;
	uint status = e->event.reason;
	char ssidbuf[SSID_FMT_BUF_LEN];
#endif /* BCMDBG || WLMSG_INFORM */

#if defined(BCMDBG) || defined(WLMSG_INFORM)
	event_name = bcmevent_get_name(msg);
#endif /* BCMDBG || WLMSG_INFORM */

	BCM_REFERENCE(wlc);

	if (addr != NULL)
		bcm_ether_ntoa(addr, eabuf);
	else
		strncpy(eabuf, "<NULL>", 7);

	switch (msg) {
	case WLC_E_START:
	case WLC_E_DEAUTH:
	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:
	case WLC_E_DISASSOC:
	case WLC_E_EAPOL_MSG:
	case WLC_E_BCNRX_MSG:
	case WLC_E_BCNSENT_IND:
	case WLC_E_ROAM_PREP:
	case WLC_E_BCNLOST_MSG:
	case WLC_E_PROBREQ_MSG:
#ifdef WLP2P
	case WLC_E_PROBRESP_MSG:
	case WLC_E_P2P_PROBREQ_MSG:
#endif
#if defined(NDIS) && (NDISVER >= 0x0620)
	case WLC_E_ASSOC_IND_NDIS:
	case WLC_E_REASSOC_IND_NDIS:
	case WLC_E_IBSS_COALESCE:
#endif /* NDIS && (NDISVER >= 0x0620) */
	case WLC_E_AUTHORIZED:
	case WLC_E_PROBREQ_MSG_RX:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s\n",
		           WLCWLUNIT(wlc), event_name, eabuf));
		break;

	case WLC_E_ASSOC:
	case WLC_E_REASSOC:
		if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, SUCCESS\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_TIMEOUT) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, TIMEOUT\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_ABORT) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, ABORT\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_NO_ACK) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, NO_ACK\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_UNSOLICITED) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, UNSOLICITED\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_FAIL) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, FAILURE, status %d\n",
			           WLCWLUNIT(wlc), event_name, eabuf, (int)status));
		} else {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, unexpected status %d\n",
			           WLCWLUNIT(wlc), event_name, eabuf, (int)result));
		}
		break;

	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, reason %d\n",
		           WLCWLUNIT(wlc), event_name, eabuf, (int)status));
		break;

#if defined(BCMDBG) || defined(WLMSG_INFORM)
	case WLC_E_AUTH:
	case WLC_E_AUTH_IND: {
		char err_msg[32];

		if (auth_type == DOT11_OPEN_SYSTEM) {
			auth_str = "Open System";
		} else if (auth_type == DOT11_SHARED_KEY) {
			auth_str = "Shared Key";
		} else {
			(void)snprintf(err_msg, sizeof(err_msg), "AUTH unknown: %d",
				(int)auth_type);
			auth_str = err_msg;
		}

		if (msg == WLC_E_AUTH_IND) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, %s\n",
			           WLCWLUNIT(wlc), event_name, eabuf, auth_str));
		} else if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, %s, SUCCESS\n",
			           WLCWLUNIT(wlc), event_name, eabuf, auth_str));
		} else if (result == WLC_E_STATUS_TIMEOUT) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, %s, TIMEOUT\n",
			           WLCWLUNIT(wlc), event_name, eabuf, auth_str));
		} else if (result == WLC_E_STATUS_FAIL) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, %s, FAILURE, status %d\n",
			           WLCWLUNIT(wlc), event_name, eabuf, auth_str, (int)status));
		}
		break;
	}
#endif /* BCMDBG || WLMSG_INFORM */
	case WLC_E_JOIN:
	case WLC_E_ROAM:
	case WLC_E_BSSID:
	case WLC_E_SET_SSID:
		if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_FAIL) {
			WL_INFORM(("wl%d: MACEVENT: %s, failed\n",
			           WLCWLUNIT(wlc), event_name));
		} else if (result == WLC_E_STATUS_NO_NETWORKS) {
			WL_INFORM(("wl%d: MACEVENT: %s, no networks found\n",
			           WLCWLUNIT(wlc), event_name));
		} else if (result == WLC_E_STATUS_ABORT) {
			WL_INFORM(("wl%d: MACEVENT: %s, ABORT\n",
			           WLCWLUNIT(wlc), event_name));
		} else {
			WL_INFORM(("wl%d: MACEVENT: %s, unexpected status %d\n",
			           WLCWLUNIT(wlc), event_name, (int)result));
		}
		break;

	case WLC_E_BEACON_RX:
		if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s, SUCCESS\n",
			           WLCWLUNIT(wlc), event_name));
		} else if (result == WLC_E_STATUS_FAIL) {
			WL_INFORM(("wl%d: MACEVENT: %s, FAIL\n",
			           WLCWLUNIT(wlc), event_name));
		} else {
			WL_INFORM(("wl%d: MACEVENT: %s, result %d\n",
			           WLCWLUNIT(wlc), event_name, result));
		}
		break;


	case WLC_E_LINK:
		WL_INFORM(("wl%d: MACEVENT: %s %s\n",
		           WLCWLUNIT(wlc), event_name,
		           (e->event.flags&WLC_EVENT_MSG_LINK)?"UP":"DOWN"));
		break;

	case WLC_E_MIC_ERROR:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, Group: %s, Flush Q: %s\n",
		           WLCWLUNIT(wlc), event_name, eabuf,
		           (e->event.flags&WLC_EVENT_MSG_GROUP)?"Yes":"No",
		           (e->event.flags&WLC_EVENT_MSG_FLUSHTXQ)?"Yes":"No"));
		break;

	case WLC_E_ICV_ERROR:
	case WLC_E_UNICAST_DECODE_ERROR:
	case WLC_E_MULTICAST_DECODE_ERROR:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s\n",
		           WLCWLUNIT(wlc), event_name, eabuf));
		break;

	case WLC_E_TXFAIL:
		/* TXFAIL messages are too numerous for WL_INFORM() */
		break;

	case WLC_E_COUNTRY_CODE_CHANGED: {
		char cstr[16];
		memset(cstr, 0, sizeof(cstr));
		memcpy(cstr, e->data, MIN(e->event.datalen, sizeof(cstr) - 1));
		WL_INFORM(("wl%d: MACEVENT: %s New Country: %s\n", WLCWLUNIT(wlc),
		           event_name, cstr));
		break;
	}

	case WLC_E_RETROGRADE_TSF:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s\n",
		           WLCWLUNIT(wlc), event_name, eabuf));
		break;

#ifdef WIFI_ACT_FRAME
	case WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE:
#endif
	case WLC_E_SCAN_COMPLETE:
		if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s, SUCCESS\n",
			           WLCWLUNIT(wlc), event_name));
		} else if (result == WLC_E_STATUS_ABORT) {
			WL_INFORM(("wl%d: MACEVENT: %s, ABORTED\n",
			           WLCWLUNIT(wlc), event_name));
		} else {
			WL_INFORM(("wl%d: MACEVENT: %s, result %d\n",
			           WLCWLUNIT(wlc), event_name, result));
		}
		break;

	case WLC_E_AUTOAUTH:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, result %d\n",
		           WLCWLUNIT(wlc), event_name, eabuf, (int)result));
		break;

	case WLC_E_ADDTS_IND:
		if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, SUCCESS\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_TIMEOUT) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, TIMEOUT\n",
			           WLCWLUNIT(wlc), event_name, eabuf));
		} else if (result == WLC_E_STATUS_FAIL) {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, FAILURE, status %d\n",
			           WLCWLUNIT(wlc), event_name, eabuf, (int)status));
		} else {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, unexpected status %d\n",
			           WLCWLUNIT(wlc), event_name, eabuf, (int)result));
		}
		break;

	case WLC_E_DELTS_IND:
		if (result == WLC_E_STATUS_SUCCESS) {
			WL_INFORM(("wl%d: MACEVENT: %s success ...\n",
			           WLCWLUNIT(wlc), event_name));
		} else if (result == WLC_E_STATUS_UNSOLICITED) {
			WL_INFORM(("wl%d: MACEVENT: DELTS unsolicited %s\n",
			           WLCWLUNIT(wlc), event_name));
		} else {
			WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, unexpected status %d\n",
			           WLCWLUNIT(wlc), event_name, eabuf, (int)result));
		}
		break;

	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_NET_LOST:
		WL_INFORM(("wl%d: PFNEVENT: %s, SSID %s, SSID len %d\n",
		           WLCWLUNIT(wlc), event_name,
		           (wlc_format_ssid(ssidbuf, e->data, e->event.datalen), ssidbuf),
		           e->event.datalen));
		break;

	case WLC_E_PSK_SUP:
		WL_INFORM(("wl%d: MACEVENT: %s, state %d, reason %d\n", WLCWLUNIT(wlc),
		           event_name, result, status));
		break;



	case WLC_E_PSM_WATCHDOG:
		WL_INFORM(("wl%d: MACEVENT: %s, psmdebug 0x%x, phydebug 0x%x, psm_brc 0x%x\n",
		           WLCWLUNIT(wlc), event_name, result, status, auth_type));
		break;

	case WLC_E_TRACE:
		/* We don't want to trace the trace event */
		break;

#ifdef WIFI_ACT_FRAME
	case WLC_E_ACTION_FRAME_COMPLETE:
		WL_INFORM(("wl%d: MACEVENT: %s status: %s\n", WLCWLUNIT(wlc), event_name,
		           (result == WLC_E_STATUS_NO_ACK?"NO ACK":"ACK")));
		break;
#endif /* WIFI_ACT_FRAME */

	/*
	 * Events that don't require special decoding
	 */
	case WLC_E_ASSOC_REQ_IE:
	case WLC_E_ASSOC_RESP_IE:
	case WLC_E_PMKID_CACHE:
	case WLC_E_PRUNE:
	case WLC_E_RADIO:
	case WLC_E_IF:
	case WLC_E_EXTLOG_MSG:
	case WLC_E_RSSI:
	case WLC_E_ESCAN_RESULT:
	case WLC_E_DCS_REQUEST:
	case WLC_E_CSA_COMPLETE_IND:
#ifdef WIFI_ACT_FRAME
	case WLC_E_ACTION_FRAME:
	case WLC_E_ACTION_FRAME_RX:
#endif
#ifdef WLP2P
	case WLC_E_P2P_DISC_LISTEN_COMPLETE:
#endif
#if defined(NDIS) && (NDISVER >= 0x0620)
	case WLC_E_PRE_ASSOC_IND:
	case WLC_E_PRE_REASSOC_IND:
	case WLC_E_AP_STARTED:
	case WLC_E_DFS_AP_STOP:
	case WLC_E_DFS_AP_RESUME:
#endif
#if defined(NDIS) && (NDISVER >= 0x0630)
	case WLC_E_ACTION_FRAME_RX_NDIS:
	case WLC_E_AUTH_REQ:
	case WLC_E_SPEEDY_RECREATE_FAIL:
	case WLC_E_ASSOC_RECREATED:
#endif /* NDIS && (NDISVER >= 0x0630) */
#ifdef PROP_TXSTATUS
	case WLC_E_FIFO_CREDIT_MAP:
	case WLC_E_BCMC_CREDIT_SUPPORT:
#endif
#ifdef P2PO
	case WLC_E_SERVICE_FOUND:
	case WLC_E_P2PO_ADD_DEVICE:
	case WLC_E_P2PO_DEL_DEVICE:
#endif
#if defined(P2PO) || defined(ANQPO)
	case WLC_E_GAS_FRAGMENT_RX:
	case WLC_E_GAS_COMPLETE:
#endif
	case WLC_E_WAKE_EVENT:
	case WLC_E_NATIVE:
		WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#ifdef WLTDLS
	case WLC_E_TDLS_PEER_EVENT:
		WL_INFORM(("wl%d: MACEVENT: %s, MAC %s, reason %d\n",
			WLCWLUNIT(wlc), event_name, eabuf, (int)status));
		break;
#endif /* WLTDLS */
#if defined(WLPKTDLYSTAT) && defined(WLPKTDLYSTAT_IND)
	case WLC_E_PKTDELAY_IND:
		WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#endif /* defined(WLPKTDLYSTAT) && defined(WLPKTDLYSTAT_IND) */
#if defined(WL_PROXDETECT)
	case WLC_E_PROXD:
		WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#endif /* defined(WL_PROXDETECT) */

	case WLC_E_CCA_CHAN_QUAL:
		WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
	case WLC_E_PSTA_PRIMARY_INTF_IND:
		WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#ifdef WLINTFERSTAT
	case WLC_E_TXFAIL_THRESH:
	WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#endif  /* WLINTFERSTAT */
	case WLC_E_RMC_EVENT:
		WL_INFORM(("wl%d: MACEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#ifdef DPSTA
	case WLC_E_DPSTA_INTF_IND:
		WL_INFORM(("wl%d: DPSTAEVENT: %s\n", WLCWLUNIT(wlc), event_name));
		break;
#endif /* DPSTA */
	default:
		WL_INFORM(("wl%d: MACEVENT: UNSUPPORTED %d, MAC %s, result %d, status %d,"
			" auth %d\n", WLCWLUNIT(wlc), msg, eabuf, (int)result, (int)status,
			(int)auth_type));
		break;
	}
}
#endif /* BCMDBG || BCMDBG_ERR */

/* Immediate event processing.
 * Process the event and any events generated during the processing,
 * and queue all these events for deferred processing as well.
 */
void
wlc_event_process(wlc_eventq_t *eq, wlc_event_t *e)
{
#if defined(BCMDBG) || defined(BCMDBG_ERR)
	wlc_event_print(eq, e);
#endif /* BCMDBG || BCMDBG_ERR */

	/* deliver event to the port OS right now */
	wl_event_sync(eq->wl, e->event.ifname, e);

	/* queue the event for 0 delay timer delivery */
	wlc_eventq_enq(eq, e);
}

/* Deferred event processing */
static void
wlc_eventq_process(void *arg)
{
	wlc_eventq_t *eq = (wlc_eventq_t *)arg;
	wlc_event_t *etmp;

	while ((etmp = wlc_eventq_deq(eq))) {
		/* Perform OS specific event processing */
		wl_event(eq->wl, etmp->event.ifname, etmp);
		/* Perform common event notification */
		wlc_eventq_handle_ind(eq, etmp);
		wlc_event_free(eq, etmp);
	}
}

#ifndef WLNOEIND
static int
wlc_eventq_register_ind_ext(wlc_eventq_t *eq, eventmsgs_ext_t* iovar_msg, uint8 *mask)
{
	int i;
	int current_event_mask_size;


	/*  re-using the event_msgs_ext iovar struct for convenience, */
	/*	but only using some fields -- */
	/*	if changed remember to check callers */

	current_event_mask_size = MIN(eq->event_inds_mask_len, iovar_msg->len);

	switch (iovar_msg->command) {
		case EVENTMSGS_SET_BIT:
			for (i = 0; i < current_event_mask_size; i++)
				eq->event_inds_mask[i] |= mask[i];
			break;
		case EVENTMSGS_RESET_BIT:
			for (i = 0; i < current_event_mask_size; i++)
				eq->event_inds_mask[i] &= mask[i];
			break;
		case EVENTMSGS_SET_MASK:
			bcopy(mask, eq->event_inds_mask, current_event_mask_size);
			break;
		default:
			return BCME_BADARG;
	};

	wlc_enable_probe_req(
		eq->wlc,
		PROBE_REQ_EVT_MASK,
		wlc_eventq_test_ind(eq, WLC_E_PROBREQ_MSG)? PROBE_REQ_EVT_MASK:0);
#if defined(MSGTRACE) || defined(LOGTRACE)
	if (isset(eq->event_inds_mask, WLC_E_TRACE)) {
#ifdef MSGTRACE
		msgtrace_start();
#endif
#ifdef LOGTRACE
		logtrace_start();
#endif
	} else {
#ifdef MSGTRACE
		msgtrace_stop();
#endif
#ifdef LOGTRACE
		logtrace_stop();
#endif
	}
#endif /* MSGTRACE || LOGTRACE */

	return 0;
}

static int
wlc_eventq_query_ind_ext(wlc_eventq_t *eq, eventmsgs_ext_t* in_iovar_msg,
	eventmsgs_ext_t* out_iovar_msg, uint8 *mask)
{
	out_iovar_msg->len = MIN(eq->event_inds_mask_len, in_iovar_msg->len);
	out_iovar_msg->maxgetsize = eq->event_inds_mask_len;
	bcopy(eq->event_inds_mask, mask, out_iovar_msg->len);
	return 0;
}

int
wlc_eventq_test_ind(wlc_eventq_t *eq, int et)
{
	return isset(eq->event_inds_mask, et);
}

static int
wlc_eventq_handle_ind(wlc_eventq_t *eq, wlc_event_t *e)
{
	wlc_bsscfg_t *cfg;
	struct ether_addr *da;
	struct ether_addr *sa;

	cfg = wlc_bsscfg_find_by_wlcif(eq->wlc, e->wlcif);
	ASSERT(cfg != NULL);

	if (!cfg) {
		WL_ERROR(("wl%d: wlc_eventq_handle_ind: cfg is null\n", eq->wlc->pub->unit));
		return BCME_ERROR;
	}

	da = &cfg->cur_etheraddr;
	sa = &cfg->cur_etheraddr;

	if (wlc_eventq_test_ind(eq, e->event.event_type))
		wlc_event_sendup(eq, e, da, sa, e->data, e->event.datalen);
	return 0;
}

void
wlc_eventq_flush(wlc_eventq_t *eq)
{
	if (eq->cb)
		eq->cb(eq->cb_ctx);
	if (eq->tpending) {
		wl_del_timer(eq->wl, eq->timer);
		eq->tpending = FALSE;
	}
}
#endif /* !WLNOEIND */

static void
wlc_eventq_bss_deinit(void *ctx, wlc_bsscfg_t *cfg)
{
	struct wlc_eventq *eq = (struct wlc_eventq *)ctx;

	wlc_eventq_flush(eq);
}

static void
wlc_eventq_bss_updown(void *ctx, bsscfg_up_down_event_data_t *evt)
{
	if (!evt->up) {
		struct wlc_eventq *eq = (struct wlc_eventq *)ctx;

		wlc_eventq_flush(eq);
	}
}

/*
 * Local Functions
 */
static void
wlc_eventq_timer_cb(void *arg)
{
	struct wlc_eventq* eq = (struct wlc_eventq*)arg;

	ASSERT(eq->tpending == TRUE);

	_wlc_eventq_process(eq);

	ASSERT(eq->tpending == FALSE);
}


#ifndef WLNOEIND
/* Abandonable helper function for PROP_TXSTATUS */
static void
wlc_event_mark_packet(wlc_info_t *wlc, void *p)
{
#ifdef PROP_TXSTATUS
	if (PROP_TXSTATUS_ENAB(wlc->pub)) {
		PKTSETTYPEEVENT(wlc->osh, p);
		/* this is implied for event packets anyway */
		PKTSETNODROP(wlc->osh, p);
	}
#endif
}

static void
wlc_assign_event_msg(wlc_info_t *wlc, wl_event_msg_t *msg, const wlc_event_t *e,
	uint8 *data, uint32 len)
{
	void *databuf;

	ASSERT(msg && e);

	/* translate the wlc event into bcm event msg */
	msg->version = hton16(BCM_EVENT_MSG_VERSION);
	msg->event_type = hton32(e->event.event_type);
	msg->status = hton32(e->event.status);
	msg->reason = hton32(e->event.reason);
	msg->auth_type = hton32(e->event.auth_type);
	msg->datalen = hton32(e->event.datalen);
	msg->flags = hton16(e->event.flags);
	bzero(msg->ifname, sizeof(msg->ifname));
	strncpy(msg->ifname, e->event.ifname, sizeof(msg->ifname) - 1);
	msg->ifidx = e->event.ifidx;
	msg->bsscfgidx = e->event.bsscfgidx;

	if (e->addr)
		bcopy(e->event.addr.octet, msg->addr.octet, ETHER_ADDR_LEN);

	databuf = (char *)(msg + 1);
	if (len)
		bcopy(data, databuf, len);
}

#ifndef WLNOEIND
static void
wlc_event_sendup(wlc_eventq_t *eq, const wlc_event_t *e,
	struct ether_addr *da, struct ether_addr *sa, uint8 *data, uint32 len)
{
	wlc_info_t *wlc = eq->wlc;
	void *p;
	char *ptr;
	bcm_event_t *msg;
	uint pktlen;
	wlc_bsscfg_t *cfg;
	struct scb *scb = NULL;

	BCM_REFERENCE(wlc);

	ASSERT(e != NULL);
	ASSERT(e->wlcif != NULL);


	pktlen = sizeof(bcm_event_t) + len + 2;
#ifdef DONGLEBUILD
	pktlen += (BCMEXTRAHDROOM + 2);
#endif
	if ((p = PKTGET(wlc->osh, pktlen, FALSE)) == NULL) {
		WL_ERROR(("wl%d: wlc_event_sendup: failed to get a pkt\n", wlc->pub->unit));
		return;
	}

	ASSERT(ISALIGNED(PKTDATA(wlc->osh, p), sizeof(uint32)));

#ifdef DONGLEBUILD
	/* make room for headers; ensure we start on an odd 16 bit offset */
	PKTPULL(wlc->osh, p, BCMEXTRAHDROOM + 2);
#endif
	msg = (bcm_event_t *) PKTDATA(wlc->osh, p);

	bcopy(da, &msg->eth.ether_dhost, ETHER_ADDR_LEN);
	bcopy(sa, &msg->eth.ether_shost, ETHER_ADDR_LEN);

	/* Set the locally administered bit on the source mac address if both
	 * SRC and DST mac addresses are the same. This prevents the downstream
	 * bridge from dropping the packet.
	 * Clear it if both addresses are the same and it's already set.
	 */
	if (!bcmp(&msg->eth.ether_shost, &msg->eth.ether_dhost, ETHER_ADDR_LEN))
		ETHER_TOGGLE_LOCALADDR(&msg->eth.ether_shost);

	msg->eth.ether_type = hton16(ETHER_TYPE_BRCM);

	/* BCM Vendor specific header... */
	msg->bcm_hdr.subtype = hton16(BCMILCP_SUBTYPE_VENDOR_LONG);
	msg->bcm_hdr.version = BCMILCP_BCM_SUBTYPEHDR_VERSION;
	bcopy(BRCM_OUI, &msg->bcm_hdr.oui[0], DOT11_OUI_LEN);
	/* vendor spec header length + pvt data length (private indication
	 * hdr + actual message itself)
	 */
	msg->bcm_hdr.length = hton16(BCMILCP_BCM_SUBTYPEHDR_MINLENGTH +
	                             BCM_MSG_LEN +
	                             (uint16)len);
	msg->bcm_hdr.usr_subtype = hton16(BCMILCP_BCM_SUBTYPE_EVENT);

	/* update the event struct */
	wlc_assign_event_msg(wlc, &msg->event, e, data, len);

	/* fixup lengths */
	msg->bcm_hdr.length = ntoh16(msg->bcm_hdr.length);
	msg->bcm_hdr.length += sizeof(wl_event_msg_t);
	msg->bcm_hdr.length = hton16(msg->bcm_hdr.length);

	PKTSETLEN(wlc->osh, p, (sizeof(bcm_event_t) + len + 2));

	ptr = (char *)(msg + 1);
	/* Last 2 bytes of the message are 0x00 0x00 to signal that there are
	 * no ethertypes which are following this
	 */
	ptr[len + 0] = 0x00;
	ptr[len + 1] = 0x00;

	wlc_event_mark_packet(wlc, p);

	cfg = wlc_bsscfg_find_by_wlcif(wlc, e->wlcif);
	ASSERT(cfg != NULL);

	if (e->wlcif->type == WLC_IFTYPE_WDS)
		scb = e->wlcif->u.scb;

	wlc_sendup_event(wlc, cfg, scb, p);
}
#endif /* !WLNOEIND */

#if defined(MSGTRACE) || defined(LOGTRACE)
#include <rte_dev.h>
void
wlc_event_sendup_trace(wlc_info_t *wlc, hnd_dev_t *wl_rtedev, uint8 *hdr, uint16 hdrlen,
                       uint8 *buf, uint16 buflen)
{
	void *p;
	bcm_event_t *msg;
	char *ptr, *databuf;
	struct lbuf *lb;
	uint16 len;
	osl_t *osh = wlc->osh;
	hnd_dev_t *busdev = wl_rtedev->chained;

	if (busdev == NULL)
		return;

	if (! wlc_eventq_test_ind(wlc->eventq, WLC_E_TRACE))
		return;

	len = hdrlen + buflen;
	ASSERT(len < (wlc->pub->tunables->rxbufsz - sizeof(bcm_event_t) - 2));

	if ((p = PKTGET(osh, wlc->pub->tunables->rxbufsz, FALSE)) == NULL) {
		return;
	}

	ASSERT(ISALIGNED(PKTDATA(osh, p), sizeof(uint32)));

	/* make room for headers; ensure we start on an odd 16 bit offset */
	PKTPULL(osh, p, BCMEXTRAHDROOM + 2);

	msg = (bcm_event_t *) PKTDATA(osh, p);

	msg->eth.ether_type = hton16(ETHER_TYPE_BRCM);

	/* BCM Vendor specific header... */
	msg->bcm_hdr.subtype = hton16(BCMILCP_SUBTYPE_VENDOR_LONG);
	msg->bcm_hdr.version = BCMILCP_BCM_SUBTYPEHDR_VERSION;
	bcopy(BRCM_OUI, &msg->bcm_hdr.oui[0], DOT11_OUI_LEN);
	/* vendor spec header length + pvt data length (private indication hdr + actual message
	 * itself)
	 */
	msg->bcm_hdr.length = hton16(BCMILCP_BCM_SUBTYPEHDR_MINLENGTH + BCM_MSG_LEN + (uint16)len);
	msg->bcm_hdr.usr_subtype = hton16(BCMILCP_BCM_SUBTYPE_EVENT);

	PKTSETLEN(osh, p, (sizeof(bcm_event_t) + len + 2));

	/* update the event struct */
	/* translate the wlc event into bcm event msg */
	msg->event.version = hton16(BCM_EVENT_MSG_VERSION);
	msg->event.event_type = hton32(WLC_E_TRACE);
	msg->event.status = hton32(WLC_E_STATUS_SUCCESS);
	msg->event.reason = 0;
	msg->event.auth_type = 0;
	msg->event.datalen = hton32(len);
	msg->event.flags = 0;
	msg->event.addr = wlc->perm_etheraddr;
	bzero(msg->event.ifname, sizeof(msg->event.ifname));
	msg->event.ifidx = 0;
	msg->event.flags |= WLC_EVENT_MSG_UNKIF;
	msg->event.bsscfgidx = 0;
	msg->event.flags |= WLC_EVENT_MSG_UNKBSS;

	/* fixup lengths */
	msg->bcm_hdr.length = ntoh16(msg->bcm_hdr.length);
	msg->bcm_hdr.length += sizeof(wl_event_msg_t);
	msg->bcm_hdr.length = hton16(msg->bcm_hdr.length);

	PKTSETLEN(osh, p, (sizeof(bcm_event_t) + len + 2));

	/* Copy the data */
	databuf = (char *)(msg + 1);
	bcopy(hdr, databuf, hdrlen);
	bcopy(buf, databuf+hdrlen, buflen);

	ptr = (char *)databuf;

	PKTSETMSGTRACE(p, TRUE);

	/* Last 2 bytes of the message are 0x00 0x00 to signal that there are no ethertypes which
	 * are following this
	 */
	ptr[len+0] = 0x00;
	ptr[len+1] = 0x00;
	lb = PKTTONATIVE(osh, p);

	if (busdev->ops->xmit(NULL, busdev, lb) != 0) {
		lb_free(lb);
	}
}
#endif /* MSGTRACE */

int
wlc_eventq_set_ind(wlc_eventq_t* eq, uint et, bool enab)
{
	if (et >= WLC_E_LAST)
		return -1;
	if (enab)
		setbit(eq->event_inds_mask, et);
	else
		clrbit(eq->event_inds_mask, et);

	if (et == WLC_E_PROBREQ_MSG)
		wlc_enable_probe_req(eq->wlc, PROBE_REQ_EVT_MASK, enab? PROBE_REQ_EVT_MASK:0);

	return 0;
}
#endif /* !WLNOEIND */
