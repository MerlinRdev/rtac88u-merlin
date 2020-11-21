/*
 * Common interface to the 802.11 Station Control Block (scb) structure
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
 * $Id: wlc_scb.c 530263 2015-01-29 18:21:14Z $
 */

/**
 * @file
 * @brief
 * SCB is a per-station data structure that is stored in the wl driver. SCB container provides a
 * mechanism through which different wl driver modules can each allocate and maintain private space
 * in the scb used for their own purposes. The scb subsystem (wlc_scb.c) does not need to know
 * anything about the different modules that may have allocated space in scb. It can also be used
 * by per-port code immediately after wlc_attach() has been done (but before wlc_up()).
 *
 * - "container" refers to the entire space within scb that can be allocated opaquely to other
 *   modules.
 * - "cubby" refers to the per-module private space in the container.
 */


#include <wlc_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>

#include <d11.h>
#include <wlc_rate.h>
#include <wlc_pub.h>
#include <wlc_keymgmt.h>
#include <wlc_bsscfg.h>
#include <wlc.h>
#include <wlc_scb.h>
#include <wlc_scb_ratesel.h>


#ifdef PROP_TXSTATUS
#include <wl_export.h>
#include <wlfc_proto.h>
#include <wl_wlfc.h>
#include <wlc_apps.h>
#ifdef WLAMPDU
#include <wlc_ampdu.h>
#endif /* WLAMPDU */
#endif /* PROP_TXSTATUS */

#include <wlc_macfltr.h>
#include <wlc_lq.h>

#define SCB_MAGIC 0x0505a5a5

#define	SCBHASHINDEX(hash, id)	((id[3] ^ id[4] ^ id[5]) % (hash))

#ifdef SCBFREELIST
#ifdef INT_SCB_OPT
#error "SCBFREELIST incompatible with INT_SCB_OPT"
/* To make it compatible, freelist needs to track internal vs external */
#endif /* INT_SCB_OPT */
#endif /* SCBFREELIST */
/** structure for storing per-cubby client info */
typedef struct cubby_info {
	scb_cubby_init_t	fn_init;	/* fn called during scb malloc */
	scb_cubby_deinit_t	fn_deinit;	/* fn called during scb free */
	scb_cubby_dump_t 	fn_dump;	/* fn called during scb dump */
} cubby_info_t;

typedef struct cubby_info_ctx {
	void			*context;	/* context to be passed to all cb fns */
} cubby_info_ctx_t;

/** structure for storing public and private global scb module state */
struct scb_module {
	wlc_info_t	*wlc;			/* global wlc info handle */
	uint16		nscb;			/* total number of allocated scbs */
	uint16		aging_threshold;	/* min # scbs before starting aging */
	uint		scbtotsize;		/* total scb size including container */
	uint 		ncubby;			/* current num of cubbies */
	cubby_info_t	*cubby_info;		/* cubby client info */
	cubby_info_ctx_t *cubby_info_ctx;	/* cubby client context info */
#ifdef SCBFREELIST
	struct scb      *free_list;		/* Free list of SCBs */
#endif
	int		cfgh;			/* scb bsscfg cubby handle */
	bcm_notif_h 	scb_state_notif_hdl;	/* scb state notifier handle. */
#ifdef SCB_MEMDBG
	uint32		scballoced;		/* how many scb calls to 'wlc_scb_allocmem' */
	uint32		scbfreed;		/* how many scb calls to 'wlc_scb_freemem' */
	uint32		freelistcount;		/* how many scb's are in free_list */
#endif /* SCB_MEMDBG */
};

/** station control block - one per remote MAC address */
struct scb_info {
	struct scb 	*scbpub;	/* public portion of scb */
	struct scb_info *hashnext;	/* pointer to next scb under same hash entry */
	struct scb_info	*next;		/* pointer to next allocated scb */
#ifdef SCB_MEMDBG
	struct scb_info *hashnext_copy;
	struct scb_info *next_copy;
#endif
};

static void wlc_scb_hash_add(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int bandunit,
	struct scb *scb);
static void wlc_scb_hash_del(wlc_info_t *wlc, wlc_bsscfg_t *cfg,
	struct scb *scbd);
static void wlc_scb_list_add(wlc_info_t *wlc, wlc_bsscfg_t *cfg,
	struct scb *scb);
static void wlc_scb_list_del(wlc_info_t *wlc, wlc_bsscfg_t *cfg,
	struct scb *scbd);

static struct scb *wlc_scbvictim(wlc_info_t *wlc);

static int wlc_scbinit(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int bandunit,
	struct scb *scb);
static void wlc_scbdeinit(wlc_info_t *wlc, struct scb *scbd);

static struct scb_info *wlc_scb_allocmem(scb_module_t *scbstate);
static void wlc_scb_freemem(scb_module_t *scbstate, struct scb_info *scbinfo);

static void wlc_scb_init_rates(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int bandunit,
	struct scb *scb);

#if defined(BCMDBG)
static int wlc_scb_dump(wlc_info_t *wlc, struct bcmstrbuf *b);
/** SCB Flags Names Initialization */
static const bcm_bit_desc_t scb_flags[] =
{
	{SCB_NONERP, "NonERP"},
	{SCB_LONGSLOT, "LgSlot"},
	{SCB_SHORTPREAMBLE, "ShPre"},
	{SCB_8021XHDR, "1X"},
	{SCB_WPA_SUP, "WPASup"},
	{SCB_DEAUTH, "DeA"},
	{SCB_WMECAP, "WME"},
	{SCB_BRCM, "BRCM"},
	{SCB_WDS_LINKUP, "WDSLinkUP"},
	{SCB_LEGACY_AES, "LegacyAES"},
	{SCB_MYAP, "MyAP"},
	{SCB_PENDING_PROBE, "PendingProbe"},
	{SCB_AMSDUCAP, "AMSDUCAP"},
	{SCB_USEME, "XXX"},
	{SCB_HTCAP, "HT"},
	{SCB_RECV_PM, "RECV_PM"},
	{SCB_AMPDUCAP, "AMPDUCAP"},
	{SCB_IS40, "40MHz"},
	{SCB_NONGF, "NONGFCAP"},
	{SCB_APSDCAP, "APSDCAP"},
	{SCB_PENDING_PSPOLL, "PendingPSPoll"},
	{SCB_RIFSCAP, "RIFSCAP"},
	{SCB_HT40INTOLERANT, "40INTOL"},
	{SCB_WMEPS, "WMEPSOK"},
	{SCB_COEX_MGMT, "OBSSCoex"},
	{SCB_IBSS_PEER, "IBSS Peer"},
	{SCB_STBCCAP, "STBC"},
	{0, NULL}
};
static const bcm_bit_desc_t scb_flags2[] =
{
	{SCB2_SGI20_CAP, "SGI20"},
	{SCB2_SGI40_CAP, "SGI40"},
	{SCB2_RX_LARGE_AGG, "LGAGG"},
	{SCB2_LDPCCAP, "LDPC"},
	{SCB2_VHTCAP, "VHT"},
	{SCB2_AMSDU_IN_AMPDU_CAP, "AGG^2"},
	{SCB2_P2P, "P2P"},
	{SCB2_DWDS_ACTIVE, "DWDS_ACTIVE"},
	{0, NULL}
};
static const bcm_bit_desc_t scb_flags3[] =
{
	{SCB3_A4_DATA, "A4_DATA"},
	{SCB3_A4_NULLDATA, "A4_NULLDATA"},
	{SCB3_A4_8021X, "A4_8021X"},
	{SCB3_DWDS_CAP, "DWDS_CAP"},
	{0, NULL}
};
static const bcm_bit_desc_t scb_states[] =
{
	{AUTHENTICATED, "AUTH"},
	{ASSOCIATED, "ASSOC"},
	{PENDING_AUTH, "AUTH_PEND"},
	{PENDING_ASSOC, "ASSOC_PEND"},
	{AUTHORIZED, "AUTH_8021X"},
	{0, NULL}
};
#endif 

#if defined(PKTC) || defined(PKTC_DONGLE)
static void wlc_scb_pktc_disable(struct scb *scb);
#endif

#ifdef SCBFREELIST
static void wlc_scbfreelist_free(scb_module_t *scbstate);
#if defined(BCMDBG)
static void wlc_scbfreelist_dump(scb_module_t *scbstate, struct bcmstrbuf *b);
#endif 
#endif /* SCBFREELIST */

#define SCBINFO(_scb) (_scb ? (struct scb_info *)((_scb)->scb_priv) : NULL)

#ifdef SCB_MEMDBG

#define SCBSANITYCHECK(_scb)  { \
		if (((_scb) != NULL) &&				\
		    ((((_scb))->magic != SCB_MAGIC) ||	\
		     (SCBINFO(_scb)->hashnext != SCBINFO(_scb)->hashnext_copy) || \
		     (SCBINFO(_scb)->next != SCBINFO(_scb)->next_copy)))	\
			osl_panic("scbinfo corrupted: magic: 0x%x hn: %p hnc: %p n: %p nc: %p\n", \
			      ((_scb))->magic, SCBINFO(_scb)->hashnext, \
			      SCBINFO(_scb)->hashnext_copy,		\
			      SCBINFO(_scb)->next, SCBINFO(_scb)->next_copy);	\
	}

#define SCBFREESANITYCHECK(_scb)  { \
		if (((_scb) != NULL) &&				\
		    ((((_scb))->magic != ~SCB_MAGIC) || \
		     (SCBINFO(_scb)->next != SCBINFO(_scb)->next_copy)))	\
			osl_panic("scbinfo corrupted: magic: 0x%x hn: %p hnc: %p n: %p nc: %p\n", \
			      ((_scb))->magic, SCBINFO(_scb)->hashnext, \
			      SCBINFO(_scb)->hashnext_copy,		\
			      SCBINFO(_scb)->next, SCBINFO(_scb)->next_copy);	\
	}

#else

#define SCBSANITYCHECK(_scbinfo)	do {} while (0)
#define SCBFREESANITYCHECK(_scbinfo)	do {} while (0)

#endif /* SCB_MEMDBG */

/** bsscfg cubby */
typedef struct scb_bsscfg_cubby {
	struct scb	**scbhash[MAXBANDS];	/* scb hash table */
	uint8		nscbhash;		/* scb hash table size */
	struct scb	*scb;			/* station control block link list */
} scb_bsscfg_cubby_t;

#define SCB_BSSCFG_CUBBY(ss, cfg) ((scb_bsscfg_cubby_t *)BSSCFG_CUBBY(cfg, (ss)->cfgh))

static int wlc_scb_bsscfg_init(void *context, wlc_bsscfg_t *cfg);
static void wlc_scb_bsscfg_deinit(void *context, wlc_bsscfg_t *cfg);
#if defined(BCMDBG)
static void wlc_scb_bsscfg_dump(void *context, wlc_bsscfg_t *cfg, struct bcmstrbuf *b);
#else
#define wlc_scb_bsscfg_dump NULL
#endif

/* # scb hash buckets */
#define SCB_HASH_N	((wlc->pub->tunables->maxscb + 7) / 8)
#define SCB_HASH_SZ	(sizeof(struct scb *) * MAXBANDS * SCB_HASH_N)

static int
wlc_scb_bsscfg_init(void *context, wlc_bsscfg_t *cfg)
{
	scb_module_t *scbstate = (scb_module_t *)context;
	wlc_info_t *wlc = scbstate->wlc;
	scb_bsscfg_cubby_t *scb_cfg = SCB_BSSCFG_CUBBY(scbstate, cfg);
	uint8 nscbhash, *scbhash;
	uint32 i;

	nscbhash = SCB_HASH_N;			    /* # scb hash buckets */

	scbhash = MALLOCZ(wlc->osh, SCB_HASH_SZ);
	if (scbhash == NULL)
		return BCME_NOMEM;

	scb_cfg->nscbhash = nscbhash;
	for (i = 0; i < MAXBANDS; i++) {
		scb_cfg->scbhash[i] = (struct scb **)
			((uintptr)scbhash + (i * nscbhash * sizeof(struct scb *)));
	}

	return BCME_OK;
}

static void
wlc_scb_bsscfg_deinit(void *context, wlc_bsscfg_t *cfg)
{
	scb_module_t *scbstate = (scb_module_t *)context;
	wlc_info_t *wlc = scbstate->wlc;
	scb_bsscfg_cubby_t *scb_cfg = SCB_BSSCFG_CUBBY(scbstate, cfg);

	/* clear all scbs */
	wlc_scb_bsscfg_scbclear(wlc, cfg, TRUE);

	/* N.B.: the hash is contiguously allocated across multiple bands */
	MFREE(wlc->osh, scb_cfg->scbhash[0], SCB_HASH_SZ);
}

/* # scb cubby registry entries */
#define SCB_CUBBY_REG_N  (wlc->pub->tunables->maxscbcubbies)
#define SCB_CUBBY_REG_SZ (sizeof(scb_module_t) + sizeof(cubby_info_ctx_t) * SCB_CUBBY_REG_N)

/* Min # scbs to have before starting aging.
 * Set to 1 for now as nop.
 */
#define SCB_AGING_THRESHOLD	1

scb_module_t *
BCMATTACHFN(wlc_scb_attach)(wlc_info_t *wlc)
{
	scb_module_t *scbstate;

	if ((scbstate = MALLOCZ(wlc->osh, SCB_CUBBY_REG_SZ)) == NULL) {
		WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
		          wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		goto fail;
	}

	/* OBJECT REGISTRY: check if shared key has value already stored */
	scbstate->cubby_info = (cubby_info_t *) obj_registry_get(wlc->objr, OBJR_SCB_CUBBY);
	if (scbstate->cubby_info == NULL) {
		int len = (sizeof(cubby_info_t) * SCB_CUBBY_REG_N);
		if ((scbstate->cubby_info = MALLOC(wlc->osh, len)) == NULL) {
			WL_ERROR(("wl%d: %s: out of mem, malloced %d bytes\n",
				wlc->pub->unit, __FUNCTION__, MALLOCED(wlc->osh)));
		}
		/* OBJECT REGISTRY: We are the first instance, store value for key */
		obj_registry_set(wlc->objr, OBJR_SCB_CUBBY, scbstate->cubby_info);
	}

	/* OBJECT REGISTRY: Reference the stored value in both instances */
	(void)obj_registry_ref(wlc->objr, OBJR_SCB_CUBBY);
	scbstate->cubby_info_ctx = (cubby_info_ctx_t *)
		((uintptr)scbstate + sizeof(scb_module_t));

	scbstate->wlc = wlc;
	/* TODO: maybe need a tunable here. */
	/* This is to limit the scb aging in low memory situation to happen
	 * less often in cases like aging out the only scb.
	 * For SCBFREELIST build we can set it to wlc->pub->tunables->maxscb
	 * to bypass the low memory processing.
	 */
	scbstate->aging_threshold = SCB_AGING_THRESHOLD;

	/* reserve cubby in the bsscfg container for per-bsscfg private data */
	if ((scbstate->cfgh = wlc_bsscfg_cubby_reserve(wlc, sizeof(scb_bsscfg_cubby_t),
		wlc_scb_bsscfg_init, wlc_scb_bsscfg_deinit, wlc_scb_bsscfg_dump,
		(void *)scbstate)) < 0) {
		WL_ERROR(("wl%d: %s: wlc_bsscfg_cubby_reserve failed\n",
			wlc->pub->unit, __FUNCTION__));
		goto fail;
	}

	scbstate->scbtotsize = sizeof(struct scb);
	scbstate->scbtotsize += sizeof(int) * MA_WINDOW_SZ; /* sizeof rssi_window */

#if defined(BCMDBG)
	wlc_dump_register(wlc->pub, "scb", (dump_fn_t)wlc_scb_dump, (void *)wlc);
#endif

	/* create notification list for scb state change. */
	if (bcm_notif_create_list(wlc->notif, &scbstate->scb_state_notif_hdl) != BCME_OK) {
		WL_ERROR(("wl%d: %s: scb bcm_notif_create_list() failed\n",
			wlc->pub->unit, __FUNCTION__));
		goto fail;
	}

	return scbstate;

fail:
	wlc_scb_detach(scbstate);
	return NULL;
}

void
BCMATTACHFN(wlc_scb_detach)(scb_module_t *scbstate)
{
	wlc_info_t *wlc;

	if (!scbstate)
		return;

	wlc = scbstate->wlc;

	if (scbstate->scb_state_notif_hdl != NULL)
		bcm_notif_delete_list(&scbstate->scb_state_notif_hdl);

#ifdef SCBFREELIST
	wlc_scbfreelist_free(scbstate);
#endif

	if (scbstate->cubby_info &&
	    (obj_registry_unref(wlc->objr, OBJR_SCB_CUBBY) == 0)) {
		int len = (sizeof(cubby_info_t) * SCB_CUBBY_REG_N);
		obj_registry_set(wlc->objr, OBJR_SCB_CUBBY, NULL);
		MFREE(wlc->osh, scbstate->cubby_info, len);
		scbstate->cubby_info = NULL;
	}

	MFREE(wlc->osh, scbstate, SCB_CUBBY_REG_SZ);
}

/* Methods for iterating along a list of scb */

/** Direct access to the next */
static struct scb *
wlc_scb_next(struct scb *scb)
{
	if (scb) {
		SCBSANITYCHECK(scb);
		return (SCBINFO(scb)->next ? SCBINFO(scb)->next->scbpub : NULL);
	}
	return NULL;
}

static struct wlc_bsscfg *
wlc_scb_next_bss(scb_module_t *scbstate, int idx)
{
	wlc_info_t *wlc = scbstate->wlc;
	wlc_bsscfg_t *next_bss = NULL;

	/* get next bss walking over hole */
	while (idx < WLC_MAXBSSCFG) {
		next_bss = WLC_BSSCFG(wlc, idx);
		if (next_bss != NULL)
			break;
		idx++;
	}
	return next_bss;
}

/** Initialize an iterator keeping memory of the next scb as it moves along the list */
void
wlc_scb_iterinit(scb_module_t *scbstate, struct scb_iter *scbiter, wlc_bsscfg_t *bsscfg)
{
	scb_bsscfg_cubby_t *scb_cfg;
	ASSERT(scbiter != NULL);

	if (bsscfg == NULL) {
		/* walk scbs of all bss */
		scbiter->all = TRUE;
		scbiter->next_bss = wlc_scb_next_bss(scbstate, 0);
		if (scbiter->next_bss == NULL) {
			/* init next scb pointer also to null */
			scbiter->next = NULL;
			return;
		}
	} else {
		/* walk scbs of specified bss */
		scbiter->all = FALSE;
		scbiter->next_bss = bsscfg;
	}

	ASSERT(scbiter->next_bss != NULL);
	scb_cfg = SCB_BSSCFG_CUBBY(scbstate, scbiter->next_bss);
	SCBSANITYCHECK(scb_cfg->scb);

	/* Prefetch next scb, so caller can free an scb before going on to the next */
	scbiter->next = scb_cfg->scb;
}

/** move the iterator */
struct scb *
wlc_scb_iternext(scb_module_t *scbstate, struct scb_iter *scbiter)
{
	scb_bsscfg_cubby_t *scb_cfg;
	struct scb *scb;

	ASSERT(scbiter != NULL);

	while (scbiter->next_bss) {

		/* get the next scb in the current bsscfg */
		if ((scb = scbiter->next) != NULL) {
			/* get next scb of bss */
			scbiter->next = wlc_scb_next(scb);
			return scb;
		}

		/* get the next bsscfg if we have run out of scbs in the current bsscfg */
		if (scbiter->all) {
			scbiter->next_bss =
			        wlc_scb_next_bss(scbstate, WLC_BSSCFG_IDX(scbiter->next_bss) + 1);
			if (scbiter->next_bss != NULL) {
				scb_cfg = SCB_BSSCFG_CUBBY(scbstate, scbiter->next_bss);
				scbiter->next = scb_cfg->scb;
			}
		} else {
			scbiter->next_bss = NULL;
		}
	}

	/* done with all bsscfgs and scbs */
	scbiter->next = NULL;

	return NULL;
}

/**
 * Multiple modules have the need of reserving some private data storage related to a specific
 * communication partner. During ATTACH time, this function is called multiple times, typically one
 * time per module that requires this storage. This function does not allocate memory, but
 * calculates values to be used for a future memory allocation by wlc_scb_allocmem() instead.
 *
 * Return value: negative values are errors.
 */
int
BCMATTACHFN(wlc_scb_cubby_reserve)(wlc_info_t *wlc, uint size, scb_cubby_init_t fn_init,
	scb_cubby_deinit_t fn_deinit, scb_cubby_dump_t fn_dump, void *context)
{
	uint offset;
	scb_module_t *scbstate = wlc->scbstate;
	cubby_info_t *cubby_info;
	cubby_info_ctx_t *cubby_info_ctx;

	ASSERT(scbstate->nscb == 0);
	ASSERT((scbstate->scbtotsize % PTRSZ) == 0);

	if (scbstate->ncubby >= (uint)SCB_CUBBY_REG_N) {
		ASSERT(scbstate->ncubby < (uint)SCB_CUBBY_REG_N);
		return BCME_NORESOURCE;
	}

	/* housekeeping info is stored in scb_module struct */
	cubby_info_ctx = &scbstate->cubby_info_ctx[scbstate->ncubby];
	cubby_info = &scbstate->cubby_info[scbstate->ncubby++];
	cubby_info->fn_init = fn_init;
	cubby_info->fn_deinit = fn_deinit;
	cubby_info->fn_dump = fn_dump;
	cubby_info_ctx->context = context;

	/* actual cubby data is stored at the end of scb's */
	offset = scbstate->scbtotsize;

	/* roundup to pointer boundary */
	scbstate->scbtotsize = ROUNDUP(scbstate->scbtotsize + size, PTRSZ);

	return offset;
}

struct wlcband *
wlc_scbband(wlc_info_t *wlc, struct scb *scb)
{
	return wlc->bandstate[scb->bandunit];
}


#ifdef SCBFREELIST
static struct scb_info *
wlc_scbget_free(scb_module_t *scbstate)
{
	struct scb_info *ret = NULL;

	if (scbstate->free_list == NULL)
		return NULL;

	ret = SCBINFO(scbstate->free_list);
	SCBFREESANITYCHECK(ret->scbpub);
	scbstate->free_list = (ret->next ? ret->next->scbpub : NULL);
#ifdef SCB_MEMDBG
	ret->next_copy = NULL;
#endif
	ret->next = NULL;

#ifdef SCB_MEMDBG
	scbstate->freelistcount = (scbstate->freelistcount > 0) ? (scbstate->freelistcount - 1) : 0;
#endif /* SCB_MEMDBG */

	return ret;
}

static void
wlc_scbadd_free(scb_module_t *scbstate, struct scb_info *ret)
{
	SCBFREESANITYCHECK(scbstate->free_list);
	ret->next = SCBINFO(scbstate->free_list);
	scbstate->free_list = ret->scbpub;
#ifdef SCB_MEMDBG
	ret->scbpub->magic = ~SCB_MAGIC;
	ret->next_copy = ret->next;
#endif

#ifdef SCB_MEMDBG
	scbstate->freelistcount += 1;
#endif /* SCB_MEMDBG */

	return;
}

static void
wlc_scbfreelist_free(scb_module_t *scbstate)
{
	struct scb_info *ret = NULL;

	ret = SCBINFO(scbstate->free_list);
	while (ret) {
		SCBFREESANITYCHECK(ret->scbpub);
		scbstate->free_list = (ret->next ? ret->next->scbpub : NULL);
		wlc_scb_freemem(scbstate, ret);
		ret = scbstate->free_list ? SCBINFO(scbstate->free_list) : NULL;

#ifdef SCB_MEMDBG
		scbstate->freelistcount = (scbstate->freelistcount > 0) ?
			(scbstate->freelistcount - 1) : 0;
#endif /* SCB_MEMDBG */
	}
}

#if defined(BCMDBG)
static
void wlc_scbfreelist_dump(scb_module_t *scbstate, struct bcmstrbuf *b)
{
	struct scb_info *entry = NULL;
	int i = 1;

#ifdef SCB_MEMDBG
	bcm_bprintf(b, "scbfreelist (count: %7u):\n", scbstate->freelistcount);
#else
	bcm_bprintf(b, "scbfreelist:\n");
#endif /* SCB_MEMDBG */

	entry = SCBINFO(scbstate->free_list);
	while (entry) {
		SCBFREESANITYCHECK(entry->scbpub);
		bcm_bprintf(b, "%d: 0x%x\n", i, entry);
		entry = entry->next ? SCBINFO(entry->next->scbpub) : NULL;
		i++;
	}
}
#endif 
#endif /* SCBFREELIST */

void
wlc_internalscb_free(wlc_info_t *wlc, struct scb *scb)
{
	wlc_scbfree(wlc, scb);
}

static void
wlc_scb_reset(scb_module_t *scbstate, struct scb_info *scbinfo)
{
	struct scb *scbpub = scbinfo->scbpub;

	bzero((char*)scbinfo, sizeof(struct scb_info));
	scbinfo->scbpub = scbpub;
	bzero(scbpub, scbstate->scbtotsize);
	scbpub->scb_priv = (void *) scbinfo;
	/* init substructure pointers */
	scbpub->rssi_window = (int *)((char *)scbpub + sizeof(struct scb));
}

/**
 * After all the modules indicated how much cubby space they need in the scb, the actual scb can be
 * allocated. This happens one time fairly late within the attach phase, but also when e.g.
 * communication with a new remote party is started.
 */
static struct scb_info *
wlc_scb_allocmem(scb_module_t *scbstate)
{
	wlc_info_t *wlc = scbstate->wlc;
	struct scb_info *scbinfo = NULL;
	struct scb *scbpub;

	/* Make sure free_mem never gets below minimum threshold due to scb_allocs */
	if (OSL_MEM_AVAIL() <= wlc->pub->tunables->min_scballoc_mem) {
		WL_ERROR(("wl%d: %s: low memory. %d bytes left.\n",
		          wlc->pub->unit, __FUNCTION__, OSL_MEM_AVAIL()));
		return NULL;
	}

	scbinfo = MALLOCZ(wlc->osh, sizeof(struct scb_info));
	if (scbinfo == NULL) {
		WL_ERROR(("wl%d: %s: malloc failure for scbinfo %d\n",
			wlc->pub->unit, __FUNCTION__, (int)sizeof(struct scb_info)));
		goto fail;
	}
	scbpub = MALLOC(wlc->osh, scbstate->scbtotsize);
	if (scbpub == NULL) {
		WL_ERROR(("wl%d: %s: malloc failure for scbpub %d\n",
			wlc->pub->unit, __FUNCTION__, (int)scbstate->scbtotsize));
		goto fail;
	}
	scbinfo->scbpub = scbpub;


#ifdef SCB_MEMDBG
	scbstate->scballoced++;
#endif /* SCB_MEMDBG */

	return scbinfo;

fail:
	wlc_scb_freemem(scbstate, scbinfo);
	return NULL;
}

struct scb *
wlc_internalscb_alloc(wlc_info_t *wlc, wlc_bsscfg_t *cfg,
	const struct ether_addr *ea, struct wlcband *band)
{
	struct scb_info *scbinfo = NULL;
	scb_module_t *scbstate = wlc->scbstate;
	int bcmerror = 0;
	struct scb *scb;

	if (TRUE &&
#ifdef SCBFREELIST
	    /* If not found on freelist then allocate a new one */
	    (scbinfo = wlc_scbget_free(scbstate)) == NULL &&
#endif
	    (scbinfo = wlc_scb_allocmem(scbstate)) == NULL) {
		WL_ERROR(("wl%d: %s: Couldn't alloc internal scb\n",
		          wlc->pub->unit, __FUNCTION__));
		return NULL;
	}
	wlc_scb_reset(scbstate, scbinfo);

	scb = scbinfo->scbpub;
	scb->bsscfg = cfg;
	scb->ea = *ea;

	/* used by hwrs and bcmc scbs */
	scb->flags2 |= SCB2_INTERNAL;

	bcmerror = wlc_scbinit(wlc, cfg, band->bandunit, scb);
	if (bcmerror) {
		WL_ERROR(("wl%d: %s failed with err %d\n",
			wlc->pub->unit, __FUNCTION__, bcmerror));
		wlc_internalscb_free(wlc, scb);
		return NULL;
	}

	wlc_scb_init_rates(wlc, cfg, band->bandunit, scb);

#ifdef TXQ_MUX
	WL_ERROR(("%s: --------------------->allocated internal SCB:%p\n", __FUNCTION__, scb));
#endif

	return scb;
}

static struct scb *
wlc_userscb_alloc(wlc_info_t *wlc, wlc_bsscfg_t *cfg,
	const struct ether_addr *ea, struct wlcband *band)
{
	scb_module_t *scbstate = wlc->scbstate;
	struct scb_info *scbinfo = NULL;
	struct scb *oldscb;
	int bcmerror;
	struct scb *scb;

	/* Make sure we live within our budget, and kick someone out if needed. */
	if (scbstate->nscb >= wlc->pub->tunables->maxscb ||
	    /* age scb in low memory situation as well */
	    (OSL_MEM_AVAIL() <= wlc->pub->tunables->min_scballoc_mem &&
	     /* apply scb aging in low memory situation in a limited way
	      * to prevent it ages too often.
	      */
	     scbstate->nscb >= scbstate->aging_threshold)) {
		/* free the oldest entry */
		if (!(oldscb = wlc_scbvictim(wlc))) {
			WL_ERROR(("wl%d: %s: no SCBs available to reclaim\n",
			          wlc->pub->unit, __FUNCTION__));
			return NULL;
		}
		SCB_DEAUTH_PEND_SET(oldscb);
		if (!wlc_scbfree(wlc, oldscb)) {
			WL_ERROR(("wl%d: %s: Couldn't free a victimized scb\n",
			          wlc->pub->unit, __FUNCTION__));
			return NULL;
		}
	}
	ASSERT(scbstate->nscb < wlc->pub->tunables->maxscb);

	if (TRUE &&
#ifdef SCBFREELIST
	    /* If not found on freelist then allocate a new one */
	    (scbinfo = wlc_scbget_free(scbstate)) == NULL &&
#endif
	    (scbinfo = wlc_scb_allocmem(scbstate)) == NULL) {
		WL_ERROR(("wl%d: %s: Couldn't alloc user scb\n",
		          wlc->pub->unit, __FUNCTION__));
		return NULL;
	}
	wlc_scb_reset(scbstate, scbinfo);

	scbstate->nscb++;

	scb = scbinfo->scbpub;
	scb->bsscfg = cfg;
	scb->ea = *ea;

	bcmerror = wlc_scbinit(wlc, cfg, band->bandunit, scb);
	if (bcmerror) {
		WL_ERROR(("wl%d: %s failed with err %d\n", wlc->pub->unit, __FUNCTION__, bcmerror));
		wlc_scbfree(wlc, scb);
		return NULL;
	}

	/* add it to the link list */
	wlc_scb_list_add(wlc, cfg, scb);

	/* install it in the cache */
	wlc_scb_hash_add(wlc, cfg, band->bandunit, scb);

	wlc_scb_init_rates(wlc, cfg, band->bandunit, scb);

#ifdef TXQ_MUX
	WL_ERROR(("%s(): --------------------->allocated user SCB:%p\n", __FUNCTION__, scb));
#endif

	return scb;
}

static int
wlc_scbinit(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int bandunit, struct scb *scb)
{
	scb_module_t *scbstate = wlc->scbstate;
	cubby_info_t *cubby_info;
	cubby_info_ctx_t *cubby_info_ctx;
	uint i;
	int bcmerror = 0;

	ASSERT(scb != NULL);

	scb->used = wlc->pub->now;
	scb->bandunit = bandunit;

	for (i = 0; i < NUMPRIO; i++)
		scb->seqctl[i] = 0xFFFF;
	scb->seqctl_nonqos = 0xFFFF;

#ifdef SCB_MEMDBG
	scb->magic = SCB_MAGIC;
#endif

	/* no other inits are needed for internal scb */
	if (SCB_INTERNAL(scb)) {
#ifdef INT_SCB_OPT
		return BCME_OK;
#endif
	}

	for (i = 0; i < scbstate->ncubby; i++) {
		cubby_info = &scbstate->cubby_info[i];
		cubby_info_ctx = &scbstate->cubby_info_ctx[i];
		if (cubby_info->fn_init) {
			bcmerror = cubby_info->fn_init(cubby_info_ctx->context, scb);
			if (bcmerror) {
				WL_ERROR(("wl%d: %s: Cubby failed\n",
				          wlc->pub->unit, __FUNCTION__));
				return bcmerror;
			}
		}
	}

#if defined(AP)
	wlc_scb_rssi_init(scb, WLC_RSSI_INVALID);
#endif
#ifdef PSPRETEND
	scb->ps_pretend = PS_PRETEND_NOT_ACTIVE;
	scb->ps_pretend_failed_ack_count = 0;
#endif
	return bcmerror;
}

static void
wlc_scb_freemem(scb_module_t *scbstate, struct scb_info *scbinfo)
{
	wlc_info_t *wlc = scbstate->wlc;
	struct scb *scbpub;

	BCM_REFERENCE(wlc);

	if (scbinfo == NULL)
		return;

	scbpub = scbinfo->scbpub;
	if (scbpub == NULL)
		goto exit1;


	MFREE(wlc->osh, scbpub, scbstate->scbtotsize);

exit1:
	MFREE(wlc->osh, scbinfo, sizeof(struct scb_info));

#ifdef SCB_MEMDBG
	scbstate->scbfreed++;
#endif /* SCB_MEMDBG */
}

#ifdef PROP_TXSTATUS
#define SCBHANDLE_PS_STATE_MASK (1 << 8)
#define SCBHANDLE_INFORM_PKTPEND_MASK (1 << 9)

static struct scb *
wlc_scbfind_from_wlcif(wlc_info_t *wlc, struct wlc_if *wlcif, uint8 *addr)
{
	struct scb *scb = NULL;
	wlc_bsscfg_t *bsscfg;
	bsscfg = wlc_bsscfg_find_by_wlcif(wlc, wlcif);

	if (!bsscfg)
		return NULL;

	if (BSSCFG_STA(bsscfg) && bsscfg->BSS)
		scb = wlc_scbfind(wlc, bsscfg, &bsscfg->BSSID);
	else if (!ETHER_ISMULTI(addr)) {
		scb = wlc_scbfind(wlc, bsscfg, (struct ether_addr *)addr);
	} else
		scb = bsscfg->bcmc_scb[wlc->band->bandunit];

	return scb;
}

void
wlc_scb_update_available_traffic_info(wlc_info_t *wlc, uint8 mac_handle, uint8 ta_bmp)
{
	struct scb *scb;
	struct scb_iter scbiter;

	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		if (scb->mac_address_handle &&
			(scb->mac_address_handle == mac_handle)) {
			SCB_PROPTXTSTATUS_SETTIM(scb, ta_bmp);
			if (AP_ENAB(wlc->pub))
				wlc_apps_pvb_update_from_host(wlc, scb);
			break;
		}
	}
}

bool
wlc_flow_ring_scb_update_available_traffic_info(wlc_info_t *wlc, uint8 mac_handle,
	uint8 tid, bool op)
{
	struct scb *scb;
	struct scb_iter scbiter;
	uint8 ta_bmp;
	bool  ret = TRUE;

	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		if (scb->mac_address_handle &&
			(scb->mac_address_handle == mac_handle)) {
			ta_bmp = SCB_PROPTXTSTATUS_TIM(scb);
			ta_bmp = (ta_bmp & ~(0x1 << tid));
			ta_bmp = (ta_bmp | (op << tid));
			SCB_PROPTXTSTATUS_SETTIM(scb, ta_bmp);
#ifdef AP
			if (BSSCFG_AP(scb->bsscfg)) {
				ret = wlc_apps_pvb_update_from_host(wlc, scb);
				if (!ret) {
					ta_bmp = (ta_bmp & ~(0x1 << tid));
					SCB_PROPTXTSTATUS_SETTIM(scb, ta_bmp);
				}
			}
#endif /* AP */
			break;
		}
	}
	return ret;
}
uint16
wlc_flow_ring_get_scb_handle(wlc_info_t *wlc, struct wlc_if *wlcif, uint8 *da)
{
	struct scb *scb;
	uint16	ret = 0xff;

	scb = wlc_scbfind_from_wlcif(wlc, wlcif, da);

	if (!scb || !scb->bsscfg)
		return ret;

	if (BSSCFG_AP(scb->bsscfg) || BSSCFG_AWDL(wlc, scb->bsscfg)) {
		ret = scb->mac_address_handle;
		if (BSSCFG_AP(scb->bsscfg)) {
			ret |= SCBHANDLE_INFORM_PKTPEND_MASK;
			if (!SCB_ISMULTI(scb) && SCB_PS(scb))
				ret |= SCBHANDLE_PS_STATE_MASK;
		}
	}
	return ret;
}

void wlc_flush_flowring_pkts(wlc_info_t *wlc, struct wlc_if *wlcif, uint8 *addr,
	uint16 flowid, uint8 tid)
{
	struct pktq tmp_q;
	void *pkt;
	int prec;
	struct scb *pktscb, *scb;

	scb = wlc_scbfind_from_wlcif(wlc, wlcif, addr);

	if (!scb || !scb->bsscfg)
		return;
#ifdef WLAMPDU
	wlc_ampdu_flush_pkts(wlc, scb, tid);
#endif
	pktq_init(&tmp_q, WLC_PREC_COUNT, PKTQ_LEN_DEFAULT);

	/* De-queue the packets and free them */
	while ((pkt = pktq_deq(WLC_GET_TXQ(wlc->active_queue), &prec))) {
		pktscb = WLPKTTAGSCBGET(pkt);
		if (!pktscb || (scb != pktscb) || (flowid != PKTFRAGFLOWRINGID(wlc->osh, pkt))) {
			pktq_penq(&tmp_q, prec, pkt);
			continue;
		}
		PKTFREE(wlc->osh, pkt, TRUE);
	}
	/* Enqueue back retrest of the packets */
	while ((pkt = pktq_deq(&tmp_q, &prec))) {
		pktq_penq(WLC_GET_TXQ(wlc->active_queue), prec, pkt);
	}
}
#endif /* PROP_TXSTATUS */

bool
wlc_scbfree(wlc_info_t *wlc, struct scb *scbd)
{
	struct scb_info *remove = SCBINFO(scbd);
	scb_module_t *scbstate = wlc->scbstate;

	if (scbd->permanent)
		return FALSE;

	/* Return if SCB is already being deleted else mark it */
	if (scbd->mark & SCB_DEL_IN_PROG)
		return FALSE;

	if (SCB_INTERNAL(scbd)) {
		goto free;
	}

	wlc_scb_resetstate(wlc, scbd);

	/* delete it from the hash */
	wlc_scb_hash_del(wlc, SCB_BSSCFG(scbd), scbd);

	/* delete it from the link list */
	wlc_scb_list_del(wlc, SCB_BSSCFG(scbd), scbd);

	/* update total allocated scb number */
	scbstate->nscb--;

free:
	scbd->mark |= SCB_DEL_IN_PROG;

	wlc_scbdeinit(wlc, scbd);

#ifdef SCBFREELIST
	wlc_scbadd_free(scbstate, remove);
#else
	/* free scb memory */
	wlc_scb_freemem(scbstate, remove);
#endif

	return TRUE;
}

static void
wlc_scbdeinit(wlc_info_t *wlc, struct scb *scbd)
{
	scb_module_t *scbstate = wlc->scbstate;
	cubby_info_t *cubby_info;
	cubby_info_ctx_t *cubby_info_ctx;
	uint i;
	uint8 prio;

	/* no other cleanups are needed for internal scb */
	if (SCB_INTERNAL(scbd)) {
#ifdef INT_SCB_OPT
		return;
#endif
	}

	for (i = 0; i < scbstate->ncubby; i++) {
		uint j = scbstate->ncubby - 1 - i;
		cubby_info = &scbstate->cubby_info[j];
		cubby_info_ctx = &scbstate->cubby_info_ctx[j];
		if (cubby_info->fn_deinit)
			cubby_info->fn_deinit(cubby_info_ctx->context, scbd);
	}

#ifdef PROP_TXSTATUS
	if (PROP_TXSTATUS_ENAB(wlc->pub)) {
		/* release MAC handle back to the pool, if applicable */
		if (scbd->mac_address_handle) {
			wlfc_MAC_table_update(wlc->wl, &scbd->ea.octet[0],
				WLFC_CTL_TYPE_MACDESC_DEL,
				scbd->mac_address_handle, ((scbd->bsscfg->wlcif == NULL) ?
				0 : scbd->bsscfg->wlcif->index));
			wlfc_release_MAC_descriptor_handle(wlc->wlfc_data,
				scbd->mac_address_handle);
			WLFC_DBGMESG(("STA: MAC-DEL for [%02x:%02x:%02x:%02x:%02x:%02x], "
				"handle: [%d], if:%d, t_idx:%d..\n",
				scbd->ea.octet[0], scbd->ea.octet[1], scbd->ea.octet[2],
				scbd->ea.octet[3], scbd->ea.octet[4], scbd->ea.octet[5],
				scbd->mac_address_handle,
				((scbd->bsscfg->wlcif == NULL) ? 0 : scbd->bsscfg->wlcif->index),
				WLFC_MAC_DESC_GET_LOOKUP_INDEX(scbd->mac_address_handle)));
		}
	}
#endif /* PROP_TXSTATUS */

#ifdef AP
	/* free any leftover authentication state */
	if (scbd->challenge) {
		MFREE(wlc->osh, scbd->challenge, 2 + scbd->challenge[1]);
		scbd->challenge = NULL;
	}
	/* free wpaie if stored */
	if (scbd->wpaie) {
		MFREE(wlc->osh, scbd->wpaie, scbd->wpaie_len);
		scbd->wpaie_len = 0;
		scbd->wpaie = NULL;
	}
#endif /* AP */

	/* free any frame reassembly buffer */
	for (prio = 0; prio < NUMPRIO; prio++) {
		if (scbd->fragbuf[prio]) {
			PKTFREE(wlc->osh, scbd->fragbuf[prio], FALSE);
			scbd->fragbuf[prio] = NULL;
			scbd->fragresid[prio] = 0;
		}
	}
}

static void
wlc_scb_list_add(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, struct scb *scb)
{
	struct scb_info *scbinfo = SCBINFO(scb);
	scb_bsscfg_cubby_t *scb_cfg;

	ASSERT(bsscfg != NULL);

	scb_cfg = SCB_BSSCFG_CUBBY(wlc->scbstate, bsscfg);

	SCBSANITYCHECK((scb_cfg)->scb);

	/* update scb link list */
	scbinfo->next = SCBINFO(scb_cfg->scb);
#ifdef SCB_MEMDBG
	scbinfo->next_copy = scbinfo->next;
#endif
	scb_cfg->scb = scbinfo->scbpub;
}

static void
wlc_scb_list_del(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, struct scb *scbd)
{
	scb_bsscfg_cubby_t *scb_cfg;
	struct scb_info *scbinfo;
	struct scb_info *remove = SCBINFO(scbd);

	ASSERT(bsscfg != NULL);

	/* delete it from the link list */

	scb_cfg = SCB_BSSCFG_CUBBY(wlc->scbstate, bsscfg);
	scbinfo = SCBINFO(scb_cfg->scb);
	if (scbinfo == remove) {
		scb_cfg->scb = wlc_scb_next(scbd);
		return;
	}

	while (scbinfo) {
		SCBSANITYCHECK(scbinfo->scbpub);
		if (scbinfo->next == remove) {
			scbinfo->next = remove->next;
#ifdef SCB_MEMDBG
			scbinfo->next_copy = scbinfo->next;
#endif
			break;
		}
		scbinfo = scbinfo->next;
	}
	ASSERT(scbinfo != NULL);
}

/** free all scbs of a bsscfg */
void
wlc_scb_bsscfg_scbclear(struct wlc_info *wlc, wlc_bsscfg_t *bsscfg, bool perm)
{
	struct scb_iter scbiter;
	struct scb *scb;

	if (wlc->scbstate == NULL)
		return;

	FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb) {
		if (scb->permanent) {
			if (!perm)
				continue;
			scb->permanent = FALSE;
		}
		wlc_scbfree(wlc, scb);
	}
}


static struct scb *
wlc_scbvictim(wlc_info_t *wlc)
{
	uint oldest;
	struct scb *scb;
	struct scb *oldscb;
	uint now, age;
	struct scb_iter scbiter;
#if defined(BCMDBG) || defined(WLMSG_ASSOC)
	char eabuf[ETHER_ADDR_STR_LEN];
#endif /* BCMDBG || WLMSG_ASSOC */
	wlc_bsscfg_t *bsscfg = NULL;

#ifdef AP
	/* search for an unauthenticated scb */
	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		if (!scb->permanent && (scb->state == 0))
			return scb;
	}
#endif /* AP */

	/* free the oldest scb */
	now = wlc->pub->now;
	oldest = 0;
	oldscb = NULL;
	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		bsscfg = SCB_BSSCFG(scb);
		ASSERT(bsscfg != NULL);
		if (BSSCFG_STA(bsscfg) && bsscfg->BSS && SCB_ASSOCIATED(scb))
			continue;
		if (!scb->permanent && ((age = (now - scb->used)) >= oldest)) {
			oldest = age;
			oldscb = scb;
		}
	}
	/* handle extreme case(s): all are permanent ... or there are no scb's at all */
	if (oldscb == NULL)
		return NULL;

#ifdef AP
	bsscfg = SCB_BSSCFG(oldscb);

	if (BSSCFG_AP(bsscfg)) {
		/* if the oldest authenticated SCB has only been idle a short time then
		 * it is not a candidate to reclaim
		 */
		if (oldest < SCB_SHORT_TIMEOUT)
			return NULL;
	}
#endif /* AP */

	WL_ASSOC(("wl%d: %s: relcaim scb %s, idle %d sec\n",  wlc->pub->unit, __FUNCTION__,
	          bcm_ether_ntoa(&oldscb->ea, eabuf), oldest));

	return oldscb;
}

#if defined(PKTC) || defined(PKTC_DONGLE)
void
wlc_scb_pktc_enable(struct scb *scb, const wlc_key_info_t *key_info)
{
	wlc_bsscfg_t *bsscfg = SCB_BSSCFG(scb);
	wlc_info_t *wlc = bsscfg->wlc;
	wlc_key_info_t tmp_ki;

	SCB_PKTC_DISABLE(scb);

	if (wlc->wet && BSSCFG_STA(bsscfg))
		return;

	/* No chaining for wds, non qos, non ampdu stas */
	if (SCB_A4_DATA(scb) || !SCB_QOS(scb) || !SCB_WME(scb) || !SCB_AMPDU(scb))
		return;

	if (!SCB_ASSOCIATED(scb) && !SCB_AUTHORIZED(scb))
		return;

#ifdef PKTC_DONGLE
	if (BSS_TDLS_BUFFER_STA(scb->bsscfg))
		return;
#endif

	if (key_info == NULL) {
		(void)wlc_keymgmt_get_scb_key(wlc->keymgmt, scb, WLC_KEY_ID_PAIRWISE,
			WLC_KEY_FLAG_NONE, &tmp_ki);
		key_info = &tmp_ki;
	}

	if (!WLC_KEY_ALLOWS_PKTC(key_info, SCB_BSSCFG(scb)))
		return;

	SCB_PKTC_ENABLE(scb);
}

static void
wlc_scb_pktc_disable(struct scb *scb)
{
	wlc_bsscfg_t *bsscfg = SCB_BSSCFG(scb);

	if (bsscfg && !SCB_AUTHORIZED(scb) && !SCB_ASSOCIATED(scb)) {
		int32 cidx;
		wlc_info_t *wlc = bsscfg->wlc;
		/* Invalidate rfc entry if scb is in it */
		cidx = BSSCFG_AP(bsscfg) ? 1 : 0;
		if (wlc->pktc_info->rfc[cidx].scb == scb) {
			WL_NONE(("wl%d: %s: Invalidate rfc %d before freeing scb %p\n",
			         wlc->pub->unit, __FUNCTION__, cidx, scb));
			wlc->pktc_info->rfc[cidx].scb = NULL;
		}
		SCB_PKTC_DISABLE(scb);
	}
}
#endif /* PKTC || PKTC_DONGLE */

/* Only notify registered clients of the following states' change. */
static uint8 scb_state_change_notif_mask = AUTHENTICATED | ASSOCIATED | AUTHORIZED;

/** "|" operation. */
void
wlc_scb_setstatebit(wlc_info_t *wlc, struct scb *scb, uint8 state)
{
	scb_module_t *scbstate;
	uint8	oldstate;

	WL_NONE(("set state %x\n", state));
	ASSERT(scb != NULL);

	scbstate = wlc->scbstate;
	oldstate = scb->state;

	if (state & AUTHENTICATED)
	{
		scb->state &= ~PENDING_AUTH;
	}
	if (state & ASSOCIATED)
	{
		ASSERT((scb->state | state) & AUTHENTICATED);
		scb->state &= ~PENDING_ASSOC;
	}

	scb->state |= state;
	WL_NONE(("wlc_scb : state = %x\n", scb->state));

#if defined(PKTC) || defined(PKTC_DONGLE)
	/* When transitioning to ASSOCIATED/AUTHORIZED state try if we can
	 * enable packet chaining for this SCB.
	 */
	if (SCB_BSSCFG(scb))
		wlc_scb_pktc_enable(scb, NULL);
#endif

	if ((oldstate & scb_state_change_notif_mask) != (scb->state & scb_state_change_notif_mask))
	{
		scb_state_upd_data_t data;
		data.scb = scb;
		data.oldstate = oldstate;
		bcm_notif_signal(scbstate->scb_state_notif_hdl, &data);
	}
}

/** "& ~" operation */
void
wlc_scb_clearstatebit(wlc_info_t *wlc, struct scb *scb, uint8 state)
{
	scb_module_t *scbstate;
	uint8	oldstate;

	ASSERT(scb != NULL);
	WL_NONE(("clear state %x\n", state));

	scbstate = wlc->scbstate;
	oldstate = scb->state;

	scb->state &= ~state;
	WL_NONE(("wlc_scb : state = %x\n", scb->state));

#if defined(PKTC) || defined(PKTC_DONGLE)
	/* Clear scb pointer in rfc */
	wlc_scb_pktc_disable(scb);
#endif

	if ((oldstate & scb_state_change_notif_mask) != (scb->state & scb_state_change_notif_mask))
	{
		scb_state_upd_data_t data;
		data.scb = scb;
		data.oldstate = oldstate;
		bcm_notif_signal(scbstate->scb_state_notif_hdl, &data);
	}
}


/** reset all state. */
void
wlc_scb_resetstate(wlc_info_t *wlc, struct scb *scb)
{
	WL_NONE(("reset state\n"));

	wlc_scb_clearstatebit(wlc, scb, ~0);
}

/** set/change rateset and init/reset ratesel */
static void
wlc_scb_init_rates(wlc_info_t *wlc, wlc_bsscfg_t *cfg, int bandunit, struct scb *scb)
{
	wlcband_t *band = wlc->bandstate[bandunit];
	wlc_rateset_t *rs;

	/* use current, target, or per-band default rateset? */
	if (wlc->pub->up &&
	    wlc_valid_chanspec(wlc->cmi, cfg->target_bss->chanspec))
		if (cfg->associated)
			rs = &cfg->current_bss->rateset;
		else
			rs = &cfg->target_bss->rateset;
	else
		rs = &band->defrateset;

	/*
	 * Initialize the per-scb rateset:
	 * - if we are AP, start with only the basic subset of the
	 *	network rates.  It will be updated when receive the next
	 *	probe request or association request.
	 * - if we are IBSS and gmode, special case:
	 *	start with B-only subset of network rates and probe for ofdm rates
	 * - else start with the network rates.
	 *	It will be updated on join attempts.
	 */
	if (BSS_P2P_ENAB(wlc, cfg)) {
		wlc_rateset_filter(rs /* src */, &scb->rateset /* dst */,
		                   FALSE, WLC_RATES_OFDM, RATE_MASK,
		                   wlc_get_mcsallow(wlc, cfg));
	}
	else if (BSSCFG_AP(cfg)) {
		uint8 mcsallow = BSS_N_ENAB(wlc, cfg) ? WLC_MCS_ALLOW : 0;
		wlc_rateset_filter(rs /* src */, &scb->rateset /* dst */,
		                   TRUE, WLC_RATES_CCK_OFDM, RATE_MASK,
		                   mcsallow);
	}
	else if (!cfg->BSS && band->gmode) {
		wlc_rateset_filter(rs /* src */, &scb->rateset /* dst */,
				FALSE, WLC_RATES_CCK, RATE_MASK, 0);
		/* if resulting set is empty, then take all network rates instead */
		if (scb->rateset.count == 0) {
			wlc_rateset_filter(rs /* src */, &scb->rateset /* dst */,
					FALSE, WLC_RATES_CCK_OFDM, RATE_MASK, 0);
		}
	}
	else {
		wlc_rateset_filter(rs /* src */, &scb->rateset /* dst */,
				FALSE, WLC_RATES_CCK_OFDM, RATE_MASK, 0);
	}

	if (!SCB_INTERNAL(scb)) {
		wlc_scb_ratesel_init(wlc, scb);
#ifdef STA
		/* send ofdm rate probe */
		if (BSSCFG_STA(cfg) && !cfg->BSS && band->gmode &&
		    wlc->pub->up)
			wlc_rateprobe(wlc, cfg, &scb->ea, WLC_RATEPROBE_RSPEC);
#endif /* STA */
	}

	wlc_keymgmt_notify(wlc->keymgmt, WLC_KEYMGMT_NOTIF_SCB_BSSCFG_CHANGED,
			NULL, scb, NULL, NULL);
}

static void
wlc_scb_bsscfg_reinit(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg)
{
	uint prev_count;
	const wlc_rateset_t *rs;
	wlcband_t *band;
	struct scb *scb;
	struct scb_iter scbiter;
	bool cck_only;
	bool reinit_forced;

	WL_INFORM(("wl%d: %s: bandunit 0x%x phy_type 0x%x gmode 0x%x\n", wlc->pub->unit,
		__FUNCTION__, wlc->band->bandunit, wlc->band->phytype, wlc->band->gmode));

	/* sanitize any existing scb rates against the current hardware rates */
	FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb) {
		prev_count = scb->rateset.count;
		/* Keep only CCK if gmode == GMODE_LEGACY_B */
		band = wlc_scbband(wlc, scb);
		if (BAND_2G(band->bandtype) && (band->gmode == GMODE_LEGACY_B)) {
			rs = &cck_rates;
			cck_only = TRUE;
		} else {
			rs = &band->hw_rateset;
			cck_only = FALSE;
		}
		if (!wlc_rate_hwrs_filter_sort_validate(&scb->rateset /* [in+out] */, rs /* [in] */,
			FALSE, wlc->stf->txstreams)) {
			/* continue with default rateset.
			 * since scb rateset does not carry basic rate indication,
			 * clear basic rate bit.
			 */
			WL_RATE(("wl%d: %s: invalid rateset in scb 0x%p bandunit 0x%x "
				"phy_type 0x%x gmode 0x%x\n", wlc->pub->unit, __FUNCTION__,
				scb, band->bandunit, band->phytype, band->gmode));
#ifdef BCMDBG
			wlc_rateset_show(wlc, &scb->rateset, &scb->ea);
#endif

			wlc_rateset_default(&scb->rateset, &band->hw_rateset,
			                    band->phytype, band->bandtype, cck_only, RATE_MASK,
			                    wlc_get_mcsallow(wlc, scb->bsscfg),
			                    CHSPEC_WLC_BW(scb->bsscfg->current_bss->chanspec),
			                    wlc->stf->txstreams);
			reinit_forced = TRUE;
		}
		else
			reinit_forced = FALSE;

		/* if the count of rates is different, then the rate state
		 * needs to be reinitialized
		 */
		if (reinit_forced || (scb->rateset.count != prev_count))
			wlc_scb_ratesel_init(wlc, scb);

		WL_RATE(("wl%d: %s: bandunit 0x%x, phy_type 0x%x gmode 0x%x. final rateset is\n",
			wlc->pub->unit, __FUNCTION__,
			band->bandunit, band->phytype, band->gmode));
#ifdef BCMDBG
		wlc_rateset_show(wlc, &scb->rateset, &scb->ea);
#endif
		wlc_keymgmt_notify(wlc->keymgmt, WLC_KEYMGMT_NOTIF_SCB_BSSCFG_CHANGED,
			NULL /* oldcfg */, scb, NULL, NULL);
	}
}

void
wlc_scb_reinit(wlc_info_t *wlc)
{
	int32 idx;
	wlc_bsscfg_t *bsscfg;

	FOREACH_BSS(wlc, idx, bsscfg) {
		wlc_scb_bsscfg_reinit(wlc, bsscfg);
	}
}

static INLINE struct scb* BCMFASTPATH
_wlc_scbfind(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, const struct ether_addr *ea, int bandunit)
{
	int indx;
	struct scb_info *scbinfo;
	scb_bsscfg_cubby_t *scb_cfg;

	ASSERT(bsscfg != NULL);

	/* All callers of wlc_scbfind() should first be checking to see
	 * if the SCB they're looking for is a BC/MC address.  Because we're
	 * using per bsscfg BCMC SCBs, we can't "find" BCMC SCBs without
	 * knowing which bsscfg.
	 */
	ASSERT(ea && !ETHER_ISMULTI(ea));


	/* search for the scb which corresponds to the remote station ea */
	scb_cfg = SCB_BSSCFG_CUBBY(wlc->scbstate, bsscfg);
	indx = SCBHASHINDEX(scb_cfg->nscbhash, ea->octet);
	scbinfo = (scb_cfg->scbhash[bandunit][indx] ?
	           SCBINFO(scb_cfg->scbhash[bandunit][indx]) :
	           NULL);
	for (; scbinfo; scbinfo = scbinfo->hashnext) {
		SCBSANITYCHECK(scbinfo->scbpub);

		if (eacmp((const char*)ea, (const char*)&(scbinfo->scbpub->ea)) == 0)
			break;
	}

	return (scbinfo ? scbinfo->scbpub : NULL);
}

/** Find station control block corresponding to the remote id */
struct scb * BCMFASTPATH
wlc_scbfind(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, const struct ether_addr *ea)
{
	struct scb *scb = NULL;

	scb = _wlc_scbfind(wlc, bsscfg, ea, wlc->band->bandunit);

#if defined(WLMCHAN)
/* current band could be different, so search again for all scb's */
	if (!scb && MCHAN_ACTIVE(wlc->pub) && NBANDS(wlc) > 1)
		scb = wlc_scbfindband(wlc, bsscfg, ea, OTHERBANDUNIT(wlc));
#endif 
	return scb;
}

/**
 * Lookup station control block corresponding to the remote id.
 * If not found, create a new entry.
 */
static INLINE struct scb *
_wlc_scblookup(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, const struct ether_addr *ea, int bandunit)
{
	struct scb *scb;
	struct wlcband *band;
#if defined(BCMDBG) || defined(WLMSG_ASSOC)
	char sa[ETHER_ADDR_STR_LEN];
#endif

	/* Don't allocate/find a BC/MC SCB this way. */
	ASSERT(!ETHER_ISMULTI(ea));
	if (ETHER_ISMULTI(ea))
		return NULL;

	/* apply mac filter */
	switch (wlc_macfltr_addr_match(wlc->macfltr, bsscfg, ea)) {
	case WLC_MACFLTR_ADDR_DENY:
		WL_ASSOC(("wl%d.%d mac restrict: Denying %s\n",
		          wlc->pub->unit, WLC_BSSCFG_IDX(bsscfg),
		          bcm_ether_ntoa(ea, sa)));
		return NULL;
	case WLC_MACFLTR_ADDR_NOT_ALLOW:
		WL_ASSOC(("wl%d.%d mac restrict: Not allowing %s\n",
		          wlc->pub->unit, WLC_BSSCFG_IDX(bsscfg),
		          bcm_ether_ntoa(ea, sa)));
		return NULL;
#ifdef BCMDBG
	case WLC_MACFLTR_ADDR_ALLOW:
		WL_ASSOC(("wl%d.%d mac restrict: Allowing %s\n",
		          wlc->pub->unit, WLC_BSSCFG_IDX(bsscfg),
		          bcm_ether_ntoa(ea, sa)));
		break;
	case WLC_MACFLTR_ADDR_NOT_DENY:
		WL_ASSOC(("wl%d.%d mac restrict: Not denying %s\n",
		          wlc->pub->unit, WLC_BSSCFG_IDX(bsscfg),
		          bcm_ether_ntoa(ea, sa)));
		break;
	case WLC_MACFLTR_DISABLED:
		WL_NONE(("wl%d.%d no mac restrict: lookup %s\n",
		         wlc->pub->unit, WLC_BSSCFG_IDX(bsscfg),
		         bcm_ether_ntoa(ea, sa)));
		break;
#endif /* BCMDBG */
	}

	if ((scb = _wlc_scbfind(wlc, bsscfg, ea, bandunit)))
		return (scb);

	/* no scb match, allocate one for the desired bandunit */
	band = wlc->bandstate[bandunit];
	return wlc_userscb_alloc(wlc, bsscfg, ea, band);
}

struct scb *
wlc_scblookup(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, const struct ether_addr *ea)
{
	return (_wlc_scblookup(wlc, bsscfg, ea, wlc->band->bandunit));
}

struct scb *
wlc_scblookupband(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, const struct ether_addr *ea, int bandunit)
{
	/* assert that the band is the current band, or we are dual band and it is the other band */
	ASSERT((bandunit == (int)wlc->band->bandunit) ||
	       (NBANDS(wlc) > 1 && bandunit == (int)OTHERBANDUNIT(wlc)));

	return (_wlc_scblookup(wlc, bsscfg, ea, bandunit));
}

/** Get scb from band */
struct scb * BCMFASTPATH
wlc_scbfindband(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, const struct ether_addr *ea, int bandunit)
{
	/* assert that the band is the current band, or we are dual band and it is the other band */
	ASSERT((bandunit == (int)wlc->band->bandunit) ||
	       (NBANDS(wlc) > 1 && bandunit == (int)OTHERBANDUNIT(wlc)));

	return (_wlc_scbfind(wlc, bsscfg, ea, bandunit));
}

/**
 * Determine if any SCB associated to ap cfg
 * cfg specifies a specific ap cfg to compare to.
 * If cfg is NULL, then compare to any ap cfg.
 */
bool
wlc_scb_associated_to_ap(wlc_info_t *wlc, wlc_bsscfg_t *cfg)
{
	struct scb_iter scbiter;
	struct scb *scb;
	bool associated = FALSE;

	ASSERT((cfg == NULL) || BSSCFG_AP(cfg));

	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		if (SCB_ASSOCIATED(scb) && BSSCFG_AP(scb->bsscfg)) {
			if ((cfg == NULL) || (cfg == scb->bsscfg)) {
				associated = TRUE;
			}
		}
	}

	return (associated);
}

void
wlc_scb_switch_band(wlc_info_t *wlc, struct scb *scb, int new_bandunit,
	wlc_bsscfg_t *bsscfg)
{
	/* first, del scb from hash table in old band */
	wlc_scb_hash_del(wlc, bsscfg, scb);
	/* next add scb to hash table in new band */
	wlc_scb_hash_add(wlc, bsscfg, new_bandunit, scb);
	return;
}

/**
 * Move the scb's band info.
 * Parameter description:
 *
 * wlc - global wlc_info structure
 * bsscfg - the bsscfg that is about to move to a new chanspec
 * chanspec - the new chanspec the bsscfg is moving to
 *
 */
void
wlc_scb_update_band_for_cfg(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, chanspec_t chanspec)
{
	struct scb_iter scbiter;
	struct scb *scb, *stale_scb;
	int bandunit;
	bool reinit = FALSE;

	FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb) {
		if (SCB_ASSOCIATED(scb)) {
			bandunit = CHSPEC_WLCBANDUNIT(chanspec);
			if (scb->bandunit != (uint)bandunit) {
				/* We're about to move our scb to the new band.
				 * Check to make sure there isn't an scb entry for us there.
				 * If there is one for us, delete it first.
				 */
				if ((stale_scb = _wlc_scbfind(wlc, bsscfg,
				                      &bsscfg->BSSID, bandunit)) &&
				    (stale_scb->permanent == FALSE)) {
					WL_ASSOC(("wl%d.%d: %s: found stale scb %p on %s band, "
					          "remove it\n",
					          wlc->pub->unit, bsscfg->_idx, __FUNCTION__,
					          stale_scb,
					          (bandunit == BAND_5G_INDEX) ? "5G" : "2G"));
					/* mark the scb for removal */
					stale_scb->stale_remove = TRUE;
				}
				/* Now perform the move of our scb to the new band */
				wlc_scb_switch_band(wlc, scb, bandunit, bsscfg);
				reinit = TRUE;
			}
		}
	}
	/* remove stale scb's marked for removal */
	FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb) {
		if (scb->stale_remove == TRUE) {
			WL_ASSOC(("remove stale scb %p\n", scb));
			scb->stale_remove = FALSE;
			wlc_scbfree(wlc, scb);
		}
	}

	if (reinit) {
		wlc_scb_reinit(wlc);
	}
}

struct scb *
wlc_scbibssfindband(wlc_info_t *wlc, const struct ether_addr *ea, int bandunit,
                    wlc_bsscfg_t **bsscfg)
{
	int idx;
	wlc_bsscfg_t *cfg;
	struct scb *scb = NULL;

	/* assert that the band is the current band, or we are dual band
	 * and it is the other band.
	 */
	ASSERT((bandunit == (int)wlc->band->bandunit) ||
	       (NBANDS(wlc) > 1 && bandunit == (int)OTHERBANDUNIT(wlc)));

	FOREACH_IBSS(wlc, idx, cfg) {
		/* Find the bsscfg and scb matching specified peer mac */
		scb = _wlc_scbfind(wlc, cfg, ea, bandunit);
		if (scb != NULL) {
			*bsscfg = cfg;
			break;
		}
	}

	return scb;
}

struct scb *
wlc_scbapfind(wlc_info_t *wlc, const struct ether_addr *ea, wlc_bsscfg_t **bsscfg)
{
	int idx;
	wlc_bsscfg_t *cfg;
	struct scb *scb = NULL;

	*bsscfg = NULL;

	FOREACH_UP_AP(wlc, idx, cfg) {
		/* Find the bsscfg and scb matching specified peer mac */
		scb = wlc_scbfind(wlc, cfg, ea);
		if (scb != NULL) {
			*bsscfg = cfg;
			break;
		}
	}

	return scb;
}

struct scb * BCMFASTPATH
wlc_scbbssfindband(wlc_info_t *wlc, const struct ether_addr *hwaddr,
                   const struct ether_addr *ea, int bandunit, wlc_bsscfg_t **bsscfg)
{
	int idx;
	wlc_bsscfg_t *cfg;
	struct scb *scb = NULL;

	/* assert that the band is the current band, or we are dual band
	 * and it is the other band.
	 */
	ASSERT((bandunit == (int)wlc->band->bandunit) ||
	       (NBANDS(wlc) > 1 && bandunit == (int)OTHERBANDUNIT(wlc)));

	*bsscfg = NULL;

	FOREACH_BSS(wlc, idx, cfg) {
		/* Find the bsscfg and scb matching specified hwaddr and peer mac */
		if (eacmp(cfg->cur_etheraddr.octet, hwaddr->octet) == 0) {
			scb = _wlc_scbfind(wlc, cfg, ea, bandunit);
			if (scb != NULL) {
				*bsscfg = cfg;
				break;
			}
		}
	}

	return scb;
}

static void
wlc_scb_hash_add(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, int bandunit, struct scb *scb)
{
	scb_bsscfg_cubby_t *scb_cfg;
	int indx;
	struct scb_info *scbinfo;

	ASSERT(bsscfg != NULL);

	scb->bandunit = bandunit;

	scb_cfg = SCB_BSSCFG_CUBBY(wlc->scbstate, bsscfg);
	indx = SCBHASHINDEX(scb_cfg->nscbhash, scb->ea.octet);
	scbinfo = (scb_cfg->scbhash[bandunit][indx] ?
	           SCBINFO(scb_cfg->scbhash[bandunit][indx]) : NULL);

	SCBINFO(scb)->hashnext = scbinfo;
#ifdef SCB_MEMDBG
	SCBINFO(scb)->hashnext_copy = SCBINFO(scb)->hashnext;
#endif

	scb_cfg->scbhash[bandunit][indx] = scb;
}

static void
wlc_scb_hash_del(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, struct scb *scbd)
{
	scb_bsscfg_cubby_t *scb_cfg;
	int indx;
	struct scb_info *scbinfo;
	struct scb_info *remove = SCBINFO(scbd);
	int bandunit = scbd->bandunit;

	ASSERT(bsscfg != NULL);

	scb_cfg = SCB_BSSCFG_CUBBY(wlc->scbstate, bsscfg);
	indx = SCBHASHINDEX(scb_cfg->nscbhash, scbd->ea.octet);

	/* delete it from the hash */
	scbinfo = (scb_cfg->scbhash[bandunit][indx] ?
	           SCBINFO(scb_cfg->scbhash[bandunit][indx]) : NULL);
	ASSERT(scbinfo != NULL);
	SCBSANITYCHECK(scbinfo->scbpub);
	/* special case for the first */
	if (scbinfo == remove) {
		if (scbinfo->hashnext)
		    SCBSANITYCHECK(scbinfo->hashnext->scbpub);
		scb_cfg->scbhash[bandunit][indx] =
		        (scbinfo->hashnext ? scbinfo->hashnext->scbpub : NULL);
	} else {
		for (; scbinfo; scbinfo = scbinfo->hashnext) {
			SCBSANITYCHECK(scbinfo->hashnext->scbpub);
			if (scbinfo->hashnext == remove) {
				scbinfo->hashnext = remove->hashnext;
#ifdef SCB_MEMDBG
				scbinfo->hashnext_copy = scbinfo->hashnext;
#endif
				break;
			}
		}
		ASSERT(scbinfo != NULL);
	}
}

void
wlc_scb_sortrates(wlc_info_t *wlc, struct scb *scb)
{
	wlcband_t *band = wlc_scbband(wlc, scb);

	wlc_rate_hwrs_filter_sort_validate(&scb->rateset /* [in+out] */,
		&band->hw_rateset /* [in] */, FALSE,
		wlc->stf->txstreams);
}

void
BCMINITFN(wlc_scblist_validaterates)(wlc_info_t *wlc)
{
	struct scb *scb;
	struct scb_iter scbiter;

	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		wlc_scb_sortrates(wlc, scb);
		if (scb->rateset.count == 0)
			wlc_scbfree(wlc, scb);
	}
}

#if defined(AP) || defined(WLTDLS)
int
wlc_scb_rssi(struct scb *scb)
{
	int rssi = 0, cnt;
	int i;

	for (i = 0, cnt = 0; i < MA_WINDOW_SZ; i++)
		if (scb->rssi_window[i] != WLC_RSSI_INVALID)
		{
			rssi += scb->rssi_window[i];
			cnt++;
		}
	if (cnt > 1) rssi /= cnt;

	return (rssi);
}

int8
wlc_scb_rssi_chain(struct scb *scb, int chain)
{
	int8 rssi_avg = WLC_RSSI_INVALID, cnt;
	int32 rssi = 0;
	int i;

	for (i = 0, cnt = 0; i < MA_WINDOW_SZ; i++) {
		if (scb->rssi_chain[chain][i] != WLC_RSSI_INVALID) {
			rssi += scb->rssi_chain[chain][i];
			cnt++;
		}
	}

	if (cnt >= 1) {
		rssi_avg = rssi/cnt;
	}

	return (rssi_avg);
}

/* return the rssi of last received packet per scb and
 * per antenna chain.
 */
int8
wlc_scb_pkt_rssi_chain(struct scb *scb, int chain)
{
	int last_rssi_index;
	int8 rssi = 0;

	last_rssi_index = MODDEC_POW2(scb->rssi_index, MA_WINDOW_SZ);
	if ((chain >= WL_ANT_IDX_1) && (chain < WL_RSSI_ANT_MAX) &&
		(scb->rssi_chain[chain][last_rssi_index] != WLC_RSSI_INVALID))
		rssi = (int8)scb->rssi_chain[chain][last_rssi_index];

	return rssi;
}

void
wlc_scb_rssi_init(struct scb *scb, int rssi)
{
	int i, j;
	scb->rssi_enabled = 1;

	for (i = 0; i < MA_WINDOW_SZ; i++) {
		scb->rssi_window[i] = rssi;
		for (j = 0; j < WL_RSSI_ANT_MAX; j++)
			scb->rssi_chain[j][i] = rssi;
	}

	scb->rssi_index = 0;
}

/** Enable or disable RSSI update for a particular requestor module */
bool
wlc_scb_rssi_update_enable(struct scb *scb, bool enable, scb_rssi_requestor_t rid)
{
	if (enable) {
		scb->rssi_upd |= (1<<rid);
	} else {
		scb->rssi_upd &= ~(1<<rid);
	}
	return (scb->rssi_upd != 0);
}

#endif /* AP || WLTDLS */


#if defined(BCMDBG)
void
wlc_scb_dump_scb(wlc_info_t *wlc, wlc_bsscfg_t *cfg, struct scb *scb, struct bcmstrbuf *b, int idx)
{
	uint i;
	char eabuf[ETHER_ADDR_STR_LEN];
	char flagstr[64];
	char flagstr2[64];
	char flagstr3[64];
	char statestr[64];
	cubby_info_t *cubby_info;
	cubby_info_ctx_t *cubby_info_ctx;
#ifdef AP
	char ssidbuf[SSID_FMT_BUF_LEN] = "";
#endif /* AP */

	bcm_format_flags(scb_flags, scb->flags, flagstr, 64);
	bcm_format_flags(scb_flags2, scb->flags2, flagstr2, 64);
	bcm_format_flags(scb_flags3, scb->flags3, flagstr3, 64);
	bcm_format_flags(scb_states, scb->state, statestr, 64);

	if (SCB_INTERNAL(scb))
		bcm_bprintf(b, "  I");
	else
		bcm_bprintf(b, "%3d", idx);
	bcm_bprintf(b, "%s %s\n", (scb->permanent? "*":" "),
	            bcm_ether_ntoa(&scb->ea, eabuf));

	bcm_bprintf(b, "     State:0x%02x (%s) Used:%d(%d)\n",
	            scb->state, statestr, scb->used,
	            (int)(scb->used - wlc->pub->now));
	bcm_bprintf(b, "     Band:%s",
	            ((scb->bandunit == BAND_2G_INDEX) ? BAND_2G_NAME :
	             BAND_5G_NAME));
	bcm_bprintf(b, "\n");
	bcm_bprintf(b, "     Flags:0x%x", scb->flags);
	if (flagstr[0] != '\0')
		bcm_bprintf(b, " (%s)", flagstr);
	bcm_bprintf(b, "\n");
	bcm_bprintf(b, "     Flags2:0x%x", scb->flags2);
	if (flagstr2[0] != '\0')
		bcm_bprintf(b, " (%s)", flagstr2);
	bcm_bprintf(b, "\n");
	bcm_bprintf(b, "     Flags3:0x%x", scb->flags3);
	if (flagstr3[0] != '\0')
		bcm_bprintf(b, " (%s)", flagstr3);
	bcm_bprintf(b, "\n");
	bcm_bprintf(b, "\n");
	if (cfg != NULL)
		bcm_bprintf(b, "     Cfg:%d(%p)", WLC_BSSCFG_IDX(cfg), cfg);

	bcm_bprintf(b, "\n");

	wlc_dump_rateset("     rates", &scb->rateset, b);
	bcm_bprintf(b, "\n");

	if (scb->rateset.htphy_membership) {
		bcm_bprintf(b, "     membership %d(b)",
		            (scb->rateset.htphy_membership & RATE_MASK));
		bcm_bprintf(b, "\n");
		bcm_bprintf(b, "     Prop HT rates support:%d\n",
		            SCB_HT_PROP_RATES_CAP(scb));
	}

#ifdef AP
	if (cfg != NULL && BSSCFG_AP(cfg)) {
		bcm_bprintf(b, "     AID:0x%x PS:%d Listen:%d WDS:%d(%p) RSSI:%d",
		            scb->aid, scb->PS, scb->listen, (scb->wds ? 1 : 0),
		            scb->wds, wlc_scb_rssi(scb));
		wlc_format_ssid(ssidbuf, cfg->SSID, cfg->SSID_len);
		bcm_bprintf(b, " BSS %d \"%s\"\n",
		            WLC_BSSCFG_IDX(cfg), ssidbuf);
	}
#endif
#ifdef STA
	if (cfg != NULL && BSSCFG_STA(cfg)) {
		bcm_bprintf(b, "     MAXSP:%u DEFL:0x%x TRIG:0x%x DELV:0x%x\n",
		            scb->apsd.maxsplen, scb->apsd.ac_defl,
		            scb->apsd.ac_trig, scb->apsd.ac_delv);
	}
#endif
	bcm_bprintf(b,  "     WPA_auth 0x%x wsec 0x%x\n", scb->WPA_auth, scb->wsec);

#if defined(STA) && defined(DBG_BCN_LOSS)
	bcm_bprintf(b,	"	  last_rx:%d last_rx_rssi:%d last_bcn_rssi: "
	            "%d last_tx: %d\n",
	            scb->dbg_bcn.last_rx, scb->dbg_bcn.last_rx_rssi, scb->dbg_bcn.last_bcn_rssi,
	            scb->dbg_bcn.last_tx);
#endif

	for (i = 0; i < wlc->scbstate->ncubby; i++) {
		cubby_info = &wlc->scbstate->cubby_info[i];
		cubby_info_ctx = &wlc->scbstate->cubby_info_ctx[i];
		if (cubby_info->fn_dump)
			cubby_info->fn_dump(cubby_info_ctx->context, scb, b);
	}
}

static void
wlc_scb_bsscfg_dump(void *context, wlc_bsscfg_t *cfg, struct bcmstrbuf *b)
{
	scb_module_t *scbstate = (scb_module_t *)context;
	wlc_info_t *wlc = scbstate->wlc;
	int k;
	struct scb *scb;
	struct scb_iter scbiter;

#ifdef SCB_MEMDBG
	bcm_bprintf(b, "# of scbs: %u, scballoced[%u] scbfreed[%u] freelistcount[%u]\n",
		scbstate->nscb, scbstate->scballoced, scbstate->scbfreed,
		scbstate->freelistcount);
#else
	bcm_bprintf(b, "# of scbs: %u\n", scbstate->nscb);
#endif /* SCB_MEMDBG */
	bcm_bprintf(b, "# of cubbies: %u, scb size: %u\n",
	            scbstate->ncubby, scbstate->scbtotsize);

	bcm_bprintf(b, "idx  ether_addr\n");
	k = 0;
	FOREACH_BSS_SCB(scbstate, &scbiter, cfg, scb) {
		wlc_scb_dump_scb(wlc, cfg, scb, b, k);
		k++;
	}

	return;
}

static int
wlc_scb_dump(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	int32 idx;
	wlc_bsscfg_t *bsscfg;

	FOREACH_BSS(wlc, idx, bsscfg) {
		wlc_scb_bsscfg_dump(wlc->scbstate, bsscfg, b);
	}

#ifdef SCBFREELIST
	wlc_scbfreelist_dump(wlc->scbstate, b);
#endif /* SCBFREELIST */
	return 0;
}
#endif 

int
wlc_scb_save_wpa_ie(wlc_info_t *wlc, struct scb *scb, bcm_tlv_t *ie)
{
	uint ie_len;

	ASSERT(scb != NULL);
	ASSERT(ie != NULL);

	ie_len = TLV_HDR_LEN + ie->len;

	/* Optimization */
	if (scb->wpaie != NULL && ie != NULL &&
	    scb->wpaie_len == ie_len)
		goto cp;

	/* Free old WPA IE if one exists */
	if (scb->wpaie != NULL) {
	        MFREE(wlc->osh, scb->wpaie, scb->wpaie_len);
	        scb->wpaie_len = 0;
	        scb->wpaie = NULL;
	}

	/* Store the WPA IE for later retrieval */
	if ((scb->wpaie = MALLOC(wlc->osh, ie_len)) == NULL) {
		WL_ERROR(("wl%d: %s: unable to allocate memory\n",
		          wlc->pub->unit, __FUNCTION__));
		return BCME_NOMEM;
	}

cp:	/* copy */
	bcopy(ie, scb->wpaie, ie_len);
	scb->wpaie_len = ie_len;

	return BCME_OK;
}

int
wlc_scb_state_upd_register(wlc_info_t *wlc, bcm_notif_client_callback fn, void *arg)
{
	bcm_notif_h hdl = wlc->scbstate->scb_state_notif_hdl;

	return bcm_notif_add_interest(hdl, fn, arg);
}

int
wlc_scb_state_upd_unregister(wlc_info_t *wlc, bcm_notif_client_callback fn, void *arg)
{
	bcm_notif_h hdl = wlc->scbstate->scb_state_notif_hdl;

	return bcm_notif_remove_interest(hdl, fn, arg);
}

#ifdef WL_CS_RESTRICT_RELEASE
void
wlc_scb_restrict_wd(wlc_info_t *wlc)
{
	struct scb *scb;
	struct scb_iter scbiter;

	FOREACHSCB(wlc->scbstate, &scbiter, scb) {
		if (scb->restrict_deadline) {
			scb->restrict_deadline--;
		}
		if (!scb->restrict_deadline) {
			scb->restrict_txwin = 0;
		}
	}
}

void
wlc_scb_restrict_start(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg)
{
	struct scb *scb;
	struct scb_iter scbiter;

	FOREACH_BSS_SCB(wlc->scbstate, &scbiter, bsscfg, scb) {
		if (!SCB_ISMULTI(scb) && SCB_AMPDU(scb)) {
			scb->restrict_txwin = SCB_RESTRICT_MIN_TXWIN;
			scb->restrict_deadline = SCB_RESTRICT_WD_TIMEOUT + 1;
		}
	}
}
#endif /* WL_CS_RESTRICT_RELEASE */

#ifdef PROP_TXSTATUS
int wlc_scb_wlfc_entry_add(wlc_info_t *wlc, struct scb *scb)
{
	int err = BCME_OK;

	if (wlc == NULL || scb == NULL) {
		err = BCME_BADARG;
		goto end;
	}

	if (!PROP_TXSTATUS_ENAB(wlc->pub)) {
		err = BCME_UNSUPPORTED;
		goto end;
	}

	/* allocate a handle from bitmap f we haven't already done so */
	if (scb->mac_address_handle == 0)
		scb->mac_address_handle = wlfc_allocate_MAC_descriptor_handle(
			wlc->wlfc_data);
	err = wlfc_MAC_table_update(wlc->wl, &scb->ea.octet[0],
		WLFC_CTL_TYPE_MACDESC_ADD, scb->mac_address_handle,
		((SCB_BSSCFG(scb) == NULL) ? 0 : SCB_BSSCFG(scb)->wlcif->index));

	if (err != BCME_OK) {
		WL_ERROR(("wl%d.%d: %s() wlfc_MAC_table_update() failed %d\n",
			WLCWLUNIT(wlc), WLC_BSSCFG_IDX(SCB_BSSCFG(scb)),
			__FUNCTION__, err));
	}

end:
	return err;
}
#endif /* PROP_TXSTATUS */

void
wlc_scbfind_delete(wlc_info_t *wlc, wlc_bsscfg_t *bsscfg, struct ether_addr *ea)
{
	int i;
#if defined(BCMDBG) || defined(WLMSG_ASSOC)
	char eabuf[ETHER_ADDR_STR_LEN];
#endif /* BCMDBG || WLMSG_ASSOC */
	struct scb *scb;

	for (i = 0; i < (int)NBANDS(wlc); i++) {
		/* Use band 1 for single band 11a */
		if (IS_SINGLEBAND_5G(wlc->deviceid))
			i = BAND_5G_INDEX;

		scb = wlc_scbfindband(wlc, bsscfg, ea, i);
		if (scb) {
			WL_ASSOC(("wl%d: %s: scb for the STA-%s"
				" already exists\n", wlc->pub->unit, __FUNCTION__,
				bcm_ether_ntoa(ea, eabuf)));
			wlc_scbfree(wlc, scb);
		}
	}
}
