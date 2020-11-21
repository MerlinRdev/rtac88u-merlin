/*
 * RadarDetect module implementation - iovar table/handlers & registration
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

#include <phy_cfg.h>

#include <typedefs.h>
#include <bcmutils.h>
#include <bcmendian.h>

#ifdef WLC_HIGH
#include <siutils.h>
#include <d11.h>
#include <wlc_rate.h>
#include <wlioctl.h>
#include <wlc_pub.h>
#include <wlc.h>
#endif /* WLC_HIGH */

#include <phy_api.h>
#include "phy_radar_st.h"
#include <phy_radar_iov.h>

#include <wlc_iocv_types.h>
#include <wlc_iocv_reg.h>

#ifndef ALL_NEW_PHY_MOD
#include <wlc_phy_int.h>
#endif

/* id's */
enum {
	IOV_RADAR_ARGS = 1,
	IOV_RADAR_ARGS_40MHZ = 2,
	IOV_RADAR_THRS = 3,
	IOV_PHY_DFS_LP_BUFFER = 4,
	IOV_RADAR_STATUS = 5,
	IOV_CLEAR_RADAR_STATUS = 6
};

#ifdef WLC_HIGH
/* iovar table */
static const bcm_iovar_t phy_radar_iovt[] = {
	{"radarargs", IOV_RADAR_ARGS, (0), IOVT_BUFFER, sizeof(wl_radar_args_t)},
	{"radarargs40", IOV_RADAR_ARGS_40MHZ, (0), IOVT_BUFFER, sizeof(wl_radar_args_t)},
	{"radarthrs", IOV_RADAR_THRS, (IOVF_SET_UP), IOVT_BUFFER, sizeof(wl_radar_thr_t)},
	{"radar_status", IOV_RADAR_STATUS, (0), IOVT_BUFFER, sizeof(wl_radar_status_t)},
	{"clear_radar_status", IOV_CLEAR_RADAR_STATUS, (IOVF_SET_UP), IOVT_BUFFER,
	sizeof(wl_radar_status_t)},
#if defined(BCMDBG) || defined(WLTEST)
	{"phy_dfs_lp_buffer", IOV_PHY_DFS_LP_BUFFER, 0, IOVT_UINT8, 0},
#endif 
	{NULL, 0, 0, 0, 0}
};
#endif /* WLC_HIGH */

#ifdef WLC_LOW
/* iovar handler */
static int
phy_radar_doiovar(void *ctx, uint32 aid, void *p, uint plen, void *a, uint alen, uint vsz)
{
	phy_info_t *pi = (phy_info_t *)ctx;
	phy_radar_info_t *ri = pi->radari;
	phy_radar_st_t *st = phy_radar_get_st(ri);
	int err = BCME_OK;
	int int_val = 0;
	bool bool_val;

	/* The PHY type implemenation isn't registered */
	if (st == NULL) {
		PHY_ERROR(("%s: not supported\n", __FUNCTION__));
		return BCME_UNSUPPORTED;
	}

	if (plen >= (uint)sizeof(int_val))
		bcopy(p, &int_val, sizeof(int_val));

	/* bool conversion to avoid duplication below */
	bool_val = int_val != 0;

	switch (aid) {
	case IOV_GVAL(IOV_RADAR_ARGS):
		bcopy(&st->rparams.radar_args, a, sizeof(wl_radar_args_t));
		break;

	case IOV_SVAL(IOV_RADAR_THRS): {
		wl_radar_thr_t radar_thr;

		/* len is check done before gets here */
		bzero(&radar_thr, sizeof(wl_radar_thr_t));
		bcopy(p, &radar_thr, sizeof(wl_radar_thr_t));
		if (radar_thr.version != WL_RADAR_THR_VERSION) {
			err = BCME_VERSION;
			break;
		}
		st->rparams.radar_thrs.thresh0_20_lo = radar_thr.thresh0_20_lo;
		st->rparams.radar_thrs.thresh1_20_lo = radar_thr.thresh1_20_lo;
		st->rparams.radar_thrs.thresh0_20_hi = radar_thr.thresh0_20_hi;
		st->rparams.radar_thrs.thresh1_20_hi = radar_thr.thresh1_20_hi;
		if (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi)) {
			st->rparams.radar_thrs.thresh0_40_lo = radar_thr.thresh0_40_lo;
			st->rparams.radar_thrs.thresh1_40_lo = radar_thr.thresh1_40_lo;
			st->rparams.radar_thrs.thresh0_40_hi = radar_thr.thresh0_40_hi;
			st->rparams.radar_thrs.thresh1_40_hi = radar_thr.thresh1_40_hi;
		}
		if (ISACPHY(pi)) {
			st->rparams.radar_thrs.thresh0_80_lo = radar_thr.thresh0_80_lo;
			st->rparams.radar_thrs.thresh1_80_lo = radar_thr.thresh1_80_lo;
			st->rparams.radar_thrs.thresh0_80_hi = radar_thr.thresh0_80_hi;
			st->rparams.radar_thrs.thresh1_80_hi = radar_thr.thresh1_80_hi;
		}
		phy_radar_detect_enable(pi, pi->sh->radar);
		break;
	}
	case IOV_SVAL(IOV_RADAR_ARGS): {
		wl_radar_args_t radarargs;

		if (!pi->sh->up) {
			err = BCME_NOTUP;
			break;
		}

		/* len is check done before gets here */
		bcopy(p, &radarargs, sizeof(wl_radar_args_t));
		if (radarargs.version != WL_RADAR_ARGS_VERSION) {
			err = BCME_VERSION;
			break;
		}
		bcopy(&radarargs, &st->rparams.radar_args, sizeof(wl_radar_args_t));
		/* apply radar inits to hardware if we are on the A/LP/NPHY */
		phy_radar_detect_enable(pi, pi->sh->radar);
		break;
	}
	case IOV_SVAL(IOV_PHY_DFS_LP_BUFFER):
		if (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi)) {
			pi->dfs_lp_buffer_nphy = bool_val;
		} else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_GVAL(IOV_RADAR_STATUS):
		if (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi)) {
			bcopy(&st->radar_status, a, sizeof(wl_radar_status_t));
		} else
			err = BCME_UNSUPPORTED;
		break;

	case IOV_SVAL(IOV_CLEAR_RADAR_STATUS):
		if (ISNPHY(pi) || ISHTPHY(pi) || ISACPHY(pi)) {
			st->radar_status.detected = FALSE;
			st->radar_status.count = 0;
		} else
			err = BCME_UNSUPPORTED;
		break;

	default:
		err = BCME_UNSUPPORTED;
		break;
	}

	return err;
}
#endif /* WLC_LOW */

#ifdef WLC_HIGH_ONLY

/* fixup callbacks */
static bool
phy_radar_pack_iov(wlc_info_t *wlc, uint32 aid, void *p, uint p_len, bcm_xdr_buf_t *b)
{
	switch (aid) {
	case IOV_SVAL(IOV_RADAR_ARGS):
	case IOV_SVAL(IOV_RADAR_ARGS_40MHZ): {
		wl_radar_args_t	*radar = (wl_radar_args_t*)p;

		radar->npulses = htol32(radar->npulses);
		radar->ncontig = htol32(radar->ncontig);
		radar->min_pw = htol32(radar->min_pw);
		radar->max_pw = htol32(radar->max_pw);
		radar->thresh0 = htol16(radar->thresh0);
		radar->thresh1 = htol16(radar->thresh1);
		radar->blank = htol16(radar->blank);
		radar->fmdemodcfg = htol16(radar->fmdemodcfg);
		radar->npulses_lp = htol32(radar->npulses_lp);
		radar->min_pw_lp = htol32(radar->min_pw_lp);
		radar->max_pw_lp = htol32(radar->max_pw_lp);
		radar->min_fm_lp = htol32(radar->min_fm_lp);
		radar->max_span_lp = htol32(radar->max_span_lp);
		radar->min_deltat = htol32(radar->min_deltat);
		radar->max_deltat = htol32(radar->max_deltat);
		radar->autocorr = htol16(radar->autocorr);
		radar->st_level_time = htol16(radar->st_level_time);
		radar->t2_min = htol16(radar->t2_min);
		radar->version = htol32(radar->version);
		radar->fra_pulse_err = htol32(radar->fra_pulse_err);
		radar->npulses_fra = htol32(radar->npulses_fra);
		radar->npulses_stg2 = htol32(radar->npulses_stg2);
		radar->npulses_stg3 = htol32(radar->npulses_stg3);
		radar->percal_mask = htol16(radar->percal_mask);
		radar->quant = htol32(radar->quant);
		radar->min_burst_intv_lp = htol32(radar->min_burst_intv_lp);
		radar->max_burst_intv_lp = htol32(radar->max_burst_intv_lp);
		radar->nskip_rst_lp = htol32(radar->nskip_rst_lp);
		radar->max_pw_tol = htol32(radar->max_pw_tol);
		radar->feature_mask = htol16(radar->feature_mask);
		break;
	}
	case IOV_SVAL(IOV_RADAR_THRS): {
		wl_radar_thr_t	*thr = (wl_radar_thr_t*)p;

		thr->version = htol32(thr->version);
		thr->thresh0_20_lo = htol32(thr->thresh0_20_lo);
		thr->thresh1_20_lo = htol16(thr->thresh1_20_lo);
		thr->thresh0_40_lo = htol16(thr->thresh0_40_lo);
		thr->thresh1_40_lo = htol16(thr->thresh1_40_lo);
		thr->thresh0_20_hi = htol16(thr->thresh0_20_hi);
		thr->thresh1_20_hi = htol16(thr->thresh1_20_hi);
		thr->thresh0_40_hi = htol16(thr->thresh0_40_hi);
		thr->thresh1_40_hi = htol16(thr->thresh1_40_hi);
		break;
	}
	}

	return FALSE;
}

static bool
phy_radar_unpack_iov(wlc_info_t *wlc, uint32 aid, bcm_xdr_buf_t *b, void *a, uint a_len)
{
	switch (aid) {
	case IOV_GVAL(IOV_RADAR_ARGS):
	case IOV_GVAL(IOV_RADAR_ARGS_40MHZ): {
		wl_radar_args_t	*radar = (wl_radar_args_t*)a;

		radar->npulses = ltoh32(radar->npulses);
		radar->ncontig = ltoh32(radar->ncontig);
		radar->min_pw = ltoh32(radar->min_pw);
		radar->max_pw = ltoh32(radar->max_pw);
		radar->thresh0 = ltoh16(radar->thresh0);
		radar->thresh1 = ltoh16(radar->thresh1);
		radar->blank = ltoh16(radar->blank);
		radar->fmdemodcfg = ltoh16(radar->fmdemodcfg);
		radar->npulses_lp = ltoh32(radar->npulses_lp);
		radar->min_pw_lp = ltoh32(radar->min_pw_lp);
		radar->max_pw_lp = ltoh32(radar->max_pw_lp);
		radar->min_fm_lp = ltoh32(radar->min_fm_lp);
		radar->max_span_lp = ltoh32(radar->max_span_lp);
		radar->min_deltat = ltoh32(radar->min_deltat);
		radar->max_deltat = ltoh32(radar->max_deltat);
		radar->autocorr = ltoh16(radar->autocorr);
		radar->st_level_time = ltoh16(radar->st_level_time);
		radar->t2_min = ltoh16(radar->t2_min);
		radar->version = ltoh32(radar->version);
		radar->fra_pulse_err = ltoh32(radar->fra_pulse_err);
		radar->npulses_fra = ltoh32(radar->npulses_fra);
		radar->npulses_stg2 = ltoh32(radar->npulses_stg2);
		radar->npulses_stg3 = ltoh32(radar->npulses_stg3);
		radar->percal_mask = ltoh16(radar->percal_mask);
		radar->quant = ltoh32(radar->quant);
		radar->min_burst_intv_lp = ltoh32(radar->min_burst_intv_lp);
		radar->max_burst_intv_lp = ltoh32(radar->max_burst_intv_lp);
		radar->nskip_rst_lp = ltoh32(radar->nskip_rst_lp);
		radar->max_pw_tol = ltoh32(radar->max_pw_tol);
		radar->feature_mask = ltoh16(radar->feature_mask);
		break;
	}
	}

	return FALSE;
}
#endif /* WLC_HIGH_ONLY */

/* register iovar table/handlers to the system */
int
BCMATTACHFN(phy_radar_register_iovt)(phy_info_t *pi, wlc_iocv_info_t *ii)
{
	wlc_iovt_desc_t iovd;

	ASSERT(ii != NULL);

	wlc_iocv_init_iovd(phy_radar_iovt,
	                   phy_radar_pack_iov, phy_radar_unpack_iov,
	                   phy_radar_doiovar, pi,
	                   &iovd);

	return wlc_iocv_register_iovt(ii, &iovd);
}
