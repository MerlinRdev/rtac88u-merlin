/*
 * ACPHY Calibration Manager module implementation
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
#include <bcmdefs.h>
#include <phy_dbg.h>
#include <phy_mem.h>
#include "phy_type_calmgr.h"
#include <phy_ac.h>
#include <phy_ac_calmgr.h>

/* ************************ */
/* Modules used by this module */
/* ************************ */
#include <wlc_radioreg_20691.h>
#include <wlc_phy_radio.h>
#include <wlc_phyreg_ac.h>
#include <wlc_phy_int.h>

#include <phy_utils_math.h>
#include <phy_utils_reg.h>
#include <phy_ac_info.h>


/* module private states */
struct phy_ac_calmgr_info {
	phy_info_t *pi;
	phy_ac_info_t *aci;
	phy_calmgr_info_t *cmn_info;
	/* cache coeffs */
	txcal_coeffs_t txcal_cache[PHY_CORE_MAX];
};

/* local functions */
static int phy_ac_calmgr_prepare(phy_type_calmgr_ctx_t *ctx);
static void phy_ac_calmgr_cleanup(phy_type_calmgr_ctx_t *ctx);

/* register phy type specific implementation */
phy_ac_calmgr_info_t *
BCMATTACHFN(phy_ac_calmgr_register_impl)(phy_info_t *pi, phy_ac_info_t *aci,
	phy_calmgr_info_t *cmn_info)
{
	phy_ac_calmgr_info_t *ac_info;
	phy_type_calmgr_fns_t fns;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* allocate all storage together */
	if ((ac_info = phy_malloc(pi, sizeof(phy_ac_calmgr_info_t))) == NULL) {
		PHY_ERROR(("%s: phy_malloc failed\n", __FUNCTION__));
		goto fail;
	}
	ac_info->pi = pi;
	ac_info->aci = aci;
	ac_info->cmn_info = cmn_info;

	/* register PHY type specific implementation */
	bzero(&fns, sizeof(fns));
	fns.prepare = phy_ac_calmgr_prepare;
	fns.cleanup = phy_ac_calmgr_cleanup;
	fns.ctx = ac_info;

	if (phy_calmgr_register_impl(cmn_info, &fns) != BCME_OK) {
		PHY_ERROR(("%s: phy_calmgr_register_impl failed\n", __FUNCTION__));
		goto fail;
	}

	return ac_info;

	/* error handling */
fail:
	if (ac_info != NULL)
		phy_mfree(pi, ac_info, sizeof(phy_ac_calmgr_info_t));
	return NULL;
}

void
BCMATTACHFN(phy_ac_calmgr_unregister_impl)(phy_ac_calmgr_info_t *ac_info)
{
	phy_info_t *pi;
	phy_calmgr_info_t *cmn_info;

	ASSERT(ac_info);
	pi = ac_info->pi;
	cmn_info = ac_info->cmn_info;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* unregister from common */
	phy_calmgr_unregister_impl(cmn_info);

	phy_mfree(pi, ac_info, sizeof(phy_ac_calmgr_info_t));
}

static int
phy_ac_calmgr_prepare(phy_type_calmgr_ctx_t *ctx)
{
	PHY_TRACE(("%s\n", __FUNCTION__));

	return BCME_OK;
}

static void
phy_ac_calmgr_cleanup(phy_type_calmgr_ctx_t *ctx)
{
	PHY_TRACE(("%s\n", __FUNCTION__));
}

void
wlc_phy_cal_coeffs_upd(phy_info_t *pi, txcal_coeffs_t *txcal_cache)
{
	uint8 core;
	uint16 ab_int[2];

	FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
		ab_int[0] = txcal_cache[core].txa;
		ab_int[1] = txcal_cache[core].txb;
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_WRITE,
			ab_int, TB_OFDM_COEFFS_AB, core);
		wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_WRITE,
			&txcal_cache[core].txd,	TB_OFDM_COEFFS_D, core);
		if (!TINY_RADIO(pi)) {
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_FINE_I(core),
			                         txcal_cache[core].txei);
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_FINE_Q(core),
			                         txcal_cache[core].txeq);
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_COARSE_I(core),
			                txcal_cache[core].txfi);
			phy_utils_write_radioreg(pi, RF_2069_TXGM_LOFT_COARSE_Q(core),
			                txcal_cache[core].txfq);
		}
		WRITE_PHYREGCE(pi, Core1RxIQCompA, core, txcal_cache[core].rxa);
		WRITE_PHYREGCE(pi, Core1RxIQCompB, core, txcal_cache[core].rxb);
	}
}

#if !defined(PHYCAL_CACHING)
void
wlc_phy_scanroam_cache_cal_acphy(phy_ac_calmgr_info_t *calmgri, bool set)
{
	uint16 ab_int[2];
	uint8 core;
	phy_info_t *pi;
	phy_info_acphy_t *pi_ac;

	pi = calmgri->pi;
	pi_ac = calmgri->aci;


	/* Prepare Mac and Phregs */
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);

	PHY_TRACE(("wl%d: %s: in scan/roam set %d\n", pi->sh->unit, __FUNCTION__, set));

	if (set) {
		PHY_CAL(("wl%d: %s: save the txcal for scan/roam\n",
			pi->sh->unit, __FUNCTION__));
		/* save the txcal to cache */
		FOREACH_ACTV_CORE(pi, pi->sh->phyrxchain, core) {
			wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ,
				ab_int, TB_OFDM_COEFFS_AB, core);
			calmgri->txcal_cache[core].txa = ab_int[0];
			calmgri->txcal_cache[core].txb = ab_int[1];
			wlc_phy_cal_txiqlo_coeffs_acphy(pi, CAL_COEFF_READ,
			&calmgri->txcal_cache[core].txd,
				TB_OFDM_COEFFS_D, core);
			if (!TINY_RADIO(pi)) {
				calmgri->txcal_cache[core].txei =
					(uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_FINE_I, core);
				calmgri->txcal_cache[core].txeq =
					(uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_FINE_Q, core);
				calmgri->txcal_cache[core].txfi =
					(uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_COARSE_I, core);
				calmgri->txcal_cache[core].txfq =
					(uint8)READ_RADIO_REGC(pi, RF, TXGM_LOFT_COARSE_Q, core);
			}

			calmgri->txcal_cache[core].rxa =
				READ_PHYREGCE(pi, Core1RxIQCompA, core);
			calmgri->txcal_cache[core].rxb =
				READ_PHYREGCE(pi, Core1RxIQCompB, core);
		}

		/* mark the cache as valid */
		pi_ac->txcal_cache_cookie = TXCAL_CACHE_VALID;
	} else {
		if (pi_ac->txcal_cache_cookie == TXCAL_CACHE_VALID) {
			PHY_CAL(("wl%d: %s: restore the txcal after scan/roam\n",
				pi->sh->unit, __FUNCTION__));
			/* restore the txcal from cache */
			wlc_phy_cal_coeffs_upd(pi, calmgri->txcal_cache);
		}
	}

	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);
}
#endif /* !defined(PHYCAL_CACHING) */
