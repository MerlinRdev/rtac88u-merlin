/*
 * Key Management Module Implementation - scb support
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
 * $Id: km_scb.c 523118 2014-12-26 18:53:31Z $
 */

#include "km_pvt.h"

/* internal interface */

static void
km_scb_update_key(keymgmt_t *km, wlc_key_index_t key_idx,
	scb_t *scb, bool multi_band, wlc_key_info_t *key_info)
{
	wlc_key_t *key;
	wlc_key_info_t tmp_ki;

	KM_DBG_ASSERT(scb != NULL);

	if (key_info == NULL)
		key_info = &tmp_ki;

	key = km->keys[key_idx].key;
	wlc_key_get_info(key, key_info);
	if (multi_band)
		key_info->flags |= WLC_KEY_FLAG_MULTI_BAND;
	else
		key_info->flags &= ~WLC_KEY_FLAG_MULTI_BAND;

	km_key_set_flags(key, key_info->flags);
	km->keys[key_idx].u.scb = scb;
}

static void
km_scb_cleanup(keymgmt_t *km, scb_t *scb)
{
	km_scb_t *scb_km;
	scb_t *other_scb;
	wlc_key_id_t key_id;
	KM_LOG_DECL(char eabuf[ETHER_ADDR_STR_LEN]);

	KM_LOG(("wl%d: %s: scb@%p %s enter\n",  KM_UNIT(km), __FUNCTION__,
		scb, bcm_ether_ntoa(&scb->ea, eabuf)));

	/* ignore internal scb */
	if (SCB_INTERNAL(scb))
		goto done;

	(void)key_id;

	/* check for scb in other band with the same address. If it is present,
	 * do not destroy keys, as they are shared. Also update the scb
	 * reference in km_pvt_key to other scb to keep it valid.
	 */
	other_scb =  (KM_ADDR_IS_BCMC(&scb->ea) || KM_NBANDS(km) < 2)  ?
		NULL : wlc_scbfindband(km->wlc, SCB_BSSCFG(scb),
		&scb->ea, KM_OTHERBANDUNIT(scb->bandunit));

	scb_km = KM_SCB(km, scb);

	if (!other_scb)
		km_bsscfg_reset_sta_info(km, SCB_BSSCFG(scb), NULL);

	if (scb_km->key_idx != WLC_KEY_INDEX_INVALID) {
		if (!other_scb) {
			km_free_key_block(km, WLC_KEY_FLAG_NONE, &scb_km->key_idx, 1);
			KM_DBG_ASSERT(scb_km->key_idx == WLC_KEY_INDEX_INVALID);
		} else {
			km_scb_update_key(km, scb_km->key_idx, other_scb, FALSE, NULL);
			scb_km->key_idx = WLC_KEY_INDEX_INVALID;
		}
	}



	km_scb_amt_release(km, scb);

done:
	KM_LOG(("wl%d: %s: exit\n",  KM_UNIT(km), __FUNCTION__));
}

static int
km_scb_init_internal(keymgmt_t *km, scb_t *scb)
{
	int err = BCME_OK;
	km_scb_t *scb_km;
	wlc_key_flags_t alloc_flags;
	size_t num_keys;
	wlc_key_index_t key_idx_arr[2*WLC_KEYMGMT_NUM_STA_GROUP_KEYS + 1];
	size_t key_idx_arr_pos = 0;
	wlc_key_index_t key_idx;
	wlc_key_t *key;
	wlc_key_info_t key_info;
	km_pvt_key_t *km_pvt_key;
	bool swkeys = FALSE;
	scb_t *other_scb;
	km_scb_t *other_scb_km = NULL;
	bool multi_band = FALSE;
	km_flags_t km_flags;
	wlc_key_id_t key_id;
	wlc_bsscfg_t *bsscfg;
	KM_LOG_DECL(char eabuf[ETHER_ADDR_STR_LEN]);

	STATIC_ASSERT(WLC_KEYMGMT_NUM_STA_GROUP_KEYS == 2);

	KM_LOG(("wl%d: %s: scb@%p %s enter\n",  KM_UNIT(km), __FUNCTION__,
		scb, bcm_ether_ntoa(&scb->ea, eabuf)));

	/* ignore internal scb */
	if (SCB_INTERNAL(scb)) {
		goto done;
	}

	(void)key_id;

	scb_km = KM_SCB(km, scb);
	bsscfg = SCB_BSSCFG(scb);
	KM_DBG_ASSERT(bsscfg != NULL);

	/* if scb exists in another band for the same bsscfg and address, use the key
	 * indicies allocated for it
	 */
	other_scb = (KM_ADDR_IS_BCMC(&scb->ea) || KM_NBANDS(km) < 2)  ?  NULL :
		wlc_scbfindband(km->wlc, bsscfg, &scb->ea,
		KM_OTHERBANDUNIT(scb->bandunit));

	if (other_scb) {
		other_scb_km = KM_SCB(km, other_scb);
		multi_band = TRUE;
		*scb_km = *other_scb_km;

		/* If keys have not been allocated, allocate them now. If keys are present
		 * mark them as multi-band
		 */
		if (scb_km->key_idx != WLC_KEY_INDEX_INVALID) {
			key_idx = scb_km->key_idx;

			km_scb_update_key(km, key_idx, scb, TRUE, &key_info);
			km_bsscfg_reset_sta_info(km, bsscfg, &key_info);

			goto done;
		}
	}

	swkeys = km_bsscfg_swkeys(km, bsscfg);

	alloc_flags = WLC_KEY_FLAG_NONE;
	num_keys = 1;	/* only pairwise key, by default */

	scb_km->key_idx = WLC_KEY_INDEX_INVALID;
	scb_km->amt_idx = KM_HW_AMT_IDX_INVALID;


	/* if BSS has no security, we can defer the rest since the key idx(s) in
	 * scb are now invalid. Nothing more for bcmc scb's, but the above init
	 * will allow them to be reused.
	 */
	if (KM_ADDR_IS_BCMC(&scb->ea) || !bsscfg->wsec)
		goto done;

	if (BSSCFG_AP(bsscfg))
		alloc_flags |= WLC_KEY_FLAG_AP;

	err = km_alloc_key_block(km, alloc_flags, key_idx_arr, num_keys);
	if (err != BCME_OK)
		goto done;

	/* create the pairwise key */
	key_idx = key_idx_arr[key_idx_arr_pos++];

	memset(&key_info, 0, sizeof(key_info));
	key_info.key_idx = key_idx;
	key_info.addr = scb->ea;
	key_info.flags = alloc_flags | WLC_KEY_FLAG_RX | WLC_KEY_FLAG_TX;
	if (multi_band)
		key_info.flags |= WLC_KEY_FLAG_MULTI_BAND;
	key_info.key_id = WLC_KEY_ID_PAIRWISE;
	err = km_key_create(km, &key_info, &key);
	if (err != BCME_OK) {
		km_free_key_block(km, alloc_flags, key_idx_arr, num_keys);
		goto done;
	}

	scb_km->key_idx = key_idx;

	km_pvt_key = &km->keys[key_idx];
	km_flags = KM_FLAG_TX_KEY | KM_FLAG_SCB_KEY;
	if (swkeys)
		km_flags |= KM_FLAG_SWONLY_KEY;

	km_init_pvt_key(km, km_pvt_key, CRYPTO_ALGO_NONE, key, km_flags, NULL, scb);
	km_bsscfg_reset_sta_info(km, bsscfg, &key_info);



	/* assign created keys to other scb if applicable */
	if (multi_band) {
		KM_DBG_ASSERT(other_scb_km != NULL);
		*other_scb_km = *scb_km;
	}

done:
	KM_LOG(("wl%d: %s: scb %s exit status %d\n",  KM_UNIT(km), __FUNCTION__,
		bcm_ether_ntoa(&scb->ea, eabuf), err));

	return BCME_OK;
}

/* public interface */
int
km_scb_init(void *ctx, scb_t *scb)
{
	keymgmt_t *km = (keymgmt_t *)ctx;
	km_scb_t *scb_km;

	KM_DBG_ASSERT(KM_VALID(km) && scb != NULL);
	scb_km = KM_SCB(km, scb);
	memset(scb_km, 0, sizeof(*scb_km));
	return km_scb_init_internal(km, scb);
}

void
km_scb_deinit(void *ctx,  scb_t *scb)
{
	keymgmt_t *km = (keymgmt_t *)ctx;

	KM_DBG_ASSERT(KM_VALID(km) && scb != NULL);
	km_scb_cleanup(km, scb);
}

void
km_scb_reset(keymgmt_t *km, scb_t *scb)
{
	KM_LOG_DECL(char eabuf[ETHER_ADDR_STR_LEN]);

	KM_DBG_ASSERT(KM_VALID(km));
	KM_DBG_ASSERT(scb != NULL);
	KM_LOG(("wl%d: %s: scb %s enter\n",  KM_UNIT(km), __FUNCTION__,
		bcm_ether_ntoa(&scb->ea, eabuf)));

	km_scb_cleanup(km, scb);
	km_scb_init_internal(km, scb);

	KM_LOG(("wl%d: %s: scb %s exit\n",  KM_UNIT(km), __FUNCTION__,
		bcm_ether_ntoa(&scb->ea, eabuf)));
}

wlc_key_t*
wlc_keymgmt_get_scb_key(wlc_keymgmt_t *km, scb_t *scb,
	wlc_key_id_t key_id, wlc_key_flags_t key_flags, wlc_key_info_t *key_info)
{
	km_scb_t *scb_km;
	wlc_key_index_t key_idx;
	km_pvt_key_t *km_pvt_key;
	wlc_key_t *key;
	wlc_key_info_t key_info_s;

	KM_DBG_ASSERT(KM_VALID(km));
	KM_ASSERT(scb != NULL);

	if (!key_info)
		key_info = &key_info_s;

	if (KM_IGNORED_SCB(scb))
		goto null_key;

	scb_km = KM_SCB(km, scb);
	if (!(key_flags & WLC_KEY_FLAG_GROUP) &&
		!(key_flags & WLC_KEY_FLAG_IBSS_PEER_GROUP)) { /* pairwise key */
		key_idx = scb_km->key_idx;
		if (!KM_VALID_KEY_IDX(km, key_idx))
			goto null_key;

		km_pvt_key = &km->keys[key_idx];
		KM_DBG_ASSERT(KM_VALID_KEY(km_pvt_key));

		wlc_key_get_info(km_pvt_key->key, key_info);

		/* scb key will have pairwise key id (0) except for wapi */
		if (key_id == key_info->key_id) {
			key = km_pvt_key->key;
			goto done;
		}

	}
	else if (KM_BSSCFG_NEED_STA_GROUP_KEYS(km, SCB_BSSCFG(scb)) &&
		(key_flags & WLC_KEY_FLAG_GROUP) && WLC_KEY_ID_IS_DEFAULT(key_id)) {
			key = wlc_keymgmt_get_bss_key(km, SCB_BSSCFG(scb), key_id, key_info);
			goto done;
	}

null_key:
	key = km->null_key;
	wlc_key_get_info(key, key_info);

done:
	return key;
}

wlc_key_t*
wlc_keymgmt_get_tx_key(wlc_keymgmt_t *km, scb_t *scb,
	wlc_bsscfg_t *bsscfg, wlc_key_info_t *key_info)
{
	wlc_key_t *key = NULL;

	if (scb) {
		const km_key_cache_t *kc = km->key_cache;
		if (kc && !memcmp(&scb->ea, &kc->key_info->addr, sizeof(scb->ea))) {
			key = kc->key;
			if (key_info)
				*key_info = *kc->key_info;
			goto done;
		}

		key = wlc_keymgmt_get_scb_key(km, scb, WLC_KEY_ID_PAIRWISE,
			WLC_KEY_FLAG_NONE, key_info);
	} else {
		key = wlc_keymgmt_get_bss_tx_key(km, bsscfg, FALSE, key_info);
	}

done:
	return key;
}

#ifdef BRCMAPIVTW
int
wlc_keymgmt_ivtw_enable(wlc_keymgmt_t *km, scb_t *scb, bool enable)
{
	int err = BCME_OK;
	km_scb_t *scb_km;
	wlc_bsscfg_t *bsscfg;

	KM_DBG_ASSERT(KM_VALID(km));
	if (scb == NULL)
		goto done;

	bsscfg = SCB_BSSCFG(scb);
	KM_DBG_ASSERT(bsscfg != NULL);

	/* no support for ivtw on non STA and IBSS */
	if (!BSSCFG_STA(bsscfg) || !bsscfg->BSS)
		enable = FALSE;

	scb_km = KM_SCB(km, scb);
	if (KM_VALID_KEY_IDX(km, scb_km->key_idx))
		err = km_ivtw_enable(km->ivtw, scb_km->key_idx, enable);
	err =  km_bsscfg_ivtw_enable(km, bsscfg, enable);
done:
	return err;
}
#endif /* BRCMAPIVTW */

void
km_scb_state_upd(keymgmt_t *km, scb_state_upd_data_t *data)
{
	scb_t *scb;
	wlc_bsscfg_t *bsscfg;
	wlc_key_t *key;
	wlc_key_info_t key_info;

	KM_DBG_ASSERT(data != NULL && data->scb != NULL);

	scb = data->scb;
	if (KM_IGNORED_SCB(scb))
		return;

	bsscfg = SCB_BSSCFG(scb);

	/* clear scb key on state update when wpa scb is unauthorized */
	key = wlc_keymgmt_get_scb_key(km, scb, WLC_KEY_ID_PAIRWISE, WLC_KEY_FLAG_NONE, &key_info);
	if ((key_info.algo != CRYPTO_ALGO_OFF) && !SCB_AUTHORIZED(scb) &&
		(bsscfg->WPA_auth != WPA_AUTH_DISABLED) && (bsscfg->WPA_auth != WPA_AUTH_NONE)) {
		wlc_key_set_data(key, CRYPTO_ALGO_OFF, NULL, 0);
		if (BSSCFG_STA(bsscfg))
			km_bsscfg_reset(km, bsscfg, FALSE);
	}

	/* note: scb amt is released when the scb is reset - even though
	 * it is not needed once the sta is disassociated because the process
	 * of destroying the key clears key data and thus may use the key/amt idx
	 */

#ifdef BRCMAPIVTW
	/* update ivtw for key as necessary */
	{
		int ivtw_mode;
		bool enable = FALSE;

		if (key_info.key_idx == WLC_KEY_INDEX_INVALID ||
			bsscfg->WPA_auth == WPA_AUTH_DISABLED ||
			!SCB_ASSOCIATED(scb)) {
			goto upd_ivtw;
		}

		ivtw_mode = km_ivtw_get_mode(km->ivtw);
		if (ivtw_mode == ON) {
			enable = TRUE;
		}
#ifdef WLAMPDU_HOSTREORDER
		else if (AMPDU_HOST_REORDER_ENAB(KM_PUB(km))) {
			if (ivtw_mode == OFF)
				(void)km_ivtw_set_mode(km->ivtw, AUTO);
			enable = TRUE;
		}
#endif /* WLAMPDU_HOSTREORDER */

upd_ivtw:
		(void)wlc_keymgmt_ivtw_enable(km, scb, enable);
	}
#endif /* BRCMAPIVTW */
}

km_amt_idx_t
km_scb_amt_alloc(keymgmt_t *km, scb_t *scb)
{
	km_amt_idx_t amt_idx;
	km_scb_t *scb_km;
	const wlc_bsscfg_t *bsscfg;
	const struct ether_addr *ea;
	km_amt_attr_t amt_attr;
	KM_LOG_DECL(char eabuf[ETHER_ADDR_STR_LEN]);

	KM_DBG_ASSERT(KM_VALID(km) && scb != NULL);

	bsscfg = SCB_BSSCFG(scb);
	KM_DBG_ASSERT(bsscfg != NULL);

	ea = &scb->ea;
#ifdef PSTA
	if (BSSCFG_STA(bsscfg) && PSTA_ENAB(KM_PUB(km)))
		ea = &bsscfg->cur_etheraddr; /* PSTA uses A1 match */
#endif

	scb_km = KM_SCB(km, scb);
	amt_idx = scb_km->amt_idx;
	if (amt_idx != KM_HW_AMT_IDX_INVALID) {
		/* if wlc is reinitialized - i.e. s/w state is preserved, but km_hw
		 * is reset, any reservation for this amt index is lost; so restore it
		 */
		km_hw_amt_reserve(km->hw, amt_idx, 1, TRUE);
		goto done;
	}

	/* use bss amt allocation for a sta, if available */
	if (bsscfg->BSS && BSSCFG_STA(bsscfg)) {
		amt_idx = km_bsscfg_get_amt_idx(km, bsscfg);
		if (amt_idx != KM_HW_AMT_IDX_INVALID)
			goto upd_amt;
	}

	/* there is no sta amt entry for default bss scb without wpa unless wep is not
	 * enabled and there is no beamforming. Beamforming is a vht feature that
	 * requires an amt entry - and vht does not allow wep
	 */
	if (KM_IS_DEFAULT_BSSCFG(km, bsscfg) && !BSSCFG_AP(bsscfg) &&
		(bsscfg->WPA_auth == WPA_AUTH_DISABLED) &&
		(WSEC_WEP_ENABLED(bsscfg->wsec) || !TXBF_ENAB(KM_PUB(km)))) {
		goto done;
	}

	/* reserve and own an amt entry */
	amt_idx = km_hw_amt_alloc(km->hw, ea);
	if (amt_idx == KM_HW_AMT_IDX_INVALID)
		goto done;

	scb_km->flags |= KM_SCB_FLAG_OWN_AMT;

upd_amt:
	/* preserve any valid amt attributes */
	amt_attr = wlc_clear_addrmatch(km->wlc, amt_idx);
	if (!(amt_attr & AMT_ATTR_VALID))
		amt_attr = 0;
	amt_attr |= (AMT_ATTR_VALID | AMT_ATTR_A2);
	wlc_set_addrmatch(km->wlc, amt_idx, ea, amt_attr);

	scb_km->amt_idx = amt_idx;

done:
	KM_LOG(("wl%d.%d: %s: reserved amt index %d for addr %s\n",
		KM_UNIT(km), WLC_BSSCFG_IDX(bsscfg), __FUNCTION__,
		amt_idx,  bcm_ether_ntoa(ea, eabuf)));
	return amt_idx;
}

void
km_scb_amt_release(keymgmt_t *km, scb_t *scb)
{
	km_scb_t *scb_km;

	KM_DBG_ASSERT(KM_VALID(km) && scb != NULL);

	scb_km = KM_SCB(km, scb);
	KM_LOG(("wl%d.%d: %s: releasing amt index %d\n",
		KM_UNIT(km), WLC_BSSCFG_IDX(SCB_BSSCFG(scb)), __FUNCTION__, scb_km->amt_idx));

	if (scb_km->amt_idx == KM_HW_AMT_IDX_INVALID)
		goto done;

#ifdef PSTA
	if (BSSCFG_PSTA(SCB_BSSCFG(scb)))
		km_hw_amt_reserve(km->hw, scb_km->amt_idx, 1, FALSE);
#endif

	if (!KM_SCB_OWN_AMT(scb_km)) {
		scb_km->amt_idx = KM_HW_AMT_IDX_INVALID;
		goto done;
	}

	km_hw_amt_release(km->hw, &scb_km->amt_idx);
	scb_km->flags &= ~KM_SCB_FLAG_OWN_AMT;

done:
	KM_DBG_ASSERT(scb_km->amt_idx == KM_HW_AMT_IDX_INVALID);
}

int wlc_keymgmt_get_scb_amt_idx(wlc_keymgmt_t *km, scb_t *scb)
{
	km_amt_idx_t amt_idx;
	int err;

	KM_ASSERT(KM_VALID(km));

	err = BCME_NOTFOUND;
	if (!scb || KM_IGNORED_SCB(scb))
		goto done;

	amt_idx = km_scb_amt_alloc(km, scb);
	if (amt_idx != KM_HW_AMT_IDX_INVALID)
		err = amt_idx;

done:
	return err;
}
