/*
 * Implementation of wlc_key algo 'none'
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
 * $Id: km_key_none.c 523118 2014-12-26 18:53:31Z $
 */

#include "km_key_pvt.h"

/* internal interface */
static const key_algo_callbacks_t key_none_callbacks = {
	NULL,	/* destroy */
	NULL,   /* get data */
	NULL,	/* set data */
	NULL,	/* rx mpdu */
	NULL,	/* rx msdu */
	NULL,	/* tx mpdu */
	NULL,	/* tx msdu */
	NULL	/* dump */
};

/* public interface */

int
km_key_none_init(wlc_key_t *key)
{
	KM_DBG_ASSERT(key->info.algo == CRYPTO_ALGO_NONE);

	key->info.key_len = 0;
	key->info.iv_len = 0;
	key->info.icv_len = 0;

	key->algo_impl.cb = &key_none_callbacks;
	key->algo_impl.ctx = NULL;

	return BCME_OK;
}
