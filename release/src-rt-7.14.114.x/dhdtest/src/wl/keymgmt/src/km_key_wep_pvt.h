/*
 * private interface for wlc_key algo 'wep'
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
 * $Id: km_key_wep_pvt.h 523118 2014-12-26 18:53:31Z $
 */

#ifndef km_key_wep_pvt_h_
#define km_key_wep_pvt_h_

#include "km_key_pvt.h"

#include <bcmcrypto/rc4.h>
#include <bcmcrypto/wep.h>


#define WEP_KEY_ALLOC_SIZE ROUNDUP(WEP128_KEY_SIZE, 16)
#define WEP_RC4_IV_SIZE (DOT11_IV_LEN - 1)
#define WEP_RC4_ALLOC_SIZE (WEP_RC4_IV_SIZE + WEP_KEY_ALLOC_SIZE)

#define WEP_KEY_VALID(_key) ((((_key)->info.algo == CRYPTO_ALGO_WEP1) ||\
		((_key)->info.algo == CRYPTO_ALGO_WEP128)) &&\
		((_key)->info.key_len <= WEP_KEY_ALLOC_SIZE) &&\
		((_key)->info.iv_len <= DOT11_IV_LEN))



/* context data type for wep. note that wep has two key sizes
 * as selected by key algo, and no replay protection. hence there
 * is no need to allocate rx seq (iv, replay counter).
 */
struct wep_key {
	uint8 key[WEP_KEY_ALLOC_SIZE];		/* key data */
	uint8 tx_seq[DOT11_IV_LEN];			/* LE order - need only 24 bits */

};

typedef struct wep_key wep_key_t;


#endif /* km_key_wep_pvt_h_ */
