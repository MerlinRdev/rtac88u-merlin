/*
 * Internal interface to wlc_key implementation
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
 * $Id: km_key.h 523118 2014-12-26 18:53:31Z $
 */

#ifndef _km_key_h_
#define _km_key_h_

#include <wlc_key.h>

#define KM_KEY_MAX_DATA_LEN 32

/* key dump types */
enum {
    KM_KEY_DUMP_NONE   		= 0x00,
    KM_KEY_DUMP_SW_KEYS     = 0x01,
    KM_KEY_DUMP_HW_KEYS     = 0x02,
    KM_KEY_DUMP_SECALGO     = 0x04,
    KM_KEY_DUMP_ALL    		= 0xff
};
typedef int km_key_dump_type_t;

int km_key_create(wlc_keymgmt_t *km, const wlc_key_info_t *key_info,
	wlc_key_t **key);
int km_key_destroy(wlc_key_t **key);

#if defined(BCMDBG)
void km_key_dump(wlc_key_t *key, struct bcmstrbuf *buf);
#endif 

int km_key_get_data(wlc_key_t *key, uint8* data, size_t data_size,
    size_t *data_len, wlc_key_data_type_t type, int instance, bool tx);

/* internal interfaces have additional settable key flags */
#define KM_KEY_SETTABLE_FLAGS (WLC_KEY_SETTABLE_FLAGS |\
	WLC_KEY_FLAG_TX | WLC_KEY_FLAG_RX |\
	WLC_KEY_FLAG_IBSS |\
	WLC_KEY_FLAG_IN_HW | WLC_KEY_FLAG_HW_MIC |\
	WLC_KEY_FLAG_LINUX_CRYPTO |\
	WLC_KEY_FLAG_MULTI_BAND |\
	WLC_KEY_FLAG_GTK_RESET |\
	WLC_KEY_FLAG_WAPI_HAS_PREV_KEY |\
	WLC_KEY_FLAG_NO_REPLAY_CHECK |\
	WLC_KEY_FLAG_ARM_TX_ENABLED |\
	0)

/* suppress notification for these flag changes */
#define KM_KEY_FLAGS_NOTIFY_MASK(_flags)  ((_flags) & ~(WLC_KEY_FLAG_GTK_RESET | \
	WLC_KEY_FLAG_NO_REPLAY_CHECK | \
	WLC_KEY_FLAG_ARM_TX_ENABLED | \
	0))

/* helper macros */

#define KEY_SEQ_SIZE 6 			/* key seq/PN size, except WAPI */

/* increment an seq in LE order */
#define KEY_SEQ_INCR(_seq, _sz) do {\
	size_t _i; \
	for (_i = 0; _i < (_sz); ++(_i))\
		if (++((_seq)[_i]))\
			break; \
	} while (0)

#define KEY_SEQ_DECR(_seq, _sz) do {\
	size_t _i; \
	for (_i = 0; _i < (_sz); ++(_i))\
		if (((_seq)[_i])--)\
			break; \
	} while (0)

#define KEY_SEQ_HI32(_seq) ltoh32_ua(&(_seq)[2])
#define KEY_SEQ_LO16(_seq) ltoh16_ua(&(_seq)[0])
#define KEY_SEQ_FORMAT "0x%02x%02x%02x%02x%02x%02x"
#define KEY_SEQ_FORMAT_ARG(_seq) (_seq)[5], (_seq)[4], (_seq)[3], \
	 (_seq)[2], (_seq)[1], (_seq)[0]

#define KEY_SEQ_IS_MAX(_seq) ((_seq)[0] == 0xff && (_seq)[1] == 0xff && \
		(_seq)[2] == 0xff && (_seq)[3] == 0xff && \
		(_seq)[4] == 0xff && (_seq)[5] == 0xff)

/* Cached used to optimize key lookup on rx/tx */
struct km_key_cache {
	wlc_key_t *key;
	const wlc_key_info_t *key_info;
	wlc_key_hw_index_t hw_idx;
};
typedef struct km_key_cache  km_key_cache_t;

/* prototypes */
void km_key_set_flags(wlc_key_t *key, wlc_key_flags_t flags);

/* stores input hw idx and returns previous one */
wlc_key_hw_index_t km_key_set_hw_idx(wlc_key_t *key,
	wlc_key_hw_index_t hw_idx, bool hw_mic);

/* check (rx) seq for replay */
bool km_key_seq_less(const uint8* seq1, const uint8* seq2, size_t seq_len);


/* shared key war: fixup h/w use of wep default keys for non-default bss */
int km_key_wep_rx_defkey_fixup(wlc_key_t *rx_key, uint8 *body, int body_len);

/* reset keymgmt key cache info */
void km_key_update_key_cache(wlc_key_t *key, km_key_cache_t *key_cache);

#endif /* _km_key_h_ */
