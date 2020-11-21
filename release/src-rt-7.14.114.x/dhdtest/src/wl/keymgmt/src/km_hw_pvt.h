/*
 * Key Management Module  Implementation - private header for km_hw
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
 * $Id: km_hw_pvt.h 523118 2014-12-26 18:53:31Z $
 */

#ifndef _km_hw_pvt_h_
#define _km_hw_pvt_h_

#include "km_hw_impl.h"

/* allocation support */
hw_idx_t hw_idx_alloc_specific(km_hw_t *hw, hw_idx_t hw_idx, skl_idx_t skl_idx);
void hw_idx_release(km_hw_t *hw, hw_idx_t hw_idx_start, size_t count,
	const struct ether_addr *ea);
hw_idx_t hw_idx_alloc(km_hw_t *hw, size_t count, km_alloc_key_info_t *alloc_info);
bool hw_idx_delete_ok(km_hw_t *hw, hw_idx_t hw_idx, const km_alloc_key_info_t *info);
void hw_get_hw_idx_from_alloc_info(km_hw_t *hw, const wlc_key_info_t *key_info,
	km_alloc_key_info_t *alloc_info, hw_idx_t *hw_idx);
const struct ether_addr*
hw_amt_get_addr(km_hw_t *hw, const wlc_key_info_t *key_info,
	const wlc_bsscfg_t *bsscfg, const scb_t *scb);
void hw_amt_update(km_hw_t *hw, amt_idx_t amt_idx, const struct ether_addr *ea);

#endif /* _km_hw_pvt_h_ */
