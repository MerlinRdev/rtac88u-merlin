/*
 * Key Management Module Implementation - WOWL support
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
 * $Id: km_wowl_hw.h 523118 2014-12-26 18:53:31Z $
 */

#ifndef _km_wowl_hw_h_
#define _km_wowl_hw_h_


#include "km.h"
#include "km_key.h"
#include "km_hw.h"

km_hw_t *km_wowl_hw_attach(wlc_info_t *wlc, wlc_keymgmt_t *km);
void km_wowl_hw_detach(km_hw_t **hw);
void km_wowl_hw_set_mode(km_hw_t *hw, scb_t *scb, bool enable);

#endif /* _km_wowl_hw_h_ */
