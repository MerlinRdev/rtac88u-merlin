/*
 * HTPHY RADIO control module interface (to other PHY modules).
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

#ifndef _phy_ht_radio_h_
#define _phy_ht_radio_h_

#include <phy_api.h>
#include <phy_ht.h>
#include <phy_radio.h>

/* forward declaration */
typedef struct phy_ht_radio_info phy_ht_radio_info_t;

/* register/unregister HTPHY specific implementations to/from common */
phy_ht_radio_info_t *phy_ht_radio_register_impl(phy_info_t *pi,
	phy_ht_info_t *hti, phy_radio_info_t *ri);
void phy_ht_radio_unregister_impl(phy_ht_radio_info_t *info);

uint32 phy_ht_radio_query_idcode(phy_info_t *pi);

#endif /* _phy_ht_radio_h_ */
