/*
 * DeepSleepInit module internal interface (to PHY specific implementation).
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

#ifndef _phy_type_dsi_h_
#define _phy_type_dsi_h_

#include <typedefs.h>
#include <phy_dsi.h>

/*
 * PHY type implementation interface.
 *
 * Each PHY type implements the following functionality and registers the functions
 * via a vtbl/ftbl defined below, along with a context 'ctx' pointer.
 */
typedef void phy_type_dsi_ctx_t;
typedef void (*phy_type_dsi_restore_fn_t)(phy_type_dsi_ctx_t *ctx);
typedef int  (*phy_type_dsi_save_fn_t)(phy_type_dsi_ctx_t *ctx);

typedef struct {
	phy_type_dsi_ctx_t *ctx;
	phy_type_dsi_restore_fn_t restore;
	phy_type_dsi_save_fn_t save;
} phy_type_dsi_fns_t;

/*
 * Register/unregister PHY type implementation to the common of the DSI module.
 * It returns BCME_XXXX.
 */
int phy_dsi_register_impl(phy_dsi_info_t *ri, phy_type_dsi_fns_t *fns);
void phy_dsi_unregister_impl(phy_dsi_info_t *ri);

#endif /* _phy_type_dsi_h_ */
