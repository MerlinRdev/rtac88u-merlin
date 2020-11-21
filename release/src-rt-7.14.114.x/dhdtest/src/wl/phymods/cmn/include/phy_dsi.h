/*
 * DeepSleepInit module internal interface
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
 * $Id: $
 */

#ifndef _phy_dsi_h_
#define _phy_dsi_h_

#include <typedefs.h>
#include <phy_api.h>

/* forward declaration */
typedef struct phy_dsi_info phy_dsi_info_t;

/* attach/detach */
extern phy_dsi_info_t *phy_dsi_attach(phy_info_t *pi);
extern void phy_dsi_detach(phy_dsi_info_t *di);

extern void BCMINITFN(phy_dsi_restore)(phy_info_t *pi);
extern int  phy_dsi_save(phy_info_t *pi);

extern bool phy_get_dsi_trigger_st(phy_info_t *pi);
#endif /* _phy_dsi_h_ */
