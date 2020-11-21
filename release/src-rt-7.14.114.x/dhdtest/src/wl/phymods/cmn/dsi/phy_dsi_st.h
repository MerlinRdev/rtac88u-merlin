/*
 * DeepSleepInit module internal interface.
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

#ifndef _phy_dsi_st_
#define _phy_dsi_st_

#include <typedefs.h>
#include <phy_dsi.h>

typedef struct {
	bool    trigger;
} phy_dsi_state_t;

/*
 * Query the states pointer.
 */
phy_dsi_state_t *phy_dsi_get_st(phy_dsi_info_t *di);
#endif /* _phy_dsi_st_ */
