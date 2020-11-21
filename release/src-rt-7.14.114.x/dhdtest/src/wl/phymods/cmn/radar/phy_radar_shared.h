/*
 * RadarDetect module internal interface (functions sharde by PHY type specific implementations).
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

#ifndef _phy_radar_shared_h_
#define _phy_radar_shared_h_

#include <typedefs.h>
#include <phy_api.h>

/*
 * Run the radar detect algorithm.
 */
int phy_radar_run_nphy(phy_info_t *pi);

#endif /* _phy_radar_shared_h_ */
