/*
 * Miscellaneous PHY module public interface (to MAC driver).
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
 * $Id: phy_misc_api.h 523116 2014-12-26 18:20:15Z $
 */

#ifndef _phy_misc_api_h_
#define _phy_misc_api_h_

#include <phy_api.h>


#define VASIP_NOVERSION 0xff

/*
 * Return vasip version, -1 if not present.
 */
uint8 phy_misc_get_vasip_ver(phy_info_t *pi);
/*
 * reset/activate vasip.
 */
void phy_misc_vasip_proc_reset(phy_info_t *pi, int reset);

#endif /* _phy_misc_api_h_ */
