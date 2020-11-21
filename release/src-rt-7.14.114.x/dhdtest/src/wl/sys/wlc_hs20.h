/*
 * Hotspot 2.0 module
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
 * $Id: wlc_hs20.h 523117 2014-12-26 18:32:49Z $
 */


#ifndef _wlc_hs20_h_
#define _wlc_hs20_h_

#include <wlc_cfg.h>
#include <d11.h>
#include <wlc_types.h>
#include <wlc_bsscfg.h>

/*
 * Initialize hotspot private context.
 * Returns a pointer to the hotspot private context, NULL on failure.
 */
extern wlc_hs20_info_t *wlc_hs20_attach(wlc_info_t *wlc);

/* Cleanup hotspot private context */
extern void wlc_hs20_detach(wlc_hs20_info_t *hs20);

/* return TRUE if OSEN enabled */
extern bool wlc_hs20_is_osen(wlc_hs20_info_t *hs20, wlc_bsscfg_t *cfg);

#endif /* _wlc_hs20_h_ */
