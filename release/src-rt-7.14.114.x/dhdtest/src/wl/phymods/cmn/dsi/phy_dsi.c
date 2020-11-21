/*
 * DeepSleepInit module implementation
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

#include <phy_cfg.h>
#include <typedefs.h>
#include <phy_mem.h>

#include <phy_dsi_api.h>
#include <phy_dsi.h>
#include "phy_dsi_st.h"
#include "phy_type_dsi.h"

/* module private states */
struct phy_dsi_info {
	phy_info_t *pi;
	phy_type_dsi_fns_t *fns;
	phy_dsi_state_t *st;
};

/* module private states memory layout */
typedef struct {
	phy_dsi_info_t info;
	phy_type_dsi_fns_t fns;
	phy_dsi_state_t st;
} phy_dsi_mem_t;

/* attach/detach */
phy_dsi_info_t *
BCMATTACHFN(phy_dsi_attach)(phy_info_t *pi)
{
	phy_dsi_info_t *info;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* allocate attach info storage */
	if ((info = phy_malloc(pi, sizeof(phy_dsi_mem_t))) == NULL) {
		PHY_ERROR(("%s: phy_malloc failed\n", __FUNCTION__));
		goto fail;
	}

	info->pi = pi;
	info->fns = &((phy_dsi_mem_t *)info)->fns;
	info->st = &((phy_dsi_mem_t *)info)->st;

	return info;

	/* error */
fail:
	phy_dsi_detach(info);
	return NULL;
}

void
BCMATTACHFN(phy_dsi_detach)(phy_dsi_info_t *info)
{
	phy_info_t *pi;

	PHY_TRACE(("%s\n", __FUNCTION__));

	if (info == NULL) {
		PHY_INFORM(("%s: null dsi module\n", __FUNCTION__));
		return;
	}

	pi = info->pi;

	phy_mfree(pi, info, sizeof(phy_dsi_mem_t));
}

/* register phy type specific implementations */
int
BCMATTACHFN(phy_dsi_register_impl)(phy_dsi_info_t *ri, phy_type_dsi_fns_t *fns)
{
	PHY_TRACE(("%s\n", __FUNCTION__));


	*ri->fns = *fns;
	return BCME_OK;
}

void
BCMATTACHFN(phy_dsi_unregister_impl)(phy_dsi_info_t *ri)
{
	PHY_TRACE(("%s\n", __FUNCTION__));
}

/*
 * Query the states pointer.
 */
phy_dsi_state_t *
phy_dsi_get_st(phy_dsi_info_t *di)
{
	return di->st;
}

/* Returns which path to take for phy & radio initializations
 * DeepSleepInit (true) or Normal Init (false)
 */
bool
phy_get_dsi_trigger_st(phy_info_t *pi)
{
	phy_dsi_info_t *di = pi->dsii;
	phy_dsi_state_t *st = phy_dsi_get_st(di);

	return st->trigger;
}

void
BCMINITFN(phy_dsi_restore)(phy_info_t *pi)
{
	phy_dsi_info_t *info = pi->dsii;
	phy_type_dsi_fns_t *fns = info->fns;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* redirect the request to PHY type specific implementation */
	ASSERT(fns->restore != NULL);
	(fns->restore)(fns->ctx);
}

int
phy_dsi_save(phy_info_t *pi)
{
	phy_dsi_info_t *info = pi->dsii;
	phy_type_dsi_fns_t *fns = info->fns;

	PHY_TRACE(("%s\n", __FUNCTION__));

	/* redirect the request to PHY type specific implementation */
	ASSERT(fns->save != NULL);

	return (fns->save)(fns->ctx);
}
