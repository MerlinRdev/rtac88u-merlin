/*
 * wl interfere command module
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
 * $Id: wluc_interfere.c 458728 2014-02-27 18:15:25Z $
 */

#ifdef WIN32
#include <windows.h>
#endif

#include <wlioctl.h>

#if	defined(DONGLEBUILD)
#include <typedefs.h>
#include <osl.h>
#endif

/* Because IL_BIGENDIAN was removed there are few warnings that need
 * to be fixed. Windows was not compiled earlier with IL_BIGENDIAN.
 * Hence these warnings were not seen earlier.
 * For now ignore the following warnings
 */
#ifdef WIN32
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4761)
#endif

#include <bcmutils.h>
#include <bcmendian.h>
#include "wlu_common.h"
#include "wlu.h"

static cmd_func_t wl_itfr_get_stats;

static cmd_t wl_interfere_cmds[] = {
	{ "itfr_get_stats", wl_itfr_get_stats, WLC_GET_VAR, -1,
	"get interference source information"
	},
	{ "itfr_enab", wl_varint, WLC_GET_VAR, WLC_SET_VAR,
	"get/set STA interference detection mode(STA only)\n"
	"\t 0  - disable\n"
	"\t 1  - enable maual detection\n"
	"\t 2  - enable auto detection"
	},
	{ "itfr_detect", wl_var_void, -1, WLC_SET_VAR,
	"issue an interference detection request"
	},
	{ NULL, NULL, 0, 0, NULL }
};

static char *buf;

/* module initialization */
void
wluc_interfere_module_init(void)
{
	(void)g_swap;

	/* get the global buf */
	buf = wl_get_buf();

	/* register interfere commands */
	wl_module_cmds_register(wl_interfere_cmds);
}

static int
wl_itfr_get_stats(void *wl, cmd_t *cmd, char **argv)
{
	int err;
	interference_source_rep_t *iftr_stats = NULL;
	const char *iftr_source[] = {"none", "wireless phone", "wireless video camera",
		"microwave oven", "wireless baby monitor", "bluetooth device",
		"wireless video camera or baby monitor", "bluetooth or baby monitor",
		"video camera or phone", "unidentified"}; /* sync with interference_source_t */

	UNUSED_PARAMETER(argv);

	if ((err = wlu_var_getbuf(wl, cmd->name, NULL, 0, (void*)&iftr_stats)) < 0)
		return err;

	if (iftr_stats->flags & ITFR_NOISY_ENVIRONMENT)
		printf("Feature is stopped due to noisy environment\n");
	else
		printf("Interference %s detected. last interference at timestamp %d: "
			"source is %s on %s channel\n",
			(iftr_stats->flags & ITFR_INTERFERENCED) ? "is" : "is not",
			iftr_stats->timestamp, iftr_source[iftr_stats->source],
			(iftr_stats->flags & ITFR_HOME_CHANNEL) ? "home" : "non-home");

	return err;
}
