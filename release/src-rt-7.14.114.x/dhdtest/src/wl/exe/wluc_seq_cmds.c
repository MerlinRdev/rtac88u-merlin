/*
 * wl seq_cmds command module
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
 * $Id: wluc_seq_cmds.c 458728 2014-02-27 18:15:25Z $
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

static cmd_t wl_seq_cmds[] = {
	{ "seq_start", wl_seq_start, -1, WLC_SET_VAR,
	"Initiates command batching sequence. Subsequent IOCTLs will be queued until\n"
	"seq_stop is received."},
	{ "seq_stop", wl_seq_stop, -1, WLC_SET_VAR,
	"Defines the end of command batching sequence. Queued IOCTLs will be executed."},
	{ "seq_delay", wl_varint, -1, WLC_SET_VAR,
	"Driver should spin for the indicated amount of time.\n"
	"It is only valid within the context of batched commands."},
	{ "seq_error_index", wl_varint, WLC_GET_VAR, -1,
	"Used to retrieve the index (starting at 1) of the command that failed within a batch"},
	{ NULL, NULL, 0, 0, NULL }
};

static char *buf;

/* module initialization */
void
wluc_seq_cmds_module_init(void)
{
	(void)g_swap;

	/* get the global buf */
	buf = wl_get_buf();

	/* register seq_cmds commands */
	wl_module_cmds_register(wl_seq_cmds);
}
