/*
 * wl wds command module
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
 * $Id: wluc_wds.c 458728 2014-02-27 18:15:25Z $
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

static cmd_func_t wl_wds_wpa_role_old, wl_wds_wpa_role;
#if defined(DWDS)
static cmd_func_t wl_dwds_config;
#endif

#define WDS_TYPE_USAGE	\
"\tUsage: wl wds_type -i <ifname>\n" \
"\tifname is the name of the interface to query the type.\n" \
"\tReturn values:\n" \
"\t\t0:The interface type is neither WDS nor DWDS.\n" \
"\t\t1:The interface is WDS type.\n" \
"\t\t2:The interface is DWDS type.\n"

static cmd_t wl_wds_cmds[] = {
	{ "wds", wl_maclist, WLC_GET_WDSLIST, WLC_SET_WDSLIST,
	"Set or get the list of WDS member MAC addresses.\n"
	"\tSet using a space separated list of MAC addresses.\n"
	"\twl wds xx:xx:xx:xx:xx:xx [xx:xx:xx:xx:xx:xx ...]" },
	{ "lazywds", wl_int, WLC_GET_LAZYWDS, WLC_SET_LAZYWDS,
	"Set or get \"lazy\" WDS mode (dynamically grant WDS membership to anyone)."},
	{ "wds_remote_mac", wl_macaddr, WLC_WDS_GET_REMOTE_HWADDR, -1,
	"Get WDS link remote endpoint's MAC address"},
	{ "wds_wpa_role_old", wl_wds_wpa_role_old, WLC_WDS_GET_WPA_SUP, -1,
	"Get WDS link local endpoint's WPA role (old)"},
	{ "wds_wpa_role", wl_wds_wpa_role, WLC_GET_VAR, WLC_SET_VAR,
	"Get/Set WDS link local endpoint's WPA role"},
#if defined(DWDS)
	{ "dwds_config", wl_dwds_config, -1, WLC_SET_VAR,
	"wl dwds_config <enable/disable> <sta/ap> <xx:xx:xx:xx:xx:xx>"},
#endif
	{ "wds_type", wl_varint, WLC_GET_VAR, -1,
	"Indicate whether the interface to which this IOVAR is sent is of WDS or DWDS type.\n\n"
	WDS_TYPE_USAGE},
	{ NULL, NULL, 0, 0, NULL }
};

static char *buf;

/* module initialization */
void
wluc_wds_module_init(void)
{
	/* get the global buf */
	buf = wl_get_buf();

	/* register wds commands */
	wl_module_cmds_register(wl_wds_cmds);
}

static int
wl_wds_wpa_role_old(void *wl, cmd_t *cmd, char **argv)
{
	uint remote[2];
	uint *sup = remote;
	int ret = 0;

	UNUSED_PARAMETER(argv);

	if ((ret = wlu_get(wl, WLC_WDS_GET_REMOTE_HWADDR, remote, sizeof(remote))) < 0) {
		printf("Unable to get remote endpoint's hwaddr\n");
		return ret;
	}
	if ((ret = wlu_get(wl, cmd->get, remote, sizeof(remote))) < 0) {
		printf("Unable to get local endpoint's WPA role\n");
		return ret;
	}
	printf("Local endpoing's WPA role: %s\n", dtoh32(*sup) ? "supplicant" : "authenticator");
	return 0;
}

static int
wl_wds_wpa_role(void *wl, cmd_t *cmd, char **argv)
{
	char var[256];
	char *mac;
	char *sup;
	int len;
	int ret;
	if (strlen("wds_wpa_role") + 1 + ETHER_ADDR_LEN + 1 > sizeof(var))
		return -1;
	/* build var required by WLC_GET|SET_VAR */
	len = sprintf(var, "%s", "wds_wpa_role") + 1;
	mac = var + len;
	if ((ret = wlu_get(wl, WLC_WDS_GET_REMOTE_HWADDR, mac, ETHER_ADDR_LEN)) < 0) {
		printf("Unable to get remote endpoint's hwaddr\n");
		return ret;
	}
	len += ETHER_ADDR_LEN + 1;
	if (argv[1]) {
		sup = mac + ETHER_ADDR_LEN;
		switch ((uchar)(*sup = atoi(argv[1]))) {
		case WL_WDS_WPA_ROLE_AUTH:
		case WL_WDS_WPA_ROLE_SUP:
		case WL_WDS_WPA_ROLE_AUTO:
			if ((ret = wlu_set(wl, cmd->set, var, len)) < 0)
				printf("Unable to set local endpoint's WPA role\n");
			break;
		default:
			printf("Invalid WPA role %s. %u:authenticator, %u:supplicant, %u:auto\n",
				argv[1], WL_WDS_WPA_ROLE_AUTH,
				WL_WDS_WPA_ROLE_SUP, WL_WDS_WPA_ROLE_AUTO);
			break;
		}
	}
	else if ((ret = wlu_get(wl, cmd->get, var, len)) < 0) {
		printf("Unable to get local endpoint's WPA role\n");
		return ret;
	}
	else {
		sup = var;
		printf("Local endpoint's WPA role: %s\n", *sup ? "supplicant" : "authenticator");
	}
	return ret;
}

#if defined(DWDS)
static int
wl_dwds_config(void *wl, cmd_t *cmd, char **argv)
{
	wlc_dwds_config_t dwds;
	int err;

	memset(&dwds, 0, sizeof(wlc_dwds_config_t));

	if (!*++argv) {
		printf("error: missing arguments\n");
		return -1;
	}

	if (!stricmp(*argv, "enable"))
		dwds.enable = 1;
	else if (!stricmp(*argv, "disable"))
		dwds.enable = 0;
	else {
		printf("error: unknown mode option %s\n", *argv);
		return -1;
	}
	argv++;
	/* look for sta/dwds */
	if (!stricmp(*argv, "sta"))
		dwds.mode = 1;
	else if (!stricmp(*argv, "ap"))
		dwds.mode = 0;
	else {
		printf("error: unknown mode option %s\n", *argv);
		return -1;
	}

	argv++;
	/* convert the ea string into an ea struct */
	if (!*argv || !wl_ether_atoe(*argv, &dwds.ea)) {
		printf(" ERROR: no valid ether addr provided\n");
		return -1;
	}

	if ((err = wlu_iovar_set(wl, cmd->name, &dwds, sizeof(wlc_dwds_config_t))) < 0)
		return err;

	return (0);

}
#endif /* DWDS */
