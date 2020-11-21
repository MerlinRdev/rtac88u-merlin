/*
 * wl ltecx command module
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
 * $Id: wluc_ltecx.c 458728 2014-02-27 18:15:25Z $
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

/* LTE coex funcs */
static cmd_func_t wl_wci2_config;
static cmd_func_t wl_mws_params;
static cmd_func_t wl_mws_wci2_msg;

static cmd_t wl_ltecx_cmds[] = {
	{ "wci2_config", wl_wci2_config, WLC_GET_VAR, WLC_SET_VAR,
	"Get/Set LTE coex MWS signaling config\n"
	"\tUsage: wl wci2_config <rxassert_off> <rxassert_jit> <rxdeassert_off> <rxdeassert_jit> "
	"<txassert_off> <txassert_jit> <txdeassert_off> <txdeassert_jit> "
	"<patassert_off> <patassert_jit> <inactassert_off> <inactassert_jit> "
	"<scanfreqassert_off> <scanfreqassert_jit> <priassert_off_req>"},
	{ "mws_params", wl_mws_params, WLC_GET_VAR, WLC_SET_VAR,
	"Get/Set LTE coex MWS channel params\n"
	"\tUsage: wl mws_params <rx_center_freq> <tx_center_freq> "
	"<rx_channel_bw> <tx_channel_bw> <channel_en> <channel_type>"},
	{ "mws_debug_msg", wl_mws_wci2_msg, WLC_GET_VAR, WLC_SET_VAR,
	"Get/Set LTE coex BT-SIG message\n"
	"\tUsage: wl mws_debug_msg <Message> <Interval 20us-32000us> "
	"<Repeats>"},
	{ NULL, NULL, 0, 0, NULL }
};

static char *buf;

/* module initialization */
void
wluc_ltecx_module_init(void)
{
	/* get the global buf */
	buf = wl_get_buf();

	/* register ltecx commands */
	wl_module_cmds_register(wl_ltecx_cmds);
}

static int
wl_wci2_config(void *wl, cmd_t *cmd, char **argv)
{
	uint32 val;
	char *endptr = NULL;
	uint argc;
	wci2_config_t wci2_config;
	uint16 *configp = (uint16 *)&wci2_config;
	int ret, i;

	UNUSED_PARAMETER(cmd);

	val = 0;

	/* eat command name */
	argv++;
	/* arg count */
	for (argc = 0; argv[argc]; argc++);

	memset(&wci2_config, '\0', sizeof(wci2_config_t));

	if (argc == 0) {
		/* Get and print the values */
		ret = wlu_iovar_getbuf(wl, "wci2_config", &wci2_config, sizeof(wci2_config_t),
		buf, WLC_IOCTL_SMLEN);
		if (ret)
			return ret;

		printf("rxassert_off %d rxassert_jit %d rxdeassert_off %d rxdeassert_jit %d "
			"txassert_off %d txassert_jit %d txdeassert_off %d txdeassert_jit %d "
			"patassert_off %d patassert_jit %d inactassert_off %d inactassert_jit %d "
			"scanfreqassert_off %d scanfreqassert_jit %d priassert_off_req %d\n",
			dtoh16(((uint16 *)buf)[0]), dtoh16(((uint16 *)buf)[1]),
			dtoh16(((uint16 *)buf)[2]), dtoh16(((uint16 *)buf)[3]),
			dtoh16(((uint16 *)buf)[4]), dtoh16(((uint16 *)buf)[5]),
			dtoh16(((uint16 *)buf)[6]), dtoh16(((uint16 *)buf)[7]),
			dtoh16(((uint16 *)buf)[8]), dtoh16(((uint16 *)buf)[9]),
			dtoh16(((uint16 *)buf)[10]), dtoh16(((uint16 *)buf)[11]),
			dtoh16(((uint16 *)buf)[12]), dtoh16(((uint16 *)buf)[13]),
			dtoh16(((uint16 *)buf)[14]));
		return 0;
	}

	if (argc < 15)
		goto usage;

	for (i = 0; i < 15; ++i) {
		val = strtoul(argv[i], &endptr, 0);
		if (*endptr != '\0')
			goto usage;
		configp[i] = htod16((uint16)val);
	}
	return wlu_iovar_setbuf(wl, "wci2_config", &wci2_config, sizeof(wci2_config_t),
		buf, WLC_IOCTL_SMLEN);

usage:
	return BCME_USAGE_ERROR;
}

static int
wl_mws_params(void *wl, cmd_t *cmd, char **argv)
{
	uint32 val;
	char *endptr = NULL;
	uint argc;
	mws_params_t mws_params;
	uint16 *paramsp = (uint16 *)&mws_params;
	int ret, i;

	UNUSED_PARAMETER(cmd);

	val = 0;

	/* eat command name */
	argv++;

	/* arg count */
	for (argc = 0; argv[argc]; argc++);

	memset(&mws_params, '\0', sizeof(mws_params_t));

	if (argc == 0) {
		/* Get and print the values */
		ret = wlu_iovar_getbuf(wl, "mws_params", &mws_params, sizeof(mws_params_t),
		buf, WLC_IOCTL_SMLEN);
		if (ret)
			return ret;

		printf("rx_center_freq %d tx_center_freq %d  rx_channel_bw %d tx_channel_bw %d "
			"channel_en %d channel_type %d\n",
			dtoh16(((uint16 *)buf)[0]), dtoh16(((uint16 *)buf)[1]),
			dtoh16(((uint16 *)buf)[2]), dtoh16(((uint16 *)buf)[3]), buf[8], buf[9]);
		return 0;
	}

	if (argc < 6)
		goto usage;
	for (i = 0; i < 4; ++i) {
		val = strtoul(argv[i], &endptr, 0);
		if (*endptr != '\0')
			goto usage;
		paramsp[i] = htod16((uint16)val);
	}
	val = strtoul(argv[i], &endptr, 0);
	if (*endptr != '\0')
		goto usage;
	mws_params.mws_channel_en = val;
	++i;
	val = strtoul(argv[i], &endptr, 0);
	if (*endptr != '\0')
		goto usage;
	mws_params.mws_channel_type = val;

	return wlu_iovar_setbuf(wl, "mws_params", &mws_params, sizeof(mws_params_t),
		buf, WLC_IOCTL_SMLEN);

usage:
	return BCME_USAGE_ERROR;
}

static int
wl_mws_wci2_msg(void *wl, cmd_t *cmd, char **argv)
{
	uint32 val;
	char *endptr = NULL;
	uint argc;
	mws_wci2_msg_t mws_wci2_msg;
	uint16 *paramsp = (uint16 *)&mws_wci2_msg;
	int ret, i = 0;

	UNUSED_PARAMETER(cmd);

	val = 0;

	/* eat command name */
	argv++;

	/* arg count */
	for (argc = 0; argv[argc]; argc++);

	memset(&mws_wci2_msg, '\0', sizeof(mws_wci2_msg_t));

	if (argc == 0) {
		/* Get and print the values */
		ret = wlu_iovar_getbuf(wl, "mws_debug_msg", &mws_wci2_msg, sizeof(mws_wci2_msg_t),
		buf, WLC_IOCTL_SMLEN);
		if (ret)
			return ret;

		printf("Message %d Interval %d  Repeats %d \n",
			dtoh16(((uint16 *)buf)[0]), dtoh16(((uint16 *)buf)[1]),
			dtoh16(((uint16 *)buf)[2]));
		return 0;
	}

	if (argc < 3)
		goto usage;

	for (i = 0; i < 3; ++i) {
		val = strtoul(argv[i], &endptr, 0);
		if (*endptr != '\0')
			goto usage;
		paramsp[i] = htod16((uint16)val);
	}
	if ((paramsp[1] < 20) || (paramsp[1] > 32000))
		goto usage;
	return wlu_iovar_setbuf(wl, "mws_debug_msg", &mws_wci2_msg, sizeof(mws_wci2_msg_t),
		buf, WLC_IOCTL_SMLEN);

usage:
	return BCME_USAGE_ERROR;
}
