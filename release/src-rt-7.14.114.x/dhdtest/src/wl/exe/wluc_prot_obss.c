/*
 * wl prot_obss command module
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
 * $Id: wluc_prot_obss.c 458728 2014-02-27 18:15:25Z $
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

#if defined(BCMDBG)
static cmd_func_t wl_ccastats;
#endif
static cmd_func_t wl_dyn_bwsw_params;

static cmd_t wl_prot_obss_cmds[] = {
	{ "obss_prot", wl_bcm_config, WLC_GET_VAR, WLC_SET_VAR,
	"Get/set OBSS protection (-1=auto, 0=disable, 1=enable)\n" },
#if defined(BCMDBG)
	{ "ccastats", wl_ccastats, WLC_GET_VAR, -1,
	"Usage: \n\t wl ccastats [-d num msecs] to begin measurement\n"
	"\t wl ccastats [-o option: 0=default, 1=detail, 2=1+CRS] to query for measurements" },
	{ "dump_obss_dyn_bwsw", wl_ccastats, WLC_GET_VAR, -1,
	"Usage: \n\t wl dump_obss_dyn_bwsw [-d num msecs] to begin measurement\n"
	"\t wl dump_obss_dyn_bwsw to query for the results" },
#endif
	{ "dyn_bwsw_params", wl_dyn_bwsw_params, WLC_GET_VAR, WLC_SET_VAR,
	"Configure the params for dynamic bandswitch\n"
	"\tUsage (Get): wl dyn_bwsw_params \n"
	"\tUsage (Set): wl dyn_bwsw_params actvcfm=0x03 noactcfm=0x06\n"
	"			noactincr=0x05 psense=2000\n"
	"			rxcrsthresh=0x20 secdurlim=30 \n"
	"\t To reset to default value give val 0\n"
	"\t Example : wl dyn_bwsw_params rxcrsthresh=0\n" },
	{ NULL, NULL, 0, 0, NULL }
};

static char *buf;

/* module initialization */
void
wluc_prot_obss_module_init(void)
{
	/* get the global buf */
	buf = wl_get_buf();

	/* register prot_obss commands */
	wl_module_cmds_register(wl_prot_obss_cmds);
}

#if defined(BCMDBG)
static int
wl_ccastats(void *wl, cmd_t *cmd, char **argv)
{
	cca_stats_n_flags *results;
	int err = 0;
	int msr_time;
	uint8 option = 0;
	cca_msrmnt_query req;
	char *ptr;

	/* skip the command name */
	argv++;

	/* only switch -d for now */
	if (*argv != NULL && !strcmp(*argv, "-d")) {
		argv++;
		if (*argv == NULL || (msr_time = htod32(atoi(*argv))) <= 0) {
			printf("enter correct duration\n");
			return 0;
		}
		req.msrmnt_query = 0;
		req.time_req = msr_time;

		if ((err = wlu_iovar_getbuf(wl, cmd->name, &req, sizeof(req),
		                            buf, WLC_IOCTL_MAXLEN)) < 0) {
			return err;
		}

		printf("Measuring %d ms ...\n", msr_time);
	} else {
		if (*argv != NULL && !strcmp(*argv, "-o")) {
			argv++;
			if (*argv != NULL) {
				option = (uint8)htod32(atoi(*argv));
			}
		}

		req.msrmnt_query = 1;
		req.time_req = 0;
		req.report_opt = option;

		// retrieving the results
		if ((err = wlu_iovar_getbuf(wl, cmd->name, &req, sizeof(req),
			buf, WLC_IOCTL_MAXLEN)) < 0) {
			return err;
		}

		results = (cca_stats_n_flags *)buf;
		ptr = results->buf;

		fputs(ptr, stdout);
	}

	return err;
}
#endif 

/* static function for iovar dyn_bwsw_params */
static INLINE void
dynbwsw_config_params(uint32 mask, uint32 *cfg_flag,
	uint32 *rst_flag, uint val)
{
	(!val) ? (*rst_flag = (*rst_flag | mask)) : (*cfg_flag = (*cfg_flag | mask));
}

/* Given an argument String, Set/Get the dynamic bandswitch parameters */
static int
wl_dyn_bwsw_params(void *wl, cmd_t *cmd, char **argv)
{
	int err = 0;
	uint val;
	void *ptr;
	obss_config_params_t *params;
	wlc_prot_dynbwsw_config_t *config_params;

	UNUSED_PARAMETER(cmd);

	if (*++argv) { /* write dyn_bwsw parameters */
		params = (obss_config_params_t *)malloc(sizeof(obss_config_params_t));

		if (params == NULL) {
			printf("memory alloc failure\n");
			return BCME_NOMEM;
		}

		config_params = &params->config_params;
		params->reset_mask = 0x0000;
		params->config_mask = 0x0000;
		params->version = WL_PROT_OBSS_CONFIG_PARAMS_VERSION;

		if (find_pattern(argv, "actvcfm", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_ACTIVITY_PERIOD,
				&params->config_mask, &params->reset_mask, val);
			config_params->obss_bwsw_activity_cfm_count_cfg =
				val;
		}

		if (find_pattern(argv, "noactcfm", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_NOACTIVITY_PERIOD,
				&params->config_mask, &params->reset_mask, val);

			config_params->obss_bwsw_no_activity_cfm_count_cfg =
				val;
		}
		if (find_pattern(argv, "noactincr", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_NOACTIVITY_INCR_PERIOD,
				&params->config_mask, &params->reset_mask, val);
			config_params->obss_bwsw_no_activity_cfm_count_incr_cfg =
				val;
		}

		if (find_pattern(argv, "psense", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_PSEUDO_SENSE_PERIOD,
				&params->config_mask, &params->reset_mask, val);
			config_params->obss_bwsw_pseudo_sense_count_cfg = val;
		}

		if (find_pattern(argv, "rxcrsthresh", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_RX_CRS_PERIOD,
				&params->config_mask, &params->reset_mask, val);
			config_params->obss_bwsw_rx_crs_threshold_cfg = val;
		}
		if (find_pattern(argv, "secdurlim", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_DUR_THRESHOLD,
				&params->config_mask, &params->reset_mask, val);
			config_params->obss_bwsw_dur_thres = val;
		}
		if (find_pattern(argv, "txopthresh", &val)) {
			dynbwsw_config_params(WL_OBSS_DYN_BWSW_FLAG_TXOP_PERIOD,
			&params->config_mask, &params->reset_mask, val);
			config_params->obss_bwsw_txop_threshold_cfg = val;
		}
		if ((err = wlu_var_setbuf(wl, "dyn_bwsw_params", params,
			sizeof(obss_config_params_t)) < 0)) {
			printf("wl_dyn_bwsw_params: fail to set %d\n", err);
		}
		config_params = NULL;
		free(params);
	} else {
		if ((err = wlu_var_getbuf(wl, "dyn_bwsw_params", NULL, 0, &ptr) < 0)) {
			printf("wl_dyn_bwsw_params: fail to get params\n");
			return err;
		}
		params = (obss_config_params_t *)ptr;
		config_params = &params->config_params;

		printf("Version=%d\n", params->version);
		printf("actvcfm=%d\n", config_params->obss_bwsw_activity_cfm_count_cfg);
		printf("noactcfm=%d\n",
			config_params->obss_bwsw_no_activity_cfm_count_cfg);
		printf("noactincr=%d\n",
			config_params->obss_bwsw_no_activity_cfm_count_incr_cfg);
		printf("psense=%d\n", config_params->obss_bwsw_pseudo_sense_count_cfg);
		printf("rxcrsthresh=%d\n", config_params->obss_bwsw_rx_crs_threshold_cfg);
		printf("secdurlim=%d\n", config_params->obss_bwsw_dur_thres);
		printf("txopthresh=%d\n", config_params->obss_bwsw_txop_threshold_cfg);
	}

	return err;
}
