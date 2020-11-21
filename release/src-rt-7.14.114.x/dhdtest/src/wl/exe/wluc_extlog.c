/*
 * wl extlog command module
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
 * $Id: wluc_extlog.c 458728 2014-02-27 18:15:25Z $
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

#include <wlc_extlog_idstr.h>

static int wl_extlog(void *wl, cmd_t *cmd, char **argv);
static int wl_extlog_cfg(void *wl, cmd_t *cmd, char **argv);

static cmd_t wl_extlog_cmds[] = {
	{ "extlog", wl_extlog, WLC_GET_VAR, -1,
	"get external logs\n"
	"\tUsage: wl extlog <from_last> <number>\n"
	"\tfrom_last: 1 - from the last log record; 0 - whole log recrods\n"
	"\tnumber: number of log records to get, MAX is 32."},
	{ "extlog_clr", wl_var_void, -1, WLC_SET_VAR, "clear external log records"},
	{ "extlog_cfg", wl_extlog_cfg, WLC_GET_VAR, WLC_SET_VAR,
	"get/set external log configuration"},
	{ NULL, NULL, 0, 0, NULL }
};

static char *buf;

/* module initialization */
void
wluc_extlog_module_init(void)
{
	/* get the global buf */
	buf = wl_get_buf();

	/* register extlog commands */
	wl_module_cmds_register(wl_extlog_cmds);
}

static int
wl_extlog(void *wl, cmd_t *cmd, char **argv)
{
	int argc;
	char *endptr;
	int err;
	int from_last;
	int i, j;
	char *log_p = NULL;
	wlc_extlog_req_t r_args;
	wlc_extlog_results_t *results;
	void *ptr = NULL;

	/* arg count */
	for (argc = 0; argv[argc]; argc++)
		;

	if (argc > 3)
		return BCME_USAGE_ERROR;

	if (argc == 1)
		from_last = 0;
	else {
		from_last = htod32(strtoul(argv[1], &endptr, 0));
		if ((from_last != 0) && (from_last != 1))
			return BCME_BADARG;
	}

	r_args.from_last = from_last;
	if (argc == 3)
		r_args.num = htod32(strtoul(argv[2], &endptr, 0));
	else
		r_args.num = 32;

	if ((err = wlu_var_getbuf(wl, cmd->name, &r_args, sizeof(wlc_extlog_req_t), &ptr)) < 0)
			return err;

	results = (wlc_extlog_results_t *)buf;

	printf("get external log records: %d\n", results->num);
	if (!results->num)
		return 0;

	if (results->version != EXTLOG_CUR_VER) {
		printf("version mismatch: version = 0x%x, expected 0x%0x\n",
			results->version, EXTLOG_CUR_VER);
		return 0;
	}

	log_p = (char *)&results->logs[0];

	printf("Seq:\tTime(ms) Log\n");
	for (i = 0; i < (int)results->num; i++) {
		printf("%d:\t%d\t ", ((log_record_t*)log_p)->seq_num,
			((log_record_t*)log_p)->time);
		for (j = 0; j < FMTSTR_MAX_ID; j++)
			if (((log_record_t*)log_p)->id == extlog_fmt_str[j].id)
				break;
		if (j == FMTSTR_MAX_ID) {
			printf("fmt string not found for id %d\n", ((log_record_t*)log_p)->id);
			log_p += results->record_len;
			continue;
		}

		switch (extlog_fmt_str[j].arg_type) {
			case LOG_ARGTYPE_NULL:
				printf("%s", extlog_fmt_str[j].fmt_str);
				break;

			case LOG_ARGTYPE_STR:
				printf(extlog_fmt_str[j].fmt_str, ((log_record_t*)log_p)->str);
				break;

			case LOG_ARGTYPE_INT:
				printf(extlog_fmt_str[j].fmt_str, ((log_record_t*)log_p)->arg);
				break;

			case LOG_ARGTYPE_INT_STR:
				printf(extlog_fmt_str[j].fmt_str, ((log_record_t*)log_p)->arg,
					((log_record_t*)log_p)->str);
				break;

			case LOG_ARGTYPE_STR_INT:
				printf(extlog_fmt_str[j].fmt_str, ((log_record_t*)log_p)->str,
					((log_record_t*)log_p)->arg);
				break;
			}
		log_p += results->record_len;
	}

	return 0;

}

static int
wl_extlog_cfg(void *wl, cmd_t *cmd, char **argv)
{
	int argc;
	int err = 0;
	char *endptr;
	wlc_extlog_cfg_t *r_cfg;
	wlc_extlog_cfg_t w_cfg;

	/* arg count */
	for (argc = 0; argv[argc]; argc++)
		;

	if (argc == 1) {
		err = wl_var_get(wl, cmd, argv);
		if (err < 0)
			return err;
		r_cfg = (wlc_extlog_cfg_t *)buf;
		printf("max_number=%d, module=%x, level=%d, flag=%d, version=0x%04x\n",
			r_cfg->max_number, r_cfg->module, r_cfg->level,
			r_cfg->flag, r_cfg->version);
	}
	else if (argc == 4) {
		w_cfg.module = htod16((uint16)(strtoul(argv[1], &endptr, 0)));
		w_cfg.level = (uint8)strtoul(argv[2], &endptr, 0);
		w_cfg.flag = (uint8)strtoul(argv[3], &endptr, 0);
		err = wlu_var_setbuf(wl, cmd->name, &w_cfg, sizeof(wlc_extlog_cfg_t));
	}
	else {
		fprintf(stderr, "illegal command!\n");
		return BCME_USAGE_ERROR;
	}

	return err;
}
