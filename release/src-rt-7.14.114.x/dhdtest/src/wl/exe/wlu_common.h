/*
 * Common code for wl routines
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
 * $Id: wlu_common.h 616248 2016-01-31 07:48:13Z $
 */
#include <wlioctl.h>
#include <bcmutils.h>
#if	defined(_CFE_)
#include <lib_types.h>
#include <lib_string.h>
#include <lib_printf.h>
#include <lib_malloc.h>
#include <cfe_error.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#endif /* defined(_CFE_) */
#if defined(__NetBSD__) || defined(linux) || defined(MACOSX) || defined(EFI)
#define stricmp strcasecmp
#define strnicmp strncasecmp
#elif	defined(_CRT_SECURE_NO_DEPRECATE)
#define stricmp _stricmp
#define strnicmp _strnicmp
#elif	defined(DONGLEBUILD)
#define stricmp strcmp
#define strnicmp strncmp
#elif	defined(_CFE_)
#include <bcmutils.h>
#include <osl.h>
#define isalnum(c) bcm_isalnum(c)
#define isalpha(c) bcm_isalpha(c)
#define iscntrl(c) bcm_iscntrl(c)
#define isdigit(c) bcm_isdigit(c)
#define isgraph(c) bcm_isgraph(c)
#define islower(c) bcm_islower(c)
#define isprint(c) bcm_isprint(c)
#define ispunct(c) bcm_ispunct(c)
#define isspace(c) bcm_isspace(c)
#define isupper(c) bcm_isupper(c)
#define isxdigit(c) bcm_isxdigit(c)
#define stricmp(s1, s2) lib_strcmpi((s1), (s2))
#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))
#define tolower(c) (bcm_isupper((c)) ? ((c) + 'a' - 'A') : (c))
#define fprintf(stream, fmt, args...) xprintf(fmt, ##args)
#define fputs(s, stream) puts(s)
#define malloc(size) KMALLOC((size), 0)
#define free(ptr) KFREE(ptr)
#define strnicmp(s1, s2, len) strncmp((s1), (s2), (len))
#define strspn(s, accept) (0)
#define strtol(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))
#elif	defined(BWL_STRICMP)
#define stricmp bcmstricmp
#define strnicmp bcmstrnicmp
#endif /* __NetBSD__ */


/* IOCTL swapping mode for Big Endian host with Little Endian dongle.  Default to off */
/* The below macros handle endian mis-matches between wl utility and wl driver. */
extern bool g_swap;

/* command batching data structure */
typedef struct wl_seq_cmd_pkt {
	struct wl_seq_cmd_pkt	*next;
	wl_seq_cmd_ioctl_t cmd_header;
	char * data;			/* user buffer */
} wl_seq_cmd_pkt_t;

typedef struct wl_cmd_list {
	wl_seq_cmd_pkt_t	*head;
	wl_seq_cmd_pkt_t	*tail;
} wl_cmd_list_t;

extern wl_cmd_list_t cmd_list;
extern int cmd_pkt_list_num;
extern bool cmd_batching_mode;

extern int wlu_iovar_getbuf(void* wl, const char *iovar,
	void *param, int paramlen, void *bufptr, int buflen);
extern int wlu_iovar_setbuf(void* wl, const char *iovar,
	void *param, int paramlen, void *bufptr, int buflen);
extern int wlu_var_setbuf(void *wl, const char *iovar, void *param, int param_len);
extern int wlu_iovar_getint(void *wl, const char *iovar, int *pval);
extern void init_cmd_batchingmode(void);
extern void clean_up_cmd_list(void);
extern int wl_check(void *wl);

extern int add_one_batched_cmd(int cmd, void *cmdbuf, int len);
extern int wlu_get_req_buflen(int cmd, void *cmdbuf, int len);
extern int wlu_get(void *wl, int cmd, void *cmdbuf, int len);
extern int wlu_set(void *wl, int cmd, void *cmdbuf, int len);
extern int wlu_iovar_get(void *wl, const char *iovar, void *outbuf, int len);
extern int wlu_iovar_set(void *wl, const char *iovar, void *param, int paramlen);
extern int wlu_iovar_getint(void *wl, const char *iovar, int *pval);
extern int wlu_iovar_setint(void *wl, const char *iovar, int val);
