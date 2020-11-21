/*
 * MAC debug and print functions
 * Broadcom 802.11bang Networking Device Driver
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
#ifndef WLC_MACDBG_H_
#define WLC_MACDBG_H_

#include <typedefs.h>
#include <wlc_types.h>

/* fatal reason code */
#define PSM_FATAL_ANY		0
#define PSM_FATAL_PSMWD		1
#define PSM_FATAL_SUSP		2
#define PSM_FATAL_LAST		3

#define	PRVAL(name)	bcm_bprintf(b, "%s %d ", #name, WLCNTVAL(cnt->name))
#define	PRNL()		bcm_bprintf(b, "\n")
#define PRVAL_RENAME(varname, prname)	\
	bcm_bprintf(b, "%s %d ", #prname, WLCNTVAL(cnt->varname))

#define	PRREG(name)	bcm_bprintf(b, #name " 0x%x ", R_REG(wlc->osh, &regs->name))
#define PRREG_INDEX(name, reg) bcm_bprintf(b, #name " 0x%x ", R_REG(wlc->osh, &reg))

#if defined(BCMDBG) || defined(BCMDBG_PHYDUMP) || defined(WLTEST) || \
	defined(TDLS_TESTBED) || defined(BCMDBG_AMPDU) || defined(BCMDBG_TXBF) || \
	defined(BCMDBG_DUMP_RSSI) || defined(MCHAN_MINIDUMP)
#define WL_MACDBG 1
#else
#define WL_MACDBG 0
#endif 

/* attach/detach */
extern int wlc_macdbg_attach(wlc_info_t *wlc);
extern void wlc_macdbg_detach(wlc_info_t *wlc);

#if WL_MACDBG
extern int wlc_dump_mac(wlc_info_t *wlc, struct bcmstrbuf *b);
#endif /* WL_MACDBG */


extern void wlc_dump_ucode_fatal(wlc_info_t *wlc, uint reason);
#endif /* WLC_MACDBG_H_ */
