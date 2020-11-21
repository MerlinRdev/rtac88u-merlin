/*
 * Event mechanism
 *
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
 * $Id: wlc_event.h 525520 2015-01-09 23:21:47Z $
 */


#ifndef _WLC_EVENT_H_
#define _WLC_EVENT_H_

#include <typedefs.h>
#include <proto/ethernet.h>
#include <proto/bcmevent.h>
#include <wlc_types.h>

wlc_eventq_t *wlc_eventq_attach(wlc_info_t *wlc);
void wlc_eventq_detach(wlc_eventq_t *eq);

wlc_event_t *wlc_event_alloc(wlc_eventq_t *eq);
void wlc_event_free(wlc_eventq_t *eq, wlc_event_t *e);

#ifdef WLNOEIND
#define wlc_eventq_test_ind(a, b) FALSE
#define wlc_eventq_set_ind(a, b, c) do {} while (0)
#define wlc_eventq_flush(eq) do {} while (0)
#else /* WLNOEIND */
int wlc_eventq_test_ind(wlc_eventq_t *eq, int et);
int wlc_eventq_set_ind(wlc_eventq_t* eq, uint et, bool on);
void wlc_eventq_flush(wlc_eventq_t *eq);
#endif /* WLNOEIND */

void wlc_event_process(wlc_eventq_t *eq, wlc_event_t *e);

#if defined(MSGTRACE) || defined(LOGTRACE)
#include <rte_dev.h>
extern void wlc_event_sendup_trace(wlc_info_t * wlc, hnd_dev_t * bus, uint8* hdr,
	uint16 hdrlen, uint8 *buf, uint16 buflen);
#endif

#endif  /* _WLC_EVENT_H_ */
