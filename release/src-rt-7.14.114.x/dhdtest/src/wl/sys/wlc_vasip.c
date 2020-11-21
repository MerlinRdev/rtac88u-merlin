/*
 * VASIP related functions
 * Broadcom 802.11abg Networking Device Driver
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
 * $Id: wlc_vasip.c 525069 2015-01-08 21:39:50Z $
 */

#include <wlc_cfg.h>

#ifdef VASIP_HW_SUPPORT
#include <typedefs.h>
#include <wlc_types.h>
#include <siutils.h>
#include <wlioctl.h>
#include <wlc_pub.h>
#include <wlioctl.h>
#include <wlc.h>
#include <wlc_dbg.h>
#include <phy_misc_api.h>
#include <wlc_hw_priv.h>
#include <d11vasip_code.h>
#include <pcicfg.h>
#include <wl_export.h>
#include <wlc_vasip.h>

/* defines */
#define VASIP_COUNTERS_ADDR_OFFSET	0x20000
#define VASIP_STATUS_ADDR_OFFSET	0x20100

#define VASIP_COUNTERS_LMT		256
#define VASIP_DEFINED_COUNTER_NUM	26
#define SVMP_MEM_OFFSET_MAX		0xfffff
#define SVMP_MEM_DUMP_LEN_MAX		4096

/* local prototypes */
#if defined(BCMDBG) && defined(WLC_LOW)
/* dump vasip counters from vasip program memory */
static int wlc_dump_vasip_counters(wlc_info_t *wlc, struct bcmstrbuf *b);

/* dump vasip status data from vasip program memory */
static int wlc_dump_vasip_status(wlc_info_t *wlc, struct bcmstrbuf *b);

/* clear vasip counters */
static int wlc_vasip_counters_clear(wlc_hw_info_t *wlc);

/* copy svmp memory to a buffer starting from offset of length 'len', len is count of uint16's */
static int
wlc_svmp_mem_read(wlc_hw_info_t *wlc_hw, uint16 *ret_svmp_addr, uint32 offset, uint16 len);

/* set svmp memory with a value from offset of length 'len', len is count of uint16's */
static int wlc_svmp_mem_set(wlc_hw_info_t *wlc_hw, uint32 offset, uint16 len, uint16 val);
#endif /* BCMDBG && WLC_LOW */

/* iovar table */
enum {
	IOV_VASIP_COUNTERS_CLEAR,
	IOV_SVMP_MEM
};

static int
wlc_vasip_doiovar(void *context, const bcm_iovar_t *vi, uint32 actionid, const char *name,
	void *params, uint p_len, void *arg, int len, int vsize, struct wlc_if *wlcif);

static const bcm_iovar_t vasip_iovars[] = {
#if defined(BCMDBG)
	{"vasip_counters_clear", IOV_VASIP_COUNTERS_CLEAR,
	(IOVF_SET_UP), IOVT_VOID, 0
	},
	{"svmp_mem", IOV_SVMP_MEM,
	(IOVF_SET_UP | IOVF_GET_UP), IOVT_BUFFER, sizeof(svmp_mem_t),
	},
#endif /* BCMDBG */
	{NULL, 0, 0, 0, 0}
};

void
BCMATTACHFN(wlc_vasip_detach)(wlc_info_t *wlc)
{
	wlc_module_unregister(wlc->pub, "vasip", wlc);
}

int
BCMATTACHFN(wlc_vasip_attach)(wlc_info_t *wlc)
{
	wlc_pub_t *pub = wlc->pub;
	int err = BCME_OK;

	if ((err = wlc_module_register(pub, vasip_iovars, "vasip",
		wlc, wlc_vasip_doiovar, NULL, NULL, NULL))) {
		WL_ERROR(("wl%d: %s: wlc_module_register() failed\n",
			wlc->pub->unit, __FUNCTION__));
		return err;
	}

#if defined(BCMDBG) && defined(WLC_LOW)
	wlc_dump_register(pub, "vasip_counters", (dump_fn_t)wlc_dump_vasip_counters, (void *)wlc);
	wlc_dump_register(pub, "vasip_status", (dump_fn_t)wlc_dump_vasip_status, (void *)wlc);
#endif /* VASIP_HW_SUPPORT && WLC_LOW */

	return err;
}

static int
wlc_vasip_doiovar(void *context, const bcm_iovar_t *vi, uint32 actionid, const char *name,
	void *params, uint p_len, void *arg, int len, int vsize, struct wlc_if *wlcif)
{
#if defined(BCMDBG) && defined(WLC_LOW)
	wlc_info_t *wlc = (wlc_info_t *)context;
#endif /* BCMDBG && WLC_LOW */

	int err = BCME_OK;
	switch (actionid) {

#if defined(BCMDBG) && defined(WLC_LOW)
	case IOV_SVAL(IOV_VASIP_COUNTERS_CLEAR):
		err = wlc_vasip_counters_clear(wlc->hw);
		break;

	case IOV_GVAL(IOV_SVMP_MEM): {
		svmp_mem_t *mem = (svmp_mem_t *)params;
		uint32 mem_addr;
		uint16 mem_len;

		mem_addr = mem->addr;
		mem_len = mem->len;

		if (len < mem_len) {
			err = BCME_BUFTOOSHORT;
			break;
		}

		if (mem_addr & 1) {
			err = BCME_BADADDR;
			break;
		}

		err = wlc_svmp_mem_read(wlc->hw, (uint16 *)arg, mem_addr, mem_len);
		break;
	}

	case IOV_SVAL(IOV_SVMP_MEM): {
		svmp_mem_t *mem = (svmp_mem_t *)params;
		uint32 mem_addr;
		uint16 mem_len;

		mem_addr = mem->addr;
		mem_len = mem->len;

		if (mem_addr & 1) {
			err = BCME_BADADDR;
			break;
		}

		err = wlc_svmp_mem_set(wlc->hw, mem_addr, mem_len, mem->val);
		break;
	}
#endif /* BCMDBG && WLC_LOW */
	}
	return err;
}

/* write vasip code to vasip program memory */
static void
wlc_vasip_write(wlc_hw_info_t *wlc_hw, const uint32 vasip_code[], const uint nbytes)
{
#ifndef DONGLEBUILD
	osl_t *osh = wlc_hw->osh;
	uchar * bar1va;
	uint32 bar1_size;
#endif /* DONGLEBUILD */
	uint32 *vasip_mem;
	int i, count, idx;
	uint32 vasipaddr;

	WL_TRACE(("wl%d: %s\n", wlc_hw->unit, __FUNCTION__));

	ASSERT(ISALIGNED(nbytes, sizeof(uint32)));

	count = (nbytes/sizeof(uint32));

	/* save current core */
	idx = si_coreidx(wlc_hw->sih);
	if (si_setcore(wlc_hw->sih, ACPHY_CORE_ID, 0) != NULL) {
		/* get the VASIP memory base */
		vasipaddr = si_addrspace(wlc_hw->sih, 0);
		/* restore core */
		(void)si_setcoreidx(wlc_hw->sih, idx);
	} else {
		WL_ERROR(("%s: wl%d: Failed to find ACPHY core \n",
			__FUNCTION__, wlc_hw->unit));
		ASSERT(0);
		return;
	}
#ifndef DONGLEBUILD
	OSL_PCI_WRITE_CONFIG(osh, PCI_BAR1_WIN, sizeof(uint32), vasipaddr);
	bar1_size = wl_pcie_bar1(wlc_hw->wlc->wl, &bar1va);

	BCM_REFERENCE(bar1_size);
	ASSERT(bar1va != NULL && bar1_size != 0);

	vasip_mem = (uint32 *)bar1va;
#else
	vasip_mem = (uint32 *)vasipaddr;
#endif /* DONGLEBUILD */

	/* write vasip code to program memory */
	for (i = 0; i < count; i++) {
		vasip_mem[i] = vasip_code[i];
	}

	/* save base address in global */
	wlc_hw->vasip_addr = vasip_mem;
}

/* initialize vasip */
void
wlc_vasip_init(wlc_hw_info_t *wlc_hw)
{
	const uint32 *vasip_code = NULL;
	uint nbytes = 0;
	uint8 vasipver;

	WL_TRACE(("wl%d: %s\n", wlc_hw->unit, __FUNCTION__));

	vasipver = phy_misc_get_vasip_ver((phy_info_t *)wlc_hw->band->pi);
	if (vasipver == VASIP_NOVERSION) {
		return;
	}

	if (vasipver == 0) {
		vasip_code = d11vasip0;
		nbytes = d11vasip0sz;
	} else {
		WL_ERROR(("%s: wl%d: unsupported vasipver %d \n",
			__FUNCTION__, wlc_hw->unit, vasipver));
		ASSERT(0);
		return;
	}

	if (vasip_code != NULL) {
		/* stop the vasip processor */
		phy_misc_vasip_proc_reset((phy_info_t *)wlc_hw->band->pi, 1);

		/* write binary to the vasip program memory */
		wlc_vasip_write(wlc_hw, vasip_code, nbytes);

		/* reset and start the vasip processor */
		phy_misc_vasip_proc_reset((phy_info_t *)wlc_hw->band->pi, 0);
	}
}

#if defined(BCMDBG)

/* dump vasip status data from vasip program memory */
int
wlc_dump_vasip_status(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	uint16 * status;
	wlc_hw_info_t *wlc_hw = wlc->hw;
	int ret = BCME_OK, i;

	if (!VASIP_PRESENT(wlc_hw->corerev)) {
		bcm_bprintf(b, "VASIP is not present!\n");
		return ret;
	}

	status = (uint16 *)(wlc_hw->vasip_addr + VASIP_STATUS_ADDR_OFFSET);

	for (i = 0; i < VASIP_COUNTERS_LMT; i++) {
		bcm_bprintf(b, "status[%d] %u\n", i, status[i]);
	}

	return ret;
}

/* dump vasip counters from vasip program memory */
int
wlc_dump_vasip_counters(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	uint16 * counter;
	wlc_hw_info_t *wlc_hw = wlc->hw;
	int ret = BCME_OK, i;

	if (!VASIP_PRESENT(wlc_hw->corerev)) {
		bcm_bprintf(b, "VASIP is not present!\n");
		return ret;
	}

	if (!wlc_hw->up) {
		return BCME_NOTUP;
	}

	counter = (uint16 *)(wlc_hw->vasip_addr + VASIP_COUNTERS_ADDR_OFFSET);

	bcm_bprintf(b, "samp_col_0 0x%x bfr_intrpt 0x%x bfe_mod_done 0x%x bfe_imp_done 0x%x\n"
	            "m2v_msg_0 0x%x m2v_msg_1 0x%x v2m_msg_ack 0x%x m2v_done 0x%x v2m_done 0x%x\n"
	            "m2v_buf0_ovfl 0x%x m2v_buf1_ovfl 0x%x rtc_intrpt 0x%x ihrp_intrpt 0x%x\n"
	            "grp_sel_req 0x%x grp_sel_done 0x%x precoding 0x%x matrix_pmt 0x%x\n"
	            "mem_mngmt 0x%x spctrm_analysis 0x%x v2m_msg 0x%x v2m_mailbox 0x%x\n"
	            "m2v_msg_ack0 0x%x m2v_msg_ack1 0x%x v2m_no_ack 0x%x m2v_msg_no_ack 0x%x\n",
	            counter[0], counter[1], counter[2], counter[3], counter[4], counter[5],
	            counter[6], counter[7], counter[8], counter[9], counter[10], counter[11],
	            counter[12], counter[13], counter[14], counter[15], counter[16], counter[17],
	            counter[18], counter[19], counter[20], counter[21], counter[22], counter[23],
	            counter[24], counter[25]);

	/* print for any non-zero values */
	for (i = VASIP_DEFINED_COUNTER_NUM; i < VASIP_COUNTERS_LMT; i++) {
		if (counter[i] != 0) {
			bcm_bprintf(b, "counter[%d] 0x%x ", i, counter[i]);
		}
	}

	bcm_bprintf(b, "\n");
	return ret;
}

/* clear vasip counters */
int
wlc_vasip_counters_clear(wlc_hw_info_t *wlc_hw)
{
	uint16 * counter;
	int i;

	if (!VASIP_PRESENT(wlc_hw->corerev)) {
		return BCME_UNSUPPORTED;
	}

	counter = (uint16 *)(wlc_hw->vasip_addr + VASIP_COUNTERS_ADDR_OFFSET);
	for (i = 0; i < VASIP_COUNTERS_LMT; i++) {
		counter[i] = 0;
	}
	return BCME_OK;
}

/* copy svmp memory to a buffer starting from offset of length 'len', len is count of uint16's */
int
wlc_svmp_mem_read(wlc_hw_info_t *wlc_hw, uint16 *ret_svmp_addr, uint32 offset, uint16 len)
{
	uint16 * svmp_addr;
	uint16 i;

	if (!VASIP_PRESENT(wlc_hw->corerev)) {
		return BCME_UNSUPPORTED;
	}

	if (!wlc_hw->up) {
		return BCME_NOTUP;
	}

	if (((offset + (len * sizeof(*ret_svmp_addr))) > SVMP_MEM_OFFSET_MAX) ||
	     (len * sizeof(*ret_svmp_addr)) > SVMP_MEM_DUMP_LEN_MAX) {
		return BCME_RANGE;
	}

	svmp_addr = (uint16 *)(wlc_hw->vasip_addr + offset);

	for (i = 0; i < len; i++) {
		ret_svmp_addr[i] = svmp_addr[i];
	}
	return BCME_OK;
}

/* set svmp memory with a value from offset of length 'len', len is count of uint16's */
int
wlc_svmp_mem_set(wlc_hw_info_t *wlc_hw, uint32 offset, uint16 len, uint16 val)
{
	uint16 * svmp_addr;
	uint16 i;

	if (!VASIP_PRESENT(wlc_hw->corerev)) {
		return BCME_UNSUPPORTED;
	}

	if (!wlc_hw->up) {
		return BCME_NOTUP;
	}

	if (((offset + (len * sizeof(uint16))) > SVMP_MEM_OFFSET_MAX) ||
	    (len * sizeof(uint16)) > SVMP_MEM_DUMP_LEN_MAX) {
		return BCME_RANGE;
	}

	svmp_addr = (uint16 *)(wlc_hw->vasip_addr + offset);

	for (i = 0; i < len; i++) {
		svmp_addr[i] = val;
	}
	return BCME_OK;
}
#endif /* BCMDBG */
#endif /* VASIP_HW_SUPPORT */
