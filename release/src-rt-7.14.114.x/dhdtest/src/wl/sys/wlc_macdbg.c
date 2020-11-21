/*
 * MAC debug and print functions
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
 * $Id$
 */

#include <wlc_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <bcmendian.h>
#include <wlc_types.h>
#include <wlioctl.h>
#include <proto/802.11.h>
#include <d11.h>
#include <hnddma.h>
#include <wlc_pub.h>
#include <wlc.h>
#include <wlc_bmac.h>
#include <wlc_ampdu.h>
#include <wlc_macdbg.h>
#include <wlc_bsscfg.h>

/* this is for dump_mac */
enum {
	D11REG_TYPE_IHR16 = 0,
	D11REG_TYPE_IHR32 = 1,
	D11REG_TYPE_SCR = 2,
	D11REG_TYPE_SHM = 3,
	D11REG_TYPE_IHRX16 = 4,
	D11REG_TYPE_SHMX = 5,
	D11REG_TYPE_MAX
};

typedef struct _d11regs_bmp_list {
	uint8 type;
	uint16 addr;
	uint32 bitmap;
	uint8 step;
	uint16 cnt; /* can be used together with bitmap or by itself */
} d11regs_list_t;

#define D11REG_BLK_SIZE		32
typedef struct _d11regs_addr_list {
	uint8 type;
	uint16 cnt;
	uint16 addr[D11REG_BLK_SIZE]; /* allow up to 32 per list */
} d11regs_addr_t;

#define C_CREVS_PRE40		1
#define C_CREVS_GE40		2
#define C_CREVS_ALL		(C_CREVS_PRE40 | C_CREVS_GE40)
/* this form is useful for regs located in blocks */
typedef struct _d11dbg_list1 {
	d11regs_list_t reglist;
	uint8 crevs; /* applicable to which corerevs */
} d11dbg_list1_t;

/* this form is useful for scattered regs */
typedef struct _d11dbg_list2 {
	d11regs_addr_t addrlist;
	uint8 crevs; /* applicable to which corerevs */
} d11dbg_list2_t;

/* this is for dump_ucode_fatal */
typedef struct _d11print_list {
	char name[16]; /* maximum 16 chars */
	uint16 addr;
} d11print_list_t;

/* this is for dump_shmem */
typedef struct _shmem_list {
	uint16	start;
	uint16	end;
} shmem_list_t;

static int
wlc_macdbg_doiovar(void *context, const bcm_iovar_t *vi, uint32 actionid, const char *name,
	void *params, uint p_len, void *arg, int len, int vsize, struct wlc_if *wlcif);
#if WL_MACDBG
static int wlc_dump_shmem(wlc_info_t *wlc, struct bcmstrbuf *b);
static int wlc_dump_sctpl(wlc_info_t *wlc, struct bcmstrbuf *b);
static int wlc_dump_bcntpls(wlc_info_t *wlc, struct bcmstrbuf *b);
static int wlc_dump_pio(wlc_info_t *wlc, struct bcmstrbuf *b);
static int wlc_macdbg_pmac(wlc_info_t *wlc, wl_macdbg_pmac_param_t *pmac,
	char *out_buf, int out_len);
#endif /* WL_MACDBG */
#if defined(BCMDBG)
static int wlc_dump_dma(wlc_info_t *wlc, struct bcmstrbuf *b);
#endif

/** iovar table */
enum {
	IOV_MACDBG_PMAC,		/* print mac */
	IOV_MACDBG_LAST
};

static const bcm_iovar_t macdbg_iovars[] = {
	{"pmac", IOV_MACDBG_PMAC, (IOVF_SET_UP|IOVF_GET_UP), IOVT_BUFFER, 0},
	{NULL, 0, 0, 0, 0}
};

void
BCMATTACHFN(wlc_macdbg_detach)(wlc_info_t *wlc)
{
	wlc_module_unregister(wlc->pub, "macdbg", wlc);
}

int
BCMATTACHFN(wlc_macdbg_attach)(wlc_info_t *wlc)
{
	wlc_pub_t *pub = wlc->pub;
	int err = BCME_OK;

	if ((err = wlc_module_register(pub, macdbg_iovars, "macdbg",
		wlc, wlc_macdbg_doiovar, NULL, NULL, NULL))) {
		WL_ERROR(("wl%d: %s: wlc_module_register() failed\n",
			wlc->pub->unit, __FUNCTION__));
		return err;
	}

#if WL_MACDBG
	wlc_dump_register(pub, "mac", (dump_fn_t)wlc_dump_mac, (void *)wlc);
	wlc_dump_register(pub, "shmem", (dump_fn_t)wlc_dump_shmem, (void *)wlc);
	wlc_dump_register(pub, "sctpl", (dump_fn_t)wlc_dump_sctpl, (void *)wlc);
	wlc_dump_register(pub, "bcntpl", (dump_fn_t)wlc_dump_bcntpls, (void *)wlc);
	wlc_dump_register(pub, "pio", (dump_fn_t)wlc_dump_pio, (void *)wlc);
#endif /* WL_MACDBG */
#if defined(BCMDBG)
	wlc_dump_register(pub, "dma", (dump_fn_t)wlc_dump_dma, (void *)wlc);
#endif
	return err;
}

/* add dump enum here */
static int
wlc_macdbg_doiovar(void *context, const bcm_iovar_t *vi, uint32 actionid, const char *name,
	void *params, uint p_len, void *arg, int len, int vsize, struct wlc_if *wlcif)
{
	wlc_info_t *wlc = context;
	int err = BCME_OK;
	wlc_bsscfg_t *bsscfg;
	int32 *ret_int_ptr = (int32 *)arg;

	bsscfg = wlc_bsscfg_find_by_wlcif(wlc, wlcif);
	ASSERT(bsscfg != NULL);
	BCM_REFERENCE(ret_int_ptr);
	BCM_REFERENCE(bsscfg);

	switch (actionid) {
#if WL_MACDBG
		case IOV_GVAL(IOV_MACDBG_PMAC):
		{
			err = wlc_macdbg_pmac(wlc, params, arg, len);
			break;
		}
#endif /* WL_MACDBG */
	}
	return err;
}

#if WL_MACDBG
/* dump functions */
static int
wlc_print_d11reg(wlc_info_t *wlc, int idx, int type, uint16 addr, struct bcmstrbuf *b)
{
	const char *regname[D11REG_TYPE_MAX] = {"ihr", "ihr", "scr", "shm"};
	d11regs_t *regs;
	osl_t *osh;
	uint16 val;
	volatile uint8 *paddr;

	osh = wlc->osh;
	regs = wlc->regs;
	paddr = (volatile uint8*)(&regs->biststatus) - 0xC;

	if (type == D11REG_TYPE_IHR32) {
		if (b)
			bcm_bprintf(b, "%-3d %s 0x%-4x = 0x%-8x\n",
				idx, regname[type], addr,
				R_REG(osh, (volatile uint32*)(paddr + addr)));
		else
			printf("%-3d %s 0x%-4x = 0x%-8x\n",
			       idx, regname[type], addr,
			       R_REG(osh, (volatile uint32*)(paddr + addr)));
	} else {
		switch (type) {
		case D11REG_TYPE_IHR16:
			val = R_REG(osh, (volatile uint16*)(paddr + addr));
			break;
		case D11REG_TYPE_SCR:
			wlc_bmac_copyfrom_objmem(wlc->hw, addr << 2,
				&val, sizeof(val), OBJADDR_SCR_SEL);
			break;
		case D11REG_TYPE_SHM:
			val = wlc_read_shm(wlc, addr);
			break;
		default:
			printf("Unrecognized type %d!\n", type);
			return 0;
		}
		if (b)
			bcm_bprintf(b, "%-3d %s 0x%-4x = 0x%-4x\n",
				idx, regname[type], addr, val);
		else
			printf("%-3d %s 0x%-4x = 0x%-4x\n",
			       idx, regname[type], addr, val);
	}
	return 1;
}

static int
wlc_print_d11regs(wlc_info_t *wlc, int bidx, d11regs_list_t *pregs, struct bcmstrbuf *b)
{
	uint16 addr;
	int idx;

	addr = pregs->addr;
	idx = bidx;
	if (pregs->type >= D11REG_TYPE_MAX) {
		if (b)
			bcm_bprintf(b, "%s: wrong type %d\n", __FUNCTION__, pregs->type);
		else
			printf("%s: wrong type %d\n", __FUNCTION__, pregs->type);
		return 0;
	}
	while (pregs->bitmap || pregs->cnt) {
		if ((pregs->bitmap && (pregs->bitmap & 0x1)) ||
		    (!pregs->bitmap && pregs->cnt)) {
			wlc_print_d11reg(wlc, idx, pregs->type, addr, b);
			idx ++;
		}
		pregs->bitmap = pregs->bitmap >> 1;
		if (pregs->cnt) pregs->cnt --;
		addr += pregs->step;
	}
	return (idx - bidx);
}

static int
wlc_print_d11regs_list(wlc_info_t *wlc, struct bcmstrbuf *b,
	d11dbg_list1_t *d11dbg1, uint d11dbg1_sz, d11dbg_list2_t *d11dbg2, uint d11dbg2_sz)
{
	d11regs_list_t *pregs;
	d11regs_addr_t *pdbg2;
	uint8 crevs_mask;
	int i, j, cnt;

	if (!wlc->clk)
		return BCME_NOCLK;

	printf("%s: ucode compile time 0x%04x 0x%04x\n", __FUNCTION__,
	       wlc_read_shm(wlc, 0x4), wlc_read_shm(wlc, 0x6));

	crevs_mask = D11REV_GE(wlc->pub->corerev, 40) ? C_CREVS_GE40 : C_CREVS_PRE40;

	cnt = 0;
	for (i = 0; i < (int)d11dbg1_sz; i++) {
		if (!(crevs_mask & d11dbg1[i].crevs))
			continue;
		pregs = &(d11dbg1[i].reglist);
		cnt += wlc_print_d11regs(wlc, cnt, pregs, b);
	}
	for (i = 0; i < (int)d11dbg2_sz; i++) {
		if (!(crevs_mask & d11dbg2[i].crevs))
			continue;
		pdbg2 = &(d11dbg2[i].addrlist);
		if (pdbg2->type >= D11REG_TYPE_MAX) {
			if (b)
				bcm_bprintf(b, "%s: wrong type %d. Skip %d entries.\n",
					__FUNCTION__, pdbg2->type, pdbg2->cnt);
			else
				printf("%s: wrong type %d. Skip %d entries.\n",
				       __FUNCTION__, pdbg2->type, pdbg2->cnt);
			continue;
		}
		for (j = 0; j < pdbg2->cnt; j++) {
			cnt += wlc_print_d11reg(wlc, cnt, pdbg2->type, pdbg2->addr[j], b);
		}
	}
	return cnt;
}

static int
wlc_macdbg_pmac(wlc_info_t *wlc,
	wl_macdbg_pmac_param_t *pmac, char *out_buf, int out_len)
{
	uint8 i, type;
	int err = BCME_OK, cnt;
	struct bcmstrbuf b;

	if (pmac->addr_num == 0) {
		/* No address given */
		WL_ERROR(("%s %d\n", __FUNCTION__, __LINE__));
		err = BCME_BADARG;
		goto exit;
	}

	bcm_binit(&b, out_buf, out_len);

	if (!strncmp(pmac->type, "shmx", 4)) {
		type = D11REG_TYPE_SHMX;
	} else if (!strncmp(pmac->type, "ihrx", 4)) {
		type = D11REG_TYPE_IHRX16;
	} else if (!strncmp(pmac->type, "scr", 3)) {
		type = D11REG_TYPE_SCR;
	} else if (!strncmp(pmac->type, "shm", 3)) {
		type = D11REG_TYPE_SHM;
	} else if (!strncmp(pmac->type, "ihr32", 5)) {
		type = D11REG_TYPE_IHR32;
	} else if (!strncmp(pmac->type, "ihr", 3)) {
		type = D11REG_TYPE_IHR16;
	} else {
		WL_ERROR(("Invalid selection!!\n"));
		err = BCME_BADARG;
		goto exit;
	}

	if (type == D11REG_TYPE_SCR) {
		if (pmac->step == (uint8)(-1)) {
			/* Set the default step when it is not given */
			pmac->step = 1;
		}
	} else {
		for (i = 0; i < pmac->addr_num; i++) {
			if (pmac->addr_raw) {
				/* internal address => external address. */
				if ((type == D11REG_TYPE_IHR16) ||
					(type == D11REG_TYPE_IHRX16)) {
					pmac->addr[i] += 0x200;
				}
				pmac->addr[i] <<= 1;
			}
			if ((pmac->addr[i] & 0x1)) {
				/* Odd addr not expected here for external addr */
				WL_ERROR(("%s %d\n", __FUNCTION__, __LINE__));
				err = BCME_BADARG;
				goto exit;
			}
		}
		if (pmac->step == (uint8)(-1)) {
			/* Set the default step when it is not given */
			pmac->step = (type == D11REG_TYPE_IHR32) ? 4 : 2;
		} else if ((pmac->step & (type == D11REG_TYPE_IHR32 ? 0x3 : 0x1))) {
			/* step size validation check. */
			WL_ERROR(("%s %d\n", __FUNCTION__, __LINE__));
			err = BCME_BADARG;
			goto exit;
		}
	}

	if (pmac->num + pmac->bitmap > 0) {
		d11dbg_list1_t d11dbg1;
		if (pmac->addr_num > 1) {
			/* If num or bitmap option is on, only one address is expected.
			 */
			WL_ERROR(("%s %d\n", __FUNCTION__, __LINE__));
			err = BCME_BADARG;
			goto exit;
		}
		d11dbg1.reglist.type = type;
		d11dbg1.reglist.addr = pmac->addr[0];
		d11dbg1.reglist.bitmap = pmac->bitmap;
		d11dbg1.reglist.step = pmac->step;
		d11dbg1.reglist.cnt = pmac->num;
		d11dbg1.crevs = C_CREVS_ALL;
		WL_TRACE(("d11dbg1: type %d, addr 0x%x, bitmap 0x%x, step %d, cnt %d\n",
			d11dbg1.reglist.type,
			d11dbg1.reglist.addr,
			d11dbg1.reglist.bitmap,
			d11dbg1.reglist.step,
			d11dbg1.reglist.cnt));
		cnt = wlc_print_d11regs_list(wlc, &b, &d11dbg1, 1, NULL, 0);
	} else {
		d11dbg_list2_t d11dbg2;
		d11dbg2.addrlist.type = type;
		d11dbg2.addrlist.cnt = pmac->addr_num;
		memcpy(d11dbg2.addrlist.addr, pmac->addr, (sizeof(uint16) * pmac->addr_num));
		d11dbg2.crevs = C_CREVS_ALL;
		WL_TRACE(("d11dbg2: type %d, cnt %d\n",
			d11dbg2.addrlist.type,
			d11dbg2.addrlist.cnt));
		for (i = 0; i < pmac->num; i++) {
			WL_TRACE(("[%d] 0x%x ", i, d11dbg2.addrlist.addr[i]));
		}
		cnt = wlc_print_d11regs_list(wlc, &b, NULL, 0, &d11dbg2, 1);
	}

	if (cnt < 0) {
		err = BCME_ERROR;
	}
exit:
	return err;
}

int
wlc_dump_mac(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	int cnt;
	d11dbg_list1_t d11dbg1[] = {
		/* default list for general info */
		{{D11REG_TYPE_IHR32, 0x120, 0xf00f, 4, 0}, C_CREVS_ALL},
		{{D11REG_TYPE_IHR32, 0x180, 0x030f, 4, 0}, C_CREVS_ALL},
		{{D11REG_TYPE_IHR16, 0x804, 0, 2, 28}, C_CREVS_PRE40},
		{{D11REG_TYPE_SHM, (0xaca*2), 0, 2, 128}, C_CREVS_ALL},
	};
	d11dbg_list2_t d11dbg2[] = {
		/* common */
		{{D11REG_TYPE_IHR16, 10,
		{0x490, 0x492, 0x4ce, 0x4d8, 0x4f0, 0x500, 0x50e, 0x688,
		0x690, 0x7c0}},
		C_CREVS_ALL},
		/* for 11ac */
		{{D11REG_TYPE_IHR16, 4,
		{0x522, 0x548, 0x838, 0xa00}},
		C_CREVS_GE40},
	};

	cnt = wlc_print_d11regs_list(wlc, b, d11dbg1, ARRAYSIZE(d11dbg1),
		d11dbg2, ARRAYSIZE(d11dbg2));

	/* wlc_dump_ucode_fatal(wlc, 0); */ /* keep for testing */
	return cnt;
}

static int
wlc_dump_shmem(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	uint i;
	uint16 val, addr;

	static const shmem_list_t shmem_list[] = {
		{0x0,	0x80},
		{0x100, 0x500},
	};

	if (!wlc->clk) {
		WL_ERROR(("wl%d: %s: clock must be on\n", wlc->pub->unit, __FUNCTION__));
		return BCME_NOCLK;
	}
	for (i = 0; i < ARRAYSIZE(shmem_list); i++) {
		for (addr = shmem_list[i].start; addr < shmem_list[i].end; addr += 2) {
			val = wlc_read_shm(wlc, addr);
			bcm_bprintf(b, "0x%03x * 2: 0x%03x 0x%04x\n", addr >> 1,
			            addr, val);
		}
	}

	return 0;
}

static int
wlc_dump_sctpl(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	d11regs_t *regs;
	osl_t *osh;
	uint16 val;
	uint i;
	int gpio_sel;
	uint16 phyctl, addr0, addr1, curptr, len, offset;

	if (D11REV_LT(wlc->pub->corerev, 40)) {
		WL_ERROR(("wl%d: %s only supported for corerev >=40\n",
			wlc->pub->unit, __FUNCTION__));
		return BCME_UNSUPPORTED;
	}

	if (!wlc->clk) {
		WL_ERROR(("wl%d: %s: clock must be on\n", wlc->pub->unit, __FUNCTION__));
		return BCME_NOCLK;
	}

	regs = wlc->regs;
	osh = wlc->osh;
	phyctl = R_REG(osh, &regs->psm_phy_hdr_param);

	/* stop sample capture */
	W_REG(osh, &regs->psm_phy_hdr_param, phyctl & ~(1 << 4));

	gpio_sel = R_REG(osh, &regs->maccontrol1);
	addr0 = R_REG(osh, &regs->u.d11acregs.SampleCollectStartPtr);
	addr1 = R_REG(osh, &regs->u.d11acregs.SampleCollectStopPtr);
	curptr = R_REG(osh, &regs->u.d11acregs.SampleCollectCurPtr);
	len = (addr1 - addr0 + 1) * 4;
	offset = addr0 * 4;

	if (b) {
		bcm_bprintf(b, "Capture mode: maccontrol1 0x%02x phyctl 0x%02x\n",
			gpio_sel, phyctl);
		bcm_bprintf(b, "Start/stop/cur 0x%04x 0x%04x 0x%04x byt_offset 0x%04x entries %u\n",
			addr0, addr1, curptr, 4 *(curptr - addr0), len>>2);
		bcm_bprintf(b, "offset: low high\n");
	} else {
		printf("Capture mode: maccontrol1 0x%02x phyctl 0x%02x\n", gpio_sel, phyctl);
		printf("Start/stop/cur 0x%04x 0x%04x 0x%04x byt_offset 0x%04x entries %u\n",
		       addr0, addr1, curptr, 4 *(curptr - addr0), len>>2);
		printf("offset: low high\n");
	}

	while ((val = R_REG(osh, &regs->u.d11acregs.XmtTemplatePtr)) & 3)
		printf("read_txe_ram: polling XmtTemplatePtr 0x%x\n", val);

	for (i = 0; i < (uint)len; i += 4) {
		uint16 low, hiw;
		W_REG(osh, &regs->u.d11acregs.XmtTemplatePtr, (offset + i) | 2);
		while ((val = R_REG(osh, &regs->u.d11acregs.XmtTemplatePtr)) & 3)
			printf("read_txe_ram: polling XmtTemplatePtr 0x%x\n", val);
		hiw = R_REG(osh, &regs->u.d11acregs.XmtTemplateDataHi);
		low = R_REG(osh, &regs->u.d11acregs.XmtTemplateDataLo);
		if (b)
			bcm_bprintf(b, "%04X: %04X %04X\n", i, low, hiw);
		else
			printf("%04X: %04X %04X\n", i, low, hiw);
	}
	return BCME_OK;
}

/** dump beacon (from read_txe_ram in d11procs.tcl) */
static void
wlc_dump_bcntpl(wlc_info_t *wlc, struct bcmstrbuf *b, int offset, int len)
{
	d11regs_t *regs = wlc->regs;
	uint16 val;
	uint i;

	ASSERT(D11REV_GE(wlc->pub->corerev, 40));

	len = (len + 3) & ~3;

	bcm_bprintf(b, "tpl: offset %d len %d\n", offset, len);

	while ((val = R_REG(wlc->osh, &regs->u.d11acregs.XmtTemplatePtr)) & 3)
		printf("read_txe_ram: polling XmtTemplatePtr 0x%x\n", val);

	for (i = 0; i < (uint)len; i += 4) {
		W_REG(wlc->osh, &regs->u.d11acregs.XmtTemplatePtr, (offset + i) | 2);
		while ((val = R_REG(wlc->osh, &regs->u.d11acregs.XmtTemplatePtr)) & 3)
			printf("read_txe_ram: polling XmtTemplatePtr 0x%x\n", val);
		bcm_bprintf(b, "%04X: %04X%04X\n", i,
		            R_REG(wlc->osh, &regs->u.d11acregs.XmtTemplateDataHi),
		            R_REG(wlc->osh, &regs->u.d11acregs.XmtTemplateDataLo));
	}
}

static int
wlc_dump_bcntpls(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	uint16 len;

	if (D11REV_LT(wlc->pub->corerev, 40)) {
		WL_ERROR(("wl%d: %s only supported for corerev >=40\n",
		         wlc->pub->unit, __FUNCTION__));
		return BCME_UNSUPPORTED;
	}

	if (!wlc->clk) {
		WL_ERROR(("wl%d: %s: clock must be on\n", wlc->pub->unit, __FUNCTION__));
		return BCME_NOCLK;
	}

	len = wlc_read_shm(wlc, M_BCN0_FRM_BYTESZ);
	bcm_bprintf(b, "bcn 0: len %u\n", len);
	wlc_dump_bcntpl(wlc, b, D11AC_T_BCN0_TPL_BASE, len);
	len = wlc_read_shm(wlc, M_BCN1_FRM_BYTESZ);
	bcm_bprintf(b, "bcn 1: len %u\n", len);
	wlc_dump_bcntpl(wlc, b, D11AC_T_BCN1_TPL_BASE, len);

	return 0;
}

static int
wlc_dump_pio(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	int i;

	if (!wlc->clk)
		return BCME_NOCLK;

	if (!PIO_ENAB(wlc->pub))
		return 0;

	for (i = 0; i < NFIFO; i++) {
		pio_t *pio = WLC_HW_PIO(wlc, i);
		bcm_bprintf(b, "PIO %d: ", i);
		if (pio != NULL)
			wlc_pio_dump(pio, b);
		bcm_bprintf(b, "\n");
	}

	return 0;
}
#endif /* WL_MACDBG */

#if defined(BCMDBG)
static int
wlc_dump_dma(wlc_info_t *wlc, struct bcmstrbuf *b)
{
	int i;
	wl_cnt_wlc_t *cnt = wlc->pub->_cnt;

	if (!wlc->clk)
		return BCME_NOCLK;

	for (i = 0; i < NFIFO; i++) {
		PRREG_INDEX(intstatus, wlc->regs->intctrlregs[i].intstatus);
		PRREG_INDEX(intmask, wlc->regs->intctrlregs[i].intmask);
		bcm_bprintf(b, "\n");
		if (!PIO_ENAB(wlc->pub)) {
			hnddma_t *di = WLC_HW_DI(wlc, i);
			bcm_bprintf(b, "DMA %d: ", i);
			if (di != NULL) {
				dma_dumptx(di, b, TRUE);
				if ((i == RX_FIFO) ||
				    (D11REV_IS(wlc->pub->corerev, 4) && (i == RX_TXSTATUS_FIFO))) {
					dma_dumprx(di, b, TRUE);
					PRVAL(rxuflo[i]);
				} else
					PRVAL(txuflo);
				PRNL();
			}
		}
		bcm_bprintf(b, "\n");
	}

	PRVAL(dmada); PRVAL(dmade); PRVAL(rxoflo); PRVAL(dmape);
	bcm_bprintf(b, "\n");

	return 0;
}
#endif  

/* ************* end of dump_xxx function section *************************** */

/* print upon critical or fatal error */

void
wlc_dump_ucode_fatal(wlc_info_t *wlc, uint reason)
{
	wlc_pub_t *pub;
	osl_t *osh;
	d11regs_t *regs;
	uint32 phydebug, psmdebug;
	uint16 val16[4];
	uint32 val32[4];
	int i, k;
	volatile uint8 *paddr;
	/* two lists: one common, one corerev specific */
	d11print_list_t *plist[2];
	int lsize[2];
	const char reason_str[][20] = {
		"any",
		"psm watchdog",
		"mac suspend failure"
	};

	/* common list */
	d11print_list_t cmlist[] = {
		{"ifsstat", 0x690},
		{"ifsstat1", 0x698},
		{"txectl", 0x500},
		{"txestat", 0x50e},
		{"txestat2", 0x578},
		{"rxestat1", 0x41a},
		{"rxestat2", 0x41c},
		{"rcv_frmcnt", 0x40a},
		{"rxe_rxcnt", 0x418},
		{"wepctl", 0x7c0},
		{"psm_pcerr", 0x48c},
		{"psm_ihrerr", 0x4ce},
	};
	/* reg specific to corerev < 40 */
	d11print_list_t list_lt40[] = {
		{"pcmctl", 0x7d0},
		{"pcmstat", 0x7d2},
	};
	/* reg specific to corerev >= 40 */
	d11print_list_t list_ge40[] = {
		{"wepstat", 0x7c2},
		{"wep_ivloc", 0x7c4},
		{"wep_psdulen", 0x7c6},
		{"daggctl", 0x448},
		{"daggctl2", 0x8c0},
		{"dagg_bleft",  0x8c2},
		{"dagglen", 0x8c8},
		{"daggstat", 0x8c6}
	};
	d11print_list_t phyreg_ge40[] = {
		{"pktproc", 0x1f0}, /* repeat four times */
		{"pktproc", 0x1f0},
		{"pktproc", 0x1f0},
		{"pktproc", 0x1f0}
	};

	pub = wlc->pub;
	osh = wlc->osh;
	regs = wlc->regs;
	BCM_REFERENCE(val16);
	BCM_REFERENCE(val32);

	k = (reason >= PSM_FATAL_LAST) ? PSM_FATAL_ANY : reason;
	WL_PRINT(("wl%d: reason = %s. corerev %d ", pub->unit, reason_str[k], pub->corerev));
	if (!wlc->clk) {
		WL_PRINT(("%s: no clk\n", __FUNCTION__));
		return;
	}
	WL_PRINT(("ucode revision %d.%d\n",
		wlc_read_shm(wlc, UCODE_MAJOR_BOM), wlc_read_shm(wlc, UCODE_MINOR_BOM)));

	psmdebug = R_REG(osh, &regs->psmdebug);
	phydebug = R_REG(osh, &regs->phydebug);
	val32[0] = R_REG(osh, &regs->maccontrol);
	val32[1] = R_REG(osh, &regs->maccommand);
	val16[0] = R_REG(osh, &regs->psm_brc);
	val16[1] = R_REG(osh, &regs->psm_brc_1);
	val16[2] = wlc_read_shm(wlc, M_UCODE_DBGST);

	wlc_mac_event(wlc, WLC_E_PSM_WATCHDOG, NULL, psmdebug, phydebug, val16[0], NULL, 0);
	WL_PRINT(("psmdebug 0x%08x phydebug 0x%x macctl 0x%x maccmd 0x%x\n"
		 "psm_brc 0x%04x psm_brc_1 0x%04x M_UCODE_DBGST 0x%x\n",
		 psmdebug, phydebug, val32[0], val32[1], val16[0], val16[1], val16[2]));

	paddr = (volatile uint8*)(&regs->biststatus) - 0xC;
	plist[0] = cmlist;
	lsize[0] = ARRAYSIZE(cmlist);
	if (D11REV_LT(pub->corerev, 40)) {
		plist[1] = list_lt40;
		lsize[1] = ARRAYSIZE(list_lt40);
	} else {
		plist[1] = list_ge40;
		lsize[1] = ARRAYSIZE(list_ge40);
	}
	for (i = 0; i < 2; i ++) {
		for (k = 0; k < lsize[i];  k ++) {
			val16[0] = R_REG(osh, (volatile uint16*)(paddr + plist[i][k].addr));
			WL_PRINT(("%-12s 0x%-4x ", plist[i][k].name, val16[0]));
			if ((k % 4) == 3)
				WL_PRINT(("\n"));
		}
		if (k % 4) {
			WL_PRINT(("\n"));
		}
	}


#ifdef WLAMPDU_MAC
	if (AMPDU_MAC_ENAB(pub))
		wlc_dump_aggfifo(wlc, NULL);
#endif /* WLAMPDU_MAC */

	WL_PRINT(("PC :\n"));
	for (k = 0; k < 64; k += 4) {
		val32[0] = R_REG(osh, &regs->psmdebug);
		val32[1] = R_REG(osh, &regs->psmdebug);
		val32[2] = R_REG(osh, &regs->psmdebug);
		val32[3] = R_REG(osh, &regs->psmdebug);
		WL_PRINT(("0x%-8x 0x%-8x 0x%-8x 0x%-8x\n",
			val32[0], val32[1], val32[2], val32[3]));
	}
	/* phyreg */
	if (D11REV_GE(pub->corerev, 40) && D11REV_LT(pub->corerev, 64)) {
		plist[0] = phyreg_ge40;
		lsize[0] = ARRAYSIZE(phyreg_ge40);

		WL_PRINT(("phyreg :\n"));
		for (k = 0; k < lsize[0]; k++) {
			W_REG(osh, &regs->phyregaddr, plist[0][k].addr);
			val16[0] = R_REG(osh, &regs->phyregdata);
			WL_PRINT(("%-12s 0x%-4x ", plist[0][k].name, val16[0]));
			if ((k % 4) == 3)
				WL_PRINT(("\n"));
		}
		if (k % 4) {
			WL_PRINT(("\n"));
		}
	}

	if (phydebug > 0) {
		WL_PRINT(("phydebug :\n"));
		for (k = 0; k < 64; k += 4) {
			val32[0] = R_REG(osh, &regs->phydebug);
			val32[1] = R_REG(osh, &regs->phydebug);
			val32[2] = R_REG(osh, &regs->phydebug);
			val32[3] = R_REG(osh, &regs->phydebug);
			WL_PRINT(("0x%-8x 0x%-8x 0x%-8x 0x%-8x\n",
				val32[0], val32[1], val32[2], val32[3]));
		}
	}
}
