/*
 * ACPHY VCO CAL module implementation
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

#include <typedefs.h>
#include <bcmdefs.h>
#include <phy_dbg.h>
#include <phy_mem.h>
#include <phy_type_vcocal.h>
#include <phy_ac.h>
#include <phy_ac_vcocal.h>

#include <phy_ac_info.h>
#include <wlc_phy_radio.h>
#include <phy_utils_reg.h>
#include <wlc_radioreg_20691.h>
#include <wlc_radioreg_20693.h>

/* module private states */
struct phy_ac_vcocal_info {
	phy_info_t			*pi;
	phy_ac_info_t		*aci;
	phy_vcocal_info_t	*cmn_info;
/* add other variable size variables here at the end */
	acphy_vcocal_radregs_t ac_vcocal_radioregs_orig;
};

/* local functions */
static void wlc_phy_radio20693_vco_opt(phy_info_t *pi);

/* register phy type specific implementation */
phy_ac_vcocal_info_t *
BCMATTACHFN(phy_ac_vcocal_register_impl)(phy_info_t *pi, phy_ac_info_t *aci,
	phy_vcocal_info_t *cmn_info)
{
	phy_ac_vcocal_info_t *ac_info;
	phy_type_vcocal_fns_t fns;

	PHY_CAL(("%s\n", __FUNCTION__));

	/* allocate all storage together */
	if ((ac_info = phy_malloc(pi, sizeof(phy_ac_vcocal_info_t))) == NULL) {
		PHY_ERROR(("%s: phy_malloc failed\n", __FUNCTION__));
		goto fail;
	}

	/* Initialize params */
	ac_info->pi = pi;
	ac_info->aci = aci;
	ac_info->cmn_info = cmn_info;

	/* register PHY type specific implementation */
	bzero(&fns, sizeof(fns));
	fns.ctx = ac_info;

	if (phy_vcocal_register_impl(cmn_info, &fns) != BCME_OK) {
		PHY_ERROR(("%s: phy_vcocal_register_impl failed\n", __FUNCTION__));
		goto fail;
	}

	return ac_info;

	/* error handling */
fail:
	if (ac_info != NULL)
		phy_mfree(pi, ac_info, sizeof(phy_ac_vcocal_info_t));
	return NULL;
}

void
BCMATTACHFN(phy_ac_vcocal_unregister_impl)(phy_ac_vcocal_info_t *ac_info)
{
	phy_vcocal_info_t *cmn_info;
	phy_info_t *pi;

	ASSERT(ac_info);
	pi = ac_info->pi;
	cmn_info = ac_info->cmn_info;

	PHY_CAL(("%s\n", __FUNCTION__));

	/* unregister from common */
	phy_vcocal_unregister_impl(cmn_info);

	phy_mfree(pi, ac_info, sizeof(phy_ac_vcocal_info_t));
}

/* ********************************************* */
/*				Internal Definitions					*/
/* ********************************************* */
static void
wlc_phy_radio20693_vco_opt(phy_info_t *pi)
{
	uint8 core;
	phy_info_acphy_t *pi_ac = pi->u.pi_acphy;

	if (CHSPEC_IS2G(pi->radio_chanspec) &&
		(pi_ac->lpmode_2g != ACPHY_LPMODE_NONE)) {
		PHY_TRACE(("%s\n", __FUNCTION__));
		FOREACH_CORE(pi, core) {
			MOD_RADIO_REG_20693(pi, PLL_VCO3, core, rfpll_vco_en_alc, 0x1);
			MOD_RADIO_REG_20693(pi, PLL_VCO6, core, rfpll_vco_ALC_ref_ctrl, 0xa);
			MOD_RADIO_REG_20693(pi, PLL_HVLDO2, core, ldo_2p5_lowquiescenten_CP, 0x1);
			MOD_RADIO_REG_20693(pi, PLL_HVLDO2, core, ldo_2p5_lowquiescenten_VCO, 0x1);
			MOD_RADIO_REG_20693(pi, PLL_HVLDO4, core, ldo_2p5_static_load_CP, 0x1);
			MOD_RADIO_REG_20693(pi, PLL_HVLDO4, core, ldo_2p5_static_load_VCO, 0x1);
		}
	}
}

/* ********************************************* */
/*				External Functions					*/
/* ********************************************* */
void
wlc_phy_radio_tiny_vcocal(phy_info_t *pi)
{

	uint8 core;
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
	acphy_vcocal_radregs_t *porig = &(pi_ac->vcocali->ac_vcocal_radioregs_orig);

	ASSERT(TINY_RADIO(pi));

	if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
		ASSERT(!porig->is_orig);
		porig->is_orig = TRUE;

		/* Save the radio regs for core 0 */
		porig->clk_div_ovr1 = _READ_RADIO_REG(pi, RADIO_REG_20693(pi, CLK_DIV_OVR1, 0));
		porig->clk_div_cfg1 = _READ_RADIO_REG(pi, RADIO_REG_20693(pi, CLK_DIV_CFG1, 0));

		/* In core0, afeclk_6/12g_xxx_mimo_xxx needs to be off */
		MOD_RADIO_REG_20693(pi, CLK_DIV_OVR1, 0, ovr_afeclkdiv_6g_mimo_pu, 1);
		MOD_RADIO_REG_20693(pi, CLK_DIV_OVR1, 0, ovr_afeclkdiv_12g_mimo_div2_pu, 1);
		MOD_RADIO_REG_20693(pi, CLK_DIV_OVR1, 0, ovr_afeclkdiv_12g_mimo_pu, 1);
		MOD_RADIO_REG_20693(pi, CLK_DIV_CFG1, 0, afe_clk_div_6g_mimo_pu, 0);
		MOD_RADIO_REG_20693(pi, CLK_DIV_CFG1, 0, afe_clk_div_12g_mimo_div2_pu, 0);
		MOD_RADIO_REG_20693(pi, CLK_DIV_CFG1, 0, afe_clk_div_12g_mimo_pu, 0);
	}

	FOREACH_CORE(pi, core) {
		/* cal not required for non-zero cores in MIMO bu required for 80p80 */
		if ((phy_get_phymode(pi) == PHYMODE_MIMO) && (core != 0)) {
			break;
		}
		if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID)) {
			MOD_RADIO_REG_20691(pi, PLL_HVLDO3, core, ldo_2p5_ldo_VCO_vout_sel, 0xf);
			MOD_RADIO_REG_20691(pi, PLL_HVLDO3, core, ldo_2p5_ldo_CP_vout_sel, 0xf);
		}

		if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
			wlc_phy_radio20693_vco_opt(pi);
		}

		/* VCO-Cal startup seq */
		/* VCO cal mode selection */
		/* Use legacy mode */
		MOD_RADIO_REG_TINY(pi, PLL_VCOCAL10, core, rfpll_vcocal_ovr_mode, 0);

		/* # TODO: The below registers have direct PHY control in 20691 (unlike 2069?)
		 * so this reset should ideally be done by writing phy registers
		 */
		/* # Reset delta-sigma modulator */
		MOD_RADIO_REG_TINY(pi, PLL_CFG2, core, rfpll_rst_n, 0);
		/* # Reset VCO cal */
		MOD_RADIO_REG_TINY(pi, PLL_VCOCAL13, core, rfpll_vcocal_rst_n, 0);
		/* # Reset start of VCO Cal */
		MOD_RADIO_REG_TINY(pi, PLL_VCOCAL1, core, rfpll_vcocal_cal, 0);
		if (RADIOID_IS(pi->pubpi->radioid, BCM20691_ID)) {
			/* # Disable PHY direct control for vcocal reset */
			MOD_RADIO_REG_20691(pi, RFPLL_OVR1, core, ovr_rfpll_vcocal_rst_n, 1);
			/* # Disable PHY direct control for delta-sigma modulator reset signal */
			MOD_RADIO_REG_20691(pi, RFPLL_OVR1, core, ovr_rfpll_rst_n, 1);
			/* # Disable PHY direct control for vcocal start */
			MOD_RADIO_REG_20691(pi, RFPLL_OVR1, core, ovr_rfpll_vcocal_cal, 1);
		}
	}
	OSL_DELAY(11);
	FOREACH_CORE(pi, core) {
		/* cal not required for non-zero cores in MIMO but required for 80p80 */
		if ((phy_get_phymode(pi) == PHYMODE_MIMO) && (core != 0)) {
			break;
		}
		/* # Release reset */
		MOD_RADIO_REG_TINY(pi, PLL_CFG2, core, rfpll_rst_n, 1);
		/* # Release reset */
		MOD_RADIO_REG_TINY(pi, PLL_VCOCAL13, core, rfpll_vcocal_rst_n, 1);
	}
	OSL_DELAY(1);
	/* # Start VCO Cal */
	FOREACH_CORE(pi, core) {
		/* cal not required for non-zero cores in MIMO but required for 80p80 */
		if ((phy_get_phymode(pi) == PHYMODE_MIMO) && (core != 0)) {
			break;
		}
		MOD_RADIO_REG_TINY(pi, PLL_VCOCAL1, core, rfpll_vcocal_cal, 1);
	}
}

void
wlc_phy_radio2069_vcocal(phy_info_t *pi)
{
	/* Use legacy mode */
	uint8 legacy_n = 0;

	ASSERT(RADIOID_IS(pi->pubpi->radioid, BCM2069_ID));

	/* VCO cal mode selection */
	MOD_RADIO_REG(pi, RFP, PLL_VCOCAL10, rfpll_vcocal_ovr_mode, legacy_n);

	/* VCO-Cal startup seq */
	MOD_RADIO_REG(pi, RFP, PLL_CFG2, rfpll_rst_n, 0);
	MOD_RADIO_REG(pi, RFP, PLL_VCOCAL13, rfpll_vcocal_rst_n, 0);
	MOD_RADIO_REG(pi, RFP, PLL_VCOCAL1, rfpll_vcocal_cal, 0);
	OSL_DELAY(10);
	MOD_RADIO_REG(pi, RFP, PLL_CFG2, rfpll_rst_n, 1);
	MOD_RADIO_REG(pi, RFP, PLL_VCOCAL13, rfpll_vcocal_rst_n, 1);
	OSL_DELAY(1);
	MOD_RADIO_REG(pi, RFP, PLL_VCOCAL1, rfpll_vcocal_cal, 1);
}

#define MAX_2069x_VCOCAL_WAITLOOPS 100

/* vcocal should take < 120 us */
void
wlc_phy_radio2069x_vcocal_isdone(phy_info_t *pi, bool set_delay)
{
	/* Use legacy mode */
	uint8 done, itr;
	phy_info_acphy_t *pi_ac = (phy_info_acphy_t *)pi->u.pi_acphy;
	acphy_vcocal_radregs_t *porig = &(pi_ac->vcocali->ac_vcocal_radioregs_orig);

	if (ISSIM_ENAB(pi->sh->sih))
		return;

	/* Wait for vco_cal to be done, max = 100us * 10 = 1ms  */
	done = 0;
	for (itr = 0; itr < MAX_2069x_VCOCAL_WAITLOOPS; itr++) {
		OSL_DELAY(10);
		if (TINY_RADIO(pi)) {
			done = READ_RADIO_REGFLD_TINY(pi, PLL_VCOCAL14, 0, rfpll_vcocal_done_cal);
			/* In 80P80 mode, vcocal should be complete on core 0 and core 1 */
			if (phy_get_phymode(pi) == PHYMODE_80P80)
				done &= READ_RADIO_REGFLD_TINY(pi, PLL_VCOCAL14, 1,
					rfpll_vcocal_done_cal);
		} else
			done = READ_RADIO_REGFLD(pi, RFP, PLL_VCOCAL14, rfpll_vcocal_done_cal);
		if (done == 1)
			break;
	}

	/* Need to wait extra time after vcocal done bit is high for it to settle */
	if (set_delay == TRUE)
	  OSL_DELAY(120);

	ASSERT(done & 0x1);

	/* Restore the radio regs for core 0 */
	if (RADIOID_IS(pi->pubpi->radioid, BCM20693_ID)) {
		ASSERT(porig->is_orig);
		porig->is_orig = FALSE;

		phy_utils_write_radioreg(pi, RADIO_REG_20693(pi, CLK_DIV_OVR1, 0),
			porig->clk_div_ovr1);
		phy_utils_write_radioreg(pi, RADIO_REG_20693(pi, CLK_DIV_CFG1, 0),
			porig->clk_div_cfg1);
	}


	PHY_INFORM(("wl%d: %s vcocal done\n", pi->sh->unit, __FUNCTION__));
}
