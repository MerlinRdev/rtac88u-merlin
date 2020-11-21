/*
 * ACPHY RADIO control module interface (to other PHY modules).
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

#ifndef _phy_ac_radio_h_
#define _phy_ac_radio_h_

#include <phy_api.h>
#include <phy_ac.h>
#include <phy_radio.h>

#include <wlc_phytbl_20691.h>
#include <wlc_phytbl_20693.h>

#ifdef ATE_BUILD
#include <wl_ate.h>
#endif /* ATE_BUILD */

/* 20693 Radio functions */
typedef enum {
	RADIO_20693_FAST_ADC,
	RADIO_20693_SLOW_ADC
} radio_20693_adc_modes_t;

typedef struct _acphy_pmu_core1_off_radregs_t {
	uint16 pll_xtalldo1[PHY_CORE_MAX];
	uint16 pll_cp1[PHY_CORE_MAX];
	uint16 vreg_cfg[PHY_CORE_MAX];
	uint16 vreg_ovr1[PHY_CORE_MAX];
	uint16 pmu_op[PHY_CORE_MAX];
	uint16 pmu_cfg4[PHY_CORE_MAX];
	uint16 logen_cfg2[PHY_CORE_MAX];
	uint16 logen_ovr1[PHY_CORE_MAX];
	uint16 logen_cfg3[PHY_CORE_MAX];
	uint16 logen_ovr2[PHY_CORE_MAX];
	uint16 clk_div_cfg1[PHY_CORE_MAX];
	uint16 clk_div_ovr1[PHY_CORE_MAX];
	uint16 spare_cfg2[PHY_CORE_MAX];
	uint16 auxpga_ovr1[PHY_CORE_MAX];
	bool   is_orig;
} acphy_pmu_core1_off_radregs_t;

/* forward declaration */
typedef struct phy_ac_radio_info phy_ac_radio_info_t;

/* register/unregister ACPHY specific implementations to/from common */
phy_ac_radio_info_t *phy_ac_radio_register_impl(phy_info_t *pi,
	phy_ac_info_t *aci, phy_radio_info_t *ri);
void phy_ac_radio_unregister_impl(phy_ac_radio_info_t *info);

/* query and parse idcode */
uint32 phy_ac_radio_query_idcode(phy_info_t *pi);
void phy_ac_radio_parse_idcode(phy_info_t *pi, uint32 idcode);


/* ************************************************************************* */
/* ************************************************************************* */
/* ************************************************************************* */
/* ************************************************************************* */
extern void wlc_phy_set_regtbl_on_pwron_acphy(phy_info_t *pi);
extern void wlc_phy_radio20693_mimo_core1_pmu_off(phy_info_t *pi);
extern void wlc_phy_radio20693_mimo_core1_pmu_on(phy_info_t *pi);
extern void wlc_phy_radio2069_pwrdwn_seq(phy_info_t *pi);
extern void wlc_phy_radio2069_pwrup_seq(phy_info_t *pi);
extern void wlc_acphy_get_radio_loft(phy_info_t *pi, uint8 *ei0,
	uint8 *eq0, uint8 *fi0, uint8 *fq0);
extern void wlc_acphy_set_radio_loft(phy_info_t *pi, uint8, uint8, uint8, uint8);
extern int wlc_phy_chan2freq_20691(phy_info_t *pi, uint8 channel, const chan_info_radio20691_t
	**chan_info);
extern void wlc_phy_radio20693_config_bf_mode(phy_info_t *pi, uint8 core);
extern int wlc_phy_chan2freq_20693(phy_info_t *pi, uint8 channel,
	const chan_info_radio20693_pll_t **chan_info_pll,
	const chan_info_radio20693_rffe_t **chan_info_rffe);
extern int8 wlc_phy_tiny_radio_minipmu_cal(phy_info_t *pi);
extern void wlc_phy_radio20693_afeclkpath_setup(phy_info_t *pi, uint8 core,
	radio_20693_adc_modes_t adc_mode, uint8 direct_ctrl_en);
extern void wlc_phy_radio20693_adc_powerupdown(phy_info_t *pi,
	radio_20693_adc_modes_t adc_mode, uint8 pupdbit, uint8 core);
extern int wlc_phy_radio20693_altclkpln_get_chan_row(phy_info_t *pi);
extern void wlc_phy_radio20693_afecal(phy_info_t *pi);
extern void phy_ac_dsi_radio_fns(phy_info_t *pi);

/* Cleanup Chanspec */
extern void chanspec_setup_radio(phy_info_t *pi);
extern void chanspec_tune_radio(phy_info_t *pi);

#endif /* _phy_ac_radio_h_ */
