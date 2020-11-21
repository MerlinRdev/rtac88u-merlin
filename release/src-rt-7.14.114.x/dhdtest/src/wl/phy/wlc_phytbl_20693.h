/*
 * Declarations for Broadcom PHY core tables,
 * Networking Adapter Device Driver.
 *
 * THE CONTENTS OF THIS FILE IS TEMPORARY.
 * Eventually it'll be auto-generated.
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
 * All Rights Reserved.
 *
 * $Id$
 */

#ifndef _WLC_PHYTBL_20693_H_
#define _WLC_PHYTBL_20693_H_

#include <wlc_cfg.h>
#include <typedefs.h>

#include "wlc_phy_int.h"

#define NUM_ROWS_CHAN_TUNING 77
#define NUM_ALTCLKPLN_CHANS 5

/*
 * Channel Info table for the 20693 (4349).
 */

typedef struct _chan_info_radio20693_pll {
	uint8 chan;            /* channel number */
	uint16 freq;            /* in Mhz */
	/* other stuff */
	uint16 pll_vcocal1;
	uint16 pll_vcocal11;
	uint16 pll_vcocal12;
	uint16 pll_frct2;
	uint16 pll_frct3;
	uint16 pll_hvldo1;
	uint16 pll_lf4;
	uint16 pll_lf5;
	uint16 pll_lf7;
	uint16 pll_lf2;
	uint16 pll_lf3;
	uint16 spare_cfg1;
	uint16 spare_cfg14;
	uint16 spare_cfg13;
	uint16 txmix2g_cfg5;
	uint16 txmix5g_cfg6;
	uint16 pa5g_cfg4;
	uint16 PHY_BW1a;
	uint16 PHY_BW2;
	uint16 PHY_BW3;
	uint16 PHY_BW4;
	uint16 PHY_BW5;
	uint16 PHY_BW6;
} chan_info_radio20693_pll_t;

/* Rev5,7 & 6,8 */
typedef struct _chan_info_radio20693_rffe {
	uint16 lna2g_tune;
	uint16 lna5g_tune;
	uint16 pa2g_cfg2;
} chan_info_radio20693_rffe_t;

typedef struct _chan_info_radio20693_altclkplan {
	uint8 channel;
	uint8 bw;
	uint8 afeclkdiv;
	uint8 adcclkdiv;
	uint8 sipodiv;
	uint8 dacclkdiv;
	uint8 dacdiv;
} chan_info_radio20693_altclkplan_t;
#if defined(BCMDBG)
#if defined(DBG_PHY_IOV)
extern radio_20xx_dumpregs_t dumpregs_20693_rev5[];

#endif	
#endif	

/* Radio referred values tables */
extern radio_20xx_prefregs_t prefregs_20693_rev5[];
extern radio_20xx_prefregs_t prefregs_20693_rev6[];
extern radio_20xx_prefregs_t prefregs_20693_rev10[];
extern radio_20xx_prefregs_t prefregs_20693_rev13[];
extern radio_20xx_prefregs_t prefregs_20693_rev14[];
extern radio_20xx_prefregs_t prefregs_20693_rev18[];
extern radio_20xx_prefregs_t prefregs_20693_rev32[];

/* Radio tuning values tables */
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev5_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev5_pll[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev6_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev6_pll[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev10_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev10_pll[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev13_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev13_pll[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev14_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev14_pll[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev18_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev18_pll[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_rffe_t chan_tuning_20693_rev32_rffe[NUM_ROWS_CHAN_TUNING];
extern chan_info_radio20693_pll_t chan_tuning_20693_rev32_pll[NUM_ROWS_CHAN_TUNING];

extern const chan_info_radio20693_altclkplan_t altclkpln_radio20693[NUM_ALTCLKPLN_CHANS];


/* For 2g ipa only, to be removed after code addition */
extern uint16 acphy_radiogainqdb_20693_majrev3[128];

#endif	/* _WLC_PHYTBL_20693_H_ */
