/*
 * ACPHY module header file
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
 * $Id: wlc_phy_ac.h 523123 2014-12-26 23:50:54Z $
 */

#ifndef _wlc_phy_ac_h_
#define _wlc_phy_ac_h_

#include <typedefs.h>
#include <wlc_phy_int.h>

#include <phy_ac_info.h>

#include "phy_api.h"
#include "phy_ac_ana.h"
#include "phy_ac_radio.h"
#include "phy_ac_tbl.h"
#include "phy_ac_tpc.h"
#include "phy_ac_radar.h"
#include "phy_ac_antdiv.h"
#include "phy_ac_temp.h"
#include "phy_ac_rssi.h"
#include "phy_ac_rxiqcal.h"
#include "phy_ac_txiqlocal.h"
#include "phy_ac_papdcal.h"
#include "phy_ac_vcocal.h"

/* ********************************************************************************** */
/* *** PLEASE DO NOT ADD ANY DEFINITION HERE. DO IT IN THE RELEVANT PHYMOD/MODULE DIR *** */
/* *** THIS FILE WILL BE REMOVED SOON *** */
/* ********************************************************************************** */

/* *********************** Remove ************************** */
void wlc_phy_get_initgain_dB_acphy(phy_info_t *pi, int16 *initgain_dB);
uint8 wlc_phy_rxgainctrl_encode_gain_acphy(phy_info_t *pi, uint8 core,
	int8 gain_dB, bool trloss, bool lna1byp, uint8 *gidx);
void wlc_phy_farrow_setup_tiny(phy_info_t *pi, chanspec_t chanspec);

#endif /* _wlc_phy_ac_h_ */
