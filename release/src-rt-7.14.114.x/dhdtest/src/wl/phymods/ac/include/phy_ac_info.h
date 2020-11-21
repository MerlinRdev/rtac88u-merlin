/*
 * ACPHY Core module internal interface (to other PHY modules).
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

#ifndef _phy_ac_info_h_
#define _phy_ac_info_h_

#include <bcmdevs.h>

#include "phy_ac_ana.h"
#include "phy_ac_antdiv.h"
#include "phy_ac_btcx.h"
#include "phy_ac_cache.h"
#include "phy_ac_calmgr.h"
#include "phy_ac_chanmgr.h"
#include "phy_ac_fcbs.h"
#include "phy_ac_lpc.h"
#include "phy_ac_misc.h"
#include "phy_ac_noise.h"
#include "phy_ac_papdcal.h"
#include "phy_ac_radar.h"
#include "phy_ac_radio.h"
#include "phy_ac_rxgcrs.h"
#include "phy_ac_rssi.h"
#include "phy_ac_rxiqcal.h"
#include "phy_ac_rxspur.h"
#include "phy_ac_samp.h"
#include "phy_ac_tbl.h"
#include "phy_ac_temp.h"
#include "phy_ac_tpc.h"
#include "phy_ac_tssical.h"
#include "phy_ac_txiqlocal.h"
#include "phy_ac_vcocal.h"
#include "phy_ac_dsi.h"


/*
 * ACPHY Core REV info and mapping to (Major/Minor)
 * http://confluence.broadcom.com/display/WLAN/ACPHY+Major+and+Minor+PHY+Revision+Mapping
 *
 * revid  | chip used | (major, minor) revision
 *  0     : 4360A0/A2 (3x3) | (0, 0)
 *  1     : 4360B0    (3x3) | (0, 1)
 *  2     : 4335A0    (1x1 + low-power improvment, no STBC/antdiv) | (1, 0)
 *  3     : 4350A0/B0 (2x2 + low-power 2x2) | (2, 0)
 *  4     : 4345TC    (1x1 + tiny radio)  | (3, 0)
 *  5     : 4335B0    (1x1, almost the same as rev2, txbf NDP stuck fix) | (1, 1)
 *  6     : 4335C0    (1x1, rev5 + bug fixes + stbc) | (1, 2)
 *  7     : 4345A0    (1x1 + tiny radio, rev4 improvment) | (3, 1)
 *  8     : 4350C0    (2x2 + low-power 2x2) | (2, 1)
 *  9     : 43602A0   (3x3 + MDP for lower power, proprietary 256 QAM for 11n) | (5, 0)
 *  10    : 4349TC2A0
 *  11    : 43457A0   (Based on 4345A0 plus support A4WP (Wireless Charging)
 *  12    : 4349A0    (1x1, Reduced Area Tiny-2 Radio plus channel bonding 80+80 supported)
 *  13    : 4345B0/43457B0	(1x1 + tiny radio, phyrev 7 improvements) | (3, 3)
 *  14    : 4350C2    (2x2 + low power) | (2, 4)
 *  15    : 4354A1/43569A0    (2x2 + low-power 2x2) | (2, 3)
 *  16    : 43909A0   (1x1 + tiny radio) | (3, 4)
 *  18    : 43602A1   (3x3 + MDP for lower power, proprietary 256 QAM for 11n) | (5, 1)
 *  20    : 4345C0    (1x1 + tiny radio) | (3, 4)
 *  26    : 4364A0    (3x3 of 4364, derived from 43602)
 *  27    : 4364A0    (1x1 of 4364, derived from 4345C0)
 *  32	  : 4365A0    (4x4) | (32, 0)
 */

/* Major Revs */
#define USE_HW_MINORVERSION(phy_rev) (ACREV_IS(phy_rev, 12))

#define ACMAJORREV_32(phy_rev) \
	(ACREV_IS(phy_rev, 32))

#define ACMAJORREV_GE32(phy_rev) \
	(ACREV_GE(phy_rev, 32))

#define ACMAJORREV_5(phy_rev) \
	(ACREV_IS(phy_rev, 9) || ACREV_IS(phy_rev, 18) || ACREV_IS(phy_rev, 26))

#define ACMAJORREV_4(phy_rev) \
	(ACREV_IS(phy_rev, 12) || ACREV_IS(phy_rev, 24))

#define ACMAJORREV_3(phy_rev) \
	(ACREV_IS(phy_rev, 4) || ACREV_IS(phy_rev, 7) || ACREV_IS(phy_rev, 10) || \
	 ACREV_IS(phy_rev, 11) || ACREV_IS(phy_rev, 13) || ACREV_IS(phy_rev, 16) || \
	 ACREV_IS(phy_rev, 20) || ACREV_IS(phy_rev, 27))

#define ACMAJORREV_2(phy_rev) \
	(ACREV_IS(phy_rev, 3) || ACREV_IS(phy_rev, 8) || ACREV_IS(phy_rev, 15) || \
		ACREV_IS(phy_rev, 17))

#define ACMAJORREV_1(phy_rev) \
	(ACREV_IS(phy_rev, 2) || ACREV_IS(phy_rev, 5) || ACREV_IS(phy_rev, 6))

#define ACMAJORREV_0(phy_rev) \
	(ACREV_IS(phy_rev, 0) || ACREV_IS(phy_rev, 1))

/* Minor Revs */
#ifdef BCMPHYACMINORREV
#define HW_ACMINORREV(pi) (BCMPHYACMINORREV)
#else /* BCMPHYACMINORREV */
/* Read MinorVersion HW reg for chipc with phyrev >= 12 */
#define HW_ACMINORREV(pi) ((pi)->u.pi_acphy->phy_minor_rev)
#endif /* BCMPHYACMINORREV */

#define HW_MINREV_IS(pi, val) ((HW_ACMINORREV(pi)) == (val))

#define SW_ACMINORREV_0(phy_rev) \
	(ACREV_IS(phy_rev, 0) || ACREV_IS(phy_rev, 2) || ACREV_IS(phy_rev, 3) || \
	 ACREV_IS(phy_rev, 4) || ACREV_IS(phy_rev, 9) || ACREV_IS(phy_rev, 10) || \
	 ACREV_IS(phy_rev, 12))

#define SW_ACMINORREV_1(phy_rev) \
	(ACREV_IS(phy_rev, 1) || ACREV_IS(phy_rev, 5) || ACREV_IS(phy_rev, 7) || \
	 ACREV_IS(phy_rev, 8) || ACREV_IS(phy_rev, 18))

#define SW_ACMINORREV_2(phy_rev) \
	(ACREV_IS(phy_rev, 6) || ACREV_IS(phy_rev, 11) || ACREV_IS(phy_rev, 24) || \
	 ACREV_IS(phy_rev, 26))

#define SW_ACMINORREV_3(phy_rev) \
	(ACREV_IS(phy_rev, 13) || ACREV_IS(phy_rev, 15))

#define SW_ACMINORREV_4(phy_rev) \
	(ACREV_IS(phy_rev, 14) || ACREV_IS(phy_rev, 16))

#define SW_ACMINORREV_5(phy_rev) \
	(ACREV_IS(phy_rev, 17) || ACREV_IS(phy_rev, 20))

#define SW_ACMINORREV_6(phy_rev) \
	(ACREV_IS(phy_rev, 27))

/* To get the MinorVersion for chips that do not have this as hw register */
#define GET_SW_ACMINORREV(phy_rev) \
	(SW_ACMINORREV_0(phy_rev) ? 0 : (SW_ACMINORREV_1(phy_rev) ? 1 : \
	(SW_ACMINORREV_2(phy_rev) ? 2 : (SW_ACMINORREV_3(phy_rev) ? 3 : \
	(SW_ACMINORREV_4(phy_rev) ? 4 : 5)))))

#define ACMINORREV_0(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 0) : SW_ACMINORREV_0((pi)->pubpi->phy_rev))

#define ACMINORREV_1(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 1) : SW_ACMINORREV_1((pi)->pubpi->phy_rev))

#define ACMINORREV_2(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 2) : SW_ACMINORREV_2((pi)->pubpi->phy_rev))

#define ACMINORREV_3(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 3) : SW_ACMINORREV_3((pi)->pubpi->phy_rev))

#define ACMINORREV_4(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 4) : SW_ACMINORREV_4((pi)->pubpi->phy_rev))

#define ACMINORREV_5(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 5) : SW_ACMINORREV_5((pi)->pubpi->phy_rev))

#define ACMINORREV_6(pi) \
	(USE_HW_MINORVERSION((pi)->pubpi->phy_rev) ? \
	HW_MINREV_IS(pi, 6) : SW_ACMINORREV_6((pi)->pubpi->phy_rev))
#if defined(ACCONF) && ACCONF
#endif	/*	ACCONF */

#define RSDB_FAMILY(pi)	ACMAJORREV_4((pi)->pubpi->phy_rev)

/* ********************************************************************* */
/* The following definitions shared between calibration, cache modules				*/
/* ********************************************************************* */
#define IQTBL_CACHE_COOKIE_OFFSET	95
#define TXCAL_CACHE_VALID		0xACDC

#define PHY_REG_BANK_CORE1_OFFSET	0x200

/* ACPHY PHY REV mapping to MAJOR/MINOR Revs */
/* Major Revs */

/* Macro's ACREV0 and ACREV3 indicate an old(er) vs a new(er) 2069 radio */
#if ACCONF || ACCONF2
#ifdef BCMCHIPID
#define  ACREV0 (BCMCHIPID == BCM4360_CHIP_ID || BCMCHIPID == BCM4352_CHIP_ID || \
		 BCMCHIPID == BCM43460_CHIP_ID || BCMCHIPID == BCM43526_CHIP_ID || \
		 BCM43602_CHIP(BCMCHIPID) || BCMCHIPID == BCM4364_CHIP_ID)
#define  ACREV3 BCM4350_CHIP(BCMCHIPID)
#define  ACREV32 (BCMCHIPID == BCM4365_CHIP_ID || BCMCHIPID == BCM4366_CHIP_ID)
#define	 ACREV0_SUB (BCMCHIPID == BCM4364_CHIP_ID)
#else
extern int acphychipid;
#define ACREV0 (CHIPID(acphychipid) == BCM4360_CHIP_ID || CHIPID(acphychipid) == BCM4352_CHIP_ID ||\
		CHIPID(acphychipid) == BCM43460_CHIP_ID || \
		CHIPID(acphychipid) == BCM43526_CHIP_ID || \
		BCM43602_CHIP(acphychipid) || CHIPID(acphychipid) == BCM4364_CHIP_ID)
#define ACREV3 BCM4350_CHIP(acphychipid)
#define ACREV32 (CHIPID(acphychipid) == BCM4365_CHIP_ID || CHIPID(acphychipid) == BCM4366_CHIP_ID)
#define	 ACREV0_SUB (CHIPID(acphychipid) == BCM4364_CHIP_ID)
#endif /* BCMCHIPID */
#else
#define ACREV0 0
#define ACREV3 0
#define ACREV32 0
#define ACREV0_SUB 0
#endif /* ACCONF || ACCONF2 */

#define ACPHY_GAIN_VS_TEMP_SLOPE_2G 7   /* units: db/100C */
#define ACPHY_GAIN_VS_TEMP_SLOPE_5G 7   /* units: db/100C */
#define ACPHY_SWCTRL_NVRAM_PARAMS 5
#define ACPHY_RSSIOFFSET_NVRAM_PARAMS 4
#define ACPHY_GAIN_DELTA_2G_PARAMS 2
/* The above variable had only 2 params (gain settings)
 * ELNA ON and ELNA OFF
 * With Olympic, we added 2 more gain settings for 2 Routs
 * Order of the variables is - ELNA_On, ELNA_Off, Rout_1, Rout_2
 * Both the routs should be with ELNA on, as
 * Elna_on - Elna_off offset is added to tr_loss and
 * ELNA_Off value is never used again.
 */
#define ACPHY_GAIN_DELTA_2G_PARAMS_EXT 4
#define ACPHY_GAIN_DELTA_ELNA_ON 0
#define ACPHY_GAIN_DELTA_ELNA_OFF 1
#define ACPHY_GAIN_DELTA_ROUT_1 2
#define ACPHY_GAIN_DELTA_ROUT_2 3

#define ACPHY_GAIN_DELTA_5G_PARAMS_EXT 4
#define ACPHY_GAIN_DELTA_5G_PARAMS 2
#define ACPHY_RCAL_OFFSET  0x10  /* otp offset for Wlan RCAL code */
#define ACPHY_RCAL_VAL_1X1 0xa  /* hard coded rcal_trim val for 1X1 chips */
#define ACPHY_RCAL_VAL_2X2 0x9  /* hard coded rcal_trim val for 2X2 chips */

/* ACPHY tables */
#define ACPHY_TBL_ID_MCS                          1
#define ACPHY_TBL_ID_TXEVMTBL                     2
#define ACPHY_TBL_ID_NVNOISESHAPINGTBL	          3
#define ACPHY_TBL_ID_NVRXEVMSHAPINGTBL	          4
#define ACPHY_TBL_ID_PHASETRACKTBL                5
#define ACPHY_TBL_ID_SQTHRESHOLD                  6
#define ACPHY_TBL_ID_RFSEQ                        7
#define ACPHY_TBL_ID_RFSEQEXT                     8
#define ACPHY_TBL_ID_ANTSWCTRLLUT                 9
#define ACPHY_TBL_ID_FEMCTRLLUT                  10
#define ACPHY_TBL_ID_GAINLIMIT                   11
#define ACPHY_TBL_ID_IQLOCAL                     12
#define ACPHY_TBL_ID_IQLOCAL0                    12
#define ACPHY_TBL_ID_PAPR                        13
#define ACPHY_TBL_ID_SAMPLEPLAY                  14
#define ACPHY_TBL_ID_DUPSTRNTBL                  15
#define ACPHY_TBL_ID_BFMUSERINDEX                16
#define ACPHY_TBL_ID_BFECONFIG                   17
#define ACPHY_TBL_ID_BFEMATRIX                   18
#define ACPHY_TBL_ID_FASTCHSWITCH                19
#define ACPHY_TBL_ID_RFSEQBUNDLE                 20
#define ACPHY_TBL_ID_LNAROUT                     21
#define ACPHY_TBL_ID_MCDSNRVAL                   22
#define ACPHY_TBL_ID_BFRRPT                      23
#define ACPHY_TBL_ID_BFERPT                      24
#define ACPHY_TBL_ID_NVADJTBL                    25
#define ACPHY_TBL_ID_PHASETRACKTBL_1X1           26
#define ACPHY_TBL_ID_SCD_DBMTBL                  27
#define ACPHY_TBL_ID_DCD_DBMTBL                  28
#define ACPHY_TBL_ID_SLNAGAIN                    29
#define ACPHY_TBL_ID_BFECONFIG2X2TBL             30
#define ACPHY_TBL_ID_SLNAGAINBTEXTLNA            31
#define ACPHY_TBL_ID_GAINCTRLBBMULTLUTS          32
#define ACPHY_TBL_ID_ESTPWRSHFTLUTS              33
#define ACPHY_TBL_ID_CHANNELSMOOTHING_1x1        34
#define ACPHY_TBL_ID_SGIADJUST                   35
#define ACPHY_TBL_ID_SLNAGAINTR                  36
#define ACPHY_TBL_ID_SLNAGAINEXTLNATR            37
#define ACPHY_TBL_ID_SLNACLIPSTAGETOTGAIN        38
#define ACPHY_TBL_ID_VASIPREGISTERS              41
#define ACPHY_TBL_ID_IQLOCAL1                    44
#define ACPHY_TBL_ID_ESTPWRLUTS0                 64
#define ACPHY_TBL_ID_IQCOEFFLUTS0                65
#define ACPHY_TBL_ID_LOFTCOEFFLUTS0              66
#define ACPHY_TBL_ID_RFPWRLUTS0                  67
#define ACPHY_TBL_ID_GAIN0                       68
#define ACPHY_TBL_ID_GAINBITS0                   69
#define ACPHY_TBL_ID_RSSICLIPGAIN0               70
#define ACPHY_TBL_ID_EPSILON0                    71
#define ACPHY_TBL_ID_SCALAR0                     72
#define ACPHY_TBL_ID_CORE0CHANESTTBL             73
#define ACPHY_TBL_ID_CORE0CHANSMTH_CHAN          78
#define ACPHY_TBL_ID_CORE0CHANSMTH_FLTR          79
#define ACPHY_TBL_ID_SNOOPAGC                    80
#define ACPHY_TBL_ID_SNOOPPEAK                   81
#define ACPHY_TBL_ID_SNOOPCCKLMS                 82
#define ACPHY_TBL_ID_SNOOPLMS                    83
#define ACPHY_TBL_ID_SNOOPDCCMP                  84
#define ACPHY_TBL_ID_BBPDEPSILON_I0              86
#define ACPHY_TBL_ID_BBPDEPSILON_Q0              87
#define ACPHY_TBL_ID_FDSS_MCSINFOTBL0            88
#define ACPHY_TBL_ID_FDSS_SCALEADJUSTFACTORSTBL0 89
#define ACPHY_TBL_ID_FDSS_BREAKPOINTSTBL0        90
#define ACPHY_TBL_ID_FDSS_SCALEFACTORSTBL0       91
#define ACPHY_TBL_ID_FDSS_SCALEFACTORSDELTATBL0  92
#define ACPHY_TBL_ID_PAPDLUTSELECT0              93
#define ACPHY_TBL_ID_GAINLIMIT0                  94
#define ACPHY_TBL_ID_TXGAINCTRLBBMULTLUTS0       95
#define ACPHY_TBL_ID_ESTPWRLUTS1                 96
#define ACPHY_TBL_ID_IQCOEFFLUTS1                97
#define ACPHY_TBL_ID_LOFTCOEFFLUTS1              98
#define ACPHY_TBL_ID_RFPWRLUTS1                  99
#define ACPHY_TBL_ID_GAIN1                      100
#define ACPHY_TBL_ID_GAINBITS1                  101
#define ACPHY_TBL_ID_RSSICLIPGAIN1              102
#define ACPHY_TBL_ID_EPSILON1                   103
#define ACPHY_TBL_ID_SCALAR1                    104
#define ACPHY_TBL_ID_CORE1CHANESTTBL            105
#define ACPHY_TBL_ID_CORE1CHANSMTH_CHAN         110
#define ACPHY_TBL_ID_CORE1CHANSMTH_FLTR         111
#define ACPHY_TBL_ID_BBPDEPSILON_I1             118
#define ACPHY_TBL_ID_BBPDEPSILON_Q1             119
#define ACPHY_TBL_ID_PAPDLUTSELECT1             125
#define ACPHY_TBL_ID_GAINLIMIT1                 126
#define ACPHY_TBL_ID_TXGAINCTRLBBMULTLUTS1      127

/* Table IDs 128 and 129 have conflicting table names
	for 4349:
		ID 128 -- ACPHY_TBL_ID_GAINCTRLBBMULTLUTS0
		ID 129 -- ACPHY_TBL_ID_ESTPWRSHFTLUTS0
	for other chips:
		ID 128 -- ACPHY_TBL_ID_ESTPWRLUTS2
		ID 129 -- ACPHY_TBL_ID_IQCOEFFLUTS2

	This is handled by appropriately conditioning the code
	with major and minor revs
*/
#define ACPHY_TBL_ID_ESTPWRLUTS2                128
#define ACPHY_TBL_ID_IQCOEFFLUTS2               129

#define ACPHY_TBL_ID_GAINCTRLBBMULTLUTS0 	128
#define ACPHY_TBL_ID_ESTPWRSHFTLUTS0    	129

#define ACPHY_TBL_ID_LOFTCOEFFLUTS2             130
#define ACPHY_TBL_ID_RFPWRLUTS2                 131
#define ACPHY_TBL_ID_GAIN2                      132
#define ACPHY_TBL_ID_GAINBITS2                  133
#define ACPHY_TBL_ID_RSSICLIPGAIN2              134
#define ACPHY_TBL_ID_EPSILON2                   135
#define ACPHY_TBL_ID_SCALAR2                    136
#define ACPHY_TBL_ID_CORE2CHANESTTBL            137
#define ACPHY_TBL_ID_DYNRADIOREGTBL1    	147
#define ACPHY_TBL_ID_ADCSAMPCAP_PATH1   	148
#define ACPHY_TBL_ID_GAINLIMIT2                 158
#define ACPHY_TBL_ID_TXGAINCTRLBBMULTLUTS2      159
#define ACPHY_TBL_ID_GAINCTRLBBMULTLUTS1 	160
#define ACPHY_TBL_ID_ESTPWRSHFTLUTS1    	161

#define ACPHY_NUM_DIG_FILT_COEFFS 		15
#define ACPHY_TBL_LEN_NVNOISESHAPINGTBL         256
#define ACPHY_SPURWAR_NTONES_OFFSET              24 /* Starting offset for spurwar */
#define ACPHY_NV_NTONES_OFFSET                    0 /* Starting offset for nvshp */
#define ACPHY_SPURWAR_NTONES                      8 /* Numver of tones for spurwar */
#define ACPHY_NV_NTONES                          24 /* Numver of tones for nvshp */
/* Number of tones(spurwar+nvshp) to be written */
#define ACPHY_SPURWAR_NV_NTONES                  32

/* channel smoothing 1x1 and core0 chanest data formate */
#define UNPACK_FLOAT_AUTO_SCALE			1

#define CORE0CHANESTTBL_TABLE_WIDTH		32
#define CORE0CHANESTTBL_INTEGER_DATA_SIZE	14
#define CORE0CHANESTTBL_INTEGER_DATA_MASK	((1 << CORE0CHANESTTBL_INTEGER_DATA_SIZE) - 1)
#define CORE0CHANESTTBL_INTEGER_MAXVALUE	((CORE0CHANESTTBL_INTEGER_DATA_MASK+1)>>1)

#define CORE0CHANESTTBL_FLOAT_FORMAT		1
#define CORE0CHANESTTBL_REV0_DATA_SIZE		12
#define CORE0CHANESTTBL_REV0_EXP_SIZE		6
#define CORE0CHANESTTBL_REV2_DATA_SIZE		9
#define CORE0CHANESTTBL_REV2_EXP_SIZE		5

#define CHANNELSMOOTHING_FLOAT_FORMAT		0
#define CHANNELSMOOTHING_FLOAT_DATA_SIZE	11
#define CHANNELSMOOTHING_FLOAT_EXP_SIZE		8
#define CHANNELSMOOTHING_DATA_OFFSET		256

#define RXEVMTBL_DEPTH      256
#define RXNOISESHPTBL_DEPTH 256
#define IQLUT_DEPTH         128
#define LOFTLUT_DEPTH       128
#define ESTPWRSHFTLUT_DEPTH 64

/* hirssi elnabypass */
#define PHY_SW_HIRSSI_UCODE_CAP(pi)	ACMAJORREV_0((pi)->pubpi->phy_rev)
#define PHY_SW_HIRSSI_PERIOD      5    /* 5 second timeout */
#define PHY_SW_HIRSSI_OFF         (-1)
#define PHY_SW_HIRSSI_BYP_THR    (-13)
#define PHY_SW_HIRSSI_RES_THR    (-15)
#define PHY_SW_HIRSSI_W1_BYP_REG  ACPHY_W2W1ClipCnt3(rev)
#define PHY_SW_HIRSSI_W1_BYP_CNT  31
#define PHY_SW_HIRSSI_W1_RES_REG  ACPHY_W2W1ClipCnt1(rev)
#define PHY_SW_HIRSSI_W1_RES_CNT  31

#define ACPHY_TBL_ID_ESTPWRLUTS(core)	\
	(((core == 0) ? ACPHY_TBL_ID_ESTPWRLUTS0 : \
	((core == 1) ? ACPHY_TBL_ID_ESTPWRLUTS1 : ACPHY_TBL_ID_ESTPWRLUTS2)))

#define ACPHY_TBL_ID_CHANEST(core)	\
	(((core == 0) ? ACPHY_TBL_ID_CORE0CHANESTTBL : \
	((core == 1) ? ACPHY_TBL_ID_CORE1CHANESTTBL : ACPHY_TBL_ID_CORE2CHANESTTBL)))

#define ACPHY_LNAROUT_BAND_OFFSET(pi, chanspec) \
	((!(ACMAJORREV_4((pi)->pubpi->phy_rev) && (ACMINORREV_0(pi) || \
	   ACMINORREV_1(pi))) && CHSPEC_IS5G(chanspec)) ? 8 : 0)

#define ACPHY_LNAROUT_CORE_WRT_OFST(phy_rev, core) (24*core)

/* lnaRoutLUT WAR for 4349A2: Read offset is always kept to 0 */
#define ACPHY_LNAROUT_CORE_RD_OFST(pi, core) \
	(ACMAJORREV_4((pi)->pubpi->phy_rev) ? \
	  (ACMINORREV_1(pi) ? 0 : (8*core)) : (24*core))

/* ACPHY RFSeq Commands */
#define ACPHY_RFSEQ_RX2TX		0x0
#define ACPHY_RFSEQ_TX2RX		0x1
#define ACPHY_RFSEQ_RESET2RX		0x2
#define ACPHY_RFSEQ_UPDATEGAINH		0x3
#define ACPHY_RFSEQ_UPDATEGAINL		0x4
#define ACPHY_RFSEQ_UPDATEGAINU		0x5

#define ACPHY_SPINWAIT_RFSEQ_STOP		1000
#define ACPHY_SPINWAIT_RFSEQ_FORCE		200000
#define ACPHY_SPINWAIT_RUNSAMPLE		1000
#define ACPHY_SPINWAIT_TXIQLO			20000
#define ACPHY_SPINWAIT_IQEST			10000

#define ACPHY_NUM_BW                    3
#define ACPHY_NUM_CHANS                 123
#define ACPHY_NUM_BW_2G                 2

#define ACPHY_ClassifierCtrl_classifierSel_MASK 0x7

#define ACPHY_RFSEQEXT_TBL_WIDTH	60
#define ACPHY_GAINLMT_TBL_WIDTH		8
#define ACPHY_GAINDB_TBL_WIDTH		8
#define ACPHY_GAINBITS_TBL_WIDTH	8



/* JIRA(CRDOT11ACPHY-142) - Don't use idx = 0 of lna1/lna2 */
#define ACPHY_MIN_LNA1_LNA2_IDX 1
/* We're ok to use index 0 on tiny phys */
#define ACPHY_MIN_LNA1_LNA2_IDX_TINY 0


/* AvVmid from NVRAM */
#define ACPHY_NUM_BANDS 5
#define ACPHY_AVVMID_NVRAM_PARAMS 2

/* wrapper macros to enable invalid register accesses error messages */
#if defined(BCMDBG_PHYREGS_TRACE)
#define _PHY_REG_READ(pi, reg)			phy_utils_read_phyreg_debug(pi, reg, #reg)
#define _PHY_REG_MOD(pi, reg, mask, val)	phy_utils_mod_phyreg_debug(pi, reg, mask, val, #reg)
#define _READ_RADIO_REG(pi, reg)		phy_utils_read_radioreg_debug(pi, reg, #reg)
#define _MOD_RADIO_REG(pi, reg, mask, val) \
	phy_utils_mod_radioreg_debug(pi, reg, mask, val, #reg)
#define _PHY_REG_WRITE(pi, reg, val)		phy_utils_write_phyreg_debug(pi, reg, val, #reg)
#else
#define _PHY_REG_READ(pi, reg)			phy_utils_read_phyreg(pi, reg)
#define _PHY_REG_MOD(pi, reg, mask, val)	phy_utils_mod_phyreg(pi, reg, mask, val)
#define _READ_RADIO_REG(pi, reg)		phy_utils_read_radioreg(pi, reg)
#define _MOD_RADIO_REG(pi, reg, mask, val)	phy_utils_mod_radioreg(pi, reg, mask, val)
#define _PHY_REG_WRITE(pi, reg, val)		phy_utils_write_phyreg(pi, reg, val)
#endif /* BCMDBG_PHYREGS_TRACE */


#if PHY_CORE_MAX == 1	/* Single PHY core chips */

#define ACPHY_REG_FIELD_MASK(pi, reg, core, field)	\
	ACPHY_Core0##reg##_##field##_MASK(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_SHIFT(pi, reg, core, field)	\
	ACPHY_Core0##reg##_##field##_SHIFT(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_MASKE(pi, reg, core, field)	\
	ACPHY_##reg##0_##field##_MASK(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_SHIFTE(pi, reg, core, field)	\
	ACPHY_##reg##0_##field##_SHIFT(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_MASKM(pi, reg0, reg1, core, field)	\
	ACPHY_##reg0##0_##reg1##_##field##_MASK(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_SHIFTM(pi, reg0, reg1, core, field)	\
	ACPHY_##reg0##0_##reg1##_##field##_SHIFT(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_MASKEE(pi, reg, core, field)	\
	ACPHY_##reg##0_##field##0_MASK(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_SHIFTEE(pi, reg, core, field)	\
	ACPHY_##reg##0_##field##0_SHIFT(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_MASKXE(pi, reg, field, core)	\
	ACPHY_##reg##_##field##0_MASK(pi->pubpi->phy_rev)
#define ACPHY_REG_FIELD_SHIFTXE(pi, reg, field, core)	\
	ACPHY_##reg##_##field##0_SHIFT(pi->pubpi->phy_rev)

#elif PHY_CORE_MAX == 2	/* Dual PHY core chips */

#define ACPHY_REG_FIELD_MASK(pi, reg, core, field) \
	((core == 0) ? ACPHY_Core0##reg##_##field##_MASK(pi->pubpi->phy_rev) : \
	 ACPHY_Core1##reg##_##field##_MASK(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_SHIFT(pi, reg, core, field) \
	((core == 0) ? ACPHY_Core0##reg##_##field##_SHIFT(pi->pubpi->phy_rev) : \
	 ACPHY_Core1##reg##_##field##_SHIFT(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_MASKE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##_MASK(pi->pubpi->phy_rev) : \
	 ACPHY_##reg##1_##field##_MASK(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_SHIFTE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##_SHIFT(pi->pubpi->phy_rev) : \
	 ACPHY_##reg##1_##field##_SHIFT(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_MASKM(pi, reg0, reg1, core, field) \
	((core == 0) ? ACPHY_##reg0##0_##reg1##_##field##_MASK(pi->pubpi->phy_rev) : \
	 ACPHY_##reg0##1_##reg1##_##field##_MASK(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_SHIFTM(pi, reg0, reg1, core, field) \
	((core == 0) ? ACPHY_##reg0##0_##reg1##_##field##_SHIFT(pi->pubpi->phy_rev) : \
	 ACPHY_##reg0##1_##reg1##_##field##_SHIFT(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_MASKEE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##0_MASK(pi->pubpi->phy_rev) : \
	 ACPHY_##reg##1_##field##1_MASK(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_SHIFTEE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##0_SHIFT(pi->pubpi->phy_rev) : \
	 ACPHY_##reg##1_##field##1_SHIFT(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_MASKXE(pi, reg, field, core) \
	((core == 0) ? ACPHY_##reg##_##field##0_MASK(pi->pubpi->phy_rev) : \
	ACPHY_##reg##_##field##1_MASK(pi->pubpi->phy_rev))
#define ACPHY_REG_FIELD_SHIFTXE(pi, reg, field, core) \
	((core == 0) ? ACPHY_##reg##_##field##0_SHIFT(pi->pubpi->phy_rev) : \
	ACPHY_##reg##_##field##1_SHIFT(pi->pubpi->phy_rev))

#else

#define ACPHY_REG_FIELD_MASK(pi, reg, core, field) \
	((core == 0) ? ACPHY_Core0##reg##_##field##_MASK(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_Core1##reg##_##field##_MASK(pi->pubpi->phy_rev) : \
	ACPHY_Core2##reg##_##field##_MASK(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_SHIFT(pi, reg, core, field) \
	((core == 0) ? ACPHY_Core0##reg##_##field##_SHIFT(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_Core1##reg##_##field##_SHIFT(pi->pubpi->phy_rev) : \
	ACPHY_Core2##reg##_##field##_SHIFT(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_MASKE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##_MASK(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg##1_##field##_MASK(pi->pubpi->phy_rev) : \
	ACPHY_##reg##2_##field##_MASK(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_SHIFTE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##_SHIFT(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg##1_##field##_SHIFT(pi->pubpi->phy_rev) : \
	ACPHY_##reg##2_##field##_SHIFT(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_MASKM(pi, reg0, reg1, core, field) \
	((core == 0) ? ACPHY_##reg0##0_##reg1##_##field##_MASK(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg0##1_##reg1##_##field##_MASK(pi->pubpi->phy_rev) : \
	ACPHY_##reg0##2_##reg1##_##field##_MASK(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_SHIFTM(pi, reg0, reg1, core, field) \
	((core == 0) ? ACPHY_##reg0##0_##reg1##_##field##_SHIFT(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg0##1_##reg1##_##field##_SHIFT(pi->pubpi->phy_rev) : \
	ACPHY_##reg0##2_##reg1##_##field##_SHIFT(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_MASKEE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##0_MASK(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg##1_##field##1_MASK(pi->pubpi->phy_rev) : \
	ACPHY_##reg##2_##field##2_MASK(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_SHIFTEE(pi, reg, core, field) \
	((core == 0) ? ACPHY_##reg##0_##field##0_SHIFT(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg##1_##field##1_SHIFT(pi->pubpi->phy_rev) : \
	ACPHY_##reg##2_##field##2_SHIFT(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_MASKXE(pi, reg, field, core) \
	((core == 0) ? ACPHY_##reg##_##field##0_MASK(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg##_##field##1_MASK(pi->pubpi->phy_rev) : \
	ACPHY_##reg##_##field##2_MASK(pi->pubpi->phy_rev)))
#define ACPHY_REG_FIELD_SHIFTXE(pi, reg, field, core) \
	((core == 0) ? ACPHY_##reg##_##field##0_SHIFT(pi->pubpi->phy_rev) : \
	((core == 1) ? ACPHY_##reg##_##field##1_SHIFT(pi->pubpi->phy_rev) : \
	ACPHY_##reg##_##field##2_SHIFT(pi->pubpi->phy_rev)))

#endif	/* PHY_CORE_MAX */

/**
 * Register and register bitfield access macro's. Macro postfixes and associated (example)
 * register bitfields:
 *
 * None: e.g. ACPHY_RfctrlCmd_chip_pu (does not take core# into account)
 * C   : e.g. ACPHY_Core2FastAgcClipCntTh_fastAgcNbClipCntTh
 * CE  : e.g. ACPHY_RfctrlOverrideAfeCfg0_afe_iqdac_pwrup
 * CEE : e.g. ACPHY_EpsilonTableAdjust0_epsilonOffset0
 */

#define ACPHYREGCE(pi, reg, core) ((ACPHY_##reg##0(pi->pubpi->phy_rev)) \
	+ ((core) * PHY_REG_BANK_CORE1_OFFSET))
#define ACPHYREGCM(pi, reg0, reg1, core) ((ACPHY_##reg0##0_##reg1(pi->pubpi->phy_rev)) \
	+ ((core) * PHY_REG_BANK_CORE1_OFFSET))
#define ACPHYREGC(pi, reg, core) ((ACPHY_Core0##reg(pi->pubpi->phy_rev)) \
	+ ((core) * PHY_REG_BANK_CORE1_OFFSET))

/* When the broadcast bit is set in the PHY reg address
 * it writes to the corresponding registers in all the cores
 */
#define ACPHY_REG_BROADCAST ((ACREV0 || ACREV3) ? 0x1000 : 0)

#define WRITE_PHYREG(pi, reg, value)					\
	_PHY_REG_WRITE(pi, ACPHY_##reg(pi->pubpi->phy_rev), (value))

#define WRITE_PHYREGC(pi, reg, core, value)			\
	_PHY_REG_WRITE(pi, ACPHYREGC(pi, reg, core), (value))

#define WRITE_PHYREGCE(pi, reg, core, value)			\
	_PHY_REG_WRITE(pi, ACPHYREGCE(pi, reg, core), (value))

#define MOD_PHYREG(pi, reg, field, value)				\
	_PHY_REG_MOD(pi, ACPHY_##reg(pi->pubpi->phy_rev),		\
		ACPHY_##reg##_##field##_MASK(pi->pubpi->phy_rev),	\
		((value) << ACPHY_##reg##_##field##_##SHIFT(pi->pubpi->phy_rev)))

#define MOD_PHYREGC(pi, reg, core, field, value)			\
	_PHY_REG_MOD(pi,						\
	            ACPHYREGC(pi, reg, core),				\
	            ACPHY_REG_FIELD_MASK(pi, reg, core, field),		\
	            ((value) << ACPHY_REG_FIELD_SHIFT(pi, reg, core, field)))

#define MOD_PHYREGCE(pi, reg, core, field, value)			\
	_PHY_REG_MOD(pi,						\
	            ACPHYREGCE(pi, reg, core),				\
	            ACPHY_REG_FIELD_MASKE(pi, reg, core, field),	\
	            ((value) << ACPHY_REG_FIELD_SHIFTE(pi, reg, core, field)))

#define MOD_PHYREGCM(pi, reg0, reg1, core, field, value)			\
	_PHY_REG_MOD(pi,						\
	            ACPHYREGCM(pi, reg0, reg1, core),				\
	            ACPHY_REG_FIELD_MASKM(pi, reg0, reg1, core, field),	\
	            ((value) << ACPHY_REG_FIELD_SHIFTM(pi, reg0, reg1, core, field)))

#define MOD_PHYREGCEE(pi, reg, core, field, value)			\
	_PHY_REG_MOD(pi,						\
	            ACPHYREGCE(pi, reg, core),				\
	            ACPHY_REG_FIELD_MASKEE(pi, reg, core, field),		\
	            ((value) << ACPHY_REG_FIELD_SHIFTEE(pi, reg, core, field)))

#define MOD_PHYREGCXE(pi, reg, field, core, value)			\
	_PHY_REG_MOD(pi,						\
	            ACPHY_##reg(pi->pubpi->phy_rev),				\
	            ACPHY_REG_FIELD_MASKXE(pi, reg, field, core),		\
	            ((value) << ACPHY_REG_FIELD_SHIFTXE(pi, reg, field, core)))

#define READ_PHYREG(pi, reg) \
	_PHY_REG_READ(pi, ACPHY_##reg(pi->pubpi->phy_rev))

#define READ_PHYREGC(pi, reg, core) \
	_PHY_REG_READ(pi, ACPHYREGC(pi, reg, core))

#define READ_PHYREGCE(pi, reg, core) \
	_PHY_REG_READ(pi, ACPHYREGCE(pi, reg, core))

#define READ_PHYREGFLD(pi, reg, field)				\
	((READ_PHYREG(pi, reg)					\
	 & ACPHY_##reg##_##field##_##MASK(pi->pubpi->phy_rev)) >>	\
	 ACPHY_##reg##_##field##_##SHIFT(pi->pubpi->phy_rev))

#define READ_PHYREGFLDC(pi, reg, core, field) \
	((READ_PHYREGC(pi, reg, core) \
		& ACPHY_REG_FIELD_MASK(pi, reg, core, field)) \
		>> ACPHY_REG_FIELD_SHIFT(pi, reg, core, field))

#define READ_PHYREGFLDCE(pi, reg, core, field) \
	((READ_PHYREGCE(pi, reg, core) \
		& ACPHY_REG_FIELD_MASKE(pi, reg, core, field)) \
		>> ACPHY_REG_FIELD_SHIFTE(pi, reg, core, field))

/* Used in PAPD cal */
#define READ_PHYREGFLDCEE(pi, reg, core, field) \
	((READ_PHYREGCE(pi, reg, core) \
		& ACPHY_REG_FIELD_MASKEE(pi, reg, core, field)) \
		>> ACPHY_REG_FIELD_SHIFTEE(pi, reg, core, field))

/* Required for register fields like RxFeCtrl1.iqswap{0,1,2} */
#define READ_PHYREGFLDCXE(pi, reg, field, core) \
	((READ_PHYREG(pi, reg) \
		& ACPHY_REG_FIELD_MASKXE(pi, reg, field, core)) \
		>> ACPHY_REG_FIELD_SHIFTXE(pi, reg, field, core))


#ifdef WLRSDB
#define ACPHYREG_BCAST(pi, reg, val) \
{\
	if (ACMAJORREV_4(pi->pubpi->phy_rev)) {\
		ASSERT(phy_get_phymode(pi) != PHYMODE_80P80); \
		WRITE_PHYREG_BCAST(pi, reg, val); \
	} else if (ACMAJORREV_3(pi->pubpi->phy_rev)) {\
		_PHY_REG_WRITE(pi, ACPHY_##reg(pi->pubpi->phy_rev), val); \
	} else {\
		_PHY_REG_WRITE(pi, ACPHY_##reg(pi->pubpi->phy_rev) | ACPHY_REG_BROADCAST, val); \
	}\
}
#else
#define ACPHYREG_BCAST(pi, reg, val) \
	_PHY_REG_WRITE(pi, ACPHY_##reg(pi->pubpi->phy_rev) | ACPHY_REG_BROADCAST, val)
#endif /* WLRSDB */

/* BCAST for rsdb family of chips */
#define WRITE_PHYREG_BCAST(pi, reg, val) \
{ \
		int core_idx = 0; \
		FOREACH_CORE(pi, core_idx) { \
			_PHY_REG_WRITE(pi, ACPHY_##reg(pi->pubpi->phy_rev) + \
				((core_idx) * PHY_REG_BANK_CORE1_OFFSET), (val)); \
		} \
}

/* helper macro for table driven writes */
#define ACPHY_REG(pi, regname)	ACPHY_##regname(pi->pubpi->phy_rev)

/* radio-specific macros */
#define RADIO_REG_2069X(pi, id, regnm, core)	RF##core##_##id##_##regnm(pi->pubpi->radiorev)

#define RADIO_REG_20691(pi, regnm, core)	RADIO_REG_2069X(pi, 20691, regnm, 0)

#if PHY_CORE_MAX == 1	/* Single PHY core chips */
#define RADIO_REG_20693(pi, regnm, core)	RADIO_REG_2069X(pi, 20693, regnm, 0)
#elif PHY_CORE_MAX == 2	/* Dual PHY core chips */
#define RADIO_REG_20693(pi, regnm, core)	\
	((core == 0) ? RADIO_REG_2069X(pi, 20693, regnm, 0) : \
	 RADIO_REG_2069X(pi, 20693, regnm, 1))
#else
#define RADIO_REG_20693(pi, regnm, core)	\
	((core == 0) ? RADIO_REG_2069X(pi, 20693, regnm, 0) : \
	((core == 1) ? RADIO_REG_2069X(pi, 20693, regnm, 1) : INVALID_ADDRESS))
#endif	/* PHY_CORE_MAX */

#define RADIO_REG(pi, regnm, core)	\
	((RADIOID_IS((pi)->pubpi->radioid, BCM20691_ID)) \
		? RADIO_REG_20691(pi, regnm, core) : \
	 (RADIOID_IS((pi)->pubpi->radioid, BCM20693_ID)) \
		? RADIO_REG_20693(pi, regnm, core) : INVALID_ADDRESS)

#define MOD_RADIO_REG(pi, regpfx, regnm, fldname, value) \
	_MOD_RADIO_REG(pi, \
	              regpfx##_2069_##regnm, \
	              RF_2069_##regnm##_##fldname##_MASK, \
	              ((value) << RF_2069_##regnm##_##fldname##_SHIFT))

#define MOD_RADIO_REGC(pi, regnm, core, fldname, value) \
	_MOD_RADIO_REG(pi, \
	               RF_2069_##regnm(core), \
	               RF_2069_##regnm##_##fldname##_MASK, \
	               ((value) << RF_2069_##regnm##_##fldname##_SHIFT))

#define READ_RADIO_REG(pi, regpfx, regnm) \
	_READ_RADIO_REG(pi, regpfx##_2069_##regnm)

#define READ_RADIO_REG_20691(pi, regnm, core) \
	_READ_RADIO_REG(pi, RADIO_REG_20691(pi, regnm, core))

#define READ_RADIO_REG_20693(pi, regnm, core) \
	_READ_RADIO_REG(pi, RADIO_REG_20693(pi, regnm, core))

#define READ_RADIO_REG_TINY(pi, regnm, core) \
	((RADIOID_IS((pi)->pubpi->radioid, BCM20691_ID)) ? READ_RADIO_REG_20691(pi, regnm, core) : \
	 (RADIOID_IS((pi)->pubpi->radioid, BCM20693_ID)) ? \
	 READ_RADIO_REG_20693(pi, regnm, core) : 0)

#define READ_RADIO_REGC(pi, regpfx, regnm, core) \
	_READ_RADIO_REG(pi, regpfx##_2069_##regnm(core))

#define READ_RADIO_REGFLD(pi, regpfx, regnm, fldname) \
	((_READ_RADIO_REG(pi, regpfx##_2069_##regnm) & \
	              RF_2069_##regnm##_##fldname##_MASK) \
	              >> RF_2069_##regnm##_##fldname##_SHIFT)

#define READ_RADIO_REGFLDC(pi, regnmcr, regnm, fldname) \
	((_READ_RADIO_REG(pi, regnmcr) & \
	              RF_2069_##regnm##_##fldname##_MASK) \
	              >> RF_2069_##regnm##_##fldname##_SHIFT)

#define READ_RADIO_REGFLD_20691(pi, regnm, core, fldname) \
	((_READ_RADIO_REG(pi, RADIO_REG_20691(pi, regnm, core)) & \
		RF_20691_##regnm##_##fldname##_MASK(pi->pubpi->radiorev)) \
		>> RF_20691_##regnm##_##fldname##_SHIFT(pi->pubpi->radiorev))

#define READ_RADIO_REGFLD_20693(pi, regnm, core, fldname) \
	((_READ_RADIO_REG(pi, RADIO_REG_20693(pi, regnm, core)) & \
		RF_20693_##regnm##_##fldname##_MASK(pi->pubpi->radiorev)) \
		>> RF_20693_##regnm##_##fldname##_SHIFT(pi->pubpi->radiorev))

#define READ_RADIO_REGFLD_TINY(pi, regnm, core, fldname) \
	((RADIOID_IS((pi)->pubpi->radioid, BCM20691_ID)) \
		? READ_RADIO_REGFLD_20691(pi, regnm, core, fldname) : \
	 (RADIOID_IS((pi)->pubpi->radioid, BCM20693_ID)) \
		? READ_RADIO_REGFLD_20693(pi, regnm, core, fldname) : 0)

#define MOD_RADIO_REG_2069X(pi, id, regnm, core, fldname, value) \
	_MOD_RADIO_REG(pi, \
		RADIO_REG_##id(pi, regnm, core), \
		RF_##id##_##regnm##_##fldname##_MASK(pi->pubpi->radiorev), \
		((value) << RF_##id##_##regnm##_##fldname##_SHIFT(pi->pubpi->radiorev)))

#define MOD_RADIO_REG_20691(pi, regnm, core, fldname, value) \
	MOD_RADIO_REG_2069X(pi, 20691, regnm, core, fldname, value)

#define MOD_RADIO_REG_20693(pi, regnm, core, fldname, value) \
	MOD_RADIO_REG_2069X(pi, 20693, regnm, core, fldname, value)

#define MOD_RADIO_REG_TINY(pi, regnm, core, fldname, value) \
	(RADIOID_IS((pi)->pubpi->radioid, BCM20691_ID)) \
		? MOD_RADIO_REG_20691(pi, regnm, core, fldname, value) : \
	(RADIOID_IS((pi)->pubpi->radioid, BCM20693_ID)) \
		? MOD_RADIO_REG_20693(pi, regnm, core, fldname, value) : BCM_REFERENCE(pi)

/* 2069 iPA or ePA radio */
/* Check Minor Radio Revid */
#define ACRADIO_2069_EPA_IS(radio_rev_id) \
	((RADIO2069REV(radio_rev_id) == 2) || (RADIO2069REV(radio_rev_id) == 3) || \
	 (RADIO2069REV(radio_rev_id) == 4) || (RADIO2069REV(radio_rev_id) == 7) || \
	 (RADIO2069REV(radio_rev_id) ==  8) || (RADIO2069REV(radio_rev_id) == 18) || \
	 (RADIO2069REV(radio_rev_id) == 24) || (RADIO2069REV(radio_rev_id) == 26) || \
	 (RADIO2069REV(radio_rev_id) == 34) || (RADIO2069REV(radio_rev_id) == 36))

/* 20691 iPA or ePA radio */
/* Check Minor Radio Revid */
#define ACRADIO_20691_EPA_IS(radio_rev_id) \
	((RADIO20691REV(radio_rev_id) == 13) || (RADIO20691REV(radio_rev_id) == 30))

#define ACPHY_DISABLE_STALL(pi)	MOD_PHYREG(pi, RxFeCtrl1, disable_stalls, 1)
#define ACPHY_ENABLE_STALL(pi, stall_val) MOD_PHYREG(pi, RxFeCtrl1, disable_stalls, stall_val)

/* *********************************************** */
/* The following definitions shared between chanmgr, tpc ... */
/* *********************************************** */
#ifndef D11AC_IOTYPES
/* 80 MHz support is included if D11AC_IOTYPES is defined */
#define CHSPEC_IS80(chspec) (0)
#define WL_CHANSPEC_CTL_SB_LL (0)
#define WL_CHANSPEC_CTL_SB_LU (0)
#define WL_CHANSPEC_CTL_SB_UL (0)
#define WL_CHANSPEC_CTL_SB_UU (0)
#endif /* D11AC_IOTYPES */

/* ********************************************************************* */
/* The following definitions shared between attach, radio, rxiqcal and phytbl ... */
/* ********************************************************************* */

/* On a monolithic driver, receive status is always converted to host byte order early
 * in the receive path (wlc_bmac_recv()). On a split driver, receive status is stays
 * in little endian on the on-chip processor, but is converted to host endian early
 * in the host driver receive path (wlc_recv()). We assume all on-chip processors
 * are little endian, and therefore, these macros do not require ltoh conversion.
 * ltoh conversion would be harmful on a monolithic driver running on a big endian
 * host, where the conversion has already been done. Complain if there is ever
 * a big-endian on-chip processor.
 */
#if defined(WLC_LOW_ONLY) && defined(IL_BIGENDIAN)
#error "Receive status needs endian conversion"
#endif

#ifndef ACPHY_HACK_PWR_STATUS
#define ACPHY_HACK_PWR_STATUS(rxs)	(((rxs)->PhyRxStatus_1 & PRXS1_ACPHY_BIT_HACK) >> 3)
#endif
/* Get Rx power on core 0 */
#ifndef ACPHY_RXPWR_ANT0
#define ACPHY_RXPWR_ANT0(rxs)	(((rxs)->PhyRxStatus_2 & PRXS2_ACPHY_RXPWR_ANT0) >> 8)
#endif
/* Get Rx power on core 1 */
#ifndef ACPHY_RXPWR_ANT1
#define ACPHY_RXPWR_ANT1(rxs)	((rxs)->PhyRxStatus_3 & PRXS3_ACPHY_RXPWR_ANT1)
#endif
/* Get Rx power on core 2 */
#ifndef ACPHY_RXPWR_ANT2
#define ACPHY_RXPWR_ANT2(rxs)	(((rxs)->PhyRxStatus_3 & PRXS3_ACPHY_RXPWR_ANT2) >> 8)
#endif
#ifndef ACPHY_RXPWR_ANT4
#define ACPHY_RXPWR_ANT4(rxs)	((rxs)->PhyRxStatus_4 & PRXS3_ACPHY_RXPWR_ANT4)
#endif

#define ACPHY_VCO_2P5V	1
#define ACPHY_VCO_1P35V	0

/* Macro to enable clock gating changes in different cores */
#define SAMPLE_SYNC_CLK_BIT 	17

#define ACPHY_FEMCTRL_ACTIVE(pi)			\
		((ACMAJORREV_0((pi)->pubpi->phy_rev) ||	\
		  ACMAJORREV_1((pi)->pubpi->phy_rev) ||	\
		  ACMAJORREV_2((pi)->pubpi->phy_rev) ||	\
		  ACMAJORREV_5((pi)->pubpi->phy_rev))	\
			? ((BF3_FEMTBL_FROM_NVRAM((pi)->u.pi_acphy)) == 0) : 0)

/* ************************************************ */
/* Makefile driven Board flags and FemCtrl settings */
/* ************************************************ */

#ifdef FEMCTRL
#define BFCTL(x)			FEMCTRL
#else
#define BFCTL(x)			((x)->sromi->femctrl)
#endif /* FEMCTRL */

#ifdef BOARD_FLAGS
#define BF_ELNA_2G(x)			((BOARD_FLAGS & BFL_SROM11_EXTLNA) != 0)
#define BF_ELNA_5G(x)			((BOARD_FLAGS & BFL_SROM11_EXTLNA_5GHz) != 0)
#define BF_SROM11_BTCOEX(x)		((BOARD_FLAGS & BFL_SROM11_BTCOEX) != 0)
#define BF_SROM11_GAINBOOSTA01(x)	((BOARD_FLAGS & BFL_SROM11_GAINBOOSTA01) != 0)
#else
#define BF_ELNA_2G(x)			((x)->sromi->elna2g_present)
#define BF_ELNA_5G(x)			((x)->sromi->elna5g_present)
#define BF_SROM11_BTCOEX(x)		((x)->sromi->bt_coex)
#define BF_SROM11_GAINBOOSTA01(x)	((x)->sromi->gainboosta01)
#endif /* BOARD_FLAGS */

#ifdef BOARD_FLAGS2
#define BF2_SROM11_APLL_WAR(x)		((BOARD_FLAGS2 & BFL2_SROM11_APLL_WAR) != 0)
#define BF2_2G_SPUR_WAR(x)		((BOARD_FLAGS2 & BFL2_2G_SPUR_WAR) != 0)
#define BF2_DAC_SPUR_IMPROVEMENT(x)	((BOARD_FLAGS2 & BFL2_DAC_SPUR_IMPROVEMENT) != 0)
#else
#define BF2_SROM11_APLL_WAR(x)		((x)->sromi->rfpll_5g)
#define BF2_2G_SPUR_WAR(x)		((x)->sromi->spur_war_enb_2g)
#define BF2_DAC_SPUR_IMPROVEMENT(x)	((x)->sromi->dac_spur_improve)
#endif /* BOARD_FLAGS2 */

#ifdef BOARD_FLAGS3
#define BF3_FEMCTRL_SUB(x)		(BOARD_FLAGS3 & BFL3_FEMCTRL_SUB)
#define BF3_AGC_CFG_2G(x)		((BOARD_FLAGS3 & BFL3_AGC_CFG_2G) != 0)
#define BF3_AGC_CFG_5G(x)		((BOARD_FLAGS3 & BFL3_AGC_CFG_5G) != 0)
#define BF3_5G_SPUR_WAR(x)		((BOARD_FLAGS3 & BFL3_5G_SPUR_WAR) != 0)
#define BF3_RCAL_WAR(x)			((BOARD_FLAGS3 & BFL3_RCAL_WAR) != 0)
#define BF3_RCAL_OTP_VAL_EN(x)		((BOARD_FLAGS3 & BFL3_RCAL_OTP_VAL_EN) != 0)
#define BF3_BBPLL_SPR_MODE_DIS(x)	((BOARD_FLAGS3 & BFL3_BBPLL_SPR_MODE_DIS) != 0)
#define BF3_VLIN_EN_FROM_NVRAM(x)	(0)
#define BF3_PPR_BIT_EXT(x) \
	((BOARD_FLAGS3 & BFL3_PPR_BIT_EXT) >> BFL3_PPR_BIT_EXT_SHIFT)
#define BF3_TXGAINTBLID(x) \
	((BOARD_FLAGS3 & BFL3_TXGAINTBLID) >> BFL3_TXGAINTBLID_SHIFT)
#define BF3_TSSI_DIV_WAR(x) \
	((BOARD_FLAGS3 & BFL3_TSSI_DIV_WAR) >> BFL3_TSSI_DIV_WAR_SHIFT)
#define BF3_2GTXGAINTBL_BLANK(x) \
	((BOARD_FLAGS3 & BFL3_2GTXGAINTBL_BLANK) >> BFL3_2GTXGAINTBL_BLANK_SHIFT)
#define BF3_5GTXGAINTBL_BLANK(x) \
	((BOARD_FLAGS3 & BFL3_5GTXGAINTBL_BLANK) >>	BFL3_5GTXGAINTBL_BLANK_SHIFT)
#define BF3_PHASETRACK_MAX_ALPHABETA(x) \
	((BOARD_FLAGS3 & BFL3_PHASETRACK_MAX_ALPHABETA) >> BFL3_PHASETRACK_MAX_ALPHABETA_SHIFT)
#define BF3_LTECOEX_GAINTBL_EN(x) \
	((BOARD_FLAGS3 & BFL3_LTECOEX_GAINTBL_EN) >> BFL3_LTECOEX_GAINTBL_EN_SHIFT)
#define BF3_ACPHY_LPMODE_2G(x) \
	((BOARD_FLAGS3 & BFL3_ACPHY_LPMODE_2G) >> BFL3_ACPHY_LPMODE_2G_SHIFT)
#define BF3_ACPHY_LPMODE_5G(x) \
	((BOARD_FLAGS3 & BFL3_ACPHY_LPMODE_5G) >> BFL3_ACPHY_LPMODE_5G_SHIFT)
#define BF3_FEMTBL_FROM_NVRAM(x) \
	((BOARD_FLAGS3 & BFL3_FEMTBL_FROM_NVRAM) >> BFL3_FEMTBL_FROM_NVRAM_SHIFT)
#define BF3_AVVMID_FROM_NVRAM(x) \
	((BOARD_FLAGS3 & BFL3_AVVMID_FROM_NVRAM) >> BFL3_AVVMID_FROM_NVRAM_SHIFT)
#define BF3_RSDB_1x1_BOARD(x) \
	((BOARD_FLAGS3 & BFL3_1X1_RSDB_ANT) >> BFL3_1X1_RSDB_ANT_SHIFT)
#else
#define BF3_FEMCTRL_SUB(x)			((x)->sromi->femctrl_sub)
#define BF3_AGC_CFG_2G(x)			((x)->sromi->agc_cfg_2g)
#define BF3_AGC_CFG_5G(x)			((x)->sromi->agc_cfg_5g)
#define BF3_5G_SPUR_WAR(x)			((x)->sromi->spur_war_enb_5g)
#define BF3_RCAL_WAR(x)				((x)->sromi->rcal_war)
#define BF3_RCAL_OTP_VAL_EN(x)			((x)->sromi->rcal_otp_val_en)
#define BF3_PPR_BIT_EXT(x)			((x)->sromi->ppr_bit_ext)
#define BF3_TXGAINTBLID(x)			((x)->sromi->txgaintbl_id)
#define BF3_BBPLL_SPR_MODE_DIS(x)		((x)->sromi->bbpll_spr_modes_dis)
#define BF3_VLIN_EN_FROM_NVRAM(x)		((x)->sromi->vlin_en_from_nvram)
#define BF3_TSSI_DIV_WAR(x)			((x)->sromi->tssi_div_war)
#define BF3_2GTXGAINTBL_BLANK(x)		((x)->sromi->txgaintbl2g_blank)
#define BF3_5GTXGAINTBL_BLANK(x)		((x)->sromi->txgaintbl5g_blank)
#define BF3_PHASETRACK_MAX_ALPHABETA(x)		((x)->sromi->phasetrack_max_alphabeta)
#define BF3_LTECOEX_GAINTBL_EN(x)		((x)->sromi->ltecoex_gaintbl_en)
#define BF3_ACPHY_LPMODE_2G(x)			((x)->sromi->lpmode_2g)
#define BF3_ACPHY_LPMODE_5G(x)			((x)->sromi->lpmode_5g)
#define BF3_FEMTBL_FROM_NVRAM(x)		((x)->sromi->femctrl_from_nvram)
#define BF3_AVVMID_FROM_NVRAM(x)		((x)->sromi->avvmid_from_nvram)
#define BF3_RSDB_1x1_BOARD(x)			((x)->sromi->rsdb_1x1_board)
#endif /* BOARD_FLAGS3 */

#ifdef PHY_IBOARD
#define ACPHY_IBOARD(pi)	TRUE
#else
#define ACPHY_IBOARD(pi)	FALSE
#endif

/* ********************************************************************** */
/* The following definitions used all over PHY. Should be moved to Utils? */
/* ********************************************************************** */

extern void wlc_phy_init_test_acphy(phy_info_t *pi);

/* ************************************************ */
/* The following definitions used by phy_info_acphy */
/* ************************************************ */

typedef struct {
	int8 rssi_corr_normal[PHY_CORE_MAX][ACPHY_NUM_BW_2G];
	int8 rssi_corr_normal_5g[PHY_CORE_MAX][ACPHY_RSSIOFFSET_NVRAM_PARAMS][ACPHY_NUM_BW];
} acphy_nvram_rssioffset_t;

typedef struct {
	uint32 swctrlmap_2g[ACPHY_SWCTRL_NVRAM_PARAMS];
	uint32 swctrlmapext_2g[ACPHY_SWCTRL_NVRAM_PARAMS];
	uint32 swctrlmap_5g[ACPHY_SWCTRL_NVRAM_PARAMS];
	uint32 swctrlmapext_5g[ACPHY_SWCTRL_NVRAM_PARAMS];
	int8 txswctrlmap_2g;
	uint16 txswctrlmap_2g_mask;
	int8 txswctrlmap_5g;
} acphy_nvram_femctrl_t;

typedef struct {
	int8 rssi_corr_normal[PHY_CORE_MAX][ACPHY_NUM_BW_2G];
	int8 rssi_corr_normal_5g[PHY_CORE_MAX][ACPHY_RSSIOFFSET_NVRAM_PARAMS][ACPHY_NUM_BW];
	int8 rssi_corr_gain_delta_2g[PHY_CORE_MAX][ACPHY_GAIN_DELTA_2G_PARAMS][ACPHY_NUM_BW_2G];
	int8 rssi_corr_gain_delta_2g_sub[PHY_CORE_MAX][ACPHY_GAIN_DELTA_2G_PARAMS_EXT]
	[ACPHY_NUM_BW_2G][CH_2G_GROUP_NEW];
	int8 rssi_corr_gain_delta_5g[PHY_CORE_MAX][ACPHY_GAIN_DELTA_5G_PARAMS][ACPHY_NUM_BW]
	[CH_5G_4BAND];
	int8 rssi_corr_gain_delta_5g_sub[PHY_CORE_MAX][ACPHY_GAIN_DELTA_5G_PARAMS_EXT][ACPHY_NUM_BW]
	[CH_5G_4BAND];
	int8 rssi_tr_offset;
} acphy_rssioffset_t;

typedef struct {
	uint16 femctrlmask_2g, femctrlmask_5g;
	acphy_nvram_femctrl_t nvram_femctrl;
	acphy_rssioffset_t  rssioffset;
	uint8 rssi_cal_freq_grp[14];
	acphy_fem_rxgains_t femrx_2g[PHY_CORE_MAX];
	acphy_fem_rxgains_t femrx_5g[PHY_CORE_MAX];
	acphy_fem_rxgains_t femrx_5gm[PHY_CORE_MAX];
	acphy_fem_rxgains_t femrx_5gh[PHY_CORE_MAX];
	int16 rxgain_tempadj_2g;
	int16 rxgain_tempadj_5gl;
	int16 rxgain_tempadj_5gml;
	int16 rxgain_tempadj_5gmu;
	int16 rxgain_tempadj_5gh;
	int32 ed_thresh2g;
	int32 ed_thresh5g;
	int32 ed_thresh_default;
#ifndef FEMCTRL
	uint8 femctrl;
#endif /* FEMCTRL */
#ifndef BOARD_FLAGS
	bool elna2g_present, elna5g_present;
	uint8 gainboosta01;
	uint8 bt_coex;
#endif /* BOARD_FLAGS */
#ifndef BOARD_FLAGS2
	uint8 rfpll_5g;
	uint8 spur_war_enb_2g;
	uint8 dac_spur_improve;
#endif /* BOARD_FLAGS2 */
#ifndef BOARD_FLAGS3
	uint8 femctrl_sub;
	uint8 agc_cfg_2g;
	uint8 agc_cfg_5g;
	uint8 spur_war_enb_5g;
	uint8 rcal_war;
	uint8 txgaintbl_id;
	uint8 ppr_bit_ext;
	uint8 rcal_otp_val_en;
	uint8 bbpll_spr_modes_dis;
	uint8 vlin_en_from_nvram;
	uint8 tssi_div_war;
	uint8 txgaintbl2g_blank;
	uint8 txgaintbl5g_blank;
	uint8 phasetrack_max_alphabeta;
	uint8 ltecoex_gaintbl_en;
	uint8 lpmode_2g;
	uint8 lpmode_5g;
	uint8 femctrl_from_nvram;
	uint8 avvmid_from_nvram;
	uint8 rsdb_1x1_board;
#endif /* BOARD_FLAGS3 */
} acphy_srom_info_t;

/* The PHY Information for AC PHY structure definition */
struct phy_info_acphy
{
	/* ************************************************************************************ */

	phy_info_t		*pi;
	phy_ac_ana_info_t	*anai;
	phy_ac_btcx_info_t	*btcxi;
	phy_ac_cache_info_t	*cachei;
	phy_ac_calmgr_info_t	*calmgri;
	phy_ac_chanmgr_info_t	*chanmgri;
	phy_ac_fcbs_info_t	*fcbsi;
	phy_ac_lpc_info_t	*lpci;
	phy_ac_misc_info_t	*misci;
	phy_ac_noise_info_t	*noisei;
	phy_ac_papdcal_info_t	*papdcali;
	phy_ac_radar_info_t	*radari;
	phy_ac_radio_info_t	*radioi;
	phy_ac_rxgcrs_info_t	*rxgcrsi;
	phy_ac_rssi_info_t	*rssii;
	phy_ac_rxiqcal_info_t	*rxiqcali;
	phy_ac_rxspur_info_t	*rxspuri;
	phy_ac_samp_info_t	*sampi;
	phy_ac_tbl_info_t	*tbli;
	phy_ac_tpc_info_t	*tpci;
	phy_ac_antdiv_info_t	*antdivi;
	phy_ac_temp_info_t	*tempi;
	phy_ac_txiqlocal_info_t	*txiqlocali;
	phy_ac_vcocal_info_t	*vcocali;
	phy_ac_tssical_info_t	*tssicali;
	phy_ac_dsi_info_t	*dsii;

	/* ************************************************************************************ */

	acphy_txcal_radioregs_t *ac_txcal_radioregs_orig;
	acphy_rxcal_phyregs_t	*ac_rxcal_phyregs_orig;
	acphy_desense_values_t	*curr_desense, *zero_desense, *total_desense;
	acphy_hwaci_setup_t	*hwaci_args;
	acphy_desense_values_t	*bt_desense;
	acphy_rx_fdiqi_ctl_t	*fdiqi;
	acphy_srom_info_t	*sromi;
	acphy_aci_params_t	*aci;

	/* ************************************************************************************ */

	chan_info_tx_farrow(*tx_farrow)[ACPHY_NUM_CHANS];
	chan_info_rx_farrow(*rx_farrow)[ACPHY_NUM_CHANS];

	/* ************************************************************************************ */

	int8	txpwrindex[PHY_CORE_MAX];		/* index if hwpwrctrl if OFF */
	int8	phy_noise_all_core[PHY_CORE_MAX];	/* noise power in dB for all cores */
	int8	phy_noise_in_crs_min[PHY_CORE_MAX];	/* noise power in dB for all cores */
	int8	phy_noise_pwr_array[PHY_SIZE_NOISE_ARRAY][PHY_CORE_MAX];
	int8	phy_noise_cache_crsmin[PHY_SIZE_NOISE_CACHE_ARRAY][PHY_CORE_MAX];
	uint8	txpwridx_for_rxiqcal[PHY_CORE_MAX];
	int16	idle_tssi[PHY_CORE_MAX];
	int8	txpwr_offset[PHY_CORE_MAX];		/* qdBm signed offset for tx pwr */
	uint8	txpwrindex_hw_save[PHY_CORE_MAX];	/* txpwr start index for hwpwrctrl */
	uint8	acphy_txpwr_idx_2G[PHY_CORE_MAX]; 	/* txpwr index for 2G band */
	uint8	acphy_txpwr_idx_5G[PHY_CORE_MAX]; 	/* txpwr index for 2G band */
	int16	acphy_papd_epsilon_offset[PHY_CORE_MAX];
	uint8   core_freq_mapping[PHY_CORE_MAX];
	uint16	bb_mult_save[PHY_CORE_MAX];

	/* ************************************************************************************ */

	int8	lna2_complete_gaintbl[12];
	uint8	rxgainctrl_stage_len[ACPHY_MAX_RX_GAIN_STAGES];
	int16 	rxgainctrl_maxout_gains[ACPHY_MAX_RX_GAIN_STAGES];

	/* ************************************************************************************ */

	uint8	dac_mode;
	uint8	bb_mult_save_valid;
	uint16	deaf_count;
	uint16 	saved_bbconf;
	int8 	phy_noise_counter;		/* Dummy variable for noise averaging */
	bool 	trigger_crsmin_cal;
	uint8 	phy_crs_th_from_crs_cal;
	uint8 	phy_debug_crscal_counter;
	uint8 	phy_debug_crscal_channel;
	uint32	phy_caps;			/* Capabilities queried from the registers */
	bool	init;
	bool 	init_done;
	int8 	bt_sw_state;
	uint8	curr_band2g;
	uint8 	band2g_init_done;
	uint8 	band5g_init_done;
	uint8 	prev_subband;
	uint8 	curr_subband;
	int	bbmult_comp;
	uint8	vlin_txidx;
	uint32	curr_bw;
	uint8	curr_spurmode;
	uint8 	fast_adc_en;
	uint16	rccal_gmult;
	uint16 	rccal_gmult_rc;
	uint8	rccal_dacbuf;
	uint16	rccal_adc_gmult;
	bool	crsmincal_enable;		/* Flag for enabling auto crsminpower cal */
	bool 	force_crsmincal;
	uint8	crsmincal_run;
	uint8	srom_2g_pdrange_id;		/* pdet_range_id */
	uint8 	srom_5g_pdrange_id;
	bool	srom_tworangetssi2g;		/* 2 range tssi */
	bool 	srom_tworangetssi5g;
	bool	srom_paprdis;			/* papr disable */
	int8 	srom_papdwar;			/* papd war enable and threshold */
	bool	srom_lowpowerrange2g;		/* low range tssi */
	bool 	srom_lowpowerrange5g;
	uint8	srom_pagc2g;			/* iPa Pa gain override */
	uint8 	srom_pagc2g_ovr;
	uint8 	srom_pagc5g;
	uint8 	srom_pagc5g_ovr;
	uint16	txcal_cache_cookie;
	uint8	radar_cal_active;		/* to mask radar detect during cal's tone-play */
	uint8	srom_tssisleep_en;		/* TSSI sleep enable */
	bool	acphy_papd_kill_switch_en;	/* indicate if lna kill switch is enabled */
	bool	acphy_force_papd_cal;
	uint	acphy_papd_last_cal;		/* time of last papd cal */
	uint32	acphy_papd_recal_counter;
	uint8	papdmode;
	bool	acphy_papdcomp;
	uint8	acphy_papd_skip;		/* skip papd calibration for IPA case */
	int8 	papd_lut0_cal_idx;		/* PAPD index for lut0 */
	int8 	papd_lut1_cal_idx; 		/* PAPD index for lut1 */
	int8 	pacalidx_iovar;			/* force papd cal index */
	bool	limit_desense_on_rssi;
	int32	btc_mode;
	int16	current_temperature;
	bool	poll_adc_WAR;
	uint8	vlinpwr2g_from_nvram;
	uint8 	vlinpwr5g_from_nvram;
	uint16	vlinmask2g_from_nvram;
	uint16 	vlinmask5g_from_nvram;
	uint16	rfldo;
	uint8	acphy_force_lpvco_2G;
	uint8 	acphy_prev_lp_mode;
	uint8 	acphy_lp_status;
	uint8	acphy_lp_mode;			/* To select the low power mode */
	uint8 	acphy_enable_smth;
	uint8 	acphy_smth_dump_mode;
	uint8 	acphy_4335_radio_pd_status;
	uint16	rxRfctrlCoreRxPus0;
	uint16	rxRfctrlOverrideRxPus0;
	uint16 	afeRfctrlCoreAfeCfg10;
	uint16	afeRfctrlCoreAfeCfg20;
	uint16	afeRfctrlOverrideAfeCfg0;
	uint16 	txRfctrlCoreTxPus0;
	uint16	txRfctrlOverrideTxPus0;
	uint16 	radioRfctrlCmd;
	uint16	radioRfctrlCoreGlobalPus;
	uint16	radioRfctrlOverrideGlobalPus;
	uint16	AfePuCtrl;
	bool	ac_rxldpc_override;		/* LDPC override for RX, both band */
	bool	rxiqcal_percore_2g;
	bool	rxiqcal_percore_5g;
	uint16	clip1_th, edcrs_en;
	bool	hirssi_en;			/* hirssi elna bypass */
	uint16	hirssi_period;
	uint16	hirssi_byp_cnt;
	uint16	hirssi_res_cnt;
	int8	hirssi_byp_rssi;
	int8	hirssi_res_rssi;
	bool	hirssi_elnabyp2g_en;
	bool	hirssi_elnabyp5g_en;
	int16	hirssi_timer2g;
	int16	hirssi_timer5g;
	uint8	rssi_coresel;
	uint16	offset_targetpwr;		/* target offset power (in qDb) */
	void	*chan_tuning;
	uint32	chan_tuning_tbl_len;
	int8	pa_mode;			/* Modes: High Efficiency, High Linearity */
	uint16	*fectrl_idx, *fectrl_val;
	uint16 	fectrl_table_len;
	uint16	fectrl_sparse_table_len;
	uint8	fectrl_spl_entry_flag;
	bool	mdgain_trtx_allowed;
	uint16	initGain_codeA;
	uint16	initGain_codeB;
	bool	rxgaincal_rssical;		/* 0 = rxgain error cal and 1 = RSSI error cal */
	bool	rssi_cal_rev;			/* 0 = OLD ad 1 = NEW */
	bool	rud_agc_enable;
	int	last_rssi;
	uint8	ant_swOvr_state_core0;
	uint8 	ant_swOvr_state_core1;
	uint8 	antdiv_rfswctrlpin_a0;
	uint8 	antdiv_rfswctrlpin_a1;
	uint8 	crisscross_priority_core_80p80;
	uint8 	crisscross_priority_core_rsdb;
	uint8 	is_crisscross_actv;
	bool	vco_12GHz;
	uint8	use_fast_adc_20_40;
	uint16	*gaintbl_2g;
	uint16 	*gaintbl_5g;
	bool	hw_aci_status;
	uint16	phy_minor_rev;
	uint8	phyrxchain_old;
	uint8	vasipver;
	uint8	CCTrace;			/* Chanspec Call Trace */
	int	fc; 				/* Center Freq */
	bool	FifoReset; 			/* flag to hold FifoReset val */

	acphy_lp_modes_t	lpmode_2g;
	acphy_lp_modes_t 	lpmode_5g;

	/* ------------------------------------------------------------------------------------ */
	/*                       Pointers to Structs (with cflags)                              */
	/* ------------------------------------------------------------------------------------ */

	/* #ifdef PREASSOC_PWRCTRL */
	phy_pwr_ctrl_s		*pwr_ctrl_save;
	/* /#endif */

	/* ------------------------------------------------------------------------------------ */
	/*                        Variables in acphy info (with cflags)                         */
	/* ------------------------------------------------------------------------------------ */

	/* #ifdef BCMLTECOEX */
	int32	ltecx_mode;
	/* #endif */

	/* #ifdef BCMLTECOEX */
	int8	ltecx_elna_bypass_status;
	/* #endif */

	/* #if defined(WLOLPC) || defined (BCMDBG) || defined(WLTEST) */
	bool	olpc_dbg_mode;
	/* #endif */

	/* #ifndef WLC_DISABLE_ACI */
	uint8	hwaci_max_states_2g, hwaci_max_states_5g;
	/* #endif */ /* !WLC_DISABLE_ACI */

	bool	perratedpd2g;			/* Per Rate DPD */
	bool	perratedpd5g;
	uint16	rxstats[NUM_80211_RATES + 1];

	/* #ifdef WL_PROXDETECT */
	bool 	tof_tx;
	bool	tof_active;
	bool 	tof_setup_done;
	bool	tof_smth_forced;
	uint16	tof_shm_ptr;
	uint16 	tof_rfseq_bundle_offset;
	uint8	tof_core;
	uint8	tof_rx_fdiqcomp_enable;
	uint8 	tof_tx_fdiqcomp_enable;
	int8	tof_smth_enable;
	int8 	tof_smth_dump_mode;
	uint16	tof_ucode_dlys_us[2][5];
	uint8	tof_sc_FS;
	uint8	tof_seq_log2_n;
	uint8	tof_seq_spb_len;
	uint32	*tof_seq_spb;
	/* #endif */

	/* ------------------------------------------------------------------------------------ */
	/*                        Add new Variables after this line                             */
	/* ------------------------------------------------------------------------------------ */
	bool _apapd;
};

/* Chanspec Call Trace - Various call Paths */
#define CALLED_ON_INIT		1
#define CALLED_ON_BW_CHG	2
#define CALLED_ON_BAND_CHG	4

/* Chanspec Call Trace - Which Path ? */
#define CCT_INIT(x)	(mboolisset((x)->CCTrace, CALLED_ON_INIT))
#define CCT_BW_CHG(x)	(mboolisset((x)->CCTrace, CALLED_ON_BW_CHG))
#define CCT_BAND_CHG(x)	(mboolisset((x)->CCTrace, CALLED_ON_BAND_CHG))

/* Chanspec Call Trace - Clear All */
#define CCT_CLR(x)	(mboolclr((x)->CCTrace, CALLED_ON_INIT | CALLED_ON_BW_CHG |\
			CALLED_ON_BAND_CHG))

/*
 * Masks for PA mode selection of linear
 * vs. high efficiency modes.
 */

#define PAMODE_HI_LIN_MASK		0x0000FFFF
#define PAMODE_HI_EFF_MASK		0xFFFF0000

typedef enum {
	PAMODE_HI_LIN = 0,
	PAMODE_HI_EFF
} acphy_swctrl_pa_modes_t;

typedef enum {
	ACPHY_TEMPSENSE_VBG = 0,
	ACPHY_TEMPSENSE_VBE = 1
} acphy_tempsense_cfg_opt_t;

#define CAL_COEFF_READ    0
#define CAL_COEFF_WRITE   1
#define CAL_COEFF_WRITE_BIQ2BYP   2
#define MPHASE_TXCAL_CMDS_PER_PHASE  2 /* number of tx iqlo cal commands per phase in mphase cal */
#define ACPHY_RXCAL_TONEAMP 181


/* PAPD 80MMz WAR for 4349A0: lowering dac clk */
#define PAPD_80MHZ_WAR_4349A0(pi) (ACMAJORREV_4((pi)->pubpi->phy_rev) && ACMINORREV_0(pi))

extern void wlc_tiny_setup_coarse_dcc(phy_info_t *pi);
extern void wlc_dcc_fsm_reset(phy_info_t *pi);
extern void wlc_phy_tiny_rfseq_mode_set(phy_info_t *pi, bool cal_mode);
extern void wlc_phy_mlua_adjust_acphy(phy_info_t *pi, bool btactive);
extern void wlc_phy_tx_farrow_mu_setup(phy_info_t *pi, uint16 MuDelta_l, uint16 MuDelta_u,
	uint16 MuDeltaInit_l, uint16 MuDeltaInit_u);
extern bool wlc_phy_hirssi_elnabypass_shmem_read_clear_acphy(phy_info_t *pi);
extern void wlc_phy_dac_rate_mode_acphy(phy_info_t *pi, uint8 dac_rate_mode);
extern void wlc_phy_radio_tiny_vcocal(phy_info_t *pi);
extern uint16 wlc_phy_get_dac_rate_from_mode(phy_info_t *pi, uint8 dac_rate_mode);
extern void wlc_phy_rxcal_txrx_gainctrl_acphy_tiny(phy_info_t *pi);
extern void wlc_phy_runsamples_acphy(phy_info_t *pi, uint16 num_samps, uint16 loops,
	uint16 wait, uint8 iqmode, uint8 mac_based);
extern void wlc_phy_loadsampletable_acphy(phy_info_t *pi, math_cint32 *tone_buf,
	uint16 num_samps, bool alloc, bool conj);
extern void wlc_phy_aci_updsts_acphy(phy_info_t *pi);
extern uint8 phy_get_rsdbbrd_corenum(phy_info_t *pi, uint8 core);

/* *********************** Remove ************************** */
void wlc_phy_get_initgain_dB_acphy(phy_info_t *pi, int16 *initgain_dB);
#endif /* _phy_ac_info_h_ */
