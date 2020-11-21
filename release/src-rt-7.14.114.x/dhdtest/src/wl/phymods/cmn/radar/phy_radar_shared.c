/*
 * RadarDetect module implementation (shared by PHY implementations)
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

#include <phy_cfg.h>
#include <typedefs.h>
#include <phy_api.h>
#include <phy_radar.h>
#include "phy_radar_st.h"
#include <phy_radar_utils.h>
#include <phy_radar_shared.h>

#include <phy_utils_reg.h>

#ifndef ALL_NEW_PHY_MOD
#include <wlc_phy_int.h>

#include <wlc_phy_n.h>
#include <wlc_phyreg_n.h>

#include <wlc_phy_ht.h>
#include <wlc_phyreg_ht.h>

#include <wlc_phy_ac.h>
#include <phy_ac_info.h>
#include <wlc_phyreg_ac.h>
#endif /* !ALL_NEW_PHY_MOD */

/* generate an n-th tier pw list */
static void
wlc_phy_radar_generate_tpw(uint16 *inlist, uint16 *outlist, int length, int n)
{
	int i;

	for (i = 0; i < (length - n); i++) {
		outlist[i] = inlist[i + n - 1];
	}
}

static void
wlc_phy_radar_generate_tfm(int16 *inlist, int16 *outlist, int length, int n)
{
	int i;

	for (i = 0; i < (length - n); i++) {
		outlist[i] = inlist[i + n - 1];
	}
}

static void
wlc_phy_radar_read_table_rev_lt7(phy_info_t *pi, phy_radar_st_t *st, int min_pulses)
{
	int i, core;
	uint16 w0, w1, w2;
	int max_fifo_size = 255;

	if (NREV_GE(pi->pubpi->phy_rev, 3))
		max_fifo_size = 510;

	/* True? Maximum table size is 85 entries for .11n */
	/* Format is different from earlier .11a PHYs */
	bzero(st->radar_work.tstart_list_n, sizeof(st->radar_work.tstart_list_n));
	bzero(st->radar_work.width_list_n, sizeof(st->radar_work.width_list_n));
	bzero(st->radar_work.fm_list_n, sizeof(st->radar_work.fm_list_n));

	if (NREV_GE(pi->pubpi->phy_rev, 3)) {
		st->radar_work.nphy_length[0] =
			phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoCtrl) & 0x3ff;
		if (GET_RDR_NANTENNAS(pi) > 1)
			st->radar_work.nphy_length[1] =
				phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoCtrl) & 0x3ff;
	} else {
		st->radar_work.nphy_length[0] =
			phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoCtrl) & 0x1ff;
		if (GET_RDR_NANTENNAS(pi) > 1)
			st->radar_work.nphy_length[1] =
				phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoCtrl) & 0x1ff;
	}
	/* Rev 3: nphy_length is <=510 because words are read/written in multiples of 3 */

	if (st->radar_work.nphy_length[0]  > max_fifo_size) {
		PHY_RADAR(("FIFO LENGTH in ant 0 is greater than max_fifo_size\n"));
		st->radar_work.nphy_length[0]  = 0;
	}

	if (GET_RDR_NANTENNAS(pi) > 1 && st->radar_work.nphy_length[1] > max_fifo_size) {
		PHY_RADAR(("FIFO LENGTH in ant 1 is greater than max_fifo_size\n"));
		st->radar_work.nphy_length[1]  = 0;
	}

#ifdef BCMDBG
	/* enable pulses received at each antenna messages if feature_mask bit-2 is set */
	if (st->rparams.radar_args.feature_mask & 0x4) {
		if (st->radar_work.nphy_length[0] > 5)
			PHY_RADAR(("ant 0:%d\n", st->radar_work.nphy_length[0]));
		if (GET_RDR_NANTENNAS(pi) > 1 && st->radar_work.nphy_length[1] > 5)
			PHY_RADAR(("ant 1:%d\n", st->radar_work.nphy_length[1]));
	}
#endif /* BCMDBG */

	for (core = 0; core < GET_RDR_NANTENNAS(pi); core++) {
		st->radar_work.nphy_length[core] /= 3;	/* 3 words per pulse */

		/* use the last sample for bin5 */
		st->radar_work.tstart_list_bin5[core][0] = st->radar_work.tstart_list_tail[core];
		st->radar_work.width_list_bin5[core][0] = st->radar_work.width_list_tail[core];
		st->radar_work.fm_list_bin5[core][0] = st->radar_work.fm_list_tail[core];
		for (i = 0; i < st->radar_work.nphy_length[core]; i++) {
			if (core == 0) {
				/* Read out FIFO 0 */
				w0 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
				w1 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
				w2 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
			} else {
				/* Read out FIFO 0 */
				w0 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
				w1 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
				w2 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
			}
			/* USE ONLY 255 of 511 FIFO DATA if feature_mask bit-15 set */
			if ((i < 128 && (st->rparams.radar_args.feature_mask & 0x8000)) ||
				((st->rparams.radar_args.feature_mask & 0x8000) == 0)) {
				if (IS20MHZ(pi)) {
					st->radar_work.tstart_list_n[core][i] =
						(uint32) (((w0 << 12) + (w1 & 0x0fff)) << 4);
					st->radar_work.width_list_n[core][i] =
						((w2 & 0x00ff) << 4) + ((w1 >> 12) & 0x000f);
				} else if (IS40MHZ(pi)) {
					st->radar_work.tstart_list_n[core][i] =
						(uint32) ((((w0 << 12) + (w1 & 0x0fff)) << 4) >> 1);
					st->radar_work.width_list_n[core][i] =
						(((w2 & 0x00ff) << 4) + ((w1 >> 12) & 0x000f)) >> 1;
				} else {
					st->radar_work.tstart_list_n[core][i] =
						(uint32) ((((w0 << 12) + (w1 & 0x0fff)) << 4) >> 2);
					st->radar_work.width_list_n[core][i] =
						(((w2 & 0x00ff) << 4) + ((w1 >> 12) & 0x000f)) >> 2;
				}
				st->radar_work.tstart_list_bin5[core][i+1] =
					st->radar_work.tstart_list_n[core][i];
				st->radar_work.width_list_bin5[core][i+1] =
					st->radar_work.width_list_n[core][i];
				st->radar_work.fm_list_n[core][i] = (w2 >> 8) & 0x00ff;
				st->radar_work.fm_list_bin5[core][i+1] =
					st->radar_work.fm_list_n[core][i];
			}
		}
		/* save the last (tail) sample */
		st->radar_work.tstart_list_tail[core] = st->radar_work.tstart_list_bin5[core][i+1];
		st->radar_work.width_list_tail[core] = st->radar_work.width_list_bin5[core][i+1];
		st->radar_work.fm_list_tail[core] = st->radar_work.fm_list_bin5[core][i+1];

		if (st->radar_work.nphy_length[core] > 128 &&
		    (st->rparams.radar_args.feature_mask & 0x8000))
			st->radar_work.nphy_length[core] = 128;
		st->radar_work.nphy_length_bin5[core] = st->radar_work.nphy_length[core] + 1;
	}
}

static void
wlc_phy_radar_read_table(phy_info_t *pi, phy_radar_st_t *st, int min_pulses)
{
	int i;
	uint8 core;
	uint16 w0, w1, w2, w3;
	int max_fifo_size = 512;
	int FMOFFSET;

	/* Maximum table size is 128 entries for .11n rev7 forward */
	bzero(st->radar_work.tstart_list_n, sizeof(st->radar_work.tstart_list_n));
	bzero(st->radar_work.width_list_n, sizeof(st->radar_work.width_list_n));
	bzero(st->radar_work.fm_list_n, sizeof(st->radar_work.fm_list_n));

	if (ISACPHY(pi)) {
		if (GET_RDR_NANTENNAS(pi) > 1) {
			st->radar_work.nphy_length[0] = phy_utils_read_phyreg(pi,
				ACPHY_Antenna0_radarFifoCtrl(pi->pubpi->phy_rev)) & 0x3ff;
			st->radar_work.nphy_length[1] = phy_utils_read_phyreg(pi,
				ACPHY_Antenna1_radarFifoCtrl(pi->pubpi->phy_rev)) & 0x3ff;
		} else {
			st->radar_work.nphy_length[0] = phy_utils_read_phyreg(pi,
				ACPHY_Antenna0_radarFifoCtrl(pi->pubpi->phy_rev)) & 0x3ff;
		}
	} else if (ISHTPHY(pi)) {
		st->radar_work.nphy_length[0] =
			phy_utils_read_phyreg(pi, HTPHY_Antenna0_radarFifoCtrl) & 0x3ff;
		if (GET_RDR_NANTENNAS(pi) > 1)
			st->radar_work.nphy_length[1] =
				phy_utils_read_phyreg(pi, HTPHY_Antenna1_radarFifoCtrl) & 0x3ff;
	} else {
		st->radar_work.nphy_length[0] =
			phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoCtrl) & 0x3ff;
		if (GET_RDR_NANTENNAS(pi) > 1)
			st->radar_work.nphy_length[1] =
				phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoCtrl) & 0x3ff;
	}
	if (ISACPHY(pi) && TONEDETECTION)
	  FMOFFSET = 256;
	else
	  FMOFFSET = 0;

	for (core = 0; core < GET_RDR_NANTENNAS(pi); core++) {
	    if (st->radar_work.nphy_length[core] > max_fifo_size) {
		PHY_RADAR(("FIFO LENGTH in ant %d is greater than max_fifo_size of %d\n",
			core, max_fifo_size));
		st->radar_work.nphy_length[core]  = 0;
	    }
	}

#ifdef BCMDBG
	/* enable pulses received at each antenna messages if feature_mask bit-2 is set */
	if (st->rparams.radar_args.feature_mask & 0x4) {
		if (st->radar_work.nphy_length[0] > 5)
			PHY_RADAR(("ant 0:%d\n", st->radar_work.nphy_length[0]));
		if (GET_RDR_NANTENNAS(pi) > 1 && st->radar_work.nphy_length[1] > 5)
			PHY_RADAR(("ant 1:%d\n", st->radar_work.nphy_length[1]));
	}
#endif /* BCMDBG */

	for (core = 0; core < GET_RDR_NANTENNAS(pi); core++) {
		st->radar_work.nphy_length[core] /= 4;	/* 4 words per pulse */

		/* use the last sample for bin5 */
		st->radar_work.tstart_list_bin5[core][0] = st->radar_work.tstart_list_tail[core];
		st->radar_work.width_list_bin5[core][0] = st->radar_work.width_list_tail[core];
		st->radar_work.fm_list_bin5[core][0] = st->radar_work.fm_list_tail[core];
		for (i = 0; i < st->radar_work.nphy_length[core]; i++) {
			if (core == 0) {
				/* Read out FIFO 0 */
				if (ISACPHY(pi)) {
					w0 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna0_radarFifoData(pi->pubpi->phy_rev));
					w1 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna0_radarFifoData(pi->pubpi->phy_rev));
					w2 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna0_radarFifoData(pi->pubpi->phy_rev));
					w3 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna0_radarFifoData(pi->pubpi->phy_rev));
				} else if (ISHTPHY(pi)) {
					w0 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna0_radarFifoData);
					w1 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna0_radarFifoData);
					w2 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna0_radarFifoData);
					w3 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna0_radarFifoData);
				} else {
					w0 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
					w1 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
					w2 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
					w3 = phy_utils_read_phyreg(pi, NPHY_Antenna0_radarFifoData);
				}
			} else {
				if (ISACPHY(pi)) {
					w0 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna1_radarFifoData(pi->pubpi->phy_rev));
					w1 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna1_radarFifoData(pi->pubpi->phy_rev));
					w2 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna1_radarFifoData(pi->pubpi->phy_rev));
					w3 = phy_utils_read_phyreg(pi,
						ACPHY_Antenna1_radarFifoData(pi->pubpi->phy_rev));
				} else if (ISHTPHY(pi)) {
					w0 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna1_radarFifoData);
					w1 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna1_radarFifoData);
					w2 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna1_radarFifoData);
					w3 = phy_utils_read_phyreg(pi,
					                           HTPHY_Antenna1_radarFifoData);
				} else {
					w0 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
					w1 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
					w2 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
					w3 = phy_utils_read_phyreg(pi, NPHY_Antenna1_radarFifoData);
				}
			}

			if (IS20MHZ(pi)) {
				st->radar_work.tstart_list_n[core][i] = (uint32)((w0 << 16) +
					((w1 & 0x0fff) << 4) + (w3 & 0xf));
				st->radar_work.width_list_n[core][i] = ((w3 & 0x10) << 8) +
					((w2 & 0x00ff) << 4) + ((w1 >> 12) & 0x000f);
			} else if (IS40MHZ(pi)) {
				st->radar_work.tstart_list_n[core][i] = (uint32)(((w0 << 16) +
					((w1 & 0x0fff) << 4) + (w3 & 0xf)) >> 1);
				st->radar_work.width_list_n[core][i] = (((w3 & 0x10) << 8) +
					((w2 & 0x00ff) << 4) + ((w1 >> 12) & 0x000f)) >> 1;
			} else {
				st->radar_work.tstart_list_n[core][i] = (uint32)(((w0 << 16) +
					((w1 & 0x0fff) << 4) + (w3 & 0xf)) >> 2);
				st->radar_work.width_list_n[core][i] = (((w3 & 0x10) << 8) +
					((w2 & 0x00ff) << 4) + ((w1 >> 12) & 0x000f)) >> 2;
			}

			st->radar_work.tstart_list_bin5[core][i + 1] =
				st->radar_work.tstart_list_n[core][i];
			st->radar_work.width_list_bin5[core][i + 1] =
				st->radar_work.width_list_n[core][i];
			st->radar_work.fm_list_n[core][i] =
				((w3 & 0x20) << 3) + ((w2 >> 8) & 0x00ff) - FMOFFSET;
			st->radar_work.fm_list_bin5[core][i + 1] =
				st->radar_work.fm_list_n[core][i];
		}
		/* save the last (tail) sample */
		st->radar_work.tstart_list_tail[core] = st->radar_work.tstart_list_bin5[core][i+1];
		st->radar_work.width_list_tail[core] = st->radar_work.width_list_bin5[core][i+1];

		st->radar_work.fm_list_tail[core] = st->radar_work.fm_list_bin5[core][i+1];
		st->radar_work.nphy_length_bin5[core] = st->radar_work.nphy_length[core] + 1;
	}
}

static bool wlc_phy_radar_detect_uniform_pw_check(int num_pulses, radar_work_t *rt, uint16 j,
	int init_min_detected_pw, int init_max_detected_pw, int *min_detected_pw_p,
	int *max_detected_pw_p, int *pulse_interval_p,  int first_interval,
	int *detected_pulse_index_p, int max_pw_tol, uint32 feature_mask)
{
	int k;

	*min_detected_pw_p = init_min_detected_pw;
	*max_detected_pw_p = init_max_detected_pw;
	for (k = 0; k < num_pulses; k++) {
		if (feature_mask & 0x10) {
		  PHY_RADAR(("RADAR: k=%d, j=%d, j-k=%d, *min_detected_pw_p=%d,"
				" *max_detected_pw_p=%d, *max_detected_pw_p -"
				" *min_detected_pw_p=%d, *pulse_interval_p=%d\n",
				k, j, j-k, *min_detected_pw_p, *max_detected_pw_p,
				*max_detected_pw_p -
				*min_detected_pw_p, *pulse_interval_p));
		}
		if (rt->tiern_pw[0][j-k] <= *min_detected_pw_p)
			*min_detected_pw_p = rt->tiern_pw[0][j-k];
		if (rt->tiern_pw[0][j-k] >= *max_detected_pw_p)
			*max_detected_pw_p = rt->tiern_pw[0][j-k];
	}
	if ((*max_detected_pw_p - *min_detected_pw_p <=
		max_pw_tol) ||
		(*max_detected_pw_p - 2 * *min_detected_pw_p <=
		2 * max_pw_tol)) {
		*pulse_interval_p = first_interval;
		*detected_pulse_index_p = j -
			num_pulses + 1;
		return TRUE;	/* radar detected */
	} else {
		return FALSE;	/* radar NOT detected */
	}
}

static bool
wlc_phy_radar_detect_pri_pw_filter(phy_info_t *pi,
	radar_work_t *rt, radar_params_t *rparams, uint16 j, int tol,
	int first_interval, int pw_2us, int pw5us, int pw15us, int pw20us, int pw30us,
	int i250us, int i500us, int i625us, int i5000us,
	int pw2us, int i833us, int i2500us, int i3333us,
	int i938us, int i3066us, int i3030us, int i1000us)
{
	if ((ABS(rt->tiern_list[0][j] - first_interval) < tol ||
	     ABS(rt->tiern_list[0][j] - (first_interval >> 1)) < tol ||
	     ABS(rt->tiern_list[0][j] - first_interval*2) < tol ||
	     ABS(rt->tiern_list[0][j] - first_interval*3) < tol ||
	     ABS(rt->tiern_list[0][j] - first_interval*4) < tol ||
	     ABS(rt->tiern_list[0][j] - first_interval*5) < tol ||
	     ABS(rt->tiern_list[0][j] - first_interval*6) < tol) &&

	    /* fcc filters */
	    (((rparams->radar_args.feature_mask & 0x800) &&
	      (first_interval >= rparams->radar_args.min_deltat) &&

	      /* type 2 filter */
	      ((rt->tiern_pw[0][j] <= rparams->max_type2_pw &&
	        rt->tiern_pw[0][j+1] <= rparams->max_type2_pw &&
	        first_interval >= rparams->min_type2_intv - tol &&
	        first_interval <= rparams->max_type2_intv + tol) ||

	       /* type 3,4 filter */
	       (rt->tiern_pw[0][j] <= rparams->max_type4_pw &&
	        rt->tiern_pw[0][j+1] <= rparams->max_type4_pw &&
	        (!ISACPHY(pi) ||
	         rt->tiern_pw[0][j] >= rparams->min_type3_pw) &&
	        (!ISACPHY(pi) ||
	         rt->tiern_pw[0][j+1] >= rparams->min_type3_pw) &&
	        first_interval >= rparams->min_type3_4_intv - tol &&
	        first_interval <= rparams->max_type3_4_intv + tol) ||

	       /* fcc weather radar filter */
	       (rt->tiern_pw[0][j] <= pw_2us &&
	        rt->tiern_pw[0][j+1] <= pw_2us &&
	        first_interval >= rparams->max_type3_4_intv - tol &&
	        first_interval <= i3066us + pw20us) ||

			/* korean type 3 filter */
			(rt->tiern_pw[0][j] <= pw_2us &&
			rt->tiern_pw[0][j+1] <= pw_2us &&
			first_interval >= i3030us - tol &&
			first_interval <= i3030us + tol) ||

	       /* type 1 filter */
	       (rt->tiern_pw[0][j] <= rparams->max_type1_pw &&
	        rt->tiern_pw[0][j+1] <= rparams->max_type1_pw &&
	        first_interval >= rparams->type1_intv - tol &&
	        first_interval <= rparams->type1_intv + tol) ||

	       /* type 6, jp4 filter */
	       (rt->tiern_pw[0][j] <= rparams->max_type1_pw &&
	        rt->tiern_pw[0][j+1] <= rparams->max_type1_pw &&
	        first_interval >= rparams->jp4_intv - tol &&
	        first_interval <= rparams->jp4_intv + tol) ||

	       /*  jp2+1 filter */
	       (rt->tiern_pw[0][j] <= rparams->max_type1_pw &&
	        rt->tiern_pw[0][j+1] <= rparams->max_type1_pw &&
	        first_interval >= rparams->jp2_1_intv - tol &&
	        first_interval <= rparams->jp2_1_intv + tol) ||

	       /* type jp1-2, jp2-3 filter */
	       (rt->tiern_pw[0][j] <= rparams->max_jp1_2_pw &&
	        rt->tiern_pw[0][j+1] <= rparams->max_jp1_2_pw &&
	        first_interval >= rparams->jp1_2_intv - tol &&
	        first_interval <= rparams->jp2_3_intv + tol))) ||

			/* etsi filters */
			((rparams->radar_args.feature_mask & 0x1000) &&

			/* type 1, 2, 5, 6 filter */
			((rt->tiern_pw[0][j] <= pw15us &&
			rt->tiern_pw[0][j+1] <= pw15us &&
			first_interval >= i625us - tol*3 &&
			first_interval <= i5000us + tol) ||

			/* packet based staggered types 4, 5 */
			(rt->tiern_pw[0][j] <= pw2us &&
			rt->tiern_pw[0][j+1] <= pw2us &&
			first_interval >= i833us - tol &&
			first_interval <= i2500us + tol) ||

			/* type 3 filter */
			(rt->tiern_pw[0][j] <= pw15us &&
			rt->tiern_pw[0][j+1] <= pw15us &&
			first_interval >= i250us - tol &&
			first_interval <= i500us + tol) ||

			/* type 4 filter */
			(rt->tiern_pw[0][j] <= pw30us &&
			rt->tiern_pw[0][j+1] <= pw30us &&
			first_interval >= i250us - tol &&
			first_interval <= i500us + tol))))) {
				return TRUE;
			} else {
				return FALSE;
			}
}

#ifdef BCMDBG
static void
wlc_phy_radar_output_iv_pw(const char* type, const radar_work_t *rt, const radar_params_t *rparams,
                           int first_interval, int idx)
{
	if (rparams->radar_args.feature_mask & 0x80) {
		PHY_RADAR(("Check %s. IV:%d-%d ", type, first_interval, idx));
		PHY_RADAR(("PW:%d,%d-%d ", rt->tiern_pw[0][idx], rt->tiern_pw[0][idx+1], idx));
	}
}
#define PHY_RADAR_OUTPUT_IV_PW(type, rt, rparams, first_interval, idx) \
	wlc_phy_radar_output_iv_pw(type, rt, rparams, first_interval, idx)
#else /* !BCMDBG */
#define PHY_RADAR_OUTPUT_IV_PW(type, rt, rparams, first_interval, idx)
#endif /* BCMDBG */

static void
wlc_phy_radar_detect_run_epoch(phy_info_t *pi, uint i,
	radar_work_t *rt, radar_params_t *rparams,
	uint32 *epoch_list, int epoch_length,
	int pw_2us, int pw15us, int pw20us, int pw30us,
	int i250us, int i500us, int i625us, int i5000us,
	int pw2us, int i833us, int i2500us, int i3333us,
	int i938us, int i3066us,
	uint *det_type_p, int *pulse_interval_p, int *nconsecq_pulses_p,
	int *detected_pulse_index_p, int *min_detected_pw_p, int *max_detected_pw_p,
	int *fm_min_p, int *fm_max_p)
{
	uint16 j;
	int k;
	bool radar_detected = FALSE;

	/* int ndetected_staggered; */
	uint det_type = 0;
	char stag_det_seq[32];
	int tiern_length[RDR_NTIERS];
	int detected_pulse_index = 0;
	int nconsecq_pulses = 0;
	int nconsecq_eu_t1, nconsecq_eu_t2, nconsecq_eu_t4;
	int nconseq2even, nconseq2odd, nconseq2evenT5, nconseq2evenT6;
	int nconseq3a, nconseq3b, nconseq3c, nconseq3_EU5, nconseq3_EU6;
	int first_interval;
	int first_interval_eu_t1;
	int first_interval_eu_t2;
	int first_interval_eu_t4;
	int tol;
	int pulse_interval = 0;
	int pw5us, pw1us, i518us;
	int i3030us;
	bool fm_pass_eu_t4;
	int fm_dif, fm_tol;
	int eu_type1_cnt, eu_type2_cnt;
	int npulses_eu_t1, npulses_eu_t2, npulses_eu_t4;
	int i600us, i800us, i1000us, i1250us, i1500us, i2000us;
	int low_intv_eu_t1;
	int low_intv_eu_t2;
	int npulses_jp1_2_jp2_3, npulses_jp2_1, npulses_fcc_1, npulses_jp4;
	int nconsecq_jp1_2_jp2_3, nconsecq_jp2_1, nconsecq_fcc_1, nconsecq_jp4;
	int jp1_2_jp2_3_cnt, jp2_1_cnt, fcc_1_cnt, jp4_cnt;
	int first_interval_jp1_2_jp2_3, first_interval_jp2_1,
	  first_interval_fcc_1, first_interval_jp4;
	int pw0p5us;
	/* int pw2p5us; */
	int i333us, i1389us, i1428us;
	int staggered_T1_EU5, staggered_T2_EU5,	staggered_T3_EU5;
	int staggered_T1_EU6, staggered_T2_EU6,	staggered_T3_EU6;

	/* pri limits for fcc tdwr radars */
	pw0p5us = 12 + 12;
	pw1us = 38 + 12;
	/* pw2p5us = 60; */
	pw5us = 110 + 12;
	i333us = 6660;
	i1389us = 27780;
	i1428us = 28560;
	i518us = 10360;
	i3030us = 60606;  /* korean type 3 intv */
	i600us  = 12000;
	i800us  = 16000;
	i1000us = 20000;
	i1250us = 25000;
	i1500us = 30000;
	i2000us = 40000;

	npulses_eu_t1 = rparams->radar_args.npulses_fra & 0xf;
	npulses_eu_t2 = (rparams->radar_args.npulses_fra >> 4) & 0xf;
	npulses_eu_t4 = (rparams->radar_args.npulses_fra >> 8) & 0xf;

	npulses_jp1_2_jp2_3 = (rparams->radar_args.ncontig >> 10) & 0x7;
	npulses_jp2_1 = (rparams->radar_args.ncontig >> 13) & 0x7;
	npulses_jp4 = npulses_jp2_1;
	npulses_fcc_1 =	npulses_jp2_1;
	switch ((rparams->radar_args.npulses_fra >> 12) & 0x3) {
		case 0:
			low_intv_eu_t1 = i1000us;
			break;
		case 1:
			low_intv_eu_t1 = i1250us;
			break;
		case 2:
			low_intv_eu_t1 = i1500us;
			break;
		case 3:
			low_intv_eu_t1 = i2000us;
			break;
		default:
			low_intv_eu_t1 = i1250us;
			break;
	}

	switch ((rparams->radar_args.npulses_fra >> 14) & 0x3) {
		case 0:
			low_intv_eu_t2 = i600us;
			break;
		case 1:
			low_intv_eu_t2 = i800us;
			break;
		case 2:
			low_intv_eu_t2 = i1000us;
			break;
		case 3:
			low_intv_eu_t2 = i2000us;
			break;
		default:
			low_intv_eu_t2 = i1000us;
			break;
	}

	/* initialize staggered radar detection variables */
	snprintf(stag_det_seq, sizeof(stag_det_seq), "%s", "");

	bzero(rt->tiern_list[0], sizeof(rt->tiern_list[0]));
	bzero(rt->tiern_pw[0], sizeof(rt->tiern_pw[0]));
	bzero(rt->tiern_fm[0], sizeof(rt->tiern_fm[0]));

	/*
	 * generate lists
	 */
	wlc_phy_radar_generate_tlist(epoch_list, rt->tiern_list[0], epoch_length, 1);
	wlc_phy_radar_generate_tpw(rt->width_list + rt->epoch_start[i],
		rt->tiern_pw[0], epoch_length, 1);
	wlc_phy_radar_generate_tfm(rt->fm_list + rt->epoch_start[i],
		rt->tiern_fm[0], epoch_length, 1);
	/* ndetected_staggered = 0; */

	tiern_length[0] = epoch_length;
	wlc_phy_radar_filter_list(rt->tiern_list[0], &(tiern_length[0]), rparams->min_tint,
		rparams->max_tint);

	/* Detect contiguous only pulses */
	detected_pulse_index = 0;
	nconsecq_pulses = 0;
	nconsecq_eu_t1 = 0;
	nconsecq_eu_t2 = 0;
	nconsecq_eu_t4 = 0;
	nconseq2even = 0;
	nconseq2odd = 0;
	nconseq2evenT5 = 0;
	nconseq2evenT6 = 0;
	nconseq3a = 0;
	nconseq3b = 0;
	nconseq3c = 0;
	nconseq3_EU5 = 0;
	nconseq3_EU6 = 0;
	eu_type1_cnt = 0;
	eu_type2_cnt = 0;
	jp1_2_jp2_3_cnt = 0;
	jp2_1_cnt = 0;
	fcc_1_cnt = 0;
	jp4_cnt = 0;
	nconsecq_jp1_2_jp2_3 = 0;
	nconsecq_jp2_1 = 0;
	nconsecq_fcc_1 = 0;
	nconsecq_jp4 = 0;

	radar_detected = 0;
	det_type = RADAR_TYPE_NONE;

	tol = rparams->radar_args.quant;
	first_interval = rt->tiern_list[0][0];
	first_interval_eu_t1 = rt->tiern_list[0][0];
	first_interval_eu_t2 = rt->tiern_list[0][0];
	first_interval_eu_t4 = rt->tiern_list[0][0];
	first_interval_jp1_2_jp2_3 = rt->tiern_list[0][0];
	first_interval_jp2_1 = rt->tiern_list[0][0];
	first_interval_fcc_1 = rt->tiern_list[0][0];
	first_interval_jp4 = rt->tiern_list[0][0];

	staggered_T1_EU5 = i3333us;
	staggered_T2_EU5 = i3333us;
	staggered_T3_EU5 = i3333us;
	staggered_T1_EU6 = i2500us;
	staggered_T2_EU6 = i2500us;
	staggered_T3_EU6 = i2500us;

	if (rparams->radar_args.feature_mask & 0x400) {
	  for (j = 0; j < epoch_length-2; j++) {
	    if ((rt->tiern_list[0][j] < i3333us + tol) && (rt->tiern_list[0][j] > i2500us - tol)) {
	      if (staggered_T1_EU5 - rt->tiern_list[0][j] > 32) {
		staggered_T2_EU5 = staggered_T1_EU5;
		staggered_T1_EU5 = rt->tiern_list[0][j];
	      } else if ((rt->tiern_list[0][j] - staggered_T1_EU5) >
			 (20000000 / (20000000 / staggered_T1_EU5 - 20) -  staggered_T1_EU5 - 32) &&
			 (rt->tiern_list[0][j] - staggered_T1_EU5) <
			 (20000000 / (20000000 / staggered_T1_EU5  - 50) - staggered_T1_EU5 + 32) &&
			 (staggered_T2_EU5 - rt->tiern_list[0][j] > 32)) {
		staggered_T3_EU5 = staggered_T2_EU5;
		staggered_T2_EU5 = rt->tiern_list[0][j];
	      } else if ((rt->tiern_list[0][j] - staggered_T2_EU5)  >
			 (20000000 / (20000000 / staggered_T2_EU5 - 20) -  staggered_T2_EU5 - 32) &&
			 (rt->tiern_list[0][j] - staggered_T2_EU5) <
			 (20000000 / (20000000 / staggered_T2_EU5 - 50) - staggered_T2_EU5 + 32) &&
			 (staggered_T3_EU5 - rt->tiern_list[0][j] > 32)) {
		staggered_T3_EU5 = rt->tiern_list[0][j];
	      }
	    }
	  }
	for (j = 0; j < epoch_length - 2; j++) {
	  if ((rt->tiern_list[0][j] < i2500us + tol) && (rt->tiern_list[0][j] > i833us -tol)) {
	    if (staggered_T1_EU6 - rt->tiern_list[0][j] > 32) {
	      staggered_T2_EU6 = staggered_T1_EU6;
	      staggered_T1_EU6 = rt->tiern_list[0][j];
	    } else if ((rt->tiern_list[0][j] - staggered_T1_EU6) >
		       (20000000 / (20000000 / staggered_T1_EU6 - 80) - staggered_T1_EU6 -32) &&
		       (rt->tiern_list[0][j] - staggered_T1_EU6) <
		       (20000000 / MAX(1, (20000000 / staggered_T1_EU6 - 400)) -
		       staggered_T1_EU6 + 32) &&
		       (staggered_T2_EU6 - rt->tiern_list[0][j] > 32)) {
	      staggered_T3_EU6 = staggered_T2_EU6;
	      staggered_T2_EU6 = rt->tiern_list[0][j];
	    } else if ((rt->tiern_list[0][j] - staggered_T2_EU6) >
		       (20000000 / (20000000 / staggered_T2_EU6 - 80) - staggered_T2_EU6 - 32) &&
		       (rt->tiern_list[0][j] - staggered_T2_EU6) <
		       (20000000 / MAX(1, (20000000 / staggered_T2_EU6 - 400)) -
		       staggered_T2_EU6 + 32) &&
		       (staggered_T3_EU6 - rt->tiern_list[0][j] > 32)) {
	      staggered_T3_EU6 = rt->tiern_list[0][j];
	    }
	  }
	}


#ifdef BCMDBG
	if (rparams->radar_args.feature_mask & 0x600) {
	  PHY_RADAR(("Staggered Pulse Intervial EU5 T1=%d, T2=%d, T3 = %d\n",
	    staggered_T1_EU5, staggered_T2_EU5, staggered_T3_EU5));
	  PHY_RADAR(("Staggered Pulse Intervial EU6 T1=%d, T2=%d , T3 = %d\n",
	    staggered_T1_EU6, staggered_T2_EU6, staggered_T3_EU6));
	}
#endif /* BCMDBG */
}

	for (j = 0; j < epoch_length - 2; j++) {

		/* contiguous pulse detection */

		/* fcc, japan, korean, and EU type 3 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
			first_interval, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) &&
			!radar_detected) {
			nconsecq_pulses++;

			if (rparams->radar_args.feature_mask & 0x10) {
			  PHY_RADAR(("nconsecq_pulses=%d\n", nconsecq_pulses));
			}

			/* if number of detected pulses > threshold, */
			/* check detected pulse for pulse width tolerance */
			if (nconsecq_pulses >= rparams->radar_args.npulses-1) {
				if (wlc_phy_radar_detect_uniform_pw_check
					(rparams->radar_args.npulses-1,
					rt, j, rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					det_type = RADAR_TYPE_UNCLASSIFIED;
					radar_detected = 1;
					*nconsecq_pulses_p = nconsecq_pulses;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("Uclassified", rt, rparams,
					                       first_interval, j);
				}
			}
		} else {
			nconsecq_pulses = 0;
			first_interval = rt->tiern_list[0][j];
		}
		/* FCC Type 1 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
		        first_interval_fcc_1, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_fcc_1++;

			/* jp2_1 counting */
			if (rt->tiern_pw[0][j] <= pw1us &&
				first_interval_fcc_1 >= i1428us - tol &&
				first_interval_fcc_1 <= i1428us + tol) {
				++fcc_1_cnt;
			}
			if (rparams->radar_args.feature_mask & 0x10) {
			  PHY_RADAR(("nconsecq_fcc_1=%d, fcc_1_cnt=%d\n",
			     nconsecq_fcc_1, fcc_1_cnt));
			}

			/* special case smaller number of npulses for fcc_1 */
			if ((nconsecq_fcc_1 >= npulses_fcc_1 - 1) &&
				(rparams->radar_args.feature_mask & 0x800) &&
				(fcc_1_cnt >= npulses_fcc_1 - 2)) {
				if (wlc_phy_radar_detect_uniform_pw_check(npulses_fcc_1-1, rt, j,
					rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval_fcc_1, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					radar_detected = 1;
					det_type = RADAR_TYPE_FCC_5;
					*nconsecq_pulses_p = nconsecq_fcc_1;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("FCC-T1", rt, rparams,
					                       first_interval_fcc_1, j);
					npulses_fcc_1 = npulses_fcc_1 - 3;
				}
			}
		} else {
			nconsecq_fcc_1 = 0;
			fcc_1_cnt = 0;
			first_interval_fcc_1 = rt->tiern_list[0][j];
		}


		/* Japan jp1_2, jp2_3 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
			first_interval_jp1_2_jp2_3, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_jp1_2_jp2_3++;

			/* jp1_2, jp2_3 counting */
			if (rt->tiern_pw[0][j] <= rparams->max_jp1_2_pw &&
			first_interval_jp1_2_jp2_3 >= rparams->jp1_2_intv - tol &&
			first_interval_jp1_2_jp2_3 <= rparams->jp2_3_intv + tol) {
				++jp1_2_jp2_3_cnt;
			}
			if (rparams->radar_args.feature_mask & 0x10) {
			  PHY_RADAR(("nconsecq_jp1_2_jp2_3=%d, jp1_2_jp2_3_cnt=%d\n",
			     nconsecq_jp1_2_jp2_3, jp1_2_jp2_3_cnt));
			}

			/* special case smaller number of npulses for jp1_2, jp2_3 */
			if ((nconsecq_jp1_2_jp2_3 >= npulses_jp1_2_jp2_3 - 1) &&
				(rparams->radar_args.feature_mask & 0x800) &&
				(jp1_2_jp2_3_cnt >= npulses_jp1_2_jp2_3 - 2)) {
				if (wlc_phy_radar_detect_uniform_pw_check
					(npulses_jp1_2_jp2_3-1, rt, j,
					rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval_jp1_2_jp2_3, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					radar_detected = 1;
					det_type = RADAR_TYPE_JP1_2_JP2_3;
					*nconsecq_pulses_p = nconsecq_jp1_2_jp2_3;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("JP-T1_2", rt, rparams,
						first_interval_jp1_2_jp2_3, j);
					npulses_jp1_2_jp2_3 = npulses_jp1_2_jp2_3 - 3;
				}
			}
		} else {
			nconsecq_jp1_2_jp2_3 = 0;
			jp1_2_jp2_3_cnt = 0;
			first_interval_jp1_2_jp2_3 = rt->tiern_list[0][j];
		}

		/* Japan jp2_1 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
		        first_interval_jp2_1, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_jp2_1++;

			/* jp2_1 counting */
			if (rt->tiern_pw[0][j] <= pw0p5us &&
				first_interval_jp2_1 >= i1389us - tol &&
				first_interval_jp2_1 <= i1389us + tol) {
				++jp2_1_cnt;
			}
			if (rparams->radar_args.feature_mask & 0x10) {
				PHY_RADAR(("nconsecq_jp2_1=%d, jp2_1_cnt=%d\n",
				   nconsecq_jp2_1, jp2_1_cnt));
			}

			/* special case smaller number of npulses for jp2_1 */
			if ((nconsecq_jp2_1 >= npulses_jp2_1 - 1) &&
				(rparams->radar_args.feature_mask & 0x800) &&
				(jp2_1_cnt >= npulses_jp2_1 - 2)) {
				if (wlc_phy_radar_detect_uniform_pw_check(npulses_jp2_1-1, rt, j,
					rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval_jp2_1, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					radar_detected = 1;
					det_type = RADAR_TYPE_JP2_1;
					*nconsecq_pulses_p = nconsecq_jp2_1;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("JP-T2_1", rt, rparams,
					                       first_interval_jp2_1, j);
					npulses_jp2_1 = npulses_jp2_1 - 3;
				}
			}
		} else {
			nconsecq_jp2_1 = 0;
			jp2_1_cnt = 0;
			first_interval_jp2_1 = rt->tiern_list[0][j];
		}

		/* Japan jp4 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
		        first_interval_jp4, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_jp4++;
			/* jp4 counting */
			if (rt->tiern_pw[0][j] <= pw1us &&
				first_interval_jp4 >= i333us - tol &&
				first_interval_jp4 <= i333us + tol) {
				++jp4_cnt;
			}
			if (rparams->radar_args.feature_mask & 0x10) {
				PHY_RADAR(("nconsecq_jp4=%d, jp4_cnt=%d\n",
				   nconsecq_jp4, jp4_cnt));
			}

			/* special case smaller number of npulses for jp4 */
			if ((nconsecq_jp4 >= npulses_jp4 - 1) &&
				(rparams->radar_args.feature_mask & 0x800) &&
				(jp4_cnt >= npulses_jp4 - 2)) {
				if (wlc_phy_radar_detect_uniform_pw_check(npulses_jp4-1, rt, j,
					rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval_jp4, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					radar_detected = 1;
					det_type = RADAR_TYPE_JP4;
					*nconsecq_pulses_p = nconsecq_jp4;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("JP-T4", rt, rparams,
					                       first_interval_jp4, j);
					npulses_jp4 = npulses_jp4 - 3;
				}
			}
		} else {
			nconsecq_jp4 = 0;
			jp4_cnt = 0;
			first_interval_jp4 = rt->tiern_list[0][j];
		}

		/* EU type 1 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
		        first_interval_eu_t1, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_eu_t1++;

			/* esti type 1 counting */
			if ((rt->tiern_pw[0][j] <= pw5us &&
				rt->tiern_pw[0][j+1] <= pw5us &&
				first_interval_eu_t1 >= low_intv_eu_t1 &&
				first_interval_eu_t1 <= i5000us + tol)) {
				++eu_type1_cnt;
			}

			/* special case smaller number of npulses for EU types 1 */
			if ((nconsecq_eu_t1 >= npulses_eu_t1 - 1) &&
				(rparams->radar_args.feature_mask & 0x1000) &&
				(eu_type1_cnt >= npulses_eu_t1 - 2)) {
				if (wlc_phy_radar_detect_uniform_pw_check(npulses_eu_t1-1, rt, j,
					rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval_eu_t1, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					radar_detected = 1;
					det_type = RADAR_TYPE_ETSI_1;
					*nconsecq_pulses_p = nconsecq_eu_t1;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("EU-T1", rt, rparams,
					                       first_interval_eu_t1, j);
					eu_type1_cnt = npulses_eu_t1 - 3;
				}
			}
		} else {
			nconsecq_eu_t1 = 0;
			eu_type1_cnt = 0;
			first_interval_eu_t1 = rt->tiern_list[0][j];
		}

		/* EU type 2 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
		        first_interval_eu_t2, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_eu_t2++;

			/* esti type 2 counting */
			if ((rt->tiern_pw[0][j] <= pw15us &&
			rt->tiern_pw[0][j+1] <= pw15us &&
			first_interval_eu_t2 >= low_intv_eu_t2 &&
			first_interval_eu_t2 <= i5000us + tol)) {
				++eu_type2_cnt;
			}

			/* special case smaller number of npulses for EU types 2 */
			if ((nconsecq_eu_t2 >= npulses_eu_t2 - 1) &&
				(rparams->radar_args.feature_mask & 0x1000) &&
				(eu_type2_cnt >= npulses_eu_t2 - 2)) {
				if (wlc_phy_radar_detect_uniform_pw_check(npulses_eu_t2-1, rt, j,
					rparams->radar_args.max_pw,
					rparams->radar_args.min_pw,
					min_detected_pw_p, max_detected_pw_p, &pulse_interval,
					first_interval_eu_t2, &detected_pulse_index,
					rparams->radar_args.max_pw_tol,
					rparams->radar_args.feature_mask)) {
					radar_detected = 1;
					det_type = RADAR_TYPE_ETSI_2;
					*nconsecq_pulses_p = nconsecq_eu_t2;
					break;
				} else {
					PHY_RADAR_OUTPUT_IV_PW("EU-T2", rt, rparams,
					                       first_interval_eu_t2, j);
					eu_type2_cnt = npulses_eu_t2 - 3;
				}
			}
		} else {
			nconsecq_eu_t2 = 0;
			eu_type2_cnt = 0;
			first_interval_eu_t2 = rt->tiern_list[0][j];
		}

		/* EU type 4 detection */
		if (wlc_phy_radar_detect_pri_pw_filter(pi,
			rt, rparams, j, tol,
		        first_interval_eu_t4, pw_2us, pw5us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us, i3030us, i1000us) && !radar_detected) {
			nconsecq_eu_t4++;

			/* special cases for EU type 4: smaller npulses and checking fm */
			if ((nconsecq_eu_t4 >= npulses_eu_t4 - 1) &&
				(rparams->radar_args.feature_mask & 0x1000)) {
				*min_detected_pw_p = rparams->radar_args.max_pw;
				*max_detected_pw_p = rparams->radar_args.min_pw;
				fm_pass_eu_t4 = TRUE;
				*fm_min_p = 1030;
				*fm_max_p = 0;
				fm_tol = 0;
				fm_dif = 0;
				for (k = 0; k < npulses_eu_t4; k++) {
					if (rt->tiern_pw[0][j-k+1] <= *min_detected_pw_p)
						*min_detected_pw_p = rt->tiern_pw[0][j-k+1];
					if (rt->tiern_pw[0][j-k+1] >= *max_detected_pw_p)
						*max_detected_pw_p = rt->tiern_pw[0][j-k+1];
					/* check fm against absolute value threshold */
					if ((uint32) rt->tiern_fm[0][j-k+1] <
						((rparams->radar_args.fra_pulse_err >> 8) &
						 0xff) - tol) {
						fm_pass_eu_t4 = FALSE;
						if ((rparams->radar_args.feature_mask & 0x200) == 0)
						{
							break;
						}
					/* verify fm to be the same in a burst */
					} else if (255 - tol > rt->tiern_fm[0][j-k+1] &&
					      (ISACPHY(pi) && TONEDETECTION)) {
					  fm_pass_eu_t4 = FALSE;
							if ((rparams->radar_args.feature_mask &
								0x200) == 0)
							{
								break;
							}
					    }
					else {
					  if (!(ISACPHY(pi) && TONEDETECTION)) {
						if (rt->tiern_fm[0][j-k+1] < *fm_min_p)
							*fm_min_p = rt->tiern_fm[0][j-k+1];
						if (rt->tiern_fm[0][j-k+1] > *fm_max_p)
							*fm_max_p = rt->tiern_fm[0][j-k+1];
						fm_dif = ABS(*fm_max_p - *fm_min_p);
						fm_tol = ((*fm_min_p+*fm_max_p)/2*
							((rparams->radar_args.ncontig >> 6) &
							0xf))/16;
						if (fm_dif > fm_tol) {
							fm_pass_eu_t4 = FALSE;
							if ((rparams->radar_args.feature_mask &
								0x200) == 0)
							{
								break;
							}
						}
					  }
					}
					if (rparams->radar_args.feature_mask & 0x200) {
						PHY_RADAR(("EU-T4: k=%d,fm=%d,fm_th=%d,"
							"*fm_min_p=%d,*fm_max_p=%d,"
							"fm_dif=%d,fm_tol=%d,fm_pass=%d,pw=%d\n",
							k, rt->tiern_fm[0][j-k+1],
							(rparams->radar_args.fra_pulse_err >> 8) &
							0xff, *fm_min_p, *fm_max_p, fm_dif, fm_tol,
							fm_pass_eu_t4, rt->tiern_pw[0][j-k+1]));
					}
				}
				if (((*max_detected_pw_p - *min_detected_pw_p <=
					rparams->radar_args.max_pw_tol) ||
					(*max_detected_pw_p - 2 * *min_detected_pw_p <=
					2 * rparams->radar_args.max_pw_tol)) &&
					fm_pass_eu_t4) {
					pulse_interval = first_interval_eu_t4;
					detected_pulse_index = j -
						npulses_eu_t4 + 1;
					radar_detected = 1;
					det_type = RADAR_TYPE_ETSI_4;
					*nconsecq_pulses_p = nconsecq_eu_t4;
					break;	/* radar detected */
				} else {
					PHY_RADAR_OUTPUT_IV_PW("EU-T4", rt, rparams,
					                       first_interval_eu_t4, j);
				}
			}
		} else {
			nconsecq_eu_t4 = 0;
			first_interval_eu_t4 = rt->tiern_list[0][j];
		}

	if (rparams->radar_args.feature_mask & 0x400) {
	/* staggered 2/3 single filters */
		/* staggered 2 even */
		if (j >= 2 && !radar_detected) {
		  if ((ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-2]) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T1_EU5) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T2_EU5) < 32 ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU5 + staggered_T2_EU5)) < 32 ||
		       ABS(rt->tiern_list[0][j] - (2*staggered_T1_EU5 + staggered_T2_EU5)) < 32 ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU5 + 2*staggered_T2_EU5)) < 32) &&
		      ((rt->tiern_pw[0][j] <= pw1us &&
		      ((rt->tiern_list[0][j] >= i518us -
		      rparams->radar_args.quant &&
		      rt->tiern_list[0][j] <= i938us +
		      rparams->radar_args.quant) ||
		      (rt->tiern_list[0][j] >= i3066us -
		      rparams->radar_args.quant &&
		      rt->tiern_list[0][j] <= i3066us +
		      rparams->radar_args.quant)) &&
		      rparams->radar_args.feature_mask & 0x800) || /* fcc */
		      (rt->tiern_pw[0][j] <= pw2us &&
		      rt->tiern_list[0][j] >= i2500us - tol &&
		      rt->tiern_list[0][j] <= 3*i3333us + tol &&
		      rparams->radar_args.feature_mask &
		      0x1000))) { /* etsi */
		    nconseq2evenT5++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq2evenT5 >=
					rparams->radar_args.npulses_stg2) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq2evenT5; k++) {
						/*
						PHY_RADAR(("EVEN neven=%d, k=%d pw= %d\n",
						nconseq2even, k,rt->tiern_pw[0][j-2*k]));
						*/
						if (rt->tiern_pw[0][j-k+1] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
						if (rt->tiern_pw[0][j-k+1] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p - 2 *
						*min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol) {
						radar_detected = 1;
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG2;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						rparams->radar_args.npulses_stg2 + 1;
						*nconsecq_pulses_p = nconseq2evenT5;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("EVEN RESET neven=%d, j=%d "
					"intv=%d pw= %d\n",
					nconseq2evenT5, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq2evenT5 = 0;
			}
		}
		/* staggered 2/3 single filters */
		/* staggered 2 even */
		if (j >= 2 && !radar_detected) {
		  if ((ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-2]) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T1_EU6) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T2_EU6) < 32 ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU6 + staggered_T2_EU6)) < 32 ||
		       ABS(rt->tiern_list[0][j] - (2 * staggered_T1_EU6 + staggered_T2_EU6)) < 32 ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU6 + 2 * staggered_T2_EU6)) <
		       32) &&
		       ((rt->tiern_pw[0][j] <= pw1us &&
		       ((rt->tiern_list[0][j] >= i518us -
		       rparams->radar_args.quant &&
		       rt->tiern_list[0][j] <= i938us +
		       rparams->radar_args.quant) ||
		       (rt->tiern_list[0][j] >= i3066us -
		       rparams->radar_args.quant &&
		       rt->tiern_list[0][j] <= i3066us +
		       rparams->radar_args.quant)) &&
		       rparams->radar_args.feature_mask & 0x800) || /* fcc */
		       (rt->tiern_pw[0][j] <= pw2us &&
		       rt->tiern_list[0][j] >= i833us - tol &&
		       rt->tiern_list[0][j] <= 3*i2500us + tol &&
		       rparams->radar_args.feature_mask & 0x1000))) { /* etsi */
		    nconseq2evenT6++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq2evenT6 >=
					rparams->radar_args.npulses_stg2) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq2evenT6; k++) {
						/*
						PHY_RADAR(("EVEN neven=%d, k=%d pw= %d\n",
						nconseq2even, k,rt->tiern_pw[0][j-2*k]));
						*/
						if (rt->tiern_pw[0][j-k+1] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
						if (rt->tiern_pw[0][j-k+1] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p - 2 *
						*min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol) {
						radar_detected = 1;
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG2;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						rparams->radar_args.npulses_stg2 + 1;
						*nconsecq_pulses_p = nconseq2evenT6;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("EVEN RESET neven=%d, j=%d "
					"intv=%d pw= %d\n",
					nconseq2evenT6, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq2evenT6 = 0;
			}
		}
		/* staggered 3-a */
		if ((j >= 3 && !radar_detected) &&
			(rparams->radar_args.feature_mask & 0x1000)) {
		  if ((ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-3]) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T1_EU5) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T2_EU5) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T3_EU5) < 32 ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU5 + staggered_T2_EU5)) <
		       tol ||
		       ABS(rt->tiern_list[0][j] - (staggered_T2_EU5 + staggered_T3_EU5)) <
		       tol ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU5 + staggered_T3_EU5)) <
		       tol ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU5 + staggered_T2_EU5 +
		       staggered_T3_EU5)) < tol) &&
		       rt->tiern_pw[0][j] <= pw2us &&
		       (rt->tiern_list[0][j] >= i2500us - tol &&
		       rt->tiern_list[0][j] <= 3*i3333us + tol)) {
				nconseq3_EU5++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq3_EU5 >=
					rparams->radar_args.npulses_stg3) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < rparams->radar_args.npulses_stg3; k++) {
						/*
						PHY_RADAR(("3A n3a=%d, k=%d pw= %d\n",
						nconseq3a, k,rt->tiern_pw[0][j-3*k]));
						*/
						if (rt->tiern_pw[0][j-k+1] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
						if (rt->tiern_pw[0][j-k+1] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p -
						2 * *min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol)
					{
						radar_detected = 1;
						*nconsecq_pulses_p = nconseq3_EU5;
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG3;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						rparams->radar_args.npulses_stg3 + 1;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("3A RESET n3a=%d, j=%d intv=%d "
					"pw= %d\n",
					nconseq3_EU5, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq3_EU5 = 0;
			}
		}

		/* staggered 3-a */
		if ((j >= 3 && !radar_detected) &&
			(rparams->radar_args.feature_mask & 0x1000)) {
		  if ((ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-3]) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T1_EU6) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T2_EU6) < 32 ||
		       ABS(rt->tiern_list[0][j] - staggered_T3_EU6) < 32 ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU6 + staggered_T2_EU6)) < tol ||
		       ABS(rt->tiern_list[0][j] - (staggered_T2_EU6 + staggered_T3_EU6)) < tol ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU6 + staggered_T3_EU6)) < tol ||
		       ABS(rt->tiern_list[0][j] - (staggered_T1_EU6 + staggered_T2_EU6 +
		       staggered_T3_EU6)) < tol) &&
		       rt->tiern_pw[0][j] <= pw2us &&
		       (rt->tiern_list[0][j] >= i833us - tol &&
		       rt->tiern_list[0][j] <= 3*i2500us + tol)) {
				nconseq3_EU6++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq3_EU6 >=
					rparams->radar_args.npulses_stg3) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq3_EU6; k++) {
						/*
						PHY_RADAR(("3A n3a=%d, k=%d pw= %d\n",
						nconseq3a, k,rt->tiern_pw[0][j-3*k]));
						*/
						if (rt->tiern_pw[0][j-k+1] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
						if (rt->tiern_pw[0][j-k+1] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-k+1];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p -
						2 * *min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol)
					{
						radar_detected = 1;
						*nconsecq_pulses_p = nconseq3_EU6;
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG3;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						rparams->radar_args.npulses_stg3 + 1;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("3A RESET n3a=%d, j=%d intv=%d, pw= %d\n",
					nconseq3_EU6, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq3_EU6 = 0;
			}
		}

	} else {
		/* staggered 2/3 single filters */
		/* staggered 2 even */
		if (j >= 2 && j % 2 == 0 && !radar_detected) {
			if (ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-2]) < tol &&
				((rt->tiern_pw[0][j] <= pw1us &&
				((rt->tiern_list[0][j] >= i518us -
				rparams->radar_args.quant &&
				rt->tiern_list[0][j] <= i938us +
				rparams->radar_args.quant) ||
				(rt->tiern_list[0][j] >= i3066us -
				rparams->radar_args.quant &&
				rt->tiern_list[0][j] <= i3066us +
				rparams->radar_args.quant)) &&
				/* fcc */
				rparams->radar_args.feature_mask & 0x800) ||
				(/* rt->tiern_pw[0][j] <= pw2us && */
				rt->tiern_list[0][j] >= i833us - tol &&
				rt->tiern_list[0][j] <= i3333us + tol &&
				rparams->radar_args.feature_mask & 0x1000))) {
				/* etsi */
				nconseq2even++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq2even + nconseq2odd >=
					rparams->radar_args.npulses_stg2-1) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq2even; k++) {
						/*
						PHY_RADAR(("EVEN neven=%d, k=%d pw= %d\n",
						nconseq2even, k,rt->tiern_pw[0][j-2*k]));
						*/
						if (rt->tiern_pw[0][j-2*k] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-2*k];
						if (rt->tiern_pw[0][j-2*k] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-2*k];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p - 2 *
						*min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol) {
						radar_detected = 1;
					}
					if (nconseq2odd > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq2odd; k++) {
							/*
							PHY_RADAR(("EVEN nodd=%d,"
								"k=%d pw= %d\n",
								nconseq2odd, k,
								rt->tiern_pw[0][j-2*k-1]));
							*/
							if (rt->tiern_pw[0][j-2*k-1] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-2*k-1];
							if (rt->tiern_pw[0][j-2*k-1] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-2*k-1];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
						}
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG2;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						2 * rparams->radar_args.npulses_stg2 + 1;
						*nconsecq_pulses_p = nconseq2odd;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("EVEN RESET neven=%d, j=%d "
					"intv=%d pw= %d\n",
					nconseq2even, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq2even = 0;
			}
		}

		/* staggered 2 odd */
		if (j >= 3 && j % 2 == 1 && !radar_detected) {
			if (ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-2]) < tol &&
				((rt->tiern_pw[0][j] <= pw1us &&
				((rt->tiern_list[0][j] >= i518us -
				rparams->radar_args.quant &&
				rt->tiern_list[0][j] <= i938us +
				rparams->radar_args.quant) ||
				(rt->tiern_list[0][j] >= i3066us -
				rparams->radar_args.quant &&
				rt->tiern_list[0][j] <= i3066us +
				rparams->radar_args.quant)) &&
				/* fcc */
				rparams->radar_args.feature_mask & 0x800) ||
				(/* rt->tiern_pw[0][j] <= pw2us && */
				rt->tiern_list[0][j] >= i833us - tol &&
				rt->tiern_list[0][j] <= i3333us + tol &&
				rparams->radar_args.feature_mask & 0x1000))) {
				/* etsi */
				nconseq2odd++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq2even + nconseq2odd >=
					rparams->radar_args.npulses_stg2-1) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq2odd; k++) {
						/*
						 * PHY_RADAR(("ODD nodd=%d, k=%d pw= %d\n",
						 * nconseq2odd, k,rt->tiern_pw[0][j-2*k]));
						 */
						if (rt->tiern_pw[0][j-2*k] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-2*k];
						if (rt->tiern_pw[0][j-2*k] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
								rt->tiern_pw[0][j-2*k];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p -
						2 * *min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol) {
						radar_detected = 1;
					}
					if (nconseq2even > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq2even; k++) {
							/*
							   PHY_RADAR(("ODD neven=%d,"
							   " k=%d pw= %d\n",
							   nconseq2even, k,
							   rt->tiern_pw[0][j-2*k-1]));
							 */
							if (rt->tiern_pw[0][j-2*k-1] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
									rt->tiern_pw[0][j-2*k-1];
							if (rt->tiern_pw[0][j-2*k-1] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
									rt->tiern_pw[0][j-2*k-1];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
							radar_detected = 1;
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG2;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
							2 * rparams->radar_args.npulses_stg2 + 1;
						*nconsecq_pulses_p = nconseq2even;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("ODD RESET nodd=%d, j=%d "
						"intv=%d pw= %d\n",
						nconseq2odd, j, rt->tiern_list[0][j],
						rt->tiern_pw[0][j]));
				nconseq2odd = 0;
			}
		}

		/* staggered 3-a */
		if ((j >= 3 && j % 3 == 0 && !radar_detected) &&
			(rparams->radar_args.feature_mask & 0x1000)) {
			if (ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-3]) < tol &&
				/* rt->tiern_pw[0][j] <= pw2us && */
				(rt->tiern_list[0][j] >= i833us - tol &&
				rt->tiern_list[0][j] <= i3333us + tol)) {
				nconseq3a++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq3a + nconseq3b + nconseq3c >=
					rparams->radar_args.npulses_stg3-1) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq3a; k++) {
						/*
						PHY_RADAR(("3A n3a=%d, k=%d pw= %d\n",
						nconseq3a, k,rt->tiern_pw[0][j-3*k]));
						*/
						if (rt->tiern_pw[0][j-3*k] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-3*k];
						if (rt->tiern_pw[0][j-3*k] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-3*k];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p -
						2 * *min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol)
					{
						radar_detected = 1;
						*nconsecq_pulses_p = nconseq3a;
					}
					if (nconseq3c > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq3c; k++) {
							/*
							PHY_RADAR(("3A n3c=%d, k=%d "
							"pw= %d\n",
							nconseq3c, k,
							rt->tiern_pw[0][j-3*k-1]));
							*/
							if (rt->tiern_pw[0][j-3*k-1] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-3*k-1];
							if (rt->tiern_pw[0][j-3*k-1] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-3*k-1];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
							*nconsecq_pulses_p = nconseq3c;
						}
					}
					if (nconseq3b > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq3b; k++) {
							/*
							PHY_RADAR(("3A n3b=%d, k=%d "
							"pw= %d\n",
							nconseq3b, k,
							rt->tiern_pw[0][j-3*k-2]));
							*/
							if (rt->tiern_pw[0][j-3*k-2] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-3*k-2];
							if (rt->tiern_pw[0][j-3*k-2] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-3*k-2];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
							*nconsecq_pulses_p = nconseq3b;
						}
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG3;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						3 * rparams->radar_args.npulses_stg3 + 1;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("3A RESET n3a=%d, j=%d intv=%d "
					"pw= %d\n",
					nconseq3a, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq3a = 0;
			}
		}

		/* staggered 3-b */
		if ((j >= 4 && j % 3 == 1) && !radar_detected &&
			(rparams->radar_args.feature_mask & 0x1000)) {
			if (ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-3]) < tol &&
				/* rt->tiern_pw[0][j] <= pw2us && */
				(rt->tiern_list[0][j] >= i833us - tol &&
				rt->tiern_list[0][j] <= i3333us + tol)) {
				nconseq3b++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq3a + nconseq3b + nconseq3c >=
					rparams->radar_args.npulses_stg3-1) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq3b; k++) {
						/*
						PHY_RADAR(("3B n3b=%d, k=%d pw= %d\n",
						nconseq3b, k,rt->tiern_pw[0][j-3*k]));
						*/
						if (rt->tiern_pw[0][j-3*k] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-3*k];
						if (rt->tiern_pw[0][j-3*k] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-3*k];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p -
						2 * *min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol)
					{
						radar_detected = 1;
						*nconsecq_pulses_p = nconseq3b;
					}
					if (nconseq3a > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq3a; k++) {
							/*
							PHY_RADAR(("3B n3a=%d, k=%d "
							"pw= %d\n",
							nconseq3a, k,
							rt->tiern_pw[0][j-3*k-1]));
							*/
							if (rt->tiern_pw[0][j-3*k-1] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-3*k-1];
							if (rt->tiern_pw[0][j-3*k-1] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-3*k-1];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
							*nconsecq_pulses_p = nconseq3a;
						}
					}
					if (nconseq3c > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq3c; k++) {
							/*
							PHY_RADAR(("3B n3c=%d, k=%d "
							"pw= %d\n",
							nconseq3c, k,
							rt->tiern_pw[0][j-3*k-2]));
							*/
							if (rt->tiern_pw[0][j-3*k-2] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-3*k-2];
							if (rt->tiern_pw[0][j-3*k-2] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-3*k-2];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
							*nconsecq_pulses_p = nconseq3c;
						}
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG3;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						3 * rparams->radar_args.npulses_stg3 + 1;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("3B RESET n3b=%d, j=%d intv=%d pw= %d\n",
					nconseq3b, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq3b = 0;
			}
		}

		/* staggered 3-c */
		if ((j >= 5 && j % 3 == 2) && !radar_detected &&
			(rparams->radar_args.feature_mask & 0x1000)) {
			if (ABS(rt->tiern_list[0][j] - rt->tiern_list[0][j-3]) < tol &&
				/* rt->tiern_pw[0][j] <= pw2us && */
				(rt->tiern_list[0][j] >= i833us - tol &&
				rt->tiern_list[0][j] <= i3333us + tol)) {
				nconseq3c++;
				/* check detected pulse for pulse width tolerance */
				if (nconseq3a + nconseq3b + nconseq3c >=
					rparams->radar_args.npulses_stg3-1) {
					*min_detected_pw_p = rparams->radar_args.max_pw;
					*max_detected_pw_p = rparams->radar_args.min_pw;
					for (k = 0; k < nconseq3c; k++) {
						/*
						PHY_RADAR(("3C n3c=%d, k=%d pw= %d\n",
						nconseq3c, k,rt->tiern_pw[0][j-3*k]));
						*/
						if (rt->tiern_pw[0][j-3*k] <=
							*min_detected_pw_p)
							*min_detected_pw_p =
							rt->tiern_pw[0][j-3*k];
						if (rt->tiern_pw[0][j-3*k] >=
							*max_detected_pw_p)
							*max_detected_pw_p =
							rt->tiern_pw[0][j-3*k];
					}
					if (*max_detected_pw_p - *min_detected_pw_p <
						rparams->radar_args.max_pw_tol ||
						*max_detected_pw_p -
						2 * *min_detected_pw_p <
						2 * rparams->radar_args.max_pw_tol)
					{
						radar_detected = 1;
						*nconsecq_pulses_p = nconseq3c;
					}
					if (nconseq3b > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq3b; k++) {
							/*
							PHY_RADAR(("3C n3b=%d, k=%d "
							"pw= %d\n",
							nconseq3b, k,
							rt->tiern_pw[0][j-3*k-1]));
							*/
							if (rt->tiern_pw[0][j-3*k-1] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-3*k-1];
							if (rt->tiern_pw[0][j-3*k-1] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-3*k-1];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
							*nconsecq_pulses_p = nconseq3b;
						}
					}
					if (nconseq3a > 0) {
						*min_detected_pw_p =
							rparams->radar_args.max_pw;
						*max_detected_pw_p =
							rparams->radar_args.min_pw;
						for (k = 0; k < nconseq3a; k++) {
							/*
							PHY_RADAR(("3C n3a=%d, k=%d "
							"pw= %d\n",
							nconseq3a, k,
							rt->tiern_pw[0][j-3*k-2]));
							*/
							if (rt->tiern_pw[0][j-3*k-2] <=
								*min_detected_pw_p)
								*min_detected_pw_p =
								rt->tiern_pw[0][j-3*k-2];
							if (rt->tiern_pw[0][j-3*k-2] >=
								*max_detected_pw_p)
								*max_detected_pw_p =
								rt->tiern_pw[0][j-3*k-2];
						}
						if (*max_detected_pw_p -
							*min_detected_pw_p <
							rparams->radar_args.max_pw_tol ||
							*max_detected_pw_p -
							2 * *min_detected_pw_p <
							2 * rparams->radar_args.max_pw_tol)
						{
							radar_detected = 1;
							*nconsecq_pulses_p = nconseq3a;
						}
					}
					if (radar_detected) {
						det_type = RADAR_TYPE_STG3;
						pulse_interval = rt->tiern_list[0][j];
						detected_pulse_index = j -
						3 * rparams->radar_args.npulses_stg3 + 1;
						break;	/* radar detected */
					}
				}
			} else {
				if (rparams->radar_args.feature_mask & 0x20)
					PHY_RADAR(("3C RESET n3c=%d, j=%d intv=%d pw= %d\n",
					nconseq3c, j, rt->tiern_list[0][j],
					rt->tiern_pw[0][j]));
				nconseq3c = 0;
			}
		}
	}
	}  /* for (j = 0; j < epoch_length - 2; j++) */

	if (radar_detected) {
		*det_type_p = det_type;
		*pulse_interval_p = pulse_interval;
		*detected_pulse_index_p = detected_pulse_index;
	} else {
		*det_type_p = RADAR_TYPE_NONE;
		*pulse_interval_p = 0;
		*nconsecq_pulses_p = 0;
		*detected_pulse_index_p = 0;
	}
}

int
phy_radar_run_nphy(phy_info_t *pi)
{
	phy_radar_info_t *ri = pi->radari;
	phy_radar_st_t *st = phy_radar_get_st(ri);
/* NOTE: */
/* PLEASE UPDATE THE DFS_SW_VERSION #DEFINE'S IN FILE WLC_PHY_INT.H */
/* EACH TIME ANY DFS FUNCTION IS MODIFIED EXCEPT RADAR THRESHOLD CHANGES */
	uint16 i;
	uint16 j;
	int k;
	int wr_ptr;
	uint8 ant;
	uint16 mlength;
	uint32 *epoch_list;
	int epoch_length = 0;
	int epoch_detected;
	int pulse_interval;
	uint32 tstart;
	uint16 width;
	int16 fm;
	bool valid_lp;
	bool filter_pw = TRUE;
	bool filter_fm = TRUE;
	int32 deltat;
	radar_work_t *rt;
	radar_params_t *rparams;
	uint det_type;
	int skip_type = 0;
	int min_detected_pw;
	int max_detected_pw;
	int pw_2us, pw15us, pw20us, pw30us;
	int i250us, i500us, i625us, i5000us;
	int pw2us, i833us, i2500us, i3333us;
	int i938us, i3066us;
	/* int i633us, i658us; */
	int nconsecq_pulses = 0;
	int detected_pulse_index = 0;
	uint32 max_lp_buffer_span;
	int32 deltat2 = 0;
	int32 salvate_intv = 0;
	uint32 tmp_uint32;
	int pw_dif, pw_tol, fm_dif, fm_tol;
	int fm_min = 0;
	int fm_max = 0;
	int FMCOMBINEOFFSET;

	rt = &st->radar_work;
	rparams = &st->rparams;

	pw_2us  = 40+16;
	pw15us = 300+45;
	pw20us = 400+60;
	pw30us = 600+90;
	i250us = 5000;    /* 4000 Hz */
	i500us = 10000;   /* 2000Hz */
	i625us = 12500;   /* 1600 Hz */
	/* i633us = 12658; */   /* 1580 Hz */
	/* i658us = 13158; */   /* 1520 Hz */
	i938us = 18760;   /* 1066.1 Hz */
	i3066us = 61312;  /* 326.2 Hz */
	i5000us = 100000; /* 200 Hz */
	/* staggered */
	pw2us  = 40+ 20;
	i833us = 16667;   /* 1200 Hz */
	i2500us = 50000;  /* 400 Hz */
	i3333us = 66667;  /* 300 Hz */
	max_lp_buffer_span = MAX_LP_BUFFER_SPAN_20MHZ;  /* 12sec */

	/* clear LP buffer if requested, and print LP buffer count history */
	if (pi->dfs_lp_buffer_nphy != 0) {
		pi->dfs_lp_buffer_nphy = 0;
		st->first_radar_indicator = 1;
		PHY_RADAR(("DFS LP buffer =  "));
		for (i = 0; i < rt->lp_len_his_idx; i++) {
			PHY_RADAR(("%d, ", rt->lp_len_his[i]));
		}
		PHY_RADAR(("%d; now CLEARED\n", rt->lp_length));
		rt->lp_length = 0;
		rt->lp_pw_fm_matched = 0;
		rt->lp_n_non_single_pulses = 0;
		rt->lp_cnt = 0;
		rt->lp_skip_cnt = 0;
		rt->lp_skip_tot = 0;
		rt->lp_csect_single = 0;
		rt->lp_len_his_idx = 0;
		rt->last_detection_time = pi->sh->now;
		rt->last_detection_time_lp = pi->sh->now;
	}
	if (!rparams->radar_args.npulses) {
		PHY_ERROR(("%s: radar params not initialized\n", __FUNCTION__));
		return RADAR_TYPE_NONE;
	}

	/* initialize variable */
	pulse_interval = 0;

	min_detected_pw = rparams->radar_args.max_pw;
	max_detected_pw = rparams->radar_args.min_pw;

	/* suspend mac before reading phyregs */
	wlapi_suspend_mac_and_wait(pi->sh->physhim);
	phy_utils_phyreg_enter(pi);

	if (D11REV_IS(pi->sh->corerev, 11) || D11REV_IS(pi->sh->corerev, 12)) {
		wlapi_bmac_mctrl(pi->sh->physhim, MCTL_PHYLOCK,  MCTL_PHYLOCK);
		(void)R_REG(pi->sh->osh, &pi->regs->maccontrol);
		OSL_DELAY(1);
	}

	if (ISNPHY(pi) && NREV_LT(pi->pubpi->phy_rev, 7))
		wlc_phy_radar_read_table_rev_lt7(pi, st, 1);
	else
		wlc_phy_radar_read_table(pi, st, 1);

	if (D11REV_IS(pi->sh->corerev, 11) || D11REV_IS(pi->sh->corerev, 12))
		wlapi_bmac_mctrl(pi->sh->physhim, MCTL_PHYLOCK,  0);

	/* restart mac after reading phyregs */
	phy_utils_phyreg_exit(pi);
	wlapi_enable_mac(pi->sh->physhim);

	/* skip radar detect if doing periodic cal
	 * (the test-tones utilized during cal can trigger
	 * radar detect)
	 * NEED TO BE HERE AFTER READING DATA FROM (CLEAR) THE FIFO
	 */
	if (ISNPHY(pi)) {
	  if (pi->u.pi_nphy->nphy_rxcal_active) {
	    pi->u.pi_nphy->nphy_rxcal_active = FALSE;
	    PHY_RADAR(("DOING RXCAL, SKIP RADARS\n"));
	    return RADAR_TYPE_NONE;
	  }
	}
	if (ISHTPHY(pi)) {
	  if (pi->u.pi_htphy->radar_cal_active) {
	    pi->u.pi_htphy->radar_cal_active = FALSE;
	    PHY_RADAR(("DOING CAL, SKIP RADARS\n"));
	    return RADAR_TYPE_NONE;
	  }
	}
	if (ISACPHY(pi)) {
	  if (pi->u.pi_acphy->radar_cal_active) {
	    pi->u.pi_acphy->radar_cal_active = FALSE;
	    PHY_RADAR(("DOING CAL, SKIP RADARS\n"));
	    return RADAR_TYPE_NONE;
	  }
	}
	if (ISACPHY(pi) && TONEDETECTION)
	  FMCOMBINEOFFSET =
	    MAX((2 << (((rparams->radar_args.st_level_time>> 12) & 0xf) -1)) -1, 0);
	else
	  FMCOMBINEOFFSET = 0;

	/*
	 * Reject if no pulses recorded
	 */
	if (GET_RDR_NANTENNAS(pi) == 2) {
	    if ((rt->nphy_length[0] < 1) && (rt->nphy_length[1] < 1)) {
			return RADAR_TYPE_NONE;
		}
	} else {
		if (rt->nphy_length[0] < 1) {
		return RADAR_TYPE_NONE;
	    }
	}

#ifdef BCMDBG
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
	    if ((rparams->radar_args.feature_mask & 0x8) ||
	        (rparams->radar_args.feature_mask & 0x40))  {
	      PHY_RADAR(("\nShort pulses entering radar_detect_run "));
	      PHY_RADAR(("\nAnt %d: %d pulses, ", ant, rt->nphy_length[ant]));

	      PHY_RADAR(("\ntstart0=[  "));
			for (i = 0; i < rt->nphy_length[ant]; i++)
			  PHY_RADAR(("%u ", rt->tstart_list_n[ant][i]));
					  PHY_RADAR(("];"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->nphy_length[ant]; i++)
				PHY_RADAR(("%u ", rt->tstart_list_n[ant][i] -
				   rt->tstart_list_n[ant][i - 1]));

			PHY_RADAR(("\nPulse Widths:  "));
			for (i = 0; i < rt->nphy_length[ant]; i++)
			  PHY_RADAR(("%d-%d ", rt->width_list_n[ant][i], i));

			PHY_RADAR(("\nFM:  "));
			for (i = 0; i < rt->nphy_length[ant]; i++)
			  PHY_RADAR(("%d-%d ", rt->fm_list_n[ant][i], i));
			PHY_RADAR(("\n"));
	    }
	}
#endif /* BCMDBG */

	/* t2_min[15:12] = x; if n_non_single >= x && lp_length > npulses_lp => bin5 detected */
	/* t2_min[11:10] = # times combining adjacent pulses < min_pw_lp  */
	/* t2_min[9] = fm_tol enable */
	/* t2_min[8] = skip_type 5 enable */
	/* t2_min[7:4] = y; bin5 remove pw <= 10*y  */
	/* t2_min[3:0] = t; non-bin5 remove pw <= 5*y */

	/* remove "noise" pulses with small pw */
	if (!(rparams->radar_args.feature_mask & 0x600)) {
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
		wr_ptr = 0;
		mlength = rt->nphy_length_bin5[ant];
		for (i = 0; i < rt->nphy_length_bin5[ant]; i++) {
			if (rt->width_list_bin5[ant][i] >
			    10*((rparams->radar_args.t2_min >> 4) & 0xf) &&
			    rt->fm_list_bin5[ant][i] >
			    ((ISACPHY(pi) && TONEDETECTION) ?
			     rparams->radar_args.min_fm_lp : rparams->radar_args.min_fm_lp/2)) {
				rt->tstart_list_bin5[ant][wr_ptr] = rt->tstart_list_bin5[ant][i];
				rt->fm_list_bin5[ant][wr_ptr] = rt->fm_list_bin5[ant][i];
				rt->width_list_bin5[ant][wr_ptr] = rt->width_list_bin5[ant][i];
				++wr_ptr;
			} else {
				mlength--;
			}
		}	/* for mlength loop */
		rt->nphy_length_bin5[ant] = mlength;
	}	/* for ant loop */
	}

#ifdef BCMDBG
	/* output ant0/1 fifo data */
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
	    if (rparams->radar_args.feature_mask & 0x8) {
			if ((rparams->radar_args.feature_mask & 0x1) == 0) { /* bin5 */
				if (rt->nphy_length_bin5[ant] > 0) {
				  PHY_RADAR(("\nBin5 after removing noise pulses with pw <= %d",
				  ((rparams->radar_args.t2_min >> 4) & 0xf) * 10));
				  PHY_RADAR(("\nAnt %d: %d pulses, ", ant,
				  rt->nphy_length_bin5[ant]));

				  PHY_RADAR(("\ntstart0=[  "));
				  for (i = 0; i < rt->nphy_length_bin5[ant]; i++)
				    PHY_RADAR(("%u ", rt->tstart_list_bin5[ant][i]));
				  PHY_RADAR(("];"));

				  PHY_RADAR(("\nInterval:  "));
				  for (i = 1; i < rt->nphy_length_bin5[ant]; i++)
				    PHY_RADAR(("%d ", rt->tstart_list_bin5[ant][i] -
				    rt->tstart_list_bin5[ant][i - 1]));

				  PHY_RADAR(("\nPulse Widths:  "));
				  for (i = 0; i < rt->nphy_length_bin5[ant]; i++)
				    PHY_RADAR(("%d-%d ", rt->width_list_bin5[ant][i], i));

				  PHY_RADAR(("\nFM:  "));
				  for (i = 0; i < rt->nphy_length_bin5[ant]; i++)
				    PHY_RADAR(("%d-%d ", rt->fm_list_bin5[ant][i], i));
				  PHY_RADAR(("\n"));
				}
			} else { /* short pulse */
			if (rt->nphy_length[ant] > 0) {
				PHY_RADAR(("\nShort pulses entering radar_detect_run "));
				PHY_RADAR(("\nAnt %d: %d pulses, ", ant, rt->nphy_length[ant]));

				PHY_RADAR(("\ntstart0=[  "));
				for (i = 0; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%u ", rt->tstart_list_n[ant][i]));
				PHY_RADAR(("];"));

				PHY_RADAR(("\nInterval:  "));
				for (i = 1; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%d ", rt->tstart_list_n[ant][i] -
					rt->tstart_list_n[ant][i - 1]));

				PHY_RADAR(("\nPulse Widths:  "));
				for (i = 0; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%d-%d ", rt->width_list_n[ant][i], i));

				PHY_RADAR(("\nFM:  "));
				for (i = 0; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%d-%d ", rt->fm_list_n[ant][i], i));
				PHY_RADAR(("\n"));
			}
	      }
	    }
	}
#endif /* BCMDBG */

	if (rparams->radar_args.feature_mask & 0x800) {   /* if fcc */

	/* START LONG PULSES (BIN5) DETECTION */

	/* Combine pulses that are adjacent for ant 0 and ant 1 */
	  for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
	    rt->length =  rt->nphy_length_bin5[ant];
	    for (k = 0; k < ((rparams->radar_args.t2_min >> 10) & 0x3); k++) {
		mlength = rt->length;
		if (mlength > 1) {
			for (i = 1; i < mlength; i++) {
				deltat = ABS((int32)(rt->tstart_list_bin5[ant][i] -
					rt->tstart_list_bin5[ant][i-1]));

				if (deltat <= (int32)rparams->radar_args.max_pw_lp) {
					rt->width_list_bin5[ant][i-1] =
					  deltat + rt->width_list_bin5[ant][i];
					/* print pulse combining debug messages */
					if (0) {
						PHY_RADAR(("*%d,%d,%d ",
						rt->tstart_list_bin5[ant][i] -
						rt->tstart_list_bin5[ant][i-1],
						rt->width_list_bin5[ant][i],
						rt->width_list_bin5[ant][i-1]));
					}
					rt->fm_list_bin5[ant][i-1] =
					  (rt->fm_list_bin5[ant][i-1] + rt->fm_list_bin5[ant][i]) -
					  FMCOMBINEOFFSET;
					for (j = i; j < mlength - 1; j++) {
						rt->tstart_list_bin5[ant][j] =
							rt->tstart_list_bin5[ant][j+1];
						rt->width_list_bin5[ant][j] =
							rt->width_list_bin5[ant][j+1];
						rt->fm_list_bin5[ant][j] =
						  rt->fm_list_bin5[ant][j+1];
					}
					mlength--;
					rt->length--;
					/* i--; */
				     }	/* if deltat */
			}	/* for mlength loop */
		}	/* mlength > 1 */
	    }
	  } /* for ant loop */

	/* Now combine, sort, and remove duplicated pulses from the each antenna */
	rt->length = 0;
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
		for (i = 0; i < rt->nphy_length_bin5[ant]; i++) {
			rt->tstart_list[rt->length] = rt->tstart_list_bin5[ant][i];
			rt->width_list[rt->length] = rt->width_list_bin5[ant][i];
			rt->fm_list[rt->length] = rt->fm_list_bin5[ant][i];
			rt->length++;
		}
	}

	for (i = 1; i < rt->length; i++) {	/* insertion sort */
		tstart = rt->tstart_list[i];
		width = rt->width_list[i];
		fm = rt->fm_list[i];
		j = i;
		while ((j > 0) && rt->tstart_list[j - 1] > tstart) {
			rt->tstart_list[j] = rt->tstart_list[j - 1];
			rt->width_list[j] = rt->width_list[j - 1];
			rt->fm_list[j] = rt->fm_list[j - 1];
			j--;
		}
		rt->tstart_list[j] = tstart;
		rt->width_list[j] = width;
		rt->fm_list[j] = fm;
	}

#ifdef BCMDBG
	/* output fifo data */
	if (rparams->radar_args.feature_mask & 0x8)  {
		if ((rparams->radar_args.feature_mask & 0x1) == 0) { /* bin5 */
			if ((rt->length > 0))  {
			PHY_RADAR(("\nBin5 after combining pulses from two antennas"));
			PHY_RADAR(("\n%d pulses, ", rt->length));

			PHY_RADAR(("\ntstart=[  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
			PHY_RADAR(("];"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->length; i++)
				PHY_RADAR(("%d ", rt->tstart_list[i] -
					rt->tstart_list[i - 1]));

			PHY_RADAR(("\nPulse Widths:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->width_list[i], i));

			PHY_RADAR(("\nFM:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
			PHY_RADAR(("\n"));
			}
		}
	}
#endif /* BCMDBG */

	/* Should be done only for 2 chain cases */
	/* Combine pulses that are adjacent among 2 antennas */
	if (GET_RDR_NANTENNAS(pi) == 2) {
		for (k = 0; k < 1; k++) {
		mlength = rt->length;
		if (mlength > 1) {
			for (i = 1; i < mlength; i++) {
				deltat = ABS((int32)(rt->tstart_list[i] -
					rt->tstart_list[i-1]));

				if (deltat <= (int32)rparams->radar_args.max_pw_lp) {
					rt->width_list[i-1] = deltat + rt->width_list[i];
					/* print pulse combining debug messages */
					if (0) {
						PHY_RADAR(("*%d,%d,%d ",
						rt->tstart_list[i] -
						rt->tstart_list[i-1],
						rt->width_list[i],
						rt->width_list[i-1]));
					}

					rt->fm_list[i-1] = rt->fm_list[i-1] +
						rt->fm_list[i]-FMCOMBINEOFFSET;
					for (j = i; j < mlength - 1; j++) {
						rt->tstart_list[j] =
							rt->tstart_list[j+1];
						rt->width_list[j] =
							rt->width_list[j+1];
						rt->fm_list[j] = rt->fm_list[j+1];
					}
					mlength--;
					rt->length--;
				}	/* if deltat */
			}	/* for mlength loop */
		}	/* mlength > 1 */
	    }
	}

#ifdef BCMDBG
	/* output fifo data */
	if (rparams->radar_args.feature_mask & 0x8)  {
		if ((rparams->radar_args.feature_mask & 0x1) == 0) { /* bin5 */
			if ((rt->length > 0))  {
			PHY_RADAR(("\nBin5 after combining pulses that are adjcent"));
			PHY_RADAR(("\n%d pulses, ", rt->length));

			PHY_RADAR(("\ntstart=[  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
			PHY_RADAR(("];"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->length; i++)
				PHY_RADAR(("%d ", rt->tstart_list[i] -
					rt->tstart_list[i - 1]));

			PHY_RADAR(("\nPulse Widths:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->width_list[i], i));

			PHY_RADAR(("\nFM:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
			PHY_RADAR(("\n"));
			}
		}
	}
#endif /* BCMDBG */

	/* remove pulses that are spaced < quant (128/256) */
	for (i = 1; i < rt->length; i++) {
		deltat = ABS((int32)(rt->tstart_list[i] - rt->tstart_list[i-1]));
		if (deltat < 128) {
			for (j = i - 1; j < (rt->length); j++) {
				rt->tstart_list[j] = rt->tstart_list[j+1];
				rt->width_list[j] = rt->width_list[j+1];
				rt->fm_list[j] = rt->fm_list[j+1];
			}
			rt->length--;
		}
	}

#ifdef BCMDBG
	/* output fifo data */
	if (rparams->radar_args.feature_mask & 0x8)  {
		if ((rparams->radar_args.feature_mask & 0x1) == 0) {
			if ((rt->length > 0))  { /* bin5 */
			PHY_RADAR(("\nBin5 after removing pulses that are spaced < %d\n",
				rparams->radar_args.quant));
			PHY_RADAR(("%d pulses, ", rt->length));

			PHY_RADAR(("\ntstart=[  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
			PHY_RADAR(("];"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->length; i++)
				PHY_RADAR(("%d ", rt->tstart_list[i] -
					rt->tstart_list[i - 1]));
			PHY_RADAR(("\n"));

			PHY_RADAR(("Pulse Widths:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->width_list[i], i));
			PHY_RADAR(("\n"));

			PHY_RADAR(("FM:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
			PHY_RADAR(("\n"));
			}
		}
	}
#endif /* BCMDBG */

	/* prune lp buffer */
	/* remove any entry outside the time max delta_t_lp */
	if (rt->lp_length > 1) {
		deltat = ABS((int32)(rt->lp_buffer[rt->lp_length - 1] - rt->lp_buffer[0]));
		i = 0;
		while ((i < (rt->lp_length - 1)) &&
			(deltat > (int32)max_lp_buffer_span)) {
			i++;
			deltat = ABS((int32)(rt->lp_buffer[rt->lp_length - 1] - rt->lp_buffer[i]));
		}

		if (i > 0) {
			for (j = i; j < rt->lp_length; j++)
				rt->lp_buffer[j-i] = rt->lp_buffer[j];

			rt->lp_length -= i;
		}
	}
	/* First perform FCC-5 detection */
	/* add new pulses */

	/* process each new pulse */
	for (i = 0; i < rt->length; i++) {
		deltat = ABS((int32)(rt->tstart_list[i] - rt->last_tstart));
		salvate_intv = ABS((int32) (rt->tstart_list[i] - rt->last_skipped_time));
		valid_lp = (rt->width_list[i] >= rparams->radar_args.min_pw_lp) &&
			(rt->width_list[i] <= rparams->radar_args.max_pw_lp) &&
			(rt->fm_list[i] >= rparams->radar_args.min_fm_lp) &&
			(deltat >= rparams->radar_args.max_pw_lp);

		/* filter out: max_deltat_l < pburst_intv_lp < min_burst_intv_lp */
		/* this was skip-type = 2, now not skipping for this */
		valid_lp = valid_lp &&(deltat <= (int32) rparams->max_deltat_lp ||
			deltat >= (int32) rparams->radar_args.min_burst_intv_lp);
		if ((salvate_intv > (int32) rparams->max_deltat_lp &&
			salvate_intv < (int32) rparams->radar_args.min_burst_intv_lp)) {
			valid_lp = FALSE;
		}

		if (rt->lp_length > 0 && rt->lp_length < LP_BUFFER_SIZE) {
			valid_lp = valid_lp &&
				(rt->tstart_list[i] != rt->lp_buffer[rt->lp_length]);
		}

		skip_type = 0;
		if (valid_lp && deltat
			>= (int32) rparams->radar_args.min_burst_intv_lp &&
			deltat < (int32) rparams->radar_args.max_burst_intv_lp) {
			rt->lp_cnt = 0;
		}

		/* skip the pulse if outside of pulse interval range (1-2ms), */
		/* burst to burst interval not within range, more than 3 pulses in a */
		/* burst, and not skip salvated */

		if ((valid_lp && ((rt->lp_length != 0)) &&
			((deltat < (int32) rparams->min_deltat_lp) ||
			(deltat > (int32) rparams->max_deltat_lp &&
			deltat < (int32) rparams->radar_args.min_burst_intv_lp) ||
			(deltat > (int32) rparams->radar_args.max_burst_intv_lp) ||
			(rt->lp_cnt > 2)))) {	/* possible skip lp */

			/* get skip type */
			if (deltat < (int32) rparams->min_deltat_lp) {
				skip_type = 1;
			} else if (deltat > (int32) rparams->max_deltat_lp &&
				deltat < (int32) rparams->radar_args.min_burst_intv_lp) {
				skip_type = 2;
			} else if (deltat > (int32) rparams->radar_args.max_burst_intv_lp) {
				skip_type = 3;
				rt->lp_cnt = 0;
			} else if (rt->lp_cnt > 2) {
				skip_type = 4;
			} else {
				skip_type = 999;
			}

			/* skip_salvate */
			if (((salvate_intv > (int32) rparams->min_deltat_lp &&
				salvate_intv < (int32) rparams->max_deltat_lp)) ||
				((salvate_intv > (int32)rparams->radar_args.min_burst_intv_lp) &&
				(salvate_intv < (int32)rparams->radar_args.max_burst_intv_lp))) {
				/* note valid_lp is not reset in here */
				skip_type = -1;  /* salvated PASSED */
				if (salvate_intv >= (int32) rparams->radar_args.min_burst_intv_lp &&
					salvate_intv <
						(int32) rparams->radar_args.max_burst_intv_lp) {
					rt->lp_cnt = 0;
				}
			}
		} else {  /* valid lp not by skip salvate */
				skip_type = -2;
		}

		width = 0;
		fm = 0;
		pw_dif = 0;
		fm_dif = 0;
		pw_tol = rparams->radar_args.max_span_lp & 0xff;
		fm_tol = 0;
		/* monitor the number of pw and fm matching */
		/* max_span_lp[15:12] = skip_tot max */
		/* max_span_lp[11:8] = x, x/16 = % alowed fm tollerance */
		/* max_span_lp[7:0] = alowed pw tollerance */
		if (valid_lp && skip_type <= 0) {
			if (rt->lp_cnt == 0) {
				rt->lp_pw[0] = rt->width_list[i];
				rt->lp_fm[0] = rt->fm_list[i];
			} else if (rt->lp_cnt == 1) {
				width = rt->lp_pw[0];
				fm = rt->lp_fm[0];
				pw_dif = ABS(rt->width_list[i] - width);
				fm_dif = ABS(rt->fm_list[i] - fm);
				if (rparams->radar_args.t2_min & 0x200) {
					fm_tol = (fm*((rparams->radar_args.max_span_lp >> 8)
						& 0xf))/16;
				} else {
					fm_tol = 999;
				}
				if (pw_dif < pw_tol && fm_dif < fm_tol) {
					rt->lp_pw[1] = rt->width_list[i];
					rt->lp_fm[1] = rt->fm_list[i];
					++rt->lp_n_non_single_pulses;
					++rt->lp_pw_fm_matched;
				} else if (rt->lp_just_skipped) {
					width = rt->lp_skipped_pw;
					fm = rt->lp_skipped_fm;
					pw_dif = ABS(rt->width_list[i] - width);
					fm_dif = ABS(rt->fm_list[i] - fm);
					if (rparams->radar_args.t2_min & 0x200) {
						fm_tol = (fm*((rparams->radar_args.max_span_lp >> 8)
							& 0xf))/16;
					} else {
						fm_tol = 999;
					}
					if (pw_dif < pw_tol && fm_dif < fm_tol) {
						rt->lp_pw[1] = rt->width_list[i];
						rt->lp_fm[1] = rt->fm_list[i];
						++rt->lp_n_non_single_pulses;
						++rt->lp_pw_fm_matched;
						skip_type = -1;  /* salvated PASSED */
					} else {
						if (rparams->radar_args.t2_min & 0x100) {
							skip_type = 5;
						}
					}
				} else {
					if (rparams->radar_args.t2_min & 0x100) {
						skip_type = 5;
					}
				}
			} else if (rt->lp_cnt == 2) {
				width = rt->lp_pw[1];
				fm = rt->lp_fm[1];
				pw_dif = ABS(rt->width_list[i] - width);
				fm_dif = ABS(rt->fm_list[i] - fm);
				if (rparams->radar_args.t2_min & 0x200) {
					fm_tol = (fm*((rparams->radar_args.max_span_lp >> 8)
						& 0xf))/16;
				} else {
					fm_tol = 999;
				}
				if (pw_dif < pw_tol && fm_dif < fm_tol) {
					rt->lp_pw[2] = rt->width_list[i];
					rt->lp_fm[2] = rt->fm_list[i];
					++rt->lp_n_non_single_pulses;
					++rt->lp_pw_fm_matched;
				} else if (rt->lp_just_skipped) {
					width = rt->lp_skipped_pw;
					fm = rt->lp_skipped_fm;
					pw_dif = ABS(rt->width_list[i] - width);
					fm_dif = ABS(rt->fm_list[i] - fm);
					if (rparams->radar_args.t2_min & 0x200) {
						fm_tol = (fm*((rparams->radar_args.max_span_lp >> 8)
							& 0xf))/16;
					} else {
						fm_tol = 999;
					}
					if (pw_dif < pw_tol && fm_dif < fm_tol) {
						rt->lp_pw[2] = rt->width_list[i];
						rt->lp_fm[2] = rt->fm_list[i];
						++rt->lp_n_non_single_pulses;
						++rt->lp_pw_fm_matched;
						skip_type = -1;  /* salvated PASSED */
					} else {
						if (rparams->radar_args.t2_min & 0x100) {
							skip_type = 5;
						}
					}
				} else {
					if (rparams->radar_args.t2_min & 0x100) {
						skip_type = 5;
					}
				}
			}
		}

		if (valid_lp && skip_type != -1 && skip_type != -2) {	/* skipped lp */
			valid_lp = FALSE;
			rt->lp_skip_cnt++;
			rt->lp_skip_tot++;
			rt->lp_just_skipped = TRUE;
			rt->lp_skipped_pw = rt->width_list[i];
			rt->lp_skipped_fm = rt->fm_list[i];

			tmp_uint32 = rt->last_skipped_time;
			rt->last_skipped_time = rt->tstart_list[i];

#ifdef BCMDBG
			/* print "SKIPPED LP" debug messages */
			PHY_RADAR(("Skipped LP:"
/*
				" KTstart=%u Klast_ts=%u Klskip=%u"
*/
				" nLP=%d nSKIP=%d KIntv=%u"
				" Ksalintv=%d PW=%d FM=%d"
				" Type=%d pulse#=%d skip_tot=%d csect_single=%d\n",
/*
			(rt->tstart_list[i])/1000, rt->last_tstart/1000, tmp_uint32/1000,
*/
				rt->lp_length, rt->lp_skip_cnt, deltat/1000, salvate_intv/1000,
				rt->width_list[i], rt->fm_list[i],
				skip_type, rt->lp_cnt, rt->lp_skip_tot, rt->lp_csect_single));
			if (skip_type == 5) {
				PHY_RADAR(("           "
					" pw2=%d pw_dif=%d pw_tol=%d fm2=%d fm_dif=%d fm_tol=%d\n",
					width, pw_dif, pw_tol, fm, fm_dif, fm_tol));
			}
			if (skip_type == 999) {
				PHY_RADAR(("UNKOWN SKIP TYPE: %d\n", skip_type));
			}
#endif /* BCMDBG */

			/* if a) 2 consecutive skips, or */
			/*    b) too many consective singles, or */
			/*    c) too many total skip so far */
			/*  then reset lp buffer ... */
			if (rt->lp_skip_cnt >= rparams->radar_args.nskip_rst_lp) {
				if (rt->lp_len_his_idx < LP_LEN_HIS_SIZE) {
					rt->lp_len_his[rt->lp_len_his_idx] = rt->lp_length;
					rt->lp_len_his_idx++;
				}
				rt->lp_length = 0;
				rt->lp_skip_tot = 0;
				rt->lp_skip_cnt = 0;
				rt->lp_csect_single = 0;
				rt->lp_pw_fm_matched = 0;
				rt->lp_n_non_single_pulses = 0;
				rt->lp_cnt = 0;
			}
		} else if (valid_lp && (rt->lp_length < LP_BUFFER_SIZE)) {	/* valid lp */
			/* reset consecutive singles counter if pulse # > 0 */
			if (rt->lp_cnt > 0) {
				rt->lp_csect_single = 0;
			} else {
				++rt->lp_csect_single;
			}

			rt->lp_just_skipped = FALSE;
			/* print "VALID LP" debug messages */
			rt->lp_skip_cnt = 0;
			PHY_RADAR(("Valid LP:"
/*
				" KTstart=%u KTlast_ts=%u Klskip=%u"
*/
				" KIntv=%u"
				" Ksalintv=%d PW=%d FM=%d pulse#=%d"
				" pw2=%d pw_dif=%d pw_tol=%d fm2=%d fm_dif=%d fm_tol=%d\n",
/*
				(rt->tstart_list[i])/1000, rt->last_tstart/1000,
					rt->last_skipped_time/1000,
*/
				deltat/1000, salvate_intv/1000,
				rt->width_list[i], rt->fm_list[i], rt->lp_cnt,
				width, pw_dif, pw_tol,
				fm, fm_dif, fm_tol));
			PHY_RADAR(("         "
				" nLP=%d nSKIP=%d skipped_salvate=%d"
				" pw_fm_matched=%d #non-single=%d skip_tot=%d csect_single=%d\n",
				rt->lp_length + 1, rt->lp_skip_cnt, (skip_type == -1 ? 1 : 0),
				rt->lp_pw_fm_matched,
				rt->lp_n_non_single_pulses, rt->lp_skip_tot, rt->lp_csect_single));

				rt->lp_buffer[rt->lp_length] = rt->tstart_list[i];
				rt->lp_length++;
				rt->last_tstart = rt->tstart_list[i];
				rt->last_skipped_time = rt->tstart_list[i];

				rt->lp_cnt++;

			if (rt->lp_csect_single >= ((rparams->radar_args.t2_min >> 12) & 0xf)) {
				if (rt->lp_length > rparams->radar_args.npulses_lp / 2)
					rt->lp_length -= rparams->radar_args.npulses_lp / 2;
				else
					rt->lp_length = 0;
			}
		}
	}

	if (rt->lp_length > LP_BUFFER_SIZE)
		PHY_ERROR(("WARNING: LP buffer size is too long\n"));

#ifdef RADAR_DBG
	PHY_RADAR(("\n FCC-5 \n"));
	for (i = 0; i < rt->lp_length; i++) {
		PHY_RADAR(("%u  ", rt->lp_buffer[i]));
	}
	PHY_RADAR(("\n"));
#endif
	if (rt->lp_length >= rparams->radar_args.npulses_lp) {
		/* reject detection spaced more than 3 minutes */
		deltat2 = (uint32) (pi->sh->now - rt->last_detection_time_lp);
		PHY_RADAR(("last_detection_time_lp=%u, watchdog_timer=%u, deltat2=%d,"
			" deltat2_min=%d, deltat2_sec=%d\n",
			rt->last_detection_time_lp, pi->sh->now, deltat2, deltat2/60, deltat2%60));
		rt->last_detection_time_lp = pi->sh->now;
		tmp_uint32 = RADAR_TYPE_NONE;
		if ((uint32) deltat2 < (rparams->radar_args.fra_pulse_err & 0xff)*60 ||
		    (st->first_radar_indicator == 1 && (uint32) deltat2 < 30*60)) {
			if (rt->lp_csect_single <= rparams->radar_args.npulses_lp - 2 &&
				rt->lp_skip_tot < ((rparams->radar_args.max_span_lp >> 12) & 0xf)) {
				PHY_RADAR(("FCC-5 Radar Detection. Time from last detection"
					" = %u, = %dmin %dsec\n",
					deltat2, deltat2 / 60, deltat2 % 60));

				tmp_uint32 = RADAR_TYPE_FCC_5;
				st->radar_status.detected = TRUE;
				st->radar_status.count =
					st->radar_status.count + 1;
				st->radar_status.pretended = FALSE;
				st->radar_status.radartype = tmp_uint32;
				st->radar_status.timenow = (uint32) (pi->sh->now);
				st->radar_status.timefromL = deltat2;
				st->radar_status.ch = pi->radio_chanspec;
				st->radar_status.lp_csect_single = rt->lp_csect_single;
			}
		}
#ifdef BCMDBG
		else {
			if (rparams->radar_args.feature_mask & 0x4000) {
				PHY_RADAR(("SKIPPED false FCC-5 Radar Detection."
					" Time from last detection = %u, = %dmin %dsec,"
					" ncsect_single=%d\n",
					deltat2, deltat2 / 60, deltat2 % 60, rt->lp_csect_single));
			}
		}
#endif /* BCMDBG */
		if (rt->lp_len_his_idx < LP_LEN_HIS_SIZE) {
			rt->lp_len_his[rt->lp_len_his_idx] = rt->lp_length;
			rt->lp_len_his_idx++;
		}
		rt->lp_length = 0;
		rt->lp_pw_fm_matched = 0;
		rt->lp_n_non_single_pulses = 0;
		rt->lp_skip_tot = 0;
		rt->lp_csect_single = 0;
		st->first_radar_indicator = 0;
		return tmp_uint32;
	}
	}	/* end of if fcc */
	/*
	 * Reject if no pulses recorded
	 */
	if (GET_RDR_NANTENNAS(pi) == 2) {
	    if (rt->nphy_length[0] < (rparams->radar_args.npulses) &&
	    rt->nphy_length[1] < (rparams->radar_args.npulses)) {
		    return RADAR_TYPE_NONE;
	    }
	} else if (GET_RDR_NANTENNAS(pi) == 1) {
		if (rt->nphy_length[0] < (rparams->radar_args.npulses)) {
			return RADAR_TYPE_NONE;
	    }
	}

	/* START SHORT PULSES (NON-BIN5) DETECTION */
	/* remove "noise" pulses with  pw> 400 and fm<500 */
	if (ISACPHY(pi) && TONEDETECTION) {
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
		wr_ptr = 0;
		mlength = rt->nphy_length[ant];
		for (i = 0; i < rt->nphy_length[ant]; i++) {
		  if (rt->width_list_n[ant][i]  < (rparams->radar_args.st_level_time & 0x0fff) ||
		      rt->fm_list_n[ant][i] > rparams->radar_args.min_fm_lp) {
				rt->tstart_list_n[ant][wr_ptr] = rt->tstart_list_n[ant][i];
				rt->fm_list_n[ant][wr_ptr] = rt->fm_list_n[ant][i];
				rt->width_list_n[ant][wr_ptr] = rt->width_list_n[ant][i];
				++wr_ptr;
			} else {
				mlength--;
			}
		}	/* for mlength loop */
		rt->nphy_length[ant] = mlength;
	}	/* for ant loop */
	}
	/* Combine pulses that are adjacent */
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
		for (k = 0; k < 2; k++) {
			mlength = rt->nphy_length[ant];
			if (mlength > 1) {
			for (i = 1; i < mlength; i++) {
				deltat = ABS((int32)(rt->tstart_list_n[ant][i] -
					rt->tstart_list_n[ant][i-1]));
				if ((deltat < (int32)rparams->radar_args.min_deltat && FALSE) ||
				    (deltat <= (int32)rparams->radar_args.max_pw && TRUE))
				{
					if (NREV_GE(pi->pubpi->phy_rev, 7) || ISHTPHY(pi) ||
						ISACPHY(pi)) {
						if (((uint32)(rt->tstart_list_n[ant][i] -
							rt->tstart_list_n[ant][i-1]))
							<= (uint32) rparams->radar_args.max_pw) {
							rt->width_list_n[ant][i-1] =
							ABS((int32)(rt->tstart_list_n[ant][i] -
							rt->tstart_list_n[ant][i-1])) +
							rt->width_list_n[ant][i];
						}
					} else {
#ifdef BCMDBG
						/* print pulse combining debug messages */
						PHY_RADAR(("*%d,%d,%d ",
							rt->tstart_list_n[ant][i] -
							rt->tstart_list_n[ant][i-1],
							rt->width_list_n[ant][i],
							rt->width_list_n[ant][i-1]));
#endif
						if (rparams->radar_args.feature_mask & 0x2000) {
							rt->width_list_n[ant][i-1] =
								(rt->width_list_n[ant][i-1] >
								rt->width_list_n[ant][i] ?
								rt->width_list_n[ant][i-1] :
								rt->width_list_n[ant][i]);
						} else {
							rt->width_list_n[ant][i-1] =
								rt->width_list_n[ant][i-1] +
								rt->width_list_n[ant][i];
						}
					}

					/* Combine fm */
					if (ISACPHY(pi) && TONEDETECTION) {
						rt->fm_list_n[ant][i-1] = (rt->fm_list_n[ant][i-1] >
							rt->fm_list_n[ant][i] ?
							rt->fm_list_n[ant][i-1] :
							rt->fm_list_n[ant][i]);
					} else {
						rt->fm_list_n[ant][i-1] = rt->fm_list_n[ant][i-1] +
							rt->fm_list_n[ant][i];
					}

					for (j = i; j < mlength - 1; j++) {
						rt->tstart_list_n[ant][j] =
							rt->tstart_list_n[ant][j+1];
						rt->width_list_n[ant][j] =
							rt->width_list_n[ant][j+1];
						rt->fm_list_n[ant][j] =
							rt->fm_list_n[ant][j+1];
					}
					mlength--;
					rt->nphy_length[ant]--;
				}
			} /* for i < mlength */
			} /* mlength > 1 */
		}
	}
#ifdef BCMDBG
	/* output ant0/ant1 fifo data */
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
	    if (rparams->radar_args.feature_mask & 0x8)  {
			if ((rparams->radar_args.feature_mask & 0x1) == 1 &&
				(rt->nphy_length[ant] > 0)) {	/* short pulses */
				PHY_RADAR(("\nShort Pulse After combining adacent pulses"));
				PHY_RADAR(("\nAnt %d: %d pulses, ", ant, rt->nphy_length[ant]));

				PHY_RADAR(("\ntstart0=[  "));
				for (i = 0; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%u ", rt->tstart_list_n[ant][i]));
				PHY_RADAR(("];"));

				PHY_RADAR(("\nInterval:  "));
				for (i = 1; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%d ", rt->tstart_list_n[ant][i] -
						rt->tstart_list_n[ant][i - 1]));

				PHY_RADAR(("\nPulse Widths:  "));
				for (i = 0; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%d-%d ", rt->width_list_n[ant][i], i));

				PHY_RADAR(("\nFM:  "));
				for (i = 0; i < rt->nphy_length[ant]; i++)
					PHY_RADAR(("%d-%d ", rt->fm_list_n[ant][i], i));
				PHY_RADAR(("\n"));

			}
	    }
	}
#endif /* BCMDBG */

	/* Now combine, sort, and remove duplicated pulses from the 2 antennas */
	bzero(rt->tstart_list, sizeof(rt->tstart_list));
	bzero(rt->width_list, sizeof(rt->width_list));
	bzero(rt->fm_list, sizeof(rt->fm_list));
	rt->length = 0;
	for (ant = 0; ant < GET_RDR_NANTENNAS(pi); ant++) {
		for (i = 0; i < rt->nphy_length[ant]; i++) {
			rt->tstart_list[rt->length] = rt->tstart_list_n[ant][i];
			rt->width_list[rt->length] = rt->width_list_n[ant][i];
			rt->fm_list[rt->length] = rt->fm_list_n[ant][i];
			rt->length++;
		}
	}

	for (i = 1; i < rt->length; i++) {	/* insertion sort */
		tstart = rt->tstart_list[i];
		width = rt->width_list[i];
		fm = rt->fm_list[i];
		j = i;
		while ((j > 0) && rt->tstart_list[j - 1] > tstart) {
			rt->tstart_list[j] = rt->tstart_list[j - 1];
			rt->width_list[j] = rt->width_list[j - 1];
			rt->fm_list[j] = rt->fm_list[j - 1];
			j--;
		}
		rt->tstart_list[j] = tstart;
		rt->width_list[j] = width;
		rt->fm_list[j] = fm;
	}

#ifdef BCMDBG
	/* output fifo data */
	if (rparams->radar_args.feature_mask & 0x8)  {
		if ((rparams->radar_args.feature_mask & 0x1) == 1 &&
			(rt->length > 0)) {	/* short pulses */
			PHY_RADAR(("\nShort pulses after combining pulses from two antennas"));
			PHY_RADAR(("\n%d pulses, ", rt->length));

			PHY_RADAR(("\ntstart0=[  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
			PHY_RADAR(("];"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->length; i++)
				PHY_RADAR(("%d ", rt->tstart_list[i] -
					rt->tstart_list[i - 1]));

			PHY_RADAR(("\nPulse Widths:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->width_list[i], i));

			PHY_RADAR(("\nFM:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
			PHY_RADAR(("\n"));
		}
	}
#endif /* BCMDBG */

	/* Should be done only for 2 chain cases */
	if (GET_RDR_NANTENNAS(pi) == 2) {
		for (k = 0; k < 1; k++) {
			mlength = rt->length;
			if (mlength > 1) {
				for (i = 1; i < mlength; i++) {
					deltat = ABS((int32)(rt->tstart_list[i] -
						rt->tstart_list[i-1]));
					if ((deltat < (int32)rparams->radar_args.min_deltat &&
					FALSE) ||
					(deltat <= (int32)rparams->radar_args.max_pw &&
					TRUE)) {
						if (NREV_GE(pi->pubpi->phy_rev, 7) ||
							ISHTPHY(pi) || ISACPHY(pi)) {
							if (((uint32)(rt->tstart_list[i] -
							rt->tstart_list[i-1]))
							<= (uint32) rparams->radar_args.max_pw) {
								rt->width_list[i-1] =
								ABS((int32)(rt->tstart_list[i] -
								rt->tstart_list[i-1])) +
								rt->width_list[i];
							}
						} else {
							if (rparams->radar_args.feature_mask &
								0x2000) {
								rt->width_list[i-1] =
									(rt->width_list[i-1] >
									rt->width_list[i]) ?
									rt->width_list[i-1] :
									rt->width_list[i];
							} else {
								rt->width_list[i-1] =
									rt->width_list[i-1] +
									rt->width_list[i];
							}
						}
						/* Combine fm */
						if (ISACPHY(pi) && TONEDETECTION) {
							rt->fm_list[i-1] =
								(rt->fm_list[i-1] > rt->fm_list[i] ?
								rt->fm_list[i-1] : rt->fm_list[i]);
						} else {
							rt->fm_list[i-1] =
								rt->fm_list[i-1] + rt->fm_list[i];
						}

						for (j = i; j < mlength - 1; j++) {
							rt->tstart_list[j] =
								rt->tstart_list[j+1];
							rt->width_list[j] =
								rt->width_list[j+1];
							rt->fm_list[j] =
								rt->fm_list[j+1];
						}
						mlength--;
						rt->length--;
					}
				}
			}
		}
	}

	/* remove pulses spaced less than min_deltat */
	for (i = 1; i < rt->length; i++) {
		deltat = (int32)(rt->tstart_list[i] - rt->tstart_list[i-1]);
		if (deltat < (int32)rparams->radar_args.min_deltat) {
			for (j = i; j < (rt->length - 1); j++) {
				rt->tstart_list[j] = rt->tstart_list[j+1];
				rt->width_list[j] = rt->width_list[j+1];
				rt->fm_list[j] = rt->fm_list[j+1];
			}
			rt->length--;
		}
	}

#ifdef BCMDBG
	/* output fifo data */
	if (rparams->radar_args.feature_mask & 0x8)  {
		if ((rparams->radar_args.feature_mask & 0x1) == 1 &&
			(rt->length > 0)) {	/* short pulses */
			PHY_RADAR(("\nShort pulses after removing pulses that are"
				" space > min_deltat (1ms)\n"));
			PHY_RADAR(("%d pulses, ", rt->length));

			PHY_RADAR(("\ntstart0=[  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
			PHY_RADAR(("];\n"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->length; i++)
				PHY_RADAR(("%d ", rt->tstart_list[i] -
					rt->tstart_list[i - 1]));
			PHY_RADAR(("\n"));

			PHY_RADAR(("Pulse Widths:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->width_list[i], i));

			PHY_RADAR(("\nFM:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
			PHY_RADAR(("\n"));
		}
	}
#endif /* BCMDBG */

	/* remove entries spaced greater than max_deltat */
	if (rt->length > 1) {
		deltat = ABS((int32)(rt->tstart_list[rt->length - 1] - rt->tstart_list[0]));
		i = 0;
		while ((i < (rt->length - 1)) &&
		       (ABS(deltat) > (int32)rparams->radar_args.max_deltat)) {
			i++;
			deltat = ABS((int32)(rt->tstart_list[rt->length - 1] - rt->tstart_list[i]));
		}
		if (i > 0) {
			for (j = i; j < rt->length; j++) {
				rt->tstart_list[j-i] = rt->tstart_list[j];
				rt->width_list[j-i] = rt->width_list[j];
				rt->fm_list[j-1] = rt->fm_list[j];
			}
			rt->length -= i;
		}
	}

	/*
	 * filter based on pulse width
	 */
	if (filter_pw) {
		j = 0;
		for (i = 0; i < rt->length; i++) {
			if ((rt->width_list[i] >= rparams->radar_args.min_pw) &&
				(rt->width_list[i] <= rparams->radar_args.max_pw)) {
				rt->width_list[j] = rt->width_list[i];
				rt->tstart_list[j] = rt->tstart_list[i];
				rt->fm_list[j] = rt->fm_list[i];
				j++;
			}
		}
		rt->length = j;
	}
	if (ISACPHY(pi) && TONEDETECTION) {
	if (filter_fm) {
		j = 0;
		for (i = 0; i < rt->length; i++) {
			if ((rt->fm_list[i] >= -50)) {
				rt->width_list[j] = rt->width_list[i];
				rt->tstart_list[j] = rt->tstart_list[i];
				rt->fm_list[j] = rt->fm_list[i];
				j++;
			}
		}
		rt->length = j;
	}
	}

#ifdef BCMDBG
	/* output fifo data */
	if (rparams->radar_args.feature_mask & 0x8)  {
		if ((rparams->radar_args.feature_mask & 0x1) == 1 &&
			(rt->length > 0)) {	/* short pulses */
			PHY_RADAR(("\nShort pulses after removing pulses that are"
				" space < max_deltat (150ms)\n"));
			PHY_RADAR(("%d pulses, ", rt->length));

			PHY_RADAR(("\ntstart0=[  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
			PHY_RADAR(("];\n"));

			PHY_RADAR(("\nInterval:  "));
			for (i = 1; i < rt->length; i++)
				PHY_RADAR(("%d ", rt->tstart_list[i] -
					rt->tstart_list[i - 1]));
			PHY_RADAR(("\n"));

			PHY_RADAR(("Pulse Widths:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->width_list[i], i));

			PHY_RADAR(("\nFM:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
			PHY_RADAR(("\n"));
		}
	}
#endif /* BCMDBG */

	if (NREV_GE(pi->pubpi->phy_rev, 3) || ISHTPHY(pi) || ISACPHY(pi)) { /* nphy rev >= 3 */
/*
		ASSERT(rt->length <= RDR_NANTENNAS * RDR_LIST_SIZE);
*/
		if (rt->length > RDR_NANTENNAS * RDR_LIST_SIZE) {
			rt->length = RDR_NANTENNAS * RDR_LIST_SIZE;
			PHY_RADAR(("WARNING: radar rt->length=%d > list_size=%d\n",
				rt->length, RDR_NANTENNAS * RDR_LIST_SIZE));
		}
	} else {
/*
		ASSERT(rt->length <= RDR_LIST_SIZE);
*/
		if (rt->length > RDR_LIST_SIZE) {
			rt->length = RDR_LIST_SIZE;
			PHY_RADAR(("WARNING: radar rt->length = %d > RDR_LIST_SIZE = %d\n",
				rt->length, RDR_LIST_SIZE));
		}
	}

	/*
	 * Break pulses into epochs.
	 */
	rt->nepochs = 1;
	rt->epoch_start[0] = 0;
	for (i = 1; i < rt->length; i++) {
		if ((int32)(rt->tstart_list[i] - rt->tstart_list[i-1]) > rparams->max_blen) {
			rt->epoch_finish[rt->nepochs-1] = i - 1;
			rt->epoch_start[rt->nepochs] = i;
			rt->nepochs++;
		}
		if (rt->nepochs >= RDR_EPOCH_SIZE) {
			if (rparams->radar_args.feature_mask & 0x100) {
				PHY_RADAR(("WARNING: number of epochs %d > epoch size = %d\n",
					rt->nepochs, RDR_EPOCH_SIZE));
			}
			break;
		}
	}
	rt->epoch_finish[rt->nepochs - 1] = i;

	det_type = 0;

	/*
	 * Run the detector for each epoch
	 */
	for (i = 0; i < rt->nepochs && (pulse_interval == 0) && (det_type == 0); i++) {

		/*
		 * Generate 0th order tier list (time delta between received pulses)
		 * Quantize and filter delta pulse times delta pulse times are
		 * returned in sorted order from smallest to largest.
		 */
		epoch_list = rt->tstart_list + rt->epoch_start[i];
		epoch_length = (rt->epoch_finish[i] - rt->epoch_start[i] + 1);
		if (epoch_length > RDR_NTIER_SIZE) {
			if (rparams->radar_args.feature_mask & 0x100) {
				PHY_RADAR(("WARNING: DFS epoch_length=%d > TIER_SIZE=%d!!\n",
					epoch_length, RDR_NTIER_SIZE));
			}
			epoch_length = RDR_NTIER_SIZE;
		}

		wlc_phy_radar_detect_run_epoch(pi, i, rt, rparams, epoch_list, epoch_length,
			pw_2us, pw15us, pw20us, pw30us,
			i250us, i500us, i625us, i5000us,
			pw2us, i833us, i2500us, i3333us,
			i938us, i3066us,
			&det_type, &pulse_interval,
			&nconsecq_pulses, &detected_pulse_index,
			&min_detected_pw, &max_detected_pw,
			&fm_min, &fm_max);
	}

#ifdef BCMDBG
	if (rparams->radar_args.feature_mask & 0x2) {
		/*	Changed to display intervals instead of tstart
			PHY_RADAR(("Start Time:  "));
			for (i = 0; i < rt->length; i++)
				PHY_RADAR(("%u ", rt->tstart_list[i]));
		*/
		PHY_RADAR(("\nShort pulses before pruning (filtering)"));
		PHY_RADAR(("\nInterval:  "));
		for (i = 1; i < rt->length; i++)
			PHY_RADAR(("%d-%d ", rt->tstart_list[i] - rt->tstart_list[i - 1], i));
		PHY_RADAR(("\n"));

		PHY_RADAR(("Pulse Widths:  "));
		for (i = 0; i < rt->length; i++)
			PHY_RADAR(("%d-%d ", rt->width_list[i], i));
		PHY_RADAR(("\n"));

		PHY_RADAR(("FM:  "));
		for (i = 0; i < rt->length; i++)
			PHY_RADAR(("%d-%d ", rt->fm_list[i], i));
		PHY_RADAR(("\n"));
	}

	if (rparams->radar_args.feature_mask & 0x80) {
		PHY_RADAR(("\nShort pulses after pruning (filtering)"));
		PHY_RADAR(("\nPruned Intv: "));
		for (i = 0; i < epoch_length-2; i++)
			PHY_RADAR(("%d-%d ", rt->tiern_list[0][i], i));
		PHY_RADAR(("\n"));

		PHY_RADAR(("Pruned PW:  "));
		for (i = 0; i <  epoch_length-1; i++)
			PHY_RADAR(("%d-%d ", rt->tiern_pw[0][i], i));
		PHY_RADAR(("\n"));

		PHY_RADAR(("Pruned FM:  "));
		for (i = 0; i <  epoch_length-1; i++)
			PHY_RADAR(("%d-%d ", rt->tiern_fm[0][i], i));
		PHY_RADAR(("\n"));
		PHY_RADAR(("nconsecq_pulses=%d max_pw_delta=%d min_pw=%d max_pw=%d \n",
			nconsecq_pulses, max_detected_pw - min_detected_pw, min_detected_pw,
			max_detected_pw));
	}
#endif /* BCMDBG */

	epoch_detected = i;

	if (pulse_interval || det_type != 0) {
		bzero(st->radar_status.intv, sizeof(st->radar_status.intv));
		bzero(st->radar_status.pw, sizeof(st->radar_status.pw));
		bzero(st->radar_status.fm, sizeof(st->radar_status.fm));

		BCM_REFERENCE(epoch_detected);
		PHY_RADAR(("\nPruned Intv: "));
		for (i = 0; i < epoch_length-1; i++) {
			PHY_RADAR(("%d-%d ", rt->tiern_list[0][i], i));
			if (i >= detected_pulse_index && i < detected_pulse_index + 10) {
				st->radar_status.intv[i - detected_pulse_index] =
					rt->tiern_list[0][i];
			}
		}
		PHY_RADAR(("\n"));

		PHY_RADAR(("Pruned PW:  "));
		for (i = 0; i <  epoch_length-1; i++) {
			PHY_RADAR(("%i-%d ", rt->tiern_pw[0][i], i));
			if (i >= detected_pulse_index && i < detected_pulse_index + 10) {
				st->radar_status.pw[i - detected_pulse_index] = rt->tiern_pw[0][i];
			}
		}
		PHY_RADAR(("\n"));

		PHY_RADAR(("Pruned FM:  "));
		for (i = 0; i <  epoch_length-1; i++) {
			PHY_RADAR(("%i-%d ", rt->tiern_fm[0][i], i));
			if (i >= detected_pulse_index && i < detected_pulse_index + 10) {
				st->radar_status.fm[i - detected_pulse_index] = rt->tiern_fm[0][i];
			}
		}
		PHY_RADAR(("\n"));

		PHY_RADAR(("Nepochs=%d len=%d epoch_#=%d; det_idx=%d "
				"pw_delta=%d min_pw=%d max_pw=%d \n",
				rt->nepochs, epoch_length, epoch_detected, detected_pulse_index,
				max_detected_pw - min_detected_pw, min_detected_pw,
				max_detected_pw));

		deltat2 = (uint32) (pi->sh->now - rt->last_detection_time);
		/* detection not valid if detected pulse index too large */
		if (detected_pulse_index < ((rparams->radar_args.ncontig) & 0x3f) -
			rparams->radar_args.npulses) {
			rt->last_detection_time = pi->sh->now;
		}
		/* reject detection spaced more than 3 minutes and detected pulse index too larg */
		if (((uint32) deltat2 < (rparams->radar_args.fra_pulse_err & 0xff)*60 ||
			(st->first_radar_indicator == 1 && (uint32) deltat2 < 30*60)) &&
			(detected_pulse_index <
			((rparams->radar_args.ncontig) & 0x3f)
			- rparams->radar_args.npulses)) {
			PHY_RADAR(("Type %d Radar Detection. Detected pulse index=%d"
				" fm_min=%d fm_max=%d nconsecq_pulses=%d."
				" Time from last detection = %u, = %dmin %dsec \n",
				det_type, detected_pulse_index, fm_min, fm_max, nconsecq_pulses,
				deltat2, deltat2/60, deltat2%60));
			st->radar_status.detected = TRUE;
			st->radar_status.count = st->radar_status.count + 1;
			st->radar_status.pretended = FALSE;
			st->radar_status.radartype = det_type;
			st->radar_status.timenow = (uint32) pi->sh->now;
			st->radar_status.timefromL = (uint32) deltat2;
			st->radar_status.detected_pulse_index =  detected_pulse_index;
			st->radar_status.nconsecq_pulses = nconsecq_pulses;
			st->radar_status.ch = pi->radio_chanspec;
			st->first_radar_indicator = 0;
			return (det_type + (min_detected_pw << 4) +  (pulse_interval << 14));
		} else {
			if (rparams->radar_args.feature_mask & 0x4000) {
				PHY_RADAR(("SKIPPED false Type %d Radar Detection."
					" min_pw=%d pw_delta=%d pri=%d"
					" fm_min=%d fm_max=%d nconsecq_pulses=%d. Time from last"
					" detection = %u, = %dmin %dsec",
					det_type, min_detected_pw,
					max_detected_pw - max_detected_pw,
					pulse_interval, fm_min, fm_max, nconsecq_pulses, deltat2,
					deltat2 / 60, deltat2 % 60));
				if (detected_pulse_index < ((rparams->radar_args.ncontig) & 0x3f) -
					rparams->radar_args.npulses)
					PHY_RADAR((". Detected pulse index: %d\n",
						detected_pulse_index));
				else
					PHY_ERROR((". Detected pulse index too high: %d\n",
						detected_pulse_index));
			}
			return (RADAR_TYPE_NONE);
		}
	}
	return (RADAR_TYPE_NONE);
}
