/*******************************************************************************
 *   @file   pps_utils.c
 *   @brief  PPS timer substrate: capture, period validation, drift measurement
 *   @author Gabriel Diaconu (Alexandrugabriel.Diaconu@analog.com)
 ********************************************************************************
* Copyright (c) 2026 Analog Devices, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
* EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <stdbool.h>
#include <stdlib.h>
#include "pps_utils.h"
#include "gnss_utils.h"
#include "rtc_utils.h"
#include "common_data.h"
#include "pqlib_example.h"
#include "interrupt.h"
#include "no_os_timer.h"
#include "no_os_error.h"
#include "no_os_print_log.h"
#include "tmr_regs.h"

static struct no_os_timer_desc *pps_timer_desc;

static volatile uint32_t timestamp_gnss_pps = 0;
static volatile uint32_t timestamp_rtc_pps  = 0;

static volatile uint32_t gnss_pps_period = 0;

volatile pps_state_t pps_ctx = PPS_WAIT;
uint32_t pps_real_freq_hz = PPS_TIMER_FREQ_HZ;

/* Coarse client-facing status. Written and read only from the main loop
 * (state machine writes, IIO show reads) — no ISR access, so not volatile. */
static time_sync_status_t time_sync_status = TIME_SYNC_UNSYNCED;

/* Anchor caches the absolute ms time and HW timer value of the first
 * RMSONERDY after each PPS. Subsequent cycles in the same second are
 * computed relative to the anchor, avoiding a race where the PPS ISR
 * updates ts_gnss before the state machine increments time_ms. */
static int64_t  anchor_ms = 0;
static uint32_t anchor_hw_ts = 0;
static bool need_anchor = true;

/* Sub-second phase reference. When GNSS PPS is present the anchor offset is
 * measured from the GNSS edge (most accurate). During holdover (GNSS missing,
 * the RTC alone carries the clock) it switches to the RTC edge so per-cycle
 * timestamps stay aligned to the second boundary without any GNSS pulse. This
 * is what lets the system run on RTC alone. */
static bool anchor_on_rtc = false;

int pps_timer_init(void)
{
	int ret;

	ret = no_os_timer_init(&pps_timer_desc, &pps_timer_ip);
	if (ret)
		return ret;

	ret = no_os_timer_start(pps_timer_desc);
	if (ret) {
		no_os_timer_remove(pps_timer_desc);
		pps_timer_desc = NULL;
		return ret;
	}

	pr_info("PPS timer initialized (id=%d, %luHz)\n\r",
		PPS_TIMER_ID, (unsigned long)PPS_TIMER_FREQ_HZ);

	return 0;
}

uint32_t timer_read_count(void)
{
	uint32_t cnt = 0;

	no_os_timer_counter_get(pps_timer_desc, &cnt);

	return cnt;
}

bool gnss_pps_period_valid(uint32_t period_ticks)
{
	return period_ticks >= PPS_TIMER_FREQ_HZ * 9 / 10 &&
	       period_ticks <= PPS_TIMER_FREQ_HZ * 11 / 10;
}

void capture_pps_gnss(uint32_t cnt)
{
	uint32_t period;
	if (timestamp_gnss_pps != 0) {

		period = cnt - timestamp_gnss_pps;

		if (gnss_pps_period_valid(period)) {
			gnss_pps_period = period;
		} else {
			pr_warning("PPS gap detected: period=%lu ticks (expected ~%lu)\n",
				   (unsigned long)period, (unsigned long)PPS_TIMER_FREQ_HZ);
		}
	}

	timestamp_gnss_pps = cnt;
}

uint32_t pps_get_gnss_period(void)
{
	return gnss_pps_period;
}

void capture_pps_rtc(uint32_t cnt)
{
	timestamp_rtc_pps = cnt;
}

int pps_get_drift_us(int32_t *drift_us)
{
	if (!drift_us)
		return -EINVAL;

	uint32_t gnss_ts = timestamp_gnss_pps;
	uint32_t rtc_ts  = timestamp_rtc_pps;

	int32_t diff_ticks = (int32_t)(rtc_ts - gnss_ts);
	*drift_us = (int32_t)((int64_t)diff_ticks * 1000000 / pps_real_freq_hz);

	return 0;
}

time_sync_status_t pps_get_status(void)
{
	return time_sync_status;
}

void pps_get_raw_timestamps(uint32_t *gnss_ts, uint32_t *rtc_ts)
{
	if (gnss_ts)
		*gnss_ts = timestamp_gnss_pps;
	if (rtc_ts)
		*rtc_ts = timestamp_rtc_pps;
}

uint32_t pps_get_sync_edge(void)
{
	/* HW timer capture of the pulse that marked the current second boundary:
	 * the GNSS edge normally, or the RTC edge during holdover. Callers offset
	 * from this to place sub-second time, so it must follow the same source
	 * the state machine used to advance time_ms. */
	return anchor_on_rtc ? timestamp_rtc_pps : timestamp_gnss_pps;
}

void pqm_update_timestamp(void)
{
	uint32_t hw_ts;
	int32_t delta_ticks;
	int32_t ticks_from_anchor;

	/* Per-cycle external timestamp (runs BEFORE PPS state machine).
	 * Runs before time_ms is incremented, so the first RMSONERDY
	 * after each PPS caches a consistent (ms, hw_ts) anchor; later cycles
	 * compute relative to it. */
	if (consume_rmsonerdy_ts(&hw_ts)) {
		if (need_anchor) {
			/* First RMSONERDY after PPS: anchor absolute ms to this
			 * cycle. Edge follows the active source (GNSS or RTC). */
			delta_ticks = (int32_t)(hw_ts - pps_get_sync_edge());
			if (delta_ticks < 0)
				delta_ticks += pps_real_freq_hz;
			anchor_ms = time_ms +
				    delta_ticks * 1000LL / pps_real_freq_hz;
			anchor_hw_ts = hw_ts;
			need_anchor = false;
		}

		/* Remaining cycles: offset from anchor (uint32 wrap safe via int32) */
		ticks_from_anchor = (int32_t)(hw_ts - anchor_hw_ts);
		if (ticks_from_anchor < 0)
			ticks_from_anchor += pps_real_freq_hz;

		pqlibExample.inputCycle.timestamp =
			anchor_ms + ticks_from_anchor * 1000LL / pps_real_freq_hz;
	}
}

void pps_run_state_machine(void)
{
	static uint32_t window_start = 0;
	static uint16_t miss_count = 0;
	static bool initial_sync_done = false;
	static bool pps_aligned = false;
	int32_t drift_us = 0;
	uint8_t gnss_pps_state;
	uint8_t rtc_pps_state;
	uint32_t period;
	uint32_t next_epoch;
	uint32_t gnss_timeout;
	uint32_t rtc_timeout;

	gnss_pps_state = get_gnss_pps_state();
	rtc_pps_state = get_rtc_pps_state();

	/* --- PPS state machine: pairs GNSS and RTC pulses, tracks drift --- */
	switch (pps_ctx) {
	case PPS_WAIT:
		/* Idle: wait for either GNSS or RTC PPS edge */
		if (gnss_pps_state && rtc_pps_state) {
			pps_ctx = PPS_BOTH;
		} else if (gnss_pps_state) {
			window_start = timer_read_count();
			pps_ctx = PPS_WAIT_RTC;
		} else if (rtc_pps_state) {
			window_start = timer_read_count();
			pps_ctx = PPS_WAIT_GNSS;
		}
		break;

	case PPS_WAIT_RTC:
		/* GNSS arrived first; wait for RTC.
		 * Wide window until aligned, then tighten to 12ms (mirrors WAIT_GNSS).
		 * On timeout → PPS_RTC_MISS: advance clock on GNSS alone. */
		rtc_timeout = (initial_sync_done || pps_aligned)
			      ? (pps_real_freq_hz * 12 / 1000)
			      : (pps_real_freq_hz);
		if (rtc_pps_state) {
			pps_ctx = PPS_BOTH;
		} else if (timer_read_count() - window_start > rtc_timeout) {
			pps_ctx = PPS_RTC_MISS;
		}
		break;

	case PPS_WAIT_GNSS:
		/* RTC arrived first; wait for GNSS.
		 * Wide (1000ms) until aligned, then tighten to 12ms (mirrors WAIT_RTC). */
		gnss_timeout = (initial_sync_done || pps_aligned)
			       ? (pps_real_freq_hz * 12 / 1000)
			       : (pps_real_freq_hz);
		if (gnss_pps_state) {
			pps_ctx = PPS_BOTH;
		} else if (timer_read_count() - window_start > gnss_timeout) {
			pps_ctx = PPS_GNSS_MISS;
		}
		break;

	case PPS_BOTH: {
		/* Both PPS edges received — advance clock, update freq, check drift */
		time_ms += 1000;
		need_anchor = true;
		anchor_on_rtc = false;  /* GNSS marked this second boundary */
		newSyncTimeAvailable = 1;
		/* Both edges fresh this second: GNSS is disciplining and the
		 * GNSS-RTC drift is meaningful. */
		time_sync_status = TIME_SYNC_GNSS_LOCKED;

		/* EMA filter (tau=16s) for HW timer frequency calibration */
		period = pps_get_gnss_period();
		if (period > 0)
			pps_real_freq_hz = (pps_real_freq_hz * 15 + period) / 16;

		pps_get_drift_us(&drift_us);

		/* If GNSS-RTC drift exceeds threshold, re-sync RTC at next PPS.
		 * Otherwise they are already aligned — mark pps_aligned so the
		 * GNSS wait window can tighten to 12ms (warm-boot path). */
		if (abs(drift_us) > SYNC_THRESHOLD_US) {
			next_epoch = (uint32_t)((time_ms + 1000) / 1000);
			arm_rtc_sync(next_epoch);
			pps_ctx = PPS_ARMED;
		} else {
			pps_aligned = true;
			pps_ctx = PPS_WAIT;
		}

		pr_info("[Drift PPS] d=%ld us\n\r", (long)drift_us);

		reset_gnss_pps_flag();
		reset_rtc_pps_flag();
		break;
	}

	case PPS_GNSS_MISS: {
		/* GNSS pulse missing — advance clock on RTC alone (holdover).
		 * Anchor sub-second timestamps to the RTC edge so cycles stay
		 * aligned without any GNSS pulse: this is the RTC-only path. */
		miss_count++;
		time_ms += 1000;
		need_anchor = true;
		anchor_on_rtc = true;
		newSyncTimeAvailable = 1;  /* fresh wall time, carried by RTC */
		/* GNSS gone: RTC carries the clock, drift no longer measurable. */
		time_sync_status = TIME_SYNC_HOLDOVER;
		reset_rtc_pps_flag();
		pps_ctx = PPS_WAIT;
		break;
	}

	case PPS_RTC_MISS: {
		/* RTC pulse missing — advance clock on GNSS alone.
		 * Mirror of PPS_GNSS_MISS: anchor stays on GNSS edge (anchor_on_rtc=false),
		 * no drift correction possible without both edges. */
		miss_count++;
		time_ms += 1000;
		need_anchor = true;
		anchor_on_rtc = false;
		newSyncTimeAvailable = 1;
		/* GNSS still disciplines the clock, but with no RTC edge this
		 * second the GNSS-RTC drift cannot be computed. */
		time_sync_status = TIME_SYNC_GNSS_LOCKED;
		reset_gnss_pps_flag();
		pps_ctx = PPS_WAIT;
		break;
	}

	case PPS_ARMED:
		/* RTC sync armed, waiting for next PPS to write new epoch */
		if (gnss_pps_state) {
			time_ms += 1000;
			need_anchor = true;
			newSyncTimeAvailable = 1;
			reset_gnss_pps_flag();
		}
		if (rtc_pps_state)
			reset_rtc_pps_flag();
		break;

	case PPS_SYNC: {
		/* RTC write in progress (ISR-driven), wait for completion */
		if (sync_state == SYNC_DONE) {
			initial_sync_done = true;
			pps_ctx = PPS_WAIT;
		}

		if (gnss_pps_state) {
			time_ms += 1000;
			need_anchor = true;
			newSyncTimeAvailable = 1;
			reset_gnss_pps_flag();
		}
		if (rtc_pps_state)
			reset_rtc_pps_flag();
		break;
	}
	}
}

int time_sync_boot(void)
{
	int status = 0;

	/* Phase 1: Wait for GNSS PPS signal with optional fix */
	bool gnss_available =
		false;  /* PPS signal present (used for freq calibration) */
	bool gnss_fix_valid = false;  /* GNSS timestamp valid (has actual fix) */
	uint32_t fix_epoch = 0, fix_frac = 0;

	uint32_t phase1_start = timer_read_count();
	uint32_t phase1_timeout_ticks = PPS_TIMER_FREQ_HZ * GNSS_BOOT_PHASE1_TIMEOUT_S;
	uint32_t phase2_timeout_ticks = PPS_TIMER_FREQ_HZ * GNSS_BOOT_PHASE2_TIMEOUT_S;
	uint32_t phase3_timeout_ticks = PPS_TIMER_FREQ_HZ * GNSS_BOOT_PHASE3_TIMEOUT_S;

	pr_info("Phase 1: Waiting for GNSS fix (timeout %us)...\n\r",
		GNSS_BOOT_PHASE1_TIMEOUT_S);

	while ((uint32_t)(timer_read_count() - phase1_start) < phase1_timeout_ticks) {
		if (get_gnss_pps_state()) {
			gnss_available = true;
			reset_gnss_pps_flag();
			status = gnss_process_pps(&fix_epoch, &fix_frac);
			if (status == 0) {
				gnss_fix_valid = true;
				time_ms = (int64_t)fix_epoch * 1000;
				pr_info("GNSS fix valid: epoch=%lu\n\r", (unsigned long)fix_epoch);
				break;
			} else {
				pr_info("GNSS PPS present but no fix yet\n\r");
				// pr_warning("GNSS read error during boot: %d\n\r", status);
			}
		}
	}

	if (!gnss_fix_valid) {
		/* No GNSS fix: fall back to RTC for the absolute timestamp,
		 * regardless of whether a PPS signal is present. If the RTC read
		 * also fails there is no time source at all — bail out now rather
		 * than running Phase 2/3 anchored to nothing. */
		uint32_t rtc_epoch = 0;
		if (rtc_get_time_burst(&rtc_epoch)) {
			pr_warning("No time source: no GNSS fix and RTC read failed\n\r");
			return -ETIMEDOUT;
		}
		time_ms = (int64_t)rtc_epoch * 1000;
		pr_info("Using RTC time: epoch=%lu\n\r", (unsigned long)rtc_epoch);
	}

	/* Phase 2: Measure GNSS frequency (GNSS_CALIB_PPS_COUNT PPS events) — only if GNSS available */
	if (gnss_available) {
		uint32_t phase2_start = timer_read_count();

		uint32_t event_count = 0;
		uint32_t t_start = 0, t_end = 0;

		pr_info("Phase 2: Measuring frequency (%u PPS, timeout %us)...\n\r",
			GNSS_CALIB_PPS_COUNT, GNSS_BOOT_PHASE2_TIMEOUT_S);

		while (event_count < GNSS_CALIB_PPS_COUNT &&
		       (uint32_t)(timer_read_count() - phase2_start) < phase2_timeout_ticks) {
			if (get_gnss_pps_state()) {
				event_count++;
				if (event_count == 1) {
					t_start = timer_read_count();
				} else if (event_count == GNSS_CALIB_PPS_COUNT) {
					t_end = timer_read_count();
				}
				reset_gnss_pps_flag();
			}
		}

		if (event_count == GNSS_CALIB_PPS_COUNT) {
			uint32_t elapsed_ticks = t_end - t_start;
			uint32_t freq_measured = elapsed_ticks /
						 (GNSS_CALIB_PPS_COUNT - 1);

			if (gnss_pps_period_valid(freq_measured)) {
				pps_real_freq_hz = freq_measured;
				pr_info("Calibrated: %lu Hz (elapsed %lu ticks)\n\r",
					(unsigned long)pps_real_freq_hz, (unsigned long)elapsed_ticks);
			} else {
				pps_real_freq_hz = PPS_TIMER_FREQ_HZ;
				pr_info("Frequency out of range: %lu Hz, using nominal\n\r",
					(unsigned long)freq_measured);
			}
		} else {
			pr_info("Calibration incomplete (%u/%u PPS), using nominal\n\r",
				event_count, GNSS_CALIB_PPS_COUNT);
			pps_real_freq_hz = PPS_TIMER_FREQ_HZ;
		}

		reset_gnss_pps_flag();
		reset_rtc_pps_flag();
	}

	/* Phase 3: Re-anchor to next GNSS PPS — only if GNSS available */
	if (gnss_available) {
		uint32_t phase3_start = timer_read_count();

		pr_info("Phase 3: Re-anchoring (timeout %us)...\n\r",
			GNSS_BOOT_PHASE3_TIMEOUT_S);

		while (!get_gnss_pps_state() &&
		       (uint32_t)(timer_read_count() - phase3_start) < phase3_timeout_ticks) {
			/* Wait for next PPS */
		}

		if (get_gnss_pps_state()) {
			if (gnss_fix_valid) {
				/* GNSS fix available: advance from Phase 1 epoch.
				 * Phase 2 consumed (GNSS_CALIB_PPS_COUNT - 1) intervals +
				 * Phase 3 adds 1 more = GNSS_CALIB_PPS_COUNT seconds total. */
				time_ms += (int64_t)GNSS_CALIB_PPS_COUNT * 1000;
			} else {
				/* No fix: use RTC as time reference at this PPS edge */
				uint32_t anchor_epoch = 0;
				if (rtc_get_time_burst(&anchor_epoch) == 0)
					time_ms = (int64_t)(anchor_epoch + 1) * 1000;
			}
			reset_gnss_pps_flag();
			pr_info("Re-anchored time_ms to %llu ms\n\r",
				(unsigned long long)time_ms);
		} else {
			pr_info("Phase 3 re-anchor timeout\n\r");
		}
	}

	return 0;
}
