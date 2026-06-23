/*******************************************************************************
 *   @file   pps_utils.h
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

#ifndef __PPS_UTILS_H__
#define __PPS_UTILS_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	SYNC_DONE,
	SYNC_PEND_WR,
	SYNC_SETTLE,
} sync_state_t;

typedef enum {
	PPS_WAIT,
	PPS_WAIT_RTC,
	PPS_WAIT_GNSS,
	PPS_BOTH,
	PPS_GNSS_MISS,
	PPS_RTC_MISS,
	PPS_ARMED,
	PPS_SYNC,
} pps_state_t;

/* Coarse synchronization status, reported over IIO. Distinguishes a
 * GNSS-locked board (µs-accurate) from one free-running on the RTC (drifting). */
typedef enum {
	TIME_SYNC_UNSYNCED,	/* no fix yet / boot not completed */
	TIME_SYNC_GNSS_LOCKED,	/* GNSS PPS disciplining the clock */
	TIME_SYNC_HOLDOVER,	/* GNSS lost, RTC carrying the clock */
} time_sync_status_t;

extern volatile pps_state_t pps_ctx;	/* PPS pairing state (ISR + main loop) */
extern volatile sync_state_t sync_state;	/* RTC realignment sequencing */
extern uint32_t pps_real_freq_hz;	/* calibrated capture-timer frequency */

/**
 * @brief Initialise the free-running PPS capture timer (TMR1 @ 60 MHz).
 * @return 0 on success, negative errno otherwise.
 */
int pps_timer_init(void);

/**
 * @brief Read the current capture-timer count.
 * @return 32-bit tick value; wraps every ~71.5 s at 60 MHz.
 */
uint32_t timer_read_count(void);

/**
 * @brief Check whether a measured PPS period is within ±10% of one second.
 * @param period_ticks Interval between two edges, in timer ticks.
 * @return true if the period is plausible (rejects glitches and gaps).
 */
bool gnss_pps_period_valid(uint32_t period_ticks);

/**
 * @brief Record a GNSS 1PPS edge and update the measured GNSS period.
 * @param cnt Capture-timer count latched in the GNSS PPS ISR.
 */
void capture_pps_gnss(uint32_t cnt);

/**
 * @brief Record an RTC 1PPS edge.
 * @param cnt Capture-timer count latched in the RTC PPS ISR.
 */
void capture_pps_rtc(uint32_t cnt);

/**
 * @brief Compute the GNSS↔RTC phase offset from the last captured edges.
 * @param drift_us Output: (rtc_ts − gnss_ts) in microseconds; positive = RTC
 *                 arrives after GNSS. Meaningful only when both edges are fresh.
 * @return 0 on success, -EINVAL if @p drift_us is NULL.
 */
int pps_get_drift_us(int32_t *drift_us);

/**
 * @brief Current coarse synchronization status (locked / holdover / unsynced).
 */
time_sync_status_t pps_get_status(void);

/**
 * @brief Read the last captured GNSS and RTC edge counts.
 * @param gnss_ts Output for the GNSS edge (ignored if NULL).
 * @param rtc_ts  Output for the RTC edge (ignored if NULL).
 */
void pps_get_raw_timestamps(uint32_t *gnss_ts, uint32_t *rtc_ts);

/**
 * @brief Capture of the edge that marked the current second boundary.
 * @return The GNSS edge normally, or the RTC edge during holdover; sub-second
 *         timestamps are offset from this value.
 */
uint32_t pps_get_sync_edge(void);

/**
 * @brief Last valid GNSS PPS period, in timer ticks.
 * @return Measured period, or 0 if none has been captured yet.
 */
uint32_t pps_get_gnss_period(void);

/**
 * @brief Stamp the current metrology cycle with an absolute timestamp.
 *
 * Anchors absolute ms to the first RMSONERDY after each PPS, then offsets
 * later cycles from that anchor. Runs before the state machine advances the
 * clock so the anchor stays consistent within the second.
 */
void pqm_update_timestamp(void);

/**
 * @brief Advance the synchronization state machine by one iteration.
 *
 * Pairs GNSS and RTC pulses per second, advances the wall clock, refines the
 * timer frequency (EMA), measures drift, and arms an RTC resync past the
 * threshold. Handles the holdover paths when either edge is missing.
 */
void pps_run_state_machine(void);

/**
 * @brief Run the 3-phase GNSS boot calibration (fix wait, frequency measure,
 *        re-anchor). Assumes PPS timer and GNSS PPS interrupt are already up.
 * @return 0 on success, -ETIMEDOUT if no time reference could be obtained
 *         (neither a GNSS fix nor an RTC fallback time).
 */
int time_sync_boot(void);

#endif /* __PPS_UTILS_H__ */
