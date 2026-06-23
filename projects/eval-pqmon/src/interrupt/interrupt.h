/***************************************************************************//**
 *   @file   interrupt.h
 *   @brief  Interrupt configuration for eval-pqmon project.
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
#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include "nmea_ubx.h"
#include "pps_utils.h"

extern volatile sync_state_t sync_state;

/**
 * @brief Initialize GNSS PPS interrupt on P0.6.
 * @param gnss_dev - The GNSS device structure. IRQ and GPIO descriptors
 *                   are stored inside it.
 * @return 0 in case of success, negative error code otherwise.
 */
int gnss_pps_interrupt_init(struct gnss_dev *gnss_dev);

/**
 * @brief Remove GNSS PPS interrupt and release resources.
 * @param gnss_dev - The GNSS device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int gnss_pps_interrupt_remove(struct gnss_dev *gnss_dev);

/**
 * @brief Get GNSS PPS flag state.
 * @return 1 if a PPS pulse was received since last reset, 0 otherwise.
 */
uint8_t get_gnss_pps_state(void);

/**
 * @brief Clear the GNSS PPS flag after processing.
 */
void reset_gnss_pps_flag(void);

/* RTC PPS interrupt (P2.13) */

/**
 * @brief Initialize RTC PPS interrupt on P2.13.
 * @return 0 in case of success, negative error code otherwise.
 */
int rtc_pps_interrupt_init(void);

/**
 * @brief Remove RTC PPS interrupt and release resources.
 * @return 0 in case of success, negative error code otherwise.
 */
int rtc_pps_interrupt_remove(void);

/**
 * @brief Get RTC PPS flag state.
 * @return 1 if a PPS pulse was received since last reset, 0 otherwise.
 */
uint8_t get_rtc_pps_state(void);

/**
 * @brief Clear the RTC PPS flag after processing.
 */
void reset_rtc_pps_flag(void);

/**
 * @brief Re-enable RTC PPS IRQ after processing current pulse.
 * @return 0 in case of success, negative error code otherwise.
 */
int rtc_pps_irq_enable(void);

/**
 * @brief Manually disable RTC PPS IRQ.
 * @return 0 in case of success, negative error code otherwise.
 */
int rtc_pps_irq_disable(void);

/**
 * @brief Initialize the RTC sync timer (one-shot) for sync pre-compensation.
 * @return 0 in case of success, negative error code otherwise.
 */
int rtc_sync_timer_init(void);

/**
 * @brief Arm RTC sync via the sync timer one-shot pre-compensation.
 *
 * Fires the sync timer at (period/2 - 2ms) after the last GNSS PPS rising edge so
 * that SWRST completes at the falling edge and the I2C write completes at
 * the next rising edge. Requires at least 2 GNSS PPS edges (measured period).
 * Does nothing if period is not yet available.
 *
 * @param next_epoch - Unix epoch to write to RTC.
 */
void arm_rtc_sync(uint32_t next_epoch);

/**
 * @brief Initialize RMSONERDY interrupt on P0.30.
 * @return 0 in case of success, negative error code otherwise.
 */
int rmsonerdy_interrupt_init(void);

/**
 * @brief Consume hardware timestamp captured by RMSONERDY ISR.
 * @param out_ts - TMR1 counter value captured at RMSONERDY rising edge.
 * @return true if a new timestamp was available, false otherwise.
 */
bool consume_rmsonerdy_ts(uint32_t *out_ts);

#endif /* __INTERRUPT_H__ */