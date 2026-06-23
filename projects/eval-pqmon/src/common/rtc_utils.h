/*******************************************************************************
 *   @file   rtc_utils.h
 *   @brief  RTC utils for eval-pqmon project
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
 *    software without specific prior warranty.
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

#ifndef __RTC_UTILS_H__
#define __RTC_UTILS_H__

#include <stdint.h>
#include <time.h>
#include "max31343.h"

/* Last read RTC timestamp — populated by rtc_get_time_burst() */
extern struct max31343_time_stamp rtc_time_stamp;

/**
 * @brief Pre-compute the BCD time-register burst buffer from a Unix epoch.
 *
 * Runs the expensive gmtime()/BCD conversion in the main context and stores the
 * result in sync_burst_buf, so the ISR-side write (rtc_trigger_write) only has
 * to push the bytes over I2C.
 * @param unix_epoch Absolute time to program, in seconds.
 */
void rtc_prepare_sync(uint32_t unix_epoch);

/**
 * @brief Reset the MAX31343: SWRST then re-enable the oscillator (ENOSC).
 *
 * Zeroes the sub-second prescaler phase (the only lever for it, as the chip has
 * no sub-second register), clears status flags, and disables interrupts.
 * @return 0 on success, negative error code otherwise.
 */
int rtc_trigger_reset(void);

/**
 * @brief Burst-write the pre-computed time registers over I2C (ISR-safe).
 *
 * Writes the buffer prepared by rtc_prepare_sync(); 
 * @return 0 on success, negative error code otherwise.
 */
int rtc_trigger_write(void);

/**
 * @brief Read the current RTC time as a Unix epoch via a single burst read.
 * @param rtc_time Output: current time in seconds since the Unix epoch.
 * @return 0 on success, negative error code otherwise.
 */
int rtc_get_time_burst(uint32_t *rtc_time);

#endif /* __RTC_UTILS_H__ */
