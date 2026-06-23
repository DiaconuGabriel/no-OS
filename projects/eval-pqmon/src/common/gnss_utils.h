/***************************************************************************//**
 *   @file   gnss_utils.h
 *   @brief  Header file of gnss utils for eval-pqmon project.
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

#ifndef __GNSS_UTILS_H__
#define __GNSS_UTILS_H__

#include "no_os_gnss.h"
#include "nmea_ubx_gnss.h"
#include "nmea_ubx.h"
#include "common_data.h"

extern struct no_os_gnss_desc *gnss_desc;
extern struct no_os_gnss_precise_time gnss_precise_time;

/**
* @brief Initialize GNSS device, GPIOs, and PPS interrupt.
*
* @return 0 in case of success, negative error code otherwise.
*/
int gnss_start(void);

/**
* @brief Process PPS interrupt and retrieve GNSS timing data.
* @param unix_epoch - Pointer to store Unix epoch timestamp.
* @param fractional_data - Pointer to store fractional seconds.
* @return 0 in case of success, negative error code otherwise.
*/
int gnss_process_pps(uint32_t *unix_epoch, uint32_t *fractional_data);

/**
 * @brief Auto-detect GNSS module baudrate, switch to target, and init driver.
 * @param target_baud_rate - Desired UART baudrate for GNSS communication.
 * @return 0 in case of success, negative error code otherwise.
 */
int gnss_init_set_baud(uint32_t target_baud_rate);

#endif /* __GNSS_UTILS_H__ */