/*******************************************************************************
 *   @file   common_data.h
 *   @brief  Common data header file
 *   @author Robert Budai (robert.budai@analog.com)
 ********************************************************************************
 * Copyright (c) 2024 Analog Devices, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
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

#ifndef __COMMON_DATA_H__
#define __COMMON_DATA_H__

#if defined(PQM_CONN_ETH)
#include "w5500_network.h"
#endif

#if defined(PQM_CONN_T1L)
#include "lwip_socket.h"
#include "lwip_adin1110.h"
#include "adin1110.h"
#endif

#include "adi_pqlib.h"
#include "iio.h"
#include "iio_app.h"
#include "iio_pqm.h"
#include "iio_types.h"
#include "no_os_gpio.h"
#include "no_os_i2c.h"
#include "no_os_irq.h"
#include "no_os_spi.h"
#include "no_os_timer.h"
#include "no_os_uart.h"
#include "parameters.h"
#include "pqlib_example.h"
#include "afe_calibration.h"
#ifdef PQM_TIME_SYNC
#include "pps_utils.h"
#include "rtc_utils.h"
#include "gnss_utils.h"
#endif

#define FW_VERSION 2.2

#define IIO_BUFF_TYPE int16_t
#define SAMPLES_PER_CHANNEL_PLATFORM 256
#define MAX_SIZE_BASE_ADDR (SAMPLES_PER_CHANNEL_PLATFORM * TOTAL_PQM_CHANNELS)
#define MAX_SIZE_BASE_ADDR_WITH_SIZE                                           \
  (MAX_SIZE_BASE_ADDR * sizeof(IIO_BUFF_TYPE))

#define TOTAL_PQM_CHANNELS 11
#define VOLTAGE_CH_NUMBER 3
#define MAX_CH_ATTRS 23
#ifdef PQM_TIME_SYNC
#define PQM_DEVICE_ATTR_NUMBER 73
#else
#define PQM_DEVICE_ATTR_NUMBER 63
#endif
#define WAVEFORM_BUFFER_LENGTH (256 * 7)
#define MAX_EVENT_NUMBER 6

/* GNSS module UART line settings */
#define GNSS_UART_BAUDRATE		115200
#define GNSS_UART_PARITY		NO_OS_UART_PAR_NO
#define GNSS_UART_STOP			NO_OS_UART_STOP_1_BIT

/* PPS pulse: 1 Hz, 500 ms high */
#define GNSS_PPS_1HZ         		1
#define GNSS_PPS_LENGTH 			500

/* RTC drift threshold for triggering sync (microseconds) */
#define SYNC_THRESHOLD_US       10000

/* GNSS boot sequence timeouts (seconds).
 * P1 waits for first PPS edge (fix optional; cold-boot fix can need ~60s and
 * falls back to RTC). P2 needs >= GNSS_CALIB_PPS_COUNT s — keep P2 >= 2x count
 * or calibration times out. P3 waits one re-anchor edge (~1s). */
#define GNSS_BOOT_PHASE1_TIMEOUT_S  35
#define GNSS_BOOT_PHASE2_TIMEOUT_S  20
#define GNSS_BOOT_PHASE3_TIMEOUT_S  5

/* PPS edges sampled during Phase 2 frequency calibration. The measurement spans
 * (GNSS_CALIB_PPS_COUNT - 1) one-second intervals; Phase 3 advances the time
 * anchor by GNSS_CALIB_PPS_COUNT seconds total (Phase 2 intervals + 1). */
#define GNSS_CALIB_PPS_COUNT        10

#define PQM_GNSS_SLAVE 0
extern IIO_BUFF_TYPE iio_data_buffer_loc[MAX_SIZE_BASE_ADDR];

#if defined(PQM_CONN_ETH)
extern const struct no_os_spi_init_param w5500_spi_init_params;

extern struct w5500_init_param w5500_ip;
extern struct w5500_network_init_param w5500_network_ip;
#endif

#if defined(PQM_CONN_USB)
extern struct no_os_uart_init_param iio_demo_usb_ip;
#elif defined(PQM_CONN_SERIAL)
extern struct no_os_uart_init_param iio_demo_serial_ip;
#elif defined(PQM_CONN_T1L)
extern struct no_os_uart_init_param iio_demo_serial_ip;
extern const struct no_os_gpio_init_param adin1110_int_ip;
extern const struct no_os_gpio_init_param adin1110_rst_gpio_ip;
extern const struct no_os_gpio_init_param adin1110_swpd_ip;
extern const struct no_os_gpio_init_param adin1110_tx2p4_ip;
extern const struct no_os_gpio_init_param adin1110_mssel_ip;
extern const struct no_os_gpio_init_param adin1110_cfg0_ip;
extern const struct no_os_gpio_init_param adin1110_cfg1_ip;
extern const struct no_os_spi_init_param adin1110_spi_ip;

extern struct adin1110_init_param adin1110_ip;
extern struct lwip_network_param lwip_ip;
#endif

extern struct pqm_init_para pqm_ip;
extern struct no_os_spi_init_param spi_egy_ip;
extern struct no_os_uart_init_param uart_ip_stdio;
extern struct no_os_gpio_init_param reset_gpio_ip;
extern struct no_os_gpio_init_param intr_gpio_ip;
extern struct no_os_irq_init_param afe_callback_ctrl_ip;
extern struct no_os_callback_desc afe0_callback_desc;

#ifdef PQM_TIME_SYNC
extern struct no_os_i2c_init_param rtc_i2c_ip;
extern struct max31343_dev *rtc_desc;
extern struct max31343_init_param rtc_init_param;

extern struct no_os_timer_init_param pps_timer_ip;
extern struct no_os_irq_init_param gnss_pps_irq_ip;
extern struct no_os_gpio_init_param rtc_pps_gpio_ip;
extern struct no_os_irq_init_param rtc_pps_irq_ip;
extern struct no_os_gpio_init_param gnss_pps_gpio_ip;
extern struct no_os_gpio_init_param gnss_reset_gpio_ip;
extern struct no_os_uart_init_param uart_gnss_ip;
extern struct no_os_irq_init_param gnss_nvic_ip;
extern struct no_os_irq_init_param rtc_nvic_ip;
extern struct no_os_irq_init_param rtc_sync_timer_nvic_ip;
#endif

static const char *const pqm_v_consel_available[] = {
	[VCONSEL_4W_WYE] = "4W_WYE",
	[VCONSEL_3W_DELTA_VB_VA_NEGVC] = "3W_DELTA_VA_VB_NEGVC",
	[VCONSEL_4W_WYE_VB_NEGVA_NEGVC] = "4W_WYE_VB_NEGVA_NEGVC",
	[VCONSEL_4W_DELTA_VB_NEGVA] = "4W_DELTA_VB_NEGVA",
	[VCONSEL_4W_DELTA_VA_VB_VC] = "4W_DELTA_VA_VB_VC",
};

static const char *const pqm_flicker_model_available[] = {
	[ADI_PQLIB_FLICKER_MODEL_230V_50HZ] = "230V_50HZ",
	[ADI_PQLIB_FLICKER_MODEL_120V_50HZ] = "120V_50HZ",
	[ADI_PQLIB_FLICKER_MODEL_230V_60HZ] = "230V_60HZ",
	[ADI_PQLIB_FLICKER_MODEL_120V_60HZ] = "120V_60HZ",
};

static const char *const pqm_calibration_type_available[] = {
	[CALIBRATION_TYPE_GAIN] = "GAIN",
	[CALIBRATION_TYPE_OFFSET] = "OFFSET",
};

static const char *const pqm_calibration_phase_available[] = {
	[CALIBRATION_PHASE_A] = "A",
	[CALIBRATION_PHASE_B] = "B",
	[CALIBRATION_PHASE_C] = "C",
};

static const char *const pqm_nominal_frequency_available[] = {
	[ADI_PQLIB_NOMINAL_FREQUENCY_50HZ] = "50",
	[ADI_PQLIB_NOMINAL_FREQUENCY_60HZ] = "60",
};

struct pqm_desc {
	uint8_t reg[TOTAL_PQM_CHANNELS];
	float pqm_global_attr[PQM_DEVICE_ATTR_NUMBER];
	uint32_t pqm_ch_attr[TOTAL_PQM_CHANNELS][MAX_CH_ATTRS];
	uint32_t active_ch;
	uint32_t ext_buff_len;
	int16_t *ext_buff;
};

struct pqm_init_para {
	float dev_global_attr[PQM_DEVICE_ATTR_NUMBER];
	uint32_t dev_ch_attr[TOTAL_PQM_CHANNELS][MAX_CH_ATTRS];
	uint32_t ext_buff_len;
	int16_t *ext_buff;
};

#endif /* __COMMON_DATA_H__ */
