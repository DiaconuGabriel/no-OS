/***************************************************************************//**
 *   @file   gnss_utils.c
 *   @brief  Defines gnss utils for eval-pqmon project.
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
#include "gnss_utils.h"
#include "no_os_error.h"

/* GNSS device descriptor when using no-OS API */
struct no_os_gnss_desc *gnss_desc = NULL;
/* GPIO reset descriptor */
static struct no_os_gpio_desc *gpio_gnss_reset_desc;
/* Platform extra — access to gnss_device from gnss_process_pps */
static struct nmea_ubx_gnss_extra *platform_extra;
/* Precise time populated at each PPS */
struct no_os_gnss_precise_time gnss_precise_time;

int gnss_start(void)
{
	int32_t ret;

	/* Initialize GNSS reset GPIO */
	ret = no_os_gpio_get_optional(&gpio_gnss_reset_desc, &gnss_reset_gpio_ip);
	if (ret) {
		pr_warning("Failed to initialize GNSS reset GPIO (continuing without reset)\n");
		gpio_gnss_reset_desc = NULL;
	}

	if (gpio_gnss_reset_desc) {
		ret = no_os_gpio_direction_output(gpio_gnss_reset_desc, NO_OS_GPIO_HIGH);
		if (ret) {
			pr_err("Failed to set GNSS reset GPIO direction\n");
			goto remove_gnss_reset;
		}
	}

	ret = gnss_init_set_baud(GNSS_UART_BAUDRATE);
	if (ret) {
		pr_err("Failed to initialize GNSS device: %d\n", ret);
		goto remove_gnss_reset;
	}

	pr_info("GNSS device initialized successfully\n\r");
	pr_info("Starting GNSS measurement loop...\n");

	/* PPS output configured automatically via no-OS API init parameters if UBX capable */
	if (platform_extra->gnss_device->device_type == GNSS_DEVICE_UBX_CAPABLE) {
		bool pps_enabled = false;
		ret = gnss_ubx_is_time_pulse_enabled(platform_extra->gnss_device,
						     &pps_enabled);
		if (ret)
			pr_info("PPS configuration complete. Status: unknown (%d)\n", ret);
		else
			pr_info("PPS configuration complete. Status: %s\n",
				pps_enabled ? "ENABLED" : "DISABLED");
	}

	return 0;

remove_gnss_reset:
	if (gpio_gnss_reset_desc)
		no_os_gpio_remove(gpio_gnss_reset_desc);
	pr_err("ERROR\n");
	return ret;
}

int gnss_process_pps(uint32_t *unix_epoch, uint32_t *fractional_data)
{
	int ret = 0;
	bool timing_valid = false;

	if (!unix_epoch || !fractional_data)
		return -EINVAL;

	ret = no_os_gnss_refresh_timing_data(gnss_desc);
	if (ret) {
		pr_err("Failed to refresh timing data: %d\n", ret);
		no_os_irq_enable(platform_extra->gnss_device->irq_ctrl,
				 GNSS_PPS_IRQ_PIN);
		return ret;
	}

	ret = no_os_gnss_is_timing_valid(gnss_desc, &timing_valid);
	if (ret || !timing_valid) {
		pr_warning("Timing data is not valid, waiting for fix...\n");
		no_os_irq_enable(platform_extra->gnss_device->irq_ctrl,
				 GNSS_PPS_IRQ_PIN);
		return -EAGAIN;
	}

	ret = no_os_gnss_get_unified_timing(gnss_desc, &gnss_precise_time);
	if (ret) {
		pr_warning("Failed to get unified timing: %d\n", ret);
		return ret;
	}

	/* Reject fix if time accuracy is invalid or too low (>1ms = no real fix) */
	if (gnss_precise_time.time_accuracy > 1000000) {
		pr_warning("Rejecting fix: accuracy=%lu ns too low\n",
			   (unsigned long)gnss_precise_time.time_accuracy);
		return -EAGAIN;
	}

	no_os_gnss_get_unix_epoch_unified(gnss_desc, unix_epoch, fractional_data);

	return ret;
}

int gnss_init_set_baud(uint32_t target_baud_rate)
{
	int ret;
	struct nmea_ubx_gnss_init_param platform_init_param;
	struct no_os_gnss_init_param no_os_param;
	struct no_os_uart_desc *uart_trans = NULL;
	const uint32_t try_bauds[] = {
		target_baud_rate, 921600, 460800, 230400, 115200, 57600, 38400
	};
	const uint8_t poll[] = {0xB5, 0x62, 0x06, 0x00, 0x00, 0x00, 0x06, 0x18};
	uint32_t found_baud = 0;

	/* Release old descriptor first — gnss_remove() freed gpio_reset internally */
	bool is_reinit = (gnss_desc != NULL);
	if (gnss_desc) {
		no_os_gnss_remove(gnss_desc);
		gnss_desc = NULL;
		gpio_gnss_reset_desc = NULL;
		no_os_gpio_get_optional(&gpio_gnss_reset_desc, &gnss_reset_gpio_ip);
		if (gpio_gnss_reset_desc)
			no_os_gpio_direction_output(gpio_gnss_reset_desc, NO_OS_GPIO_HIGH);
	}

	/* hw_reset with valid (freshly obtained) GPIO descriptor */
	if (gpio_gnss_reset_desc) {
		no_os_gpio_set_value(gpio_gnss_reset_desc, NO_OS_GPIO_LOW);
		no_os_mdelay(100);
		no_os_gpio_set_value(gpio_gnss_reset_desc, NO_OS_GPIO_HIGH);
		no_os_mdelay(1000);
	}

	/* IRQ-driven RX so reads are non-blocking: empty software FIFO returns
	 * -EAGAIN instead of blocking, enabling the poll loop below. */
	uart_gnss_ip.asynchronous_rx = true;

	for (size_t i = 0; i < NO_OS_ARRAY_SIZE(try_bauds); i++) {
		uint32_t baud = try_bauds[i];
		struct no_os_uart_desc *uart_test = NULL;
		bool duplicate = false;
		bool responded = false;

		/* Skip duplicate baudrates */
		for (size_t j = 0; j < i; j++) {
			if (try_bauds[j] == baud) {
				duplicate = true;
				break;
			}
		}
		if (duplicate)
			continue;

		pr_info("GNSS: trying %lu baud\n\r", baud);

		uart_gnss_ip.baud_rate = baud;
		ret = no_os_uart_init(&uart_test, &uart_gnss_ip);
		if (ret)
			continue;

		/* Send CFG-PRT poll at the tested baudrate.
		 * CFG-PRT responds immediately from RAM (no GPS fix dependency),
		 * keeping the per-candidate timeout short. */
		no_os_uart_write(uart_test, poll, sizeof(poll));

		/* Wait for response at the SAME baudrate.
		 * Correct baudrate: module replies with a CFG-PRT frame, detected by
		 * its UBX sync byte 0xB5 (common to every UBX message).
		 * Wrong baudrate: read returns -EAGAIN or garbage without 0xB5.
		 *
		 * With asynchronous_rx the ISR fills rx_fifo; no_os_uart_read drains
		 * it without blocking (returns -EAGAIN when empty). Do NOT use
		 * read_nonblocking here: in this driver that op only re-arms the async
		 * transaction and returns 0, so it never hands back a received byte. */
		for (uint32_t ms = 0; ms < 100 && !responded; ms++) {
			uint8_t ch;
			if (no_os_uart_read(uart_test, &ch, 1) && ch == 0xB5) {
				responded = true;
				break;
			}
			no_os_mdelay(1);
		}

		no_os_uart_remove(uart_test);

		if (responded) {
			found_baud = baud;
			pr_info("GNSS: found at %lu baud\n", found_baud);
			break;
		}
	}

	if (!found_baud) {
		pr_err("GNSS: no baudrate worked\n");
		return -ENODEV;
	}

	/* Scan done. The nmea_ubx driver reads UBX ACKs with blocking reads
	 * (gnss_ubx_receive_packet), incompatible with the -EAGAIN polling above.
	 * Restore blocking RX before any driver-driven config. */
	uart_gnss_ip.asynchronous_rx = false;

	/* Module found at found_baud → config target baud */
	if (found_baud != target_baud_rate) {
		uart_gnss_ip.baud_rate = found_baud;
		ret = no_os_uart_init(&uart_trans, &uart_gnss_ip);
		if (!ret) {
			struct gnss_dev tmp_dev = {0};
			tmp_dev.uart_desc = uart_trans;
			gnss_ubx_set_val_no_ack(&tmp_dev, UBLOX_CFG_UART1_BAUDRATE,
						target_baud_rate, 4,
						GNSS_CONFIG_LAYER_ALL);
			no_os_mdelay(100);
			no_os_uart_remove(uart_trans);
			pr_info("GNSS: switched from %lu to %lu\n",
				found_baud, target_baud_rate);
		}
	}

	uart_gnss_ip.baud_rate = target_baud_rate;
	platform_init_param = (struct nmea_ubx_gnss_init_param) {
		.gnss_init_param = {
			.uart_init = &uart_gnss_ip,
			.gpio_reset = gpio_gnss_reset_desc,
			.device_type = GNSS_DEVICE_UBX_CAPABLE,
			.ubx_input_enable = ENABLE,
			.nmea_input_enable = DISABLE,
			.ubx_output_enable = ENABLE,
			.nmea_output_enable = DISABLE,
			.baud_rate = target_baud_rate
		}
	};
	/* On reinit, skip PPS config — module already outputs PPS with saved
	 * settings, and gnss_init_pps() blocks on UBX ACK when UART recovers. */
	no_os_param = (struct no_os_gnss_init_param) {
		.device_id = 0,
		.pps_config = { .pps_enable   = !is_reinit,
				.frequency = GNSS_PPS_1HZ,
				.pulse_length = GNSS_PPS_LENGTH
			      },
		.platform_ops = &nmea_ubx_gnss_ops,
		.extra = &platform_init_param
	};

	pr_info("GNSS: starting no_os_gnss_init...\n");
	ret = no_os_gnss_init(&gnss_desc, &no_os_param);
	pr_info("GNSS: no_os_gnss_init done, ret=%d\n", ret);
	if (ret) {
		pr_err("GNSS: init failed at target %lu: %d\n",
		       target_baud_rate, ret);
		return ret;
	}

	platform_extra = (struct nmea_ubx_gnss_extra *)gnss_desc->extra;

	return 0;
}
