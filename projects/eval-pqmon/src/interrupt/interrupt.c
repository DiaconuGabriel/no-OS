/***************************************************************************//**
 *   @file   interrupt.c
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

#include <stdbool.h>
#include "interrupt.h"
#include "common_data.h"
#include "no_os_irq.h"
#include "no_os_gpio.h"
#include "no_os_error.h"
#include "rtc_utils.h"
#include "pps_utils.h"
#include "tmr_regs.h"
#include "tmr.h"
#include "maxim_irq.h"

static volatile uint8_t gnss_pps_flag;
static volatile uint8_t rtc_pps_flag;
static volatile uint8_t wait_3_rtc_pps_count = 2;
volatile sync_state_t sync_state = SYNC_DONE;

static volatile uint32_t rmsonerdy_hw_ts = 0;
static volatile bool rmsonerdy_hw_ts_valid = false;
static struct no_os_irq_ctrl_desc *rmsonerdy_irq_desc;
static struct no_os_gpio_desc     *rmsonerdy_gpio_desc;

static struct no_os_irq_ctrl_desc *gnss_nvic_desc;
static struct no_os_callback_desc  rtc_pps_cb;
static struct no_os_irq_ctrl_desc *rtc_pps_irq_desc;
static struct no_os_irq_ctrl_desc *rtc_nvic_desc;
static struct no_os_gpio_desc     *rtc_pps_gpio_desc;
static struct no_os_irq_ctrl_desc *sync_timer_nvic_desc;

static void rmsonerdy_isr(void *context)
{
	rmsonerdy_hw_ts = timer_read_count();
	rmsonerdy_hw_ts_valid = true;
}

bool consume_rmsonerdy_ts(uint32_t *out_ts)
{
	if (!rmsonerdy_hw_ts_valid)
		return false;
	*out_ts = rmsonerdy_hw_ts;
	rmsonerdy_hw_ts_valid = false;
	return true;
}

static void gnss_pps_cb_fn(void *context)
{
	uint32_t now = timer_read_count();
	gnss_pps_flag = 1;
	capture_pps_gnss(now);
}

static void rtc_pps_cb_fn(void *context)
{
	uint32_t now = timer_read_count();

	switch (sync_state) {
	case SYNC_PEND_WR:
		capture_pps_rtc(now);
		rtc_trigger_write();
		/* The MAX31343 commits the written time one second later (datasheet:
		 * "Time will be updated 1 second after setting registers"), and the
		 * divider re-locks (+4ms transient) at that commit. */
		wait_3_rtc_pps_count = 2;
		sync_state = SYNC_SETTLE;
		break;

	case SYNC_SETTLE:
		wait_3_rtc_pps_count--;
		if (wait_3_rtc_pps_count == 0)
			sync_state = SYNC_DONE;
		break;

	default:
		rtc_pps_flag = 1;
		capture_pps_rtc(now);
		break;
	}
}

uint8_t get_gnss_pps_state(void)
{
	return gnss_pps_flag;
}

void reset_gnss_pps_flag(void)
{
	gnss_pps_flag = 0;
}

uint8_t get_rtc_pps_state(void)
{
	return rtc_pps_flag;
}

void reset_rtc_pps_flag(void)
{
	rtc_pps_flag = 0;
}

int rtc_pps_irq_enable(void)
{
	if (!rtc_pps_irq_desc)
		return -ENODEV;

	return no_os_irq_enable(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN);
}

int rtc_pps_irq_disable(void)
{
	if (!rtc_pps_irq_desc)
		return -ENODEV;

	return no_os_irq_disable(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN);
}

int gnss_pps_interrupt_init(struct gnss_dev *gnss_dev)
{
	int ret;
	struct no_os_gpio_desc *irq_pin;
	struct no_os_irq_ctrl_desc *gnss_irq_desc;

	if (!gnss_dev)
		return -ENODEV;

	ret = no_os_gpio_get(&irq_pin, &gnss_pps_gpio_ip);
	if (ret)
		return ret;

	ret = no_os_gpio_direction_input(irq_pin);
	if (ret)
		return ret;

	ret = no_os_irq_ctrl_init(&gnss_irq_desc, &gnss_pps_irq_ip);
	if (ret)
		return ret;

	struct no_os_callback_desc p2_cb = {
		.callback   = gnss_pps_cb_fn,
		.ctx        = gnss_irq_desc,
		.event      = NO_OS_EVT_GPIO,
		.peripheral = NO_OS_GPIO_IRQ,
		.handle     = NULL
	};

	ret = no_os_irq_register_callback(gnss_irq_desc, GNSS_PPS_IRQ_PIN, &p2_cb);
	if (ret)
		return ret;

	ret = no_os_irq_trigger_level_set(gnss_irq_desc, GNSS_PPS_IRQ_PIN,
					  NO_OS_IRQ_EDGE_RISING);
	if (ret)
		return ret;

	ret = no_os_irq_set_priority(gnss_irq_desc, GNSS_PPS_IRQ_PIN, 2);
	if (ret)
		return ret;

	gnss_dev->irq_ctrl = gnss_irq_desc;
	gnss_dev->irq_gpio = irq_pin;
	gnss_dev->irq_cb   = p2_cb;

	ret = no_os_irq_ctrl_init(&gnss_nvic_desc, &gnss_nvic_ip);
	if (ret)
		return ret;

	ret = no_os_irq_set_priority(gnss_nvic_desc, NVIC_GNSS_PPS_IRQ, 1);
	if (ret)
		return ret;

	ret = no_os_irq_enable(gnss_nvic_desc, NVIC_GNSS_PPS_IRQ);
	if (ret)
		return ret;

	ret = no_os_irq_enable(gnss_irq_desc, GNSS_PPS_IRQ_PIN);
	if (ret)
		return ret;

	reset_gnss_pps_flag();

	pr_info("GNSS PPS interrupt initialized (P%d.%d)\n\r",
		GNSS_PPS_IRQ_PORT, GNSS_PPS_IRQ_PIN);

	return 0;
}

int gnss_pps_interrupt_remove(struct gnss_dev *gnss_dev)
{
	int ret;

	if (!gnss_dev)
		return -ENODEV;

	ret = no_os_irq_disable(gnss_dev->irq_ctrl, GNSS_PPS_IRQ_PIN);
	if (ret)
		return ret;

	ret = no_os_irq_unregister_callback(gnss_dev->irq_ctrl, GNSS_PPS_IRQ_PIN,
					    &gnss_dev->irq_cb);
	if (ret)
		return ret;

	ret = no_os_irq_ctrl_remove(gnss_dev->irq_ctrl);
	if (ret)
		return ret;

	ret = no_os_gpio_remove(gnss_dev->irq_gpio);
	if (ret)
		return ret;

	if (gnss_nvic_desc) {
		no_os_irq_ctrl_remove(gnss_nvic_desc);
		gnss_nvic_desc = NULL;
	}

	return 0;
}

int rtc_pps_interrupt_init(void)
{
	int ret;

	ret = no_os_gpio_get(&rtc_pps_gpio_desc, &rtc_pps_gpio_ip);
	if (ret)
		return ret;

	ret = no_os_gpio_direction_input(rtc_pps_gpio_desc);
	if (ret)
		return ret;

	ret = no_os_irq_ctrl_init(&rtc_pps_irq_desc, &rtc_pps_irq_ip);
	if (ret)
		return ret;

	rtc_pps_cb = (struct no_os_callback_desc) {
		.callback   = rtc_pps_cb_fn,
		.ctx        = rtc_pps_irq_desc,
		.event      = NO_OS_EVT_GPIO,
		.peripheral = NO_OS_GPIO_IRQ,
		.handle     = NULL,
	};

	ret = no_os_irq_register_callback(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN,
					  &rtc_pps_cb);
	if (ret)
		return ret;

	ret = no_os_irq_trigger_level_set(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN,
					  NO_OS_IRQ_EDGE_RISING);
	if (ret)
		return ret;

	ret = no_os_irq_set_priority(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN, 2);
	if (ret)
		return ret;

	ret = no_os_irq_ctrl_init(&rtc_nvic_desc, &rtc_nvic_ip);
	if (ret)
		return ret;

	ret = no_os_irq_set_priority(rtc_nvic_desc, NVIC_RTC_PPS_IRQ, 1);
	if (ret)
		return ret;

	ret = no_os_irq_enable(rtc_nvic_desc, NVIC_RTC_PPS_IRQ);
	if (ret)
		return ret;

	ret = no_os_irq_enable(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN);
	if (ret)
		return ret;

	reset_rtc_pps_flag();

	pr_info("RTC PPS interrupt initialized (P%d.%d)\n\r",
		RTC_PPS_IRQ_PORT, RTC_PPS_IRQ_PIN);

	return 0;
}

int rtc_pps_interrupt_remove(void)
{
	if (rtc_pps_irq_desc) {
		no_os_irq_disable(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN);
		no_os_irq_unregister_callback(rtc_pps_irq_desc, RTC_PPS_IRQ_PIN,
					      &rtc_pps_cb);
		no_os_irq_ctrl_remove(rtc_pps_irq_desc);
		rtc_pps_irq_desc = NULL;
	}

	if (rtc_pps_gpio_desc) {
		no_os_gpio_remove(rtc_pps_gpio_desc);
		rtc_pps_gpio_desc = NULL;
	}

	return 0;
}

static void sync_timer_oneshot_cb_fn(void *context)
{
	no_os_irq_disable(sync_timer_nvic_desc, RTC_SYNC_TIMER_IRQ);

	rtc_pps_irq_disable();

	rtc_trigger_reset();

	pps_ctx = PPS_SYNC;
	sync_state = SYNC_PEND_WR;
	rtc_pps_irq_enable();
}

int rtc_sync_timer_init(void)
{
	int ret;
	static struct no_os_callback_desc sync_timer_cb_desc = {
		.callback   = sync_timer_oneshot_cb_fn,
		.ctx        = NULL,
		.event      = NO_OS_EVT_TIM_ELAPSED,
		.peripheral = NO_OS_TIM_IRQ,
		.handle     = NULL,
	};

	ret = no_os_irq_ctrl_init(&sync_timer_nvic_desc, &rtc_sync_timer_nvic_ip);
	if (ret)
		return ret;

	ret = no_os_irq_register_callback(sync_timer_nvic_desc, RTC_SYNC_TIMER_IRQ,
					  &sync_timer_cb_desc);
	if (ret)
		return ret;

	mxc_tmr_cfg_t cfg = {
		.pres    = MXC_TMR_PRES_1,
		.mode    = MXC_TMR_MODE_ONESHOT,
		.cmp_cnt = pps_real_freq_hz,
		.pol     = 0,
	};
	MXC_TMR_Init(RTC_SYNC_TIMER_REGS, &cfg);

	pr_info("RTC sync timer initialized\n\r");

	return 0;
}

int rmsonerdy_interrupt_init(void)
{
	int ret;

	static const struct no_os_gpio_init_param rmsonerdy_gpio_ip = {
		.port         = INTR_GPIO_PORT_NUM,
		.number       = INTR_GPIO_PIN_NUM,
		.pull         = NO_OS_PULL_DOWN,
		.platform_ops = &max_gpio_ops,
		.extra        = INTR_GPIO_EXTRA,
	};

	static const struct no_os_irq_init_param rmsonerdy_irq_ip = {
		.irq_ctrl_id  = INTR_GPIO_IRQ_ID,
		.platform_ops = INTR_OPS,
	};

	ret = no_os_gpio_get(&rmsonerdy_gpio_desc, &rmsonerdy_gpio_ip);
	if (ret)
		return ret;

	ret = no_os_gpio_direction_input(rmsonerdy_gpio_desc);
	if (ret)
		return ret;

	ret = no_os_irq_ctrl_init(&rmsonerdy_irq_desc, &rmsonerdy_irq_ip);
	if (ret)
		return ret;

	static struct no_os_callback_desc rmsonerdy_cb = {
		.callback   = rmsonerdy_isr,
		.ctx        = NULL,
		.event      = NO_OS_EVT_GPIO,
		.peripheral = NO_OS_GPIO_IRQ,
		.handle     = NULL,
	};

	ret = no_os_irq_register_callback(rmsonerdy_irq_desc, INTR_GPIO_PIN_NUM,
					  &rmsonerdy_cb);
	if (ret)
		return ret;

	ret = no_os_irq_trigger_level_set(rmsonerdy_irq_desc, INTR_GPIO_PIN_NUM,
					  NO_OS_IRQ_EDGE_FALLING);
	if (ret)
		return ret;

	ret = no_os_irq_set_priority(rmsonerdy_irq_desc, INTR_GPIO_PIN_NUM, 2);
	if (ret)
		return ret;

	ret = no_os_irq_enable(rmsonerdy_irq_desc, INTR_GPIO_PIN_NUM);
	if (ret)
		return ret;

	pr_info("RMSONERDY interrupt initialized (P%d.%d)\n\r",
		INTR_GPIO_PORT_NUM, INTR_GPIO_PIN_NUM);

	return 0;
}

void arm_rtc_sync(uint32_t next_epoch)
{
	uint32_t tick_gnss;
	uint32_t period = pps_real_freq_hz;
	uint32_t now;
	uint32_t elapsed_from_pps;
	uint32_t target_total;
	int32_t delay;

	pps_get_raw_timestamps(&tick_gnss, NULL);
	rtc_prepare_sync(next_epoch);

	now = timer_read_count();
	elapsed_from_pps = now - tick_gnss;
	target_total = (period / 2) - (period * 59 / 10000);
	delay = (int32_t)(target_total - elapsed_from_pps);
	if (delay <= 0)
		delay += period;

	/* Raw register access (intentional, not portable): the no-OS timer API
	 * has no compare-register setter — the threshold can only be set at
	 * init(), and re-arming via remove()+init() would heap-alloc on every
	 * PPS in this timing-critical path. We reprogram the sync timer's compare
	 * (cmp) each PPS, so direct register writes are required. The whole sequence
	 * runs with IRQs disabled to stay atomic w.r.t. the next PPS edge. */
	__disable_irq();
	RTC_SYNC_TIMER_REGS->cn  &= ~MXC_F_TMR_CN_TEN;
	RTC_SYNC_TIMER_REGS->cnt  = 0x1;
	RTC_SYNC_TIMER_REGS->cmp  = (uint32_t)delay;
	RTC_SYNC_TIMER_REGS->intr = MXC_F_TMR_INTR_IRQ;
	NVIC_EnableIRQ(RTC_SYNC_TIMER_IRQ);
	RTC_SYNC_TIMER_REGS->cn  |= MXC_F_TMR_CN_TEN;
	__enable_irq();
}

