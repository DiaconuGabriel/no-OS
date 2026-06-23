/***************************************************************************//**
 *   @file   rtc_utils.c
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
#include "rtc_utils.h"
#include "common_data.h"
#include "max31343.h"
#include "no_os_util.h"
#include "tmr_regs.h"

/* Last read RTC timestamp — updated by rtc_get_time_burst() */
struct max31343_time_stamp rtc_time_stamp;

/* Pre-computed burst buffer for ISR use */
static uint8_t sync_burst_buf[8];

void rtc_prepare_sync(uint32_t unix_epoch)
{
	time_t t = (time_t)unix_epoch;
	struct tm *tm_info = gmtime(&t);

	if (!tm_info)
		return;

	sync_burst_buf[0] = MAX31343_R_SECONDS;
	sync_burst_buf[1] = no_os_bin2bcd(tm_info->tm_sec);
	sync_burst_buf[2] = no_os_bin2bcd(tm_info->tm_min);
	sync_burst_buf[3] = no_os_bin2bcd(tm_info->tm_hour);
	sync_burst_buf[4] = 0;
	sync_burst_buf[5] = no_os_bin2bcd(tm_info->tm_mday);
	sync_burst_buf[6] = no_os_bin2bcd(tm_info->tm_mon + 1);
	sync_burst_buf[7] = no_os_bin2bcd(tm_info->tm_year % 100);
}

int rtc_trigger_reset(void)
{
	int ret;

	ret = max31343_reg_write(rtc_desc, MAX31343_R_RTC_RESET,
				 MAX31343_F_RTC_RESET_SWRST);
	if (ret)
		return ret;

	ret = max31343_reg_write(rtc_desc, MAX31343_R_RTC_RESET, 0x00);
	if (ret)
		return ret;

	/* Clear OSF and all status flags — read clears all */
	uint8_t status;
	ret = max31343_reg_read(rtc_desc, MAX31343_R_STATUS, &status);
	if (ret)
		return ret;

	ret = max31343_reg_write(rtc_desc, MAX31343_R_CFG1,
				 MAX31343_F_CFG1_ENOSC);
	if (ret)
		return ret;

	ret = max31343_reg_write(rtc_desc, MAX31343_R_INT_EN, 0x00);
	if (ret)
		return ret;

	return 0;
}

int rtc_trigger_write(void)
{
	int ret;

	ret = no_os_i2c_write(rtc_desc->i2c_desc, sync_burst_buf, 8, 1);

	return ret;
}

int rtc_get_time_burst(uint32_t *rtc_time)
{
	struct tm tm_info;
	uint8_t buf[7];
	uint8_t reg = MAX31343_R_SECONDS;
	int ret;

	ret = no_os_i2c_write(rtc_desc->i2c_desc, &reg, 1, 0);
	if (ret)
		return ret;

	ret = no_os_i2c_read(rtc_desc->i2c_desc, buf, 7, 1);
	if (ret)
		return ret;

	rtc_time_stamp.sec  = no_os_bcd2bin(buf[0]);
	rtc_time_stamp.min  = no_os_bcd2bin(buf[1]);
	rtc_time_stamp.hr   = no_os_bcd2bin(buf[2]);
	/* buf[3] = DAY of week, skip */
	rtc_time_stamp.day  = no_os_bcd2bin(buf[4]);
	rtc_time_stamp.mon  = no_os_bcd2bin(buf[5]);
	rtc_time_stamp.year = no_os_bcd2bin(buf[6]) + 2000;

	tm_info.tm_sec  = rtc_time_stamp.sec;
	tm_info.tm_min  = rtc_time_stamp.min;
	tm_info.tm_hour = rtc_time_stamp.hr;
	tm_info.tm_mday = rtc_time_stamp.day;
	tm_info.tm_mon  = rtc_time_stamp.mon - 1;
	tm_info.tm_year = rtc_time_stamp.year - 1900;
	tm_info.tm_isdst = -1;

	*rtc_time = (uint32_t)mktime(&tm_info);
	return 0;
}