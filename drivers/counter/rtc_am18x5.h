/*
 * Copyright (c) 2022 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTC_AM18X5_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTC_AM18X5_H_

#include <sys/timeutil.h>
#include <time.h>

struct am18x5_rt_sec {
	uint8_t sec_one : 4;
	uint8_t sec_ten : 3;
	uint8_t unused : 1;
} __packed;

struct am18x5_rtc_min {
	uint8_t min_one : 4;
	uint8_t min_ten : 3;
	uint8_t unused : 1;
} __packed;

struct am18x5_rtc_hours {
	uint8_t hr_one : 4;
	uint8_t hr_ten : 2;
	uint8_t twelve_hr : 1;
	uint8_t unused : 1;
} __packed;

struct am18x5_rtc_date {
	uint8_t date_one : 4;
	uint8_t date_ten : 2;
	uint8_t unused : 2;
} __packed;

struct am18x5_rtc_month {
	uint8_t month_one : 4;
	uint8_t month_ten : 1;
	uint8_t unused : 3;
} __packed;

struct am18x5_rtc_year {
	uint8_t year_one : 4;
	uint8_t year_ten : 4;
} __packed;

struct am18x5_rtc_weekday {
	uint8_t weekday : 3;
	uint8_t unused : 5;
} __packed;

struct am18x5_time_registers {
	struct am18x5_rtc_sec rtc_sec;
	struct am18x5_rtc_min rtc_min;
	struct am18x5_rtc_hours rtc_hours;
	struct am18x5_rtc_date rtc_date;
	struct am18x5_rtc_month rtc_month;
	struct am18x5_rtc_year rtc_year;
	struct am18x5_rtc_weekday rtc_weekday;
};

enum am18x_registers {
	REG_RTC_HTH	= 0x0,
	REG_RTC_SEC	= 0x01,
	REG_RTC_MIN	= 0x02,
	REG_RTC_HOUR	= 0x03,
	REG_RTC_DATE	= 0x04,
	REG_RTC_MONTH	= 0x05,
	REG_RTC_YEAR	= 0x06,
	REG_RTC_WEEKDAY	= 0x07,
	REG_ALM_HTH	= 0x08,
	REG_ALM_SEC	= 0x09,
	REG_ALM_MIN	= 0x0A,
	REG_ALM_HOUR	= 0x0B,
	REG_ALM_DATE	= 0x0C,
	REG_ALM_MONTH	= 0x0D,
	REG_ALM_WEEKDAY	= 0x0E,
	REG_RTC_STATUS	= 0x0F,
	REG_RTC_CTRL1	= 0x10,
	REG_RTC_CTRL2	= 0x11,
	REG_RTC_INTMSK	= 0x12,
	REG_RTC_OS_CTRL	= 0x1C,
	REG_RTC_OS_STAT	= 0x1D,
	REG_RTC_CFG	= 0x1F,
	REG_RTC_TRICKLE	= 0x20,
	REG_RTC_AFCTRL	= 0x26,
	REG_RTC_ID0	= 0x28,
	REG_RTC_ASTAT	= 0x2F,
	REG_RTC_OCTRL	= 0x30,
};

int am18x5_rtc_set_time(const struct device *dev, time_t unix_time);

