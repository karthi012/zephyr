/*
 * Copyright (c) 2022 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_am18x5

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/timeutil.h>
#include <time.h>

LOG_MODULE_REGISTER(AM18X5, CONFIG_COUNTER_LOG_LEVEL);

#define	RTC_REG_HTH      0x0
#define	RTC_REG_SEC      0x01
#define	RTC_REG_MIN      0x02
#define	RTC_REG_HOUR     0x03
#define	RTC_REG_DATE     0x04
#define	RTC_REG_MONTH    0x05
#define	RTC_REG_YEAR     0x06
#define	RTC_REG_WEEKDAY  0x07
#define	REG_ALM_HTH      0x08
#define	REG_ALM_SEC      0x09
#define	REG_ALM_MIN      0x0A
#define	REG_ALM_HOUR     0x0B
#define	REG_ALM_DATE     0x0C
#define	REG_ALM_MONTH    0x0D
#define	REG_ALM_WEEKDAY  0x0E
#define	RTC_REG_STATUS   0x0F
#define	RTC_REG_CTRL1    0x10
#define	RTC_REG_CTRL2    0x11
#define	RTC_REG_INTMSK   0x12
#define	RTC_REG_OS_CTRL  0x1C
#define	RTC_REG_OS_STAT  0x1D
#define	RTC_REG_CFG      0x1F
#define	RTC_REG_TRICKLE  0x20
#define	RTC_REG_AFCTRL   0x26
#define	RTC_REG_ID0      0x28
#define	RTC_REG_ASTAT    0x2F
#define	RTC_REG_OCTRL    0x30

#define AM18X5_IDENTITY_UP_PART 0x18
#define AM18X5_IDENTITY_LW_PART	0x5
#define TRICKLE_REG_ENABLE	0x9D
#define RTC_REG_CTRL1_WR	BIT(0)
#define RTC_REG_CTRL1_ARST	BIT(2)

struct am18x5_config {
	struct counter_config_info generic;
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpios;

	bool diode;
	uint8_t resistor;
};

struct am18x5_data {
	struct k_sem lock;
	struct tm time_register;
};


int am18x5_rtc_start(const struct device *dev)
{
	return 0;
}

int am18x5_rtc_stop(const struct device *dev)
{
	return 0;
}

int am18x5_rtc_set_value(const struct device *dev, uint32_t ticks)
{
	return 0;
}

int am18x5_rtc_get_value(const struct device *dev, uint32_t *ticks)
{
	return 0;
}

int am18x5_rtc_get_value_64(const struct device *dev, uint64_t *ticks)
{
	return 0;
}

int am18x5_rtc_set_alarm(const struct device *dev, uint8_t chan_id,
						const struct counter_alarm_cfg *alarm_cfg)
{
	return 0;
}

int am18x5_rtc_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	return 0;
}

int am18x5_rtc_set_top_value(const struct device *dev,  const struct counter_top_cfg *cfg)
{
	return 0;
}

uint32_t am18x5_rtc_get_pending_int(const struct device *dev)
{
	return 0;
}

uint32_t am18x5_rtc_get_top_value(const struct device *dev)
{
	return 0;
}

uint32_t am18x5_rtc_get_guard_period(const struct device *dev, uint32_t flags)
{
	return 0;
}

int am18x5_rtc_set_guard_period(const struct device *dev, uint32_t ticks, uint32_t flags)
{
	return 0;
}

uint32_t am18x5_rtc_get_freq(const struct device *dev)
{
	return 0;
}

struct counter_driver_api am18x5_api = {
	.start = am18x5_rtc_start,
	.stop = am18x5_rtc_stop,
	.set_value = am18x5_rtc_set_value,
	.get_value = am18x5_rtc_get_value,
	.set_alarm = am18x5_rtc_set_alarm,
	.cancel_alarm = am18x5_rtc_cancel_alarm,
	.set_top_value = am18x5_rtc_set_top_value,
	.get_pending_int = am18x5_rtc_get_pending_int,
	.get_top_value = am18x5_rtc_get_top_value,
	.get_guard_period = am18x5_rtc_get_guard_period,
	.set_guard_period = am18x5_rtc_set_guard_period,
	.get_freq = am18x5_rtc_get_freq,
};

static int am18x5_init(const struct device *dev)
{
	struct am18x5_data *data = dev->data;
	struct am18x5_config *cfg = dev->config;
	int rc = 0;
	int trickle_cfg = 0;
	uint8_t buf = 0;
	uint8_t part_number[2] = { 0 };

	k_sem_init(&data->lock, 0, 1);

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C device %s is not ready", cfg->i2c.bus->name);
		rc = -ENODEV;
		goto out;
	}

	part_number[0] = RTC_REG_ID0;

	rc = i2c_write_read_dt(&cfg->i2c, &part_number[0], sizeof(part_number[0]),
							part_number, sizeof(part_number));
	if (rc != 0) {
		LOG_ERR("Failed to read Hardware ID register");
		goto out;
	}

	if (part_number[0] != AM18X5_IDENTITY_UP_PART ||
				part_number[1] != AM18X5_IDENTITY_LW_PART) {
		LOG_ERR("Hardware ID mismatch");
		rc = -EINVAL;
		goto out;
	}

	rc = i2c_reg_read_byte_dt(&cfg->i2c, RTC_REG_CTRL1, &buf);
	if (rc != 0) {
		LOG_ERR("Failed to read CTRL1 register");
		goto out;
	}

	WRITE_BIT(buf, 0, 1);

	rc = i2c_reg_write_byte_dt(&cfg->i2c, RTC_REG_CTRL, buf);
	if (rc != 0) {
		LOG_ERR("Failed to configure control register");
		goto out;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, RTC_REG_CFG, TRICKLE_REG_ENABLE);
	if (rc != 0) {
		LOG_ERR("Failed to write configuration key register");
		goto out;
	}

	trickle_cfg |= (RTC_REG_TRICKLE_CHARGE_EN |
			(cfg->diode ? RTC_REG_TC_STANDARD : RTC_REG_TC_SCHOTTKY) |
			cfg->resistor);

	rc = i2_reg_write_byte_dt(&cfg->i2c, RTC_REG_TRICKLE, trickle_cfg);
	if (rc != 0) {
		LOG_ERR("Failed to configure trickle register");
		goto out;
	}

	LOG_INF("RTC_initialised Successfully");

out:
	k_sem_give(&data->lock);
	return rc;
}

#define INST_DT_AM18X5(index)								\
	static struct am18x5_data am18x5_data_##index;					\
											\
	static const struct am18x5_config am18x5_config_##index = {			\
		.generic = {								\
			.max_top_value = UINT32_MAX,					\
			.freq = 1,							\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,				\
			.channels = 2,							\
		},									\
		.i2c = I2C_DT_SPEC_INST_GET(index),					\
		.int_gpios = GPIO_DT_SPEC_INST_GET_OR(index, int_gpios, {0}),		\
		.diode = (IS_ENABLED(DT_INST_PROP(index, tc_diode)) ? 0x8 : 0x4),	\
	};										\
											\
		DEVICE_DT_INST_DEFINE(index, am18x5_init, NULL,				\
			&am18x5_data_##index,						\
			&am18x5_config_##index,						\
			POST_KERNEL,							\
			CONFIG_COUNTER_INIT_PRIORITY,					\
			&am18x5_api);							\

DT_INST_FOREACH_STATUS_OKAY(INST_DT_AM18X5);

