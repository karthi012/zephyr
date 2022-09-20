/*
 * Copyright (c) 2022 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_am18x5 

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>
#include <time.h>
#include "rtc_am18x5.h" 

LOG_MODULE_REGISTER(AM18X5, CONFIG_COUNTER_LOG_LEVEL);

#define RTC_REG_CTRL1_WR                BIT(0) 
#define RTC_REG_CTRL1_ARST              BIT(2)
#define RTC_REG_CTRL1_12_24             BIT(6)

#define RTC_REG_TRICKLE_CHARGE_EN       0xa0
#define RTC_REG_TC_STANDARD             0x8
#define RTC_REG_TC_SCHOTTKY             0x4

#define RTC_TIME_REG_SIZE	sizeof(struct am18x5_time_registers)

#define MAX_WRITE_SIZE		RTC_TIME_REG_SIZE

#define UNIX_YEAR_OFFSET	69

#define RTC_BCD_DECODE(reg_prefix) (reg_prefix##_one + reg_prefix##_ten * 10)

struct am18x5_config {
	struct counter_config_info generic;
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpios;
	uint8_t addr;

	/* trickle charge settings */
	bool diode;
	uint8_t resistor;
};

struct am18x5_data { 
	struct device *am18x5;
	struct k_sem lock;
	struct am18x5_time_registers registers;
};

static time_t decode_rtc(const struct device *dev)
{
	struct am18x5_data *data = dev->data;
	time_t time_unix = 0;
	struct tm time = { 0 };
	
	time.tm_sec = RTC_BCD_DECODE(data->registers.rtc_sec.sec);
        time.tm_sec = RTC_BCD_DECODE(data->registers.rtc_min.min);
        time.tm_hour = RTC_BCD_DECODE(data->registers.rtc_hours.hr);
        time.tm_mday = RTC_BCD_DECODE(data->registers.rtc_date.date);
        time.tm_wday = data->registers.rtc_weekday.weekday;
        /* tm struct starts months at 0, AM18X5 starts at 1*/
        time.tm_mon = RTC_BCD_DECODE(data->registers.rtc_month.month) - 1;
        /* tm struct uses years since 1900 but unix time uses year since 1970 */
        time.tm_year = RTC_BCD_DECODE(data->registers.rtc_year.year) +
                UNIX_YEAR_OFFSET;

        time_unix = timeutil_timegm(&time);

        LOG_DBG("Unix time is %d\n", (uint32_t)time_unix);

        return time_unix;
}

static int encode_rtc(const struct device *dev, struct tm *time_buffer)
{
        struct am18x5_data *data = dev->data;
        uint8_t month;
        uint8_t year_since_epoch;

        /* In a tm struct, months start at 0, mcp7940n starts with 1 */
        month = time_buffer->tm_mon + 1;

        if (time_buffer->tm_year < UNIX_YEAR_OFFSET) {
                return -EINVAL;
        }
        year_since_epoch = time_buffer->tm_year - UNIX_YEAR_OFFSET;

        /* Set external oscillator configuration bit */
        data->registers.rtc_sec.start_osc = 1;

        data->registers.rtc_sec.sec_one = time_buffer->tm_sec % 10;
        data->registers.rtc_sec.sec_ten = time_buffer->tm_sec / 10;
        data->registers.rtc_min.min_one = time_buffer->tm_min % 10;
        data->registers.rtc_min.min_ten = time_buffer->tm_min / 10;
        data->registers.rtc_hours.hr_one = time_buffer->tm_hour % 10;
        data->registers.rtc_hours.hr_ten = time_buffer->tm_hour / 10;
        data->registers.rtc_weekday.weekday = time_buffer->tm_wday;
        data->registers.rtc_date.date_one = time_buffer->tm_mday % 10;
        data->registers.rtc_date.date_ten = time_buffer->tm_mday / 10;
        data->registers.rtc_month.month_one = month % 10;
        data->registers.rtc_month.month_ten = month / 10;
        data->registers.rtc_year.year_one = year_since_epoch % 10;
        data->registers.rtc_year.year_ten = year_since_epoch / 10;

	return 0;
}

static int write_data_block(const struct device *dev, uint8_t addr, uint8_t size)
{
	struct am18x5_data *data = dev->data;
	const struct am18x5_config *cfg = dev->config;
	int rc = 0;
	uint8_t time_data[MAX_WRITE_SIZE + 1]; 
	uint8_t *write_block_start;

	if (size > MAX_WRITE_SIZE) {
		return -EINVAL;
	}

	if (addr == RTC_REG_SEC) {
		write_block_start = (uint8_t *)&data->registers;
	} else {
		return -EINVAL;
	}

	time_data[0] = addr;
	memcpy(&time_data[1], write_block_start, size);

	rc = i2c_write(cfg->i2c, time_data, size + 1, cfg->addr);

	return rc;
}

static int am18x5_start(const struct device *dev)
{
	struct am18x5_config *cfg = dev->config;
	struct am18x5_data *data = dev->data;
	uint8_t buf = 0;
	int rc = 0;

	k_sem_take(&data->lock, K_FOREVER);
	
	rc = i2c_reg_read_byte(dev,  cfg->addr, RTC_REG_CTRL1, &buf);
	if(rc < 0) {
		LOG_ERR("Failed to read control register");
		goto out;
	}
	/* Clear the STOP Bit in Control1 register starts the RTC*/
	WRITE_BIT(buf, 7, 0);

	rc = i2c_reg_write_byte(dev, cfg->addr,  RTC_REG_CTRL1, &buf);
	if(rc != 0) {
		LOG_ERR("Failed to write control register");
		goto out;
	}

out:
	k_sem_give(&data->lock);

	return rc;
}

static int am18x5_stop(const struct device *dev)
{
	struct am18x5_data *data = dev->data;
	struct am18x5_config *cfg = dev->config;
	uint8_t buf = 0;
	int rc = 0;

	k_sem_take(&data->lock, K_FOREVER);

	rc = i2c_reg_read_byte(dev, cfg->addr, RTC_REG_CTRL1, &buf);
	if(rc != 0) {
		LOG_ERR("Failed to read control register");
		goto out;
	}

	WRITE_BIT(buf, 7, 1);

	rc = i2c_reg_read_byte(dev, cfg->addr, RTC_REG_CTRL1, &buf);
	if(rc != 0) {
		LOG_ERR("Failed to write Control register");
		goto out;
	};

out:
	k_sem_give(&data->lock);

	return rc;
}

static int read_time(const struct device *dev, time_t *unix_time)
{
	struct am18xt_data *data = dev->data;
	const struct am18x5_config *cfg = dev->config;
	uint8_t addr = RTC_REG_SEC;
	int rc = 0;

	rc = i2c_write_read(cfg->i2c, cfg->addr, sizeof(addr), &data->registers,
				RTC_TIME_REG_SIZE);
	
	if(rc >= 0) {
		*unix_time = decode_rtc(dev);
	}

	return rc;
}	

	
static int am18x5_counter_get_value(const struct device *dev, uint32_t *ticks)
{
	struct am18x5_data *data = dev->data;
	time_t unix_time;
	int rc = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* Get time */
	rc = read_time(dev, &unix_time);
	
	if(rc >= 0) {
		*ticks = unix_time;
	}

	k_sem_give(&data->lock);

	return rc;
}

static int am18x5_counter_set_alarm(const struct device *dev)
{
	return 0 ;
}

static int am18x5_counter_cancel_alarm(const struct device *dev)
{
	 return 0; 
}

static int am18x5_counter_set_top_value(const struct device *dev)
{
	return 0;
}

static int am18x5_counter_get_pending_int(const struct device *dev)
{
	return 0;
}

static uint32_t am18x5_counter_get_top_value(const struct device *dev)
{
        return UINT32_MAX;
}


int set_time(const struct device *dev, time_t unix_time)
{
	struct am18x5_config *cfg = dev->cfg;
	struct am18x5_data *data = dev->data;
	struct tm time_buffer = { 0 } ;
	int rc = 0;
	uint8_t buf = 0;
	
	if(unix_time > UINT32_MAX) {
		LOG_ERR("Unix time must be 32-bit");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	gmtime_r(&unix_time, &time_buffer);

	LOG_DBG("Desired time is %d-%d-%d %d:%d:%d\n", (time_buffer.tm_year + 1900),
                (time_buffer.tm_mon + 1), time_buffer.tm_mday, time_buffer.tm_hour,
                time_buffer.tm_min, time_buffer.tm_sec);

	rc = encode_rtc(dev, &time_buffer);
	if(rc < 0) {
		goto out;
	}

	rc = i2c_reg_read_byte(dev, cfg->addr, RTC_REG_CTRL1, &buf);
	if(rc != 0) {
		LOG_ERR("Failed to read the Control register");
		goto out;
	}	
		
	WRITE_BIT(buf, 0, 1);

	rc = i2c_reg_write_byte(dev, cfg->addr, RTC_REG_CTRL1, buf);
	if(rc != 0) {
		LOG_ERR("Failed to write Control register");
		goto out;
	}

	rc = write_data_block(dev, RTC_REG_SEC, RTC_TIME_REG_SIZE);
	if(rc < 0) {
		LOG_ERR("Failed to update time register");
		goto out;
	}

	WRITE_BIT(buf, 0, 0);

	rc = i2c_reg_write_byte(dev, cfg->addr, RTC_REG_CTRL1, buf);
	if(rc != 0) {
		LOG_WRN("Failed to write Control register");
		LOG_WRN("Not possible to prevent inadvertent access counters register");
	}

out:
	k_sem_give(&data->lock);

	return rc;
}	

static int am18x5_init(const struct device *dev)
{
	struct am18x5_data *data = dev->data;
	const struct am18x5_config *cfg = dev->config;
	int rc = 0;
	time_t unix_time = 0;
	int trickle_cfg = -EINVAL;
	uint8_t buf = 0;
	uint8_t part_number[2] = { 0 };
	
	printk("RTC initialise satred\n");
	k_sem_init(&data->lock, 0, 1);

	if(!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C device %s is not ready", cfg->i2c.bus->name);
		rc = -ENODEV;
		goto out;
	}

	rc = i2c_write_read(cfg->i2c, cfg->addr, RTC_REG_ID0, 2, part_number, sizeof(part_number));
	if(rc != 0){
		LOG_ERR("Failed to read Hardware ID register");
		rc = -EIO;
		goto out;
	}

	if(part_number[0] != AM18X5_IDENTITY_UP_PART || part_number[1] != AM18X5_IDENTITY_LW_PART){
		LOG_ERR("Hardware ID mismatch!");
		rc = -EINVAL;
		goto out;
	}

	rc = i2c_reg_read_byte(cfg->i2c, cfg->addr, RTC_REG_CTRL1, &buf);
	if(rc != 0) {
		LOG_ERR("Failed to read CTRL1 register");
		rc = -EIO;
		goto out;
	}

	buf |= ((buf ~(RTC_REG_CTRL1_12_24 | RTC_REG_CTRL1_ARST)) | RTC_REG_CTRL1_WR); 

	rc = i2c_reg_write_byte(cfg->i2c, cfg->addr, RTC_REG_CFG, buf); 
	if(rc != 0) {
		LOG_ERR("Failed to write configuration key register");
		rc = -EIO;
		goto out;
	}

	trickle_cfg |= (RTC_REG_TRICKLE_CHARGE_EN |
                       (cfg->diode ? RTC_REG_TC_STANDARD : RTC_REG_TC_SCHOTTKY) |
                       cfg->resistor);
	rc = i2c_reg_write_byte(cfg->i2c,cfg->addr, RTC_REG_TRICKLE, trickle_cfg);
	if(rc != 0) {
		LOG_ERR("Failed to write trickle regiter");
		rc = -EIO;
		goto out;
	}

	printk("RTC Initialised Successfully\n");
	LOG_INF("RTC Initialised Successfully");

out:
	k_sem_give(&data->lock);
}

static const struct counter_driver_api  am18x5_api = {
        .start = am18x5_counter_start,
        .stop = am18x5_counter_stop,
        .get_value = am18x5_counter_get_value,
        .set_alarm = am18x5_counter_set_alarm,
        .cancel_alarm = am18x5_counter_cancel_alarm,
        .set_top_value = am18x5_counter_set_top_value,
        .get_pending_int = am18x5_counter_get_pending_int,
        .get_top_value = am18x5_counter_get_top_value,
};

#define INST_DT_AM18X5(index)                                                   \
                                                                                        \
        static struct am18x5_data am18x5_data_##index;                              \
                                                                                        \
        static const struct am18x5_config am18x5_config_##index = {                 \
                .generic = {                                                            \
                        .max_top_value = UINT32_MAX,                                    \
                        .freq = 1,                                                      \
                        .flags = COUNTER_CONFIG_INFO_COUNT_UP,                          \
                        .channels = 2,                                                  \
                },                                                                      \
                .i2c = I2C_DT_SPEC_INST_GET(index),                                     \
		.addr = DT_INST_REG_ADDR(index),					\
                .int_gpios = GPIO_DT_SPEC_INST_GET_OR(index, int_gpios, {0}),   	\
		.diode = (IS_ENABLED(DT_INST_PROP(index, tc_diode)), ? 0x8 : 0x4),	\
		.resistor = DT_INST_PROP(index, tc_resistor),				\
        };                                                                              \
                                                                                        \
        DEVICE_DT_INST_DEFINE(index, am18x5_init, NULL,                               \
                    &am18x5_data_##index,                                             \
                    &am18x5_config_##index,                                           \
                    POST_KERNEL,                                                        \
                    CONFIG_COUNTER_INIT_PRIORITY,                                       \
                    &am18x5_api);

DT_INST_FOREACH_STATUS_OKAY(INST_DT_AM18X5);

