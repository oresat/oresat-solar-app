/*
 * Copyright (c) 2021 Jimmy Johnson <catch22@fastmail.net>
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tmp101

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "tmp101.h"

LOG_MODULE_REGISTER(TMP101, CONFIG_SENSOR_LOG_LEVEL);

int tmp101_reg_read(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct tmp101_config *cfg = dev->config;
	int result;

	result = i2c_burst_read_dt(&cfg->i2c_spec, reg, (uint8_t *) val, 2);

	if (result < 0) {
		return result;
	}

	*val = sys_be16_to_cpu(*val);

	return 0;
}

int tmp101_reg_read_config(const struct device *dev, uint8_t *val)
{
	const struct tmp101_config *cfg = dev->config;
	int result;

	result = i2c_burst_read_dt(&cfg->i2c_spec, TI_TMP101_REG_CONF, val, 1);

	if (result < 0) {
		return result;
	}

	return 0;
}

int tmp101_reg_write(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct tmp101_config *cfg = dev->config;
	uint8_t tx_buf[3];
	int result;

	tx_buf[0] = reg;
	sys_put_be16(val, &tx_buf[1]);

	result = i2c_write_dt(&cfg->i2c_spec, tx_buf, sizeof(tx_buf));

	if (result < 0) {
		return result;
	}

	return 0;
}

int tmp101_write_config(const struct device *dev, uint8_t mask, uint8_t conf)
{
	const struct tmp101_config *cfg = dev->config;
	uint8_t config = 0;
	uint8_t tx_buf[2];
	int result;

	result = tmp101_reg_read_config(dev, &config);

	if (result < 0) {
		return result;
	}

	config &= mask;
	config |= conf;

	tx_buf[0] = TI_TMP101_REG_CONF;
	tx_buf[1] = config;

	result = i2c_write_dt(&cfg->i2c_spec, tx_buf, sizeof(tx_buf));

	if (result < 0) {
		return result;
	}

	return 0;
}

int ti_tmp101_read_temp(const struct device *dev)
{
	struct tmp101_data *drv_data = dev->data;
	int result;

	/* clear previous temperature readings */

	drv_data->sample = 0U;

	/* Get the most recent temperature measurement */

	result = tmp101_reg_read(dev, TI_TMP101_REG_TEMP, &drv_data->sample);

	if (result < 0) {
		return result;
	}

	return 0;
}

static int tmp101_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct tmp101_data *drv_data = dev->data;
	uint8_t config, converting_mask;
	int result;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	if (!drv_data->one_shot_mode) {
		/* Read the latest temperature result */
		return ti_tmp101_read_temp(dev);
	}

	/* Trigger the conversion */
	result = tmp101_write_config(dev,
					TI_TMP101_MODE_MASK(dev),
					TI_TMP101_MODE_ONE_SHOT(dev));
	if (result < 0) {
		return result;
	}

	/* Typical conversion time:
	 *   TMP108: 27ms
	 *   AS6212: 36ms
	 *   TMP101: 40ms, for 9 bits of resolution
	 * Maximum conversion time:
	 *   TMP108: 35ms
	 *   AS6212: 51ms
	 *   TMP101: 600ms
	 */
	const uint32_t conv_time_min = 40;
	const uint32_t conv_time_max = 600;
	const uint32_t poll_period = 5;

	k_sleep(K_MSEC(conv_time_min));
	converting_mask = TI_TMP101_MODE_SHUTDOWN(dev);

	for (int i = conv_time_min; i < conv_time_max; i += poll_period) {
		/* Read the config register */
		result = tmp101_reg_read_config(dev, &config);
		if (result < 0) {
			return result;
		}
		if ((config & converting_mask) != 0) {
			/* Conversion has finished */
			LOG_DBG("Conversion complete after %d ms", i);
			return ti_tmp101_read_temp(dev);
		}
		/* Wait before reading again */
		k_sleep(K_MSEC(poll_period));
	}

	/* Conversion timed out */
	return -EAGAIN;
}

static int tmp101_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct tmp101_data *drv_data = dev->data;
	int32_t uval;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	uval = ((int32_t)drv_data->sample * TMP101_TEMP_MULTIPLIER(dev)) / TMP101_TEMP_DIVISOR(dev);
	return sensor_value_from_micro(val, uval);
}

static int tmp101_attr_get(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   struct sensor_value *val)
{
	int result;
	uint8_t tmp_val;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch ((int) attr) {
	case SENSOR_ATTR_CONFIGURATION:
		result =  tmp101_reg_read_config(dev, &tmp_val);
		val->val1 = tmp_val;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return result;
}

static int tmp101_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	struct tmp101_data *drv_data = dev->data;
	const struct tmp101_config *config = dev->config;
	__maybe_unused uint16_t reg_value;
	__maybe_unused int32_t uval;
	uint8_t mode;
	int result = 0;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch ((int) attr) {
#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
	case SENSOR_ATTR_ALERT:
		/* Spec Sheet Errata: TM is set on reset not cleared */
		if (val->val1 == 1) {
			mode = TI_TMP101_CONF_TM_INT(dev);
		} else {
			mode = TI_TMP101_CONF_TM_CMP(dev);
		}

		result = tmp101_write_config(dev,
					     TI_TMP101_CONF_TM_MASK(dev),
					     mode);
		break;

	case SENSOR_ATTR_LOWER_THRESH:
		uval = sensor_value_to_micro(val);
		reg_value = (uval * TMP101_TEMP_DIVISOR(dev)) / TMP101_TEMP_MULTIPLIER(dev);
		result = tmp101_reg_write(dev,
					  TI_TMP101_REG_LOW_LIMIT,
					  reg_value);
		break;

	case SENSOR_ATTR_UPPER_THRESH:
		uval = sensor_value_to_micro(val);
		reg_value = (uval * TMP101_TEMP_DIVISOR(dev)) / TMP101_TEMP_MULTIPLIER(dev);
		result = tmp101_reg_write(dev,
					  TI_TMP101_REG_HIGH_LIMIT,
					  reg_value);
		break;

	case SENSOR_ATTR_TMP101_ALERT_POLARITY:
		if (val->val1 == 1) {
			mode = TI_TMP101_CONF_POL_HIGH(dev);
		} else {
			mode = TI_TMP101_CONF_POL_LOW(dev);
		}
		result = tmp101_write_config(dev,
					     TI_TMP101_CONF_POL_MASK(dev),
					     mode);
		break;
#endif /* CONFIG_TMP101_ALERT_INTERRUPTS */

	case SENSOR_ATTR_RESOLUTION:
      if (val->val1 >= 12) {
         mode = TI_TMP101_RES_12bit(dev);
      } else if (val->val1 >= 11) {
         mode = TI_TMP101_RES_11bit(dev);
      } else if (val->val1 >= 10) {
         mode = TI_TMP101_RES_10bit(dev);
      } else {
         mode = TI_TMP101_RES_09bit(dev);
      }
		result = tmp101_write_config(dev,
					     TI_TMP101_RES_MASK(dev),
					     mode);
		break;

	case SENSOR_ATTR_TMP101_FAULT_QUEUE:
      if (val->val1 >= 6) {
         mode = TI_TMP101_FAULT_QUEUE_6(dev);
      } else if (val->val1 >= 4) {
         mode = TI_TMP101_FAULT_QUEUE_4(dev);
      } else if (val->val1 >= 2) {
         mode = TI_TMP101_FAULT_QUEUE_2(dev);
      } else {
         mode = TI_TMP101_FAULT_QUEUE_1(dev);
      }
		result = tmp101_write_config(dev,
					     TI_TMP101_FAULT_QUEUE_MASK(dev),
					     mode);
		break;

	case SENSOR_ATTR_TMP101_SHUTDOWN_MODE:
		result = tmp101_write_config(dev,
					     TI_TMP101_MODE_MASK(dev),
					     TI_TMP101_MODE_SHUTDOWN(dev));
		drv_data->one_shot_mode = false;
		break;

	case SENSOR_ATTR_TMP101_CONTINUOUS_CONVERSION_MODE:
		result = tmp101_write_config(dev,
					     TI_TMP101_MODE_MASK(dev),
					     TI_TMP101_MODE_CONTINUOUS(dev));
		drv_data->one_shot_mode = false;
		break;

	case SENSOR_ATTR_TMP101_ONE_SHOT_MODE:
		result = tmp101_write_config(dev,
					     TI_TMP101_MODE_MASK(dev),
					     TI_TMP101_MODE_ONE_SHOT(dev));
		drv_data->one_shot_mode = true;
		break;

	default:
		return -ENOTSUP;
	}

	if (result < 0) {
		return result;
	}

	return 0;
}

static DEVICE_API(sensor, tmp101_driver_api) = {
	.attr_set = tmp101_attr_set,
	.attr_get = tmp101_attr_get,
	.sample_fetch = tmp101_sample_fetch,
	.channel_get = tmp101_channel_get,
#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
	.trigger_set = tmp_101_trigger_set,
#endif
};

#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
static int setup_interrupts(const struct device *dev)
{
	struct tmp101_data *drv_data = dev->data;
	const struct tmp101_config *config = dev->config;
	const struct gpio_dt_spec *alert_gpio = &config->alert_gpio;
	int result;

	if (!device_is_ready(alert_gpio->port)) {
		LOG_ERR("tmp101: gpio controller %s not ready",
			alert_gpio->port->name);
		return -ENODEV;
	}

	result = gpio_pin_configure_dt(alert_gpio, GPIO_INPUT);

	if (result < 0) {
		return result;
	}

	gpio_init_callback(&drv_data->temp_alert_gpio_cb,
			   tmp101_trigger_handle_alert,
			   BIT(alert_gpio->pin));

	result = gpio_add_callback(alert_gpio->port,
				   &drv_data->temp_alert_gpio_cb);

	if (result < 0) {
		return result;
	}

	result = gpio_pin_interrupt_configure_dt(alert_gpio,
						 GPIO_INT_EDGE_BOTH);

	if (result < 0) {
		return result;
	}

	return 0;
}
#endif

static int tmp101_init(const struct device *dev)
{
	const struct tmp101_config *cfg = dev->config;
	int result = 0;

	if (!device_is_ready(cfg->i2c_spec.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->i2c_spec.bus->name);
		return -ENODEV;
	}

#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
	struct tmp101_data *drv_data = dev->data;

	/* save this driver instance for passing to other functions */
	drv_data->tmp101_dev = dev;

	result = setup_interrupts(dev);

	if (result < 0) {
		return result;
	}
#endif
	/* clear and set configuration registers back to default values */
	result = tmp101_write_config(dev,
				     0x00,
				     TMP101_CONF_RST(dev));
	return result;
}

#define TMP101_DEFINE(inst, t)                                                                     \
	static struct tmp101_data tmp101_prv_data_##inst##t;                                       \
	static const struct tmp101_config tmp101_config_##inst##t = {                              \
		.i2c_spec = I2C_DT_SPEC_INST_GET(inst),                                            \
		IF_ENABLED(CONFIG_TMP101_ALERT_INTERRUPTS,                                         \
			   (.alert_gpio = GPIO_DT_SPEC_INST_GET(inst, alert_gpios),))              \
		.reg_def = t##_CONF};                                                              \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, &tmp101_init, NULL, &tmp101_prv_data_##inst##t,         \
				     &tmp101_config_##inst##t, POST_KERNEL,                        \
				     CONFIG_SENSOR_INIT_PRIORITY, &tmp101_driver_api);

#define TMP101_INIT(n) TMP101_DEFINE(n, TI_TMP101)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_tmp101
DT_INST_FOREACH_STATUS_OKAY(TMP101_INIT)
