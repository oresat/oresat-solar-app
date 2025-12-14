/*
 * Copyright (c) 2021 Jimmy Johnson <catch22@fastmail.net>
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TMP101_TMP101_H_
#define ZEPHYR_DRIVERS_SENSOR_TMP101_TMP101_H_

#include <stdint.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor/tmp101.h>

#define TI_TMP101_REG_TEMP		0x00   /** Temperature register */
#define TI_TMP101_REG_CONF		0x01   /** Configuration register */
#define TI_TMP101_REG_LOW_LIMIT		0x02   /** Low alert set register */
#define TI_TMP101_REG_HIGH_LIMIT	0x03   /** High alert set register */

#define AMS_AS6212_CONF                                                                            \
	{.CONF_CR0 = 0x0040,                                                                       \
	 .CONF_CR1 = 0x0080,                                                                       \
	 .CONF_SLEEP = 0x0100,                                                                     \
	 .CONF_M1 = 0x0000,                                                                        \
	 .CONF_TM = 0x0200,                                                                        \
	 .CONF_M0 = 0x8000,                                                                        \
	 .CONF_RST = 0x0080,                                                                       \
	 .TEMP_MULT = 15625,                                                                       \
	 .TEMP_DIV = 2,                                                                            \
	 IF_ENABLED(CONFIG_TMP108_ALERT_INTERRUPTS, (.CONF_POL = 0x0400))}

#define TI_TMP108_CONF                                                                             \
	{.CONF_M0 = 0x0100,                                                                        \
	 .CONF_M1 = 0x0200,                                                                        \
	 .CONF_TM = 0x0400,                                                                        \
	 .CONF_CR0 = 0x2000,                                                                       \
	 .CONF_CR1 = 0x4000,                                                                       \
	 .CONF_RST = 0x0022,                                                                       \
	 .TEMP_MULT = 15625,                                                                       \
	 .TEMP_DIV = 4,                                                                            \
	 IF_ENABLED(CONFIG_TMP108_ALERT_INTERRUPTS,                                                \
		    (.CONF_HYS0 = 0x0010, .CONF_HYS1 = 0x0020, .CONF_POL = 0x0080))}

#define TI_TMP101_CONF                                                                             \
	{.CONF_SD = 0x01,                                                                        \
	 .CONF_TM = 0x02,                                                                        \
	 .CONF_F0 = 0x08,                                                                       \
	 .CONF_F1 = 0x10,                                                                       \
	 .CONF_R0 = 0x20,                                                                       \
	 .CONF_R1 = 0x40,                                                                       \
	 .CONF_OS = 0x80,                                                                       \
	 .CONF_ALERT = 0x80,                                                                       \
	 .CONF_RST = 0x00,                                                                       \
	 .TEMP_MULT = 15625,                                                                       \
	 .TEMP_DIV = 4,                                                                            \
	 IF_ENABLED(CONFIG_TMP101_ALERT_INTERRUPTS, (CONF_POL = 0x04))}

#define TI_TMP101_MODE_CONTINUOUS(x) 0
#define TI_TMP101_MODE_SHUTDOWN(x) TI_TMP101_GET_CONF(x, CONF_SD)
#define TI_TMP101_MODE_ONE_SHOT(x) (TI_TMP101_GET_CONF(x, CONF_SD) | TI_TMP101_GET_CONF(x, CONF_OS))
#define TI_TMP101_MODE_MASK(x)	~(TI_TMP101_GET_CONF(x, CONF_SD) | TI_TMP101_GET_CONF(x, CONF_OS))

#define TI_TMP101_CONF_POL_LOW(x) 0
#define TI_TMP101_CONF_POL_HIGH(x) TI_TMP101_GET_CONF(x, CONF_POL)
#define TI_TMP101_CONF_POL_MASK(x) ~(TI_TMP101_GET_CONF(x, CONF_POL))

#define TI_TMP101_CONF_TM_CMP(x) 0
#define TI_TMP101_CONF_TM_INT(x) TI_TMP101_GET_CONF(x, CONF_TM)
#define TI_TMP101_CONF_TM_MASK(x) ~(TI_TMP101_GET_CONF(x, CONF_TM))

#define TI_TMP101_FAULT_QUEUE_1(x)	0
#define TI_TMP101_FAULT_QUEUE_2(x)	TI_TMP101_GET_CONF(x, CONF_F0)
#define TI_TMP101_FAULT_QUEUE_4(x)	TI_TMP101_GET_CONF(x, CONF_F1)
#define TI_TMP101_FAULT_QUEUE_6(x)	(TI_TMP101_GET_CONF(x, CONF_F1) | \
				TI_TMP101_GET_CONF(x, CONF_F0))
#define TI_TMP101_FAULT_QUEUE_MASK(x) ~(TI_TMP101_GET_CONF(x, CONF_F1) | \
				 TI_TMP101_GET_CONF(x, CONF_F0))

#define TI_TMP101_RES_09bit(x)	0
#define TI_TMP101_RES_10bit(x)	TI_TMP101_GET_CONF(x, CONF_R0)
#define TI_TMP101_RES_11bit(x)	TI_TMP101_GET_CONF(x, CONF_R1)
#define TI_TMP101_RES_12bit(x)	(TI_TMP101_GET_CONF(x, CONF_R1) | \
				TI_TMP101_GET_CONF(x, CONF_R0))
#define TI_TMP101_RES_MASK(x) ~(TI_TMP101_GET_CONF(x, CONF_R1) | \
				 TI_TMP101_GET_CONF(x, CONF_R0))

#define TMP101_TEMP_MULTIPLIER(x)	TI_TMP101_GET_CONF(x, TEMP_MULT)
#define TMP101_TEMP_DIVISOR(x)	TI_TMP101_GET_CONF(x, TEMP_DIV)
#define TMP101_CONF_RST(x)	TI_TMP101_GET_CONF(x, CONF_RST)

#define TI_TMP101_CONF_NA 0x0000

struct tmp_101_reg_def {
	uint8_t CONF_SD;    /** Shutdown mode configuration bit */
	uint8_t CONF_TM;    /** Thermostat mode configuration bit */
	uint8_t CONF_F0; /** Fault queue 0 configuration bit */
	uint8_t CONF_F1;   /** Fault queue 1 configuration bit */
	uint8_t CONF_R0;   /** Resolution 0 configuration bit */
	uint8_t CONF_R1;    /** Resolution 1 configuration bit */
	uint8_t CONF_OS;    /** One shot setting bit */
	uint8_t CONF_ALERT;    /** Alert bit */
	int32_t TEMP_MULT;   /** Temperature multiplier */
	int32_t TEMP_DIV;    /** Temperature divisor */
	uint8_t CONF_RST;   /** default reset values on init */
#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
	uint8_t CONF_POL;  /** Alert pin Polarity configuration bit */
#endif
};

#define TI_TMP101_GET_CONF(x, cfg) ((struct tmp101_config *)(x->config))->reg_def.cfg

struct tmp101_config {
	const struct i2c_dt_spec i2c_spec;
	struct tmp_101_reg_def reg_def;
#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
	const struct gpio_dt_spec alert_gpio;
#endif /* CONFIG_TMP101_ALERT_INTERRUPTS */
};

struct tmp101_data {
	int16_t sample;

	bool one_shot_mode;

#ifdef CONFIG_TMP101_ALERT_INTERRUPTS
	const struct device *tmp101_dev;

	const struct sensor_trigger *temp_alert_trigger;
	sensor_trigger_handler_t temp_alert_handler;

	struct gpio_callback temp_alert_gpio_cb;
#endif /* CONFIG_TMP101_ALERT_INTERRUPTS */
};

int tmp_101_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int tmp101_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);

int ti_tmp101_read_temp(const struct device *dev);
void tmp101_trigger_handle_one_shot(struct k_work *work);
void tmp101_trigger_handle_alert(const struct device *port,
				 struct gpio_callback *cb,
				 gpio_port_pins_t pins);

#endif /*  ZEPHYR_DRIVERS_SENSOR_TMP101_TMP101_H_ */
