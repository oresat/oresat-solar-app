/**
 * i2c_sensor.c
 *
 * Original code came from zephyr/samples/sensor/bme280.
 *
 * Modified slightly to run as a thread and use logging.
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h> 
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oresat_i2c_sensors, LOG_LEVEL_DBG);

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* 1000 msec = 1 sec */
#define BME_SLEEP_TIME_MS   5000

static const struct device *check_bme280_device(const struct device *bme_dev)
{
	if (bme_dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		LOG_ERR("\nError: no device found.");
		return NULL;
	}

	if (!device_is_ready(bme_dev)) {
		LOG_ERR("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.",
		       bme_dev->name);
		return NULL;
	}

	LOG_INF("Found device \"%s\", getting sensor data", bme_dev->name);
	return bme_dev;
}

/*
 * Get a device structure from a devicetree node with compatible
 * "bosch,bme280". (If there are multiple, just pick one.)
 */
SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bme280),
		{SENSOR_CHAN_AMBIENT_TEMP, 0},
		{SENSOR_CHAN_HUMIDITY, 0},
		{SENSOR_CHAN_PRESS, 0});

RTIO_DEFINE(ctx, 1, 1);

static int handle_bme280(void)
{
	int ret;
	uint8_t buf[128];
	const struct device *const bme_dev = DEVICE_DT_GET_ANY(bosch_bme280);

	LOG_INF("Starting I2C sensor demo");

	/* Can we use the I2C sensor? */
	if (check_bme280_device(bme_dev) == NULL) {
		LOG_ERR("Could not find the BME280");
		return -1;
	}

	/* Thread loop */
	while (1) {
		ret = sensor_read(&iodev, &ctx, buf, 128);
		if (ret != 0) {
			LOG_ERR("%s: sensor_read() failed: %d", bme_dev->name, ret);
			break;
		}

		const struct sensor_decoder_api *decoder;

		ret = sensor_get_decoder(bme_dev, &decoder);
		if (ret != 0) {
			LOG_ERR("%s: sensor_get_decode() failed: %d", bme_dev->name, ret);
			break;
		}

		uint32_t temp_fit = 0;
		struct sensor_q31_data temp_data = {0};

		decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_AMBIENT_TEMP, 0},
						&temp_fit, 1, &temp_data);

		uint32_t press_fit = 0;
		struct sensor_q31_data press_data = {0};

		decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_PRESS, 0},
						&press_fit, 1, &press_data);

		uint32_t hum_fit = 0;
		struct sensor_q31_data hum_data = {0};

		decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_HUMIDITY, 0},
						&hum_fit, 1, &hum_data);

		printk("temp: %s%d.%d; press: %s%d.%d; humidity: %s%d.%d\n",
			PRIq_arg(temp_data.readings[0].temperature, 6, temp_data.shift),
			PRIq_arg(press_data.readings[0].pressure, 6, press_data.shift),
			PRIq_arg(hum_data.readings[0].humidity, 6, hum_data.shift));

		k_msleep(BME_SLEEP_TIME_MS);
	}
	return ret;
}

K_THREAD_DEFINE(bme280_id, STACKSIZE, handle_bme280, NULL, NULL, NULL, PRIORITY, 0, 0);

