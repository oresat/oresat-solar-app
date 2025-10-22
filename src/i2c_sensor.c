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

static const struct device *const ina = DEVICE_DT_GET_ONE(ti_ina226);
static const struct device *tmp1;
static const struct device *tmp2;

static int init_ina226(void)
{
	LOG_INF("Starting INA226 reading");

	if (!device_is_ready(ina)) {
		LOG_ERR("Device %s is not ready.", ina->name);
		return -ENODEV;
	}
	return 0;
}

static int handle_ina226(void)
{
	struct sensor_value v_bus;
	struct sensor_value power;
	struct sensor_value current;
	int rc;

	rc = sensor_sample_fetch(ina);
	if (rc) {
		LOG_ERR("Could not fetch sensor data: %d", rc);
	} else {
		sensor_channel_get(ina, SENSOR_CHAN_VOLTAGE, &v_bus);
		sensor_channel_get(ina, SENSOR_CHAN_POWER, &power);
		sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &current);

		printk("Bus: %f [V] -- "
			   "Power: %f [W] -- "
			   "Current: %f [A]\n",
			   sensor_value_to_double(&v_bus),
			   sensor_value_to_double(&power),
			   sensor_value_to_double(&current));
	}
	return rc;
}

static int init_tmp101(void)
{
	LOG_INF("Starting TMP108 reading");

	tmp1 = DEVICE_DT_GET_ONE(ti_tmp108);
	tmp2 = tmp1; // DEVICE_NAME_GET(tmp101_cell2);

	if (!tmp1 || !tmp2) {
		LOG_ERR("Tmp108 devices not found");
		return -ENODEV;
	}

	if (!device_is_ready(tmp1) || !device_is_ready(tmp2)) {
		LOG_ERR("tmp108 devices not ready");
		return -ENODEV;
	}

#if 0
	sensor_attr_set(tmp1,
			SENSOR_CHAN_AMBIENT_TEMP,
			SENSOR_ATTR_TMP108_CONTINUOUS_CONVERSION_MODE,
			NULL);

#if CONFIG_APP_ENABLE_ONE_SHOT
	enable_one_shot(tmp1);
#endif

#if CONFIG_APP_REPORT_TEMP_ALERTS
	enable_temp_alerts(tmp1);
#endif
#endif

	return 0;
}

int get_temperature_continuous(const struct device *tmp108)
{

	struct sensor_value temp_value;
	int rc;

	rc = sensor_channel_get(tmp108,
				    SENSOR_CHAN_AMBIENT_TEMP,
				    &temp_value);

	if (rc) {
		LOG_ERR("Sensor_channel_get failed: %d", rc);
		return rc;
	}

	printk("temperature is %gC\n", sensor_value_to_double(&temp_value));
	return 0;
}

static int handle_tmp101(void)
{
	int rc;

	rc = sensor_sample_fetch(tmp1);
	if (rc) {
		LOG_ERR("tmp1 sensor_sample_fetch failed: %d", rc);
		return rc;
	}
	//rc = sensor_sample_fetch(tmp2);
	//if (rc) {
	//	LOG_ERR("tmp2 sensor_sample_fetch failed: %d", rc);
	//	break;
	//}

#if 1 // !CONFIG_APP_ENABLE_ONE_SHOT
	get_temperature_continuous(tmp1);
	//get_temperature_continuous(tmp2);
#endif
	return 0;
}

static int handle_i2c_sensors(void)
{
	int rc1;
	int rc2;

	rc1 = init_ina226();
	rc2 = init_tmp101();
	while (true) {
		if (!rc1) {
			rc1 = handle_ina226();
		}
		if (!rc2) {
			rc2 = handle_tmp101();
		}
		k_sleep(K_MSEC(2000));
	}
	return 0;
}

K_THREAD_DEFINE(ina226_id, STACKSIZE, handle_i2c_sensors, NULL, NULL, NULL, PRIORITY, 0, 0);

