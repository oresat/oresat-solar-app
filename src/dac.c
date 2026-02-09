/**
 * dac.c
 *
 * Original code came from zephyr/samples/driver/dac.
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
#include <zephyr/dsp/print_format.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oresat_dac_ina226, LOG_LEVEL_DBG);

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY_DAC 6

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 0
#define DAC_RESOLUTION 0
#endif

static int handle_dac(void)
{
	int ret;
	const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);
	const struct dac_channel_cfg dac_ch_cfg = {
		.channel_id  = DAC_CHANNEL_ID,
		.resolution  = DAC_RESOLUTION,
	#if defined(CONFIG_DAC_BUFFER_NOT_SUPPORT)
		.buffered = false,
	#else
		.buffered = true,
	#endif /* CONFIG_DAC_BUFFER_NOT_SUPPORT */
	};

	LOG_INF("Starting DAC demo");

	/* Can we use the DAC? */
	if (!device_is_ready(dac_dev)) {
		LOG_ERR("DAC device %s is not ready", dac_dev->name);
		return -1;
	}

	/* Set it up */
	ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
	if (ret != 0) {
		LOG_ERR("Setting up of DAC channel failed with code %d", ret);
		return ret;
	}

	/* Number of valid DAC values, e.g. 4096 for 12-bit DAC */
	const int dac_values = 1U << DAC_RESOLUTION;
	/*
	 * 100 usec sleep leads to about 0.4 sec signal period for 12-bit
	 * DACs. For DACs with lower resolution, sleep time needs to
	 * be increased.
	 */
	const int sleep_time = (4096 / dac_values > 0 ? 4096 / dac_values : 1) * 100;

	LOG_DBG("Generating sawtooth signal at DAC channel %d.", DAC_CHANNEL_ID);
	LOG_DBG("Number of DAC samples per cycle: %d, sleep time per sample (us): %d",
		   dac_values, sleep_time);
	int cycle = 0;

	while (1) {
		for (int i = 0; i < dac_values; i++) {
			ret = dac_write_value(dac_dev, DAC_CHANNEL_ID, i);
			if (ret != 0) {
				LOG_ERR("dac_write_value() failed with code %d", ret);
				return ret;
			}
			k_sleep(K_USEC(sleep_time));
		}
		LOG_INF("Cycle %u", ++cycle);
	}
	return 0;
}

K_THREAD_DEFINE(dac_id, STACKSIZE, handle_dac, NULL, NULL, NULL, PRIORITY_DAC, 0, 0);

