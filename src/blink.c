/**
 * blink.c
 *
 * Code came from https://github.com/Blen-E/pracblink, which
 * originally from zephyr/samples/basic/blinky.
 *
 * Modified slightly to run as a thread and use logging.
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h> 
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oresat_blink, LOG_LEVEL_DBG);

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* 1000 msec = 1 sec */
#define LED_SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static int handle_blink(void)
{
	int ret;
	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

	LOG_INF("Starting blink demo");

	/* Can we use the LED? */
	if (!device_is_ready(led.port)) {
		LOG_ERR("LED device is not ready");
		return -1;
	}

	/* Set it up */
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure the LED GPIO pin: %d", ret);
		return ret;
	}

	/* Thread loop */
	while (1) {
		gpio_pin_toggle_dt(&led);
		LOG_DBG("The light is blinking!");
		k_msleep(LED_SLEEP_TIME_MS);
	}
	return 0;
}

K_THREAD_DEFINE(blink_id, STACKSIZE, handle_blink, NULL, NULL, NULL, PRIORITY, 0, 0);

