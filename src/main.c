/**
 * main.c
 *
 * Simple main that only logs a bootup message. The remainder
 * of the demos are implemented as independent threads
 * in blink.c, dac.c, i2c_sensor.c, and adc.c.
 *
 * These can be disabled at compile time by adding:
 *   CONFIG_BLINK=n
 * for example, to prj.conf. See Kconfig for the options or run
 * west build -t menuconfig for an interacive configuration
 * editor.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <version.h>
#include <app_version.h>

LOG_MODULE_REGISTER(oresat_mcxn947_solar, LOG_LEVEL_DBG);

int main(void)
{
	LOG_INF("Oresat MCXN947 Solar Board App");
	LOG_INF("   Oresat   Board: %s", CONFIG_BOARD_TARGET);
	LOG_INF("   App    Version: %s", APP_VERSION_STRING);
	LOG_INF("   Zephyr Version: %s", KERNEL_VERSION_STRING);

	return 0;
}
