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
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <version.h>
#include <app_version.h>

LOG_MODULE_REGISTER(oresat_mcxn947_solar, CONFIG_LOG_DEFAULT_LEVEL);

/** Dump Hardware Info
 *    Show the unique processor id and the reason for reset.
 *
 *    This requires:
 *    #include <zephyr/drivers/hwinfo.h>
 *    prj.conf:
 *    CONFIG_HWINFO=y
 *    Names of reset cause bits defined in same header; e.g.
 *    RESET_POR
 **/
static void dump_hwinfo(void)
{
	uint8_t hwid[32] = {0};
	uint32_t reset_cause;
	int reset_cause_ret;
	int hwid_len;

	hwid_len = hwinfo_get_device_id(hwid,sizeof(hwid)); // returns size of the device id
	reset_cause_ret = hwinfo_get_reset_cause(&reset_cause);
	if (!reset_cause_ret) {
		hwinfo_clear_reset_cause();
	}

	if (hwid_len < 0) {
		LOG_ERR("    Chip     HWID: UNNKOWN (%d", hwid_len);
	} else {
		LOG_HEXDUMP_INF(hwid, hwid_len, "    Chip     HWID: ");
	}
	if (reset_cause_ret < 0) {
		LOG_ERR("      Reset Cause: UNKNOWN (%d)", reset_cause_ret);
	} else {
		LOG_INF("      Reset Cause: 0x%08x", reset_cause);
	}
}

int main(void)
{
	LOG_INF("Oresat MCXN947 Solar Board App");
	LOG_INF("   Oresat   Board: %s", CONFIG_BOARD_TARGET);
	dump_hwinfo();
	LOG_INF("   App    Version: %s", APP_VERSION_STRING);
	LOG_INF("   Zephyr Version: %s", KERNEL_VERSION_STRING);

	return 0;
}
