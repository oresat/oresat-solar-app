#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_CAN)
#include <canopennode.h>
#endif

#include "gpios.h"

LOG_MODULE_REGISTER(gpios, CONFIG_LOG_DEFAULT_LEVEL);

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(solargpios)

static const struct gpio_dt_spec ina226_nalert = GPIO_DT_SPEC_GET(BP_NODE, ina226_nalert_gpios);
static const struct gpio_dt_spec cell1_tmp101_alert = GPIO_DT_SPEC_GET(BP_NODE, cell1_tmp101_alert_gpios);
static const struct gpio_dt_spec cell2_tmp101_alert = GPIO_DT_SPEC_GET(BP_NODE, cell2_tmp101_alert_gpios);
static const struct gpio_dt_spec lt1618_enable = GPIO_DT_SPEC_GET(BP_NODE, lt1618_enable_gpios);
const struct gpio_dt_spec hw_rev_bit_0 = GPIO_DT_SPEC_GET(BP_NODE, hw_rev_bit_0_gpios);
const struct gpio_dt_spec hw_rev_bit_1 = GPIO_DT_SPEC_GET(BP_NODE, hw_rev_bit_1_gpios);
const struct gpio_dt_spec hw_rev_bit_2 = GPIO_DT_SPEC_GET(BP_NODE, hw_rev_bit_2_gpios);
const struct gpio_dt_spec tcan_nfault_oc = GPIO_DT_SPEC_GET(BP_NODE, tcan_nfault_oc_gpios);
const struct gpio_dt_spec tcan_silent = GPIO_DT_SPEC_GET(BP_NODE, tcan_silent_gpios);

static unsigned raw_board_rev = HW_REV_UNKNOWN;

int init_gpios(void)
{
	int ret;
	int err = 0;
	static bool initialized; // make idempotent

	if (initialized) {
		return 0;
	}
	initialized = true;

	ret = gpio_pin_configure_dt(&ina226_nalert, GPIO_INPUT);
	if (ret) {
		err = ret;
	}
	ret = gpio_pin_configure_dt(&cell1_tmp101_alert, GPIO_INPUT);
	if (ret) {
		err = ret;
	}
	ret = gpio_pin_configure_dt(&cell2_tmp101_alert, GPIO_INPUT);
	if (ret) {
		err = ret;
	}
	ret = gpio_pin_configure_dt(&lt1618_enable, GPIO_OUTPUT_ACTIVE); // enable the LT1618
	if (ret) {
		err = ret;
	}

	ret = gpio_pin_configure_dt(&hw_rev_bit_0, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		LOG_ERR("Could not configure hw_rev_bit_0 as input with pull up: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&hw_rev_bit_1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		LOG_ERR("Could not configure hw_rev_bit_1 as input with pull up: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&hw_rev_bit_2, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		LOG_ERR("Could not configure hw_rev_bit_2 as input with pull up: %d", ret);
		err = ret;
	}

	raw_board_rev = gpio_pin_get_dt(&hw_rev_bit_0) << 0 |
				gpio_pin_get_dt(&hw_rev_bit_1) << 1 |
				gpio_pin_get_dt(&hw_rev_bit_2) << 2;

	LOG_INF("Board Rev %u", get_board_hw_rev());

	return err;
}

bool get_ina226_alert(void)
{
	return gpio_pin_get_dt(&ina226_nalert) != 0; // high == fault
}

bool get_cell1_tmp101_alert(void)
{
	return gpio_pin_get_dt(&cell1_tmp101_alert) == 0; // low == fault
}

bool get_cell2_tmp101_alert(void)
{
	return gpio_pin_get_dt(&cell2_tmp101_alert) == 0; // low == fault
}

int set_lt1618_enable(bool enable)
{
	return gpio_pin_configure_dt(&lt1618_enable, enable ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
}

board_hw_rev_t get_board_hw_rev(void)
{
	if (raw_board_rev == HW_REV_UNKNOWN) { // initialization of GPIOs failed, or was not called
		return HW_REV_1_0_0;
	}
	if (raw_board_rev == 7) { // no rev pins hooked up, so the pullups on the inputs make these all read as 1
		return HW_REV_1_0_0;
	}
	// Otherwise, use the value as a relative offset from HW_REV_1_2_0.
	// NOTE: this is different from how the hw rev lines work on the mag card. In that case, values != 7 are
	// the board rev already, and no offset is needed.
	return HW_REV_1_2_0 + (board_hw_rev_t)raw_board_rev;
}

int init_can_gpios(void)
{
	int ret;
	int err = 0;
	static bool gc_initialized = false;

	if (gc_initialized) {
		return 0;
	}
	gc_initialized = true;

	ret = gpio_pin_configure_dt(&tcan_nfault_oc, GPIO_INPUT | GPIO_PULL_UP); // goes high on fault; v1.2+ boards only
	if (ret) {
		LOG_ERR("Could not configure tcan_nfault_oc as input with pull up: %d", ret);
		err = ret;
	}

	ret = gpio_pin_configure_dt(&tcan_silent, GPIO_OUTPUT_INACTIVE); // put silent in disabled (normal) mode
	if (ret) {
		LOG_ERR("Could not configure tcan_silent as output inactive: %d", ret);
		err = ret;
	}

	return err;
}

int set_can_silent(void)
{
	if (get_board_hw_rev() != HW_REV_1_0_0) {
		return gpio_pin_configure_dt(&tcan_silent, GPIO_OUTPUT_ACTIVE); // drive the pin high
	}
	return 0;
}

int set_can_normal(void)
{
	if (get_board_hw_rev() != HW_REV_1_0_0) {
		return gpio_pin_configure_dt(&tcan_silent, GPIO_OUTPUT_INACTIVE); // drive the pin low
	}
	return 0;
}

bool get_can_fault(void)
{
	if (get_board_hw_rev() != HW_REV_1_0_0) {
		return gpio_pin_get_dt(&tcan_nfault_oc) != 0; // high == fault
	}
	return false;
}
