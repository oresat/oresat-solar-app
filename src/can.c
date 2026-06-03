#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include <CO_OD.h>
//#include <board_sensors.h>
#include <oresat.h>

#include "can_util.h"

LOG_MODULE_REGISTER(can_thread, LOG_LEVEL_DBG);

#define CAN_INTERFACE (DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					 DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
								CONFIG_CAN_DEFAULT_BITRATE) / 1000))

#define CAN_THREAD_STACK_SIZE 2048
#define CAN_THREAD_PRIORITY 0
extern const k_tid_t can_id;

static bool run_self_tests(void)
{
	LOG_INF("Running system self-tests...");

	// Placeholder for board/app-specific checks
	bool tests_passed = true;

	return tests_passed;
}

static void handle_can(void *p1, void *p2, void *p3)
{
	int err;
	uint16_t timeout;
	uint16_t wr_timeout_count;
	uint32_t elapsed = 0U;
	int64_t timestamp;
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	struct canopen_context can = {.dev = CAN_INTERFACE};
	uint8_t node_id = DEFAULT_NODE_ID;

	k_thread_name_set(can_id, "can_thread");

	LOG_INF("Starting CAN thread");

	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings subsys initialization: fail (err %d)\n", err);
	} else {
		store_node_id(0x10);
		node_id = load_node_id();
	}

	// Confirm a newly booted MCUboot image if self-tests pass
	if (!IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT)) {
		/* do nothing */
	} else if (!boot_is_img_confirmed()) {
		LOG_INF("New firmware detected. Pending confirmation.");

		if (run_self_tests()) {
			LOG_INF("Self-tests passed. Confirming image.");
			int rc = boot_write_img_confirmed();
			if (rc < 0) {
				LOG_ERR("Failed to confirm MCUboot image: %d", rc);
			}
		} else {
			LOG_ERR("Self-tests failed. Rebooting to revert firmware...");
			k_msleep(500); // Let logs flush
			sys_reboot(SYS_REBOOT_COLD);
		}
	} else {
		LOG_INF("Firmware already confirmed.");
	}

	oresat_fix_pdo_cob_ids(node_id);

	LOG_DBG("Opening CAN device");
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface is not ready");
		k_msleep(1000);
		__ASSERT(false, "Fatal error");
	}

	LOG_INF("Starting CANopenNode (node_id=%u, bitrate=%u kbps)",
			(unsigned)node_id, (unsigned)CAN_BITRATE);

	while (reset != CO_RESET_APP) {
		elapsed = 0U;
		wr_timeout_count = 0U;

		err = CO_init(&can, node_id, CAN_BITRATE);
		if (err != CO_ERROR_NO) {
			LOG_ERR("CO_init failed (err = %d)", err);
			k_msleep(1000);
			__ASSERT(false, "Fatal error");
		}

		if (IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0], CO->em);
		}
		CO_CANsetNormalMode(CO->CANmodule[0]);

		LOG_INF("Waiting for CANopen requests...");

		while (true) {
			bool_t syncWas = false;

			timeout = 1U;
			timestamp = k_uptime_get();

			if (wr_timeout_count++ >= 1000U) {
				wr_timeout_count = 0U;

				/* Read inputs */
				CO_process_RPDO(CO, syncWas);

				/* Write outputs */
				CO_process_TPDO(CO, syncWas, timeout * 1000U * 1000U);
			}

			reset = CO_process(CO, (uint16_t)elapsed, &timeout);
			if (reset != CO_RESET_NOT) {
				break;
			}

			if (timeout > 0) {
				k_sleep(K_MSEC(timeout));
				elapsed = (uint32_t)k_uptime_delta(&timestamp);
			} else {
				elapsed = 0U;
			}

			if (reset == CO_RESET_COMM) {
				LOG_INF("Resetting communication");
			}
		}
	}
	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}

K_THREAD_DEFINE(can_id, CAN_THREAD_STACK_SIZE, handle_can, NULL, NULL, NULL, CAN_THREAD_PRIORITY, 0, 0);
