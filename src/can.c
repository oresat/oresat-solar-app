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
#include <oresat.h>

#include "can_util.h"
#include "gpios.h"

LOG_MODULE_REGISTER(can_thread, LOG_LEVEL_INF);

#define CAN_INTERFACE (DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					 DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
								CONFIG_CAN_DEFAULT_BITRATE) / 1000))

#define CAN_THREAD_STACK_SIZE 2048
#define CAN_THREAD_PRIORITY 0
#define CAN_STARTUP_DELAY_MS 250

#define MAX_FAULT_PRINTS 10

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
	uint32_t elapsed = 0U;
	int64_t timestamp;
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	struct canopen_context can = {.dev = CAN_INTERFACE};
	uint8_t node_id = DEFAULT_NODE_ID;
	int print_fault = 0;

	k_thread_name_set(can_id, "can_thread");
	k_sleep(K_MSEC(CAN_STARTUP_DELAY_MS));

	LOG_INF("Starting CAN thread");

	err = init_gpios();
	if (err) {
		// We will not know what hardware we are running on for certain.
		// But for CAN, that just means we will assume we cannot query the CAN hardware fault line,
		// nor can we control CAN silence.
		LOG_WRN("Could not initialize general GPIOs: %d", err);
	}

	err = init_can_gpios();
	if (err) {
		// Not a fatal error -- GPIOs are optional (hardware fault reporting and sleep control)
		LOG_WRN("Error initializing CAN GPIOs: %d", err);
	}

	err = settings_subsys_init();
	if (err) {
		// We should not continue because we will not be able to use the proper CAN node id,
		// nor, if the shell is enabled, will we be able to store a selected id.
		LOG_ERR("settings subsys initialization: fail (err %d)", err);
		goto fatal_error;
	} else {
		node_id = load_node_id();
		if (node_id == DEFAULT_NODE_ID) {
			LOG_WRN("CAN node id has not been set!");
#if defined(CONFIG_SHELL)
			LOG_WRN("--> Use shell command to set (type help).");
#else
			LOG_WRN("--> Firmware not built with CONFIG_SHELL=y. Rebuild and flash to update CAN node id.");
#endif
			if (IS_ENABLED(CONFIG_CAN_ID_MUST_BE_SET)) {
				LOG_ERR("Halting.");
				goto fatal_error;
			}
		}
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
			LOG_ERR("Self-tests failed.");
			k_msleep(500); // Let logs flush
#if defined(CONFIG_REBOOT)
			LOG_ERR("Rebooting to revert firmware...");
			sys_reboot(SYS_REBOOT_COLD);
#else
			LOG_ERR("Please reboot to revert firmware.");
#endif
		}
	} else {
		LOG_INF("Firmware already confirmed.");
	}

	oresat_fix_pdo_cob_ids(node_id);

	LOG_DBG("Opening CAN device");
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface is not ready");
		goto fatal_error;
	}

	set_can_normal(); // if using a newer board, tell the TCAN337 to not be silent

	LOG_INF("Starting CANopenNode (node_id=0x%02x, bitrate=%u kbps)",
			(unsigned)node_id, (unsigned)CAN_BITRATE);

	while (reset != CO_RESET_APP) {
		elapsed = 0U;

		err = CO_init(&can, node_id, CAN_BITRATE);
		if (err != CO_ERROR_NO) {
			LOG_ERR("CO_init failed (err = %d)", err);
			goto fatal_error;
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

			if (get_can_fault()) {
				// TODO: should we do anything, or is it caught by CO_isError() below?
				// Investigate and modify if needed.
				if (print_fault < MAX_FAULT_PRINTS) {
					print_fault++;
					LOG_WRN("CAN HW FAULT");
				}
			} else {
				print_fault = 0; // ensure we print spurious errors mixed with normal operation
			}

			if (CO_isError(CO->em, CO_EM_CAN_TX_OVERFLOW)) {
				CO_errorReset(CO->em, CO_EM_CAN_TX_OVERFLOW, 111);
			}

			reset = CO_process(CO, (uint16_t)elapsed, &timeout);
			if (reset != CO_RESET_NOT) {
				break;
			}

			if (timeout > 0) {
				/* Read inputs */
				CO_process_RPDO(CO, syncWas);

				/* Write outputs */
				CO_process_TPDO(CO, syncWas, timeout * 1000U);

				k_sleep(K_MSEC(timeout));
				elapsed = (uint32_t)k_uptime_delta(&timestamp);
			} else {
				elapsed = 0U;
			}
		}
		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}

	// The CAN host asked us to reset ourselves. Clean up and then do so (if enabled in this build).
	CO_delete(&can);
#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#endif

fatal_error:
	k_msleep(1000);
	__ASSERT(false, "Fatal error");
}

K_THREAD_DEFINE(can_id, CAN_THREAD_STACK_SIZE, handle_can, NULL, NULL, NULL, CAN_THREAD_PRIORITY, 0, 0);
