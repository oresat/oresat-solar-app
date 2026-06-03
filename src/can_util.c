#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

#include "can_util.h"

LOG_MODULE_REGISTER(can_util, LOG_LEVEL_DBG);

uint8_t load_node_id(void)
{
	int rc;
	uint8_t node_id = 0;

	settings_load();

	rc = settings_load_one("node_id", (void *)&node_id, sizeof(node_id));
	if (rc < 0) {
		LOG_ERR("Error loading node_id from settings: %d", rc);
	}
	if (!node_id) {
		node_id = DEFAULT_NODE_ID;
	}
	return node_id;
}

int store_node_id(uint8_t node_id)
{
	int rc;

	rc = settings_save_one("node_id", &node_id, sizeof(node_id));
	if (rc < 0) {
		LOG_ERR("Error saving node_id to settings: %d", rc);
	}
	return rc;
}

#if defined(CONFIG_SHELL)
static int cmd_node_id(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t node_id = load_node_id();

	if (argc < 2) {
		shell_print(sh, "Current CAN node id: %u (0x%02x)", node_id, node_id);
		if (node_id == DEFAULT_NODE_ID) {
			shell_warn(sh, "CAN node id is not set! Using default. Likely to conflict with another card.");
		}
	} else {
		node_id = atoi(argv[1]);
		if (store_node_id(node_id) < 0) {
			shell_error(sh, "Unable to store CAN node id %u (0x%02x)", node_id, node_id);
		} else {
			shell_print(sh, "CAN node id %u (0x%02x) stored.", node_id, node_id);
		}
	}

	return 0;
}

SHELL_CMD_ARG_REGISTER(nodeid, NULL, "CAN node id [<new id value>]", cmd_node_id, 1, 2);
#endif
