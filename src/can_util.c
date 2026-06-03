#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "can_util.h"

LOG_MODULE_REGISTER(can_util, LOG_LEVEL_DBG);

#define DEFAULT_NODE_ID 0x7C

uint8_t load_node_id(void)
{
	int rc;
	uint8_t node_id;

	LOG_INF("Load settings...");
	settings_load();

	rc = settings_load_one("node_id", (void *)&node_id, sizeof(node_id));
	if (rc < 0) {
		LOG_ERR("Error loading node_id from settings: %d", rc);
		node_id = DEFAULT_NODE_ID;
	}
	return node_id;
}

void store_node_id(uint8_t node_id)
{
	int rc;

	LOG_INF("Storing settings...");

	rc = settings_save_one("node_id", &node_id, sizeof(node_id));
	if (rc < 0) {
		LOG_ERR("Error saving node_id to settings: %d", rc);
	}
}

