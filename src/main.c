#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <canopennode.h>
#include <OD.h>
#include <board_sensors.h>
#include <oresat.h>

#include "batt.h"

LOG_MODULE_REGISTER(app_battery, CONFIG_APP_BATTERY_LOG_LEVEL);

#define CAN_INTERFACE (DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					 DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, CONFIG_CAN_DEFAULT_BITRATE) / 1000))

#define BATT_THREAD_STACK_SIZE 1024
#define BATT_THREAD_PRIORITY 0

static K_THREAD_STACK_DEFINE(batt_stack, BATT_THREAD_STACK_SIZE);
static struct k_thread batt_thread_data;

int main(void)
{
	uint8_t node_id = oresat_get_node_id();

	LOG_INF("Oresat app battery starting up on board: %s, node: %u", CONFIG_BOARD_TARGET, (unsigned int)node_id);

	oresat_fix_pdo_cob_ids(node_id);

	LOG_DBG("Opening CAN");
	canopennode_init(CAN_INTERFACE, CAN_BITRATE, node_id);
	LOG_DBG("Initializing sensors");
	board_sensors_init();

	k_tid_t batt_thread = k_thread_create(&batt_thread_data, batt_stack, K_THREAD_STACK_SIZEOF(batt_stack),
										  batt_thread_handler, NULL, NULL, NULL,
										  BATT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(batt_thread, "battery_thread");

// the battery thread is now running; wait until it exits, if ever, then clean up
	k_thread_join(batt_thread, K_FOREVER);
	k_thread_stack_free(batt_stack);

	canopennode_stop(CAN_INTERFACE);
	sys_reboot(SYS_REBOOT_COLD);
	LOG_INF("Done.");

	return 0;
}
