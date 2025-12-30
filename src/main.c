#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <canopennode.h>
#include <CO_OD.h>
#include <board_sensors.h>
#include <oresat.h>

#include "batt.h"
#include "diag.h"

LOG_MODULE_REGISTER(app_battery, CONFIG_APP_BATTERY_LOG_LEVEL);

#define CAN_INTERFACE (DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					 DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
					            CONFIG_CAN_DEFAULT_BITRATE) / 1000))

#define BATT_THREAD_STACK_SIZE 2048
#define BATT_THREAD_PRIORITY 0

static K_THREAD_STACK_DEFINE(batt_stack, BATT_THREAD_STACK_SIZE);
static struct k_thread batt_thread_data;

#define CAN_THREAD_STACK_SIZE 2048
#define CAN_THREAD_PRIORITY 0

static K_THREAD_STACK_DEFINE(can_stack, CAN_THREAD_STACK_SIZE);
static struct k_thread can_thread_data;

void can_thread_handler(void *p1, void *p2, void *p3)
{
	int err;
	uint16_t timeout;
	uint32_t elapsed = 0U;
	int64_t timestamp;
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	struct canopen_context can = {.dev = CAN_INTERFACE};
	uint8_t node_id = oresat_get_node_id();

	LOG_INF("Starting CAN thread");

	oresat_fix_pdo_cob_ids(node_id);

	LOG_DBG("Opening CAN device");
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface is not ready");
		return;
	}
	err = CO_init(&can, node_id, CAN_BITRATE);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_init failed (err = %d)", err);
		return;
	}
	LOG_INF("CANopen stack initialized for node %u", node_id);

	CO_CANsetNormalMode(CO->CANmodule[0]);

	while (true) {
		bool_t syncWas = false;

		timeout = 1000U;
		timestamp = k_uptime_get();

		/* Read inputs */
		CO_process_RPDO(CO, syncWas);

		/* Write outputs */
		CO_process_TPDO(CO, syncWas, timeout * 1000U);

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
	}

	CO_delete(&can);
}

int main(void)
{
	LOG_INF("Oresat app battery starting up on board: %s", CONFIG_BOARD_TARGET);

	LOG_DBG("Initializing sensors");
	board_sensors_init();

	k_tid_t can_thread = k_thread_create(&can_thread_data, can_stack, K_THREAD_STACK_SIZEOF(can_stack),
										  can_thread_handler, NULL, NULL, NULL,
										  CAN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(can_thread, "can_thread");

	k_tid_t batt_thread = k_thread_create(&batt_thread_data, batt_stack, K_THREAD_STACK_SIZEOF(batt_stack),
										  batt_thread_handler, NULL, NULL, NULL,
										  BATT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(batt_thread, "battery_thread");

	// the battery thread is now running; wait until it exits, if ever, then clean up
	k_thread_join(can_thread, K_FOREVER);
	k_thread_join(batt_thread, K_FOREVER);
	k_thread_stack_free(batt_stack);
	k_thread_stack_free(can_stack);

#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#endif
	LOG_INF("Done.");

	return 0;
}
