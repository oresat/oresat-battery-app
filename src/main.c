#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <board_sensors.h>

#include "diag.h"

LOG_MODULE_REGISTER(app_battery, CONFIG_APP_BATTERY_LOG_LEVEL);

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	LOG_ERR("Unrecoverable error 0x%x: rebooting!", reason);
	LOG_PANIC(); // attempt to flush the logs

#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

int main(void)
{
	LOG_INF("Oresat app battery starting up on board: %s", CONFIG_BOARD_TARGET);

	LOG_DBG("Initializing sensors");
	board_sensors_init();

	LOG_INF("Done.");
	return 0;
}
