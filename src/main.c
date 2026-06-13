#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <board_sensors.h>
#include <version.h>
#include <app_version.h>

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
	LOG_INF("Oresat app battery starting up");
	LOG_INF("   Oresat   Board: %s", CONFIG_BOARD_TARGET);
	dump_hwinfo();
	LOG_INF("   App    Version: %s", APP_VERSION_STRING);
	LOG_INF("   Zephyr Version: %s", KERNEL_VERSION_STRING);

	LOG_DBG("Initializing sensors");
	board_sensors_init();

	LOG_INF("Done.");
	return 0;
}
