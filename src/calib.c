#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <zephyr/fatal.h>

#include "batt.h"
#include "calib.h"
#include "max17205_intf.h"

// TODO:
// - Zephyr console does not have a read timeout; switch to shell
//   instead of using console_getchar()

LOG_MODULE_REGISTER(calib, CONFIG_APP_BATTERY_LOG_LEVEL);

// DEBUG_PRINT is required for ENABLE_NV_MEMORY_UPDATE_CODE to do anything
#if DEBUG_PRINT && CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE
#define NV_WRITE_PROMPT_ENABLED 1
#else
#define NV_WRITE_PROMPT_ENABLED 0
#endif

// DEBUG_PRINT is required for ENABLE_LEARN_COMPLETE to do anything
#if DEBUG_PRINT && CONFIG_ENABLE_LEARN_COMPLETE
#define LEARN_COMPLETE_ENABLED 1
#else
#define LEARN_COMPLETE_ENABLED 0
#endif

#define CALIB_THREAD_STACK_SIZE 2048
#define CALIB_THREAD_PRIORITY 0
extern const k_tid_t calib_id;

#define CALIB_INTERVAL_MS 120000 // 2 minutes

/**
 * Helper function to trigger write of volatile memory on MAX71205 chip.
 * Returns true if NV was written, false otherwise.
 */
#if NV_WRITE_PROMPT_ENABLED
static bool prompt_nv_write(const struct device *dev, const char *pack_str)
{
	LOG_DBG("Write NV RAM to NV%s", pack_str);

	uint8_t num_writes_left = 0;
	if (max17205_read_writes_remaining(dev, &num_writes_left) == 0) {
		LOG_DBG("Num_writes_left = %u", num_writes_left);
	}

	if (num_writes_left > 0) {
		// Answer n to just use the changes in the volatile registers
		LOG_DBG("Write NV memory on MAX17205 for %s ? y/n? ", pack_str);
		uint8_t ch = console_getchar();

		LOG_DBG("");

		if (ch == 'y') {
			LOG_DBG("Attempting to write non volatile memory on MAX17205...");
			k_msleep(50);

			if (max17205_nv_program(dev) == 0) {
				LOG_DBG("Successfully wrote non volatile memory on MAX17205...");
			} else {
				LOG_DBG("Failed to write non volatile memory on MAX17205...");
			}
			return true; // NV changes made
		} else {
			LOG_DBG("Update skipped.");
		}
	} else {
		LOG_DBG("No more NV writes remain.");
	}

	return false; // no NV changes made
}
#endif // NV_WRITE_PROMPT_ENABLED

//If state of charge is known to be full, set LS bits D6-D0 of LearnCfg register to 0b111
//and write MixCap and RepCap registers to 2600.
#if LEARN_COMPLETE_ENABLED
static bool update_learning_complete(const struct device *dev, pack_t *pack)
{
	batt_pack_data_t *pack_data = &pack->data;
	uint16_t state;
	bool ret = false;

	if ((pack_data->batt_mV > BATT_FULL_THRESHOLD_MV) &&
		(pack_data->avg_current_mA < EOC_THRESHOLD_MA) &&
		(pack_data->avg_current_mA >= 0) &&
		(pack_data->full_capacity_mAh >= CELL_CAPACITY_MAH) ) {

		LOG_DBG("Pack %d seems full", pack->pack_number);

		int rc = max17205_read_learn_stage(dev, &state);
		if (rc) {
			LOG_DBG("Error reading learn state");
			return ret;
		}
		LOG_DBG("Learning state = %u", state);

		if (state == MAX17205_LEARN_COMPLETE) {
			LOG_DBG("Learning is already complete.");
		} else {
			rc = max17205_write_learn_stage(dev, MAX17205_LEARN_COMPLETE);
			if (rc) {
				LOG_DBG("Error writing learn state");
				return ret;
			}
			rc = max17205_read_learn_stage(dev, &state);
			if (rc) {
				LOG_DBG("Error checking learn state");
				return ret;
			}
			if (state != MAX17205_LEARN_COMPLETE) {
				LOG_DBG("Error setting state = %u; is %u", MAX17205_LEARN_COMPLETE, state);
				return ret;
			}
			LOG_DBG("Learning state set = %u", state);
			ret = true;
		}

		pack_data->mix_capacity_mAh = pack_data->reported_capacity_mAh = pack_data->full_capacity_mAh;

		rc = max17205_write_capacity(dev, MAX17205_CHAN_MIX_CAPACITY,
									 pack_data->mix_capacity_mAh);
		if (rc != 0) {
			LOG_DBG("Failed to write MIXCAP");
			return false;
		}

		rc = max17205_write_capacity(dev, SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY,
									 pack_data->reported_capacity_mAh);
		if (rc != 0) {
			LOG_DBG("Failed to write REPCAP");
			return false;
		}

		LOG_DBG("Mixcap and repcap set to %u", pack_data->full_capacity_mAh);
	}
	return ret;
}
#endif // LEARN_COMPLETE_ENABLED

static void handle_calib(void *p1, void *p2, void *p3)
{
#if DEBUG_PRINT
	unsigned int i;
#if NV_WRITE_PROMPT_ENABLED
	bool nv_written = false;
#endif
	pack_t *pack;

	k_thread_name_set(calib_id, "calib_thread");

	while (true) {
		/* Update calibration roughly this often, synchronized to battery event */
		k_msleep(CALIB_INTERVAL_MS);
		(void)batt_event_wait(true, K_FOREVER);

		for (i = 0; i < NUM_PACKS; i++) {
			pack = get_pack(i);
			LOG_DBG("%s:", pack->name);
			max17205_print_volatile_memory(pack->dev);

	#if LEARN_COMPLETE_ENABLED
			// If LEARN_COMPLETE_ENABLED=1 and NV_WRITE_PROMPT_ENABLED=1 are both enabled, we will only prompt to update
			// NV when learning is complete. If LEARN_COMPLETE_ENABLED is not 1 but the others are, then we will only prompt to update
			// NV if there is a change to NV RAM required (done prior to the main loop).
			pack->updated = update_learning_complete(pack->dev, pack);
	#endif // LEARN_COMPLETE_ENABLED
	#if NV_WRITE_PROMPT_ENABLED
			if (pack->init && pack->updated) {
				nv_written |= prompt_nv_write(pack->dev, pack->name);
				pack->updated = false;
			}
	#endif // NV_WRITE_PROMPT_ENABLED
		}

	#if NV_WRITE_PROMPT_ENABLED
		if (nv_written) {
			LOG_DBG("Done with NV RAM update code, set CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE=0 and re-write firmware.");
			LOG_ERR("Halting system");
			LOG_PANIC(); // attempt to flush the logs
			k_fatal_halt(100);
			CODE_UNREACHABLE;
		}
	#endif // NV_WRITE_PROMPT_ENABLED
	}
#endif // DEBUG_PRINT
}

K_THREAD_DEFINE(calib_id, CALIB_THREAD_STACK_SIZE, handle_calib, NULL, NULL, NULL, CALIB_THREAD_PRIORITY, 0, 0);
