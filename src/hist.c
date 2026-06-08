#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fcb.h>
#include <zephyr/sys/crc.h>

#include <zephyr/device.h>
#include <zephyr/console/console.h>
#include <canopennode.h>
#include <CO_OD.h>
#include <oresat.h>

#include "batt.h"
#include "hist.h"

// TODO:
// - test history wrap-around
// - Zephyr console does not have a read timeout; switch to shell instead of
//   using console_getchar()

LOG_MODULE_REGISTER(hist, CONFIG_APP_BATTERY_LOG_LEVEL);

// entry to append to already-written flash at next update interval
typedef struct __attribute__((packed)) history_data {
	uint8_t hist_data[HIST_DATA_SIZE];
	uint16_t rst_cycle;			// number of reset cycles so far
	uint16_t minute : 12;		// number of minutes so far during a cycle
	uint16_t unused : 3;		// must be 0
	uint16_t estimated : 1;		// set to 1 if previous value was invalid or none found in flash
	uint16_t crc;				// crc calculated over all fields prior to this field
} history_data_t;

#define HIST_PARTITION FIXED_PARTITION_ID(hist_partition)
#define HIST_PARTITION_SIZE FIXED_PARTITION_SIZE(hist_partition)
#define HIST_PAGE_SIZE DT_PROP(DT_NODELABEL(flash0), erase_block_size)
#define NUM_HIST_ENTRIES ((HIST_PARTITION_SIZE) / sizeof(history_data_t))
#define HIST_FCB_NUM_AREAS (HIST_PARTITION_SIZE / HIST_PAGE_SIZE)
#define HIST_FCB_MAGIC 0xF000BAAA

static history_data_t last_valid_history_entry;
static struct fcb_entry last_valid_loc;
static uint16_t reset_cycle_count;
static struct fcb hist_fcb;

static int hist_find_last(void);

int hist_init(void)
{
	static struct flash_sector hist_fcb_area[HIST_FCB_NUM_AREAS];
	uint32_t cnt = sizeof(hist_fcb_area) / sizeof(hist_fcb_area[0]);
	int rc;

	LOG_DBG("Hist page size %u, page count %u", HIST_PAGE_SIZE, cnt);

	rc = flash_area_get_sectors(HIST_PARTITION, &cnt, hist_fcb_area);
	if (rc != 0 && rc != -ENOMEM) {
		LOG_ERR("Error on flash_area_get_sectors(): %d", rc);
		return rc;
	}

	hist_fcb.f_magic = HIST_FCB_MAGIC;
	hist_fcb.f_version = 1;
	hist_fcb.f_sectors = hist_fcb_area;
	hist_fcb.f_sector_cnt = cnt;
	hist_fcb.f_scratch_cnt = 1;

	rc = fcb_init(HIST_PARTITION, &hist_fcb);
	if (rc != 0) {
		const struct flash_area *fap;

		rc = flash_area_open(HIST_PARTITION, &fap);
		if (rc != 0) {
			LOG_ERR("Error opening flash area: %d", rc);
			return rc;
		}

		rc = flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);

		if (rc != 0) {
			LOG_ERR("Error flattening flash area: %d", rc);
			return rc;
		}

		rc = fcb_init(HIST_PARTITION, &hist_fcb);
	}

	if (rc) {
		LOG_ERR("Error initializing history FCB: %d", rc);
	} else {
		LOG_DBG("Battery history FCB initialized");
		rc = hist_find_last();
	}
	return rc;
}

static void hist_entry_print(history_data_t *data, const char *prefix)
{
#if DEBUG_PRINT
	(void)data;
#endif
	LOG_DBG("%srst_cycle=%d, minute=%d, unused=%d, estimated=%d, crc=0x%04X",
			prefix ? prefix : "",
			data->rst_cycle, data->minute, data->unused, data->estimated, data->crc);
#if VERBOSE_DEBUG
	printk("   Data: ");
	for (int i = 0; i < HIST_DATA_SIZE; i++) {
		printk("%02X ", data->hist_data[i]);
	}
	printk("\n");
#endif
}

static int hist_find_last(void)
{
	history_data_t data;
	struct fcb_entry loc = {0};
#if VERBOSE_DEBUG
	char prefix[60];
#endif
	uint16_t check_crc;
	int rc;

	memset(&last_valid_history_entry, 0, sizeof(last_valid_history_entry));
	memset(&last_valid_loc, 0, sizeof(last_valid_loc));

	LOG_DBG("Runtime history:\r\nentry size=%zu, entry count=%u",
			  sizeof(history_data_t), NUM_HIST_ENTRIES);

	if (fcb_is_empty(&hist_fcb)) {
		LOG_INF("Runtime history is empty.");
		return 0;
	}

	for (unsigned int i = 0; i < NUM_HIST_ENTRIES; i++) {
		rc = fcb_getnext(&hist_fcb, &loc);
		if (rc == -ENOTSUP) {
			LOG_DBG("End of history at entry %d=u.", i);
			rc = 0; // if the flash contains garbage, erase it when there are no entries
			if (!last_valid_loc.fe_sector) { // no good entries found
				LOG_WRN("Bad data found in otherwise empty history partition (%u); erasing", i);
				rc = fcb_clear(&hist_fcb);
				if (rc) {
					LOG_ERR("Error erasing history partition");
				}
			}
			break;
		}
		else if (rc) {
			LOG_ERR("Error getting next FCB entry: %d", rc);
				break;
		}
		rc = fcb_flash_read(&hist_fcb, loc.fe_sector, loc.fe_data_off, &data, loc.fe_data_len);
		if (rc) {
			LOG_ERR("Error reading next FCB entry: %d", rc);
			break;
		}
#if VERBOSE_DEBUG
		snprintk(prefix, sizeof(prefix), "%d. sect_ofs=0x%08x, data_ofs=0x%04x ", i,
				 (unsigned int)loc.fe_sector->fs_off, loc.fe_data_off);
		hist_entry_print(&data, prefix);
#endif // VERBOSE_DEBUG
		check_crc = crc16_ccitt(0, (uint8_t *)&data, sizeof(data) - sizeof(data.crc));

		if (check_crc == data.crc) {
			// Entry is valid if either it is from a subsequent reset cycle (which monotonically increases),
			// or from the same reset cycle and the same or a later minute of run time.
			// NOTE: if this runs longer than 68 hours without a reset, the minute value will wrap around.
			// In orbit, we are reset once per day, so this should be fine.
			if ((data.rst_cycle > last_valid_history_entry.rst_cycle) ||
				((data.rst_cycle == last_valid_history_entry.rst_cycle) && (data.minute >= last_valid_history_entry.minute))) {
				last_valid_history_entry = data;
				last_valid_loc = loc;
			} else {
				LOG_DBG("Most recent entry found.");
				break;
			}
		} else {
			LOG_WRN("CRC failure on entry %u", i);
		}
	}
	// We found a good entry, so we can advance the reset cycle count.
	if (last_valid_loc.fe_sector != NULL) {
		LOG_DBG("Selected last valid history entry:");
		hist_entry_print(&last_valid_history_entry, NULL);

		reset_cycle_count = last_valid_history_entry.rst_cycle;

		// We have started a new cycle, so advance this counter once
		reset_cycle_count++;
		return 0;
	}
	return rc;
}

static void hist_create(history_data_t *dest)
{
	dest->rst_cycle = reset_cycle_count;
	dest->minute = k_uptime_seconds() / 60;
	dest->unused = 0;
	dest->estimated = false;
	dest->crc = crc16_ccitt(0, (uint8_t *)dest, sizeof(*dest) - sizeof(dest->crc));
}

static bool hist_add_next(history_data_t *new_data)
{
	int rc;
	bool ok = true;

	rc = fcb_append(&hist_fcb, sizeof(history_data_t), &last_valid_loc);
	if (rc == -ENOSPC) {
		LOG_INF("At end of page. Rotating fcb.");
		rc = fcb_rotate(&hist_fcb);
		if (rc) {
			LOG_ERR("Error on fcb_rotate(): %d", rc);
			return false;
		}
		rc = fcb_append(&hist_fcb, sizeof(history_data_t), &last_valid_loc);
	}

	if (rc)
	{
		LOG_ERR("Error on fcb_append(): %d", rc);
		return false;
	}
	rc = fcb_flash_write(&hist_fcb, last_valid_loc.fe_sector, last_valid_loc.fe_data_off,
						 new_data, last_valid_loc.fe_data_len);
	if (rc) {
		LOG_ERR("Error on fcb_flash_write(): %d", rc);
		ok = false;
	}
	rc = fcb_append_finish(&hist_fcb, &last_valid_loc);
	if (rc) {
		LOG_ERR("Error on fcb_append_finish(): %d", rc);
		ok = false;
	}

	// history storage is full or error writing flash, so let caller recover
	memset(&last_valid_history_entry, 0, sizeof(last_valid_history_entry));
	return ok;
}

#if CONFIG_HIST_STORE_PROMPT
static bool hist_erase(void)
{
	return fcb_clear(&hist_fcb) == 0;
}

static bool hist_test(void)
{

/*
- erase all
- repeatedly:
	- write large prime number of entries, keeping track of total written, with
	  the fields in the entry growing predictably between each write
	- check return value of hist_find_last()
	- confirm last_valid_history_entry matches the last one written
	- keep going until we've gone at least 2 * NUM_HIST_ENTRIES
*/
	LOG_INF("Test failed.");
	return false;
}
#endif /* CONFIG_HIST_STORE_PROMPT */

bool hist_load_current(uint8_t hist_data[HIST_DATA_SIZE])
{
	if ((last_valid_history_entry.minute == 0) && (last_valid_history_entry.crc == 0)) {
		LOG_DBG("No latest history to load");
		return false;
	}
	memcpy(hist_data, last_valid_history_entry.hist_data, HIST_DATA_SIZE);
	return true;
}

bool hist_store_current(const uint8_t hist_data[HIST_DATA_SIZE])
{
	history_data_t new_data;
	char prefix[60] = "";
	bool ret = false;

#if CONFIG_HIST_STORE_PROMPT
	for (;;) {
		LOG_DBG("********** Store hist e(rase), t(est), y(es), n(o)? ");
		uint8_t ch = console_getchar();

		LOG_DBG("");
		if (ch == 'e') {
			LOG_DBG("Erasing *************");
			hist_erase();
			return true;
		} else if (ch == 't') {
			LOG_INF("Testing *************");
			hist_test();
		} else if (ch != 'y') {
			LOG_DBG("Not storing ************");
			return true;
		}
		break;
	}
#endif // CONFIG_HIST_STORE_PROMPT

	// copy in the caller's data to be stored
	memcpy(new_data.hist_data, hist_data, HIST_DATA_SIZE);
	hist_create(&new_data);

	LOG_DBG("Storing new history entry:");

	if (hist_add_next(&new_data)) {
#if VERBOSE_DEBUG
			snprintk(prefix, sizeof(prefix), "sect_ofs=0x%08x, data_ofs=0x%04x ",
					 (unsigned int)last_valid_loc.fe_sector->fs_off, last_valid_loc.fe_data_off);
#endif
			LOG_DBG("Done.");
			ret = true;
	} else {
		LOG_INF("ERROR: Unable to store current history entry.");
	}
	hist_entry_print(&new_data, prefix);
	return ret;
}

