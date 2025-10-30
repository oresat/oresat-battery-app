#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fcb.h>

#include <zephyr/device.h>
#include <zephyr/console/console.h>
#include <canopennode.h>
#include <OD.h>
#include <oresat.h>

#include "batt.h"
#include "hist.h"
#include "calib.h"

LOG_MODULE_REGISTER(hist, CONFIG_APP_BATTERY_LOG_LEVEL);

// raw register values to store at runtime per pack;
// restoring after a reset should result in accurate
// fuel gauge estimates
typedef struct __attribute__((packed)) runtime_pack_data {
    uint16_t mixcap;            // MAX17205_AD_MIXCAP
    uint16_t repcap;            // MAX17205_AD_REPCAP
} runtime_pack_data_t;

// entry to append to already-written flash at next update interval
typedef struct __attribute__((packed)) runtime_battery_data {
    runtime_pack_data_t rt_packs[NUM_PACKS];
    uint16_t rst_cycle;         // number of reset cycles so far
    uint16_t minute : 12;       // number of minutes so far during a cycle
    uint16_t unused : 3;        // must be 0
    uint16_t estimated : 1;     // set to 1 if previous value was invalid or none found in flash
    uint16_t crc;               // crc calculated over all fields prior to this field
} runtime_battery_data_t;


#define HIST_PARTITION FIXED_PARTITION_ID(hist_partition)
#define HIST_PARTITION_SIZE FIXED_PARTITION_SIZE(hist_partition)
#define NUM_BATT_HIST_ENTRIES ((HIST_PARTITION_SIZE) / sizeof(runtime_battery_data_t))

static runtime_battery_data_t last_valid_history_entry;
static struct fcb_entry last_valid_loc;
static uint16_t reset_cycle_count;
static struct fcb hist_fcb;

static void batt_hist_find_last(void);

uint32_t crc32(uint8_t *src, size_t len, uint32_t crc)
{
    return 0;
}

void batt_hist_init(void)
{
    int rc = fcb_init(HIST_PARTITION, &hist_fcb);

    if (rc) {
        LOG_ERR("Error initializing battery history FCB: %d", rc);
    } else {
        LOG_DBG("Battery history FCB initialized");
        batt_hist_find_last();
    }
}

static void runtime_entry_print(runtime_battery_data_t *data, unsigned int start, unsigned int count, const char *prefix)
{
#if DEBUG_PRINT
    (void)data;
#endif
    LOG_DBG("%srst_cycle=%d, minute=%d, unused=%d, estimated=%d, crc=0x%08X",
            prefix ? prefix : "",
            data->rst_cycle, data->minute, data->unused, data->estimated, data->crc);
    for (unsigned int j = start; j < start + count; j++) {
        LOG_DBG("   pack %u: mixcap:0x%04X, repcap:0x%04X", j + 1, data->rt_packs[j].mixcap, data->rt_packs[j].repcap);
    }
}

static void batt_hist_find_last(void)
{
    runtime_battery_data_t data;
    struct fcb_entry loc = {0};
#if VERBOSE_DEBUG
    char prefix[10];
#endif
    uint32_t check_crc;
    int rc;

    memset(&last_valid_history_entry, 0, sizeof(last_valid_history_entry));
    memset(&last_valid_loc, 0, sizeof(last_valid_loc));

    LOG_DBG("Runtime battery history:\r\nentry size=%zu, entry count=%u",
              sizeof(runtime_battery_data_t), NUM_BATT_HIST_ENTRIES);

    if (fcb_is_empty(&hist_fcb)) {
        LOG_INF("Runtime battery history is empty.");
        return;
    }

    for (unsigned int i = 0; i < NUM_BATT_HIST_ENTRIES; i++) {
        rc = fcb_getnext(&hist_fcb, &loc);
        if (rc) {
            LOG_ERR("Error getting next FCB entry: %d", rc);
            break;
        }
        rc = fcb_flash_read(&hist_fcb, loc.fe_sector, loc.fe_data_off, &data, loc.fe_data_len);
        if (rc) {
            LOG_ERR("Error reading next FCB entry: %d", rc);
            break;
        }
#if VERBOSE_DEBUG
        snprintk(prefix, sizeof(prefix), "%d. ", i);
        runtime_entry_print(&data, 0, NUM_PACKS, prefix);
#endif // VERBOSE_DEBUG
        check_crc = crc32((uint8_t *)&data, sizeof(data) - sizeof(data.crc), 0);

        if (check_crc == data.crc) {
            last_valid_history_entry = data;
            last_valid_loc = loc;
        } else {
            LOG_DBG("CRC failure on entry %u", i);
        }
    }
    // we found a good entry, so save a copy in RAM
    if (last_valid_loc.fe_sector != NULL) {
        LOG_DBG("Selected last valid runtime entry:");
        runtime_entry_print(&last_valid_history_entry, 0, NUM_PACKS, NULL);

        reset_cycle_count = last_valid_history_entry.rst_cycle;

        // We have started a new cycle, so advance this counter once
        reset_cycle_count++;
    }
}

static void batt_hist_utilize(runtime_battery_data_t *src, pack_t *pack)
{
    unsigned int i = pack->pack_number - 1;
    uint16_t tmp;
    int rc;

    tmp = MIN(src->rt_packs[i].mixcap, CELL_CAPACITY_MAH_RAW);
    rc = max17205_reg_write(pack->dev, MAX17205_AD_MIXCAP, tmp);
    if (rc) {
        LOG_DBG("Error writing AD_MIXCAP");
    }
    tmp = MIN(src->rt_packs[i].repcap, CELL_CAPACITY_MAH_RAW);
    rc = max17205_reg_write(pack->dev, MAX17205_AD_REPCAP, tmp);
    if (rc) {
        LOG_DBG("Error writing AD_REPCAP");
    }
}

void batt_hist_load_latest(pack_t *pack)
{
    if ((last_valid_history_entry.minute == 0) && (last_valid_history_entry.crc == 0)) {
        LOG_DBG("No latest battery history to load");
        return;
    }

    LOG_DBG("Loading entry to pack %u:", pack->pack_number);
    runtime_entry_print(&last_valid_history_entry, pack->pack_number - 1, 1, NULL);

    batt_hist_utilize(&last_valid_history_entry, pack);
}

static void batt_hist_create(runtime_battery_data_t *dest)
{
    pack_t *pack;
    uint16_t tmp;
    int rc;

    for (unsigned int i = 0; i < NUM_PACKS; i++) {
        pack = get_pack(i);
        rc = max17205_reg_read(pack->dev, MAX17205_AD_MIXCAP, &tmp);
        if (rc) {
            dest->rt_packs[i].mixcap = 0;
        } else {
            // clamp to design limit to handle case where full pack is in storage,
            // self discharges, then is charged later -- MAX17205 will then make max cap
            // higher than it should be
            dest->rt_packs[i].mixcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
        }
        rc = max17205_reg_read(pack->dev, MAX17205_AD_REPCAP, &tmp);
        if (rc) {
            dest->rt_packs[i].repcap = 0;
        } else {
            dest->rt_packs[i].repcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
        }
    }
    dest->rst_cycle = reset_cycle_count;
    dest->minute = k_uptime_seconds() / 60;
    dest->unused = 0;
    dest->estimated = false;
    dest->crc = crc32((uint8_t *)dest, sizeof(*dest) - sizeof(dest->crc), 0);
}

static bool batt_hist_add_next(runtime_battery_data_t *new_data)
{
    int rc;
    bool ok = true;

    rc = fcb_append(&hist_fcb, sizeof(runtime_battery_data_t), &last_valid_loc);
    if (rc) {
        LOG_ERR("Error on fcb_append(): %d", rc);
        return false;
    }
    rc = fcb_flash_write(&hist_fcb, last_valid_loc.fe_sector, last_valid_loc.fe_data_off, new_data, last_valid_loc.fe_data_len);
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

bool batt_hist_store_current(void)
{
    runtime_battery_data_t new_data;

#if CONFIG_HIST_STORE_PROMPT
    LOG_DBG("********** Store batt_hist e(rase), y(es), n(o)? ");
    uint8_t ch = console_getchar(); // TODO: Zephyr console does not have a read timeout; switch to shell

    LOG_DBG("");
    if (ch == 'e') {
        LOG_DBG("Erasing *************");
        flash_area_erase(HIST_PARTITION, 0, HIST_PARTITION_SIZE);
        return true;
    } else if (ch != 'y') {
        LOG_DBG("Not storing ************");
        return true;
    }
#endif // CONFIG_HIST_STORE_PROMPT

    batt_hist_create(&new_data);

    LOG_DBG("Storing new runtime battery entry:");
    runtime_entry_print(&new_data, 0, NUM_PACKS, NULL);

    if (batt_hist_add_next(&new_data)) {
            LOG_DBG("Done.");
            return true;
    }
    LOG_DBG("ERROR: Unable to store current battery history.");
    return false;
}

