#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>
#include <zephyr/drivers/gpio.h>
#include <canopennode.h>
#include <OD.h>
#include <oresat.h>

#include "batt.h"
#include "calib.h"
#include "hist.h"

//#include "max17205.h"
//#include "CANopen.h"
//#include "chtime.h"
//#include "oresat_f0.h"
//#include "flash_f0.h"
//#include "crc.h"
#include <sys/param.h>
#include <string.h>

// TODO:
// - re-review writes to N-version of registers vs. non-N
// - do we need to still deal with PACKCFG here? done in the driver now <-- remove once tested to confirm driver is working

LOG_MODULE_DECLARE(app_battery, CONFIG_APP_BATTERY_LOG_LEVEL);

// Some of the code and data below requires 2 packs of 2 cells. Any other configuration may require changes.
STATIC_ASSERT(NUM_CELLS == 2);
STATIC_ASSERT(NUM_PACKS == 2);

#if 0
#if DEBUG_PRINT
#pragma message("Debug messages enabled")
#else
#pragma message("Debug messages disabled")
#endif

#if VERBOSE_DEBUG
#pragma message("Verbose debug messages enabled")
#else
#pragma message("Verbose debug messages disabled")
#endif

// If batt_nv_programing_cfg registers do not match current, rewrite the RAM shadow then prompt to write to NV.
#if CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE
#pragma message("NV memory update code enabled")
#else
#pragma message("NV memory update code disabled")
#endif

// If state of charge is known to be full, set LS bits D6-D0 of LearnCfg register to 0b111
// and write MixCap and RepCap registers to 2600.
#if CONFIG_ENABLE_LEARN_COMPLETE
#pragma message("Enable learn complete enabled")
#else
#pragma message("Enable learn complete disabled")
#endif

// Recommend setting ENABLE_HEADERS to 0 for battery board v2.1. Otherwise, brownouts can occur, causing
// the C3 to reboot the battery board.
#if CONFIG_ENABLE_HEATERS
#pragma message("Heaters enabled")
#else
#pragma message("Heaters disabled")
#endif
#endif

#define NV_WRITE_PROMPT_TIMEOUT_S 15

// Voltage below which we should stop everything until charging starts
#define SHUTDOWN_MV 2850

// Subtask periods
#define CALIB_INTERVAL_MS 120000 // 2 minutes
#define RUNTIME_BATT_STORE_INTERVAL_MS 300000U // 5 minutes
#define LED_TOGGLE_INTERVAL_MS 500 // 0.5 seconds
#define BATT_TASK_SLEEP_INTERVAL_MS 10

typedef enum {
    BATTERY_OD_ERROR_INFO_CODE_NONE = 0,
    BATTERY_OD_ERROR_INFO_CODE_PACK_1_COMM_ERROR,
    BATTERY_OD_ERROR_INFO_CODE_PACK_2_COMM_ERROR,
    BATTERY_OD_ERROR_INFO_CODE_PACK_FAIL_SAFE_HEATING,
    BATTERY_OD_ERROR_INFO_CODE_PACK_FAIL_SAFE_CHARGING,
} battery_od_error_info_code_t;

typedef enum {
    BATTERY_STATE_MACHINE_STATE_NOT_HEATING = 0,
    BATTERY_STATE_MACHINE_STATE_HEATING,
} battery_heating_state_machine_state_t;

typedef enum {
    STATUS_BIT_HEATER = 0,
    STATUS_BIT_DCHG_DIS,
    STATUS_BIT_CHG_DIS,
    STATUS_BIT_DCHG_STAT,
    STATUS_BIT_CHG_STAT
} status_bits;

#if CONFIG_ENABLE_HEATERS
static battery_heating_state_machine_state_t current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
#endif

// All packs defined here, including non-zero initial data.
#define BP_NODE DT_NODELABEL(battpacks)

static const struct gpio_dt_spec moarpwr = GPIO_DT_SPEC_GET(BP_NODE, moarpwr_gpios);

static pack_t packs[NUM_PACKS] = {
    {
     .heater_on = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, heater_gpios, 0),
     .line_dchg_dis = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, discharge_disable_gpios, 0),
     .line_chg_dis = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, charge_disable_gpios, 0),
     .line_dchg_stat = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, discharge_stat_oc_gpios, 0),
     .line_chg_stat = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, charge_stat_oc_gpios, 0),
     .line_alert = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, alert_gpios, 0),
     .pack_number = 1,
     .name = "Pack 1"
    },
    {
     .heater_on = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, heater_gpios, 1),
     .line_dchg_dis = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, discharge_disable_gpios, 1),
     .line_chg_dis = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, charge_disable_gpios, 1),
     .line_dchg_stat = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, discharge_stat_oc_gpios, 1),
     .line_chg_stat = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, charge_stat_oc_gpios, 1),
     .line_alert = GPIO_DT_SPEC_GET_BY_IDX(BP_NODE, alert_gpios, 1),
     .pack_number = 2,
     .name = "Pack 2"
    }
};

static void batt_init(void)
{
}

#if 0
#define GPIO_NODE DT_NODELABEL(gpiob)
#define CAN_ID_PIN_0 3
#define CAN_ID_PIN_1 4
/* Not sure if these CAN ID signals are actually used, but they're on the schematic */
static int get_can_id(void)
{
    const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);
    int id = 0;
    int val;

    val = gpio_pin_configure(gpio_dev, CAN_ID_PIN_0, GPIO_INPUT | GPIO_PULL_UP);
    if (val < 0) {
        return val;
    }
    val = gpio_pin_configure(gpio_dev, CAN_ID_PIN_1, GPIO_INPUT | GPIO_PULL_UP);
    if (val < 0) {
        return val;
    }

    val = gpio_pin_get(gpio_dev, CAN_ID_PIN_0);
    if (val < 0) {
        return val;
    }
    id = val;

    val = gpio_pin_get(gpio_dev, CAN_ID_PIN_1);
    if (val < 0) {
        return val;
    }
    id |= val << 1;

    return id;
}
#endif

pack_t *get_pack(unsigned int pack)
{
    if (pack < NUM_PACKS) {
        return &packs[pack];
    } else {
        return NULL;
    }
}

static int heaters_init(void)
{
    int ret;
    int pack;

    ret = gpio_pin_configure_dt(&moarpwr, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        return ret;
    }

    for (pack = 0; pack < NUM_PACKS; pack++) {
        ret = gpio_pin_configure_dt(&packs[pack].heater_on, GPIO_OUTPUT_INACTIVE);
        if (ret) {
            return ret;
        }
        ret = gpio_pin_configure_dt(&packs[pack].line_chg_dis, GPIO_OUTPUT_INACTIVE);
        if (ret) {
            return ret;
        }
        ret = gpio_pin_configure_dt(&packs[pack].line_dchg_dis, GPIO_OUTPUT_INACTIVE);
        if (ret) {
            return ret;
        }
        ret = gpio_pin_configure_dt(&packs[pack].line_chg_stat, GPIO_INPUT);
        if (ret) {
            return ret;
        }
        ret = gpio_pin_configure_dt(&packs[pack].line_dchg_stat, GPIO_INPUT);
        if (ret) {
            return ret;
        }
        ret = gpio_pin_configure_dt(&packs[pack].line_alert, GPIO_INPUT);
        if (ret) {
            return ret;
        }
    }
    return 0;
}

static void heaters_on(bool on)
{
    if (on) {
        gpio_pin_set_dt(&moarpwr, 1);
        gpio_pin_set_dt(&packs[0].heater_on, 1);
        gpio_pin_set_dt(&packs[1].heater_on, 1);
        LOG_DBG("Heaters ON\n\n");
    } else {
        gpio_pin_set_dt(&packs[0].heater_on, 0);
        gpio_pin_set_dt(&packs[1].heater_on, 0);
        gpio_pin_set_dt(&moarpwr, 0);
        LOG_DBG("Heaters OFF\n\n");
    }
}

/**
 * @brief Runs the battery state machine, responsible for turning on/off heaters, charging, discharging etc.
 */
#if CONFIG_ENABLE_HEATERS
static void run_battery_heating_state_machine(void)
{
    unsigned int i;

    for (i = 0; i < NUM_PACKS; i++) {
        if (!packs[i].data.is_data_valid) {
            LOG_DBG("FAILSAFE: ");
            heaters_on(false);
            return;
        }
    }

    switch (current_battery_state_machine_state) {
        case BATTERY_STATE_MACHINE_STATE_HEATING:
            heaters_on(true);

            //Once they’re greater than 5 °C or the combined pack capacity is < 25%
            bool warm_enough = true;
            uint16_t total_state_of_charge = 0;
            for (i = 0; i < NUM_PACKS; i++) {
                if( packs[i].data.avg_temp_1_C <= 5 ) {
                    warm_enough = false;
                }
                total_state_of_charge += packs[i].data.present_state_of_charge;
            }
            total_state_of_charge /= NUM_PACKS;
            if( warm_enough || (total_state_of_charge < 25) ) {
                current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
                LOG_DBG("Turning heaters OFF");
            }
            break;
        case BATTERY_STATE_MACHINE_STATE_NOT_HEATING:
            heaters_on(false);

            //Once they’re less than -5 °C and the combined pack capacity is > 25%
            bool too_cold = false;
            bool full_enough = false;
            for (i = 0; i < NUM_PACKS; i++) {
                if( packs[i].data.avg_temp_1_C < -5 ) {
                    too_cold = true;
                }
                if( packs[i].data.present_state_of_charge > 25 ) {
                    full_enough = true;
                }
            }
            if( too_cold && full_enough ) {
                current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_HEATING;
                LOG_DBG("Turning heaters ON");
            }
            break;
        default:
            current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
            LOG_DBG("Unknown state: turning heaters OFF");
            break;
    }
}
#endif // CONFIG_ENABLE_HEATERS


/**
 * @brief Query the MAX17 chip for a given pack and populate *pk_data with the current status/state represented in the MAX17
 *
 * @param[in] *pack Destination into which to store MAX17205
 *       pack data.
 */
static void update_battery_charging_state(const pack_t *pack)
{
    LOG_DBG("LINE_DCHG_STAT_PK%d = %u", pack->pack_number, gpio_pin_get_dt(&pack->line_dchg_stat));
    LOG_DBG("LINE_CHG_STAT_PK%d  = %u", pack->pack_number, gpio_pin_get_dt(&pack->line_chg_stat));

#if CONFIG_ENABLE_CHARGING_CONTROL
    const batt_pack_data_t * const pk_data = &pack->data;

    if (!pk_data->is_data_valid) {
        //fail safe mode
        gpio_pin_set_dt(&pack->line_dchg_dis, 1);
        (pack->line_chg_dis);
        LOG_DBG("ERROR: %s data is invalid; disabling charging and discharging", pack->name);
        return;
    }
    if( pk_data->v_cell_mV < 3000 || pk_data->present_state_of_charge < 20 ) {
        //Disable discharge on both packs
        LOG_DBG("Disabling discharge on pack %u", pack->pack_number);
        gpio_pin_set_dt(&pack->line_dchg_dis, 1);
    } else {
        LOG_DBG("Enabling discharge on pack %u", pack->pack_number);
        //Allow discharge on both packs
        gpio_pin_set_dt(&pack->line_dchg_dis, 0);
    }


    if( pk_data->v_cell_mV > 4100 ) {
        LOG_DBG("Disabling charging on pack %u", pack->pack_number);
        gpio_pin_set_dt(&pack->line_chg_dis, 1);
    } else {
        LOG_DBG("Enabling charging on pack %u", pack->pack_number);
        gpio_pin_set_dt(&pack->line_chg_dis, 0);
    }
#endif // CONFIG_ENABLE_CHARGING_CONTROL
}

/**
 * @param dev[in] The MAX17205 driver object to use to query
 *           pack data from
 * @param *dest[out] Destination into which to store pack data
 *        currently tracked in the MAX17205
 *
 * @return true on success, false otherwise
 */
static bool populate_pack_data(const struct device *dev, batt_pack_data_t *dest)
{
    int rc;
    memset(dest, 0, sizeof(*dest));

    dest->is_data_valid = true;


    if( (rc = max17205_read_average_temperature(dev, MAX17205_CHAN_TEMP_1, &dest->temp_1_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_average_temperature(dev, MAX17205_CHAN_TEMP_2, &dest->temp_2_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_average_temperature(dev, SENSOR_CHAN_GAUGE_TEMP, &dest->int_temp_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_average_temperature(dev, MAX17205_CHAN_AVG_TEMP_1, &dest->avg_temp_1_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_average_temperature(dev, MAX17205_CHAN_AVG_TEMP_2, &dest->avg_temp_2_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_average_temperature(dev, MAX17205_CHAN_AVG_INT_TEMP, &dest->avg_int_temp_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_max_temperature(dev, &dest->temp_max_C)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_min_temperature(dev, &dest->temp_min_C)) != 0 ) {
        dest->is_data_valid = false;
    }

    /* Record pack and cell voltages to object dictionary */
    if( (rc = max17205_read_voltage(dev, MAX17205_CHAN_V_CELL_1, &dest->v_cell_1_mV)) != 0 ) {
        dest->is_data_valid = false;
    }

    if( (rc = max17205_read_voltage(dev, MAX17205_CHAN_V_CELL_AVG, &dest->v_cell_avg_mV)) != 0 ) {
    dest->is_data_valid = false;
   }
    if( (rc = max17205_read_voltage(dev, MAX17205_CHAN_V_CELL, &dest->v_cell_mV)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_batt(dev, &dest->batt_mV)) != 0 ) {
        dest->is_data_valid = false;
    }

    if( dest->is_data_valid ) {
        dest->v_cell_2_mV = dest->batt_mV - dest->v_cell_1_mV;
    }

    if( (rc = max17205_read_max_voltage(dev, &dest->v_cell_max_volt_mV)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_min_voltage(dev, &dest->v_cell_min_volt_mV)) != 0 ) {
        dest->is_data_valid = false;
    }

    if( (rc = max17205_read_current(dev, &dest->current_mA)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_avg_current(dev, &dest->avg_current_mA)) != 0 ) {
        dest->is_data_valid = false;
    }

    if( (rc = max17205_read_max_current(dev, &dest->max_current_mA)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_min_current(dev, &dest->min_current_mA)) != 0 ) {
        dest->is_data_valid = false;
    }

    /* capacity */
    if( (rc = max17205_read_capacity(dev, SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY, &dest->full_capacity_mAh)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_capacity(dev, SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY, &dest->available_capacity_mAh)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_capacity(dev, MAX17205_CHAN_MIX_CAPACITY, &dest->mix_capacity_mAh)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_capacity(dev, SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY, &dest->reported_capacity_mAh)) != 0 ) {
        dest->is_data_valid = false;
    }

    /* state of charge */
    if( (rc = max17205_read_time(dev, SENSOR_CHAN_GAUGE_TIME_TO_EMPTY, &dest->time_to_empty_seconds)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_time(dev, SENSOR_CHAN_GAUGE_TIME_TO_FULL, &dest->time_to_full_seconds)) != 0 ) {
        dest->is_data_valid = false;
    }

    if( (rc = max17205_read_percentage(dev, MAX17205_CHAN_AVAILABLE_SOC, &dest->available_state_of_charge)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_percentage(dev, MAX17205_CHAN_PRESENT_SOC, &dest->present_state_of_charge)) != 0 ) {
        dest->is_data_valid = false;
    }
    if( (rc = max17205_read_percentage(dev, MAX17205_CHAN_REPORTED_SOC, &dest->reported_state_of_charge)) != 0 ) {
        dest->is_data_valid = false;
    }

    /* other info */
    if( (rc = max17205_read_cycles(dev, &dest->cycles)) != 0 ) {
        dest->is_data_valid = false;
    }

    LOG_DBG("");

    LOG_DBG("Temperature (C): Th1: avg = %d, cur = %d, Th2: avg = %d, cur = %d, Int: avg = %d, cur = %d, max = %d, min = %d",
              dest->avg_temp_1_C, dest->temp_1_C, dest->avg_temp_2_C, dest->temp_2_C, dest->avg_int_temp_C, dest->int_temp_C, dest->temp_min_C, dest->temp_max_C);

    LOG_DBG("Voltage (mV):    cell1 = %u, cell2 = %u, vcell = %u, max = %d, min %d, batt = %u",
              dest->v_cell_1_mV, dest->v_cell_2_mV, dest->v_cell_mV, dest->v_cell_max_volt_mV, dest->v_cell_min_volt_mV, dest->batt_mV);

    LOG_DBG("Current (mA):    cur = %d, max = %d, min = %d, avg = %d",
              dest->current_mA, dest->max_current_mA, dest->min_current_mA, dest->avg_current_mA);

    LOG_DBG("Capacity (mAh):  full = %u, available = %u, mix = %u, reported = %u",
              dest->full_capacity_mAh, dest->available_capacity_mAh, dest->mix_capacity_mAh, dest->reported_capacity_mAh);

    LOG_DBG("Time (seconds):  to_empty =%u, to_full = %u",
              dest->time_to_empty_seconds, dest->time_to_full_seconds);

    LOG_DBG("SOC (%%):         reported = %u%%",
              dest->reported_state_of_charge);

    LOG_DBG("cycles = %u", dest->cycles);

    LOG_DBG("");

    return dest->is_data_valid;
}

/**
 * @brief Populates CANOpen data structure values with values from the current battery pack data.
 *
 * @param *pack_data[in] Source of data for populating/publishing pack data.
 */
static void populate_od_pack_data(pack_t *pack)
{
    batt_pack_data_t *pack_data = &pack->data;
    uint8_t state_bitmask = 0;

    if( gpio_pin_get_dt(&pack->heater_on) ) {
        state_bitmask |= (1 << STATUS_BIT_HEATER);
    }
    if( gpio_pin_get_dt(&pack->line_dchg_dis) ) {
        state_bitmask |= (1 << STATUS_BIT_DCHG_DIS);
    }
    if (gpio_pin_get_dt(&pack->line_chg_dis) ) {
        state_bitmask |= (1 << STATUS_BIT_CHG_DIS);
    }
    if( gpio_pin_get_dt(&pack->line_dchg_stat) ) {
        state_bitmask |= (1 << STATUS_BIT_DCHG_STAT);
    }
    if( gpio_pin_get_dt(&pack->line_chg_stat) ) {
        state_bitmask |= (1 << STATUS_BIT_CHG_STAT);
    }

    if (pack->pack_number == 1) {
        OD_RAM.x4000_pack_1.status = state_bitmask;
        OD_RAM.x4000_pack_1.vbatt = MIN(pack_data->batt_mV, UINT16_MAX);
        OD_RAM.x4000_pack_1.vcell_max = pack_data->v_cell_max_volt_mV;
        OD_RAM.x4000_pack_1.vcell_min = pack_data->v_cell_min_volt_mV;
        OD_RAM.x4000_pack_1.vcell = pack_data->v_cell_mV;
        OD_RAM.x4000_pack_1.vcell_1 = pack_data->v_cell_1_mV;
        OD_RAM.x4000_pack_1.vcell_2 = pack_data->v_cell_2_mV;
        OD_RAM.x4000_pack_1.vcell_avg = pack_data->v_cell_avg_mV;
        OD_RAM.x4000_pack_1.current = CLAMP(pack_data->current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.current_avg = CLAMP(pack_data->avg_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.current_max = CLAMP(pack_data->max_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.current_min = CLAMP(pack_data->min_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.full_capacity = MIN(pack_data->full_capacity_mAh, UINT16_MAX);
        OD_RAM.x4000_pack_1.reported_capacity = MIN(pack_data->reported_capacity_mAh, UINT16_MAX);
        OD_RAM.x4000_pack_1.time_to_empty = MIN(pack_data->time_to_empty_seconds, UINT16_MAX);
        OD_RAM.x4000_pack_1.time_to_full = MIN(pack_data->time_to_full_seconds, UINT16_MAX);
        OD_RAM.x4000_pack_1.cycles = pack_data->cycles;
        OD_RAM.x4000_pack_1.reported_state_of_charge = pack_data->reported_state_of_charge;
        OD_RAM.x4000_pack_1.temperature = CLAMP(pack_data->temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4000_pack_1.temperature_avg = CLAMP(pack_data->avg_temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4000_pack_1.temperature_max = pack_data->temp_max_C;
        OD_RAM.x4000_pack_1.temperature_min = pack_data->temp_min_C;
    } else if (pack->pack_number == 2) {
        OD_RAM.x4001_pack_2.status = state_bitmask;
        OD_RAM.x4001_pack_2.vbatt = MIN(pack_data->batt_mV, UINT16_MAX);
        OD_RAM.x4001_pack_2.vcell_max = pack_data->v_cell_max_volt_mV;
        OD_RAM.x4001_pack_2.vcell_min = pack_data->v_cell_min_volt_mV;
        OD_RAM.x4001_pack_2.vcell = pack_data->v_cell_mV;
        OD_RAM.x4001_pack_2.vcell_1 = pack_data->v_cell_1_mV;
        OD_RAM.x4001_pack_2.vcell_2 = pack_data->v_cell_2_mV;
        OD_RAM.x4001_pack_2.vcell_avg = pack_data->v_cell_avg_mV;
        OD_RAM.x4001_pack_2.current = CLAMP(pack_data->current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.current_avg = CLAMP(pack_data->avg_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.current_max = CLAMP(pack_data->max_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.current_min = CLAMP(pack_data->min_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.full_capacity = MIN(pack_data->full_capacity_mAh, UINT16_MAX);
        OD_RAM.x4001_pack_2.reported_capacity = MIN(pack_data->reported_capacity_mAh, UINT16_MAX);
        OD_RAM.x4001_pack_2.time_to_empty = MIN(pack_data->time_to_empty_seconds, UINT16_MAX);
        OD_RAM.x4001_pack_2.time_to_full = MIN(pack_data->time_to_full_seconds, UINT16_MAX);
        OD_RAM.x4001_pack_2.cycles = pack_data->cycles;
        OD_RAM.x4001_pack_2.reported_state_of_charge = pack_data->reported_state_of_charge;
        OD_RAM.x4001_pack_2.temperature = CLAMP(pack_data->temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4001_pack_2.temperature_avg = CLAMP(pack_data->avg_temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4001_pack_2.temperature_max = pack_data->temp_max_C;
        OD_RAM.x4001_pack_2.temperature_min = pack_data->temp_min_C;
    } else {
        LOG_DBG("ERROR: pack number not expected: %d", pack->pack_number);
    }
}

static bool are_batteries_critically_low(void)
{
    unsigned int i;

    for (i = 0; i < NUM_PACKS; i++) {
        if ((packs[i].data.v_cell_1_mV < SHUTDOWN_MV) || (packs[i].data.v_cell_2_mV < SHUTDOWN_MV)) {
            LOG_DBG("Batteries are critically low!");
            return true;
        }
    }

    LOG_DBG("Batteries are not critically low");
    return false;
}

static bool check_for_critically_low_batteries(void)
{
    int rc;
    unsigned int i;

    LOG_DBG("Check for critically low batteries");
    for (i = 0; i < NUM_PACKS; i++) {
        if ((rc = max17205_read_voltage(packs[i].dev, MAX17205_CHAN_V_CELL_1, &packs[i].data.v_cell_1_mV)) != 0) {
            packs[i].data.v_cell_1_mV = 0;
        }
        if ((rc = max17205_read_voltage(packs[i].dev, MAX17205_CHAN_V_CELL_2, &packs[i].data.v_cell_2_mV)) != 0) {
            packs[i].data.v_cell_2_mV = 0;
        }
    }
    return are_batteries_critically_low();
}

/* Battery monitoring thread */
void batt_thread_handler(void *p1, void *p2, void *p3)
{
    (void)p1;
    (void)p2;
    (void)p3;
    unsigned int i;

	packs[0].dev = DEVICE_DT_GET(DT_NODELABEL(pack1));
	packs[1].dev = DEVICE_DT_GET(DT_NODELABEL(pack2));
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

    console_init();

	if (!device_is_ready(packs[0].dev)) {
		printk("sensor: device pack1 not ready.\n");
		return;
	}
	if (!device_is_ready(packs[1].dev)) {
		printk("sensor: device pack2 not ready.\n");
		return;
	}
    if (!device_is_ready(led.port)) {
        return;
    }
    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE)) {
        return;
    }

    batt_init();
    batt_hist_init();
    heaters_init();
    heaters_on(false);

    check_for_critically_low_batteries();

    for (i = 0; i < NUM_PACKS; i++) {
        packs[i].updated = nv_ram_write(packs[i].dev, packs[i].name);

        max17205_print_volatile_memory(packs[i].dev);
        max17205_print_nonvolatile_memory(packs[i].dev);
#if VERBOSE_DEBUG
        max17205_read_history(packs[i].dev);
#endif
    }

    // Let MAX17205s startup and run for a bit. Overwriting the MIXCAP and REPCAP values
    // too quickly leads to bad measurements otherwise.
    k_msleep(500);
    for (i = 0; i < NUM_PACKS; i++) {
        batt_hist_load_latest(&packs[i]);
    }

    uint32_t loop = 0;
    int64_t ms = 0;
    int64_t next_hist_update_ms = RUNTIME_BATT_STORE_INTERVAL_MS + k_uptime_get();
    int64_t next_led_update_ms = LED_TOGGLE_INTERVAL_MS + k_uptime_get();
    int64_t next_calib_update_ms = CALIB_INTERVAL_MS + k_uptime_get();

    for (;;) {
        ms = k_uptime_get();

        if (ms >= next_calib_update_ms) {
            next_calib_update_ms = ms + CALIB_INTERVAL_MS;
            manage_calibration();
        }

        if (ms >= next_hist_update_ms) {
            next_hist_update_ms = ms + RUNTIME_BATT_STORE_INTERVAL_MS;
            batt_hist_store_current();
        }

        if (ms >= next_led_update_ms) {
            next_hist_update_ms = ms + LED_TOGGLE_INTERVAL_MS;
            gpio_pin_toggle_dt(&led);
            loop++;
            if (loop % 2 == 0) {
                continue; // we want light to blink at 2Hz, but code to run at 1Hz
            }
            // else falls through
        } else {
            k_msleep(BATT_TASK_SLEEP_INTERVAL_MS);
            continue;
        }

#if DEBUG_PRINT
        LOG_DBG("================================= loop %u, %u.%03u s", loop, (uint32_t)(ms / 1000), (uint32_t)(ms % 1000));
#endif

        for (i = 0; i < NUM_PACKS; i++) {
            LOG_DBG("Populating %s Data", packs[i].name);

            if (populate_pack_data(packs[i].dev, &packs[i].data) ) {
                populate_od_pack_data(&packs[i]);
            } else {
                // TODO: do we need to keep this around? Adding the loop results in the wrong error code on pack 2.
            }
        }

#if CONFIG_ENABLE_HEATERS
        if (!are_batteries_critically_low()) {
            run_battery_heating_state_machine();
        }
#endif

        for (i = 0; i < NUM_PACKS; i++) {
            update_battery_charging_state(&packs[i]);
        }
    }
}

void batt_close(void)
{
    LOG_DBG("Terminating battery thread...");

    for (int i = 0; i < NUM_PACKS; i++) {
        // TODO: is there a way to stop a driver?
        // max17205Stop(packs[i].drv);
    }

    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

    gpio_pin_set_dt(&led, GPIO_OUTPUT_INACTIVE);
}
