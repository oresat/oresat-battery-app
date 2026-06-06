#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>
#include <zephyr/drivers/gpio.h>
#include <canopennode.h>
#include <CO_OD.h>
#include <oresat.h>
#include <board_sensors.h>

#include "batt.h"
#include "max17205_intf.h"
#include "hist.h"

#include <sys/param.h>
#include <string.h>

// TODO:
// - re-review writes to N-version of registers vs. non-N
// - do we need to still deal with PACKCFG here? done in the driver now <-- remove once tested to confirm driver is working

LOG_MODULE_DECLARE(app_battery, CONFIG_APP_BATTERY_LOG_LEVEL);

#define BATT_THREAD_STACK_SIZE 2048
#define BATT_THREAD_PRIORITY 0
extern const k_tid_t batt_id;

K_EVENT_DEFINE(batt_event);

// Some of the code and data below requires 2 packs of 2 cells. Any other configuration may require changes.
#if (NUM_CELLS != 2)
#error "NUM_CELLS is not 2!"
#endif
#if (NUM_PACKS != 2)
#error "NUM_PACKS is not 2!"
#endif

#define NV_WRITE_PROMPT_TIMEOUT_S 15

// Voltage below which we should stop everything until charging starts
#define SHUTDOWN_MV 2850

// Subtask periods
#define PACK_HIST_STORE_INTERVAL_MS 300000U // 5 minutes
#define LED_TOGGLE_INTERVAL_MS 500 // 0.5 seconds
#define DATA_INTERVAL_MS 4000 // 4 seconds
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
static const struct gpio_dt_spec can_shutdown = GPIO_DT_SPEC_GET(BP_NODE, can_shutdown_gpios);
static const struct gpio_dt_spec can_silent = GPIO_DT_SPEC_GET(BP_NODE, can_silent_gpios);

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

// raw register values to store at runtime per pack;
// restoring after a reset should result in accurate
// fuel gauge estimates
typedef struct __attribute__((packed)) runtime_pack_data {
	uint16_t mixcap;			// MAX17205_AD_MIXCAP
	uint16_t repcap;			// MAX17205_AD_REPCAP
} pack_hist_data_t;

static pack_hist_data_t hist_data[NUM_PACKS];

// Failsafe flags -- tolerate hardware failures on everything not essential to controlling heaters
static unsigned num_packs_usable;

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

	ret = gpio_pin_configure_dt(&can_shutdown, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&can_silent, GPIO_OUTPUT_INACTIVE);
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
		ret = gpio_pin_configure_dt(&packs[pack].line_chg_stat, GPIO_INPUT | GPIO_PULL_UP);
		if (ret) {
			return ret;
		}
		ret = gpio_pin_configure_dt(&packs[pack].line_dchg_stat, GPIO_INPUT | GPIO_PULL_UP);
		if (ret) {
			return ret;
		}
		ret = gpio_pin_configure_dt(&packs[pack].line_alert, GPIO_INPUT | GPIO_PULL_UP);
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

#if defined(CONFIG_BOARD_ORESAT_STM32_BATTERY_CARD)
//static const struct device *const dev_vref = DEVICE_DT_GET_ONE(st_stm32_vref);
#if !DT_HAS_ALIAS(die_temp0)
#error No die temp!
#endif
#define TEMP_NODE DT_ALIAS(die_temp0)
static const struct device *const dev_die_temp = DEVICE_DT_GET(TEMP_NODE);

/**
 * @brief Read processor temperature
 */
int16_t read_die_temp(void)
{
	struct sensor_value die_temp;

	sensor_sample_fetch(dev_die_temp);
	sensor_channel_get(dev_die_temp, SENSOR_CHAN_DIE_TEMP, &die_temp);

	LOG_INF("Processor die temperature: %d", die_temp.val1);
	return (int16_t)die_temp.val1;
}
#else
#error "read_die_temp is not yet supported"
#endif

/**
 * @brief Runs the battery state machine, responsible for turning on/off heaters, charging, discharging etc.
 */
#if CONFIG_ENABLE_HEATERS
static void run_battery_heating_state_machine(void)
{
	unsigned int i;

	if (num_packs_usable == NUM_PACKS) {  // do not compromise battery viability if we cannot obtain capacity at all from one or neither
		for (i = 0; i < NUM_PACKS; i++) {
			if (packs[i].enabled && !packs[i].data.is_data_valid) {
				LOG_DBG("FAILSAFE: ");
				heaters_on(false);
				return;
			}
		}
	}

	uint16_t total_state_of_charge = 0;
	bool full_enough = false; // Once the batteries are > 25 percent full, it's ok to run the heaters
	bool warm_enough = false; // Once they’re greater than 5 °C, can turn off heaters
	bool too_cold = false;    // Once they’re less than -5 °C, can turn on heaters

	if (!num_packs_usable) {
		int16_t die_temp = read_die_temp();

		total_state_of_charge = 50;
		full_enough = true; // we don't know so assume we're ok to run the heaters if neede
		if (die_temp < -5) {
			too_cold = true;
		}
		if (die_temp > 5) {
			warm_enough = true;
		}
	} else {
		for (i = 0; i < NUM_PACKS; i++) {
			if (packs[i].enabled) {
				total_state_of_charge += packs[i].data.present_state_of_charge;
			}
			if (packs[i].data.avg_temp_1_C < -5) {
				too_cold = true;
			}
			if (packs[i].data.avg_temp_1_C > 5) {
				warm_enough = true;
			}
		}
		total_state_of_charge /= num_packs_usable;
		if (total_state_of_charge > 25) {
			full_enough = true;
		}
	}

	switch (current_battery_state_machine_state) {
		case BATTERY_STATE_MACHINE_STATE_HEATING:
			heaters_on(true);
			if (warm_enough || !full_enough) {
				current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
				LOG_DBG("Turning heaters OFF");
			}
			break;
		case BATTERY_STATE_MACHINE_STATE_NOT_HEATING:
			heaters_on(false);
			if (too_cold && full_enough) {
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
 *	 pack data.
 */
static void update_battery_charging_state(const pack_t *pack)
{
	LOG_DBG("LINE_DCHG_STAT_PK%d = %u", pack->pack_number, gpio_pin_get_dt(&pack->line_dchg_stat));
	LOG_DBG("LINE_CHG_STAT_PK%d  = %u", pack->pack_number, gpio_pin_get_dt(&pack->line_chg_stat));
	LOG_DBG("LINE_ALERT_PK%d     = %u", pack->pack_number, gpio_pin_get_dt(&pack->line_alert));

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
 *		 pack data from
 * @param *dest[out] Destination into which to store pack data
 *    currently tracked in the MAX17205
 * @param enabled True if we can talk to this MAX17205
 *
 * @return true on success, false otherwise
 */
static bool populate_pack_data(const struct device *dev, batt_pack_data_t *dest, bool enabled)
{
	int rc;

	memset(dest, 0, sizeof(*dest));
	dest->is_data_valid = true;

	if (!enabled) { // hardware failure
		int16_t die_temp = read_die_temp();

		// data will be all zeros, but populate the temperature as die temp if we can
		dest->temp_1_C = dest->temp_2_C = die_temp;

		LOG_DBG("");
		LOG_WRN("PACK DISABLED");
		LOG_DBG("Temperature (C): die temp = %d", die_temp);
		LOG_DBG("");
		return true;
	}

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

	LOG_DBG("SOC (%%):        reported = %u%%",
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

	CO_LOCK_OD();
	if (pack->pack_number == 1) {
		CO_OD_RAM.pack_1.status = state_bitmask;
		CO_OD_RAM.pack_1.vbatt = MIN(pack_data->batt_mV, UINT16_MAX);
		CO_OD_RAM.pack_1.vcell_max = pack_data->v_cell_max_volt_mV;
		CO_OD_RAM.pack_1.vcell_min = pack_data->v_cell_min_volt_mV;
		CO_OD_RAM.pack_1.vcell = pack_data->v_cell_mV;
		CO_OD_RAM.pack_1.vcell_1 = pack_data->v_cell_1_mV;
		CO_OD_RAM.pack_1.vcell_2 = pack_data->v_cell_2_mV;
		CO_OD_RAM.pack_1.vcell_avg = pack_data->v_cell_avg_mV;
		CO_OD_RAM.pack_1.current = CLAMP(pack_data->current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_1.current_avg = CLAMP(pack_data->avg_current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_1.current_max = CLAMP(pack_data->max_current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_1.current_min = CLAMP(pack_data->min_current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_1.full_capacity = MIN(pack_data->full_capacity_mAh, UINT16_MAX);
		CO_OD_RAM.pack_1.reported_capacity = MIN(pack_data->reported_capacity_mAh, UINT16_MAX);
		CO_OD_RAM.pack_1.time_to_empty = MIN(pack_data->time_to_empty_seconds, UINT16_MAX);
		CO_OD_RAM.pack_1.time_to_full = MIN(pack_data->time_to_full_seconds, UINT16_MAX);
		CO_OD_RAM.pack_1.cycles = pack_data->cycles;
		CO_OD_RAM.pack_1.reported_state_of_charge = pack_data->reported_state_of_charge;
		CO_OD_RAM.pack_1.temperature = CLAMP(pack_data->temp_1_C, INT8_MIN, INT8_MAX);
		CO_OD_RAM.pack_1.temperature_avg = CLAMP(pack_data->avg_temp_1_C, INT8_MIN, INT8_MAX);
		CO_OD_RAM.pack_1.temperature_max = pack_data->temp_max_C;
		CO_OD_RAM.pack_1.temperature_min = pack_data->temp_min_C;
	} else if (pack->pack_number == 2) {
		CO_OD_RAM.pack_2.status = state_bitmask;
		CO_OD_RAM.pack_2.vbatt = MIN(pack_data->batt_mV, UINT16_MAX);
		CO_OD_RAM.pack_2.vcell_max = pack_data->v_cell_max_volt_mV;
		CO_OD_RAM.pack_2.vcell_min = pack_data->v_cell_min_volt_mV;
		CO_OD_RAM.pack_2.vcell = pack_data->v_cell_mV;
		CO_OD_RAM.pack_2.vcell_1 = pack_data->v_cell_1_mV;
		CO_OD_RAM.pack_2.vcell_2 = pack_data->v_cell_2_mV;
		CO_OD_RAM.pack_2.vcell_avg = pack_data->v_cell_avg_mV;
		CO_OD_RAM.pack_2.current = CLAMP(pack_data->current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_2.current_avg = CLAMP(pack_data->avg_current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_2.current_max = CLAMP(pack_data->max_current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_2.current_min = CLAMP(pack_data->min_current_mA, INT16_MIN, INT16_MAX);
		CO_OD_RAM.pack_2.full_capacity = MIN(pack_data->full_capacity_mAh, UINT16_MAX);
		CO_OD_RAM.pack_2.reported_capacity = MIN(pack_data->reported_capacity_mAh, UINT16_MAX);
		CO_OD_RAM.pack_2.time_to_empty = MIN(pack_data->time_to_empty_seconds, UINT16_MAX);
		CO_OD_RAM.pack_2.time_to_full = MIN(pack_data->time_to_full_seconds, UINT16_MAX);
		CO_OD_RAM.pack_2.cycles = pack_data->cycles;
		CO_OD_RAM.pack_2.reported_state_of_charge = pack_data->reported_state_of_charge;
		CO_OD_RAM.pack_2.temperature = CLAMP(pack_data->temp_2_C, INT8_MIN, INT8_MAX);
		CO_OD_RAM.pack_2.temperature_avg = CLAMP(pack_data->avg_temp_2_C, INT8_MIN, INT8_MAX);
		CO_OD_RAM.pack_2.temperature_max = pack_data->temp_max_C;
		CO_OD_RAM.pack_2.temperature_min = pack_data->temp_min_C;
	} else {
		LOG_DBG("ERROR: pack number not expected: %d", pack->pack_number);
	}
	CO_UNLOCK_OD();
}

static bool are_batteries_critically_low(void)
{
	unsigned int i;
	bool critically_low = false;

	for (i = 0; i < NUM_PACKS; i++) {
		if (!packs[i].enabled) {
			continue;
		}
		if ((packs[i].data.v_cell_1_mV < SHUTDOWN_MV) || (packs[i].data.v_cell_2_mV < SHUTDOWN_MV)) {
			LOG_WRN("Batteries are critically low! Pack %u Cell 1:%u mV, Cell 2:%u mV Batt:%u mV", i + 1,
					packs[i].data.v_cell_1_mV, packs[i].data.v_cell_2_mV, packs[i].data.batt_mV);
			critically_low = true;
		}
	}

	if (!critically_low) {
		LOG_INF("Batteries are not critically low");
	}
	return critically_low;
}

static bool check_for_critically_low_batteries(void)
{
	int rc;
	unsigned int i;
	uint16_t v1;
	uint16_t v2;
	uint32_t vbatt;

	LOG_INF("Check for critically low batteries");
	for (i = 0; i < NUM_PACKS; i++) {
		if (!packs[i].enabled) {
			LOG_DBG("p:%u fuel gauge disabled", i);
			continue;
		}
		if ((rc = max17205_read_batt(packs[i].dev, &vbatt)) != 0) {
			vbatt = 0;
		}
		if ((rc = max17205_read_voltage(packs[i].dev, MAX17205_CHAN_V_CELL_1, &v1)) != 0) {
			v1 = 0;
		}
		v2 = (uint16_t)vbatt - v1;
		LOG_DBG("p:%u, vbatt:%u, v1:%u, v2:%u", i + 1, vbatt, v1, v2);
		packs[i].data.batt_mV = vbatt;
		packs[i].data.v_cell_1_mV = v1;
		packs[i].data.v_cell_2_mV = v2;
	}
	return are_batteries_critically_low();
}

static void populate_pack_hist_data(const struct device *dev, pack_hist_data_t *data)
{
	uint16_t tmp;
	int rc;

	rc = max17205_reg_read(dev, MAX17205_AD_MIXCAP, &tmp);
	if (rc) {
		data->mixcap = 0;
		LOG_DBG("Unable to read AD_MIXCAP");
	} else {
		// clamp to design limit to handle case where full pack is in storage,
		// self discharges, then is charged later -- MAX17205 will then make max cap
		// higher than it should be
		data->mixcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
		LOG_DBG("Read 0x%04X from AD_MIXCAP", tmp);
	}
	rc = max17205_reg_read(dev, MAX17205_AD_REPCAP, &tmp);
	if (rc) {
		data->repcap = 0;
		LOG_DBG("Unable to read AD_REPCAP");
	} else {
		data->repcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
		LOG_DBG("Read 0x%04X from AD_REPCAP", tmp);
	}
}

static void utilize_pack_hist_data(const struct device *dev, const pack_hist_data_t *data)
{
	uint16_t tmp;
	int rc;

	tmp = MIN(data->mixcap, CELL_CAPACITY_MAH_RAW);
	if (tmp) {
		rc = max17205_reg_write(dev, MAX17205_AD_MIXCAP, tmp);
		if (rc) {
			LOG_DBG("Error writing AD_MIXCAP");
		} else {
			LOG_DBG("Wrote 0x%04X to AD_MIXCAP", tmp);
		}
	} else {
		LOG_DBG("Leaving AD_MIXCAP at default");
	}
	tmp = MIN(data->repcap, CELL_CAPACITY_MAH_RAW);
	if (tmp) {
		rc = max17205_reg_write(dev, MAX17205_AD_REPCAP, tmp);
		if (rc) {
			LOG_DBG("Error writing AD_REPCAP");
		} else {
			LOG_DBG("Wrote 0x%04X to AD_REPCAP", tmp);
		}
	} else {
		LOG_DBG("Leaving AD_REPCAP at default");
	}
}

/* Battery monitoring thread */
static void handle_batt(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;
	unsigned int i;
	int rc;

	k_thread_name_set(batt_id, "battery_thread");
	LOG_INF("Starting battery thread");

	// Ensure value in batt.h matches size of array above
	__ASSERT((sizeof(pack_hist_data_t) * NUM_PACKS) == HIST_DATA_SIZE, "sizeof(hist_data) must match HIST_DATA_SIZE");

	packs[0].dev = DEVICE_DT_GET(DT_NODELABEL(pack1));
	packs[1].dev = DEVICE_DT_GET(DT_NODELABEL(pack2));
	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

	console_init();

	if (!device_is_ready(led.port)) {
		LOG_ERR("LED is not ready.");
		goto fatal_exit;
	}
	rc = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (rc) {
		LOG_ERR("Error configuring LED output: %d", rc);
		goto fatal_exit;
	}

	if (!device_is_ready(packs[0].dev)) {
		LOG_ERR("sensor: device pack1 not ready.");
		packs[0].enabled = false;
	} else {
		packs[0].enabled = true;
		num_packs_usable++;
	}
	if (!device_is_ready(packs[1].dev)) {
		LOG_ERR("sensor: device pack2 not ready.");
		packs[1].enabled = false;
	} else {
		packs[1].enabled = true;
		num_packs_usable++;
	}

	if ((rc = hist_init()) != 0) {
		goto fatal_exit;
	}

	if ((rc = heaters_init()) != 0) {
		LOG_ERR("Error initializing heaters: %d", rc);
		goto fatal_exit;
	}
	if (IS_ENABLED(CONFIG_ENABLE_HEATERS)) {
		LOG_INF("HEATERS ENABLED IN BUILD");
	}
	heaters_on(false);

	check_for_critically_low_batteries();
	k_msleep(3000);

	for (i = 0; i < NUM_PACKS; i++) {
		if (!packs[i].enabled) {
			LOG_WRN(" **** PACK %d DISABLED ****", i + 1);
			continue;
		}
		LOG_INF("**** PACK %d ****", i + 1);

		LOG_INF("Check config registers; update if needed");
		packs[i].updated = nv_ram_write(packs[i].dev, packs[i].name);

		LOG_DBG("**** PACK %d ****", i + 1);
		max17205_print_volatile_memory(packs[i].dev);
#if VERBOSE_DEBUG
		LOG_DBG("**** PACK %d ****", i + 1);
		max17205_print_nonvolatile_memory(packs[i].dev);
		max17205_read_history(packs[i].dev);
#endif
	}

	// Let MAX17205s startup and run for a bit. Overwriting the MIXCAP and REPCAP values
	// too quickly leads to bad measurements otherwise.
	k_msleep(500);

	// Retrieve and use most recently stored hist data from flash
	LOG_INF("Loading most recent capacity history");
	if (hist_load_current((uint8_t *)hist_data)) {
		for (i = 0; i < NUM_PACKS; i++) {
			if (packs[i].enabled) {
				LOG_DBG("Loading entry to pack %u:", i);
				utilize_pack_hist_data(packs[i].dev, &hist_data[i]);
			}
		}
	}

	uint32_t loop = 0;
	int64_t ms = 0;
	int64_t now = k_uptime_get();
	int64_t next_hist_update_ms = PACK_HIST_STORE_INTERVAL_MS + now;
	int64_t next_led_update_ms = LED_TOGGLE_INTERVAL_MS + now;
	int64_t next_data_update_ms = DATA_INTERVAL_MS + now;

	LOG_INF("Entering main battery loop");
	for (;;) {
		ms = k_uptime_get();

		if (ms >= next_hist_update_ms) {
			next_hist_update_ms = ms + PACK_HIST_STORE_INTERVAL_MS;

			for (i = 0; i < NUM_PACKS; i++) {
				if (packs[i].enabled) {
					populate_pack_hist_data(packs[i].dev, &hist_data[i]);
				}
			}
			LOG_INF("Storing capacity history");
			hist_store_current((const uint8_t *)hist_data);
		}

		if (ms >= next_led_update_ms) {
			next_led_update_ms = ms + LED_TOGGLE_INTERVAL_MS;
			gpio_pin_toggle_dt(&led);
		}

		if (ms >= next_data_update_ms) {
			next_data_update_ms = ms + DATA_INTERVAL_MS;
			loop++;
			// fall through
		} else {
			k_msleep(BATT_TASK_SLEEP_INTERVAL_MS);
			continue;
		}

		LOG_INF("================================= loop %u, %u.%03u s", loop, (uint32_t)(ms / 1000), (uint32_t)(ms % 1000));

		(void)read_die_temp();
		for (i = 0; i < NUM_PACKS; i++) {
			LOG_INF("Read %s data; send to CAN", packs[i].name);

			if (populate_pack_data(packs[i].dev, &packs[i].data, packs[i].enabled) ) {
				board_sensors_fill_od();
				populate_od_pack_data(&packs[i]);
			} else {
				LOG_WRN("Data not valid!");
			}
			update_battery_charging_state(&packs[i]);
		}

		/* Notify any other waiting threads the data is ready */
		k_event_post(&batt_event, BATT_EVENT_UPDATED);

#if CONFIG_ENABLE_HEATERS
		if (!are_batteries_critically_low()) {
			run_battery_heating_state_machine();
		}
#endif
	}

fatal_exit:
	k_msleep(1000);
	__ASSERT(false, "Fatal error");
}

uint32_t batt_event_wait(bool reset, k_timeout_t timeout)
{
	return k_event_wait(&batt_event, BATT_EVENT_UPDATED, reset, timeout);
}

void batt_close(void)
{
	LOG_ERR("Terminating battery thread...");

	for (int i = 0; i < NUM_PACKS; i++) {
		// Is there a way to stop a driver? Not currently.
		// max17205Stop(packs[i].drv);
	}

	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

	gpio_pin_set_dt(&led, GPIO_OUTPUT_INACTIVE);
}

K_THREAD_DEFINE(batt_id, BATT_THREAD_STACK_SIZE, handle_batt, NULL, NULL, NULL, BATT_THREAD_PRIORITY, 0, 0);
