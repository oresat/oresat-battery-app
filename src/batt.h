#ifndef _BATT_H_
#define _BATT_H_

// Dump complete battery history
#if IS_ENABLED(CONFIG_VERBOSE_DEBUG)
#define VERBOSE_DEBUG 1
#else
#define VERBOSE_DEBUG 0
#endif

#if CONFIG_APP_BATTERY_LOG_LEVEL >= LOG_LEVEL_INF
#define DEBUG_PRINT 1
#else
#define DEBUG_PRINT 0
#endif

typedef struct {
	bool is_data_valid;

	uint32_t batt_mV;
	uint16_t v_cell_max_volt_mV;
	uint16_t v_cell_min_volt_mV;
	uint16_t v_cell_mV;
	uint16_t v_cell_1_mV;
	uint16_t v_cell_2_mV;
	uint16_t v_cell_avg_mV;

	int16_t current_mA;
	int16_t avg_current_mA;
	int16_t max_current_mA;
	int16_t min_current_mA;

	uint32_t full_capacity_mAh;
	uint32_t reported_capacity_mAh;

	// the next 2 are not reported over CAN
	uint32_t available_capacity_mAh;
	uint32_t mix_capacity_mAh;

	uint32_t time_to_empty_seconds;
	uint32_t time_to_full_seconds;

	uint16_t cycles; // count

	uint8_t reported_state_of_charge; //Percent

	// the next 2 are not reported over CAN
	uint8_t available_state_of_charge; //Percent
	uint8_t present_state_of_charge; //Percent

	int16_t int_temp_C;
	int16_t avg_int_temp_C;
	int8_t temp_max_C;
	int8_t temp_min_C;

	// the next 4 are not reported over CAN
	int16_t temp_1_C;
	int16_t temp_2_C;
	int16_t avg_temp_1_C;
	int16_t avg_temp_2_C;
} batt_pack_data_t;

// All state for a pack is contained in this structure.
typedef struct pack {
	bool init;
	bool updated;
	const struct device *dev;
	batt_pack_data_t data;
	const struct gpio_dt_spec heater_on;
	const struct gpio_dt_spec line_dchg_dis;
	const struct gpio_dt_spec line_chg_dis;
	const struct gpio_dt_spec line_dchg_stat;
	const struct gpio_dt_spec line_chg_stat;
	const struct gpio_dt_spec line_alert;
	uint8_t pack_number;
	char *name;
} pack_t;

// Full battery detection thresholds

#define BATT_FULL_THRESHOLD_MV 8000
#define EOC_THRESHOLD_MA 50
#define CELL_CAPACITY_MAH 2600
#define CELL_CAPACITY_MAH_RAW 0x1450

#define MAX_HIST_STORE_RETRIES 4

#define NUM_CELLS 2 /* Number of cells per pack */
#define NUM_PACKS 2 /* Number of packs */

// number of bytes of data to store in history;
// must match size of private pack_hist_data_t in batt.c
#define HIST_DATA_SIZE (4 * NUM_PACKS)

#define ARRAY_LEN(x) (sizeof(x)/sizeof(x[0]))

#define BATT_EVENT_UPDATED 0x0001

/**
 * Get access to pack_t structure for specified pack number.
 *
 * @param pack - the pack number to retrieve.
 *
 * @return pack_t* - pointer to pack structure if pack is in the
 *  	   range of 0 to (NUM_PACKS - 1), otherwise NULL.
 */
extern pack_t *get_pack(unsigned int pack);

/**
 * Wait for battery data updated event. This can be used by
 * one or more threads; all that are waiting will be woken at
 * the same time, when BATT_EVENT_UPDATED is posted.
 *
 * @param reset   - bool set true to clear already-pending
 *  			  event, otherwise leave as-is.
 * @param timeout - timeout; use K_NO_WAIT or K_FOREVER else
 *  			  a K_MSEC() or other macro for specific time
 *  			  duration.
 * #return uint32_t - 0 if timed-out, else event value that
 * 				  occurred.
 */
extern uint32_t batt_event_wait(bool reset, k_timeout_t timeout);

#endif
