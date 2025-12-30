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

// Simplify conditionals below -- DEBUG_PRINT is required for ENABLE_NV_MEMORY_UPDATE_CODE to do anything
#if DEBUG_PRINT && CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE
#define NV_WRITE_PROMPT_ENABLED 1
#else
#define NV_WRITE_PROMPT_ENABLED 0
#endif

#if DEBUG_PRINT && CONFIG_ENABLE_LEARN_COMPLETE
#define LEARN_COMPLETE_ENABLED 1
#else
#define LEARN_COMPLETE_ENABLED 0
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

#define ARRAY_LEN(x) (sizeof(x)/sizeof(x[0]))

extern pack_t *get_pack(unsigned int pack);

/* Battery monitoring thread prototypes */
extern void batt_thread_handler(void *p1, void *p2, void *p3);

#endif
