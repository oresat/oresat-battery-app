#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "max17205.h"

LOG_MODULE_REGISTER(max17205_emul, CONFIG_EMUL_LOG_LEVEL);

#define DT_DRV_COMPAT maxim_max17205
#define REG_ADDR(reg) ((reg) & 0x00FFU)

static inline uint16_t maH_to_raw(uint16_t rsense_mohms, uint32_t dest_mAh) {
  return (uint16_t)((dest_mAh * rsense_mohms * 1000U) / 5000U);
}

static inline uint16_t percentage_to_raw(uint16_t perc) { return 256U * perc; }

static inline uint16_t cycles_to_raw(uint16_t cycles) {
  return (uint16_t)(100U * cycles / 16U);
}

static inline uint16_t batt_voltage_to_raw(uint16_t mV) {
  return (uint16_t)((100U * mV) / 125U);
}
static inline uint16_t voltage_to_raw(uint16_t mV) {
  return (uint16_t)((64U * mV) / 5U);
}
static inline uint16_t min_max_voltage_to_raw(uint32_t min_mV,
                                              uint32_t max_mV) {
  if (min_mV > 5100) {
    LOG_WRN("clamping min voltage to 5100");
    min_mV = 5100;
  }
  if (max_mV > 5100) {
    LOG_WRN("clamping max voltage to 5100");
    max_mV = 5100;
  }

  uint8_t min_raw = (uint8_t)(min_mV / 20);
  uint8_t max_raw = (uint8_t)(max_mV / 20);

  return (uint16_t)(min_raw | (max_raw << 8));
}

static inline int16_t current_to_raw(uint16_t rsense_mohms, int16_t mA) {
  int64_t ret = ((int64_t)mA * ((int32_t)rsense_mohms * 10000)) / 15625;

  return (int16_t)(ret);
}

static inline uint16_t temp_to_raw(int32_t temp) {
  return (uint16_t)(256 * temp / 1000);
}

static inline int16_t avg_temp_to_raw(int16_t avg_temp) {
  int16_t raw = (int16_t)((avg_temp + 273) * 10);
  int16_t temp = (int16_t)raw / 10 - 273;
  return raw;
}

static inline uint16_t time_to_raw(uint32_t time) {
  return (uint16_t)(time * 1000U / 5625U);
}

/** Static configuration for the emulator */
struct max17205_emul_cfg {
  /** I2C address of emulator */
  uint16_t addr;
  /* Value of Rsense resistor in milliohms (typically 5 or 10) */
  uint16_t rsense_mohms;
};

struct max17205_emul_data {
  uint16_t regs[MAX17205_AD_MAXVALUE + 1];
};

static int emul_max17205_reg_write(const struct emul *target, uint16_t reg,
                                   uint16_t val) {
  struct max17205_emul_data *data = target->data;

  LOG_DBG("write: 0x%03x 0x%04x", reg, val);

  if (reg <= MAX17205_AD_MAXVALUE) {
    data->regs[reg] = val;
  } else {
    LOG_ERR("recieved invalid write address %u", reg);
    return -EIO;
  }

  return 0;
}

static int emul_max17205_reg_read(const struct emul *target, uint16_t reg,
                                  uint16_t *val) {
  struct max17205_emul_data *data = target->data;

  *val = data->regs[reg];

  LOG_DBG("read 0x%03x = 0x%04x", reg, *val);

  return 0;
}

static int max17205_emul_transfer_i2c(const struct emul *target,
                                      struct i2c_msg *msgs, int num_msgs,
                                      int addr) {
  int ret = 0;

  __ASSERT_NO_MSG(msgs && num_msgs);

  i2c_dump_msgs_rw(target->dev, msgs, num_msgs, addr, false);

  switch (num_msgs) {
  case 1:
    if (msgs[0].flags & I2C_MSG_READ) {
      LOG_ERR("Unexpected read");
      return -EIO;
    }
    if (msgs[0].len != 3) {
      LOG_ERR("Unexpected msg0 length %d", msgs->len);
      return -EIO;
    }

    ret = emul_max17205_reg_write(target, msgs[0].buf[0],
                                  sys_get_le16(msgs[0].buf + 1));
    if (ret) {
      LOG_ERR("emul_max17205_reg_write returned %d", ret);
    }
    break;
  case 2:
    if (msgs[0].flags & I2C_MSG_READ) {
      LOG_ERR("Unexpected read");
      return -EIO;
    }
    if (msgs[0].len != 2) {
      LOG_ERR("Unexpected msg0 length %d", msgs[0].len);
      return -EIO;
    }
    if (!(msgs[1].flags & I2C_MSG_READ)) {
      LOG_ERR("Unexpected write");
      return -EIO;
    }
    if (msgs[1].len != 2) {
      LOG_ERR("Unexpected msg1 length %d", msgs[1].len);
      return -EIO;
    }

    uint16_t val;
    uint16_t reg_addr = (msgs[0].buf[1] << 8) + (msgs[0].buf[0]);

    ret = emul_max17205_reg_read(target, reg_addr, &val);
    if (ret) {
      LOG_ERR("emul_max17205_reg_read returned %d", ret);
      return ret;
    }

    sys_put_le16(val, msgs[1].buf);
    break;
  default:
    LOG_ERR("Invalid number of messages: %d", num_msgs);
    return -EIO;
  }

  return ret;
}

static const struct i2c_emul_api max17205_emul_api_i2c = {
    .transfer = max17205_emul_transfer_i2c,
};

static int emul_max17205_init(const struct emul *target,
                              const struct device *parent) {
  struct max17205_emul_data *data = target->data;
  const struct max17205_emul_cfg *cfg = target->cfg;

  ARG_UNUSED(parent);

  memset(data->regs, 0, sizeof(data->regs));

  /* Status */
  data->regs[MAX17205_AD_STATUS] = MAX17205_STATUS_POR;

  /* SOC */
  data->regs[MAX17205_AD_AVSOC] = percentage_to_raw(100);
  data->regs[MAX17205_AD_VFSOC] = percentage_to_raw(100);
  data->regs[MAX17205_AD_REPSOC] = percentage_to_raw(100);

  /* Voltage */
  data->regs[MAX17205_AD_BATT] = batt_voltage_to_raw(7400);
  data->regs[MAX17205_AD_AVGCELL1] = voltage_to_raw(3700);
  data->regs[MAX17205_AD_AVGCELL2] = voltage_to_raw(3700);
  data->regs[MAX17205_AD_VCELL] = voltage_to_raw(3700);
  data->regs[MAX17205_AD_AVGVCELL] = voltage_to_raw(3700);
  data->regs[MAX17205_AD_MAXMINVOLT] = min_max_voltage_to_raw(0, 3800);

  /* Current */
  data->regs[MAX17205_AD_CURRENT] = current_to_raw(cfg->rsense_mohms, 2000);
  data->regs[MAX17205_AD_AVGCURRENT] = current_to_raw(cfg->rsense_mohms, 2000);

  /* Temperature */
  data->regs[MAX17205_AD_TEMP1] = avg_temp_to_raw(20);
  data->regs[MAX17205_AD_TEMP2] = avg_temp_to_raw(20);
  data->regs[MAX17205_AD_AVGTEMP1] = avg_temp_to_raw(20);
  data->regs[MAX17205_AD_AVGTEMP2] = avg_temp_to_raw(20);
  data->regs[MAX17205_AD_AVGINTTEMP] = avg_temp_to_raw(20);
  data->regs[MAX17205_AD_INTTEMP] = avg_temp_to_raw(20);

  /* Config */
  data->regs[MAX17205_AD_CONFIG] = 0x3C1CU;
  data->regs[MAX17205_AD_CYCLES] = cycles_to_raw(1);

  /* Capacity */
  data->regs[MAX17205_AD_REPCAP] = encode_capacity(cfg->rsense_mohms, 1000);
  data->regs[MAX17205_AD_MIXCAP] = encode_capacity(cfg->rsense_mohms, 1000);
  data->regs[MAX17205_AD_FULLCAPREP] = encode_capacity(cfg->rsense_mohms, 1000);
  data->regs[MAX17205_AD_AVCAP] = encode_capacity(cfg->rsense_mohms, 1000);

  /* Time */
  data->regs[MAX17205_AD_TTF] = time_to_raw(0);
  data->regs[MAX17205_AD_TTE] = time_to_raw(5625);

  return 0;
}

#define MAX17205_EMUL(n)                                                       \
  static struct max17205_emul_data max17205_emul_data_##n;                     \
  static const struct max17205_emul_cfg max17205_emul_cfg_##n = {              \
      .addr = DT_INST_REG_ADDR(n),                                             \
      .rsense_mohms = DT_INST_PROP(n, rsense_mohms),                           \
  };                                                                           \
  EMUL_DT_INST_DEFINE(n, emul_max17205_init, &max17205_emul_data_##n,          \
                      &max17205_emul_cfg_##n, &max17205_emul_api_i2c, NULL);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_EMUL)
