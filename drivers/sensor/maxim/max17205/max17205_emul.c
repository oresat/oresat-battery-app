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

static uint8_t const soc_percent = 75;
static uint16_t const vbat_mV = 7400;
static uint8_t const charging = 1;
static uint16_t const cycles = 12;
static int16_t const current_val = 100;
static int16_t const temp_val = (25 * 256);

/** Static configuration for the emulator */
struct max17205_emul_cfg {
  /** I2C address of emulator */
  uint16_t addr;
};

struct max17205_emul_data {
  uint16_t regs[MAX17205_AD_MAXVALUE + 1];
};

static int emul_max17205_reg_write(const struct emul *target, uint16_t reg,
                                   uint16_t val) {
  struct max17205_emul_data *data = target->data;

  LOG_DBG("0x%03x 0x%04x", reg, val);

  if (reg <= MAX17205_AD_MAXVALUE) {
    data->regs[reg] = val;
  }

  switch (reg) {
  case MAX17205_AD_COMMAND:
    if (val == MAX17205_COMMAND_HARDWARE_RESET) {
      data->regs[MAX17205_AD_STATUS] |= MAX17205_STATUS_POR;
    }
    break;
  case MAX17205_AD_STATUS:
    data->regs[MAX17205_AD_STATUS] = val;
    break;
  case MAX17205_AD_CONFIG2:
    if (val & MAX17205_CONFIG2_POR_CMD) {
      data->regs[MAX17205_AD_STATUS] &= ~MAX17205_STATUS_POR;
    }
    break;
  default:
    break;
  }

  return 0;
}

static int emul_max17205_reg_read(const struct emul *target, uint16_t reg,
                                  uint16_t *val) {
  struct max17205_emul_data *data = target->data;

  switch (reg) {
  case MAX17205_AD_STATUS:
    *val = data->regs[MAX17205_AD_STATUS];
    break;
  case MAX17205_AD_REPSOC:
  case MAX17205_AD_VFSOC:
  case MAX17205_AD_AVSOC:
    *val = (uint16_t)(((uint32_t)soc_percent * 256U));
    break;
  case MAX17205_AD_BATT:
    *val = (uint16_t)(((uint32_t)vbat_mV * 100U) / 125U);
    break;
  case MAX17205_AD_VCELL:
  case MAX17205_AD_AVGVCELL:
  case MAX17205_AD_AVGCELL1:
    *val = (uint16_t)(((uint32_t)(vbat_mV / 2) * 64U) / 5U);
    break;
  case MAX17205_AD_CURRENT:
  case MAX17205_AD_AVGCURRENT:
    *val = (uint16_t)current_val;
    break;
  case MAX17205_AD_CONFIG:
    *val = 0x3C1CU;
    break;
  case MAX17205_AD_CYCLES:
    *val = cycles;
    break;
  case MAX17205_AD_TEMP1:
  case MAX17205_AD_TEMP2:
  case MAX17205_AD_INTTEMP:
  case MAX17205_AD_AVGTEMP1:
  case MAX17205_AD_AVGTEMP2:
  case MAX17205_AD_AVGINTTEMP:
    *val = (uint16_t)(temp_val + (273 * 10));
    break;
  case MAX17205_AD_FULLCAPREP:
  case MAX17205_AD_AVCAP:
  case MAX17205_AD_MIXCAP:
  case MAX17205_AD_REPCAP:
    *val = 1000U;
    break;
  case MAX17205_AD_TTE:
  case MAX17205_AD_TTF:
    *val = 100U;
    break;
  case MAX17205_AD_MAXMINVOLT:
  case MAX17205_AD_MAXMINCURR:
  case MAX17205_AD_MAXMINTEMP:
    *val = data->regs[reg];
    break;
  case MAX17205_AD_LEARNCFG:
    *val = data->regs[reg];
    break;
  case MAX17205_AD_FSTAT:
    *val = charging ? 0 : 0;
    break;
  case MAX17205_AD_COMMSTAT:
    *val = data->regs[reg];
    break;
  default:
    if (reg <= MAX17205_AD_MAXVALUE) {
      *val = data->regs[reg];
    } else {
      LOG_ERR("Unknown register 0x%03x read", reg);
      return -EIO;
    }
    break;
  }

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
    if (msgs[0].len != 1) {
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

    ret = emul_max17205_reg_read(target, msgs[0].buf[0], &val);
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

  ARG_UNUSED(parent);

  memset(data->regs, 0, sizeof(data->regs));
  data->regs[MAX17205_AD_STATUS] = MAX17205_STATUS_POR;

  return 0;
}

#define MAX17205_EMUL(n)                                                       \
  static struct max17205_emul_data max17205_emul_data_##n;                     \
  static const struct max17205_emul_cfg max17205_emul_cfg_##n = {              \
      .addr = DT_INST_REG_ADDR(n)};                                            \
  EMUL_DT_INST_DEFINE(n, emul_max17205_init, &max17205_emul_data_##n,          \
                      &max17205_emul_cfg_##n, &max17205_emul_api_i2c, NULL);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_EMUL)
