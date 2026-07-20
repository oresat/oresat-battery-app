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

static uint8_t const soc_percent_init = 75;
static uint16_t const vbat_mV_init = 7400;
static uint8_t const charging_init = 1;
static uint16_t const cycles_init = 12;
static int16_t const current_val_init = 100;
static int16_t const temp_val_init = (25 * 256);

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

  LOG_DBG("write: 0x%03x 0x%04x", reg, val);

  if (reg <= MAX17205_AD_MAXVALUE) {
    data->regs[reg] = val;
  } else {
    LOG_ERR("recieved invalid write address %u", reg);
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
  data->regs[MAX17205_AD_AVSOC] =
      (uint16_t)(((uint32_t)soc_percent_init * 256U));
  data->regs[MAX17205_AD_BATT] =
      (uint16_t)(((uint32_t)vbat_mV_init * 100U) / 125U);
  data->regs[MAX17205_AD_AVGCELL1] =
      (uint16_t)(((uint32_t)(vbat_mV_init / 2) * 64U) / 5U);
  data->regs[MAX17205_AD_AVGCURRENT] = (uint16_t)current_val_init;
  data->regs[MAX17205_AD_CONFIG] = 0x3C1CU;
  data->regs[MAX17205_AD_CYCLES] = cycles_init;
  data->regs[MAX17205_AD_AVGINTTEMP] = (uint16_t)(temp_val_init + (273 * 10));
  data->regs[MAX17205_AD_REPCAP] = 1000u;
  data->regs[MAX17205_AD_TTF] = 100u;
  return 0;
}

#define MAX17205_EMUL(n)                                                       \
  static struct max17205_emul_data max17205_emul_data_##n;                     \
  static const struct max17205_emul_cfg max17205_emul_cfg_##n = {              \
      .addr = DT_INST_REG_ADDR(n)};                                            \
  EMUL_DT_INST_DEFINE(n, emul_max17205_init, &max17205_emul_data_##n,          \
                      &max17205_emul_cfg_##n, &max17205_emul_api_i2c, NULL);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_EMUL)
