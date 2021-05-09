/*
 * Copyright (c) 2021 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stmpe1600

/**
 * @file Driver for STMPE1600 I2C-based GPIO driver.
 */
#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include "gpio_utils.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(stmpe1600);

/* Register definitions */
#define REG_CHIP_ID_LSB 0x00    /* const 0x00 */
#define REG_CHIP_ID_MSB 0x01    /* const 0x16 */
#define REG_VERSION_ID  0x02    /* Revision number (const 0x01) */
#define REG_SYS_CTRL    0x03    /* Reset and interrupt control */
#define REG_IEGPIOR_LSB 0x08    /* GPIO interrupt enable register */
#define REG_IEGPIOR_MSB 0x09
#define REG_ISGPIOR_LSB 0x0A    /* GPIO interrupt status register */
#define REG_ISGPIOR_MSB 0x0B
#define REG_GPMR_LSB    0x10    /* GPIO monitor pin state register */
#define REG_GPMR_MSB    0x11
#define REG_GPSR_LSB    0x12    /* GPIO set pin state register */
#define REG_GPSR_MSB    0x13
#define REG_GPDR_LSB    0x14    /* GPIO set pin direction register */
#define REG_GPDR_MSB    0x15
#define REG_GPPIR_LSB   0x16    /* GPIO polarity inversion register */
#define REG_GPPIR_MSB   0x17


/** Configuration data */
struct stmpe1600_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;

	/** The master I2C device's name */
	const char *const i2c_master_dev_name;

	/** The slave address of the chip */
	uint16_t i2c_slave_addr;
};

/** Runtime driver data */
struct stmpe1600_drvdata {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	/** Master I2C device */
	const struct device *i2c_master;

	/** Driver lock */
	struct k_sem lock;
};

static int write_reg16(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct stmpe1600_config *const config = dev->config;
	struct stmpe1600_drvdata *const drvdata = (struct stmpe1600_drvdata *const) dev->data;
	const struct device *i2c_master = drvdata->i2c_master;
	uint16_t i2c_addr = config->i2c_slave_addr;
	int ret = 0;
	uint16_t transfer_data = sys_cpu_to_le16(value);

	LOG_DBG("STMPE1600[0x%02X]: write REG[0x%02X..0x%02X] = %04x",
		i2c_addr, reg, reg + 1, value);

	ret = i2c_burst_write(
		i2c_master, i2c_addr, reg,
		(uint8_t *)&transfer_data, sizeof(transfer_data)
		);

	if (ret != 0) {
		LOG_ERR("STMPE1600[0x%02X]: write error REG[0x%02X..0x%02X]: %d",
			i2c_addr, reg, reg + 1, ret);
	}
	return ret;
}

static int read_reg16(const struct device *dev, uint8_t reg, uint16_t *value)
{
	const struct stmpe1600_config *const config = dev->config;
	struct stmpe1600_drvdata *const drvdata = (struct stmpe1600_drvdata *const) dev->data;
	const struct device *i2c_master = drvdata->i2c_master;
	uint16_t i2c_addr = config->i2c_slave_addr;
	int ret = 0;
	uint16_t transfer_data = 0;

	LOG_DBG("STMPE1600[0x%02X]: read REG[0x%02X..0x%02X]", i2c_addr, reg, reg + 1);

	ret = i2c_burst_read(
		i2c_master, i2c_addr, reg,
		(uint8_t *) &transfer_data, sizeof(transfer_data)
		);

	if (ret != 0) {
		LOG_ERR("STMPE1600[0x%02X]: read error REG[0x%02X..0x%02X]: %d",
			i2c_addr, reg, reg + 1, ret);
	} else {
		*value = sys_le16_to_cpu(transfer_data);
		LOG_DBG("STMPE1600[0x%02X]: read REG[0x%02X..0x%02X] => %04x",
			i2c_addr, reg, reg + 1, *value);
	}
	return ret;
}

static int set_pin_dir(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	uint16_t reg_dir = 0, reg_state = 0;
	int ret = 0;

	ret = read_reg16(dev, REG_GPDR_LSB, &reg_dir);
	if (ret) {
		return ret;
	}
	ret = read_reg16(dev, REG_GPSR_LSB, &reg_state);
	if (ret) {
		return ret;
	}

	if ((flags & GPIO_INPUT) != 0U) {
		reg_dir &= ~BIT(pin);
	} else {
		reg_dir |= BIT(pin);
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			reg_state |= BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			reg_state &= ~BIT(pin);
		}
	}

	ret = write_reg16(dev, REG_GPDR_LSB, reg_dir);
	if (ret) {
		return ret;
	}
	ret = write_reg16(dev, REG_GPSR_LSB, reg_state);
	return ret;
}

static int stmpe1600_init(const struct device *dev)
{
	const struct stmpe1600_config *const config = dev->config;
	struct stmpe1600_drvdata *const drvdata = (struct stmpe1600_drvdata *const) dev->data;
	const struct device *i2c_master;
	uint16_t chip_id = 0;
	int ret = 0;

	LOG_DBG("STMPE1600[0x%02X] init", config->i2c_slave_addr);

	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	drvdata->i2c_master = i2c_master;

	k_sem_init(&drvdata->lock, 1, 1);

	k_sem_take(&drvdata->lock, K_FOREVER);
	ret = read_reg16(dev, REG_CHIP_ID_LSB, &chip_id);
	k_sem_give(&drvdata->lock);

	if (ret) {
		LOG_ERR("STMPE1600[0x%02X]: Unable to read Chip ID", config->i2c_slave_addr);
		return ret;
	}

	if (chip_id != 0x1600) {
		LOG_ERR("STMPE1600[0x%02X]: Invalid Chip ID", config->i2c_slave_addr);
		return -EINVAL;
	}

	return ret;
}

static int stmpe1600_configure(const struct device *dev,
			       gpio_pin_t pin, gpio_flags_t flags)
{
	struct stmpe1600_drvdata *const drvdata = (struct stmpe1600_drvdata *const) dev->data;
	const struct stmpe1600_config *const config = dev->config;
	uint16_t i2c_addr = config->i2c_slave_addr;
	int ret = 0;

	/* No support for disconnected pin */
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drvdata->lock, K_FOREVER);

	ret = set_pin_dir(dev, pin, flags);
	if (ret) {
		LOG_ERR("STMPE1600[0x%X]: error setting pin direction (%d)", i2c_addr, ret);
	}

	k_sem_give(&drvdata->lock);
	return ret;
}

static int stmpe1600_port_get_raw(const struct device *dev, uint32_t *value)
{
	struct stmpe1600_drvdata *const drvdata = (struct stmpe1600_drvdata *const) dev->data;
	int ret = 0;
	uint16_t reg_value = 0;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drvdata->lock, K_FOREVER);

	ret = read_reg16(dev, REG_GPMR_LSB, &reg_value);
	if (ret == 0) {
		*value = reg_value;
	}

	k_sem_give(&drvdata->lock);
	return ret;
}

static int stmpe1600_port_set_masked_raw(const struct device *dev,
					 uint32_t mask, uint32_t value)
{
	struct stmpe1600_drvdata *const drvdata = (struct stmpe1600_drvdata *const)dev->data;
	uint16_t buf;
	int ret = 0;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drvdata->lock, K_FOREVER);

	if (mask != 0xffff) {
		ret = read_reg16(dev, REG_GPSR_LSB, &buf);
	}
	if (ret == 0) {
		buf = (buf & ~mask) | (mask & value);
		ret = write_reg16(dev, REG_GPSR_LSB, buf);
	}

	k_sem_give(&drvdata->lock);
	return ret;
}

static int stmpe1600_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	return stmpe1600_port_set_masked_raw(dev, mask, mask);
}

static int stmpe1600_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	return stmpe1600_port_set_masked_raw(dev, mask, 0);
}

static const struct gpio_driver_api stmpe1600_drv_api = {
	.pin_configure = stmpe1600_configure,
	.port_get_raw = stmpe1600_port_get_raw,
	.port_set_masked_raw = stmpe1600_port_set_masked_raw,
	.port_set_bits_raw = stmpe1600_port_set_bits_raw,
	.port_clear_bits_raw = stmpe1600_port_clear_bits_raw,
};

#define STMPE1600_INIT(inst)					     \
	static struct stmpe1600_config stmpe1600_##inst##_config = { \
		.i2c_master_dev_name = DT_INST_BUS_LABEL(inst),	     \
		.i2c_slave_addr = DT_INST_REG_ADDR(inst),	     \
	};							     \
								     \
	static struct stmpe1600_drvdata stmpe1600_##inst##_drvdata;  \
								     \
	DEVICE_DT_INST_DEFINE(inst, stmpe1600_init, NULL,	     \
			      &stmpe1600_##inst##_drvdata,	     \
			      &stmpe1600_##inst##_config,	     \
			      POST_KERNEL,			     \
			      CONFIG_GPIO_STMPE1600_INIT_PRIORITY,   \
			      &stmpe1600_drv_api);

DT_INST_FOREACH_STATUS_OKAY(STMPE1600_INIT)
