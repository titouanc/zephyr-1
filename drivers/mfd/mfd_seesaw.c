/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adafruit_seesaw

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/mfd_seesaw.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mfd_seesaw, CONFIG_MFD_SEESAW_LOG_LEVEL);

static const char * const module_names[] = {
	[SEESAW_MOD_STATUS] = "STATUS",
	[SEESAW_MOD_GPIO] = "GPIO",
	[SEESAW_MOD_SERCOM0] = "SERCOM0",
	[SEESAW_MOD_SERCOM1] = "SERCOM1",
	[SEESAW_MOD_SERCOM2] = "SERCOM2",
	[SEESAW_MOD_SERCOM3] = "SERCOM3",
	[SEESAW_MOD_SERCOM4] = "SERCOM4",
	[SEESAW_MOD_SERCOM5] = "SERCOM5",
	[SEESAW_MOD_PWM] = "PWM",
	[SEESAW_MOD_ADC] = "ADC",
	[SEESAW_MOD_DAC] = "DAC",
	[SEESAW_MOD_INTERRUPT] = "INTERRUPT",
	[SEESAW_MOD_DAP] = "DAP",
	[SEESAW_MOD_EEPROM] = "EEPROM",
	[SEESAW_MOD_NEOPIXEL] = "NEOPIXEL",
	[SEESAW_MOD_TOUCH] = "TOUCH",
	[SEESAW_MOD_KEYPAD] = "KEYPAD",
	[SEESAW_MOD_ENCODER] = "ENCODER",
};

#define SEESAW_MODULE_NAME(_mod) \
	((_mod) < SEESAW_MOD_MAX_NUM) ? module_names[_mod] : "(unknown)"

enum seesaw_mod_status {
	STATUS_HW_ID = 0x01,
	STATUS_VERSION = 0x02,
	STATUS_OPTIONS = 0x03,
	STATUS_TEMP = 0x04,
	STATUS_SWRST = 0x7F,
};

struct seesaw_config {
	struct i2c_dt_spec i2c;
};

struct seesaw_data {
	struct k_sem sem;
	uint32_t modules;
	const struct device *claimed_modules[SEESAW_MOD_MAX_NUM];
};

int seesaw_claim_module(const struct device *dev, uint32_t module,
			const struct device *subdevice)
{
	struct seesaw_data *drv_data = dev->data;

	if (module > SEESAW_MOD_MAX_NUM) {
		LOG_ERR("Cannot claim module %d: invalid module number", module);
		return -EINVAL;
	}

	if (!(drv_data->modules & BIT(module))) {
		LOG_ERR("Cannot claim %s: not implemented", SEESAW_MODULE_NAME(module));
		return -ENOTSUP;
	}

	if (drv_data->claimed_modules[module]) {
		LOG_ERR("Cannot claim %s: already in use", SEESAW_MODULE_NAME(module));
		return -EBUSY;
	}

	drv_data->claimed_modules[module] = subdevice;
	LOG_DBG("Module %s claimed by %s", SEESAW_MODULE_NAME(module), subdevice->name);
	return 0;
}

int seesaw_cmd(const struct device *dev, enum seesaw_mod module, uint8_t reg)
{
	int ret;
	struct seesaw_data *drv_data = dev->data;
	const struct seesaw_config *const config = dev->config;
	uint8_t buf[] = {module, reg};

	k_sem_take(&drv_data->sem, K_FOREVER);
	ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
	k_sem_give(&drv_data->sem);

	return ret;
}

int seesaw_write(const struct device *dev, enum seesaw_mod module, uint8_t reg,
		 const uint8_t *buf, size_t len)
{
	int ret;
	struct seesaw_data *drv_data = dev->data;
	const struct seesaw_config *const config = dev->config;
	const uint8_t request[] = {module, reg};
	struct i2c_msg transaction[] = {
		{.buf = request, .len = sizeof(request), .flags = I2C_MSG_WRITE},
		{.buf = buf, .len = len, .flags = I2C_MSG_WRITE | I2C_MSG_STOP},
	};

	k_sem_take(&drv_data->sem, K_FOREVER);
	ret = i2c_transfer_dt(&config->i2c, transaction, ARRAY_SIZE(transaction));
	k_sem_give(&drv_data->sem);

	return ret;
}

int seesaw_read(const struct device *dev, enum seesaw_mod module, uint8_t reg,
		uint8_t *buf, size_t len)
{
	int ret;
	struct seesaw_data *drv_data = dev->data;
	const struct seesaw_config *const config = dev->config;
	const uint8_t request[] = {module, reg};

	k_sem_take(&drv_data->sem, K_FOREVER);

	ret = i2c_write_dt(&config->i2c, request, sizeof(request));
	if (ret == 0) {
		k_sleep(K_MSEC(1));  /* Wait for seesaw to prepare response */
		ret = i2c_read_dt(&config->i2c, buf, len);
	}

	k_sem_give(&drv_data->sem);
	return ret;
}

static int seesaw_init(const struct device *dev)
{
	int ret;
	uint32_t data;
	struct seesaw_data *drv_data = dev->data;

	k_sem_init(&drv_data->sem, 1, 1);

	ret = seesaw_cmd(dev, SEESAW_MOD_STATUS, STATUS_SWRST);
	if (ret) {
		LOG_ERR("[%s] Unable to software reset", dev->name);
		return ret;
	}
	k_sleep(K_MSEC(10));

	ret = seesaw_read(dev, SEESAW_MOD_STATUS, STATUS_OPTIONS,
			  (uint8_t *)&data, sizeof(data));
	if (ret) {
		LOG_ERR("[%s] Unable to query options", dev->name);
		return ret;
	}
	drv_data->modules = sys_be32_to_cpu(data);

	LOG_DBG("[%s] Available modules: %08X", dev->name, drv_data->modules);
	for (size_t i = 0; i < SEESAW_MOD_MAX_NUM; i++) {
		if (drv_data->modules & BIT(i)) {
			LOG_DBG("- %s", module_names[i]);
		}
	}

	return 0;
}

#define SEESAW_INIT(inst)							\
	static struct seesaw_data seesaw_##inst##_data;				\
	static const struct seesaw_config seesaw_##inst##_config = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),				\
	};									\
	DEVICE_DT_INST_DEFINE(inst, seesaw_init, NULL, &seesaw_##inst##_data,	\
				  &seesaw_##inst##_config, POST_KERNEL,		\
				  CONFIG_MFD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_INIT)
