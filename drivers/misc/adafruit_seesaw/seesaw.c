/*
 * Copyright (c) 2024 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/misc/seesaw.h>
#include <zephyr/drivers/i2c.h>

#define DT_DRV_COMPAT adafruit_seesaw

enum seesaw_mod_status {
	STATUS_HW_ID = 0x01,
	STATUS_VERSION = 0x02,
	STATUS_OPTIONS = 0x03,
	STATUS_TEMP = 0x04,
	STATUS_SWRST = 0x7F,
};

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(seesaw);

struct seesaw_config {
	struct i2c_dt_spec i2c;
};

struct seesaw_data {
	struct k_sem sem;
	uint32_t modules;
	uint32_t claimed_modules;
};

int seesaw_claim_module(const struct device *dev, uint32_t module)
{
	if (!device_is_ready(dev)) {
		return -EIO;
	}
	struct seesaw_data *drv_data = dev->data;
	if (module > 31) {
		return -EINVAL;
	}
	if (!(drv_data->modules & BIT(module))) {
		return -ENOTSUP;
	}
	if (drv_data->claimed_modules & BIT(module)) {
		return -EBUSY;
	}
	drv_data->claimed_modules |= BIT(module);
	return 0;
}

int seesaw_cmd(const struct device *dev, enum seesaw_mod module, uint8_t reg)
{
	const struct seesaw_config *const config = dev->config;
	struct seesaw_data *drv_data = dev->data;
	uint8_t transaction[] = {module, reg};

	k_sem_take(&drv_data->sem, K_FOREVER);
	int r = i2c_write_dt(&config->i2c, transaction, sizeof(transaction));

	k_sem_give(&drv_data->sem);
	return r;
}

int seesaw_write(const struct device *dev, enum seesaw_mod module, uint8_t reg, const uint8_t *buf,
		 size_t len)
{
	const struct seesaw_config *const config = dev->config;

	if (len > 126) {
		return -E2BIG;
	}

	uint8_t transaction[128] = {module, reg};
	memcpy(&transaction[2], buf, len);

	struct seesaw_data *drv_data = dev->data;
	k_sem_take(&drv_data->sem, K_FOREVER);

	int r = i2c_write_dt(&config->i2c, transaction, 2 + len);

	k_sem_give(&drv_data->sem);
	return r;
}

int seesaw_read(const struct device *dev, enum seesaw_mod module, uint8_t reg, uint8_t *buf,
		size_t len)
{
	const struct seesaw_config *const config = dev->config;
	const uint8_t request[] = {module, reg};

	struct seesaw_data *drv_data = dev->data;
	k_sem_take(&drv_data->sem, K_FOREVER);

	int r = i2c_write_dt(&config->i2c, request, sizeof(request));
	if (r == 0) {
		k_sleep(K_MSEC(1));
		r = i2c_read_dt(&config->i2c, buf, len);
	}

	k_sem_give(&drv_data->sem);
	return r;
}

static int seesaw_init(const struct device *dev)
{
	struct seesaw_data *drv_data = dev->data;

	k_sem_init(&drv_data->sem, 1, 1);

	int r = seesaw_cmd(dev, MOD_STATUS, STATUS_SWRST);
	if (r) {
		LOG_ERR("[%s] Unable to software reset", dev->name);
		return r;
	}
	k_sleep(K_MSEC(10));

	uint32_t data;
	r = seesaw_read(dev, MOD_STATUS, STATUS_OPTIONS, (uint8_t *)&data, sizeof(data));
	if (r) {
		LOG_ERR("[%s] Unable to query options", dev->name);
		return r;
	}
	drv_data->modules = sys_be32_to_cpu(data);

	LOG_INF("[%s] Available modules: %08X", dev->name, drv_data->modules);
	if ((drv_data->modules) & BIT(MOD_GPIO)) {
		LOG_DBG("- GPIO");
	}
	if ((drv_data->modules) & BIT(MOD_SERCOM0)) {
		LOG_DBG("- SERCOM0");
	}
	if ((drv_data->modules) & BIT(MOD_SERCOM1)) {
		LOG_DBG("- SERCOM1");
	}
	if ((drv_data->modules) & BIT(MOD_SERCOM2)) {
		LOG_DBG("- SERCOM2");
	}
	if ((drv_data->modules) & BIT(MOD_SERCOM3)) {
		LOG_DBG("- SERCOM3");
	}
	if ((drv_data->modules) & BIT(MOD_SERCOM4)) {
		LOG_DBG("- SERCOM4");
	}
	if ((drv_data->modules) & BIT(MOD_SERCOM5)) {
		LOG_DBG("- SERCOM5");
	}
	if ((drv_data->modules) & BIT(MOD_TIMER)) {
		LOG_DBG("- TIMER");
	}
	if ((drv_data->modules) & BIT(MOD_ADC)) {
		LOG_DBG("- ADC");
	}
	if ((drv_data->modules) & BIT(MOD_DAC)) {
		LOG_DBG("- DAC");
	}
	if ((drv_data->modules) & BIT(MOD_INTERRUPT)) {
		LOG_DBG("- INTERRUPT");
	}
	if ((drv_data->modules) & BIT(MOD_DAP)) {
		LOG_DBG("- DAP");
	}
	if ((drv_data->modules) & BIT(MOD_EEPROM)) {
		LOG_DBG("- EEPROM");
	}
	if ((drv_data->modules) & BIT(MOD_NEOPIXEL)) {
		LOG_DBG("- NEOPIXEL");
	}
	if ((drv_data->modules) & BIT(MOD_TOUCH)) {
		LOG_DBG("- TOUCH");
	}
	if ((drv_data->modules) & BIT(MOD_KEYPAD)) {
		LOG_DBG("- KEYPAD");
	}
	if ((drv_data->modules) & BIT(MOD_ENCODER)) {
		LOG_DBG("- ENCODER");
	}

	return 0;
}

#define SEESAW_INIT(inst)                                                                          \
	static struct seesaw_data seesaw_##inst##_data;                                            \
	static const struct seesaw_config seesaw_##inst##_config = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(0, seesaw_init, NULL, &seesaw_##inst##_data,                         \
			      &seesaw_##inst##_config, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,      \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_INIT)
