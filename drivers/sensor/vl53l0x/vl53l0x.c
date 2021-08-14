/* vl53l0x.c - Driver for ST VL53L0X time of flight sensor */

#define DT_DRV_COMPAT st_vl53l0x

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/__assert.h>
#include <zephyr/types.h>
#include <device.h>
#include <logging/log.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

LOG_MODULE_REGISTER(VL53L0X, CONFIG_SENSOR_LOG_LEVEL);

/* All the values used in this driver are coming from ST datasheet and examples.
 * It can be found here:
 *   http://www.st.com/en/embedded-software/stsw-img005.html
 * There are also examples of use in the L4 cube FW:
 *   http://www.st.com/en/embedded-software/stm32cubel4.html
 */
#define VL53L0X_INITIAL_ADDR      0x29
#define VL53L0X_REG_WHO_AM_I   0xC0
#define VL53L0X_CHIP_ID        0xEEAA
#define VL53L0X_SETUP_SIGNAL_LIMIT         (0.1*65536)
#define VL53L0X_SETUP_SIGMA_LIMIT          (60*65536)
#define VL53L0X_SETUP_MAX_TIME_FOR_RANGING     33000
#define VL53L0X_SETUP_PRE_RANGE_VCSEL_PERIOD   18
#define VL53L0X_SETUP_FINAL_RANGE_VCSEL_PERIOD 14

struct vl53l0x_data {
	const struct device *i2c;
	uint16_t dt_addr;
	struct gpio_dt_spec xshut_spec;
	bool started;
	VL53L0X_Dev_t vl53l0x;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
};

static int vl53l0x_setup_single_shot(struct vl53l0x_data *drv_data)
{
	int ret;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;

	ret = VL53L0X_StaticInit(&drv_data->vl53l0x);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_StaticInit failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_PerformRefCalibration(&drv_data->vl53l0x,
					    &VhvSettings,
					    &PhaseCal);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_PerformRefCalibration failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_PerformRefSpadManagement(&drv_data->vl53l0x,
					       (uint32_t *)&refSpadCount,
					       &isApertureSpads);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_PerformRefSpadManagement failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetDeviceMode(&drv_data->vl53l0x,
				    VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetDeviceMode failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x,
					  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
					  1);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetLimitCheckEnable sigma failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				1);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetLimitCheckEnable signal rate failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				VL53L0X_SETUP_SIGNAL_LIMIT);

	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetLimitCheckValue signal rate failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x,
					 VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
					 VL53L0X_SETUP_SIGMA_LIMIT);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetLimitCheckValue sigma failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&drv_data->vl53l0x,
					    VL53L0X_SETUP_MAX_TIME_FOR_RANGING);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x,
					  VL53L0X_VCSEL_PERIOD_PRE_RANGE,
					  VL53L0X_SETUP_PRE_RANGE_VCSEL_PERIOD);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetVcselPulsePeriod pre range failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	ret = VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x,
					VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
					VL53L0X_SETUP_FINAL_RANGE_VCSEL_PERIOD);
	if (ret) {
		LOG_ERR("[%02X] VL53L0X_SetVcselPulsePeriod final range failed (%d)",
			drv_data->dt_addr, ret);
		return ret;
	}

	return ret;
}

static int vl53l0x_check_device_infos(struct vl53l0x_data *drv_data)
{
	uint16_t vl53l0x_id = 0U;
	VL53L0X_Error ret;
	VL53L0X_DeviceInfo_t vl53l0x_dev_info;
	(void)memset(&vl53l0x_dev_info, 0, sizeof(VL53L0X_DeviceInfo_t));

	ret = VL53L0X_GetDeviceInfo(&drv_data->vl53l0x, &vl53l0x_dev_info);
	if (ret < 0) {
		LOG_ERR("[%02X] Could not get info from device.",
			drv_data->dt_addr);
		return -ENODEV;
	}

	LOG_DBG("[%02X] VL53L0X_GetDeviceInfo = %d", drv_data->dt_addr, ret);
	LOG_DBG("   Device Name : %s", log_strdup(vl53l0x_dev_info.Name));
	LOG_DBG("   Device Type : %s", log_strdup(vl53l0x_dev_info.Type));
	LOG_DBG("   Device ID : %s", log_strdup(vl53l0x_dev_info.ProductId));
	LOG_DBG("   ProductRevisionMajor : %d",
		    vl53l0x_dev_info.ProductRevisionMajor);
	LOG_DBG("   ProductRevisionMinor : %d",
		    vl53l0x_dev_info.ProductRevisionMinor);

	ret = VL53L0X_RdWord(&drv_data->vl53l0x,
			     VL53L0X_REG_WHO_AM_I,
			     (uint16_t *) &vl53l0x_id);
	if ((ret < 0) || (vl53l0x_id != VL53L0X_CHIP_ID)) {
		LOG_ERR("[%02X] Issue on device identification",
			drv_data->dt_addr);
		return -ENOTSUP;
	}
	return 0;
}

static int vl53l0x_start(struct vl53l0x_data *drv_data)
{
	int r;
	LOG_DBG("[%02X] Starting", drv_data->dt_addr);

	/* Pull XSHUT high to start the sensor */
	if (drv_data->xshut_spec.port){
		r = gpio_pin_set(drv_data->xshut_spec.port, drv_data->xshut_spec.pin, 1);
		if (r < 0) {
			LOG_ERR("[%02X] Unable to shutdown sensor", drv_data->dt_addr);
			return -EIO;
		}
		k_sleep(K_MSEC(10));
	}

#ifdef CONFIG_VL53L0X_RECONFIGURE_ADDRESS
	if (drv_data->dt_addr != VL53L0X_INITIAL_ADDR){
		r = VL53L0X_SetDeviceAddress(&drv_data->vl53l0x, 2*drv_data->dt_addr);
		if (r != 0) {
			LOG_ERR("[%02X] Unable to reconfigure I2C address",
				drv_data->dt_addr);
			return -EIO;
		}

		drv_data->vl53l0x.I2cDevAddr = drv_data->dt_addr;
		LOG_DBG("[%02X] I2C address reconfigured", drv_data->dt_addr);
		k_sleep(K_MSEC(2));
	}
#endif

	r = vl53l0x_check_device_infos(drv_data);
	if (r < 0) {
		return -ENODEV;
	}

	r = VL53L0X_DataInit(&drv_data->vl53l0x);
	if (r < 0) {
		LOG_ERR("[%02X] VL53L0X_DataInit return error (%d)", drv_data->dt_addr, r);
		return -ENOTSUP;
	}

	r = vl53l0x_setup_single_shot(drv_data);
	if (r < 0) {
		LOG_ERR("[%02X] Unable to configure single shot mode", drv_data->dt_addr);
		return -ENOTSUP;
	}

	drv_data->started = true;
	LOG_DBG("[%02X] Started", drv_data->dt_addr);
	return 0;
}

static int vl53l0x_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	int r;
	struct vl53l0x_data *drv_data = dev->data;
	VL53L0X_Error ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL
			|| chan == SENSOR_CHAN_DISTANCE
			|| chan == SENSOR_CHAN_PROX);

	if (! drv_data->started) {
		r = vl53l0x_start(drv_data);
		if (r != 0) {
			return r;
		}
	}

	ret = VL53L0X_PerformSingleRangingMeasurement(&drv_data->vl53l0x,
					&drv_data->RangingMeasurementData);
	if (ret < 0) {
		LOG_ERR("[%02X] Could not perform measurment (error=%d)",
			drv_data->dt_addr, ret);
		return -EINVAL;
	}

	return 0;
}


static int vl53l0x_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	int r;
	struct vl53l0x_data *drv_data = (struct vl53l0x_data *)dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_DISTANCE
			|| chan == SENSOR_CHAN_PROX);

	if (! drv_data->started) {
		r = vl53l0x_start(drv_data);
		if (r != 0) {
			return r;
		}
	}

	if (chan == SENSOR_CHAN_PROX) {
		if (drv_data->RangingMeasurementData.RangeMilliMeter <=
		    CONFIG_VL53L0X_PROXIMITY_THRESHOLD) {
			val->val1 = 1;
		} else {
			val->val1 = 0;
		}
		val->val2 = 0;
	} else {
		val->val1 = drv_data->RangingMeasurementData.RangeMilliMeter / 1000;
		val->val2 = (drv_data->RangingMeasurementData.RangeMilliMeter % 1000) * 1000;
	}

	return 0;
}

static const struct sensor_driver_api vl53l0x_api_funcs = {
	.sample_fetch = vl53l0x_sample_fetch,
	.channel_get = vl53l0x_channel_get,
};

static int vl53l0x_init(const struct device *dev)
{
	int r;
	struct vl53l0x_data *drv_data = dev->data;

	/* Initialize the HAL peripheral with the default sensor address,
	   ie. the address on power up */
	drv_data->vl53l0x.I2cDevAddr = VL53L0X_INITIAL_ADDR;
	drv_data->vl53l0x.i2c = drv_data->i2c;

#ifdef CONFIG_VL53L0X_RECONFIGURE_ADDRESS
	if (! drv_data->xshut_spec.port){
		LOG_ERR("[%02X] Missing XSHUT gpio spec", drv_data->dt_addr);
		return -ENOTSUP;
	}
#else
	if (drv_data->dt_addr == VL53L0X_INITIAL_ADDR){
		LOG_ERR("[%02X] Invalid device address (should be 0x%X or CONFIG_VL53L0X_RECONFIGURE_ADDRESS should be enabled)", drv_data->dt_addr, VL53L0X_INITIAL_ADDR);
		return -ENOTSUP;
	}
#endif

	if (drv_data->xshut_spec.port){
		r = gpio_pin_configure(drv_data->xshut_spec.port,
				       drv_data->xshut_spec.pin,
				       GPIO_OUTPUT);
		if (r < 0){
			LOG_ERR("[%02X] Unable to configure GPIO as output", drv_data->dt_addr);
		}
	}

#ifdef CONFIG_VL53L0X_RECONFIGURE_ADDRESS
	/* Pull XSHUT low to shut down the sensor for now */
	r = gpio_pin_set(drv_data->xshut_spec.port,
			 drv_data->xshut_spec.pin,
			 0);
	if (r < 0) {
		LOG_ERR("[%02X] Unable to shutdown sensor", drv_data->dt_addr);
		return -EIO;
	}
	// k_sleep(K_MSEC(10));
	LOG_DBG("[%02X] Shutdown", drv_data->dt_addr);
#else
	r = vl53l0x_start(drv_data);
	if (r){
		return r;
	}
#endif

	LOG_DBG("[%02X] Initialized", drv_data->dt_addr);
	return 0;
}

#define VL53L0X_INIT(inst)						\
	static struct vl53l0x_data vl53l0x_##inst##_driver = {		\
		.started = false,					\
		.i2c = DEVICE_DT_GET(DT_INST_BUS(inst)),		\
		.dt_addr = DT_INST_REG_ADDR(inst),			\
		.xshut_spec =						\
			GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {}),\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, vl53l0x_init, NULL,			\
			      &vl53l0x_##inst##_driver, NULL,		\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &vl53l0x_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(VL53L0X_INIT)
