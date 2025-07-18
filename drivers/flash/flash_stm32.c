/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2017 BayLibre, SAS.
 * Copyright (c) 2019 Centaur Analytics, Inc
 * Copyright (c) 2023 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define DT_DRV_COMPAT st_stm32_flash_controller

#include <string.h>
#if defined(CONFIG_SOC_SERIES_STM32H5X)
#include <zephyr/cache.h>
#endif /* CONFIG_SOC_SERIES_STM32H5X */
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/stm32_flash_api_extensions.h>
#include <zephyr/init.h>
#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <zephyr/logging/log.h>

#include "flash_stm32.h"

LOG_MODULE_REGISTER(flash_stm32, CONFIG_FLASH_LOG_LEVEL);

/* Let's wait for double the max erase time to be sure that the operation is
 * completed.
 */
#define STM32_FLASH_TIMEOUT	\
	(2 * DT_PROP(DT_INST(0, st_stm32_nv_flash), max_erase_time))

static const struct flash_parameters flash_stm32_parameters = {
	.write_block_size = FLASH_STM32_WRITE_BLOCK_SIZE,
	/* Some SoCs (L0/L1) use an EEPROM under the hood. Distinguish
	 * between them based on the presence of the PECR register. */
#if defined(FLASH_PECR_ERASE)
	.erase_value = 0,
#else
	.erase_value = 0xff,
#endif
};

bool __weak flash_stm32_valid_range(const struct device *dev, off_t offset,
				    uint32_t len, bool write)
{
	if (write && !flash_stm32_valid_write(offset, len)) {
		return false;
	}
	return flash_stm32_range_exists(dev, offset, len);
}

int __weak flash_stm32_check_configuration(void)
{
	return 0;
}


#if !defined(CONFIG_SOC_SERIES_STM32WBX)
static int flash_stm32_check_status(const struct device *dev)
{

	if (FLASH_STM32_REGS(dev)->FLASH_STM32_SR & FLASH_STM32_SR_ERRORS) {
		LOG_DBG("Status: 0x%08lx",
			(unsigned long)FLASH_STM32_REGS(dev)->FLASH_STM32_SR &
							FLASH_STM32_SR_ERRORS);
		/* Clear errors to unblock usage of the flash */
		FLASH_STM32_REGS(dev)->FLASH_STM32_SR = FLASH_STM32_REGS(dev)->FLASH_STM32_SR &
							FLASH_STM32_SR_ERRORS;
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_SOC_SERIES_STM32WBX */

int flash_stm32_wait_flash_idle(const struct device *dev)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(STM32_FLASH_TIMEOUT));
	bool expired = false;
	int rc;
	uint32_t busy_flags;

	rc = flash_stm32_check_status(dev);
	if (rc < 0) {
		return -EIO;
	}

	busy_flags = FLASH_STM32_SR_BUSY;

/* Some Series can't modify FLASH_CR reg while CFGBSY is set. Wait as well */
#if defined(FLASH_STM32_SR_CFGBSY)
	busy_flags |= FLASH_STM32_SR_CFGBSY;
#endif

	while ((FLASH_STM32_REGS(dev)->FLASH_STM32_SR & busy_flags)) {
		if (expired) {
			LOG_ERR("Timeout! val: %d ms", STM32_FLASH_TIMEOUT);
			return -EIO;
		}

		/* Check if expired, but always read status register one more time.
		 * If the calling thread is pre-emptive we may have been scheduled out after reading
		 * the status register, and scheduled back after timeout has expired.
		 */
		expired = sys_timepoint_expired(timeout);
	}

	return 0;
}

static void flash_stm32_flush_caches(const struct device *dev,
				     off_t offset, size_t len)
{
#if defined(CONFIG_SOC_SERIES_STM32F0X) || defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || defined(CONFIG_SOC_SERIES_STM32L5X) || \
	defined(CONFIG_SOC_SERIES_STM32U3X) || defined(CONFIG_SOC_SERIES_STM32U5X) || \
	defined(CONFIG_SOC_SERIES_STM32H5X)
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(len);
#elif defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X)
	ARG_UNUSED(offset);
	ARG_UNUSED(len);

	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

	if (regs->ACR & FLASH_ACR_DCEN) {
		regs->ACR &= ~FLASH_ACR_DCEN;
		regs->ACR |= FLASH_ACR_DCRST;
		regs->ACR &= ~FLASH_ACR_DCRST;
		regs->ACR |= FLASH_ACR_DCEN;
	}
#elif defined(CONFIG_SOC_SERIES_STM32F7X)
	SCB_InvalidateDCache_by_Addr((uint32_t *)(FLASH_STM32_BASE_ADDRESS
						  + offset), len);
#endif
}

static int flash_stm32_read(const struct device *dev, off_t offset,
			    void *data,
			    size_t len)
{
	if (!flash_stm32_valid_range(dev, offset, len, false)) {
		LOG_ERR("Read range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("Read offset: %ld, len: %zu", (long int) offset, len);

	memcpy(data, (uint8_t *) FLASH_STM32_BASE_ADDRESS + offset, len);

	return 0;
}

static int flash_stm32_erase(const struct device *dev, off_t offset,
			     size_t len)
{
	int rc;

	if (!flash_stm32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Erase range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_stm32_sem_take(dev);

	LOG_DBG("Erase offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_stm32_cr_lock(dev, false);
	if (rc == 0) {
		rc = flash_stm32_block_erase_loop(dev, offset, len);
	}

	flash_stm32_flush_caches(dev, offset, len);

	int rc2 = flash_stm32_cr_lock(dev, true);

	if (!rc) {
		rc = rc2;
	}

	flash_stm32_sem_give(dev);

	return rc;
}

static int flash_stm32_write(const struct device *dev, off_t offset,
			     const void *data, size_t len)
{
	int rc;

	if (!flash_stm32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Write range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_stm32_sem_take(dev);

	LOG_DBG("Write offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_stm32_cr_lock(dev, false);
	if (rc == 0) {
		rc = flash_stm32_write_range(dev, offset, data, len);
	}

	int rc2 = flash_stm32_cr_lock(dev, true);

	if (!rc) {
		rc = rc2;
	}

	flash_stm32_sem_give(dev);

	return rc;
}

int flash_stm32_cr_lock(const struct device *dev, bool enable)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

	int rc = 0;

	if (enable) {
		rc = flash_stm32_wait_flash_idle(dev);
		if (rc) {
			flash_stm32_sem_give(dev);
			return rc;
		}
	}

#if defined(FLASH_SECURITY_NS)
	if (enable) {
		regs->NSCR |= FLASH_STM32_NSLOCK;
	} else {
		if (regs->NSCR & FLASH_STM32_NSLOCK) {
			regs->NSKEYR = FLASH_KEY1;
			regs->NSKEYR = FLASH_KEY2;
		}
	}
#elif defined(FLASH_CR_LOCK)
	if (enable) {
		regs->CR |= FLASH_CR_LOCK;
	} else {
		if (regs->CR & FLASH_CR_LOCK) {
			regs->KEYR = FLASH_KEY1;
			regs->KEYR = FLASH_KEY2;
		}
	}
#else
	if (enable) {
		regs->PECR |= FLASH_PECR_PRGLOCK;
		regs->PECR |= FLASH_PECR_PELOCK;
	} else {
		if (regs->PECR & FLASH_PECR_PRGLOCK) {
			LOG_DBG("Disabling write protection");
			regs->PEKEYR = FLASH_PEKEY1;
			regs->PEKEYR = FLASH_PEKEY2;
			regs->PRGKEYR = FLASH_PRGKEY1;
			regs->PRGKEYR = FLASH_PRGKEY2;
		}
		if (FLASH->PECR & FLASH_PECR_PRGLOCK) {
			LOG_ERR("Unlock failed");
			rc = -EIO;
		}
	}
#endif /* FLASH_SECURITY_NS */

	if (enable) {
		LOG_DBG("Enable write protection");
	} else {
		LOG_DBG("Disable write protection");
	}

	return rc;
}

#if defined(CONFIG_FLASH_EX_OP_ENABLED) && defined(CONFIG_FLASH_STM32_BLOCK_REGISTERS)
int flash_stm32_control_register_disable(const struct device *dev)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

#if defined(FLASH_CR_LOCK) /* F0, F1, F2, F3, F4, F7, L4, G0, G4, WB, WL */
	/*
	 * Access to control register can be disabled by writing wrong key to
	 * the key register. Option register will remain disabled until reset.
	 * Writing wrong key causes a bus fault, so we need to set FAULTMASK to
	 * disable faults, and clear bus fault pending bit before enabling them
	 * again.
	 */
	regs->CR |= FLASH_CR_LOCK;

	__set_FAULTMASK(1);
	regs->KEYR = 0xffffffff;

	/* Clear Bus Fault pending bit */
	SCB->SHCSR &= ~SCB_SHCSR_BUSFAULTPENDED_Msk;
	__set_FAULTMASK(0);

	return 0;
#else
	ARG_UNUSED(regs);

	return -ENOTSUP;
#endif
}

int flash_stm32_option_bytes_disable(const struct device *dev)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

#if defined(FLASH_OPTCR_OPTLOCK) /* F2, F4, F7 */
	/*
	 * Access to option register can be disabled by writing wrong key to
	 * the key register. Option register will remain disabled until reset.
	 * Writing wrong key causes a bus fault, so we need to set FAULTMASK to
	 * disable faults, and clear bus fault pending bit before enabling them
	 * again.
	 */
	regs->OPTCR |= FLASH_OPTCR_OPTLOCK;

	__set_FAULTMASK(1);
	regs->OPTKEYR = 0xffffffff;

	/* Clear Bus Fault pending bit */
	SCB->SHCSR &= ~SCB_SHCSR_BUSFAULTPENDED_Msk;
	__set_FAULTMASK(0);

	return 0;
#else
	ARG_UNUSED(regs);

	return -ENOTSUP;
#endif
}
#endif /* CONFIG_FLASH_STM32_BLOCK_REGISTERS */

static const struct flash_parameters *
flash_stm32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_stm32_parameters;
}

/* Gives the total logical device size in bytes and return 0. */
static int flash_stm32_get_size(const struct device *dev, uint64_t *size)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_SOC_SERIES_STM32H5X)
	/* Disable the ICACHE to ensure all memory accesses are non-cacheable.
	 * This is required on STM32H5, where the manufacturing flash must be
	 * accessed in non-cacheable mode - otherwise, a bus error occurs.
	 */
	cache_instr_disable();
#endif /* CONFIG_SOC_SERIES_STM32H5X */

	*size = (uint64_t)LL_GetFlashSize() * 1024U;

#if defined(CONFIG_SOC_SERIES_STM32H5X)
	/* Re-enable the ICACHE (unconditonally - it should always be turned on) */
	cache_instr_enable();
#endif /* CONFIG_SOC_SERIES_STM32H5X */

	return 0;
}

static struct flash_stm32_priv flash_data = {
	.regs = (FLASH_TypeDef *) DT_INST_REG_ADDR(0),
	/* Getting clocks information from device tree description depending
	 * on the presence of 'clocks' property.
	 */
#if DT_INST_NODE_HAS_PROP(0, clocks)
	.pclken = {
		.enr = DT_INST_CLOCKS_CELL(0, bits),
		.bus = DT_INST_CLOCKS_CELL(0, bus),
	}
#endif
};

static DEVICE_API(flash, flash_stm32_api) = {
	.erase = flash_stm32_erase,
	.write = flash_stm32_write,
	.read = flash_stm32_read,
	.get_parameters = flash_stm32_get_parameters,
	.get_size = flash_stm32_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_stm32_page_layout,
#endif
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_stm32_ex_op,
#endif
};

static int stm32_flash_init(const struct device *dev)
{
	int rc;
	/* Below is applicable to F0, F1, F3, G0, G4, L1, L4, L5, U5 & WB55 series.
	 * For F2, F4, F7 series, this is not applicable.
	 */
#if DT_INST_NODE_HAS_PROP(0, clocks)
	struct flash_stm32_priv *p = FLASH_STM32_PRIV(dev);
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/*
	 * On STM32 F0, F1, F3 & L1 series, flash interface clock source is
	 * always HSI, so statically enable HSI here.
	 */
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32F1X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L1X)
	LL_RCC_HSI_Enable();

	while (!LL_RCC_HSI_IsReady()) {
	}
#endif

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	if (clock_control_on(clk, (clock_control_subsys_t)&p->pclken) != 0) {
		LOG_ERR("Failed to enable clock");
		return -EIO;
	}
#endif

#ifdef CONFIG_SOC_SERIES_STM32WBX
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_HSEM);
#endif /* CONFIG_SOC_SERIES_STM32WBX */

	flash_stm32_sem_init(dev);

	LOG_DBG("Flash @0x%x initialized. BS: %zu",
		FLASH_STM32_BASE_ADDRESS,
		flash_stm32_parameters.write_block_size);

	/* Check Flash configuration */
	rc = flash_stm32_check_configuration();
	if (rc < 0) {
		return rc;
	}

#if ((CONFIG_FLASH_LOG_LEVEL >= LOG_LEVEL_DBG) && CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout *layout;
	size_t layout_size;

	flash_stm32_page_layout(dev, &layout, &layout_size);
	for (size_t i = 0; i < layout_size; i++) {
		LOG_DBG("Block %zu: bs: %zu count: %zu", i,
			layout[i].pages_size, layout[i].pages_count);
	}
#endif

	return 0;
}

DEVICE_DT_INST_DEFINE(0, stm32_flash_init, NULL,
		    &flash_data, NULL, POST_KERNEL,
		    CONFIG_FLASH_INIT_PRIORITY, &flash_stm32_api);
