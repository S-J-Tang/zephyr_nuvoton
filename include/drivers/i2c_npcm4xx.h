/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_NPCM4XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_NPCM4XX_H_


bool is_i2c_npcm_device_master_status_idle(const struct device *dev);
int i2c_npcm_device_disable(const struct device *dev);

#endif
