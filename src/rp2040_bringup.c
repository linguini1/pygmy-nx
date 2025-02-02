/****************************************************************************
 * boards/arm/rp2040/pygmy/src/rp2040_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stddef.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp2040_pico.h"

#if defined(CONFIG_ADC) && defined(CONFIG_RP2040_ADC)
#include "rp2040_adc.h"
#endif

#ifdef CONFIG_WATCHDOG
#include "rp2040_wdt.h"
#endif

#ifdef CONFIG_SENSORS_MS56XX
#include "rp2040_i2c.h"
#include <nuttx/sensors/ms56xx.h>
#endif

#ifdef CONFIG_SENSORS_LIS2MDL
#include "rp2040_gpio.h"
#include "rp2040_i2c.h"
#include <nuttx/sensors/lis2mdl.h>
#endif

#ifdef CONFIG_I2C_EE_24XX
#include "rp2040_i2c.h"
#include <nuttx/eeprom/i2c_xx24xx.h>
#endif

#ifdef CONFIG_LPWAN_RN2903
#include <nuttx/wireless/lpwan/rn2903.h>
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_SENSORS_LIS2MDL) && defined(CONFIG_SCHED_HPWORK)
static int pygmy_lis2mdl_attach(xcpt_t handler, FAR void *arg) {
  int err;
  err = rp2040_gpio_irq_attach(GPIO_MAG_INT, RP2040_GPIO_INTR_EDGE_HIGH,
                               handler, arg);
  if (err < 0) {
    return err;
  }

  rp2040_gpio_enable_irq(GPIO_MAG_INT);
  return err;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_bringup
 ****************************************************************************/

int rp2040_bringup(void) {

  int ret;

  /* Chip bring-up (doing it our own way, not calling rp2040_common_bringup)
   */

  /* I2C interfaces */

#ifdef CONFIG_RP2040_I2C_DRIVER
#ifdef CONFIG_RP2040_I2C0
  ret = board_i2cdev_initialize(0);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to initialize I2C0.\n");
  }
#endif

#ifdef CONFIG_RP2040_I2C1
  ret = board_i2cdev_initialize(1);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to initialize I2C1.\n");
  }
#endif
#endif

  /* SPI interfaces */

#ifdef CONFIG_RP2040_SPI_DRIVER
#ifdef CONFIG_RP2040_SPI0
  ret = board_spidev_initialize(0);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to initialize SPI0.\n");
  }
#endif

#ifdef CONFIG_RP2040_SPI1
  ret = board_spidev_initialize(1);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to initialize SPI1.\n");
  }
#endif
#endif

  /* Initialize ADC */

#if defined(CONFIG_ADC) && defined(CONFIG_RP2040_ADC)

#ifdef CONFIG_RPC2040_ADC_CHANNEL0
#define ADC_0 true
#else
#define ADC_0 false
#endif

#ifdef CONFIG_RPC2040_ADC_CHANNEL1
#define ADC_1 true
#else
#define ADC_1 false
#endif

#ifdef CONFIG_RPC2040_ADC_CHANNEL2
#define ADC_2 true
#else
#define ADC_2 false
#endif

#ifdef CONFIG_RPC2040_ADC_CHANNEL3
#define ADC_3 true
#else
#define ADC_3 false
#endif

#ifdef CONFIG_RPC2040_ADC_TEMPERATURE
#define ADC_TEMP true
#else
#define ADC_TEMP false
#endif

  ret = rp2040_adc_setup("/dev/adc0", ADC_0, ADC_1, ADC_2, ADC_3, ADC_TEMP);
  if (ret != OK) {
    syslog(LOG_ERR, "Failed to initialize ADC Driver: %d\n", ret);
  }

#endif /* defined(CONFIG_ADC) && defined(CONFIG_RP2040_ADC) */

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = rp2040_wdt_init();
  if (ret < 0) {
    syslog(LOG_ERR, "ERROR: Failed to initialize watchdog drivers: %d\n", ret);
  }
#endif

  /* Procfs file system */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0) {
    serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
  }
#endif

  // TODO copy from rp2040_common_bringup code for items that I actually
  // want

  /* Peripherals
   * TODO LSM6DSO32
   * TODO GPS
   * TODO RN2903
   * TODO ADC for battery charge
   */

  /* EEPROM at 0x50 (currently writeable) */

#ifdef CONFIG_I2C_EE_24XX
  ret = ee24xx_initialize(rp2040_i2cbus_initialize(1), 0x50, "eeprom",
                          EEPROM_M24C32, false);
  if (ret < 0) {
    syslog(LOG_ERR, "Could not register EEPROM device: %d\n", ret);
  }
#endif

  /* Barometric pressure sensor at 0x77 */

#ifdef CONFIG_SENSORS_MS56XX
  /* Try to register MS56xx device at I2C0 */

  ret = ms56xx_register(rp2040_i2cbus_initialize(0), 0, MS56XX_ADDR0,
                        MS56XX_MODEL_MS5611);
  if (ret < 0) {
    syslog(LOG_ERR, "Couldn't register MS5611 at %u: %d\n", MS56XX_ADDR0, ret);
  }
#endif

  /* Magnetometer at 0x1E */

#ifdef CONFIG_SENSORS_LIS2MDL
  /* Try to register LIS2MDL device at I2C0 */

  /* Only use interrupt driven mode if HPWORK is enabled */

#ifdef CONFIG_SCHED_HPWORK
  ret = lis2mdl_register(rp2040_i2cbus_initialize(0), 0, 0x1e,
                         &pygmy_lis2mdl_attach);
#else
  ret = lis2mdl_register(rp2040_i2cbus_initialize(0), 0, 0x1e, NULL);
#endif /* CONFIG_SCHED_HPWORK */

  if (ret < 0) {
    syslog(LOG_ERR, "Couldn't register LIS2MDL at 0x1E: %d\n", ret);
  }
#endif

  /* SD card on SPI1 */

#ifdef CONFIG_RP2040_SPISD
  /* Mount the SPI-based MMC/SD block driver */

  ret = board_spisd_initialize(0, 1);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to initialize SPI device to MMC/SD: %d\n", ret);
  }
#endif

#ifdef CONFIG_LPWAN_RN2903

#if CONFIG_RP2040_UART1_BAUD != 57600
#error "CONFIG_RP2040_UART1_BAUD must be set to 57600 for RN2903"
#endif /* CONFIG_RP2040_UART1_BAUD != 57600 */

#if CONFIG_UART1_BAUD != 57600
#error "CONFIG_UART1_BAUD must be set to 57600 for RN2903"
#endif /* CONFIG_RP2040_UART1_BAUD != 57600 */

#ifndef CONFIG_STANDARD_SERIAL
#error "CONFIG_STANDARD_SERIAL must be enabled for RN2903"
#endif /* CONFIG_STANDARD_SERIAL */

  /* Register the RN2903 device driver */

  ret = rn2903_register("/dev/rn2903", "/dev/ttyS1");
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to register RN2903 device driver: %d\n", ret);
  }
#endif

  return OK;
}
