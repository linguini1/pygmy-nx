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

#ifdef CONFIG_SENSORS_LSM6DSO32
#include "rp2040_i2c.h"
#include <nuttx/sensors/lsm6dso32.h>
#endif

#ifdef CONFIG_I2C_EE_24XX
#include "rp2040_i2c.h"
#include <nuttx/eeprom/i2c_xx24xx.h>
#endif

#ifdef CONFIG_LPWAN_RN2XX3
#include <nuttx/wireless/lpwan/rn2xx3.h>
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

#if defined(CONFIG_SENSORS_LSM6DSO32) && defined(CONFIG_SCHED_HPWORK)
static int pygmy_lsm6dso32_gy_attach(xcpt_t handler, FAR void *arg)
{
  int err;
  err = rp2040_gpio_irq_attach(GPIO_GYRO_INT, RP2040_GPIO_INTR_EDGE_HIGH,
                               handler, arg);
  if (err < 0)
    {
      return err;
    }

  rp2040_gpio_enable_irq(GPIO_GYRO_INT);
  return err;
}

static int pygmy_lsm6dso32_xl_attach(xcpt_t handler, FAR void *arg)
{
  int err;
  err = rp2040_gpio_irq_attach(GPIO_XL_INT, RP2040_GPIO_INTR_EDGE_HIGH,
                               handler, arg);
  if (err < 0)
    {
      return err;
    }

  rp2040_gpio_enable_irq(GPIO_XL_INT);
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

  /* Peripherals
   * TODO GPS
   * TODO ADC for battery charge
   */

  /* EEPROM at 0x50 (currently writeable) */

#ifdef CONFIG_I2C_EE_24XX
  ret = ee24xx_initialize(rp2040_i2cbus_initialize(1), 0x50, "eeprom",
                          EEPROM_M24C32, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Could not register EEPROM device: %d\n", ret);
    }
#endif

  /* Barometric pressure sensor at 0x77 */

#ifdef CONFIG_SENSORS_MS56XX
  /* Try to register MS56xx device at I2C0 */

  ret = ms56xx_register(rp2040_i2cbus_initialize(0), 0, MS56XX_ADDR0,
                        MS56XX_MODEL_MS5607);
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

#ifdef CONFIG_SENSORS_LSM6DSO32
  /* Try to register LSM6DSO32 IMU at 0x6b on I2C0 */

  /* Only use interrupt driven mode if HPWORK is enabled */

  struct lsm6dso32_config_s lsm6dso32_config = {
      .gy_int = LSM6DSO32_INT1,
      .xl_int = LSM6DSO32_INT2,
  };

#ifdef CONFIG_SCHED_HPWORK
  lsm6dso32_config.gy_attach = pygmy_lsm6dso32_gy_attach;
  lsm6dso32_config.xl_attach = pygmy_lsm6dso32_xl_attach;
#else
  lsm6dso32_config.gy_attach = NULL;
  lsm6dso32_config.xl_attach = NULL;
#endif /* CONFIG_SCHED_HPWORK */

  ret = lsm6dso32_register(rp2040_i2cbus_initialize(0), 0x6b, 0,
                           &lsm6dso32_config);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Couldn't register LSM6DSO32 at 0x6b: %d\n", ret);
    }
#endif

    /* SD card on SPI1 */

#ifdef CONFIG_RP2040_SPISD

#if !defined(CONFIG_RP2040_SPI1)
#error "Pygmy needs SPI1 enabled for SD card access"
#endif

  /* Mount the SPI-based MMC/SD block driver */

  ret = pygmy_spisd_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Pygmy failed to initialize SPI device to MMC/SD: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_LPWAN_RN2XX3

#if CONFIG_RP2040_UART1_BAUD != 57600
#error "CONFIG_RP2040_UART1_BAUD must be set to 57600 for RN2XX3"
#endif /* CONFIG_RP2040_UART1_BAUD != 57600 */

#if CONFIG_UART1_BAUD != 57600
#error "CONFIG_UART1_BAUD must be set to 57600 for RN2XX3"
#endif /* CONFIG_RP2040_UART1_BAUD != 57600 */

#ifndef CONFIG_STANDARD_SERIAL
#error "CONFIG_STANDARD_SERIAL must be enabled for RN2XX3"
#endif /* CONFIG_STANDARD_SERIAL */

  /* Register the RN2XX3 device driver */

  ret = rn2xx3_register("/dev/rn2903", "/dev/ttyS1");
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to register RN2XX3 device driver: %d\n", ret);
  }
#endif

  return OK;
}
