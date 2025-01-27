/****************************************************************************
 * boards/arm/rp2040/pygmy/src/pygmy_spisd.c
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
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mmcsd.h>

#include "rp2040_gpio.h"
#include "rp2040_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_RP2040_SPISD_SLOT_NO
#define CONFIG_RP2040_SPISD_SLOT_NO 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pygmy_spisd_initialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *
 ****************************************************************************/

int pygmy_spisd_initialize(int minor, int bus)
{
  int ret;
  struct spi_dev_s *spi;

  /* Initialize spi device */

  spi = rp2040_spibus_initialize(bus);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize spi%d.\n", bus);
      return -ENODEV;
    }

    /* Pull up RX */

#ifdef CONFIG_RP2040_SPI1
  if (bus == 1)
    {
      rp2040_gpio_set_pulls(CONFIG_RP2040_SPI1_RX_GPIO, true, false);
    }
#else
  #error "The Pygmy requires SPI1 enabled for SPISD."
#endif

  /* TODO use littlefs & vfat split partitions */

  /* Get the SPI driver instance for the SD chip select */

  finfo("Initializing Pygmy SPI1 for the MMC/SD slot\n");

  ret = mmcsd_spislotinitialize(minor, CONFIG_RP2040_SPISD_SLOT_NO, spi);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind SPI device to MMC/SD slot %d: %d\n",
           CONFIG_RP2040_SPISD_SLOT_NO, ret);
      return ret;
    }

  /* Mount filesystem */

  ret = nx_mount("/dev/mmcsd0", "/mnt/sd0", "vfat", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the SDCARD. %d\n", ret);
    }

  return OK;
}
