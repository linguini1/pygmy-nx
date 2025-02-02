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
#include <nuttx/fs/partition.h>
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

#define MMCSD_DEVNAME "/dev/mmcsd0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct partition_state_s
{
  int partition;
  int err;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void partition_handler(struct partition_s *part, void *arg)
{
  struct partition_state_s *state = (struct partition_state_s *)(arg);

  finfo("Pygmy partition handler called.");

  char devname[] = "/dev/mmcsd0p0";

  if (state->partition < 10 && part->index == state->partition)
    {
      finfo("Found partition %u\n", state->partition);
      devname[sizeof(devname) - 2] = state->partition + 48;
      state->err = register_blockpartition(devname, 0, "/dev/mmcsd0",
                                           part->firstblock, part->nblocks);
    }
}

/****************************************************************************
 * Name: pygmy_register_partition
 *
 * Description:
 *   Register partitions found in mmcsd0
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int pygmy_register_partition(unsigned partition)
{
  int err;
  struct partition_state_s part = {.partition = partition, .err = ENOENT};

  finfo("Pygmy trying to register partition %u\n", partition);

  err = parse_block_partition("/dev/mmcsd0", partition_handler, &part);
  if (err < 0)
    {
      return err;
    }
  return part.err;
}

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

  /* Find the two partitions */

  for (int i = 0; i < 2; i++)
    {
      ret = pygmy_register_partition(i);
      if (ret)
        {
          ferr("Couldn't register partition %d: %d\n", i, ret);
        }
    }

  /* Mount user friendly FAT filesystem */

  ret = nx_mount(MMCSD_DEVNAME "p0", "/ejectfs", "vfat", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount " MMCSD_DEVNAME "p0. %d\n", ret);
    }

  /* Mount power safe filesystem */

  ret = nx_mount(MMCSD_DEVNAME "p1", "/pwrfs", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount " MMCSD_DEVNAME "p0. %d\n", ret);
    }

  return OK;
}
