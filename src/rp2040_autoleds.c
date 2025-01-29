/****************************************************************************
 * boards/arm/rp2040/pygmy/src/rp2040_autoleds.c
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

/* There are four LED status indicators located on the Pygmy flight computer.
 * The functions of these LEDs include:
 *
 *   - ARM: Green when power is supplied to the MCU/board is armed, not software
 *          controllable.
 *   - START: Green when NuttX has started successfully. Software controlled.
 *   - EJECT: Green when the SD card can be ejected (for use in applications).
 *            Software controlled.
 *   - PANIC: Red and flashing when the NuttX kernel has panicked.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "rp2040_gpio.h"

#include "rp2040_pico.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LEDs for output */

  rp2040_gpio_init(GPIO_LED_START);
  rp2040_gpio_init(GPIO_LED_PANIC);

  rp2040_gpio_setdir(GPIO_LED_START, true);
  rp2040_gpio_setdir(GPIO_LED_PANIC, true);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turn on the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:  /* Turn on the start LED */
        rp2040_gpio_put(GPIO_LED_START, true);
        break;

      case LED_ASSERTION:
      case LED_PANIC:  /* Turn on the panic LED */
        rp2040_gpio_put(GPIO_LED_PANIC, true);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn off the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_STARTED:  /* Turn off the start LED */
        rp2040_gpio_put(GPIO_LED_START, false);
        break;

      case LED_PANIC:  /* Turn off the panic LED */
        rp2040_gpio_put(GPIO_LED_PANIC, false);
        break;

      default:
        break;
    }
}
