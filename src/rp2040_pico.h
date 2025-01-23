/****************************************************************************
 * boards/arm/rp2040/pygmy/src/rp2040_pico.h
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

#ifndef __BOARDS_ARM_RP2040_PYGMY_SRC_PYGMY_H
#define __BOARDS_ARM_RP2040_PYGMY_SRC_PYGMY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* LEDs */

#define GPIO_LED_EJECT       22 /* The board's 'eject' LED pin */
#define GPIO_LED_START       24 /* The board's 'start' LED pin */
#define GPIO_LED_PANIC       25 /* The board's 'panic' LED pin */

/* Buzzers */

#define GPIO_BUZZER          23 /* The board's arming buzzer pin */

int rp2040_bringup(void);

#endif /* __BOARDS_ARM_RP2040_PYGMY_SRC_PYGMY_H */
