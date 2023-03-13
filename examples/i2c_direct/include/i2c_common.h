/**
 * examples/i2c_direct/include/i2c_common.h
 *
 *  Copyright (C) 2022, Ixy Design Studio, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef __EXAMPLES_I2C_DIRECT_INCLUDE_I2C_COMMON_H
#define __EXAMPLES_I2C_DIRECT_INCLUDE_I2C_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include "debug_printf.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct _i2c_ctrl_type
{
  int fd;
  int i2c_addr;
  int speed;
} i2c_ctrl_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int i2c_reg_write(i2c_ctrl_t *pi2c, uint8_t reg, uint8_t value);
int i2c_reg_write_burst(i2c_ctrl_t *pi2c,
  uint8_t reg, uint8_t *pvalue, int len);
int i2c_reg_read(i2c_ctrl_t *pi2c,
  uint8_t reg, uint8_t *value, int16_t len);

#if defined(__cplusplus)
}
#endif

#endif /* __EXAMPLES_I2C_DIRECT_INCLUDE_I2C_COMMON_H */
