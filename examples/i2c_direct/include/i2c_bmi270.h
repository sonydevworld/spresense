/**
 * examples/i2c_direct/include/i2c_bmi270.h
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

#ifndef __EXAMPLES_I2C_DIRECT_INCLUDE_I2C_BMI270_H
#define __EXAMPLES_I2C_DIRECT_INCLUDE_I2C_BMI270_H

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
#include "i2c_common.h"
#include "debug_printf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_DEVNAME_FOR_BMI270  "/dev/i2c0"
#define BMI270_I2C_ADDRESS  (0x69)
#define I2C_SPEED           (400*1000) /* fast mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct _axis_type
{
  int16_t x;
  int16_t y;
  int16_t z;
} axis_t;

typedef struct _i2c_bmi270_type
{
  /* i2c */

  i2c_ctrl_t i2c;

  /* fetched fifo */

  uint8_t *fifo;
  int fifo_depth;

  /* fetched data ACC/GYR */

  int acc_table_pos;
  int gyr_table_pos;
  axis_t *acc_table;
  axis_t *gyr_table;
} i2c_bmi270_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int init_bmi270(i2c_bmi270_t *pctrl);
void fini_bmi270(i2c_bmi270_t *pctrl);
int exec_dequeue_fifo(i2c_bmi270_t *pctrl);
int get_latest_acc(axis_t *pd, i2c_bmi270_t *pctrl);
int get_latest_gyr(axis_t *pd, i2c_bmi270_t *pctrl);

/* bmi270.c */

int get_bmi270_config_file_size(void);
uint8_t *get_bmi270_config_file_addr(void);

#if defined(__cplusplus)
}
#endif

#endif /* __EXAMPLES_I2C_DIRECT_INCLUDE_I2C_BMI270_H */
