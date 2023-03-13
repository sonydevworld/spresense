/**
 * examples/i2c_direct/i2c_common.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "i2c_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief I2C Device register write
 *
 * @param fd file descriptor
 * @param reg register id
 * @param value write value [in]
 * @return int success == 0
 */

int
i2c_reg_write
(i2c_ctrl_t *pi2c, uint8_t reg, uint8_t value)
{
  int ret;
  struct i2c_msg_s i2c_msg;
  struct i2c_transfer_s i2c_transfer;
  uint8_t txbuffer[2];

  txbuffer[0] = reg;
  txbuffer[1] = value;
  i2c_msg.addr   = pi2c->i2c_addr;
  i2c_msg.flags  = 0;
  i2c_msg.buffer = txbuffer;
  i2c_msg.length = 2;
  i2c_msg.frequency = pi2c->speed;

  i2c_transfer.msgv = &i2c_msg;
  i2c_transfer.msgc = 1;
  ret = ioctl(pi2c->fd, I2CIOC_TRANSFER,
  (unsigned long)(uintptr_t)&i2c_transfer);
  if (ret < 0)
    {
      printf("I2C:Write Error(reg 0x%02x) %d\n", reg, ret);
    }

  return ret;
}

/**
 * @brief I2C Device register burst write
 *
 * @param fd file descriptor
 * @param reg register id
 * @param pvalue write value [in]
 * @param len burst length
 * @return int success == 0
 */

int
i2c_reg_write_burst
(i2c_ctrl_t *pi2c, uint8_t reg, uint8_t *pvalue, int len)
{
  int ret;
  struct i2c_msg_s i2c_msg[2];
  struct i2c_transfer_s i2c_transfer;
  uint8_t txbuffer[1];

  txbuffer[0] = reg;
  i2c_msg[0].addr   = pi2c->i2c_addr;
  i2c_msg[0].flags  = 0;
  i2c_msg[0].buffer = txbuffer;
  i2c_msg[0].length = 1;
  i2c_msg[0].frequency = pi2c->speed;

  i2c_msg[1].addr   = pi2c->i2c_addr;
  i2c_msg[1].flags  = 0;
  i2c_msg[1].buffer = pvalue;
  i2c_msg[1].length = len;
  i2c_msg[1].frequency = pi2c->speed;

  i2c_transfer.msgv = i2c_msg;
  i2c_transfer.msgc = 2;
  ret = ioctl(pi2c->fd, I2CIOC_TRANSFER,
  (unsigned long)(uintptr_t)&i2c_transfer);
  if (ret < 0)
    {
      printf("I2C:Write Error(reg 0x%02x) %d\n", reg, ret);
    }

  return ret;
}

/**
 * @brief I2C Device register read
 *
 * @param fd file descriptor
 * @param reg register id
 * @param value read value [out]
 * @param len burst length
 * @return int success == 0
 */

int
i2c_reg_read
(i2c_ctrl_t *pi2c, uint8_t reg, uint8_t * value, int16_t len)
{
  int ret;
  struct i2c_msg_s i2c_msg[2];
  struct i2c_transfer_s i2c_transfer;

  /* Write Data SEQ */

  i2c_msg[0].addr   = pi2c->i2c_addr;
  i2c_msg[0].flags  = 0;
  i2c_msg[0].buffer = &reg;
  i2c_msg[0].length = 1;
  i2c_msg[0].frequency = pi2c->speed;

  /* Read Data SEQ */

  i2c_msg[1].addr   = pi2c->i2c_addr;
  i2c_msg[1].flags  = I2C_M_READ;
  i2c_msg[1].buffer = value;
  i2c_msg[1].length = len;
  i2c_msg[1].frequency = pi2c->speed;

  i2c_transfer.msgv = i2c_msg;
  i2c_transfer.msgc = 2;
  ret = ioctl(pi2c->fd, I2CIOC_TRANSFER,
  (unsigned long)(uintptr_t)&i2c_transfer);
  if (ret < 0)
    {
      printf("I2C:Read Error(reg 0x%02x) %d\n", reg, ret);
    }

  return ret;
}
