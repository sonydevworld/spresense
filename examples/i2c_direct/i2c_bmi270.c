/**
 * examples/i2c_direct/i2c_bmi270.c
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

#include "i2c_bmi270.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONV */

#define CONV(a, n) (int16_t)((((uint16_t)a[n + 1]) << 8) | ((uint16_t)a[n]))

/* Register MAP */

#define BMI270_REG_CHIPID     (0x00)
#define BMI270_REG_ERR_REG    (0x02)
#define BMI270_REG_STATUS     (0x03)
#define BMI270_REG_DATA(n)    (0x04 + n)
#define BMI270_REG_SENSORTIME0 (0x18)
#define BMI270_REG_SENSORTIME1 (0x19)
#define BMI270_REG_SENSORTIME2 (0x1a)
#define BMI270_REG_EVENT       (0x1b)
#define BMI270_REG_INT_STATUS0 (0x1c)
#define BMI270_REG_INT_STATUS1 (0x1d)
#define BMI270_REG_SC_OUT0     (0x1e)
#define BMI270_REG_SC_OUT1     (0x1f)
#define BMI270_REG_WR_GEST_ACT (0x20)
#define BMI270_REG_INTERNAL_STATUS (0x21)
#define BMI270_REG_TEMPERATURE_0 (0x22)
#define BMI270_REG_TEMPERATURE_1 (0x23)
#define BMI270_REG_FIFO_LENGTH_0 (0x24)
#define BMI270_REG_FIFO_LENGTH_1 (0x25)
#define BMI270_REG_FIFO_DATA  (0x26)
#define BMI270_REG_FEAT_PAGE  (0x2f)
#define BMI270_REG_FEATURES   (0x30)
#define BMI270_REG_ACC_CONF   (0x40)
#define BMI270_REG_ACC_RANGE  (0x41)
#define BMI270_REG_GYR_CONF   (0x42)
#define BMI270_REG_GYR_RANGE  (0x43)
#define BMI270_REG_AUX_CONF   (0x44)
#define BMI270_REG_FIFO_DOWNS (0x45)
#define BMI270_REG_FIFO_WTM_0 (0x46)
#define BMI270_REG_FIFO_WTM_1 (0x47)
#define BMI270_REG_FIFO_CONFIG_0 (0x48)
#define BMI270_REG_FIFO_CONFIG_1 (0x49)
#define BMI270_REG_SATURATION   (0x4A)
#define BMI270_REG_AUX_DEV_ID   (0x4B)
#define BMI270_REG_AUX_IF_CONF  (0x4C)
#define BMI270_REG_AUX_RD_ADDR  (0x4D)
#define BMI270_REG_AUX_WR_ADDR  (0x4E)
#define BMI270_REG_AUX_WR_DATA  (0x4F)

#define BMI270_REG_ERR_REG_MSK  (0x52)
#define BMI270_REG_INT1_IO_CTRL (0x53)
#define BMI270_REG_INT2_IO_CTRL (0x54)
#define BMI270_REG_INT_LATCH    (0x55)
#define BMI270_REG_INT1_MAP_FEAT (0x56)
#define BMI270_REG_INT2_MAP_FEAT (0x57)
#define BMI270_REG_INT_MAP_DATA  (0x58)

#define BMI270_REG_INIT_CTRL     (0x59)
#define BMI270_REG_INIT_ADDR_0   (0x5b)
#define BMI270_REG_INIT_ADDR_1   (0x5c)
#define BMI270_REG_INIT_DATA     (0x5e)
#define BMI270_REG_INTERNAL_ERROR (0x5f)

#define BMI270_REG_AUX_IF_TRIM  (0x68)
#define BMI270_REG_GYR_CRT_CONF (0x69)
#define BMI270_REG_NVM_CONF     (0x6a)
#define BMI270_REG_IF_CONF      (0x6b)
#define BMI270_REG_DRV          (0x6c)
#define BMI270_REG_ACC_SELF_TEST (0x6d)
#define BMI270_REG_GYR_SELF_TEST_AXES (0x6e)
#define BMI270_REG_NV_CONF      (0x70)
#define BMI270_REG_OFFSET(n)    (0x71 + n)
#define BMI270_REG_PWR_CONF     (0x7c)
#define BMI270_REG_PWR_CTRL     (0x7d)
#define BMI270_REG_CMD          (0x7e)

#define BMI270_STORE_TABLE_LENGTH (1024)
#define BMI270_FIFO_MAX_LENGTH  (2560)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int enable_fifo_bmi270(i2c_ctrl_t *pi2c);
static void bmi270_fifo_decoder(i2c_bmi270_t *pctrl);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief start BMI270 FIFO
 *
 * @param fd file descriptor
 * @return int success == 0
 */

static int
enable_fifo_bmi270
(i2c_ctrl_t *pi2c)
{
  int ret;
  ret = i2c_reg_write(pi2c, BMI270_REG_FIFO_CONFIG_1, 0xd0);
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write(pi2c, BMI270_REG_FIFO_CONFIG_0, 0x02);
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write(pi2c, BMI270_REG_CMD, 0xb0);
  if (ret < 0)
    {
      return -1;
    }

  return 0;
}

/**
 * @brief Decoder for BMI270 FIFO
 *
 * @param fifo dequeued data
 * @param fifo_depth dequeued length
 */

static void
bmi270_fifo_decoder
(i2c_bmi270_t *pctrl)
{
  int i = 0;
  int fifo_depth = pctrl->fifo_depth;
  uint8_t *fifo = pctrl->fifo;

  while (i < fifo_depth)
    {
      DPRINT_DEBUG("[%02x]", fifo[i]);
      if (fifo[i] & 0x40)
        {
          uint8_t type = ((fifo[i] >> 2) & 0x0f);
          i += 1;
          switch (type)
            {
              case 0:
                DPRINT_DEBUG(" - SKIP - %02x",
                fifo[i]);
                i += 1;
                break;
              case 1:
                DPRINT_DEBUG(" - TIME - %02x,%02x",
                fifo[i], fifo[i + 1]);
                i += 2;
                break;
              case 2:
                DPRINT_DEBUG(" - CFGF - %02x,%02x,%02x,%02x",
                fifo[i], fifo[i + 1], fifo[i + 2], fifo[i + 3]);
                i += 4;
                break;
            }
        }
      else if (fifo[i] & 0x80)
        {
          uint8_t enable = ((fifo[i] >> 2) & 0x0f);
          i += 1;
          if (enable & 0x02)
            {
              DPRINT_DEBUG(" - GYR - %d,%d,%d",
              CONV(fifo, i), CONV(fifo, i + 2), CONV(fifo, i + 4));
              if (pctrl->gyr_table_pos >= BMI270_STORE_TABLE_LENGTH)
                {
                  printf("store overflow\n");
                }
              else
                {
                  pctrl->gyr_table[pctrl->gyr_table_pos].x =
                  CONV(fifo, i);
                  pctrl->gyr_table[pctrl->gyr_table_pos].y =
                  CONV(fifo, i + 2);
                  pctrl->gyr_table[pctrl->gyr_table_pos].z =
                  CONV(fifo, i + 4);
                  pctrl->gyr_table_pos++;
                }

              i += 6;
            }

          if (enable & 0x01)
            {
              DPRINT_DEBUG(" - ACC - %d,%d,%d",
              CONV(fifo, i), CONV(fifo, i + 2), CONV(fifo, i + 4));
              if (pctrl->acc_table_pos >= BMI270_STORE_TABLE_LENGTH)
                {
                  printf("store overflow\n");
                }
              else
                {
                  pctrl->acc_table[pctrl->acc_table_pos].x =
                  CONV(fifo, i);
                  pctrl->acc_table[pctrl->acc_table_pos].y =
                  CONV(fifo, i + 2);
                  pctrl->acc_table[pctrl->acc_table_pos].z =
                  CONV(fifo, i + 4);
                  pctrl->acc_table_pos++;
                }

              i += 6;
            }
        }
    }

  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief finialize BMI270
 *
 * @param pctrl control structure
 */

void
fini_bmi270
(i2c_bmi270_t *pctrl)
{
  if (pctrl->acc_table != NULL)
    {
      free(pctrl->acc_table);
      pctrl->acc_table = NULL;
    }

  if (pctrl->gyr_table != NULL)
    {
      free(pctrl->gyr_table);
      pctrl->gyr_table = NULL;
    }

  if (pctrl->fifo != NULL)
    {
      free(pctrl->fifo);
      pctrl->fifo = NULL;
    }

  return;
}

/**
 * @brief initialization BMI270 and start mesurement
 *
 * @param pctrl control structure
 * @return int success == 0
 */

int
init_bmi270
(i2c_bmi270_t *pctrl)
{
  int ret;
  uint8_t chipid;
  i2c_ctrl_t *pi2c = &pctrl->i2c;
  struct timespec waittime;

  /* alloc working memory */

  pctrl->fifo = NULL;
  pctrl->acc_table = NULL;
  pctrl->gyr_table = NULL;

  pctrl->fifo = (uint8_t *)malloc(BMI270_FIFO_MAX_LENGTH);
  if (pctrl->fifo == NULL)
    {
      return -1;
    }

  /* alloc store memory for ACCEL */

  pctrl->acc_table = (axis_t *)malloc(BMI270_STORE_TABLE_LENGTH);
  if (pctrl->acc_table == NULL)
    {
      return -1;
    }

  pctrl->acc_table_pos = 0;

  /* alloc store memory for GYRO */

  pctrl->gyr_table = (axis_t *)malloc(BMI270_STORE_TABLE_LENGTH);
  if (pctrl->acc_table == NULL)
    {
      return -1;
    }

  pctrl->gyr_table_pos = 0;

  ret = i2c_reg_read(pi2c, BMI270_REG_CHIPID, &chipid, 1);
  if ((ret < 0) || (chipid != 0x24))
    {
      printf("communication:ERROR %d, chipid=0x%02x", ret, chipid);
      return -1;
    }

  DPRINT_INFO("chipid = 0x%02x", chipid);

  ret = i2c_reg_write(pi2c, BMI270_REG_CMD, 0xb6);
  if (ret < 0)
    {
      /* NOP */
    }

  /* -- WAIT 1ms -- */

  waittime.tv_sec = 0;
  waittime.tv_nsec = 1000 * 1000;
  nanosleep(&waittime, NULL);

  /** clear PREV status */

  ret = i2c_reg_write(pi2c, BMI270_REG_PWR_CONF, 0x00);
  if (ret < 0)
    {
      return -1;
    }

  /* -- WAIT 450us -- */

  waittime.tv_sec = 0;
  waittime.tv_nsec = 450 * 1000;
  nanosleep(&waittime, NULL);

  ret = i2c_reg_write(pi2c, BMI270_REG_INIT_CTRL, 0x00);
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write_burst(pi2c, BMI270_REG_INIT_DATA,
  get_bmi270_config_file_addr(),
  get_bmi270_config_file_size());
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write(pi2c, BMI270_REG_INIT_CTRL, 0x01);
  if (ret < 0)
    {
      return -1;
    }

  /* -- WAIT 20ms -- */

  waittime.tv_sec = 0;
  waittime.tv_nsec = 20 * 1000 * 1000;
  nanosleep(&waittime, NULL);

  uint8_t internal_stat;
  ret = i2c_reg_read(pi2c, BMI270_REG_INTERNAL_STATUS, &internal_stat, 1);
  if ((ret < 0) || ((internal_stat & 0x0f) != 0x01))
    {
      printf("communication:ERROR %d, "
      "internal_stat=0x%02x\n", ret, internal_stat);
      return -1;
    }

  /* -- Initialize success -- */

  DPRINT_INFO("init success(0x%02x)", internal_stat);

  /* -- Start Mesurement ACC & GYR -- */

  ret = i2c_reg_write(pi2c, BMI270_REG_PWR_CTRL, 0x0e);
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write(pi2c, BMI270_REG_ACC_CONF, 0xa8);
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write(pi2c, BMI270_REG_GYR_CONF, 0xa9);
  if (ret < 0)
    {
      return -1;
    }

  ret = i2c_reg_write(pi2c, BMI270_REG_PWR_CONF, 0x02);
  if (ret < 0)
    {
      return -1;
    }

  ret = enable_fifo_bmi270(pi2c);
  if (ret < 0)
    {
      return -1;
    }

  return 0;
}

/**
 * @brief Fetch FIFO of BMI270
 *
 * @param fd I2C descriptor
 * @return int success == 0
 */

int
exec_dequeue_fifo
(i2c_bmi270_t *pctrl)
{
  int ret;
  uint8_t fifo_len_reg[2];
  i2c_ctrl_t *pi2c = &pctrl->i2c;

  ret = i2c_reg_read(pi2c, BMI270_REG_FIFO_LENGTH_0, fifo_len_reg, 2);
  if (ret < 0)
    {
      return -1;
    }

  pctrl->fifo_depth = CONV(fifo_len_reg, 0);
  DPRINT_DEBUG("FIFO_LENGTH=%d", pctrl->fifo_depth);

  ret = i2c_reg_read(pi2c, BMI270_REG_FIFO_DATA,
  pctrl->fifo, pctrl->fifo_depth);
  if (ret < 0)
    {
      return -1;
    }

  bmi270_fifo_decoder(pctrl);
  return 0;
}

/**
 * @brief get latest accel value
 *
 * @param pd values[output]
 * @return int success == 0
 */

int
get_latest_acc
(axis_t *pd, i2c_bmi270_t *pctrl)
{
  int ret = -1;
  int apos = pctrl->acc_table_pos - 1;
  if (apos >= 0)
    {
      pd->x = pctrl->acc_table[apos].x;
      pd->y = pctrl->acc_table[apos].y;
      pd->z = pctrl->acc_table[apos].z;
      ret = 0;
    }

  pctrl->acc_table_pos = 0;
  return ret;
}

/**
 * @brief get latest gyro value
 *
 * @param pd values[output]
 * @return int success == 0
 */

int
get_latest_gyr
(axis_t *pd, i2c_bmi270_t *pctrl)
{
  int ret = -1;
  int gpos = pctrl->gyr_table_pos - 1;
  if (gpos >= 0)
    {
      pd->x = pctrl->gyr_table[gpos].x;
      pd->y = pctrl->gyr_table[gpos].y;
      pd->z = pctrl->gyr_table[gpos].z;
      ret = 0;
    }

  pctrl->gyr_table_pos = 0;
  return ret;
}
