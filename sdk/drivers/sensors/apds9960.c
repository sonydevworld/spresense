/****************************************************************************
 * drivers/sensors/apds9960.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>
#include <arch/types.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/apds9960.h>
#include <nuttx/irq.h>

#if defined(CONFIG_I2C) && defined(CONFIG_APDS9960)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APDS9960_ADDR                0x39      /* I2C Slave Address */
#define APDS9960_FREQ                400000    /* I2C data rate [Hz] */
#define APDS9960_DEVICEID            0xAB      /* Device ID */

/* APDS9960 Registers */

#define APDS9960_ENABLE              0x80      /* Enable */
#define APDS9960_CONFIG1             0x8D      /* Config1 */
#define APDS9960_CONFIG2             0x90      /* Config2 */
#define APDS9960_REG_ID              0x92      /* Device ID */
#define APDS9960_GCONFIG1            0xA2      /* Gesture Config1 */
#define APDS9960_GCONFIG2            0xA3      /* Gesture Config2 */
#define APDS9960_GCONFIG3            0xAA      /* Gesture Config3 */
#define APDS9960_GCONFIG4            0xAB      /* Gesture Config4 */
#define APDS9960_GFIFOU              0xFC      /* Gesture FIFO Data, UP */
#define APDS9960_GFIFOD              0xFD      /* Gesture FIFO Data, DOWN */
#define APDS9960_GFIFOL              0xFE      /* Gesture FIFO Data, LEFT */
#define APDS9960_GFIFOR              0xFF      /* Gesture FIFO Data, RIGHT */

/* Register ENABLE */

#define APDS9960_ENABLE_GEN          (1 << 6)  /* Gesture */
#define APDS9960_ENABLE_PIEN         (1 << 5)  /* Proximity Interrupt */
#define APDS9960_ENABLE_AIEN         (1 << 4)  /* ALS Interupt */
#define APDS9960_ENABLE_WEN          (1 << 3)  /* Wait */
#define APDS9960_ENABLE_PEN          (1 << 2)  /* Proximity Detect */
#define APDS9960_ENABLE_AEN          (1 << 1)  /* ALS Enable */
#define APDS9960_ENABLE_PON          (1 << 0)  /* Power ON */
#define APDS9960_ENABLE_STANDBY      0x00      /* Standby */

/* Configuration Register Two (0x90) */

/* LED Boost(Additional LDR current during gesture LED pulses) */

#define APDS9960_CONFIG2_LEDB_POS    (1 << 4)
#define APDS9960_CONFIG2_LEDB100     0         /* 100% */
#define APDS9960_CONFIG2_LEDB150     1         /* 150% */
#define APDS9960_CONFIG2_LEDB200     2         /* 200% */
#define APDS9960_CONFIG2_LEDB300     3         /* 300% */

/* Gesture Configuration Two Register (0xA3) */

/* Gesture Gain Control */

#define APDS9960_GCONFIG2_GGAIN_POS  (1 << 5)
#define APDS9960_GCONFIG2_GGAIN1X    0         /* 1x */
#define APDS9960_GCONFIG2_GGAIN2X    1         /* 2x */
#define APDS9960_GCONFIG2_GGAIN4X    2         /* 4x */
#define APDS9960_GCONFIG2_GGAIN8X    3         /* 8x */

/* Gesture LED Drive Strength */

#define APDS9960_GCONFIG2_GLD_POS    (1 << 3)
#define APDS9960_GCONFIG2_GLD1000    0         /* 100  mA */
#define APDS9960_GCONFIG2_GLD500     1         /*  50  mA */
#define APDS9960_GCONFIG2_GLD250     2         /*  25  mA */
#define APDS9960_GCONFIG2_GLD1250    3         /*  12.5mA */


/* Gesture Wait Time */

#define APDS9960_GCONFIG2_GWTIME_POS (1 << 0)
#define APDS9960_GCONFIG2_GWTIME00   0         /*  0.0ms */
#define APDS9960_GCONFIG2_GWTIME28   1         /*  2.8ms */
#define APDS9960_GCONFIG2_GWTIME56   2         /*  5.6ms */
#define APDS9960_GCONFIG2_GWTIME84   3         /*  8.4ms */
#define APDS9960_GCONFIG2_GWTIME140  4         /* 14.0ms */
#define APDS9960_GCONFIG2_GWTIME224  5         /* 22.4ms */
#define APDS9960_GCONFIG2_GWTIME308  6         /* 30.8ms */
#define APDS9960_GCONFIG2_GWTIME392  7         /* 39.2ms */

/* Gesture Configuration Four Register (0xAB) */

#define APDS9960_GCONFIG4_GMODE      (1 << 0)  /* Gesture Mode ON */
#define APDS9960_GCONFIG4_GIEN       (1 << 1)  /* Gesture interrupt enable */
#define APDS9960_GCONFIG4_CLR        (1 << 2)  /* Clears GFIFO */


/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
/**
 * @brief Structure for apds9930 device
 */

struct apds9960_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  int port;                     /* I2C port */
  int freq;                     /* I2C Frequency <= 400kHz */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int apds9960_open(FAR struct file *filep);
static int apds9960_close(FAR struct file *filep);
static ssize_t apds9960_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t apds9960_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int apds9960_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_apds9960fops =
{
  apds9960_open,  /* open */
  apds9960_close, /* close */
  apds9960_read,  /* read */
  apds9960_write, /* write */
  0,              /* seek */
  apds9960_ioctl, /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,              /* poll */
#endif
  0               /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9960_getreg8
 *
 * Description:
 *   Read from an 8-bit APDS9960 register
 *
 ****************************************************************************/

static uint8_t apds9960_getreg8(FAR struct apds9960_dev_s *priv,
                                uint8_t                   regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: apds9960_putreg8
 *
 * Description:
 *   Write to an 8-bit APDS9960 register
 *
 ****************************************************************************/

static int apds9960_putreg8(FAR struct apds9960_dev_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: apds9960_checkid
 *
 * Description:
 *   Read and verify the APDS9960 chip ID
 *
 ****************************************************************************/

static int apds9960_checkid(FAR struct apds9960_dev_s *priv)
{
  uint8_t id;

  /* Read Device ID */

  id = apds9960_getreg8(priv, APDS9960_REG_ID);

  if (id != APDS9960_DEVICEID)
    {
      /* Device ID is not Correct */

      snerr("Wrong Device ID! %02x (Exp:%02x)\n", id, APDS9960_DEVICEID);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9960_open
 *
 * Description:
 *   This function is called whenever the APDS9960 device is opened.
 *
 ****************************************************************************/

static int apds9960_open(FAR struct file *filep)
{
  uint8_t reg_val = 0;
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9960_dev_s *priv  = inode->i_private;

  /* Set ENABLE register
   * POWER ON, GESTURE ENABLE, PROXIMITY DETECT ENABLE
   */

  reg_val = APDS9960_ENABLE_PON | APDS9960_ENABLE_GEN | APDS9960_ENABLE_PEN;
  apds9960_putreg8(priv, APDS9960_ENABLE, reg_val);

  /* Set Configuration Register Two
   * Gesture LED Drive Strength 300%(max)
   */

  reg_val = 0;
  reg_val |= APDS9960_CONFIG2_LEDB300 * APDS9960_CONFIG2_LEDB_POS;
  apds9960_putreg8(priv, APDS9960_CONFIG2, reg_val);

  /* Set Gesture Configuration Two Register
   * Gain 8x, LED Drive 100mA, Wait Time 2.8ms
   */

  reg_val = 0;
  reg_val |= APDS9960_GCONFIG2_GGAIN8X  * APDS9960_GCONFIG2_GGAIN_POS;
  reg_val |= APDS9960_GCONFIG2_GLD1000  * APDS9960_GCONFIG2_GLD_POS;
  reg_val |= APDS9960_GCONFIG2_GWTIME28 * APDS9960_GCONFIG2_GWTIME_POS;

  apds9960_putreg8(priv, APDS9960_GCONFIG2, reg_val);

  /* Set Gesture Configuration Four Register
   * Gesture Interrupt disable, GMODE ON
   */

  reg_val = 0;
  reg_val = APDS9960_GCONFIG4_GMODE;
  apds9960_putreg8(priv, APDS9960_GCONFIG4, reg_val);

  return OK;
}

/****************************************************************************
 * Name: apds9960_close
 *
 * Description:
 *   This routine is called when the APDS9960 device is closed.
 *
 ****************************************************************************/

static int apds9960_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9960_dev_s *priv  = inode->i_private;

  apds9960_putreg8(priv, APDS9960_ENABLE, APDS9960_ENABLE_STANDBY);

  return OK;
}

/****************************************************************************
 * Name: apds9960_read
 ****************************************************************************/

static ssize_t apds9960_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9960_dev_s *priv  = inode->i_private;
  struct gesture_data_s* gesture_data = (struct gesture_data_s*)buffer;

  /* Read gesture FIFO level register */

  int work = apds9960_getreg8(priv, 0xAE);

  /* If FIFO has some data */

  if (work != 0)
    {
      gesture_data->up    = apds9960_getreg8(priv, APDS9960_GFIFOU);
      gesture_data->down  = apds9960_getreg8(priv, APDS9960_GFIFOD);
      gesture_data->left  = apds9960_getreg8(priv, APDS9960_GFIFOL);
      gesture_data->right = apds9960_getreg8(priv, APDS9960_GFIFOR);
    }
  else
    {
      snerr("No updated data in apds9960.\n");
      return 0;
    }

  return sizeof(struct gesture_data_s);
}

/****************************************************************************
 * Name: apds9960_write
 ****************************************************************************/

static ssize_t apds9960_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: apds9960_ioctl
 ****************************************************************************/

static int apds9960_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9960_init
 *
 * Description:
 *   Initialize the APDS9960 device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9960
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9960_init(FAR struct i2c_master_s *i2c, int port)
{
  FAR struct apds9960_dev_s tmp;
  FAR struct apds9960_dev_s *priv = &tmp;
  int ret;

  /* Setup temporary device structure for initialization */

  priv->i2c = i2c;
  priv->addr = APDS9960_ADDR;
  priv->port = port;
  priv->freq = APDS9960_FREQ;

  /* Check Device ID */

  ret = apds9960_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9960_register
 *
 * Description:
 *   Register the APDS9960 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9960
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9960_register(FAR const char *devpath,
                      FAR struct i2c_master_s *i2c, int port)
{
  FAR struct apds9960_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the APDS9960 device structure */

  priv =
    (FAR struct apds9960_dev_s *)kmm_malloc(sizeof(struct apds9960_dev_s));

  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = APDS9960_ADDR;
  priv->port = port;
  priv->freq = APDS9960_FREQ;

  /* Register the character driver */

  (void) snprintf(path, sizeof(path), "%s%d", devpath, 0);
  ret = register_driver(path, &g_apds9960fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("APDS9960 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_APDS9960 */

