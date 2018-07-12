/****************************************************************************
 * bsp/board/common/src/cxd56_sensors.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/board/common/cxd56_bmi160.h>
#include <arch/board/common/cxd56_kx022.h>
#include <arch/board/common/cxd56_bmp280.h>
#include <arch/board/common/cxd56_bm1383glv.h>
#include <arch/board/common/cxd56_ak09912.h>
#include <arch/board/common/cxd56_bm1422gmv.h>
#include <arch/board/common/cxd56_apds9930.h>
#include <arch/board/common/cxd56_apds9960.h>
#include <arch/board/common/cxd56_lt1pa01.h>
#include <arch/board/common/cxd56_bh1721fvc.h>
#include <arch/board/common/cxd56_rpr0521rs.h>
#include <arch/board/common/cxd56_bh1745nuc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef SENSOR_I2C
#  error "SENSOR_I2C must be defined in board.h !!"
#endif
#ifndef SENSOR_SPI
#  error "SENSOR_SPI must be defined in board.h !!"
#endif

#if defined(CONFIG_BMI160) && defined(CONFIG_KX022)
#  error "Duplicate accelerometer sensor device."
#endif

#if defined(CONFIG_BMP280) && defined(CONFIG_BM1383GLV)
#  error "Duplicate pressure sensor device."
#endif

#if defined(CONFIG_AK09912) && defined(CONFIG_BM1422GMV)
#  error "Duplicate magnetic sensor device."
#endif

/* Configuration Sanity check */

#ifdef CONFIG_APDS9930
#  define _APDS9930  1
#else
#  define _APDS9930  0
#endif

#ifdef CONFIG_LT1PA01
#  define _LT1PA01  1
#else
#  define _LT1PA01  0
#endif

#ifdef CONFIG_BH1721FVC
#  define _BH1721FVC  1
#else
#  define _BH1721FVC  0
#endif

#ifdef CONFIG_RPR0521RS
#  define _RPR0521RS  1
#else
#  define _RPR0521RS  0
#endif

#if (_APDS9930 + _LT1PA01 + _BH1721FVC + _RPR0521RS) > 1
# error "Duplicate proximity and ambient light sensor device."
#endif

/* Sensor Device Registration Macro */

#define _DEVICE_WOPATH(_name, _bus) \
  { \
    .name = #_name, \
    .devpath = NULL, \
    .bus = _bus, \
    { \
      .init = board_ ## _name ##_initialize, \
    }, \
  }

#define _DEVICE(_name, _path, _bus) \
  { \
    .name = #_name, \
    .devpath = _path, \
    .bus = _bus, \
    { \
      .initdev = board_ ## _name ##_initialize, \
    }, \
  }

#define _I2C_DEVICE(_name, _path) _DEVICE(_name, _path, SENSOR_I2C)
#define _SPI_DEVICE(_name, _path) _DEVICE(_name, _path, SENSOR_SPI)

#define _I2C_DEVICE_WOPATH(_name) _DEVICE_WOPATH(_name, SENSOR_I2C)
#define _SPI_DEVICE_WOPATH(_name) _DEVICE_WOPATH(_name, SENSOR_SPI)

/************************************************************************************
 * Private Types
 ************************************************************************************/

typedef int (*_init_t)(int bus);
typedef int (*_initdev_t)(FAR const char *devpath, int bus);

struct sensor_device_s
{
  const char    *name;    /* Sensor device name */
  const char    *devpath; /* Sensor device path */
  int           bus;      /* I2C or SPI bus number */
  union
  {
    _init_t     init;     /* Sensor initializer w/o devpath */
    _initdev_t  initdev;  /* Sensor initializer with devpath */
  } init_u;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sensor_device_s sensor_device[] =
{
#ifdef CONFIG_BMI160
#  ifdef CONFIG_BMI160_I2C
  _I2C_DEVICE_WOPATH(bmi160), /* Accel + Gyro */
#  else /* CONFIG_BMI160_SPI */
  _SPI_DEVICE_WOPATH(bmi160),
#  endif
#endif
#ifdef CONFIG_KX022
  _I2C_DEVICE(kx022, "/dev/accel"), /* Accel */
#endif
#ifdef CONFIG_BMP280
  _I2C_DEVICE_WOPATH(bmp280), /* Pressure */
#endif
#ifdef CONFIG_BM1383GLV
  _I2C_DEVICE(bm1383glv, "/dev/press"),
#endif
#ifdef CONFIG_AK09912
  _I2C_DEVICE(ak09912, "/dev/mag"), /* Magnetic */
#endif
#ifdef CONFIG_BM1422GMV
  _I2C_DEVICE(bm1422gmv, "/dev/mag"),
#endif
#ifdef CONFIG_APDS9930
  _I2C_DEVICE_WOPATH(apds9930), /* Proximity + Light */
#endif
#ifdef CONFIG_LT1PA01
  _I2C_DEVICE_WOPATH(lt1pa01),
#endif
#ifdef CONFIG_BH1721FVC
  _I2C_DEVICE(bh1721fvc, "/dev/light"),
#endif
#ifdef CONFIG_RPR0521RS
  _I2C_DEVICE_WOPATH(rpr0521rs),
#endif
#ifdef CONFIG_APDS9960
  _I2C_DEVICE(apds9960, "/dev/gesture"), /* Gesture */
#endif
#ifdef CONFIG_BH1745NUC
  _I2C_DEVICE(bh1745nuc, "/dev/color"), /* Color */
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sensors_initialize
 *
 * Description:
 *   Perform sensor devices initialization
 *
 ****************************************************************************/

int board_sensors_initialize(void)
{
  int ret = 0;
  int i;
  FAR struct sensor_device_s *dev;

  ret = board_power_control(POWER_SENSOR, true);
  if (ret)
    {
      _err("Failed to power on sensor: %d\n", ret);
      return -EPERM;
    }

  /* Wait for power-up max time */

  up_mdelay(10);

  /* Initialize each sensor device */

  for (i = 0; i < sizeof(sensor_device) / sizeof(sensor_device[0]); i++)
    {
      dev = &sensor_device[i];
      if (dev->devpath)
        {
          ret = dev->init_u.initdev(dev->devpath, dev->bus);
        }
      else
        {
          ret = dev->init_u.init(dev->bus);
        }

      if (ret < 0)
        {
          _err("Failed to init %s at bus %d: %d\n", dev->name, dev->bus, ret);
        }
    }

  return ret;
}
