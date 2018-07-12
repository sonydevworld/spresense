/****************************************************************************
 * drivers/video/isx012.c
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/isx012.h>
#include <arch/board/board.h>
#include <arch/chip/cisif.h>

#include "isx012_reg.h"

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The following macro is enabled because */
/* it is to make stable startup. (other case) */
/* #define ISX012_NOT_USE_NSTBY */

/* The following macro is disabled because it is to see detailed control. */
/* #define ISX012_CHECK_IN_DETAIL */

/* The following macro is disabled because the AF setting timing is delayed. */
/* #define ISX012_FIRST_SET_AF */

/* The following macro is disabled because the AF is not used. */
/* #define ISX012_AF_EN */

/* Skip invalid frame because it occurs first due to the spec of isx012. */
#define ISX012_FRAME_SKIP_EN

#define OUT_HSIZE_QVGA           (320)
#define OUT_VSIZE_QVGA           (240)
#define OUT_HSIZE_VGA            (640)
#define OUT_VSIZE_VGA            (480)
#define OUT_HSIZE_HD            (1280)
#define OUT_VSIZE_HD             (720)
#define OUT_HSIZE_QUADVGA       (1280)
#define OUT_VSIZE_QUADVGA        (960)
#define OUT_HSIZE_FULLHD        (1920)
#define OUT_VSIZE_FULLHD        (1080)
#define OUT_HSIZE_3M            (2048)
#define OUT_VSIZE_3M            (1536)
#define OUT_HSIZE_5M            (2560)
#define OUT_VSIZE_5M            (1920)

#define OUT_YUV_VSIZE_MIN         (64)
#define OUT_YUV_HSIZE_MIN         (96)
#define OUT_JPG_VSIZE_MIN         (64)
#define OUT_JPG_HSIZE_MIN         (96)
#define OUT_YUV_15FPS_VSIZE_MAX  (360)
#define OUT_YUV_15FPS_HSIZE_MAX  (480)
#define OUT_YUV_30FPS_VSIZE_MAX  (360)
#define OUT_YUV_30FPS_HSIZE_MAX  (480)
#define OUT_YUV_60FPS_VSIZE_MAX  (360)
#define OUT_YUV_60FPS_HSIZE_MAX  (480)
#define OUT_YUV_120FPS_VSIZE_MAX (240)
#define OUT_YUV_120FPS_HSIZE_MAX (320)
#define OUT_JPG_15FPS_VSIZE_MAX (1944)
#define OUT_JPG_15FPS_HSIZE_MAX (2592)
#define OUT_JPG_30FPS_VSIZE_MAX  (960)
#define OUT_JPG_30FPS_HSIZE_MAX (1280)
#define OUT_JPG_60FPS_VSIZE_MAX  (480)
#define OUT_JPG_60FPS_HSIZE_MAX  (640)
#define OUT_JPG_120FPS_VSIZE_MAX (240)
#define OUT_JPG_120FPS_HSIZE_MAX (320)

#define OUT_YUVINT_30FPS_VSIZE_MAX  (240)
#define OUT_YUVINT_30FPS_HSIZE_MAX  (400)
#define OUT_JPGINT_30FPS_VSIZE_MAX  (960)
#define OUT_JPGINT_30FPS_HSIZE_MAX (1280)
#define OUT_JPGINT_15FPS_VSIZE_MAX (1224)
#define OUT_JPGINT_15FPS_HSIZE_MAX (1632)

#define AWB_ISX012_ATM              (0x20)
#define AWB_ISX012_CLEARWEATHER     (0x04)
#define AWB_ISX012_SHADE            (0x05)
#define AWB_ISX012_CLOUDYWEATHER    (0x06)
#define AWB_ISX012_FLUORESCENTLIGHT (0x07)
#define AWB_ISX012_LIGHTBULB        (0x08)

#define SHUTTER_ISX012_MAX          (65535)
#define BRIGHTNESS_ISX012_MAX         (100)
#define CONTRAST_ISX012_MAX           (100)
#define JPEG_QUALITY_ISX012_MAX       (100)

#define AF_EXT_TIMEOUT              (500) /* ms */
#define AF_EXT_WAIT_TIME              (5) /* ms */
#define AF_EXT_DELAY_TIME             (0) /* TODO:ms */
#define VINT_TIMEOUT                (400) /* ms */
#define VINT_WAIT_TIME                (5) /* ms */
#define VINT_DELAY_TIME               (0) /* ms */
#define CAMERA_MODE_TIMEOUT         (800) /* TODO: 2vsync is 400ms.worst:5fps*/
#define CAMERA_MODE_WAIT_TIME        (10) /* ms */
#define CAMERA_MODE_DELAY_TIME        (0) /* ms */
#define DEVICE_STATE_TIMEOUT        (100) /* ms */
#define DEVICE_STATE_WAIT_TIME        (1) /* ms */
#define DEVICE_STATE_DELAY_TIME       (2) /* ms */

#define I2CFREQ_STANDARD         (100000) /* Standard mode : 100kHz */
#define I2CFREQ_FAST             (400000) /* Fast mode     : 400kHz */

/* Debug option */
#ifdef CONFIG_DEBUG_IMAGER_ERROR
#  define imagererr(format, ...)     _err(format, ##__VA_ARGS__)
#else
/* #  define imagererr(x...) */
#  define imagererr                  printf
#endif

#ifdef CONFIG_DEBUG_IMAGER_WARN
#  define imagerwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define imagerwarn(x...)
#endif

#ifdef CONFIG_DEBUG_IMAGER_INFO
#  define imagerinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#ifndef ISX012_AF_EN
#  define imagerinfo(x...)
#else
#  define imagerinfo                 printf
#endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct isx012_maxsize_s {
  uint16_t yuv_vsize_max;
  uint16_t yuv_hsize_max;
  uint16_t jpg_vsize_max;
  uint16_t jpg_hsize_max;
  uint8_t  fps_type;
  uint8_t  sensor_mode;
};

typedef struct isx012_maxsize_s isx012_maxsize_t;

struct isx012_setparam_s {
  uint8_t  fps;
  uint8_t  format;
  uint8_t  sensor_mode;
  uint16_t hsize;
  uint16_t vsize;
  uint16_t int_hsize;
  uint16_t int_vsize;
};

typedef struct isx012_setparam_s isx012_setparam_t;

#define ARRAY_NENTRIES(a) (sizeof(a)/sizeof(isx012_reg_t))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* register operations */
static uint16_t isx012_getreg(isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize);
static int     isx012_putreg(isx012_dev_t *priv,
                          uint16_t regaddr, uint16_t regval, uint16_t regsize);
static int     isx012_putreglist(isx012_dev_t *priv,
                             FAR const isx012_reg_t *reglist, size_t nentries);
#ifdef ISX012_CHECK_IN_DETAIL
static int     isx012_putregs(isx012_dev_t *priv,
                          uint16_t regaddr, uint8_t *regvals, uint8_t regsize);
static int     isx012_chipid(FAR struct i2c_master_s *i2c);
#endif

static int isx012_chk_int_state(isx012_dev_t *priv,
                                uint8_t  sts,       uint32_t delay_time,
                                uint32_t wait_time, uint32_t timeout);
#ifdef ISX012_AF_EN
static int isx012_chk_reg_bit(isx012_dev_t *priv,
                              uint16_t reg,       uint8_t mask_bit,
                              uint32_t delay_time,
                              uint32_t wait_time, uint32_t timeout);
#endif
static int isx012_chk_param(isx012_param_t *param, isx012_setparam_t *set_param);
static int isx012_set_mode_param(isx012_dev_t *priv,
                                 isx012_setparam_t set_moni_param,
                                 isx012_setparam_t set_cap_param);
static int isx012_change_camera_mode(isx012_dev_t *priv, isx012_mode_t mode);
static int isx012_change_device_state(isx012_dev_t *priv, isx012_state_t state);
static int isx012_write_reg(isx012_dev_t *priv, isx012_reg_t *reg);
static int isx012_read_reg(isx012_dev_t *priv, isx012_reg_t *reg);
static int isx012_change_mode_param(isx012_dev_t *priv, FAR isx012_t *imager);
static int isx012_change_color(isx012_dev_t *priv, uint8_t val);
static int isx012_change_iso(isx012_dev_t *priv, uint8_t val);
static int isx012_change_shutter(isx012_dev_t *priv, uint16_t val);
static int isx012_change_ev_correction(isx012_dev_t *priv, uint8_t val);
static int isx012_change_brightness(isx012_dev_t *priv, uint8_t val);
static int isx012_change_contrast(isx012_dev_t *priv, uint8_t val);
static int isx012_change_jpeg_quality(isx012_dev_t *priv, uint8_t val);
static int isx012_change_ygamma(isx012_dev_t *priv, uint8_t val);
static int isx012_change_awb(isx012_dev_t *priv, uint8_t val);
static int isx012_change_photometry(isx012_dev_t *priv, uint8_t val);
static int isx012_get_iso(isx012_dev_t *priv, uint8_t *val);
static int isx012_get_shutterl(isx012_dev_t *priv, uint16_t *val);
static int isx012_get_shutterh(isx012_dev_t *priv, uint16_t *val);
static int isx012_check_resolution(int16_t hsize, int16_t vsize);
static int isx012_change_crop(isx012_dev_t *priv, isx012_param_crop_t *param);
static int isx012_set_moni_refresh(isx012_dev_t *priv, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static isx012_dev_t   g_private;
static isx012_state_t g_state;
static isx012_mode_t  g_mode;
static uint32_t       g_i2c_freq = I2CFREQ_STANDARD;
static int            g_af_loadsts = 0;

static isx012_maxsize_t g_rate_to_maxsize[RATE_ISX012_MAX] = {
 {OUT_YUV_120FPS_VSIZE_MAX, OUT_YUV_120FPS_HSIZE_MAX, OUT_JPG_120FPS_VSIZE_MAX,
  OUT_JPG_120FPS_HSIZE_MAX, REGVAL_FPSTYPE_120FPS, REGVAL_SENSMODE_1_8},/*120*/

 {OUT_YUV_60FPS_VSIZE_MAX,  OUT_YUV_60FPS_HSIZE_MAX,  OUT_JPG_60FPS_VSIZE_MAX,
  OUT_JPG_60FPS_HSIZE_MAX, REGVAL_FPSTYPE_60FPS, REGVAL_SENSMODE_1_4},   /*60*/

 {OUT_YUV_30FPS_VSIZE_MAX,  OUT_YUV_30FPS_HSIZE_MAX,  OUT_JPG_30FPS_VSIZE_MAX,
  OUT_JPG_30FPS_HSIZE_MAX, REGVAL_FPSTYPE_30FPS, REGVAL_SENSMODE_1_2},   /*30*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_15FPS, REGVAL_SENSMODE_ALLPIX},/*15*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_10FPS, REGVAL_SENSMODE_ALLPIX},/*10*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_7_5FPS, REGVAL_SENSMODE_ALLPIX},/*7*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_6FPS, REGVAL_SENSMODE_ALLPIX}, /* 6*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_5FPS, REGVAL_SENSMODE_ALLPIX}, /* 5*/
};

static isx012_maxsize_t g_rate_to_max_interleave[RATE_ISX012_MAX] = {
 {                         0,                          0,
                           0,                          0,
                           0,                   0},    /* Rate120Fps*/
 {                         0,                          0,
                           0,                          0,
                           0,                   0},    /* Rate60Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_30FPS_VSIZE_MAX, OUT_JPGINT_30FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_30FPS,       REGVAL_SENSMODE_1_2},    /* Rate30Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_15FPS,       REGVAL_SENSMODE_ALLPIX}, /* Rate15Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_10FPS,       REGVAL_SENSMODE_ALLPIX}, /* Rate10Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_7_5FPS,      REGVAL_SENSMODE_ALLPIX},/* Rate7_5Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_6FPS,        REGVAL_SENSMODE_ALLPIX},  /* Rate6Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_5FPS,        REGVAL_SENSMODE_ALLPIX},  /* Rate5Fps */
};

#ifndef ISX012_NOT_USE_NSTBY
static const isx012_reg_t g_isx012_presleep[] = {
  {PLL_CKSEL, 0x00, 0x01}, /* PLL_CKSEL */
  {SRCCK_DIV, 0x00, 0x01}, /* SRCCK_DIV */
  {INCK_SET,  0x17, 0x01}, /* INCK_SET */
};
#define ISX012_PRESLEEP_NENTRIES ARRAY_NENTRIES(g_isx012_presleep)
#endif

static const isx012_reg_t g_isx012_def_init[] = {
#ifdef ISX012_NOT_USE_NSTBY
  {PLL_CKSEL,         0x00, 0x01},
  {SRCCK_DIV,         0x00, 0x01},
#endif
  {DRIVABILITY,       0xAA, 0x01},
  {VIFCONFIG,       0x0200, 0x02},
  {YUVCONFIG_TN,    0xFF0A, 0x02},
  {ILCODELEN,         0x00, 0x01},
  {AFMODE_MONI,       0x01, 0x01},
  {YUVCONFIG,       0xFF6A, 0x02},
  {VIF_REC601_Y,    0x10FE, 0x02},
  {VIF_REC601_C,    0x10F0, 0x02},
  {HSENS_MODE_SEL,    0x11, 0x01},
  {VIF_CLKCONFIG1,    0x30, 0x01},
  {VIF_CLKCONFIG2,    0x30, 0x01},
  {VIF_CLKCONFIG3,    0x30, 0x01},
  {VIF_CLKCONFIG4,    0x30, 0x01},
  {VIF_CLKCONFIG5,    0x30, 0x01},
  {VIF_CLKCONFIG6,    0x30, 0x01},
  {VIF_CLKCONFIG7,    0x30, 0x01},
  {VIF_CLKCONFIG8,    0x30, 0x01},
  {VIF_CLKCONFIG9,    0x30, 0x01},
  {VIF_CLKCONFIG10,   0x30, 0x01},
  {VIF_CLKCONFIG11,   0x30, 0x01},
  {VIF_CLKCONFIG12,   0x30, 0x01},
  {VIF_CLKCONFIG13,   0x11, 0x01},
  {VIF_CLKCONFIG14,   0x11, 0x01},
  {VIF_CLKCONFIG15,   0x11, 0x01},
  {VIF_CLKCONFIG16,   0x11, 0x01},
#ifdef ISX012_NOT_USE_NSTBY
  {INCK_SET,        0x17,   0x01}, /* INCK_SET */
#endif
  {FAST_MODECHG_EN,   0x01, 0x01},
  {FAST_SHT_MODE_SEL, 0x01, 0x01},
  {CAP_HALF_AE_CTRL,  0x07, 0x01}, /* HAFREL=HIGHSPEED, CAP=Auto  */
  {HALF_AWB_CTRL,     0x01, 0x01},
  {AESPEED_FAST,      0x0F, 0x01},
  {FASTMOVE_TIMEOUT,  0x2D, 0x01},
  {YGAMMA_MODE,       0x01, 0x01},
};
#define ISX012_RESET_NENTRIES ARRAY_NENTRIES(g_isx012_def_init)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint16_t isx012_getreg(isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint16_t regval;
  volatile uint8_t buffer[2];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = g_i2c_freq; /* FREQUENCY */
  config.address   = ISX012_I2C_SLV_ADDR;
  config.addrlen   = 7;
  buffer[0] = regaddr >> 8;
  buffer[1] = regaddr & 0xff;

  /* Write the register address */
  ret = i2c_write(priv->i2c, &config, (uint8_t *)buffer, 2);
  if (ret < 0)
    {
      imagererr("i2c_write failed: %d\n", ret);
      return 0;
    }

  /* Restart and read 16bits from the register */
  ret = i2c_read(priv->i2c, &config, (uint8_t *)buffer, regsize);
  if (ret < 0)
    {
      imagererr("i2c_read failed: %d\n", ret);
      return 0;
    }

  memcpy((uint8_t *)&regval, (uint8_t *)buffer, regsize);

  return regval;
}

static int isx012_putreg(isx012_dev_t *priv,
                         uint16_t regaddr, uint16_t regval, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint8_t buffer[4];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = g_i2c_freq; /* FREQUENCY */
  config.address   = ISX012_I2C_SLV_ADDR;
  config.addrlen   = 7;

  /* Set up for the transfer */
  buffer[0] = regaddr >> 8;   /* RegAddr Hi */
  buffer[1] = regaddr & 0xff; /* RegAddr Low*/

  memcpy((uint8_t *)&buffer[2], (uint8_t *)&regval, regsize);

  /* And do it */
  ret = i2c_write(priv->i2c, &config, (uint8_t *)buffer, regsize+2);
  if (ret < 0)
    {
      imagererr("i2c_write failed: %d\n", ret);
    }

  return ret;
}

static int isx012_putreglist(isx012_dev_t *priv,
                             FAR const isx012_reg_t *reglist, size_t nentries)
{
  FAR const isx012_reg_t *entry;
  int ret = OK;

  for (entry = reglist; nentries > 0; nentries--, entry++)
    {
      ret = isx012_putreg(priv, entry->regaddr, entry->regval, entry->regsize);
      if (ret < 0)
        {
          imagererr("isx012_putreg failed: %d\n", ret);
          return ret;
        }

    }
  return ret;
}

#ifdef ISX012_CHECK_IN_DETAIL
static int isx012_putregs(isx012_dev_t *priv,
                          uint16_t regaddr, uint8_t *regvals, uint8_t regsize)
{
  struct i2c_config_s config;
  volatile uint8_t buffer[16];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = g_i2c_freq; /* FREQUENCY */
  config.address   = ISX012_I2C_SLV_ADDR;
  config.addrlen   = 7;

  /* Set up for the transfer */
  buffer[0] = regaddr >> 8;   /* RegAddr Hi */
  buffer[1] = regaddr & 0xff; /* RegAddr Low*/

  memcpy((uint8_t *)&buffer[2], (uint8_t *)regvals, regsize);
  ret = i2c_write(priv->i2c, &config, (uint8_t *)buffer, regsize+2);
  if (ret < 0)
    {
      imagererr("i2c_write failed: %d\n", ret);
    }

  return ret;
}

static int isx012_chipid(isx012_dev_t *priv)
{
  uint8_t inck;

  inck = isx012_getreg(priv, 0x06, 0x01);
  if (inck != 0x06 && inck != 0x17)
    {
      imagererr("Unsupported sensor INCK=%02x\n",inck);
      return -ENODEV;
    }

  imagerinfo("INCK=%02x\n", inck);
  return OK;
}
#endif

static int isx012_chk_int_state(isx012_dev_t *priv,
                                uint8_t sts, uint32_t delay_time,
                                uint32_t wait_time, uint32_t timeout)
{
  int ret = 0;
  volatile uint8_t data;
  uint32_t time = 0;

  usleep(delay_time*1000);
  while(time < timeout)
    {
      data = isx012_getreg(priv, INTSTS0, sizeof(data));
      data = data & sts;
      if (data != 0)
        {
          ret = isx012_putreg(priv, INTCLR0, data, sizeof(data));
          return ret;
        }

      usleep(wait_time*1000);
      time += wait_time;
    }
  return ERROR;
}

#ifdef ISX012_AF_EN
static int isx012_chk_reg_bit(isx012_dev_t *priv,
                              uint16_t reg, uint8_t mask_bit,
                              uint32_t delay_time, uint32_t wait_time,
                              uint32_t timeout)
{
  volatile uint8_t data;
  uint32_t time = 0;

  usleep(delay_time*1000);
  while(time < timeout)
    {
      data = isx012_getreg(priv, reg, sizeof(data));
      imagerinfo("ISX012: AF_EXT_AFRAMDRVFIN: %x\n", data);
      data = data & mask_bit;
      if (data != 0)
        {
          return OK;
        }

      usleep(wait_time*1000);
      time += wait_time;
    }
  return -EPERM;
}
#endif /* ISX012_AF_EN */

static int isx012_chk_param(isx012_param_t *param,
                            isx012_setparam_t *set_param)
{
  if (param->rate >= RATE_ISX012_MAX) {
    return -EPERM;
  }
  set_param->int_hsize = 0;
  set_param->int_vsize = 0;

  switch (param->format)
    {
      case FORMAT_ISX012_YUV:
      case FORMAT_ISX012_RGB565:
        if (param->yuv_hsize < OUT_YUV_HSIZE_MIN ||
            param->yuv_hsize > g_rate_to_maxsize[param->rate].yuv_hsize_max ||
            param->yuv_vsize < OUT_YUV_VSIZE_MIN ||
            param->yuv_vsize > g_rate_to_maxsize[param->rate].yuv_vsize_max)
          {
            return -EPERM;
          }

        if (param->yuv_hsize % 2 != 0 || param->yuv_vsize % 2 != 0)
          {
            return -EPERM;
          }

        if (param->format == FORMAT_ISX012_YUV)
          {
            set_param->format = REGVAL_OUTFMT_YUV;
          }
        else
          {
            set_param->format = REGVAL_OUTFMT_RGB;
          }
 
        set_param->hsize = param->yuv_hsize;
        set_param->vsize = param->yuv_vsize;
        set_param->fps = g_rate_to_maxsize[param->rate].fps_type;
        set_param->sensor_mode = g_rate_to_maxsize[param->rate].sensor_mode;
        break;
      case FORMAT_ISX012_JPEG_MODE1:
        if (param->jpeg_hsize < OUT_JPG_HSIZE_MIN ||
            param->jpeg_hsize > g_rate_to_maxsize[param->rate].jpg_hsize_max ||
            param->jpeg_vsize < OUT_JPG_VSIZE_MIN ||
            param->jpeg_vsize > g_rate_to_maxsize[param->rate].jpg_vsize_max)
          {
            return -EPERM;
          }
 
        if (param->jpeg_hsize % 2 != 0 || param->jpeg_vsize % 2 != 0)
          {
            return -EPERM;
          }
 
        set_param->format = REGVAL_OUTFMT_JPEG;
        set_param->hsize = param->jpeg_hsize;
        set_param->vsize = param->jpeg_vsize;
        set_param->fps = g_rate_to_maxsize[param->rate].fps_type;
        set_param->sensor_mode = g_rate_to_maxsize[param->rate].sensor_mode;
        break;
      case FORMAT_ISX012_JPEG_MODE1_INT:
        if (param->jpeg_hsize < OUT_JPG_HSIZE_MIN ||
            param->jpeg_hsize > g_rate_to_max_interleave[param->rate].jpg_hsize_max ||
            param->jpeg_vsize < OUT_JPG_VSIZE_MIN ||
            param->jpeg_vsize > g_rate_to_max_interleave[param->rate].jpg_vsize_max ||
            param->yuv_hsize < OUT_YUV_HSIZE_MIN ||
            param->yuv_hsize > g_rate_to_max_interleave[param->rate].yuv_hsize_max ||
            param->yuv_vsize < OUT_YUV_VSIZE_MIN ||
            param->yuv_vsize > g_rate_to_max_interleave[param->rate].yuv_vsize_max)
          {
            return -EPERM;
          }

        if (param->jpeg_hsize % 2 != 0 || param->jpeg_vsize % 2 != 0 ||
            param->yuv_hsize % 2 != 0 || param->yuv_vsize % 2 != 0)
          {
            return -EPERM;
          }

        set_param->format = REGVAL_OUTFMT_INTERLEAVE;
        set_param->hsize = param->jpeg_hsize;
        set_param->vsize = param->jpeg_vsize;
        set_param->int_hsize = param->yuv_hsize;
        set_param->int_vsize = param->yuv_vsize;
        set_param->fps = g_rate_to_max_interleave[param->rate].fps_type;
        set_param->sensor_mode = g_rate_to_max_interleave[param->rate].sensor_mode;
        break;
      default:
        return -EPERM;
    }
  return OK;
}

static int isx012_set_mode_param(isx012_dev_t *priv,
                                 isx012_setparam_t set_moni_param,
                                 isx012_setparam_t set_cap_param)
{
  int ret = 0;
  isx012_setparam_t *mparam = &set_moni_param;
  isx012_setparam_t *cparam = &set_cap_param;

  ret = isx012_putreg(priv, FPSTYPE_MONI,  mparam->fps, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, OUTFMT_MONI,   mparam->format, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, SENSMODE_MONI, mparam->sensor_mode, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, HSIZE_MONI,    mparam->hsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, VSIZE_MONI,    mparam->vsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, FPSTYPE_CAP,   cparam->fps, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, OUTFMT_CAP,    cparam->format, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, SENSMODE_CAP,  cparam->sensor_mode, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, HSIZE_CAP,     cparam->hsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, VSIZE_CAP,     cparam->vsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  if (mparam->format == REGVAL_OUTFMT_INTERLEAVE)
    {
      ret = isx012_putreg(priv, HSIZE_TN, mparam->int_hsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

      ret = isx012_putreg(priv, VSIZE_TN, mparam->int_vsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

    }

  if (cparam->format == REGVAL_OUTFMT_INTERLEAVE)
    {
      ret = isx012_putreg(priv, HSIZE_TN, cparam->int_hsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

      ret = isx012_putreg(priv, VSIZE_TN, cparam->int_vsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

    }

  return OK;
}

/******************************************************************************
 * isx012_change_cisif
 *****************************************************************************/
static int isx012_change_cisif(isx012_dev_t *priv, cisif_param_t *param)
{
  int ret = 0;
  isx012_format_t format;
  uint16_t        hsize;
  uint16_t        vsize;
  cisif_sarea_t cis_area;
  cisif_param_t cis_param;

  memset(&cis_param, 0, sizeof(cisif_param_t));
  cis_area.strg_addr = (uint8_t *)param->sarea.strg_addr;
  cis_area.strg_size = param->sarea.strg_size;
  cis_area.capnum    = param->sarea.capnum;
  cis_area.interval  = param->sarea.interval;

  if (g_mode == MODE_ISX012_MONITORING)
    {
      format = priv->image.moni_param.format;
      if (format == FORMAT_ISX012_YUV)
        {
          hsize  = priv->image.moni_param.yuv_hsize;
          vsize  = priv->image.moni_param.yuv_vsize;

          cis_param.format = FORMAT_CISIF_YUV;
          cis_param.yuv_param.hsize = hsize;
          cis_param.yuv_param.vsize = vsize;
          cis_param.yuv_param.comp_func = param->yuv_param.comp_func;
        }
      else if (format == FORMAT_ISX012_JPEG_MODE1)
        {
          hsize  = priv->image.moni_param.jpeg_hsize;
          vsize  = priv->image.moni_param.jpeg_vsize;

          cis_param.format = FORMAT_CISIF_JPEG;
          cis_param.jpg_param.comp_func = param->jpg_param.comp_func;
        }
    }
  else if (g_mode == MODE_ISX012_CAPTURE)
    {
      format = priv->image.cap_param.format;
      if (format == FORMAT_ISX012_YUV)
        {
          hsize  = priv->image.cap_param.yuv_hsize;
          vsize  = priv->image.cap_param.yuv_vsize;

          cis_param.format = FORMAT_CISIF_YUV;
          cis_param.yuv_param.hsize = hsize;
          cis_param.yuv_param.vsize = vsize;
          cis_param.yuv_param.comp_func = param->yuv_param.comp_func;
        }
      else if (format == FORMAT_ISX012_JPEG_MODE1)
        {
          hsize  = priv->image.cap_param.jpeg_hsize;
          vsize  = priv->image.cap_param.jpeg_vsize;

          cis_param.format = FORMAT_CISIF_JPEG;
          cis_param.jpg_param.comp_func = param->jpg_param.comp_func;
        }
    }
  else
    {
      format = FORMAT_ISX012_JPEG_MODE1;
    }

  if (param->sarea.capnum > 1)
    {
      ret = cxd56_cisifcontinuouscapture(&cis_param, &cis_area);
    }
  else if (format == FORMAT_ISX012_YUV)
    {
      ret = cxd56_cisifcaptureframe(&cis_param, &cis_area, NULL);
    }
  else
    {
      ret = cxd56_cisifcaptureframe(&cis_param, NULL, &cis_area);
    }

  return ret;
}

/******************************************************************************
 * isx012_change_camera_mode
 *****************************************************************************/
static int isx012_change_camera_mode(isx012_dev_t *priv, isx012_mode_t mode)
{
  int ret = 0;
  uint8_t mode_data;
  uint8_t format_data;
  uint32_t vifmode;
#ifdef ISX012_FRAME_SKIP_EN
  uint8_t mask_num;
  int i;
#endif /* ISX012_FRAME_SKIP_EN */

  if (g_state != STATE_ISX012_ACTIVE)
    {
      return -EPERM;
    }

  switch (mode)
    {
      case MODE_ISX012_MONITORING:
        if (g_mode == MODE_ISX012_MONITORING)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_MONI, 1);
        mode_data = REGVAL_MODESEL_MON;
        break;
      case MODE_ISX012_CAPTURE:
        if (g_mode == MODE_ISX012_CAPTURE)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_CAP, 1);
        mode_data = REGVAL_MODESEL_CAP;
        break;
      case MODE_ISX012_HALFRELEASE:
        if (g_mode != MODE_ISX012_MONITORING)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_MONI, 1);
        mode_data = REGVAL_MODESEL_HREL;
        break;
      default:
        return -EPERM;
    }

  switch (format_data) /* mode parallel */
    {
      case REGVAL_OUTFMT_YUV:
        vifmode = REGVAL_VIFMODE_YUV_PARALLEL;
        break;
      case REGVAL_OUTFMT_JPEG:
        vifmode = REGVAL_VIFMODE_JPEG_PARALLEL;
        break;
      case REGVAL_OUTFMT_INTERLEAVE:
        vifmode = REGVAL_VIFMODE_INTERLEAVE_PARALLEL;
        break;
      case REGVAL_OUTFMT_RGB:
        vifmode = REGVAL_VIFMODE_RGB_PARALLEL;
        break;
      default:
        vifmode = REGVAL_VIFMODE_YUV_PARALLEL;
        break;
    }

  ret = isx012_putreg(priv, VIFMODE, vifmode, sizeof(vifmode));
  if (ret < 0)
    {
      return ret;
    }

  isx012_putreg(priv, INTCLR0, CM_CHANGED_STS, 1);

  ret = isx012_putreg(priv, MODESEL, mode_data, sizeof(mode_data));
  if (ret < 0)
    {
      return ret;
    }

  /* Wait CM_CHANGED */
  ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                   CAMERA_MODE_DELAY_TIME,
                                   CAMERA_MODE_WAIT_TIME,
                                   CAMERA_MODE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  g_mode = mode;

#ifdef ISX012_FRAME_SKIP_EN
  if (mode != MODE_ISX012_HALFRELEASE)
    {
      isx012_putreg(priv, INTCLR0, VINT_STS, 1);
      mask_num = isx012_getreg(priv, RO_MASK_NUM, sizeof(mask_num));
      for (i=0; i<mask_num; i++)
        {
          /* Wait Next VINT */
          ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                           VINT_WAIT_TIME, VINT_TIMEOUT);
          if (ret != 0)
            {
              return ret;
            }
        }
    }
#endif /* ISX012_FRAME_SKIP_EN */

  return OK;
}

/******************************************************************************
 * isx012_change_device_state
 *****************************************************************************/
static int isx012_change_device_state(isx012_dev_t *priv, isx012_state_t state)
{
  int ret = 0;
#ifdef ISX012_FRAME_SKIP_EN
  int i;
  uint8_t mute_cnt;
#endif /* ISX012_FRAME_SKIP_EN */

  if (g_state == STATE_ISX012_PRESLEEP || g_state == state)
    {
      return -EPERM;
    }

  switch (state)
    {
      case STATE_ISX012_SLEEP:
        isx012_putreg(priv, INTCLR0, OM_CHANGED_STS, 1);
        board_isx012_set_sleep(1);
        break;
      case STATE_ISX012_ACTIVE:
        isx012_putreg(priv, INTCLR0, OM_CHANGED_STS | CM_CHANGED_STS, 1);
        board_isx012_release_sleep();
        break;
      case STATE_ISX012_PRESLEEP:
        return -EPERM;
      default:
        return -EPERM;
    }

  /* Wait OM_CHANGED */
  ret = isx012_chk_int_state(priv, OM_CHANGED_STS,
                                   DEVICE_STATE_DELAY_TIME,
                                   DEVICE_STATE_WAIT_TIME,
                                   DEVICE_STATE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  g_state = state;

  if (state == STATE_ISX012_ACTIVE)
    {
      /* Wait CM_CHANGED -> Monitoring */
      ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                       CAMERA_MODE_DELAY_TIME,
                                       CAMERA_MODE_WAIT_TIME,
                                       CAMERA_MODE_TIMEOUT);
      if (ret != 0)
        {
          return ret;
        }

#ifdef ISX012_FRAME_SKIP_EN
      mute_cnt = isx012_getreg(priv, MUTECNT, sizeof(mute_cnt));
      isx012_putreg(priv, INTCLR0, VINT_STS, 1);
      for (i=0; i<mute_cnt; i++)
        {
          /* Wait Next VINT */
          ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                           VINT_WAIT_TIME, VINT_TIMEOUT);
          if (ret != 0)
            {
              return ret;
            }
        }
#endif  /* ISX012_FRAME_SKIP_EN */

#ifdef ISX012_AF_EN
#ifndef ISX012_FIRST_SET_AF
      if (!g_af_loadsts)
        {
          imagerinfo("AF Driver load.\n");
          /* AF default driver load */
          ret = isx012_putreg(priv, AF_EXT, 1, 1);
          if (ret < 0)
            {
              board_isx012_set_reset();
              return ret;
            }

          ret = isx012_chk_reg_bit(priv, AF_EXT,
                                         AF_EXT_AFRAMDRVFIN,
                                         AF_EXT_DELAY_TIME,
                                         AF_EXT_WAIT_TIME,
                                         AF_EXT_TIMEOUT);
          if (ret < 0 || ret > 0)
            {
              board_isx012_set_reset();
              return ret;
            }

          g_af_loadsts = 1;
        }

      ret = isx012_putreg(priv, AF_RESTART_F, 1, 1);
      if (ret < 0)
        {
          return ret;
        }
#endif /* ISX012_FIRST_SET_AF */
#endif /* ISX012_AF_EN */
    }

  g_mode = MODE_ISX012_MONITORING;

  return OK;
}

/******************************************************************************
 * isx012_change_mode_param
 *****************************************************************************/
static int isx012_change_mode_param(isx012_dev_t *priv, FAR isx012_t *imager)
{
  int ret;
  isx012_setparam_t set_moni_param;
  isx012_setparam_t set_cap_param;

  ret = isx012_chk_param(&imager->moni_param, &set_moni_param);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_chk_param(&imager->cap_param, &set_cap_param);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_set_mode_param(priv, set_moni_param, set_cap_param);
  if (ret < 0)
    {
      return ret;
    }

  priv->image.moni_param.format     = imager->moni_param.format;
  priv->image.moni_param.rate       = imager->moni_param.rate;
  priv->image.cap_param.format      = imager->cap_param.format;
  priv->image.cap_param.rate        = imager->cap_param.rate;
  priv->image.moni_param.yuv_hsize  = imager->moni_param.yuv_hsize;
  priv->image.moni_param.yuv_vsize  = imager->moni_param.yuv_vsize;
  priv->image.moni_param.jpeg_hsize = imager->moni_param.jpeg_hsize;
  priv->image.moni_param.jpeg_vsize = imager->moni_param.jpeg_vsize;
  priv->image.cap_param.yuv_hsize   = imager->cap_param.yuv_hsize;
  priv->image.cap_param.yuv_vsize   = imager->cap_param.yuv_vsize;
  priv->image.cap_param.jpeg_hsize  = imager->cap_param.jpeg_hsize;
  priv->image.cap_param.jpeg_vsize  = imager->cap_param.jpeg_vsize;

  return OK;
}

/******************************************************************************
 * isx012_change_color
 *****************************************************************************/
static int isx012_change_color(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val >= COLOR_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, FMODE, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_iso
 *****************************************************************************/
static int isx012_change_iso(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val >= ISX012_ISO_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, ISO_TYPE1, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_shutter
 *****************************************************************************/
static int isx012_change_shutter(isx012_dev_t *priv, uint16_t val)
{
  int ret;

  if (val > SHUTTER_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, SHT_PREMODE_TYPE1, val, 2);
  return ret;
}

/******************************************************************************
 * isx012_change_ev_correction
 *****************************************************************************/
static int isx012_change_ev_correction(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val >= EV_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, EVSEL, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_brightness
 *****************************************************************************/
static int isx012_change_brightness(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val > BRIGHTNESS_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, UIBRIGHTNESS, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_contrast
 *****************************************************************************/
static int isx012_change_contrast(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val > CONTRAST_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, UICONTRAST, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_jpeg_quality
 *****************************************************************************/
static int isx012_change_jpeg_quality(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val > JPEG_QUALITY_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, INT_QLTY2, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_ygamma
 *****************************************************************************/
static int isx012_change_ygamma(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val >= YGAMMA_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, YGAMMA_MODE, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_awb
 *****************************************************************************/
static int isx012_change_awb(isx012_dev_t *priv, uint8_t val)
{
  int ret;
  uint8_t num;
  const uint8_t convawb[] =
  {
    AWB_ISX012_ATM,
    AWB_ISX012_CLEARWEATHER,
    AWB_ISX012_SHADE,
    AWB_ISX012_CLOUDYWEATHER,
    AWB_ISX012_FLUORESCENTLIGHT,
    AWB_ISX012_LIGHTBULB,
  };

  if (val >= AWB_ISX012_MAX)
    {
      return -EINVAL;
    }

  num = convawb[val];
  ret = isx012_putreg(priv, AWB_SN1, num, 1);
  return ret;
}

/******************************************************************************
 * isx012_change_photometry
 *****************************************************************************/
static int isx012_change_photometry(isx012_dev_t *priv, uint8_t val)
{
  int ret;

  if (val >= PHOTOMETRY_ISX012_MAX)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, AE_SUB_SN1, val, 1);
  return ret;
}

/******************************************************************************
 * isx012_get_iso
 *****************************************************************************/
static int isx012_get_iso(isx012_dev_t *priv, uint8_t *val)
{
  *val = isx012_getreg(priv, ISOSENS_OUT, 1);
  return OK;
}

/******************************************************************************
 * isx012_get_shutterl
 *****************************************************************************/
static int isx012_get_shutterl(isx012_dev_t *priv, uint16_t *val)
{
  *val = isx012_getreg(priv, SHT_TIME_OUT_L, 2);
  return OK;
}

/******************************************************************************
 * isx012_get_shutterh
 *****************************************************************************/
static int isx012_get_shutterh(isx012_dev_t *priv, uint16_t *val)
{
  *val = isx012_getreg(priv, SHT_TIME_OUT_H, 2);
  return OK;
}

static int isx012_check_resolution(int16_t hsize, int16_t vsize)
{
  int reso;

  if ((hsize == OUT_HSIZE_QVGA) && (vsize == OUT_VSIZE_QVGA))
    {
      reso = RESOLUTION_ISX012_QVGA;
    }
  else if ((hsize == OUT_HSIZE_VGA) && (vsize == OUT_VSIZE_VGA))
    {
      reso = RESOLUTION_ISX012_VGA;
    }
  else if ((hsize == OUT_HSIZE_HD) && (vsize == OUT_VSIZE_HD))
    {
      reso = RESOLUTION_ISX012_HD;
    }
  else if ((hsize == OUT_HSIZE_QUADVGA) && (vsize == OUT_VSIZE_QUADVGA))
    {
      reso = RESOLUTION_ISX012_QUADVGA;
    }
  else if ((hsize == OUT_HSIZE_FULLHD) && (vsize == OUT_VSIZE_FULLHD))
    {
      reso = RESOLUTION_ISX012_FULLHD;
    }
  else if ((hsize == OUT_HSIZE_3M) && (vsize == OUT_VSIZE_3M))
    {
      reso = RESOLUTION_ISX012_3M;
    }
  else if ((hsize == OUT_HSIZE_5M) && (vsize == OUT_VSIZE_5M))
    {
      reso = RESOLUTION_ISX012_5M;
    }
  else
    {
      return -EINVAL;
    }

  return reso;
}

/******************************************************************************
 * isx012_change_crop
 *****************************************************************************/
static int isx012_change_crop(isx012_dev_t *priv, isx012_param_crop_t *p)
{
  int ret = OK;
  int idx;

  const isx012_reg_t regs[EZOOM_ISX012_REGNUM] =
    {
      { EZOOM_MAG, 0x0100, 2 },
      { OFFSET_X,  0x0000, 2 },
      { OFFSET_Y,  0x0000, 2 },
    };

  if (!p->crop)
    {
      for (idx = 0; idx < EZOOM_ISX012_REGNUM; idx++)
        {
          ret = isx012_putreg(priv,
                              regs[idx].regaddr,
                              regs[idx].regval,
                              regs[idx].regsize);
          if (ret < 0)
            {
              return ret;
            }

        }

    }
  else
    {
      int format;
      int rate;
      int reso;
      uint16_t hsize;
      uint16_t vsize;
      uint16_t px_offset;
      uint16_t ezoom_val[EZOOM_ISX012_REGNUM];
      uint16_t ezoom_mag_subsmpl[] =
      {
        1024,   /* QVGA     : x4  */
         512,   /* VGA      : x2  */
         256,   /* HD       : x1  */
         256,   /* Quad-VGA : x1  */
           0,   /* FULLHD   : invalid  */
           0,   /* 3M       : invalid  */
           0,   /* 5M       : invalid  */
      };
      uint16_t ezoom_mag_fllpx[] =
      {
        2048,   /* QVGA     : x8  */
        1024,   /* VGA      : x4  */
         512,   /* HD       : x2  */
         512,   /* Quad-VGA : x2  */
         256,   /* FULLHD   : x1  */
         256,   /* 3M       : x1  */
         256    /* 5M       : x1  */
      };

      if (g_mode == MODE_ISX012_MONITORING)
        {
          rate   = priv->image.moni_param.rate;
          format = priv->image.moni_param.format;
          hsize = (format == FORMAT_ISX012_YUV) ?
            priv->image.moni_param.yuv_hsize:priv->image.moni_param.jpeg_hsize;
          vsize = (format == FORMAT_ISX012_YUV) ?
            priv->image.moni_param.yuv_vsize:priv->image.moni_param.jpeg_vsize;
        }
      else
        {
          rate   = priv->image.cap_param.rate;
          format = priv->image.cap_param.format;
          hsize = (format == FORMAT_ISX012_YUV) ?
            priv->image.cap_param.yuv_hsize:priv->image.cap_param.jpeg_hsize;
          vsize = (format == FORMAT_ISX012_YUV) ?
            priv->image.cap_param.yuv_vsize:priv->image.cap_param.jpeg_vsize;
        }

      reso = isx012_check_resolution(hsize, vsize);
      if (reso < 0)
        {
          return reso;
        }

      if ((rate>= RATE_ISX012_15FPS) && (rate <= RATE_ISX012_5FPS))
        {
          ezoom_val[EZOOM_ISX012_MAG] = ezoom_mag_fllpx[reso];
          px_offset = EZOOM_ISX012_OFFSET_PX;
        }
      else if (rate == RATE_ISX012_30FPS)
        {
          if (!ezoom_mag_subsmpl[reso]) /* can not crop */
            {
              return -EPERM;
            }

          ezoom_val[EZOOM_ISX012_MAG] = ezoom_mag_subsmpl[reso];
          px_offset = (EZOOM_ISX012_OFFSET_PX << 1);
        }
      else
        {
          return -EPERM; /* can not crop */
        }

      ezoom_val[EZOOM_ISX012_OFFSET_X] = p->x_offset * px_offset;
      ezoom_val[EZOOM_ISX012_OFFSET_Y] = -(p->y_offset) * px_offset;

      for (idx = 0; idx < EZOOM_ISX012_REGNUM; idx++)
        {
          ret = isx012_putreg(priv,
                              regs[idx].regaddr,
                              ezoom_val[idx],
                              regs[idx].regsize);
          if (ret < 0)
            {
              return ret;
            }

        }
    }

  return ret;
}

/******************************************************************************
 * isx012_read_reg
 *****************************************************************************/
static int isx012_read_reg(isx012_dev_t *priv, isx012_reg_t *reg)
{
  reg->regval = isx012_getreg(priv, reg->regaddr, reg->regsize);

  return OK;
}

/******************************************************************************
 * isx012_write_reg
 *****************************************************************************/
static int isx012_write_reg(isx012_dev_t *priv, isx012_reg_t *reg)
{
  int ret;

  ret = isx012_putreg(priv, reg->regaddr, reg->regval, reg->regsize);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/******************************************************************************
 * isx012_set_moni_refresh
 *****************************************************************************/
static int isx012_set_moni_refresh(isx012_dev_t *priv, unsigned long arg)
{
  int ret = 0;
  uint8_t mask_num;
  int i;

  if (g_state != STATE_ISX012_ACTIVE)
    {
      return -EPERM;
    }

  if (g_mode != MODE_ISX012_MONITORING)
    {
      return -EPERM;
    }

  /* Set MONI_REFRESH */
  isx012_putreg(priv, INTCLR0, CM_CHANGED_STS, 1);
  ret = isx012_putreg(priv, MONI_REFRESH, 1, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait CM_CHANGED */
  ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                   CAMERA_MODE_DELAY_TIME,
                                   CAMERA_MODE_WAIT_TIME,
                                   CAMERA_MODE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  /* Invalid frame skip */
  isx012_putreg(priv, INTCLR0, VINT_STS, 1);
  mask_num = isx012_getreg(priv, RO_MASK_NUM, sizeof(mask_num));
  for (i=0; i<mask_num; i++)
    {
      /* Wait Next VINT */
      ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                       VINT_WAIT_TIME, VINT_TIMEOUT);
      if (ret != 0)
        {
          return ret;
        }
    }

  return OK;
}

int isx012_initialize(isx012_dev_t *priv)
{
  int ret;
  isx012_setparam_t set_moni_param;
  isx012_setparam_t set_cap_param;

  ret = isx012_chk_param(&(priv->image.moni_param), &set_moni_param);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_chk_param(&(priv->image.cap_param), &set_cap_param);
  if (ret < 0)
    {
      return ret;
    }

#ifdef ISX012_NOT_USE_NSTBY
  board_isx012_release_sleep();
  board_isx012_release_reset();
  usleep(6000);
#else
  board_isx012_release_reset();
  usleep(6000);
#endif

#ifdef ISX012_CHECK_IN_DETAIL
  /* check the chip id*/
  ret = isx012_chipid(priv);
  if (ret < 0)
    {
      imagererr("isx012_chipid failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }
#endif

  /* Wait OM_CHANGED Power OFF -> PreSleep */
  ret = isx012_chk_int_state(priv, OM_CHANGED_STS, DEVICE_STATE_DELAY_TIME,
                             DEVICE_STATE_WAIT_TIME, DEVICE_STATE_TIMEOUT);
  if (ret != OK)
    {
      imagererr("OM_CHANGED_STS(PreSleep) is Not occured: %d\n", ret);
      return ret;
    }

  g_state = STATE_ISX012_PRESLEEP;

#ifndef ISX012_NOT_USE_NSTBY
  /* set the isx012 clock */
  /* Write INCK_SET register ISX012 change state PreSleep -> Sleep */
  ret = isx012_putreglist(priv, g_isx012_presleep, ISX012_PRESLEEP_NENTRIES);
  if (ret != OK)
    {
      imagererr("isx012_putreglist(INCK_SET) failed: %d\n", ret);
      return ret;
    }

  /* Wait OM_CHANGED PreSleep -> Sleep */
  ret = isx012_chk_int_state(priv, OM_CHANGED_STS, DEVICE_STATE_DELAY_TIME,
                             DEVICE_STATE_WAIT_TIME, DEVICE_STATE_TIMEOUT);
  if (ret != OK)
    {
      imagererr("OM_CHANGED_STS(Sleep) is Not occured: %d\n", ret);
      return ret;
    }
#endif

  g_state = STATE_ISX012_SLEEP;
  g_i2c_freq = I2CFREQ_FAST;

  /* initialize the isx012 hardware */
  ret = isx012_putreglist(priv, g_isx012_def_init, ISX012_RESET_NENTRIES);
  if (ret < 0)
    {
      imagererr("isx012_putreglist failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

#ifdef ISX012_AF_EN
#ifdef ISX012_FIRST_SET_AF
/*  board_isx012_release_sleep(); */

  /* AF default driver load */
  ret = isx012_putreg(priv, AF_EXT, 1, 1);
  if (ret < 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  ret = isx012_chk_reg_bit(priv, AF_EXT,
                                 AF_EXT_AFRAMDRVFIN,
                                 AF_EXT_DELAY_TIME,
                                 AF_EXT_WAIT_TIME,
                                 AF_EXT_TIMEOUT);
  if (ret < 0 || ret > 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  g_af_loadsts = 1;

/*  board_isx012_set_sleep(1); */
/*  g_state = STATE_ISX012_SLEEP; */
#endif
#endif

  ret = isx012_set_mode_param(priv, set_moni_param, set_cap_param);
  if (ret < 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  return OK;
}

int isx012_open( void )
{
  FAR struct isx012_dev_s *priv = &g_private;
  int ret = 0;

  ret = board_isx012_power_on();
  if (ret < 0)
    {
      imagererr("Failed to power on %d\n", ret);
      return ret;
    }

  ret = isx012_initialize(priv);
  if (ret < 0)
    {
      imagererr("Failed to open %d\n", ret);
      board_isx012_set_reset();
      board_isx012_power_off();
      return ret;
    }

  ret = cxd56_cisifinit();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisifinit %d\n", ret);
    }

  return ret;
}

int isx012_close( void )
{
  int ret = 0;

  if (g_state == STATE_ISX012_ACTIVE)
    {
      board_isx012_set_sleep(1);
    }

  board_isx012_set_reset();

  ret = board_isx012_power_off();
  if (ret < 0)
    {
      imagererr("Failed to power off %d\n", ret);
      return ret;
    }

  ret = cxd56_cisiffinalize();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisiffinalize %d\n", ret);
      return ret;
    }

  g_i2c_freq = I2CFREQ_STANDARD;
  g_state    = STATE_ISX012_POWEROFF;
  g_af_loadsts = 0;

  return ret;
}

int isx012_ioctl(int cmd, unsigned long arg)
{
  FAR struct isx012_dev_s *priv  = &g_private;
  int ret = OK;

  switch (cmd)
    {
      case IMGIOC_SETSTATE:
        ret = isx012_change_device_state(priv, arg);
        break;
      case IMGIOC_SETMODE:
        ret = isx012_change_camera_mode(priv, arg);
        break;
      case IMGIOC_SETMODEP:
        ret = isx012_change_mode_param(priv, (isx012_t *)arg);
        break;
      case IMGIOC_SETCISIF:
        ret = isx012_change_cisif(priv, (cisif_param_t *)arg);
        break;
      case IMGIOC_CHGCOLOR:
        ret = isx012_change_color(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGISO:
        ret = isx012_change_iso(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGSHUTTER:
        ret = isx012_change_shutter(priv, (uint16_t)arg);
        break;
      case IMGIOC_CHGEV:
        ret = isx012_change_ev_correction(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGBRIGHT:
        ret = isx012_change_brightness(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGCONTRAST:
        ret = isx012_change_contrast(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGJQUALITY:
        ret = isx012_change_jpeg_quality(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGYGAMMA:
        ret = isx012_change_ygamma(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGAWB:
        ret = isx012_change_awb(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGPMETRY:
        ret = isx012_change_photometry(priv, (uint8_t)arg);
        break;
      case IMGIOC_CHGCROP:
        ret = isx012_change_crop(priv, (isx012_param_crop_t *)arg);
        break;
      case IMGIOC_GETISO:
        ret = isx012_get_iso(priv, (uint8_t *)arg);
        break;
      case IMGIOC_GETSHTL:
        ret = isx012_get_shutterl(priv, (uint16_t *)arg);
        break;
      case IMGIOC_GETSHTH:
        ret = isx012_get_shutterh(priv, (uint16_t *)arg);
        break;
      case IMGIOC_READREG:
        ret = isx012_read_reg(priv, (isx012_reg_t *)arg);
        break;
      case IMGIOC_WRITEREG:
        ret = isx012_write_reg(priv, (isx012_reg_t *)arg);
        break;
      case IMGIOC_MONIREF:
        ret = isx012_set_moni_refresh(priv, arg);
        break;
      default:
        imagererr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

int isx012_register(FAR struct i2c_master_s *i2c)
{
  FAR struct isx012_dev_s *priv = &g_private;

  g_i2c_freq = I2CFREQ_STANDARD;
  g_state    = STATE_ISX012_POWEROFF;
  g_af_loadsts = 0;

  priv->i2c  = i2c;
  priv->addr = ISX012_I2C_SLV_ADDR;
  priv->freq = g_i2c_freq;
  /* moni=YUV, cap=JPEG */
  priv->image.moni_param.format     = FORMAT_ISX012_YUV;
  priv->image.moni_param.rate       = RATE_ISX012_30FPS;
  priv->image.cap_param.format      = FORMAT_ISX012_JPEG_MODE1;
  priv->image.cap_param.rate        = RATE_ISX012_15FPS;
  /* moni=QVGA, cap=VGA */
  priv->image.moni_param.yuv_hsize  = OUT_HSIZE_QVGA;
  priv->image.moni_param.yuv_vsize  = OUT_VSIZE_QVGA;
  priv->image.moni_param.jpeg_hsize = OUT_HSIZE_QVGA;
  priv->image.moni_param.jpeg_vsize = OUT_VSIZE_QVGA;
  priv->image.cap_param.yuv_hsize   = OUT_HSIZE_QVGA;
  priv->image.cap_param.yuv_vsize   = OUT_VSIZE_QVGA;
  priv->image.cap_param.jpeg_hsize  = OUT_HSIZE_FULLHD;
  priv->image.cap_param.jpeg_vsize  = OUT_VSIZE_FULLHD;
  sem_init(&priv->wait, 0, 0);

  return OK;
}

int isx012_unregister(void)
{
  FAR struct isx012_dev_s *priv = &g_private;

  sem_destroy(&priv->wait);

  return OK;
}
