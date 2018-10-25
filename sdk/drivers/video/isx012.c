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
#include <arch/board/board.h>
#include <arch/chip/cisif.h>
#include <arch/irq.h>

#include <nuttx/video/isx012.h>
#include <video/video.h>
#include <video/video_halif.h>
#include "isx012_reg.h"
#include "isx012_range.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The following macro is enabled because */
/* it is to make stable startup. (other case) */
/* #define ISX012_NOT_USE_NSTBY */

/* The following macro is disabled because it is to see detailed control. */
/* #define ISX012_CHECK_IN_DETAIL */

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

#define VINT_TIMEOUT                (400) /* ms */
#define VINT_WAIT_TIME                (5) /* ms */
#define VINT_DELAY_TIME               (0) /* ms */
#define CAMERA_MODE_TIMEOUT         (800) /* ms */
#define CAMERA_MODE_WAIT_TIME        (10) /* ms */
#define CAMERA_MODE_DELAY_TIME        (0) /* ms */
#define DEVICE_STATE_TIMEOUT        (100) /* ms */
#define DEVICE_STATE_WAIT_TIME        (1) /* ms */
#define DEVICE_STATE_DELAY_TIME       (2) /* ms */

#define I2CFREQ_STANDARD         (100000) /* Standard mode : 100kHz */
#define I2CFREQ_FAST             (400000) /* Fast mode     : 400kHz */

#define ISX012_SIZE_STEP              (2)

#ifdef CONFIG_DEBUG_IMAGER_ERROR
#  define imagererr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define imagererr(x...)
#endif

#ifdef CONFIG_DEBUG_IMAGER_WARN
#  define imagerwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define imagerwarn(x...)
#endif

#ifdef CONFIG_DEBUG_IMAGER_INFO
#  define imagerinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define imagerinfo(x...)
#endif

#define CHECK_RANGE(value,min,max,step) do { \
                                          if ((value < min) || \
                                              (value > max) || \
                                              ((value - min) % step != 0)) \
                                            { \
                                              return -EINVAL;\
                                            } \
                                         } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum isx012_state_e {
  STATE_ISX012_PRESLEEP,
  STATE_ISX012_SLEEP,
  STATE_ISX012_ACTIVE,
  STATE_ISX012_POWEROFF,
};

typedef enum isx012_state_e isx012_state_t;

struct isx012_reg_s {
  uint16_t regaddr;
  uint16_t regval;
  uint8_t  regsize;
};

typedef struct isx012_reg_s isx012_reg_t;

struct isx012_conv_v4l2_to_regval_s
{
  int32_t v4l2;
  int16_t regval;
};

typedef struct isx012_conv_v4l2_to_regval_s isx012_conv_v4l2_to_regval_t;

struct isx012_modeparam_s {
  uint8_t  fps;         /* use ISX012 register setting value */
  uint32_t format;      /* use V4L2 definition */
  uint16_t hsize;
  uint16_t vsize;
  uint16_t int_hsize;
  uint16_t int_vsize;
};

typedef struct isx012_modeparam_s isx012_modeparam_t;

struct isx012_param_s
{
  isx012_modeparam_t monitor;  /* Parameter for monitor mode */
  isx012_modeparam_t capture;  /* Parameter for capture mode */
};

typedef struct isx012_param_s isx012_param_t;

struct isx012_dev_s
{
  FAR struct i2c_master_s *i2c;        /* I2C interface */
  uint8_t                 i2c_addr;    /* I2C address */
  int                     i2c_freq;    /* Frequency */
  isx012_state_t          state;       /* ISX012 status */
  bool                    dma_state;   /* true means "in DMA" */
  uint8_t                 mode;        /* ISX012 mode */
  isx012_param_t          param;       /* ISX012 paramerters */
  void                    *video_priv; /* pointer to video private data */
};

typedef struct isx012_dev_s isx012_dev_t;


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
static int isx012_set_mode_param(isx012_dev_t *priv,
                                 enum v4l2_buf_type type,
                                 isx012_modeparam_t *param);
static int isx012_change_camera_mode(isx012_dev_t *priv, uint8_t mode);
static int isx012_change_device_state(isx012_dev_t *priv, isx012_state_t state);
static int isx012_set_supported_frminterval(uint32_t fps_index,
                                            FAR struct v4l2_fract *interval);
static int8_t isx012_get_maximum_fps(FAR struct v4l2_frmivalenum *frmival);


/* video driver HAL infterface */

static int isx012_open(FAR void *video_private);
static int isx012_close(void);
static int isx012_do_halfpush(bool enable);
static int isx012_set_buftype(enum v4l2_buf_type type);
static int isx012_set_buf(uint32_t bufaddr, uint32_t bufsize);
static int isx012_cancel_dma(void);
static int isx012_check_fmt(enum v4l2_buf_type buf_type,
                            uint32_t           pixel_format);
static int isx012_get_range_of_fmt(FAR struct v4l2_fmtdesc *format);
static int isx012_get_range_of_framesize(FAR struct v4l2_frmsizeenum
                                         *frmsize);
static int isx012_try_format(FAR struct v4l2_format *format);
static int isx012_set_format(FAR struct v4l2_format *format);
static int isx012_get_range_of_frameinterval(FAR struct v4l2_frmivalenum
                                             *frmival);
static int isx012_set_frameinterval(FAR struct v4l2_streamparm *parm);
static int isx012_get_range_of_ctrlvalue(FAR struct v4l2_query_ext_ctrl
                                         *range);
static int isx012_get_menu_of_ctrlvalue(FAR struct v4l2_querymenu *menu);
static int isx012_get_ctrlvalue(uint16_t ctrl_class,
                                FAR struct v4l2_ext_control *control);
static int isx012_set_ctrlvalue(uint16_t ctrl_class,
                                FAR struct v4l2_ext_control *control);
static int isx012_refresh(void);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static isx012_dev_t   g_isx012_private;

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
  {FRM_FIX_SN1_2,     0xFF, 0x01}, /* Fix framerate */
  {FAST_MODECHG_EN,   0x01, 0x01},
  {FAST_SHT_MODE_SEL, 0x01, 0x01},
  {CAP_HALF_AE_CTRL,  0x07, 0x01}, /* HAFREL=HIGHSPEED, CAP=Auto  */
  {HALF_AWB_CTRL,     0x01, 0x01},
  {AESPEED_FAST,      0x0F, 0x01},
  {FASTMOVE_TIMEOUT,  0x2D, 0x01},
  {YGAMMA_MODE,       0x01, 0x01},
  {INT_QLTY2,         0x50, 0x01},
};
#define ISX012_RESET_NENTRIES ARRAY_NENTRIES(g_isx012_def_init)

static isx012_conv_v4l2_to_regval_t
 g_isx012_supported_colorfx[ISX012_MAX_COLOREFFECT + 1] = {
  {V4L2_COLORFX_NONE,         REGVAL_EFFECT_NONE},
  {V4L2_COLORFX_BW,           REGVAL_EFFECT_MONOTONE},
  {V4L2_COLORFX_SEPIA,        REGVAL_EFFECT_SEPIA},
  {V4L2_COLORFX_NEGATIVE,     REGVAL_EFFECT_NEGPOS},
  {V4L2_COLORFX_SKETCH,       REGVAL_EFFECT_SKETCH},
  {V4L2_COLORFX_SOLARIZATION, REGVAL_EFFECT_SOLARIZATION},
  {V4L2_COLORFX_PASTEL,       REGVAL_EFFECT_PASTEL}};

static isx012_conv_v4l2_to_regval_t
 g_isx012_supported_presetwb[ISX012_MAX_PRESETWB + 1] = {
  {V4L2_WHITE_BALANCE_AUTO,         REGVAL_AWB_ATM},
  {V4L2_WHITE_BALANCE_INCANDESCENT, REGVAL_AWB_LIGHTBULB},
  {V4L2_WHITE_BALANCE_FLUORESCENT,  REGVAL_AWB_FLUORESCENTLIGHT},
  {V4L2_WHITE_BALANCE_DAYLIGHT,     REGVAL_AWB_CLEARWEATHER},
  {V4L2_WHITE_BALANCE_CLOUDY,       REGVAL_AWB_CLOUDYWEATHER},
  {V4L2_WHITE_BALANCE_SHADE,        REGVAL_AWB_SHADE}};

static isx012_conv_v4l2_to_regval_t
 g_isx012_supported_photometry[ISX012_MAX_PHOTOMETRY + 1] = {
  {V4L2_EXPOSURE_METERING_AVERAGE,         REGVAL_PHOTOMETRY_AVERAGE},
  {V4L2_EXPOSURE_METERING_CENTER_WEIGHTED, REGVAL_PHOTOMETRY_CENTERWEIGHT},
  {V4L2_EXPOSURE_METERING_SPOT,            REGVAL_PHOTOMETRY_SPOT},
  {V4L2_EXPOSURE_METERING_MATRIX,          REGVAL_PHOTOMETRY_MULTIPATTERN}};

static isx012_conv_v4l2_to_regval_t
 g_isx012_supported_iso[ISX012_MAX_ISO + 1] = {
  {25*1000,   REGVAL_ISO_25},
  {32*1000,   REGVAL_ISO_32},
  {40*1000,   REGVAL_ISO_40},
  {50*1000,   REGVAL_ISO_50},
  {64*1000,   REGVAL_ISO_64},
  {80*1000,   REGVAL_ISO_80},
  {100*1000,  REGVAL_ISO_100},
  {125*1000,  REGVAL_ISO_125},
  {160*1000,  REGVAL_ISO_160},
  {200*1000,  REGVAL_ISO_200},
  {250*1000,  REGVAL_ISO_250},
  {320*1000,  REGVAL_ISO_320},
  {400*1000,  REGVAL_ISO_400},
  {500*1000,  REGVAL_ISO_500},
  {640*1000,  REGVAL_ISO_640},
  {800*1000,  REGVAL_ISO_800},
  {1000*1000, REGVAL_ISO_1000},
  {1250*1000, REGVAL_ISO_1250},
  {1600*1000, REGVAL_ISO_1600}};

static struct video_devops_s g_isx012_video_devops =
{
  .open                       = isx012_open,
  .close                      = isx012_close,
  .do_halfpush                = isx012_do_halfpush,
  .set_buftype                = isx012_set_buftype,
  .set_buf                    = isx012_set_buf,
  .cancel_dma                 = isx012_cancel_dma,
  .get_range_of_fmt           = isx012_get_range_of_fmt,
  .get_range_of_framesize     = isx012_get_range_of_framesize,
  .try_format                 = isx012_try_format,
  .set_format                 = isx012_set_format,
  .get_range_of_frameinterval = isx012_get_range_of_frameinterval,
  .set_frameinterval          = isx012_set_frameinterval,
  .get_range_of_ctrlvalue     = isx012_get_range_of_ctrlvalue,
  .get_menu_of_ctrlvalue      = isx012_get_menu_of_ctrlvalue,
  .get_ctrlvalue              = isx012_get_ctrlvalue,
  .set_ctrlvalue              = isx012_set_ctrlvalue,
  .refresh                    = isx012_refresh,
};


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t isx012_getreg(isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint16_t regval;
  volatile uint8_t buffer[2];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
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
  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
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

static int isx012_replace_fmt_v4l2val_to_regval(uint32_t v4l2val, uint8_t *regval)
{
  if (regval == NULL)
    {
      return -EINVAL;
    }

  switch (v4l2val)
    {
      case V4L2_PIX_FMT_UYVY:
        *regval = REGVAL_OUTFMT_YUV;
        break;

      case V4L2_PIX_FMT_JPEG:
        *regval = REGVAL_OUTFMT_JPEG;
        break;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        *regval = REGVAL_OUTFMT_INTERLEAVE;
        break;

      default:
        /* Unsupported format */

        return -EINVAL;
    }

  return OK;
}

static int isx012_set_mode_param(isx012_dev_t *priv,
                                 enum v4l2_buf_type type,
                                 isx012_modeparam_t *param)
{
  int ret = 0;
  uint8_t format;
  uint16_t fps_regaddr;
  uint16_t fmt_regaddr;
  uint16_t sensmode_regaddr;
  uint16_t hsize_regaddr;
  uint16_t vsize_regaddr;
  uint8_t  sensmode;

  /* Get register address for type  */

  if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      fps_regaddr      = FPSTYPE_MONI;
      fmt_regaddr      = OUTFMT_MONI;
      sensmode_regaddr = SENSMODE_MONI;
      hsize_regaddr    = HSIZE_MONI;
      vsize_regaddr    = VSIZE_MONI;
    }
  else
    {
      fps_regaddr      = FPSTYPE_CAP;
      fmt_regaddr      = OUTFMT_CAP;
      sensmode_regaddr = SENSMODE_CAP;
      hsize_regaddr    = HSIZE_CAP;
      vsize_regaddr    = VSIZE_CAP;
    }

  ret = isx012_putreg(priv, fps_regaddr,  param->fps, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_replace_fmt_v4l2val_to_regval(param->format, &format);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, fmt_regaddr,   format, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  switch (param->fps)
    {
      case REGVAL_FPSTYPE_120FPS:
        sensmode = REGVAL_SENSMODE_1_8;
        break;

      case REGVAL_FPSTYPE_60FPS:
        sensmode = REGVAL_SENSMODE_1_4;
        break;

      case REGVAL_FPSTYPE_30FPS:
        sensmode = REGVAL_SENSMODE_1_2;
        break;

      default:
        sensmode = REGVAL_SENSMODE_ALLPIX;
        break;
    }

  ret = isx012_putreg(priv, sensmode_regaddr, sensmode, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, hsize_regaddr,    param->hsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, vsize_regaddr,    param->vsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  if (format == REGVAL_OUTFMT_INTERLEAVE)
    {
      ret = isx012_putreg(priv, HSIZE_TN, param->int_hsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

      ret = isx012_putreg(priv, VSIZE_TN, param->int_vsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }
    }

  return ret;
}

void isx012_callback(uint8_t code, uint32_t size, uint32_t addr)
{
  enum v4l2_buf_type type;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  if (priv->mode == REGVAL_MODESEL_CAP)
    {
      /* ISX012 capture mode = still capture */

      type = V4L2_BUF_TYPE_STILL_CAPTURE;
    }
  else
    {
      /* ISX012 monitor mode or halfrelease mode = video capture */

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

  video_common_notify_dma_done(code, type, size, priv->video_priv);
 
  return;
}


/****************************************************************************
 * isx012_change_camera_mode
 ****************************************************************************/
static int isx012_change_camera_mode(isx012_dev_t *priv, uint8_t mode)
{
  int ret = 0;
  uint8_t format_data;
  uint32_t vifmode;
#ifdef ISX012_FRAME_SKIP_EN
  uint8_t mask_num;
  int i;
#endif /* ISX012_FRAME_SKIP_EN */

  if (priv->state != STATE_ISX012_ACTIVE)
    {
      return -EPERM;
    }

  switch (mode)
    {
      case REGVAL_MODESEL_MON:
        if (priv->mode == REGVAL_MODESEL_MON)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_MONI, 1);
        break;
      case REGVAL_MODESEL_CAP:
        if (priv->mode == REGVAL_MODESEL_CAP)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_CAP, 1);
        break;
      case REGVAL_MODESEL_HREL:
        if (priv->mode != REGVAL_MODESEL_MON)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_MONI, 1);
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

  ret = isx012_putreg(priv, MODESEL, mode, sizeof(mode));
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

  priv->mode = mode;

#ifdef ISX012_FRAME_SKIP_EN
  if (mode != REGVAL_MODESEL_HREL)
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

/****************************************************************************
 * isx012_change_device_state
 ****************************************************************************/
static int isx012_change_device_state(isx012_dev_t *priv,
                                      isx012_state_t state)
{
  int ret = 0;
#ifdef ISX012_FRAME_SKIP_EN
  int i;
  uint8_t mute_cnt;
#endif /* ISX012_FRAME_SKIP_EN */

  if (priv->state == STATE_ISX012_PRESLEEP || priv->state == state)
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

  priv->state = state;

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
    }

  priv->mode = REGVAL_MODESEL_MON;

  return OK;
}

int init_isx012(FAR struct isx012_dev_s *priv)
{
  int ret;

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

  priv->state = STATE_ISX012_PRESLEEP;

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

  priv->state = STATE_ISX012_SLEEP;
  priv->i2c_freq = I2CFREQ_FAST;

  /* initialize the isx012 hardware */
  ret = isx012_putreglist(priv, g_isx012_def_init, ISX012_RESET_NENTRIES);
  if (ret < 0)
    {
      imagererr("isx012_putreglist failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

  /* monitor mode default format: YUV4:2:2 QVGA */

  priv->param.monitor.fps         = REGVAL_FPSTYPE_30FPS;
  priv->param.monitor.format      = V4L2_PIX_FMT_UYVY;
  priv->param.monitor.hsize       = VIDEO_HSIZE_QVGA;
  priv->param.monitor.vsize       = VIDEO_VSIZE_QVGA;
  priv->param.monitor.int_hsize   = 0;
  priv->param.monitor.int_vsize   = 0;

  ret = isx012_set_mode_param(priv,
                              V4L2_BUF_TYPE_VIDEO_CAPTURE,
                              &priv->param.monitor);
  if (ret < 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  /* capture mode default format: JPEG FULLHD */

  priv->param.capture.fps         = REGVAL_FPSTYPE_15FPS;
  priv->param.capture.format      = V4L2_PIX_FMT_JPEG;
  priv->param.capture.hsize       = VIDEO_HSIZE_FULLHD;
  priv->param.capture.vsize       = VIDEO_VSIZE_FULLHD;
  priv->param.capture.int_hsize   = 0;
  priv->param.capture.int_vsize   = 0;

  ret = isx012_set_mode_param(priv,
                              V4L2_BUF_TYPE_STILL_CAPTURE,
                              &priv->param.capture);
  if (ret < 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  return ret;
}

static int isx012_open(FAR void *video_private)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  int ret = 0;

  ret = board_isx012_power_on();
  if (ret < 0)
    {
      imagererr("Failed to power on %d\n", ret);
      return ret;
    }

  ret = init_isx012(priv);
  if (ret < 0)
    {
      imagererr("Failed to init_isx012 %d\n", ret);
      board_isx012_set_reset();
      board_isx012_power_off();
      return ret;
    }

  ret = cxd56_cisifinit();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisifinit %d\n", ret);
      return ret;
    }

  /* Save video private information address */

  g_isx012_private.video_priv = video_private;

  return ret;
}

static int isx012_close(void)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  g_isx012_private.video_priv = NULL;

  int ret = 0;

  if (priv->state == STATE_ISX012_ACTIVE)
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

  ret = cxd56_cisifstopcapture();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisifstopcapture %d\n", ret);
      return ret;
    }

  ret = cxd56_cisiffinalize();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisiffinalize %d\n", ret);
      return ret;
    }

  priv->i2c_freq = I2CFREQ_STANDARD;
  priv->state    = STATE_ISX012_POWEROFF;

  return ret;
}

static int isx012_do_halfpush(bool enable)
{
  int ret;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  if (enable)
    {
      /* state transition : MONITORING -> HALFRELEASE */

      ret = isx012_change_camera_mode(priv, REGVAL_MODESEL_HREL);
    }
  else
    {
      /* state transition : HALFRELEASE -> MONITORING */

      ret = isx012_change_camera_mode(priv, REGVAL_MODESEL_MON);
    }

  return ret;
}

static int isx012_set_buftype(enum v4l2_buf_type type)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  uint8_t                 mode;

  if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      mode = REGVAL_MODESEL_MON;
    }
  else
    {
      mode = REGVAL_MODESEL_CAP;
    }

  /* In no active case, activate */

  if (priv->state != STATE_ISX012_ACTIVE)
    {
      isx012_change_device_state(priv, STATE_ISX012_ACTIVE);
    }

  return isx012_change_camera_mode(priv, mode);
}

static int isx012_set_buf(uint32_t bufaddr, uint32_t bufsize)
{
  int ret;
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  cisif_param_t cis_param = {0};
  cisif_sarea_t sarea = {0};
  isx012_modeparam_t *mode_param = NULL;

  sarea.strg_addr     = (uint8_t *)bufaddr;
  sarea.strg_size     = bufsize;

  if (priv->dma_state)
    {
      ret = cxd56_cisifsetdmabuf(&sarea);
    }
  else
    {
      if (priv->mode == REGVAL_MODESEL_CAP)
        {
          mode_param = &priv->param.capture;
        }
      else
        {
          mode_param = &priv->param.monitor;
        }

      switch (mode_param->format)
        {
          case V4L2_PIX_FMT_UYVY:
            /* Set YUV 4:2:2 information */

            cis_param.yuv_param.hsize = mode_param->hsize;
            cis_param.yuv_param.vsize = mode_param->vsize;

            break;

          case V4L2_PIX_FMT_JPEG:
            /* Set JPEG information */

            /* no setting */

            break;

          case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
            /* Set JPEG + YUV 4:2:2 information */

            cis_param.yuv_param.hsize = mode_param->int_hsize;
            cis_param.yuv_param.vsize = mode_param->int_vsize;

            break;

          default:
            /* Unsupported format */

            return -EINVAL;
        }

      cis_param.format    = mode_param->format;
      cis_param.comp_func = isx012_callback;

      ret = cxd56_cisifstartcapture(&cis_param, &sarea);
      if (ret != OK)
        {
          return ret;
        }

      priv->dma_state = true;
    }

  return ret;
}

static int isx012_cancel_dma(void)
{
  int ret;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  ret =  cxd56_cisifstopcapture();
  if (ret != OK)
    {
      return ret; 
    }

  priv->dma_state = false;
  return ret;
}

static int isx012_check_fmt(enum v4l2_buf_type buf_type,
                            uint32_t           pixel_format)
{
  switch (buf_type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        if (pixel_format != V4L2_PIX_FMT_UYVY)
          {
            /* Unsupported format */

            return -EINVAL;
          }

        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        if ((pixel_format != V4L2_PIX_FMT_JPEG) &&
#if 0 /* To Be Supported */
            (pixel_format != V4L2_PIX_FMT_JPEG_WITH_SUBIMG) &&
#endif
            (pixel_format != V4L2_PIX_FMT_UYVY))
          {
            /* Unsupported format */

            return -EINVAL;
          }

        break;

      default:
        /* Unsupported type */

        return -EINVAL;
    }

  return OK;
}

static int isx012_get_range_of_fmt(FAR struct v4l2_fmtdesc *format)
{
  if (format == NULL)
    {
      return -EINVAL;
    }

  switch (format->type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:

        switch (format->index)
          {
            case 0:
              /* YUV 4:2:2 */

              strncpy(format->description, "YUV 4:2:2", V4L2_FMT_DSC_MAX);
              format-> pixelformat = V4L2_PIX_FMT_UYVY;

              break;
 
            default:  /* 1, 2, ... */
              return -EINVAL;
          }

        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        switch (format->index)
          {
            case 0:
              /* JPEG */

              strncpy(format->description, "JPEG", V4L2_FMT_DSC_MAX);
              format->pixelformat = V4L2_PIX_FMT_JPEG;

              break;

            case 1:
#if 0 /* To Be Supported */
              /* JPEG + YUV 4:2:2 */

              strncpy(format->description,
                      "JPEG + YUV 4:2:2",
                      V4L2_FMT_DSC_MAX);
              format->pixelformat        = V4L2_PIX_FMT_JPEG_WITH_SUBIMG;
              format->subimg_pixelformat = V4L2_PIX_FMT_UYVY;

              break;

            case 2:
#endif
              /* YUV 4:2:2 */

              strncpy(format->description, "YUV 4:2:2", V4L2_FMT_DSC_MAX);
              format->pixelformat = V4L2_PIX_FMT_UYVY;

              break;

            default:  /* 3, 4, ... */
              return -EINVAL;
          }

        break;

      default:
        /* Unsupported type */

        return -EINVAL;
    }

  return OK;
}

static int isx012_get_range_of_framesize(FAR struct v4l2_frmsizeenum *frmsize)
{
  int ret;

  if (frmsize == NULL)
    {
      return -EINVAL;
    }

  if (frmsize->index != 0)
    {
      return -EINVAL;
    }

  ret = isx012_check_fmt(frmsize->buf_type, frmsize->pixel_format);
  if (ret != OK)
    {
      return ret;
    }

  switch (frmsize->pixel_format)
    {
      case V4L2_PIX_FMT_UYVY:                /* YUV 4:2:2 */
        frmsize->type                        = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->stepwise.min_width          = OUT_YUV_HSIZE_MIN;
        frmsize->stepwise.max_width          = OUT_YUV_15FPS_HSIZE_MAX;
        frmsize->stepwise.step_width         = ISX012_SIZE_STEP;
        frmsize->stepwise.min_height         = OUT_YUV_VSIZE_MIN;
        frmsize->stepwise.max_height         = OUT_YUV_15FPS_VSIZE_MAX;
        frmsize->stepwise.step_height        = ISX012_SIZE_STEP;

        break;

      case V4L2_PIX_FMT_JPEG:                /* JPEG */
        frmsize->type                        = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->stepwise.min_width          = OUT_JPG_HSIZE_MIN;
        frmsize->stepwise.max_width          = OUT_JPG_15FPS_HSIZE_MAX;
        frmsize->stepwise.step_width         = ISX012_SIZE_STEP;
        frmsize->stepwise.min_height         = OUT_JPG_VSIZE_MIN;
        frmsize->stepwise.max_height         = OUT_JPG_15FPS_VSIZE_MAX;
        frmsize->stepwise.step_height        = ISX012_SIZE_STEP;

        break;
#if 0 /* To Be Supported */
      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:    /* JPEG + YUV 4:2:2 */
        if (frmsize->subimg_pixel_format != V4L2_PIX_FMT_UYVY)
          {
            /* Unsupported pixel format */

            return -EINVAL;
          }

        frmsize->type                        = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->stepwise.min_width          = OUT_JPG_HSIZE_MIN;
        frmsize->stepwise.max_width          = OUT_JPGINT_15FPS_HSIZE_MAX;
        frmsize->stepwise.step_width         = ISX012_SIZE_STEP;
        frmsize->stepwise.min_height         = OUT_JPG_VSIZE_MIN;
        frmsize->stepwise.max_height         = OUT_JPGINT_15FPS_VSIZE_MAX;
        frmsize->stepwise.step_height        = ISX012_SIZE_STEP;

        frmsize->subimg_type                 = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->subimg.stepwise.min_width   = OUT_YUV_HSIZE_MIN;
        frmsize->subimg.stepwise.max_width   = OUT_YUVINT_30FPS_HSIZE_MAX;
        frmsize->subimg.stepwise.step_width  = ISX012_SIZE_STEP;
        frmsize->subimg.stepwise.min_height  = OUT_YUV_VSIZE_MIN;
        frmsize->subimg.stepwise.max_height  = OUT_YUVINT_30FPS_VSIZE_MAX;
        frmsize->subimg.stepwise.step_height = ISX012_SIZE_STEP;
 
        break;
#endif
      default:
        /* Unsupported pixel format */

        return -EINVAL;
    } 

  return OK;
}

static int isx012_try_format(FAR struct v4l2_format *format)
{
  int ret;
  FAR struct v4l2_frmsizeenum support;

  if (format == NULL)
    {
      return -EINVAL;
    }

  /* Get supported frame size information */

  support.index               = 0;
  support.buf_type            = format->type;
  support.pixel_format        = format->fmt.pix.pixelformat;
  support.subimg_pixel_format = format->fmt.pix.subimg_pixelformat;

  ret = isx012_get_range_of_framesize(&support);
  if (ret != OK)
    {
      return ret;
    }

  CHECK_RANGE(format->fmt.pix.width, 
              support.stepwise.min_width,
              support.stepwise.max_width,
              support.stepwise.step_width);

  CHECK_RANGE(format->fmt.pix.height,
              support.stepwise.min_height,
              support.stepwise.max_height,
              support.stepwise.step_height);

  if (support.pixel_format == V4L2_PIX_FMT_JPEG_WITH_SUBIMG)
    {
      CHECK_RANGE(format->fmt.pix.subimg_width,
                  support.subimg.stepwise.min_width,
                  support.subimg.stepwise.max_width,
                  support.subimg.stepwise.step_width);

      CHECK_RANGE(format->fmt.pix.subimg_height,
                  support.subimg.stepwise.min_height,
                  support.subimg.stepwise.max_height,
                  support.subimg.stepwise.step_height);
    }

  return OK;
}

static int isx012_set_format(FAR struct v4l2_format *format)
{
  int                     ret;
  int8_t                  max_fps;
  struct v4l2_frmivalenum frmival;
  isx012_modeparam_t      mode_param;
  FAR isx012_modeparam_t  *current_param;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  ret = isx012_try_format(format);
  if (ret < 0)
    {
      return ret;
    }

  frmival.index               = 0;
  frmival.buf_type            = format->type;
  frmival.pixel_format        = format->fmt.pix.pixelformat;
  frmival.width               = format->fmt.pix.width;
  frmival.height              = format->fmt.pix.height; 
  frmival.subimg_pixel_format = format->fmt.pix.subimg_pixelformat;
  frmival.subimg_width        = format->fmt.pix.subimg_width;
  frmival.subimg_height       = format->fmt.pix.subimg_height; 

  max_fps = isx012_get_maximum_fps(&frmival);
  if (max_fps < 0)
    {
      return max_fps;
    }

  switch (format->type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        current_param = &priv->param.monitor;
        break;
 
      case V4L2_BUF_TYPE_STILL_CAPTURE:
        current_param = &priv->param.capture;
        break;

      default:
        return -EINVAL;
    }

  memcpy(&mode_param, current_param, sizeof(mode_param));

  mode_param.format    = format->fmt.pix.pixelformat;
  mode_param.hsize     = format->fmt.pix.width;
  mode_param.vsize     = format->fmt.pix.height;
  mode_param.int_hsize = format->fmt.pix.subimg_width;
  mode_param.int_vsize = format->fmt.pix.subimg_height;

  if (mode_param.fps < max_fps)
    {
      mode_param.fps = max_fps;
    } 

  ret = isx012_set_mode_param(priv,
                              format->type,
                              &mode_param);
  if (ret != OK)
    {
      return ret;
    }

  memcpy(current_param, &mode_param, sizeof(mode_param));

  return OK;
}

static int isx012_set_supported_frminterval(uint32_t fps_index,
                                            FAR struct v4l2_fract *interval)
{
  switch (fps_index)
    {
      case REGVAL_FPSTYPE_120FPS:
        interval->numerator   = 1;
        interval->denominator = 120;

        break;

      case REGVAL_FPSTYPE_60FPS:
        interval->numerator   = 1;
        interval->denominator = 60;

        break;

      case REGVAL_FPSTYPE_30FPS:
        interval->numerator   = 1;
        interval->denominator = 30;

        break;

      case REGVAL_FPSTYPE_15FPS:
        interval->numerator   = 1;
        interval->denominator = 15;

        break;

      case REGVAL_FPSTYPE_7_5FPS:
        interval->numerator   = 2;
        interval->denominator = 15;

        break;

      case REGVAL_FPSTYPE_6FPS:
        interval->numerator   = 1;
        interval->denominator = 6;

        break;

      case REGVAL_FPSTYPE_5FPS:
        interval->numerator   = 1;
        interval->denominator = 5;

        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int8_t isx012_get_maximum_fps(FAR struct v4l2_frmivalenum *frmival)
{
  int     ret;
  uint8_t max_fps = REGVAL_FPSTYPE_120FPS;

  if (frmival == NULL)
    {
      return -EINVAL;
    }

  ret = isx012_check_fmt(frmival->buf_type, frmival->pixel_format);
  if (ret != OK)
    {
      return ret;
    }

  switch (frmival->pixel_format)
    {
      case V4L2_PIX_FMT_UYVY:                /* YUV 4:2:2 */
        if ((frmival->width  < OUT_YUV_HSIZE_MIN) ||
            (frmival->height < OUT_YUV_VSIZE_MIN) ||
            (frmival->width  > OUT_YUV_15FPS_HSIZE_MAX) ||
            (frmival->height > OUT_YUV_15FPS_VSIZE_MAX))
          {
            /* IN frame size is out of range */

            return -EINVAL;
          }
        else if ((frmival->width  <= OUT_YUV_120FPS_HSIZE_MAX) &&
                 (frmival->height <= OUT_YUV_120FPS_VSIZE_MAX))
          {
            /* support 120FPS, 60FPS, 30FPS, 15FPS, 7.5FPS, 6FPS, and 5FPS */

            max_fps = REGVAL_FPSTYPE_120FPS;
          }
        else
          {
            /* support 60FPS, 30FPS, 15FPS, 7.5FPS, 6FPS, and 5FPS */

            max_fps = REGVAL_FPSTYPE_60FPS;
          }

        break;

      case V4L2_PIX_FMT_JPEG:                /* JPEG */
        if ((frmival->width  < OUT_JPG_HSIZE_MIN) ||
            (frmival->height < OUT_JPG_VSIZE_MIN) ||
            (frmival->width  > OUT_JPG_15FPS_HSIZE_MAX) ||
            (frmival->height > OUT_JPG_15FPS_VSIZE_MAX))
          {
            /* IN frame size is out of range */

            return -EINVAL;
          }
        else if ((frmival->width  <= OUT_JPG_120FPS_HSIZE_MAX) &&
                 (frmival->height <= OUT_JPG_120FPS_VSIZE_MAX))
          {
             /* support 120FPS, 60FPS, 30FPS, 15FPS, 7.5FPS, 6FPS, and 5FPS */

            max_fps = REGVAL_FPSTYPE_120FPS;
          }
        else if ((frmival->width  <= OUT_JPG_60FPS_HSIZE_MAX) &&
                 (frmival->height <= OUT_JPG_60FPS_VSIZE_MAX))
          {
            /* support 60FPS, 30FPS, 15FPS, 7.5FPS, 6FPS, and 5FPS */

            max_fps = REGVAL_FPSTYPE_60FPS;
          }
        else if ((frmival->width  <= OUT_JPG_30FPS_HSIZE_MAX) &&
                 (frmival->height <= OUT_JPG_30FPS_VSIZE_MAX))
          {
            /* support 30FPS, 15FPS, 7.5FPS, 6FPS, and 5FPS */

            max_fps = REGVAL_FPSTYPE_30FPS;
          }
        else
          {
            /* support 15FPS, 7.5FPS, 6FPS, and 5FPS */

            max_fps = REGVAL_FPSTYPE_15FPS;
          }

        break;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:    /* JPEG + YUV 4:2:2 */
        if (frmival->subimg_pixel_format != V4L2_PIX_FMT_UYVY)
          {
            /* Unsupported pixel format */

            return -EINVAL;
          }

        if ((frmival->width         < OUT_JPG_HSIZE_MIN) ||
            (frmival->height        < OUT_JPG_VSIZE_MIN) ||
            (frmival->width         > OUT_JPGINT_15FPS_HSIZE_MAX) ||
            (frmival->height        > OUT_JPGINT_15FPS_VSIZE_MAX) ||
            (frmival->subimg_width  < OUT_YUV_HSIZE_MIN) ||
            (frmival->subimg_height < OUT_YUV_VSIZE_MIN) ||
            (frmival->subimg_width  > OUT_YUVINT_30FPS_HSIZE_MAX) ||
            (frmival->subimg_width  > OUT_YUVINT_30FPS_VSIZE_MAX))
          {
            /* IN frame size is out of range */

            return -EINVAL;
          }
        else if ((frmival->width  <= OUT_JPGINT_30FPS_HSIZE_MAX) &&
                 (frmival->height <= OUT_JPGINT_30FPS_VSIZE_MAX))
          {
            /* support 30FPS, 15FPS, 7.5FPS, 6FPS, 5FPS */

            max_fps = REGVAL_FPSTYPE_30FPS;
          }
        else
          {
            /* support 15FPS, 7.5FPS, 6FPS, 5FPS */

            max_fps = REGVAL_FPSTYPE_15FPS;
          }

        break;

      default:
        return -EINVAL;
    }

  return (int8_t)max_fps;
}

static int isx012_get_range_of_frameinterval
           (FAR struct v4l2_frmivalenum *frmival)
{
  int    ret;
  int8_t max_fps;

  max_fps = isx012_get_maximum_fps(frmival);
  if (max_fps < 0)
    {
      return max_fps;
    }

  frmival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
  ret = isx012_set_supported_frminterval(frmival->index + max_fps,
                                         &frmival->discrete);
  return ret;
}

static int isx012_change_fraction_to_fps(FAR struct v4l2_fract *interval)
{
  if (interval->denominator == interval->numerator * 120)         /* 120FPS */
    {
      return REGVAL_FPSTYPE_120FPS;
    }
  else if(interval->denominator == interval->numerator * 60)      /* 60FPS */
    {
      return REGVAL_FPSTYPE_60FPS;
    }
  else if(interval->denominator == interval->numerator * 30)      /* 30FPS */
    {
      return REGVAL_FPSTYPE_30FPS;
    }
  else if(interval->denominator == interval->numerator * 15)      /* 15FPS */
    {
      return REGVAL_FPSTYPE_15FPS;
    }
  else if(interval->denominator * 10 == interval->numerator * 75) /* 7.5FPS */
    {
      return REGVAL_FPSTYPE_7_5FPS;
    }
  else if(interval->denominator == interval->numerator * 6)       /* 6FPS */
    {
      return REGVAL_FPSTYPE_6FPS;
    }
  else if(interval->denominator == interval->numerator * 5)       /* 5FPS */
    {
      return REGVAL_FPSTYPE_5FPS;
    }
  else
    {
      return -EINVAL;
    }
}

static int isx012_set_frameinterval(FAR struct v4l2_streamparm *parm)
{
  int                     ret;
  int8_t                  fps;
  int8_t                  max_fps;
  uint16_t                fps_regaddr;
  FAR isx012_modeparam_t  *modeparam;
  struct v4l2_frmivalenum frmival;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  fps = isx012_change_fraction_to_fps(&parm->parm.capture.timeperframe);
  if (fps < 0)
    {
      return fps;
    }

  frmival.buf_type = parm->type;
  switch (frmival.buf_type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        modeparam = &priv->param.monitor;
        fps_regaddr = FPSTYPE_MONI;
        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        modeparam = &priv->param.capture;
        fps_regaddr = FPSTYPE_CAP;
        break;

      default:
        return -EINVAL;
    }

  /* Get maximum fps settable value in current image format */

  frmival.pixel_format        = modeparam->format;
  frmival.height              = modeparam->vsize;
  frmival.width               = modeparam->hsize;
  frmival.subimg_pixel_format = V4L2_PIX_FMT_UYVY;
  frmival.subimg_height       = modeparam->int_vsize;
  frmival.subimg_width        = modeparam->int_hsize; 
  max_fps = isx012_get_maximum_fps(&frmival);
  if (max_fps < 0)
    {
      return fps;
    }

  if (fps < max_fps)
    {
      return -EINVAL;
    }

  ret = isx012_putreg(priv, fps_regaddr, fps, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  modeparam->fps = fps;

  return OK;
}

static int isx012_get_range_of_ctrlvalue(FAR struct v4l2_query_ext_ctrl *range)
{
  if (range == NULL)
    {
      return -EINVAL;
    }

  switch (range->ctrl_class)
    {
      case V4L2_CTRL_CLASS_USER:
        switch (range->id)
          {
            case V4L2_CID_BRIGHTNESS:
              range->type          = ISX012_TYPE_BRIGHTNESS;
              range->minimum       = ISX012_MIN_BRIGHTNESS;
              range->maximum       = ISX012_MAX_BRIGHTNESS;
              range->step          = ISX012_STEP_BRIGHTNESS;
              range->default_value = ISX012_DEF_BRIGHTNESS;
              strncpy(range->name,
                      ISX012_NAME_BRIGHTNESS,
                      sizeof(range->name));

              break;

            case V4L2_CID_CONTRAST:
              range->type          = ISX012_TYPE_CONTRAST;
              range->minimum       = ISX012_MIN_CONTRAST;
              range->maximum       = ISX012_MAX_CONTRAST;
              range->step          = ISX012_STEP_CONTRAST;
              range->default_value = ISX012_DEF_CONTRAST;
              strncpy(range->name,
                      ISX012_NAME_CONTRAST,
                      sizeof(range->name));

              break;

            case V4L2_CID_SATURATION:
              range->type          = ISX012_TYPE_SATURATION;
              range->minimum       = ISX012_MIN_SATURATION;
              range->maximum       = ISX012_MAX_SATURATION;
              range->step          = ISX012_STEP_SATURATION;
              range->default_value = ISX012_DEF_SATURATION;
              strncpy(range->name,
                      ISX012_NAME_SATURATION,
                      sizeof(range->name));

              break;

            case V4L2_CID_HUE:
              range->type          = ISX012_TYPE_HUE;
              range->minimum       = ISX012_MIN_HUE;
              range->maximum       = ISX012_MAX_HUE;
              range->step          = ISX012_STEP_HUE;
              range->default_value = ISX012_DEF_HUE;
              strncpy(range->name,
                      ISX012_NAME_HUE,
                      sizeof(range->name));

              break;

            case V4L2_CID_AUTO_WHITE_BALANCE:
              range->type          = ISX012_TYPE_AUTOWB;
              range->minimum       = ISX012_MIN_AUTOWB;
              range->maximum       = ISX012_MAX_AUTOWB;
              range->step          = ISX012_STEP_AUTOWB;
              range->default_value = ISX012_DEF_AUTOWB;
              strncpy(range->name,
                      ISX012_NAME_AUTOWB,
                      sizeof(range->name));

              break;
#if 0 /* To Be Supported */
            case V4L2_CID_RED_BALANCE:
              range->type          = ISX012_TYPE_REDBALANCE;
              range->minimum       = ISX012_MIN_REDBALANCE;
              range->maximum       = ISX012_MAX_REDBALANCE;
              range->step          = ISX012_STEP_REDBALANCE;
              range->default_value = ISX012_DEF_REDBALANCE;
              strncpy(range->name,
                      ISX012_NAME_REDBALANCE,
                      sizeof(range->name));

              break;

            case V4L2_CID_BLUE_BALANCE:
              range->type          = ISX012_TYPE_BLUEBALANCE;
              range->minimum       = ISX012_MIN_BLUEBALANCE;
              range->maximum       = ISX012_MAX_BLUEBALANCE;
              range->step          = ISX012_STEP_BLUEBALANCE;
              range->default_value = ISX012_DEF_BLUEBALANCE;
              strncpy(range->name,
                      ISX012_NAME_BLUEBALANCE,
                      sizeof(range->name));

              break;

            case V4L2_CID_GAMMA_CURVE:
              range->type          = ISX012_TYPE_GAMMACURVE;
              range->minimum       = ISX012_MIN_GAMMACURVE;
              range->maximum       = ISX012_MAX_GAMMACURVE;
              range->step          = ISX012_STEP_GAMMACURVE;
              range->default_value = ISX012_DEF_GAMMACURVE;
              strncpy(range->name,
                      ISX012_NAME_GAMMACURVE,
                      sizeof(range->name));

              break;
#endif
            case V4L2_CID_EXPOSURE:
              range->type          = ISX012_TYPE_EXPOSURE;
              range->minimum       = ISX012_MIN_EXPOSURE;
              range->maximum       = ISX012_MAX_EXPOSURE;
              range->step          = ISX012_STEP_EXPOSURE;
              range->default_value = ISX012_DEF_EXPOSURE;
              strncpy(range->name,
                      ISX012_NAME_EXPOSURE,
                      sizeof(range->name));

              break;

            case V4L2_CID_HFLIP:
              range->type          = ISX012_TYPE_HFLIP;
              range->minimum       = ISX012_MIN_HFLIP;
              range->maximum       = ISX012_MAX_HFLIP;
              range->step          = ISX012_STEP_HFLIP;
              range->default_value = ISX012_DEF_HFLIP;
              strncpy(range->name,
                      ISX012_NAME_HFLIP,
                      sizeof(range->name));

              break;

            case V4L2_CID_VFLIP:
              range->type          = ISX012_TYPE_VFLIP;
              range->minimum       = ISX012_MIN_VFLIP;
              range->maximum       = ISX012_MAX_VFLIP;
              range->step          = ISX012_STEP_VFLIP;
              range->default_value = ISX012_DEF_VFLIP;
              strncpy(range->name,
                      ISX012_NAME_VFLIP,
                      sizeof(range->name));

              break;

            case V4L2_CID_HFLIP_STILL:
              range->type          = ISX012_TYPE_HFLIP_STILL;
              range->minimum       = ISX012_MIN_HFLIP_STILL;
              range->maximum       = ISX012_MAX_HFLIP_STILL;
              range->step          = ISX012_STEP_HFLIP_STILL;
              range->default_value = ISX012_DEF_HFLIP_STILL;
              strncpy(range->name,
                      ISX012_NAME_HFLIP_STILL,
                      sizeof(range->name));

              break;

            case V4L2_CID_VFLIP_STILL:
              range->type          = ISX012_TYPE_VFLIP_STILL;
              range->minimum       = ISX012_MIN_VFLIP_STILL;
              range->maximum       = ISX012_MAX_VFLIP_STILL;
              range->step          = ISX012_STEP_VFLIP_STILL;
              range->default_value = ISX012_DEF_VFLIP_STILL;
              strncpy(range->name,
                      ISX012_NAME_VFLIP_STILL,
                      sizeof(range->name));

              break;

            case V4L2_CID_SHARPNESS:
              range->type          = ISX012_TYPE_SHARPNESS;
              range->minimum       = ISX012_MIN_SHARPNESS;
              range->maximum       = ISX012_MAX_SHARPNESS;
              range->step          = ISX012_STEP_SHARPNESS;
              range->default_value = ISX012_DEF_SHARPNESS;
              strncpy(range->name,
                      ISX012_NAME_SHARPNESS,
                      sizeof(range->name));

              break;

            case V4L2_CID_COLOR_KILLER:
              range->type          = ISX012_TYPE_COLORKILLER;
              range->minimum       = ISX012_MIN_COLORKILLER;
              range->maximum       = ISX012_MAX_COLORKILLER;
              range->step          = ISX012_STEP_COLORKILLER;
              range->default_value = ISX012_DEF_COLORKILLER;
              strncpy(range->name,
                      ISX012_NAME_COLORKILLER,
                      sizeof(range->name));

              break;

            case V4L2_CID_COLORFX:
              range->type          = ISX012_TYPE_COLOREFFECT;
              range->minimum       = ISX012_MIN_COLOREFFECT;
              range->maximum       = ISX012_MAX_COLOREFFECT;
              range->step          = ISX012_STEP_COLOREFFECT;
              range->default_value = ISX012_DEF_COLOREFFECT;
              strncpy(range->name,
                      ISX012_NAME_COLOREFFECT,
                      sizeof(range->name));

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      case V4L2_CTRL_CLASS_CAMERA:
        switch (range->id)
          {
#if 0 /* To Be Supported */
            case V4L2_CID_EXPOSURE_AUTO:
              range->type          = ISX012_TYPE_EXPOSUREAUTO;
              range->minimum       = ISX012_MIN_EXPOSUREAUTO;
              range->maximum       = ISX012_MAX_EXPOSUREAUTO;
              range->step          = ISX012_STEP_EXPOSUREAUTO;
              range->default_value = ISX012_DEF_EXPOSUREAUTO;
              strncpy(range->name,
                      ISX012_NAME_EXPOSUREAUTO,
                      sizeof(range->name));

              break;

            case V4L2_CID_EXPOSURE_ABSOLUTE:
              range->type          = ISX012_TYPE_EXPOSURETIME;
              range->minimum       = ISX012_MIN_EXPOSURETIME;
              range->maximum       = ISX012_MAX_EXPOSURETIME;
              range->step          = ISX012_STEP_EXPOSURETIME;
              range->default_value = ISX012_DEF_EXPOSURETIME;
              strncpy(range->name,
                      ISX012_NAME_EXPOSURETIME,
                      sizeof(range->name));

              break;
#endif
            case V4L2_CID_EXPOSURE_METERING:
              range->type          = ISX012_TYPE_PHOTOMETRY;
              range->minimum       = ISX012_MIN_PHOTOMETRY;
              range->maximum       = ISX012_MAX_PHOTOMETRY;
              range->step          = ISX012_STEP_PHOTOMETRY;
              range->default_value = ISX012_DEF_PHOTOMETRY;
              strncpy(range->name,
                      ISX012_NAME_PHOTOMETRY,
                      sizeof(range->name));

              break;

            case V4L2_CID_ZOOM_ABSOLUTE:
              range->type          = ISX012_TYPE_ZOOM;
              range->minimum       = ISX012_MIN_ZOOM;
              range->maximum       = ISX012_MAX_ZOOM;
              range->step          = ISX012_STEP_ZOOM;
              range->default_value = ISX012_DEF_ZOOM;
              strncpy(range->name,
                      ISX012_NAME_ZOOM,
                      sizeof(range->name));

              break;

            case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
              range->type          = ISX012_TYPE_PRESETWB;
              range->minimum       = ISX012_MIN_PRESETWB;
              range->maximum       = ISX012_MAX_PRESETWB;
              range->step          = ISX012_STEP_PRESETWB;
              range->default_value = ISX012_DEF_PRESETWB;
              strncpy(range->name,
                      ISX012_NAME_PRESETWB,
                      sizeof(range->name));

              break;

            case V4L2_CID_WIDE_DYNAMIC_RANGE:
              range->type          = ISX012_TYPE_YGAMMA;
              range->minimum       = ISX012_MIN_YGAMMA;
              range->maximum       = ISX012_MAX_YGAMMA;
              range->step          = ISX012_STEP_YGAMMA;
              range->default_value = ISX012_DEF_YGAMMA;
              strncpy(range->name,
                      ISX012_NAME_YGAMMA,
                      sizeof(range->name));

              break;

            case V4L2_CID_ISO_SENSITIVITY:
              range->type          = ISX012_TYPE_ISO;
              range->minimum       = ISX012_MIN_ISO;
              range->maximum       = ISX012_MAX_ISO;
              range->step          = ISX012_STEP_ISO;
              range->default_value = ISX012_DEF_ISO;
              strncpy(range->name,
                      ISX012_NAME_ISO,
                      sizeof(range->name));

              break;

            case V4L2_CID_ISO_SENSITIVITY_AUTO:
              range->type          = ISX012_TYPE_ISOAUTO;
              range->minimum       = ISX012_MIN_ISOAUTO;
              range->maximum       = ISX012_MAX_ISOAUTO;
              range->step          = ISX012_STEP_ISOAUTO;
              range->default_value = ISX012_DEF_ISOAUTO;
              strncpy(range->name,
                      ISX012_NAME_ISOAUTO,
                      sizeof(range->name));

              break;

            case V4L2_CID_3A_LOCK:
              range->type          = ISX012_TYPE_3ALOCK;
              range->minimum       = ISX012_MIN_3ALOCK;
              range->maximum       = ISX012_MAX_3ALOCK;
              range->step          = ISX012_STEP_3ALOCK;
              range->default_value = ISX012_DEF_3ALOCK;
              strncpy(range->name,
                      ISX012_NAME_3ALOCK,
                      sizeof(range->name));

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      case V4L2_CTRL_CLASS_JPEG:
        switch (range->id)
          {
            case V4L2_CID_JPEG_COMPRESSION_QUALITY:
              range->type          = ISX012_TYPE_JPGQUALITY;
              range->minimum       = ISX012_MIN_JPGQUALITY;
              range->maximum       = ISX012_MAX_JPGQUALITY;
              range->step          = ISX012_STEP_JPGQUALITY;
              range->default_value = ISX012_DEF_JPGQUALITY;
              strncpy(range->name,
                      ISX012_NAME_JPGQUALITY,
                      sizeof(range->name));

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      default:
        /* Unsupported control class */

        return -EINVAL;
    }

  return OK;
}

static int isx012_get_menu_of_ctrlvalue(FAR struct v4l2_querymenu *menu)
{
  if (menu == NULL)
    {
      return -EINVAL;
    }

  switch (menu->ctrl_class)
    {
      case V4L2_CTRL_CLASS_USER:
        switch (menu->id)
          {
            case V4L2_CID_COLORFX:
              if (menu->index > ISX012_MAX_COLOREFFECT)
                {
                  return -EINVAL;
                } 

              menu->value = g_isx012_supported_colorfx[menu->index].v4l2;

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      case V4L2_CTRL_CLASS_CAMERA:
        switch (menu->id)
          {
            case V4L2_CID_EXPOSURE_METERING:
              if (menu->index > ISX012_MAX_PHOTOMETRY)
                {
                  return -EINVAL;
                }

              menu->value = g_isx012_supported_photometry[menu->index].v4l2;

              break;

            case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
              if (menu->index > ISX012_MAX_PRESETWB)
                {
                  return -EINVAL;
                }

              menu->value = g_isx012_supported_presetwb[menu->index].v4l2;

              break;

            case V4L2_CID_ISO_SENSITIVITY:
              if (menu->index > ISX012_MAX_ISO)
                {
                  return -EINVAL;
                }

              menu->value = g_isx012_supported_iso[menu->index].v4l2;

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      default:
        /* Unsupported control class */

        return -EINVAL;
    }

  return OK;
}

static int isx012_get_ctrlvalue(uint16_t ctrl_class,
                                FAR struct v4l2_ext_control *control)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  int16_t    readvalue;
  uint8_t    cnt;
  int        ret = -EINVAL;

  if (control == NULL)
    {
      return -EINVAL;
    }

  switch (ctrl_class)
    {
      case V4L2_CTRL_CLASS_USER:
        switch (control->id)
          {
            case V4L2_CID_BRIGHTNESS:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_BRIGHTNESS,
                                             ISX012_SIZE_BRIGHTNESS);
              break;

            case V4L2_CID_CONTRAST:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_CONTRAST,
                                             ISX012_SIZE_CONTRAST);
              break;

            case V4L2_CID_SATURATION:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_SATURATION,
                                             ISX012_SIZE_SATURATION);
              break;

            case V4L2_CID_HUE:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_HUE,
                                             ISX012_SIZE_HUE);
              break;

            case V4L2_CID_AUTO_WHITE_BALANCE:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_AUTOWB,
                                        ISX012_SIZE_AUTOWB);

              control->value = (~readvalue) & REGVAL_CPUEXT_BIT_AWBSTOP;

              break;
#if 0 /* To Be Supported */
            case V4L2_CID_RED_BALANCE:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_REDBALANCE,
                                             ISX012_SIZE_REDBALANCE);
              break;

            case V4L2_CID_BLUE_BALANCE:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_BLUEBALANCE,
                                             ISX012_SIZE_BLUEBALANCE);
              break;
#endif
            case V4L2_CID_EXPOSURE:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_EXPOSURE,
                                             ISX012_SIZE_EXPOSURE);
              break;

            case V4L2_CID_HFLIP:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_HFLIP,
                                        ISX012_SIZE_HFLIP);

              if (readvalue & REGVAL_READVECT_BIT_H)
                {
                  control->value = true;
                }
              else
                {
                  control->value = false;
                }

              break;

            case V4L2_CID_VFLIP:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_VFLIP,
                                        ISX012_SIZE_VFLIP);

              if (readvalue & REGVAL_READVECT_BIT_V)
                {
                  control->value = true;
                }
              else
                {
                  control->value = false;
                }

              break;

            case V4L2_CID_HFLIP_STILL:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_HFLIP_STILL,
                                        ISX012_SIZE_HFLIP_STILL);

              if (readvalue & REGVAL_READVECT_BIT_H)
                {
                  control->value = true;
                }
              else
                {
                  control->value = false;
                }

              break;

            case V4L2_CID_VFLIP_STILL:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_VFLIP_STILL,
                                        ISX012_SIZE_VFLIP_STILL);

              if (readvalue & REGVAL_READVECT_BIT_V)
                {
                  control->value = true;
                }
              else
                {
                  control->value = false;
                }

              break;

            case V4L2_CID_COLOR_KILLER:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_COLORKILLER,
                                        ISX012_SIZE_COLORKILLER);

              if (readvalue == REGVAL_EFFECT_MONOTONE)
                {
                  control->value = true;
                }
              else
                {
                  control->value = false;
                }

              break;

            case V4L2_CID_COLORFX:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_COLOREFFECT,
                                        ISX012_SIZE_COLOREFFECT);

              for (cnt = 0; cnt <= ISX012_MAX_COLOREFFECT; cnt++)
                {
                  if (g_isx012_supported_colorfx[cnt].regval == readvalue)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              control->value = g_isx012_supported_colorfx[cnt].v4l2; 

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      case V4L2_CTRL_CLASS_CAMERA:
        switch (control->id)
          {
#if 0 /* To Be Supported */
            case V4L2_CID_EXPOSURE_AUTO:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_EXPOSUREAUTO,
                                        ISX012_SIZE_EXPOSUREAUTO);

              if (readvalue)
                {
                  control->value = V4L2_EXPOSURE_MANUAL;
                }
              else
                {
                  control->value = V4L2_EXPOSURE_AUTO;
                }

              break;

            case V4L2_CID_EXPOSURE_ABSOLUTE:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_EXPOSURETIME,
                                             ISX012_SIZE_EXPOSURETIME);

              break;
#endif
            case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_PRESETWB,
                                        ISX012_SIZE_PRESETWB);

              for (cnt = 0; cnt <= ISX012_MAX_PRESETWB; cnt++)
                {
                  if (g_isx012_supported_presetwb[cnt].regval == readvalue)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              control->value = g_isx012_supported_presetwb[cnt].v4l2;

              break;

            case V4L2_CID_WIDE_DYNAMIC_RANGE:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_YGAMMA,
                                        ISX012_SIZE_YGAMMA);
              if (readvalue)
                {
                  control->value = false;
                }
              else
                {
                  control->value = true;
                }

              break;

            case V4L2_CID_ISO_SENSITIVITY:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_ISO,
                                        ISX012_SIZE_ISO);

              for (cnt = 0; cnt <= ISX012_MAX_ISO; cnt++)
                {
                  if (g_isx012_supported_iso[cnt].regval == readvalue)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              control->value = g_isx012_supported_iso[cnt].v4l2;

              break;
 
            case V4L2_CID_ISO_SENSITIVITY_AUTO:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_ISOAUTO,
                                        ISX012_SIZE_ISOAUTO);
              if (readvalue == REGVAL_ISO_AUTO)
                {
                  control->value = V4L2_ISO_SENSITIVITY_AUTO;
                }
              else
                {
                  control->value = V4L2_ISO_SENSITIVITY_MANUAL;
                }
              break;

            case V4L2_CID_EXPOSURE_METERING:
              readvalue = isx012_getreg(priv,
                                        ISX012_REG_PHOTOMETRY,
                                        ISX012_SIZE_PHOTOMETRY);

              for (cnt = 0; cnt <= ISX012_MAX_PHOTOMETRY; cnt++)
                {
                  if (g_isx012_supported_photometry[cnt].regval == readvalue)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              control->value = g_isx012_supported_photometry[cnt].v4l2;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;


      case V4L2_CTRL_CLASS_JPEG:
        switch (control->id)
          {
            case V4L2_CID_JPEG_COMPRESSION_QUALITY:
              control->value = isx012_getreg(priv,
                                             ISX012_REG_JPGQUALITY,
                                             ISX012_SIZE_JPGQUALITY);
              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      default:
        /* Unsupported control class */

        return -EINVAL;
    }

  return OK;
}

static int isx012_set_ctrlvalue(uint16_t ctrl_class,
                                FAR struct v4l2_ext_control *control)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  int       ret = -EINVAL;
  uint8_t   cnt;
#if 0 /* Used in temporarily unsupported function(V4L2_CID_GAMMA_CURVE) */
  uint8_t   *write_src;
  uint16_t  write_dst;
#endif
  uint16_t  regval;

  if (control == NULL)
    {
      return -EINVAL;
    }

  switch (ctrl_class)
    {
      case V4L2_CTRL_CLASS_USER:
        switch (control->id)
          {
            case V4L2_CID_BRIGHTNESS:
              CHECK_RANGE(control->value,
                          ISX012_MIN_BRIGHTNESS,
                          ISX012_MAX_BRIGHTNESS,
                          ISX012_STEP_BRIGHTNESS);

              ret = isx012_putreg(priv,
                                  ISX012_REG_BRIGHTNESS,
                                  control->value,
                                  ISX012_SIZE_BRIGHTNESS);

              break;

            case V4L2_CID_CONTRAST:
              CHECK_RANGE(control->value,
                          ISX012_MIN_CONTRAST,
                          ISX012_MAX_CONTRAST,
                          ISX012_STEP_CONTRAST);

              ret = isx012_putreg(priv,
                                  ISX012_REG_CONTRAST,
                                  control->value,
                                  ISX012_SIZE_CONTRAST);

              break;

            case V4L2_CID_SATURATION:
              CHECK_RANGE(control->value,
                          ISX012_MIN_SATURATION,
                          ISX012_MAX_SATURATION,
                          ISX012_STEP_SATURATION);

              ret = isx012_putreg(priv,
                                  ISX012_REG_SATURATION,
                                  control->value,
                                  ISX012_SIZE_SATURATION);

              break;

            case V4L2_CID_HUE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_HUE,
                          ISX012_MAX_HUE,
                          ISX012_STEP_HUE);

              ret = isx012_putreg(priv,
                                  ISX012_REG_HUE,
                                  control->value,
                                  ISX012_SIZE_HUE);

              break;

            case V4L2_CID_AUTO_WHITE_BALANCE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_AUTOWB,
                          ISX012_MAX_AUTOWB,
                          ISX012_STEP_AUTOWB);

              regval = isx012_getreg(priv,
                                     ISX012_REG_AUTOWB,
                                     ISX012_SIZE_AUTOWB);

              if (control->value)
                {
                  /* Because true means setting auto white balance
                   * turn off the stop bit
                   */

                  regval &= ~REGVAL_CPUEXT_BIT_AWBSTOP;
                }
              else
                {
                  /* Because false means stopping auto white balance,
                   * turn on the stop bit.
                   */

                  regval |= REGVAL_CPUEXT_BIT_AWBSTOP;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_AUTOWB,
                                  regval,
                                  ISX012_SIZE_AUTOWB);

              break;
#if 0 /* To Be Supported */
            case V4L2_CID_RED_BALANCE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_REDBALANCE,
                          ISX012_MAX_REDBALANCE,
                          ISX012_STEP_REDBALANCE);

              ret = isx012_putreg(priv,
                                  ISX012_REG_REDBALANCE,
                                  control->value,
                                  ISX012_SIZE_REDBALANCE);

              break;

            case V4L2_CID_BLUE_BALANCE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_BLUEBALANCE,
                          ISX012_MAX_BLUEBALANCE,
                          ISX012_STEP_BLUEBALANCE);

              ret = isx012_putreg(priv,
                                  ISX012_REG_BLUEBALANCE,
                                  control->value,
                                  ISX012_SIZE_BLUEBALANCE);

              break;

            case V4L2_CID_GAMMA_CURVE:
              if (control->p_u8 == NULL)
                {
                  return -EINVAL;
                }

              write_src = control->p_u8;
              write_dst = ISX012_REG_GAMMACURVE;
 
              for (cnt = 0; cnt < ISX012_ELEMS_GAMMACURVE; cnt++)
                {
                  CHECK_RANGE(*write_src,
                              ISX012_MIN_GAMMACURVE,
                              ISX012_MAX_GAMMACURVE,
                              ISX012_STEP_GAMMACURVE);

                  ret = isx012_putreg(priv,
                                      write_dst,
                                      *write_src,
                                      ISX012_SIZE_GAMMACURVE);
                }

              break;
#endif
            case V4L2_CID_EXPOSURE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_EXPOSURE,
                          ISX012_MAX_EXPOSURE,
                          ISX012_STEP_EXPOSURE);

              ret = isx012_putreg(priv,
                                  ISX012_REG_EXPOSURE,
                                  control->value,
                                  ISX012_SIZE_EXPOSURE);

              break;

            case V4L2_CID_HFLIP:
              CHECK_RANGE(control->value,
                          ISX012_MIN_HFLIP,
                          ISX012_MAX_HFLIP,
                          ISX012_STEP_HFLIP);

              regval = isx012_getreg(priv,
                                     ISX012_REG_HFLIP,
                                     ISX012_SIZE_HFLIP);

              if (control->value)
                {
                  regval |= REGVAL_READVECT_BIT_H;
                }
              else
                {
                  regval &= ~REGVAL_READVECT_BIT_H;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_HFLIP,
                                  regval,
                                  ISX012_SIZE_HFLIP);

              break;

            case V4L2_CID_VFLIP:
              CHECK_RANGE(control->value,
                          ISX012_MIN_VFLIP,
                          ISX012_MAX_VFLIP,
                          ISX012_STEP_VFLIP);

              regval = isx012_getreg(priv,
                                     ISX012_REG_VFLIP,
                                     ISX012_SIZE_VFLIP);

              if (control->value)
                {
                  regval |= REGVAL_READVECT_BIT_V;
                }
              else
                {
                  regval &= ~REGVAL_READVECT_BIT_V;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_VFLIP,
                                  regval,
                                  ISX012_SIZE_VFLIP);

              break;

            case V4L2_CID_HFLIP_STILL:
              CHECK_RANGE(control->value,
                          ISX012_MIN_HFLIP_STILL,
                          ISX012_MAX_HFLIP_STILL,
                          ISX012_STEP_HFLIP_STILL);

              regval = isx012_getreg(priv,
                                     ISX012_REG_HFLIP_STILL,
                                     ISX012_SIZE_HFLIP_STILL);

              if (control->value)
                {
                  regval |= REGVAL_READVECT_BIT_H;
                }
              else
                {
                  regval &= ~REGVAL_READVECT_BIT_H;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_HFLIP_STILL,
                                  regval,
                                  ISX012_SIZE_HFLIP_STILL);

              break;

            case V4L2_CID_VFLIP_STILL:
              CHECK_RANGE(control->value,
                          ISX012_MIN_VFLIP_STILL,
                          ISX012_MAX_VFLIP_STILL,
                          ISX012_STEP_VFLIP_STILL);

              regval = isx012_getreg(priv,
                                     ISX012_REG_VFLIP_STILL,
                                     ISX012_SIZE_VFLIP_STILL);

              if (control->value)
                {
                  regval |= REGVAL_READVECT_BIT_V;
                }
              else
                {
                  regval &= ~REGVAL_READVECT_BIT_V;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_VFLIP_STILL,
                                  regval,
                                  ISX012_SIZE_VFLIP_STILL);

              break;

            case V4L2_CID_SHARPNESS:
              CHECK_RANGE(control->value,
                          ISX012_MIN_SHARPNESS,
                          ISX012_MAX_SHARPNESS,
                          ISX012_STEP_SHARPNESS);

              ret = isx012_putreg(priv,
                                  ISX012_REG_SHARPNESS,
                                  control->value,
                                  ISX012_SIZE_SHARPNESS);

              break;

            case V4L2_CID_COLOR_KILLER:
              CHECK_RANGE(control->value,
                          ISX012_MIN_COLORKILLER,
                          ISX012_MAX_COLORKILLER,
                          ISX012_STEP_COLORKILLER);

              if (control->value)
                {
                  regval = REGVAL_EFFECT_MONOTONE;
                }
              else
                {
                  regval = REGVAL_EFFECT_NONE;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_COLORKILLER,
                                  regval,
                                  ISX012_SIZE_COLORKILLER);

              break;

            case V4L2_CID_COLORFX:
              for (cnt = 0; cnt <= ISX012_MAX_COLOREFFECT; cnt++)
                {
                  if (g_isx012_supported_colorfx[cnt].v4l2 == control->value)
                    {
                      ret = OK;
                      break;
                    } 
                }

              if (ret != OK)
                {
                  return ret;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_COLOREFFECT,
                                  g_isx012_supported_colorfx[cnt].regval,
                                  ISX012_SIZE_COLOREFFECT);

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      case V4L2_CTRL_CLASS_CAMERA:
        switch (control->id)
          {
#if 0 /* To Be Supported */
            case V4L2_CID_EXPOSURE_AUTO:
              break;

            case V4L2_CID_EXPOSURE_ABSOLUTE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_EXPOSURETIME,
                          ISX012_MAX_EXPOSURETIME,
                          ISX012_STEP_EXPOSURETIME);

              ret = isx012_putreg(priv,
                                  ISX012_REG_EXPOSURETIME,
                                  control->value,
                                  ISX012_SIZE_EXPOSURETIME);
              break;
#endif
            case V4L2_CID_WIDE_DYNAMIC_RANGE:
              CHECK_RANGE(control->value,
                          ISX012_MIN_YGAMMA,
                          ISX012_MAX_YGAMMA,
                          ISX012_STEP_YGAMMA);

              if (control->value)
                {
                  regval = REGVAL_YGAMMA_AUTO;
                }
              else
                {
                  regval = REGVAL_YGAMMA_OFF;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_YGAMMA,
                                  regval,
                                  ISX012_SIZE_YGAMMA);

              break;

            case V4L2_CID_ISO_SENSITIVITY:
              for (cnt = 0; cnt <= ISX012_MAX_ISO; cnt++)
                {
                  if (g_isx012_supported_iso[cnt].v4l2
                       == control->value)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_ISO,
                                  g_isx012_supported_iso[cnt].regval,
                                  ISX012_SIZE_ISO);

              break;

            case V4L2_CID_ISO_SENSITIVITY_AUTO:
              CHECK_RANGE(control->value,
                          ISX012_MIN_ISOAUTO,
                          ISX012_MAX_ISOAUTO,
                          ISX012_STEP_ISOAUTO);

              if (control->value == V4L2_ISO_SENSITIVITY_AUTO)
                {
                  ret = isx012_putreg(priv,
                                      ISX012_REG_ISOAUTO,
                                      REGVAL_ISO_AUTO,
                                      ISX012_SIZE_ISOAUTO);
                }
              else
                {
                  /* In manual case, read auto adjust value and set it */

                  regval = isx012_getreg(priv,
                                         ISX012_REG_ISOAUTOVALUE,
                                         ISX012_SIZE_ISOAUTOVALUE);
                  ret = isx012_putreg(priv,
                                      ISX012_REG_ISO,
                                      regval,
                                      ISX012_SIZE_ISO);
                }

              break;

            case V4L2_CID_EXPOSURE_METERING:
              for (cnt = 0; cnt <= ISX012_MAX_PHOTOMETRY; cnt++)
                {
                  if (g_isx012_supported_photometry[cnt].v4l2
                       == control->value)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_PHOTOMETRY,
                                  g_isx012_supported_photometry[cnt].regval,
                                  ISX012_SIZE_PHOTOMETRY);

              break;

            case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
              for (cnt = 0; cnt <= ISX012_MAX_PRESETWB; cnt++)
                {
                  if (g_isx012_supported_presetwb[cnt].v4l2 == control->value)
                    {
                      ret = OK;
                      break;
                    }
                }

              if (ret != OK)
                {
                  return ret;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_PRESETWB,
                                  g_isx012_supported_presetwb[cnt].regval,
                                  ISX012_SIZE_PRESETWB);

              break;

            case V4L2_CID_3A_LOCK:
              CHECK_RANGE(control->value,
                          ISX012_MIN_3ALOCK,
                          ISX012_MAX_3ALOCK,
                          ISX012_STEP_3ALOCK);

              regval = 0;

              if ((control->value & V4L2_LOCK_EXPOSURE)
                    == V4L2_LOCK_EXPOSURE)
                {
                  regval |= REGVAL_CPUEXT_BIT_AESTOP;
                }

              if ((control->value & V4L2_LOCK_WHITE_BALANCE)
                    == V4L2_LOCK_WHITE_BALANCE)
                {
                  regval |= REGVAL_CPUEXT_BIT_AWBSTOP;
                }

              ret = isx012_putreg(priv,
                                  ISX012_REG_3ALOCK,
                                  regval,
                                  ISX012_SIZE_3ALOCK);

              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      case V4L2_CTRL_CLASS_JPEG:
        switch (control->id)
          {
            case V4L2_CID_JPEG_COMPRESSION_QUALITY:
              CHECK_RANGE(control->value,
                          ISX012_MIN_JPGQUALITY,
                          ISX012_MAX_JPGQUALITY,
                          ISX012_STEP_JPGQUALITY);

              ret = isx012_putreg(priv,
                                  ISX012_REG_JPGQUALITY,
                                  control->value,
                                  ISX012_SIZE_JPGQUALITY);
              break;

            default:
              /* Unsupported control id */

              return -EINVAL;
          }

        break;

      default:
        /* Unsupported control class */

        return -EINVAL;
    }

  return ret;
}

static int isx012_refresh(void)
{
  int ret = 0;
  uint8_t mask_num;
  int i;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  if (priv->state != STATE_ISX012_ACTIVE)
    {
      /* In inactive state, setting is reflected in activated timing */

      return OK;
    }

  if (priv->mode != REGVAL_MODESEL_MON)
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int isx012_register(FAR struct i2c_master_s *i2c)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  /* Save i2c information */

  priv->i2c        = i2c;
  priv->i2c_addr   = ISX012_I2C_SLV_ADDR;
  priv->i2c_freq   = I2CFREQ_STANDARD;

  /* Initialize other information */

  priv->state      = STATE_ISX012_POWEROFF;

  return OK;
}

int isx012_unregister(void)
{
  /* no procedure */

  return OK;
}

FAR struct video_devops_s *isx012_initialize(void)
{
  /* return address of video operations variable */

  return &g_isx012_video_devops;
}

int isx012_uninitialize(void)
{
  /* No procedure */

  return OK;
}


