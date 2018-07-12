/****************************************************************************
 * bsp/include/nuttx/video/isx012.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_ISX012_H
#define __INCLUDE_NUTTX_VIDEO_ISX012_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _IMGIOCBASE   (0x1100)

#define _IMGIOC(nr)       _IOC(_IMGIOCBASE,nr)

/**
 * @defgroup img_ioctl IOCTL commands
 * @{
 */

#define IMGIOC_SETSTATE    _IMGIOC(0x0001)
#define IMGIOC_SETMODE     _IMGIOC(0x0002)
#define IMGIOC_SETMODEP    _IMGIOC(0x0003)
#define IMGIOC_SETCISIF    _IMGIOC(0x0004)
#define IMGIOC_CHGCOLOR    _IMGIOC(0x0005)
#define IMGIOC_CHGISO      _IMGIOC(0x0006)
#define IMGIOC_CHGSHUTTER  _IMGIOC(0x0007)
#define IMGIOC_CHGEV       _IMGIOC(0x0008)
#define IMGIOC_CHGBRIGHT   _IMGIOC(0x0009)
#define IMGIOC_CHGCONTRAST _IMGIOC(0x000A)
#define IMGIOC_CHGJQUALITY _IMGIOC(0x000B)
#define IMGIOC_CHGYGAMMA   _IMGIOC(0x000C)
#define IMGIOC_CHGAWB      _IMGIOC(0x000D)
#define IMGIOC_CHGPMETRY   _IMGIOC(0x000E)
#define IMGIOC_CHGCROP     _IMGIOC(0x000F)
#define IMGIOC_GETISO      _IMGIOC(0x0010)
#define IMGIOC_GETSHTL     _IMGIOC(0x0011)
#define IMGIOC_GETSHTH     _IMGIOC(0x0012)
#define IMGIOC_READREG     _IMGIOC(0x0013)
#define IMGIOC_WRITEREG    _IMGIOC(0x0014)
#define IMGIOC_MONIREF     _IMGIOC(0x0015)

#define EZOOM_ISX012_OFFSET_PX   (16)

enum isx012_state_e {
  STATE_ISX012_PRESLEEP,
  STATE_ISX012_SLEEP,
  STATE_ISX012_ACTIVE,
  STATE_ISX012_POWEROFF,
};

typedef enum isx012_state_e isx012_state_t;

enum isx012_format_e {
  FORMAT_ISX012_YUV = 0,
  FORMAT_ISX012_RGB565,
  FORMAT_ISX012_JPEG_MODE1,
  FORMAT_ISX012_JPEG_MODE1_INT,
  FORMAT_ISX012_MAX
};

typedef enum isx012_format_e isx012_format_t;

enum isx012_resolution_e
{
  RESOLUTION_ISX012_QVGA = 0,
  RESOLUTION_ISX012_VGA,
  RESOLUTION_ISX012_QUADVGA,
  RESOLUTION_ISX012_HD,
  RESOLUTION_ISX012_FULLHD,
  RESOLUTION_ISX012_3M,
  RESOLUTION_ISX012_5M,
  RESOLUTION_ISX012_MAX
};

typedef enum isx012_resolution_e isx012_resolution_t;

enum isx012_rate_e {
  RATE_ISX012_120FPS = 0,
  RATE_ISX012_60FPS,
  RATE_ISX012_30FPS,
  RATE_ISX012_15FPS,
  RATE_ISX012_10FPS,
  RATE_ISX012_7_5FPS,
  RATE_ISX012_6FPS,
  RATE_ISX012_5FPS,
  RATE_ISX012_MAX,
};

typedef enum isx012_rate_e isx012_rate_t;

enum isx012_set_param_id_e
{
  PARAM_ISX012_ID_COLOR = 0,
  PARAM_ISX012_ID_ISO,
  PARAM_ISX012_ID_SHUTTER,
  PARAM_ISX012_ID_EV_CORRECTION,
  PARAM_ISX012_ID_BRIGHTNESS,
  PARAM_ISX012_ID_CONTRAST,
  PARAM_ISX012_ID_JPEG_QUALITY,
  PARAM_ISX012_ID_YGAMMA,
  PARAM_ISX012_ID_AWB,
  PARAM_ISX012_ID_PHOTOMETRY,
  PARAM_ISX012_ID_MAX
};

enum isx012_get_param_id_e
{
  PARAM_ISX012_ID_RES_ISO = 0,
  PARAM_ISX012_ID_RES_SHT_L,
  PARAM_ISX012_ID_RES_SHT_H,
  PARAM_ISX012_ID_RES_MAX
};

typedef enum isx012_param_id_e isx012_param_id_t;

enum isx012_color_mode_e
{
  COLOR_ISX012_NORMAL = 0,
  COLOR_ISX012_SOLARIZATION,
  COLOR_ISX012_NEGATIVE_POSITIVE,
  COLOR_ISX012_SEPIA,
  COLOR_ISX012_MONO,
  COLOR_ISX012_PASTEL,
  COLOR_ISX012_SKETCH,
  COLOR_ISX012_MAX
};

typedef enum isx012_color_mode_e isx012_color_mode_t;

enum isx012_iso_e
{
  ISX012_ISO_NONE = 0,
  ISX012_ISO25,
  ISX012_ISO32,
  ISX012_ISO40,
  ISX012_ISO50,
  ISX012_ISO64,
  ISX012_ISO80,
  ISX012_ISO100,
  ISX012_ISO125,
  ISX012_ISO160,
  ISX012_ISO200,
  ISX012_ISO250,
  ISX012_ISO320,
  ISX012_ISO400,
  ISX012_ISO500,
  ISX012_ISO640,
  ISX012_ISO800,
  ISX012_ISO1000,
  ISX012_ISO1250,
  ISX012_ISO1600,
  ISX012_ISO_MAX
};

typedef enum isx012_iso_e isx012_iso_t;

enum isx012_ev_e
{
  EV_ISX012_M2 = -6,
  EV_ISX012_M5_3,
  EV_ISX012_M4_3,
  EV_ISX012_M1,
  EV_ISX012_M2_3,
  EV_ISX012_M1_3,
  EV_ISX012_OFF,
  EV_ISX012_P1_3,
  EV_ISX012_P2_3,
  EV_ISX012_P1,
  EV_ISX012_P4_3,
  EV_ISX012_P5_3,
  EV_ISX012_P2,
  EV_ISX012_MAX
};

typedef enum isx012_ev_e isx012_ev_t;

enum isx012_ygamma_e
{
  YGAMMA_ISX012_AUTO = 0,
  YGAMMA_ISX012_OFF,
  YGAMMA_ISX012_MAX
};

typedef enum isx012_ygamma_e isx012_ygamma_t;

enum isx012_awb_e
{
  AWB_ISX012_ATM = 0,
  AWB_ISX012_CLEARWEATHER,
  AWB_ISX012_SHADE,
  AWB_ISX012_CLOUDYWEATHER,
  AWB_ISX012_FLUORESCENTLIGHT,
  AWB_ISX012_LIGHTBULB,
  AWB_ISX012_MAX
};

typedef enum isx012_awb_e isx012_awb_t;

enum isx012_photometry_e
{
  PHOTOMETRY_ISX012_AVERAGE = 0,
  PHOTOMETRY_ISX012_CENTERWEIGHT,
  PHOTOMETRY_ISX012_SPOT,
  PHOTOMETRY_ISX012_MULTIPATTERN,
  PHOTOMETRY_ISX012_MAX
};

typedef enum isx012_photometry_e isx012_photometry_t;

enum isx012_mode_e
{
  MODE_ISX012_MONITORING,
  MODE_ISX012_CAPTURE,
  MODE_ISX012_HALFRELEASE,
};

typedef enum isx012_mode_e isx012_mode_t;

enum isx012_ezoom_e
{
  EZOOM_ISX012_MAG,
  EZOOM_ISX012_OFFSET_X,
  EZOOM_ISX012_OFFSET_Y,
  EZOOM_ISX012_REGNUM
};

typedef enum isx012_ezoom_e isx012_ezoom_t;

enum isx012_crop_e
{
  CROP_ISX012_DISABLE = 0,
  CROP_ISX012_ENABLE,
  CROP_ISX012_CTRL_MAX
};

typedef enum isx012_crop_e isx012_crop_t;

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct isx012_param_s
{
  isx012_format_t     format;
  uint16_t            jpeg_hsize;
  uint16_t            jpeg_vsize;
  uint16_t            yuv_hsize;
  uint16_t            yuv_vsize;
  isx012_rate_t       rate;
};

typedef struct isx012_param_s isx012_param_t;

struct isx012_param_crop_s
{
  int16_t             x_offset;
  int16_t             y_offset;
  isx012_crop_t       crop;
};

typedef struct isx012_param_crop_s isx012_param_crop_t;

struct isx012_s
{
  isx012_param_t moni_param;
  isx012_param_t cap_param;
};

typedef struct isx012_s isx012_t;

struct isx012_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t  addr;                /* I2C address */
  int      freq;                /* Frequency */
  isx012_t image;               /* image param */
  sem_t wait;
};

typedef struct isx012_dev_s isx012_dev_t;

struct isx012_reg_s {
  uint16_t regaddr;
  uint16_t regval;
  uint8_t  regsize;
};

typedef struct isx012_reg_s isx012_reg_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int isx012_initialize(isx012_dev_t *priv);
int isx012_open( void );
int isx012_close( void );
int isx012_ioctl(int cmd, unsigned long arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_ISX012_H */
