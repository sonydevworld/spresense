/****************************************************************************
 * drivers/video/video.c
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

#include <sys/ioctl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/cisif.h>
#include <arch/board/board.h>
#include <nuttx/video/video.h>
#include <nuttx/video/isx012.h>

#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>

#include <time.h>

#include "video_internal.h"
#include "isx012_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display time ON/OFF */
/* #define VIDEO_TIME_MEASURE */

/* At initialization, it automatically transits to ACTIVE_MODE */
/* #define VIDEO_INIT_ACTIVE */

#define video_printf(format, ...)   _info(format, ##__VA_ARGS__)

/*------------------
 * Message queue
 *----------------*/
#define VIDEO_API_REQ_QUEUE     "video/api_req"
#define VIDEO_API_RESP_QUEUE    "video/api_resp"

#define VIDEO_TRUE              (1)
#define VIDEO_FALSE             (0)

#define VIDEO_EOI_CORRECT_MAX_SIZE  (32)

#define VIDEO_INIT_REGNUM       (5)
#define VIDEO_AE_AUTO_REGNUM    (6)
#define VIDEO_AE_NOW_REGNUM     (6)
#define VIDEO_AWB_AUTO_REGNUM   (3)
#define VIDEO_AWB_NOW_REGNUM    (3)

#define VIDEO_ISX012_HALFREL_TIMEOUT      (2*1000*1000)   /* usec */
#define VIDEO_ISX012_HALFREL_WAITTIME     (20*1000)       /* usec */
#define VIDEO_ISX012_HALF_MOVE_STS_REG    (0x01B0)
#define VIDEO_ISX012_MOVE_AWB_F           (0x01)
#define VIDEO_ISX012_MOVE_AE_F            (0x02)
#define VIDEO_CISIF_TRANSEND_TIMEOUT      (1*1000)        /* msec */

#define VIDEO_V4_BUF_MAX_CNT  (256)

#define VIDEO_DEV_PATH_LEN     (32)

/* Debug option */
#ifdef CONFIG_DEBUG_VIDEO_ERROR
#  define videoerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define videoerr(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_WARN
#  define videowarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define videowarn(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_INFO
#  define videoinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define videoinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum
{
  VIDEO_APIID_CHG_IMGSNS_STATE = 0,
  VIDEO_APIID_CAP_FRAME,
  VIDEO_APIID_SET_CAP_PARAM,
  VIDEO_APIID_SET_IMGSNS_PARAM,
  VIDEO_APIID_SET_IMGSNS_PARAM_ALL,
  VIDEO_APIID_WRITE_IMGSNS_REG,
  VIDEO_APIID_READ_IMGSNS_REG,
  VIDEO_APIID_DO_HALFRELEASE,
  VIDEO_APIID_GET_AUTO_PARAM,
  VIDEO_APIID_CONTI_CAP,
  VIDEO_APIID_MAX,
} video_api_id_e;

struct video_mng_s
{
  int fd;
  pid_t pid_main;
  sem_t sem_cisifsync;
  video_cap_param_t cap_param[VIDEO_MODE_MAX];
  video_img_sns_param_all_t imgsns_param_all;
  video_img_sns_state_e imgsns_state;
  uint8_t init;
  uint8_t devpath[VIDEO_DEV_PATH_LEN];
};

typedef struct video_mng_s video_mng_t;

struct video_cisif_result_s
{
  uint32_t addr;
  uint32_t size;
  uint8_t code;
  uint8_t last_frame;
  uint8_t errint;
};

typedef struct video_cisif_result_s video_cisif_result_t;

struct video_size_s
{
  uint16_t h;
  uint16_t v;
};

typedef struct video_size_s video_size_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void video_callback_cisif(uint8_t code, uint8_t last_frame,
                                 uint32_t size, uint32_t addr);
static void video_init_internal_param(video_mng_t *priv);
static int  video_init_image_sensor(video_mng_t *priv);
static int  video_chg_img_sns_state(video_mng_t *priv,
                                    video_api_chg_img_sns_state_t *p);
static int  video_capture_frame(video_mng_t *priv, video_api_cap_frame_t *p);
static int  video_set_img_sns_param(video_mng_t *priv,
                                    uint32_t id,
                                    uint32_t val);
static int  video_set_frame_info(video_mng_t *priv,
                                 video_api_cap_frame_t *p,
                                 video_cisif_result_t *res);
static int  video_get_picture_info(video_picture_info_t *pict_info);
static int  video_change_img_sns_crop(video_api_cap_frame_t *p);

static uint32_t video_correct_jpeg_size(uint32_t addr, uint32_t size);
static int  video_twaisem(sem_t *sem, uint32_t timeout_ms);

#ifdef VIDEO_TIME_MEASURE
static uint64_t video_get_msec_tim(void)
{
    struct timespec tp;
    if (clock_gettime(CLOCK_REALTIME, &tp)) {
        return 0;
    }
    return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}
#endif /* VIDEO_TIME_MEASURE */

/****************************************************************************
 * Private Data
 ****************************************************************************/
static video_cisif_result_t g_v_cisif;
static video_mng_t         *g_v_mng;

static v4l2_buffer_t *g_v4_buf = NULL;
static uint32_t       g_v4_buf_rcnt = 1;
static uint32_t       g_v4_buf_cnt  = 0;
static uint32_t       g_v4_buf_mode = VIDEO_MODE_MONITORING;

static const video_size_t video_rs2sz[VIDEO_RESOLUTION_MAX] =
{
  { VIDEO_HSIZE_QVGA,     VIDEO_VSIZE_QVGA    },
  { VIDEO_HSIZE_VGA,      VIDEO_VSIZE_VGA     },
  { VIDEO_HSIZE_QUADVGA,  VIDEO_VSIZE_QUADVGA },
  { VIDEO_HSIZE_HD,       VIDEO_VSIZE_HD      },
  { VIDEO_HSIZE_FULLHD,   VIDEO_VSIZE_FULLHD  },
  { VIDEO_HSIZE_3M,       VIDEO_VSIZE_3M      },
  { VIDEO_HSIZE_5M,       VIDEO_VSIZE_5M      }
};

#ifdef VIDEO_TIME_MEASURE
static uint32_t video_time_start;
static uint32_t video_time_stop;
#define DBG_MS_TIME_START()    \
  video_time_start = (uint32_t)video_get_msec_tim()
#define DBG_MS_TIME_STOP(x)     \
  video_time_stop = (uint32_t)video_get_msec_tim(); \
  video_printf("%s: time:%d[ms]\n", \
                x,(uint32_t)(video_time_stop - video_time_start))
#define DBG_TIME_START()      DBG_MS_TIME_START()
#define DBG_TIME_STOP(x)      DBG_MS_TIME_STOP(x)
#else
#define DBG_TIME_START()      
#define DBG_TIME_STOP(x)      
#endif /* VIDEO_TIME_MEASURE */
/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct file_operations g_videofops =
{
  video_open,               /* open */
  video_close,              /* close */
  0,                        /* read */
  0,                        /* write */
  0,                        /* seek */
  video_ioctl,              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                        /* poll */
#endif
  0                         /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int video_twaisem(sem_t *sem, uint32_t timeout_ms)
{
  struct timespec abstime = { 0 };
  unsigned long long tmp;
  int ret;

  clock_gettime(CLOCK_REALTIME, &abstime);
  tmp = abstime.tv_nsec / 1000000;
  tmp += timeout_ms;
  abstime.tv_sec += tmp / 1000;
  abstime.tv_nsec = tmp % 1000 * 1000000 + abstime.tv_nsec % 1000000;

  ret = sem_timedwait(sem, &abstime);
  if (ret != 0)
    {
      video_printf("ERROR: sem_timedwait() error. %d\n", errno);
      ret = ETIMEDOUT;
    }

  return ret;
}

static void video_callback_cisif(
  uint8_t code,
  uint8_t last_frame,
  uint32_t size,
  uint32_t addr)
{

  g_v_cisif.size = size;

  if (code == 0)
    {
      g_v_cisif.addr = addr;
      g_v_cisif.code = code;
      g_v_cisif.last_frame = last_frame;
      sem_post(&g_v_mng->sem_cisifsync);
    }
  else
    {
      if (g_v_cisif.errint == 0)
        {
          g_v_cisif.errint = ENOMEM;
        }
    }
}

static int video_init_image_sensor(video_mng_t *priv)
{
  DBG_TIME_START();
  isx012_open();
  DBG_TIME_STOP("open isx012 driver");

#ifdef VIDEO_INIT_ACTIVE
  /* Cmera Device status Sleep -> Active */
  if (isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_ACTIVE) < 0)
    {
      video_printf("ERROR: Failed to ioctl IMGIOC_SETSTATE. %d\n", ret);
      return -ENODEV;
    }
  /* After Camera mode -> Monitoring */

  priv->imgsns_state = VIDEO_STATE_ACTIVE;
#else
  priv->imgsns_state = VIDEO_STATE_SLEEP;
#endif /* VIDEO_INIT_NOT_ACTIVE */

  return 0;
}

static void video_init_internal_param(video_mng_t *priv)
{
  priv->cap_param[VIDEO_MODE_CAPTURE].format     = VIDEO_FORMAT_JPEG;
  priv->cap_param[VIDEO_MODE_CAPTURE].framerate  = VIDEO_15FPS;
  priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_hsize = VIDEO_HSIZE_FULLHD;
  priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_vsize = VIDEO_VSIZE_FULLHD;

  priv->cap_param[VIDEO_MODE_MONITORING].format     = VIDEO_FORMAT_YUV;
  priv->cap_param[VIDEO_MODE_MONITORING].framerate  = VIDEO_30FPS;
  priv->cap_param[VIDEO_MODE_MONITORING].yuv_hsize  = VIDEO_HSIZE_QVGA;
  priv->cap_param[VIDEO_MODE_MONITORING].yuv_vsize  = VIDEO_VSIZE_QVGA;
}

static int video_chg_img_sns_state(video_mng_t *priv,
                                   video_api_chg_img_sns_state_t *p)
{
  int ret;
  video_img_sns_state_e next_state;

  if (priv->imgsns_state == p->state)
    {
      /* no change state */
      return 0;
    }

  next_state = p->state;
  switch (next_state)
    {
      case VIDEO_STATE_ACTIVE:
        if (priv->imgsns_state != VIDEO_STATE_SLEEP)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_ACTIVE);
        DBG_TIME_STOP("ioctl IMGIOC_SETSTATE(ACTIVE)");
        break;

      case VIDEO_STATE_SLEEP:
        if (priv->imgsns_state != VIDEO_STATE_ACTIVE)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_SLEEP);
        DBG_TIME_STOP("ioctl IMGIOC_SETSTATE(SLEEP)");
        break;

      case VIDEO_STATE_POWOFF:
        DBG_TIME_START();
        ret = isx012_close();
        DBG_TIME_STOP("close -> IMGSNS_POWER_OFF");
        break;

      case VIDEO_STATE_POWON:
        if (priv->imgsns_state != VIDEO_STATE_POWOFF)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = video_init_image_sensor(priv);
        DBG_TIME_STOP("open -> IMGSNS_POWER_ON");
        if (ret == 0)
          {
#ifdef VIDEO_INIT_ACTIVE
            next_state = VIDEO_STATE_ACTIVE;
#else
            next_state = VIDEO_STATE_SLEEP;
#endif /* VIDEO_INIT_ACTIVE */
          }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (ret == 0)
    {
      priv->imgsns_state = next_state;
    }

  return ret;
}

static int video_change_img_sns_crop(video_api_cap_frame_t *p)
{
  int ret;
  isx012_param_crop_t param;

  if (p->crop_ctrl == VIDEO_CROP_ENABLE)
    {
      param.crop = VIDEO_CROP_ENABLE;
      param.x_offset = p->crop.x_offset;
      param.y_offset = p->crop.y_offset;
      ret = isx012_ioctl(IMGIOC_CHGCROP, (unsigned long)&param);
      if (ret < 0)
        {
          return ret;
        }

    }
  else if (p->crop_ctrl == VIDEO_CROP_DISABLE)
    {
      param.crop = VIDEO_CROP_DISABLE;
      ret = isx012_ioctl(IMGIOC_CHGCROP, (unsigned long)&param);
      if (ret < 0)
        {
          return ret;
        }
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

static int video_capture_frame(video_mng_t *priv, video_api_cap_frame_t *p)
{
  int ret = 0;
  cisif_param_t    cis;
  video_img_format_e format;

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
    {
      return -EBUSY;
    }

  video_change_img_sns_crop(p);

  if (p->mode == VIDEO_MODE_CAPTURE)
    {
      video_printf("ioctl IMGIOC_SETMODE(CAPTURE) call.\n");
      DBG_TIME_START();
      ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_CAPTURE);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(CAPTURE)");
      if (ret < 0)
        {
          video_printf("ERROR: ioctl IMGIOC_SETMODE(CAPTURE):%d\n", ret);
          return ret;
        }

    }

  memset(&g_v_cisif, 0, sizeof(g_v_cisif));
  format = priv->cap_param[p->mode].format;
  if (format == VIDEO_FORMAT_YUV)
    {
      cis.yuv_param.comp_func = video_callback_cisif;
    }
  else
    {
      cis.jpg_param.comp_func = video_callback_cisif;
    }

  cis.sarea.strg_addr = (uint8_t *)p->buffer.addr;
  cis.sarea.strg_size = p->buffer.size;
  cis.sarea.capnum    = 1;
  ret = isx012_ioctl(IMGIOC_SETCISIF, (unsigned long)&cis);
  if (ret != OK)
    {
      video_printf("ERROR: cxd56_cisifcaptureframe() %d\n", ret);
      g_v_cisif.errint = ret;
      goto exit;
    }

  ret = video_twaisem(&priv->sem_cisifsync, VIDEO_CISIF_TRANSEND_TIMEOUT);
  if (ret != 0)
    {
      g_v_cisif.errint = ret;
    }

  DBG_TIME_STOP("cxd56_cisifcaptureframe() -> trans end.");

  if ((g_v_cisif.code != 0) || (g_v_cisif.errint != 0))
    {
      video_printf("ERROR :cisif err = %d\n", g_v_cisif.errint);
    }
  else
    {
      video_set_frame_info(priv, p, &g_v_cisif);
      video_get_picture_info(&p->info.pict_info);
    }

exit:
  if (p->mode == VIDEO_MODE_CAPTURE)
    {
      video_printf("ioctl IMGIOC_SETMODE(MONITORING) call.\n");
      DBG_TIME_START();
      ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_MONITORING);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
      if (ret < 0)
        {
          video_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
        }

    }

  if (g_v_cisif.errint != 0)
    {
      ret = -g_v_cisif.errint;
    }

  return ret;
}

static uint32_t video_correct_jpeg_size(uint32_t addr, uint32_t size)
{
  uint8_t *jpg;
  uint32_t eoi_offset;

  jpg = (uint8_t *)(addr + size - 1);
  for (eoi_offset=0; eoi_offset < VIDEO_EOI_CORRECT_MAX_SIZE; eoi_offset++)
    {
      if ((*jpg == 0xD9) && (*(jpg-1) == 0xFF))
        {
          break;
        }
      jpg--;
    }

  return (size - eoi_offset);
}

static int video_set_frame_info(video_mng_t *priv, 
                                video_api_cap_frame_t *p,
                                video_cisif_result_t *res)
{
  p->info.mode = p->mode;
  memcpy(&p->info.cap_param,
         &priv->cap_param[p->mode],
         sizeof(video_cap_param_t));

  p->info.out_addr = res->addr;
  p->info.out_size = res->size;

  if (p->mode == VIDEO_MODE_CAPTURE)
    {
      p->info.h_size = priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_hsize;
      p->info.v_size = priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_vsize;
    }
  else
    {
      p->info.h_size = priv->cap_param[VIDEO_MODE_MONITORING].yuv_hsize;
      p->info.v_size = priv->cap_param[VIDEO_MODE_MONITORING].yuv_vsize;
    }

  if ((priv->cap_param[p->mode].format == VIDEO_FORMAT_JPEG) &&
      (res->errint == 0))
    {
      p->info.out_size = video_correct_jpeg_size(res->addr, res->size);
    }

  return 0;
}

static int video_get_picture_info(video_picture_info_t *pict_info)
{
  int ret;
  uint8_t val8;
  uint16_t val16;
  uint32_t shutter_speed = 0;
  video_iso_e iso_sens = 0;

  /* ISO */

  ret = isx012_ioctl(IMGIOC_GETISO, (unsigned long)&val8);
  if (ret != 0)
    {
      return ret;
    }

  iso_sens = val8;

  /* Shutter low */

  ret = isx012_ioctl(IMGIOC_GETSHTL, (unsigned long)&val16);
  if (ret != 0)
    {
      return ret;
    }

  shutter_speed |= (uint32_t)val16;

  /* Shutter high */

  ret = isx012_ioctl(IMGIOC_GETSHTH, (unsigned long)&val16);
  if (ret != 0)
    {
      return ret;
    }

  shutter_speed |= (uint32_t)((val16 << 16) & 0xFFFF0000);

  pict_info->iso_sens      = iso_sens;
  pict_info->shutter_speed = shutter_speed;

  return 0;
}

static int video_set_img_sns_param(video_mng_t *priv, uint32_t id, uint32_t val)
{
  int ret;
  int api;

  switch (id)
  {
    case VIDEO_PARAM_ID_COLOR:
      api = IMGIOC_CHGCOLOR;
      break;

    case VIDEO_PARAM_ID_ISO:
      api = IMGIOC_CHGISO;
      break;

    case VIDEO_PARAM_ID_SHUTTER:
      api = IMGIOC_CHGSHUTTER;
      break;

    case VIDEO_PARAM_ID_EV_CORRECTION:
      api = IMGIOC_CHGEV;
      break;

    case VIDEO_PARAM_ID_BRIGHTNESS:
      api = IMGIOC_CHGBRIGHT;
      break;

    case VIDEO_PARAM_ID_CONTRAST:
      api = IMGIOC_CHGCONTRAST;
      break;

    case VIDEO_PARAM_ID_JPEG_QUALITY:
      api = IMGIOC_CHGJQUALITY;
      break;

    case VIDEO_PARAM_ID_YGAMMA:
      api = IMGIOC_CHGYGAMMA;
      break;

    case VIDEO_PARAM_ID_AWB:
      api = IMGIOC_CHGAWB;
      break;

    case VIDEO_PARAM_ID_PHOTOMETRY:
      api = IMGIOC_CHGPMETRY;
      break;

    default:
      return -EINVAL;
  }

  ret = isx012_ioctl(api, val);
  if (ret != 0)
    {
      return ret;
    }

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int video_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  video_mng_t           *priv  = inode->i_private;
  int ret = ERROR;

  if (0 != sem_init(&priv->sem_cisifsync, 0, 0))
    {
      video_printf("ERROR: Failed to sem_init(sem_cisifsync).\n");
      return ret;
    }

  video_init_internal_param(priv);
  ret = video_init_image_sensor(priv);
  if (ret != 0)
    {
      return ret;
    }

  ret = video_set_img_sns_param(priv, VIDEO_PARAM_ID_JPEG_QUALITY, 75);
  return ret;
}

int video_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  video_api_chg_img_sns_state_t p;
  int ret = ERROR;

  p.state = VIDEO_STATE_POWOFF;
  ret = video_chg_img_sns_state(priv, &p);
  if (ret != 0)
    {
      return ret;
    }

  if (0 != sem_destroy(&priv->sem_cisifsync))
    {
      video_printf("ERROR: Failed to sem_destroy(sem_cisifsync).\n");
      ret = ERROR;
    }

  if (g_v4_buf)
    {
      free(g_v4_buf);
    }

  return ret;
}

int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  int ret = OK;

  v4l2_format_t                *fmt_lp = (v4l2_format_t *)arg;
  v4l2_requestbuffers_t        *req_lp = (v4l2_requestbuffers_t *)arg;
  v4l2_buffer_t                *buf_lp = (v4l2_buffer_t *)arg;
  enum v4l2_buf_type           *type   = (enum v4l2_buf_type *)arg;
  video_api_cap_frame_t         cap;
  video_api_chg_img_sns_state_t stat_l;
  isx012_t                      isx012_format;

  switch (cmd)
    {
      case VIDIOC_S_FMT:
        if (fmt_lp == NULL || &(fmt_lp->fmt.pix) == NULL)
          {
            ret = -EPERM;
          }
        else
          {
            if (fmt_lp->fmt.pix.field != V4L2_FIELD_ANY)
              {
                ret = -ENOSYS;
              }
            else if ((fmt_lp->fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) &&
                     (fmt_lp->fmt.pix.pixelformat != V4L2_PIX_FMT_JPEG))
              {
                ret = -ENOSYS;
              }
            else
              {
                /* YUV: monitor, JPEG: capture */

                if (fmt_lp->fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
                  {
                    /* Change paramer for monitoring */

                    isx012_format.moni_param.format     = VIDEO_FORMAT_YUV;
                    isx012_format.moni_param.rate       = VIDEO_30FPS;
                    isx012_format.moni_param.yuv_hsize  = fmt_lp->fmt.pix.width;
                    isx012_format.moni_param.yuv_vsize  = fmt_lp->fmt.pix.height;

                    /* Maintain current setting for capture */

                    isx012_format.cap_param.format      = VIDEO_FORMAT_JPEG;
                    isx012_format.cap_param.rate        = VIDEO_15FPS;
                    isx012_format.cap_param.jpeg_hsize  = priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_hsize;
                    isx012_format.cap_param.jpeg_vsize  = priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_vsize;
                  }
                else  /* VIDEO_MODE_CAPTURE */
                  {
                    /* Change paramer for capture */

                    isx012_format.cap_param.format      = VIDEO_FORMAT_JPEG;
                    isx012_format.cap_param.rate        = VIDEO_15FPS;
                    isx012_format.cap_param.jpeg_hsize  = fmt_lp->fmt.pix.width; 
                    isx012_format.cap_param.jpeg_vsize  = fmt_lp->fmt.pix.height;

                    /* Maintain current setting for monitoring */

                    isx012_format.moni_param.format     = VIDEO_FORMAT_YUV;
                    isx012_format.moni_param.rate       = VIDEO_30FPS;
                    isx012_format.moni_param.yuv_hsize  = priv->cap_param[VIDEO_MODE_MONITORING].yuv_hsize;
                    isx012_format.moni_param.yuv_vsize  = priv->cap_param[VIDEO_MODE_MONITORING].yuv_vsize;
                  }

                ret = isx012_ioctl(IMGIOC_SETMODEP, (unsigned long)&isx012_format);
                if (ret < 0)
                  {
                    video_printf("ERROR: ioctl IMGIOC_SETMODEP %d.\n", ret);
                    return ret;
                  }

                if (fmt_lp->fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
                  {
                    /* Update internal information for monitoring */

                    g_v4_buf_mode = VIDEO_MODE_MONITORING;
                    priv->cap_param[VIDEO_MODE_MONITORING].yuv_hsize = fmt_lp->fmt.pix.width;
                    priv->cap_param[VIDEO_MODE_MONITORING].yuv_vsize = fmt_lp->fmt.pix.height;
                  }
                else
                  {
                    /* Update internal information for capture */

                    g_v4_buf_mode = VIDEO_MODE_CAPTURE;
                    priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_hsize = fmt_lp->fmt.pix.width;
                    priv->cap_param[VIDEO_MODE_CAPTURE].jpeg_vsize = fmt_lp->fmt.pix.height;
                  } 
              }
          }

        break;
      case VIDIOC_REQBUFS:
        if (req_lp == NULL)
          {
            ret = -EPERM;
          }
        else
          {
            if (req_lp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
              {
                ret = -ENOSYS;
              }
            else if (req_lp->memory != V4L2_MEMORY_USERPTR)
              {
                ret = -ENOSYS;
              }
            else
              {
                g_v4_buf_rcnt = req_lp->count;
                if (g_v4_buf_rcnt > VIDEO_V4_BUF_MAX_CNT)
                  {
                    ret = -ENOMEM;
                  }
                else
                  {
                    g_v4_buf = calloc(g_v4_buf_rcnt, sizeof(v4l2_buffer_t));
                    g_v4_buf_cnt = 0;
                    if (g_v4_buf == NULL)
                      {
                        ret = -ENOMEM;
                      }

                  }

              }

          }

        break;
      case VIDIOC_QBUF:
        if (buf_lp == NULL)
          {
            ret = -EPERM;
          }
        else
          {
            if (buf_lp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
              {
                ret = -ENOSYS;
              }
            else if (buf_lp->memory != V4L2_MEMORY_USERPTR)
              {
                ret = -ENOSYS;
              }
            else if (buf_lp->index >= g_v4_buf_rcnt)
              {
                ret = -EPERM;
              }
            else if (buf_lp->index >= VIDEO_V4_BUF_MAX_CNT)
              {
                ret = -ENOMEM;
              }
            else
              {
                g_v4_buf[buf_lp->index].m.userptr = buf_lp->m.userptr;
                g_v4_buf[buf_lp->index].length    = buf_lp->length;
              }

          }

        break;
      case VIDIOC_DQBUF:
        if (buf_lp == NULL)
          {
            ret = -EPERM;
          }
        else
          {
            if (buf_lp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
              {
                ret = -ENOSYS;
              }
            else if (buf_lp->memory != V4L2_MEMORY_USERPTR)
              {
                ret = -ENOSYS;
              }
            else
              {
                cap.mode        = g_v4_buf_mode;
                cap.crop_ctrl   = VIDEO_CROP_DISABLE;
                cap.buffer.addr = g_v4_buf[g_v4_buf_cnt].m.userptr;
                cap.buffer.size = g_v4_buf[g_v4_buf_cnt].length;
                ret = video_capture_frame(priv, &cap);
                if (!ret)
                  {
                    buf_lp->m.userptr = g_v4_buf[g_v4_buf_cnt].m.userptr;
                    buf_lp->length    = g_v4_buf[g_v4_buf_cnt].length;
                    buf_lp->bytesused = g_v_cisif.size;
                    buf_lp->index     = g_v4_buf_cnt;
                    g_v4_buf_cnt = (g_v4_buf_cnt < (g_v4_buf_rcnt-1)) ?
                                   g_v4_buf_cnt+1 : 0;
                  }

              }

          }

        break;
      case VIDIOC_STREAMON:
        if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
          {
            ret = -ENOSYS;
          }
        else
          {
            stat_l.state = VIDEO_STATE_ACTIVE;
            ret = video_chg_img_sns_state(priv, &stat_l);
            usleep(100000);
          }

        break;
      default:
        videoerr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

int video_register(FAR const char *devpath)
{
  video_mng_t *priv;
  char path[VIDEO_DEV_PATH_LEN];
  int ret;

  /* Initialize video device structure */

  priv = (FAR struct video_mng_s *)kmm_malloc(sizeof(struct video_mng_s));
  if (!priv)
    {
      videoerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  g_v_mng = priv;
  sem_init(&priv->sem_cisifsync, 0, 0);

  /* Register the character driver */

  (void) snprintf(path, sizeof(path), "%s%d", devpath, 0);
  ret = register_driver(path, &g_videofops, 0666, priv);
  if (ret < 0)
    {
      videoerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }
  else
    {
      strncpy((char *)g_v_mng->devpath, path, VIDEO_DEV_PATH_LEN - 1);
      videoerr("ISX012 driver loaded successfully!\n");
    }

  return ret;
}

int video_unregister(void)
{
  int ret = 0;

  if (g_v_mng)
    {
      unregister_driver((const char *)g_v_mng->devpath);
      sem_destroy(&g_v_mng->sem_cisifsync);
      kmm_free(g_v_mng);
      g_v_mng = NULL;
      return ret;
    }

  return -ENODEV;
}
