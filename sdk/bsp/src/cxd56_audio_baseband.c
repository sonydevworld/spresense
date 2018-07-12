/****************************************************************************
 * bsp/src/cxd56_audio_baseband.c
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

#include <nuttx/kmalloc.h>

#include <arch/chip/cxd56_audio.h>
#include "audio/audio_io_config.h"
#include "cxd56_clock.h"
#include "arch/board/board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AUDIO_DEV_PATH_LEN  32

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cxd56_audio_bb_open(FAR struct file *filep);
static int cxd56_audio_bb_close(FAR struct file *filep);
static ssize_t cxd56_audio_bb_read(FAR struct file *filep,
                                   FAR char *buffer, size_t len);
static ssize_t cxd56_audio_bb_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen);
static int cxd56_audio_bb_ioctl(FAR struct file *filep, int cmd,
                                unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_audio_bb_fops =
{
  cxd56_audio_bb_open,  /* open */
  cxd56_audio_bb_close, /* close */
  cxd56_audio_bb_read,  /* read */
  cxd56_audio_bb_write, /* write */
  NULL,                 /* seek */
  cxd56_audio_bb_ioctl, /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  NULL,                 /* poll */
#endif
};

struct cxd56_audio_bb_dev_s
{
  sem_t devsem;
  char devpath[AUDIO_DEV_PATH_LEN];
};

static struct cxd56_audio_bb_dev_s g_audio_bb_dev;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_audio_bb_power(int type, unsigned long arg)
{
  int ret = 0;
  int target = 0;
  uint32_t mode = (uint32_t)arg;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_ENABLE:
        {
          if (mode & AUDIO_CXD5247)
            {
              _info("Power ON audio\n");
              if (mode & AUDIO_CXD5247_AVDD)
                {
                  target |= CXD5247_AVDD;
                }
              if (mode & AUDIO_CXD5247_DVDD)
                {
                  target |= CXD5247_DVDD;
                }
              ret = board_aca_power_control(target, true);
              if (ret != 0)
                {
                  printf("Failed Aca Power ON(%d)\n", ret);
                }
            }
          if (mode & AUDIO_CXD56xx)
            {
              cxd56_audio_clock_enable(AUD_MCLK_EXT, 0);
              setAudioIoMclk();
            }
        }
        break;

      case AUDIO_IOCTL_TYPE_DISABLE:
        {
          if (mode & AUDIO_CXD56xx)
            {
              cxd56_audio_clock_disable();
            }
          if (mode & AUDIO_CXD5247)
            {
              _info("Power OFF audio\n");
              if (mode & AUDIO_CXD5247_AVDD)
                {
                  target |= CXD5247_AVDD;
                }
              if (mode & AUDIO_CXD5247_DVDD)
                {
                  target |= CXD5247_DVDD;
                }
              ret = board_aca_power_control(target, false);
              if (ret != 0)
                {
                  printf("Failed Aca Power OFF(%d)\n", ret);
                }
            }
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_common(int type, unsigned long arg)
{
  int ret = 0;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_ENABLE:
        {
          FAR struct audio_bb_power_param_s *param;
          param = (FAR struct audio_bb_power_param_s *)arg;
          ret = AS_PowerOnBaseBand(param->rate, param->bypass_mode);
        }
        break;

      case AUDIO_IOCTL_TYPE_DISABLE:
        {
          ret = AS_PowerOffBaseBand();
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_input(int type, unsigned long arg)
{
  int ret = 0;
  FAR struct audio_bb_input_param_s *param;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_ENABLE:
        {
          param = (FAR struct audio_bb_input_param_s *)arg;
          ret = AS_BaseBandEnable_input(param->micMode, param->micGain);
        }
        break;

      case AUDIO_IOCTL_TYPE_DISABLE:
        {
          ret = AS_BaseBandDisable_input((asMicMode)arg);
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_output(int type, unsigned long arg)
{
  int ret = 0;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_ENABLE:
        {
          ret = AS_BaseBandEnable_output((asOutDeviceId)arg);
        }
        break;

      case AUDIO_IOCTL_TYPE_DISABLE:
        {
          ret = AS_BaseBandDisable_output();
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_mic(int type, unsigned long arg)
{
  int ret = 0;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_SET:
        {
          ret = AS_SetMicGain((int32_t *)arg);
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_volume(int type, unsigned long arg)
{
  int ret = 0;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_SET:
        {
          ret = AS_SetVolume((FAR asCodecVol *)arg);
        }
        break;

      case AUDIO_IOCTL_TYPE_MUTE:
        {
          ret = AS_MuteVolume((asCodecVolSelId)arg);
        }
        break;

      case AUDIO_IOCTL_TYPE_UNMUTE:
        {
          ret = AS_UnMuteVolume((asCodecVolSelId)arg);
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_clearstereo(int type, unsigned long arg)
{
  int ret = 0;
  FAR struct audio_bb_clearstereo_param_s *param;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_INIT:
        {
          param = (FAR struct audio_bb_clearstereo_param_s *)arg;
          ret = AS_InitClearStereo(param->csEn, param->csSign, param->csVol);
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_beep(int type, unsigned long arg)
{
  int ret = 0;
  FAR struct audio_bb_beep_param_s *param;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_SET:
        {
          param = (FAR struct audio_bb_beep_param_s *)arg;
          ret = AS_SetBeepParam(param->beepFreq, param->beepVol);
        }
        break;

      case AUDIO_IOCTL_TYPE_ENABLE:
        {
          ret = AS_BeepEnable();
        }
        break;

      case AUDIO_IOCTL_TYPE_DISABLE:
        {
          ret = AS_BeepDisable();
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

int cxd56_audio_bb_datapath(int type, unsigned long arg)
{
  int ret = 0;
  FAR struct audio_bb_datapath_param_s *param;

  switch (type)
    {
      case AUDIO_IOCTL_TYPE_RESET:
        {
          ret = AS_ClearAudioDataPathAll();
        }
        break;

      case AUDIO_IOCTL_TYPE_SET:
        {
          param = (FAR struct audio_bb_datapath_param_s *)arg;
          ret = AS_SetAudioDataPath(param->pPathSelParam,
                                    param->getDmacId,
                                    param->setDmacId);
        }
        break;

      case AUDIO_IOCTL_TYPE_CLEAR:
        {
          ret = AS_ClearAudioDataPath((FAR asPathSelParam *)arg);
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl type(%d)\n", type);
          ret = -EINVAL;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_audio_bb_initialize
 *
 * Description:
 *   This function initialize the audio baseband device.
 *
 ****************************************************************************/

static int cxd56_audio_bb_initialize(FAR struct cxd56_audio_bb_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: cxd56_audio_bb_finalize
 *
 * Description:
 *   This function finalize the audio baseband device.
 *
 ****************************************************************************/

static int cxd56_audio_bb_finalize(FAR struct cxd56_audio_bb_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: cxd56_audio_bb_open
 *
 * Description:
 *   This function is called when the audio baseband device is opened.
 *
 ****************************************************************************/

static int cxd56_audio_bb_open(FAR struct file *filep)
{
  return 0;
}

/****************************************************************************
 * Name: cxd56_audio_bb_close
 *
 * Description:
 *   This function is called when the audio baseband device is closed.
 *
 ****************************************************************************/

static int cxd56_audio_bb_close(FAR struct file *filep)
{
  return 0;
}

/****************************************************************************
 * Name: cxd56_audio_bb_read
 *
 * Description:
 *   The standard read method for the audio baseband device.
 *
 ****************************************************************************/

static ssize_t cxd56_audio_bb_read(FAR struct file *filep,
                                   FAR char *buffer,
                                   size_t len)
{
  return 0;
}

/****************************************************************************
 * Name: cxd56_audio_bb_write
 *
 * Description:
 *   The standard write method for the audio baseband device.
 *
 ****************************************************************************/

static ssize_t cxd56_audio_bb_write(FAR struct file *filep,
                                    FAR const char *buffer,
                                    size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: cxd56_audio_bb_ioctl
 *
 * Description:
 *   The standard ioctl method for the audio baseband device.
 *
 ****************************************************************************/

static int cxd56_audio_bb_ioctl(FAR struct file *filep,
                                int cmd,
                                unsigned long arg)
{
  int ret = 0;

  FAR struct cxd56_audio_bb_dev_s *priv;
  FAR struct inode *inode = filep->f_inode;
  priv = (FAR struct cxd56_audio_bb_dev_s *)inode->i_private;
  sem_wait(&priv->devsem);

  int type = cmd & 0xff;
  cmd &= 0xff00;

  switch (cmd)
    {
      case CXD56_AUDIO_IOCTL_BB_POWER:
        {
          ret = cxd56_audio_bb_power(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_COMMON:
        {
          ret = cxd56_audio_bb_common(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_INPUT:
        {
          ret = cxd56_audio_bb_input(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_OUTPUT:
        {
          ret = cxd56_audio_bb_output(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_MICGAIN:
        {
          ret = cxd56_audio_bb_mic(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_VOLUME:
        {
          ret = cxd56_audio_bb_volume(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_CLEARSTEREO:
        {
          ret = cxd56_audio_bb_clearstereo(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_BEEP:
        {
          ret = cxd56_audio_bb_beep(type, arg);
        }
        break;

      case CXD56_AUDIO_IOCTL_BB_DATAPATH:
        {
          ret = cxd56_audio_bb_datapath(type, arg);
        }
        break;

      default:
        {
          printf("ERROR: Invalid ioctl command(%d)\n", cmd);
          ret = -EINVAL;
        }
        break;
    }

  sem_post(&priv->devsem);

  return ret;
}

/****************************************************************************
 * Name: cxd56_audio_bb_register
 *
 * Description:
 *   This function registers the audio baseband driver so that can be used
 *   with device path.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to be registers in the NuttX
 *             pseudo-filesystem.
 *             The recommended convention is to name audio baseband drivers
 *             based on the function they provide such
 *             as "/dev/audio/baseband".
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_audio_bb_register(FAR const char *devpath)
{
  FAR struct cxd56_audio_bb_dev_s *priv;
  int ret = 0;

  priv = &g_audio_bb_dev;

  if (strlen(devpath) > AUDIO_DEV_PATH_LEN)
    {
      return -ENAMETOOLONG;
    }

  memset(priv, 0, sizeof(struct cxd56_audio_bb_dev_s));
  sem_init(&priv->devsem, 0, 1);
  strncpy(priv->devpath, devpath, AUDIO_DEV_PATH_LEN - 1);

  ret = cxd56_audio_bb_initialize(priv);
  if (ret < 0)
    {
      printf("Failed to initialize audio baseband device:%d\n", ret);
      goto _err0;
    }

  ret = register_driver(devpath, &g_audio_bb_fops, 0666, priv);
  if (ret < 0)
    {
      printf("Failed to register audio baseband driver:%d\n", ret);
      goto _err0;
    }

  _info("'%s' loaded\n", (FAR const char *)devpath);

  return ret;

_err0:
  sem_destroy(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cxd56_audio_bb_unregister
 *
 * Description:
 *   This function unregisters the audio baseband driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver which is used for registration.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_audio_bb_unregister(FAR const char *devpath)
{
  FAR struct cxd56_audio_bb_dev_s *priv;
  int ret = 0;

  priv = &g_audio_bb_dev;

  if (strncmp(devpath, priv->devpath, AUDIO_DEV_PATH_LEN) == 0)
    {
      ret = cxd56_audio_bb_finalize(priv);
      if (ret < 0)
        {
          printf("Failed to finalize audio baseband device:%d\n", ret);
        }

      unregister_driver(priv->devpath);
      sem_destroy(&priv->devsem);

      _info("'%s' unloaded\n", (FAR const char *)devpath);

      return ret;
    }
  return -ENODEV;
}
