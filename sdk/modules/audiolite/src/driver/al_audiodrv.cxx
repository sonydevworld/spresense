/****************************************************************************
 * modules/audiolite/src/base/al_audiodrv.cxx
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <nuttx/audio/audio.h>

#include <audiolite/al_debug.h>
#include <audiolite/al_audiodrv.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALDRV_MODE_NONE   (0)
#define ALDRV_MODE_OUTPUT (1)
#define ALDRV_MODE_INPUT  (2)

#define ALDRV_THREADMSG_TERM (1)

/****************************************************************************
 * Class: audiolite_driver
 ****************************************************************************/

SINGLETON_INST(audiolite_driver);

audiolite_driver::audiolite_driver() : _fd(-1), _mode(ALDRV_MODE_NONE),
                                       _mq(-1), _tid(-1), _listener(NULL),
                                       _enq_cnt(0)
{
}

audiolite_driver::~audiolite_driver()
{
  if (_fd >= 0)
    {
      reset();
    }
}

int audiolite_driver::setup_driver(audiolite_drvlistener *l,
                                   const char *devpath)
{
  int ret = -EALREADY;
  struct mq_attr attr;

  if (_fd < 0)
    {
      if (l == NULL)
        {
          return -EINVAL;
        }

      ret = open(CONFIG_ALDRV_OUTDEVNAME, O_RDWR | O_CLOEXEC);
      if (ret >= 0)
        {
          _fd = ret;
          ret = -EIO;
          if (ioctl(_fd, AUDIOIOC_RESERVE, 0) < 0)
            {
              goto error_close;
            }

          mq_unlink(CONFIG_ALDRV_MSGQNAME);

          attr.mq_maxmsg = CONFIG_ALDRV_MQDEPTH;
          attr.mq_msgsize = sizeof(struct audio_msg_s);
          attr.mq_curmsgs = 0;
          attr.mq_flags = 0;
          _mq = mq_open(CONFIG_ALDRV_MSGQNAME,
                O_RDWR | O_CREAT, 0644, &attr);

          if (_mq < (mqd_t)0)
            {
              goto error_close;
            }

          if (ioctl(_fd, AUDIOIOC_REGISTERMQ,
                    (unsigned long)_mq) < 0)
            {
              goto error_mqclose;
            }

          ret = OK;
          _listener = l;
        }
    }

  return ret;

error_mqclose:
  mq_close(_mq);
  mq_unlink(CONFIG_ALDRV_MSGQNAME);

error_close:
  close(_fd);
  _fd = -1;

  return ret;
}

int audiolite_driver::as_output(audiolite_drvlistener *l)
{
  int ret = setup_driver(l, CONFIG_ALDRV_OUTDEVNAME);

  if (ret == 0)
    {
      _mode = ALDRV_MODE_OUTPUT;
    }

  return ret;
}

int audiolite_driver::as_input(audiolite_drvlistener *l)
{
  int ret = setup_driver(l, CONFIG_ALDRV_INDEVNAME);

  if (ret == 0)
    {
      _mode = ALDRV_MODE_INPUT;
    }

  return ret;
}

int audiolite_driver::set_audioparam(int fs, int bps, int chnum)
{
  int ret = -EAGAIN;

  if (_fd >= 0)
    {
      struct audio_caps_desc_s cap;

      cap.caps.ac_len = sizeof(struct audio_caps_s);
      cap.caps.ac_type = _mode == ALDRV_MODE_OUTPUT ? AUDIO_TYPE_OUTPUT
                                                    : AUDIO_TYPE_INPUT;
      cap.caps.ac_channels = chnum;
      cap.caps.ac_chmap = 0;
      cap.caps.ac_controls.hw[0] = fs;
      cap.caps.ac_controls.b[2] = bps;
      cap.caps.ac_controls.b[3] = 0;  /* Upper fs over 65535 */

      ret = ioctl(_fd, AUDIOIOC_CONFIGURE,
                  (unsigned long)(uintptr_t)&cap);
      if (ret == 0)
        {
          ret = set_volume(1000);
        }
    }

  return ret;
}

int audiolite_driver::enqueue_buffer(FAR struct ap_buffer_s *apb)
{
  int ret = -EAGAIN;

  if (_fd >= 0)
    {
      struct audio_buf_desc_s desc;

      desc.numbytes = apb->nbytes;
      desc.u.buffer = apb;

      ret = ioctl(_fd, AUDIOIOC_ENQUEUEBUFFER,
                  (unsigned long)(uintptr_t)&desc);

      if (ret == 0)
        {
          _enq_cnt++;
          if (_enq_cnt == CONFIG_ALDRV_TRIGER_ENQCOUNT)
            {
              ret = start_driver();
            }
        }
    }

  return ret;
}

int audiolite_driver::set_volume(int vol)
{
  if (_fd >= 0)
    {
      struct audio_caps_desc_s cap_desc;

      cap_desc.caps.ac_len            = sizeof(struct audio_caps_s);
      cap_desc.caps.ac_type           = AUDIO_TYPE_FEATURE;
      cap_desc.caps.ac_format.hw      = AUDIO_FU_VOLUME;
      cap_desc.caps.ac_controls.hw[0] = vol;

      return ioctl(_fd, AUDIOIOC_CONFIGURE,
                   (unsigned long)(uintptr_t)&cap_desc);
    }

  return -EAGAIN;
}

int audiolite_driver::start_driver(void)
{
  return ioctl(_fd, AUDIOIOC_START, 0);
}

int audiolite_driver::start(void)
{
  int ret = -EAGAIN;

  if (_fd >= 0)
    {
      _enq_cnt = 0;
      if (_tid < 0)
        {
          ret = mossfw_create_thread_attr(&_tid,
                                          audiolite_driver::message_thread,
                                          this,
                                          CONFIG_ALDRV_PRIO,
                                          CONFIG_ALDRV_STACKSZ);
          if (ret != 0)
            {
              _tid = -1;
            }
          else
            {
              pthread_setname_np(_tid, "al_driver");
            }
        }
    }

  return ret;
}

int audiolite_driver::stop(void)
{
  int ret = -EAGAIN;

  if (_fd >= 0)
    {
      ret = ioctl(_fd, AUDIOIOC_STOP, 0);
    }

  return ret;
}

int audiolite_driver::stop_thread(void)
{
  int ret = -EAGAIN;

  if (_fd >= 0 && _tid >= 0)
    {
      mqd_t smq;
      struct audio_msg_s term_msg;

      smq = mq_open(CONFIG_ALDRV_MSGQNAME, O_WRONLY);
      if (smq >= (mqd_t)0)
        {
          term_msg.msg_id = AUDIO_MSG_USER;
          term_msg.u.data = ALDRV_THREADMSG_TERM;
          mq_send(smq, (FAR const char *)&term_msg, sizeof(term_msg), 0);
          mq_close(smq);

          mossfw_thread_join(&_tid);
          _tid = -1;
          ret = OK;
        }
      else
        {
          ret = smq;
        }
    }

  return ret;
}

void audiolite_driver::reset(void)
{
  if (_fd >= 0)
    {
      ioctl(_fd, AUDIOIOC_STOP, 0);
      stop_thread();
      ioctl(_fd, AUDIOIOC_UNREGISTERMQ, (unsigned long)_mq);
      ioctl(_fd, AUDIOIOC_RELEASE, 0);
      ioctl(_fd, AUDIOIOC_SHUTDOWN, 0);
      close(_fd);
      mq_close(_mq);
      mq_unlink(CONFIG_ALDRV_MSGQNAME);

      _mq = -1;
      _fd = -1;
      _mode = ALDRV_MODE_NONE;
      _listener = NULL;
    }
}

void *audiolite_driver::message_thread(void *arg)
{
  bool running = true;
  struct audio_msg_s msg;
  unsigned int prio;
  ssize_t size;
  audiolite_driver *drv = (audiolite_driver *)arg;

  while (running)
    {
      size = mq_receive(drv->_mq, (FAR char *)&msg, sizeof(msg), &prio);

      if (size != sizeof(msg))
        {
          continue;
        }

      switch (msg.msg_id)
        {
          case AUDIO_MSG_DEQUEUE:
            if (drv->_listener)
              {
                if (drv->_mode == ALDRV_MODE_OUTPUT)
                  {
                    drv->_listener->on_pusheddata(
                                      (struct ap_buffer_s *)msg.u.ptr);
                  }
                else
                  {
                    drv->_listener->on_popeddata(
                                      (struct ap_buffer_s *)msg.u.ptr);
                  }
              }

            break;

          case AUDIO_MSG_UNDERRUN:
            if (drv->_listener)
              {
                if (drv->_mode == ALDRV_MODE_OUTPUT)
                  {
                    drv->_listener->on_underflowed();
                  }
                else
                  {
                    drv->_listener->on_overflowed();
                  }
              }

            break;

          case AUDIO_MSG_COMPLETE:
            if (drv->_listener)
              {
                drv->_listener->on_stopped();
              }

            break;

          case AUDIO_MSG_USER:
            switch (msg.u.data)
              {
                case ALDRV_THREADMSG_TERM:
                  al_ddebug("Terminate messave reach\n");
                  running = false;
                  break;
              }

            break;

          default:
            break;
        }
    }

  return NULL;
}
