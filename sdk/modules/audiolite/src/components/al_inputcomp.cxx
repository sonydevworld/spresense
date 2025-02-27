/****************************************************************************
 * modules/audiolite/src/components/al_inputcomp.cxx
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

#include <nuttx/config.h>
#include <assert.h>

#include <audiolite/al_debug.h>
#include <audiolite/al_memalloc.h>
#include <audiolite/al_inputcomp.h>
#include <audiolite/al_audiodrv.h>
#include <audiolite/al_eventlistener.h>

#define MIC_DEVFILE  "/dev/audio/pcm_in0"
#define MIC_MSGQFILE "/tmp/micmq"
#define MIC_MAXCH (4)
#define I2SIN_DEVFILE  "/dev/audio/pcm_in1"
#define I2SIN_MSGQFILE "/tmp/i2sinmq"
#define I2SIN_MAXCH (2)

/****************************************************************************
 * Class: audiolite_inputcomp
 ****************************************************************************/

audiolite_inputcomp::audiolite_inputcomp(bool isi2s) :
  audiolite_component(0,1), _tid(-1), _is_running(false), _is_stopped(true)
{
  if (isi2s)
    {
      _driver = new audiolite_driver(ALDRV_MODE_INPUT,
                                     I2SIN_DEVFILE,
                                     I2SIN_MSGQFILE,
                                     I2SIN_MAXCH);
    }
  else
    {
      _driver = new audiolite_driver(ALDRV_MODE_INPUT,
                                     MIC_DEVFILE,
                                     MIC_MSGQFILE,
                                     MIC_MAXCH);
    }

  ASSERT(_driver != NULL);
  _driver->set_listener(this);
  mossfw_lock_init(&_slock);
  mossfw_condition_init(&_scond);
}

audiolite_inputcomp::~audiolite_inputcomp()
{
  _driver->reset();
  al_ddebug("Thread join.\n");
  if (_tid >= 0)
    {
      _is_running = false;
      if (_pool)
        {
          _pool->disable_pool();
        }

      notice_stop(true);

      mossfw_thread_join(&_tid);
    }

  delete _driver;
}

int audiolite_inputcomp::set_micgain(int vol)
{
  return _driver->set_volume(vol);
}

int audiolite_inputcomp::start_thread()
{
  int ret = 0;

  if (_tid < 0)
    {
      _is_running = true;
      _is_stopped = false;
      ret = mossfw_create_thread_attr(&_tid,
                                      audiolite_inputcomp::inject_worker,
                                      this,
                                      CONFIG_ALINPUTCOMP_PRIO,
                                      CONFIG_ALINPUTCOMP_STACKSZ);
      if (ret != 0)
        {
          _tid = -1;
          _is_running = false;
          _is_stopped = true;
        }
    }
  else
    {
      notice_stop(false);
    }

  return ret;
}

void audiolite_inputcomp::notice_stop(bool isstop)
{
  al_ddebug("Enter\n");
  mossfw_lock_take(&_slock);
  _is_stopped = isstop;
  mossfw_condition_notice(&_scond);
  mossfw_lock_give(&_slock);
}

/* Inherited member functions from audiolite_component */

int audiolite_inputcomp::on_starting(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  int ret;

  al_ddebug("Entry\n");
  if (_pool == NULL)
    {
      publish_event(AL_EVENT_DRVERROR, (unsigned long)(-ENOEXEC));
      return -1;
    }

  ret = audiolite_component::on_starting(inode, onode);

  al_ddebug("Leave\n");
  return ret;
}

void audiolite_inputcomp::on_started(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  int ret;

  al_ddebug("Entry\n");

  _pool->enable_pool();

  ret = _driver->start(samplingrate(), samplebitwidth(), channels());
  if (ret != 0)
    {
      publish_event(AL_EVENT_DRVERROR, (unsigned long)ret);
      return;
    }

  ret = start_thread();
  if (ret != 0)
    {
      publish_event(AL_EVENT_INITERROR, (unsigned long)ret);
      _driver->stop();
      return;
    }

  audiolite_component::on_started(inode, onode);
  al_ddebug("Leave\n");
}

void audiolite_inputcomp::on_canceled(audiolite_inputnode *inode,
                                       audiolite_outputnode *onode)
{
  al_ddebug("Enter\n");
  _driver->reset();
  audiolite_component::on_canceled(inode, onode);
}

void audiolite_inputcomp::on_stopping(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  al_ddebug("Enter\n");
  audiolite_component::on_stopping(inode, onode);
  notice_stop(true);
}

void audiolite_inputcomp::on_stop(audiolite_inputnode *inode,
                                   audiolite_outputnode *onode)
{
  al_ddebug("Enter\n");
  _driver->stop();
  audiolite_component::on_stop(inode, onode);
  al_ddebug("Leave\n");
}

/* Inherited member functions from audiolite_drvlistener */

void audiolite_inputcomp::on_popeddata(FAR struct ap_buffer_s *apb)
{
  audiolite_memapbuf *mem =
      audiolite_memapbuf::local_cast((dq_entry_t *)apb);

  if (mem)
    {
      mem->set_fs(samplingrate());
      mem->set_channels(channels());
      mem->set_storedsize(mem->get_fullsize());
      _outs[0]->push_data(mem);
      mem->release();
    }
  else
    {
      al_ddebug("Error no memory on audiolite_inputcomp::on_popeddata\n");
    }
}

void audiolite_inputcomp::on_stopped(void)
{
  publish_event(AL_EVENT_STOPINPUT, 0);
}

void audiolite_inputcomp::on_overflowed(void)
{
  publish_event(AL_EVENT_OVERFLOW, 0);
}

void *audiolite_inputcomp::inject_worker(void *arg)
{
  audiolite_memapbuf *mem;
  audiolite_inputcomp *thiz = (audiolite_inputcomp *)arg;

  while (thiz->_is_running)
    {
      if (thiz->_is_stopped)
        {
          mossfw_lock_take(&thiz->_slock);
          mossfw_condition_wait(&thiz->_scond, &thiz->_slock);
          mossfw_lock_give(&thiz->_slock);
        }
      else
        {
          mem = (audiolite_memapbuf *)thiz->_pool->allocate();
          if (mem)
            {
              if (thiz->_is_stopped)
                {
                  mem->release();
                }
              else
                {
                  if (thiz->_driver->enqueue_buffer(mem->get_raw_abuf()) != OK)
                    {
                      mem->release();
                      usleep(1);
                    }
                }
            }
        }
    }

  return NULL;
}
