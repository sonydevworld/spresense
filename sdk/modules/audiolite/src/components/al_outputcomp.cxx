/****************************************************************************
 * modules/audiolite/src/components/al_outputcomp.cxx
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
#include <audiolite/al_outputcomp.h>
#include <audiolite/al_audiodrv.h>
#include <audiolite/al_eventlistener.h>

#define SPK0_DEVFILE  "/dev/audio/pcm0"
#define SPK0_MSGQFILE "/tmp/spk0mq"
#define SPK0_MAXCH (2)
#define SPK1_DEVFILE  "/dev/audio/pcm1"
#define SPK1_MSGQFILE "/tmp/spk1mq"
#define SPK1_MAXCH (2)

/****************************************************************************
 * Class: audiolite_outputcomp
 ****************************************************************************/

audiolite_outputcomp::audiolite_outputcomp(bool is_sub) :
    audiolite_component(1,0, 16, true, CONFIG_ALOUTCOMP_PRIO,
                                       CONFIG_ALOUTCOMP_STACKSZ),
    enqueue_enable(false)
{
  set_operatorname("outputcomp");
  if  (is_sub)
    {
      _driver = new audiolite_driver(ALDRV_MODE_OUTPUT,
                                     SPK1_DEVFILE,
                                     SPK1_MSGQFILE,
                                     SPK1_MAXCH);
    }
  else
    {
      _driver = new audiolite_driver(ALDRV_MODE_OUTPUT,
                                     SPK0_DEVFILE,
                                     SPK0_MSGQFILE,
                                     SPK0_MAXCH);
    }

  ASSERT(_driver != NULL);
}

audiolite_outputcomp::~audiolite_outputcomp()
{
  _driver->reset();
  delete _driver;
}

int audiolite_outputcomp::set_volume(int vol)
{
  return _driver->set_volume(vol);
}

/* Inherited member functions from audiolite_component */

void audiolite_outputcomp::on_data()
{
  audiolite_memapbuf *mem =
              (audiolite_memapbuf *)pop_data();
  if (mem)
    {
      if (enqueue_enable)
        {
          if (_driver->enqueue_buffer(mem->get_raw_abuf()) != OK)
            {
              mem->release();
            }
        }
      else
        {
          mem->release();
        }
    }
}

int audiolite_outputcomp::on_starting(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  int ret;

  audiolite_component::on_starting(inode, onode);
  enqueue_enable = true;
  _driver->set_listener(this);
  ret = _driver->start(samplingrate(), samplebitwidth(), channels());
  if (ret != 0)
    {
      publish_event(AL_EVENT_DRVERROR, (unsigned long)ret);
      return -1;
    }

  return 0;
}

void audiolite_outputcomp::on_started(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  audiolite_component::on_started(inode, onode);
}

void audiolite_outputcomp::on_canceled(audiolite_inputnode *inode,
                                       audiolite_outputnode *onode)
{
  enqueue_enable = false;
  _driver->stop();
  audiolite_component::on_canceled(inode, onode);
}

void audiolite_outputcomp::on_stop(audiolite_inputnode *inode,
                                   audiolite_outputnode *onode)
{
  enqueue_enable = false;
  _driver->stop();
  audiolite_component::on_stop(inode, onode);
}

/* Inherited member functions from audiolite_drvlistener */

void audiolite_outputcomp::on_pusheddata(FAR struct ap_buffer_s *apb)
{
  audiolite_memapbuf *mem =
      audiolite_memapbuf::local_cast((dq_entry_t *)apb);

  mem->release();
}

void audiolite_outputcomp::on_stopped(void)
{
  publish_event(AL_EVENT_STOPOUTPUT, 0);
}

void audiolite_outputcomp::on_underflowed(void)
{
  publish_event(AL_EVENT_UNDERFLOW, 0);
}
