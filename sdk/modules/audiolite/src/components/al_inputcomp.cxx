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

#include <audiolite/al_debug.h>
#include <audiolite/al_memalloc.h>
#include <audiolite/al_inputcomp.h>
#include <audiolite/al_audiodrv.h>
#include <audiolite/al_eventlistener.h>

/****************************************************************************
 * Class: audiolite_inputcomp
 ****************************************************************************/

audiolite_inputcomp::audiolite_inputcomp() : audiolite_source(0,1),
                                             _tid(-1), _is_running(false)
{
}

audiolite_inputcomp::~audiolite_inputcomp()
{
  audiolite_driver::terminate_instance();
}

int audiolite_inputcomp::start_thread()
{
  int ret = 0;

  if (_tid < 0)
    {
      _is_running = true;
      ret = mossfw_create_thread_attr(&_tid,
                                      audiolite_inputcomp::inject_worker,
                                      this,
                                      CONFIG_ALINPUTCOMP_PRIO,
                                      CONFIG_ALINPUTCOMP_STACKSZ);
      if (ret != 0)
        {
          _tid = -1;
          _is_running = false;
        }
    }

  return ret;
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

  audiolite_component::on_starting(inode, onode);
  ret = audiolite_driver::get_instance()->as_input(this);
  if (ret != 0)
    {
      publish_event(AL_EVENT_DRVERROR, (unsigned long)ret);
      return -1;
    }

  ret = audiolite_driver::get_instance()
                            ->set_audioparam(samplingrate(),
                                             samplebitwidth(),
                                             channels());
  if (ret != 0)
    {
      publish_event(AL_EVENT_INVALIDSYSPARAM, (unsigned long)ret);
      audiolite_driver::get_instance()->reset();
      return -1;
    }

  ret = start_thread();
  if (ret != 0)
    {
      publish_event(AL_EVENT_INITERROR, (unsigned long)ret);
      audiolite_driver::get_instance()->reset();
      return -1;
    }

  al_ddebug("Leave\n");
  return 0;
}

void audiolite_inputcomp::on_started(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  al_ddebug("Entry\n");
  audiolite_driver::get_instance()->start();
  audiolite_component::on_started(inode, onode);
  al_ddebug("Leave\n");
}
void audiolite_inputcomp::on_canceled(audiolite_inputnode *inode,
                                       audiolite_outputnode *onode)
{
  audiolite_driver::get_instance()->reset();
  audiolite_component::on_canceled(inode, onode);
}

void audiolite_inputcomp::on_stop(audiolite_inputnode *inode,
                                   audiolite_outputnode *onode)
{
  al_ddebug("Enter\n");
  audiolite_driver::get_instance()->reset();
  _is_running = false;
  if (_pool)
    {
      _pool->disable_pool();
    }

  al_ddebug("Thread join.\n");
  mossfw_thread_join(&_tid);
  audiolite_component::on_stop(inode, onode);
  al_ddebug("Leave\n");
}

/* Inherited member functions from audiolite_drvlistener */

void audiolite_inputcomp::on_popeddata(FAR struct ap_buffer_s *apb)
{
  audiolite_memapbuf *mem =
      audiolite_memapbuf::local_cast((dq_entry_t *)apb);

  mem->set_fs(samplingrate());
  mem->set_channels(channels());
  mem->set_storedsize(mem->get_fullsize());

  _outs[0]->push_data(mem);
  mem->release();
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
      mem = (audiolite_memapbuf *)thiz->_pool->allocate();
      if (mem)
        {
          audiolite_driver::get_instance()
                        ->enqueue_buffer(mem->get_raw_abuf());
        }
    }

  return NULL;
}

/* Inherited member functions from audiolite_source */

int audiolite_inputcomp::start()
{
  return audiolite_component::start((audiolite_inputnode *)NULL);
}

void audiolite_inputcomp::stop()
{
  audiolite_component::stop((audiolite_inputnode *)NULL);
}

void audiolite_inputcomp::pause()
{
  audiolite_component::stop((audiolite_inputnode *)NULL);
}

int audiolite_inputcomp::resume()
{
  return audiolite_component::start((audiolite_inputnode *)NULL);
}
