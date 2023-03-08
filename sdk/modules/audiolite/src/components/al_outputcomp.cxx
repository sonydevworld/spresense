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

#include <audiolite/al_debug.h>
#include <audiolite/al_memalloc.h>
#include <audiolite/al_outputcomp.h>
#include <audiolite/al_audiodrv.h>
#include <audiolite/al_eventlistener.h>

/****************************************************************************
 * Class: audiolite_outputcomp
 ****************************************************************************/

audiolite_outputcomp::audiolite_outputcomp() : audiolite_component(1,0, 
                                               16, true,
                                               CONFIG_ALOUTCOMP_PRIO,
                                               CONFIG_ALOUTCOMP_STACKSZ)
{
  set_operatorname("outputcomp");
}

audiolite_outputcomp::~audiolite_outputcomp()
{
  audiolite_driver::terminate_instance();
}

/* Inherited member functions from audiolite_component */

void audiolite_outputcomp::on_data()
{
  audiolite_memapbuf *mem =
              (audiolite_memapbuf *)_ins[0]->pop_data(NULL);
  if (audiolite_driver::get_instance()->enqueue_buffer(mem->get_raw_abuf()) != OK)
    {
      mem->release();
    }
}

int audiolite_outputcomp::on_starting(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  int ret;

  audiolite_component::on_starting(inode, onode);
  ret = audiolite_driver::get_instance()->as_output(this);
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

  return 0;
}

void audiolite_outputcomp::on_started(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  audiolite_driver::get_instance()->start();
  audiolite_component::on_started(inode, onode);
}

void audiolite_outputcomp::on_canceled(audiolite_inputnode *inode,
                                       audiolite_outputnode *onode)
{
  audiolite_driver::get_instance()->reset();
  audiolite_component::on_canceled(inode, onode);
}

void audiolite_outputcomp::on_stop(audiolite_inputnode *inode,
                                   audiolite_outputnode *onode)
{
  audiolite_driver::get_instance()->reset();
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
