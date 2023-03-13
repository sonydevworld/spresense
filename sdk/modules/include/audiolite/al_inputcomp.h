/****************************************************************************
 * modules/include/audiolite/al_inputcomp.h
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

#ifndef __INCLUDE_AUDIOLITE_INPUT_COMPONENT_H
#define __INCLUDE_AUDIOLITE_INPUT_COMPONENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/al_source.h>
#include <audiolite/al_audiodrv.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_component
 ****************************************************************************/

class audiolite_inputcomp : public audiolite_source,
                            public audiolite_drvlistener
{
  private:
    mossfw_thread_t _tid;
    volatile bool _is_running;

    int start_thread();
    static void *inject_worker(void *arg);

  public:
    audiolite_inputcomp();
    ~audiolite_inputcomp();

  /* Inherited member functions from audiolite_component */

    int on_starting(audiolite_inputnode *inode,
                    audiolite_outputnode *onode);
    void on_started(audiolite_inputnode *inode,
                    audiolite_outputnode *onode);
    void on_canceled(audiolite_inputnode *inode,
                     audiolite_outputnode *onode);
    void on_stop(audiolite_inputnode *inode,
                 audiolite_outputnode *onode);

  /* Inherited member functions from audiolite_drvlistener */

    void on_pusheddata(FAR struct ap_buffer_s *apb){/* Never happened. */};
    void on_stopped(void);
    void on_underflowed(void){ /* Never happened. */ };
    void on_popeddata(struct ap_buffer_s *apb);
    void on_overflowed(void);

  /* Inherited member functions from audiolite_source */

    int start();
    void stop();
    void pause();
    int resume();
};

#endif /* __INCLUDE_AUDIOLITE_INPUT_COMPONENT_H */
