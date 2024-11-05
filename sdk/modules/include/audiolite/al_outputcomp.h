/****************************************************************************
 * modules/include/audiolite/al_outputcomp.h
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

#ifndef __INCLUDE_AUDIOLITE_OUTPUT_COMPONENT_H
#define __INCLUDE_AUDIOLITE_OUTPUT_COMPONENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/al_component.h>
#include <audiolite/al_audiodrv.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_outputcomp
 ****************************************************************************/

class audiolite_outputcomp : public audiolite_component,
                             public audiolite_drvlistener
{
  private:
    volatile bool enqueue_enable;
    audiolite_driver *_driver;

  public:
    audiolite_outputcomp(bool is_sub = false);
    ~audiolite_outputcomp();

    int set_volume(int vol);

  /* Inherited member functions from audiolite_component */

    bool can_breakdata(audiolite_outputnode *out) { return true; };
    void on_data();
    int on_starting(audiolite_inputnode *inode,
                    audiolite_outputnode *onode);
    void on_started(audiolite_inputnode *inode,
                    audiolite_outputnode *onode);
    void on_canceled(audiolite_inputnode *inode,
                     audiolite_outputnode *onode);
    void on_stop(audiolite_inputnode *inode,
                 audiolite_outputnode *onode);

  /* Inherited member functions from audiolite_drvlistener */

    void on_pusheddata(FAR struct ap_buffer_s *apb);
    void on_stopped(void);
    void on_underflowed(void);
    void on_popeddata(struct ap_buffer_s *apb){/* Never happened. */};
    void on_overflowed(void){ /* Never happened. */ };
};

#endif  /* __INCLUDE_AUDIOLITE_OUTPUT_COMPONENT_H */
