/****************************************************************************
 * modules/include/audiolite/al_component.h
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

#ifndef __INCLUDE_AUDIOLITE_COMPONENT_H
#define __INCLUDE_AUDIOLITE_COMPONENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <audiolite/al_nodecomm.h>
#include <audiolite/al_inputnode.h>
#include <audiolite/al_outputnode.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_component
 ****************************************************************************/

class audiolite_component : public audiolite_nodecomm_if
{
  protected:
    audiolite_inputnode  **_ins;
    audiolite_outputnode **_outs;
    mossfw_callback_op_t *_op;
    audiolite_mempool *_pool;
    int _innum;
    int _outnum;
    int _state;

    int search_inputidx(audiolite_inputnode *node);
    static int operate_cb(mossfw_callback_op_t *op, unsigned long arg);
    void set_operatorname(const char *name);

    audiolite_mem *pop_data(int no = 0, int *used = NULL)
    {
      return (_ins && _innum > no) ? _ins[no]->pop_data(used) : NULL;
    };

    int push_data(audiolite_mem *mem, int no = 0)
    {
      if (_outs && _outnum > no) return _outs[no]->push_data(mem);
      return -1;
    };

#ifdef _ALDEBUG_ENABLE
  public:
    const char *dbg_name;
#endif

  public:
    audiolite_component(int inputnum = 1,
                        int outputnum = 1,
                        int depth = 16,
                        bool is_sync = true,
                        int prio = 105,
                        int stack_sz = CONFIG_PTHREAD_STACK_DEFAULT);
    virtual ~audiolite_component();

    void set_mempool(audiolite_mempool *pool);

    /* Call backs */

    int start(audiolite_inputnode *node);
    void cancel(audiolite_inputnode *node);
    void stop(audiolite_inputnode *node);
    void reflesh(audiolite_inputnode *node);

    int start(audiolite_outputnode *node);
    void cancel(audiolite_outputnode *node);
    void stop(audiolite_outputnode *node);
    void reflesh(audiolite_outputnode *node);

    void data_came(mossfw_callback_op_t *op);

    virtual bool can_breakdata(audiolite_outputnode *out);
    virtual void on_data();
    virtual int on_starting(audiolite_inputnode *inode,
                            audiolite_outputnode *onode);
    virtual void on_started(audiolite_inputnode *inode,
                           audiolite_outputnode *onode);
    virtual void on_canceled(audiolite_inputnode *inode,
                            audiolite_outputnode *onode);
    virtual void on_stopping(audiolite_inputnode *inode,
                             audiolite_outputnode *onode);
    virtual void on_stop(audiolite_inputnode *inode,
                        audiolite_outputnode *onode);

    int publish_event(int evtid, unsigned long arg); 
    int samplingrate();
    int channels();
    int samplebitwidth();

    audiolite_inputnode *get_input(int id = 0);
    audiolite_outputnode *get_output(int id = 0);

    audiolite_component *bind(audiolite_component *cmp);
    int bind(audiolite_inputnode *in, int outid = 0);

    int unbind(audiolite_inputnode *in);
    int unbindall();

    virtual int start();
    virtual void stop();
    virtual void suspend();
    virtual int resume();
};

#endif /* __INCLUDE_AUDIOLITE_COMPONENT_H */
