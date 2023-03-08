/****************************************************************************
 * modules/include/audiolite/al_inputnode.h
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

#ifndef __INCLUDE_AUDIOLITE_OUTPUTNODE_H
#define __INCLUDE_AUDIOLITE_OUTPUTNODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <mossfw/mossfw_component.h>

#include <audiolite/al_memalloc.h>
#include <audiolite/al_nodecomm.h>

/****************************************************************************
 * Class Pre-definitions
 ****************************************************************************/

class audiolite_inputnode;
class audiolite_component;
class audiolite_outputnode;

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_outputnode
 ****************************************************************************/

class audiolite_outputnode
{
  private:
    audiolite_component *_component;
    audiolite_inputnode *_binded;
    audiolite_nodecomm _comm;
    mossfw_output_t *_output;
    mossfw_lock_t _lock;

  public:
    audiolite_outputnode(audiolite_component *cmp);
    ~audiolite_outputnode();

    int bind(audiolite_inputnode *in);
    int unbind(audiolite_inputnode *in);
    int unbind();
    bool can_breakdata();
    int push_data(audiolite_mem *mem);
    int fanout();

  friend class audiolite_component;
  friend int audiolite_start(audiolite_outputnode *node);
  friend void audiolite_stop(audiolite_outputnode *node);
  friend void audiolite_reflesh(audiolite_outputnode *node);
  friend int audiolite_start(audiolite_inputnode *node);
  friend void audiolite_stop(audiolite_inputnode *node);
  friend void audiolite_reflesh(audiolite_inputnode *node);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int audiolite_start(audiolite_outputnode *node);
void audiolite_stop(audiolite_outputnode *node);
void audiolite_reflesh(audiolite_outputnode *node);

#endif /* __INCLUDE_AUDIOLITE_OUTPUTNODE_H */
