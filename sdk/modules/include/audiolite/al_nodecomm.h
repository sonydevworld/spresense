/****************************************************************************
 * modules/include/audiolite/al_nodecomm.h
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

#ifndef __INCLUDE_AUDIOLITE_NODECOMM_H
#define __INCLUDE_AUDIOLITE_NODECOMM_H

#include <stdlib.h>

/****************************************************************************
 * Class Pre-definitions
 ****************************************************************************/

class audiolite_inputnode;
class audiolite_outputnode;

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_nodecomm_if
 ****************************************************************************/

class audiolite_nodecomm_if
{
  public:
    virtual ~audiolite_nodecomm_if(){};
    virtual int start(audiolite_inputnode *node) = 0;
    virtual void cancel(audiolite_inputnode *node) = 0;
    virtual void stop(audiolite_inputnode *node) = 0;
    virtual int start(audiolite_outputnode *node) = 0;
    virtual void cancel(audiolite_outputnode *node) = 0;
    virtual void stop(audiolite_outputnode *node) = 0;

    virtual void reflesh(audiolite_inputnode *node) = 0;
    virtual void reflesh(audiolite_outputnode *node) = 0;
};

/****************************************************************************
 * class: audiolite_nodecomm
 ****************************************************************************/

class audiolite_nodecomm
{
  private:
    audiolite_nodecomm_if *_if;
    audiolite_inputnode *_innode;
    audiolite_outputnode *_outnode;

  public:
    audiolite_nodecomm(audiolite_nodecomm_if *i, audiolite_inputnode *n)
        : _if(i), _innode(n), _outnode(NULL) {};
    audiolite_nodecomm(audiolite_nodecomm_if *i, audiolite_outputnode *n)
        : _if(i), _innode(NULL), _outnode(n) {};

    int start(){ if (_if) return _if->start(_innode); return 0;};
    void cancel(){ _if->cancel(_innode); };
    void stop(){ _if->stop(_innode); };
    void reflesh(){ _if->reflesh(_innode); };

    int rstart(){ if (_if) return _if->start(_outnode); return 0;};
    void rcancel(){ if (_if) _if->cancel(_outnode); };
    void rstop(){ if (_if) _if->stop(_outnode); };
    void rreflesh(){ if (_if) _if->reflesh(_outnode); };
};

#endif /* __INCLUDE_AUDIOLITE_NODECOMM_H */
