/****************************************************************************
 * modules/include/audiolite/al_decoder.h
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

#ifndef __INCLUDE_AUDIOLITE_DECODER_H
#define __INCLUDE_AUDIOLITE_DECODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <audiolite/al_component.h>
#include <audiolite/al_stream.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_decoder
 ****************************************************************************/

class audiolite_decoder : public audiolite_component
{
  protected:
    audiolite_stream *_stream;
    audiolite_mempoolapbuf *_omempool;
    int _prio;
    int _stacksz;
    mossfw_thread_t _tid;
    const char *_tname;
    volatile bool _isplay;
    volatile bool _ispause;
    volatile bool _is_thrdrun;

    int start_thread(const char *name);
    void stop_thread();
    virtual void decode_runner() = 0;

    static void *inject_worker(void *arg);

  public:
    audiolite_decoder(const char *name,
                      int prio = -1, int stack_sz = -1);
    ~audiolite_decoder();

    int start();
    void stop();
    void pause();
    int resume();

    virtual int start_decode() = 0;
    virtual int stop_decode() = 0;
    virtual int pause_decode() = 0;
    virtual int resume_decode() = 0;

    void set_stream(audiolite_stream *st) { _stream = st; };
    void set_outputmempool(audiolite_mempoolapbuf *pool)
    {
      _omempool = pool;
    }
};

#endif  /* __INCLUDE_AUDIOLITE_DECODER_H */
