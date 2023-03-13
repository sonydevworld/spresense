/****************************************************************************
 * modules/include/audiolite/al_audiodrv.h
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

#ifndef __INCLUDE_AUDIOLITE_AUDIODRV_H
#define __INCLUDE_AUDIOLITE_AUDIODRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <mqueue.h>
#include <mossfw/mossfw_lock.h>

#include <audiolite/al_singleton.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_drvlistener
 ****************************************************************************/

class audiolite_drvlistener
{
  public:
    virtual ~audiolite_drvlistener(){};
    virtual void on_pusheddata(FAR struct ap_buffer_s *apb) = 0;
    virtual void on_popeddata(FAR struct ap_buffer_s *apb) = 0;
    virtual void on_stopped(void) = 0;
    virtual void on_overflowed(void) = 0;
    virtual void on_underflowed(void) = 0;
};

/****************************************************************************
 * class: audiolite_driver
 ****************************************************************************/

class audiolite_driver
{
  private:
    SINGLETON_MEMBER(audiolite_driver);
    int _fd;
    int _mode;
    mqd_t _mq;
    mossfw_thread_t _tid;
    audiolite_drvlistener *_listener;
    int _enq_cnt;

    audiolite_driver();
    ~audiolite_driver();

    int start_driver(void);
    int stop_thread(void);
    int setup_driver(audiolite_drvlistener *l, const char *devpath);

  public:
    SINGLETON_METHODS(audiolite_driver);

    int as_output(audiolite_drvlistener *l);
    int as_input(audiolite_drvlistener *l);

    int set_audioparam(int fs, int bps, int chnum);
    int enqueue_buffer(FAR struct ap_buffer_s *apb);
    int set_volume(int vol);
    int start(void);
    int stop(void);

    void reset(void);

    static void *message_thread(void *arg);
};

#endif /* __INCLUDE_AUDIOLITE_AUDIODRV_H */
