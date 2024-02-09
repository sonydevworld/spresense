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

#include <errno.h>
#include <mqueue.h>
#include <mossfw/mossfw_lock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALDRV_MODE_OUTPUT (1)
#define ALDRV_MODE_INPUT  (2)

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
  protected:
    int _fd;
    int _mode;
    int _volume;
    mqd_t _mq;
    mossfw_thread_t _tid;
    audiolite_drvlistener *_listener;
    mossfw_lock_t _lock;

    int _fs;
    int _bps;
    int _chnum;
    int _maxch;
    const char *_devpath;
    const char *_mqpath;
    int _is_started;

    void cleanup_messageq();
    int update_audioparam();
    int stop_thread(void);
    int setup_driver();
    int set_volume_nolock(int vol);
    int single_ioctl(int ioc);
    int set_audioparam(int fs, int bps, int chnum);

  public:
    audiolite_driver(int mode, const char *devpath,
                     const char *mqpath, int maxch);
    ~audiolite_driver();

    void set_listener(audiolite_drvlistener *l){ _listener = l; };
    int set_volume(int vol);

    int enqueue_buffer(FAR struct ap_buffer_s *apb);
    int start(int fs, int bps, int chnum);
    int stop(void);
    int pause(void);
    int resume(void);

    void reset(void);

    static void *message_thread(void *arg);
};

#endif /* __INCLUDE_AUDIOLITE_AUDIODRV_H */
