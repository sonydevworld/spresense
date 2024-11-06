/****************************************************************************
 * modules/include/audiolite/al_memalloc.h
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

#ifndef __INCLUDE_AUDIOLITE_MEMALLOC_H
#define __INCLUDE_AUDIOLITE_MEMALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <sys/time.h>

#include <nuttx/audio/audio.h>

#include <mossfw/mossfw_memoryallocator.h>
#include <mossfw/mossfw_lock.h>
#include <mossfw/mossfw_data.h>

/****************************************************************************
 * Class Pre-definitions
 ****************************************************************************/

class audiolite_mem;

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_timeprofile
 ****************************************************************************/

class audiolite_timeprofile
{
  public:
    struct timeval timekeeper;
    int min_remain;
    uint32_t max_zero_timeus;

    audiolite_timeprofile(void)
    {
      reflesh(0);
    };

    void reflesh(int rem);
    void update_remain(int rem);

    uint32_t measure_interval();
    void measure_start();
    void measure_stop();

    uint32_t zero_time() { return max_zero_timeus; };
    int minimum_remain() { return min_remain; };
};

/****************************************************************************
 * class: audiolite_mempool
 ****************************************************************************/

class audiolite_mempool : public audiolite_timeprofile
{
  public:
    virtual ~audiolite_mempool(){};
    virtual audiolite_mem *allocate(bool blocking = true) = 0;
    virtual void memfree(audiolite_mem *mem) = 0;
    virtual void disable_pool() = 0;
    virtual void enable_pool() = 0;
    virtual int remaining() = 0;
};

/****************************************************************************
 * class: audiolite_mem
 ****************************************************************************/

class audiolite_mem : public mossfw_data_t
{
  private:
    mossfw_allocator_t _alloc;

  protected:
    int _sz;

  public:
    audiolite_mem(void);
    virtual ~audiolite_mem(void){};

  protected:
    virtual void setup_instance(int sz, char *mem,
                                audiolite_mempool *pool);
    int unrefer();

  private:
    audiolite_mem *operator=(audiolite_mem *mem);
    static void mempoolmossfw_free(void *priv, mossfw_data_t *mem);

  public:

    void release(void);
    virtual audiolite_mem *reference(void);

    void *get_data(void) { return (void *)data.xc; };
    int get_fs(void) { return fs; };
    void set_fs(int hz) { fs = hz; };
    int get_storedsize(void) { return data_bytes; };
    void set_storedsize(int sz) { data_bytes = sz; };
    int get_fullsize(void) { return _sz; };
    int get_typebytes(void);
    virtual void set_eof() = 0;
    virtual void clear_eof() = 0;
    virtual bool is_eof() = 0;

  friend audiolite_mempool;
};

/****************************************************************************
 * class: audiolite_memapbuf
 ****************************************************************************/

class audiolite_memapbuf : public audiolite_mem
{
  protected:
    struct ap_buffer_s _abuf;

  public:
    audiolite_memapbuf(void);
    void setup_instance(int sz, char *mem,
                                audiolite_mempool *pool);
    void reset_audiodata(void);
    void set_fs(int hz);
    void set_storedsize(int sz);
    void set_channels(int ch);
    int  get_channels(void);
    void set_eof();
    void clear_eof();
    bool is_eof();
    struct ap_buffer_s *get_raw_abuf();
    dq_entry_t *get_link(void);

    static audiolite_memapbuf *local_cast(dq_entry_t *ent);
};

/****************************************************************************
 * class: audiolite_mempoolapbuf
 ****************************************************************************/

class audiolite_mempoolapbuf : public audiolite_mempool
{
  private:
    audiolite_memapbuf *_insts;
    struct dq_queue_s _free_mem;
    int _blknum;
    char * _ownmem;
    bool _pool_enable;
    mossfw_lock_t _lock;
    mossfw_condition_t _cond;

    void _add_freemem(audiolite_memapbuf *mem);
    bool _create_instance(int block_size, int block_num,
                          char *mem, int memsize);

  public:
    audiolite_mempoolapbuf(void);
    ~audiolite_mempoolapbuf(void);
    bool create_instance(int block_size, int block_num);
    bool create_instance(int block_size, int block_num,
                         char *mem, int memsize);
    virtual audiolite_mem *allocate(bool blocking = true);
    virtual void memfree(audiolite_mem *mem);
    void disable_pool();
    void enable_pool();
    int remaining();
};

/****************************************************************************
 * class: audiolite_sysmsg
 ****************************************************************************/

struct audiolite_evtmsg_s
{
  dq_entry_t link;
  int evtid;
  void *issuer;
  unsigned long arg;
};

class audiolite_sysmsg : public audiolite_mem
{
  protected:
    struct audiolite_evtmsg_s _msg;

  public:
    audiolite_sysmsg(void);

    void setup_instance(audiolite_mempool *pool);
    int get_evtid() { return _msg.evtid; };
    void *get_issuer() { return _msg.issuer; };
    unsigned long get_arg() { return _msg.arg; };
    void set_msgcontent(int evtid, void *issuer, unsigned long arg);

    dq_entry_t *get_link(void) { return &_msg.link; };

    void set_eof() {};
    void clear_eof() {};
    bool is_eof() { return false; };

    static audiolite_sysmsg *local_cast(dq_entry_t *ent);
};

/****************************************************************************
 * class: audiolite_mempoolsysmsg
 ****************************************************************************/

class audiolite_mempoolsysmsg : public audiolite_mempool
{
  private:
    audiolite_sysmsg *_insts;
    int _msgnum;
    struct dq_queue_s _free_mem;
    bool _pool_enable;
    mossfw_lock_t _lock;
    mossfw_condition_t _cond;

    void _add_freemem(audiolite_sysmsg *mem);

  public:
    audiolite_mempoolsysmsg(void);
    ~audiolite_mempoolsysmsg(void);
    bool create_instance(int msgnum);
    virtual audiolite_mem *allocate(bool blocking = true);
    virtual void memfree(audiolite_mem *mem);
    void disable_pool();
    void enable_pool();
    int remaining();
};

#endif /* __INCLUDE_AUDIOLITE_MEMALLOC_H */
