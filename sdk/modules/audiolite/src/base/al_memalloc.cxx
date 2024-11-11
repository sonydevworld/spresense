/****************************************************************************
 * modules/audiolite/src/base/al_memalloc.cxx
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <semaphore.h>

#include <nuttx/audio/audio.h>

#include <mossfw/mossfw_lock.h>
#include <mossfw/mossfw_data.h>

#include <audiolite/al_debug.h>
#include <audiolite/al_memalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALIGNMENT(m)  (((m + 3) / 4) * 4)

/****************************************************************************
 * Class: audiolite_timeprofile
 ****************************************************************************/

/***********************************************
 * Public Class audiolite_timeprofile Methods
 ***********************************************/

void audiolite_timeprofile::reflesh(int rem)
{
  min_remain = rem;
  max_zero_timeus = 0;
  measure_interval();
}

void audiolite_timeprofile::update_remain(int rem)
{
  if (min_remain > rem) min_remain = rem;
}

uint32_t audiolite_timeprofile::measure_interval()
{
  struct timeval current;
  struct timeval duration;

  gettimeofday(&current, NULL);
  timersub(&current, &timekeeper, &duration);
  timekeeper = current;
  return (duration.tv_sec * 1000000 + duration.tv_usec);
}

void audiolite_timeprofile::measure_start()
{
  measure_interval();
}

void audiolite_timeprofile::measure_stop()
{
  uint32_t tmp;
  tmp = (uint64_t)measure_interval();

  if (max_zero_timeus < tmp)
    {
      max_zero_timeus = tmp;
    }
}

/****************************************************************************
 * Class: audiolite_mem
 ****************************************************************************/

/***********************************************
 * Protected Class audiolite_mem Methods
 ***********************************************/

void audiolite_mem::setup_instance(int sz, char *mem,
                                audiolite_mempool *pool)
{
  this->refcnt = 0;
  this->data_bytes = 0;
  this->timestamp = 0;
  this->fs = 0;
  this->_alloc.priv = (void *)pool;
  this->data.xc = (mossfw_data_v1c_t *)mem;
  this->_sz = sz;
}

int audiolite_mem::unrefer()
{
  int rcnt;

  mossfw_lock_take(&this->lock);
  rcnt = --this->refcnt;
  mossfw_lock_give(&this->lock);

  return rcnt;
}

/***********************************************
 * Private Class audiolite_mem Methods
 ***********************************************/

audiolite_mem *audiolite_mem::operator=(audiolite_mem *mem)
{
  return mem->reference();
}

/***********************************************
 * Private Static Class audiolite_mem Methods
 ***********************************************/

void audiolite_mem::mempoolmossfw_free(void *priv, mossfw_data_t *mem)
{
  audiolite_mem *m = (audiolite_mem *)mem;
  audiolite_mempool *pool = (audiolite_mempool *)priv;

  if (m->unrefer() <= 0)
    {
      pool->memfree(m);
    }
}

/***********************************************
 * Public Class audiolite_mem Methods
 ***********************************************/

audiolite_mem::audiolite_mem(void)
{
  _alloc.free = audiolite_mem::mempoolmossfw_free;
  _alloc.priv = NULL;
  this->allocator = &_alloc;
  mossfw_lock_init(&this->lock);
  setup_instance(0, NULL, NULL);
};

void audiolite_mem::release(void)
{
  this->allocator->free(this->allocator->priv, this);
}

audiolite_mem *audiolite_mem::reference(void)
{
  mossfw_lock_take(&this->lock);
  this->refcnt++;
  mossfw_lock_give(&this->lock);

  return this;
}

int audiolite_mem::get_typebytes(void)
{
  /* TODO: Now data bytes is fixed as 2 */

  return 2;
}

/****************************************************************************
 * Class: audiolite_memapbuf
 ****************************************************************************/

audiolite_memapbuf::audiolite_memapbuf(void) : audiolite_mem()
{
  memset(&_abuf, 0, sizeof(_abuf));
  _abuf.i.samplerate = 0;
  _abuf.i.channels   = 1;
  _abuf.crefs        = 1;
  _abuf.nmaxbytes    = 0;
  _abuf.nbytes       = 0;
  _abuf.flags        = 0;
  _abuf.samp         = NULL;
}

/***********************************************
 * Public Class audiolite_memapbuf Methods
 ***********************************************/

void audiolite_memapbuf::setup_instance(int sz, char *mem,
                            audiolite_mempool *pool)
{
  audiolite_mem::setup_instance(sz, mem, pool);
  _abuf.nmaxbytes = sz;
  _abuf.samp = (uint8_t *)mem;
}

void audiolite_memapbuf::reset_audiodata(void)
{
  _abuf.curbyte = 0;
  _abuf.nbytes = 0;
  _abuf.flags = 0;
  set_storedsize(0);
}

void audiolite_memapbuf::set_fs(int hz)
{
  _abuf.i.samplerate = hz;
  audiolite_mem::set_fs(hz);
}

void audiolite_memapbuf::set_storedsize(int sz)
{
  _abuf.nbytes = sz;
  audiolite_mem::set_storedsize(sz);
}

void audiolite_memapbuf::set_channels(int ch)
{
  _abuf.i.channels = ch;
}

int audiolite_memapbuf::get_channels(void)
{
  return (int)_abuf.i.channels;
}

void audiolite_memapbuf::set_eof()
{
  _abuf.flags |= AUDIO_APB_FINAL;
}

void audiolite_memapbuf::clear_eof()
{
  _abuf.flags &= (~AUDIO_APB_FINAL);
}

bool audiolite_memapbuf::is_eof()
{
  return (!(!(_abuf.flags & AUDIO_APB_FINAL)));
}

struct ap_buffer_s *audiolite_memapbuf::get_raw_abuf()
{
  return &_abuf;
}

dq_entry_t *audiolite_memapbuf::get_link(void)
{
  return &_abuf.dq_entry;
}

audiolite_memapbuf *audiolite_memapbuf::local_cast(dq_entry_t *ent)
{
  char *tmp = (char *)ent;

  if (tmp)
    {
      tmp = tmp - (uintptr_t)&(((audiolite_memapbuf *)(0))->_abuf);
    }

  return (audiolite_memapbuf *)tmp;
}

/****************************************************************************
 * class: audiolite_mempoolapbuf
 ****************************************************************************/

/***********************************************
 * Private Class audiolite_mempoolapbuf Methods
 ***********************************************/

void audiolite_mempoolapbuf::_add_freemem(audiolite_memapbuf *mem)
{
  mossfw_lock_take(&_lock);

  dq_addlast(mem->get_link(), &_free_mem);

  if (dq_count(&_free_mem) == 1)
    {
      /* Free mem block is one after memfree() means
       * the ZERO period is finished.
       * So stop measure of zero period.
       */

      measure_stop();
    }

  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

bool audiolite_mempoolapbuf::_create_instance(int block_size, int block_num,
                      char *mem, int memsize)
{
  int i;

  if (_insts)
    {
      delete [] _insts;
      _insts = NULL;
    }

  block_size = ALIGNMENT(block_size);
  _insts = new audiolite_memapbuf[block_num];
  if (_insts)
    {
      for (i = 0; i < block_num; i++)
        {
          _insts[i].setup_instance(block_size, &mem[block_size * i], this);
          _add_freemem(&_insts[i]);
        }

      _blknum = block_num;
      reflesh(_blknum);

      return true;
    }

  return false;
}

/***********************************************
 * Public Class audiolite_mempoolapbuf Methods
 ***********************************************/

audiolite_mempoolapbuf::audiolite_mempoolapbuf(void) : _insts(NULL), _blknum(0), _ownmem(NULL), _pool_enable(true)
{
  dq_init(&_free_mem);
  mossfw_lock_init(&_lock);
  mossfw_condition_init(&_cond);
}

audiolite_mempoolapbuf::~audiolite_mempoolapbuf(void)
{
  if (_insts)
    {
      delete [] _insts;
    }

  if (_ownmem)
    {
      free(_ownmem);
    }

  mossfw_lock_fin(&_lock);
}

bool audiolite_mempoolapbuf::create_instance(int block_size, int block_num)
{
  bool ret = false;
  int memsize = ALIGNMENT(block_size) * block_num;

  if (_ownmem)
    {
      free(_ownmem);
      _ownmem = NULL;
    }

  _ownmem = (char *)malloc(memsize);
  if (_ownmem != NULL)
    {
      ret = _create_instance(block_size, block_num, _ownmem, memsize);
      if (!ret)
        {
          free(_ownmem);
        }
    }

  return ret;
}

bool audiolite_mempoolapbuf::create_instance(int block_size, int block_num,
                     char *mem, int memsize)
{
  if (memsize >= (ALIGNMENT(block_size) * block_num))
    {
      if (_ownmem)
        {
          free(_ownmem);
          _ownmem = NULL;
        }

      return _create_instance(block_size, block_num, mem, memsize);
    }

  return false;
}

audiolite_mem *audiolite_mempoolapbuf::allocate(bool blocking)
{
  int qsz;
  audiolite_memapbuf *ret = NULL;
  dq_entry_t *tmp = NULL;

  mossfw_lock_take(&_lock);

  qsz = dq_count(&_free_mem);
  if (qsz == 1)
    {
      /* Remaining free memory block is 1.
       * And called allocate(), so it will be ZERO.
       * Start measure zero period then.
       */

      measure_start();
    }

  if (_pool_enable)
    {
      tmp = dq_remfirst(&_free_mem);
      while (_pool_enable && blocking && tmp == NULL)
        {
          mossfw_condition_wait(&_cond, &_lock);
          if (_pool_enable)
            {
              tmp = dq_remfirst(&_free_mem);
            }
        }
    }

  update_remain(qsz);
  mossfw_lock_give(&_lock);

  if (tmp)
    {
      ret = audiolite_memapbuf::local_cast(tmp);
      ret->reset_audiodata();
      ret->reference();
    }

  return ret;
}

void audiolite_mempoolapbuf::memfree(audiolite_mem *mem)
{
  audiolite_memapbuf *tmp = (audiolite_memapbuf *)(mem);
  _add_freemem(tmp);
}

void audiolite_mempoolapbuf::disable_pool()
{
  mossfw_lock_take(&_lock);
  _pool_enable = false;
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

void audiolite_mempoolapbuf::enable_pool()
{
  mossfw_lock_take(&_lock);
  _pool_enable = true;
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

int audiolite_mempoolapbuf::remaining()
{
  int ret;
  mossfw_lock_take(&_lock);
  ret = dq_count(&_free_mem);
  mossfw_lock_give(&_lock);

  return ret;
}

/****************************************************************************
 * class: audiolite_sysmsg
 ****************************************************************************/

audiolite_sysmsg::audiolite_sysmsg(void) : audiolite_mem()
{
  set_msgcontent(0, NULL, 0);
}

void audiolite_sysmsg::setup_instance(audiolite_mempool *pool)
{
  audiolite_mem::setup_instance(sizeof(audiolite_sysmsg), NULL, pool);
  this->data_bytes = sizeof(audiolite_sysmsg);
}

void audiolite_sysmsg::set_msgcontent(int evtid,
                                         void *issuer,
                                         unsigned long arg)
{
  _msg.evtid = evtid;
  _msg.issuer = issuer;
  _msg.arg = arg;
}

audiolite_sysmsg *audiolite_sysmsg::local_cast(dq_entry_t *ent)
{
  char *tmp = (char *)ent;

  if (tmp)
    {
      tmp = tmp - (uintptr_t)&(((audiolite_sysmsg *)(0))->_msg);
    }

  return (audiolite_sysmsg *)tmp;
}

/****************************************************************************
 * class: audiolite_mempoolsysmsg
 ****************************************************************************/

void audiolite_mempoolsysmsg::_add_freemem(audiolite_sysmsg *mem)
{
  mossfw_lock_take(&_lock);

  dq_addlast(mem->get_link(), &_free_mem);

  if (dq_count(&_free_mem) == 1)
    {
      /* Free mem block is one after memfree() means
       * the ZERO period is finished.
       * So stop measure of zero period.
       */

      measure_stop();
    }

  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

audiolite_mempoolsysmsg::audiolite_mempoolsysmsg(void) :
    _insts(NULL), _msgnum(0), _pool_enable(true)
{
  dq_init(&_free_mem);
  mossfw_lock_init(&_lock);
  mossfw_condition_init(&_cond);
}

audiolite_mempoolsysmsg::~audiolite_mempoolsysmsg(void)
{
  if (_insts)
    {
      delete [] _insts;
    }

  mossfw_lock_fin(&_lock);
}

bool audiolite_mempoolsysmsg::create_instance(int msgnum)
{
  int i;

  if (_insts)
    {
      delete [] _insts;
      _insts = NULL;
    }

  _msgnum = msgnum;
  _insts = new audiolite_sysmsg[msgnum];
  if (_insts)
    {
      for (i = 0; i < msgnum; i++)
        {
          _insts[i].setup_instance(this);
          _add_freemem(&_insts[i]);
        }

      reflesh(msgnum);

      return true;
    }

  return false;
}

audiolite_mem *audiolite_mempoolsysmsg::allocate(bool blocking)
{
  int qsz;
  audiolite_sysmsg *ret = NULL;
  dq_entry_t *tmp;

  mossfw_lock_take(&_lock);

  qsz = dq_count(&_free_mem);
  if (qsz == 1)
    {
      /* Remaining free memory block is 1.
       * And called allocate(), so it will be ZERO.
       * Start measure zero period then.
       */

      measure_start();
    }

  tmp = dq_remfirst(&_free_mem);
  while (_pool_enable && blocking && tmp == NULL)
    {
      mossfw_condition_wait(&_cond, &_lock);
      tmp = dq_remfirst(&_free_mem);
    }

  update_remain(qsz);
  mossfw_lock_give(&_lock);

  if (tmp)
    {
      ret = audiolite_sysmsg::local_cast(tmp);
      ret->set_msgcontent(0, NULL, 0);
      ret->reference();
    }

  return ret;
}

void audiolite_mempoolsysmsg::memfree(audiolite_mem *mem)
{
  audiolite_sysmsg *tmp = (audiolite_sysmsg *)(mem);
  _add_freemem(tmp);
}

void audiolite_mempoolsysmsg::disable_pool()
{
  mossfw_lock_take(&_lock);
  _pool_enable = false;
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

void audiolite_mempoolsysmsg::enable_pool()
{
  mossfw_lock_take(&_lock);
  _pool_enable = true;
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

int audiolite_mempoolsysmsg::remaining()
{
  int ret;
  mossfw_lock_take(&_lock);
  ret = dq_count(&_free_mem);
  mossfw_lock_give(&_lock);

  return ret;
}
