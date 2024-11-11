/****************************************************************************
 * modules/audiolite/src/base/al_worker.cxx
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

#include <nuttx/config.h>

#include <errno.h>
#include <audiolite/al_debug.h>
#include <audiolite/al_worker.h>

/****************************************************************************
 * class: audiolite_workermemq
 ****************************************************************************/

audiolite_workermemq::audiolite_workermemq(int maxsz)
                              : _max_qsz(maxsz), _en(true)
{
  mossfw_lock_init(&_lock);
  mossfw_condition_init(&_cond);
  dq_init(&_mem_proc);
}

audiolite_workermemq::~audiolite_workermemq()
{
  audiolite_memapbuf *mem;
  disable();

  mem = pop();
  while (mem)
    {
      mem->release();
      mem = pop();
    }
}

void audiolite_workermemq::enable()
{
  mossfw_lock_take(&_lock);
  _en = true;
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

void audiolite_workermemq::disable()
{
  mossfw_lock_take(&_lock);
  _en = false;
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);
}

void audiolite_workermemq::push(audiolite_memapbuf *mem)
{
  al_dinfo("[%08x] pushed %d : %08x\n", this, dq_count(&_mem_proc), mem);

  mossfw_lock_take(&_lock);

  while (_en && dq_count(&_mem_proc) >= (uint32_t)_max_qsz)
    {
      mossfw_condition_wait(&_cond, &_lock);
    }

  if (_en)
    {
      dq_addlast(mem->get_link(), &_mem_proc);
    }
  else
    {
      mem->release();
    }

  mossfw_lock_give(&_lock);
}

audiolite_memapbuf *audiolite_workermemq::pop()
{
  audiolite_memapbuf *mem = NULL;
  dq_entry_t *tmp;

  mossfw_lock_take(&_lock);
  tmp = dq_remfirst(&_mem_proc);
  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);

  if (tmp)
    {
      mem = audiolite_memapbuf::local_cast(tmp);
    }

  al_dinfo("[%08x] poped %d : %08x\n", this, dq_count(&_mem_proc), mem);
  return mem;
}

audiolite_memapbuf *audiolite_workermemq::pop(unsigned char *adr)
{
  audiolite_memapbuf *mem = NULL;
  dq_entry_t *tmp;

  mossfw_lock_take(&_lock);

  for (tmp = dq_peek(&_mem_proc); tmp; tmp = tmp->blink)
    {
      mem = audiolite_memapbuf::local_cast(tmp);
      if (adr == alworker_addr_convert(mem->get_data()))
        {
          dq_rem(tmp, &_mem_proc);
          break;
        }
      mem = NULL;
    }

  mossfw_condition_notice(&_cond);
  mossfw_lock_give(&_lock);

  al_dinfo("[%08x] poped with addr (%08x) %d : %08x\n",
           this, adr, dq_count(&_mem_proc), mem);
  return mem;
}

/****************************************************************************
 * class: audiolite_worker
 ****************************************************************************/

void *audiolite_worker::msg_receiver(void *arg)
{
  int ret = 0;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;
  audiolite_worker *alw = (audiolite_worker *)arg;

  // while (alw->is_running())
  while (alw->_running)
    {
      hdr = al_receive_messageto(&alw->_wtask, &opt, 10);

      mossfw_lock_take(&alw->_lock);

      if (hdr.u32 != AL_COMM_NO_MSG && alw->_hdlr)
        {
          ret = alw->_hdlr(hdr, &opt, alw->_hdlr_arg);
        }

      mossfw_lock_give(&alw->_lock);

      if (ret != 0)
        {
          break;
        }

      pthread_yield();
    }

  return NULL;
}

bool audiolite_worker::is_running()
{
  bool ret;
  mossfw_lock_take(&_lock);
  ret = _running;
  mossfw_lock_give(&_lock);
  return ret;
}

void audiolite_worker::mod_running(bool running)
{
  mossfw_lock_take(&_lock);
  _running = running;
  mossfw_lock_give(&_lock);
}

int audiolite_worker::wait_bootmsg()
{
  return OK;
}

audiolite_worker::audiolite_worker() : _tid(-1), _hdlr_arg(NULL),
                                       _hdlr(NULL), _running(false)
{
  mossfw_lock_init(&_lock);
}

audiolite_worker::~audiolite_worker()
{
  terminate_worker();
}

int audiolite_worker::bringup_worker(const char *dspname, bool is_spk,
                                     const char *rcvname, int prio,
                                     int stksz)
{
  int ret = -EINVAL;

  if (_hdlr != NULL && _tid == -1)
    {
      ret = initialize_alworker(&_wtask, dspname, is_spk);
      if (ret == AL_COMM_ERR_SUCCESS)
        {
          ret = wait_bootmsg();
          if (ret == 0)
            {
              mod_running(true);
              ret = mossfw_create_thread_attr(&_tid,
                                  audiolite_worker::msg_receiver,
                                  this, prio, stksz);
              if (ret == 0)
                {
                  pthread_setname_np(_tid, rcvname);
                }
              else
                {
                  _tid = -1;
                  mod_running(false);
                }
            }
          else
            {
              finalize_alworker(&_wtask);
            }
        }
    }

  return ret;
}

void audiolite_worker::terminate_worker()
{
  if (_tid >= 0)
    {
      mod_running(false);
      mossfw_thread_join(&_tid);
      finalize_alworker(&_wtask);
      _tid = -1;
    }
}

void audiolite_worker::set_msghandler(alwkr_msghandler_t hdr, void *arg)
{
  mossfw_lock_take(&_lock);
  _hdlr = hdr;
  _hdlr_arg = arg;
  mossfw_lock_give(&_lock);
}
