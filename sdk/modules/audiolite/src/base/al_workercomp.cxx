/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <audiolite/al_eventlistener.h>
#include <audiolite/al_workercmd.h>
#include <audiolite/al_workercomp.h>

/****************************************************************************
 * Class Method Functions
 ****************************************************************************/

void audiolite_stdworker_msglistener::push_data(audiolite_workercomp *wcomp,
                                                audiolite_memapbuf *mem)
{
  wcomp->push_data(mem);
}

int audiolite_stdworker_msglistener::get_inqsize(audiolite_workercomp *wcomp)
{
  return wcomp->_inq.get_qsize();
}

int audiolite_stdworker_msglistener::get_outqsize(audiolite_workercomp *wcomp)
{
  return wcomp->_outq.get_qsize();
}

audiolite_memapbuf *audiolite_stdworker_msglistener::allocate(audiolite_workercomp *wcomp)
{
  return (audiolite_memapbuf *)wcomp->_pool->allocate();
}

audiolite_workercomp::audiolite_workercomp(const char *workername,
                                           int inqsz, int outqsz,
                                           int inputnum, int outputnum,
                                           int depth, int prio, int stack_sz)
  : audiolite_component(inputnum, outputnum, depth, true, prio, stack_sz),
    _inq(inqsz), _outq(outqsz), _worker_booted(false),
    _worker_terminated(true), _lsnr(NULL), _thd(this, this, 105, 2048)
{
  if (workername != NULL)
    {
      strncpy(_workerspk, workername, WORKERCOMP_SPKNAMESZ - 1);
    }

  _worker.set_msghandler(audiolite_workercomp::handle_message, this);
}

audiolite_workercomp::~audiolite_workercomp()
{
}

int audiolite_workercomp::handle_message(al_comm_msghdr_t hdr,
                                         al_comm_msgopt_t *opt,
                                         void *arg)
{
  int ret = 0;
  audiolite_workercomp *thiz = (audiolite_workercomp *)arg;
  audiolite_stdworker_msglistener *listener = thiz->_lsnr;
  al_wtask_t *wtask = thiz->_worker.getwtask();
  audiolite_memapbuf *mem;
  audiolite_memapbuf *nmem;

  if (listener == NULL)
    {
      return -1;
    }

  if (CHECK_HDR(hdr, SYS, SYS_BOOT))
    {
      listener->bootup(thiz, wtask, hdr.opt, opt->addr);
      thiz->_worker_booted = true;
    }
  else if (CHECK_HDR(hdr, SYS, SYS_ERR))
    {
      listener->error(thiz, wtask, hdr.opt, opt->errcode);
    }
  else if (CHECK_HDR(hdr, SYS, SYS_PLAY))
    {
      /* Just received. Do nothing. */
    }
  else if (CHECK_HDR(hdr, SYS, SYS_PARAM))
    {
      /* Just received. Do nothing. */
    }
  else if (CHECK_HDR(hdr, SYS, SYS_DBG))
    {
      listener->debug(thiz, wtask, hdr.opt);
    }
  else if (CHECK_HDR(hdr, INST, INST_INFO))
    {
      listener->info(thiz, wtask, hdr.opt, opt->dec_chs, opt->hz,
                     opt->dec_layer, opt->dec_kbps);
    }
  else if (CHECK_HDR(hdr, INST, INST_DONE))
    {
      listener->done(thiz, wtask, hdr.opt);
    }
  else if (CHECK_HDR(hdr, SYS, SYS_TERM))
    {
      listener->term(thiz);
      thiz->_worker_terminated = true;

      /* Release all memory */

      for (mem = thiz->_inq.pop(); mem; mem = thiz->_inq.pop())
        {
          mem->release();
        }

      for (mem = thiz->_outq.pop(); mem; mem = thiz->_outq.pop())
        {
          mem->release();
        }

      ret = -1;
    }
  else if (CHECK_HDR(hdr, FMEM, MEM_RELEASE))
    {
      mem = thiz->_inq.pop(opt->addr);
      if (mem)
        {
          nmem = listener->release_inmem(thiz, wtask, mem, opt->size);
          mem->release();
          if (nmem)
            {
              thiz->_inq.push(nmem);
              alworker_inject_imem(wtask, nmem);
            }
        }
    }
  else if (CHECK_HDR(hdr, OMEM, MEM_RELEASE))
    {
      mem = thiz->_outq.pop(opt->addr);
      if (mem)
        {
          if (opt->eof == 1)
            {
              mem->set_eof();
            }
          else
            {
              mem->clear_eof();
            }

          mem->set_storedsize(opt->size);
          nmem = listener->release_outmem(thiz, wtask, mem);
          mem->release();
          if (nmem)
            {
              thiz->_outq.push(nmem);
              alworker_inject_omem(wtask, nmem);
            }
        }
    }
  else if (hdr.grp == AL_COMM_MESSAGE_USER)
    {
      listener->usermsg(thiz, wtask, hdr, opt);
    }
  else if (hdr.u32 != AL_COMM_NO_MSG)
    {
      thiz->publish_event(AL_EVENT_UNKNOWN, hdr.u32);
    }

  if (hdr.type == AL_COMM_MSGTYPE_SYNC)
    {
      alworker_send_resp(wtask, hdr, ret);
    }

  return ret;
}

void audiolite_workercomp::on_data()
{
  audiolite_memapbuf *mem = (audiolite_memapbuf *)pop_data();

  if (mem)
    {
      if (_worker_booted == true)
        {
          _inq.push(mem);
          alworker_inject_imem(_worker.getwtask(), mem);
        }
      else
        {
          mem->release();
        }
    }
}

int audiolite_workercomp::start_worker()
{
  int ret = OK;
  char thdname[WORKERCOMP_SPKNAMESZ + 4];

  if (_lsnr == NULL)
    {
      ret = ERROR;
    }
  else if (!_worker_booted)
    {
      ret = OK;
      snprintf(thdname, WORKERCOMP_SPKNAMESZ + 3, "%ssvr", _workerspk);
      if (_pool)
        {
          /* If output memory pool is exist, injection thread is needed */

          if (_thd.start())
            {
              _pool->enable_pool();
            }
          else
            {
              ret = ERROR;
            }
        }

      if (ret == OK)
        {
          _inq.enable();
          _outq.enable();
          ret = _worker.bringup_worker(_workerspk, true,
                                       thdname, 105, 2048);
          if (ret != OK)
            {
              if (_pool)
                {
                  _pool->disable_pool();
                  _thd.stop();
                }

              _inq.disable();
              _outq.disable();
            }
        }
    }

  return ret;
}

void audiolite_workercomp::stop_worker()
{
  if (_pool)
    {
      _pool->disable_pool();
    }

  _inq.disable();

  if (_worker_booted)
    {
      _thd.stop();

      _worker_booted = false;
      _worker_terminated = false;
      alworker_send_term(_worker.getwtask());
      while (_worker_terminated == false)
        {
          usleep(1);
        }
    }

  _worker.terminate_worker();
  _outq.disable();
}

int audiolite_workercomp::on_starting(audiolite_inputnode *inode,
                                           audiolite_outputnode *onode)
{
  return start_worker();
}

void audiolite_workercomp::on_canceled(audiolite_inputnode *inode,
                                            audiolite_outputnode *onode)
{
  stop_worker();
}

void audiolite_workercomp::on_stopping(audiolite_inputnode *inode,
                                            audiolite_outputnode *onode)
{
  stop_worker();
}

bool audiolite_workercomp::before_start(void *arg)
{
  audiolite_workercomp *thiz = (audiolite_workercomp *)arg;

  return thiz->_worker_booted;
}

int audiolite_workercomp::run(void *arg)
{
  audiolite_workercomp *thiz = (audiolite_workercomp *)arg;
  audiolite_memapbuf *mem;

  if (thiz->_worker_booted)
    {
      mem = (audiolite_memapbuf *)_pool->allocate();

      if (mem)
        {
          thiz->_outq.push(mem);
          alworker_inject_omem(thiz->_worker.getwtask(), mem);
        }
      else
        {
          usleep(1);  /* Yield */
        }
    }

  return 1;
}

void audiolite_workercomp::before_stop(void *arg)
{
}
