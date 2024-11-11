/****************************************************************************
 * modules/audiolite/src/base/al_component.cxx
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

#include <errno.h>
#include <audiolite/al_debug.h>
#include <audiolite/al_evthandler.h>
#include <audiolite/al_component.h>

/****************************************************************************
 * Class: audiolite_component
 ****************************************************************************/

/***********************************************
 * Protected Class audiolite_component Methods
 ***********************************************/

int audiolite_component::search_inputidx(audiolite_inputnode *node)
{
  int i;

  for (i = 0; i < _innum; i++)
    {
      if (_ins[i] == node)
        {
          return i;
        }
    }

  return -1;
}

/***********************************************
 * Public Class audiolite_component Methods
 ***********************************************/

audiolite_component::audiolite_component(int inputnum, int outputnum,
                                         int depth, bool is_async,
                                         int prio, int stack_sz) :
    _ins(NULL), _outs(NULL), _op(NULL), _pool(NULL),
    _innum(inputnum), _outnum(outputnum),
    _state(0)
#ifdef _ALDEBUG_ENABLE
    , dbg_name(NULL)
#endif
{
  if (inputnum > 0)
    {
#ifdef __linux__
      _op = mossfw_callback_op_create(audiolite_component::operate_cb,
                            (unsigned long)this, is_async);
#else
      _op = mossfw_callback_op_create_attr(audiolite_component::operate_cb,
                            (unsigned long)this, is_async, prio, stack_sz);
#endif
      if (_op == NULL)
        {
          printf("[AudioLite] Fatal Error! No memory operator\n");
          _innum = 0;
          _outnum = 0;
          return;
        }

      set_operatorname("al_comp_op");

      _ins = (audiolite_inputnode **)malloc(sizeof(audiolite_inputnode *) * inputnum);
      if (_ins)
        {
          for (int i = 0; i < inputnum; i++)
            {
              _ins[i] = new audiolite_inputnode(this, depth);
              if (_ins[i])
                {
                  _ins[i]->setup_operation(_op);
                }
              else
                {
                  printf("[AudioLite] Fatal Error! No memory innode\n");
                  for (int j = i - 1; j >= 0; j--)
                    {
                      delete _ins[j];
                    }

                  free(_ins);
                  _ins = NULL;
                  _innum = 0;
                  _outnum = 0;
                  return;
                }
            }
        }
      else
        {
          printf("[AudioLite] Fatal Error! No memory inputnode array\n");
          _innum = 0;
          _outnum = 0;
          return;
        }
    }

  if (outputnum > 0)
    {
      _outs = (audiolite_outputnode **)malloc(sizeof(audiolite_outputnode *) * outputnum);
      if (_outs)
        {
          for (int i = 0; i < outputnum; i++)
            {
              _outs[i] = new audiolite_outputnode(this);
              if (_outs[i] == NULL)
                {
                  printf("[AudioLite] Fatal Error! No memory outnode\n");
                  for (int j = i - 1; j >= 0; j--)
                    {
                      delete _outs[j];
                    }

                  free(_outs);
                  _outs = NULL;

                  if (_ins)
                    {
                      for (int k = 0; k < _innum; k++)
                        {
                          delete _ins[k];
                        }

                      free(_ins);
                      _ins = NULL;
                    }

                  _innum = 0;
                  _outnum = 0;
                  return;
                }
            }
        }
      else
        {
          printf("[AudioLite] Fatal Error! No memory outputnode array\n");
          if (_ins)
            {
              for (int i = 0; i < _innum; i++)
                {
                  delete _ins[i];
                }

              free(_ins);
              _ins = NULL;
            }

          _innum = 0;
          _outnum = 0;
          return;
        }
    }
}

audiolite_component::~audiolite_component()
{
  /* Stop own operation */

  for (int i = 0; i < _innum; i++)
    {
      _ins[i]->reset_operation();
    }

  /* Stop own operation */

  unbindall();

  if (_op)
    {
      mossfw_callback_op_delete(_op);
    }

  for (int i = 0; i < _innum; i++)
    {
      delete _ins[i];
    }

  for (int i = 0; i < _outnum; i++)
    {
      delete _outs[i];
    }

  if (_ins)
    {
     free(_ins);
    }

  if (_outs)
    {
      free(_outs);
    }
}

void audiolite_component::set_operatorname(const char *name)
{
  if (_op && _op->async)
    {
      pthread_setname_np(_op->async->tid, name);
    }
}

void audiolite_component::set_mempool(audiolite_mempool *pool)
{
  _pool = pool;
}

/* Call backs */

void audiolite_component::cancel(audiolite_inputnode *node)
{
  audiolite_inputnode *tmp;

  for (int i = 0; i < _outnum; i++)
    {
      for (tmp = _outs[i]->_binded; tmp; tmp = tmp->_nextnode)
        {
          tmp->_comm.cancel();
        }
    }

  on_canceled(node, NULL);
}

int audiolite_component::start(audiolite_inputnode *node)
{
  int i;
  int ret;
  audiolite_inputnode *tmp;

  al_ddebug("Entry\n");

  ret = on_starting(node, NULL);

  if (ret != OK)
    {
      return ret;
    }
  else
    {
      for (i = 0; i < _outnum; i++)
        {
          ret = audiolite_start(_outs[i]);
          if (ret != OK)
            {
              goto error_cancel;
            }
        }

      on_started(node, NULL);
    }

  al_ddebug("Leave successfully\n");
  return OK;

error_cancel:

  for (; i > 0; i--)
    {
      for (tmp = _outs[i]->_binded; tmp; tmp = tmp->_nextnode)
        {
          tmp->_comm.cancel();
        }
    }

  on_canceled(node, NULL);

  al_ddebug("Leave with error\n");
  return ret;
}

void audiolite_component::stop(audiolite_inputnode *node)
{
  al_ddebug("Entry\n");

  on_stopping(node, NULL);

  for (int i = 0; i < _outnum; i++)
    {
      al_ddebug("Call %d/%d\n", i, _outnum);
      audiolite_stop(_outs[i]);
    }

  al_ddebug("Call on_stop\n");
  on_stop(node, NULL);
  al_ddebug("Leave\n");
}

void audiolite_component::cancel(audiolite_outputnode *node)
{
  for (int i = 0; i < _innum; i++)
    {
      if (_ins[i]->_connected)
        {
          _ins[i]->_connected->_comm.rcancel();
        }
    }

  on_canceled(NULL, node);
}

int audiolite_component::start(audiolite_outputnode *node)
{
  int i;
  int ret;

  al_ddebug("Entry\n");
  ret = on_starting(NULL, node);

  if (ret != OK)
    {
      return ret;
    }
  else
    {
      for (i = 0; i < _innum; i++)
        {
          ret = audiolite_start(_ins[i]);
          if (ret != OK)
            {
              goto error_rcancel;
            }
        }

      on_started(NULL, node);
    }

  al_ddebug("Leave successfully\n");
  return OK;

error_rcancel:

  for (; i > 0; i--)
    {
      if (_ins[i]->_connected)
        {
          _ins[i]->_connected->_comm.rcancel();
        }
    }

  on_canceled(NULL, node);

  al_ddebug("Leave with error\n");
  return 0;
}

void audiolite_component::stop(audiolite_outputnode *node)
{
  for (int i = 0; i < _outnum; i++)
    {
      audiolite_stop(_outs[i]);
    }

  on_stop(NULL, node);
}

bool audiolite_component::can_breakdata(audiolite_outputnode *out)
{
  /* Default behavior is just check on input 0 node. */

  if (_innum > 0)
    {
      return _ins[0]->can_breakdata();
    }
  else
    {
      return true;
    }
}

void audiolite_component::data_came(mossfw_callback_op_t *op)
{
  on_data();
}

void audiolite_component::on_data()
{
  /* Default on_data() */

  audiolite_mem *mem = _ins[0]->pop_data(NULL);

  if (mem)
    {
      /* Default behavior is just pass to output 0 on input 0 */

      if (_outnum > 0)
        {
          if (mem->get_storedsize() != 0)
            {
              _outs[0]->push_data(mem);
            }
          else
            {
              printf("[AudioLite] %s(%d) : ZERO size memory is pushed\n"
                     "                     This might be BUG. \n"
                     "                     Check it if it is expected\n",
                     __func__, __LINE__);
            }
        }

      mem->release();
    }

  /* Drop data of other inputs if it exists */

  for (int i = 1; i < _innum; i++)
    {
      mem = _ins[i]->pop_data(NULL);
      mem->release();
    }
}

int audiolite_component::on_starting(audiolite_inputnode *inode,
                                     audiolite_outputnode *onode)
{
  /* Do nothing as default. */

  return OK;
}

void audiolite_component::on_canceled(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  /*  Do nothing as default. */
}

void audiolite_component::on_started(audiolite_inputnode *inode,
                                     audiolite_outputnode *onode)
{
  /* Default on_started() */

  for (int i = 0; i < _innum; i++)
    {
      _ins[i]->start_operation();
    }
}

void audiolite_component::on_stopping(audiolite_inputnode *inode,
                                      audiolite_outputnode *onode)
{
  al_ddebug("Enter\n");
  for (int i = 0; i < _innum; i++)
    {
      _ins[i]->stop_operation();
    }
}

void audiolite_component::on_stop(audiolite_inputnode *inode,
                                  audiolite_outputnode *onode)
{
  /* Default on_stop() */

  al_ddebug("Enter\n");
  for (int i = 0; i < _innum; i++)
    {
      _ins[i]->release_allstoredbuff();
    }
}

void audiolite_component::reflesh(audiolite_inputnode *node)
{
  audiolite_mem *mem;

  for (int i = 0; i < _outnum; i++)
    {
      audiolite_reflesh(_outs[i]);
    }

  for (int i = 0; i < _innum; i++)
    {
      while ((mem = _ins[i]->pop_data(NULL)) != NULL)
        {
          mem->release();
        }
    }
}

void audiolite_component::reflesh(audiolite_outputnode *node)
{
  audiolite_mem *mem;

  for (int i = 0; i < _innum; i++)
    {
      while ((mem = _ins[i]->pop_data(NULL)) != NULL)
        {
          mem->release();
        }
    }

  for (int i = 0; i < _innum; i++)
    {
      audiolite_reflesh(_ins[i]);
    }
}


int audiolite_component::publish_event(int evtid, unsigned long arg)
{
  return audiolite_evthandler::get_instance()->publish_event(evtid, this, arg);
}

int audiolite_component::samplingrate()
{
  return audiolite_evthandler::get_instance()->get_fs();
}

int audiolite_component::channels()
{
  return audiolite_evthandler::get_instance()->get_chnum();
}

int audiolite_component::samplebitwidth()
{
  return audiolite_evthandler::get_instance()->get_bitwidth();
}

audiolite_inputnode *audiolite_component::get_input(int id)
{
  if (id < _innum)
    {
      return _ins[id];
    }

  return NULL;
}

audiolite_outputnode *audiolite_component::get_output(int id)
{
  if (id < _outnum)
    {
      return _outs[id];
    }

  return NULL;
}

audiolite_component *audiolite_component::bind(audiolite_component *cmp)
{
  bind(cmp->get_input(), 0);
  return cmp;
}

int audiolite_component::bind(audiolite_inputnode *in, int outid)
{
  if (in != NULL && outid < _outnum)
    {
      return _outs[outid]->bind(in);
    }

  return -EINVAL;
}

int audiolite_component::unbind(audiolite_inputnode *in)
{
  int ret = -EINVAL;

  if (in && in->_connected)
    {
      ret = in->_connected->unbind(in);
    }

  return ret;
}

int audiolite_component::unbindall()
{
  /* unbind all inputs */

  for (int i = 0; i < _innum; i++)
    {
      unbind(_ins[i]);
    }

  /* Unbind all outputs */

  for (int i = 0; i < _outnum; i++)
    {
      _outs[i]->unbind();
    }

  return 0;
}

int audiolite_component::start()
{
  if (_innum != 0)
    {
      return -ENOTSUP;
    }

  return start((audiolite_inputnode *)NULL);
}

void audiolite_component::stop()
{
  if (_innum == 0)
    {
      stop((audiolite_inputnode *)NULL);
    }
}

void audiolite_component::suspend()
{
  if (_innum == 0)
    {
      stop((audiolite_inputnode *)NULL);
    }
}

int audiolite_component::resume()
{
  if (_innum != 0)
    {
      return -ENOTSUP;
    }

  return start((audiolite_inputnode *)NULL);
}

/***********************************************
 * Static Protected Class audiolite_component Methods
 ***********************************************/

int audiolite_component::operate_cb(mossfw_callback_op_t *op, unsigned long arg)
{
  audiolite_component *comp = (audiolite_component *)arg;

  if (comp)
    {
      comp->data_came(op);
    }

  return 0;
}
