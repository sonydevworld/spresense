/****************************************************************************
 * modules/audiolite/src/base/al_outputnode.cxx
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

#include <mossfw/mossfw_component.h>

#include <audiolite/al_debug.h>
#include <audiolite/al_outputnode.h>
#include <audiolite/al_inputnode.h>
#include <audiolite/al_component.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATATYPE  (MOSSFW_DATA_TYPE_SHORT      \
                  | MOSSFW_DATA_TYPEGRP_V1    \
                  | MOSSFW_DATA_TYPENAME_AUDIO \
                  | MOSSFW_DATA_TYPEARRAY_ARRAY)

/****************************************************************************
 * class: audiolite_outputnode
 ****************************************************************************/

/***********************************************
 * Public Class audiolite_outputnode Methods
 ***********************************************/

audiolite_outputnode::audiolite_outputnode(audiolite_component *cmp) :
    _component(cmp), _binded(NULL), _comm(cmp, this), _output(NULL)
{
  mossfw_lock_init(&_lock);
  _output = mossfw_output_create(DATATYPE);
}

audiolite_outputnode::~audiolite_outputnode()
{
  unbind();
  if (_output)
    {
      mossfw_output_delete(_output);
    }
}

int audiolite_outputnode::bind(audiolite_inputnode *in)
{
  int ret = OK;
  audiolite_inputnode *tmp;

  mossfw_lock_take(&_lock);

  if (mossfw_bind_inout(_output, in->_input, 0) == OK)
    {
      if (_binded)
        {
          for (tmp = _binded;
               tmp->_nextnode;
               tmp = tmp->_nextnode);
          tmp->_nextnode = in;
        }
      else
        {
          _binded = in;
        }

      in->_connected = this;
      in->_nextnode = NULL;
    }
  else
    {
      ret = -EINVAL;
    }

  mossfw_lock_give(&_lock);

  return ret;
}

int audiolite_outputnode::unbind()
{
  while (_binded)
    {
      if (unbind(_binded) != 0)
        {
          return -EINVAL;
        }
    }

  return OK;
}

int audiolite_outputnode::unbind(audiolite_inputnode *in)
{
  audiolite_inputnode *tmp;
  audiolite_inputnode *last;

  if (!in)
    {
      return -EINVAL;
    }

  mossfw_lock_take(&_lock);

  if (_binded == in)
    {
      _binded = in->_nextnode;
      in->_connected = NULL;
      in->_nextnode = NULL;
      mossfw_unbind_inout(_output, in->_input);
    }
  else
    {
      last = NULL;
      for (tmp = _binded;
           tmp;
           tmp = tmp->_nextnode)
        {
          if (tmp == in)
            {
              last->_nextnode = tmp->_nextnode;
              tmp->_connected = NULL;
              tmp->_nextnode = NULL;
              mossfw_unbind_inout(_output, tmp->_input);
              break;
            }

          last = tmp;
        }
    }

  mossfw_lock_give(&_lock);

  return 0;
}

bool audiolite_outputnode::can_breakdata()
{
  if (_component)
    {
      return _component->can_breakdata(this);
    }
  else
    {
      return true;
    }
}

int audiolite_outputnode::push_data(audiolite_mem *mem)
{
  return mossfw_deliver_dataarray(_output, (mossfw_data_t *)mem);
}

int audiolite_outputnode::fanout()
{
  return mossfw_get_fanout(_output);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int audiolite_start(audiolite_outputnode *node)
{
  int i;
  int ret;
  int fanout;
  audiolite_inputnode *tmp;

  al_ddebug("Entry\n");

  if (!node)
    {
      return OK;
    }

  al_ddebug("Check funout\n");

  fanout = node->fanout();

  al_ddebug("      funout is %d\n", fanout);

  for (tmp = node->_binded; tmp; tmp = tmp->_nextnode)
    {
      al_ddebug("Remain %d\n", fanout);
      ret = tmp->_comm.start();
      if (ret != OK)
        {
          goto error_occured_cancel;
        }

      fanout--;
    }

  return OK;

error_occured_cancel:

  /* Canceling start() */

  fanout = node->fanout() - fanout;
  for (i = 0, tmp = node->_binded;
       i < fanout && tmp;
       tmp = tmp->_nextnode, i++)
    {
      tmp->_comm.cancel();
    }

  return ret;
}

void audiolite_stop(audiolite_outputnode *node)
{
  audiolite_inputnode *tmp;

  al_ddebug("Entry\n");
  if (node)
    {
      for (tmp = node->_binded; tmp; tmp = tmp->_nextnode)
        {
          al_ddebug("Call comm stop : Next node %08x\n", tmp->_nextnode);
          tmp->_comm.stop();
        }

      al_ddebug("Done all binded input\n");
    }

  al_ddebug("Leave\n");
}

void audiolite_reflesh(audiolite_outputnode *node)
{
  audiolite_inputnode *tmp;

  if (node)
    {
      for (tmp = node->_binded; tmp; tmp = tmp->_nextnode)
        {
          tmp->_comm.reflesh();
        }
    }
}
