/****************************************************************************
 * modules/audiolite/src/base/al_inputnode.cxx
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
#include <audiolite/al_inputnode.h>
#include <audiolite/al_outputnode.h>
#include <audiolite/al_component.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATATYPE  (MOSSFW_DATA_TYPE_SHORT      \
                  | MOSSFW_DATA_TYPEGRP_V1    \
                  | MOSSFW_DATA_TYPENAME_AUDIO \
                  | MOSSFW_DATA_TYPEARRAY_ARRAY)

/****************************************************************************
 * class: audiolite_inputnode
 ****************************************************************************/

/***********************************************
 * Public Class audiolite_inputnode Methods
 ***********************************************/

audiolite_inputnode::audiolite_inputnode(audiolite_component *cmp,
                                         int depth) :
      _component(cmp), _comm(cmp, this),
      _connected(NULL), _input(NULL), _op(NULL), _nextnode(NULL)
{
  _input = mossfw_input_create(DATATYPE, depth);
}

audiolite_inputnode::~audiolite_inputnode()
{
  if (_connected)
    {
      _connected->unbind(this);
    }

  if (_input)
    {
      mossfw_input_delete(_input);
    }
}

int audiolite_inputnode::setup_operation(mossfw_callback_op_t *op)
{
  int ret = -EALREADY;

  if (_op == NULL)
    {
      _op = op;
      mossfw_set_waitcondition(_input, -1, _op);
      ret = 0;
    }

  return ret;
}

int audiolite_inputnode::reset_operation()
{
  int ret = -EINVAL;

  if (_op)
    {
      mossfw_stop_callback_op(_op);
      mossfw_unset_waitcondition(_input, _op);
      _op = NULL;
      ret = OK;
    }

  return ret;
}

void audiolite_inputnode::start_operation()
{
  if (_op)
    {
      mossfw_update_waitsize(_input, 1);
    }
}

void audiolite_inputnode::stop_operation()
{
  if (_op)
    {
      mossfw_update_waitsize(_input, -1);
    }
}

void audiolite_inputnode::release_allstoredbuff()
{
  audiolite_mem *mem;

  while ((mem = (audiolite_mem *)mossfw_release_delivereddata_array(_input))
          != NULL)
    {
      mem->release();
    }
}

bool audiolite_inputnode::can_breakdata()
{
  bool ret = true;

  if (_connected)
    {
      if (_connected->fanout() >= 2)
        {
          ret = false;
        }
      else
        {
          ret = _connected->can_breakdata();
        }
    }

  return ret;
}

audiolite_mem *audiolite_inputnode::pop_data(int *used)
{
  return (audiolite_mem *)mossfw_get_delivereddata_array(_input, 0, used);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int audiolite_start(audiolite_inputnode *node)
{
  int ret = 0;

  if (node && node->_connected)
    {
      ret = node->_connected->_comm.rstart();
    }

  return ret;
}

void audiolite_stop(audiolite_inputnode *node)
{
  if (node && node->_connected)
    {
      node->_connected->_comm.rstop();
    }
}

void audiolite_reflesh(audiolite_inputnode *node)
{
  if (node && node->_connected)
    {
      node->_connected->_comm.rreflesh();
    }
}
