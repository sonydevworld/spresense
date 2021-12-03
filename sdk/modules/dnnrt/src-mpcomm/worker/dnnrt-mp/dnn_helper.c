/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_helper.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <mpcomm/mpcomm.h>

#include "dnn_helper.h"
#include "dnn_controller.h"
#include "dnn_exec_function.h"
#include "dnn_scratch_buffer.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dnn_helper_initialize(void *scratch_buffer)
{
  dnn_scratch_buffer_set(scratch_buffer);

  return 0;
}

static int dnn_helper_exec_affine(rt_function_t *f, int begin, int end)
{
  return dnnrt_exec_affine(f, begin, end);
}

static int dnn_helper_exec_convolution(rt_function_t *f, int begin, int end)
{
  return dnnrt_exec_convolution(f, begin, end);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void dnn_helper_handle_msg(void *data)
{
  dnn_msg_t *dnn_msg = (dnn_msg_t *)data;

  switch (dnn_msg->id)
    {
      case DNN_HELPER_MSG_INIT:
        dnn_msg->ret = dnn_helper_initialize((void *)dnn_msg->arg[0]);
        break;
      case DNN_HELPER_EXEC_AFFINE:
        dnn_msg->ret =
          dnn_helper_exec_affine((rt_function_t *)dnn_msg->arg[0],
                                 dnn_msg->arg[1], dnn_msg->arg[2]);
        break;
      case DNN_HELPER_EXEC_CONVOLUTION:
        dnn_msg->ret =
          dnn_helper_exec_convolution((rt_function_t *)dnn_msg->arg[0],
                                      dnn_msg->arg[1], dnn_msg->arg[2]);
        break;
      default:
        dnn_msg->ret = -EINVAL;
        break;
    }
}
