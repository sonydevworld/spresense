/****************************************************************************
 * modules/dnnrt/src-mp/runtime/mp_manager.c
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "runtime_client.h"

#define DNNRT_BINCLONE_WORKER_IMAGE "dnnrt-mp"

typedef struct dnn_mptask
{
  mpmq_t mq;                    /* between nuttx core and target ASMP core */
  int8_t in_use;
} dnn_mptask_t;

typedef struct lib_global_context
{
  mptask_t bin_clone_task;
  dnn_mptask_t mptask[MAX_MP_CORE];
  mp_message_buffer_t msg_buf;  /* messages send from master to library must be 
                                 * placed on Nuttx memory, * so reallocate
                                 * buffer here */
} lib_global_context_t;

static lib_global_context_t s_mp_gctx;

static lib_global_context_t *dnn_mpmgr_global_context(void)
{
  return &s_mp_gctx;
}

static int dnn_mpmgr_post_msg(dnn_mptask_t * task, int8_t msgid, void *data)
{
  return mpmq_send(&task->mq, msgid, (uint32_t) data);
}

static void dnn_mpmgr_process_msg(dnn_mptask_t * task, int msg, uint32_t data)
{
  switch (msg)
    {
    case MP_MSG_MALLOC:
      {
        mp_alloc_memory_t *alloc = (mp_alloc_memory_t *) data;
        alloc->addr = malloc(alloc->bsize);
        dnn_mpmgr_post_msg(task, MP_MSG_MALLOC | MP_MSG_ACK_BIT, alloc);
        break;
      }

    case MP_MSG_FREE:
      {
        mp_free_memory_t *freemem = (mp_free_memory_t *) data;
        free(freemem->addr);
        dnn_mpmgr_post_msg(task, MP_MSG_FREE | MP_MSG_ACK_BIT, freemem);
        break;
      }

    case MP_MSG_REALLOC:
      {
        mp_alloc_memory_t *alloc = (mp_alloc_memory_t *) data;
        alloc->addr = realloc(alloc->addr, alloc->bsize);
        dnn_mpmgr_post_msg(task, MP_MSG_REALLOC | MP_MSG_ACK_BIT, alloc);
        break;
      }

    default:
      {
        /* ignore any unknown message */
        break;
      }
    }
}

static int
dnn_mpmgr_send_msg(dnn_mptask_t * task, int8_t msgid, void *data, uint32_t ms)
{
  int ret = mpmq_send(&task->mq, msgid, (uint32_t) data);

  if (ret < 0)
    {
      return ret;
    }

  int resp;
  uint32_t rdata;
  for (;;)
    {
      if (ms)
        {
          resp = mpmq_timedreceive(&task->mq, &rdata, ms);
        }
      else
        {
          resp = mpmq_receive(&task->mq, &rdata);
        }

      if (resp < 0)
        {
          return resp;
        }

      if (resp == (msgid | MP_MSG_ACK_BIT))
        {
          break;
        }
      else
        {
          /* process requests from MP core while API request is pending */
          dnn_mpmgr_process_msg(task, resp, rdata);
        }
    }

  return 0;
}

int dnn_mpmgr_call_api(int api, int num_args, ...)
{
  int i, ret;
  mp_api_call_t api_call;
  lib_global_context_t *ctx = dnn_mpmgr_global_context();
  dnn_mptask_t *master = &ctx->mptask[0];
  va_list vl;

  if (!master->in_use)
    {
      ret = -EPERM;
      goto bye;
    }

  int max_args = _S(api_call.arg);
  if (num_args > max_args)
    {
      num_args = max_args;
    }

  memset(&api_call, 0, sizeof(api_call));
  api_call.api = api;
  va_start(vl, num_args);
  for (i = 0; i < num_args; ++i)
    {
      api_call.arg[i] = va_arg(vl, int);
    }

  ret = dnn_mpmgr_send_msg(master, MP_MSG_CALL_API, &api_call, 0);
  if (ret)
    {
      goto bye;
    }
  ret = api_call.ret;
bye:
  return ret;
}

static int dnn_mpmgr_start_task(lib_global_context_t * ctx, int slave_num)
{
  int i, ret = 0;
  cpu_set_t cpu_set;

  /* already initialized? */
  if (ctx->mptask[0].in_use)
    {
      ret = -EPERM;
      goto bye;
    }

  /* init mptask and message queue */
  ret = mptask_init_secure(&ctx->bin_clone_task, DNNRT_BINCLONE_WORKER_IMAGE);
  if (ret < 0)
    {
      goto bye;
    }

  ret = mptask_assign_cpus(&ctx->bin_clone_task, slave_num + 1);
  if (ret < 0)
    {
      goto bye;
    }

  ret = mptask_getcpuidset(&ctx->bin_clone_task, &cpu_set);
  if (ret < 0)
    {
      goto bye;
    }

  int task_idx = 0;
  int cpuid;
  for (cpuid = 0; cpuid < MP_CPUID_MAX; ++cpuid)
    {
      if ((1 << cpuid) & cpu_set)
        {
          ret = mpmq_init(&ctx->mptask[task_idx++].mq, MP_MQ_KEY, cpuid);
          if (ret < 0)
            {
              goto bye;
            }
        }
    }

  ret = mptask_exec(&ctx->bin_clone_task);
  if (ret < 0)
    {
      goto bye;
    }

  /* initialize each worker */
  for (i = 0; i < (slave_num + 1); ++i)
    {
      dnn_mptask_t *task = &ctx->mptask[i];

      mp_task_init_t init = {
        .cpu_set = cpu_set,
        .load_addr = 0,
        .msg_buf = &ctx->msg_buf,
        .ret = 0,
      };

      if (slave_num == 0)
        {
          init.load_addr = (void *)ctx->bin_clone_task.loadaddr;
        }
      else
        {
          init.load_addr = (void *)ctx->bin_clone_task.bin[i].loadaddr;
        }

      ret = dnn_mpmgr_send_msg(task, MP_MSG_INIT, &init, 0);
      if (init.ret < 0)
        {
          ret = init.ret;
        }
      if (ret < 0)
        {
          goto bye;
        }
      task->in_use = true;
    }
bye:
  return ret;
}

int dnn_mpmgr_load(int slave_num)
{
  int ret;
  lib_global_context_t *ctx = dnn_mpmgr_global_context();

  if (MAX_MP_CORE == 0)
    {
      ret = -ENOMEM;
      goto bye;
    }

  ret = dnn_mpmgr_start_task(ctx, slave_num);
bye:
  if (ret != 0)
    {
      dnn_mpmgr_unload();
    }

  return ret;
}

int dnn_mpmgr_unload(void)
{
  int i;
  lib_global_context_t *ctx = dnn_mpmgr_global_context();

  for (i = 0; i < MAX_MP_CORE; ++i)
    {
      dnn_mptask_t *task = &ctx->mptask[i];

      if (task->in_use)
        {
          dnn_mpmgr_send_msg(task, MP_MSG_QUIT, 0, 10);
          mpmq_destroy(&task->mq);
        }
      memset(task, 0, sizeof(*task));
    }

  return mptask_destroy(&ctx->bin_clone_task, true /* force */ , 0);
}
