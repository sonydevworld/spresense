/****************************************************************************
 * mpcomm/supervisor/supervisor.c
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

#include <sdk/config.h>
#include <sdk/debug.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include <mpcomm/supervisor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPCOMM_KEY_MQ 1

#ifdef CONFIG_MPCOMM_DEBUG_ERROR
#  define mpcerr(fmt, ...)  syslog(LOG_ERR, fmt, ## __VA_ARGS__)
#else
#  define mpcerr(fmt, ...)
#endif
#ifdef CONFIG_MPCOMM_DEBUG_WARN
#  define mpcwarn(fmt, ...)  syslog(LOG_WARNING, fmt, ## __VA_ARGS__)
#else
#  define mpcwarn(fmt, ...)
#endif
#ifdef CONFIG_MPCOMM_DEBUG_INFO
#  define mpcinfo(fmt, ...)  syslog(LOG_INFO, fmt, ## __VA_ARGS__)
#else
#  define mpcinfo(fmt, ...)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int init_core(worker_info_t *worker, const char *filepath)
{
  int ret;

  ret = mptask_init(&worker->mptask, filepath);
  if (ret != 0)
    {
      mpcerr("mptask_init() failure. %d\n", ret);
      return ret;
    }

  ret = mptask_assign(&worker->mptask);
  if (ret != 0)
    {
      mpcerr("mptask_assign() failure. %d\n", ret);
      return ret;
    }

  worker->cpuid = mptask_getcpuid(&worker->mptask);

  ret = mpmq_init(&worker->mq, MPCOMM_KEY_MQ, worker->cpuid);
  if (ret < 0)
    {
      mpcerr("mpmq_init() failure. %d\n", ret);
      return ret;
    }

  ret = mptask_bindobj(&worker->mptask, &worker->mq);
  if (ret < 0)
    {
      mpcerr("mptask_bindobj(mq) failure. %d\n", ret);
      return ret;
    }

  ret = mptask_exec(&worker->mptask);
  if (ret < 0)
    {
      mpcerr("mptask_exec() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

static int deinit_core(worker_info_t *worker)
{
  int ret;
  int wret;

  ret = mpmq_send(&worker->mq, MPCOMM_MSG_ID_DEINIT, 0);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  wret = -1;
  ret = mptask_destroy(&worker->mptask, false, &wret);
  if (ret < 0)
    {
      mpcerr("mptask_destroy() failure. %d\n", ret);
      return ret;
    }

  mpmq_destroy(&worker->mq);

  return ret;
}

static int supervisor_wait_helper_done(mpcomm_supervisor_context_t *ctx,
                                       uint8_t helper_idx)
{
  int ret;
  uint32_t msgdata;

  ret = mpmq_receive(&ctx->helpers[helper_idx].mq, &msgdata);
  if (ret < 0)
    {
      mpcerr("mpmq_recieve() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

static int supervisor_send_controller_done(mpcomm_supervisor_context_t *ctx)
{
  int ret;

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_DONE, 2);

  return ret;
}

static int supervisor_send_helper_done(mpcomm_supervisor_context_t *ctx,
                                       uint8_t helper_idx)
{
  int ret;

  ret = mpmq_send(&ctx->helpers[helper_idx].mq, MPCOMM_MSG_ID_DONE, 2);

  return ret;
}

static int supervisor_malloc_msg(mpcomm_supervisor_context_t *ctx,
                                 mpcomm_malloc_msg_t *msg)
{
  int i;

  *msg->ptr = malloc(msg->size);

  if (msg->cpuid == ctx->controller.cpuid)
    {
      supervisor_send_controller_done(ctx);
    }
  else
    {
      for (i = 0; i < ctx->helper_num; i++)
        {
          if (msg->cpuid == ctx->helpers[i].cpuid)
            {
              supervisor_send_helper_done(ctx, i);
            }
        }
    }

  return 0;
}

static int supervisor_free_msg(mpcomm_supervisor_context_t *ctx,
                               mpcomm_free_msg_t *msg)
{
  int i;

  free(msg->ptr);

  if (msg->cpuid == ctx->controller.cpuid)
    {
      supervisor_send_controller_done(ctx);
    }
  else
    {
      for (i = 0; i < ctx->helper_num; i++)
        {
          if (msg->cpuid == ctx->helpers[i].cpuid)
            {
              supervisor_send_helper_done(ctx, i);
            }
        }
    }

  return 0;
}

static int supervisor_error_msg(mpcomm_supervisor_context_t *ctx, int ret)
{
  (void) ctx;

  mpcerr("supervisor_error_msg() mpcerr %d.\n", ret);
  return 0;
}

static int supervisor_unknown_msg(mpcomm_supervisor_context_t *ctx)
{
  (void) ctx;

  mpcerr("supervisor_handle_msg() unknown msg.\n");
  return 0;
}

static int supervisor_handle_msg(mpcomm_supervisor_context_t *ctx, int id,
                                 void *data, uint8_t *quit_loop)
{
  int ret = 0;

  switch (id)
    {
      case MPCOMM_MSG_ID_MALLOC:
        ret = supervisor_malloc_msg(ctx, (mpcomm_malloc_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_FREE:
        ret = supervisor_free_msg(ctx, (mpcomm_free_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_ERROR:
        ret = supervisor_error_msg(ctx, (int)data);
        break;
      case MPCOMM_MSG_ID_DONE:
        *quit_loop = 1;
        break;
      default:
        ret = supervisor_unknown_msg(ctx);
        break;
    }

  return ret;
}

static uint8_t supervisor_loop(mpcomm_supervisor_context_t *ctx)
{
  int msgid;
  uint32_t msgdata;
  uint8_t quit_loop = 0;

  msgid = mpmq_receive(&ctx->controller.mq, &msgdata);

  supervisor_handle_msg(ctx, msgid, (void *)msgdata, &quit_loop);

  return quit_loop;
}

static int supervisor_init(mpcomm_supervisor_context_t *ctx,
                           const char *filepath,
                           uint8_t helper_num)
{
  int ret;
  int i;

  ret = init_core(&ctx->controller, filepath);
  if (ret < 0)
    {
      mpcerr("init_core() failure. %d\n", ret);
      return ret;
    }

  for (i = 0; i < helper_num; i++)
    {
      ret = init_core(&ctx->helpers[i], filepath);
      if (ret < 0)
        {
          mpcerr("init_core() failure. %d\n", ret);
          return ret;
        }

      ctx->helpers_cpuset |= (1 << ctx->helpers[i].cpuid);
    }

  mpcomm_init_msg_t init_msg;
  init_msg.mode = MPCOMM_MODE_CONTROLLER;
  init_msg.helpers_cpuset = ctx->helpers_cpuset;
  init_msg.loadaddr = ctx->controller.mptask.loadaddr;

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_INIT,
                  (uint32_t)&init_msg);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  ret = mpcomm_supervisor_wait_controller_done(ctx);
  if (ret < 0)
    {
      mpcerr("mpcomm_supervisor_wait_controller_done() failure. %d\n", ret);
      return ret;
    }

  init_msg.mode = MPCOMM_MODE_HELPER;
  init_msg.controller_cpuid = ctx->controller.cpuid;

  for (i = 0; i < helper_num; i++)
    {
      init_msg.loadaddr = ctx->helpers[i].mptask.loadaddr;
      ret = mpmq_send(&ctx->helpers[i].mq, MPCOMM_MSG_ID_INIT,
                      (uint32_t)&init_msg);
      if (ret < 0)
        {
          mpcerr("mpmq_send() failure. %d\n", ret);
          return ret;
        }

      ret = supervisor_wait_helper_done(ctx, i);
      if (ret < 0)
        {
          mpcerr("supervisor_wait_helper_done() failure. %d\n", ret);
          return ret;
        }
    }

  ctx->helper_num = helper_num;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mpcomm_supervisor_init(mpcomm_supervisor_context_t **ctx,
                           const char *filepath,
                           uint8_t helper_num)
{
  int ret;

  mpcomm_supervisor_context_t *supervisor_context =
    (mpcomm_supervisor_context_t *) kmm_malloc(
      sizeof(mpcomm_supervisor_context_t));
  if (!supervisor_context)
    {
      mpcerr("Failed to allocate context\n");
      return -ENOMEM;
    }

  ret = supervisor_init(supervisor_context, filepath, helper_num);
  if (ret < 0)
    {
      mpcerr("supervisor_init() failure. %d\n", ret);
      kmm_free(supervisor_context);
      return ret;
    }

  *ctx = supervisor_context;

  return ret;
}

int mpcomm_supervisor_deinit(mpcomm_supervisor_context_t *ctx)
{
  int ret;
  int i;

  ret = deinit_core(&ctx->controller);
  if (ret < 0)
    {
      mpcerr("init_core() failure. %d\n", ret);
      return ret;
    }

  for (i = 0; i < MPCOMM_MAX_HELPERS; i++)
    {
      ret = deinit_core(&ctx->helpers[i]);
      if (ret < 0)
        {
          mpcerr("init_core() failure. %d\n", ret);
          return ret;
        }
    }

  kmm_free(ctx);

  return ret;
}

int mpcomm_supervisor_send_controller(mpcomm_supervisor_context_t *ctx,
                                      void *data)
{
  int ret;

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_USER_FUNC,
                  (uint32_t)data);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  return ret;
}

int mpcomm_supervisor_wait_controller_done(mpcomm_supervisor_context_t *ctx)
{
  (void) ctx;

  for (;;)
    {
      if (supervisor_loop(ctx))
        {
          break;
        }
    }

  return 0;
}
