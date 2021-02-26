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
 * Private Variables
 ****************************************************************************/

static supervisor_context_t supervisor_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static supervisor_context_t *get_supervisor_context(void)
{
  return &supervisor_ctx;
}

static int init_core(worker_info_t *worker, const char *filepath,
                     const char *filename)
{
  int ret;
  char fullpath[128];

  snprintf(fullpath, 128, "%s/%s", filepath, filename);

  ret = mptask_init(&worker->mptask, fullpath);
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

static int supervisor_wait_helper_done(uint8_t helper_idx)
{
  int ret;
  uint32_t msgdata;
  supervisor_context_t *ctx = get_supervisor_context();

  ret = mpmq_receive(&ctx->helpers[helper_idx].mq, &msgdata);
  if (ret < 0)
    {
      mpcerr("mpmq_recieve() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

static int supervisor_send_controller_done(void)
{
  int ret;
  supervisor_context_t *ctx = get_supervisor_context();

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_DONE, 2);

  return ret;
}

static int supervisor_malloc_msg(supervisor_malloc_msg_t *msg)
{
  *msg->ptr = malloc(msg->size);

  supervisor_send_controller_done();

  return 0;
}

static int supervisor_free_msg(supervisor_free_msg_t *msg)
{
  free(msg->ptr);

  supervisor_send_controller_done();

  return 0;
}

static int supervisor_error_msg(int ret)
{
  mpcerr("supervisor_error_msg() mpcerr %d.\n", ret);
  return 0;
}

static int supervisor_unknown_msg(void)
{
  mpcerr("supervisor_handle_msg() unknown msg.\n");
  return 0;
}

static int supervisor_handle_msg(int id, void *data, uint8_t *quit_loop)
{
  int ret = 0;

  switch (id)
    {
      case MPCOMM_MSG_ID_MALLOC:
        ret = supervisor_malloc_msg((supervisor_malloc_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_FREE:
        ret = supervisor_free_msg((supervisor_free_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_ERROR:
        ret = supervisor_error_msg((int)data);
        break;
      case MPCOMM_MSG_ID_DONE:
        *quit_loop = 1;
        break;
      default:
        ret = supervisor_unknown_msg();
        break;
    }

  return ret;
}

static uint8_t supervisor_loop(void)
{
  int msgid;
  uint32_t msgdata;
  uint8_t quit_loop = 0;
  supervisor_context_t *ctx = get_supervisor_context();

  msgid = mpmq_receive(&ctx->controller.mq, &msgdata);

  supervisor_handle_msg(msgid, (void *)msgdata, &quit_loop);

  return quit_loop;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int supervisor_init(const char *filepath, uint8_t helper_num)
{
  int ret;
  int i;
  supervisor_context_t *ctx = get_supervisor_context();

  ret = init_core(&ctx->controller, filepath, "controller");
  if (ret < 0)
    {
      mpcerr("init_core() failure. %d\n", ret);
      return ret;
    }

  for (i = 0; i < helper_num; i++)
    {
      ret = init_core(&ctx->helpers[i], filepath, "helper");
      if (ret < 0)
        {
          mpcerr("init_core() failure. %d\n", ret);
          return ret;
        }

      ctx->helpers_cpuset |= (1 << ctx->helpers[i].cpuid);
    }

  controller_init_msg_t controller_init_msg;
  controller_init_msg.helpers_cpuset = ctx->helpers_cpuset;
  controller_init_msg.loadaddr = ctx->controller.mptask.loadaddr;

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_INIT,
                  (uint32_t)&controller_init_msg);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  ret = supervisor_wait_controller_done();
  if (ret < 0)
    {
      mpcerr("supervisor_wait_controller_done() failure. %d\n", ret);
      return ret;
    }

  helper_init_msg_t helper_init_msg;
  helper_init_msg.controller_cpuid = ctx->controller.cpuid;

  for (i = 0; i < helper_num; i++)
    {
      helper_init_msg.loadaddr = ctx->helpers[i].mptask.loadaddr;
      ret = mpmq_send(&ctx->helpers[i].mq, MPCOMM_MSG_ID_INIT,
                      (uint32_t)&helper_init_msg);
      if (ret < 0)
        {
          mpcerr("mpmq_send() failure. %d\n", ret);
          return ret;
        }

      ret = supervisor_wait_helper_done(i);
      if (ret < 0)
        {
          mpcerr("supervisor_wait_controller_done() failure. %d\n", ret);
          return ret;
        }
    }

  ctx->helper_num = helper_num;

  return ret;
}

int supervisor_deinit(void)
{
  int ret;
  int i;
  supervisor_context_t *ctx = get_supervisor_context();

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

  return ret;
}

int supervisor_send_controller(void *data)
{
  int ret;
  supervisor_context_t *ctx = get_supervisor_context();

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_USER_FUNC,
                  (uint32_t)data);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  return ret;
}

int supervisor_wait_controller_done(void)
{
  for (;;)
    {
      if (supervisor_loop())
        {
          break;
        }
    }

  return 0;
}
