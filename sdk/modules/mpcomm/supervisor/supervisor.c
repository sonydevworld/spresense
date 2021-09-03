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
      mpcerr("mptask_init() failure: %d.\n", ret);
      return ret;
    }

  ret = mptask_assign(&worker->mptask);
  if (ret != 0)
    {
      mpcerr("mptask_assign() failure: %d.\n", ret);
      return ret;
    }

  worker->cpuid = mptask_getcpuid(&worker->mptask);

  ret = mpmq_init(&worker->mq, MPCOMM_KEY_MQ, worker->cpuid);
  if (ret < 0)
    {
      mpcerr("mpmq_init() failure: %d.\n", ret);
      return ret;
    }

  ret = mptask_bindobj(&worker->mptask, &worker->mq);
  if (ret < 0)
    {
      mpcerr("mptask_bindobj(mq) failure: %d.\n", ret);
      return ret;
    }

  ret = mptask_exec(&worker->mptask);
  if (ret < 0)
    {
      mpcerr("mptask_exec() failure: %d.\n", ret);
      return ret;
    }

  return ret;
}

static int deinit_core(worker_info_t *worker)
{
  int ret;
  int wret;

  ret = mpmq_send(&worker->mq, MPCOMM_MSG_ID_DEINIT, 0);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure: %d.\n", ret);
      return ret;
    }

  wret = -1;
  ret = mptask_destroy(&worker->mptask, false, &wret);
  if (ret < 0)
    {
      mpcerr("mptask_destroy() failure: %d.\n", ret);
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
      mpcerr("mpmq_recieve() failure: %d.\n", ret);
      return ret;
    }

  return 0;
}

static int supervisor_send_controller_done(mpcomm_supervisor_context_t *ctx)
{
  int ret;

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_DONE, 2);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure: %d.\n", ret);
      return ret;
    }

  return ret;
}

static int supervisor_send_helper_done(mpcomm_supervisor_context_t *ctx,
                                       uint8_t helper_idx)
{
  int ret;

  ret = mpmq_send(&ctx->helpers[helper_idx].mq, MPCOMM_MSG_ID_DONE, 2);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure: %d.\n", ret);
      return ret;
    }

  return ret;
}

static int supervisor_send_worker_done(mpcomm_supervisor_context_t *ctx,
                                       cpuid_t cpuid)
{
  int ret = 0;
  int i;

  if (cpuid == ctx->controller.cpuid)
    {
      ret = supervisor_send_controller_done(ctx);
      if (ret < 0)
        {
          mpcerr("supervisor_send_controller_done() failure: %d.\n", ret);
          return ret;
        }
    }
  else
    {
      for (i = 0; i < ctx->helper_num; i++)
        {
          if (cpuid == ctx->helpers[i].cpuid)
            {
              ret = supervisor_send_helper_done(ctx, i);
              if (ret < 0)
                {
                  mpcerr("supervisor_send_helper_done() failure: %d.\n",
                         ret);
                  return ret;
                }
            }
        }
    }

  return ret;
}

static int supervisor_malloc_msg(mpcomm_supervisor_context_t *ctx,
                                 mpcomm_malloc_msg_t *msg)
{
  int ret;
  void *ptr;

  ptr = malloc(msg->size);
  if (!ptr)
    {
      mpcerr("malloc() failure.\n");
      return -ENOMEM;
    }

  *msg->ptr = ptr;

  ret = supervisor_send_worker_done(ctx, msg->cpuid);
  if (ret < 0)
    {
      mpcerr("supervisor_send_worker_done() failure: %d.\n", ret);
      return ret;
    }

  return ret;
}

static int supervisor_free_msg(mpcomm_supervisor_context_t *ctx,
                               mpcomm_free_msg_t *msg)
{
  int ret;

  free(msg->ptr);

  ret = supervisor_send_worker_done(ctx, msg->cpuid);
  if (ret < 0)
    {
      mpcerr("supervisor_send_worker_done() failure: %d.\n", ret);
      return ret;
    }

  return 0;
}

static int supervisor_error_msg(mpcomm_supervisor_context_t *ctx,
                                mpcomm_error_msg_t *msg)
{
  int ret;

  mpcerr("ERROR CPU: %d error: %d.\n", msg->cpuid, msg->error);
  fflush(stdout);

  ret = supervisor_send_worker_done(ctx, msg->cpuid);
  if (ret < 0)
    {
      mpcerr("supervisor_send_worker_done() failure: %d.\n", ret);
      return ret;
    }

  return 0;
}

static int supervisor_log_msg(mpcomm_supervisor_context_t *ctx,
                              mpcomm_log_msg_t *msg)
{
  int ret;

  mpcinfo("LOG CPU: %d log: %s\n", msg->cpuid, msg->log);
  fflush(stdout);

  ret = supervisor_send_worker_done(ctx, msg->cpuid);
  if (ret < 0)
    {
      mpcerr("supervisor_send_worker_done() failure: %d.\n", ret);
      return ret;
    }

  return 0;
}

static int supervisor_unknown_msg(mpcomm_supervisor_context_t *ctx, int id)
{
  (void) ctx;

  mpcerr("supervisor_unknown_msg() unknown msg: %d.\n", id);

  return 0;
}

static int controller_listener_handle_msg(mpcomm_supervisor_context_t *ctx,
                                          int id, void *data)
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
        ret = supervisor_error_msg(ctx, (mpcomm_error_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_LOG:
        ret = supervisor_log_msg(ctx, (mpcomm_log_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_DONE:
        ret = sem_post(&ctx->sem_done);
        break;
      default:
        ret = supervisor_unknown_msg(ctx, id);
        break;
    }

  return ret;
}

static FAR void *controller_listener_entry(FAR void *arg)
{
  int ret;
  int msgid;
  uint32_t msgdata;
  mpcomm_supervisor_context_t *ctx = (mpcomm_supervisor_context_t *)arg;

  for (;;)
    {
      msgid = mpmq_receive(&ctx->controller.mq, &msgdata);

      ret = controller_listener_handle_msg(ctx, msgid, (void *)msgdata);
      if (ret < 0)
        {
          mpcerr("controller_listener_handle_msg(%d) failure: %d.\n",
                 msgid, ret);
        }
    }

  return NULL;
}

static int controller_listener_initialize(mpcomm_supervisor_context_t *ctx)
{
  int ret;
  pthread_attr_t attr;
  struct sched_param sch_param;

  pthread_attr_init(&attr);
  sch_param.sched_priority = 110;
  attr.stacksize = 1024;
  pthread_attr_setschedparam(&attr, &sch_param);

  ret = pthread_create(&ctx->controller_listener_pid,
                       &attr,
                       (pthread_startroutine_t)controller_listener_entry,
                       (pthread_addr_t)ctx);
  if (ret < 0)
    {
      mpcerr("pthread_create() failure: %d.\n", ret);
      return ret;
    }

  return ret;
}

static int controller_listener_uninitialize(mpcomm_supervisor_context_t *ctx)
{
  int ret;

  ret = pthread_cancel(ctx->controller_listener_pid);
  if (ret < 0)
    {
      mpcerr("pthread_create() failure: %d.\n", ret);
      return ret;
    }

  ret = pthread_join(ctx->controller_listener_pid, NULL);
  if (ret < 0)
    {
      mpcerr("pthread_create() failure: %d.\n", ret);
      return ret;
    }

  return ret;
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
      mpcerr("init_core() failure: %d.\n", ret);
      return ret;
    }

  ctx->helpers_cpuset = 0;

  for (i = 0; i < helper_num; i++)
    {
      ret = init_core(&ctx->helpers[i], filepath);
      if (ret < 0)
        {
          mpcerr("init_core() failure: %d.\n", ret);
          return ret;
        }

      ctx->helpers_cpuset |= (1 << ctx->helpers[i].cpuid);
    }

  ret = sem_init(&ctx->sem_done, 0, 0);
  if (ret < 0)
    {
      mpcerr("sem_init() failure: %d.\n", ret);
      return ret;
    }

  ret = controller_listener_initialize(ctx);
  if (ret < 0)
    {
      mpcerr("controller_listener_initialize() failure: %d.\n", ret);
      return ret;
    }

  mpcomm_init_msg_t init_msg;
  init_msg.mode = MPCOMM_MODE_CONTROLLER;
  init_msg.helpers_cpuset = ctx->helpers_cpuset;
  init_msg.loadaddr = ctx->controller.mptask.loadaddr;

  ret = mpmq_send(&ctx->controller.mq, MPCOMM_MSG_ID_INIT,
                  (uint32_t)&init_msg);
  if (ret < 0)
    {
      mpcerr("mpmq_send() failure: %d.\n", ret);
      return ret;
    }

  ret = mpcomm_supervisor_wait_controller_done(ctx, NULL);
  if (ret < 0)
    {
      mpcerr("mpcomm_supervisor_wait_controller_done() failure: %d.\n", ret);
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
          mpcerr("mpmq_send() failure: %d.\n", ret);
          return ret;
        }

      ret = supervisor_wait_helper_done(ctx, i);
      if (ret < 0)
        {
          mpcerr("supervisor_wait_helper_done() failure: %d.\n", ret);
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
      mpcerr("Failed to allocate context.\n");
      return -ENOMEM;
    }

  ret = supervisor_init(supervisor_context, filepath, helper_num);
  if (ret < 0)
    {
      mpcerr("supervisor_init() failure: %d.\n", ret);
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
      mpcerr("deinit_core() failure: %d.\n", ret);
      return ret;
    }

  for (i = 0; i < ctx->helper_num; i++)
    {
      ret = deinit_core(&ctx->helpers[i]);
      if (ret < 0)
        {
          mpcerr("deinit_core() failure: %d.\n", ret);
          return ret;
        }
    }

  ret = controller_listener_uninitialize(ctx);
  if (ret < 0)
    {
      mpcerr("controller_listener_uninitialize() failure: %d.\n", ret);
      return ret;
    }

  ret = sem_destroy(&ctx->sem_done);
  if (ret < 0)
    {
      mpcerr("sem_destroy() failure: %d.\n", ret);
      return ret;
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
      mpcerr("mpmq_send() failure: %d.\n", ret);
      return ret;
    }

  return ret;
}

int mpcomm_supervisor_wait_controller_done(mpcomm_supervisor_context_t *ctx,
                                           const struct timespec *abstime)
{
  int ret;

  if (abstime != NULL)
    {
      ret = sem_timedwait(&ctx->sem_done, abstime);
      if (ret < 0)
        {
          mpcerr("sem_timedwait() failure: %d.\n", ret);
          return ret;
        }
    }
  else
    {
      ret = sem_wait(&ctx->sem_done);
      if (ret < 0)
        {
          mpcerr("sem_wait() failure: %d.\n", ret);
          return ret;
        }
    }

  return ret;
}
