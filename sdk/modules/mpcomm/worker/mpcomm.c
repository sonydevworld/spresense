/****************************************************************************
 * mpcomm/worker/mpcomm.c
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

#include <asmp.h>

#include <mpcomm/mpcomm.h>

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static mpcomm_context_t mpcomm_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static mpcomm_context_t *get_mpcomm_context(void)
{
  return &mpcomm_ctx;
}

static int mpcomm_send_supervisor_done(void)
{
  int ret;
  mpcomm_context_t *ctx = get_mpcomm_context();

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_DONE,
                  asmp_getglobalcpuid());

  return ret;
}

static int mpcomm_send_controller_done(void)
{
  int ret;
  mpcomm_context_t *ctx = get_mpcomm_context();

  ret = mpmq_send(&ctx->mq_2_controller, MPCOMM_MSG_ID_DONE,
                  asmp_getglobalcpuid());

  return ret;
}

static int mpcomm_init_msg(mpcomm_init_msg_t *msg)
{
  int ret;
  int helper_idx = 0;
  cpuid_t cpuid;
  mpcomm_context_t *ctx = get_mpcomm_context();

  for (cpuid = 3; cpuid < MPCOMM_CPUID_MAX; ++cpuid)
    {
      if ((1 << cpuid) & msg->helpers_cpuset)
        {
          if (msg->mode == MPCOMM_MODE_CONTROLLER)
            {
              ret = mpmq_init(&ctx->helpers[helper_idx].mq, 0, cpuid);
              if (ret < 0)
                {
                  return ret;
                }
            }

          ctx->helpers[helper_idx].cpuid = cpuid;
          ++helper_idx;
        }
    }

  ctx->helpers_cpuset = msg->helpers_cpuset;
  ctx->helpers_doneset = msg->helpers_cpuset;
  ctx->helpers_num = helper_idx;

  if (msg->mode == MPCOMM_MODE_HELPER)
    {
      ret = mpmq_init(&ctx->mq_2_controller, 0, msg->controller_cpuid);
      if (ret < 0)
        {
          return ret;
        }
    }

  ctx->mode = msg->mode;
  ctx->loadaddr = msg->loadaddr;

  ret = mpcomm_send_supervisor_done();

  return ret;
}

static int mpcomm_user_func_msg(void *data)
{
  int ret;
  mpcomm_context_t *ctx = get_mpcomm_context();

  if (mpcomm_is_controller())
    {
      ctx->controller_user_func(data);

      ret = mpcomm_send_supervisor_done();
    }
  else
    {
      ctx->helper_user_func(data);

      ret = mpcomm_send_controller_done();
    }

  return ret;
}

static int mpcomm_done_msg(cpuid_t cpuid)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  if (cpuid == 2)
    {
      ctx->supervisor_done = 1;
    }
  else
    {
      ctx->helpers_doneset |= (1 << cpuid);
    }

  return 0;
}

static int mpcomm_unknown_msg(void)
{
  int ret;

  ret = mpcomm_send_error(-EINVAL);

  return ret;
}

static int mpcomm_handle_msg(int id, void *data)
{
  int ret = 0;
  mpcomm_context_t *ctx = get_mpcomm_context();

  switch (id)
    {
      case MPCOMM_MSG_ID_INIT:
        ret = mpcomm_init_msg((mpcomm_init_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_DEINIT:
        ctx->quit_loop = 1;
        break;
      case MPCOMM_MSG_ID_USER_FUNC:
        ret = mpcomm_user_func_msg(data);
        break;
      case MPCOMM_MSG_ID_DONE:
        ret = mpcomm_done_msg((cpuid_t)(uint32_t)data);
        break;
      case MPCOMM_MSG_ID_MALLOC:
      case MPCOMM_MSG_ID_FREE:
      case MPCOMM_MSG_ID_ERROR:
      case MPCOMM_MSG_ID_LOG:
        if (mpcomm_is_controller())
          {
            ret = mpmq_send(&ctx->mq_2_supervisor, id, (uint32_t)data);
          }
        else
          {
            ret = mpcomm_unknown_msg();
          }
        break;
      default:
        ret = mpcomm_unknown_msg();
        break;
    }

  return ret;
}

static void mpcomm_init(void)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  mpmq_init(&ctx->mq_2_supervisor, 0, asmp_getglobalcpuid());
}

static void mpcomm_loop(void)
{
  int msgid;
  uint32_t msgdata;
  mpcomm_context_t *ctx = get_mpcomm_context();

  msgid = mpmq_receive(&ctx->mq_2_supervisor, &msgdata);

  mpcomm_handle_msg(msgid, (void *)msgdata);
}

static int mpcomm_wait_supervisor_done(void)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  while (!ctx->supervisor_done && !ctx->quit_loop)
    {
      mpcomm_loop();
    }

  return 0;
}

static int mpcomm_send_msg(int8_t id, void *msg)
{
  int ret;
  mpcomm_context_t *ctx = get_mpcomm_context();

  ctx->supervisor_done = 0;

  if (mpcomm_is_controller())
    {
      ret = mpmq_send(&ctx->mq_2_supervisor, id,
                      (uint32_t)MEM_V2P(msg));
    }
  else
    {
      ret = mpmq_send(&ctx->mq_2_controller, id,
                      (uint32_t)MEM_V2P(msg));
    }

  mpcomm_wait_supervisor_done();

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mpcomm_main(mpcomm_user_func_t controller_user_func,
                mpcomm_user_func_t helper_user_func)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  mpcomm_init();

  ctx->controller_user_func = controller_user_func;
  ctx->helper_user_func = helper_user_func;
  ctx->quit_loop = 0;

  while (!ctx->quit_loop)
    {
      mpcomm_loop();
    }

  return 0;
}

int mpcomm_send_helper(uint8_t helper_index, void *data)
{
  int ret;
  mpcomm_context_t *ctx = get_mpcomm_context();

  if (mpcomm_is_controller())
    {
      ctx->helpers_doneset &= ~(1 << ctx->helpers[helper_index].cpuid);

      ret = mpmq_send(&ctx->helpers[helper_index].mq,
                      MPCOMM_MSG_ID_USER_FUNC,
                      (uint32_t)data);
    }
  else
    {
      ret = -EPERM;
    }

  return ret;
}

int mpcomm_wait_helper_done(uint8_t helper_index)
{
  int ret = 0;
  mpcomm_context_t *ctx = get_mpcomm_context();

  if (mpcomm_is_controller())
    {
      while (((ctx->helpers_doneset
            & (1 << ctx->helpers[helper_index].cpuid)) == 0)
            && !ctx->quit_loop)
        {
          mpcomm_loop();
        }

      if (ctx->quit_loop)
        {
          ret = -ECANCELED;
        }
    }
  else
    {
      ret = -EPERM;
    }

  return ret;
}

int mpcomm_wait_helpers_done(void)
{
  int ret = 0;
  mpcomm_context_t *ctx = get_mpcomm_context();

  if (mpcomm_is_controller())
    {
      while ((ctx->helpers_doneset != ctx->helpers_cpuset)
             && !ctx->quit_loop)
        {
          mpcomm_loop();
        }

      if (ctx->quit_loop)
        {
          ret = -ECANCELED;
        }
    }
  else
    {
      ret = -EPERM;
    }

  return ret;
}

int mpcomm_get_helpers_num(void)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  return ctx->helpers_num;
}

bool mpcomm_is_controller(void)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  return ctx->mode == MPCOMM_MODE_CONTROLLER;
}

void *mpcomm_memory_virt_to_phys(void *addr)
{
  mpcomm_context_t *ctx = get_mpcomm_context();

  return (uintptr_t)addr < CONFIG_RAM_START ?
           addr + (uint32_t)ctx->loadaddr : addr;
}

int mpcomm_send_malloc(void **ptr, size_t size)
{
  int ret;
  *ptr = NULL;

  mpcomm_malloc_msg_t malloc_msg;
  malloc_msg.cpuid = asmp_getglobalcpuid();
  malloc_msg.ptr = MEM_V2P(ptr);
  malloc_msg.size = size;

  ret = mpcomm_send_msg(MPCOMM_MSG_ID_MALLOC, (void *)&malloc_msg);

  return ret;
}

int mpcomm_send_free(void *ptr)
{
  int ret;

  mpcomm_free_msg_t free_msg;
  free_msg.cpuid = asmp_getglobalcpuid();
  free_msg.ptr = ptr;

  ret = mpcomm_send_msg(MPCOMM_MSG_ID_FREE, (void *)&free_msg);

  return ret;
}

int mpcomm_send_error(int err)
{
  int ret;

  mpcomm_error_msg_t error_msg;
  error_msg.cpuid = asmp_getglobalcpuid();
  error_msg.error = err;

  ret = mpcomm_send_msg(MPCOMM_MSG_ID_ERROR, (void *)&error_msg);

  return ret;
}

int mpcomm_send_log(char *log)
{
  int ret;

  mpcomm_log_msg_t log_msg;
  log_msg.cpuid = asmp_getglobalcpuid();
  log_msg.log = MEM_V2P(log);

  ret = mpcomm_send_msg(MPCOMM_MSG_ID_LOG, (void *)&log_msg);

  return ret;
}
