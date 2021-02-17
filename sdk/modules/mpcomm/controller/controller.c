/****************************************************************************
 * mpcomm/controller/controller.c
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

#include <stdint.h>

#include <asmp.h>

#include <mpcomm/controller.h>

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static controller_context_t controller_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static controller_context_t *get_controller_context(void)
{
  return &controller_ctx;
}

static int controller_send_supervisor_done(void)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_DONE,
                  asmp_getglobalcpuid());

  return ret;
}

static int controller_init_msg(controller_init_msg_t *msg)
{
  int ret;
  int helper_idx = 0;
  cpuid_t cpuid;
  controller_context_t *ctx = get_controller_context();

  for (cpuid = 3; cpuid < MPCOMM_CPUID_MAX; ++cpuid)
    {
      if ((1 << cpuid) & msg->helpers_cpuset)
        {
          ret = mpmq_init(&ctx->helpers[helper_idx].mq, 0, cpuid);
          if (ret < 0)
            {
              return ret;
            }

          ctx->helpers[helper_idx].cpuid = cpuid;
          ++helper_idx;
        }
    }

  ctx->helpers_cpuset = msg->helpers_cpuset;
  ctx->helpers_doneset = msg->helpers_cpuset;
  ctx->helpers_num = helper_idx;
  ctx->loadaddr = msg->loadaddr;

  ret = controller_send_supervisor_done();

  return ret;
}

static int controller_user_func_msg(void *data)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  ctx->user_func(data);

  ret = controller_send_supervisor_done();

  return ret;
}

static int controller_done_msg(cpuid_t cpuid)
{
  controller_context_t *ctx = get_controller_context();

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

static int controller_unknown_msg(void)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_ERROR,
                  asmp_getglobalcpuid());

  return ret;
}

static int controller_handle_msg(int id, void *data)
{
  int ret = 0;
  controller_context_t *ctx = get_controller_context();

  switch (id)
    {
      case MPCOMM_MSG_ID_INIT:
        ret = controller_init_msg((controller_init_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_DEINIT:
        ctx->quit_loop = 1;
        break;
      case MPCOMM_MSG_ID_USER_FUNC:
        ret = controller_user_func_msg(data);
        break;
      case MPCOMM_MSG_ID_DONE:
        ret = controller_done_msg((cpuid_t)(uint32_t)data);
        break;
      default:
        ret = controller_unknown_msg();
        break;
    }

  return ret;
}

static void controller_init(void)
{
  controller_context_t *ctx = get_controller_context();

  mpmq_init(&ctx->mq_2_supervisor, 0, asmp_getglobalcpuid());
}

static void controller_loop(void)
{
  int msgid;
  uint32_t msgdata;
  controller_context_t *ctx = get_controller_context();

  msgid = mpmq_receive(&ctx->mq_2_supervisor, &msgdata);

  controller_handle_msg(msgid, (void *)msgdata);
}

static int controller_wait_supervisor_done(void)
{
  controller_context_t *ctx = get_controller_context();

  while (!ctx->supervisor_done && !ctx->quit_loop)
    {
      controller_loop();
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int controller_main(controller_user_func_t user_func)
{
  controller_context_t *ctx = get_controller_context();

  controller_init();

  ctx->user_func = user_func;
  ctx->quit_loop = 0;

  while (!ctx->quit_loop)
    {
      controller_loop();
    }

  return 0;
}

int controller_send_helper(uint8_t helper_index, void *data)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  ctx->helpers_doneset &= ~(1 << ctx->helpers[helper_index].cpuid);

  ret = mpmq_send(&ctx->helpers[helper_index].mq, MPCOMM_MSG_ID_USER_FUNC,
                  (uint32_t)data);

  return ret;
}

int controller_wait_helper_done(uint8_t helper_index)
{
  int ret = 0;
  controller_context_t *ctx = get_controller_context();

  while (((ctx->helpers_doneset
         & (1 << ctx->helpers[helper_index].cpuid)) == 0)
         && !ctx->quit_loop)
    {
      controller_loop();
    }

  if (ctx->quit_loop)
    {
      ret = 1;
    }

  return ret;
}

int controller_wait_helpers_done(void)
{
  int ret = 0;
  controller_context_t *ctx = get_controller_context();

  while ((ctx->helpers_doneset != ctx->helpers_cpuset) && !ctx->quit_loop)
    {
      controller_loop();
    }

  if (ctx->quit_loop)
    {
      ret = 1;
    }

  return ret;
}

int controller_get_helpers_num(void)
{
  controller_context_t *ctx = get_controller_context();

  return ctx->helpers_num;
}

void *controller_memory_virt_to_phys(void *addr)
{
  controller_context_t *ctx = get_controller_context();

  return addr + (uint32_t)ctx->loadaddr;
}

int controller_send_malloc(void **ptr, size_t size)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  supervisor_malloc_msg_t supervisor_malloc_msg;
  supervisor_malloc_msg.ptr = ptr;
  supervisor_malloc_msg.size = size;

  ctx->supervisor_done = 0;

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_MALLOC,
                  (uint32_t)MEM_V2P(&supervisor_malloc_msg));

  controller_wait_supervisor_done();

  return ret;
}

int controller_send_free(void *ptr)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  supervisor_free_msg_t supervisor_free_msg;
  supervisor_free_msg.ptr = ptr;

  ctx->supervisor_done = 0;

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_FREE,
                  (uint32_t)MEM_V2P(&supervisor_free_msg));

  controller_wait_supervisor_done();

  return ret;
}

int controller_send_error(int err)
{
  int ret;
  controller_context_t *ctx = get_controller_context();

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_ERROR, err);

  return ret;
}
