/****************************************************************************
 * mpcomm/helper/helper.c
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

#include <asmp.h>

#include <mpcomm/helper.h>

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static helper_context_t helper_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static helper_context_t *get_helper_context(void)
{
  return &helper_ctx;
}

static int helper_send_supervisor_done(void)
{
  int ret;
  helper_context_t *ctx = get_helper_context();

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_DONE,
                  asmp_getglobalcpuid());

  return ret;
}

static int helper_init_msg(helper_init_msg_t *msg)
{
  int ret;
  helper_context_t *ctx = get_helper_context();

  ret = mpmq_init(&ctx->mq_2_controller, 0, msg->controller_cpuid);
  if (ret < 0)
    {
      return ret;
    }

  ctx->loadaddr = msg->loadaddr;

  ret = helper_send_supervisor_done();
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

static int helper_send_controller_done(void)
{
  int ret;
  helper_context_t *ctx = get_helper_context();

  ret = mpmq_send(&ctx->mq_2_controller, MPCOMM_MSG_ID_DONE,
                  asmp_getglobalcpuid());

  return ret;
}

static int helper_user_func_msg(void *data)
{
  int ret;
  helper_context_t *ctx = get_helper_context();

  ctx->user_func(data);

  ret = helper_send_controller_done();

  return ret;
}

static int helper_unknown_msg(void)
{
  int ret;
  helper_context_t *ctx = get_helper_context();

  ret = mpmq_send(&ctx->mq_2_supervisor, MPCOMM_MSG_ID_ERROR,
                  asmp_getglobalcpuid());

  return ret;
}

static int helper_handle_msg(int id, void *data)
{
  int ret = 0;
  helper_context_t *ctx = get_helper_context();

  switch (id)
    {
      case MPCOMM_MSG_ID_INIT:
        ret = helper_init_msg((helper_init_msg_t *)data);
        break;
      case MPCOMM_MSG_ID_DEINIT:
        ctx->quit_loop = 1;
        break;
      case MPCOMM_MSG_ID_USER_FUNC:
        ret = helper_user_func_msg(data);
        break;
      default:
        ret = helper_unknown_msg();
        break;
    }

  return ret;
}

static void helper_init(void)
{
  helper_context_t *ctx = get_helper_context();

  mpmq_init(&ctx->mq_2_supervisor, 0, asmp_getglobalcpuid());
}

static void helper_loop(void)
{
  int msgid;
  uint32_t msgdata;
  helper_context_t *ctx = get_helper_context();

  msgid = mpmq_receive(&ctx->mq_2_supervisor, &msgdata);

  helper_handle_msg(msgid, (void *)msgdata);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int helper_main(helper_user_func_t user_func)
{
  helper_context_t *ctx = get_helper_context();

  helper_init();

  ctx->user_func = user_func;
  ctx->quit_loop = 0;

  while (!ctx->quit_loop)
    {
      helper_loop();
    }

  return 0;
}

void *helper_memory_virt_to_phys(void *addr)
{
  helper_context_t *ctx = get_helper_context();

  return addr + (uint32_t)ctx->loadaddr;
}
