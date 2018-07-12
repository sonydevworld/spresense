/****************************************************************************
 * modules/asmp/supervisor/mpmq.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <asmp/types.h>
#include <asmp/mpmq.h>

#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "cxd56_icc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPMQ_TIMEDOUT    (1<<0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mpmq_do_send(mpmq_t *mq, int8_t msgid, uint32_t data, int32_t ms)
{
  iccmsg_t msg;

  msg.cpuid = mq->cpuid;
  msg.msgid = msgid;
  msg.data  = data;

  return cxd56_iccsendmsg(&msg, ms);
}

static int mpmq_do_recv(mpmq_t *mq, uint32_t *data, int32_t ms)
{
  iccmsg_t msg;
  int ret;

  msg.cpuid = mq->cpuid;

  ret = cxd56_iccrecvmsg(&msg, ms);
  if (ret < 0)
    {
      return ret;
    }

  if (data)
    {
      *data = msg.data;
    }
  
  return msg.msgid;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Initialize MP message queue
 */

int mpmq_init(mpmq_t *mq, key_t key, cpuid_t cpu)
{
  if (!mq || !key)
    {
      return -EINVAL;
    }
  if (cpu < 0 && cpu >= 8)
    {
      return -EINVAL;
    }

  memset(mq, 0, sizeof(mpmq_t));
  mpobj_init(mq, MQ, key);

  mq->cpuid = cpu;

  return cxd56_iccinitmsg(mq->cpuid);
}

/**
 * Destroy MP message queue
 */

int mpmq_destroy(mpmq_t *mq)
{
  if (!mq)
    {
      return -EINVAL;
    }

  cxd56_iccuninitmsg(mq->cpuid);

  memset(mq, 0, sizeof(mpmq_t));
  mq->cpuid = -1;

  return OK;
}

/**
 * Send message via MP message queue
 */

int mpmq_send(mpmq_t *mq, int8_t msgid, uint32_t data)
{
  if (!mq)
    {
      return -EINVAL;
    }

  return mpmq_do_send(mq, msgid, data, 0);
}

/**
 * Send message via MP message queue with timeout
 */

int mpmq_timedsend(mpmq_t *mq, int8_t msgid, uint32_t data,
                   uint32_t ms)
{
  if (!mq)
    {
      return -EINVAL;
    }

  return mpmq_do_send(mq, msgid, data, ms);
}

/**
 * Receive message via MP message queue
 */

int mpmq_receive(mpmq_t *mq, uint32_t *data)
{
  if (!mq)
    {
      return -EINVAL;
    }

  return mpmq_do_recv(mq, data, 0);
}

/**
 * Receive message via MP message queue with timeout
 */

int mpmq_timedreceive(mpmq_t *mq, uint32_t *data, uint32_t ms)
{
  if (!mq)
    {
      return -EINVAL;
    }

  return mpmq_do_recv(mq, data, ms);
}

/**
 * Request POSIX signal for message arrival.
 */

int mpmq_notify(mpmq_t *mq, int signo, void *sigdata)
{
  if (!mq)
    {
      return -EINVAL;
    }

  return cxd56_iccnotify(mq->cpuid, signo, sigdata);
}
