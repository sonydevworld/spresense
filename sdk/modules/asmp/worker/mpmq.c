/****************************************************************************
 * modules/asmp/worker/mpmq.c
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

#include <asmp/types.h>
#include <asmp/mpmq.h>
#include <asmp/mptask.h>

#include <errno.h>

#include "arm_arch.h"
#include "chip.h"

#include "arch/cpufifo.h"
#include "asmp.h"
#include "common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

union msg {
  uint32_t   word[2];
  struct {
    /* Little endian */
    
    uint16_t pid;
    uint8_t  msgid;
    uint8_t  proto:4;
    uint8_t  cpuid:4;
    
    uint32_t data;
  };
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Initialize MP message queue
 */

int mpmq_init(mpmq_t *mq, key_t key, cpuid_t cpu)
{
  mpbindobj_t *obj;

  if (!mq)
    {
      return -EINVAL;
    }

  wk_memset(mq, 0, sizeof(mpmq_t));

  mpobj_init(mq, MQ, key);

  if (key)
    {
      obj = asmp_findmpbindobj(MPOBJTYPE_MQ, key);
      if (!obj)
        {
          return -ENOENT;
        }
      mq->cpuid = (cpuid_t) obj->value;
    }
  else
    {
      if (cpu < 3 && cpu > 7)
        {
          return -EINVAL;
        }
      mq->cpuid = cpu;
    }

  /* If target CPU ID is same with worker CPU, then replace
   * to supervisor CPU ID (2).
   */

  if (mq->cpuid == asmp_getglobalcpuid())
    {
      mq->cpuid = 2;
    }

  return OK;
}

/**
 * Destroy MP message queue
 */

int mpmq_destroy(mpmq_t *mq)
{
  wk_memset(mq, 0, sizeof(mpmq_t));

  return OK;
}

/**
 * Send message via MP message queue
 */

int mpmq_send(mpmq_t *mq, int8_t msgid, uint32_t data)
{
  union msg m;
  int ret;

  m.cpuid = mq->cpuid;
  m.msgid = msgid;
  m.pid = 0;
  m.proto = 0;
  m.data = data;

  do
    {
      ret = cpufifo_push(m.word);
    }
  while(ret);
  
  return OK;
}

/**
 * Send message via MP message queue with timeout
 *
 * TODO: Implement timeout process
 */

int mpmq_timedsend(mpmq_t *mq, int8_t msgid, uint32_t data,
                   uint32_t ms)
{
  union msg m;
  int ret;

  m.cpuid = mq->cpuid;
  m.msgid = msgid;
  m.pid = 0;
  m.proto = 0;
  m.data = data;

  do
    {
      ret = cpufifo_push(m.word);
    }
  while(ret);
  
  return OK;
}

/**
 * Receive message via MP message queue
 */

int mpmq_receive(mpmq_t *mq, uint32_t *data)
{
  union msg m;
  int ret;

  do
    {
      ret = cpufifo_pull(PROTO_MSG, m.word);
    }
  while(ret);

  *data = m.data;
  
  return m.msgid;
}

/**
 * Receive message via MP message queue with timeout
 */

int mpmq_timedreceive(mpmq_t *mq, uint32_t *data, uint32_t ms)
{
  union msg m;
  int ret;

  do
    {
      ret = cpufifo_pull(PROTO_MSG, m.word);
    }
  while(ret);

  *data = m.data;
  
  return m.msgid;
}
