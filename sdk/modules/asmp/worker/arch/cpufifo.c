/****************************************************************************
 * modules/asmp/worker/arch/cpufifo.c
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

#include <stdint.h>

#include <asmp/types.h>

#include "arm_arch.h"
#include "chip.h"

#include "arch/cpufifo.h"

#define CXD56_FIF_PUSH_FULL    (CXD56_CPUFIFO_BASE + 0x00)
#define CXD56_FIF_PUSH_WRD0    (CXD56_CPUFIFO_BASE + 0x04)
#define CXD56_FIF_PUSH_WRD1    (CXD56_CPUFIFO_BASE + 0x08)
#define CXD56_FIF_PUSH_CMP     (CXD56_CPUFIFO_BASE + 0x0c)
#define CXD56_FIF_PULL_EMP     (CXD56_CPUFIFO_BASE + 0x10)
#define CXD56_FIF_PULL_WRD0    (CXD56_CPUFIFO_BASE + 0x14)
#define CXD56_FIF_PULL_WRD1    (CXD56_CPUFIFO_BASE + 0x18)
#define CXD56_FIF_PULL_CMP     (CXD56_CPUFIFO_BASE + 0x1c)

#define sighead(id, no, data) ((((id) & 0xf) << 28) | (0xf << 24) | ((no) << 16) | ((data) & 0xffff))

#define PROTOCOL(x)      (((x)[0] >> 24) & 0xf)

struct cpufifo_msg_s
{
  int valid;
  uint32_t word[2];
};

static struct cpufifo_msg_s g_msgbuf;
static struct cpufifo_msg_s g_sysctlbuf;

int cpufifo_push(uint32_t data[2])
{
  if (getreg32(CXD56_FIF_PUSH_FULL))
    {
      return -1;
    }

  putreg32(data[0], CXD56_FIF_PUSH_WRD0);
  putreg32(data[1], CXD56_FIF_PUSH_WRD1);
  putreg32(1, CXD56_FIF_PUSH_CMP);

  return 0;
}

int cpufifo_pull(int proto, uint32_t data[2])
{
  struct cpufifo_msg_s *m = NULL;
  uint32_t buf[2];

  if (!getreg32(CXD56_FIF_PULL_EMP))
    {
      int p;

      buf[0] = getreg32(CXD56_FIF_PULL_WRD0);
      buf[1] = getreg32(CXD56_FIF_PULL_WRD1);
      putreg32(1, CXD56_FIF_PULL_CMP);

      /* Store to message buffer, if multiple message has been received for
       * each protocols, then overwritten.
       */

      p = PROTOCOL(buf);
      if (p == PROTO_MSG)
        {
          m = &g_msgbuf;
        }
      else if (p == PROTO_SYSCTL)
        {
          m = &g_sysctlbuf;
        }
      else
        {
          return -1;
        }

      m->valid = 1;
      m->word[0] = buf[0];
      m->word[1] = buf[1];
    }

  if (proto == PROTO_MSG)
    {
      m = &g_msgbuf;
    }
  else if (proto == PROTO_SYSCTL)
    {
      m = &g_sysctlbuf;
    }

  if (m && m->valid)
    {
      m->valid = 0;
      data[0] = m->word[0];
      data[1] = m->word[1];
    }
  else
    {
      return -1;
    }

  return 0;
}

void _signal(int8_t cpuid, int8_t signo, uint32_t data)
{
  uint32_t word[2];
  word[0] = sighead(cpuid, signo, 0);
  word[1] = data;

  while(cpufifo_push(word) < 0);
}
