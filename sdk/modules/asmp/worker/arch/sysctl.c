/****************************************************************************
 * modules/asmp/worker/arch/sysctl.c
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

#include <sdk/config.h>

#include <stdint.h>

#include "cpufifo.h"
#include "sysctl.h"

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

int sysctl(uint8_t id, uint32_t data)
{
  union msg m;
  int ret;

  /* sysctl always throw to system CPU */

  m.cpuid = 0;
  m.msgid = id;
  m.proto = 12;
  m.pid   = 0xffff;
  m.data  = data;

  /* Send any message to system CPU */

  do
    {
      ret = cpufifo_push(m.word);
    }
  while(ret);

  /* Wait for reply message from system CPU */

  do
    {
      ret = cpufifo_pull(PROTO_SYSCTL, m.word);
    }
  while (ret);

  ret = (int)m.data;

  return ret;
}
