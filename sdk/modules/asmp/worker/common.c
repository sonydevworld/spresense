/****************************************************************************
 * modules/asmp/worker/common.c
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

#include <stdio.h> /* size_t */
#include <errno.h>

#include <asmp/types.h>
#include <asmp/mptask.h>
#include <asmp/mpsignal.h>

#include "arch/intrinsics.h"
#include "arch/cpufifo.h"
#include "common.h"

/* Enough to hold any data from supervisor */

#define WORKER_RESERVED_AREA   128

char __attribute__((aligned(4)))
mpframework_reserved[WORKER_RESERVED_AREA] = "RSVD";

void *wk_memset(void *s, int c, size_t n)
{
  unsigned char *p = (unsigned char *)s;
  while (n-- > 0)
    {
      *p++ = c;
    }
  return s;
}

void *wk_memcpy(void *dest, const void *src, size_t n)
{
  unsigned char *pout = (unsigned char *)dest;
  unsigned char *pin  = (unsigned char *)src;

  while (n-- > 0)
    {
      *pout++ = *pin++;
    }

  return dest;
}

void wk_exit(int status)
{
  _signal(2, MPSIGEXIT, (uint32_t)status);

  for (;;)
    {
      wfi();
    }

  /* NOTREACHED */
}

mpbindobj_t *asmp_findmpbindobj(mpobjtype_t type, key_t key)
{
  mpbindobj_t *obj = (mpbindobj_t *) mpframework_reserved;
  int i;

  for (i = 0; i < NMPBINDS; i++, obj++)
    {
      if (obj->type == type && obj->key == key)
        {
          return obj;
        }
    }

  return NULL;
}
