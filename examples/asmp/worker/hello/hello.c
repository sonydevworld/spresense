/****************************************************************************
 * asmp/worker/hello/hello.c
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
#include <errno.h>

#include <asmp/types.h>
#include <asmp/mpshm.h>
#include <asmp/mpmutex.h>
#include <asmp/mpmq.h>

#include "asmp.h"

/* MP object keys. Must be synchronized with supervisor. */

#define KEY_SHM   1
#define KEY_MQ    2
#define KEY_MUTEX 3

#define MSG_ID_SAYHELLO 1

#define ASSERT(cond) if (!(cond)) wk_abort()

static const char helloworld[] = "Hello, ASMP World!";

static char *strcopy(char *dest, const char *src)
{
  char *d = dest;
  while (*src) *d++ = *src++;
  *d = '\0';

  return dest;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  mpmutex_t mutex;
  mpshm_t shm;
  mpmq_t mq;
  uint32_t msgdata;
  char *buf;
  int ret;

  /* Initialize MP Mutex */

  ret = mpmutex_init(&mutex, KEY_MUTEX);
  ASSERT(ret == 0);

  /* Initialize MP message queue,
   * On the worker side, 3rd argument is ignored.
   */

  ret = mpmq_init(&mq, KEY_MQ, 0);
  ASSERT(ret == 0);

  /* Initialize MP shared memory */

  ret = mpshm_init(&shm, KEY_SHM, 1024);
  ASSERT(ret == 0);

  /* Map shared memory to virtual space */

  buf = (char *)mpshm_attach(&shm, 0);
  ASSERT(buf);

  /* Receive message from supervisor */

  ret = mpmq_receive(&mq, &msgdata);
  ASSERT(ret == MSG_ID_SAYHELLO);

  /* Copy hello message to shared memory */

  mpmutex_lock(&mutex);
  strcopy(buf, helloworld);
  mpmutex_unlock(&mutex);

  /* Free virtual space */

  mpshm_detach(&shm);

  /* Send done message to supervisor */

  msgdata ^= 0xffffffff;
  ret = mpmq_send(&mq, MSG_ID_SAYHELLO, msgdata);
  ASSERT(ret == 0);

  return 0;
}
