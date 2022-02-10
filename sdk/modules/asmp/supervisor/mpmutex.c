/****************************************************************************
 * modules/asmp/supervisor/mpmutex.c
 *
 *   Copyright 2018,2021 Sony Semiconductor Solutions Corporation
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

#include <nuttx/config.h>
#include <assert.h>
#include <debug.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <asmp/types.h>
#include <asmp/mpmutex.h>

#include <stdio.h>
#include <errno.h>

#include "arm_arch.h"
#include "chip.h"

#include "cxd56_sph.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ASMP_DEBUG_ERROR
#  define mperr(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define mperr(fmt, ...)
#endif
#ifdef CONFIG_ASMP_DEBUG_WARN
#  define mpwarn(fmt, ...)  _warn(fmt, ## __VA_ARGS__)
#else
#  define mpwarn(fmt, ...)
#endif
#ifdef CONFIG_ASMP_DEBUG_INFO
#  define mpinfo(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define mpinfo(fmt, ...)
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* Manage free hardware semaphore bitmap */

static uint32_t g_freemutexes;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpmutex_mutexalloc
 *
 * Description:
 *   Allocate free hardware semaphore. Searching bitmap for faster allocation.
 * 
 ****************************************************************************/

static inline int mpmutex_mutexalloc(void)
{
  int8_t i;
  
  for (i = 0; i < 16; i++)
    {
      if (g_freemutexes & (1 << i))
        {
          g_freemutexes &= ~(1 << i);
          return i;
        }
    }

  return -1;
}

/**
 * mpmutex_mutexfree
 */

static inline void mpmutex_mutexfree(int8_t tag)
{
  ASSERT(tag >= 0 && tag < 16);
  g_freemutexes |= (1 << tag);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Initialize MP mutex
 */

int mpmutex_init(mpmutex_t *mutex, key_t key)
{
  char fullpath[64];
  int tag;
  int fd;
  irqstate_t flags;

  if (!mutex || !key)
    {
      return -EINVAL;
    }

  mpobj_init(mutex, MUTEX, key);

  flags = enter_critical_section();

  tag = mpmutex_mutexalloc();
  if (tag < 0)
    {
      leave_critical_section(flags);
      return -ENOENT;
    }

  snprintf(fullpath, sizeof(fullpath), "/dev/hsem%d", tag);
  fd = open(fullpath, 0);
  leave_critical_section(flags);
  if (fd < 0)
    {
      mperr("%s not found.\n", fullpath);
      mpmutex_mutexfree(tag);
      return -ENOENT;
    }

  mutex->fd = fd;
  mutex->tag = tag;

  return OK;
}

/**
 * Destroy MP mutex
 */

int mpmutex_destroy(mpmutex_t *mutex)
{
  irqstate_t flags;

  if (!mutex)
    {
      return -EINVAL;
    }

  /* Close hardware semaphore device */

  close(mutex->fd);

  /* Free fast search bitmap */

  flags = enter_critical_section();
  mpmutex_mutexfree(mutex->tag);
  leave_critical_section(flags);

  mutex->tag = -1;
  mutex->fd = -1;

  return OK;
}

/**
 * Lock MP mutex
 */

int mpmutex_lock(mpmutex_t *mutex)
{
  int ret;

  if (!mutex)
    {
      return -EINVAL;
    }

  ret = ioctl(mutex->fd, HSLOCK, 0);
  if (ret != 0)
    {
      return -errno;
    }

  return OK;
}

/**
 * Try to lock MP mutex
 */

int mpmutex_trylock(mpmutex_t *mutex)
{
  int ret;

  if (!mutex)
    {
      return -EINVAL;
    }

  ret = ioctl(mutex->fd, HSTRYLOCK, 0);
  if (ret != 0)
    {
      return -errno;
    }

  return OK;
}

/**
 * Unlock MP mutex
 */

int mpmutex_unlock(mpmutex_t *mutex)
{
  int ret;

  if (!mutex)
    {
      return -EINVAL;
    }

  ret = ioctl(mutex->fd, HSUNLOCK, 0);
  if (ret != 0)
    {
      return -errno;
    }

  return OK;
}

/**
 * Initialize mpmutex
 */

void mpmutex_initialize(void)
{
  if (cxd56_sphinitialize("hsem") != 0)
    {
      mperr("Hardware semaphore initialization failure.\n");
    }

  g_freemutexes = 0x7ff8;
}
