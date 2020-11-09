/****************************************************************************
 * modules/asmp/worker/mpmutex.c
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
#include <arch/irq.h>

#include <asmp/types.h>
#include <asmp/mpmutex.h>
#include <asmp/mptask.h>

#include <errno.h>

#include "arm_arch.h"
#include "chip.h"

#include "hardware/cxd56_sph.h"

#include "asmp.h"
#include "common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sph_state_unlocked(sts)   (STS_STATE(sts) == STATE_IDLE)
#define sph_state_locked(sts)     (STS_STATE(sts) == STATE_LOCKED)
#define sph_state_busy(sts)       (STS_STATE(sts) == STATE_LOCKEDANDRESERVED)

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Initialize MP mutex
 */

int mpmutex_init(mpmutex_t *mutex, key_t key)
{
  mpbindobj_t *obj;

  if (!mutex || !key)
    {
      return -EINVAL;
    }

  wk_memset(mutex, 0, sizeof(mpmutex_t));

  obj = asmp_findmpbindobj(MPOBJTYPE_MUTEX, key);
  if (!obj)
    {
      return -ENOENT;
    }

  mpobj_init(mutex, MUTEX, key);
  mutex->tag = obj->value;

  return OK;
}

/**
 * Destroy MP mutex
 */

int mpmutex_destroy(mpmutex_t *mutex)
{
  if (!mutex)
    {
      return -EINVAL;
    }

  mutex->tag = -1;

  return OK;
}

/**
 * Lock MP mutex
 */

int mpmutex_lock(mpmutex_t *mutex)
{
  int ret;

  for (;;)
    {
      ret = mpmutex_trylock(mutex);
      if (ret != -EBUSY)
        {
          return ret;
        }
    }

  /* NOTREACHED */
}

/**
 * Try to lock MP mutex
 */

int mpmutex_trylock(mpmutex_t *mutex)
{
  uint32_t sts;
  cpuid_t cpu;
  int tag;

  if (!mutex)
    {
      return -EINVAL;
    }

  tag = mutex->tag;
  cpu = asmp_getglobalcpuid();

  sts = getreg32(CXD56_SPH_STS(tag));
  if (sph_state_unlocked(sts))
    {
      putreg32(REQ_LOCK, CXD56_SPH_REQ(tag));

      /* If lock owner ID is my CPU number, it is successful locked. */

      sts = getreg32(CXD56_SPH_STS(tag));
      if (sph_state_locked(sts) && LOCK_OWNER(sts) == cpu)
        {
          return OK;
        }
    }

  return -EBUSY;
}

/**
 * Unlock MP mutex
 */

int mpmutex_unlock(mpmutex_t *mutex)
{
  if (!mutex)
    {
      return -EINVAL;
    }

  putreg32(REQ_UNLOCK, CXD56_SPH_REQ(mutex->tag));

  return OK;
}
