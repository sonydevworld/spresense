/****************************************************************************
 * modules/asmp/supervisor/mptask.c
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

#include <sys/stat.h>

#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <mm/tile.h>
#include <asmp/types.h>
#include <asmp/mptask.h>
#include <asmp/mpmq.h>
#include <asmp/mpshm.h>
#include <asmp/mpmutex.h>
#include <asmp/mpsignal.h>

#include <arch/chip/pm.h>

#include "up_arch.h"
#include "cxd56_clock.h"
#include "cxd56_sysctl.h"

#include "mptask.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Valid affinity CPU bit set mask */

#define CPUAFMASK 0x3eu

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static cpu_set_t g_freecpus;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int mptask_obj_is_duplicated(mptask_t *task, mpobj_t *obj)
{
  int i;

  if (!task || !obj)
    {
      return -EINVAL;
    }

  if (task->nbounds >= NMPBINDS)
    {
      return -ENOENT;
    }

  if (obj->key == 0)
    {
      return -EINVAL;
    }

  /* Check key is duplicated */

  for (i = 0; i < task->nbounds; i++)
    {
      if (task->bounds[i].key == obj->key)
        {
          return -EINVAL;
        }
    }

  return OK;
}

static off_t get_filelen(const char *filename)
{
  struct stat buf;
  int ret;

  ret = stat(filename, &buf);
  if (ret < 0)
    {
      return 0;
    }

  if (!S_ISREG(buf.st_mode))
    {
      return 0;
    }

  return buf.st_size;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mptask_cpu_free(mptask_t *task)
{
  mpinfo("Free CPU %d\n", task->cpuid);
  CPU_SET(task->cpuid, &g_freecpus);
}

int mptask_init(mptask_t *task, const char *filename)
{
  int fd;
  int i;
  off_t size;

  if (!task)
    {
      return -EINVAL;
    }

  memset(task, 0, sizeof(mptask_t));

  /* Set all of CPUs to permitted */

  for (i = 1; i < NMPCPUS; i++)
    {
      CPU_SET(i, &task->attr.affinity);
    }

  fd = open(filename, O_RDONLY);
  if (fd < 0)
    {
      return -ENOENT;
    }

  size = get_filelen(filename);
  if (!size)
    {
      close(fd);
      return -errno;
    }

  task->fd = fd;
  task->filelen = size;

  sem_init(&task->wait, 0, 0);

  return OK;
}

int mptask_attr_init(mptask_attr_t *attr)
{
  if (!attr)
    {
      return -EINVAL;
    }

  memset(attr, 0, sizeof(mptask_attr_t));

  return OK;
}

int mptask_bind(mptask_t *task, mpobj_t *obj)
{
  mpbindobj_t *bobj;
  int ret;

  /* Secure MP task can not be bound. */

  if (task_is_secure(task))
    {
      return -EPERM;
    }

  ret = mptask_obj_is_duplicated(task, obj);
  if (ret < 0)
    {
      return ret;
    }

  bobj = &task->bounds[task->nbounds];
  switch (obj->type)
    {
    case MPOBJTYPE_MQ:
      bobj->value = ((mpmq_t *)obj)->cpuid;
      break;

    case MPOBJTYPE_SHM:
      bobj->value = ((mpshm_t *)obj)->paddr;
      break;

    case MPOBJTYPE_MUTEX:
      bobj->value = ((mpmutex_t *)obj)->tag;
      break;

    default:
      return -EINVAL;
    }

  bobj->key = obj->key;
  bobj->type = obj->type;
  
  task->nbounds++;

  return OK;
}

int mptask_setattr(mptask_t *task, const mptask_attr_t *attr)
{
  if (!task || !attr)
    {
      return -EINVAL;
    }

  /* Check affinity */

  if (attr->affinity & ~CPUAFMASK)
    {
      return -EINVAL;
    }

  task->attr.affinity = attr->affinity;

  return OK;
}

int mptask_getattr(mptask_t *task, mptask_attr_t *attr)
{
  if (!task || !attr)
    {
      return -EINVAL;
    }

  memcpy(attr, &task->attr, sizeof(mptask_attr_t));

  return OK;
}

int mptask_assign(mptask_t *task)
{
  cpu_set_t can;
  int i;

  CPU_AND(&can, &g_freecpus, &task->attr.affinity);
  
  for (i = 1; i<NMPCPUS; i++)
    {
      if (CPU_ISSET(i, &can))
        {
          task->cpuid = i;
          CPU_CLR(i, &g_freecpus);

          mpinfo("Allocate CPU %d\n", task->cpuid);

          return OK;
        }
    }

  return -ENOENT;
}

cpuid_t mptask_getcpuid(mptask_t *task)
{
  return task->cpuid == 0 ? -ENOENT : task->cpuid + 2;
}

int mptask_join(mptask_t *task, int *exit_status)
{
  if (!task)
    {
      return -EINVAL;
    }

  if (exit_status)
    {
      *exit_status = -ENOTCONN;
    }

  if (task_is_exec(task))
    {
      mptask_semtake(&task->wait);
    }

  if (task_is_exit(task))
    {
      if (exit_status)
        {
          *exit_status = task->attr.exit_status;
        }

      return OK;
    }

  return -EPERM;
}

#ifdef SDK_EXPERIMENTAL

int mptask_pause(mptask_t *task)
{
  if (!task)
    {
      return -EINVAL;
    }

  if (!task_is_exec(task))
    {
      return -EPERM;
    }

  /*
   * REVISIT: Send pause signal to worker or tell status to worker.
   */

  /* Just disable to stop CPU */

  cxd56_cpu_clock_disable(task->cpuid);

  /* Permit hot sleep */

  up_pm_release_wakelock(&g_mptask_wlock);

  task_set_pause(task);

  return OK;
}

int mptask_restart(mptask_t *task)
{
  if (!task)
    {
      return -EINVAL;
    }

  if (!task_is_paused(task))
    {
      return -EPERM;
    }

  /* Suppress hot sleep */

  up_pm_acquire_wakelock(&g_mptask_wlock);

  cxd56_cpu_clock_enable(task->cpuid);

  task_set_exec(task);

  return OK;
}

#endif /* SDK_EXPERIMENTAL */

void mptask_initialize(void)
{
  /* Add all CPUs to free bit set */

  g_freecpus = CPUAFMASK;
}
