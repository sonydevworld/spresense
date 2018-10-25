/****************************************************************************
 * modules/lte/util/thrdpool.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "thrdpool.h"
#include "dbg_if.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define THRDPOOL_THRDNAME_MAX_LEN (16)
#define THRDPOOL_CONDWAIT(handle, timeout) \
  do { sys_wait_semaphore(&handle, timeout); } while (0)
#define THRDPOOL_CONDSIGNAL(handle) \
  do { sys_post_semaphore(&handle); } while (0)
#define THRDPOOL_INIT_BIN_SEM_SET(param) \
  { \
    (param).initial_count = 0; \
    (param).max_count     = 1; \
  }
/****************************************************************************
 * Private Types
 ****************************************************************************/

enum thrdpool_thrdstate_e
{
  THRDPOOL_WAITING = 0,
  THRDPOOL_RUNNABLE
};

struct thrdpool_queelements_s
{
  CODE thrdpool_jobif_t job;
  FAR void              *arg;
};

struct thrdpool_share_s
{
  sys_mq_t          quehandle;
  sys_thread_cond_t delwaitcond;
  sys_mutex_t       delwaitcondmtx;
};

struct thrdpool_info_s
{
  FAR struct thrdpool_share_s *share;
  sys_task_t                  thrdhandle;
  enum thrdpool_thrdstate_e   state;
};

struct thrdpool_datatable_s
{
  FAR struct thrdpool_s      thrdpoolif;
  uint16_t                   maxthrdnum;
  uint16_t                   maxquenum;
  struct thrdpool_share_s    share;
  FAR struct thrdpool_info_s *thrdinfolist;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t thrdpool_runjob(
  FAR struct thrdpool_s *thiz, CODE thrdpool_jobif_t job, FAR void *arg);
static uint32_t thrdpool_getfreethrds(FAR struct thrdpool_s *thiz);
static void thrdpool_thrdmain(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thrdpool_runjob
 *
 * Description:
 *   Enqueues the processing that the thread does.
 *
 * Input Parameters:
 *   thiz  struct thrdpool_s pointer(i.e. instance of threadpool).
 *   job   Pointer to the processing function conforming to the job_if.
 *   arg   argument of @job.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

static int32_t thrdpool_runjob(
  FAR struct thrdpool_s *thiz, CODE thrdpool_jobif_t job, FAR void *arg)
{
  FAR struct thrdpool_datatable_s *table = NULL;
  int32_t                         ret    = 0;
  struct thrdpool_queelements_s   element;

  if (!thiz || !job)
    {
      DBGIF_LOG_ERROR("Incorrect argument.\n");
      return -EINVAL;
    }

  table = (FAR struct thrdpool_datatable_s*)thiz;
  element.job = job;
  element.arg = arg;

  ret = sys_send_mqueue(&table->share.quehandle, (FAR int8_t *)&element,
    sizeof(struct thrdpool_queelements_s), SYS_TIMEO_FEVR);
  DBGIF_ASSERT(0 == ret, "Queue send failed.\n");

  return 0;
}

/****************************************************************************
 * Name: thrdpool_getfreethrds
 *
 * Description:
 *   Get number of available threads.
 *
 * Input Parameters:
 *   thiz  struct thrdpool_s pointer(i.e. instance of threadpool).
 *
 * Returned Value:
 *   Number of available threads.
 *
 ****************************************************************************/

static uint32_t thrdpool_getfreethrds(FAR struct thrdpool_s *thiz)
{
  FAR struct thrdpool_datatable_s *table = NULL;
  uint16_t                        num    = 0;
  uint16_t                        count  = 0;

  if (!thiz)
    {
      DBGIF_LOG_ERROR("thiz pointer is NULL.\n");
      return 0;
    }

  table = (FAR struct thrdpool_datatable_s*)thiz;
  for (num = 0; num < table->maxthrdnum; num++)
    {
      if (table->thrdinfolist[num].state == THRDPOOL_WAITING)
        {
          count++;
        }
    }

  return count;
}

/****************************************************************************
 * Name: thrdpool_thrdmain
 *
 * Description:
 *   The main loop of the thread to create. 
 *
 * Input Parameters:
 *   arg  Information for the thread to operate.(i.e. struct thrdpool_info_s)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void thrdpool_thrdmain(FAR void *arg)
{
  FAR struct thrdpool_info_s    *info = (FAR struct thrdpool_info_s*)arg;
  struct thrdpool_queelements_s recvbuf;

  while(1)
    {
      recvbuf.job = NULL;
      recvbuf.arg = NULL;
      if (sys_recv_mqueue(&info->share->quehandle, (FAR int8_t *)&recvbuf,
        sizeof(struct thrdpool_queelements_s), SYS_TIMEO_FEVR) < 0)
        {
          DBGIF_LOG_ERROR("Receive failed from queue.\n");
          continue;
        }

      if (recvbuf.job == NULL)
        {

          /* Receive delete packet. */

          break;
        }

      info->state = THRDPOOL_RUNNABLE;

      /* Perform actual processing. */

      recvbuf.job(recvbuf.arg);

      info->state = THRDPOOL_WAITING;
    }

  sys_signal_thread_cond(&info->share->delwaitcond,
                         &info->share->delwaitcondmtx);
  sys_delete_task(SYS_OWN_TASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thrdpool_create
 *
 * Description:
 *   Create an object of threadpool and get the instance.
 *
 * Input Parameters:
 *   set  Threadpool create setting.
 *
 * Returned Value:
 *   struct thrdpool_s pointer(i.e. instance of threadpool).
 *
 ****************************************************************************/

FAR struct thrdpool_s *thrdpool_create(FAR struct thrdpool_set_s *set)
{
  static uint16_t                 thrdcount   = 0;
  FAR struct thrdpool_datatable_s *table      = NULL;
  uint16_t                        num         = 0;
  sys_cretask_s                   thread_param;
  sys_cremq_s                     que_param;
  char                            thrdname[THRDPOOL_THRDNAME_MAX_LEN];

  if (!set || set->maxthrdnum <= 0 || set->maxquenum <= 0)
    {
      DBGIF_LOG_ERROR("Incorrect argument.\n");
      errno = EINVAL;
      goto errout;
    }

  /* Create data table. */

  table = (FAR struct thrdpool_datatable_s *)
    SYS_MALLOC(sizeof(struct thrdpool_datatable_s));
  if (!table)
    {
      DBGIF_LOG_ERROR("thrdpool_s table allocate failed.\n");
      errno = ENOMEM;
      goto errout;
    }

  memset(table, 0, sizeof(*table));
  table->maxthrdnum = set->maxthrdnum;

  /* Set interface. */

  table->thrdpoolif.runjob       = thrdpool_runjob;
  table->thrdpoolif.getfreethrds = thrdpool_getfreethrds;

  /* Create queue. */

  que_param.numof_queue = set->maxquenum;
  que_param.queue_size  = sizeof(struct thrdpool_queelements_s);
  if (sys_create_mqueue(&table->share.quehandle, &que_param) < 0)
    {
      DBGIF_LOG_ERROR("Queue create failed.\n");
      goto errout_with_tablefree;
    }

  if (sys_create_thread_cond_mutex(&table->share.delwaitcond,
                                   &table->share.delwaitcondmtx) < 0)
    {
      DBGIF_LOG_ERROR("sys_create_thread_cond_mutex failed.\n");
      goto errout_with_quedelete;
    }

  /* Create threads data. */
  
  thread_param.function   = thrdpool_thrdmain;
  thread_param.name       = (FAR int8_t *)thrdname;
  thread_param.priority   = set->thrdpriority;
  thread_param.stack_size = set->thrdstacksize;
  table->thrdinfolist     = (FAR struct thrdpool_info_s *)
    SYS_MALLOC(sizeof(struct thrdpool_info_s) * table->maxthrdnum);
  if (!table->thrdinfolist)
    {
      DBGIF_LOG_ERROR("thrdinfolist create failed.\n");
      goto errout_with_semdelete;
    }

  /* Create threads */

  for (num = 0; num < table->maxthrdnum; num++)
    {
      table->thrdinfolist[num].share = &table->share;
      table->thrdinfolist[num].state = THRDPOOL_WAITING;
      thread_param.arg = (FAR void *)&table->thrdinfolist[num];
      snprintf(thrdname, sizeof(thrdname),
        "thrdpool_no%02d", (int)(++thrdcount));
      if (sys_create_task(&table->thrdinfolist[num].thrdhandle,
            &thread_param) < 0)
        {
          DBGIF_LOG1_ERROR("thread create failed. number:%d\n", num);
          goto errout_with_thrddelete;
        }
    }

  return (FAR struct thrdpool_s *)table;

errout_with_thrddelete:
  for (; 0 < num; num--)
    {
      sys_delete_task(&table->thrdinfolist[num - 1].thrdhandle);
    }

  SYS_FREE(table->thrdinfolist);
errout_with_semdelete:
  sys_delete_thread_cond_mutex(&table->share.delwaitcond,
                               &table->share.delwaitcondmtx);
errout_with_quedelete:
  sys_delete_mqueue(&table->share.quehandle);
errout_with_tablefree:
  SYS_FREE(table);
  errno = ENOMEM;
errout:
  return NULL;
}

/****************************************************************************
 * Name: thrdpool_delete
 *
 * Description:
 *   Delete object of threadpool.
 *
 * Input Parameters:
 *   thiz  struct thrdpool_s pointer(i.e. instance of threadpool).
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t thrdpool_delete(FAR struct thrdpool_s *thiz)
{
  FAR struct thrdpool_datatable_s *table = NULL;
  uint16_t                        num    = 0;
  struct thrdpool_queelements_s   element;
  int32_t                         ret    = 0;

  if (!thiz)
    {
      DBGIF_LOG_ERROR("thiz pointer is NULL.\n");
      return -EINVAL;
    }

  table = (FAR struct thrdpool_datatable_s *)thiz;
  element.job = NULL;

  for (num = 0; num < table->maxthrdnum; num++)
    {
      sys_lock_mutex(&table->share.delwaitcondmtx);

      /* Send delete request to thread main */

      ret = sys_send_mqueue(&table->share.quehandle, (FAR int8_t *)&element,
        sizeof(struct thrdpool_queelements_s), SYS_TIMEO_FEVR);
      DBGIF_ASSERT(0 == ret, "Queue send failed.");

      sys_thread_cond_wait(&table->share.delwaitcond,
                           &table->share.delwaitcondmtx);

      sys_unlock_mutex(&table->share.delwaitcondmtx);
    }

  DBGIF_LOG_DEBUG("All thread delete success.\n");
  sys_delete_thread_cond_mutex(&table->share.delwaitcond,
                               &table->share.delwaitcondmtx);
  sys_delete_mqueue(&table->share.quehandle);
  SYS_FREE(table->thrdinfolist);
  SYS_FREE(table);
  return 0;
}
