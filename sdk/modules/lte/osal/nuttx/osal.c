/****************************************************************************
 * modules/lte/osal/nuttx/osal.c
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

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "osal.h"
#include "dbg_if.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SEM_PSHARED      0
#define MQ_NAME_PREFIX   "osal"
#define MQ_CMODE (S_IWOTH | S_IROTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR)
#define MQ_PRIO          0
/* There is a relationship that the priority of the receiving task must be
 * higher than the priority of the SPI transfer task.
 */

#if defined(CONFIG_LTE_CALLBACK_TASK_PRIORITY) && \
    defined(CONFIG_MODEM_ALTMDM_XFER_TASK_PRIORITY)
#  define TASK_PRIO_LOW    50
#  define TASK_PRIO_NORMAL (CONFIG_LTE_CALLBACK_TASK_PRIORITY)
#  define TASK_PRIO_HIGH   (CONFIG_MODEM_ALTMDM_XFER_TASK_PRIORITY + 10)
#  if ((TASK_PRIO_LOW > TASK_PRIO_NORMAL) || \
       (TASK_PRIO_HIGH < TASK_PRIO_NORMAL))
#    error CONFIG_LTE_CALLBACK_TASK_PRIORITY is out of range
#  endif
#  if (TASK_PRIO_HIGH > SCHED_PRIORITY_MAX)
#    error CONFIG_MODEM_ALTMDM_XFER_TASK_PRIORITY is too high (<245)
#  endif
#else
#  define TASK_PRIO_LOW    50
#  define TASK_PRIO_NORMAL 100
#  define TASK_PRIO_HIGH   180
#endif
#define TASK_PRIO_MIN    (SYS_TASK_PRIO_LOW)
#define TASK_PRIO_MAX    (SYS_TASK_PRIO_HIGH)
#define NUM_OF_TASK_PRIO 3
#define MYADDR_LEN       9       /* 32bit + '\0' */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct task_info_s
{
  char      myaddr[MYADDR_LEN];
  FAR void  *arg;
  CODE void (*function)(FAR void *arg);
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_entry
 *
 * Description:
 *   A new task's entry.
 *
 * Input Parameters:
 *   argc  Number of arguments for user task's entry.
 *   argv  Argument list for user task's entry..
 *
 * Returned Value:
 *   If the procedure successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

static int task_entry(int argc, FAR char *argv[])
{
  FAR struct task_info_s *addr;
  void                   *arg;
  CODE void              (*func)(FAR void *arg);
  char                   *endptr;

  if (argc != 2)
    {
      DBGIF_LOG_ERROR("Invalid argument\n");
      return -EINVAL;
    }

  /* It converts from a character string to a numeric value */

  addr = (struct task_info_s *)strtol(argv[1], &endptr, 16);

  if (errno != ERANGE)
    {
      if ((*endptr != '\0'))
        {
          DBGIF_LOG_ERROR("No digits were found\n");
          return -EINVAL;
        }
      else if (addr == NULL)
        {
          DBGIF_LOG_ERROR("value is null\n");
          return -EINVAL;
        }
    }
  else if ((long)addr == LONG_MAX)
    {
      DBGIF_LOG_ERROR("strtol overflow\n");
      return -ERANGE;
    }
  else if ((long)addr == LONG_MIN)
    {
      DBGIF_LOG_ERROR("strtol underflow\n");
      return -ERANGE;
    }

  /* Save it to a local variables before free it */

  arg  = addr->arg;
  func = addr->function;

  /* Free task infometion before enter the user entry */

  SYS_FREE(addr);

  DBGIF_LOG3_DEBUG("get addr: %08x arg:%08x func:%08x\n", addr, arg, func);

  /* Enter the user entry */

  func(arg);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sys_create_task
 *
 * Description:
 *   Create a new task.
 *
 * Input Parameters:
 *   task   Used to pass a handle to the created task
 *          out of the sys_create_task() function.
 *   params A value that will passed into the created task
 *          as the task's parameter.
 *
 * Returned Value:
 *   If the task was created successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_create_task(FAR sys_task_t *task,
                        FAR const sys_cretask_s *params)
{
  int32_t                ret;
  int32_t                prio;
  char                   *arg[2];
  FAR struct task_info_s *param;
  const int32_t          prio_table[NUM_OF_TASK_PRIO] =
    {
      TASK_PRIO_LOW,
      TASK_PRIO_NORMAL,
      TASK_PRIO_HIGH
    };

  if ((TASK_PRIO_MIN <= params->priority) &&
      (params->priority <= TASK_PRIO_MAX))
    {
      prio = prio_table[params->priority];
    }
  else
    {
      DBGIF_LOG1_ERROR("Invalid parameter:%d\n", params->priority);

      return -EINVAL;
    }

  param = (FAR struct task_info_s *)SYS_MALLOC(sizeof(struct task_info_s));
  if (!param)
    {
      DBGIF_LOG_ERROR("Memory allocation failure\n");
      return -ENOMEM;
    }

  snprintf(param->myaddr, MYADDR_LEN, "%08x", param);
  param->myaddr[MYADDR_LEN - 1] = '\0';
  param->arg = params->arg;
  param->function = params->function;

  arg[0] = &param->myaddr[0];
  arg[1] = NULL;

  DBGIF_LOG3_DEBUG("my addr: %s arg:%08x func:%08x\n", param->myaddr, params->arg, params->function);

  ret = task_create((FAR const char *)params->name, prio, params->stack_size,
                    task_entry, (FAR char * const *)arg);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to create task:%d errno:%d\n", ret);
      SYS_FREE(param);
      return ret;
    }

  *task = ret;

  return 0;
}

/****************************************************************************
 * Name: sys_delete_task
 *
 * Description:
 *   Delete a specified task.
 *
 * Input Parameters:
 *   task   The handle of the task to be deleted.
 *          If task set to SYS_OWN_TASK then delete caller own task.
 *
 * Returned Value:
 *   If the task was deleted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_delete_task(FAR sys_task_t *task)
{
  int32_t ret;

  if (task == SYS_OWN_TASK)
    {
      ret = task_delete(0);
    }
  else
    {
      ret = task_delete(*task);
    }
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to delete task:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_sleep_task
 *
 * Description:
 *   Make self task sleep.
 *
 * Input Parameters:
 *   timeout_ms   The sleep time in milliseconds.
 *
 * Returned Value:
 *   If the task was slept successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_sleep_task(int32_t timeout_ms)
{
  usleep(timeout_ms * 1000);

  return 0;
}


/****************************************************************************
 * Name: sys_enable_dispatch
 *
 * Description:
 *   Resume the scheduler.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   If the scheduler was resumed successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_enable_dispatch(void)
{
  return sched_unlock();
}

/****************************************************************************
 * Name: sys_disable_dispatch
 *
 * Description:
 *   Suspend the scheduler.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   If the scheduler was suspended successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_disable_dispatch(void)
{
  return sched_lock();
}

/****************************************************************************
 * Name: sys_create_semaphore
 *
 * Description:
 *   Create a new semaphore.
 *
 * Input Parameters:
 *   sem   Used to pass a handle to the created semaphore
 *          out of the sys_create_semaphore() function.
 *   params A value that will passed into the created semaphore
 *          as the semaphore's parameter.
 *
 * Returned Value:
 *   If the semaphore was created successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_create_semaphore(FAR sys_sem_t *sem,
                             FAR const sys_cresem_s *params)
{
  int32_t ret;

  ret = sem_init(sem, SEM_PSHARED, params->initial_count);
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to initialize semaphore:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_delete_semaphore
 *
 * Description:
 *   Delete a specified semaphore.
 *
 * Input Parameters:
 *   sem   The handle of the semaphore to be deleted.
 *
 * Returned Value:
 *   If the semaphore was deleted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_delete_semaphore(FAR sys_sem_t *sem)
{
  int32_t ret;

  ret = sem_destroy(sem);
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to delete semaphore:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_wait_semaphore
 *
 * Description:
 *   Waiting a semaphore for become available.
 *
 * Input Parameters:
 *   sem        The handle of the semaphore to be waited.
 *   timeout_ms The time in milliseconds to wait for the semaphore
 *              to become available.
 *              If timeout_ms set to LTE_SYS_TIMEO_FEVR then wait until
 *              the semaphore to become available.
 *
 * Returned Value:
 *   If the semaphore was become available then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_wait_semaphore(FAR sys_sem_t *sem, int32_t timeout_ms)
{
  int32_t         ret;
  int32_t         l_errno;
  struct timespec abs_time;
  struct timespec curr_time;

  if (timeout_ms == SYS_TIMEO_FEVR)
    {
      for (;;)
        {
          ret = sem_wait(sem);
          if (ret < 0)
            {
              l_errno = errno;
              if (l_errno == EINTR)
                {
                  continue;
                }

              DBGIF_LOG2_ERROR("Failed to wait semaphore:%d errno:%d\n", ret, l_errno);
              return -l_errno;
            }
          break;
        }
    }
  else
    {
      /* Get current time. */

      ret = clock_gettime(CLOCK_REALTIME, &curr_time);
      if (ret != OK)
        {
          DBGIF_LOG2_ERROR("Failed to get time:%d errno:%d\n", ret, errno);
          return -errno;
        }

      abs_time.tv_sec = timeout_ms / 1000;
      abs_time.tv_nsec =
        (timeout_ms - (abs_time.tv_sec * 1000)) * 1000 * 1000;

      abs_time.tv_sec += curr_time.tv_sec;
      abs_time.tv_nsec += curr_time.tv_nsec;

      /* Check more than 1 sec. */

      if (abs_time.tv_nsec >= (1000 * 1000 * 1000))
        {
          abs_time.tv_sec += 1;
          abs_time.tv_nsec -= (1000 * 1000 * 1000);
        }

      ret = sem_timedwait(sem, &abs_time);
      if (ret < 0)
        {
          DBGIF_LOG2_ERROR("Failed to wait semaphore:%d errno:%d\n", ret, errno);
          return -errno;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: sys_post_semaphore
 *
 * Description:
 *   Post a semaphore.
 *
 * Input Parameters:
 *   sem The handle of the semaphore to be posted.
 *
 * Returned Value:
 *   If the semaphore was posted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_post_semaphore(FAR sys_sem_t *sem)
{
  int32_t ret;

  ret = sem_post(sem);
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to post semaphore:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_create_mutex
 *
 * Description:
 *   Create a new mutex.
 *
 * Input Parameters:
 *   mutex  Used to pass a handle to the created mutex
 *          out of the sys_create_mutex() function.
 *   params A value that will passed into the created mutex
 *          as the mutex's parameter.
 *
 * Returned Value:
 *   If the mutex was created successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_create_mutex(FAR sys_mutex_t *mutex,
                         FAR const sys_cremtx_s *params)
{
  int32_t ret;

  ret = pthread_mutex_init(mutex, NULL);
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to initialize mutex:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_delete_mutex
 *
 * Description:
 *   Delete a specified mutex.
 *
 * Input Parameters:
 *   mutex The handle of the mutex to be deleted.
 *
 * Returned Value:
 *   If the mutex was deleted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_delete_mutex(FAR sys_mutex_t *mutex)
{
  int32_t ret;

  ret = pthread_mutex_destroy(mutex);
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to destory mutex:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_lock_mutex
 *
 * Description:
 *   Lock a mutex.
 *
 * Input Parameters:
 *   mutex The handle of the mutex to be locked.
 *
 * Returned Value:
 *   If the mutex was locked successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_lock_mutex(FAR sys_mutex_t *mutex)
{
  int32_t ret;
  int32_t l_errno;

  for (;;)
    {
      ret = pthread_mutex_lock(mutex);
      if (ret < 0)
        {
          l_errno = errno;
          if (l_errno == EINTR)
            {
              continue;
            }
          DBGIF_LOG2_ERROR("Failed to lock mutex:%d errno:%d\n", ret, l_errno);
          return -l_errno;
        }
      break;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_unlock_mutex
 *
 * Description:
 *   Unlock a mutex.
 *
 * Input Parameters:
 *   mutex The handle of the mutex to be unlocked.
 *
 * Returned Value:
 *   If the mutex was unlocked successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_unlock_mutex(FAR sys_mutex_t *mutex)
{
  int32_t ret;

  ret = pthread_mutex_unlock(mutex);
  if (ret < 0)
    {
      
      DBGIF_LOG2_ERROR("Failed to unlock mutex:%d errno:%d\n", ret, errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_create_mqueue
 *
 * Description:
 *   Create a new message queue.
 *
 * Input Parameters:
 *   mq     Used to pass a handle to the created message queue
 *          out of the sys_create_mqueue() function.
 *   params A value that will passed into the created message queue
 *          as the message queue's parameter.
 *
 * Returned Value:
 *   If the message queue was created successfully then 0 is
 *   returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_create_mqueue(FAR sys_mq_t *mq, FAR const sys_cremq_s *params)
{
  static uint32_t suffix_num = 0;
  struct mq_attr  mq_attr;
  mqd_t           mqd;

  memset(mq, 0, sizeof(sys_mq_t));
  memset(&mq_attr, 0, sizeof(struct mq_attr));

  sched_lock();
  snprintf((char*)mq->name, sizeof(mq->name), MQ_NAME_PREFIX"%d", suffix_num);
  suffix_num++;
  sched_unlock();

  mq_attr.mq_maxmsg  = params->numof_queue;
  mq_attr.mq_msgsize = params->queue_size;
  mq_attr.mq_flags   = 0;

  mqd = mq_open((char*)mq->name, (O_RDWR | O_CREAT), MQ_CMODE, &mq_attr);
  if (mqd == ((mqd_t)-1))
    {
      DBGIF_LOG2_ERROR("Failed to initialize mq:%d errno:%d\n", mqd, errno);
      return -errno;
    }

  /* Close message queue here, because other sys_xxx_mqueue() may be called
   * by different task context.
   */

  mq_close(mqd);

  return 0;

}

/****************************************************************************
 * Name: sys_delete_mqueue
 *
 * Description:
 *   Delete a specified message queue.
 *
 * Input Parameters:
 *   mq The handle of the message queue to be deleted.
 *
 * Returned Value:
 *   If the message queue was deleted successfully then 0 is
 *   returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_delete_mqueue(FAR sys_mq_t *mq)
{
  int32_t ret;

  ret = mq_unlink((char*)mq->name);
  if (ret < 0)
    {
      DBGIF_LOG2_ERROR("Failed to unlink mq:%d errno:%d\n", ret, errno);
      return -errno;
    }

  memset(mq, 0, sizeof(sys_mq_t));

  return 0;
}

/****************************************************************************
 * Name: sys_send_mqueue
 *
 * Description:
 *   Send message by a message queue.
 *
 * Input Parameters:
 *   mq         The handle of the message queue to be send.
 *   message    The message to be send.
 *   len        The length of the message.
 *   timeout_ms The time in milliseconds to block until send timeout occurs.
 *              If timeout_ms set to LTE_SYS_TIMEO_FEVR then wait until
 *              the message queue to become available.
 *
 * Returned Value:
 *   If the message queue was sent successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_send_mqueue(FAR sys_mq_t *mq, FAR int8_t *message, size_t len,
                        int32_t timeout_ms)
{
  int32_t         ret;
  int32_t         l_errno;
  struct timespec abs_time;
  struct timespec curr_time;
  mqd_t           mqd;

  /* Need to mq_open() and close() each time this function called.
   * Because task context which called this function may be different.
   */

  mqd = mq_open((char *)mq->name, O_WRONLY);

  if (mqd < 0)
    {
      l_errno = errno;
      DBGIF_LOG1_ERROR("Failed to mq_open errno:%d\n", l_errno);
      return -l_errno;
    }

  if (timeout_ms == SYS_TIMEO_FEVR)
    {
      for (;;)
        {
          ret = mq_send(mqd, (const char*)message, len, MQ_PRIO);
          if (ret < 0)
            {
              l_errno = errno;
              if (l_errno == EINTR)
                {
                  continue;
                }
              DBGIF_LOG2_ERROR("Failed to send mq:%d errno:%d\n", ret, l_errno);
              mq_close(mqd);
              return -l_errno;
            }
          break;
        }
    }
  else
    {
      /* Get current time. */

      ret = clock_gettime(CLOCK_REALTIME, &curr_time);
      if (ret != OK)
        {
          l_errno = errno;
          DBGIF_LOG2_ERROR("Failed to get time:%d errno:%d\n", ret, l_errno);
          mq_close(mqd);
          return -l_errno;
        }

      abs_time.tv_sec = timeout_ms / 1000;
      abs_time.tv_nsec =
        (timeout_ms - (abs_time.tv_sec * 1000)) * 1000 * 1000;

      abs_time.tv_sec += curr_time.tv_sec;
      abs_time.tv_nsec += curr_time.tv_nsec;

      /* Check more than 1 sec. */

      if (abs_time.tv_nsec >= (1000 * 1000 * 1000))
        {
          abs_time.tv_sec += 1;
          abs_time.tv_nsec -= (1000 * 1000 * 1000);
        }

      ret = mq_timedsend(mqd, (const char*)message, len, MQ_PRIO,
                         &abs_time);
      if (ret < 0)
        {
          l_errno = errno;
          DBGIF_LOG2_ERROR("Failed to send mq:%d errno:%d\n", ret, l_errno);
          mq_close(mqd);
          return -l_errno;
        }
    }

  mq_close(mqd);

  return 0;
}

/****************************************************************************
 * Name: sys_recv_mqueue
 *
 * Description:
 *   Receive message from a message queue.
 *
 * Input Parameters:
 *   mq         The handle of the mq to be received.
 *   message    The buffer to be received.
 *   len        The length of the buffer.
 *   timeout_ms The time in milliseconds to block until receive timeout
 *              occurs.
 *              If timeout_ms set to LTE_SYS_TIMEO_FEVR then wait until
 *              the message queue to become not empty.
 *
 * Returned Value:
 *   If the message queue was received successfully then the length of the
 *   selected message in bytes is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_recv_mqueue(FAR sys_mq_t *mq, FAR int8_t *message, size_t len,
                        int32_t timeout_ms)
{
  int32_t         ret;
  int32_t         l_errno;
  unsigned int    prio;
  struct timespec abs_time;
  struct timespec curr_time;
  mqd_t           mqd;

  /* Need to mq_open() and close() each time this function called.
   * Because task context which called this function may be different.
   */

  mqd = mq_open((char *)mq->name, O_RDONLY);

  if (timeout_ms == SYS_TIMEO_FEVR)
    {
      for (;;)
        {
          ret = mq_receive(mqd, (char*)message, len, &prio);
          if (ret < 0)
            {
              l_errno = errno;
              if (l_errno == EINTR)
                {
                  continue;
                }
              DBGIF_LOG2_ERROR("Failed to receive mq:%d errno:%d\n", ret, l_errno);
              mq_close(mqd);
              return -l_errno;
            }
          break;
        }
    }
  else
    {
      /* Get current time. */

      ret = clock_gettime(CLOCK_REALTIME, &curr_time);
      if (ret != OK)
        {
          l_errno = errno;
          DBGIF_LOG2_ERROR("Failed to get time:%d errno:%d\n", ret, l_errno);
          mq_close(mqd);
          return -l_errno;
        }

      abs_time.tv_sec = timeout_ms / 1000;
      abs_time.tv_nsec =
        (timeout_ms - (abs_time.tv_sec * 1000)) * 1000 * 1000;

      abs_time.tv_sec += curr_time.tv_sec;
      abs_time.tv_nsec += curr_time.tv_nsec;

      /* Check more than 1 sec. */

      if (abs_time.tv_nsec >= (1000 * 1000 * 1000))
        {
          abs_time.tv_sec += 1;
          abs_time.tv_nsec -= (1000 * 1000 * 1000);
        }

      ret = mq_timedreceive(mqd, (char*)message, len, &prio, &abs_time);
      if (ret < 0)
        {
          l_errno = errno;
          DBGIF_LOG2_ERROR("Failed to receive mq:%d errno:%d\n", ret, l_errno);
          mq_close(mqd);
          return -l_errno;
        }
    }

  mq_close(mqd);

  return ret;
}

/****************************************************************************
 * Name: sys_create_evflag
 *
 * Description:
 *   Create a new event flag.
 *
 * Input Parameters:
 *   flag   Used to pass a handle to the created event flag
 *          out of the sys_create_evflag() function.
 *   params A value that will passed into the created event flag
 *          as the event flag's parameter.
 *
 * Returned Value:
 *   If the event flag was created successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_create_evflag(FAR sys_evflag_t *flag,
                          FAR const sys_creevflag_s *params)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_delete_evflag
 *
 * Description:
 *   Delete a specified event flag.
 *
 * Input Parameters:
 *   flag The handle of the event flag to be deleted.
 *
 * Returned Value:
 *   If the event flag was deleted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_delete_evflag(FAR sys_evflag_t *flag)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_wait_evflag
 *
 * Description:
 *   Waiting a event flag to be set.
 *
 * Input Parameters:
 *   flag       The handle of the event flag to be waited.
 *   wptn       The wait bit pattern.
 *   wmode      The wait mode. If wmode set to LTE_SYS_WMODE_TWF_ANDW then
 *              the release condition requires all the bits in wptn
 *              to be set.
 *              If wmode set to LTE_SYS_WMODE_TWF_ORW then the release 
 *              condition only requires at least one bit in wptn to be set.
 *   flagptn    The current bit pattern.
 *   autoclr    Whether or not to clear bits automatically
 *              when function returns.
 *   timeout_ms The time in milliseconds to wait for the event flag
 *              to become set.
 *              If timeout_ms set to LTE_SYS_TIMEO_FEVR then wait until
 *              the event flag to become set.
 *
 * Returned Value:
 *   If the event flag was waited successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_wait_evflag(FAR sys_evflag_t *flag, sys_evflag_ptn_t wptn,
                        sys_evflag_mode_t wmode, bool autoclr,
                        FAR sys_evflag_ptn_t *flagptn, int32_t timeout_ms)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_set_evflag
 *
 * Description:
 *   Set a specified event flag.
 *
 * Input Parameters:
 *   flag       The handle of the event flag to be waited.
 *   setptn     The set bit pattern.
 *
 * Returned Value:
 *   If the event flag was seted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_set_evflag(FAR sys_evflag_t *flag, sys_evflag_ptn_t setptn)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_set_evflag_isr
 *
 * Description:
 *   Set a specified event flag. A version of sys_set_evflag() that can be
 *   called from an interrupt service routine(ISR).
 *
 * Input Parameters:
 *   flag       The handle of the event flag to be waited.
 *   setptn     The set bit pattern.
 *
 * Returned Value:
 *   If the event flag was seted successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_set_evflag_isr(FAR sys_evflag_t *flag, sys_evflag_ptn_t setptn)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_clear_evflag
 *
 * Description:
 *   Clear a specified event flag.
 *
 * Input Parameters:
 *   flag       The handle of the event flag to be waited.
 *   clrptn     The clear bit pattern.
 *
 * Returned Value:
 *   If the event flag was cleared successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_clear_evflag(FAR sys_evflag_t *flag, sys_evflag_ptn_t clrptn)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_start_timer
 *
 * Description:
 *   Create and start timer.
 *
 * Input Parameters:
 *   timer        The handle of the timer to be started.
 *   period_ms    The period of the timer in milliseconds.
 *   autoreload   Whether or not ro start the timer repeatedly.
 *   callback     The function called when the timer expired.
 *
 * Returned Value:
 *   If the timer was started successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_start_timer(FAR sys_timer_t *timer,
                        uint32_t period_ms, bool autoreload,
                        CODE sys_timer_cb_t callback)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_stop_timer
 *
 * Description:
 *   Stop and delete timer.
 *
 * Input Parameters:
 *   timer        The handle of the timer to be stopped.
 *
 * Returned Value:
 *   If the timer was started successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_stop_timer(FAR sys_timer_t *timer)
{
  /* Currently not support */

  return 0;
}

/****************************************************************************
 * Name: sys_thread_cond_init
 *
 * Description:
 *   The sys_thread_cond_init() function shall initialize the condition
 *   variable referenced by cond with attributes referenced by attr.
 *   If attr is NULL, the default condition variable attributes shall be
 *   used.
 *
 * Input Parameters:
 *   cond        Condition variable.
 *   cond_attr   Condition attributes.
 *
 * Returned Value:
 *   If successful, shall return zero.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_thread_cond_init(FAR sys_thread_cond_t *cond,
                             FAR sys_thread_condattr_t *cond_attr)
{
  int32_t ret;

  ret = pthread_cond_init(cond, cond_attr);
  if (ret != 0)
    {
      DBGIF_LOG1_ERROR("Failed to initialize thread condition:%d\n", ret);
      return -ret;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_thread_cond_destroy
 *
 * Description:
 *   The sys_thread_cond_destroy() function shall destroy the given
 *   condition variable specified by cond.
 *
 * Input Parameters:
 *   cond        Condition variable.
 *
 * Returned Value:
 *   If successful, shall return zero.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_thread_cond_destroy(FAR sys_thread_cond_t *cond)
{
  int32_t ret;

  ret = pthread_cond_destroy(cond);
  if (ret != 0)
    {
      DBGIF_LOG1_ERROR("Failed to destroy thread condition:%d\n", ret);
      return -ret;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_thread_cond_wait
 *
 * Description:
 *   The sys_thread_cond_destroy() functions shall block on a condition
 *   variable.
 *
 * Input Parameters:
 *   cond        Condition variable.
 *   mutex       The handle of the mutex.
 *
 * Returned Value:
 *   If successful, shall return zero.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_thread_cond_wait(FAR sys_thread_cond_t *cond,
                             FAR sys_mutex_t *mutex)
{
  return sys_thread_cond_timedwait(cond, mutex, SYS_TIMEO_FEVR);
}

/****************************************************************************
 * Name: sys_thread_cond_wait
 *
 * Description:
 *   The pthread_cond_timedwait() functions shall block on a condition
 *   variable.
 *
 * Input Parameters:
 *   cond        Condition variable.
 *   mutex       The handle of the mutex.
 *   timeout_ms  The time in milliseconds to wait.
 *
 * Returned Value:
 *   If successful, shall return zero.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_thread_cond_timedwait(FAR sys_thread_cond_t *cond,
                                  FAR sys_mutex_t *mutex,
                                  int32_t timeout_ms)
{
  int32_t         ret;
  int32_t         l_errno;
  struct timespec abs_time;
  struct timespec curr_time;

  if (timeout_ms == SYS_TIMEO_FEVR)
    {
      ret = pthread_cond_wait(cond, mutex);
    }
  else
    {
      /* Get current time. */

      ret = clock_gettime(CLOCK_REALTIME, &curr_time);
      if (ret != OK)
        {
          l_errno = errno;
          DBGIF_LOG2_ERROR("Failed to get time:%d errno:%d\n", ret, errno);
          return -l_errno;
        }

      abs_time.tv_sec  = timeout_ms / 1000;
      abs_time.tv_nsec =
        (timeout_ms - (abs_time.tv_sec * 1000)) * 1000 * 1000;

      abs_time.tv_sec  += curr_time.tv_sec;
      abs_time.tv_nsec += curr_time.tv_nsec;

      /* Check more than 1 sec. */

      if (abs_time.tv_nsec >= (1000 * 1000 * 1000))
        {
          abs_time.tv_sec  += 1;
          abs_time.tv_nsec -= (1000 * 1000 * 1000);
        }

      ret = pthread_cond_timedwait(cond, mutex, &abs_time);
    }

  if (ret != 0)
    {
      DBGIF_LOG1_ERROR("Failed to wait thread condition:%d.\n", ret);
      return -ret;
    }

  return 0;
}

/****************************************************************************
 * Name: sys_thread_cond_signal
 *
 * Description:
 *   The pthread_cond_signal() function shall unblock at least one of the
 *   threads that are blocked on the specified condition variable cond.
 *
 * Input Parameters:
 *   cond        Condition variable.
 *
 * Returned Value:
 *   If successful, shall return zero.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_thread_cond_signal(FAR sys_thread_cond_t *cond)
{
  int32_t ret;

  ret = pthread_cond_signal(cond);
  if (ret != 0)
    {
      DBGIF_LOG1_ERROR("Failed to signal thread condition:%d\n", ret);
      return -ret;
    }

  return 0;
}
