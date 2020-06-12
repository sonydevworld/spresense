/****************************************************************************
 * modules/lte/include/osal/osal.h
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

#ifndef __MODULES_LTE_INCLUDE_OSAL_OSAL_H
#define __MODULES_LTE_INCLUDE_OSAL_OSAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "cc.h"
#include "osal_opt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef SYS_MALLOC
#  define SYS_MALLOC(sz) malloc(sz)
#endif

#ifndef SYS_FREE
#  define SYS_FREE(ptr)  free(ptr)
#endif

#define SYS_TIMEO_FEVR     (-1)      /* timeout forever */
#define SYS_WMODE_TWF_ANDW (0x0000)  /* wait mode : AND */
#define SYS_WMODE_TWF_ORW  (0x0001)  /* wait mode : OR */

#define SYS_TASK_PRIO_LOW    (0)
#define SYS_TASK_PRIO_NORMAL (1)
#define SYS_TASK_PRIO_HIGH   (2)

#define SYS_OWN_TASK (NULL)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  FAR void         *arg;
  CODE void (*function)(FAR void *arg);
  FAR const int8_t *name;
  int32_t          priority;
  uint32_t         stack_size;
} sys_cretask_s;

typedef struct
{
  uint32_t initial_count;
  uint32_t max_count;
} sys_cresem_s;

typedef struct
{
  int8_t dummy;
} sys_cremtx_s;

typedef struct
{
  int32_t      numof_queue;
  int32_t      queue_size;
} sys_cremq_s;

typedef struct
{
  int8_t dummy;
} sys_creevflag_s;

typedef void (*sys_timer_cb_t)(sys_timer_t timer);

#ifndef bool
  typedef uint32_t bool;
#  define true     (1)
#  define false    (0)
#endif

/****************************************************************************
 * Public Function Prototypes
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
                        FAR const sys_cretask_s *params);

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

int32_t sys_delete_task(FAR sys_task_t *task);

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

int32_t sys_sleep_task(int32_t timeout_ms);

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

int32_t sys_enable_dispatch(void);

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

int32_t sys_disable_dispatch(void);

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
                             FAR const sys_cresem_s *params);

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

int32_t sys_delete_semaphore(FAR sys_sem_t *sem);

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
 *              If timeout_ms set to SYS_TIMEO_FEVR then wait until
 *              the semaphore to become available.
 *
 * Returned Value:
 *   If the semaphore was become available then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_wait_semaphore(FAR sys_sem_t *sem, int32_t timeout_ms);

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

int32_t sys_post_semaphore(FAR sys_sem_t *sem);

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
                         FAR const sys_cremtx_s *params);

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

int32_t sys_delete_mutex(FAR sys_mutex_t *mutex);

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

int32_t sys_lock_mutex(FAR sys_mutex_t *mutex);

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

int32_t sys_unlock_mutex(FAR sys_mutex_t *mutex);

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

int32_t sys_create_mqueue(FAR sys_mq_t *mq, FAR const sys_cremq_s *params);

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

int32_t sys_delete_mqueue(FAR sys_mq_t *mq);

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
 *              If timeout_ms set to SYS_TIMEO_FEVR then wait until
 *              the message queue to become available.
 *
 * Returned Value:
 *   If the message queue was sent successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_send_mqueue(FAR sys_mq_t *mq, FAR int8_t *message, size_t len,
                        int32_t timeout_ms);

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
 *              If timeout_ms set to SYS_TIMEO_FEVR then wait until
 *              the message queue to become not empty.
 *
 * Returned Value:
 *   If the message queue was received successfully then the length of the
 *   selected message in bytes is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_recv_mqueue(FAR sys_mq_t *mq, FAR int8_t *message, size_t len,
                        int32_t timeout_ms);

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
                          FAR const sys_creevflag_s *params);

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

int32_t sys_delete_evflag(FAR sys_evflag_t *flag);

/****************************************************************************
 * Name: sys_wait_evflag
 *
 * Description:
 *   Waiting a event flag to be set.
 *
 * Input Parameters:
 *   flag       The handle of the event flag to be waited.
 *   wptn       The wait bit pattern.
 *   wmode      The wait mode. If wmode set to SYS_WMODE_TWF_ANDW then
 *              the release condition requires all the bits in wptn
 *              to be set.
 *              If wmode set to SYS_WMODE_TWF_ORW then the release
 *              condition only requires at least one bit in wptn to be set.
 *   flagptn    The current bit pattern.
 *   autoclr    Whether or not to clear bits automatically
 *              when function returns.
 *   timeout_ms The time in milliseconds to wait for the event flag
 *              to become set.
 *              If timeout_ms set to SYS_TIMEO_FEVR then wait until
 *              the event flag to become set.
 *
 * Returned Value:
 *   If the event flag was waited successfully then 0 is returned.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t sys_wait_evflag(FAR sys_evflag_t *flag, sys_evflag_ptn_t wptn,
                        sys_evflag_mode_t wmode, bool autoclr,
                        FAR sys_evflag_ptn_t *flagptn, int32_t timeout_ms);

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

int32_t sys_set_evflag(FAR sys_evflag_t *flag, sys_evflag_ptn_t setptn);

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

int32_t sys_set_evflag_isr(FAR sys_evflag_t *flag, sys_evflag_ptn_t setptn);

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

int32_t sys_clear_evflag(FAR sys_evflag_t *flag, sys_evflag_ptn_t clrptn);

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
                        CODE sys_timer_cb_t callback);

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

int32_t sys_stop_timer(FAR sys_timer_t *timer);

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
                             FAR sys_thread_condattr_t *cond_attr);

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

int32_t sys_thread_cond_destroy(FAR sys_thread_cond_t *cond);

/****************************************************************************
 * Name: sys_thread_cond_wait
 *
 * Description:
 *   The sys_thread_cond_wait() functions shall block on a condition
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
                             FAR sys_mutex_t *mutex);

/****************************************************************************
 * Name: sys_thread_cond_timedwait
 *
 * Description:
 *   The sys_thread_cond_timedwait() functions shall block on a condition
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
                                  int32_t timeout_ms);

/****************************************************************************
 * Name: sys_thread_cond_signal
 *
 * Description:
 *   The sys_thread_cond_signal() function shall unblock at least one of the
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

int32_t sys_thread_cond_signal(FAR sys_thread_cond_t *cond);


/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sys_create_thread_cond_mutex
 *
 * Description:
 *   Create thread condition and mutex resorces. This function is utility
 *   wrapper for using sys_thread_cond_init.
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

static inline int32_t sys_create_thread_cond_mutex(
  FAR sys_thread_cond_t *cond, FAR sys_mutex_t *mutex)
{
  int32_t      ret;
  sys_cremtx_s mtx_param = {0};

  ret = sys_create_mutex(mutex, &mtx_param);
  if (ret == 0)
    {
      ret = sys_thread_cond_init(cond, NULL);
      if (ret != 0)
        {
          sys_delete_mutex(mutex);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sys_delete_thread_cond_mutex
 *
 * Description:
 *   Delete thread condition and mutex resorces. This function is utility
 *   wrapper for using sys_thread_cond_destroy.
 *
 * Input Parameters:
 *   cond        Condition variable.
 *   mutex       The handle of the mutex.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void sys_delete_thread_cond_mutex(FAR sys_thread_cond_t *cond,
                                                FAR sys_mutex_t *mutex)
{
  sys_delete_mutex(mutex);
  sys_thread_cond_destroy(cond);
}

/****************************************************************************
 * Name: sys_wait_thread_cond
 *
 * Description:
 *   The sys_thread_cond_timedwait() functions shall block on a condition
 *   variable. This function is utility wrapper for
 *   using sys_thread_cond_wait.
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

static inline int32_t sys_wait_thread_cond(FAR sys_thread_cond_t *cond,
                                           FAR sys_mutex_t *mutex,
                                           int32_t timeout_ms)
{
  int32_t ret;

  sys_lock_mutex(mutex);

  ret = sys_thread_cond_timedwait(cond, mutex, timeout_ms);

  sys_unlock_mutex(mutex);

  return ret;
}

/****************************************************************************
 * Name: sys_signal_thread_cond
 *
 * Description:
 *   The sys_signal_thread_cond() function shall unblock at least one of the
 *   threads that are blocked on the specified condition variable cond.
 *   This function is utility wrapper for using sys_thread_cond_signal.
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

static inline int32_t sys_signal_thread_cond(FAR sys_thread_cond_t *cond,
                                             FAR sys_mutex_t *mutex)
{
  int32_t ret;

  sys_lock_mutex(mutex);

  ret = sys_thread_cond_signal(cond);

  sys_unlock_mutex(mutex);

  return ret;
}

#endif /* __MODULES_LTE_INCLUDE_OSAL_OSAL_H */
