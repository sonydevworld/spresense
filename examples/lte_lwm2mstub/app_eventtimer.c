/****************************************************************************
 * examples/lte_lwm2mstub/app_eventtimer.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>

#include "app_parameter.h"

extern void increment_value(void);  /* from lte_lwm2mstub_main.c */

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define MY_TIMER_SIGNAL 17
#define SIGVALUE_INT  42

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t sem;
static volatile bool is_running = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * name: timer_exriration
 ****************************************************************************/

static void timer_expiration(int signo, siginfo_t *info, void *ucontext)
{
  increment_value();
}

/****************************************************************************
 * name: timer_task
 ****************************************************************************/

static int timer_task(int argc, char *argv[])
{
  sigset_t           set;
  struct sigaction   act;
  struct sigaction   oact;
  struct sigevent    notify;
  struct itimerspec  timer;
  timer_t            timerid;
  struct app_parameter_s *param;

  param = get_appparam();

  sem_init(&sem, 0, 0);

  /* Start waiter thread  */

  sigemptyset(&set);
  sigaddset(&set, MY_TIMER_SIGNAL);
  sigprocmask(SIG_UNBLOCK, &set, NULL);

  act.sa_sigaction = timer_expiration;
  act.sa_flags  = SA_SIGINFO;

  sigfillset(&act.sa_mask);
  sigdelset(&act.sa_mask, MY_TIMER_SIGNAL);

  sigaction(MY_TIMER_SIGNAL, &act, &oact);

  /* Create the POSIX timer */

  notify.sigev_notify            = SIGEV_SIGNAL;
  notify.sigev_signo             = MY_TIMER_SIGNAL;
  notify.sigev_value.sival_int   = SIGVALUE_INT;

  timer_create(CLOCK_REALTIME, &notify, &timerid);

  /* Start the POSIX timer */

  timer.it_value.tv_sec     = param->time_period;
  timer.it_value.tv_nsec    = 0;
  timer.it_interval.tv_sec  = param->time_period;
  timer.it_interval.tv_nsec = 0;

  timer_settime(timerid, 0, &timer, NULL);

  /* Take the semaphore */

  while (is_running)
    {
      sem_wait(&sem);
    }

  sem_destroy(&sem);

  /* Then delete the timer */

  timer_delete(timerid);

  /* Detach the signal handler */

  act.sa_handler = SIG_DFL;
  sigaction(MY_TIMER_SIGNAL, &act, &oact);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: timer_setup
 ****************************************************************************/

int timer_setup(struct app_parameter_s *param)
{
  int ret = OK;

  if (!is_running && param->time_period > 0)
    {
      printf("Timer will start\n");
      is_running = true;
      ret = task_create("lwm2m_tmtask", 100, 2048, timer_task, NULL);
      ret = ret < 0 ? ERROR : OK;
    }

  return ret;
}

/****************************************************************************
 * name: timer_unset
 ****************************************************************************/

int timer_unset(void)
{
  is_running = false;
  return OK;
}
