/****************************************************************************
 * examples/gnss_addon/gnss_addon_pps.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If the time difference is greater than this threshold, adjust the time. */

#define THRESHOLD_TIME_DIFF_USEC 150

/* The pin number to receive 1PPS signal.
 * This pin need to be physically connected.
 */

#define PPS_PIN   PIN_I2S0_DATA_OUT

/* Toggle LED for debug */

#define LED       GPIO_LED3

#ifdef DEBUG_TIME
#  define DPRINTF(...) printf(__VA_ARGS__)
#else
#  define DPRINTF(...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_pps_sem;
static struct timespec g_adjts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void toggle_led(void)
{
  static int toggle = 0;
  board_gpio_write(LED, toggle = !toggle);
}

static int pps_handler(int irq, void *context, void *arg)
{
  sem_post(&g_pps_sem);
  return 0;
}

static int64_t diff_usec(struct timespec *t1, struct timespec *t2)
{
  int64_t usec;

  if (t1->tv_sec == t2->tv_sec)
    {
      if (t1->tv_nsec >= t2->tv_nsec)
        {
          usec = (t1->tv_nsec - t2->tv_nsec) / NSEC_PER_USEC;
        }
      else
        {
          usec = (t2->tv_nsec - t1->tv_nsec) / NSEC_PER_USEC;
        }
    }
  else if (t1->tv_sec >= t2->tv_sec)
    {
      usec = (t1->tv_sec - t2->tv_sec) * USEC_PER_SEC;
      usec += (t1->tv_nsec - t2->tv_nsec) / NSEC_PER_USEC;
    }
  else
    {
      usec = (t2->tv_sec - t1->tv_sec) * USEC_PER_SEC;
      usec += (t2->tv_nsec - t1->tv_nsec) / NSEC_PER_USEC;
    }

  return usec;
}

static pthread_addr_t pps_daemon(pthread_addr_t arg)
{
  int ret;
  struct timespec now;

  sem_init(&g_pps_sem, 0, 0);

  board_gpio_config(PPS_PIN, 0, true, false, PIN_PULLDOWN);
  board_gpio_intconfig(PPS_PIN, INT_RISING_EDGE, false, pps_handler);

  while (1)
    {
      /* Wait for an interrupt of PPS signal. */

      ret = sem_wait(&g_pps_sem);
      if (ret < 0)
        {
          break;
        }

      /* Compare the RTC current time with the PPS target time. If the
       * deviation of current time is greater than the threshold value,
       * adjust time by setting PPS time to RTC.
       */

      clock_gettime(CLOCK_REALTIME, &now);

      if (diff_usec(&now, &g_adjts) > THRESHOLD_TIME_DIFF_USEC)
        {
          clock_settime(CLOCK_REALTIME, &g_adjts);
        }

      toggle_led();

      DPRINTF(" %lld\n",
              (int64_t)(now.tv_sec - g_adjts.tv_sec) * USEC_PER_SEC
              + (now.tv_nsec - g_adjts.tv_nsec) / NSEC_PER_USEC);
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int setup_pps(void)
{
  int ret;
  pthread_attr_t attr;
  struct sched_param param;
  pthread_t pps;

  /* Start a thread for setting PPS time to RTC. The thread priority should
   * be slightly higher than the normal application.
   */

  pthread_attr_init(&attr);
  pthread_attr_getschedparam(&attr, &param);
  param.sched_priority++;
  pthread_attr_setschedparam(&attr, &param);
  ret = pthread_create(&pps, &attr, pps_daemon, NULL);
  if (ret == OK)
    {
      pthread_setname_np(pps, "pps_daemon");
    }

  return ret;
}

int start_ppsint(void)
{
  /* Enable the interrupt from PPS signal. */

  board_gpio_int(PPS_PIN, true);
  return 0;
}

int stop_ppsint(void)
{
  /* Disable the interrupt from PPS signal. */

  board_gpio_int(PPS_PIN, false);
  return 0;
}

int set_ppstime(struct timespec ts)
{
  /* Set the future PPS time. */

  g_adjts = ts;
  return 0;
}
