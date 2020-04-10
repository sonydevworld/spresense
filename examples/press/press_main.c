/****************************************************************************
 * press/press_main.c
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>

#include <nuttx/sensors/bmp280.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_PRESS_DEVNAME
#  define PRESS_DEVNAME CONFIG_EXAMPLES_PRESS_DEVNAME
#else
#  define PRESS_DEVNAME "/dev/press0"
#endif
#ifdef CONFIG_EXAMPLES_TEMP_DEVNAME
#  define TEMP_DEVNAME CONFIG_EXAMPLES_TEMP_DEVNAME
#else
#  define TEMP_DEVNAME "/dev/temp0"
#endif

#ifdef CONFIG_EXAMPLES_PRESS_SIGNO
#  define PRESS_SIG CONFIG_EXAMPLES_PRESS_SIGNO
#else
#  define PRESS_SIG 14
#endif
#ifdef CONFIG_EXAMPLES_TEMP_SIGNO
#  define TEMP_SIG  CONFIG_EXAMPLES_TEMP_SIGNO
#else
#  define TEMP_SIG  15
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t g_t_fine;
static struct bmp280_press_adj_s  g_press_adj;
static struct bmp280_temp_adj_s   g_temp_adj;
static struct scutimestamp_s g_ts = {0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp280_compensate_T_int32
 *
 * Description:
 *   calculate compensate tempreture
 *
 * Input Parameters:
 *   adc_T - uncompensate value of tempreture.
 *
 * Returned Value:
 *   calculate result of compensate tempreture.
 *
 ****************************************************************************/

static int32_t bmp280_compensate_T_int32(int32_t adc_T, struct bmp280_temp_adj_s *adj)
{
  int32_t var1;
  int32_t var2;
  int32_t T;

  var1 = ((((adc_T >> 3) - ((int32_t)adj->dig_T1 << 1))) * ((int32_t)adj->dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)adj->dig_T1)) * ((adc_T >> 4) - ((int32_t)adj->dig_T1))) >> 12) *
  ((int32_t)adj->dig_T3)) >> 14;

  g_t_fine = var1 + var2;

  T = (g_t_fine * 5 + 128) >> 8;

  return T;
}

/****************************************************************************
 * Name: bmp280_compensate_P_int32
 *
 * Description:
 *   calculate compensate pressure
 *
 * Input Parameters:
 *   adc_P - uncompensate value of pressure.
 *
 * Returned Value:
 *   calculate result of compensate pressure.
 *
 ****************************************************************************/

static uint32_t bmp280_compensate_P_int32(int32_t adc_P, struct bmp280_press_adj_s *adj)
{
  int32_t var1;
  int32_t var2;
  uint32_t p;

  var1 = (((int32_t)g_t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)adj->dig_P6);
  var2 = var2 + ((var1 * ((int32_t)adj->dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)adj->dig_P4) << 16);
  var1 = (((adj->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t)adj->dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)adj->dig_P1)) >> 15);

  /* avoid exception caused by division by zero */

  if (var1 == 0)
    {
      return 0;
    }

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

  if (p < 0x80000000)
    {
      p = (p << 1) / ((uint32_t)var1);
    }
  else
    {
      p = (p / (uint32_t)var1) * 2;
    }

  var1 = (((int32_t)adj->dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)adj->dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + adj->dig_P7) >> 4));

  return p;
}

static inline int sample2int(struct bmp280_meas_s *m)
{
   return (((int)m->msb << 12) |
           ((int)m->lsb << 4) |
           ((int)m->xlsb >> 4));
}

static void dump_data(void *pressdata, void *tempdata, int nrsamples)
{
  struct bmp280_meas_s *p = pressdata;
  struct bmp280_meas_s *t = tempdata;
  int up;
  int ut;
  int compensated_p;
  int compensated_t;
  int i;

  for (i = 0; i < nrsamples; i++, p++, t++)
    {
      up = sample2int(p);
      ut = sample2int(t);

      /* Compensate pressure values with temperature */

      compensated_t = bmp280_compensate_T_int32(ut, &g_temp_adj);
      compensated_p = bmp280_compensate_P_int32(up, &g_press_adj);

      printf("[%3d] p=%6d (uncomp:%6d) t=%d (uncomp:%d) \n",
             i, compensated_p, up, compensated_t, ut);
    }
}

static void show_scutime(struct scutimestamp_s *ts)
{
  static struct scutimestamp_s prev = { 0, 0 };
  struct scutimestamp_s delta;

  delta.sec = ts->sec - prev.sec;
  if (ts->tick < prev.tick)
    {
      /* Borrow from sec */

      delta.tick = 32768 - (prev.tick - ts->tick);
      delta.sec--;
    }
  else
    {
      delta.tick = ts->tick - prev.tick;
    }

  if (prev.sec == 0 && prev.tick == 0)
    {
      delta.sec = 0;
      delta.tick = 0;
    }

  printf("SCU Timestamp: %lu.%05u (%lu.%05u)\n",
         ts->sec, (uint32_t)ts->tick * 100000 / 32768,
         delta.sec, (uint32_t)delta.tick * 100000 / 32768);

  prev.sec = ts->sec;
  prev.tick = ts->tick;
}

static int setup_scu(int fd, int signo, uint32_t size, struct scufifo_wm_s *wm)
{
  int ret;

  /* Set FIFO size to 3 bytes * 128 Hz = 384 */

  ret = ioctl(fd, SCUIOC_SETFIFO, (unsigned long)size);
  if (ret < 0)
    {
      printf("SETFIFO failed. %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 128 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 2) = 128
   */

  ret = ioctl(fd, SCUIOC_SETSAMPLE, 2);
  if (ret < 0)
    {
      printf("SETSAMPLE failed. %d\n", ret);
      return ret;
    }

  ret = ioctl(fd, SCUIOC_SETWATERMARK, (unsigned long)(uintptr_t)wm);
  if (ret < 0)
    {
      printf("SETWATERMARK failed. %d\n", ret);
      return ret;
    }

  return OK;
}

static void sensing_main(int pressfd, int tempfd)
{
  sigset_t  set;
  siginfo_t info;
  struct scufifo_wm_s wm;
  struct scutimestamp_s ts, *tsptr;
  void     *pressbuf = NULL, *tempbuf = NULL;
  size_t    size;
  int       ready;
  int       ret;

  size = sizeof(struct bmp280_meas_s) * 128;

  pressbuf = malloc(size);
  if (!pressbuf)
    {
      fprintf(stderr, "Memory allocation failure.\n");
      return;
    }
  memset(pressbuf, 0, size);

  tempbuf = malloc(1024);
  if (!tempbuf)
    {
      fprintf(stderr, "Memory allocation failure.\n");
      free(pressbuf);
      return;
    }
  memset(tempbuf, 0, size);

  /* Setup pressure watermark.
   * SCU timestamp will be updated by driver, so it must be global or heap.
   */

  wm.signo = PRESS_SIG;
  wm.ts = &g_ts;
  wm.watermark = 128;
  ret = setup_scu(pressfd, PRESS_SIG, size * 2, &wm);
  if (ret < 0)
    {
      fprintf(stderr, "start_sampling failed. %d\n", ret);
      goto finish;
    }

  wm.signo = TEMP_SIG;
  wm.ts = NULL;
  wm.watermark = 128;
  ret = setup_scu(tempfd, TEMP_SIG, size * 2, &wm);
  if (ret < 0)
    {
      fprintf(stderr, "start_sampling failed. %d\n", ret);
      goto finish;
    }

  sigemptyset(&set);
  sigaddset(&set, PRESS_SIG);
  sigaddset(&set, TEMP_SIG);

  sigprocmask(SIG_BLOCK, &set, NULL);

  /* Start sequencer */

  ioctl(pressfd, SCUIOC_START, 0);
  ioctl(tempfd, SCUIOC_START, 0);

  ready = 0;
  for(;;)
    {
      ret = sigwaitinfo(&set, &info);
      if (ret < 0)
        {
          int errcode = errno;
          if (errcode == EINTR)
            {
              continue;
            }
          fprintf(stderr, "sigwaitinfo failed. %d\n", errcode);
          break;
        }

      if (ret == TEMP_SIG)
        {
          ret = read(tempfd, tempbuf, size);
          if (ret < 0)
            {
              fprintf(stderr, "read temperature data failed. %d\n", errno);
              break;
            }
          ready = 1;
        }
      else if (ret == PRESS_SIG)
        {
          /* Save timestamp before update by driver */

          tsptr = (struct scutimestamp_s *)info.si_value.sival_ptr;
          ts.sec = tsptr->sec;
          ts.tick = tsptr->tick;

          ret = read(pressfd, pressbuf, size);
          if (ret < 0)
            {
              fprintf(stderr, "read press data failed. %d\n", errno);
              break;
            }

          if (ready)
            {
              /* Dump compensate pressure data with temperatures */

              dump_data(pressbuf, tempbuf, 128);
              show_scutime(&ts);
            }
        }
      else
        {
          fprintf(stderr, "Unknown signal. %d\n", ret);
        }
    }

  ioctl(pressfd, SCUIOC_STOP, 0);
  ioctl(tempfd, SCUIOC_STOP, 0);

 finish:
  free(pressbuf);
  free(tempbuf);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * sensor_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int pressfd, tempfd;
  int ret;

  printf("Sensing start...\n");

  pressfd = open(PRESS_DEVNAME, O_RDONLY);
  if (pressfd < 0)
    {
      printf("Device %s open failure. %d\n", PRESS_DEVNAME, errno);
      return -1;
    }

  tempfd = open(TEMP_DEVNAME, O_RDONLY);
  if (tempfd < 0)
    {
      printf("Device %s open failure. %d\n", TEMP_DEVNAME, errno);
      return -1;
    }

  /* Preparing compensate the sensing data */

  ret = ioctl(pressfd, SNIOC_GETADJ, (unsigned long)(uintptr_t)&g_press_adj);
  if (ret < 0)
    {
      printf("SNIOC_GETADJ failed. %d\n", ret);
      return ret;
    }

  ret = ioctl(tempfd, SNIOC_GETADJ, (unsigned long)(uintptr_t)&g_temp_adj);
  if (ret < 0)
    {
      printf("SNIOC_GETADJ failed. %d\n", ret);
      return ret;
    }

  printf("[Calibration parameters]\n");
  printf("dig_T1 = %6d\n", g_temp_adj.dig_T1);
  printf("dig_T2 = %6d\n", g_temp_adj.dig_T2);
  printf("dig_T3 = %6d\n", g_temp_adj.dig_T3);
  printf("dig_P1 = %6d\n", g_press_adj.dig_P1);
  printf("dig_P2 = %6d\n", g_press_adj.dig_P2);
  printf("dig_P3 = %6d\n", g_press_adj.dig_P3);
  printf("dig_P4 = %6d\n", g_press_adj.dig_P4);
  printf("dig_P5 = %6d\n", g_press_adj.dig_P5);
  printf("dig_P6 = %6d\n", g_press_adj.dig_P6);
  printf("dig_P7 = %6d\n", g_press_adj.dig_P7);
  printf("dig_P8 = %6d\n", g_press_adj.dig_P8);
  printf("dig_P9 = %6d\n", g_press_adj.dig_P9);

  sensing_main(pressfd, tempfd);

  close(pressfd);
  close(tempfd);

  printf("done.\n");

  return 0;
}
