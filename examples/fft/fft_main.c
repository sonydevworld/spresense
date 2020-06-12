/****************************************************************************
 * examples/fft/fft_main.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <math.h>

#include "dsp_rpc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check configuration.  This is not all of the configuration settings that
 * are required -- only the more obvious.
 */

#if CONFIG_NFILE_DESCRIPTORS < 1
#  error "You must provide file descriptors via CONFIG_NFILE_DESCRIPTORS in your configuration file"
#endif

#ifdef CONFIG_EXAMPLES_FFT_DSP_PATH
#  define DSP_LIBFILE CONFIG_EXAMPLES_FFT_DSP_PATH
#else
#  define DSP_LIBFILE "/mnt/sd0/BIN/DSPFFT"
#endif

#define TEST_NUM            16
#define TEST_FREQ_OFFSET    100
#define TEST_AMPLITUDE      1
#define TEST_FS             48000
//#define SAMPLES             256
//#define SAMPLES             512
#define SAMPLES             1024
//#define SAMPLES             2048

#ifdef CONFIG_EXAMPLES_FFT_COMPLEX_DATA
# define SAMPLE_NUM 2
#else
# define SAMPLE_NUM 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float *g_testInput[TEST_NUM];
static float g_peakBuffer[TEST_NUM];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern uint64_t cxd56_rtc_count(void);

static void fft_init_sample_data(int sample_num)
{
  int i;
  float frequency;

  /* Generate test input data (real, complex) */

  for (i = 0; i < TEST_NUM; i++)
    {
      frequency = TEST_FREQ_OFFSET + i * 1000.0f;
      g_testInput[i] = (float*)malloc(sizeof(float) * SAMPLES * sample_num);
      for (uint16_t n = 0; n < SAMPLES; n++)
        {
          g_testInput[i][sample_num * n]
            = TEST_AMPLITUDE * sinf(2.0 * M_PI * frequency * n / TEST_FS);
          if (sample_num == 2)
            {
              g_testInput[i][sample_num * n + 1] = 0.0f;
            }
        }
    }
}

static void fft_fini_sample_data(void)
{
  int i;

  for (i = 0; i < TEST_NUM; i++)
    {
      free(g_testInput[i]);
    }
}

static int fft_test(int num)
{
  /* Prepare for input data  */

  fft_init_sample_data(SAMPLE_NUM);

  /* Set FFT hamming window */

  fft_window(FFT_WIN_HAMMING);

  /* Set the storage buffer for FFT peak freqency */

  memset(g_peakBuffer, 0, sizeof(g_peakBuffer));
  fft_peak(g_peakBuffer, 1, TEST_FS);

  uint64_t now1 = cxd56_rtc_count();

  /* FFT calculation */

  fft(g_testInput[num], NULL, SAMPLES);

  uint64_t now2 = cxd56_rtc_count();

  printf("FFT times: %d\n", 1);
  printf("FFT samples: %d\n", SAMPLES);
  printf("FFT total time: %8.5f [ms]\n", (now2 - now1) / 32.768f);
  printf("FFT frequency: %5.3f (%5.3f) [Hz]\n",
         g_peakBuffer[0], TEST_FREQ_OFFSET + num * 1000.0f);

  fft_fini_sample_data();
  return OK;
}

static int fft_request_test(void)
{
  int i;
  float frequency;
  fft_desc_t desc[TEST_NUM];
  uint32_t addr;
  int ret;

  /* Prepare for input data  */

  fft_init_sample_data(SAMPLE_NUM);

  /* Set FFT hamming window */

  fft_window(FFT_WIN_HAMMING);

  /* Set the storage buffer for FFT peak freqency */

  memset(g_peakBuffer, 0, sizeof(g_peakBuffer));
  fft_peak(g_peakBuffer, TEST_NUM, TEST_FS);

  /* Set descriptors */

  for (i = 0; i < TEST_NUM; i++)
    {
      desc[i].pSrc = g_testInput[i];
      desc[i].pDst = NULL;
      desc[i].fftLen = SAMPLES;

      if (i == (TEST_NUM - 1))
        {
          /* Last descriptor */

          desc[i].pNext = NULL;
          desc[i].attribute = FFT_ATTR_NOTIFY;
        }
      else
        {
          desc[i].pNext = &desc[i + 1];
          desc[i].attribute = 0;
        }
    }

  uint64_t now1 = cxd56_rtc_count();

  /* FFT calculation */

  fft_request(desc);

  /* Wait until FFT is completed */

  ret = fft_wait_response(0, &addr);

  uint64_t now2 = cxd56_rtc_count();

  printf("FFT times: %d\n", TEST_NUM);
  printf("FFT samples: %d\n", SAMPLES);
  printf("FFT total time: %8.5f [ms]\n", (now2 - now1) / 32.768f);
  printf("FFT execution result: ret=%d addr=0x%08x\n", ret, addr);

  /* Display the results of FFT calculation */

  printf("Num  Expected  PeakFrequency [Hz]\n");
  for (i = 0; i < TEST_NUM; i++)
    {
      frequency = TEST_FREQ_OFFSET + i * 1000.0f;
      printf("%2d: %9.3f %9.3f\n", i, frequency, g_peakBuffer[i]);
    }

  fft_fini_sample_data();
  return OK;
}

static int fft_monitor(int argc, FAR char *argv[])
{
  int ret = OK;
  uint32_t addr;
  int i;

  for (; ; )
    {
      ret = fft_stream_output(0, &addr);
      if (ret < 0)
        {
          printf("ERROR: ret=%d\n", ret);
          break;
        }

      printf("FFT execution result: ret=%d addr=0x%08x\n", ret, addr);
      for (i = 0; i < TEST_NUM; i++)
        {
          printf("%2d: %9.3f\n", i, g_peakBuffer[i]);
        }
    }

  return ret;
}

static int fft_stream_test(void)
{
  int pid;
  int i;

  /* Prepare for input data (only real data) */

  fft_init_sample_data(1);

  /* Set FFT hamming window */

  fft_window(FFT_WIN_HAMMING);

  /* Set the storage buffer for FFT peak freqency */

  memset(g_peakBuffer, 0, sizeof(g_peakBuffer));
  fft_peak(g_peakBuffer, TEST_NUM, TEST_FS);

  /* Stream init */

  fft_stream_init(SAMPLES, SAMPLES / 2, TYPE_FLOAT);

  pid = task_create("fft_monitor", 200, 1024, fft_monitor, NULL);
  if (pid < 0)
    {
      return ERROR;
    }

  for (i = 0; i < TEST_NUM; i++)
    {
      fft_stream_input(&g_testInput[i][SAMPLES * 0 / 4], SAMPLES);
      fft_stream_input(&g_testInput[i][SAMPLES * 1 / 4], SAMPLES);
      fft_stream_input(&g_testInput[i][SAMPLES * 2 / 4], SAMPLES);
      fft_stream_input(&g_testInput[i][SAMPLES * 3 / 4], SAMPLES);
      usleep(1000 * 1000);
    }

  if (pid > 0)
    {
      task_delete(pid);
    }

  fft_fini_sample_data();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fft_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  ret = load_library(DSP_LIBFILE);
  if (ret < 0)
    {
      printf("ERROR: Not a loadable DSP\n");
      return ERROR;
    }

  printf("****** fft_test:\n");
  ret = fft_test(5);
  printf("---> %s\n", ret == OK ? "success" : "failed");

  printf("****** fft_request_test:\n");
  ret = fft_request_test();
  printf("---> %s\n", ret == OK ? "success" : "failed");

  printf("****** fft_stream_test:\n");
  ret = fft_stream_test();
  printf("---> %s\n", ret == OK ? "success" : "failed");

  unload_library();

  return 0;
}
