/****************************************************************************
 * examples/fft/dsp_rpc.c
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

#include <stdio.h>
#include <string.h>

#include <assert.h>

#include <asmp/mptask.h>
#include <asmp/mpmq.h>

#include "dsp_rpc.h"
#include "resource.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define errmsg(x, ...) fprintf(stderr, x, ## __VA_ARGS__)

#define ARGVAL(v) (uint32_t)(v)
#define ARGPTR(p) ((uint32_t)(uintptr_t)(p))

#define ASYNC_BUF_NUM 8

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mptask_t g_dsptask = {0};
static mpmq_t   g_dspmq = {0};
static mpmq_t   g_dspmq2 = {0};
static uint32_t g_buffer[8];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t *get_async_buffer(void)
{
  static uint32_t s_buffer[ASYNC_BUF_NUM][8];
  static int s_index = 0;

  uint32_t *buffer = s_buffer[s_index];

  s_index++;
  s_index %= ASYNC_BUF_NUM;

  return buffer;
}

static int dsp_send(void *args)
{
  int ret;
  uint32_t data;

  data = (uint32_t)(uintptr_t)args;

  /* Send ASYNC message to DSP */

  ret = mpmq_send(&g_dspmq, DSP_RPC_MSG_ASYNC, data);
  if (ret < 0)
    {
      errmsg("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

static int dsp_rpc(void *args)
{
  int ret;
  uint32_t data;

  data = (uint32_t)(uintptr_t)args;

  /* Send RPC message to DSP */

  ret = mpmq_send(&g_dspmq, DSP_RPC_MSG, data);
  if (ret < 0)
    {
      errmsg("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  /* Wait for DSP function is done */

  ret = mpmq_receive(&g_dspmq, &data);
  if (ret < 0)
    {
      errmsg("mpmq_recieve() failure. %d\n", ret);
      return ret;
    }

  return (int)data;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void fft(void *pSrc, void *pDst, uint32_t fftLen)
{
  uint32_t *args = g_buffer;

  assert((fftLen == 16) || (fftLen == 32) || (fftLen == 64) ||
         (fftLen == 128) || (fftLen == 256) || (fftLen == 512) ||
         (fftLen == 1024) || (fftLen == 2048));

  args[0] = DSP_FFT;
  args[1] = ARGPTR(pSrc);
  args[2] = ARGPTR(pDst);
  args[3] = ARGPTR(fftLen);

  (void)dsp_rpc(args);
}

void fft_request(fft_desc_t *pDesc)
{
  uint32_t *args = get_async_buffer();

  args[0] = DSP_FFT_REQUEST;
  args[1] = ARGPTR(pDesc);

  (void)dsp_send(args);
}

int fft_wait_response(uint32_t ms, void *paddr)
{
  int ret;
  uint32_t data;

  if (ms)
    {
      ret = mpmq_timedreceive(&g_dspmq, &data, ms);
    }
  else
    {
      ret = mpmq_receive(&g_dspmq, &data);
    }

  if (ret == DSP_RPC_RET)
    {
      if (paddr)
        {
          *(uint32_t *)paddr = data;
        }
    }

  return ret;
}

void fft_stream_init(uint32_t fftLen, uint32_t fftShift, fft_stream_type_t type)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_FFT_STREAM_INIT;
  args[1] = ARGPTR(fftLen);
  args[2] = ARGPTR(fftShift);
  args[3] = ARGPTR(type);

  (void)dsp_rpc(args);
}

void fft_stream_input(void *pBuf, size_t size)
{
  uint32_t *args = get_async_buffer();

  args[0] = DSP_FFT_STREAM_INPUT;
  args[1] = ARGPTR(pBuf);
  args[2] = ARGPTR(size);

  (void)dsp_send(args);
}

int fft_stream_output(uint32_t ms, void *paddr)
{
  int ret;
  uint32_t data;

  if (ms)
    {
      ret = mpmq_timedreceive(&g_dspmq2, &data, ms);
    }
  else
    {
      ret = mpmq_receive(&g_dspmq2, &data);
    }

  if (ret == DSP_RPC_RET)
    {
      if (paddr)
        {
          *(uint32_t *)paddr = data;
        }
    }

  return ret;
}

void fft_window(int type)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_FFT_WINDOW;
  args[1] = ARGPTR(type);

  (void)dsp_rpc(args);
}

void fft_peak(float *pPeakBuf, int bufLen, int fs)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_FFT_PEAK;
  args[1] = ARGPTR(pPeakBuf);
  args[2] = ARGPTR(bufLen);
  args[3] = ARGPTR(fs);

  (void)dsp_rpc(args);
}

int load_library(const char *filename)
{
  int ret;

  memset(g_buffer, 0, sizeof(g_buffer));

  /* Initialize DSP Math library */

  ret = mptask_init(&g_dsptask, filename);
  if (ret != 0)
    {
      errmsg("mptask_init() failure. %d\n", ret);
      return ret;
    }

  ret = mptask_assign(&g_dsptask);
  if (ret != 0)
    {
      errmsg("mptask_asign() failure. %d\n", ret);
      return ret;
    }

  /* Initialize MP message queue with asigned CPU ID, and bind it to MP task */

  ret = mpmq_init(&g_dspmq, DSP_MQID, mptask_getcpuid(&g_dsptask));
  if (ret < 0)
    {
      errmsg("mpmq_init() failure. %d\n", ret);
      return ret;
    }
  ret = mptask_bindobj(&g_dsptask, &g_dspmq);
  if (ret < 0)
    {
      errmsg("mptask_bindobj(mq) failure. %d\n", ret);
      return ret;
    }

  ret = mpmq_init(&g_dspmq2, DSP_MQID2, mptask_getcpuid(&g_dsptask));
  if (ret < 0)
    {
      errmsg("mpmq_init() failure. %d\n", ret);
      return ret;
    }
  ret = mptask_bindobj(&g_dsptask, &g_dspmq2);
  if (ret < 0)
    {
      errmsg("mptask_bindobj(mq) failure. %d\n", ret);
      return ret;
    }

  ret = mptask_exec(&g_dsptask);
  if (ret < 0)
    {
      errmsg("mptask_exec() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

void unload_library(void)
{
  /* Send quit request to DSP */

  mpmq_send(&g_dspmq, DSP_RPC_UNLOAD, 0);

  /* Destroy DSP and successfully done. */

  (void)mptask_destroy(&g_dsptask, false, NULL);
  mpmq_destroy(&g_dspmq);
}

