/****************************************************************************
 * examples/fft/dsp/dsp_main.c
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

#include <errno.h>
#include <stdlib.h>

#include <asmp/types.h>
#include <asmp/mpmq.h>
#include <asmp/mpshm.h>

#include "asmp.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "resource.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ASSERT(cond) if (!(cond)) wk_abort()

/* Internal error must be other than returns of math functions. */

#define INTERNAL_ERROR -99

/* Utility macros for declare RPC entry function and construct
 * their table.
 */

#define RPCENTRY(sym) { DSP_ ## sym, rpcentry_ ## sym }
#define DECLARE_RPCENTRY(sym) static int rpcentry_ ## sym
#define RPCENTRY_TERMINATE { 0, NULL }

#ifdef CONFIG_EXAMPLES_FFT_COMPLEX_DATA
#  define SAMPLE_NUM 2
#else
#  define SAMPLE_NUM 1
#endif

#ifndef CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE
#  define CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE (1024 * 8)
#endif

#ifndef CONFIG_EXAMPLES_FFT_SAMPLES_INPUT_BUFFER_SIZE
#  define CONFIG_EXAMPLES_FFT_SAMPLES_INPUT_BUFFER_SIZE (1024 * 4)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpcentry_s
{
  uint32_t hash;
  int (*func)(uint32_t *args);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mpmq_t g_mq;
static mpmq_t g_mq2;

static float32_t g_workBuffer[CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE];

/* For DSP_FFT_STREAM_INPUT */

static float32_t g_inBuffer[CONFIG_EXAMPLES_FFT_SAMPLES_INPUT_BUFFER_SIZE];
static uint8_t *g_inTop = (uint8_t*)g_inBuffer;
static uint8_t *g_inBottom = (uint8_t*)(g_inBuffer
                            + CONFIG_EXAMPLES_FFT_SAMPLES_INPUT_BUFFER_SIZE);
static uint8_t *g_inWptr = (uint8_t*)g_inBuffer;
static uint8_t *g_inRptr = (uint8_t*)g_inBuffer;

/* For DSP_FFT_STREAM_INIT */

static uint32_t g_fftLen;
static uint32_t g_fftShift;
static fft_stream_type_t g_streamType;

/* For DSP_FFT_WINDOW */

static int g_preProcessWindow = FFT_WIN_RECTANGLE;

/* For DSP_FFT_PEAK */

static float32_t g_fs = 0.0f;
static float32_t *g_pPeakBuf;
static int g_peakBufLen = 0;
static int g_peakCount = 0;
static int g_postProcessPeak = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *allocate_buffer(uint32_t fftLen)
{
  static int index = 0;
  void *workPtr;

  if ((index + fftLen) > CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE)
    {
      index = 0;
    }

  workPtr = &g_workBuffer[index];
  arm_fill_f32(0.0f, workPtr, fftLen);

  index += fftLen;

  return workPtr;
}

static int post_peak(float *pData, uint32_t fftLen)
{
  uint32_t index;
  float32_t maxValue;
  float32_t delta;

  arm_max_f32(pData, fftLen / 2, &maxValue, &index);

  delta = 0.5 * (pData[index - 1] - pData[index + 1])
    / (pData[index - 1] + pData[index + 1] - (2.0f * pData[index]));
  g_pPeakBuf[g_peakCount++] = (index + delta) * g_fs / (fftLen - 1);

  if (g_peakBufLen <= g_peakCount)
    {
      g_peakCount = 0;
    }

  return OK;
}

static int window_hamming(float *pData, uint32_t fftLen)
{
  int ret = OK;
  int i;
  float weight;

  for (i = 0; i < fftLen / 2; i++)
    {
      weight = 0.54f - (0.46f * arm_cos_f32(2 * PI * (float)i / (fftLen - 1)));
      pData[SAMPLE_NUM * i] *= weight;
      pData[SAMPLE_NUM * fftLen - (SAMPLE_NUM * i + 1)] *= weight;
    }

  return ret;
}

static int window_hanning(float *pData, uint32_t fftLen)
{
  int ret = OK;
  int i;
  float weight;

  for (i = 0; i < fftLen / 2; i++)
    {
      weight = 0.54f - (1.0f - arm_cos_f32(2 * PI * (float)i / (fftLen - 1)));
      pData[SAMPLE_NUM * i] *= weight;
      pData[SAMPLE_NUM * fftLen - (SAMPLE_NUM * i + 1)] *= weight;
    }

  return ret;
}

#ifdef CONFIG_EXAMPLES_FFT_COMPLEX_DATA

static int fft(float *pSrc, float *pDst, uint32_t fftLen)
{
  const arm_cfft_instance_f32 *S;

  /* pre-process */

  if (g_preProcessWindow == FFT_WIN_HAMMING)
    {
      window_hamming(pSrc, fftLen);
    }
  else if (g_preProcessWindow == FFT_WIN_HANNING)
    {
      window_hanning(pSrc, fftLen);
    }

  /* calculation */

  switch (fftLen)
    {
      case 16:
        S = &arm_cfft_sR_f32_len16;
        break;
      case 32:
        S = &arm_cfft_sR_f32_len32;
        break;
      case 64:
        S = &arm_cfft_sR_f32_len64;
        break;
      case 128:
        S = &arm_cfft_sR_f32_len128;
        break;
      case 256:
        S = &arm_cfft_sR_f32_len256;
        break;
      case 512:
        S = &arm_cfft_sR_f32_len512;
        break;
      case 1024:
        S = &arm_cfft_sR_f32_len1024;
        break;
      case 2048:
        S = &arm_cfft_sR_f32_len2048;
        break;
#if 0
      /* Exclude to save the program size */

      case 4096:
        S = &arm_cfft_sR_f32_len4096;
        break;
#endif
      default:
        return ERROR;
    }

  arm_cfft_f32(S, pSrc, 0, 1);
  arm_cmplx_mag_f32(pSrc, pDst, fftLen);

  /* post-process */

  if (g_postProcessPeak)
    {
      post_peak(pDst, fftLen);
    }
  return OK;
}

#else /* CONFIG_EXAMPLES_FFT_REAL_DATA */

static int fft(float *pSrc, float *pDst, uint32_t fftLen)
{
  arm_rfft_fast_instance_f32 S;

  switch (fftLen)
    {
      case 32:
        arm_rfft_32_fast_init_f32(&S);
        break;
      case 64:
        arm_rfft_64_fast_init_f32(&S);
        break;
      case 128:
        arm_rfft_128_fast_init_f32(&S);
        break;
      case 256:
        arm_rfft_256_fast_init_f32(&S);
        break;
      case 512:
        arm_rfft_512_fast_init_f32(&S);
        break;
      case 1024:
        arm_rfft_1024_fast_init_f32(&S);
        break;
      case 2048:
        arm_rfft_2048_fast_init_f32(&S);
        break;
#if 0
      /* Exclude to save the program size */

      case 4096:
        arm_rfft_4096_fast_init_f32(&S);
        break;
#endif
      default:
        return ERROR;
    }

  /* pre-process */

  if (g_preProcessWindow == FFT_WIN_HAMMING)
    {
      window_hamming(pSrc, fftLen);
    }
  else if (g_preProcessWindow == FFT_WIN_HANNING)
    {
      window_hanning(pSrc, fftLen);
    }

  /* calculation */

  float32_t *tmpBuf = allocate_buffer(fftLen);

  arm_rfft_fast_f32(&S, pSrc, tmpBuf, 0);

  arm_cmplx_mag_f32(&tmpBuf[2], &pDst[1], fftLen / 2 - 1);
  pDst[0] = tmpBuf[0];
  pDst[fftLen / 2] = tmpBuf[1];

  /* post-process */

  if (g_postProcessPeak)
    {
      post_peak(pDst, fftLen);
    }

  return OK;
}

#endif

static int fft_async(fft_desc_t *pDesc)
{
  int ret = OK;

  for (; pDesc; pDesc = pDesc->pNext)
    {
      if (!pDesc->pDst)
        {
          pDesc->pDst = allocate_buffer(pDesc->fftLen * SAMPLE_NUM);
        }
      ret = fft(pDesc->pSrc, pDesc->pDst, pDesc->fftLen);
      if (pDesc->attribute & FFT_ATTR_NOTIFY)
        {
          mpmq_send(&g_mq, DSP_RPC_RET,
                    mpshm_virt2phys(NULL, pDesc->pDst));
        }
    }
  return ret;
}

static void ringbuf_write_float(void *pBuf, size_t size)
{
  float32_t *pFrom = (float32_t *)pBuf;
  float32_t *pTo = (float32_t *)g_inWptr;

  if (size <= (g_inBottom - g_inWptr))
    {
      arm_copy_f32(pFrom, pTo, size / 4);
    }
  else
    {
      int part = g_inBottom - g_inWptr;
      arm_copy_f32(pFrom, pTo, part / 4);
      arm_copy_f32(&pFrom[part / 4], (float32_t*)g_inTop, (size - part) / 4);
    }
}

static void ringbuf_write_short(void *pBuf, size_t size)
{
  int i;
  int16_t *pFrom = (int16_t *)pBuf;
  float32_t *pTo = (float32_t *)g_inWptr;

  if (size <= (g_inBottom - g_inWptr))
    {
      for (i = 0; i < size / 4; i++)
        {
          *pTo++ = (float32_t)pFrom[i] / 32767.0f;
        }
    }
  else
    {
      int part = g_inBottom - g_inWptr;
      for (i = 0; i < part / 4; i++)
        {
          *pTo++ = (float32_t)pFrom[i] / 32767.0f;
        }
      pTo = (float32_t *)g_inTop;
      for (; i < size / 4; i++)
        {
          *pTo++ = (float32_t)pFrom[i] / 32767.0f;
        }
    }
}

static float32_t *ringbuf_read_float(size_t size)
{
  float32_t *pSrc;
  float32_t *pFrom = (float32_t *)g_inRptr;
  int part;

#ifdef CONFIG_EXAMPLES_FFT_COMPLEX_DATA
  int k, l;
  pSrc = (float32_t *)allocate_buffer(g_fftLen * SAMPLE_NUM);

  if ((g_inRptr + size) <= g_inBottom)
    {
      for (k = 0; k < g_fftLen; k++)
        {
          pSrc[2 * k] = pFrom[k];
          pSrc[2 * k + 1] = 0.0f;
        }
    }
  else
    {
      part = g_inBottom - g_inRptr;
      for (k = 0; k < part / 4; k++)
        {
          pSrc[2 * k] = pFrom[k];
          pSrc[2 * k + 1] = 0.0f;
        }
      pFrom = (float32_t *)g_inTop;
      for (l = 0; k < size / 4; k++, l++)
        {
          pSrc[2 * k] = pFrom[l];
          pSrc[2 * k + 1] = 0.0f;
        }
    }
#else /* CONFIG_EXAMPLES_FFT_REAL_DATA */
  pSrc = (float32_t *)allocate_buffer(g_fftLen);

  if ((g_inRptr + size) <= g_inBottom)
    {
      arm_copy_f32(pFrom, pSrc, size / 4);
    }
  else
    {
      part = g_inBottom - g_inRptr;
      arm_copy_f32(pFrom, pSrc, part / 4);
      arm_copy_f32((float32_t*)g_inTop, &pSrc[part / 4], (size - part) / 4);
    }
#endif

  return pSrc;
}

#define CXD56_CPUFIFO_BASE      0x4600c400
#define CXD56_FIF_PULL_EMP      (CXD56_CPUFIFO_BASE + 0x10)

static int fft_stream_out(void)
{
  int ret = OK;

  /* Get the stored size of ring buffer */

  int stored = (g_inRptr <= g_inWptr) ?
    g_inWptr - g_inRptr : (g_inBottom - g_inRptr) + (g_inWptr - g_inTop);

  if ((stored <= 0) || (0 == g_fftLen))
    {
      return ret;
    }

  /* Read from the ring buffer and calculate FFT */

  int fftSize = g_fftLen * sizeof(float32_t);

  while (stored >= fftSize)
    {
      /* if next message is received, exit from the processing  */

      if (!*(volatile uint32_t *)CXD56_FIF_PULL_EMP)
        {
          break;
        }

      /* Allocate for input data */

      float32_t *pSrc = ringbuf_read_float(fftSize);

      /* Allocate for output data */

      void *pDst = allocate_buffer(g_fftLen * SAMPLE_NUM);

      /* Do FFT and notify to application cpu */

      fft(pSrc, pDst, g_fftLen);

      mpmq_send(&g_mq2, DSP_RPC_RET, mpshm_virt2phys(NULL, pDst));

      int consume = (g_fftShift > 0) ? g_fftShift : g_fftLen;
      consume *= sizeof(float32_t);

      g_inRptr += consume;
      if (g_inRptr >= g_inBottom)
        {
          g_inRptr = g_inTop + (g_inRptr - g_inBottom);
        }

      stored -= consume;
    }

  return ret;
}

static int fft_stream_in(void *pBuf, size_t size)
{
  int ret = OK;

  if (g_streamType == TYPE_SHORT)
    {
      size *= sizeof(float32_t) / sizeof(short);
    }

  /* Get the remain size of ring buffer */

  int remain = (g_inWptr < g_inRptr) ?
    g_inRptr - g_inWptr : (g_inBottom - g_inWptr) + (g_inRptr - g_inTop);

  /* Write to the ring buffer with converting float type */

  if (remain > size)
    {
      if (g_streamType == TYPE_FLOAT)
        {
          ringbuf_write_float(pBuf, size);
        }
      else
        {
          ringbuf_write_short(pBuf, size);
        }

      g_inWptr += size;
      if (g_inWptr >= g_inBottom)
        {
          g_inWptr = g_inTop + (g_inWptr - g_inBottom);
        }
    }

  /* Calculate FFT */

  ret = fft_stream_out();

  return ret;
}

DECLARE_RPCENTRY(FFT)(uint32_t *args)
{
  int ret = OK;
  float32_t *pSrc = (float32_t *)args[1];
  float32_t *pDst = (float32_t *)args[2];
  uint32_t fftLen = (uint32_t)args[3];

  if (!pDst)
    {
      pDst = allocate_buffer(fftLen * 2);
    }

  ret = fft(pSrc, pDst, fftLen);

  return ret;
}

DECLARE_RPCENTRY(FFT_REQUEST)(uint32_t *args)
{
  int ret = OK;
  fft_desc_t *pDesc = (fft_desc_t*)args[1];

  ret = fft_async(pDesc);

  return ret;
}

DECLARE_RPCENTRY(FFT_STREAM_INIT)(uint32_t *args)
{
  int ret = OK;
  uint32_t fftLen = (uint32_t)args[1];
  uint32_t fftShift = (uint32_t)args[2];
  fft_stream_type_t type = (fft_stream_type_t)args[3];

  g_fftLen = fftLen;
  g_fftShift = fftShift;
  g_streamType = type;

  /* Clear input buffer */

  g_inWptr = (uint8_t*)g_inBuffer;
  g_inRptr = (uint8_t*)g_inBuffer;

  return ret;
}

DECLARE_RPCENTRY(FFT_STREAM_INPUT)(uint32_t *args)
{
  int ret = OK;
  void *pBuf = (void *)args[1];
  size_t size = (size_t)args[2];

  ret = fft_stream_in(pBuf, size);

  return ret;
}

DECLARE_RPCENTRY(FFT_WINDOW)(uint32_t *args)
{
  int ret = OK;
  int type = (int)args[1];

  g_preProcessWindow = type;

  return ret;
}

DECLARE_RPCENTRY(FFT_PEAK)(uint32_t *args)
{
  int ret = OK;
  float32_t *pPeakBuf = (float32_t *)args[1];
  int bufLen          = (int)args[2];
  float32_t fs        = (float32_t)args[3];

  g_pPeakBuf = pPeakBuf;
  g_peakBufLen = bufLen;
  g_fs = fs;

  g_peakCount = 0;

  if (!g_pPeakBuf || (g_peakBufLen == 0))
    {
      g_postProcessPeak = 0;
    }
  else
    {
      g_postProcessPeak = 1;
    }

  return ret;
}

struct rpcentry_s g_rpcentry[] =
{
  RPCENTRY(FFT),
  RPCENTRY(FFT_REQUEST),
  RPCENTRY(FFT_STREAM_INIT),
  RPCENTRY(FFT_STREAM_INPUT),
  RPCENTRY(FFT_WINDOW),
  RPCENTRY(FFT_PEAK),
  RPCENTRY_TERMINATE
};

static int rpcmain(uint32_t *args)
{
  struct rpcentry_s *e;

  for (e = g_rpcentry; e->hash; e++)
    {
      if (e->hash == args[0])
        {
          return e->func(args);
        }
    }

  return INTERNAL_ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  uint32_t msgdata;
  int ret;

  /* Initialize MP message queue,
   * On the worker side, 3rd argument is ignored.
   */

  ret = mpmq_init(&g_mq, DSP_MQID, 0);
  ASSERT(ret == 0);

  ret = mpmq_init(&g_mq2, DSP_MQID2, 0);
  ASSERT(ret == 0);

  for (;;)
    {
      uint32_t *args;

      /* Receive message from supervisor */

      ret = mpmq_receive(&g_mq, &msgdata);
      if (ret == DSP_RPC_UNLOAD)
        {
          break;
        }
      if (ret == DSP_RPC_MSG)
        {
          /* Synchronous */

          args = (uint32_t *)msgdata;
          ret = rpcmain(args);
          if (ret == INTERNAL_ERROR)
            {
              mpmq_send(&g_mq, DSP_RPC_UNDEF, args[0]);
            }
          else
            {
              mpmq_send(&g_mq, DSP_RPC_RET, ret);
            }
        }
      else if (ret == DSP_RPC_MSG_ASYNC)
        {
          /* Asynchronous */

          args = (uint32_t *)msgdata;
          ret = rpcmain(args);
          if (ret == INTERNAL_ERROR)
            {
              mpmq_send(&g_mq, DSP_RPC_UNDEF, args[0]);
            }
        }
    }

  return 0;
}
