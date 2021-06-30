/****************************************************************************
 * externals/tensorflow/tf_audio_provider.cxx
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include <strings.h>

#include <memutils/simple_fifo/CMN_SimpleFifo.h>

#include "tf_audio_util.h"

#include "spresense_audio_provider.h"

/****************************************************************************
 * Application parameters
 ****************************************************************************/

/* SAMPLING_RATE
 * 44.1kHz : AS_SAMPLINGRATE_44100
 * 48kHz   : AS_SAMPLINGRATE_48000
 * 192kHz  : AS_SAMPLINGRATE_192000
 */

#define SAMPLINGRATE          AS_SAMPLINGRATE_48000

/* CHANNEL_NUM
 * MONO (1ch)   : AS_CHANNEL_MONO
 * STEREO (2ch) : AS_CHANNEL_STEREO
 * 4ch          : AS_CHANNEL_4CH
 */

#define CHANNEL_NUM        AS_CHANNEL_MONO

/* BIT_LENGTH
 * 16bit : AS_BITLENGTH_16
 * 24bit : AS_BITLENGTH_24
 */

#define BIT_LENGTH            AS_BITLENGTH_16

/* Recording default time(sec). */

#define PCM_CAPTURE_TIME     (10)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define READ_BUFSIZE  (AUDIO_FRAME_SAMPLE_LENGTH * CHANNEL_NUM * 2)  /* 768sample, 1ch, 16bit */

/****************************************************************************
 * Private data
 ****************************************************************************/

static CMN_SimpleFifoHandle s_fifo_handle;
static int16_t s_captured_samples[READ_BUFSIZE/2];
static int16_t s_provide_samples[READ_BUFSIZE/2];  /* This is for 16kHz sample */
static int32_t s_lasttime_ms = 0;
static bool is_initialized = false;

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static void outputDeviceCallback(uint32_t size)
{
    /* do nothing */
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static void app_attention_callback(const ErrorAttentionParam *attparam)
{
  printf("Attention!! %s L%d ecode %d subcode %ld\n",
          attparam->error_filename,
          attparam->line_number,
          attparam->error_code,
          attparam->error_att_sub_code);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

extern "C"
int spresense_audio_getsamples(
    int start_ms, int duration_ms, int required_fs,
    int *sample_size, int16_t ** audio_samples)
{
  size_t samples_in_fifo;

  if (!is_initialized)
    {
      if (!initailze_audio_captureing(&s_fifo_handle,
            app_attention_callback, outputDeviceCallback,
            SAMPLINGRATE, CHANNEL_NUM, BIT_LENGTH))
        {
          printf("Error: initialize_audio_captureing()\n");
          return -1;
        }
      if (!start_recording(&s_fifo_handle))
        {
          printf("Error: start_recording()\n");
          finalize_audio_capturing();
          return -1;
        }
      is_initialized = true;
    }

  *sample_size = 0;
  size_t remain = 512;
  while (remain)
    {
      samples_in_fifo = CMN_SimpleFifoGetOccupiedSize(&s_fifo_handle);
      samples_in_fifo = samples_in_fifo > READ_BUFSIZE ? READ_BUFSIZE : samples_in_fifo;
      samples_in_fifo = samples_in_fifo > (remain*2*3) ? (remain*2*3) : samples_in_fifo;

      if (samples_in_fifo!=0)
        {
          if (CMN_SimpleFifoPoll(&s_fifo_handle,
                                (void*)s_captured_samples,
                                samples_in_fifo) == 0)
            {
              printf("ERROR: Fail to get data from simple FIFO.\n");
              return -1;
            }
          else
            {
              samples_in_fifo /= 2; /* bytes to half words */
              for (size_t i=0; i<samples_in_fifo; i+=3, (*sample_size)++, remain--)
                {
                  s_provide_samples[*sample_size] = s_captured_samples[i];
                }
              // s_lasttime_ms += samples_in_fifo * 1000 / SAMPLINGRATE;
            }
        }
    }
    
  *audio_samples = s_provide_samples;
  return 0;
}

extern "C"
int32_t spresense_audio_lasttimestamp(void)
{
  s_lasttime_ms += 100; /* Referd from MOCK */
  return s_lasttime_ms;
}
