/****************************************************************************
 * audio_oscillator/audio_synthesizer.cxx
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include "audio/audio_synthesizer_api.h"
#include "audio/audio_outputmix_api.h"
#include "include/mem_layout.h"
#include "include/msgq_id.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSP file path */

#define DSPBIN_PATH           "/mnt/sd0/BIN"

/* Waiting for reception */

#define RECEIVE_OBJECT_REPLY()  receive_object_reply(__LINE__)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_AUDIO_OSCILLATOR_USEPOSTPROC
static struct
{
  uint8_t  channel_num;
  uint8_t  bit_width;
}
osc_param;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_AUDIO_OSCILLATOR_USEPOSTPROC
static void synthesizer_done_callback(AsPcmDataParam param)
{
  /* You can process a data here. 
   *
   * Example)
   *
   * Outputs the waveform of the number of channels
   * input from the oscillator as two channels.
   * 
   */ 

  uint8_t   input_ch    = osc_param.channel_num;
  uint16_t  byte_length = osc_param.bit_width / 8;
  int16_t  *input       = (int16_t *)param.mh.getPa();
  int16_t  *output      = input;
  uint32_t  size        = param.size / (byte_length * input_ch);
  int32_t   data_l;

  for (uint32_t j = 0; j < size; j++)
    {
      data_l = 0;

      for (uint8_t i = 0; i < input_ch; i++)
        {
          data_l += *input++;
        }

      data_l /= input_ch;

      *output++ = data_l;
      *output++ = data_l;
    }

  param.size = ((uint8_t *)output - (uint8_t *)param.mh.getPa());

  /* Notify the processed data to the mixer */

  AsSendDataOutputMixer data;

  data.handle   = OutputMixer0;
  data.callback = param.callback;
  data.pcm      = param;

  AS_SendDataOutputMixer(&data);
}
#endif

/* ------------------------------------------------------------------------ */
static bool receive_object_reply(int line_no)
{
  AudioObjReply info;

  AS_ReceiveObjectReply(MSGQ_AUD_APP, &info);

  if (info.result != OK)
    {
      printf("RECEIVE_OBJECT_REPLY[%d] error!\n", line_no);
      printf("  id        0x%x\n", info.id);
      printf("  type      0x%x\n", info.type);
      printf("  module_id 0x%x\n", info.module_id);
      printf("  result    0x%x\n", info.result);
      return false;
    }

  return true;
}

/* ------------------------------------------------------------------------ */
bool app_create_synthesizer(void)
{
  /* Create Synthesizer */

  AsCreateSynthesizerParam_t  syn =
  {
    .msgq_id =
    {
      .synthesizer = MSGQ_AUD_SYNTHESIZER,
      .mng         = MSGQ_AUD_APP,
      .dsp         = MSGQ_AUD_DSP,
    },
    .pool_id =
    {
      .output      = S0_REND_PCM_BUF_POOL,
      .dsp         = S0_OSC_APU_CMD_POOL,
    }
  };

  return AS_CreateMediaSynthesizer(&syn, NULL);
}

/* ------------------------------------------------------------------------ */
bool app_set_synthesizer_status(void)
{
  AsActivateSynthesizer act;

  act.cb = NULL;

  AS_ActivateMediaSynthesizer(&act);

  return RECEIVE_OBJECT_REPLY();
}

/* ------------------------------------------------------------------------ */
bool app_init_synthesizer(uint8_t  channel_num,
                          uint8_t  bit_width,
                          uint32_t sampling_rate,
                          uint16_t sample_size,
                          uint16_t attack,
                          uint16_t decay,
                          uint16_t sustain,
                          uint16_t release)
{
  AsInitSynthesizerParam  init;

  init.type                     = AsSynthesizerSinWave;
  init.channel_num              = channel_num;
  init.sampling_rate            = sampling_rate;
  init.bit_width                = bit_width;
  init.sample_size              = sample_size;

  /* PCM data path setting */

#ifdef CONFIG_EXAMPLES_AUDIO_OSCILLATOR_USEPOSTPROC
  init.data_path                = AsSynthesizerDataPathMessage;
  init.dest.msg.id              = MSGQ_AUD_OUTPUT_MIX;
  init.dest.msg.identifier      = OutputMixer0;
#else
  osc_param.channel_num         = channel_num;
  osc_param.bit_width           = bit_width;
  init.data_path                = AsSynthesizerDataPathCallback;
  init.dest.cb                  = synthesizer_done_callback;
#endif

  /* Initial value setting of envelope generator */

  init.attack                   = attack;
  init.decay                    = decay;
  init.sustain                  = sustain;
  init.release                  = release;

  sprintf(init.dsp_path, "%s/%s", DSPBIN_PATH, "OSCPROC");

  AS_InitMediaSynthesizer(&init);

  return RECEIVE_OBJECT_REPLY();
}

/* ------------------------------------------------------------------------ */
bool app_init_synthesizer(uint8_t  channel_num,
                          uint8_t  bit_width,
                          uint32_t sampling_rate,
                          uint16_t sample_size)
{
  return app_init_synthesizer(channel_num,
                              bit_width,
                              sampling_rate,
                              sample_size,
                              1,
                              1,
                              100,
                              1);
}

/* ------------------------------------------------------------------------ */
bool app_start_synthesizer(void)
{
  AS_StartMediaSynthesizer();

  return RECEIVE_OBJECT_REPLY();
}

/* ------------------------------------------------------------------------ */
bool app_stop_synthesizer(void)
{
  AS_StopMediaSynthesizer();

  return RECEIVE_OBJECT_REPLY();
}

/* ------------------------------------------------------------------------ */
bool app_deactive_synthesizer(void)
{
  AS_DeactivateMediaSynthesizer();

  return RECEIVE_OBJECT_REPLY();
}

/* ------------------------------------------------------------------------ */
bool app_set_frequency_synthesizer(uint8_t  channel_number,
                                   uint32_t frequency[])
{
  AsSetSynthesizer set_param;
  bool             res = true;

  for (int i = 0; i < channel_number; i++)
    {
      set_param.channel_no = i;
      set_param.frequency  = frequency[i];

      AS_SetFrequencyMediaSynthesizer(&set_param);

      if (!(res = RECEIVE_OBJECT_REPLY()))
        {
          break;
        }
    }

  return res;
}

/* ------------------------------------------------------------------------ */
bool app_set_envelope_synthesizer(uint8_t  channel_no,
                                  uint16_t attack,
                                  uint16_t decay,
                                  uint16_t sustain,
                                  uint16_t release)
{
  AsSetSynthesizer set_param;

  set_param.channel_no = channel_no;
  set_param.attack     = attack;
  set_param.decay      = decay;
  set_param.sustain    = sustain;
  set_param.release    = release;

  AS_SetEnvelopeMediaSynthesizer(&set_param);

  return RECEIVE_OBJECT_REPLY();
}
