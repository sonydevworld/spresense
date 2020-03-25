/****************************************************************************
 * audio_oscillator/audio_oscillator_main.cxx
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

#include <sdk/config.h>
#include <stdio.h>
#include <arch/board/board.h>
#include "oscillator.h"
#include "audio/audio_synthesizer_api.h"
#include "audio_synthesizer_melody.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default Volume. -20dB */

#define VOLUME                -160

/* Maximum number of channels */

#define CHANNEL_NUMBER        8

/* Oscillator parameter */

#define OSC_CH_NUM            CHANNEL_NUMBER
#define OSC_BIT_LEN           AS_BITLENGTH_16
#define OSC_SAMPLING_RATE     AS_SAMPLINGRATE_48000
#define OSC_SAMPLE_SIZE       240
#define OSC_EFFECT_ATTACK     10
#define OSC_EFFECT_DECAY      50
#define OSC_EFFECT_SUSTAIN    100
#define OSC_EFFECT_RELEASE    200

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern bool app_initialize(void);
extern void app_finalize(void);
extern bool app_set_volume(int master_db);
extern bool app_init_postproc(uint8_t  channel_num,
                              uint8_t  bit_width,
                              uint32_t sampling_rate);

/* Synthesizer */

extern bool app_create_synthesizer(void);
extern bool app_set_synthesizer_status(void);
extern bool app_start_synthesizer(void);
extern bool app_stop_synthesizer(void);
extern bool app_deactive_synthesizer(void);
extern bool app_set_frequency_synthesizer(uint8_t  channel_number,
                                          uint32_t frequency[]);
extern bool app_set_envelope_synthesizer(uint8_t  channel_no,
                                         uint16_t attack,
                                         uint16_t decay,
                                         uint16_t sustain,
                                         uint16_t release);
extern bool app_init_synthesizer(uint8_t  channel_num,
                                 uint8_t  bit_width,
                                 uint32_t sampling_rate,
                                 uint16_t sample_size);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void app_main_process(void)
{
  p_node = node;

 /* Set output mute. */

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");
    }

  /* Start synthesizer operation. */

  if (!app_start_synthesizer())
    {
      printf("Error: app_start_synthesizer() failure.\n");
      return;
    }

  /* Try the effect on channel 0 */

  app_set_envelope_synthesizer(0,
                               OSC_EFFECT_ATTACK,
                               OSC_EFFECT_DECAY,
                               OSC_EFFECT_SUSTAIN,
                               OSC_EFFECT_RELEASE);

  printf("Running...\n");

  for (; p_node->fs[0] != M_END; p_node++)
    {
      /* Set frequency. */

      if (!app_set_frequency_synthesizer(CHANNEL_NUMBER,
                                         p_node->fs))
        {
          printf("Error: app_set_frequency_synthesizer() failure.\n");
          break;
        }

      usleep(300 * 1000);
    }

 /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
    }

  /* Stop synthesizer operation. */

  if (!app_stop_synthesizer())
    {
      printf("Error: app_stop_operation() failure.\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  printf("Start AudioOscillator example\n");

  /* Waiting for SD card mounting. */

  sleep(1);

  /* Initialize */

  if (!app_initialize())
    {
      printf("Error: app_initialize() failure.\n");
    }

  /* Sreate synthesizer sub system */

  else if (!app_create_synthesizer())
    {
      printf("Error: app_initialize() failure.\n");
    }

  else if (!app_init_postproc(OSC_CH_NUM,
                              OSC_BIT_LEN,
                              OSC_SAMPLING_RATE))
    {
      printf("Error: app_init_postproc() failure.\n");
    }

  /* Set synthesizer operation mode. */

  else if (!app_set_synthesizer_status())
    {
      printf("Error: app_set_synthesizer_status() failure.\n");
    }

  /* Initialize synthesizer. */

  else if (!app_init_synthesizer(OSC_CH_NUM,
                                 OSC_BIT_LEN,
                                 OSC_SAMPLING_RATE,
                                 OSC_SAMPLE_SIZE))
    {
      printf("Error: app_init_synthesizer() failure.\n");
    }
  else
    {
      /* Set volume */

      app_set_volume(VOLUME);

      /* Running... */

      app_main_process();
    }

  /* Unload synthesizer operation. */

  if (!app_deactive_synthesizer())
    {
      printf("Error: app_deactive_synthesizer failuer.\n");
    }

  /* Finalize */

  app_finalize();

  printf("Exit AudioOscillator example\n");

  return 0;
}
