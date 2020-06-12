/****************************************************************************
 * audio_beep/audio_beep_main.cxx
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <arch/board/board.h>
#include <arch/chip/audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*------------------------------
 * User definable definitions
 *------------------------------
 */

/*------------------------------
 * Definition specified by config
 *------------------------------
 */

/*------------------------------
 * Definition of example fixed
 *------------------------------
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool app_power_on(void)
{
  /* Enable I2S pin. */

  cxd56_audio_en_i2s_io();

  /* Enable speaker output. */

  cxd56_audio_set_spout(true);

  /* Power on Audio driver */

  if (cxd56_audio_poweron() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  /* Enable BaseBand driver output */

  if (cxd56_audio_en_output() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  return true;
}

static bool app_power_off(void)
{
  /* Disable speaker output. */

  cxd56_audio_set_spout(false);

  /* Disable I2S pin. */

  cxd56_audio_dis_i2s_io();

  /* Power off Audio driver */

  if (cxd56_audio_dis_output() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  /* Disable BaseBand driver output */

  if (cxd56_audio_poweroff() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  return true;
}

static bool app_beep(bool en = false, int16_t vol = 255, uint16_t freq = 0)
{
  if (!en)
    {
      /* Stop beep */

      if (cxd56_audio_stop_beep() != CXD56_AUDIO_ECODE_OK)
        {
          return false;
        }
    }

  if (0 != freq)
    {
      /* Set beep frequency parameter */

      if (cxd56_audio_set_beep_freq(freq) != CXD56_AUDIO_ECODE_OK)
        {
          return false;
        }
    }

  if (255 != vol)
    {
      /* Set beep volume parameter */

      if (cxd56_audio_set_beep_vol(vol) != CXD56_AUDIO_ECODE_OK)
        {
          return false;
        }
    }

  if (en)
    {
      /* Play beep */

      if (cxd56_audio_play_beep() != CXD56_AUDIO_ECODE_OK)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  /* Define beep scale and pronunciation time */

  struct {
    int fs;       /* frequency */
    int time;     /* Pronunciation time[ms] */
  }
  note[] = {
    {262, 500},
    {294, 500},
    {330, 500},
    {349, 500},
    {392, 500},
    {440, 500},
    {494, 500},
    {523, 1000},
    {523, 500},
    {494, 500},
    {440, 500},
    {392, 500},
    {349, 500},
    {330, 500},
    {294, 500},
    {262, 1000},
    {0,0}
  },
  *p_note;

  /* Set I/O parameters for power on. */

  if (!app_power_on()) 
    {
      printf("Error: app_power_on() failure.\n");
      return 1;
    }

  /* Cancel output mute. */

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");
      return 1;
    }

  printf("Start AudioBeep example\n");

  for (p_note = note; p_note->fs; p_note++)
    {
      /* Set beep. */

      if (!app_beep(1, -40, p_note->fs))
        {
          break;
        }
      usleep(p_note->time * 1000L);
    }

  /* Beep off. */

  if (!app_beep())
    {
      printf("Error: app_beep() failuer.\n");
      return 1;
    }

  /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

  printf("Stop  AudioBeep example\n");

  /* Set I/O parameters for power off. */

  if (!app_power_off())
    {
      printf("Error: app_power_off() failure.\n");
      return 1;
    }

  printf("Exit  AudioBeep example\n");

  return 0;
}
