/****************************************************************************
 * audio_through/audio_through_main.cxx
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
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <arch/board/board.h>
#include <asmp/mpshm.h>
#include <arch/chip/pm.h>
#include <sys/stat.h>

#include "memutils/os_utils/chateau_osal.h"
#include "audio/audio_high_level_api.h"
#include "memutils/message/Message.h"
#include "include/msgq_id.h"
#include "include/msgq_pool.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default Volume. -20dB */

#define DEF_VOLUME -20

/* Play time(sec) */

#define PLAY_TIME 30

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

enum audio_through_test_path_e
{
  TEST_PATH_IN_MIC_OUT_SP = 0,
  TEST_PATH_IN_MIC_OUT_I2S,
  TEST_PATH_IN_I2S_OUT_SP,
  TEST_PATH_IN_MIC_I2S_OUT_I2S_SP,
  TEST_PATH_NUM
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

char s_path_name[TEST_PATH_NUM][64] =
{
  "mic in -> sp out",
  "mic in -> i2s out",
  "i2s in -> sp out",
  "mic in -> i2s out, and i2s in -> sp outt"
};

/* For share memory. */

static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool printAudCmdResult(uint8_t command_code, AudioResult& result)
{
  if (AUDRLT_ERRORRESPONSE == result.header.result_code)
    {
      printf("Command code(0x%x): AUDRLT_ERRORRESPONSE:"
             "Module id(0x%x): Error code(0x%x)\n",
              command_code,
              result.error_response_param.module_id,
              result.error_response_param.error_code);
      return false;
    }
  else if (AUDRLT_ERRORATTENTION == result.header.result_code)
    {
      printf("Command code(0x%x): AUDRLT_ERRORATTENTION\n", command_code);
      return false;
    }
  return true;
}

static void app_attention_callback(const ErrorAttentionParam *attparam)
{
  printf("Attention!! %s L%d ecode %d subcode %d\n",
          attparam->error_filename,
          attparam->line_number,
          attparam->error_code,
          attparam->error_att_sub_code);
}

static bool app_create_audio_sub_system(void)
{
  /* Create manager of AudioSubSystem. */

  AudioSubSystemIDs ids;
  ids.app         = MSGQ_AUD_APP;
  ids.mng         = MSGQ_AUD_MGR;
  ids.player_main = 0xFF;
  ids.player_sub  = 0xFF;
  ids.mixer       = 0xFF;
  ids.recorder    = 0xFF;
  ids.effector    = 0xFF;
  ids.recognizer  = 0xFF;

  AS_CreateAudioManager(ids, app_attention_callback);

  /* Set callback function of attention message */

  return true;
}

static void app_deact_audio_sub_system(void)
{
  AS_DeleteAudioManager();
}

static bool app_power_on(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_POWERON;
  command.header.command_code  = AUDCMD_POWERON;
  command.header.sub_code      = 0x00;
  command.power_on_param.enable_sound_effect = AS_DISABLE_SOUNDEFFECT;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_power_off(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_POWEROFF_STATUS;
  command.header.command_code  = AUDCMD_SETPOWEROFFSTATUS;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_ready(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_READY_STATUS;
  command.header.command_code  = AUDCMD_SETREADYSTATUS;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_volume(int master_db)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SETVOLUME;
  command.header.command_code  = AUDCMD_SETVOLUME;
  command.header.sub_code      = 0;
  command.set_volume_param.input1_db = 0;
  command.set_volume_param.input2_db = AS_VOLUME_MUTE;
  command.set_volume_param.master_db = master_db;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_mute()
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SETVOLUME;
  command.header.command_code  = AUDCMD_SETVOLUME;
  command.header.sub_code      = 0;
  command.set_volume_param.input1_db = 0;
  command.set_volume_param.input2_db = AS_VOLUME_MUTE;
  command.set_volume_param.master_db = AS_VOLUME_MUTE;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_through_status(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_THROUGH_STATUS;
  command.header.command_code = AUDCMD_SETTHROUGHSTATUS;
  command.header.sub_code = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_through_path(audio_through_test_path_e path_type)
{
  AudioResult result;
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_THROUGH_PATH;
  command.header.command_code = AUDCMD_SETTHROUGHPATH;
  command.header.sub_code = 0x00;

  switch (path_type)
    {
      case TEST_PATH_IN_MIC_OUT_SP: /* mic in -> sp out */
        command.set_through_path.path1.en  = true;
        command.set_through_path.path1.in  = AS_THROUGH_PATH_IN_MIC;
        command.set_through_path.path1.out = AS_THROUGH_PATH_OUT_MIXER1;
        command.set_through_path.path2.en  = false;
        break;

      case TEST_PATH_IN_MIC_OUT_I2S: /* mic in -> i2s out */
        command.set_through_path.path1.en  = true;
        command.set_through_path.path1.in  = AS_THROUGH_PATH_IN_MIC;
        command.set_through_path.path1.out = AS_THROUGH_PATH_OUT_I2S1;
        command.set_through_path.path2.en  = false;
        break;

      case TEST_PATH_IN_I2S_OUT_SP: /* i2s in -> sp out */
        command.set_through_path.path1.en  = true;
        command.set_through_path.path1.in  = AS_THROUGH_PATH_IN_I2S1;
        command.set_through_path.path1.out = AS_THROUGH_PATH_OUT_MIXER1;
        command.set_through_path.path2.en  = false;
        break;

      case TEST_PATH_IN_MIC_I2S_OUT_I2S_SP: /* mic in -> i2s out, and i2s in -> sp out */
        command.set_through_path.path1.en  = true;
        command.set_through_path.path1.in  = AS_THROUGH_PATH_IN_MIC;
        command.set_through_path.path1.out = AS_THROUGH_PATH_OUT_I2S1;
        command.set_through_path.path2.en  = true;
        command.set_through_path.path2.in  = AS_THROUGH_PATH_IN_I2S1;
        command.set_through_path.path2.out = AS_THROUGH_PATH_OUT_MIXER1;
        break;

      default:
        printf("path type is illegal(%d)\n", path_type);
        return false;
    }

  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_mic_gain(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITMICGAIN;
  command.header.command_code  = AUDCMD_INITMICGAIN;
  command.header.sub_code      = 0;
  command.init_mic_gain_param.mic_gain[0] = 210;
  command.init_mic_gain_param.mic_gain[1] = 210;
  command.init_mic_gain_param.mic_gain[2] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[3] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[4] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[5] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[6] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[7] = AS_MICGAIN_HOLD;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_libraries(void)
{
  int ret;
  uint32_t addr = MSGQ_TOP_DRM;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, 1024 * 128);
  if (ret < 0)
    {
      printf("Error: mpshm_init() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      printf("Error: mpshm_remap() failure. %d\n", ret);
      return false;
    }

  /* Initalize MessageLib. */

  err_t err = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  return true;
}

static bool app_finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy shared memory. */

  int ret;
  ret = mpshm_detach(&s_shm);
  if (ret < 0)
    {
      printf("Error: mpshm_detach() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_destroy(&s_shm);
  if (ret < 0)
    {
      printf("Error: mpshm_destroy() failure. %d\n", ret);
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  printf("Start AudioThrough example\n");

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
      return 1;
    }

  /* Next, Create the features used by AudioSubSystem. */

  if (!app_create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");
      return 1;
    }

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_power_on())
    {
      printf("Error: app_power_on() failure.\n");
      return 1;
    }

  /* Set through operation mode. */

  if (!app_set_through_status())
    {
      printf("Error: app_set_through_status() failure.\n");
      return 1;
    }

  /* Initialize Mic gain */

  if (!app_init_mic_gain())
    {
      printf("Error: app_init_mic_gain() failure.\n");
      return 1;
    }

  /* Start through operation. */

  for (int type = 0; type < TEST_PATH_NUM; type++)
    {
       /* Set output mute. */

       if (board_external_amp_mute_control(true) != OK)
         {
           printf("Error: board_external_amp_mute_control(true) failuer.\n");
           return 1;
         }

      if (!app_set_through_path((audio_through_test_path_e)type))
        {
          printf("Error: app_set_through_path() failure.\n");
          return 1;
        }

      /* If output speaker, cancel mute. */

      if (!(type == TEST_PATH_IN_MIC_OUT_I2S))
        {
          /* Cancel mute. */

          if (!app_set_volume(DEF_VOLUME))
            {
              printf("Error: app_set_volume() failure.\n");
              return 1;
            }

          if (board_external_amp_mute_control(false) != OK)
            {
              printf("Error: board_external_amp_mute_control(false) failuer.\n");
              return 1;
            }
        }

      /* Running... */

      printf("Test path %s(%d sec)\n", s_path_name[type], PLAY_TIME);
      for(int i = 0; i < PLAY_TIME; i++)
        {
          sleep(1);
        }

      /* Set mute. */

      app_set_mute();
    } 

  /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

  /* Return the state of AudioSubSystem before voice_call operation. */

  if (!app_set_ready())
    {
      printf("Error: app_set_ready() failure.\n");
      return 1;
    }

  /* Change AudioSubsystem to PowerOff state. */

  if (!app_power_off())
    {
      printf("Error: app_power_off() failure.\n");
      return 1;
    }

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
      return 1;
    }

  printf("Exit AudioThrough example\n");

  return 0;
}
