/****************************************************************************
 * voice_command/voice_command_main.cxx
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

#include <sdk/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <asmp/mpshm.h>
#include <arch/board/board.h>
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_high_level_api.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSPBIN_PATH "/mnt/sd0/BIN"

/* application running time(sec) */

#define VOICE_COMMAND_RUNNING_TIME 30

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
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
/* TODO: Stop keeping data locally */
static char mfe_coef_table[1024] __attribute__((aligned(4))) =
{
  0x81, 0x02, 0x01, 0x00, 0x90, 0x01, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40, 0x9A, 0x99, 0x19, 0x3F,
  0xCD, 0xCC, 0x4C, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0xCD, 0xCC, 0xCC, 0x3D, 0x9A, 0x99, 0x19, 0x3F,
  0x9A, 0x99, 0x99, 0x3F, 0x8F, 0xC2, 0xF5, 0x3D, 0x9A, 0x99, 0x19, 0x3E, 0x0A, 0xD7, 0x23, 0x3E,
  0xB8, 0x1E, 0x05, 0x3E, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
  0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
  0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
  0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
  0x0A, 0xD7, 0x23, 0x3D, 0x9A, 0x99, 0x19, 0x3D, 0x29, 0x5C, 0x0F, 0x3D, 0xB8, 0x1E, 0x05, 0x3D,
  0x8F, 0xC2, 0xF5, 0x3C, 0xAE, 0x47, 0xE1, 0x3C, 0xCD, 0xCC, 0xCC, 0x3C, 0xEC, 0x51, 0xB8, 0x3C,
  0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C,
  0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C,
  0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0x23, 0x3D, 0x63, 0xEE, 0x2A, 0x3F, 0x63, 0xEE, 0x2A, 0x3F,
  0xB2, 0x9D, 0x87, 0x3F, 0x63, 0xEE, 0x2A, 0x3F, 0x63, 0xEE, 0x2A, 0x3F, 0xB2, 0x9D, 0x87, 0x3F,
  0x63, 0xEE, 0x2A, 0x3F, 0x63, 0xEE, 0x2A, 0x3F, 0xB2, 0x9D, 0x87, 0x3F, 0x4C, 0xA6, 0x1A, 0x3F,
  0x4C, 0xA6, 0x1A, 0x3F, 0xCC, 0x5D, 0x8B, 0x3F, 0xA7, 0x79, 0x17, 0x3F, 0xA7, 0x79, 0x17, 0x3F,
  0x7A, 0xC7, 0x89, 0x3F, 0xEE, 0x5A, 0x12, 0x3F, 0xEE, 0x5A, 0x12, 0x3F, 0xBD, 0xE3, 0x94, 0x3F,
  0x7C, 0x61, 0x02, 0x3F, 0x7C, 0x61, 0x02, 0x3F, 0xAC, 0x8B, 0xA3, 0x3F, 0x5A, 0xF5, 0x29, 0x3F,
  0x5A, 0xF5, 0x29, 0x3F, 0x86, 0x5A, 0xA3, 0x3F, 0xF5, 0xB9, 0x6A, 0x3F, 0xF5, 0xB9, 0x6A, 0x3F,
  0xC4, 0xB1, 0xBE, 0x3F, 0x2B, 0xF6, 0x9F, 0x3F, 0x2B, 0xF6, 0x9F, 0x3F, 0x31, 0x99, 0xF2, 0x3F,
  0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00,
  0xE8, 0x03, 0x00, 0x00, 0xE2, 0x04, 0x00, 0x00, 0xDC, 0x05, 0x00, 0x00, 0x08, 0x07, 0x00, 0x00,
  0xD0, 0x07, 0x00, 0x00, 0xC4, 0x09, 0x00, 0x00, 0xB8, 0x0B, 0x00, 0x00, 0xA0, 0x0F, 0x00, 0x00,
  0x40, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x47, 0x27, 0xBC,
  0x5B, 0xEB, 0x0B, 0xBC, 0xCD, 0xCC, 0x4C, 0xBC, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00,
  0x33, 0x33, 0x33, 0x3F, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0xA0, 0x41, 0x66, 0x66, 0x66, 0x3F,
  0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x3F, 0x58, 0x02, 0x00, 0x00,
  0xC8, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00,
  0xE8, 0x03, 0x00, 0x00, 0x33, 0x33, 0x33, 0x3F, 0x9A, 0x99, 0x99, 0x3F, 0x0A, 0xD7, 0x23, 0x3C,
  0xCD, 0xCC, 0x4C, 0x3E, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x3F, 0xA3, 0xE8, 0xA1, 0x3E,
  0xEC, 0x51, 0x38, 0x3E, 0x8F, 0xC2, 0xF5, 0x3D, 0x00, 0x00, 0x40, 0x3F, 0x33, 0x33, 0x33, 0x3F,
  0x9A, 0x99, 0x59, 0x3F, 0x9A, 0x99, 0x59, 0x3F, 0x00, 0x00, 0x80, 0x3E, 0x52, 0xB8, 0x7E, 0x3F,
  0x33, 0x33, 0x73, 0x3F, 0x52, 0xB8, 0x7E, 0x3F, 0x00, 0x00, 0x00, 0x3F, 0xB9, 0xFC, 0x7F, 0x3F,
  0x3B, 0xDF, 0x7F, 0x3F, 0x9A, 0x99, 0x99, 0x3F, 0x8F, 0xC2, 0x75, 0x3C, 0x6F, 0x12, 0x03, 0x3B,
  0x6F, 0x12, 0x83, 0x3A, 0xCD, 0xCC, 0x4C, 0x3E, 0x0A, 0xD7, 0x23, 0x3C, 0x0A, 0xD7, 0x23, 0x3C,
  0x66, 0x66, 0x66, 0x3F, 0x52, 0xB8, 0x7E, 0x3F, 0x77, 0xBE, 0x7F, 0x3F, 0x1C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x30, 0x90, 0xA9, 0x42, 0x4B, 0xC0, 0x30, 0x43, 0x4E, 0x42, 0x8A, 0x43,
  0x23, 0x5F, 0xC0, 0x43, 0x70, 0x12, 0xFB, 0x43, 0xE3, 0x5F, 0x1D, 0x44, 0x95, 0xE9, 0x3F, 0x44,
  0xE4, 0x60, 0x65, 0x44, 0xAF, 0x02, 0x87, 0x44, 0xF9, 0x0D, 0x9D, 0x44, 0xB6, 0xF7, 0xB4, 0x44,
  0x76, 0xE8, 0xCE, 0x44, 0x3A, 0x0C, 0xEB, 0x44, 0x5E, 0xC9, 0x04, 0x45, 0xE3, 0x57, 0x15, 0x45,
  0xC1, 0x4D, 0x27, 0x45, 0x6F, 0xC9, 0x3A, 0x45, 0xFB, 0xEB, 0x4F, 0x45, 0x3D, 0xD9, 0x66, 0x45,
  0x1B, 0xB8, 0x7F, 0x45, 0x62, 0x59, 0x8D, 0x45, 0x7D, 0xFB, 0x9B, 0x45, 0x33, 0xDB, 0xAB, 0x45,
  0x6F, 0x13, 0xBD, 0x45, 0x69, 0xC1, 0xCF, 0x45, 0xCE, 0x04, 0xE4, 0x45, 0x00, 0x00, 0xFA, 0x45,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x05, 0x00, 0x00, 0x00, 0x77, 0xBE, 0x7F, 0x3F, 0x66, 0x66, 0x66, 0x3F, 0x00, 0x00, 0x00, 0x00,
  0x6F, 0x12, 0x83, 0x3A, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
  0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
  0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
  0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
  0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x6F, 0x12, 0x03, 0x3B, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
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
  bool result = false;

  /* Create manager of AudioSubSystem. */

  AudioSubSystemIDs ids;
  ids.app         = MSGQ_AUD_APP;
  ids.mng         = MSGQ_AUD_MGR;
  ids.player_main = 0xFF;
  ids.player_sub  = 0xFF;
  ids.mixer       = MSGQ_AUD_OUTPUT_MIX;
  ids.recorder    = 0xFF;
  ids.effector    = MSGQ_AUD_SOUND_EFFECT;
  ids.recognizer  = MSGQ_AUD_RCG_CMD;

  AS_CreateAudioManager(ids, app_attention_callback);

  /* Create effector feature. */

  AsCreateEffectorParam_t effector_create_param;
  effector_create_param.msgq_id.effector   = MSGQ_AUD_SOUND_EFFECT;
  effector_create_param.msgq_id.mng        = MSGQ_AUD_MGR;
  effector_create_param.msgq_id.recognizer = MSGQ_AUD_RCG_CMD;
  effector_create_param.msgq_id.dsp        = MSGQ_AUD_DSP;
  effector_create_param.pool_id.mic_in     = MIC_IN_BUF_POOL;
  effector_create_param.pool_id.i2s_in     = I2S_IN_BUF_POOL;
  effector_create_param.pool_id.sphp_out   = HP_OUT_BUF_POOL;
  effector_create_param.pool_id.i2s_out    = I2S_OUT_BUF_POOL;
  effector_create_param.pool_id.mfe_out    = MFE_OUT_BUF_POOL;

  result = AS_CreateEffector(&effector_create_param);
  if (!result)
    {
      printf("Error: AS_CreateEffector failure. system memory insufficient!\n");
      return false;
    }

  AsCreateRecognizerParam_t recognizer_create_param;
  recognizer_create_param.msgq_id.recognizer = MSGQ_AUD_RCG_CMD;
  recognizer_create_param.msgq_id.mng        = MSGQ_AUD_MGR;
  recognizer_create_param.msgq_id.dsp        = MSGQ_AUD_DSP;
  recognizer_create_param.pool_id.wuwsr_in   = WUWSR_IN_BUF_POOL;

  result = AS_CreateRecognizer(&recognizer_create_param);
  if (!result)
    {
      printf("Error: AS_activateRecognizer failed. system memory insufficient!\n");
      return false;
    }

  /* Create renderer feature. */

  AsCreateRendererParam_t renderer_create_param;
  renderer_create_param.msgq_id.dev0_req  = MSGQ_AUD_RND_SPHP;
  renderer_create_param.msgq_id.dev0_sync = MSGQ_AUD_RND_SPHP_SYNC;
  renderer_create_param.msgq_id.dev1_req  = MSGQ_AUD_RND_I2S;
  renderer_create_param.msgq_id.dev1_sync = MSGQ_AUD_RND_I2S_SYNC;

  result = AS_CreateRenderer(&renderer_create_param);
  if (!result)
    {
      printf("Error: AS_CreateRenderer failure. system memory insufficient!\n");
      return false;
    }

  /* Create capture feature. */

  AsCreateCaptureParam_t capture_create_param;
  capture_create_param.msgq_id.dev0_req  = MSGQ_AUD_CAP_MIC;
  capture_create_param.msgq_id.dev0_sync = MSGQ_AUD_CAP_MIC_SYNC;
  capture_create_param.msgq_id.dev1_req  = MSGQ_AUD_CAP_I2S;
  capture_create_param.msgq_id.dev1_sync = MSGQ_AUD_CAP_I2S_SYNC;

  result = AS_CreateCapture(&capture_create_param);
  if (!result)
    {
      printf("Error: AS_CreateCapture failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

static void app_deact_audio_sub_system(void)
{
  AS_DeleteAudioManager();
  AS_DeleteEffector();
  AS_DeleteRecognizer();
  AS_DeleteRenderer();
  AS_DeleteCapture();
}

static bool printAudCmdResult(uint8_t command_code, AudioResult& result)
{
  if (AUDRLT_ERRORRESPONSE == result.header.result_code) {
    printf("Command code(0x%x): AUDRLT_ERRORRESPONSE:"
           "Module id(0x%x): Error code(0x%x)\n",
            command_code,
            result.error_response_param.module_id,
            result.error_response_param.error_code);
    return false;
  }
  else if (AUDRLT_ERRORATTENTION == result.header.result_code) {
    printf("Command code(0x%x): AUDRLT_ERRORATTENTION\n", command_code);
    return false;
  }
  return true;
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
  AS_ReceiveAudioResult( &result );
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

static bool app_init_output_select(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITOUTPUTSELECT;
  command.header.command_code  = AUDCMD_INITOUTPUTSELECT;
  command.header.sub_code      = 0;
  command.init_output_select_param.output_device_sel = AS_OUT_SP;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_i2s_param(void)
{
  AudioCommand command;
  command.header.packet_length  = LENGTH_INITI2SPARAM;
  command.header.command_code   = AUDCMD_INITI2SPARAM;
  command.header.sub_code       = 0;
  command.init_i2s_param.i2s_id = AS_I2S1;
  command.init_i2s_param.rate   = 48000;
  command.init_i2s_param.bypass_mode_en = AS_I2S_BYPASS_MODE_DISABLE;
  AS_SendAudioCommand(&command);

  AudioResult result;
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
  command.init_mic_gain_param.mic_gain[2] = 210;
  command.init_mic_gain_param.mic_gain[3] = 210;
  command.init_mic_gain_param.mic_gain[4] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[5] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[6] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[7] = AS_MICGAIN_HOLD;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_bbactive_status(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_BASEBAND_STATUS;
  command.header.command_code  = AUDCMD_SETBASEBANDSTATUS;
  command.header.sub_code      = 0x00;
  command.set_baseband_status_param.with_MFE           = AS_SET_BBSTS_WITH_MFE_ACTIVE;
  command.set_baseband_status_param.with_Voice_Command = AS_SET_BBSTS_WITH_VCMD_ACTIVE;
  command.set_baseband_status_param.with_MPP           = AS_SET_BBSTS_WITH_MPP_NONE;
  command.set_baseband_status_param.input_device       = AS_INPUT_DEVICE_AMIC1CH_I2S2CH;
  command.set_baseband_status_param.output_device      = AS_OUTPUT_DEVICE_SP2CH_I2S2CH;
  snprintf(command.set_baseband_status_param.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", DSPBIN_PATH);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_mfe(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITMFE;
  command.header.command_code  = AUDCMD_INITMFE;
  command.header.sub_code      = 0;
  command.init_mfe_param.input_fs           = AS_SAMPLINGRATE_16000;
  command.init_mfe_param.ref_channel_num    = AS_CHANNEL_STEREO;
  command.init_mfe_param.mic_channel_num    = AS_CHANNEL_MONO;
  command.init_mfe_param.enable_echocancel  = AS_ENABLE_ECHOCANCEL;
  command.init_mfe_param.include_echocancel = AS_INCLUDE_ECHOCANCEL;
  command.init_mfe_param.mfe_mode           = AS_MFE_MODE_SPEAKING;
  command.init_mfe_param.config_table       = (uint32_t)(mfe_coef_table);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_mpp(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITMPP;
  command.header.command_code  = AUDCMD_INITMPP;
  command.header.sub_code      = 0;
  command.init_mpp_param.output_fs          = AS_SAMPLINGRATE_48000;
  command.init_mpp_param.output_channel_num = AS_CHANNEL_STEREO;
  command.init_mpp_param.mpp_mode           = AS_MPP_MODE_XLOUD_ONLY;
  command.init_mpp_param.xloud_mode         = AS_MPP_XLOUD_MODE_DISABLE;
  command.init_mpp_param.coef_mode          = AS_MPP_COEF_SPEAKER;
  command.init_mpp_param.eax_mode           = AS_MPP_EAX_DISABLE;
  command.init_mpp_param.xloud_coef_table   = 0;
  command.init_mpp_param.eax_coef_table     = 0;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_start_bb(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STARTBB;
  command.header.command_code  = AUDCMD_STARTBB;
  command.header.sub_code      = 0x00;
  command.start_bb_param.output_device     = AS_OUTPUT_DEVICE_SP2CH_I2S2CH;
  command.start_bb_param.input_device      = AS_INPUT_DEVICE_AMIC1CH_I2S2CH;
  command.start_bb_param.select_output_mic = AS_SELECT_MIC0_OR_MIC3;
  command.start_bb_param.I2S_output_data   = AS_MFE_OUTPUT_MICSIN;
  command.start_bb_param.SP_output_data    = AS_MPP_OUTPUT_I2SIN;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_stop_bb(void)
{
  AudioCommand command;
  command.header.packet_length      = LENGTH_STOPBB;
  command.header.command_code       = AUDCMD_STOPBB;
  command.header.sub_code           = 0x00;
  command.stop_bb_param.stop_device = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_volume(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SETVOLUME;
  command.header.command_code  = AUDCMD_SETVOLUME;
  command.header.sub_code      = 0;
  command.set_volume_param.input1_db = 0;
  command.set_volume_param.input2_db = AS_VOLUME_HOLD;
  command.set_volume_param.master_db = AS_VOLUME_DAC;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static void app_voice_command_Cb(uint16_t keyword, uint8_t status)
{
  /* This function is executed by recognizer feature task.
   * If do something process, do it in another task.
   */

  switch(status)
    {
      case 0:
        /* VAD is detecting. */

        puts("->VAD FALL");
        break;

      case 1:
        /* VAD is not detecting. */

        puts("VAD RISE->");
        break;

      case 2:
        /* WakeUpWord is detecting. */

        puts("*** Found Command ***");
        break;

      default:
        break;
    }
}

bool app_start_command(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_START_VOICE_COMMAND;
  command.header.command_code  = AUDCMD_STARTVOICECOMMAND;
  command.header.sub_code      = 0;
  command.start_voice_command_param.keyword   = 0;
  command.start_voice_command_param.vad_only  = 0;
  command.start_voice_command_param.callback_function = app_voice_command_Cb;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

bool app_stop_command(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STOP_VOICE_COMMAND;
  command.header.command_code  = AUDCMD_STOPVOICECOMMAND;
  command.header.sub_code      = 0;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, 1024 * 128 * 2);
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
      printf("Error: MsgLib::initFirst() failure. %d\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initPerCpu() failure. %d\n", err);
      return false;
    }

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initFirst() failure. %d\n", err);
      return false;
    }

  err = Manager::initPerCpu(mml_data_area, NUM_MEM_POOLS);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. %d\n", err);
      return false;
    }

  /* Create static memory pool of VoiceCall. */

  const NumLayout layout_no = MEM_LAYOUT_SOUNDEFFECT;
  S_ASSERT(layout_no < NUM_MEM_LAYOUTS);
  void* work_va = translatePoolAddrToVa(MEMMGR_WORK_AREA_ADDR);
  err = Manager::createStaticPools(layout_no,
                             work_va,
                             MEMMGR_MAX_WORK_SIZE,
                             MemoryPoolLayouts[layout_no]);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. %d\n", err);
      return false;
    }

  return true;
}

static bool app_finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools();

  /* Finalize memory manager. */

  MemMgrLite::Manager::finalize();

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

#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int voice_command_main(int argc, char *argv[])
#endif
{
  printf("Start VoiceCommand example\n");

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries failure.\n");
      return 1;
    }

  /* Next, Create the features used by AudioSubSystem. */

  if (!app_create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem failure.\n");
      return 1;
    }

  /* On and after this point, AudioSubSystem must be active. */

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_power_on())
    {
      printf("Error: app_power_on failure.\n");
      return 1;
    }

  /* Set the device to output the mixed audio. */

  if (!app_init_output_select())
    {
      printf("Error: app_init_output_select failure.\n");
      return 1;
    }

  /* Set operation parameters of I2S. */

  if (!app_init_i2s_param())
    {
      printf("Error: app_init_i2s_param failure.\n");
      return 1;
    }

  /* Set the initial gain of the microphone to be used. */

  if (!app_init_mic_gain())
    {
      printf("Error: app_init_mic_gain failure.\n");
      return 1;
    }

  /* Set baseband operation mode. */

  if (!app_set_bbactive_status())
    {
      printf("Error: app_set_bbactive_status failure.\n");
      return 1;
    }

   /* Set MFE operation mode. */

  if (!app_init_mfe())
    {
      printf("Error: app_init_mfe failure.\n");
      return 1;
    }

  /* Set MPP operation mode. */

  if (!app_init_mpp())
    {
      printf("Error: app_init_mpp failure.\n");
      return 1;
    }

  /* Cancel output mute. */

  if (!app_init_volume())
    {
      printf("Error: app_init_volume failure.\n");
      return 1;
    }

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");
      return 1;
    }

  /* Start baseband operation. */

  if (!app_start_bb())
    {
      printf("Error: app_init_volume failure.\n");
      return 1;
    }

  /* Start voice command operation. */

  if (!app_start_command())
    {
      printf("Error: app_start_command failure.\n");
      return 1;
    }

  /* Running... */

  printf("Running time is %d sec\n", VOICE_COMMAND_RUNNING_TIME);
  for (int i = 0; i < VOICE_COMMAND_RUNNING_TIME; i++)
    {
      sleep(1);
    }

  /* Stop voice command operation. */

  if (!app_stop_command())
    {
      printf("Error: app_stop_command failure.\n");
      return 1;
    }

  /* Stop baseband operation. */

  if (!app_stop_bb())
    {
      printf("Error: app_stop_bb failure.\n");
      return 1;
    }

  /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

  /* Return the state of AudioSubSystem before voice_command operation. */

  if (!app_set_ready())
    {
      printf("Error: app_set_ready failure.\n");
      return 1;
    }

  /* Change AudioSubsystem to PowerOff state. */

  if (!app_power_off())
    {
      printf("Error: app_power_off failure.\n");
      return 1;
    }

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries failure.\n");
      return 1;
    }

  printf("Exit VoiceCommand example\n");

  return 0;
}
