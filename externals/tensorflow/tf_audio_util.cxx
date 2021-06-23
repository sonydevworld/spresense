/****************************************************************************
 * externals/tensorflow/tf_audio_util.cxx
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
#include <asmp/mpshm.h>

#include <memutils/simple_fifo/CMN_SimpleFifo.h>
#include <memutils/memory_manager/MemHandle.h>
#include <memutils/message/Message.h>
#include <audio/audio_high_level_api.h>
#include <audio/utilities/frame_samples.h>
#include <audio/audio_recorder_api.h>
#include <audio/audio_frontend_api.h>
#include <audio/audio_capture_api.h>

#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"

#include "tf_audio_util.h"

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

#define DSPBIN_PATH   "/mnt/sd0/BIN"

#define SIMPLE_FIFO_FRAME_NUM (60)

#define SIMPLE_FIFO_BUF_SIZE  \
  (AUDIO_FRAME_SAMPLE_LENGTH * 4 /* channels */ * SIMPLE_FIFO_FRAME_NUM * 2)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mpshm_t s_shm;
static uint32_t fifo_memory[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
static AsRecorderOutputDeviceHdlr  s_output_device;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool init_fifo_info(CMN_SimpleFifoHandle *handle,
    AsRecorderOutputDeviceHdlr *odev, fifo_fill_cb_t cb);
static bool init_captureing(annotation_cb_t cb);
static void deactivate_audio(void);
static bool audio_power_on(void);
static bool audio_power_off(void);
static bool standby_audio(void);
static bool set_mic_gain(int16_t gain);
static bool audio_set_output(AsRecorderOutputDeviceHdlr *handl);
static bool set_recording_mode(uint32_t sampling_rate, uint32_t chnum, uint32_t sample_per_bits);
static bool set_audio_clkmode(uint32_t sampling_rate);
static bool prepare_audio_parts(void);
static bool cleanup_audio_parts(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool printAudCmdResult(uint8_t command_code, AudioResult& result)
{
  if (AUDRLT_ERRORRESPONSE == result.header.result_code) {
    printf("Command code(0x%x): AUDRLT_ERRORRESPONSE:"
           "Module id(0x%x): Error code(0x%lx)\n",
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

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool init_fifo_info(CMN_SimpleFifoHandle *handle,
    AsRecorderOutputDeviceHdlr *odev, fifo_fill_cb_t cb)
{
  if (CMN_SimpleFifoInitialize(handle,
                               fifo_memory,
                               sizeof(fifo_memory), NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(handle);

  odev->simple_fifo_handler = (void*)(handle);
  odev->callback_function = cb;

  return true;
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool init_captureing(annotation_cb_t cb)
{
  bool result = false;

  /* Create manager of AudioSubSystem. */

  AudioSubSystemIDs ids;
  ids.app         = MSGQ_AUD_APP;
  ids.mng         = MSGQ_AUD_MGR;
  ids.player_main = 0xFF;
  ids.player_sub  = 0xFF;
  ids.micfrontend = MSGQ_AUD_FRONTEND;
  ids.mixer       = 0xFF;
  ids.recorder    = MSGQ_AUD_RECORDER;
  ids.effector    = 0xFF;
  ids.recognizer  = 0xFF;

  AS_CreateAudioManager(ids, cb);

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MGR;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_INPUT_BUF_POOL;
  frontend_create_param.pool_id.output      = S0_NULL_POOL;
  frontend_create_param.pool_id.dsp         = S0_PRE_APU_CMD_POOL;

  AS_CreateMicFrontend(&frontend_create_param, NULL);

  /* Create Recorder. */

  AsCreateRecorderParams_t recorder_create_param;
  recorder_create_param.msgq_id.recorder      = MSGQ_AUD_RECORDER;
  recorder_create_param.msgq_id.mng           = MSGQ_AUD_MGR;
  recorder_create_param.msgq_id.dsp           = MSGQ_AUD_DSP;
  recorder_create_param.pool_id.input         = S0_INPUT_BUF_POOL;
  recorder_create_param.pool_id.output        = S0_ES_BUF_POOL;
  recorder_create_param.pool_id.dsp           = S0_ENC_APU_CMD_POOL;

  result = AS_CreateMediaRecorder(&recorder_create_param, NULL);
  if (!result)
    {
      printf("Error: AS_CreateMediaRecorder() failure. system memory insufficient!\n");
      return false;
    }

  /* Create Capture feature. */

  AsCreateCaptureParam_t capture_create_param;
  capture_create_param.msgq_id.dev0_req  = MSGQ_AUD_CAP;
  capture_create_param.msgq_id.dev0_sync = MSGQ_AUD_CAP_SYNC;
  capture_create_param.msgq_id.dev1_req  = 0xFF;
  capture_create_param.msgq_id.dev1_sync = 0xFF;

  result = AS_CreateCapture(&capture_create_param);
  if (!result)
    {
      printf("Error: As_CreateCapture() failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static void deactivate_audio(void)
{
  AS_DeleteAudioManager();
  AS_DeleteMediaRecorder();
  AS_DeleteMicFrontend();
  AS_DeleteCapture();
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool audio_power_on(void)
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

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool audio_power_off(void)
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

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool standby_audio(void)
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

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool set_mic_gain(int16_t gain)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITMICGAIN;
  command.header.command_code  = AUDCMD_INITMICGAIN;
  command.header.sub_code      = 0;
  command.init_mic_gain_param.mic_gain[0] = gain;
  command.init_mic_gain_param.mic_gain[1] = gain;
  command.init_mic_gain_param.mic_gain[2] = gain;
  command.init_mic_gain_param.mic_gain[3] = gain;
  command.init_mic_gain_param.mic_gain[4] = gain;
  command.init_mic_gain_param.mic_gain[5] = gain;
  command.init_mic_gain_param.mic_gain[6] = gain;
  command.init_mic_gain_param.mic_gain[7] = gain;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool audio_set_output(AsRecorderOutputDeviceHdlr *handl)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_RECORDER_STATUS;
  command.header.command_code  = AUDCMD_SETRECORDERSTATUS;
  command.header.sub_code      = 0x00;
  command.set_recorder_status_param.input_device          = AS_SETRECDR_STS_INPUTDEVICE_MIC;
  command.set_recorder_status_param.input_device_handler  = 0x00;
  command.set_recorder_status_param.output_device         = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
  command.set_recorder_status_param.output_device_handler = handl;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool set_recording_mode(uint32_t sampling_rate, uint32_t chnum, uint32_t sample_per_bits)
{
  AudioCommand command;
  AudioResult result;
  bool ret;

  /* Initialize Recorder. */

  command.header.packet_length = LENGTH_INIT_RECORDER;
  command.header.command_code  = AUDCMD_INITREC;
  command.header.sub_code      = 0x00;
  command.recorder.init_param.sampling_rate  = sampling_rate;
  command.recorder.init_param.channel_number = chnum;
  command.recorder.init_param.bit_length     = sample_per_bits;
  command.recorder.init_param.codec_type     = AS_CODECTYPE_LPCM;
  snprintf(command.recorder.init_param.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", DSPBIN_PATH);

  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  ret = printAudCmdResult(command.header.command_code, result);
  if (!ret)
    {
      return ret;
    }

  /* Initialize MicFrontend. */

  command.header.packet_length = LENGTH_INIT_MICFRONTEND;
  command.header.command_code  = AUDCMD_INIT_MICFRONTEND;
  command.header.sub_code      = 0x00;
  command.init_micfrontend_param.ch_num       = chnum;
  command.init_micfrontend_param.bit_length   = sample_per_bits;
  command.init_micfrontend_param.samples      = getCapSampleNumPerFrame(AS_CODECTYPE_LPCM,
                                                                        sampling_rate);
  command.init_micfrontend_param.preproc_type = AsMicFrontendPreProcThrough;
  snprintf(command.init_micfrontend_param.preprocess_dsp_path,
           AS_PREPROCESS_FILE_PATH_LEN,
           "%s/%s", DSPBIN_PATH, "PREPROC");
  command.init_micfrontend_param.data_dest    = AsMicFrontendDataToRecorder;
  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool set_audio_clkmode(uint32_t sampling_rate)
{
  AudioCommand command;
  AudioResult result;

  command.header.packet_length = LENGTH_SETRENDERINGCLK;
  command.header.command_code  = AUDCMD_SETRENDERINGCLK;
  command.header.sub_code      = 0x00;
  command.set_renderingclk_param.clk_mode
    = (sampling_rate == AS_SAMPLINGRATE_192000) ? CXD56_AUDIO_CLKMODE_HIRES : CXD56_AUDIO_CLKMODE_NORMAL;
  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool prepare_audio_parts(void)
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
      printf("Error: MsgLib::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = Manager::initPerCpu(mml_data_area, static_pools, pool_num, layout_no);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  /* Create static memory pool of VoiceCall. */

  const uint8_t sec_no = SECTION_NO0;
  const NumLayout layout_no = MEM_LAYOUT_PCM_CAPTURE;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[SECTION_NO0][layout_no][0];

  err = Manager::createStaticPools(sec_no,
                                   layout_no,
                                   work_va,
                                   S0_MEMMGR_WORK_AREA_SIZE,
                                   ptr);
  if (err != ERR_OK)
    {
      printf("Error: Manager::createStaticPools() failure. %d\n", err);
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

static bool cleanup_audio_parts(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools(SECTION_NO0);

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

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

bool start_recording(CMN_SimpleFifoHandle *handle)
{
  CMN_SimpleFifoClear(handle);

  AudioCommand command;
  command.header.packet_length = LENGTH_START_RECORDER;
  command.header.command_code  = AUDCMD_STARTREC;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

bool stop_recording(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STOP_RECORDER;
  command.header.command_code  = AUDCMD_STOPREC;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

bool initailze_audio_captureing(
    CMN_SimpleFifoHandle *fifo_handle,
    annotation_cb_t anno_cb, fifo_fill_cb_t fill_cb,
    uint32_t sampling_rate, uint32_t chnum, uint32_t sample_per_bits)
{

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!prepare_audio_parts())
    {
      printf("Error: init_libraries() failure.\n");
      return false;
    }

  /* Next, Create the features used by AudioSubSystem. */

  if (!init_captureing(anno_cb))
    {
      printf("Error: act_audiosubsystem() failure.\n");
      cleanup_audio_parts();
      return false;
    }

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!audio_power_on())
    {
      printf("Error: audio_power_on() failure.\n");
      deactivate_audio();
      cleanup_audio_parts();
      return false;
    }

  /* Initialize simple fifo. */

  if (!init_fifo_info(fifo_handle, &s_output_device, fill_cb))
    {
      printf("Error: init_fifo_info() failure.\n");
      audio_power_off();
      deactivate_audio();
      cleanup_audio_parts();
      return false;
    }

  /* Set the initial gain of the microphone to be used. */
  /* Parameter can be set from -7850 to 210 as from -78.5db to 21.0db */

  if (!set_mic_gain(0))
    {
      printf("Error: set_mic_gain() failure.\n");
      audio_power_off();
      deactivate_audio();
      cleanup_audio_parts();
      return false;
    }

  /* Set audio clock mode. */

  if (!set_audio_clkmode(sampling_rate))
    {
      printf("Error: set_audio_clkmode() failure.\n");
      audio_power_off();
      deactivate_audio();
      cleanup_audio_parts();
      return false;
    }

  /* Set recorder operation mode. */

  if (!audio_set_output(&s_output_device))
    {
      printf("Error: audio_set_output() failure.\n");
      audio_power_off();
      deactivate_audio();
      cleanup_audio_parts();
      return false;
    }

  /* Initialize recorder. */

  if (!set_recording_mode(sampling_rate, chnum, sample_per_bits))
    {
      printf("Error: set_recording_mode() failure.\n");
      standby_audio();
      audio_power_off();
      deactivate_audio();
      cleanup_audio_parts();
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ()
 *
 * Description:
 *
 ****************************************************************************/

void finalize_audio_capturing(void)
{
  if (!standby_audio())
    {
      printf("Error: standby_audio() failure.\n");

      /* To end processing */
    }

  /* Change AudioSubsystem to PowerOff state. */

  if (!audio_power_off())
    {
      printf("Error: audio_power_off() failure.\n");

      /* To end processing */
    }

  /* Deactivate the features used by AudioSubSystem. */

  deactivate_audio();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!cleanup_audio_parts())
    {
      printf("Error: cleanup_audio_parts() failure.\n");
    }
}

