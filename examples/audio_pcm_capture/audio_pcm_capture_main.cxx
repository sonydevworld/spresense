/****************************************************************************
 * audio_pcm_capture/pcm_capture_main.cxx
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

#include <nuttx/config.h>
#include <stdio.h>
#include <strings.h>
#include <asmp/mpshm.h>
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_high_level_api.h"
#include "audio/utilities/frame_samples.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"

/****************************************************************************
 * Codec parameters
 ****************************************************************************/

/* Recording default time(sec). */

#define PCM_CAPTURE_TIME     (10)

/* Sampling rate
 * 44.1kHz : AS_SAMPLINGRATE_44100
 * 48kHz   : AS_SAMPLINGRATE_48000
 * 192kHz  : AS_SAMPLINGRATE_192000
 */

#define SAMPLINGRATE          AS_SAMPLINGRATE_48000

/* Channel number
 * MONO (1ch)   : AS_CHANNEL_MONO
 * STEREO (2ch) : AS_CHANNEL_STEREO
 * 4ch          : AS_CHANNEL_4CH
 */

#define CHANNEL_NUMBER        AS_CHANNEL_4CH

/* Bit length
 * 16bit : AS_BITLENGTH_16
 * 24bit : AS_BITLENGTH_24
 */

#define BIT_LENGTH            AS_BITLENGTH_16

/* Use microphone channel number.
 *   [Analog microphone]
 *       Maximum number: 4
 *       The channel number is 1/2/4
 *   [Digital microphone]
 *       Maximum number: 8
 *       The channel number is 1/2/4/6/8
 */

#define USE_MIC_CHANNEL_NUM   (4)

/* Set the PCM buffer size. */

#define READ_SIMPLE_FIFO_SIZE (768 * USE_MIC_CHANNEL_NUM * 2)  /* 768sample, 4ch, 16bit */
#define SIMPLE_FIFO_FRAME_NUM (60)

/****************************************************************************
 * PCM capture function
 ****************************************************************************/

/*
 * Audio signal process (Modify for your application)
 *
 * [in] buf   capture data
 * [in] size  capture size
 *
 */

static void app_pcm_output(uint8_t *buf, uint32_t size)
{
  /* Sample code.
   * Please modify according to the application.
   * For example, output capture data.
   */

  printf("Size %d [%02x %02x %02x %02x ...]\n",
         size,
         buf[0],
         buf[1],
         buf[2],
         buf[3]);
}

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

using namespace MemMgrLite;

/* PCM codec search path. */

#define DSPBIN_PATH   "/mnt/sd0/BIN"

/* Section number of memory layout to use */

#define AUDIO_SECTION SECTION_NO0

/* PCM FIFO buffer size */

#define SIMPLE_FIFO_BUF_SIZE  (READ_SIMPLE_FIFO_SIZE * SIMPLE_FIFO_FRAME_NUM)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* For FIFO. */

struct fifo_info_s
{
  CMN_SimpleFifoHandle        handle;
  AsRecorderOutputDeviceHdlr  output_device;
  uint32_t fifo_area[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
  uint8_t  write_buf[READ_SIMPLE_FIFO_SIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For FIFO. */

static fifo_info_s s_fifo;

/* For share memory. */

static mpshm_t s_shm;

/* For target codec parameters. */

static uint32_t  target_samplingrate   = SAMPLINGRATE;
static uint32_t  target_channel_number = CHANNEL_NUMBER;
static uint32_t  target_bit_lengt      = BIT_LENGTH;
static uint32_t  target_recording_time = PCM_CAPTURE_TIME;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void outputDeviceCallback(uint32_t size)
{
    /* do nothing */
}

static bool app_init_simple_fifo(void)
{
  if (CMN_SimpleFifoInitialize(&s_fifo.handle,
                               s_fifo.fifo_area,
                               SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(&s_fifo.handle);

  s_fifo.output_device.simple_fifo_handler =
    (void*)(&s_fifo.handle);
  s_fifo.output_device.callback_function = outputDeviceCallback;

  return true;
}

static void app_pop_simple_fifo(bool is_end_process = false)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&s_fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > READ_SIMPLE_FIFO_SIZE) ?
        READ_SIMPLE_FIFO_SIZE : occupied_simple_fifo_size;
      if (CMN_SimpleFifoPoll(&s_fifo.handle,
                            (void*)s_fifo.write_buf,
                            output_size) == 0)
        {
          printf("ERROR: Fail to get data from simple FIFO.\n");
          break;
        }

      /* Data output */

      app_pcm_output((uint8_t*)s_fifo.write_buf, output_size);

      if (is_end_process)
        {
          /* At end processing, all remaining data is spit */

          occupied_simple_fifo_size = CMN_SimpleFifoGetOccupiedSize(&s_fifo.handle);
        }
      else
        {
          occupied_simple_fifo_size -= output_size;
        }
    }
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
  ids.micfrontend = MSGQ_AUD_FRONTEND;
  ids.mixer       = 0xFF;
  ids.recorder    = MSGQ_AUD_RECORDER;
  ids.effector    = 0xFF;
  ids.recognizer  = 0xFF;

  AS_CreateAudioManager(ids, app_attention_callback);

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

static void app_deact_audio_sub_system(void)
{
  AS_DeleteAudioManager();
  AS_DeleteMediaRecorder();
  AS_DeleteMicFrontend();
  AS_DeleteCapture();
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

static bool app_init_mic_gain(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITMICGAIN;
  command.header.command_code  = AUDCMD_INITMICGAIN;
  command.header.sub_code      = 0;
  command.init_mic_gain_param.mic_gain[0] = 0;
  command.init_mic_gain_param.mic_gain[1] = 0;
  command.init_mic_gain_param.mic_gain[2] = 0;
  command.init_mic_gain_param.mic_gain[3] = 0;
  command.init_mic_gain_param.mic_gain[4] = 0;
  command.init_mic_gain_param.mic_gain[5] = 0;
  command.init_mic_gain_param.mic_gain[6] = 0;
  command.init_mic_gain_param.mic_gain[7] = 0;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_recorder_status(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_RECORDER_STATUS;
  command.header.command_code  = AUDCMD_SETRECORDERSTATUS;
  command.header.sub_code      = 0x00;
  command.set_recorder_status_param.input_device          = AS_SETRECDR_STS_INPUTDEVICE_MIC;
  command.set_recorder_status_param.input_device_handler  = 0x00;
  command.set_recorder_status_param.output_device         = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
  command.set_recorder_status_param.output_device_handler = &s_fifo.output_device;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_micfrontend(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_MICFRONTEND;
  command.header.command_code  = AUDCMD_INIT_MICFRONTEND;
  command.header.sub_code      = 0x00;
  command.init_micfrontend_param.ch_num       = target_channel_number;
  command.init_micfrontend_param.bit_length   = target_bit_lengt;
  command.init_micfrontend_param.samples      = getCapSampleNumPerFrame(AS_CODECTYPE_LPCM,
                                                                        target_samplingrate);
  command.init_micfrontend_param.preproc_type = AsMicFrontendPreProcThrough;
  snprintf(command.init_micfrontend_param.preprocess_dsp_path,
           AS_PREPROCESS_FILE_PATH_LEN,
           "%s", "/mnt/sd0/BIN/PREPROC");
  command.init_micfrontend_param.data_dest    = AsMicFrontendDataToRecorder;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_recorder(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_RECORDER;
  command.header.command_code  = AUDCMD_INITREC;
  command.header.sub_code      = 0x00;

  command.recorder.init_param.sampling_rate  = target_samplingrate;
  command.recorder.init_param.channel_number = target_channel_number;
  command.recorder.init_param.bit_length     = target_bit_lengt;
  command.recorder.init_param.codec_type     = AS_CODECTYPE_LPCM;

  snprintf(command.recorder.init_param.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", DSPBIN_PATH);

  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_start_recorder(void)
{
  CMN_SimpleFifoClear(&s_fifo.handle);

  AudioCommand command;
  command.header.packet_length = LENGTH_START_RECORDER;
  command.header.command_code  = AUDCMD_STARTREC;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_stop_recorder(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STOP_RECORDER;
  command.header.command_code  = AUDCMD_STOPREC;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  if (!printAudCmdResult(command.header.command_code, result))
    {
      return false;
    }

  /* Spout processing of remaining data. */

  app_pop_simple_fifo(true);

  return true;
}

static bool app_set_clkmode(void)
{
  cxd56_audio_clkmode_t clk_mode;

  if (target_samplingrate == AS_SAMPLINGRATE_192000)
    {
      clk_mode = CXD56_AUDIO_CLKMODE_HIRES;
    }
  else
    {
      clk_mode = CXD56_AUDIO_CLKMODE_NORMAL;
    }

  AudioCommand command;
  command.header.packet_length = LENGTH_SETRENDERINGCLK;
  command.header.command_code  = AUDCMD_SETRENDERINGCLK;
  command.header.sub_code      = 0x00;
  command.set_renderingclk_param.clk_mode = clk_mode;
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

  const uint8_t sec_no = AUDIO_SECTION;
  const NumLayout layout_no = MEM_LAYOUT_PCM_CAPTURE;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[AUDIO_SECTION][layout_no][0];
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

static bool app_finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools(AUDIO_SECTION);

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

void app_recorde_process(uint32_t rec_time)
{
  /* Timer Start */

  time_t start_time;
  time_t cur_time;

  time(&start_time);

  do
    {
      app_pop_simple_fifo();

    } while((time(&cur_time) - start_time) < rec_time);
}

static bool app_argparse(int argc, char *argv[])
{
  struct {
    const char *target;
    uint32_t   *param;
    uint32_t    target_param;
    const char *help;
  }
  nodes[] = {
    {"48k",  &target_samplingrate,   AS_SAMPLINGRATE_48000,  "Sampling rate   48kHz"},
    {"192k", &target_samplingrate,   AS_SAMPLINGRATE_192000, "Sampling rate  192kHz"},
    {"1ch",  &target_channel_number, AS_CHANNEL_MONO,        "Channel number 1ch(MONO)"},
    {"2ch",  &target_channel_number, AS_CHANNEL_STEREO,      "Channel number 2ch(STEREO)"},
    {"4ch",  &target_channel_number, AS_CHANNEL_4CH,         "Channel number 4ch"},
    {"16b",  &target_bit_lengt,      AS_BITLENGTH_16,        "Bit length     16bit"},
    {"24b",  &target_bit_lengt,      AS_BITLENGTH_24,        "Bit length     24bit"},
    {NULL,   NULL,                   0,                      NULL},
  },
  *p_node;

  /* Set default parameter. */

  target_samplingrate   = SAMPLINGRATE;
  target_channel_number = CHANNEL_NUMBER;
  target_bit_lengt      = BIT_LENGTH;
  target_recording_time = PCM_CAPTURE_TIME;

  if (argc < 2)
    {
      /* Default configuration */

      return true;
    }

  /* Help! Display Description */

  if (strcmp(argv[1], "help") == 0)
    {
      printf("\n");

      for (p_node = nodes; p_node->target; p_node++)
        {
          printf("%-4s - %s\n", p_node->target, p_node->help);
        }
      printf("1... - recording time (sec)\n\n");

      return false;
    }

  /* Option determination */

  else
    {
      for (int i = 1; i < argc; i++)
        {
          for (p_node = nodes; p_node->target; p_node++)
            {
              if (strncasecmp(p_node->target, argv[i], strlen(p_node->target)) == 0)
                {
                  if (p_node->param)
                    {
                      *p_node->param = p_node->target_param;
                    }
                  break;
                }
            }

          /* When there is no option entered. */

          if (p_node->target == NULL)
            {
              /* Numerical judgment */

              int   time = atoi(argv[i]);

              if (time > 0)
                {
                  target_recording_time = time;
                }
              else
                {
                  printf("\nInvalid option '%s'\n", argv[i]);
                  printf("Try '%s help' for more information.\n\n", argv[0]);
                }
            }
        }
    }

  return true;
}

static const char *app_param_to_str(int id, uint32_t param)
{
  struct {
    int         id;
    uint32_t    param;
    const char *title;
  }
  nodes[] = {
    {-1, AS_SAMPLINGRATE_48000,  "48kHz"},
    {-1, AS_SAMPLINGRATE_192000, "192kHz"},
    {-2, AS_CHANNEL_MONO,        "1ch(MONO)"},
    {-2, AS_CHANNEL_STEREO,      "2ch(STEREO)"},
    {-2, AS_CHANNEL_4CH,         "4ch"},
    {-3, AS_BITLENGTH_16,        "16bit"},
    {-3, AS_BITLENGTH_24,        "24bit"},
    { 0, 0,                      "!!! Undefined value !!!"},
  },
  *p_node;

  for (p_node = nodes; p_node->id; p_node++)
    {
      if (p_node->id == id && p_node->param == param)
        {
          break;
        }
    }

  return p_node->title;
}

static void app_disp_codec_params(void)
{
  printf("\n");
  printf("Sampling rate  %s\n", app_param_to_str(-1, target_samplingrate));
  printf("Channel number %s\n", app_param_to_str(-2, target_channel_number));
  printf("Bit length     %s\n", app_param_to_str(-3, target_bit_lengt));
  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  /* Command line argument analysis */

  if (!app_argparse(argc, argv))
    {
      goto errout;
    }

  printf("Start PCM capture example\n");

  /* Show codec parameters */

  app_disp_codec_params();

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
      goto errout;
    }

  /* Next, Create the features used by AudioSubSystem. */

  if (!app_create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");
      goto errout_create_audio_sub_system;
    }

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_power_on())
    {
      printf("Error: app_power_on() failure.\n");
      goto errout_power_on;
    }

  /* Initialize simple fifo. */

  if (!app_init_simple_fifo())
    {
      printf("Error: app_init_simple_fifo() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Set the initial gain of the microphone to be used. */

  if (!app_init_mic_gain())
    {
      printf("Error: app_init_mic_gain() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Set audio clock mode. */

  if (!app_set_clkmode())
    {
      printf("Error: app_set_clkmode() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Set recorder operation mode. */

  if (!app_set_recorder_status())
    {
      printf("Error: app_set_recorder_status() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Initialize recorder. */

  if (!app_init_recorder())
    {
      printf("Error: app_init_recorder() failure.\n");
      goto errout_init_recorder;
    }

  /* Initialize MicFrontend. */

  if (!app_init_micfrontend())
    {
      printf("Error: app_init_micfrontend() failure.\n");
      goto errout_init_recorder;
    }

  /* Start recorder operation. */

  if (!app_start_recorder())
    {
      printf("Error: app_start_recorder() failure.\n");
    }
  else
    {
      /* Running... */

      printf("Running time is %d sec\n", target_recording_time);

      app_recorde_process(target_recording_time);

      /* Stop recorder operation. */

      if (!app_stop_recorder())
        {
          printf("Error: app_stop_recorder() failure.\n");

          /* To end processing */
        }
    }

errout_init_recorder:

  /* Return the state of AudioSubSystem before voice_call operation. */

  if (!app_set_ready())
    {
      printf("Error: app_set_ready() failure.\n");

      /* To end processing */
    }

errout_init_simple_fifo:

  /* Change AudioSubsystem to PowerOff state. */

  if (!app_power_off())
    {
      printf("Error: app_power_off() failure.\n");

      /* To end processing */
    }

errout_power_on:

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

errout_create_audio_sub_system:

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
    }

errout:

  printf("Exit PCM capture example\n");

  return 0;
}
