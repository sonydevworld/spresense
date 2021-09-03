/****************************************************************************
 * examples/audio_recorder/audio_util.cxx
 *
 *   Copyright 2019, 2021 Sony Semiconductor Solutions Corporation
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

#include <memutils/memory_manager/MemHandle.h>
#include <memutils/message/Message.h>
#include <audio/audio_high_level_api.h>
#include <audio/utilities/frame_samples.h>

#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"

#include "audio_util.h"

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

/* Section number of memory layout to use */

#define AUDIO_SECTION   SECTION_NO0

#define DSPBIN_PATH   "/mnt/sd0/BIN"

#define MIC_GAIN  0

/****************************************************************************
 * Private Data
 ****************************************************************************/

static CMN_SimpleFifoHandle fifo_handle;
static AsRecorderOutputDeviceHdlr  output_device_handle;

static mpshm_t s_shm;
static void * write_buf;
static size_t frame_size;

/* For WAV recording. */

static WavContainerFormat * s_container_format = NULL;
static WAVHEADER  s_wav_header;

static uint32_t es_size;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool init_memory_pools(void);
static bool set_memory_layout(int);
static bool fin_memory_pools(void);
static bool init_fifo_info(void *, size_t, fifo_fill_cb_t);
static bool printAudCmdResult(uint8_t, AudioResult &);
static bool create_audio_manager(attention_cb_t);
static bool delete_audio_manager(void);
static bool create_audio_recorder(void);
static void delete_audio_recorder(void);
static bool audio_power_on(void);
static bool audio_power_off(void);
static bool set_audio_clkmode(int);
static bool write_one_frame(FILE *, uint32_t);

#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC
static bool init_audio_preproc(void);
static bool set_audio_preproc(void);
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_memory_pools()
 *
 * Description:
 *     Initialize memory & message libraries for audio sub-system.
 ****************************************************************************/

static bool init_memory_pools(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;

  /* Initialize shared memory. */

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

  void *mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
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

  return true;
}

/****************************************************************************
 * Name: set_memory_layout()
 *
 * Description:
 *    Set a memory layout.
 ****************************************************************************/

static bool set_memory_layout(int no)
{
  /* Create static memory pool. */

  const uint8_t sec_no = AUDIO_SECTION;
  const NumLayout layout_no = no;
  void *work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[AUDIO_SECTION][layout_no][0];

  err_t err = Manager::createStaticPools(sec_no,
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
 * Name: fin_memory_pools()
 *
 * Description:
 *     Destory memory & message libraries for audio sub-system.
 ****************************************************************************/

static bool fin_memory_pools(void)
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
 * Name: init_fifo_info(void* fifo_adr, size_t fifo_size, fifo_fill_cb_t cb)
 *
 * Description:
 *     Initialze Audio FIFO with a callback.
 ****************************************************************************/

static bool init_fifo_info(void *fifo_adr, size_t fifo_size, fifo_fill_cb_t cb)
{
  if (CMN_SimpleFifoInitialize(&fifo_handle,
                               fifo_adr, fifo_size, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }

  CMN_SimpleFifoClear(&fifo_handle);

  output_device_handle.simple_fifo_handler = (void *)(&fifo_handle);
  output_device_handle.callback_function = cb;

  return true;
}

/****************************************************************************
 * Name: printAudCmdResult()
 *
 * Description:
 *     Check result packet from Audio sub-system.
 *     If any error is happened, display the details.
 ****************************************************************************/

static bool printAudCmdResult(uint8_t command_code, AudioResult &result)
{
  if (AUDRLT_ERRORRESPONSE == result.header.result_code)
    {
      printf("Command code(0x%x): AUDRLT_ERRORRESPONSE:"
             "Module id(0x%x): Error code(0x%lx)\n",
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

/****************************************************************************
 * Name: create_audio_manager(attention_cb_t att_cb)
 *
 * Description:
 *     Create audio manager for the high level API.
 ****************************************************************************/

static bool create_audio_manager(attention_cb_t cb)
{
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

  return (AS_CreateAudioManager(ids, cb) == AS_ERR_CODE_OK);
}

/****************************************************************************
 * Name: delete_audio_manager()
 *
 * Description:
 *     Delete audio manager for the high level API.
 ****************************************************************************/

static bool delete_audio_manager(void)
{
  return (AS_DeleteAudioManager() == AS_ERR_CODE_OK);
}

/****************************************************************************
 * Name: create_audio_recorder()
 *
 * Description:
 *     Create audio objects for audio recording.
 ****************************************************************************/

static bool create_audio_recorder(void)
{
  bool result = false;

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MGR;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_INPUT_BUF_POOL;
#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC
  frontend_create_param.pool_id.output      = S0_PREPROC_BUF_POOL;
#else
  frontend_create_param.pool_id.output      = S0_NULL_POOL;
#endif
  frontend_create_param.pool_id.dsp         = S0_PRE_APU_CMD_POOL;

  result = AS_CreateMicFrontend(&frontend_create_param, NULL);
  if (!result)
    {
      printf("Error:  AS_CreateMicFrontend() failure. system memory insufficient!\n");
      return false;
    }

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
 * Name: delete_audio_recorder()
 *
 * Description:
 *     Delete audio objects for audio recording.
 ****************************************************************************/

static void delete_audio_recorder(void)
{
  AS_DeleteMediaRecorder();
  AS_DeleteMicFrontend();
  AS_DeleteCapture();

  return;
}

/****************************************************************************
 * Name: audio_power_on()
 *
 * Description:
 *     Power on the audio HW.
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
 * Name: audio_power_off()
 *
 * Description:
 *     Power off the audio HW.
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
 * Name: set_clkmode()
 *
 * Description:
 *     Set capture clock of the audio HW.
 ****************************************************************************/

static bool set_audio_clkmode(int sampling_rate)
{
  int clk_mode = (sampling_rate == AS_SAMPLINGRATE_192000) ? AS_CLKMODE_HIRES : AS_CLKMODE_NORMAL;

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

/****************************************************************************
 * Name: write_one_frame()
 *
 * Description:
 *     Write the sound date to file.
 ****************************************************************************/

static bool write_one_frame(FILE *fd, uint32_t size)
{
  ssize_t ret;

  if (size == 0 || CMN_SimpleFifoGetOccupiedSize(&fifo_handle) == 0)
    {
      return true;
    }

  if (CMN_SimpleFifoPoll(&fifo_handle, write_buf, size) == 0)
    {
      printf("ERROR: Fail to get data from simple FIFO.\n");
      return false;
    }

  ret = fwrite(write_buf, 1, size, fd);
  if (ret <= 0)
    {
      printf("ERROR: Cannot write recorded data to output file.\n");
      return false;
    }

  es_size += size;

  return true;
}

#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC
/****************************************************************************
 * Name: init_audio_preproc()
 *
 * Description:
 *     Initialze recorder pre-process with subcore.
 ****************************************************************************/

static bool init_audio_preproc(void)
{
  static InitParam s_initparam;

  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_PREPROCESS_DSP;
  command.header.command_code  = AUDCMD_INIT_PREPROCESS_DSP;
  command.header.sub_code      = 0x00;
  command.init_preproc_param.packet_addr = reinterpret_cast<uint8_t *>(&s_initparam);
  command.init_preproc_param.packet_size = sizeof(s_initparam);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

/****************************************************************************
 * Name: set_audio_preproc()
 *
 * Description:
 *     Initialze recorder pre-process with subcore.
 ****************************************************************************/

static bool set_audio_preproc(void)
{
  static SetParam s_setparam;

  s_setparam.enable = true;
  s_setparam.coef   = 99;

  AudioCommand command;
  command.header.packet_length = LENGTH_SET_PREPROCESS_DSP;
  command.header.command_code  = AUDCMD_SET_PREPROCESS_DSP;
  command.header.sub_code      = 0x00;
  command.set_preproc_param.packet_addr = reinterpret_cast<uint8_t *>(&s_setparam);
  command.set_preproc_param.packet_size = sizeof(s_setparam);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: create_high_level_audio()
 *
 * Description:
 *     Initialize audio sub-system for the high level API.
 ****************************************************************************/

bool create_high_level_audio(void *fifo_adr, size_t fifo_size, size_t fsize, void *buf_adr,
                             fifo_fill_cb_t fill_cb, attention_cb_t att_cb)
{

  if (init_memory_pools() == false)
    {
      printf("Error: init_memory_pools(). \n");
      return false;
    }

  if (!set_memory_layout(MEM_LAYOUT_RECORDER))
    {
      printf("Error: set_memory_layout(MEM_LAYOUT_RECORDER). \n");
      return false;
    }

  if (create_audio_manager(att_cb) == false)
    {
      printf("Error: create_audio_manager(). \n");
      fin_memory_pools();
      return false;
    }

  if (create_audio_recorder() == false)
    {
      delete_audio_manager();
      fin_memory_pools();
      return false;
    }

  if (init_fifo_info(fifo_adr, fifo_size, fill_cb) == false)
    {
      delete_audio_recorder();
      delete_audio_manager();
      fin_memory_pools();
      return false;
    }

  write_buf = buf_adr;
  frame_size = fsize;

  if (audio_power_on() == false)
    {
      delete_audio_recorder();
      delete_audio_manager();
      fin_memory_pools();
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: standby_audio()
 *
 * Description:
 *     Reset the audio sub-system to be ready.
 ****************************************************************************/

bool standby_audio(void)
{
  if (s_container_format != NULL)
    {
      delete s_container_format;
      s_container_format = NULL;
    }

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
 * Name: set_recording_mode()
 *
 * Description:
 *     Set recording mode with parameters of
 *     sampling rate, channel number and sample per bits.
 ****************************************************************************/

bool set_recording_mode(uint8_t codec_type, format_type_e format_type, uint32_t sampling_rate,
                        uint8_t sample_per_bits, uint8_t chnum, const char *dsp_name)
{
  AudioCommand command;
  AudioResult result;
  bool ret;

  /* Set clock mode. */

  ret = set_audio_clkmode(sampling_rate);
  if (!ret)
    {
      printf("Error: set_audio_clkmode() failure. %d\n", ret);
      return ret;
    }

  /* Change recording mode. */

  command.header.packet_length = LENGTH_SET_RECORDER_STATUS;
  command.header.command_code  = AUDCMD_SETRECORDERSTATUS;
  command.header.sub_code      = 0x00;
  command.set_recorder_status_param.input_device          = AS_SETRECDR_STS_INPUTDEVICE_MIC;
  command.set_recorder_status_param.input_device_handler  = 0x00;
  command.set_recorder_status_param.output_device         = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
  command.set_recorder_status_param.output_device_handler = &output_device_handle;
  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  ret = printAudCmdResult(command.header.command_code, result);
  if (!ret)
    {
      return ret;
    }

  /* Initialize Recorder. */

  command.header.packet_length = LENGTH_INIT_RECORDER;
  command.header.command_code  = AUDCMD_INITREC;
  command.header.sub_code      = 0x00;
  command.recorder.init_param.sampling_rate  = sampling_rate;
  command.recorder.init_param.channel_number = chnum;
  command.recorder.init_param.bit_length     = sample_per_bits;
  command.recorder.init_param.codec_type     = codec_type;
  snprintf(command.recorder.init_param.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", DSPBIN_PATH);

  switch (codec_type)
    {
      case AS_CODECTYPE_LPCM:
        break;
      case AS_CODECTYPE_MP3:
        command.recorder.init_param.bitrate    = AS_BITRATE_96000;
        break;
      case AS_CODECTYPE_OPUS:
        command.recorder.init_param.bitrate    = AS_BITRATE_8000;
        command.recorder.init_param.computational_complexity = AS_INITREC_COMPLEXITY_0;
        break;
      default:
        return false;
        break;
    }

  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  ret = printAudCmdResult(command.header.command_code, result);
  if (!ret)
    {
      standby_audio();
      return ret;
    }

  switch (format_type)
    {
      case FORMAT_TYPE_WAV:
        s_container_format = new WavContainerFormat();
        if (s_container_format == NULL) return false;
        s_container_format->init(FORMAT_ID_PCM, chnum, sampling_rate, sample_per_bits);
        break;
      case FORMAT_TYPE_RAW:
      default:
        break;
    }

  /* Initialize MicFrontend. */

  command.header.packet_length = LENGTH_INIT_MICFRONTEND;
  command.header.command_code  = AUDCMD_INIT_MICFRONTEND;
  command.header.sub_code      = 0x00;
  command.init_micfrontend_param.ch_num       = chnum;
  command.init_micfrontend_param.bit_length   = sample_per_bits;
  command.init_micfrontend_param.samples      = getCapSampleNumPerFrame(codec_type, sampling_rate);
#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC
  command.init_micfrontend_param.preproc_type = AsMicFrontendPreProcUserCustom;
#else /* CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC */
  command.init_micfrontend_param.preproc_type = AsMicFrontendPreProcThrough;
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC */
  snprintf(command.init_micfrontend_param.preprocess_dsp_path,
           AS_PREPROCESS_FILE_PATH_LEN,
           "%s/%s", DSPBIN_PATH, dsp_name);
  command.init_micfrontend_param.data_dest = AsMicFrontendDataToRecorder;
  AS_SendAudioCommand(&command);

  AS_ReceiveAudioResult(&result);
  ret = printAudCmdResult(command.header.command_code, result);
  if (!ret)
    {
      standby_audio();
      return ret;
    }

#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC
  if (!init_audio_preproc())
    {
      printf("Error: init_audio_preproc() failure.\n");
      standby_audio();
      return false;
    }

  if (!set_audio_preproc())
    {
      printf("Error: set_audio_preproc() failure.\n");
      standby_audio();
      return false;
    }
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC */

  return true;
}

/****************************************************************************
 * Description:
 *     Start audio sub-system for recording.
 ****************************************************************************/

bool start_recording(void)
{
  CMN_SimpleFifoClear(&fifo_handle);

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
 * Name: stop_recording()
 *
 * Description:
 *     Stop audio sub-system.
 ****************************************************************************/

void stop_recording(FILE *fd)
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
      return;
    }

  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&fifo_handle);

  while (occupied_simple_fifo_size > 0)
    {
      size_t output_size = (occupied_simple_fifo_size > frame_size) ?
        frame_size : occupied_simple_fifo_size;
      write_one_frame(fd, output_size);
      occupied_simple_fifo_size =
        CMN_SimpleFifoGetOccupiedSize(&fifo_handle);
    }
}

/****************************************************************************
 * Name: delete_high_level_audio()
 *
 * Description:
 *     Shut down audio sub-system.
 ****************************************************************************/

bool delete_high_level_audio(void)
{
  /* Change AudioSubsystem to PowerOff state. */

  if (!audio_power_off())
    {
      printf("Error: audio_power_off() failure.\n");

      /* To end processing */
    }

  delete_audio_recorder();

  if (!delete_audio_manager())
    {
      printf("Error: delete_audio_manager() failure.\n");

      /* To end processing */
    }

  if (!fin_memory_pools())
    {
      printf("Error: fin_memory_pools() failure.\n");

      /* To end processing */
    }

  return true;
}

/****************************************************************************
 * Name: set_mic_gain()
 *
 * Description:
 *     Set mic gain.
 *     gain value can be set in -7850 to 210 as -78.5db to 21.0eb
 ****************************************************************************/

bool set_mic_gain(int16_t gain)
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
 * Name: create_wav_header(FILE *fd)
 *
 * Description:
 *    Create the wav header.
 ****************************************************************************/

bool create_wav_header(FILE *fd)
{
  es_size = 0;
  s_container_format->getHeader(&s_wav_header, 0);

  size_t ret = fwrite((const void *)&s_wav_header,
                      1,
                      sizeof(WAVHEADER),
                      fd);
  if (ret != sizeof(WAVHEADER))
    {
      printf("Fail to write file(wav header)\n");
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: update_wav_file_size(FILE *fd)
 *
 * Description:
 *     Write file size on the wav header.
 ****************************************************************************/

bool update_wav_file_size(FILE *fd)
{
  fseek(fd, 0, SEEK_SET);

  s_container_format->getHeader(&s_wav_header, es_size);

  size_t ret = fwrite((const void *)&s_wav_header,
                      1,
                      sizeof(WAVHEADER),
                      fd);
  if (ret != sizeof(WAVHEADER))
    {
      printf("Fail to write file(wav header)\n");
      return false;
    }

  return true;
}

/****************************************************************************
 * Name:write_frames(FILE* fd)
 *
 * Description:
 *     write data of some frames to the output file.
 ****************************************************************************/

bool write_frames(FILE *fd)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&fifo_handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > frame_size) ?
        frame_size : occupied_simple_fifo_size;
      if (!write_one_frame(fd, output_size)) return false;
      occupied_simple_fifo_size -= output_size;
    }

  return true;
}
