/****************************************************************************
 * audio_recorder/audio_recorder_objif_main.cxx
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
#include <dirent.h>
#include <strings.h>
#include <asmp/mpshm.h>
#include <sys/stat.h>
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_recorder_api.h"
#include "audio/audio_frontend_api.h"
#include "audio/audio_capture_api.h"
#include "audio/audio_message_types.h"
#include "audio/utilities/frame_samples.h"
#include "audio/utilities/wav_containerformat.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"
#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC
#include "userproc_command.h"
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC */

using namespace MemMgrLite;

/****************************************************************************
 * Default codec parameters
 ****************************************************************************/

/* Codec type
 * MP3  : AS_CODECTYPE_MP3
 * LPCM : AS_CODECTYPE_LPCM
 */

#define CODEC_TYPE      AS_CODECTYPE_LPCM

/* Sampling rate
 * 48 kHz   : AS_SAMPLINGRATE_48000
 * 192kHz   : AS_SAMPLINGRATE_192000
 */

#define SAMPLINGRATE    AS_SAMPLINGRATE_48000

/* Channel number
 * MONO (1ch)   : AS_CHANNEL_MONO
 * STEREO (2ch) : AS_CHANNEL_STEREO
 * 4ch          : AS_CHANNEL_4CH
 */

#define CHANNEL_NUMBER  AS_CHANNEL_STEREO

/* Bit length
 * 16bit : AS_BITLENGTH_16
 * 24bit : AS_BITLENGTH_24
 */

#define BIT_LENGTH      AS_BITLENGTH_16

/* Use microphone channel number.
 *   [Analog microphone]
 *       Maximum number: 4
 *       The channel number is 1/2/4
 *   [Digital microphone]
 *       Maximum number: 8
 *       The channel number is 1/2/4/6/8
 */

#define USE_MIC_CHANNEL_NUM  2

/* Recording time(sec). */

#define RECORDER_REC_TIME 10

/* For FIFO. */

#define READ_SIMPLE_FIFO_SIZE (3072 * USE_MIC_CHANNEL_NUM)
#define SIMPLE_FIFO_FRAME_NUM 60
#define SIMPLE_FIFO_BUF_SIZE  (READ_SIMPLE_FIFO_SIZE * SIMPLE_FIFO_FRAME_NUM)

/* setvbuf function buffer size.
 * Improved writing speed of fwrite function.
 */

#define STDIO_BUFFER_SIZE 4096

/* Length of recording file name */

#define MAX_PATH_LENGTH 128
#define MAX_EXT_LENGTH    8

/* Recording file path. */

#define RECFILE_ROOTPATH "/mnt/sd0/REC"

/* PCM codec search path. */

#define DSPBIN_PATH "/mnt/sd0/BIN"

/* Number of retries due to incomplete SD card mounting. */

#define NUMBER_OF_RETRY   3

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* For FIFO */

struct recorder_fifo_info_s
{
  CMN_SimpleFifoHandle        handle;
  AsRecorderOutputDeviceHdlr  output_device;
  uint32_t fifo_area[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
  uint8_t  write_buf[READ_SIMPLE_FIFO_SIZE];
};

struct recorder_file_info_s
{
  uint32_t  size;
  DIR      *dirp;
  FILE     *fd;
};

struct recorder_info_s
{
  struct recorder_fifo_info_s  fifo;
  struct recorder_file_info_s  file;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static recorder_info_s s_recorder_info;

/* For share memory. */

static mpshm_t s_shm;

static WavContainerFormat* s_container_format = NULL;
static WAVHEADER  s_wav_header;

/* For target codec parameters. */

static uint32_t  target_codec_type     = CODEC_TYPE;
static uint32_t  target_samplingrate   = SAMPLINGRATE;
static uint32_t  target_channel_number = CHANNEL_NUMBER;
static uint32_t  target_bit_length     = BIT_LENGTH;
static uint32_t  target_recording_time = RECORDER_REC_TIME;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void outputDeviceCallback(uint32_t size)
{
    /* do nothing */
}

static bool app_update_wav_file_size(void)
{
  fseek(s_recorder_info.file.fd, 0, SEEK_SET);

  s_container_format->getHeader(&s_wav_header, s_recorder_info.file.size);

  size_t ret = fwrite((const void *)&s_wav_header,
                      1,
                      sizeof(WAVHEADER),
                      s_recorder_info.file.fd);
  if (ret != sizeof(WAVHEADER))
    {
      printf("Fail to write file(wav header)\n");
      return false;
    }
  return true;
}

static bool app_write_wav_header(void)
{
  bool  res = false;

  if (s_container_format->init(FORMAT_ID_PCM,
                                  target_channel_number,
                                  target_samplingrate,
                                  target_bit_length))
    {
      s_container_format->getHeader(&s_wav_header, 0);

      size_t size = fwrite((const void *)&s_wav_header,
                          1,
                          sizeof(WAVHEADER),
                          s_recorder_info.file.fd);

      if (size != sizeof(WAVHEADER))
        {
          printf("Fail to write file(wav header)\n");
        }
      else
        {
          res = true;
        }
    }

  return res;
}

static bool app_open_output_file(void)
{
  static char fname[MAX_PATH_LENGTH];

  struct tm        *cur_time;
  struct timespec   cur_sec;
  char              ext[MAX_EXT_LENGTH];

  clock_gettime(CLOCK_REALTIME, &cur_sec);
  cur_time = gmtime(&cur_sec.tv_sec);

  if (target_codec_type == AS_CODECTYPE_MP3)
    {
      strcpy(ext, "mp3");
    }
  else if (target_codec_type == AS_CODECTYPE_LPCM)
    {
      strcpy(ext, "wav");
    }
  else if (target_codec_type == AS_CODECTYPE_OPUS)
    {
      strcpy(ext, "raw");
    }
  else
    {
      printf("Unsupported format\n");
      return false;
    }

  snprintf(fname,
           MAX_PATH_LENGTH,
           "%s/%04d%02d%02d_%02d%02d%02d.%s",
           RECFILE_ROOTPATH,
           cur_time->tm_year,
           cur_time->tm_mon,
           cur_time->tm_mday,
           cur_time->tm_hour,
           cur_time->tm_min,
           cur_time->tm_sec,
           ext);

  /* Create a file with the name created above */

  for (int i = 0; i < NUMBER_OF_RETRY; i++)
    {
      s_recorder_info.file.fd = fopen(fname, "w");

      if(s_recorder_info.file.fd)
        {
          break;
        }

      /* Retry due to incomplete SD card mounting. */

      usleep(100 * 1000);
    }

  if (s_recorder_info.file.fd == 0)
    {
      printf("open err(%s)\n", fname);
      return false;
    }

  setvbuf(s_recorder_info.file.fd, NULL, _IOLBF, STDIO_BUFFER_SIZE);

  printf("Record data to %s.\n", &fname[0]);

  if (target_codec_type == AS_CODECTYPE_LPCM)
    {
      if (!app_write_wav_header())
        {
          printf("Error: app_write_wav_header() failure.\n");
          return false;
        }
    }

  return true;
}

static void app_close_output_file(void)
{
  fclose(s_recorder_info.file.fd);
}

static void app_write_output_file(uint32_t size)
{
  ssize_t ret;

  if (size == 0 || CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle) == 0)
    {
      return;
    }

  if (CMN_SimpleFifoPoll(&s_recorder_info.fifo.handle,
                        (void*)s_recorder_info.fifo.write_buf,
                        size) == 0)
    {
      printf("ERROR: Fail to get data from simple FIFO.\n");
      return;
    }

  ret = fwrite(s_recorder_info.fifo.write_buf, 1, size, s_recorder_info.file.fd);
  if (ret <= 0) {
    printf("ERROR: Cannot write recorded data to output file.\n");
    app_close_output_file();
    return;
  }
  s_recorder_info.file.size += size;
}

static bool app_init_simple_fifo(void)
{
  if (CMN_SimpleFifoInitialize(&s_recorder_info.fifo.handle,
                               s_recorder_info.fifo.fifo_area,
                               SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(&s_recorder_info.fifo.handle);

  s_recorder_info.fifo.output_device.simple_fifo_handler =
    (void*)(&s_recorder_info.fifo.handle);
  s_recorder_info.fifo.output_device.callback_function = outputDeviceCallback;

  return true;
}

static void app_pop_simple_fifo(void)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > READ_SIMPLE_FIFO_SIZE) ?
        READ_SIMPLE_FIFO_SIZE : occupied_simple_fifo_size;
      app_write_output_file(output_size);
      occupied_simple_fifo_size -= output_size;
    }
}

static bool app_create_audio_sub_system(void)
{
  bool result = false;

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MGR;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_INPUT_BUF_POOL;
#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC
  frontend_create_param.pool_id.output      = S0_PREPROC_BUF_POOL;
#else
  frontend_create_param.pool_id.output      = S0_NULL_POOL;
#endif
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
  AS_DeleteMediaRecorder();
  AS_DeleteMicFrontend();
  AS_DeleteCapture();
}

static bool app_receive_object_reply(uint32_t id = 0)
{
  AudioObjReply reply_info;
  AS_ReceiveObjectReply(MSGQ_AUD_MGR, &reply_info);

  if (reply_info.type != AS_OBJ_REPLY_TYPE_REQ)
    {
      printf("app_receive_object_reply() error! type 0x%x\n",
             reply_info.type);
      return false;
    }

  if (id && reply_info.id != id)
    {
      printf("app_receive_object_reply() error! id 0x%x(request id 0x%x)\n",
             reply_info.id, id);
      return false;
    }

  if (reply_info.result != OK)
    {
      printf("app_receive_object_reply() error! result 0x%x\n",
             reply_info.result);
      return false;
    }

  return true;
}

static bool app_power_on(void)
{
  return (cxd56_audio_poweron() == CXD56_AUDIO_ECODE_OK);
}

static bool app_power_off(void)
{
  return (cxd56_audio_poweroff() == CXD56_AUDIO_ECODE_OK);
}

static bool app_set_ready(void)
{
  AsDeactivateMicFrontendParam  deact_param;

  AS_DeactivateMicFrontend(&deact_param);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_DEACT))
    {
      /* To end processing */;
    }

  AS_DeactivateMediaRecorder();

  if (!app_receive_object_reply(MSG_AUD_MRC_CMD_DEACTIVATE))
    {
      /* To end processing */;
    }

  /* Disable input */

  if (cxd56_audio_dis_input() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  return true;
}

static bool app_init_mic_gain(int16_t gain)
{
  AsMicFrontendMicGainParam micgain_param;

  for (int i = 0; i < AS_MIC_CHANNEL_MAX; i++)
    {
      micgain_param.mic_gain[i] = gain;
    }

  bool result = AS_SetMicGainMicFrontend(&micgain_param);

  if (!result)
    {
      printf("Error: AS_SetMicGainMediaRecorder() failure!\n");
      return false;
    }

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_SETMICGAIN))
    {
      printf("Error Result: AS_SetMicGainMediaRecorder()\n");
      return false;
    }

  return true;
}

static bool app_set_recorder(void)
{
  /* Enable input. */

  if (cxd56_audio_en_input() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  {
    /* Set frontend recorder parameter */

    AsActivateMicFrontend act_param;

    act_param.param.input_device = AS_SETRECDR_STS_INPUTDEVICE_MIC;
    act_param.cb                 = NULL;

    AS_ActivateMicFrontend(&act_param);

    if (!app_receive_object_reply(MSG_AUD_MFE_CMD_ACT))
      {
        return false;
      }
  }

  {
    /* Set recorder parameter */

    AsActivateRecorder  act_param;

    act_param.param.input_device          = AS_SETRECDR_STS_INPUTDEVICE_MIC;
    act_param.param.input_device_handler  = 0x00;
    act_param.param.output_device         = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
    act_param.param.output_device_handler = &s_recorder_info.fifo.output_device;
    act_param.cb                          = NULL;

    AS_ActivateMediaRecorder(&act_param);

    if (!app_receive_object_reply(MSG_AUD_MRC_CMD_ACTIVATE))
      {
        return false;
      }
  }

  return true;
}

static bool app_init_recorder(void)
{
  {
    /* Set frontend init parameter */

    AsInitMicFrontendParam  init_param;

    init_param.channel_number    = target_channel_number;
    init_param.bit_length        = target_bit_length;
    init_param.samples_per_frame = getCapSampleNumPerFrame(
                                              target_codec_type,
                                              target_samplingrate);
    init_param.data_path         = AsDataPathMessage;
    init_param.dest.msg.msgqid   = MSGQ_AUD_RECORDER;
    init_param.dest.msg.msgtype  = MSG_AUD_MRC_CMD_ENCODE;
#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC
    init_param.preproc_type      = AsMicFrontendPreProcUserCustom;
    snprintf(init_param.dsp_path, AS_PREPROCESS_FILE_PATH_LEN, "/mnt/sd0/BIN/PREPROC");
#else
    init_param.preproc_type      = AsMicFrontendPreProcThrough;
#endif

    AS_InitMicFrontend(&init_param);

    if (!app_receive_object_reply(MSG_AUD_MFE_CMD_INIT))
      {
        return false;
      }
  }

  {
    /* Set recorder init parameter */

    AsInitRecorderParam   init_param;

    switch (target_codec_type)
      {
        case AS_CODECTYPE_LPCM:
          init_param.codec_type               = AS_CODECTYPE_LPCM;
          break;
        case AS_CODECTYPE_MP3:
          init_param.codec_type               = AS_CODECTYPE_MP3;
          init_param.bitrate                  = AS_BITRATE_96000;
          break;
        case AS_CODECTYPE_OPUS:
          init_param.codec_type               = AS_CODECTYPE_OPUS;
          init_param.bitrate                  = AS_BITRATE_8000;
          init_param.computational_complexity = AS_INITREC_COMPLEXITY_0;
          break;
        default:
          /* Already checked. Not here */
          break;
      }

    init_param.sampling_rate  = target_samplingrate;
    init_param.channel_number = target_channel_number;
    init_param.bit_length     = target_bit_length;

    snprintf(init_param.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", DSPBIN_PATH);

    AS_InitMediaRecorder(&init_param);

    if (!app_receive_object_reply(MSG_AUD_MRC_CMD_INIT))
      {
        return false;
      }
  }

  return true;
}

static bool app_start_recorder(void)
{
  s_recorder_info.file.size = 0;

  if (!app_open_output_file())
    {
      return false;
    }

  CMN_SimpleFifoClear(&s_recorder_info.fifo.handle);

  AsStartMicFrontendParam cmd;

  AS_StartMicFrontend(&cmd);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_START))
    {
      return false;
    }

  AS_StartMediaRecorder();

  return app_receive_object_reply(MSG_AUD_MRC_CMD_START);
}

static bool app_stop_recorder(void)
{
  AsStopMicFrontendParam  cmd;

  cmd.stop_mode = 0;

  AS_StopMicFrontend(&cmd);

  if (!app_receive_object_reply())
    {
      return false;
    }

  AS_StopMediaRecorder();

  if (!app_receive_object_reply())
    {
      return false;
    }

  size_t occupied_simple_fifo_size = CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > READ_SIMPLE_FIFO_SIZE) ?
        READ_SIMPLE_FIFO_SIZE : occupied_simple_fifo_size;
      app_write_output_file(output_size);
      occupied_simple_fifo_size = CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle);
    }

  if (target_codec_type == AS_CODECTYPE_LPCM)
    {
      if (!app_update_wav_file_size())
        {
          printf("Error: app_update_wav_file_size() failure.\n");
        }
    }

  app_close_output_file();
  return true;
}

#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC
static bool app_init_mfe(void)
{
  static InitParam    s_initparam;
  AsInitPreProcParam  init_param;

  init_param.packet_addr = reinterpret_cast<uint8_t *>(&s_initparam);
  init_param.packet_size = sizeof(s_initparam);

  AS_InitPreprocFrontend(&init_param);

  return app_receive_object_reply(MSG_AUD_MFE_CMD_INITPREPROC);
}

static bool app_set_mfe(void)
{
  static SetParam   s_setparam;
  AsSetPreProcParam set_param;

  s_setparam.enable = true;
  s_setparam.coef   = 99;

  set_param.packet_addr = reinterpret_cast<uint8_t *>(&s_setparam);
  set_param.packet_size = sizeof(s_setparam);

  AS_SetPreprocMicFrontend(&set_param);

  return app_receive_object_reply(MSG_AUD_MFE_CMD_SETPREPROC);
}
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC */

static bool app_set_clkmode(void)
{
  cxd56_audio_clkmode_t mode;

  mode = (target_samplingrate == AS_SAMPLINGRATE_192000) ? CXD56_AUDIO_CLKMODE_HIRES : CXD56_AUDIO_CLKMODE_NORMAL;

  return (cxd56_audio_set_clkmode(mode) == CXD56_AUDIO_ECODE_OK);
}

static bool app_open_file_dir(void)
{
  DIR *dirp;
  int ret;
  const char *name = RECFILE_ROOTPATH;

  dirp = opendir("/mnt");
  if (!dirp)
    {
      printf("opendir err(errno:%d)\n",errno);
      return false;
    }
  ret = mkdir(name, 0777);
  if (ret != 0)
    {
      if(errno != EEXIST)
        {
          printf("mkdir err(errno:%d)\n",errno);
          return false;
        }
    }

  s_recorder_info.file.dirp = dirp;
  return true;
}

static bool app_close_file_dir(void)
{
  closedir(s_recorder_info.file.dirp);

  return true;
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
  const NumLayout layout_no = MEM_LAYOUT_RECORDER;
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

  if (s_container_format != NULL)
    {
      return false;
    }
  s_container_format = new WavContainerFormat();

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

  if (s_container_format != NULL)
    {
      delete s_container_format;
      s_container_format = NULL;
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
      /* Check the FIFO every 5 ms and fill if there is space. */

      usleep(5 * 1000);
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
    {"MP3",  &target_codec_type,     AS_CODECTYPE_MP3,       "Codec type     mp3"},
    {"PCM",  &target_codec_type,     AS_CODECTYPE_LPCM,      "Codec type     LPCM"},
    {"48k",  &target_samplingrate,   AS_SAMPLINGRATE_48000,  "Sampling rate   48kHz"},
    {"192k", &target_samplingrate,   AS_SAMPLINGRATE_192000, "Sampling rate  192kHz"},
    {"1ch",  &target_channel_number, AS_CHANNEL_MONO,        "Channel number 1ch(MONO)"},
    {"2ch",  &target_channel_number, AS_CHANNEL_STEREO,      "Channel number 2ch(STEREO)"},
    {"4ch",  &target_channel_number, AS_CHANNEL_4CH,         "Channel number 4ch"},
    {"16b",  &target_bit_length,     AS_BITLENGTH_16,        "Bit length     16bit"},
    {"24b",  &target_bit_length,     AS_BITLENGTH_24,        "Bit length     24bit"},
    {NULL,   NULL,                   0,                      NULL},
  },
  *p_node;
  bool  res = true;

  /* Set default parameter. */

  target_codec_type     = CODEC_TYPE;
  target_samplingrate   = SAMPLINGRATE;
  target_channel_number = CHANNEL_NUMBER;
  target_bit_length     = BIT_LENGTH;
  target_recording_time = RECORDER_REC_TIME;

  if (argc < 2)
    {
      /* Default configuration */
    }

  /* Help! Display Description */

  else if (strcmp(argv[1], "help") == 0)
    {
      printf("\n");

      for (p_node = nodes; p_node->target; p_node++)
        {
          printf("%-4s - %s\n", p_node->target, p_node->help);
        }
      printf("1... - recording time (sec)\n\n");

      res = false;
    }

  /* Option determination */

  else
    {
      for (int i = 1; i < argc; i++)
        {
          for (p_node = nodes; p_node->target; p_node++)
            {
              /* Option check */

              if (strncasecmp(p_node->target, argv[i], strlen(p_node->target)) != 0)
                {
                  continue;
                }

              /* Found the character string */

              if (p_node->param)
                {
                  *p_node->param = p_node->target_param;
                }
              break;
            }

          if (p_node->target)
            {
              continue;
            }

          /* When there is no option entered. */

          int   time = atoi(argv[i]);

          /* Numerical judgment */

          if (time > 0)
            {
              /* For numbers, use seconds */

              target_recording_time = time;
            }
          else
            {
              /* No corresponding command */

              printf("\nInvalid option '%s'\n", argv[i]);
              printf("Try '%s help' for more information.\n\n", argv[0]);
              res = false;
              break;
            }
        }
    }

  return res;
}

static const char *app_param_to_str(int id, uint32_t param)
{
  struct {
    int         id;
    uint32_t    param;
    const char *title;
  }
  nodes[] = {

    {1, AS_CODECTYPE_MP3,       "MP3"},
    {1, AS_CODECTYPE_LPCM,      "LPCM"},
    {1, AS_CODECTYPE_OPUS,      "OPUS"},
    {2, AS_SAMPLINGRATE_48000,  "48kHz"},
    {2, AS_SAMPLINGRATE_192000, "192kHz"},
    {3, AS_CHANNEL_MONO,        "1ch(MONO)"},
    {3, AS_CHANNEL_STEREO,      "2ch(STEREO)"},
    {3, AS_CHANNEL_4CH,         "4ch"},
    {4, AS_BITLENGTH_16,        "16bit"},
    {4, AS_BITLENGTH_24,        "24bit"},
    {0, 0,                      "!!! Undefined value !!!"},
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
  printf("Codec type     %s\n", app_param_to_str(1, target_codec_type));
  printf("Sampling rate  %s\n", app_param_to_str(2, target_samplingrate));
  printf("Channel number %s\n", app_param_to_str(3, target_channel_number));
  printf("Bit length     %s\n", app_param_to_str(4, target_bit_length));
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

  printf("Start audio recorder with object level i/f example\n");

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

  /* Open directory of recording file. */

  if (!app_open_file_dir())
    {
      printf("Error: app_open_file_dir() failure.\n");
      goto errour_open_file_dir;
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

  /* Set audio clock mode. */

  if (!app_set_clkmode())
    {
      printf("Error: app_set_clkmode() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Set recorder operation mode. */

  if (!app_set_recorder())
    {
      printf("Error: app_set_recorder() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Initialize recorder. */

  if (!app_init_recorder())
    {
      printf("Error: app_init_recorder() failure.\n");
      goto errout_init_recorder;
    }

 /* Set the initial gain of the microphone to be used. */

  if (!app_init_mic_gain(180)) /* set +18db @all mic */
    {
      printf("Error: app_init_mic_gain() failure.\n");
      goto errout_init_simple_fifo;
    }

#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC
  if (!app_init_mfe())
    {
      printf("Error: app_init_mfe() failure.\n");
      goto errout_init_recorder;
    }

  if (!app_set_mfe())
    {
      printf("Error: app_set_mfe() failure.\n");
      goto errout_init_recorder;
    }
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC */

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

  /* AudioSubsystem to Ready. */

  if (!app_set_ready())
    {
      printf("Error: app_set_ready() failure.\n");

      /* To end processing */
    }

errout_init_simple_fifo:

  /* AudioSubsystem to PowerOff. */

  if (!app_power_off())
    {
      printf("Error: app_power_off() failure.\n");

      /* To end processing */
    }

errout_power_on:

  /* Close directory of recording file. */

  if (!app_close_file_dir())
    {
      printf("Error: app_close_contents_dir() failure.\n");

      /* To end processing */
    }

errour_open_file_dir:

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

errout_create_audio_sub_system:

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
    }

errout:

  printf("Exit audio recorder with object level i/f example\n\n");

  return 0;
}
