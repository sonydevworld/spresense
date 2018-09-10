/****************************************************************************
 * recording_player/recording_player_main.c
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

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include <sys/stat.h>
#include "audio/audio_high_level_api.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"
#include "playlist/playlist.h"
#include <arch/chip/cxd56_audio.h>

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default Volume. -20dB */

#define PLAYER_DEF_VOLUME -200

/* Play time(sec) */

#define PLAYER_PLAY_TIME 30

/* Definition depending on Hireso mode. */

//#define CLK_MODE

#ifdef CLK_MODE
#  define CLK_MODE AS_CLKMODE_HIRES
#  define FIFO_FRAME_COEF  4
#else
#  define CLK_MODE AS_CLKMODE_NORMAL
#  define FIFO_FRAME_COEF  1
#endif

#define PLAYER_FIFO_PUSH_NUM_MAX  5

/* For FIFO(play) */

/* Definition of FIFO info.
 * The FIFO defined here refers to the buffer to pass playback data
 * between application and AudioSubSystem.
 */
 
/* Recommended frame size differs for each codec.
 *    MP3       : 1440 bytes
 *    AAC       : 1024 bytes
 *    WAV 16bit : 2560 bytes
 *    WAV 24bit : 3840 bytes
 * If the codec to be played is fixed, use this size.
 * When playing multiple codecs, select the largest size.
 * There is no problem increasing the size. If it is made smaller,
 * FIFO under is possibly generated, so it is necessary to be careful.
 */

#define PLY_SIMPLE_FIFO_FRAME_SIZE (3840 * FIFO_FRAME_COEF)

/* The number of elements in the FIFO queue used varies depending on
 * the performance of the system and must be sufficient.
 */

#define PLY_SIMPLE_FIFO_FRAME_NUM 10
#define PLY_SIMPLE_FIFO_BUF_SIZE  PLY_SIMPLE_FIFO_FRAME_SIZE * PLY_SIMPLE_FIFO_FRAME_NUM

/* For FIFO(record) */

/* Recorded data will be bigger when LPCM recording.
 * It requires 768 sample per a frame. 
 */

#define REC_SIMPLE_FIFO_FRAME_SIZE (3072 * FIFO_FRAME_COEF)

#define REC_SIMPLE_FIFO_FRAME_NUM 10
#define REC_SIMPLE_FIFO_BUF_SIZE  (REC_SIMPLE_FIFO_FRAME_SIZE * REC_SIMPLE_FIFO_FRAME_NUM)

/* Local error code. */

#define FIFO_RESULT_OK  0
#define FIFO_RESULT_ERR 1
#define FIFO_RESULT_EOF 2
#define FIFO_RESULT_FUL 3

/****************************************************************************
* Private Types Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/******** Data Dtructure ********/
/**** Player Data ****/

/* For FIFO(play) */

struct player_fifo_info_s
{
  CMN_SimpleFifoHandle          handle;
  AsPlayerInputDeviceHdlrForRAM input_device;
  uint32_t fifo_area[PLY_SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
};

/* Player info */

struct player_info_s
{
  struct player_fifo_info_s   fifo;
};

/**** Player Data ****/

/* For FIFO(record) */

struct recorder_fifo_info_s
{
  CMN_SimpleFifoHandle        handle;
  AsRecorderOutputDeviceHdlr  output_device;
  uint32_t fifo_area[REC_SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
};

/* Recorder info */

struct recorder_info_s
{
  struct recorder_fifo_info_s  fifo;
};

/* Bridge buffer between player fifo and recorder fifo */

uint8_t s_bridge_buf[REC_SIMPLE_FIFO_FRAME_SIZE];


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For control player */

static struct player_info_s  s_play_info;

/* For control recorder */

static recorder_info_s s_rec_info;

/* For share memory. */

static mpshm_t s_shm;

/* For frequency lock. */

static struct pm_cpu_freqlock_s s_player_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/******** Callback Functions ********/

/**** Mixer Functions ****/
/* Mixer Control Done */
static void outputmixer_done_callback(MsgQueId requester_dtq,
                                      MsgType reply_of,
                                      AsOutputMixDoneParam *done_param)
{
  printf("outputmixer done %d\n", done_param->done_type);
  return;
}

/* Mixer Rendering Done */
static void outmixer_send_callback(int32_t identifier, bool is_end)
{
  AsRequestNextParam next;

  next.type = (!is_end) ? AsNextNormalRequest : AsNextStopResRequest;

  AS_RequestNextPlayerProcess(AS_PLAYER_ID_0, &next);

  return;
}

/**** Player Functions ****/

/* Player Control Done */
static bool player_done_callback(AsPlayerEvent event, uint32_t result, uint32_t sub_result)
{
  printf("player done %d r 0x%x\n", event, result);
  return true;
}

/* Player Decode Done */
static void player_decode_done_callback(AsPcmDataParam pcm)
{
  AsSendDataOutputMixer data;

  data.handle   = OutputMixer0;
  data.callback = outmixer_send_callback;
  data.pcm      = pcm;

  AS_SendDataOutputMixer(&data);
}

/* Player ES Read Done (same as High Level API)*/
static void app_output_device_callback(uint32_t size)
{
    /* do nothing */
}

/**** Recorder Functions ****/

/* Recorder Control Done */
static bool recorder_done_callback(AsRecorderEvent evtype, uint32_t result, uint32_t sub_result)
{
  printf("recorder done %d r 0x%x\n", evtype, result);
  return true;
}

/* Recorder ES Write Done (same as High Level API)*/
static void app_input_device_callback(uint32_t size)
{
    /* do nothing */
}



/******** Clock Control Functions ********/
static void app_init_freq_lock(void)
{
  s_player_lock.count = 0;
  s_player_lock.info = PM_CPUFREQLOCK_TAG('A', 'P', 0);
  s_player_lock.flag = PM_CPUFREQLOCK_FLAG_HV;
}

static void app_freq_lock(void)
{
  up_pm_acquire_freqlock(&s_player_lock);
}

static void app_freq_release(void)
{
  up_pm_release_freqlock(&s_player_lock);
}


/******** FIFO Control Functions ********/
static bool app_init_simple_fifo(void)
{
  /* player fifo */

  if (CMN_SimpleFifoInitialize(&s_play_info.fifo.handle,
                               s_play_info.fifo.fifo_area,
                               PLY_SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO(player).");
      return false;
    }
  CMN_SimpleFifoClear(&s_play_info.fifo.handle);

  s_play_info.fifo.input_device.simple_fifo_handler = (void*)(&s_play_info.fifo.handle);
  s_play_info.fifo.input_device.callback_function = app_input_device_callback;

  /* recorder fifo */

  if (CMN_SimpleFifoInitialize(&s_rec_info.fifo.handle,
                               s_rec_info.fifo.fifo_area,
                               REC_SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO(recorder).");
      return false;
    }
  CMN_SimpleFifoClear(&s_rec_info.fifo.handle);

  s_rec_info.fifo.output_device.simple_fifo_handler = (void*)(&s_rec_info.fifo.handle);
  s_rec_info.fifo.output_device.callback_function = app_output_device_callback;

  return true;
}

static int app_push_player_fifo(char *buf, uint32_t size)
{
  if (CMN_SimpleFifoOffer(&s_play_info.fifo.handle,
                          (const void*)(buf),
                          size) == 0)
    {
      return FIFO_RESULT_FUL;
    }

  return FIFO_RESULT_OK;
}

static uint32_t app_pop_recorder_fifo(char *dst)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&s_rec_info.fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > REC_SIMPLE_FIFO_FRAME_SIZE) ?
                      REC_SIMPLE_FIFO_FRAME_SIZE : occupied_simple_fifo_size;

      if (CMN_SimpleFifoPoll(&s_rec_info.fifo.handle,
                             (void*)dst,
                             output_size) == 0)
        {
          printf("ERROR: Fail to get data from simple FIFO.\n");
          return 0;
        }
      else
        {
          return output_size;
        }

      occupied_simple_fifo_size -= output_size;
    }

  return 0;
}

/*** This Function for Pre-decode ***/

static bool app_prepare_player_fifo(void)
{
  size_t vacant_size;

  do
    {
      vacant_size = CMN_SimpleFifoGetVacantSize(&s_play_info.fifo.handle);

      uint32_t read_size = app_pop_recorder_fifo((char *)s_bridge_buf);
        
      if (read_size != 0)
        {
          if (app_push_player_fifo((char *)s_bridge_buf,
                                   read_size) != FIFO_RESULT_OK)
            {
              break;
            }
        }

      usleep(20 * 1000);
    }
  while (vacant_size > (PLY_SIMPLE_FIFO_BUF_SIZE * 50) / 100);

  return true;
}

/*** This Function for decoding ***/

static bool app_refill_simple_fifo(char *data, uint32_t write_size)
{
  int32_t ret = FIFO_RESULT_OK;
  size_t  vacant_size;

  vacant_size = CMN_SimpleFifoGetVacantSize(&s_play_info.fifo.handle);

  if ((vacant_size != 0) && (vacant_size > write_size))
    {
      int push_cnt = vacant_size / write_size;

      push_cnt = (push_cnt >= PLAYER_FIFO_PUSH_NUM_MAX) ?
                  PLAYER_FIFO_PUSH_NUM_MAX : push_cnt;

      for (int i = 0; i < push_cnt; i++)
        {
          if ((ret = app_push_player_fifo(data,
                                          write_size)) != FIFO_RESULT_OK)
            {
              break;
            }
        }
    }

  return (ret == FIFO_RESULT_OK) ? true : false;
}


/******** Create Functions ********/
static bool app_create_audio_sub_system(void)
{
  bool result = false;

  /* Create MediaPlayer feature. */

  AsCreatePlayerParam_t player_create_param;
  player_create_param.msgq_id.player = MSGQ_AUD_PLY;
  player_create_param.msgq_id.mng    = MSGQ_AUD_MNG;
  player_create_param.msgq_id.mixer  = MSGQ_AUD_OUTPUT_MIX;
  player_create_param.msgq_id.dsp    = MSGQ_AUD_DSP;
  player_create_param.pool_id.es     = DEC_ES_MAIN_BUF_POOL;
  player_create_param.pool_id.pcm    = REND_PCM_BUF_POOL;
  player_create_param.pool_id.dsp    = DEC_APU_CMD_POOL;

  result = AS_CreatePlayer(AS_PLAYER_ID_0, &player_create_param);

  if (!result)
    {
      printf("Error: AS_CratePlayer() failure!\n");
      return false;
    }

  /* Create OutputMixer feature. */

  AsCreateOutputMixParam_t output_mix_act_param;
  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.render_path0_filter_dsp = MSGQ_AUD_PFDSP0;
  output_mix_act_param.msgq_id.render_path1_filter_dsp = MSGQ_AUD_PFDSP1;
  output_mix_act_param.pool_id.render_path0_filter_pcm = PF0_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path1_filter_pcm = PF1_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path0_filter_dsp = PF0_APU_CMD_POOL;
  output_mix_act_param.pool_id.render_path1_filter_dsp = PF1_APU_CMD_POOL;

  result = AS_CreateOutputMixer(&output_mix_act_param);
  if (!result)
    {
      printf("Error: AS_CreateOutputMixer() failed!\n");
      return false;
    }

  /* Create renderer feature. */

  AsCreateRendererParam_t renderer_create_param;
  renderer_create_param.msgq_id.dev0_req  = MSGQ_AUD_RND_PLY;
  renderer_create_param.msgq_id.dev0_sync = MSGQ_AUD_RND_PLY_SYNC;
  renderer_create_param.msgq_id.dev1_req  = 0xFF;
  renderer_create_param.msgq_id.dev1_sync = 0xFF;

  result = AS_CreateRenderer(&renderer_create_param);
  if (!result)
    {
      printf("Error: AS_CreateRenderer() failure!\n");
      return false;
    }

  /* Create MediaRecorder feature. */

  AsCreateRecorderParam_t recorder_create_param;
  recorder_create_param.msgq_id.recorder = MSGQ_AUD_RECORDER; 
  recorder_create_param.msgq_id.mng      = MSGQ_AUD_MNG;
  recorder_create_param.msgq_id.dsp      = MSGQ_AUD_DSP_ENC;
  recorder_create_param.pool_id.input    = INPUT_BUF_POOL;
  recorder_create_param.pool_id.output   = ES_BUF_POOL;
  recorder_create_param.pool_id.dsp      = ENC_APU_CMD_POOL;

  result = AS_CreateMediaRecorder(&recorder_create_param);
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

/******** Delete Functions ********/
static void app_delete_audio_sub_system(void)
{
  AS_DeleteAudioManager();
  AS_DeletePlayer(AS_PLAYER_ID_0);
  AS_DeleteOutputMix();
  AS_DeleteRenderer();

  AS_DeleteMediaRecorder();
  AS_DeleteCapture();
}


/******** BaseBand Control Functions ********/
/**** Activation Objects ****/
static bool app_activate_baseband(void)
{
  CXD56_AUDIO_ECODE error_code;

  /* Power on audio device */

  error_code = cxd56_audio_poweron();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_poweron() error! [%d]\n", error_code);
      return false;
    }

  /* Set speaker out */

  error_code = cxd56_audio_set_spout(true);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_spout() error! [%d]\n", error_code);
      return false;
    }

  /* Enable output */

  error_code = cxd56_audio_en_output();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_en_output() error! [%d]\n", error_code);
      return false;
    }

  /* Enable input */

  error_code = cxd56_audio_en_input();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
       printf("cxd56_audio_en_input() error! [%d]\n", error_code);
       return false;
    }

  return true;
}

/**** Deactivation Objects ****/
static bool app_deactivate_baseband(void)
{
  CXD56_AUDIO_ECODE error_code;

  /* Disable output */

  error_code = cxd56_audio_dis_output();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_dis_output() error! [%d]\n", error_code);
      return false;
    }

  /* Power off audio device */

  error_code = cxd56_audio_poweroff();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_poweroff() error! [%d]\n", error_code);
      return false;
    }

  return true;
}


/**** Volume Control Function ****/
static bool app_set_volume(int master_db)
{
  /* Set volume to audio driver */

  CXD56_AUDIO_ECODE error_code;

  error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_OUT, master_db);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_vol() error! [%d]\n", error_code);
      return false;
    }

  error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_IN1, 0);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_vol() error! [%d]\n", error_code);
      return false;
    }

  error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_IN2, 0);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_vol() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

/******** Player Control Functions ********/

/**** Activation Objects ****/
static bool app_activate_player_system(void)
{
  /* Activate MediaPlayer */

  AsActivatePlayer player_act;

  player_act.param.input_device  = AS_SETPLAYER_INPUTDEVICE_RAM;
  player_act.param.ram_handler   = &s_play_info.fifo.input_device;
  player_act.param.output_device = AS_SETPLAYER_OUTPUTDEVICE_SPHP;
  player_act.cb                  = player_done_callback;

  AS_ActivatePlayer(AS_PLAYER_ID_0, &player_act);

  /* Activate OutputMixer */

  AsActivateOutputMixer mixer_act;

  mixer_act.output_device = HPOutputDevice;
  mixer_act.mixer_type    = MainOnly;
  mixer_act.pf_enable     = PostFilterDisable;
  mixer_act.cb            = outputmixer_done_callback;

  AS_ActivateOutputMixer(OutputMixer0, &mixer_act);

  return true;
}

/**** Initialize Objects ****/
static bool app_init_player(uint8_t codec_type,
                            uint32_t sampling_rate,
                            uint8_t channel_number,
                            uint8_t bit_length)
{
  AsInitPlayerParam player_init;

  player_init.codec_type     = codec_type;
  player_init.bit_length     = bit_length;
  player_init.channel_number = channel_number;
  player_init.sampling_rate  = sampling_rate;
  snprintf(player_init.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", "/mnt/sd0/BIN");
 
  AS_InitPlayer(AS_PLAYER_ID_0, &player_init);

  return true;
}

/**** Start Objects ****/
static bool app_play_player(void)
{
  AsPlayPlayerParam player_play;

  player_play.pcm_path          = AsPcmDataReply;
  player_play.pcm_dest.callback = player_decode_done_callback;

  AS_PlayPlayer(AS_PLAYER_ID_0, &player_play);

  return true;
}

/**** Stop Objects ****/
static bool app_stop_player(void)
{
  AsStopPlayerParam player_stop; 

  player_stop.stop_mode = AS_STOPPLAYER_NORMAL;

  AS_StopPlayer(AS_PLAYER_ID_0, &player_stop);

  return true;
}

/**** Deactivation Objects ****/
static bool app_deact_player_system(void)
{
  /* Deactivate MediaPlayer */

  AsDeactivatePlayer player_deact;

  AS_DeactivatePlayer(AS_PLAYER_ID_0, &player_deact);

  /* Deactivate OutputMixer */

  AsDeactivateOutputMixer mixer_deact;

  AS_DeactivateOutputMixer(OutputMixer0, &mixer_deact);

  return true;
}

/******** Recorder Control Functions ********/

/**** Activation Objects ****/
static bool app_activate_recorder_system(void)
{
  /* Activate MediaRecorder */

  AsActivateRecorder recorder_act;

  recorder_act.param.input_device          = AS_SETRECDR_STS_INPUTDEVICE_MIC;
  recorder_act.param.output_device         = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
  recorder_act.param.input_device_handler  = 0x00;
  recorder_act.param.output_device_handler = &s_rec_info.fifo.output_device;
  recorder_act.cb                          = recorder_done_callback;

  AS_ActivateMediaRecorder(&recorder_act);

  return true;
}

/**** Initialize Objects ****/
static bool app_init_recorder(uint8_t codec_type,
                              uint32_t sampling_rate,
                              uint8_t ch_num,
                              uint8_t bit_length)
{
  AsInitRecorderParam init_param;
  init_param.sampling_rate            = sampling_rate;
  init_param.channel_number           = ch_num;
  init_param.bit_length               = bit_length;
  init_param.codec_type               = codec_type;
  init_param.computational_complexity = AS_INITREC_COMPLEXITY_0;
  init_param.bitrate                  = AS_BITRATE_96000;
  snprintf(init_param.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", "/mnt/sd0/BIN");

  AS_InitMediaRecorder(&init_param);

  return true;
}

/**** Start Objects ****/
static bool app_start_recorder(void)
{
  CMN_SimpleFifoClear(&s_rec_info.fifo.handle);

  AS_StartMediaRecorder();

  return true;
}

/**** Stop Objects ****/
static bool app_stop_recorder(void)
{
  AS_StopMediaRecorder();

  return true;
}

/**** Deactivation Objects ****/
static bool app_deact_recorder_system(void)
{
  /* Deactivate MediaRecorder */

  AS_DeactivateMediaRecorder();

  return true;
}

/******** Initialize Libraries ********/

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

  err = Manager::initPerCpu(mml_data_area, NUM_MEM_POOLS);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  /* Create static memory pool of VoiceCall. */

  const NumLayout layout_no = MEM_LAYOUT_PLAYER_MAIN_ONLY;
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

/******** Finalize Libraries ********/
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


/******** Application Start Function ********/
static bool app_start(void)
{
  /* Init Recorder */

  if (!app_init_recorder(AS_CODECTYPE_LPCM,
                         AS_SAMPLINGRATE_48000,
                         AS_CHANNEL_STEREO,
                         AS_BITLENGTH_16))
    {
      printf("Error: app_recorder_player() failure.\n");
      CMN_SimpleFifoClear(&s_rec_info.fifo.handle);
      app_freq_release();
      return false;
    }

  /* Init Player */

  if (!app_init_player(AS_CODECTYPE_WAV,
                       AS_SAMPLINGRATE_48000,
                       AS_CHANNEL_STEREO,
                       AS_BITLENGTH_16))
    {
      printf("Error: app_init_player() failure.\n");
      CMN_SimpleFifoClear(&s_play_info.fifo.handle);
      app_freq_release();
      return false;
    }

  /* Start Recorder */

  if (!app_start_recorder())
    {
      printf("Error: app_start_recorder() failure.\n");
      CMN_SimpleFifoClear(&s_rec_info.fifo.handle);
      app_freq_release();
      return false;
    }

  /* Push data to simple fifo */

  if (!app_prepare_player_fifo())
    {
      printf("Error: app_prepare_player_fifo() failure.\n");
      CMN_SimpleFifoClear(&s_rec_info.fifo.handle);
      app_freq_release();
      return false;
    }

  /* Play Player */

  if (!app_play_player())
    {
      printf("Error: app_play_player() failure.\n");
      CMN_SimpleFifoClear(&s_play_info.fifo.handle);
      app_freq_release();
      return false;
    }

  return true;
}

/******** Application Stop Function ********/
static bool app_stop(void)
{
  bool result = true;

  if (!app_stop_recorder())
    {
      printf("Error: app_stop_recorder() failure.\n");
      result = false;
    }

  if (!app_stop_player())
    {
      printf("Error: app_stop_player() failure.\n");
      result = false;
    }

  return result;
}

/******** Each Frame Data Process ********/
void app_rec_play_process(uint32_t play_time)
{
  /* Timer Start */
  time_t start_time;
  time_t cur_time;

  time(&start_time);

  do
    {
      /* Check the FIFO every 2 ms and fill if there is space. */

      usleep(2 * 1000);

      /* Get record data */
    
      uint32_t read_size;

      do
        {
          read_size = app_pop_recorder_fifo((char *)s_bridge_buf);

          if (read_size == 0)
            {
              /* If no data read, try next. */

              break;;
            }

          /* Supply play data */

          if (!app_refill_simple_fifo((char *)s_bridge_buf, read_size))
            {
              break;
            }
        }
      while (read_size > 0);

    }
  while((time(&cur_time) - start_time) < play_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int recording_player_main(int argc, char *argv[])
#endif
{
  printf("Start AudioPlayer example\n");

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
      return 1;
    }

  /* Next, Activate the features used by AudioSubSystem. */

  if (!app_create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");

      /* Abnormal termination processing */

      goto errout_act_audio_sub_system;
    }

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Initialize simple fifo. */

  if (!app_init_simple_fifo())
    {
      printf("Error: app_init_simple_fifo() failure.\n");

      /* Abnormal termination processing */

      goto errout_init_simple_fifo;
    }

  /* Activate baseband drivers. */

  if (!app_activate_baseband())
    {
      printf("Error: app_activate_baseband() failure.\n");

      /* Abnormal termination processing */

      goto errout_activate_baseband;
    }

  /* Set player operation mode. */

  if (!app_activate_player_system())
    {
      printf("Error: app_activate_player_system() failure.\n");

      /* Abnormal termination processing */

      goto errout_activate_player_system;
    }

  if (!app_activate_recorder_system())
    {
      printf("Error: app_activate_recorder_system() failure.\n");

      /* Abnormal termination processing */

      goto errout_activate_recorder_system;
    }

  /* Wait activation complete. */

  usleep(100 * 1000);

  /* Cancel output mute. */

  app_set_volume(PLAYER_DEF_VOLUME);

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");

      /* Abnormal termination processing */

      goto errout_amp_mute_control;
    }

  /* Initialize frequency lock parameter. */

  app_init_freq_lock();

  /* Lock cpu frequency to high. */

  app_freq_lock();

  /* Start player operation. */

  if (!app_start())
    {
      printf("Error: app_start_player() failure.\n");

      /* Abnormal termination processing */

      goto errout_start;
    }

  /* Running... */

  printf("Running time is %d sec\n", PLAYER_PLAY_TIME);

  app_rec_play_process(PLAYER_PLAY_TIME);

  /* Stop player operation. */

  if (!app_stop())
    {
      printf("Error: app_stop() failure.\n");
      return 1;
    }

  /* Wait stop complete. */

  usleep(1000 * 1000);

  /* Unlock cpu frequency. */

  app_freq_release();

  /* Set output mute. */

errout_start:
  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

  /* Deactivate the features used by AudioSubSystem. */

errout_amp_mute_control:
  if (!app_deact_player_system())
    {
      printf("Error: app_deact_player_system() failure.\n");
      return 1;
    }

  if (!app_deact_recorder_system())
    {
      printf("Error: app_deact_recorder_system() failure.\n");
      return 1;
    }
 
  /* Wait deactivatio complete. */

  usleep(100 * 1000);

  /* Deactivate baseband */

errout_act_audio_sub_system:
errout_activate_player_system:
errout_activate_recorder_system:
  if (!app_deactivate_baseband())
    {
      printf("Error: app_deactivate_baseband() failure.\n");
 
    }

  /* Delete the features used by AudioSubSystem. */

errout_activate_baseband:
errout_init_simple_fifo:
  app_delete_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
      return 1;
    }

  printf("Exit AudioPlayer example\n");

  return 0;
}
