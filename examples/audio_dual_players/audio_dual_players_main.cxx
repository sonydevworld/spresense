/****************************************************************************
 * audio_dual_players/audio_dual_players_main.cxx
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

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*------------------------------
 * User definable definitions
 *------------------------------
 */

/* Path of playback file. */

#define PLAYBACK_FILE_PATH "/mnt/sd0/AUDIO"

/* Path of DSP image file. */

#define DSPBIN_FILE_PATH   "/mnt/sd0/BIN"

/* Default Volume. -30dB */

#define PLAYER_DEF_VOLUME -300

/* Definition of FIFO information.
 * FIFO defined here refers to a buffer for passing playback data
 * between the application and AudioSubSystem
 */ 

/* Recommended frame size differs for each codec.
 *    MP3       : 1440 bytes
 *    AAC       : 1024 bytes
 *    WAV 16bit : 2560 bytes
 *    WAV 24bit : 3840 bytes
 * If the codec to be played is fixed, use this size.
 * When playing multiple codecs, select the largest size.
 * There is no problem increasing the size. If it is made smaller,
 * so it is necessary to be careful.
 */

#define FIFO_FRAME_SIZE         3840

/* The number of elements in the FIFO queue used varies depending on
 * the performance of the system and must be sufficient.
 */

#define FIFO_ELEMENT_NUM          20

/* The number of frames to read in one reading.
 * FIFO buffer size is determined by FIFO_FRAME_SIZE, FIFO_ELEMENT_NUM, and this.
 */

#define FIFO_ONE_READ_FRAME_NUM    2

/* The number of elements pushed to the FIFO Queue at a time depends
 * on the load of the system. If the number of elements is small,
 * we will underflow if application dispatching is delayed.
 * Define a sufficient maximum value.
 */

#define PLAYER_FIFO_PUSH_NUM_MAX  10

/* Content number 1 information definition. */

#define PLAYBACK0_FILE_NAME    "Sound0.mp3"
#define PLAYBACK0_CH_NUM        AS_CHANNEL_STEREO
#define PLAYBACK0_BIT_LEN       AS_BITLENGTH_16
#define PLAYBACK0_SAMPLING_RATE AS_SAMPLINGRATE_AUTO
#define PLAYBACK0_CODEC_TYPE    AS_CODECTYPE_MP3
#define PLAYBACK0_REPEAT_COUNT  4

/* Content number 2 information definition. */

#define PLAYBACK1_FILE_NAME    "Sound1.mp3"
#define PLAYBACK1_CH_NUM        AS_CHANNEL_STEREO
#define PLAYBACK1_BIT_LEN       AS_BITLENGTH_16
#define PLAYBACK1_SAMPLING_RATE AS_SAMPLINGRATE_AUTO
#define PLAYBACK1_CODEC_TYPE    AS_CODECTYPE_MP3
#define PLAYBACK1_REPEAT_COUNT  4

/*------------------------------
 * Definition specified by config
 *------------------------------
 */

/* Definition for selection of output device.
 * Select speaker output or I2S output.
 */

#ifdef CONFIG_EXAMPLES_AUDIO_DUAL_PLAYERS_OUTPUT_DEV_SPHP
#  define PLAYER_OUTPUT_DEV AS_SETPLAYER_OUTPUTDEVICE_SPHP
#  define PLAYER_MIXER_OUT  AS_OUT_SP
#else
#  define PLAYER_OUTPUT_DEV AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT
#  define PLAYER_MIXER_OUT  AS_OUT_I2S
#endif

/* FIFO control value. */

#define FIFO_ELEMENT_SIZE  (FIFO_FRAME_SIZE * FIFO_ONE_READ_FRAME_NUM)
#define FIFO_QUEUE_SIZE    (FIFO_ELEMENT_SIZE * FIFO_ELEMENT_NUM)

/* Local error code. */

#define FIFO_RESULT_OK  0
#define FIFO_RESULT_ERR 1
#define FIFO_RESULT_EOF 2
#define FIFO_RESULT_FUL 3

/* Number of retries due to incomplete SD card mounting. */

#define NUMBER_OF_RETRY 3

/* Timeout waiting for command reception. Unit is milliseconds. */

#define RESPONSE_TIMEOUT 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* For FIFO */

struct player_fifo_info_s : public AsPlayerInputDeviceHdlrForRAM
{
  CMN_SimpleFifoHandle  handle;
  uint32_t              fifo_area[FIFO_QUEUE_SIZE/sizeof(uint32_t)];
  uint8_t               read_buf[FIFO_ELEMENT_SIZE];
};

struct Track
{
  char      title[64];
  uint8_t   channel_number;  /* Channel number. */
  uint8_t   bit_length;      /* Bit length.     */
  uint32_t  sampling_rate;   /* Sampling rate.  */
  uint8_t   codec_type;      /* Codec type.     */
  int       repeat_count;    /* Repeat count.   */
};

/* For play file */

struct player_file_info_s
{
  Track   track;
  int32_t size;
  int     fd;
};

/* Player info */

struct player_info_s
{
  AsPlayerId                  id;
  bool                        end;
  struct player_fifo_info_s   fifo;
  struct player_file_info_s   file;
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

/* For control player */

static struct player_info_s  s_player0_info;
static struct player_info_s  s_player1_info;

/* For share memory. */

static mpshm_t s_shm;

/* For frequency lock. */

static struct pm_cpu_freqlock_s s_player_lock;

/* Play time(sec) */

uint32_t  s_player_play_time = 0;   /* 0: No limit */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static void app_get_next_track(Track* track0, Track* track1)
{
  snprintf(track0->title, sizeof(track0->title), "%s", PLAYBACK0_FILE_NAME);
  track0->channel_number = PLAYBACK0_CH_NUM;
  track0->bit_length     = PLAYBACK0_BIT_LEN;
  track0->sampling_rate  = PLAYBACK0_SAMPLING_RATE;
  track0->codec_type     = PLAYBACK0_CODEC_TYPE;
  track0->repeat_count   = PLAYBACK0_REPEAT_COUNT;

  snprintf(track1->title, sizeof(track1->title), "%s", PLAYBACK1_FILE_NAME);
  track1->channel_number = PLAYBACK1_CH_NUM;
  track1->bit_length     = PLAYBACK1_BIT_LEN;
  track1->sampling_rate  = PLAYBACK1_SAMPLING_RATE;
  track1->codec_type     = PLAYBACK1_CODEC_TYPE;
  track1->repeat_count   = PLAYBACK1_REPEAT_COUNT;
}

static void app_input_device_callback(uint32_t size)
{
    /* do nothing */
}

static bool app_init_simple_fifo(struct player_fifo_info_s *fifo)
{
  if (CMN_SimpleFifoInitialize(&fifo->handle,
                                fifo->fifo_area,
                                FIFO_QUEUE_SIZE,
                                NULL) != 0)
    {
      return false;
    }

  CMN_SimpleFifoClear(&fifo->handle);

  return true;
}

static int app_push_simple_fifo(struct player_info_s *info)
{
  int ret;

  ret = read(info->file.fd, &info->fifo.read_buf, FIFO_ELEMENT_SIZE);

  if (ret < 0)
    {
      printf("Error: Fail to read file. errno:%d\n", get_errno());
      return FIFO_RESULT_ERR;
    }

  if (ret > 0 && CMN_SimpleFifoOffer(&info->fifo.handle, (const void*)(info->fifo.read_buf), ret) == 0)
    {
      return FIFO_RESULT_FUL;
    }

  info->file.size = (info->file.size - ret);

  if (info->file.size == 0)
    {
      return FIFO_RESULT_EOF;
    }

  return FIFO_RESULT_OK;
}

static bool app_first_push_simple_fifo(struct player_info_s *info)
{
  int i;
  int ret = 0;

  for(i = 0; i < FIFO_ELEMENT_NUM - 1; i++)
    {
      if ((ret = app_push_simple_fifo(info)) != FIFO_RESULT_OK)
        {
          break;
        }
    }

  return (ret != FIFO_RESULT_ERR) ? true : false;
}

static int app_refill_simple_fifo(struct player_info_s *info)
{
  int     ret = FIFO_RESULT_OK;
  size_t  vacant_size;

  vacant_size = CMN_SimpleFifoGetVacantSize(&info->fifo.handle);

  if ((vacant_size != 0) && (vacant_size > FIFO_ELEMENT_SIZE))
    {
      int push_cnt = vacant_size / FIFO_ELEMENT_SIZE;

      push_cnt = (push_cnt >= PLAYER_FIFO_PUSH_NUM_MAX) ?
                  PLAYER_FIFO_PUSH_NUM_MAX : push_cnt;

      for (int i = 0; i < push_cnt; i++)
        {
          if ((ret = app_push_simple_fifo(info)) == FIFO_RESULT_ERR)
            {
              break;
            }
        }
    }

  return ret;
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
  ids.player_main = MSGQ_AUD_PLY0;
  ids.player_sub  = MSGQ_AUD_PLY1;
  ids.micfrontend = 0xFF;
  ids.mixer       = MSGQ_AUD_OUTPUT_MIX;
  ids.recorder    = 0xFF;
  ids.effector    = 0xFF;
  ids.recognizer  = 0xFF;

  AS_CreateAudioManager(ids, app_attention_callback);

  /* Create player feature. */

  AsCreatePlayerParams_t player_create_param;
  player_create_param.msgq_id.player   = MSGQ_AUD_PLY0;
  player_create_param.msgq_id.mng      = MSGQ_AUD_MGR;
  player_create_param.msgq_id.mixer    = MSGQ_AUD_OUTPUT_MIX;
  player_create_param.msgq_id.dsp      = MSGQ_AUD_DSP0;
  player_create_param.pool_id.es       = S0_DEC_ES_MAIN_BUF_POOL;
  player_create_param.pool_id.pcm      = S0_REND_PCM_BUF_POOL;
  player_create_param.pool_id.dsp      = S0_DEC_APU_CMD_POOL;
  player_create_param.pool_id.src_work = S0_SRC_WORK_MAIN_BUF_POOL;

  /* When calling AS_CreatePlayerMulti(), use the pool area
   * for multi-core playback processing.
   * When calling AS_CreatePlayer(), use the heap area.
   */

  result = AS_CreatePlayerMulti(AS_PLAYER_ID_0, &player_create_param, NULL);

  if (!result)
    {
      printf("Error: AS_CratePlayer(0) failure. system memory insufficient!\n");
      return false;
    }

  /* Create player(sub) feature. */

  player_create_param.msgq_id.player   = MSGQ_AUD_PLY1;
  player_create_param.msgq_id.mng      = MSGQ_AUD_MGR;
  player_create_param.msgq_id.mixer    = MSGQ_AUD_OUTPUT_MIX;
  player_create_param.msgq_id.dsp      = MSGQ_AUD_DSP1;
  player_create_param.pool_id.es       = S0_DEC_ES_SUB_BUF_POOL;
  player_create_param.pool_id.pcm      = S0_REND_PCM_SUB_BUF_POOL;
  player_create_param.pool_id.dsp      = S0_DEC_APU_CMD_POOL;
  player_create_param.pool_id.src_work = S0_SRC_WORK_SUB_BUF_POOL;

  result = AS_CreatePlayerMulti(AS_PLAYER_ID_1, &player_create_param, NULL);

  if (!result)
    {
      printf("Error: AS_CratePlayer(1) failure. system memory insufficient!\n");
      return false;
    }

  /* Create mixer feature. */

  AsCreateOutputMixParams_t output_mix_act_param;
  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.render_path0_filter_dsp = MSGQ_AUD_PFDSP0;
  output_mix_act_param.msgq_id.render_path1_filter_dsp = MSGQ_AUD_PFDSP1;
  output_mix_act_param.pool_id.render_path0_filter_pcm = S0_PF0_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path1_filter_pcm = S0_PF1_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path0_filter_dsp = S0_PF0_APU_CMD_POOL;
  output_mix_act_param.pool_id.render_path1_filter_dsp = S0_PF1_APU_CMD_POOL;

  result = AS_CreateOutputMixer(&output_mix_act_param, NULL);
  if (!result)
    {
      printf("Error: AS_CreateOutputMixer() failed. system memory insufficient!\n");
      return false;
    }

  /* Create renderer feature. */

  AsCreateRendererParam_t renderer_create_param;
  renderer_create_param.msgq_id.dev0_req  = MSGQ_AUD_RND_PLY0;
  renderer_create_param.msgq_id.dev0_sync = MSGQ_AUD_RND_PLY0_SYNC;
  renderer_create_param.msgq_id.dev1_req  = MSGQ_AUD_RND_PLY1;
  renderer_create_param.msgq_id.dev1_sync = MSGQ_AUD_RND_PLY1_SYNC;

  result = AS_CreateRenderer(&renderer_create_param);
  if (!result)
    {
      printf("Error: AS_CreateRenderer() failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

static void app_deact_audio_sub_system(void)
{
  AS_DeleteAudioManager();
  AS_DeletePlayer(AS_PLAYER_ID_0);
  AS_DeletePlayer(AS_PLAYER_ID_1);
  AS_DeleteOutputMix();
  AS_DeleteRenderer();
}

static bool sendAudioCommandAndWait(AudioCommand *command, AudioResult *out_result = NULL)
{
  AudioResult result;

  AS_SendAudioCommand(command);

  AS_ReceiveAudioResult(&result);

  if (out_result)
    {
      *out_result = result;
    }

  return printAudCmdResult(command->header.command_code, result);
}

static bool sendAudioCommandAndWait(AudioCommand *command, AsPlayerId id)
{
  AudioResult result;

  AS_SendAudioCommand(command);

  while (AS_ERR_CODE_OK != AS_ReceiveAudioResult(&result, id, RESPONSE_TIMEOUT));

  return printAudCmdResult(command->header.command_code, result);
}

static bool app_power_on(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_POWERON;
  command.header.command_code  = AUDCMD_POWERON;
  command.header.sub_code      = 0x00;
  command.power_on_param.enable_sound_effect = AS_DISABLE_SOUNDEFFECT;

  return sendAudioCommandAndWait(&command);
}

static bool app_power_off(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_POWEROFF_STATUS;
  command.header.command_code  = AUDCMD_SETPOWEROFFSTATUS;
  command.header.sub_code      = 0x00;

  return sendAudioCommandAndWait(&command);
}

static bool app_set_ready(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_READY_STATUS;
  command.header.command_code  = AUDCMD_SETREADYSTATUS;
  command.header.sub_code      = 0x00;

  return sendAudioCommandAndWait(&command);
}

static bool app_get_status(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_GETSTATUS;
  command.header.command_code  = AUDCMD_GETSTATUS;
  command.header.sub_code      = 0x00;

  AudioResult result;

  sendAudioCommandAndWait(&command, &result);

  return result.notify_status.status_info;
}

static bool app_init_output_select(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITOUTPUTSELECT;
  command.header.command_code  = AUDCMD_INITOUTPUTSELECT;
  command.header.sub_code      = 0;
  command.init_output_select_param.output_device_sel = PLAYER_MIXER_OUT;

  return sendAudioCommandAndWait(&command);
}

static bool app_set_volume(int master_db)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SETVOLUME;
  command.header.command_code  = AUDCMD_SETVOLUME;
  command.header.sub_code      = 0;
  command.set_volume_param.input1_db = 0;
  command.set_volume_param.input2_db = 0;
  command.set_volume_param.master_db = master_db;

  return sendAudioCommandAndWait(&command);
}

static bool app_set_player_status(void)
{
  AudioCommand command;

  /* Set simple FIFO to input device. */

  s_player0_info.fifo.simple_fifo_handler = (void*)(&s_player0_info.fifo.handle);
  s_player0_info.fifo.callback_function   = app_input_device_callback;
  s_player1_info.fifo.simple_fifo_handler = (void*)(&s_player1_info.fifo.handle);
  s_player1_info.fifo.callback_function   = app_input_device_callback;

  command.header.packet_length = LENGTH_SET_PLAYER_STATUS;
  command.header.command_code  = AUDCMD_SETPLAYERSTATUS;
  command.header.sub_code      = 0x00;
  command.set_player_sts_param.active_player         = AS_ACTPLAYER_BOTH;
  command.set_player_sts_param.player0.input_device  = AS_SETPLAYER_INPUTDEVICE_RAM;
  command.set_player_sts_param.player0.ram_handler   = &s_player0_info.fifo;
  command.set_player_sts_param.player0.output_device = PLAYER_OUTPUT_DEV;
  command.set_player_sts_param.player1.input_device  = AS_SETPLAYER_INPUTDEVICE_RAM;
  command.set_player_sts_param.player1.ram_handler   = &s_player1_info.fifo;
  command.set_player_sts_param.player1.output_device = PLAYER_OUTPUT_DEV;

  return sendAudioCommandAndWait(&command);
}

static int app_init_player(AsPlayerId id,
                           uint8_t    codec_type,
                           uint32_t   sampling_rate,
                           uint8_t    channel_number,
                           uint8_t    bit_length)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_PLAYER;
  command.header.command_code  = AUDCMD_INITPLAYER;
  command.header.sub_code      = 0x00;
  command.player.player_id                 = id;
  command.player.init_param.codec_type     = codec_type;
  command.player.init_param.bit_length     = bit_length;
  command.player.init_param.channel_number = channel_number;
  command.player.init_param.sampling_rate  = sampling_rate;
  snprintf(command.player.init_param.dsp_path,
           AS_AUDIO_DSP_PATH_LEN,
           "%s",
           DSPBIN_FILE_PATH);

  return sendAudioCommandAndWait(&command, id);
}

static int app_play_player(AsPlayerId id)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_PLAY_PLAYER;
  command.header.command_code  = AUDCMD_PLAYPLAYER;
  command.header.sub_code      = 0x00;
  command.player.player_id     = id;

  return sendAudioCommandAndWait(&command, id);
}

static bool app_stop_player(AsPlayerId id, int mode)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STOP_PLAYER;
  command.header.command_code  = AUDCMD_STOPPLAYER;
  command.header.sub_code      = 0x00;
  command.player.player_id            = id;
  command.player.stop_param.stop_mode = mode;

  return sendAudioCommandAndWait(&command, id);
}

static bool app_set_clkmode(int clk_mode)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SETRENDERINGCLK;
  command.header.command_code  = AUDCMD_SETRENDERINGCLK;
  command.header.sub_code      = 0x00;
  command.set_renderingclk_param.clk_mode = clk_mode;

  return sendAudioCommandAndWait(&command);
}

static bool app_init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, AUD_SRAM_SIZE);
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

  /* Create static memory pool. */

  const uint8_t sec_no = AUDIO_SECTION;
  const NumLayout layout_no = MEM_LAYOUT_PLAYER_MAIN_ONLY;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[AUDIO_SECTION][layout_no][0];
  err = Manager::createStaticPools(sec_no, layout_no,
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

static int app_play_file_open(FAR const char *file_path, FAR int32_t *file_size)
{
  int fd = open(file_path, O_RDONLY);

  *file_size = 0;
  if (fd >= 0)
    {
      struct stat stat_buf;
      if (stat(file_path, &stat_buf) == OK)
        {
          *file_size = stat_buf.st_size;
        }
    }

  return fd;
}

static bool app_open_next_play_file(struct player_info_s *info)
{
  char full_path[128];

  snprintf(full_path,
           sizeof(full_path),
           "%s/%s",
           PLAYBACK_FILE_PATH,
           info->file.track.title);

  for (int i = 0; i < NUMBER_OF_RETRY; i++)
    {
      info->file.fd = app_play_file_open(full_path, &info->file.size);

      if (info->file.fd < 0)
        {
          /* Retry due to incomplete SD card mounting. */

          usleep(100 * 1000);
        }
      else
        {
          if (info->file.size == 0)
            {
              close(info->file.fd);
              info->file.fd = -1;
              printf("Error: %s file is abnormal. check files!\n", full_path);
            }
          break;
        }
    }

  return !(info->file.fd < 0);
}

static void app_play_process(struct player_info_s *info, uint32_t play_time = 0)
{
  time_t  start_time;
  time_t  cur_time;
  int     result;
  int     stop_mode = AS_STOPPLAYER_NORMAL;

  time(&start_time);

  while (1)
    {
      result = app_refill_simple_fifo(info);

      if (result == FIFO_RESULT_ERR)
        {
          printf("Player%d error\n", info->id);
          break;
        }

      /* File end check */

      if (result == FIFO_RESULT_EOF)
        {
          printf("Player%d file end.\n", info->id);
          stop_mode = AS_STOPPLAYER_ESEND;
          break;
        }

      /* Play time check */

      if (play_time > 0 && (time(&cur_time) - start_time) >= play_time)
        {
          printf("Player%d end.\n", info->id);
          break;
        }

      /* Check the FIFO every 10 ms and fill if there is space. */

      usleep(10 * 1000);
    }

  /* Stop! */

  app_stop_player(info->id, stop_mode);
}

static int player_thread(int argc, FAR char *argv[])
{
  struct player_info_s *info = (struct player_info_s*)strtoul(argv[1], NULL, 16);

  Track *t = &info->file.track;

  for (int i = 0; i < t->repeat_count; i++)
    {
      /* File open check. */

      if (!app_open_next_play_file(info))
        {
          printf("File(%s) not found. \n", t->title);
          break;
        }

      /* Initialize simple fifo. */

      if (!app_init_simple_fifo(&info->fifo))
        {
          printf("Error: app_init_simple_fifo(%d) failure.\n", info->id);
          break;
        }

      /* Audio data is first packed into FIFO. */

      if (!app_first_push_simple_fifo(info))
        {
          printf("Error: app_first_push_simple_fifo(%d) failure.\n", info->id);
          break;
        }

      /* Initialize player */

      if (!app_init_player(info->id,
                           t->codec_type,
                           t->sampling_rate,
                           t->channel_number,
                           t->bit_length))
        {
          printf("Error: app_init_player(%d) failure.\n", info->id);
        }

      /* play! */

      if (!app_play_player(info->id))
        {
          printf("Error: app_play_player(%d) failure.\n", info->id);
          break;
        }

      /* Load audio data from file and push to FIFO. */

      app_play_process(info, s_player_play_time);

      /* File close. */

      if (close(info->file.fd) != 0)
        {
          printf("Error: close(%d) failure.\n", info->id);
        }

      CMN_SimpleFifoClear(&info->fifo.handle);
    }

  info->end = true;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  int   clk_mode = -1;
  int   tsk0     = -1;
  int   tsk1     = -1;
  char *argv0[2];
  char *argv1[2];
  char  arg0[16];
  char  arg1[16];

  if (argc > 1)
    {
      /* Set play time  */

      s_player_play_time = strtoul(argv[1], NULL, 10);
    }
  else
    {
      s_player_play_time = 0;
    }

  printf("Start Audio dual players example\n");

  /* Initialize task parameter. */

  snprintf(arg0, 16, "%p", &s_player0_info);
  snprintf(arg1, 16, "%p", &s_player1_info);

  s_player0_info.id  = AS_PLAYER_ID_0;
  s_player0_info.end = false;
  s_player1_info.id  = AS_PLAYER_ID_1;
  s_player1_info.end = false;

  argv0[0] = arg0;
  argv0[1] = NULL;
  argv1[0] = arg1;
  argv1[1] = NULL;

  /* Set play audio file info. */

  app_get_next_track(&s_player0_info.file.track, &s_player1_info.file.track);

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

  /* Initialize frequency lock parameter. */

  app_init_freq_lock();

  /* Lock cpu frequency to high. */

  app_freq_lock();

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_power_on())
    {
      printf("Error: app_power_on() failure.\n");
      goto errout_power_on;
    }

  /* Set the device to output the mixed audio. */

  if (!app_init_output_select())
    {
      printf("Error: app_init_output_select() failure.\n");
      goto errout_init_output_select;
    }

  /* Get current clock mode.
   * If the sampling rate is less than 48 kHz,
   * it will be in Normal mode. Otherwise, Hi-Res mode is set.
   */

  if (s_player0_info.file.track.sampling_rate <= AS_SAMPLINGRATE_48000 &&
      s_player1_info.file.track.sampling_rate <= AS_SAMPLINGRATE_48000)
    {
      clk_mode = AS_CLKMODE_NORMAL;
    }
  else if (s_player0_info.file.track.sampling_rate == AS_SAMPLINGRATE_192000 &&
           s_player1_info.file.track.sampling_rate == AS_SAMPLINGRATE_192000)
    {
      clk_mode = AS_CLKMODE_HIRES;
    }
  else
    {
      printf("Error: sampling_rate failure.\n");
      goto errout_init_output_select;
    }

  /* Set the clock mode of the output function. */

  if (!app_set_clkmode(clk_mode))
    {
      printf("Error: app_set_clkmode() failure.\n");
      goto errout_init_output_select;
    }

  /* Set player operation mode. */

  if (!app_set_player_status())
    {
      printf("Error: app_set_player_status() failure.\n");
      goto errout_init_output_select;
    }

  /* Set output volume. */

  if (!app_set_volume(PLAYER_DEF_VOLUME))
    {
      printf("Error: app_set_player_status() failure.\n");
      goto errout_set_volume;
    }

  /* Cancel output mute. */

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");
      goto errout_set_volume;
    }

  /* Running... */

  if (s_player_play_time > 0)
    {
      printf("Running time is %d sec\n", s_player_play_time);
    }
  else
    {
      printf("Running...\n");
    }

  /* Task start */

  tsk0 = task_create("Player0", 155, 1024, player_thread, (FAR char* const*)argv0);
  tsk1 = task_create("Player1", 155, 1024, player_thread, (FAR char* const*)argv1);

  /* Wait task end. */

  while ((tsk0 > 0 && !s_player0_info.end) ||
         (tsk1 > 0 && !s_player1_info.end))
    {
      usleep(500 * 1000);
    }

  if (tsk0 > 0 )
    {
      task_delete(tsk0);
    }
  if (tsk1 > 0 )
    {
      task_delete(tsk1);
    }

  /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
    }

errout_set_volume:

  /* Return the state of AudioSubSystem before voice_call operation. */

  if (AS_MNG_STATUS_READY != app_get_status())
    {
      if (!app_set_ready())
        {
          printf("Error: app_set_ready() failure.\n");
        }
    }

errout_init_output_select:

  /* Change AudioSubsystem to PowerOff state. */

  if (!app_power_off())
    {
      printf("Error: app_power_off() failure.\n");
    }

errout_power_on:

  /* Unlock cpu frequency. */

  app_freq_release();

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

errout_create_audio_sub_system:

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
    }

errout:

  printf("Exit Audio dual players example\n\n");

  return 0;
}
