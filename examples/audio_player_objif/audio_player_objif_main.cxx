/****************************************************************************
 * audio_player_objif/audio_player_objif_main.c
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
#include "audio/audio_message_types.h"
#ifdef CONFIG_AUDIOUTILS_PLAYLIST
#include <audio/utilities/playlist.h>
#endif
#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC
#include "userproc_command.h"
#endif /* CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC */

/* Section number of memory layout to use */

#define AUDIO_SECTION   SECTION_NO0

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

#define DSPBIN_FILE_PATH "/mnt/sd0/BIN"

#ifdef CONFIG_AUDIOUTILS_PLAYLIST
/* Path of playlist file. */

#define PLAYLISTFILE_PATH  "/mnt/sd0/PLAYLIST"

/* PlayList file name. */

#define PLAY_LIST_NAME     "TRACK_DB.CSV"
#endif

/* Default Volume. -20dB */

#define PLAYER_DEF_VOLUME -200

/* Play time(sec) */

#define PLAYER_PLAY_TIME 10

/* Play file number */

#define PLAYER_PLAY_FILE_NUM 5

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

#define FIFO_FRAME_SIZE  3840

/* The number of elements in the FIFO queue used varies depending on
 * the performance of the system and must be sufficient.
 */

#define FIFO_ELEMENT_NUM  10

/* The number of elements pushed to the FIFO Queue at a time depends
 * on the load of the system. If the number of elements is small,
 * we will underflow if application dispatching is delayed.
 * Define a sufficient maximum value.
 */

#define PLAYER_FIFO_PUSH_NUM_MAX  5

/* Definition of content information to be used when not using playlist. */

#define PLAYBACK_FILE_NAME     "Sound.mp3"
#define PLAYBACK_CH_NUM        AS_CHANNEL_STEREO
#define PLAYBACK_BIT_LEN       AS_BITLENGTH_16
#define PLAYBACK_SAMPLING_RATE AS_SAMPLINGRATE_48000   
#define PLAYBACK_CODEC_TYPE    AS_CODECTYPE_MP3

/*------------------------------
 * Definition specified by config
 *------------------------------
 */

/* Definition for selection of output device.
 * Select speaker output or I2S output.
 */

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_OUTPUT_DEV_SPHP
#  define PLAYER_OUTPUT_DEV AS_SETPLAYER_OUTPUTDEVICE_SPHP
#  define PLAYER_MIXER_OUT  HPOutputDevice
#else
#  define PLAYER_OUTPUT_DEV AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT
#  define PLAYER_MIXER_OUT  I2SOutputDevice
#endif

/* Definition depending on player mode. */

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_MODE_HIRES
#  define FIFO_FRAME_NUM  4
#else
#  define FIFO_FRAME_NUM  1
#endif

/*------------------------------
 * Definition of example fixed
 *------------------------------
 */

/* FIFO control value. */

#define FIFO_ELEMENT_SIZE  (FIFO_FRAME_SIZE * FIFO_FRAME_NUM)
#define FIFO_QUEUE_SIZE    (FIFO_ELEMENT_SIZE * FIFO_ELEMENT_NUM)

/* Local error code. */

#define FIFO_RESULT_OK  0
#define FIFO_RESULT_ERR 1
#define FIFO_RESULT_EOF 2
#define FIFO_RESULT_FUL 3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* For FIFO */

struct player_fifo_info_s
{
  CMN_SimpleFifoHandle          handle;
  AsPlayerInputDeviceHdlrForRAM input_device;
  uint32_t fifo_area[FIFO_QUEUE_SIZE/sizeof(uint32_t)];
  uint8_t  read_buf[FIFO_ELEMENT_SIZE];
};

#ifndef CONFIG_AUDIOUTILS_PLAYLIST
struct Track
{
  char title[64];
  uint8_t   channel_number;  /* Channel number. */
  uint8_t   bit_length;      /* Bit length.     */
  uint32_t  sampling_rate;   /* Sampling rate.  */
  uint8_t   codec_type;      /* Codec type.     */
};
#endif

/* For play file */

struct player_file_info_s
{
  Track   track;
  int32_t size;
  DIR    *dirp;
  int     fd;
};

/* Player info */

struct player_info_s
{
  struct player_fifo_info_s   fifo;
  struct player_file_info_s   file;
#ifdef CONFIG_AUDIOUTILS_PLAYLIST
  Playlist *playlist_ins = NULL;
#else
#error "AUDIOUTILS_PLAYLIST is not enable"
#endif
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

static struct player_info_s  s_player_info;

/* For share memory. */

static mpshm_t s_shm;

/* For frequency lock. */

static struct pm_cpu_freqlock_s s_player_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void outmixer_send_callback(int32_t identifier, bool is_end)
{
  AsRequestNextParam next;

  next.type = (!is_end) ? AsNextNormalRequest : AsNextStopResRequest;

  AS_RequestNextPlayerProcess(AS_PLAYER_ID_0, &next);

  return;
}

static void player_decode_done_callback(AsPcmDataParam pcm)
{
  AsSendDataOutputMixer data;

  data.handle   = OutputMixer0;
  data.callback = outmixer_send_callback;
  data.pcm      = pcm;

  /* You can imprement any audio signal process */

  AS_SendDataOutputMixer(&data);
}

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

static bool app_open_contents_dir(void)
{
  DIR *dirp;
  const char *name = PLAYBACK_FILE_PATH;
  
  dirp = opendir(name);

  if (!dirp)
    {
      printf("Error: %s directory path error. check the path!\n", name);
      return false;
    }

  s_player_info.file.dirp = dirp;

  return true;
}

static bool app_close_contents_dir(void)
{
  closedir(s_player_info.file.dirp);

  return true;
}

#ifdef CONFIG_AUDIOUTILS_PLAYLIST
static bool app_open_playlist(void)
{
  bool result = false;

  if (s_player_info.playlist_ins != NULL)
    {
      printf("Error: Open playlist failure. Playlist is already open\n");
      return false;
    }

  s_player_info.playlist_ins = new Playlist(PLAY_LIST_NAME);
  
  result = s_player_info.playlist_ins->init(PLAYLISTFILE_PATH);
  if (!result)
    {
      printf("Error: Playlist::init() failure.\n");
      return false;
    }

  s_player_info.playlist_ins->setPlayMode(Playlist::PlayModeNormal);
  if (!result)
    {
      printf("Error: Playlist::setPlayMode() failure.\n");
      return false;
    }

  s_player_info.playlist_ins->setRepeatMode(Playlist::RepeatModeOff);
  if (!result)
    {
      printf("Error: Playlist::setRepeatMode() failure.\n");
      return false;
    }

  s_player_info.playlist_ins->select(Playlist::ListTypeAllTrack, NULL);
  if (!result)
    {
      printf("Error: Playlist::select() failure.\n");
      return false;
    }

  return true;
}

static bool app_close_playlist(void)
{
  if (s_player_info.playlist_ins == NULL)
    {
      printf("Error: Close playlist failure. Playlist is not open\n");
      return false;
    }

  delete s_player_info.playlist_ins;
  s_player_info.playlist_ins = NULL;

  return true;
}
#endif /* #ifdef CONFIG_AUDIOUTILS_PLAYLIST */

static bool app_get_next_track(Track* track)
{
  bool ret;

#ifdef CONFIG_AUDIOUTILS_PLAYLIST
  if (s_player_info.playlist_ins == NULL)
    {
      printf("Error: Get next track failure. Playlist is not open\n");
      return false;
    }

  ret = s_player_info.playlist_ins->getNextTrack(track);
#else
  snprintf(track->title, sizeof(track->title), "%s", PLAYBACK_FILE_NAME);
  track->channel_number = PLAYBACK_CH_NUM;
  track->bit_length     = PLAYBACK_BIT_LEN;
  track->sampling_rate  = PLAYBACK_SAMPLING_RATE;
  track->codec_type     = PLAYBACK_CODEC_TYPE;
#endif /* #ifdef CONFIG_AUDIOUTILS_PLAYLIST */

  return ret;
}

static void app_input_device_callback(uint32_t size)
{
    /* do nothing */
}

static bool app_init_simple_fifo(void)
{
  if (CMN_SimpleFifoInitialize(&s_player_info.fifo.handle,
                               s_player_info.fifo.fifo_area,
                               FIFO_QUEUE_SIZE,
                               NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(&s_player_info.fifo.handle);

  s_player_info.fifo.input_device.simple_fifo_handler = (void*)(&s_player_info.fifo.handle);
  s_player_info.fifo.input_device.callback_function = app_input_device_callback;

  return true;
}

static int app_push_simple_fifo(int fd)
{
  int ret;

  ret = read(fd, &s_player_info.fifo.read_buf, FIFO_ELEMENT_SIZE);
  if (ret < 0)
    {
      printf("Error: Fail to read file. errno:%d\n", get_errno());
      return FIFO_RESULT_ERR;
    }

  if (CMN_SimpleFifoOffer(&s_player_info.fifo.handle, (const void*)(s_player_info.fifo.read_buf), ret) == 0)
    {
      return FIFO_RESULT_FUL;
    }
  s_player_info.file.size = (s_player_info.file.size - ret);
  if (s_player_info.file.size == 0)
    {
      return FIFO_RESULT_EOF;
    }
  return FIFO_RESULT_OK;
}

static bool app_first_push_simple_fifo(int fd)
{
  int i;
  int ret = 0;

  for(i = 0; i < FIFO_ELEMENT_NUM - 1; i++)
    {
      if ((ret = app_push_simple_fifo(fd)) != FIFO_RESULT_OK)
        {
          break;
        }
    }

  return (ret != FIFO_RESULT_ERR) ? true : false;
}

static bool app_refill_simple_fifo(int fd)
{
  int32_t ret = FIFO_RESULT_OK;
  size_t  vacant_size;

  vacant_size = CMN_SimpleFifoGetVacantSize(&s_player_info.fifo.handle);

  if ((vacant_size != 0) && (vacant_size > FIFO_ELEMENT_SIZE))
    {
      int push_cnt = vacant_size / FIFO_ELEMENT_SIZE;

      push_cnt = (push_cnt >= PLAYER_FIFO_PUSH_NUM_MAX) ?
                  PLAYER_FIFO_PUSH_NUM_MAX : push_cnt;

      for (int i = 0; i < push_cnt; i++)
        {
          if ((ret = app_push_simple_fifo(fd)) != FIFO_RESULT_OK)
            {
              break;
            }
        }
    }

  return (ret == FIFO_RESULT_OK) ? true : false;
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

  AsCreatePlayerParams_t player_create_param;
  player_create_param.msgq_id.player = MSGQ_AUD_PLY0;
  player_create_param.msgq_id.mng    = MSGQ_AUD_MNG;
  player_create_param.msgq_id.mixer  = MSGQ_AUD_OUTPUT_MIX;
  player_create_param.msgq_id.dsp    = MSGQ_AUD_DSP;
  player_create_param.pool_id.es     = S0_DEC_ES_MAIN_BUF_POOL;
  player_create_param.pool_id.pcm    = S0_REND_PCM_BUF_POOL;
  player_create_param.pool_id.dsp    = S0_DEC_APU_CMD_POOL;
  player_create_param.pool_id.src_work = S0_SRC_WORK_BUF_POOL;

  result = AS_CreatePlayerMulti(AS_PLAYER_ID_0, &player_create_param, app_attention_callback);

  if (!result)
    {
      printf("Error: AS_CratePlayer() failure. system memory insufficient!\n");
      return false;
    }

  /* Create mixer feature. */

  AsCreateOutputMixParams_t output_mix_act_param;
  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.mng   = MSGQ_AUD_MNG;
  output_mix_act_param.msgq_id.render_path0_filter_dsp = MSGQ_AUD_PFDSP0;
  output_mix_act_param.msgq_id.render_path1_filter_dsp = MSGQ_AUD_PFDSP1;
  output_mix_act_param.pool_id.render_path0_filter_pcm = S0_PF0_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path1_filter_pcm = S0_PF1_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path0_filter_dsp = S0_PF0_APU_CMD_POOL;
  output_mix_act_param.pool_id.render_path1_filter_dsp = S0_PF1_APU_CMD_POOL;

  result = AS_CreateOutputMixer(&output_mix_act_param, app_attention_callback);
  if (!result)
    {
      printf("Error: AS_CreateOutputMixer() failed. system memory insufficient!\n");
      return false;
    }

  /* Create renderer feature. */

  AsCreateRendererParam_t renderer_create_param;
  renderer_create_param.msgq_id.dev0_req  = MSGQ_AUD_RND_PLY0;
  renderer_create_param.msgq_id.dev0_sync = MSGQ_AUD_RND_PLY0_SYNC;
  renderer_create_param.msgq_id.dev1_req  = 0xFF;
  renderer_create_param.msgq_id.dev1_sync = 0xFF;

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
  AS_DeletePlayer(AS_PLAYER_ID_0);
  AS_DeleteOutputMix();
  AS_DeleteRenderer();
}

static bool app_receive_object_reply(uint32_t id)
{
  AudioObjReply reply_info;
  AS_ReceiveObjectReply(MSGQ_AUD_MNG, &reply_info);

  if (reply_info.type != AS_OBJ_REPLY_TYPE_REQ)
    {
      printf("app_receive_object_reply() error! type 0x%x\n",
             reply_info.type);
      return false;
    }

  if (reply_info.id != id)
    {
      printf("app_receive_object_reply() error! id 0x%x(request id 0x%x)\n",
             reply_info.id, id);
      return false;
    }

  if (reply_info.result != AS_ECODE_OK)
    {
      printf("app_receive_object_reply() error! result 0x%x\n",
             reply_info.result);
      return false;
    }

  return true;
}

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

  /* Activate OutputMixer */

  AsActivateOutputMixer mixer_act;

  mixer_act.output_device = PLAYER_MIXER_OUT;
  mixer_act.mixer_type    = MainOnly;
  mixer_act.post_enable   = PostFilterDisable;
  mixer_act.cb            = NULL;

  AS_ActivateOutputMixer(OutputMixer0, &mixer_act);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_ACT))
    {
      printf("AS_ActivateOutputMixer() error!\n");
    }

  return true;
}

static bool app_deactivate_baseband(void)
{
  /* Deactivate OutputMixer */

  AsDeactivateOutputMixer mixer_deact;

  AS_DeactivateOutputMixer(OutputMixer0, &mixer_deact);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_DEACT))
    {
      printf("AS_DeactivateOutputMixer() error!\n");
    }

  CXD56_AUDIO_ECODE error_code;

  /* Power off audio device */

  error_code = cxd56_audio_poweroff();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_poweroff() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

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

static bool app_activate_player_system(void)
{
  /* Activate MediaPlayer */

  AsActivatePlayer player_act;

  player_act.param.input_device  = AS_SETPLAYER_INPUTDEVICE_RAM;
  player_act.param.ram_handler   = &s_player_info.fifo.input_device;
  player_act.param.output_device = PLAYER_OUTPUT_DEV;
  player_act.cb                  = NULL;

  /* If manager set NULL to callback function at activate function,
   * a response is sent to MessageQueueID of manager specified
   * at create function.
   * Wait until there is a response of the type specified in the argument.
   */

  AS_ActivatePlayer(AS_PLAYER_ID_0, &player_act);

  if (!app_receive_object_reply(MSG_AUD_PLY_CMD_ACT))
    {
      printf("AS_ActivatePlayer() error!\n");
    }

  return true;
}

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
  snprintf(player_init.dsp_path, AS_AUDIO_DSP_PATH_LEN, "%s", DSPBIN_FILE_PATH);
 
  AS_InitPlayer(AS_PLAYER_ID_0, &player_init);

  if (!app_receive_object_reply(MSG_AUD_PLY_CMD_INIT))
    {
      printf("AS_InitPlayer() error!\n");
    }

  return true;
}

static bool app_init_outputmixer(void)
{
  AsInitOutputMixer omix_init;

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC
  omix_init.postproc_type = AsPostprocTypeUserCustom;
#else /* CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC */
  omix_init.postproc_type = AsPostprocTypeThrough;
#endif /* CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC */
  snprintf(omix_init.dsp_path, sizeof(omix_init.dsp_path), "%s/POSTPROC", DSPBIN_FILE_PATH);

  AS_InitOutputMixer(OutputMixer0, &omix_init);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_INIT))
    {
      printf("AS_InitOutputMixer() error!\n");
      return false;
    }

  return true;
}

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC
static bool app_init_postprocess_dsp()
{
  AsInitPostProc param;
  InitParam initpostcmd;

  param.addr = reinterpret_cast<uint8_t *>(&initpostcmd);
  param.size = sizeof(initpostcmd);
  
  AS_InitPostprocOutputMixer(OutputMixer0, &param);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_INITMPP))
    {
      printf("AS_InitPostprocOutputMixer() error!\n");
      return false;
    }

  return true;
}

static bool app_set_postprocess_dsp(void)
{
  AsSetPostProc param;
  static bool s_toggle = false;
  s_toggle = (s_toggle) ? false : true;

  /* Create packet area (have to ensure until API returns.)  */

  SetParam setpostcmd;
  setpostcmd.enable = s_toggle;
  setpostcmd.coef = 99;

  param.addr = reinterpret_cast<uint8_t *>(&setpostcmd);
  param.size = sizeof(SetParam);

  AS_SetPostprocOutputMixer(OutputMixer0, &param);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_SETMPP))
    {
      printf("AS_SetPostprocOutputMixer() error!\n");
      return false;
    }
 
  return true;
}
#endif /* CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC */

static bool app_play_player(void)
{
  AsPlayPlayerParam player_play;

  player_play.pcm_path          = AsPcmDataReply;
  player_play.pcm_dest.callback = player_decode_done_callback;

  AS_PlayPlayer(AS_PLAYER_ID_0, &player_play);

  if (!app_receive_object_reply(MSG_AUD_PLY_CMD_PLAY))
    {
      printf("AS_PlayPlayer() error!\n");
    }

  return true;
}

static bool app_stop_player(int mode)
{
  AsStopPlayerParam player_stop; 

  player_stop.stop_mode = mode;

  AS_StopPlayer(AS_PLAYER_ID_0, &player_stop);

  if (!app_receive_object_reply(MSG_AUD_PLY_CMD_STOP))
    {
      printf("AS_StopPlayer() error!\n");
    }

  return true;
}

static bool app_set_clkmode(int clk_mode)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;

  cxd56_audio_clkmode_t mode;

  mode = (clk_mode == AS_CLKMODE_NORMAL)
           ? CXD56_AUDIO_CLKMODE_NORMAL : CXD56_AUDIO_CLKMODE_HIRES;

  error_code = cxd56_audio_set_clkmode(mode);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_clkmode() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

static bool app_deact_player_system(void)
{
  /* Deactivate MediaPlayer */

  AsDeactivatePlayer player_deact;

  AS_DeactivatePlayer(AS_PLAYER_ID_0, &player_deact);

  if (!app_receive_object_reply(MSG_AUD_PLY_CMD_DEACT))
    {
      printf("AS_DeactivatePlayer() error!\n");
    }

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

  /* Create static memory pool of AudioPlayer. */

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

static bool app_open_next_play_file(void)
{
  /* Get next track */

  if (!app_get_next_track(&s_player_info.file.track))
    {
      printf("Error: No more tracks to play.\n");
      return false;
    }

  char full_path[128];
  snprintf(full_path,
           sizeof(full_path),
           "%s/%s",
           PLAYBACK_FILE_PATH,
           s_player_info.file.track.title);

  s_player_info.file.fd = app_play_file_open(full_path, &s_player_info.file.size);
  if (s_player_info.file.fd < 0)
    {
      printf("Error: %s open error. check paths and files!\n", full_path);
      return false;
    }
  if (s_player_info.file.size == 0)
    {
      close(s_player_info.file.fd);
      printf("Error: %s file size is abnormal. check files!\n",full_path);
      return false;
    }

  /* Push data to simple fifo */

  if (!app_first_push_simple_fifo(s_player_info.file.fd))
    {
      printf("Error: app_first_push_simple_fifo() failure.\n");
      CMN_SimpleFifoClear(&s_player_info.fifo.handle);
      close(s_player_info.file.fd);
      return false;
    }

  return true;
}

static bool app_close_play_file(void)
{
  if (close(s_player_info.file.fd) != 0)
    {
      printf("Error: close() failure.\n");
      return false;
    }

  CMN_SimpleFifoClear(&s_player_info.fifo.handle);

  return true;
}

static bool app_start(void)
{
  /* Init Player */

  Track *t = &s_player_info.file.track;
  if (!app_init_player(t->codec_type,
                       t->sampling_rate,
                       t->channel_number,
                       t->bit_length))
    {
      printf("Error: app_init_player() failure.\n");
      app_close_play_file();
      return false;
    }

  if (!app_init_outputmixer())
    {
      printf("Error: app_init_outputmixer() failure.\n");
      app_close_play_file();
      return false;
    }

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC
  if (!app_init_postprocess_dsp())
    {
      printf("Error: app_init_postprocess_dsp() failure.\n");
      app_close_play_file();
      return false;
    }
#endif /* CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC */

  if (!app_play_player())
    {
      printf("Error: app_play_player() failure.\n");
      CMN_SimpleFifoClear(&s_player_info.fifo.handle);
      close(s_player_info.file.fd);
      app_freq_release();
      return false;
    }

  return true;
}

static bool app_stop(void)
{
  bool result = true;

  /* Set stop mode.
   * If the end of the file is detected, play back until the data empty
   * and then stop.(select AS_STOPPLAYER_ESEND)
   * Otherwise, stop immediate reproduction.(select AS_STOPPLAYER_NORMAL)
   */

  int  stop_mode = (s_player_info.file.size != 0) ?
                    AS_STOPPLAYER_NORMAL : AS_STOPPLAYER_ESEND;

  if (!app_stop_player(stop_mode))
    {
      printf("Error: app_stop_player() failure.\n");
      result = false;
    }

  if (!app_close_play_file())
    {
      printf("Error: close() failure.\n");
      result = false;
    }

  return result;
}

void app_play_process(uint32_t play_time)
{
  /* Timer Start */
  time_t start_time;
  time_t cur_time;

  time(&start_time);

  do
    {
      /* Check the FIFO every 2 ms and fill if there is space. */

      usleep(2 * 1000);
      if (!app_refill_simple_fifo(s_player_info.file.fd))
        {
          break;
        }

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC
      static int cnt = 0;
      if (cnt++ > 100)
        {
          app_set_postprocess_dsp();
          cnt = 0;
        }
#endif /* CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_USEPOSTPROC */

    } while((time(&cur_time) - start_time) < play_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  /* Initialize clock mode.
   * Clock mode indicates whether the internal processing rate of
   * AudioSubSystem is Normal mode or Hi-Res mode. 
   * The sampling rate of the playback file determines which mode
   * will be taken. When playing a Hi-Res file,
   * please set player mode to Hi-Res mode with config.
   */

  int clk_mode = -1;

  printf("Start AudioPlayer with ObjIf example\n");

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

      /* Abnormal termination processing */

      goto errout_act_audio_sub_system;
    }

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Open directory of play contents. */

  if (!app_open_contents_dir())
    {
      printf("Error: app_open_contents_dir() failure.\n");

      /* Abnormal termination processing */

      goto errout_open_contents_dir;
    }

  /* Initialize frequency lock parameter. */

  app_init_freq_lock();

  /* Lock cpu frequency to high. */

  app_freq_lock();

#ifdef CONFIG_AUDIOUTILS_PLAYLIST
  /* Open playlist. */

  if (!app_open_playlist())
    {
      printf("Error: app_open_playlist() failure.\n");

      /* Abnormal termination processing */

      goto errout_open_playlist;
    }
#endif

  /* Initialize simple fifo. */

  if (!app_init_simple_fifo())
    {
      printf("Error: app_init_simple_fifo() failure.\n");

      /* Abnormal termination processing */

      goto errout_init_simple_fifo;
    }

  /* Set the device to output the mixed audio. */

  if (!app_activate_baseband())
    {
      printf("Error: app_activate_baseband() failure.\n");

      /* Abnormal termination processing */

      goto errout_init_output_select;
    }

  /* Set player operation mode. */

  if (!app_activate_player_system())
    {
      printf("Error: app_activate_player_system() failure.\n");
      return 1;
    }

  for (int i = 0; i < PLAYER_PLAY_FILE_NUM; i++)
    {
      if (!app_open_next_play_file())
        {
          /* Abnormal termination processing */

          goto errout_open_next_play_file;
        }

      /* Get current clock mode.
       * If the sampling rate is less than 48 kHz,
       * it will be in Normal mode. Otherwise, Hi-Res mode is set.
       */

      int cur_clk_mode;
      if (s_player_info.file.track.sampling_rate <= AS_SAMPLINGRATE_48000)
        {
          cur_clk_mode = AS_CLKMODE_NORMAL;
        }
      else
        {
          cur_clk_mode = AS_CLKMODE_HIRES;
        }

      /* If clockmode is Hi-Res and player mode is not Hi-Res,
       * play the next file.
       */

#ifdef CONFIG_EXAMPLES_AUDIO_PLAYER_OBJIF_MODE_NORMAL
      if (cur_clk_mode == AS_CLKMODE_HIRES)
        {
          printf("Hi-Res file is not supported.\n"
                 "Please change player mode to Hi-Res with config.\n");
          app_close_play_file();

          /* Play next file. */

          continue;
        }
#endif

      /* If current clock mode is different from the previous clock mode,
       * perform initial setting.
       */

      if (clk_mode != cur_clk_mode)
        {
          /* Update clock mode. */

          clk_mode = cur_clk_mode;

          /* Since the initial setting is required to be in the Ready state,
           * if it is not in the Ready state, it is set to the Ready state.
           */

          if (board_external_amp_mute_control(true) != OK)
            {
              printf("Error: board_external_amp_mute_control(true) failuer.\n");

              /* Abnormal termination processing */

              goto errout_amp_mute_control;
            }

          if (!app_deactivate_baseband())
            {
              printf("Error: app_deactivate_baseband() failure.\n");
         
            }

          /* Set the clock mode of the output function. */

          if (!app_set_clkmode(clk_mode))
            {
              printf("Error: app_set_clkmode() failure.\n");

              /* Abnormal termination processing */

              goto errout_set_clkmode;
            }

          if (!app_activate_baseband())
            {
              printf("Error: app_activate_baseband() failure.\n");

              /* Abnormal termination processing */

              goto errout_init_output_select;
            }

           /* Cancel output mute. */

           app_set_volume(PLAYER_DEF_VOLUME);

          if (board_external_amp_mute_control(false) != OK)
            {
              printf("Error: board_external_amp_mute_control(false) failuer.\n");

              /* Abnormal termination processing */

              goto errout_amp_mute_control;
            }
        }

      /* Start player operation. */

      if (!app_start())
        {
          printf("Error: app_start_player() failure.\n");

          /* Abnormal termination processing */

          goto errout_start;
        }

      /* Running... */

      printf("Running time is %d sec\n", PLAYER_PLAY_TIME);

      app_play_process(PLAYER_PLAY_TIME);

      /* Stop player operation. */

      if (!app_stop())
        {
          printf("Error: app_stop() failure.\n");
          return 1;
        }

#ifndef CONFIG_AUDIOUTILS_PLAYLIST
      break;
#endif
    }

  /* Set output mute. */

errout_start:

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

#ifdef CONFIG_AUDIOUTILS_PLAYLIST
errout_open_playlist:
#endif
errout_amp_mute_control:
errout_init_simple_fifo:
errout_set_clkmode:
errout_init_output_select:
errout_open_next_play_file:

  /* Unlock cpu frequency. */

  app_freq_release();

  /* Close playlist. */

  if (!app_close_playlist())
    {
      printf("Error: app_close_playlist() failure.\n");
      return 1;
    }

  if (!app_close_contents_dir())
    {
      printf("Error: app_close_contents_dir() failure.\n");
      return 1;
    }

  /* Deactivate the features used by AudioSubSystem. */

  if (!app_deact_player_system())
    {
      printf("Error: app_deact_player_system() failure.\n");
      return 1;
    }

  /* Deactivate baseband */

  if (!app_deactivate_baseband())
    {
      printf("Error: app_deactivate_baseband() failure.\n");
 
    }

errout_open_contents_dir:
  app_deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

errout_act_audio_sub_system:
  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
      return 1;
    }

  printf("Exit AudioPlayer example\n");

  return 0;
}
