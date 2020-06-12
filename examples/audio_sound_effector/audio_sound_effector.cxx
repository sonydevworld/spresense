/****************************************************************************
 * audio_sound_effector/audio_sound_effector_main.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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
#include <asmp/mpshm.h>
#include <arch/board/board.h>
#include "audio/audio_high_level_api.h"
#include "audio/audio_frontend_api.h"
#include "audio/audio_outputmix_api.h"
#include "audio/audio_capture_api.h"
#include "audio/audio_renderer_api.h"
#include "audio/audio_message_types.h"
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

/* Default Volume. */

#define VOLUME        0

/* Execution time(sec) */

#define WORKING_TIME  10

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For share memory. */

static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void filter_sample(int16_t *ptr, uint32_t size)
{
  /* Example : RC filter for 16bit PCM */

  int16_t *ls = ptr;
  int16_t *rs = ls + 1;

  static int16_t ls_l = 0;
  static int16_t rs_l = 0;

  if (!ls_l && !rs_l)
    {
      ls_l = *ls * 10/*gain*/;
      rs_l = *rs * 10/*gain*/;
    }

  for (uint32_t cnt = 0; cnt < size; cnt += 4)
    {
      *ls = (ls_l * 995 / 1000) + ((*ls * 10/*gain*/) * 5 / 1000);
      *rs = (rs_l * 995 / 1000) + ((*rs * 10/*gain*/) * 5 / 1000);

      ls_l = *ls;
      rs_l = *rs;

      ls += 2;
      rs += 2;
    }
}

/* ------------------------------------------------------------------------ */
static void app_attention_callback(const ErrorAttentionParam *attparam)
{
  printf("Attention!! %s L%d ecode %d subcode %d\n",
          attparam->error_filename,
          attparam->line_number,
          attparam->error_code,
          attparam->error_att_sub_code);
}

/* ------------------------------------------------------------------------ */
static bool app_create_audio_sub_system(void)
{
  bool result = false;

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MNG;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_INPUT_BUF_POOL;
  frontend_create_param.pool_id.output      = S0_PREPROC_BUF_POOL;
  frontend_create_param.pool_id.dsp         = S0_PRE_APU_CMD_POOL;

  result = AS_CreateMicFrontend(&frontend_create_param, NULL);
  if (!result)
    {
      printf("Error: AS_CreateMicFrontend() failure. system memory insufficient!\n");
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

  /* Register a callback function to be notified when a problem occurs in
   * OutputMixer.
   */

  result = AS_CreateOutputMixer(&output_mix_act_param, app_attention_callback);
  if (!result)
    {
      printf("Error: AS_CreateOutputMixer() failed. system memory insufficient!\n");
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

  /* Create renderer feature. */

  AsCreateRendererParam_t renderer_create_param;
  renderer_create_param.msgq_id.dev0_req  = MSGQ_AUD_RND;
  renderer_create_param.msgq_id.dev0_sync = MSGQ_AUD_RND_SYNC;
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

/* ------------------------------------------------------------------------ */
static void app_deact_audio_sub_system(void)
{
  AS_DeleteMicFrontend();
  AS_DeleteOutputMix();
  AS_DeleteCapture();
  AS_DeleteRenderer();
}

/* ------------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------------ */
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

  /* Enable input */

  error_code = cxd56_audio_en_input();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_en_input() error! [%d]\n", error_code);
      return false;
    }

  /* Activate MicFrontend */

  AsActivateMicFrontend mfe_act;

  mfe_act.param.input_device = AsMicFrontendDeviceMic;
  mfe_act.cb = NULL;

  AS_ActivateMicFrontend(&mfe_act);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_ACT))
    {
      printf("AS_ActivateOutputMixer() error!\n");
      return false;
    }

  /* Setting OutputMixer parameters
   *
   * Disable post-processing for reduce output delay.
   */

  AsActivateOutputMixer mixer_act;

  mixer_act.output_device = HPOutputDevice;
  mixer_act.mixer_type    = MainOnly;
  mixer_act.post_enable   = PostFilterDisable;
  mixer_act.cb            = NULL;

  AS_ActivateOutputMixer(OutputMixer0, &mixer_act);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_ACT))
    {
      printf("AS_ActivateOutputMixer() error!\n");
      return false;
    }

  return true;
}

/* ------------------------------------------------------------------------ */
static void frontend_done_callback(AsPcmDataParam pcm_param)
{
  AsSendDataOutputMixer data;

  data.handle   = OutputMixer0;
  data.callback = pcm_param.callback;
  data.pcm      = pcm_param;

  /* You can imprement any audio signal process */

  if (data.pcm.size)
    {
      filter_sample((int16_t*)data.pcm.mh.getPa(), data.pcm.size);
    }

  AS_SendDataOutputMixer(&data);
}

/* ------------------------------------------------------------------------ */
static bool app_init_baseband(void)
{
  bool result;

  /* Setting MicFrontend parameters
   *
   * To minimize microphone input delay, reduce the interval of every
   * callback function calls. In this example, callback function called in
   * every 5ms (= 1sec / 48kHz * 240sample).
   * And disable DSP preprocessing for more time reducing.
   */

  AsInitMicFrontendParam init_mfe;

  init_mfe.channel_number    = AS_CHANNEL_STEREO;
  init_mfe.bit_length        = AS_BITLENGTH_16;
  init_mfe.samples_per_frame = 240;
  init_mfe.out_fs            = AS_SAMPLINGRATE_48000;
  init_mfe.preproc_type      = AsMicFrontendPreProcThrough;
  init_mfe.data_path         = AsDataPathCallback;
  init_mfe.dest.cb           = frontend_done_callback;

  result = AS_InitMicFrontend(&init_mfe);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_INIT))
    {
      printf("AS_InitMicFrontend() error!\n");
    }

  return result;
}

/* ------------------------------------------------------------------------ */
static bool app_start_baseband(void)
{
  AsStartMicFrontendParam start_mfe;

  AS_StartMicFrontend(&start_mfe);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_START))
    {
      printf("AS_StartMicFrontend() error!\n");
      return false;
    }

  return true;
}

/* ------------------------------------------------------------------------ */
static bool app_stop_baseband(void)
{
  AsStopMicFrontendParam stop_mfe;

  AS_StopMicFrontend(&stop_mfe);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_STOP))
    {
      printf("AS_StopMicFrontend() error!\n");
      return false;
    }

  return true;
}

/* ------------------------------------------------------------------------ */
static bool app_deactivate_baseband(void)
{
  /* Deactivate MicFrontend */

  AsDeactivateMicFrontendParam deact_mfe;

  AS_DeactivateMicFrontend(&deact_mfe);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_DEACT))
    {
      printf("AS_DeactivateMicFrontend() error!\n");

      /* Continue termination processing */;
    }

  /* Deactivate OutputMixer */

  AsDeactivateOutputMixer mixer_deact;

  AS_DeactivateOutputMixer(OutputMixer0, &mixer_deact);

  if (!app_receive_object_reply(MSG_AUD_MIX_CMD_DEACT))
    {
      printf("AS_DeactivateOutputMixer() error!\n");

      /* Continue termination processing */;
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

/* ------------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------------ */
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
      printf("Error: Manager::initPerCpu() failure. %d\n", err);
      return false;
    }

  return true;
}

/* ------------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------------ */
void app_working_process(uint32_t play_time)
{
  /* Here you can add the processing you want to do during the sound effector */

  sleep(play_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  printf("Start sound effector example\n");

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

  /* Set the device to output the mixed audio. */

  if (!app_activate_baseband())
    {
      printf("Error: app_activate_baseband() failure.\n");

      /* Abnormal termination processing */

      goto errout_init_output_select;
    }

  /* Volume settings */

  app_set_volume(VOLUME);

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");

      /* Abnormal termination processing */

      goto errout_init_output_select;
    }

  if (!app_init_baseband())
    {
      printf("Error: app_init_baseband() failure.\n");
    }

  if (!app_start_baseband())
    {
      printf("Error: app_start_baseband() failure.\n");
    }

  printf("Running time is %d sec\n", WORKING_TIME);

  app_working_process(WORKING_TIME);

  if (!app_stop_baseband())
    {
      printf("Error: app_stop_baseband() failure.\n");

      /* Continue termination processing */;
    }

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

errout_init_output_select:

  /* Deactivate baseband */

  if (!app_deactivate_baseband())
    {
      printf("Error: app_deactivate_baseband() failure.\n");

    }

  app_deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

errout_act_audio_sub_system:
  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
      return 1;
    }

  printf("Exit  sound effector example\n");

  return 0;
}
