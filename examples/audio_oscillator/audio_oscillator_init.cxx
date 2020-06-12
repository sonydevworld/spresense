/****************************************************************************
 * audio_oscillator/audio_oscillator_init.cxx
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

#include <sdk/config.h>
#include <stdio.h>
#include <asmp/mpshm.h>
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"
#include "audio/audio_synthesizer_api.h"
#include "audio/audio_outputmix_api.h"
#include "audio/audio_renderer_api.h"
#ifdef CONFIG_EXAMPLES_AUDIO_OSCILLATOR_USEPOSTPROC
#include "userproc_command.h"
#endif

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Section number of memory layout to use */

#define AUDIO_SECTION   SECTION_NO0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For share memory. */

static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool receive_object_reply(void)
{
  AudioObjReply reply_info;

  AS_ReceiveObjectReply(MSGQ_AUD_APP, &reply_info);

  return true;
}

/* ------------------------------------------------------------------------ */
static bool create_audio_sub_system(bool is_enable = true)
{
  bool result = true;

  /* Create mixer feature. */

  AsCreateOutputMixParams_t mix_param =
  {
    .msgq_id =
    {
      .mixer                   = MSGQ_AUD_OUTPUT_MIX,
      .mng                     = MSGQ_AUD_APP,
      .render_path0_filter_dsp = MSGQ_AUD_PFDSP0,
      .render_path1_filter_dsp = MSGQ_AUD_PFDSP1,
    },
    .pool_id =
    {
      .render_path0_filter_pcm = S0_PF0_PCM_BUF_POOL,
      .render_path1_filter_pcm = S0_PF1_PCM_BUF_POOL,
      .render_path0_filter_dsp = S0_PF0_APU_CMD_POOL,
      .render_path1_filter_dsp = S0_PF1_APU_CMD_POOL,
    },
  };

  result = AS_CreateOutputMixer(&mix_param, NULL);

  if (!result)
    {
      printf("Error: AS_CreateOutputMixer() failed. system memory insufficient!\n");
      return false;
    }

  /* Create renderer feature. */

  AsCreateRendererParam_t rend_param =
  {
    .msgq_id =
    {
      .dev0_req  = MSGQ_AUD_RND_PLY0,
      .dev0_sync = MSGQ_AUD_RND_PLY0_SYNC,
      .dev1_req  = MSGQ_AUD_RND_PLY1,
      .dev1_sync = MSGQ_AUD_RND_PLY1_SYNC,
    },
  };

  result = AS_CreateRenderer(&rend_param);

  if (!result)
    {
      printf("Error: AS_CreateRenderer() failure. system memory insufficient!\n");
      return false;
    }

  return result;
}

/* ------------------------------------------------------------------------ */
static void deact_audio_sub_system(void)
{
  /* The following delete process is not executed when it is not initialized */

  AS_DeleteOutputMix();         /* Delete OutputMixer. */

  AS_DeleteRenderer();          /* Delete Renderer. */

  AS_DeleteMediaSynthesizer();  /* Delete Oscillator. */
}

/* ------------------------------------------------------------------------ */
static bool activate_baseband(void)
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

  AsActivateOutputMixer mixer_act =
  {
    .output_device = HPOutputDevice,
    .mixer_type    = MainOnly,
#ifdef CONFIG_EXAMPLES_AUDIO_OSCILLATOR_USEPOSTPROC
    .post_enable   = PostFilterEnable,
#else
    .post_enable   = PostFilterDisable,
#endif
    .cb            = NULL,
  };

  AS_ActivateOutputMixer(OutputMixer0, &mixer_act);

  if (!receive_object_reply())
    {
      return false;
    }

  AS_ActivateOutputMixer(OutputMixer1, &mixer_act);

  return receive_object_reply();
}

/* ------------------------------------------------------------------------ */
static bool deactivate_baseband(void)
{
  /* Deactivate OutputMixer */

  AsDeactivateOutputMixer mixer_deact;

  AS_DeactivateOutputMixer(OutputMixer0, &mixer_deact);

  if (!receive_object_reply())
    {
      printf("AS_DeactivateOutputMixer(0) error!\n");
    }

  AS_DeactivateOutputMixer(OutputMixer1, &mixer_deact);

  if (!receive_object_reply())
    {
      printf("AS_DeactivateOutputMixer(1) error!\n");
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
static bool set_clkmode(void)
{
  CXD56_AUDIO_ECODE error_code;

  error_code = cxd56_audio_set_clkmode(CXD56_AUDIO_CLKMODE_NORMAL);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_clkmode() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

/* ------------------------------------------------------------------------ */
static bool init_libraries(void)
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

  /* Create static memory pool of VoiceCall. */

  const uint8_t sec_no = AUDIO_SECTION;
  const NumLayout layout_no = MEM_LAYOUT_OSCILLATOR;
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

/* ------------------------------------------------------------------------ */
static bool finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools(AUDIO_SECTION);

  /* Finalize memory manager. */

  MemMgrLite::Manager::finalize();

  /* Destroy shared memory. */

  int ret = mpshm_detach(&s_shm);

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
 * public Functions
 ****************************************************************************/

bool app_initialize(void)
{
  bool  ret = false;

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
    }

  /* Next, Create the features used by AudioSubSystem. */

  else if (!create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");
    }

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  else if (!activate_baseband())
    {
      printf("Error: activate_baseband() failure.\n");
    }

  /* Set audio clock mode. */

  else if (!set_clkmode())
    {
      printf("Error: set_clkmode() failure.\n");
    }
  else
    {
      /* Complete! */

      ret = true;
    }

  return ret;
}

/* ------------------------------------------------------------------------ */
void app_finalize(void)
{
  /* Deactivate baseband */

  if (!deactivate_baseband())
    {
      printf("Error: deactivate_baseband() failure.\n");
    }

  /* Deactivate the features used by AudioSubSystem. */

  deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
    }
}

/* ------------------------------------------------------------------------ */
bool app_set_volume(int master_db)
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
bool app_init_postproc(uint8_t  channel_num,
                       uint8_t  bit_width,
                       uint32_t sampling_rate)
{
#ifdef CONFIG_EXAMPLES_AUDIO_OSCILLATOR_USEPOSTPROC

  AsInitPostProc  init;
  InitParam       initpostcmd;

  initpostcmd.input_channel_num = channel_num;

  init.addr = reinterpret_cast<uint8_t *>(&initpostcmd);
  init.size = sizeof(initpostcmd);

  AS_InitPostprocOutputMixer(OutputMixer0, &init);

  if (!receive_object_reply())
    {
      return false;
    }

  AS_InitPostprocOutputMixer(OutputMixer1, &init);

  return receive_object_reply();

#else
  return true;
#endif
}
