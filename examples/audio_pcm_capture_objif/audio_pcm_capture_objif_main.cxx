/****************************************************************************
 * audio_pcm_capture_objif/pcm_capture_objif_main.cxx
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
#include "audio/audio_frontend_api.h"
#include "audio/audio_capture_api.h"
#include "audio/audio_message_types.h"
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

/* Size of one block to read from the FIFO
 * Set the maximum size of configurable parameters.
 */

#define SIMPLE_FIFO_READ_SIZE (768 * 4 * 4)  /* 768sample, 4ch, 24bit */

/* Number of FIFO stages */

#define SIMPLE_FIFO_FRAME_NUM (30)

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
  /* Sample code
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

/* PCM FIFO buffer size */

#define SIMPLE_FIFO_BUF_SIZE  (SIMPLE_FIFO_READ_SIZE * SIMPLE_FIFO_FRAME_NUM)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* For FIFO. */

struct fifo_info_s
{
  CMN_SimpleFifoHandle  handle;
  uint32_t              fifo_area[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
  uint8_t               write_buf[SIMPLE_FIFO_READ_SIZE];
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

  return true;
}

static void app_pop_simple_fifo(bool is_end_process = false)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&s_fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > SIMPLE_FIFO_READ_SIZE) ?
        SIMPLE_FIFO_READ_SIZE : occupied_simple_fifo_size;
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

static bool app_create_audio_sub_system(void)
{
  bool result = false;

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MGR;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_INPUT_BUF_POOL;
  frontend_create_param.pool_id.output      = S0_NULL_POOL;
  frontend_create_param.pool_id.dsp         = S0_PRE_APU_CMD_POOL;

  AS_CreateMicFrontend(&frontend_create_param, NULL);

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

  /* Disable input */

  if (cxd56_audio_dis_input() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  return true;
}

static bool app_init_mic_gain(void)
{
  cxd56_audio_mic_gain_t  mic_gain;

  mic_gain.gain[0] = 0;
  mic_gain.gain[1] = 0;
  mic_gain.gain[2] = 0;
  mic_gain.gain[3] = 0;
  mic_gain.gain[4] = 0;
  mic_gain.gain[5] = 0;
  mic_gain.gain[6] = 0;
  mic_gain.gain[7] = 0;

  return (cxd56_audio_set_micgain(&mic_gain) == CXD56_AUDIO_ECODE_OK);
}

static bool app_set_frontend(void)
{
  /* Enable input. */

  if (cxd56_audio_en_input() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  /* Set frontend parameter */

  AsActivateMicFrontend act_param;

  act_param.param.input_device = AsMicFrontendDeviceMic;
  act_param.cb                 = NULL;

  AS_ActivateMicFrontend(&act_param);

  if (!app_receive_object_reply(MSG_AUD_MFE_CMD_ACT))
    {
      return false;
    }

  return true;
}

static bool app_init_frontend(void)
{
  /* Set frontend init parameter */

  AsInitMicFrontendParam  init_param;

  init_param.channel_number           = target_channel_number;
  init_param.bit_length               = target_bit_lengt;
  init_param.samples_per_frame        = getCapSampleNumPerFrame(AS_CODECTYPE_LPCM,
                                                                target_samplingrate);
  init_param.preproc_type             = AsMicFrontendPreProcThrough;
  init_param.data_path                = AsDataPathSimpleFIFO;
  init_param.dest.simple_fifo_handler = &s_fifo.handle;

  AS_InitMicFrontend(&init_param);

  return app_receive_object_reply(MSG_AUD_MFE_CMD_INIT);
}

static bool app_start_capture(void)
{
  CMN_SimpleFifoClear(&s_fifo.handle);

  AsStartMicFrontendParam cmd;

  AS_StartMicFrontend(&cmd);

  return app_receive_object_reply(MSG_AUD_MFE_CMD_START);
}

static bool app_stop_capture(void)
{
  AsStopMicFrontendParam  cmd;

  cmd.stop_mode = 0;

  AS_StopMicFrontend(&cmd);

  if (!app_receive_object_reply())
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

  return (cxd56_audio_set_clkmode(clk_mode) == CXD56_AUDIO_ECODE_OK);
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
  bool  res = true;

  /* Set default parameter. */

  target_samplingrate   = SAMPLINGRATE;
  target_channel_number = CHANNEL_NUMBER;
  target_bit_lengt      = BIT_LENGTH;
  target_recording_time = PCM_CAPTURE_TIME;

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
    {1, AS_SAMPLINGRATE_48000,  "48kHz"},
    {1, AS_SAMPLINGRATE_192000, "192kHz"},
    {2, AS_CHANNEL_MONO,        "1ch(MONO)"},
    {2, AS_CHANNEL_STEREO,      "2ch(STEREO)"},
    {2, AS_CHANNEL_4CH,         "4ch"},
    {3, AS_BITLENGTH_16,        "16bit"},
    {3, AS_BITLENGTH_24,        "24bit"},
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
  printf("Sampling rate  %s\n", app_param_to_str(1, target_samplingrate));
  printf("Channel number %s\n", app_param_to_str(2, target_channel_number));
  printf("Bit length     %s\n", app_param_to_str(3, target_bit_lengt));
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

  printf("Start PCM capture with object level i/f example\n");

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

  /* Set frontend operation mode. */

  if (!app_set_frontend())
    {
      printf("Error: app_set_frontend() failure.\n");
      goto errout_init_simple_fifo;
    }

  /* Initialize frontend. */

  if (!app_init_frontend())
    {
      printf("Error: app_init_frontend() failure.\n");
      goto errout_init_frontend;
    }

  /* Start capture operation. */

  if (!app_start_capture())
    {
      printf("Error: app_start_capture() failure.\n");
    }
  else
    {
      /* Running... */

      printf("Running time is %d sec\n", target_recording_time);

      app_recorde_process(target_recording_time);

      /* Stop capture operation. */

      if (!app_stop_capture())
        {
          printf("Error: app_stop_capture() failure.\n");

          /* To end processing */
        }
    }

errout_init_frontend:

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

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

errout_create_audio_sub_system:

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
    }

errout:

  printf("Exit PCM capture with object level i/f example\n\n");

  return 0;
}
