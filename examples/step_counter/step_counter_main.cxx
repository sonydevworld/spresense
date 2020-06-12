/****************************************************************************
 * step_counter/step_counter_main.cxx
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

#include <nuttx/config.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <asmp/mpshm.h>

#ifndef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
#  include <arch/board/board.h>
#endif

#include "accel_sensor.h"
#include "gnss_sensor.h"
#include "sensing/sensor_api.h"
#include "sensing/logical_sensor/step_counter.h"
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"
#include "include/pool_layout.h"
#include "include/msgq_pool.h"
#include "include/fixed_fence.h"

/* Section number of memory layout to use */

#define SENSOR_SECTION   SECTION_NO0

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_STEP_COUNTER_WALKING_STRIDE
#  define CONFIG_EXAMPLES_STEP_COUNTER_WALKING_STRIDE 60
#endif
#ifndef CONFIG_EXAMPLES_STEP_COUNTER_RUNNING_STRIDE
#  define CONFIG_EXAMPLES_STEP_COUNTER_RUNNING_STRIDE 80
#endif

#define ACCEL_DATA_BUFFER_NUM  4

#define EXIT_REQUEST_KEY       0x20 /* space key */

#define message(format, ...)    printf(format, ##__VA_ARGS__)
#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
static FAR GnssSensor *sp_gnss_sensor = NULL;
#endif
static FAR StepCounterClass *sp_step_counter_ins = NULL;
static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool sensor_init_libraries(void)
{
  int ret;
  uint32_t addr = SHM_SRAM_ADDR;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, SHM_SRAM_SIZE);
  if (ret < 0)
    {
      err("Error: mpshm_init() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      err("Error: mpshm_remap() failure. %d\n", ret);
      return false;
    }

  /* Initalize MessageLib. */

  err_t err = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  if (err != ERR_OK)
    {
      err("Error: MsgLib::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      err("Error: MsgLib::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (err != ERR_OK)
    {
      err("Error: Manager::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = Manager::initPerCpu(mml_data_area, static_pools, pool_num, layout_no);
  if (err != ERR_OK)
    {
      err("Error: Manager::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  /* Create static memory pool of VoiceCall. */

  const uint8_t sec_no      = SECTION_NO0;
  const NumLayout layout_no = 0;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[sec_no][layout_no][0];
  err = Manager::createStaticPools(sec_no,
                                   layout_no,
                                   work_va,
                                   S0_MEMMGR_WORK_AREA_SIZE,
                                   ptr);
  if (err != ERR_OK)
    {
      err("Error: Manager::createStaticPools() failure. %x\n", err);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
static bool sensor_finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools(SENSOR_SECTION);

  /* Finalize memory manager. */

  MemMgrLite::Manager::finalize();

  /* Destroy shared memory. */

  int ret;
  ret = mpshm_detach(&s_shm);
  if (ret < 0)
    {
      err("Error: mpshm_detach() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_destroy(&s_shm);
  if (ret < 0)
    {
      err("Error: mpshm_destroy() failure. %d\n", ret);
      return false;
    }

  return true;
}

/****************************************************************************
 * Callback Function
 ****************************************************************************/

static int accel_read_callback(uint32_t ev_type,
                               uint32_t timestamp,
                               MemMgrLite::MemHandle &mh)
{
  sensor_command_data_mh_t packet;
  packet.header.size = 0;
  packet.header.code = SendData;
  packet.self        = accelID;
  packet.time        = timestamp;
  packet.fs          = ACCEL_SAMPLING_FREQUENCY;
  packet.size        = ACCEL_WATERMARK_NUM;
  packet.mh          = mh;

  SS_SendSensorDataMH(&packet);

  return 0;
}

/*--------------------------------------------------------------------------*/
#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
static int gnss_read_callback(uint32_t context, FAR GnssSampleData *pos)
{
  MemMgrLite::MemHandle mh;
  if (ERR_OK != mh.allocSeg(GNSS_DATA_BUF_POOL, sizeof(GnssSampleData)))
    {
      ASSERT(0);
    }

  FAR GnssSampleData *data_top = NULL;
  data_top = static_cast<GnssSampleData*>(mh.getVa());
  *data_top = *pos;

  sensor_command_data_mh_t packet;
  packet.header.size = 0;
  packet.header.code = SendData;
  packet.self        = gnssID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 1;
  packet.mh          = mh;

  SS_SendSensorDataMH(&packet);

  return 0;
}
#endif

/*--------------------------------------------------------------------------*/
#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
static int gnss_sensor_entry(int argc,  const char* argv[])
{
  GnssSensorStartSensing(sp_gnss_sensor);
  GnssSensorDestroy(sp_gnss_sensor);
  sp_gnss_sensor = NULL;
  return 0;
}
#endif

/*--------------------------------------------------------------------------*/
bool step_counter_receive_data(sensor_command_data_mh_t& data)
{
  StepCounterWrite(sp_step_counter_ins, &data);

  return true;
}

/*--------------------------------------------------------------------------*/
bool step_counter_recieve_result(sensor_command_data_mh_t& data)
{
  bool ret = true;
  FAR SensorResultStepCounter *result =
    reinterpret_cast<SensorResultStepCounter *>(data.mh.getVa());
  if (SensorOK == result->exec_result)
    {
#ifndef CONFIG_CPUFREQ_RELEASE_LOCK
      float  tempo = 0;

      switch (result->steps.movement_type)
        {
          case STEP_COUNTER_MOVEMENT_TYPE_WALK:
          case STEP_COUNTER_MOVEMENT_TYPE_RUN:

            /* Tempo values are valid for walking and running only. */

            tempo = result->steps.tempo;
            break;
          default:
            break;
        }

      message("%11.5f,%11.2f,%11.5f,%11.2f,%11lld,",
        tempo,
        result->steps.stride,
        result->steps.speed,
        result->steps.distance,
        result->steps.time_stamp
      );
#endif
      message("   %8ld,", result->steps.step);
      switch (result->steps.movement_type)
      {
      case STEP_COUNTER_MOVEMENT_TYPE_STILL:
        message("   stopping\r");
        break;
      case STEP_COUNTER_MOVEMENT_TYPE_WALK:
        message("   walking \r");
        break;
      case STEP_COUNTER_MOVEMENT_TYPE_RUN:
        message("   running \r");
        break;
      default:
        message("   UNKNOWN \r");
        break;
      }
    }
  else
    {
      ret = false;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
static void sensor_manager_api_response(unsigned int code,
                                        unsigned int ercd,
                                        unsigned int self)
{
  if (ercd != SS_ECODE_OK)
    {
      err("Error: get api response. code %d, ercd %d, self %d\n",
          code, ercd, self);
    }

  return;
}

/****************************************************************************
 * sensor_main
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  int ret;

  /* Initialize the shared memory and memory utility used by sensing. */

  sensor_init_libraries();

  /* Activate sensing feature. */

  if (!SS_ActivateSensorSubSystem(MSGQ_SEN_MGR, sensor_manager_api_response))
    {
      err("Error: SS_ActivateSensorSubSystem() failure.\n");
      return EXIT_FAILURE;
    }

  /* Setup physical sensor. */

  FAR physical_sensor_t *sensor = AccelSensorCreate(accel_read_callback);
  if (sensor == NULL)
    {
      err("Error: AccelSensorCreate() failure.\n");
      return EXIT_FAILURE;
    }

#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
  ret = GnssSensorCreate(&sp_gnss_sensor);
  if (ret < 0)
    {
      err("Error: GnssSensorCreate() failure.\n");
      return EXIT_FAILURE;
    }

  uint32_t context = 0;
  ret = GnssSensorRegisterHandler(sp_gnss_sensor,
                                  gnss_read_callback,
                                  context);
  if (ret < 0)
    {
      err("Error: GnssSensorRegisterHandler() failure.\n");
      return EXIT_FAILURE;
    }
#endif

  /* Resister sensor clients. */

  sensor_command_register_t reg;

  reg.header.size   = 0;
  reg.header.code   = ResisterClient;
  reg.self          = accelID;
  reg.subscriptions = 0; 
  reg.callback      = NULL;
  reg.callback_mh   = NULL;
  SS_SendSensorResister(&reg);

  reg.header.size   = 0;
  reg.header.code   = ResisterClient;
  reg.self          = gnssID;
  reg.subscriptions = 0; 
  reg.callback      = NULL;
  reg.callback_mh   = NULL;
  SS_SendSensorResister(&reg);

  reg.header.size   = 0;
  reg.header.code   = ResisterClient;
  reg.self          = stepcounterID;
  reg.subscriptions = (0x01 << accelID) | (0x01 << gnssID);
  reg.callback      = NULL;
  reg.callback_mh   = &step_counter_receive_data;
  SS_SendSensorResister(&reg);

  reg.header.size   = 0;
  reg.header.code   = ResisterClient;
  reg.self          = app0ID;
  reg.subscriptions = (0x01 << stepcounterID);
  reg.callback      = NULL;
  reg.callback_mh   = &step_counter_recieve_result;
  SS_SendSensorResister(&reg);

  /* Setup logical sensor. */

  sp_step_counter_ins = StepCounterCreate(S0_SENSOR_DSP_CMD_BUF_POOL);
  if (NULL == sp_step_counter_ins)
    {
      err("Error: StepCounterCreate() failure.\n");
      return EXIT_FAILURE;
    }

  ret = StepCounterOpen(sp_step_counter_ins);
  if (ret != SS_ECODE_OK)
    {
      err("Error: StepCounterOpen() failure. error = %d\n", ret);
      return EXIT_FAILURE;
    }

  /* Setup Stride setting.
   * The range of configurable stride lenght is 1 - 249[cm].
   * For the mode, set STEP_COUNTER_MODE_FIXED_LENGTH fixed.
   */

  StepCounterSetting set;
  set.walking.step_length = CONFIG_EXAMPLES_STEP_COUNTER_WALKING_STRIDE;
  set.walking.step_mode   = STEP_COUNTER_MODE_FIXED_LENGTH;
  set.running.step_length = CONFIG_EXAMPLES_STEP_COUNTER_RUNNING_STRIDE;
  set.running.step_mode   = STEP_COUNTER_MODE_FIXED_LENGTH;
  ret = StepCounterSet(sp_step_counter_ins, &set);
  if (ret != SS_ECODE_OK)
    {
      err("Error: StepCounterSet() failure. error = %d\n", ret);
      return EXIT_FAILURE;
    }

#if defined(CONFIG_CPUFREQ_RELEASE_LOCK) && \
    !defined(CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS)
  /* After here,
   * because this example program doesn't access to flash
   * and doesn't use TCXO,
   * it turn the flash and TCXO power off to reduce power consumption.
   */

  board_xtal_power_control(false);
  board_flash_power_control(false);
#endif

  message("start sensoring...\n");

  /* Start physical sensor process. */

  ret = AccelSensorOpen(sensor);
  if (ret < 0)
    {
      err("Error: AccelSensorOpen() failure.\n");
      return EXIT_FAILURE;
    }

  ret = AccelSensorStart(sensor);
  if (ret < 0)
    {
      err("Error: AccelSensorStart() failure.\n");
      return EXIT_FAILURE;
    }

#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
  task_create("gnss_sensoring",
              110,
              2048,
              (main_t)gnss_sensor_entry,
              (FAR char * const *)NULL);
#endif

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  message("-----------------------\n");
  message("       step,  move-type\n");
#else
  message("-------------------------------------------------------------------------------------\n");
  message("      tempo,     stride,      speed,   distance,    t-stamp,       step,  move-type\n");
#endif

  while(1)
    {
      /* Wait exit request. */

      if (fgetc(stdin) == EXIT_REQUEST_KEY)
        {
          break;
        }

      usleep(200 * 1000);
    }

  /* Stop physical sensor. */

  ret = AccelSensorStop(sensor);
  if (ret < 0)
    {
      err("Error: AccelSensorStop() failure.\n");
      return EXIT_FAILURE;
    }

  ret = AccelSensorClose(sensor);
  if (ret < 0)
    {
      err("Error: AccelSensorClose() failure.\n");
      return EXIT_FAILURE;
    }

#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
  GnssSensorStopSensing(sp_gnss_sensor);
#endif

  /* Release sensor clients. */

  sensor_command_release_t rel;

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = app0ID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = stepcounterID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = accelID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = gnssID;
  SS_SendSensorRelease(&rel);

#if defined(CONFIG_CPUFREQ_RELEASE_LOCK) && \
    !defined(CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS)
  /* Turn flash and TCXO power on */

  board_xtal_power_control(true);
  board_flash_power_control(true);
#endif

  /* Close logical sensor. */

  if (SS_ECODE_OK != StepCounterClose(sp_step_counter_ins))
    {
      err("Error: StepCounterOpen() failure.\n");
      return EXIT_FAILURE;
    }

  /* Deactivate sensing feature. */

  if (!SS_DeactivateSensorSubSystem())
    {
      err("Error: SS_DeactivateSensorSubSystem() failure.\n");
      return EXIT_FAILURE;
    }

  /* Destroy physical sensor. */

  AccelSensorDestroy(sensor);

  /* finalize the shared memory and memory utility used by sensing. */

  sensor_finalize_libraries();

  return EXIT_SUCCESS;
}
