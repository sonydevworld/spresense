/****************************************************************************
 * transport_mode/transport_mode_main.cxx
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

#include <sdk/config.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <arch/board/board.h>

#include "sensing/logical_sensor/transport_mode.h"
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"
#include "include/mem_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"

#include "sensor_control.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DETECTED_MF_EV  0
#define CHANGE_SCU_EV   1

#define TIME_TO_WAIT_KEY (200 * 1000) /* in microsecond */

/* Error message */

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mpshm_t s_shm;
static mqd_t s_mqfd = (mqd_t)-1;
static pthread_t s_thread_id;

static const char *s_class_strings[] =
{
  "Undetermined",
  "Staying",
  "Walking",
  "Running",
  "Ascending stairs",
  "Descending stairs",
  "Going up on escalator",
  "Going down on escalator",
  "Going up in elevator",
  "Going down in elevator",
  "Getting on train",
  "Getting on bus",
  "Getting in car",
  "Riding bicycle",
};

/****************************************************************************
 * Callback Function
 ****************************************************************************/

static bool app_receive_event(sensor_command_data_t& data)
{
  switch (data.self)
    {
      case accelID:
        {
          /* send message */

          char ev = DETECTED_MF_EV;
          mq_send(s_mqfd, (char *)&ev, sizeof(uint8_t), 10);
        }
        break;

      case tramID:
        {
          uint8_t result_type = get_async_msgtype(data.data);
          uint8_t result_data = get_async_msgparam(data.data);

          if (result_type == TramCmdTypeResult)
            {
              printf("Mode: %s\n", s_class_strings[result_data]);

#ifdef CONFIG_EXAMPLES_SENSOR_TRAM_DETAILED_INFO
              float* likelihood = TramGetLikelihood();
              printf("Likelihood of all modes\n");
              for (uint32_t i = 0; i < TRAM_NUMBER_OF_MODES; i++)
                {
                  printf("[%23s] %.4f\n",
                         s_class_strings[i+1],
                         likelihood[i]);
                }
#endif
            }
          else if (result_type == TramCmdTypeTrans)
            {
              char state;
              switch (result_data)
                {
                case ChangeScuSettings:
                  state = CHANGE_SCU_EV;
                  break;

                default:
                  err("invalide message! %d\n", result_data);
                  break;
                }

              /* Send message */

              mq_send(s_mqfd, (char *)&state, sizeof(uint8_t), 10);
              break;
            }
        }
    }

  return true;
}

/*--------------------------------------------------------------------------*/
static bool app_receive_result(sensor_command_data_mh_t& data)
{
  FAR SensorCmdTram *result_data =
    reinterpret_cast<FAR SensorCmdTram *>(data.mh.getVa());

  if (SensorOK != result_data->result.exec_result)
    {
      err("received error: result[%d], code[%d]\n",
            result_data->result.exec_result,
            result_data->result.assert_info.code);

      return false;
    }

  return true;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void regist_app_callback(void)
{
  sensor_command_register_t reg;

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = app0ID;
  reg.subscriptions     = (0x01 << accelID) | (0x01 << tramID);
  reg.callback          = app_receive_event;
  reg.callback_mh       = app_receive_result;
  reg.callback_pw       = NULL;
  SS_SendSensorResister(&reg);
}

/*--------------------------------------------------------------------------*/
static void release_app(void)
{
  sensor_command_release_t rel;

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = app0ID;
  SS_SendSensorRelease(&rel);
}

/*--------------------------------------------------------------------------*/
static int init_shm(void)
{
  int ret;
  uint32_t addr = SHM_SRAM_ADDR;
  ret = mpshm_init(&s_shm, 1, SHM_SRAM_SIZE);
  if (ret < 0)
    {
      err("mpshm_init() failure. %d\n", ret);
      return -1;
    }
  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      err("mpshm_remap() failure. %d\n", ret);
      return -1;
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
static int del_shm(void)
{
  int ret;

  ret = mpshm_detach(&s_shm);
  if (ret < 0)
    {
      err("mpshm_detach() failure. %d\n", ret);
      return -1;
    }

  ret = mpshm_destroy(&s_shm);
  if (ret < 0)
    {
      err("mpshm_destroy() failure. %d\n", ret);
      return -1;
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
static int init_memutilities(void)
{
  /* Initialize MsgLib */

  err_t ret = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  if (ret != ERR_OK)
    {
      err("MsgLib::initFirst() failure. 0x%x\n", ret);
      return -1;
    }

  ret = MsgLib::initPerCpu();
  if (ret != ERR_OK)
    {
      err("MsgLib::initPerCpu() failure. 0x%x\n", ret);
      return -1;
    }

  /* Initialize MemMgr */

  FAR void *mml_data_area =
    MemMgrLite::translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  ret = MemMgrLite::Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (ret != ERR_OK)
    {
      err("MemMgrLite::Manager::initFirst() failure. 0x%x\n", ret);
      return -1;
    }

  ret = MemMgrLite::Manager::initPerCpu(mml_data_area, NUM_MEM_POOLS);
  if (ret != ERR_OK)
    {
      err("MemMgrLite::Manager::initPerCpu() failure. 0x%x\n", ret);
      return -1;
    }

  const MemMgrLite::NumLayout layout_no = 0;
  FAR void *work_va =
    MemMgrLite::translatePoolAddrToVa(MEMMGR_WORK_AREA_ADDR);
  ret = MemMgrLite::Manager::createStaticPools(
                              layout_no, work_va,
                              MEMMGR_MAX_WORK_SIZE,
                              MemMgrLite::MemoryPoolLayouts[layout_no]);
  if (ret != ERR_OK)
    {
      err("MemMgrLite::Manager::createStaticPools() failure. 0x%x\n", ret);
      return -1;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static void finalize_memutilities(void)
{
  /* Finalize MessageLib */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools();

  /* Finalize memory manager */

  MemMgrLite::Manager::finalize();
}

/*--------------------------------------------------------------------------*/
extern "C" void *TramReceivingEvThread(FAR void *arg)
{
  int ret = 0;
  char ev;

  while (mq_receive(s_mqfd, &ev, sizeof(uint8_t), 0) != 0)
    {
      switch (ev)
        {
          case DETECTED_MF_EV:
            {
              ret = TramSendMathFuncEvent();
              if (ret != 0)
                {
                 err("failed to handle event: ACCEL_MF_EV\n");
                }
            }
            break;

          case CHANGE_SCU_EV:
            {
              ret = TramChangeScuSettings();
              if (ret != 0)
                {
                  err("failed to change SCU settings.\n");
                }
            }
            break;

          default:
            /* Do nothing */

            break;
        }
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static int open_message(void)
{
  pthread_attr_t attr;
  struct sched_param sch_param;
  struct mq_attr mqueue_attr;

  /* Fill in attributes for message queue */

  mqueue_attr.mq_maxmsg  = 20;
  mqueue_attr.mq_msgsize = 1;
  mqueue_attr.mq_flags   = 0;

  s_mqfd = mq_open("tram_mqueue", O_RDWR|O_CREAT, 0666, &mqueue_attr);
  if (s_mqfd < 0)
    {
      err("failed by mq_open(O_RDWR)\n");
      return EXIT_FAILURE;
    }

  (void)pthread_attr_init(&attr);
  sch_param.sched_priority = 110;
  pthread_attr_setschedparam(&attr, &sch_param);

  pthread_create(&s_thread_id, &attr, TramReceivingEvThread, (void *)NULL);

  return 0;
}

/*--------------------------------------------------------------------------*/
static void close_message(void)
{
  void *value;
  pthread_cancel(s_thread_id);
  pthread_join(s_thread_id, &value);

  /* Close mq fd */

  mq_close(s_mqfd);
}

/*--------------------------------------------------------------------------*/
static void sensor_manager_api_response(unsigned int code,
                                        unsigned int ercd,
                                        unsigned int self)
{
  if (ercd != 0)
    {
      err("Receive SensorSubSystem's response. code %d, ercd %d, self %d\n", code, ercd, self);
    }

  return;
}

/****************************************************************************
 * sensor_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
extern "C" int transport_mode_main(int argc, char *argv[])
#endif
{
  int ret;
  char ch = 0;

  /* Initialize shared memory */

  ret = init_shm();
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  /* Initialize memory utilities */

  ret = init_memutilities();
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  /* Open receiving mq */

  ret = open_message();
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  /* Activate sensor manager */

  if (!SS_ActivateSensorSubSystem(MSGQ_SEN_MGR, sensor_manager_api_response))
    {
      err("SS_ActivateSensorSubSystem() failure.\n");
      return EXIT_FAILURE;
    }

  /* Open transport_mode state transition */

  ret = TramOpenSensors(SENSOR_DSP_CMD_BUF_POOL);
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  /* After here,
   * because this example program doesn't access
   * to flash and doesn't use TCXO,
   * it turn the flash and TCXO power off to reduce power consumption.
   */

  board_xtal_power_control(false);
  board_flash_power_control(false);

  /* Regist application callback */

  regist_app_callback();

  /* Start sensing */

  printf("Start sensing...\n");

  TramStartSensors();

  /* Wait stop command */

  while(1)
    {
      ch = fgetc(stdin);
      if (ch == 0x20)
        {
          printf("Stop sensoring.\n");

          break;
        }

      usleep(TIME_TO_WAIT_KEY);
    }

  /* Stop transport_mode */

  TramStopSensors();

  /* Wait until sensors stop */

  sleep(1);

  release_app();

  /* Close sensors */

  TramCloseSensors();

  /* Deactivate sensor manager */

  if (!SS_DeactivateSensorSubSystem())
    {
      err("SS_DeactivateSensorSubSystem() failure.\n");
      return EXIT_FAILURE;
    }

  /* Finalize memory utilities */

  finalize_memutilities();

  /* Destory share memory */

  del_shm();

  /* Close receiving mq */

  close_message();

  /* Turn the flash and TCXO power on */

  board_xtal_power_control(true);
  board_flash_power_control(true);

  printf("transport_mode example finished\n");

  return EXIT_SUCCESS;
}
