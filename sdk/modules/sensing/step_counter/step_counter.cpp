/****************************************************************************
 * modules/sensing/step_counter/step_counter.cpp
 *
 *   Copyright 2018,2021 Sony Semiconductor Solutions Corporation
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
#include <debug.h>
#include <sys/time.h>

#include "sensing/logical_sensor/step_counter.h"
#include "sensing/logical_sensor/step_counter_command.h"
#include "dsp_sensor_version.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Step counter debug feature. */

#ifdef CONFIG_SENSING_STEPCOUNTER_DEBUG_ERROR
#  define sc_err(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define sc_err(fmt, ...)
#endif
#ifdef CONFIG_SENSING_STEPCOUNTER_DEBUG_INFO
#  define sc_info(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define sc_info(fmt, ...)
#endif

#define STEPCOUNTER_MQ_ID   1
#define DSP_BOOTED_CMD_ID   1
#define STEPCOUNTER_CMD_ID  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct step_counter_initairize_s
{
  unsigned int    type;
  unsigned int    walking_meter;
  unsigned int    running_meter;
};

struct step_counter_data_s
{
  unsigned int type : 8;
  unsigned int time : 24;

  unsigned int num : 16;
  unsigned int fs  : 16;

  FAR unsigned int *adr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern void SS_SendSensorData(FAR sensor_command_data_t *packet);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *receiver_thread_entry(FAR void *p_instance)
{
  do {
    ((FAR StepCounterClass *)p_instance)->receive();
  } while(1);
  
  return 0;
}

/* Step Counter Class */

int StepCounterClass::open(void)
{
  int errout_ret;
  int ret;
  int id;
  uint32_t msgdata = 0;

  /* Initalize Worker as task. */

  ret = mptask_init_secure(&m_mptask, "AESM");

  if (ret != 0)
    {
      sc_err("mptask_init() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  ret = mptask_assign(&m_mptask);

  if (ret != 0)
    {
      sc_err("mptask_asign() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  /* Queue for communication between Supervisor and Worker create. */

  ret = mpmq_init(&m_mq, STEPCOUNTER_MQ_ID, mptask_getcpuid(&m_mptask));
  if (ret < 0)
    {
      sc_err("mpmq_init() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto sc_errout_with_mptask_destroy;
    }

  /* Release subcore. */

  ret = mptask_exec(&m_mptask);
  if (ret != 0)
    {
      sc_err("mptask_exec() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto sc_errout_with_mpmq_destory;
    }

  /* Wait boot response event */

  id = mpmq_receive(&m_mq, &msgdata);
  if (id != DSP_BOOTED_CMD_ID)
    {
      sc_err("boot error! %d\n", id);
      errout_ret = SS_ECODE_DSP_BOOT_ERROR;
      goto sc_errout_with_mpmq_destory;
    }
  if (msgdata != DSP_STEP_COUNTER_VERSION)
    {
      sc_err("boot error! [dsp version:0x%x] [sensorutils version:0x%x]\n",
        msgdata, DSP_STEP_COUNTER_VERSION);
      errout_ret = SS_ECODE_DSP_VERSION_ERROR;
      goto sc_errout_with_mpmq_destory;
    }
  
  /* Send InitEvent and wait response. */

  ret = this->sendInit();
  if (ret != SS_ECODE_OK)
    {
      errout_ret = ret;
      goto sc_errout_with_mpmq_destory;
    }

  /* Create receive tread */

  ret = pthread_create(&m_thread_id, NULL,
                       receiver_thread_entry,
                       static_cast<pthread_addr_t>(this));
  if (ret != 0)
    {
      sc_err("Failed to create receiver_thread_entry, error=%d\n", ret);
      errout_ret = SS_ECODE_TASK_CREATE_ERROR;
    }
  else
    {
      return SS_ECODE_OK;
    }

sc_errout_with_mpmq_destory:
  ret = mpmq_destroy(&m_mq);
  DEBUGASSERT(ret == 0);

sc_errout_with_mptask_destroy:
  ret = mptask_destroy(&m_mptask, false, NULL);
  DEBUGASSERT(ret == 0);

  return errout_ret;
}

/*--------------------------------------------------------------------------*/
int StepCounterClass::close(void)
{
  int wret = -1;
  int ret = mptask_destroy(&m_mptask, false, &wret);
  if (ret < 0)
    {
      sc_err("mptask_destroy() failure. %d\n", ret);
      return SS_ECODE_DSP_UNLOAD_ERROR;
    }

  sc_info("Worker exit status = %d\n", wret);

  pthread_cancel(this->m_thread_id);
  pthread_join(this->m_thread_id, NULL);

  /* Finalize all of MP objects */

  mpmq_destroy(&m_mq);

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
int StepCounterClass::sendInit(void)
{
  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_cmd_pool_id, sizeof(SensorCmdStepCounter)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  FAR SensorCmdStepCounter *dsp_cmd = (FAR SensorCmdStepCounter *)mh.getPa();

  /* Apply default step setting to command. */

  FAR StepCounterSetParam *w_step;

  w_step = &dsp_cmd->init_cmd.setting.walking;
  w_step->step_length = STEP_COUNTER_INITIAL_WALK_STEP_LENGTH;
  w_step->step_mode   = STEP_COUNTER_MODE_FIXED_LENGTH;

  FAR StepCounterSetParam *r_step;

  r_step = &dsp_cmd->init_cmd.setting.running;
  r_step->step_length = STEP_COUNTER_INITIAL_RUN_STEP_LENGTH;
  r_step->step_mode   = STEP_COUNTER_MODE_FIXED_LENGTH;

  /* Disable debug feature. */

  FAR StepCounterDebugDumpInfo *debug_info;

  debug_info = &dsp_cmd->init_cmd.debug_dump_info;
  debug_info->addr = NULL;
  debug_info->size = 0;

  /* Send initialize command to DSP. */

  dsp_cmd->header.sensor_type = StepCounter;
  dsp_cmd->header.event_type  = InitEvent;

  int ret = mpmq_send(&m_mq, (StepCounterMode << 4) + (InitEvent << 1),
                      reinterpret_cast<int32_t>(mh.getPa()));
  if (ret < 0)
    {
      sc_err("mpmq_send() failure. %d\n", ret);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  /* Wait for initialize finished. */


  FAR SensorCmdStepCounter *msgdata = NULL;

  int id = mpmq_receive(&m_mq, (uint32_t *)&msgdata);
  if (id != STEPCOUNTER_CMD_ID)
    {
      if (msgdata == NULL)
        {
          sc_err("init error! %08x\n", id);
          return SS_ECODE_DSP_INIT_ERROR;
        }

      if (msgdata->result.exec_result != SensorOK)
        {
          sc_err("init error! %08x : %d\n",
                 id,
                 msgdata->result.exec_result);
          return SS_ECODE_DSP_INIT_ERROR;
        }
    }

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
int StepCounterClass::write(FAR sensor_command_data_mh_t *command)
{
  struct exe_mh_s exe_mh;

  /* copy memhandle of data */

  exe_mh.data = command->mh;

  /* allocate segment of command */

  if (exe_mh.cmd.allocSeg(m_cmd_pool_id, sizeof(SensorCmdStepCounter))
      != ERR_OK)
    {
      sc_err("allocSeg() failure.¥n");
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  FAR SensorCmdStepCounter *dsp_cmd  =
    (FAR SensorCmdStepCounter *)exe_mh.cmd.getPa();
  FAR SensorExecStepCounter *exec_prm = &dsp_cmd->exec_cmd;

  dsp_cmd->header.sensor_type = StepCounter;
  dsp_cmd->header.event_type  = ExecEvent;

  /* Select sensor type by incoming sensor. */

  switch (command->self)
    {
      case accelID:
        {
          exec_prm->cmd_type                 =
            STEP_COUNTER_CMD_UPDATE_ACCELERATION;
          exec_prm->update_acc.time_stamp    = command->time;
          exec_prm->update_acc.sampling_rate = command->fs;
          exec_prm->update_acc.sample_num    = command->size;
          exec_prm->update_acc.p_data        =
            reinterpret_cast<FAR ThreeAxisSample *>(exe_mh.data.getPa());
        }
        break;

      case gnssID:
        {
          exec_prm->cmd_type = STEP_COUNTER_CMD_UPDATE_GPS;
          exec_prm->update_gps = *(reinterpret_cast<FAR GnssSampleData *>
                                    (exe_mh.data.getPa()));
        }
        break;

      default:
        {
          exec_prm->cmd_type = STEP_COUNTER_CMD_UPDATE_ACCELERATION;
        }
        break;
    }

  if (!m_exe_que.push(exe_mh))
    {
      sc_err("m_exe_que.push() failure.¥n");
      return SS_ECODE_QUEUE_PUSH_ERROR;
    }

  /* Send sensored data.
   * (Data which sent to DSP is physical address of command msg.)
   */

  int ret = mpmq_send(&m_mq,
                      (StepCounterMode << 4) + (ExecEvent << 1),
                      reinterpret_cast<int32_t>(exe_mh.cmd.getPa()));
  if (ret < 0)
    {
      m_exe_que.pop();
      sc_err("mpmq_send() failure. %d\n", ret);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void StepCounterClass::set_callback(void)
{
  struct exe_mh_s           exe_mh = m_exe_que.top();
  sensor_command_data_mh_t  packet;
  SensorCmdStepCounter      cmd_data;

  /* Since the area is destroyed afterward, backup it. */

  memcpy(&cmd_data, exe_mh.cmd.getVa(), sizeof(SensorCmdStepCounter));

  if (cmd_data.header.event_type == ExecEvent &&
     (cmd_data.exec_cmd.cmd_type == STEP_COUNTER_CMD_UPDATE_ACCELERATION ||
      cmd_data.exec_cmd.cmd_type == STEP_COUNTER_CMD_UPDATE_GPS))
    {
      /* Overwrites the contents of the memory area with steps data.
       * 
       * Caution:
       *  Rewrite the memory area. Access in the original type is not possible.
       */

      memcpy(exe_mh.cmd.getVa(), &cmd_data.result, sizeof(SensorResultStepCounter));

      /* Create command. */

      packet.header.code = SendData;
      packet.header.size = sizeof(sensor_command_header_t);
      packet.self        = stepcounterID;
      packet.time        = 0;
      packet.fs          = 0;
      packet.size        = 3;
      packet.mh          = exe_mh.cmd;

      SS_SendSensorDataMH(&packet);
    }
  else
    {
      /* Not update acceleration. */
    }

  /* Pop exec queue (Free segment). */

  m_exe_que.pop();
}

/*--------------------------------------------------------------------------*/
/*!
 * @brief receive result from dsp
 */
int StepCounterClass::receive(void)
{
  int      command;
  uint32_t msgdata = 0;
  bool     active = true;

  /* Wait for worker message */

  while (active)
    {
      command = mpmq_receive(&m_mq, &msgdata);
      if (command < 0)
        {
          sc_err("mpmq_receive() failure. command(%d) < 0\n", command);
          return command;
        }
      this->set_callback();
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int StepCounterClass::set(FAR StepCounterSetting *set_param)
{
  struct exe_mh_s exe_mh;

  /* allocate segment of command */

  int result = exe_mh.cmd.allocSeg(m_cmd_pool_id,
                                   sizeof(SensorCmdStepCounter));

  if (result != ERR_OK)
    {
      sc_err("allocSeg() failure.¥n");
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  FAR SensorCmdStepCounter *dsp_cmd =
    (FAR SensorCmdStepCounter *)exe_mh.cmd.getPa();

  /* Send command. */

  dsp_cmd->header.sensor_type = StepCounter;
  dsp_cmd->header.event_type  = ExecEvent;
  dsp_cmd->exec_cmd.cmd_type = STEP_COUNTER_CMD_STEP_SET;
  dsp_cmd->exec_cmd.setting  = *set_param;

  if (!m_exe_que.push(exe_mh))
    {
      sc_err("m_exe_que.push() failure.¥n");
      return SS_ECODE_QUEUE_PUSH_ERROR;
    }

  /* Send sensored data.
   * Data sent to DSP is physical address of command msg.
   */

  int ret = mpmq_send(&m_mq,
                      (StepCounterMode << 4) + (ExecEvent << 1),
                      reinterpret_cast<int32_t>(exe_mh.cmd.getPa()));

  if (ret < 0)
    {
      m_exe_que.pop();
      sc_err("mpmq_send() failure. %d\n", ret);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  return SS_ECODE_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR StepCounterClass *StepCounterCreate(MemMgrLite::PoolId cmd_pool_id)
{
  return new StepCounterClass(cmd_pool_id);
}

/*--------------------------------------------------------------------------*/
FAR StepCounterClass *StepCounterCreate(uint8_t cmd_pool_id)
{
  MemMgrLite::PoolId pool_id;
  pool_id.sec  = 0;
  pool_id.pool = cmd_pool_id;
  return new StepCounterClass(pool_id);
}

/*--------------------------------------------------------------------------*/
int StepCounterOpen(FAR StepCounterClass *ins)
{
  int ret;

  ret = ins->open();

  return ret;
}

/*--------------------------------------------------------------------------*/
int StepCounterClose(FAR StepCounterClass *ins)
{
  int ret;

  ret = ins->close();
  delete ins;

  return ret; 
}

/*--------------------------------------------------------------------------*/
int StepCounterWrite(FAR StepCounterClass *ins,
                     FAR sensor_command_data_mh_t *command)
{
  int ret;

  ret = ins->write(command);

  return ret;
}

/*--------------------------------------------------------------------------*/
int StepCounterSet(FAR StepCounterClass *ins,
                   FAR StepCounterSetting *set)
{
  int ret;

  ret = ins->set(set);

  return ret;
}
