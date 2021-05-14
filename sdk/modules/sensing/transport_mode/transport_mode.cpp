/****************************************************************************
 * modules/sensing/transport_mode/transport_mode.cpp
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
#include <stdio.h>
#include <sys/time.h>
#include <arch/chip/pm.h>
#include <arch/chip/scu.h>

#include "sensing/logical_sensor/transport_mode.h"
#include "sensing/logical_sensor/transport_mode_command.h"
#include "dsp_sensor_version.h"
#include "tram_state_transition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Tramsport mode debug feature. */

#ifdef CONFIG_SENSING_TRAM_DEBUG_ERROR
#  define tram_err(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define tram_err(fmt, ...)
#endif
#ifdef CONFIG_SENSING_TRAM_DEBUG_INFO
#  define tram_info(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define tram_info(fmt, ...)
#endif

#define TRAM_MQ_ID        1
#define DSP_BOOTED_CMD_ID 1
#define TRAM_CMD_ID       2

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct scuev_notify_s s_scuevent =
{
  0, /* Signal number to be set by app */
  ACCEL_TRIGGER_RISE_THRESS,
  ACCEL_TRIGGER_RISE_COUNT0,
  ACCEL_TRIGGER_RISE_COUNT1,
  ACCEL_TRIGGER_RISE_DELAY,
  ACCEL_TRIGGER_FALL_THRESS,
  ACCEL_TRIGGER_FALL_COUNT0,
  ACCEL_TRIGGER_FALL_COUNT1,
  ACCEL_TRIGGER_FALL_DELAY,
  (uint32_t)SCU_EV_OUTALWAYS | SCU_EV_RISE_EN | SCU_EV_FALL_EN,
  NULL, /* Event argument to be set by app */
};

static struct math_filter_s s_filter =
{
  FILTER_POS_EE,
  0,
  0,
  0x183bfaed,
  0x80,
  0x2e8e30e5,
  0x80,
  0xcf8809ef,
  0x80,
  0xed9e44fa,
  0x00,
  0x183bfaed,
  0x80,
  0,
  0,
  0xd64b739,
  0xc0,
  0xf124e7e1,
  0x40,
  0x1ac96ea9,
  0x40,
  0xf9483b02,
  0x40,
  0xd64b739,
  0xc0,
};

struct scufifo_wm_s s_watermark =
{
  0,    /* Signal number to be set by app */
  NULL, /* Timestamp to be set by app */
  ACCEL_WATERMARK_NUM,
};

static struct ScuSettings s_accelSettingsMs =
{
  ACCEL_FIFO_NUM,
  5, /* 16Hz (32768 / 64 / (2 ^ 5) = 16) */
  3,
  &s_scuevent,
  &s_filter,
  NULL,
};

static struct ScuSettings s_accelSettingsCmd =
{
  ACCEL_FIFO_NUM,
  3,  /* 64Hz (32768 / 64 / (2 ^ 3) = 64) */
  3,
  NULL,
  NULL,
  &s_watermark,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
static struct pm_cpu_freqlock_s g_tram_freqlock;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *receiver_thread_entry(FAR void *p_instance)
{
  do
    {
      ((TramClass*)p_instance)->receive();
    } while(1);
  
  return 0;
}

/* Lock HV performance to avoid loading time becomes too long */
#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
/*--------------------------------------------------------------------------*/
static void acquire_freq_lock(void)
{
  g_tram_freqlock.count = 0;
  g_tram_freqlock.info = PM_CPUFREQLOCK_TAG('T', 'R', 0x00);
  g_tram_freqlock.flag = PM_CPUFREQLOCK_FLAG_HV;

  up_pm_acquire_freqlock(&g_tram_freqlock);
}

/*--------------------------------------------------------------------------*/
static void release_freq_lock(void)
{
  up_pm_release_freqlock(&g_tram_freqlock);
}
#endif /* CONFIG_CPUFREQ_RELEASE_LOCK */

/*--------------------------------------------------------------------------*/
int TramClass::open(FAR float *likelihood)
{
  int errout_ret;
  int ret;
  int id;
  uint32_t msgdata; /* 0 is nomal boot. */

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  acquire_freq_lock();
#endif

  /* Initalize Worker as task. */

  ret = mptask_init_secure(&m_mptask, "TRAM");
  if (ret != 0)
    {
      tram_err("mptask_init() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  ret = mptask_assign(&m_mptask);
  if (ret != 0)
    {
      tram_err("mptask_asign() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  /* Queue for communication between Supervisor and Worker create. */

  ret = mpmq_init(&m_mq, TRAM_MQ_ID, mptask_getcpuid(&m_mptask));
  if (ret < 0)
    {
      tram_err("mpmq_init() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto transport_mode_errout_with_mptask_destroy;
    }

  /* Release subcore. */

  ret = mptask_exec(&m_mptask);
  if (ret != 0)
    {
      tram_err("mptask_exec() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto transport_mode_errout_with_mpmq_destory;
    }

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  release_freq_lock();
#endif

  /* Wait boot response event. */

  id = mpmq_receive(&m_mq, &msgdata);
  if (id != DSP_BOOTED_CMD_ID)
    {
      tram_err("boot error! %d\n", id);
      errout_ret = SS_ECODE_DSP_BOOT_ERROR;
      goto transport_mode_errout_with_mpmq_destory;
    }

  if (msgdata != DSP_TRAM_VERSION)
    {
      tram_err("boot error! [dsp version:0x%x] [sensorutils version:0x%x]\n",
               msgdata, DSP_TRAM_VERSION);
      errout_ret = SS_ECODE_DSP_VERSION_ERROR;
      goto transport_mode_errout_with_mpmq_destory;
    }

  /* Send InitEvent and wait response. */

  ret = this->sendInit(likelihood);
  if (ret != SS_ECODE_OK)
    {
      errout_ret = ret;
      goto transport_mode_errout_with_mpmq_destory;
    }

  /* Create receive thread. */

  ret = pthread_create(&m_thread_id,
                       NULL,
                       receiver_thread_entry,
                       static_cast<pthread_addr_t>(this));
  if (ret != 0)
    {
      tram_err("DD_load: Failed to create receiver_thread_entry, error=%d\n",
               ret);
      errout_ret = SS_ECODE_TASK_CREATE_ERROR;
    }
  else
    {
      return SS_ECODE_OK;
    }

transport_mode_errout_with_mpmq_destory:
  ret = mpmq_destroy(&m_mq);
  if (ret < 0)
    {
      tram_err("mpmq_destroy() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_UNLOAD_ERROR;
    }

transport_mode_errout_with_mptask_destroy:
  ret = mptask_destroy(&m_mptask, false, NULL);
  if (ret < 0)
    {
      tram_err("mptask_destroy() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_UNLOAD_ERROR;
    }

  return errout_ret;
}

/*--------------------------------------------------------------------------*/
int TramClass::close(void)
{
  int wret = -1;
  int ret = mptask_destroy(&m_mptask, false, &wret);
  if (ret < 0)
    {
      tram_err("mptask_destroy() failure. %d\n", ret);
      return SS_ECODE_DSP_UNLOAD_ERROR;
    }

  pthread_cancel(this->m_thread_id);
  pthread_join(this->m_thread_id, NULL);

  /* Finalize all of MP objects. */

  ret = mpmq_destroy(&m_mq);
  if (ret < 0)
    {
      tram_err("mpmq_destroy() failure. %d\n", ret);
      return SS_ECODE_DSP_UNLOAD_ERROR;
    }
  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
int TramClass::start(void)
{
  /* Start from MS state. */
  
  TramStateTransitionSetState(this, TRAM_STATE_MS);

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
int TramClass::stop(void)
{
  /* Start from MS state. */
  
  TramStateTransitionSetState(this, TRAM_STATE_UNINITIALIZED);

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
int TramClass::sendInit(float* likelihood)
{
  /* Tentative : Not absolutely necessary to use MemHandle. */

  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_cmd_pool_id, sizeof(SensorCmdTram)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  FAR SensorCmdTram *dsp_cmd = (FAR SensorCmdTram *)mh.getVa();

  dsp_cmd->header.sensor_type = TransportationMode;
  dsp_cmd->header.event_type  = InitEvent;

  dsp_cmd->init_cmd.likelihood = likelihood;

  /* Disable debug feature. */

  TramDebugDumpInfo *debug_info;

  debug_info = &dsp_cmd->init_cmd.debug_dump_info;
  debug_info->addr = NULL;
  debug_info->size = 0;

  int ret = mpmq_send(&m_mq,
                      (TramProcMode << 4) + (InitEvent << 1),
                      reinterpret_cast<int32_t>(mh.getPa()));
  if (ret < 0)
    {
      tram_err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  /* Wait for initialized event. */

  uint32_t msgdata;

  int id = mpmq_receive(&m_mq, &msgdata);
  if ((id != TRAM_CMD_ID) &&
        (reinterpret_cast<FAR SensorCmdTram *>(msgdata)->result.exec_result
          != SensorOK))
    {
      tram_err("init error! %08x : %d\n",
               id,
               reinterpret_cast<FAR SensorCmdTram *>
                (msgdata)->result.exec_result);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
int TramClass::write(sensor_command_data_mh_t* command)
{
  struct exe_mh_s exe_mh;

  /* Copy memhandle of data. */

  exe_mh.data = command->mh;

  /* Allocate segment of command. */

  if (exe_mh.cmd.allocSeg(m_cmd_pool_id, sizeof(SensorCmdTram)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  FAR SensorCmdTram *dsp_cmd = (FAR SensorCmdTram *)exe_mh.cmd.getVa();

  dsp_cmd->header.sensor_type = TransportationMode;
  dsp_cmd->header.event_type  = ExecEvent;

  /* Select sensor type by incoming sensor. */

  switch (command->self)
    {
      case accelID:
        {
          dsp_cmd->exec_cmd.type                   = TramSensorAcc;
          dsp_cmd->exec_cmd.acc_data.time_stamp    = command->time;
          dsp_cmd->exec_cmd.acc_data.sampling_rate = command->fs;
          dsp_cmd->exec_cmd.acc_data.sample_num    = command->size;
          dsp_cmd->exec_cmd.acc_data.p_data        =
            reinterpret_cast<ThreeAxisSample*>(exe_mh.data.getPa());
        }
        break;

      case magID:
        {
          dsp_cmd->exec_cmd.type                   = TramSensorMag;
          dsp_cmd->exec_cmd.mag_data.time_stamp    = command->time;
          dsp_cmd->exec_cmd.mag_data.sampling_rate = command->fs;
          dsp_cmd->exec_cmd.mag_data.sample_num    = command->size;
          dsp_cmd->exec_cmd.mag_data.p_data        =
            reinterpret_cast<ThreeAxisSample*>(exe_mh.data.getPa());
        }
        break;

      case barometerID:
        {
          dsp_cmd->exec_cmd.type                   = TramSensorBar;
          dsp_cmd->exec_cmd.bar_data.time_stamp    = command->time;
          dsp_cmd->exec_cmd.bar_data.sampling_rate = command->fs;;
          dsp_cmd->exec_cmd.bar_data.sample_num    = command->size;
          dsp_cmd->exec_cmd.bar_data.p_data        =
            reinterpret_cast<uint32_t*>(exe_mh.data.getPa());
        }
        break;

      default:
        {
          dsp_cmd->exec_cmd.type = TramSensorAcc;
        }
        break;
    }

  if (!m_exe_que.push(exe_mh))
    {
      /* Cannot save MHandle due to system error. */

      return SS_ECODE_QUEUE_PUSH_ERROR;
    }

  /* Send sensored data.
   * (Data which sent to DSP is physical address of command msg.)
   */

  int ret = mpmq_send(&m_mq,
                      (TramProcMode << 4) + (ExecEvent << 1),
                      reinterpret_cast<int32_t>(exe_mh.cmd.getPa()));
  if (ret < 0)
    {
      tram_err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void TramClass::send_detection_result(uint32_t pred)
{
  sensor_command_data_t packet;

  /* Create command. */

  packet.header.code = SendData;
  packet.header.size = sizeof(sensor_command_header_t);
  packet.self        = tramID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 3;
  packet.is_ptr      = false;
  packet.data        = pred;

  SS_SendSensorData(&packet);
}

/*--------------------------------------------------------------------------*/
void TramClass::send_notification(TramNotification notification)
{
  sensor_command_data_t packet;
  uint32_t data;

  data = (TramCmdTypeTrans << 8) | (uint8_t)notification;

  /* Create command. */

  packet.header.code = SendData;
  packet.header.size = sizeof(sensor_command_header_t);
  packet.self        = tramID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 3;
  packet.is_ptr      = false;
  packet.data        = data;

  SS_SendSensorData(&packet);
}

/*--------------------------------------------------------------------------*/
void TramClass::set_power(uint32_t subscriptions)
{
  sensor_command_power_t packet;

  /* Create command. */

  packet.header.code = SetPower;
  packet.header.size = sizeof(sensor_command_header_t);;
  packet.self        = tramID;
  packet.subscriptions = subscriptions;

  SS_SendSensorSetPower(&packet);
}

/*--------------------------------------------------------------------------*/
void TramClass::clear_power(uint32_t subscriptions)
{
  sensor_command_power_t packet;

  /* Create command. */

  packet.header.code = ClearPower;
  packet.header.size = sizeof(sensor_command_header_t);
  packet.self        = tramID;
  packet.subscriptions = subscriptions;

  SS_SendSensorClearPower(&packet);
}

/*--------------------------------------------------------------------------*/
void TramClass::receive_async_msg(uint32_t param)
{
  uint8_t result_type;
  uint8_t result_data;

  result_type = get_async_msgtype(param);
  result_data = get_async_msgparam(param);

  if (result_type == TramCmdTypeResult)
    {
      /* Result of Tram. */
      
      send_detection_result(param);
    }
  else if (result_type == TramCmdTypeTrans)
    {
      /* Notification of state transition. */
      
      switch (result_data)
        {
        case TramStateMs:
          TramStateTransitionSetState(this, TRAM_STATE_MS);
          break;

        case TramStateCmd:
          TramStateTransitionSetState(this, TRAM_STATE_CMD);
          break;

        case TramStateTmi:
          TramStateTransitionSetState(this, TRAM_STATE_TMI);
          break;

        default:
          tram_err("invalide state: %d\n", result_data);
          ASSERT(0);
          break;
        }
    }
}

/*--------------------------------------------------------------------------*/
void TramClass::receive_sync_msg(void)
{
  struct exe_mh_s exe_mh = m_exe_que.top();
  sensor_command_data_mh_t packet;

  /* Create command. */
  packet.header.code = SendData;
  packet.header.size = sizeof(sensor_command_header_t);
  packet.self        = tramID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 3;
  packet.mh          = exe_mh.cmd;

  SS_SendSensorDataMH(&packet);

  /* Pop exec queue (Free segment). */

  m_exe_que.pop();
}

/*--------------------------------------------------------------------------*/
/*!
 * @brief receive result from dsp
 */
int TramClass::receive(void)
{
  int      command;
  uint32_t msgdata;
  bool     active = true;

  /* Wait for worker message. */

  while (active)
    {
      command = mpmq_receive(&m_mq, &msgdata);
      if (command < 0)
        {
          tram_err("mpmq_receive() failure. command(%d) < 0\n", command);
          return command;
        }
      else
        {
          if (is_async_msg(msgdata))
            {
              this->receive_async_msg(msgdata);
            }
          else
            {
              this->receive_sync_msg();
            }
        }
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int TramClass::handle_event(TramEvent event)
{
  int ret;

  switch (event)
    {
    case MathFuncEvent:
      ret = (TramStateTransitionSetState(this, TRAM_STATE_CMD) == 0) ?
              SS_ECODE_OK : SS_ECODE_STATE_ERROR;
      break;

    default:
      ret = SS_ECODE_PARAM_ERROR;
      break;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramClass::set_state(tram_state_e state)
{
  m_state = state;
  return 0;
}

/*--------------------------------------------------------------------------*/
tram_state_e TramClass::get_state(void)
{
  return m_state;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

TramClass* TramCreate(MemMgrLite::PoolId cmd_pool_id)
{
  return new TramClass(cmd_pool_id);
}

/*--------------------------------------------------------------------------*/
int TramOpen(FAR TramClass *ins, FAR float *likelihood)
{
  int ret;

  ret = ins->open(likelihood);

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramClose(FAR TramClass *ins)
{
  int ret;

  ret = ins->close();
  delete ins;

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramStart(FAR TramClass *ins)
{
  int ret;

  ret = ins->start();

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramStop(FAR TramClass *ins)
{
  int ret;

  ret = ins->stop();

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramWrite(FAR TramClass *ins, FAR sensor_command_data_mh_t *command)
{
  int ret;

  ret = ins->write(command);

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramHandleEvent(FAR TramClass *ins, TramEvent event)
{
  int ret;

  ret = ins->handle_event(event);

  return ret;
}

/*--------------------------------------------------------------------------*/
ScuSettings* TramGetAccelScuSettings(FAR TramClass *ins)
{
  FAR struct ScuSettings *settings = NULL;

  switch (ins->get_state())
    {
    case TRAM_STATE_MS:
      settings = &s_accelSettingsMs;
      break;

    case TRAM_STATE_CMD:
    case TRAM_STATE_TMI:
      settings = &s_accelSettingsCmd;
      break;

    default:
      break;
    }

  return settings;
}
