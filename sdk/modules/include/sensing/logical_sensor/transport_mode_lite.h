/****************************************************************************
 * modules/include/sensing/logical_sensor/transport_mode_lite.h
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

#ifndef __INCLUDE_SENSING_LOGICAL_SENSOR_TRANSPORT_MODE_LITE_H
#define __INCLUDE_SENSING_LOGICAL_SENSOR_TRANSPORT_MODE_LITE_H

/**
 * @defgroup logical_tramsport TRAnsport Mode rcognition API 
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <asmp/mpmq.h>
#include <asmp/mptask.h>

#include "sensing/sensor_api.h"
#include "sensing/sensor_id.h"
#include "sensing/sensor_ecode.h"
#include "sensing/logical_sensor/sensor_command.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRAMLITE_NUMBER_OF_MODES 5

#define ACCEL_SAMPLING_FREQUENCY_MS   16    /* Hz */
#define ACCEL_SAMPLING_FREQUENCY_CMD  64    /* Hz */
#define ACCEL_SAMPLING_FREQUENCY_TMI  64    /* Hz */
#define ACCEL_WATERMARK_NUM           320   /* samples */
#define ACCEL_FIFO_NUM                (ACCEL_WATERMARK_NUM * 2)  /* samples */
#define ACCEL_TRIGGER_RISE_THRESS     50
#define ACCEL_TRIGGER_RISE_COUNT0     2
#define ACCEL_TRIGGER_RISE_COUNT1     20
#define ACCEL_TRIGGER_RISE_DELAY      0
#define ACCEL_TRIGGER_FALL_THRESS     49
#define ACCEL_TRIGGER_FALL_COUNT0     2
#define ACCEL_TRIGGER_FALL_COUNT1     4
#define ACCEL_TRIGGER_FALL_DELAY      0

#define GET_SCU_ACCEL_SAMPLING_FREQUENCY(_x_) (512 / (1 << (_x_)))   /* sampling rate = 32768 / 64 / (2 ^ (_x_) ) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  TRAMLITE_STATE_UNINITIALIZED = 0,   /* uninitialized state */
  TRAMLITE_STATE_MS,                  /* motion sensing state */
  TRAMLITE_STATE_CMD,                 /* continuous motion detection state */
  TRAMLITE_STATE_TMI,                 /* transportation mode inference state */
  TRAMLITE_STATE_NUM
} tramlite_state_e;

enum TramliteEvent {
  MathFuncEvent,
};

enum TramliteNotification {
  ChangeScuSettings,
};

struct ScuSettings {
  uint16_t fifosize;
  uint8_t samplingrate;
  uint8_t elements;
  FAR struct scuev_notify_s *ev;
  FAR struct math_filter_s *mf;
  FAR struct scufifo_wm_s *wm;
};

/*--------------------------------------------------------------------
 *   TRAMLITE Class
 *--------------------------------------------------------------------
 */

class TramliteClass
{
public:

  /* public methods */

  int open(FAR float *likelihood);
  int close(void);
  int start(void);
  int stop(void);
  int write(FAR sensor_command_data_mh_t*);
  void send_detection_result(uint32_t pred);
  void send_notification(TramliteNotification notification);
  void power_ctrl(bool on, uint32_t subscriptions);
  void receive_sync_msg(void);
  void receive_async_msg(uint32_t param);
  int receive(void);
  int handle_event(TramliteEvent event);

  int set_state(tramlite_state_e state);
  tramlite_state_e get_state(void);

  TramliteClass(MemMgrLite::PoolId cmd_pool_id)
      : m_cmd_pool_id(cmd_pool_id),
        m_state(TRAMLITE_STATE_UNINITIALIZED)
  {
  };

  ~TramliteClass(){};

private:
  #define MAX_EXEC_COUNT 8

  struct exe_mh_s {
    MemMgrLite::MemHandle cmd;
    MemMgrLite::MemHandle data;
  };

  s_std::Queue<struct exe_mh_s, MAX_EXEC_COUNT> m_exe_que;

  /* private members */

  MemMgrLite::PoolId  m_cmd_pool_id;

  mptask_t    m_mptask;
  mpmq_t      m_mq;

  pthread_t m_thread_id;

  tramlite_state_e m_state;

  /* private methods */

  int sendInit(FAR float *likelihood);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Create TramliteClass instance. 
 * @param[in] cmd_pool_id : Pool id for DSP communication data
 * @return Address for instance of TramliteClass
 *
 */
TramliteClass* TramliteCreate(MemMgrLite::PoolId cmd_pool_id);

/**
 * @brief     Load TRAMLITE library and boot up as worker task.
 *            After booted up, send initialize and wait complete.
 * @param[in] ins : instance address of TramliteClass
 * @param[in] likelihood : buffer address to contain likelihood
 * @return    result of process.
 */
int TramliteOpen(TramliteClass* ins, float* likelihood = NULL);

/**
 * @brief     Destory TRAMLITE worker task.
 * @param[in] ins : instance address of TramliteClass
 * @return    result of process.
 */
int TramliteClose(TramliteClass* ins);

/**
 * @brief     Start TRAMLITE with Dsp analysing.
 * @param[in] ins : instance address of TramliteClass
 * @return    result of process.
 */
int TramliteStart(TramliteClass* ins);

/**
 * @brief     Stop TRAMLITE with Dsp analysing.
 * @param[in] ins : instance address of TramliteClass
 * @return    result of process.
 */
int TramliteStop(TramliteClass* ins);

/**
 * @brief     Send data to TRAMLITE worker task.
 * @param[in] ins : instance address of TramliteClass
 * @param[in] command : command including data to send
 * @return    result of process
 */
int TramliteWrite(TramliteClass* ins, sensor_command_data_mh_t* command);

/**
 * @brief     Send event to TRAMLITE to handle event
 * @param[in] ins : instance address of TramliteClass
 * @param[in] event : event to handle
 * @return    result of process
 */
int TramliteHandleEvent(TramliteClass* ins, TramliteEvent event);

/**
 * @brief     Get settings information of accelerator
 * @param[in] ins : instance address of TramClass
 * @return    pointer of settings information
 */
struct ScuSettings* TramliteGetAccelScuSettings(TramliteClass* ins);

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_LOGICAL_SENSOR_TRANSPORT_MODE_LITE_H */
