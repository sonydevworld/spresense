/****************************************************************************
 * modules/include/sensing/logical_sensor/transport_mode.h
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

#ifndef __INCLUDE_SENSING_LOGICAL_SENSOR_TRANSPORT_MODE_H
#define __INCLUDE_SENSING_LOGICAL_SENSOR_TRANSPORT_MODE_H

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
#include "sensing/logical_sensor/transport_mode_command.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRAM_NUMBER_OF_MODES 13             /**< Number of transportation mode */

#define ACCEL_SAMPLING_FREQUENCY_MS   16    /**< Accel sensor frequency in MS state[Hz] */
#define ACCEL_SAMPLING_FREQUENCY_CMD  64    /**< Accel sensor frequency in CMD state[Hz] */
#define ACCEL_SAMPLING_FREQUENCY_TMI  64    /**< Accel sensor frequency in TMI state[Hz] */
#define ACCEL_WATERMARK_NUM           320   /**< Accel sensor sample data watermark */
#define ACCEL_FIFO_NUM                (ACCEL_WATERMARK_NUM * 2)  /**< FIFO sample data watermark */
#define ACCEL_TRIGGER_RISE_THRESS     50    /**< Accel sensor rise threshold */
#define ACCEL_TRIGGER_RISE_COUNT0     2     /**< Accel sensor rise preventing counts */
#define ACCEL_TRIGGER_RISE_COUNT1     20    /**< Accel sensor rise actual counts */
#define ACCEL_TRIGGER_RISE_DELAY      0     /**< Rise event notification delay in samples */
#define ACCEL_TRIGGER_FALL_THRESS     49    /**< Accel sensor fall threshold */
#define ACCEL_TRIGGER_FALL_COUNT0     2     /**< Accel sensor fall preventing counts */
#define ACCEL_TRIGGER_FALL_COUNT1     4     /**< Accel sensor fall actual counts */
#define ACCEL_TRIGGER_FALL_DELAY      0     /**< Fall event notification delay in samples */

#define GET_SCU_ACCEL_SAMPLING_FREQUENCY(_x_) (512 / (1 << (_x_)))  /**< sampling rate = 32768 / 64 / (2 ^ (_x_) )Hz */

#define MAG_SAMPLING_FREQUENCY  8           /**< Mag sensor frequency[Hz] */
#define MAG_WATERMARK_NUM       40          /**< Mag sensor sample data watermark */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * TRAM state
 */

typedef enum
{
  TRAM_STATE_UNINITIALIZED = 0,   /**< uninitialized state */
  TRAM_STATE_MS,                  /**< motion sensing state */
  TRAM_STATE_CMD,                 /**< continuous motion detection state */
  TRAM_STATE_TMI,                 /**< transportation mode inference state */
  TRAM_STATE_NUM
} tram_state_e;

/**
 * TRAM event type
 */

enum TramEvent {
  MathFuncEvent,                  /**< Math function request from SCU */
};

/**
 * TRAM result type
 */

enum TramNotification {
  ChangeScuSettings,              /**< SCU setting change request from TRAM */
};

/**
 * SCU setting
 */

struct ScuSettings {
  uint16_t fifosize;              /**< Accel secsor FIFO size */
  uint8_t samplingrate;           /**< Accel secsor sampling rate[Hz] */
  uint8_t elements;               /**< Accel secsor trigger event */
  FAR struct scuev_notify_s *ev;  /**< Event notifier setting */
  FAR struct math_filter_s *mf;   /**< Math Function IIR filter setting */
  FAR struct scufifo_wm_s *wm;    /**< Watermark notification */
};

/*--------------------------------------------------------------------
 *   TRAM Class
 *--------------------------------------------------------------------
 */

class TramClass
{
public:

  /* public methods */
  int open(FAR float *likelihood);
  int close(void);
  int start(void);
  int stop(void);
  int write(FAR sensor_command_data_mh_t*);
  void send_detection_result(uint32_t pred);
  void send_notification(TramNotification notification);
  void set_power(uint32_t subscriptions);
  void clear_power(uint32_t subscriptions);
  void receive_sync_msg(void);
  void receive_async_msg(uint32_t param);
  int receive(void);
  int handle_event(TramEvent event);

  int set_state(tram_state_e state);
  tram_state_e get_state(void);

  TramClass(MemMgrLite::PoolId cmd_pool_id)
      : m_cmd_pool_id(cmd_pool_id),
        m_state(TRAM_STATE_UNINITIALIZED)
  {
  };

  ~TramClass(){};

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

  tram_state_e m_state;

  /* private methods */

  int sendInit(FAR float *likelihood);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Create TramClass instance. 
 * @param[in] cmd_pool_id : Pool id for DSP communication data
 * @return Address for instance of TramClass
 *
 */
TramClass* TramCreate(MemMgrLite::PoolId cmd_pool_id);

/**
 * @brief     Load TRAM library and boot up as worker task.
 *            After booted up, send initialize and wait complete.
 * @param[in] ins : instance address of TramClass
 * @param[in] likelihood : buffer address to contain likelihood
 * @return    result of process.
 */
int TramOpen(FAR TramClass *ins, FAR float *likelihood = NULL);

/**
 * @brief     Destory TRAM worker task.
 * @param[in] ins : instance address of TramClass
 * @return    result of process.
 */
int TramClose(FAR TramClass *ins);

/**
 * @brief     Start TRAM with Dsp analysing.
 * @param[in] ins : instance address of TramClass
 * @return    result of process.
 */
int TramStart(FAR TramClass *ins);

/**
 * @brief     Stop TRAM with Dsp analysing.
 * @param[in] ins : instance address of TramClass
 * @return    result of process.
 */
int TramStop(FAR TramClass *ins);

/**
 * @brief     Send data to TRAM worker task.
 * @param[in] ins : instance address of TramClass
 * @param[in] command : command including data to send
 * @return    result of process
 */
int TramWrite(FAR TramClass *ins, FAR sensor_command_data_mh_t *command);

/**
 * @brief     Send event to TRAM to handle event
 * @param[in] ins : instance address of TramClass
 * @param[in] event : event to handle
 * @return    result of process
 */
int TramHandleEvent(FAR TramClass *ins, TramEvent event);

/**
 * @brief     Get settings information of accelerator
 * @param[in] ins : instance address of TramClass
 * @return    pointer of settings information
 */
struct ScuSettings* TramGetAccelScuSettings(FAR TramClass* ins);

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_LOGICAL_SENSOR_TRANSPORT_MODE_H */
