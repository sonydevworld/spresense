/****************************************************************************
 * modules/include/sensing/logical_sensor/arm_gesture.h
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

#ifndef __INCLUDE_SENSING_LOGICAL_SENSOR_ARM_GESTURE_H
#define __INCLUDE_SENSING_LOGICAL_SENSOR_ARM_GESTURE_H

/**
 * @defgroup logical_armgesture Gesture API
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
#include "sensing/logical_sensor/arm_gesture_command.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

class GestureClass
{
public:

  /*** public mathods ***/
  int  open(void);
  int  close(void);
  int  write(FAR sensor_command_data_mh_t *cmd);
  void set_callback(FAR SensorCmd *cmd);
  int  receive(void);
  int  setRotation(FAR GestureSetRotation *rotation);

  GestureClass(MemMgrLite::PoolId pool_id)
    : m_cmd_pool_id(pool_id)
  {
  };

  ~GestureClass(){};

private:

  #define MAX_EXEC_COUNT 8
  struct exe_mh_s
    {
      MemMgrLite::MemHandle cmd;
      MemMgrLite::MemHandle data;
    };
  s_std::Queue<struct exe_mh_s, MAX_EXEC_COUNT> m_exe_que;

  /*** private members ***/
  MemMgrLite::PoolId  m_cmd_pool_id;

  mptask_t  m_mptask;
  mpmq_t    m_mq;

  pthread_t m_thread_id;

  /*** private mathods ***/
  int sendInit(void);

};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Create GestureClass instance. 
 * @param[in] cmd_pool_id : Pool id for DSP communication data
 * @return Address for instance of GestureClass
 */
FAR GestureClass *GestureCreate(MemMgrLite::PoolId cmd_pool_id);

/**
 * @brief     Load Gesture library and boot up as worker task.
 *            After booted up, send initialize and wait complete.
 * @param[in] ins : instance address of GestureClass
 * @return    result of process.
 */
int GestureOpen(FAR GestureClass *ins);

/**
 * @brief     Destory Gesture worker task.
 * @param[in] ins : instance address of GestureClass
 * @return    result of process.
 */
int GestureClose(FAR GestureClass *ins);

/**
 * @brief     Send data to Gesture worker task. 
 * @param[in] ins : instance address of GestureClass
 * @param[in] command : command including data to send
 * @return    result of process
 */
int GestureWrite(FAR GestureClass* ins,
                 FAR sensor_command_data_mh_t *command);

/**
 * @brief     Set Gesture coordinate
 * @param[in] ins : instance address of GestureClass
 * @param[in] rotation : coordinate transformation matrix
 * @return    result of process
 */
int GestureSetCoordinate(FAR GestureClass *ins,
                         FAR GestureSetRotation *rotation);

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_ARM_GESTURE_H */
