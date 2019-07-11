/****************************************************************************
 * modules/include/sensing/logical_sensor/compass.h
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

#ifndef  __INCLUDE_SENSING_COMPASS_H
#define  __INCLUDE_SENSING_COMPASS_H

/**
 * @defgroup logical_compass Compass API
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
#include "sensing/logical_sensor/compass_command.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct CompassResult
 * @brief The structure of result on compass commands.
 */

typedef struct
{
  int   resultcmd;    /**<
                       * Indicates the result of any command.
                       * The command type at transmission is set.
                       */
  int   exec_result;  /**< The command execution result is set. */
  int   errcode;      /**<
                       * A detailed code of the command execution result
                       * is set.
                       */
  float azimuth;      /**<
                       * Indicates calculation of azimuth from
                       * input sensor data. The unit is[rad].
                       * (*)No deviation correction.
                       */
  float pitch;        /**<
                       * Indicates calculation of pitch attitude from 
                       * input sensor data. The unit is[rad].
                       */
  float roll;         /**<
                       * Indicates calculation of roll attitude from
                       * input sensor data. The unit is[rad].
                       */
  float decl;         /**<
                       * Argument angle. The unit is[rad].
                       * (*)Use Azimuth - Decl
                       */
  int   acc_azimuth;  /**<
                       * Return the azimuth accuracy level. [0 - 3]
                       */
  int   calib_lv;     /**<
                       * Return the calibration accuracy level. [0 - 3]
                       */
} CompassResult;

/*--------------------------------------------------------------------
 *   Compass Class
 * --------------------------------------------------------------------
 */

class CompassClass
{
public:

  /* public methods */

  int open(void);
  int close(void);
  int write(FAR sensor_command_data_mh_t*);
  int result(void);
  void set_callback(uint32_t);
  int receive(void);


  CompassClass(MemMgrLite::PoolId cmd_pool_id,
               MemMgrLite::PoolId rst_pool_id)
      : m_cmd_pool_id(cmd_pool_id)
      , m_rst_pool_id(rst_pool_id)
  {
  };

  ~CompassClass(){};

private:

  #define MAX_EXEC_COUNT 10
  struct exe_mh_s
    {
      MemMgrLite::MemHandle cmd;
      MemMgrLite::MemHandle data;
    };
  s_std::Queue<struct exe_mh_s, MAX_EXEC_COUNT> m_accel_exe_que;
  s_std::Queue<struct exe_mh_s, MAX_EXEC_COUNT> m_mag_exe_que;

  /* private members */

  MemMgrLite::PoolId m_cmd_pool_id;
  MemMgrLite::PoolId m_rst_pool_id;

  mptask_t    m_mptask;
  mpmq_t      m_mq;

  pthread_t   m_thread_id;

  /* private methods */

  int sendInit(void);
  int sendFlush(void);
};

/*--------------------------------------------------------------------
 *   External Interface
 *--------------------------------------------------------------------
 */

/**
 * @brief Create CompassClass instance. 
 * @param[in] cmd_pool_id : Pool id for DSP communication data
 * @return Address for instance of CompassClass
 *
 */
FAR CompassClass *CompassCreate(MemMgrLite::PoolId cmd_pool_id,
                                MemMgrLite::PoolId rst_pool_id);

/**
 * @brief     Load Compass library and boot up as worker task.
 *            After booted up, send initialize and wait complete.
 * @param[in] ins : instance address of CompassClass
 * @return    result of process.
 */
int CompassOpen(FAR CompassClass *ins);

/**
 * @brief     Destory Compass worker task.
 * @param[in] ins : instance address of CompassClass
 * @return    result of process.
 */
int CompassClose(FAR CompassClass *ins);

/**
 * @brief     Send data to Compass worker task.
 * @param[in] ins : instance address of CompassClass
 * @param[in] command : command including data to send
 * @return    result of process
 */
int CompassWrite(FAR CompassClass *ins,
                 FAR sensor_command_data_mh_t *command);

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_COMPASS_H */

