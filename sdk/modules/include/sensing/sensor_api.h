/****************************************************************************
 * modules/include/sensing/sensor_api.h
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

#ifndef __INCLUDE_SENSING_SENSOR_API_H
#define __INCLUDE_SENSING_SENSOR_API_H

/**
 * @defgroup sensor_manger Sensor Manager API
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#ifdef __cplusplus
/* MemoryManager can used only C++.
 * If you use Memory Manager Library
 * to be selected from kconfig
 */
#include "memutils/memory_manager/MemHandle.h"
#endif
#include "memutils/message/MsgPacket.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*--------------------------------------------------------------------
 *  Command Structure
 *
 * This is Sensor Control Command for Sensor Manager
 *
 * the Sensor Manager is controlled from commands.
 * follow is these commands.
  --------------------------------------------------------------------*/
/**
 * @struct sensor_command_header_t
 * @brief  Sensor Commnad Header.
 *         All sensor command has this header.
 */
typedef struct
{
  unsigned int size    : 8;               /**< packet length */
  unsigned int code    : 8;               /**< command code  */
  unsigned int reserve : 16;              /**< reserve       */

#ifdef __cplusplus
  /** command code getter function */
  unsigned int get_code(void)
  {
    return(code);
  }
#endif /* __cplusplus */

} sensor_command_header_t;

/*--------------------------------------------------------------------------*/
/**
 * @struct sensor_command_data_t
 * @brief  The command of send some sensor data
 *         without MemHandle to the sensor manager.
 */
typedef struct
{
  sensor_command_header_t header;  /**< command header    */

  unsigned int self : 8;           /**< sender sensor ID  */
  unsigned int time : 24;          /**< time stamp        */
  unsigned int fs   : 16;          /**< frequensy         */
  unsigned int size : 16;          /**< number of samples */

  bool         is_ptr;             /**< pointer or not    */

  union
    {
      uint32_t data;               /**< send data         */
      void*    adr;                /**< send data address */
    };

#ifdef __cplusplus
  /** self sensor id getter function */
  unsigned int get_self(void)
    {
      return self;
    }
#endif /* __cplusplus */

} sensor_command_data_t;

/*--------------------------------------------------------------------------*/
#ifdef __cplusplus
/**
 * @struct sensor_command_data_mh_t
 * @brief  The command of send some sensor data with MemHandle
 *         to the sensor manager.
 *         This function only can call on C++.
 */
typedef struct
{
  sensor_command_header_t header;  /**< command header    */
  unsigned int self : 8;           /**< sender sensor ID  */
  unsigned int time : 24;          /**< time stamp        */
  unsigned int fs   : 16;          /**< frequensy         */
  unsigned int size : 16;          /**< number of samples */

/* If you use Memory Manager Library to be selected from kconfig */

  MemMgrLite::MemHandle  mh;       /**< mem handle for send data */

  unsigned int get_self(void)
    {
      return self;
    }
} sensor_command_data_mh_t;

#endif /* __cplusplus */

/*--------------------------------------------------------------------------*/
/**
 * @typedef sensor_data_callback_t
 * @brief   A function pointer for sensor data (without mem handle) callback.
 */
typedef bool (*sensor_data_callback_t)(sensor_command_data_t&);

/*--------------------------------------------------------------------------*/
/**
 * @typedef sensor_data_mh_callback_t
 * @brief   A function pointer for sensor data with MemHandle callback.
 */
typedef bool (*sensor_data_mh_callback_t)(sensor_command_data_mh_t&);


/*--------------------------------------------------------------------------*/
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
/**
 * @typedef sensor_power_callback_t
 * @brief   A function pointer for power control callback.
 */
typedef bool (*sensor_power_callback_t)(bool);
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

/*--------------------------------------------------------------------------*/
/**
 * @typedef api_response_callback_t
 * @brief   A function pointer for API response callback.
 */
typedef void (*api_response_callback_t)(unsigned int code,
                                        unsigned int ercd,
                                        unsigned int self_id);

/*--------------------------------------------------------------------------*/
/**
 * @struct sensor_command_register_t
 * @brief  The command of resister a sensor.
 */
typedef struct
{
  sensor_command_header_t header;         /**< command header                                 */

  unsigned int self: 8;                   /**< sensor ID                                      */
  unsigned int subscriptions: 24;         /**< subscription infomation                        */
  sensor_data_callback_t    callback;     /**< callback for subscription event                */
  sensor_data_mh_callback_t callback_mh;  /**< callback with MemHandle for subscription event */

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
  sensor_power_callback_t   callback_pw;  /**< callback for setpower event */
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

  unsigned int get_self(void)
    {
      return self;
    }

  unsigned int get_subscriptions(void)
    {
      return subscriptions;
    }
} sensor_command_register_t;

/*--------------------------------------------------------------------*/
/**
 * @struct sensor_command_release_t
 * @brief  The command of release the sensor.
 */
typedef struct
{
  sensor_command_header_t header;         /**< command header    */
  unsigned int self;                      /**< release sensor ID */

  unsigned int get_self(void)
    {
      return self;
    }
} sensor_command_release_t;

/*--------------------------------------------------------------------*/
/**
 * @struct sensor_command_change_subscription_t
 * @brief  The command of change subscription of client.
 */
typedef struct
{
  sensor_command_header_t header;         /**< command header             */
  unsigned int self          : 8;         /**< change sensor ID           */
  unsigned int subscriptions : 24;        /**< subscription infomation    */
  bool add;                               /**< add(true) or remode(false) */

  unsigned int get_self(void)
    {
      return self;
    }

  unsigned int get_subscriptions(void)
    {
      return subscriptions;
    }
} sensor_command_change_subscription_t;

/*--------------------------------------------------------------------*/
/**
 * @struct sensor_command_result_t
 * @brief  The command of release the sensor.
 */
typedef struct
{
  sensor_command_header_t header;         /**< command header   */

  unsigned int self : 8;                  /**< sender sensor ID */
  unsigned int time : 24;                 /**< time stamp       */
  unsigned int data;                      /**< result data      */

  unsigned int get_self(void)
    {
      return self;
    }
} sensor_command_result_t;

/*--------------------------------------------------------------------------*/
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
/**
 * @struct sensor_command_power_t
 * @brief  The command of setting sleep mode to sensor.
 */
typedef struct
{
  sensor_command_header_t header;         /**< command header                  */

  unsigned int self: 8;                   /**< sender sensor ID                */
  unsigned int subscriptions: 24;         /**< subscription infomation         */

  unsigned int get_self(void)
  {
    return self;
  }

  unsigned int get_subscriptions(void)
  {
    return subscriptions;
  }

} sensor_command_power_t;

#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------
    Command(Evant) Code.
  --------------------------------------------------------------------*/
/**
 * @enum  SensorCommandCode
 * @brief Sensor Command Code for each sensors.
 */
enum SensorCommandCode
{
  ResisterClient = 0,                     /**< Resister sensor client */
  ReleaseClient,                          /**< Reales sensor client   */
  ChangeSubscription,                     /**< Change subscriptions   */

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
  /*! Set power mode */

  SetPower,

  /*! Clear power mode */

  ClearPower,
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */ 

  /*! Sensing Data send */

  SendData,

  /*! Sensing Data send */

  SendDataMH,

  /*! Logical sensing result send */

  SendResult,

  /*! Number of sensor commands */

  SensorCommandMum
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/*--------------------------------------------------------------------
    External Sensor Manager Interface
  --------------------------------------------------------------------*/
/**
 * @brief  Activation function for sensor manager.
 *         Before using a sensor manager, it is necessary to call this API.
 * @return true: success
 */
extern bool SS_ActivateSensorSubSystem(MsgQueId selfMId,
                                       api_response_callback_t callback);

/**
 * @brief  Deativation function for sensor manager.
 * @return sucess true
 */
extern bool SS_DeactivateSensorSubSystem();

/**
 * @brief     Sender function to Sensor Manager without MemHandle.
 *            Send data is publish to own subscriber.
 * @note      This API send address of publish data.
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorData(FAR sensor_command_data_t *packet);

/**
 * @brief     Sender function to Sensor Manager without MemHandle.
 *            Send result data is publish to own subscriber.
 * @note      This API send result data of sensor process.
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorResult(FAR sensor_command_result_t *packet);

/**
 * @brief     Resister a sensor client to Sensor Manager as subscriber.
 *            Registed sensor can recieve a "publish data"
 *            from required sensors.
 *            When data is published, a registed callback function
 *            will be called.
 * @note      If there is not required sensor, then error occurs.
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorResister(FAR sensor_command_register_t *packet);

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
/**
 * @brief     When sensor power mode are controlled, this function can set.
 *            (If you enable CONFIG_SENSING_MANAGER_POWERCTRL.)
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorSetPower(FAR sensor_command_power_t *packet);
extern void SS_SendSensorClearPower(FAR sensor_command_power_t *packet);

#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

/**
 * @brief     Unresister the sensor client from Sensor Manager.
 * @note      If any sensors sbsribing own, then error occurs.
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorRelease(FAR sensor_command_release_t *packet);

/**
 * @brief     Change subscriptions of sensor client.
 * @note      Only change subscription. Effective to registered sensors.
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorChangeSubscription(FAR sensor_command_change_subscription_t *packet);

#ifdef __cplusplus

/**
 * @brief     Sender function to Sensor Manager with MemHandle
 *            Send data is publish to own subscriber.
 * @note      This API send MemHandle of publish data.
 * @param[in] packet
 * @return    void
 */
extern void SS_SendSensorDataMH(FAR sensor_command_data_mh_t *packet);

} /* extern "C" */
#endif /* __cplusplus */

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_SENSOR_API_H */

