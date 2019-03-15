/****************************************************************************
 * modules/sensing/manager/sensor_manager.h
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

#ifndef __SENSING_MANAGER_SENSOR_MANAGER_H
#define __SENSING_MANAGER_SENSOR_MANAGER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include "memutils/message/Message.h"
#include "sensing/sensor_message_types.h"
#include "sensing/sensor_id.h"
#include "sensing/sensor_api.h"
#include "sensing/sensor_ecode.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

class SensorManager
{

public:
  static void create(MsgQueId selfMId, api_response_callback_t callback);

  MsgQueId get_mid()
  {
    return m_selfMId;
  }

  ~SensorManager(){};

private:
  SensorManager(MsgQueId selfMId, api_response_callback_t callback)
      : m_selfMId(selfMId)
      , m_api_response_callback(callback)
  {
    for (int i = 0; i < 24; i++)
      {
        client_table[i].status      = 0;
        client_table[i].subscribers = 0;
        client_table[i].callback    = NULL;
        client_table[i].callback_mh = NULL;

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
        power_table[i].status       = 0;
        power_table[i].subscribers  = 0;
        power_table[i].callback     = NULL;
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */ 
      }
  };

  /*** private members ***/
  MsgQueId m_selfMId;
  api_response_callback_t m_api_response_callback;

  /** subscriber information */
  typedef struct
  {
    unsigned int status : 8;       /** status itself */
    unsigned int subscribers : 24; /** subscribers */
    sensor_data_callback_t    callback;
    sensor_data_mh_callback_t callback_mh;
  } client_info_t;

  /** subscriber database*/
  client_info_t client_table[24]; /* 24 must be config.*/

  /*** private mathods ***/
  void    run(void);

  void    parse(MsgPacket *);

  typedef void (SensorManager::*MsgProc)(MsgPacket*);
  static  MsgProc MsgProcTbl[];

  void    register_client(MsgPacket*);
  void    release_client(MsgPacket*);
  void    change_subscription(MsgPacket*);
  void    send_data(MsgPacket*);
  void    send_data_mh(MsgPacket*);
  void    send_result(MsgPacket*);

  void    ignore(MsgPacket*);
  void    response(unsigned int code, unsigned int ercd, unsigned int id);

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
  /** poweroff information */
  typedef struct
  {
    unsigned int status : 8;       /** reserve */
    unsigned int subscribers : 24; /** poweron subscribers */
    sensor_power_callback_t callback;
  } power_info_t;

  /** subscriber database*/
  power_info_t power_table[24]; /* 24 must be config.*/

  void    set_power(MsgPacket*);
  void    clear_power(MsgPacket*);

#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */ 

};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __SENSING_MANAGER_SENSOR_MANAGER_H */
