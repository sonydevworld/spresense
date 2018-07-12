/****************************************************************************
 * modules/sensing/manager/sensor_manager.cpp
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

#include "sensor_manager.h"
#include <debug.h>
#include <nuttx/arch.h>
#include <sdk/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#define MSGQ_NULL 0 /* temporal */


#define SF_TASK_PRIORITY           110
#define SF_TASK_MANAGER_STACK_SIZE 2048

/****************************************************************************
 * Private Data
 ****************************************************************************/

static SensorManager *TheSensorManager = NULL;
static MsgQueId s_selfMid = 0xFF; /* Invalid ID */
static api_response_callback_t s_response_callback = NULL;
static pid_t s_smng_pid = -1;

/****************************************************************************
 * Public Data
 ****************************************************************************/

SensorManager::MsgProc SensorManager::MsgProcTbl[] =
{
    &SensorManager::register_client,
    &SensorManager::release_client,
    &SensorManager::change_subscription,
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
    &SensorManager::set_power,
    &SensorManager::clear_power,
#else
    &SensorManager::ignore,
    &SensorManager::ignore,
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */
    &SensorManager::send_data,
#ifdef __cplusplus
    &SensorManager::send_data_mh,
#else
    &SensorManager::ignore,
#endif /* __cplusplus */
    &SensorManager::send_result
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void SensorManager::create(MsgQueId selfMId, api_response_callback_t callback)
{  
  if (TheSensorManager == NULL)
    {
      TheSensorManager = new SensorManager(selfMId, callback);
      TheSensorManager->run();
    }
  else
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------*/
void SensorManager::run(void)
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

  err_code = MsgLib::referMsgQueBlock(m_selfMId, &que);
  F_ASSERT(err_code == ERR_OK);

  while(1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------*/
void SensorManager::parse(MsgPacket* msg)
{
  F_ASSERT(MSG_IS_REQUEST(msg->getType()) != 0);

  /* For external event. */

  uint event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT((event < SENSOR_MNG_MSG_NUM));
  (this->*MsgProcTbl[event])(msg);
}

/*--------------------------------------------------------------------*/
void SensorManager::register_client(MsgPacket* packet)
{
  sensor_command_register_t reg = packet->moveParam<sensor_command_register_t>();

  client_table[reg.get_self()].status = 0x01;
  client_table[reg.get_self()].callback = reg.callback;
  client_table[reg.get_self()].callback_mh = reg.callback_mh;
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
  power_table[reg.get_self()].callback = reg.callback_pw;
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

  /* Regist as a subscriptors of required SensorID. */

  for (int i = 0, j = reg.get_subscriptions(); (j != 0) || (i < 24); i++)
    {
      if (j & (0x01<<i))
        {
          /* If required SensorID is not active, take as a error. */

          if (client_table[i].status == 0)
            {
              response(reg.header.code,
                       SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE,
                       reg.get_self());
              return;
            }

          client_table[i].subscribers |= (0x01 << reg.get_self());
          j &= ~(0x01 << i);
              
          _info("sesor id : %2d >> %08x\n", i, client_table[i].subscribers);
        }

    }

  response(reg.header.code, SS_ECODE_OK, reg.get_self());
}

/*--------------------------------------------------------------------*/
void SensorManager::release_client(MsgPacket* packet)
{
  sensor_command_release_t rel = packet->moveParam<sensor_command_release_t>();

  /* If required SensorID has any subscribers, take as error. */

  if (client_table[rel.get_self()].subscribers != 0)
    {
      response(rel.header.code,
               SS_ECODE_REQUIRED_SENSOR_STILL_ACTIVE,
               rel.get_self());
      return;
    }

  /* Delete from subscribers of every SensorID. */

  for (int i = 0; i < 24; i++)
    {
      client_table[i].subscribers &= ~(0x01 << rel.get_self());
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
      power_table[i].subscribers &= ~(0x01 << rel.get_self());
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */
    }

  client_table[rel.get_self()].status      = 0x00;
  client_table[rel.get_self()].subscribers = 0x00;
  client_table[rel.get_self()].callback    = 0x00;
  client_table[rel.get_self()].callback_mh = 0x00;
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
  power_table[rel.get_self()].status       = 0x00;
  power_table[rel.get_self()].subscribers  = 0x00;
  power_table[rel.get_self()].callback     = 0x00;
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

  response(rel.header.code, SS_ECODE_OK, rel.get_self());
}

/*--------------------------------------------------------------------*/
void SensorManager::change_subscription(MsgPacket* packet)
{
  sensor_command_change_subscription_t chg =
    packet->moveParam<sensor_command_change_subscription_t>();

  for (int i = 0, j = chg.get_subscriptions(); (j != 0) || (i < 24); i++)
    {
      if (j & (0x01<<i))
        {
          /* If required SensorID is not active, take as a error. */

          if (client_table[i].status == 0)
            {
              response(chg.header.code,
                       SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE,
                       chg.get_self());
              return;
            }

          /* Regist/delete subscribers accoding to "add" parameter. */

          if (chg.add)
            {
              client_table[i].subscribers |= (0x01 << chg.get_self());
            }
          else
            {
              client_table[i].subscribers &= ~(0x01 << chg.get_self());
            }

          j &= ~(0x01 << i);

          _info("sesor id : %2d >> %08x\n", i, client_table[i].subscribers);
        }
    }

  response(chg.header.code, SS_ECODE_OK, chg.get_self());
}

/*--------------------------------------------------------------------*/
void SensorManager::send_data(MsgPacket* packet)
{
  sensor_command_data_t data = packet->moveParam<sensor_command_data_t>();

  if (client_table[data.get_self()].status == 0x00)
    {
      response(data.header.code,
               SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE,
               data.get_self());
      return;
    }

  for (int i = 0, j = client_table[data.get_self()].subscribers;
        (j != 0) || (i < 24); i++)
    {
      if (j & (0x01 << i))
        {
          if (!client_table[i].callback)
            {
              response(data.header.code,
                       SS_ECODE_NOTIFICATION_DST_UNDEFINED,
                       data.get_self());
              return;
            }

          client_table[i].callback(data);/* callback */
          j &= ~(0x01 << i);
        }
    }

  response(data.header.code, SS_ECODE_OK, data.get_self());
}

#ifdef __cplusplus
/*--------------------------------------------------------------------*/
void SensorManager::send_data_mh(MsgPacket* packet)
{
  sensor_command_data_mh_t data = packet->moveParam<sensor_command_data_mh_t>();

  if (client_table[data.get_self()].status == 0x00)
    {
      response(data.header.code,
               SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE,
               data.get_self());
      return;
    }

  for (int i = 0, j = client_table[data.get_self()].subscribers;
        (j != 0) || (i < 24); i++)
    {
      if (j & (0x01 << i))
        {
          if (!client_table[i].callback_mh)
            {
              response(data.header.code,
                       SS_ECODE_NOTIFICATION_DST_UNDEFINED,
                       data.get_self());
              return;
            }

          client_table[i].callback_mh(data);/* callback */
          j &= ~(0x01 << i);
        }
    }

  response(data.header.code, SS_ECODE_OK, data.get_self());
}
#endif /* __cplusplus */

/*--------------------------------------------------------------------*/
void SensorManager::send_result(MsgPacket* packet)
{
  sensor_command_result_t res = packet->moveParam<sensor_command_result_t>();

  if (client_table[res.get_self()].status == 0x00)
    {
      response(res.header.code,
               SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE,
               res.get_self());
      return;
    }

  for (int i = 0, j=client_table[res.get_self()].subscribers;
        (j != 0)||(i < 24); i++)
    {
      if (j & (0x01 << i))
        {
          /* tentative */
          j &= ~(0x01 << i);
        }
    }

  response(res.header.code, SS_ECODE_OK, res.get_self());
}

/*--------------------------------------------------------------------*/
#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
void SensorManager::set_power(MsgPacket* packet)
{
  sensor_command_power_t pow = packet->moveParam<sensor_command_power_t>();

  for (int i = 0; i < 24; i++)
    {
      int j = 0x01 << pow.get_self();
      int k = 0x01 << i;

      if (client_table[i].subscribers & j)
        {
          if (pow.get_subscriptions() & k)
            {
              /* Set power on. */

              if (power_table[i].subscribers == 0)
                {
                  power_table[i].subscribers |= j;

                  if (!power_table[i].callback)
                    {
                      response(pow.header.code,
                               SS_ECODE_NOTIFICATION_DST_UNDEFINED,
                               pow.get_self());
                      return;
                    }

                  power_table[i].callback(true);
                }
            }
        }
    }

  response(pow.header.code, SS_ECODE_OK, pow.get_self());
}

/*--------------------------------------------------------------------*/
void SensorManager::clear_power(MsgPacket* packet)
{
  sensor_command_power_t pow = packet->moveParam<sensor_command_power_t>();

  for (int i = 0; i < 24; i++)
    {
      int j = 0x01 << pow.get_self();
      int k = 0x01 << i;

      if (client_table[i].subscribers & j)
        {
          if (pow.get_subscriptions() & k)
            {
              /* Set power off. */

              if (power_table[i].subscribers)
                {
                  power_table[i].subscribers &= ~j;

                  if (power_table[i].subscribers == 0)
                    {
                      if (!power_table[i].callback)
                        {
                          response(pow.header.code,
                                   SS_ECODE_NOTIFICATION_DST_UNDEFINED,
                                   pow.get_self());
                          return;
                        }

                      power_table[i].callback(false);
                    }
                }
            }
        }
    }

  response(pow.header.code, SS_ECODE_OK, pow.get_self());
}
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

/*--------------------------------------------------------------------*/
void SensorManager::ignore(MsgPacket* packet)
{
  _err("Illegal command.\n");
}

/*--------------------------------------------------------------------*/
void SensorManager::response(unsigned int code, unsigned int ercd, unsigned int id)
{
  if (m_api_response_callback)
    {
      m_api_response_callback(code, ercd, id);
    }

  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern "C"
{

int SF_SensorManagerEntry(void)
{
  SensorManager::create(s_selfMid, s_response_callback);
  return 0;
}

/*--------------------------------------------------------------------*/
/* Activation function for sensor subsystem. 
 */
bool SF_ActivateSensorSubSystem(MsgQueId selfMId, api_response_callback_t callback)
{
  s_selfMid = selfMId;
  s_response_callback = callback;
  s_smng_pid = task_create("sensor_manager",
                           SF_TASK_PRIORITY,
                           SF_TASK_MANAGER_STACK_SIZE,
                           (main_t)SF_SensorManagerEntry,
                           0);

  if (s_smng_pid < 0)
    {
      printf("ERROR SF_ActivateSensorSubSystem failed\n");
      return false;
    }
  return true;
}

/*--------------------------------------------------------------------*/
/* Deactivation function for sensor subsystem.
*/
bool SF_DeactivateSensorSubSystem()
{
  if (s_smng_pid < 0)
    {
      return false;
    }
  task_delete(s_smng_pid);

  DEBUGASSERT(TheSensorManager != NULL);
  delete TheSensorManager;
  TheSensorManager = NULL;
  s_smng_pid = -1;

  return true;
}

/*--------------------------------------------------------------------*/
/* Sender function to Sensor Manager wo MemHandle
    @param packet 
*/
void SF_SendSensorData(sensor_command_data_t* packet)
{
  err_t er = MsgLib::send<sensor_command_data_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SEND_DATA,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
void SF_SendSensorResult(sensor_command_result_t* packet)
{
  err_t er = MsgLib::send<sensor_command_result_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SEND_RESULT,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}
/*--------------------------------------------------------------------*/
void SF_SendSensorResister(sensor_command_register_t* packet)
{
  err_t er = MsgLib::send<sensor_command_register_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_REGISTER_CLIENT,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
void SF_SendSensorRelease(sensor_command_release_t* packet)
{
  err_t er = MsgLib::send<sensor_command_release_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_RELEASE_CLIENT,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
extern void SF_SendSensorChangeSubscription(sensor_command_change_subscription_t* packet)
{
  err_t er = MsgLib::send<sensor_command_change_subscription_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_CHG_SUBSCRIPTION,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
/*--------------------------------------------------------------------*/
void SF_SendSensorSetPower(sensor_command_power_t* packet)
{
  err_t er = MsgLib::send<sensor_command_power_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SET_POWER,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
void SF_SendSensorClearPower(sensor_command_power_t* packet)
{
  err_t er = MsgLib::send<sensor_command_power_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_CLEAR_POWER,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}
#endif /* CONFIG_SENSING_MANAGER_POWERCTRL */

}/* extern "C"  */

#ifdef __cplusplus
/*--------------------------------------------------------------------*/
/* Sender function to Sensor Manager wo MemHandle
    @param packet 
*/
void SF_SendSensorDataMH(sensor_command_data_mh_t* packet)
{
  err_t er = MsgLib::send<sensor_command_data_mh_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SEND_DATA_MH,
               MSGQ_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}
#endif /* __cplusplus */
