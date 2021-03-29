/****************************************************************************
 * modules/sensing/manager/sensor_manager.cpp
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
#include <nuttx/arch.h>

#include "sensor_manager.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Manager debug feature. */

#ifdef CONFIG_SENSING_MANAGER_DEBUG_ERROR
#  define sensor_err(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define sensor_err(fmt, ...)
#endif
#ifdef CONFIG_SENSING_MANAGER_DEBUG_INFO
#  define sensor_info(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define sensor_info(fmt, ...)
#endif

#define SS_TASK_PRIORITY           110
#define SS_TASK_MANAGER_STACK_SIZE 2048

/****************************************************************************
 * Private Data
 ****************************************************************************/

static SensorManager *TheSensorManager = NULL;
static MsgQueId s_selfMid = 0xFF; /* Invalid ID */
static api_response_callback_t s_response_callback = NULL;
static pthread_t s_smng_pid = INVALID_PROCESS_ID;

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
  sensor_err("Illegal command.\n");
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
FAR void *SS_SensorManagerEntry(FAR void *arg)
{
  SensorManager::create(s_selfMid, s_response_callback);
  return 0;
}

/*--------------------------------------------------------------------*/
/* Activation function for sensor subsystem. 
 */
bool SS_ActivateSensorSubSystem(MsgQueId selfMId,
                                api_response_callback_t callback)
{
  s_selfMid = selfMId;
  s_response_callback = callback;

  pthread_attr_t     attr;
  struct sched_param sch_param;
  int                ret = 0;
  pthread_attr_init(&attr);
  sch_param.sched_priority = SS_TASK_PRIORITY;
  attr.stacksize           = SS_TASK_MANAGER_STACK_SIZE;
  pthread_attr_setschedparam(&attr, &sch_param);

  ret = pthread_create(&s_smng_pid,
                       &attr,
                       (pthread_startroutine_t)SS_SensorManagerEntry,
                       (pthread_addr_t)NULL);
  if (ret < 0)
    {
      sensor_err("ERROR SS_ActivateSensorSubSystem failed\n");
      return false;
    }
  return true;
}

/*--------------------------------------------------------------------*/
/* Deactivation function for sensor subsystem.
*/
bool SS_DeactivateSensorSubSystem()
{
  if (s_smng_pid == INVALID_PROCESS_ID)
    {
      return false;
    }

  FAR void *thread_return;
  pthread_cancel(s_smng_pid);
  pthread_join(s_smng_pid, &thread_return);

  DEBUGASSERT(TheSensorManager != NULL);
  delete TheSensorManager;
  TheSensorManager = NULL;
  s_smng_pid = INVALID_PROCESS_ID;

  return true;
}

/*--------------------------------------------------------------------*/
/* Sender function to Sensor Manager wo MemHandle
    @param packet 
*/
void SS_SendSensorData(FAR sensor_command_data_t *packet)
{
  err_t er = MsgLib::send<sensor_command_data_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SEND_DATA,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
void SS_SendSensorResult(FAR sensor_command_result_t *packet)
{
  err_t er = MsgLib::send<sensor_command_result_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SEND_RESULT,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}
/*--------------------------------------------------------------------*/
void SS_SendSensorResister(FAR sensor_command_register_t *packet)
{
  err_t er = MsgLib::send<sensor_command_register_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_REGISTER_CLIENT,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
void SS_SendSensorRelease(sensor_command_release_t* packet)
{
  err_t er = MsgLib::send<sensor_command_release_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_RELEASE_CLIENT,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
extern void SS_SendSensorChangeSubscription(FAR sensor_command_change_subscription_t *packet)
{
  err_t er = MsgLib::send<sensor_command_change_subscription_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_CHG_SUBSCRIPTION,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

#ifdef CONFIG_SENSING_MANAGER_POWERCTRL
/*--------------------------------------------------------------------*/
void SS_SendSensorSetPower(FAR sensor_command_power_t *packet)
{
  err_t er = MsgLib::send<sensor_command_power_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SET_POWER,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
void SS_SendSensorClearPower(FAR sensor_command_power_t *packet)
{
  err_t er = MsgLib::send<sensor_command_power_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_CLEAR_POWER,
               MSG_QUE_NULL,
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
void SS_SendSensorDataMH(FAR sensor_command_data_mh_t *packet)
{
  err_t er = MsgLib::send<sensor_command_data_mh_t>(
               TheSensorManager->get_mid(),
               MsgPriNormal,
               MSG_SENSOR_MGR_CMD_SEND_DATA_MH,
               MSG_QUE_NULL,
               *packet);
  F_ASSERT(er == ERR_OK);
}
#endif /* __cplusplus */
