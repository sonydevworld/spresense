/****************************************************************************
 * examples/awsiot_gnsslogger/awsiot_connect.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <string.h>

#include "app_config.h"
#include "awsiot_connect.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: disconnection_handler()
 *
 * Description:
 *   This function is called back from AWS IoT libraries when the connection 
 *   is expired.
 ****************************************************************************/

static void disconnection_handler(AWS_IoT_Client *pClient, void *data)
{
  IOT_WARN("MQTT Disconnect");
  IoT_Error_t rc = FAILURE;

  if(NULL == pClient)
    {
      return;
    }

  IOT_UNUSED(data);

  if(aws_iot_is_autoreconnect_enabled(pClient))
    {
      IOT_INFO("Auto Reconnect is enabled, Reconnecting attempt will start now");
    }
  else
    {
      IOT_WARN("Auto Reconnect not enabled. Starting manual reconnect...");
      rc = aws_iot_mqtt_attempt_reconnect(pClient);
      if(NETWORK_RECONNECTED == rc)
        {
          IOT_WARN("Manual Reconnect Successful");
        }
      else
        {
          IOT_WARN("Manual Reconnect Failed - %d", rc);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: connect_awsiot()
 *
 * Description:
 *   Connect to AWS IoT endpoint according to configuration parameters.
 ****************************************************************************/

int connect_awsiot(AWS_IoT_Client *client, awsiot_app_config *config)
{
  IoT_Error_t rc = FAILURE;

  IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
  IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;
  
  /* Initialize MQTT */

  mqttInitParams.enableAutoReconnect = false;
  mqttInitParams.pHostURL = config->host_url;
  mqttInitParams.port = config->host_port;
  mqttInitParams.pRootCALocation = config->root_ca_path;
  mqttInitParams.pDeviceCertLocation = config->crt_path;
  mqttInitParams.pDevicePrivateKeyLocation = config->privkey_path;
  mqttInitParams.mqttCommandTimeout_ms = config->cmd_timeout;
  mqttInitParams.tlsHandshakeTimeout_ms = config->ssl_timeout;
  mqttInitParams.isSSLHostnameVerify = true;
  mqttInitParams.disconnectHandler = disconnection_handler;
  mqttInitParams.disconnectHandlerData = NULL;

  rc = aws_iot_mqtt_init(client, &mqttInitParams);

  if(SUCCESS != rc)
    {
      printf("aws_iot_mqtt_init returned error : %d \n", rc);
      return -1;
    }

  /* MQTT Connect */

  connectParams.keepAliveIntervalInSec = config->keepalive_ivtime;
  connectParams.isCleanSession = true;
  connectParams.MQTTVersion = MQTT_3_1_1;
  connectParams.pClientID = config->client_id;
  connectParams.clientIDLen = (uint16_t) strlen(config->client_id);
  connectParams.isWillMsgPresent = false;

  rc = aws_iot_mqtt_connect(client, &connectParams);

  if(SUCCESS != rc)
    {
      printf("Error(%d) connecting to %s:%d \n", rc, mqttInitParams.pHostURL, mqttInitParams.port);
      return -2;
    }

  /* Set auto reconnection */

  rc = aws_iot_mqtt_autoreconnect_set_status(client, true);

  if(SUCCESS != rc)
    {
      printf("Unable to set Auto Reconnect to true - %d\n", rc);
      return -3;
    }

  /* Set auto reconnection */

  return 0;
}

/****************************************************************************
 * Name: disconnect_awsiot()
 *
 * Description:
 *   Disconnect from AWS IoT endpoint.
 ****************************************************************************/

int disconnect_awsiot(AWS_IoT_Client *client)
{
  aws_iot_mqtt_disconnect(client);
  return 0;
}

/****************************************************************************
 * Name: subscribe_topic()
 *
 * Description:
 *   Subscribe a specific topic.
 ****************************************************************************/

int subscribe_topic(AWS_IoT_Client *client, const char *topic, pApplicationHandler_t handler)
{
  IoT_Error_t rc;

  rc = aws_iot_mqtt_subscribe(client, topic, strlen(topic), QOS0, handler, NULL);

  if(SUCCESS != rc)
    {
      printf("Unable to subscribe - %d\n", rc);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: publish_topic()
 *
 * Description:
 *   Publish a data onto a specific topic.
 ****************************************************************************/

int publish_topic(AWS_IoT_Client *client, const char *topic, uint8_t *data, int data_len)
{
  IoT_Error_t rc;

  IoT_Publish_Message_Params publishParam;

  publishParam.qos = QOS1;
  publishParam.payload = (void *) data;
  publishParam.isRetained = 0;
  publishParam.payloadLen = data_len;

  rc = aws_iot_mqtt_publish(client, topic, strlen(topic), &publishParam);

  if (rc == MQTT_REQUEST_TIMEOUT_ERROR)
    {
      printf("QOS1 publish ack not received.\n");
      return -2;
    }

  if (rc != SUCCESS)
    {
      printf("An error occurred of the publish : %d.\n", rc);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: wait_subscribe_message()
 *
 * Description:
 *   Wait for a message from the subscribed topic. This will return in 100ms
 *   at latest even if the message is not received.
 ****************************************************************************/

void wait_subscribe_message(AWS_IoT_Client *client)
{
  aws_iot_mqtt_yield(client, 100);
}
