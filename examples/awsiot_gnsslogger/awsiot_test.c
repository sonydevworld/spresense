/****************************************************************************
 * examples/awsiot_gnsslogger/awsiot_test.c
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

#include <nuttx/config.h>

#ifdef CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST

#include <string.h>

#include "awsiot_connect.h"
#include "awsiot_test.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TEST_TOPIC "test/spresense/topic"
#define TOPIC_DATA_FMT "{ \"device_loc\": { \"lat\": %f, \"lon\": %f } }"

#define MSG_BUF_LEN (256)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int is_publish_test = 1;
static volatile int test_pubsub_cnt = 10;
static char message_buffer[MSG_BUF_LEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_subscribe_handler()
 *
 * Description:
 *   This function is called when a subscribed topic message is published.
 ****************************************************************************/

static void test_subscribe_handler(  AWS_IoT_Client *pClient,
                                char *topicName,
                                uint16_t topicNameLen,
                                IoT_Publish_Message_Params *params,
                                void *pData)
{
  printf("Subscribe callback\n");

  if (topicNameLen < MSG_BUF_LEN)
    {
      strncpy(message_buffer, topicName, topicNameLen);
      message_buffer[topicNameLen] = '\0';
      printf("topic name = %s\n", message_buffer);
    }
  else
    {
      printf("topic name is too long %d, more than %d\n", topicNameLen, MSG_BUF_LEN);
    }

  if (params->payloadLen < MSG_BUF_LEN)
    {
      strncpy(message_buffer, params->payload, params->payloadLen);
      message_buffer[params->payloadLen] = '\0';
      printf("payload = %s\n", message_buffer);
    }
  else
    {
      printf("payload data is too long %d, more than %d\n", params->payloadLen, MSG_BUF_LEN);
    }

  test_pubsub_cnt--;
}

/****************************************************************************
 * Name: test_print_help()
 *
 * Description:
 *   Display a help message of this command. Command line arguments are enabled
 *   in test mode of this application.
 ****************************************************************************/

static void test_print_help(void)
{
  printf("Usage: > " CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PROGNAME " (-sub) ([cnt])\n");
}

/****************************************************************************
 * Name: test_execute_publish()
 *
 * Description:
 *   Test of publisher behavior. It publish a topic message every 1 sec.
 ****************************************************************************/

static void test_execute_publish(AWS_IoT_Client *client)
{
  float lat, lng;

  /* Dummy location of Tokyo Tower */

  lat = 35.65870398413011;
  lng = 139.74543454505041;

  while (test_pubsub_cnt)
    {
      sprintf(message_buffer, TOPIC_DATA_FMT, lat, lng + ((float)test_pubsub_cnt / 1000.f));
      printf("Publish \"%s\".\n", message_buffer);
      publish_topic(client, TEST_TOPIC, (uint8_t *)message_buffer, strlen(message_buffer));
      sleep(1);
      test_pubsub_cnt--;
    }
}

/****************************************************************************
 * Name: test_execute_subscribe()
 *
 * Description:
 *   Test of subscriber behavior. It waits subscribed topic messages.
 ****************************************************************************/

static void test_execute_subscribe(AWS_IoT_Client *client)
{
  if (subscribe_topic(client, TEST_TOPIC, test_subscribe_handler) != 0)
    {
      printf("Subscribe error... finish test.\n");
    }
  else
    {
      printf("Subscribe is done.. Wait for messages..\n");

      while (test_pubsub_cnt)
        {
          wait_subscribe_message(client);
          printf("Subscribing remain %d\n", test_pubsub_cnt);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_parse_arg()
 *
 * Description:
 *   Parse command line arguments for this test application.
 ****************************************************************************/

int test_parse_arg(int argc, char *argv[])
{
  test_pubsub_cnt = 10;

  if (argc==1)
    {
      /* do nothing. */
    }
  else if (argc==2)
    {
      if (strncmp(argv[1], "-sub", 4)==0)
        {
          is_publish_test = 0;
        }
      else
        {
          test_pubsub_cnt = atoi(argv[1]);
        }
    }
  else if (argc==3)
    {
      if (strncmp(argv[1], "-sub", 4)==0)
        {
          is_publish_test = 0;
        }
      else
        {
          test_print_help();
          return -1;
        }
      test_pubsub_cnt = atoi(argv[2]);
    }
  else
    {
      test_print_help();
      return -1;
    }

  if ((test_pubsub_cnt<1) || (test_pubsub_cnt>10))
    {
      return -2;
    }

  return 0;
}

/****************************************************************************
 * Name: test_execute()
 *
 * Description:
 *   Start test execution.
 ****************************************************************************/

void test_execute(AWS_IoT_Client *client)
{
  if (is_publish_test)
    {
      test_execute_publish(client);
    }
  else
    {
      test_execute_subscribe(client);
    }
}

#endif  /* CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST */
