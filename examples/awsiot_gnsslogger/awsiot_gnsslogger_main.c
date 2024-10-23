/****************************************************************************
 * examples/awsiot_gnsslogger/awsiot_gnsslogger_main.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "app_config.h"

#include "awsiot_connect.h"
#include "gnss_util.h"
#include "led_util.h"

#ifdef CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST
#include "awsiot_test.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INI_FILE_PATH "/mnt/spif/aws_iot.ini"

#define LOCATION_TOPIC      "data/%s/gps"
#define LOCATION_TOPIC_FMT  "{ \"timestamp\": %lu, \"device_loc\": { \"lat\": %f, \"lon\": %f } }"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static awsiot_app_config app_config;
static char topic_name[64];
static char topic_msg[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_system_time()
 *
 * Description:
 *   Set system time as date of arguments.
 ****************************************************************************/

static void set_system_time(uint16_t year, uint8_t month, uint8_t day,
    uint8_t hour, uint8_t minute)
{
  struct tm tm;
  struct timespec ts;

  memset(&tm, 0, sizeof(tm));
  tm.tm_sec  = 0;           /* Seconds (0-61, allows for leap seconds) */
  tm.tm_min  = minute;      /* Minutes (0-59) */
  tm.tm_hour = hour;        /* Hours (0-23) */
  tm.tm_mday = day;         /* Day of the month (1-31) */
  tm.tm_mon  = month - 1;   /* Month (0-11) */
  tm.tm_year = year - 1900; /* Years since 1900 */

  ts.tv_sec = mktime(&tm);
  ts.tv_nsec = 0;

  clock_settime(CLOCK_REALTIME, &ts);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main()
 *
 * Description:
 *   main() of this command.
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  AWS_IoT_Client client;

#ifndef CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST
  int gnss_fd;
  int sv_cnt;
  sigset_t mask;
  float lat, lng;
  struct datetime_s dt;
  int is_connected = 0;
#else
  /* Parse arguments for simple test */

  if (test_parse_arg(argc, argv) != 0)
    {
      return -1;
    }
#endif

  /* Initialize LEDs and turn on every LEDs. */

  init_leds();
  set_leds(0x0F);

  /* Parse INI file for AWS IoT settings */

  if (parse_inifile(INI_FILE_PATH, &app_config)<0)
    {
      /* Error message is displayed in parse_inifile() */

      delete_config(&app_config);
      return -1;
    }

#ifdef CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST
  printf("Set dummy time as 2021/1/1 10:00:00 for verifying date of cert file.\n");
  set_system_time(2021, 1, 1, 10, 0);

  /* Connect to AWS Endpoint. */

  printf("Connecting to AWS IoT\n");
  if (connect_awsiot(&client, &app_config)>=0)
    {

      /* Execute TEST code */

      test_execute(&client);

      printf("Disconnect from awsiot.\n");
      disconnect_awsiot(&client);
    }
  else
    {
      printf("Connection error...\n");
    }

#else /* CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST */

  /* Generate Topic name with Client ID */

  snprintf(topic_name, sizeof(topic_name), LOCATION_TOPIC, app_config.client_id);

  /* Initialize GNSS driver */

  gnss_fd = init_gnss(&mask);
  if (gnss_fd < 0)
    {
      delete_config(&app_config);
      return -1;
    }

  /* Start GNSS measurement */

  start_gnss(gnss_fd);

  /* Main loop of this application */

  while(1)
    {
      sv_cnt = 0;

      /* Get a result of GNSS measurement */

      switch (get_position(gnss_fd, &mask, &sv_cnt, &dt, &lat, &lng))
        {
          case GNSS_UTIL_STATE_FIXED:

            /* If the connection to AWS IoT server is not established. */

            if (!is_connected)
              {

                /* Now we can connect to AWS IoT server
                 * because the current date and time is fixed.
                 * It is needed for verifying cart files.
                 * So at first, the system time should be updated.
                 */

                printf("Set UTC time as %d/%d/%d %02d:%02d:00.\n",
                        dt.date.year, dt.date.month, dt.date.day,
                        dt.time.hour, dt.time.minute);
                set_system_time(dt.date.year, dt.date.month, dt.date.day,
                    dt.time.hour, dt.time.minute);

                /* Connect to AWS Endpoint. */

                printf("Connecting to AWS IoT\n");
                if (connect_awsiot(&client, &app_config)<0)
                  {
                    printf("Failed to connect AWS IoT..\n");
                    goto error_exit;
                  }

                is_connected = 1;
              }

            snprintf(topic_msg, sizeof(topic_msg), LOCATION_TOPIC_FMT,
                time(NULL), lat, lng);
            printf("Location is fixed. %d satellites is captured.\n", sv_cnt);
            printf("    Publish data to %s topic as <<%s>>\n",
               topic_name, topic_msg);

            /* LED control */

            if (sv_cnt > 7) sv_cnt = 7;
            set_leds(sv_cnt | 0x08);

            /* Publish this data */

            publish_topic(&client, topic_name, (uint8_t *)topic_msg, strlen(topic_msg));
            break;

          case GNSS_UTIL_STATE_SVCAP:
            printf("Location is NOT fixed. %d satellites is captured.\n", sv_cnt);

            /* LED control */

            if (sv_cnt > 7) sv_cnt = 7;
            set_leds(sv_cnt);

            /* Yield 10 ms for sending keep alive. */

            aws_iot_mqtt_yield(&client, 10);
            break;

          case GNSS_UTIL_STATE_TOUT: /* Timeout */
            printf("Time out signal wait.\n");

            /* Yield 10 ms for sending keep alive. */

            aws_iot_mqtt_yield(&client, 10);
            break;

          case GNSS_UTIL_STATE_ERROR: /* Error is occurred */
            printf("Known error is returned.. ");
          default:
            printf("Error happened.\n");
            break;
        }
    }

  printf("Disconnect from awsiot.\n");
  disconnect_awsiot(&client);

error_exit:

  fin_gnss(gnss_fd, &mask);

#endif /* CONFIG_EXAMPLES_AWSIOT_GNSSLOGGER_PUBSUB_TEST */

  delete_config(&app_config);

  set_leds(0x00);

  return 0;
}
