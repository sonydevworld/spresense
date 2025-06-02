/****************************************************************************
 * examples/gyrocompass_pwbimu/gyrocompass_pwbimu_main.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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
#include <poll.h>
#include <math.h>

#include <nuttx/sensors/cxd5602pwbimu.h>

#include "imu_utils.h"
#include "gyrocompass.h"

#ifdef CONFIG_EXAMPLES_GYROCOMPASS_TEST
#  include "test_on_pc/gyrocompass_test.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_A_POSTURE (1)
#define END_CAPTURE       (0)
#define QUIT_CAPTURE      (-1)

#define MAX_POSTURE             (32)

#define DEFAULT_SAMPLERATE      (1920)
#define DEFAULT_GYROSCOPERANGE  (125)
#define DEFAULT_ACCELRANGE      (4)

#define COLLECTION_DATACNT  \
          (DEFAULT_SAMPLERATE * CONFIG_EXAMPLES_GYROCOMPASS_COLLECTTIME)
#define COUNTDOWN_TOSTART_SEC (3)
#define COUNTDOWN_TOSTART (DEFAULT_SAMPLERATE * COUNTDOWN_TOSTART_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dvec3_s g_gyro[MAX_POSTURE];  /* Averages of gyro data
                                             * for each posture */

#ifdef CONFIG_EXAMPLES_GYROCOMPASS_TEST
static char *g_filenames[MAX_POSTURE];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/** wait_input() : Wait key input from terminal. */

static char wait_input(bool en_anykey)
{
  char c;
  struct pollfd pfd;
  int ret;

  pfd.fd     = fileno(stdin);
  pfd.events = POLLIN;

  while (1)
    {
      ret = poll(&pfd, 1, -1);
      if (ret <= 0)
        {
          return 'Q';
        }
      else if (pfd.revents & POLLIN)
        {
          read(pfd.fd, &c, 1);
          if (c == 'q' || c == 'c' || en_anykey)
            {
              return c;
            }
        }
    }

  return ' ';
}

/** setup_next_posture() : Interact with the user to determine the posture
 *                         for capturing IMU data.
 */

static int setup_next_posture(int cnt)
{
  char c;
  int ret = CAPTURE_A_POSTURE;

retry_setup:

  if (cnt == 0)
    {
      printf("Press 'c' key with the Spresense board stationary\n");
    }
  else
    {
      printf("Change Spresense board to a different orientation "
             "than the previous one and press 'c' key.\n");
      if (cnt >= 3)
        {
          printf("If you want to stop capturing more posture, press 'q' key\n");
        }
    }

  printf("Press key (c%s): ", cnt >= 3 ? "/q" : "");
  fflush(stdout);

  c = wait_input(false);
  printf("%c\n\n", c);
  switch (c)
    {
      case 'c':
        ret = CAPTURE_A_POSTURE;
        break;

      case 'q':
        if (cnt < 3)
          {
            printf("Now, Posture data is less than 3.\n"
                   "Do you want to quit this app? Press 'q' key then : ");
            fflush(stdout);
            c = wait_input(true);
            printf("%c\n", c);
            if (c == 'q')
              {
                printf("Quit this app\n");
                ret = QUIT_CAPTURE;
              }
            else
              {
                goto retry_setup;
              }
          }
        else
          {
            ret = END_CAPTURE;
          }

        break;

      case 'Q':
        ret = QUIT_CAPTURE;
        break;

      default:
        goto retry_setup;
        break;
    }

  return ret;
}

/** capture_imudata() : Capture IMU data in the determined posture. */

static int capture_imudata(int fd, int no, struct dvec3_s *avrg,
                           cxd5602pwbimu_data_t *imudata,
                           int capcnt, int unit)
{
  int i;

  memset(avrg, 0, sizeof(struct dvec3_s));

  printf("Capture No.%d\nCount Down to start T :", no); fflush(stdout);
  for (i = 0; i < COUNTDOWN_TOSTART; i++)
    {
      if (!pwbimu_read_imudata(fd, imudata))
        {
          printf("Canceled!!\n");
          return -1;
        }

      if ((i % unit) == 0)
        {
          printf("-%d,", COUNTDOWN_TOSTART_SEC - (i / unit));
          fflush(stdout);
        }
    }

  printf("0\nStart Capturing for %d seconds\n",
         CONFIG_EXAMPLES_GYROCOMPASS_COLLECTTIME);
  for (i = 0; i < capcnt; i++)
    {
      if (!pwbimu_read_imudata(fd, imudata))
        {
          printf("Canceled!!\n");
          return -1;
        }

      if ((i % unit) == 0)
        {
          printf(".");
          fflush(stdout);
        }

      avrg->x += imudata->gx;
      avrg->y += imudata->gy;
      avrg->z += imudata->gz;
    }

  avrg->x /= (double)capcnt;
  avrg->y /= (double)capcnt;
  avrg->z /= (double)capcnt;

  printf("\nDone\n\n");
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int i;
  int fd;
  int posture_cnt;
  struct dvec3_s bias_vec;
  double heading;
  cxd5602pwbimu_data_t imudata;

#ifdef CONFIG_EXAMPLES_GYROCOMPASS_TEST
  if (argc == 2)
    {
      /* Calculate heading angle from test data in a directory
       * in argv[1]. (Same behavior as gyrocompass.py in
          examples/cxd5602pwbimu_logger/pc_tools/gyrocompass
       */

      return gyrocompass_with_testdata(argv[1],
                                       g_filenames, g_gyro, MAX_POSTURE);
    }
#endif

  fd = pwbimu_start_sensing(DEFAULT_SAMPLERATE, DEFAULT_ACCELRANGE,
                            DEFAULT_GYROSCOPERANGE, 1);
  if (fd < 0)
    {
      printf("IMU Initialize Error\n");
      return -1;
    }

  pwbimu_drop_data50ms(fd, DEFAULT_SAMPLERATE, &imudata);

  /* Acquire gyro data for 3 or more attitudes. */

  for (posture_cnt = 0; posture_cnt < MAX_POSTURE; posture_cnt++)
    {
      /* Setup posture */

      switch (setup_next_posture(posture_cnt))
        {
          case CAPTURE_A_POSTURE:
            break;

          case END_CAPTURE:
            goto calculate_direction;
            break;  /* Just in case */

          case QUIT_CAPTURE:
          default:
            goto end_this_app;
            break;  /* Just in case */
        }

      /* Capture and calc average on the posture */

      if (capture_imudata(fd, posture_cnt + 1, &g_gyro[posture_cnt], &imudata,
                          COLLECTION_DATACNT, DEFAULT_SAMPLERATE) < 0)
        {
          goto end_this_app;
        }
    }

calculate_direction:

  /* Calculate gyro bias by cicle fitting */

  if (calc_bias_circlefitting(g_gyro, posture_cnt, &bias_vec))
    {
      printf("Invalid IMU data.\n");
      goto end_this_app;
    }

  for (i = 0; i < posture_cnt; i++)
    {
      /* Calculate heading angle */

      heading = calc_device_heading2d(g_gyro[i].x - bias_vec.x,
                                      g_gyro[i].y - bias_vec.y);
      printf("  Capture No.%d, %.2f\n", i + 1, heading);
    }

end_this_app:

  pwbimu_terminate(fd);

  return 0;
}
