/****************************************************************************
 * examples/ambient_gnsslogger/ambient_gnsslogger_main.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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
#include <ambient.h>
#include "gnss_util.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* User defeault setting */

#define MY_CHANNEL -1
#define MY_WRITEKEY ""

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct args_s
{
  int         channel;
  FAR char    *write_key;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  fprintf(stderr,
          "Usage: %s [-c <channel>] [-w <write_key>]\n"
          "Options:\n"
          "  -c: channel ID\n"
          "  -w: write key string\n"
          "\n", progname);
}

static int parse_args(FAR struct args_s *args, int argc, FAR char *argv[])
{
  int opt;

  /* default settings */

  args->channel   = MY_CHANNEL;
  args->write_key = MY_WRITEKEY;

  while ((opt = getopt(argc, argv, "c:w:")) != ERROR)
    {
      switch (opt)
        {
          case 'c':
            args->channel = atoi(optarg);
            break;
          case 'w':
            args->write_key = optarg;
            break;
          case '?':
          case ':':
          default:
            show_usage(argv[0]);
            return ERROR;
        }
    }

  /* Check arguments */

  if (args->channel < 0)
    {
      fprintf(stderr, "ERROR: Invalid channel '%d'\n", args->channel);
      return -EINVAL;
    }

  if (strlen(args->write_key) == 0)
    {
      fprintf(stderr, "ERROR: No input write_key\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  FAR ambient_ctx_t *ctx;
  struct args_s args;
  int gnss_fd;
  sigset_t mask;

  memset(&args, 0, sizeof(args));

  ret = parse_args(&args, argc, argv);
  if (ret < 0)
    {
      return ret;
    }

  /* Create ambient context */

  printf("Create: channel=%d write_key=%s\n", args.channel, args.write_key);

  ctx = ambient_create(args.channel, args.write_key);
  if (ctx == NULL)
    {
      fprintf(stderr, "ERROR: ambient_create\n");
      return -ENOMEM;
    }

  /* Initialze GNSS driver */

  gnss_fd = init_gnss(&mask);
  if (gnss_fd < 0)
    {
      fprintf(stderr, "ERROR: initialie gnss driver\n");
      return ERROR;
    }

  start_gnss(gnss_fd);

  /* Main loop of this application */

  while (1)
    {
      int state;
      int sv_cnt = 0;
      float lat;
      float lng;
      struct datetime_s dt;

      /* Get a result of GNSS measurement */

      state = get_position(gnss_fd, &mask, &sv_cnt, &dt, &lat, &lng);
      printf("gnss: state=%d, satellites=%d\n", state, sv_cnt);

      if (state == GNSS_UTIL_STATE_FIXED)
        {
          /* Set lantitude and longitude as string data */

          printf("lat=%lf lng=%lf\n", lat, lng);

          ambient_set_double(ctx, AMBIENT_LAT, lat);
          ambient_set_double(ctx, AMBIENT_LNG, lng);

          /* Send data to ambient */

          ret = ambient_send(ctx);
          if (ret < 0)
            {
              fprintf(stderr, "ERROR: ambient_send (%d)\n", ret);
            }

          /* Wait during the period of 10 seconds */

          sleep(10);
        }
      else if (state == GNSS_UTIL_STATE_ERROR)
        {
          fprintf(stderr, "ERROR: gnss state error\n");
          break;
        }
    }

  /* Finalize GNSS driver */

  fin_gnss(gnss_fd, &mask);

  /* Delete ambient context */

  ambient_delete(ctx);

  return 0;
}
