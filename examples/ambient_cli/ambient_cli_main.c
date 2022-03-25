/****************************************************************************
 * examples/ambient_cli/ambient_cli_main.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* User defeault setting */

#define MY_CHANNEL  -1
#define MY_WRITEKEY ""

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int       field;
  FAR char  *str;
} dataset;

struct args_s
{
  int         channel;
  FAR char    *write_key;
  int         datanum;
  FAR dataset *pdata;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  fprintf(stderr,
          "Usage: %s [-c <channel>] [-w <write_key>] field=data ...\n"
          "Options:\n"
          "  -c: channel ID\n"
          "  -w: write key string\n"
          "  field=data: field is 1~10. data is string to be sent.\n"
          "\n", progname);
}

static int parse_args(FAR struct args_s *args, int argc, FAR char *argv[])
{
  int i;
  int opt;
  int num;
  FAR char *ptr;
  FAR char *nptr;

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

  args->datanum = argc - optind;

  if ((args->datanum < 1) || (args->datanum > AMBIENT_MAXNUM))
    {
      fprintf(stderr, "ERROR: Invalid datanum '%d'\n", args->datanum);
      return -EINVAL;
    }

  /* Create dataset to be sent */

  args->pdata = (dataset *)zalloc(sizeof(dataset) * args->datanum);
  if (!args->pdata)
    {
      fprintf(stderr, "ERROR: Out of memory\n");
      return -ENOMEM;
    }

  /* Parse input data of [1-8]="string"
   * e.g.) if input is '1=123.45', parse field=1 and str="123.45".
   */

  for (i = 0, num = 0; i < args->datanum; i++)
    {
      ptr = argv[optind + i];
      nptr = strchr(ptr, '=');
      if (nptr)
        {
          *nptr = '\0';
          args->pdata[num].field = atoi(ptr);
          args->pdata[num].str = nptr + 1;
          num++;
        }
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
  int i;

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

  for (i = 0; i < args.datanum; i++)
    {
      if ((AMBIENT_D1 <= args.pdata[i].field) &&
          (args.pdata[i].field <= AMBIENT_MAXNUM))
        {
          /* Set data to be sent to ambient */

          printf("Data: [%d]=%s\n", args.pdata[i].field, args.pdata[i].str);
          ret = ambient_set(ctx, args.pdata[i].field, args.pdata[i].str);
          if (ret < 0)
            {
              fprintf(stderr, "ERROR: ambient_set (%d)\n", ret);
              goto errout;
            }
        }
    }

  /* Send data to ambient */

  ret = ambient_send(ctx);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: ambient_send (%d)\n", ret);
      goto errout;
    }

errout:
  /* Delete ambient context */

  ambient_delete(ctx);

  if (args.pdata)
    {
      free(args.pdata);
    }

  return ret;
}
