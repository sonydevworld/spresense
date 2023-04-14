/****************************************************************************
 * examples/fsperf/fsperf_main.c
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
#include <string.h>
#include <float.h>
#include <assert.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#define CALC_BITRATE(sz, sec) ((sz) * 8 / (sec) / 1024 / 1024)

#define FSPERF_ERROR (-1.0)

/* If you want to change STDIO buffer size, enable the following definition
 */

/* #define STDIO_BUFFER_SIZE 4096 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct testitem
{
  const char *name;
  double (*func)(const char *, size_t);
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ts_sec
 *
 * Description:
 *    Convert a timespec to a double.
 *
 ****************************************************************************/

static double ts_sec(const struct timespec *ts)
{
  return (double)ts->tv_sec + (double)ts->tv_nsec / 1e9;
}

/****************************************************************************
 * Name: ts_diff
 *
 * Description:
 *   Return the diff of two timespecs in second.
 *
 ****************************************************************************/

static double ts_diff(const struct timespec *a, const struct timespec *b)
{
  return ts_sec(a) - ts_sec(b);
}

#ifdef CONFIG_EXAMPLES_FSPERF_FWRITEREAD
/****************************************************************************
 * Name: fsperf_fwrite
 *
 * Description:
 *   Measure fwrite performance
 *
 ****************************************************************************/

static double fsperf_fwrite(const char *filepath, size_t sz)
{
  FILE            *fp;
  uint8_t         *buf;
  int             ret;
  char            filename[64];
  struct timespec start;
  struct timespec end;

  /* set up */

  buf = (uint8_t *)malloc(sz);
  if (buf == NULL)
    {
      printf("ERROR: Fail to allocate buffer size=%d\n", sz);
      return FSPERF_ERROR;
    }

  memset(buf, 0x55, sz);

  snprintf(filename, sizeof(filename), "%s%d", filepath, sz);

  /* remove in advance */

  unlink(filename);

  fp = fopen(filename, "w");
  if (fp == NULL)
    {
      printf("ERROR: Fail to open %s\n", filename);
      free(buf);
      return FSPERF_ERROR;
    }

#ifdef STDIO_BUFFER_SIZE
  ret = setvbuf(fp, NULL, _IOLBF, STDIO_BUFFER_SIZE);
  ASSERT(ret == OK);
#endif

  /* measure */

  ret = clock_gettime(CLOCK_MONOTONIC, &start);
  ASSERT(ret == OK);

  ret = fwrite(buf, 1, sz, fp);
  ASSERT(ret == sz);

  ret = clock_gettime(CLOCK_MONOTONIC, &end);
  ASSERT(ret == OK);

  /* tear down */

  fclose(fp);

  free(buf);

  return ts_diff(&end, &start);
}

/****************************************************************************
 * Name: fsperf_fread
 *
 * Description:
 *   Measure fread performance
 *
 ****************************************************************************/

static double fsperf_fread(const char *filepath, size_t sz)
{
  FILE            *fp;
  uint8_t         *buf;
  int             ret;
  char            filename[64];
  struct timespec start;
  struct timespec end;

  /* set up */

  buf = (uint8_t *)malloc(sz);
  if (buf == NULL)
    {
      printf("ERROR: Fail to allocate buffer size=%d\n", sz);
      return FSPERF_ERROR;
    }

  memset(buf, 0xaa, sz);

  snprintf(filename, sizeof(filename), "%s%d", filepath, sz);

  fp = fopen(filename, "r");
  if (fp == NULL)
    {
      printf("ERROR: Fail to open %s\n", filename);
      free(buf);
      return FSPERF_ERROR;
    }

#ifdef STDIO_BUFFER_SIZE
  ret = setvbuf(fp, NULL, _IOLBF, STDIO_BUFFER_SIZE);
  ASSERT(ret == OK);
#endif

  /* measure */

  ret = clock_gettime(CLOCK_MONOTONIC, &start);
  ASSERT(ret == OK);

  ret = fread(buf, 1, sz, fp);
  ASSERT(ret == sz);

  ret = clock_gettime(CLOCK_MONOTONIC, &end);
  ASSERT(ret == OK);

  /* tear down */

  fclose(fp);

  /* verify */

  for (int k = 0; k < sz; k++)
    {
      ASSERT(buf[k] == 0x55);
    }

  free(buf);

  return ts_diff(&end, &start);
}
#endif /* CONFIG_EXAMPLES_FSPERF_FWRITEREAD */

#ifdef CONFIG_EXAMPLES_FSPERF_WRITEREAD
/****************************************************************************
 * Name: fsperf_write
 *
 * Description:
 *   Measure write performance
 *
 ****************************************************************************/

static double fsperf_write(const char *filepath, size_t sz)
{
  int             fd;
  uint8_t         *buf;
  int             ret;
  char            filename[64];
  struct timespec start;
  struct timespec end;

  /* set up */

  buf = (uint8_t *)malloc(sz);
  if (buf == NULL)
    {
      printf("ERROR: Fail to allocate buffer size=%d\n", sz);
      return FSPERF_ERROR;
    }

  memset(buf, 0x55, sz);

  snprintf(filename, sizeof(filename), "%s%d", filepath, sz);

  /* remove in advance */

  unlink(filename);

  fd = open(filename, O_CREAT | O_RDWR);
  if (fd < 0)
    {
      printf("ERROR: Fail to open %s\n", filename);
      free(buf);
      return FSPERF_ERROR;
    }

  /* measure */

  ret = clock_gettime(CLOCK_MONOTONIC, &start);
  ASSERT(ret == OK);

  ret = write(fd, buf, sz);
  ASSERT(ret == sz);

  ret = clock_gettime(CLOCK_MONOTONIC, &end);
  ASSERT(ret == OK);

  /* tear down */

  close(fd);

  free(buf);

  return ts_diff(&end, &start);
}

/****************************************************************************
 * Name: fsperf_read
 *
 * Description:
 *   Measure read performance
 *
 ****************************************************************************/

static double fsperf_read(const char *filepath, size_t sz)
{
  int             fd;
  uint8_t         *buf;
  int             ret;
  char            filename[64];
  struct timespec start;
  struct timespec end;

  /* set up */

  buf = (uint8_t *)malloc(sz);
  if (buf == NULL)
    {
      printf("ERROR: Fail to allocate buffer size=%d\n", sz);
      return FSPERF_ERROR;
    }

  memset(buf, 0xaa, sz);

  snprintf(filename, sizeof(filename), "%s%d", filepath, sz);

  fd = open(filename, O_RDWR);
  if (fd < 0)
    {
      printf("ERROR: Fail to open %s\n", filename);
      free(buf);
      return FSPERF_ERROR;
    }

  /* measure */

  ret = clock_gettime(CLOCK_MONOTONIC, &start);
  ASSERT(ret == OK);

  ret = read(fd, buf, sz);
  ASSERT(ret == sz);

  ret = clock_gettime(CLOCK_MONOTONIC, &end);
  ASSERT(ret == OK);

  /* tear down */

  close(fd);

  /* verify */

  for (int k = 0; k < sz; k++)
    {
      ASSERT(buf[k] == 0x55);
    }

  free(buf);

  return ts_diff(&end, &start);
}
#endif /* CONFIG_EXAMPLES_FSPERF_WRITEREAD */

/****************************************************************************
 * Name: show_speed
 *
 * Description:
 *    Show the speed for read/write performance
 *
 ****************************************************************************/

void show_speed(const char *str, size_t sz, double sec)
{
  double speed;

  speed = CALC_BITRATE(sz, sec);

  printf(" %4d [KB] / %7.3lf [ms], speed= %7.3lf [Mbps]\n",
         sz / 1024, sec * 1000, speed);
}

/****************************************************************************
 * Name: show_summary
 *
 * Description:
 *    Show the statistics information for read/write performance
 *
 ****************************************************************************/

void show_summary(const char *str, size_t sz, double sec_avg,
                  double sec_min, double sec_max, int num)
{
  double speed_avg;
  double speed_min;
  double speed_max;

  speed_avg = CALC_BITRATE(sz, sec_avg);
  speed_max = CALC_BITRATE(sz, sec_min);
  speed_min = CALC_BITRATE(sz, sec_max);

  printf("--- %6s summary: Size: %4d [KB], "
        "Min: %7.3lf, Avg: %7.3lf, Max: %7.3lf [Mbps]\n",
         str, sz / 1024, speed_min, speed_avg, speed_max);
}

/****************************************************************************
 * Name: show_usage
 *
 * Description:
 *    Show the usage of this application
 *
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  fprintf(stderr, "Usage: %s [-i] [-n <num>] -f <file>\n", progname);
  fprintf(stderr, "FileSystem Performance Monitor:\n"
                  "  -i: Display the information of each result\n"
                  "  -n: Specify the repeat count, default is %d\n"
                  "  -f: Specify the path to prefix of example files\n"
                  "      e.g.\n"
                  "      \"-f /mnt/spif/test\" on SPI-Flash\n"
                  "      \"-f /mnt/sd0/test\"  on SD card\n"
                  "      \"-f /mnt/emmc/test\" on eMMC board\n",
                  CONFIG_EXAMPLES_FSPERF_REPEAT_COUNT);
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  double sec;
  double sec_sum;
  double sec_min;
  double sec_max;
  double sec_avg;
  size_t sz;
  int    num;
  int    i;
  int    option;
  int    repeat_num = CONFIG_EXAMPLES_FSPERF_REPEAT_COUNT;
  bool   info = false;
  char   *filepath = NULL;

  struct testitem item[] = {
#ifdef CONFIG_EXAMPLES_FSPERF_FWRITEREAD
    { "fwrite", fsperf_fwrite },
    { "fread",  fsperf_fread, },
#endif
#ifdef CONFIG_EXAMPLES_FSPERF_WRITEREAD
    { "write" , fsperf_write  },
    { "read",   fsperf_read,  },
#endif
  };

  while ((option = getopt(argc, argv, "in:f:")) != ERROR)
    {
      switch (option)
        {
          case 'i':
            info = true;
            break;
          case 'n':
            repeat_num = atoi(optarg);
            break;
          case 'f':
            filepath = optarg;
            break;
          default:
            show_usage(argv[0]);
        }
    }

  if (!filepath)
    {
      show_usage(argv[0]);
    }

  printf("File access speed monitor!!\n");

  for (i = 0; i < sizeof(item) / sizeof(item[0]); i++)
    {
      for (sz = 1024; sz <= 1024 * 1024; sz <<= 1)
        {
          sec_sum = 0.0;
          sec_min = DBL_MAX;
          sec_max = DBL_MIN;
          for (num = 0; num < repeat_num; num++)
            {
              sec = item[i].func(filepath, sz);
              if (sec == FSPERF_ERROR)
                {
                  break;
                }

              if (info)
                {
                  show_speed(item[i].name, sz, sec);
                }

              sec_min = MIN(sec_min, sec);
              sec_max = MAX(sec_max, sec);
              sec_sum += sec;
            }

          if (num > 0)
            {
              sec_avg = sec_sum / num;
              show_summary(item[i].name, sz,
                           sec_avg, sec_min, sec_max, num);
            }
        }
    }

  return 0;
}
