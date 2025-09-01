/****************************************************************************
 * examples/gyrocompass_pwbimu/test_on_pc/gyrocompass_test.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <dirent.h>
#include <sys/stat.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include "gyrocompass.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADDDVEC3(v, i)  \
  do { \
    (v)->x += (i)->gx; \
    (v)->y += (i)->gy; \
    (v)->z += (i)->gz; \
  } while(0)

#define DIVDVEC3(v, d)  \
  do { \
    (v)->x /= (d); \
    (v)->y /= (d); \
    (v)->z /= (d); \
  } while(0)

#define MAX_LINE_LEN 256

static char g_linebuf[MAX_LINE_LEN];

/*****************************************************************************
 * Private Functions for testing
 *****************************************************************************/

static const char *g_words[] =
{
#if 0
    "North", "North-northeast", "Northeast", "East-northeast", "East", "East-southeast", "Southeast", "South-southeast",
    "South", "South-southwest", "Southwest", "West-southwest", "West", "West-northwest", "Northwest", "North-northwest"
#else
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
#endif
};

static float hex_to_float(const char *hex)
{
  float f;
  sscanf(hex, "%08lx", (uint32_t *)&f);
  return f;
}

static int readdata_average(const char *filename, struct dvec3_s *avg)
{
  FILE *fp;
  int count = 0;

  char *token;
  char *tokens[8];
  int idx;

  memset(avg, 0, sizeof(struct dvec3_s));

  fp = fopen(filename, "r");
  if (!fp) return -1;

  while (fgets(g_linebuf, sizeof(g_linebuf), fp))
    {
      /* Parse numbers separated by ',' */

      idx = 0;
      token = strtok(g_linebuf, ",\n");

      while (token && idx < 8)
        {
          tokens[idx++] = token;
          token = strtok(NULL, ",\n");
        }

      if (idx == 8)
        {
          /* Data order
           * Timestamp,Tempareture,GyroX,GyroY,GyroZ,AccX,AccY,AccZ
           */

          avg->x += hex_to_float(tokens[2]);
          avg->y += hex_to_float(tokens[3]);
          avg->z += hex_to_float(tokens[4]);

          count++;
        }
    }

  if (count != 0)
    {
      DIVDVEC3(avg, (double)count);
    }

  fclose(fp);

  return count;
}

static int search_txtfiles_indir(const char *input_dir,
                                 char **files, int max)
{
  int file_count = 0;
  struct stat st;
  DIR *dir;
  struct dirent *entry;
  int basedir_len;

  /* Check if it is directory path or not */

  if (stat(input_dir, &st) != 0 || !S_ISDIR(st.st_mode))
    {
      printf("%s is not a directory\n", input_dir);
      return -1;
    }

  dir = opendir(input_dir);
  if (!dir)
    {
      perror("opendir");
      return -1;
    }

  basedir_len = strlen(input_dir);

  /* Search file in the directory */

  while ((entry = readdir(dir)) && file_count < max)
    {
      if (strstr(entry->d_name, ".txt"))  /* Find '*.txt' file */
        {
          /* Allocate memory for storing filename */

          files[file_count] = malloc(basedir_len +
                                     strlen(entry->d_name) + 2);

          /* Store the filename in the allocated memory */

          sprintf(files[file_count], "%s/%s", input_dir, entry->d_name);
          file_count++;
        }
    }

  closedir(dir);

  return file_count;
}

static const char *get_direction_word(double angle)
{
  int index = (int)((angle + 11.25) / 22.5);

  if (index >= 16)
    {
      index -= 16;
    }

  return g_words[index];
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int gyrocompass_with_testdata(const char *dir_name,
                              char **filenames, struct dvec3_s *avgs,
                              int maxfiles)
{
  int i;
  int fnum;
  double heading;
  struct dvec3_s bias_vec;

  fnum = search_txtfiles_indir(dir_name, filenames, maxfiles);
  if (fnum < 3)
    {
      printf("Error or no file in the directory\n");
      return -1;
    }

  for (i = 0; i < fnum; i++)
    {
      printf("Read from %s\n", filenames[i]);
      readdata_average(filenames[i], &avgs[i]);
    }

  if (calc_bias_circlefitting(avgs, fnum, &bias_vec))
    {
      fprintf(stderr, "Invalid IMU data.\n");
      return -1;
    }

  for (i = 0; i < fnum; i++)
    {
      /* Calculate heading angle with Avoiding Gyro Bias */

      heading = calc_device_heading2d(avgs[i].x - bias_vec.x,
                                      avgs[i].y - bias_vec.y);
      printf("  %s, %.2f, %s\n", filenames[i], heading, get_direction_word(heading));
    }

  for (i = 0; i < fnum; i++)
    {
      free(filenames[i]);
    }

  return 0;
}
