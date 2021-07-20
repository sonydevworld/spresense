/****************************************************************************
 * examples/digital_filter/edge16_main.c
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
#include <string.h>
#include <sys/time.h>
#include <digital_filter/edge_detection.h>

#include "wav.h"
#include "perfmon.h"

/****************************************************************************
 * Private Definition
 ****************************************************************************/

/* #define DPRINT printf */
#define DPRINT(...)

#define BASE_DIR  "/mnt/sd0/"

#define BLOCK_SIZE      (256)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int16_t sig_data[BLOCK_SIZE];
static int16_t out_data[BLOCK_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/** clear_data() */

static void clear_data(int16_t *dat, int len)
{
  memset(dat, 0, sizeof(int16_t) * len);
}

/** detect_edge() */

static void detect_edge(edge_detects_t *detector, int16_t *data,
    int16_t *odat, int len)
{
  int edge_pos = 0;
  int offset = 0;

  /* When an edge is found, mark it on the same offset of the input data
   * into output data as 3000. And keep searching another edge in remaining
   * input data until end of it.
   */

  DPRINT("Enter detect_edge(data size = %d)\n", len);
  while ((edge_pos = edge_detects(detector, &data[offset], len)) >= 0)
    {
      DPRINT(" Detected : %d\n", edge_pos);

      /* Mark the edge */

      odat[offset + edge_pos] = 30000;

      /* edge_pos value is detected offset position of input data.
       * So add 1 for next searching
       */

      edge_pos++;

      /* Calculate length of remaining data */

      len -= edge_pos;

      /* Just in case, care about if length is negative. */

      if (len < 0)
        {
          printf("error...%d\n", len);
        }

      /* Calculate Data offset */

      offset += edge_pos;

      DPRINT("    next(%d, %d)\n", offset, len);
    }
}

/** print_usage() */

static void print_usage(const char *app)
{
  printf("Usage: %s <input file> <type> <keep_num> <boundary data1>"
     " <boundary data2> (<boundary datan>...)\n"
     "       input file     : Input monoral wav file\n"
     "       type           : rise or fall\n"
     "       keep_num       : bigger or eqal 0\n"
     "       boundary data1 : 1st boundary data\n"
     "       boundary data2 : 2nd boundary data\n"
     "       boundary datan : Additional boundary data. this is option.\n",
     app);
}

/****************************************************************************
 * Main Functions of this example
 ****************************************************************************/

int main(int argc, char **argv)
{
  int n;
  struct timeval time_keep;
  uint64_t total_time = 0;
  int type;
  int16_t *edge_data;
  int edge_data_num;
  uint32_t keep_num;

  edge_detects_t *detector;

  wav_instance_t input_wav;
  wav_instance_t result_wav;

  /* Check arguments */

  if (argc < 6)
    {
      print_usage(argv[0]);
      return -1;
    }

  if (!strncmp("rise", argv[2], 5))
    {
      type = EDGE_DETECT_TYPE_RISE;
    }
  else if (!strncmp("fall", argv[2], 5))
    {
      type = EDGE_DETECT_TYPE_FALL;
    }
  else
    {
      print_usage(argv[0]);
      return -1;
    }

  keep_num = (uint32_t)atoi(argv[3]);

  edge_data_num = argc - 4;
  edge_data = (int16_t *)malloc(sizeof(int16_t) * edge_data_num);
  if (edge_data == NULL)
    {
      printf("No memory...\n");
      return -1;
    }

  for (n = 4; n < argc; n++)
    {
      edge_data[n - 4] = (int16_t)atoi(argv[n]);
    }

  /* Open input wav file */

  if (open_input_wavfile(&input_wav, argv[1]))
    {
      free(edge_data);
      printf("Error: could not open file %s\n", argv[1]);
      return -1;
    }

  /* Create edge_detect instance as rise type and keep block mode. */

  detector = edge_detection_creates(edge_data, edge_data_num, keep_num, type);
  if (detector == NULL)
    {
      close_wavfile(&input_wav);
      free(edge_data);
      printf("Error: couldn't create rise edge_detector\n");
      return -1;
    }

  /* Open output wav file to store the result of edge detection */

  result_wav.fs     = input_wav.fs;
  result_wav.bits   = input_wav.bits;
  result_wav.length = input_wav.length;
  if (open_output_wavfile(&result_wav, BASE_DIR "edge.wav"))
    {
      edge_detection_deletes(detector);
      close_wavfile(&input_wav);
      free(edge_data);
      printf("Error: could not open output file : " BASE_DIR "edge.wav\n");
      return -1;
    }

  /* Clear output buffer */

  clear_data(out_data, BLOCK_SIZE);

  /* Main loop : repeat "read data/detect edge/write data" */

  for (n = 0; n < (input_wav.length - BLOCK_SIZE + 1); n += BLOCK_SIZE)
    {
      DPRINT("==== Read data index = %d ====\n", n);

      /* Read input data from wav file */

      read_wavdata16(&input_wav, sig_data, BLOCK_SIZE);

      DPRINT("** %s \n", type == EDGE_DETECT_TYPE_RISE ? "rise" : "fall" );

      /* Mark edge position on out_data if it found */

      measure_interval(&time_keep);
      detect_edge(detector, sig_data, out_data, BLOCK_SIZE);
      total_time += (uint64_t)measure_interval(&time_keep);

      /* Save out_data in "edge.wav" file */

      write_wavdata16(&result_wav, out_data, BLOCK_SIZE);

      /* Clear "out_data" with 0 all for next use */

      clear_data(out_data, BLOCK_SIZE);
    }

  printf("Execution finished.\n");
  printf("Total size : %d, Total time %lldus\n", n, total_time);

  close_wavfile(&result_wav);
  edge_detection_deletes(detector);
  close_wavfile(&input_wav);
  free(edge_data);

  return 0;
}
