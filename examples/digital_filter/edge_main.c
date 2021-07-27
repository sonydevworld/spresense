/****************************************************************************
 * examples/digital_filter/edge_main.c
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

#define EDGE_PREVNUM  (4)
#define EDGE_KEEPNUM  (3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float sig_data[BLOCK_SIZE];
static float out_data[BLOCK_SIZE];

/* Definition of rising edge to detection */

static float rise_edge[EDGE_PREVNUM] = {5.f, 300.f, 800.f, 1000.f};

/* Definition of falling edge to detection */

static float fall_edge[EDGE_PREVNUM] = {1000.f, 800.f, 600.f, 0.f};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/** clear_data() */

static void clear_data(float *dat, int len)
{
  memset(dat, 0, sizeof(float) * len);
}

/** detect_edge() */

static void detect_edge(edge_detectf_t *detector, float *data,
    float *odat, int len)
{
  int edge_pos = 0;
  int offset = 0;

  /* When an edge is found, mark it on the same offset of the input data
   * into output data as 3000. And keep searching another edge in remaining
   * input data until end of it.
   */

  DPRINT("Enter detect_edge(data size = %d)\n", len);
  while ((edge_pos = edge_detectf(detector, &data[offset], len)) >= 0)
    {
      DPRINT(" Detected : %d\n", edge_pos);

      /* Mark the edge */

      odat[offset + edge_pos] = 30000.f;

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

/****************************************************************************
 * Main Functions of this example
 ****************************************************************************/

int main(int argc, char **argv)
{
  int n;
  struct timeval time_keep;
  uint64_t total_time = 0;
  uint64_t total_size = 0;

  edge_detectf_t *rise;
  edge_detectf_t *fall;

  wav_instance_t input_wav;
  wav_instance_t rising_edge_wav;
  wav_instance_t falling_edge_wav;

  if (argc != 2)
    {
      printf("No input file...\n");
      return -1;
    }

  /* Create edge_detect instance as rise type and keep block mode. */

  rise = edge_detection_createf(rise_edge, EDGE_PREVNUM, EDGE_KEEPNUM,
      EDGE_DETECT_TYPE_RISE);
  if (rise == NULL)
    {
      printf("Error: couldn't create rise edge_detector\n");

    }

  /* Create edge_detect instance as fall type and keep block mode. */

  fall = edge_detection_createf(fall_edge, EDGE_PREVNUM, EDGE_KEEPNUM,
      EDGE_DETECT_TYPE_FALL);
  if (fall == NULL)
    {
      edge_detection_deletef(rise);

      printf("Error: couldn't create fall edge_detector\n");
      return -1;
    }

  /* Open input wav file */

  if (open_input_wavfile(&input_wav, argv[1]))
    {
      edge_detection_deletef(rise);
      edge_detection_deletef(fall);
      printf("Error: could not open file %s\n", argv[1]);
      return -1;
    }

  /* Open output wav file to store the result of rising edge detection */

  rising_edge_wav.fs     = input_wav.fs;
  rising_edge_wav.bits   = input_wav.bits;
  rising_edge_wav.length = input_wav.length;
  if (open_output_wavfile(&rising_edge_wav, BASE_DIR "rise.wav"))
    {
      close_wavfile(&input_wav);
      edge_detection_deletef(rise);
      edge_detection_deletef(fall);
      printf("Error: could not open file " BASE_DIR "rise.wav\n");
      return -1;
    }

  /* Open output wav file to store the result of falling edge detection */

  falling_edge_wav.fs     = input_wav.fs;
  falling_edge_wav.bits   = input_wav.bits;
  falling_edge_wav.length = input_wav.length;
  if (open_output_wavfile(&falling_edge_wav, BASE_DIR "fall.wav"))
    {
      close_wavfile(&rising_edge_wav);
      close_wavfile(&input_wav);
      edge_detection_deletef(rise);
      edge_detection_deletef(fall);
      printf("Error: could not open file " BASE_DIR "fall.wav\n");
      return -1;
    }

  /* Clear output buffer */

  clear_data(out_data, BLOCK_SIZE);

  /* Main loop : repeat "read data/detect edge/write data" */

  for (n = 0; n < (input_wav.length - BLOCK_SIZE + 1); n += BLOCK_SIZE)
    {
      DPRINT("==== Read data index = %d ====\n", n);

      /* Read input data from wav file */

      read_wavdata(&input_wav, sig_data, BLOCK_SIZE);

      DPRINT("** rise \n");

      /* Mark edge position by "rise" edge_detect instance
       * on out_data if it found
       */

      measure_interval(&time_keep);
      detect_edge(rise, sig_data, out_data, BLOCK_SIZE);
      total_time += (uint64_t)measure_interval(&time_keep);

      /* Save out_data in "rise.wav" file */

      write_wavdata(&rising_edge_wav, out_data, BLOCK_SIZE);

      /* Clear "out_data" with 0 all for next use */

      clear_data(out_data, BLOCK_SIZE);

      DPRINT("** fall \n");

      /* Mark edge position by "fall" edge_detect instance
       * on out_data if it found
       */

      measure_interval(&time_keep);
      detect_edge(fall, sig_data, out_data, BLOCK_SIZE);
      total_time += (uint64_t)measure_interval(&time_keep);

      /* Save out_data in "rise.wav" file */

      write_wavdata(&falling_edge_wav, out_data, BLOCK_SIZE);

      /* Clear "out_data" with 0 all for next use */

      clear_data(out_data, BLOCK_SIZE);

      total_size += (BLOCK_SIZE * 2);
    }

  printf("Execution finished.\n");
  printf("Total size : %lld, Total time %lldus\n", total_size, total_time);

  close_wavfile(&rising_edge_wav);
  close_wavfile(&falling_edge_wav);
  close_wavfile(&input_wav);

  edge_detection_deletef(rise);
  edge_detection_deletef(fall);

  return 0;
}
