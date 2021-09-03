/****************************************************************************
 * examples/digital_filter/fir_filter_main.c
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
#include <math.h>

#include <digital_filter/fir_filter.h>

#include "wav.h"

#define BLOCK_SIZE  256

static float si_data[BLOCK_SIZE];
static float so_data[BLOCK_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_usage(const char *appname)
{
  printf("Usage: %s <input wav file> <lpf/hpf> <transition width(Hz)>"
      " <cutoff freq(Hz)> <output wav file>\n", appname);
  printf("     : %s <input wav file> <bpf/bef> <transition width(Hz)>"
      " <cutoff1 freq(Hz)> <cutoff2 freq(Hz)> <output wav file>\n", appname);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main()
 *
 * Description:
 *   main routine of this example.
 ****************************************************************************/

int main(int argc, char **argv)
{
  fir_instancef_t *fir;
  wav_instance_t input_wav;
  wav_instance_t output_wav;
  int cutoff1 = 0;
  int cutoff2 = 0;
  int tr_width;
  int n;
  int tap_num;
  char *output_filename;

  if ((argc != 6) && (argc != 7))
    {
      print_usage(argv[0]);
      return -1;
    }

  /* Transition frequency width (Hz) */

  tr_width = atoi(argv[3]);

  /* Cut off frequency (Hz) */

  cutoff1 = atoi(argv[4]);

  /* In case of BPF/BEF, one more Cut off frequency is needed */

  if (argc == 7)
    {
      cutoff2 = atoi(argv[5]);
      output_filename = argv[6];
    }
  else
    {
      output_filename = argv[5];
    }

  /* Open input wav file */

  if (open_input_wavfile(&input_wav, argv[1]))
    {
      printf("Error: no such file <%s>\n", argv[1]);
      print_usage(argv[0]);
      return -1;
    }

  /* Check tap number */

  tap_num = fir_calc_tapnumber(input_wav.fs, tr_width);
  if (tap_num >= BLOCK_SIZE)
    {
      printf("Calculated Tap size(%d) must be smaller than BLOCK_SIZE(%d).\n",
          tap_num, BLOCK_SIZE);
      printf("Try a wider transition band.\n");
      close_wavfile(&input_wav);
      return -1;
    }

  /* Create FIR filter */

  if (!strcmp(argv[2], "lpf"))
    {
      fir = fir_create_lpff(input_wav.fs, cutoff1, tr_width, BLOCK_SIZE);
    }
  else if (!strcmp(argv[2], "hpf"))
    {
      fir = fir_create_hpff(input_wav.fs, cutoff1, tr_width, BLOCK_SIZE);
    }
  else if (!strcmp(argv[2], "bpf"))
    {
      fir = fir_create_bpff(input_wav.fs, cutoff1, cutoff2, tr_width, BLOCK_SIZE);
    }
  else if (!strcmp(argv[2], "bef"))
    {
      fir = fir_create_beff(input_wav.fs, cutoff1, cutoff2, tr_width, BLOCK_SIZE);
    }
  else
    {
      close_wavfile(&input_wav);
      print_usage(argv[0]);
      return -1;
    }

  if (fir == NULL)
    {
      printf("Could not allocate fir..\n");
      close_wavfile(&input_wav);
      return -1;
    }

  printf("Filter instance = %p\n", fir);
  printf("Filter type = %s\n", argv[2]);
  printf("Filter Tap number = %d\n", fir_get_tapnumf(fir));
  printf("Filter Block size = %d\n", BLOCK_SIZE);
  printf("Filter cut-off1 freq(Hz) = %d\n", cutoff1);
  printf("Filter cut-off2 freq(Hz) = %d\n", cutoff2);
  printf("Filter transition band freq(Hz) = %d\n", tr_width);

  output_wav.fs     = input_wav.fs;
  output_wav.bits   = input_wav.bits;
  output_wav.length = input_wav.length;

  printf("Sampling rate = %d(Hz)\n", input_wav.fs);
  printf("Sampling length = %ld\n", input_wav.length);
  printf("Saving filename : %s\n", output_filename);

  /* Open output wav file */

  if (open_output_wavfile(&output_wav, output_filename))
    {
      printf("Error : could not open output file..\n");
      close_wavfile(&input_wav);
      fir_deletef(fir);
      return -1;
    }

  for (n = 0; n < (input_wav.length - BLOCK_SIZE + 1); n += BLOCK_SIZE)
    {
      /* Load PCM data from input wav file */

      read_wavdata(&input_wav, si_data, BLOCK_SIZE);

      /* Execute FIR filter */

      fir_executef(fir, si_data, so_data, BLOCK_SIZE);

      /* Save filtered data into output wav file */

      write_wavdata(&output_wav, so_data, BLOCK_SIZE);
    }

  close_wavfile(&output_wav);
  close_wavfile(&input_wav);

  fir_deletef(fir);

  printf("Done\n");
  return 0;
}
