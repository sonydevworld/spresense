/****************************************************************************
 * examples/digital_filter/fir_envelope_main.c
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

#define BLOCK_SIZE      (256)

#define BPF_CUTOFF_LOW  (2000)
#define BPF_CUTOFF_HIGH (15000)
#define BPF_TAP_NUM     (64)

#define LPF_CUTOFF_FREQ (1000)
#define LPF_TAP_NUM     (64)

static float sig_data[BLOCK_SIZE];
static float tmp_data[BLOCK_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_usage(const char *appname)
{
  printf("Usage: %s <input wav file> <output wav file>\n>", appname);
}

static void envelope_h_function(fir_instancef_t *bpf, fir_instancef_t *lpf,
    float *target_data, int len)
{
  /* Execute BPF with Absolution */

  firabs_executef(bpf, target_data, tmp_data, len);

  /* Execute LPF */

  fir_executef(lpf, tmp_data, target_data, len);
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
  fir_instancef_t *bpf;
  fir_instancef_t *lpf;
  wav_instance_t input_wav;
  wav_instance_t output_wav;
  int n;

  if (argc != 3)
    {
      print_usage(argv[0]);
      return -1;
    }

  /* Open input wav file */

  if (open_input_wavfile(&input_wav, argv[1]))
    {
      printf("Error: no such file <%s>\n", argv[1]);
      print_usage(argv[0]);
      return -1;
    }

  /* Create Filters for Envelope processing */

  bpf = fir_create_bpff_tap(input_wav.fs, BPF_CUTOFF_LOW, BPF_CUTOFF_HIGH,
      BPF_TAP_NUM, BLOCK_SIZE);
  if (bpf == NULL)
    {
      printf("Could not create Band pass filter.\n");
      close_wavfile(&input_wav);
      return -1;
    }

  lpf = fir_create_lpff_tap(input_wav.fs, LPF_CUTOFF_FREQ, LPF_TAP_NUM,
      BLOCK_SIZE);
  if (lpf == NULL)
    {
      printf("Could not create Low pass filter.\n");
      fir_deletef(bpf);
      close_wavfile(&input_wav);
      return -1;
    }

  printf("Filter Block size = %d\n", BLOCK_SIZE);
  printf("BPF Filter %d~%d : (Taps %d)\n", BPF_CUTOFF_LOW, BPF_CUTOFF_HIGH,
      fir_get_tapnumf(bpf));
  printf("LPF Filter %d : (Taps %d)\n", LPF_CUTOFF_FREQ, fir_get_tapnumf(lpf));

  output_wav.fs     = input_wav.fs;
  output_wav.bits   = input_wav.bits;
  output_wav.length = input_wav.length;

  printf("Sampling rate = %d(Hz)\n", input_wav.fs);
  printf("Sampling length = %ld\n\n", input_wav.length);
  printf("Saving filename : %s\n", argv[2]);

  /* Open output wav file */

  if (open_output_wavfile(&output_wav, argv[2]))
    {
      printf("Error : could not open output file..\n");
      close_wavfile(&input_wav);
      fir_deletef(bpf);
      fir_deletef(lpf);
      return -1;
    }

  for (n = 0; n < (input_wav.length - BLOCK_SIZE + 1); n += BLOCK_SIZE)
    {
      /* Load PCM data from input wav file */

      read_wavdata(&input_wav, sig_data, BLOCK_SIZE);

      /* Do envelope processing */

      envelope_h_function(bpf, lpf, sig_data, BLOCK_SIZE);

      /* Save enveloped data into output wav file */

      write_wavdata(&output_wav, sig_data, BLOCK_SIZE);
    }

  close_wavfile(&output_wav);
  close_wavfile(&input_wav);

  fir_deletef(lpf);
  fir_deletef(bpf);

  printf("Done\n");
  return 0;
}
