/****************************************************************************
 * examples/jpeg_decode/jpeg_decode_main.c
 *
 *   Copyright 2019, 2021 Sony Semiconductor Solutions Corporation
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
#include <fcntl.h>
#include <errno.h>

#include "jpeglib.h"

#include "jpeg_decode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define APP_FILENAME_LEN  128

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_out_filename[APP_FILENAME_LEN];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* 
 * Process the decoded result.
 * This function is called every time one line is decoded
 * from read_JPEG_file().
 * In this example, save to the SD card.
 */

void put_scanline_someplace(JSAMPLE *data, int len)
{
  FILE *fp;

  fp = fopen(g_out_filename, "ab");
  if (fp == NULL)
    {
      printf("fopen error : %d\n", errno);
      return;
    }

  if (len != fwrite(data, 1, len, fp))
    {
      printf("fwrite error : %d\n", errno);
    }

  fclose(fp);
  return;
}

/*
 * Sample routine for JPEG decompression to YUV4:2:2.
 * Assume that the source file name is passed in.
 */

int main(int argc, FAR char *argv[])
{
  int  ret;
  char *ext;

  if (argc < 2)
    {
      printf("Input filename to be decoded.\n");
      return ERROR;
    }

  /* Remove extention from input filename,
   * and add ".YUV" extention.
   */

  ext = strrchr(argv[1], '.');
  memset(g_out_filename, 0, sizeof(g_out_filename));
  strncpy(g_out_filename, argv[1], strlen(argv[1]) - strlen(ext));
  strncat(g_out_filename, ".YUV", strlen(".YUV") + 1);

  remove(g_out_filename);

  /* read_JPEG_file() is JPEG decoder function which is defined in example.c.
   * example.c is provided by IJG.
   */

  ret = read_JPEG_file(argv[1]);
  if (ret == 1)
    {
      printf("Decode result is saved in %s.\n", g_out_filename);
    }
  else
    {
      printf("Failed to decode.\n");
    }

  return 0;
}

