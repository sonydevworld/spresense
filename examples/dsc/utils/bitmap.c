/****************************************************************************
 * examples/dsc/include/bitmap.c
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

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char tag[] = { 'B', 'M' };

static const int default_header[] = {
        0, 0, 0x36, 0x28, 0, 0, 0x180001, 
        0, 0, 0x002e23, 0x002e23, 0, 0
};
#define HEADER_SIZE	(sizeof(default_header)/sizeof(default_header[0]))

static int header[HEADER_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void init_header(void)
{
	int i;
  for (i=0; i<HEADER_SIZE; i++)
    {
      header[i] = (int)default_header[i];
    }
}

static void expand_color(unsigned short rgb565,
                  unsigned char *r,
                  unsigned char *g,
                  unsigned char *b)
{
  *r = (unsigned char)((rgb565 >> 8) & 0xF8);
  *g = (unsigned char)((rgb565 >> 3) & 0xFC);
  *b = (unsigned char)((rgb565 << 3) & 0xF8);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int write_bmp(const char *filename, unsigned short *rgb565,
              int height, int width)
{
  FILE *fp;
  int padding_len;
  int bitmap_size;
  int row, col;
  unsigned char r, g, b, z;

  printf("Saveing Image into %s.\n", filename);

  /* Pad the width of the destination to a multiple of 4 */

  padding_len = (4 - ((width * 3) % 4)) % 4;
    
  bitmap_size = height * (width + padding_len) * 3;

  /* Create header */

  init_header();
  header[0] = sizeof(tag) + sizeof(header) + bitmap_size;
  header[4] = width;
  header[5] = -height;

  fp = fopen(filename, "wb");
  if (fp == NULL)
    {
      printf("Could not open %s\n", filename);
      return -1;
    }

  fwrite(&tag, sizeof(tag), 1, fp);
  fwrite(&header, sizeof(header), 1, fp);

  z = 0;

  /* For each pixel in the RGB image... */

  for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
        {
          expand_color(*rgb565, &r, &g, &b);
          fwrite(&b, 1, 1, fp);
          fwrite(&g, 1, 1, fp);
          fwrite(&r, 1, 1, fp);
          rgb565++;
        }
      for (col = 0; col < padding_len; col++)
        {
          fwrite(&z, 1, 1, fp);
        }
    }

  fclose(fp);

  return 0;
}
