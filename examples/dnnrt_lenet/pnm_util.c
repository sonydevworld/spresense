/****************************************************************************
 * dnnrt_lenet/pnm_util.c
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#define MY_BUFSIZ (28*28)       // assume images in MNIST

typedef struct _pnm_header
  {
    uint8_t magic_num;
    uint16_t width_px;
    uint16_t height_px;
    uint16_t max;
  } pnm_header;

static int read_magic_number(const char *char_buf, pnm_header * header_ptr)
{
  if (strnlen(char_buf, MY_BUFSIZ) == 3)
    {
      header_ptr->magic_num = char_buf[1] - '0';
      if (header_ptr->magic_num == 5u || header_ptr->magic_num == 6u)
        {
          /* normal case */
          return 0;
        }
      else
        {
          return -EFTYPE;
        }
    }
  else
    {
      return -EINVAL;
    }
}

static int read_image_size(const char *char_buf, pnm_header * header_ptr)
{
  if (sscanf(char_buf, "%hu %hu",
             &(header_ptr->width_px), &(header_ptr->height_px)) == 2)
    {
      return 0;
    }
  else
    {
      return -EINVAL;
    }
}

static int read_max(const char *char_buf, pnm_header * header)
{
  header->max = atoi(char_buf);
  return 0;
}

static uint32_t calc_data_bsize(const pnm_header * header)
{
  uint32_t bsize;
  bsize = header->height_px * header->width_px;
  bsize *= (header->magic_num == 5u) ? 1u : 3u;
  bsize *= (header->max <= 255) ? 1u : 2u;
  return bsize;
}

static void
read_binary_data(const pnm_header * header, const uint8_t * data_buffer,
                 float norm_factor, float *output_buffer)
{
  uint32_t out_crsr, in_crsr, color_crsr;
  uint32_t pri_color_num = (header->magic_num == 5u) ? 1u : 3u;
  uint32_t data_bsize = calc_data_bsize(header);

  out_crsr = 0u;
  for (color_crsr = 0u; color_crsr < pri_color_num; color_crsr++)
    {
      for (in_crsr = color_crsr; in_crsr < data_bsize; in_crsr += pri_color_num)
        {
          output_buffer[out_crsr] = (float)data_buffer[in_crsr] / norm_factor;
          out_crsr++;
        }
    }
}

static char *read_pnm_line(char *buf, int size, FILE *pnm_file)
{
  char *line;
  char *p;

  for (;;)
    {
      line = fgets(buf, size, pnm_file);
      if (line == NULL)
        {
          break;
        }

      for (p = line; *p != 0; ++p)
        {
          if (*p == ' ' || *p == '\t')
            {
              continue;
            }

          break;
        }

      if (*p != '#')
        {
          break;
        }
    }

  return line;
}

static int
load_pnm_internal(const char *pnm_path, float norm_factor, float *output_buffer)
{
  int err = 0;
  uint32_t exp_bsize, act_bsize;
  char char_buf[MY_BUFSIZ];
  pnm_header header;
  FILE *pnm_file;

  pnm_file = fopen(pnm_path, "r");
  if (pnm_file != NULL)
    {
      /* read magic number in .pnm */
      if (read_pnm_line(char_buf, MY_BUFSIZ, pnm_file) != NULL)
        {
          if ((err = read_magic_number(char_buf, &header)) < 0)
            {
              goto invalid_file;
            }
        }
      else
        {
          err = -EINVAL;
          goto invalid_file;
        }

      /* read height and width of this file */
      if (read_pnm_line(char_buf, MY_BUFSIZ, pnm_file) != NULL)
        {
          if ((err = read_image_size(char_buf, &header)) < 0)
            {
              goto invalid_file;
            }
        }
      else
        {
          err = -EINVAL;
          goto invalid_file;
        }

      /* read max */
      if (read_pnm_line(char_buf, MY_BUFSIZ, pnm_file) != NULL)
        {
          if ((err = read_max(char_buf, &header)) < 0)
            {
              goto invalid_file;
            }
        }
      else
        {
          err = -EINVAL;
          goto invalid_file;
        }

      /* read binary data of this image */
      exp_bsize = calc_data_bsize(&header);
      if (exp_bsize > MY_BUFSIZ)
        {
          err = -EINVAL;
          goto invalid_file;
        }
      act_bsize = fread(char_buf, 1, exp_bsize, pnm_file);
      if (exp_bsize == act_bsize)
        {
          read_binary_data(&header, (const uint8_t *)char_buf, norm_factor,
                           output_buffer);
        }
      else
        {
          err = -EINVAL;
          goto invalid_file;
        }

      fclose(pnm_file);
    }
  else
    {
      err = -errno;
      goto file_open_err;
    }

  return 0;

invalid_file:
  fclose(pnm_file);
file_open_err:
  return err;
}

int pnm_load(const char *pnm_path, float norm_factor, float *output_buffer,
             size_t output_bsize)
{
  int err;
  int valid_args = 1;
  valid_args &= (pnm_path != NULL && output_buffer != NULL);
  valid_args &= (output_bsize <= sizeof(float) * MY_BUFSIZ);
  if (valid_args)
    {
      if ((err = load_pnm_internal(pnm_path, norm_factor, output_buffer)) == 0)
        {
          /* normal case */
        }
      else
        {
          goto load_error;
        }
    }
  else
    {
      err = -EINVAL;
      goto invalid_args;
    }
  return 0;

load_error:
invalid_args:
  return err;
}
