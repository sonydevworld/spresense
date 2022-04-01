/****************************************************************************
 * dsc/utils/font_draw.c
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
#include <sdk/config.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "MonoFont_2bit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DISPLAY_WIDTH    (320)
#define DISPLAY_HEIGHT   (240)
#define DISPLAY_COLOR_BPP  (2)

#define LINE_OFSET_TOP(line)   (DISPLAY_WIDTH * (line) * DISPLAY_COLOR_BPP)
#define LINES_MEMSIZE(lines)    LINE_OFSET_TOP(lines)
#define TOP_OF_LINE(buf, line)   ((buf) + LINE_OFSET_TOP(line))

#define MIN_DATA(a, b)  ( ((a) > (b)) ? (b) : (a) )

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: draw_borderingfont
 ****************************************************************************/

void draw_borderingfont(uint8_t *frame_buff, int fb_width, int fb_height,
                    int draw_pos_x, int draw_pos_y, uint16_t fg_color,
                    uint16_t edge_color, const uint8_t *font_data,
                    int font_width, int font_height)
{
  int w;
  int h;
  int font_line_len;
  uint16_t *draw_start_addr;
  const uint8_t *font_draw_pos = font_data;

  /* Calculate actual bytes for one line of the fonct.
   * Each byte has 4 pix data (4 * 2 = 8), and end of line has padding.
   */

  font_line_len = (font_width + 3) / 4;

  /* TODO :: error check */

  draw_start_addr = (uint16_t *)(TOP_OF_LINE(frame_buff, draw_pos_y)
                                  + (DISPLAY_COLOR_BPP * draw_pos_x));

  for (h = 0;
       (h < font_height) && ((h + draw_pos_y) < fb_height);
       h++, draw_start_addr += (fb_width), font_draw_pos += font_line_len)
    {
      for (w = 0; (w < font_width) && ((w + draw_pos_x) < fb_width); )
        {
          int font_rest = font_width - w;
          int fb_rest = fb_width - (w + draw_pos_x);

          uint8_t font_byte_data = font_draw_pos[w / 4];
          uint8_t font_bit_data;
          uint8_t bit_mask = 0x03;
          int bit_shift = 0;
          uint8_t cnt;
          int i;

          cnt = MIN_DATA(font_rest, 4);
          cnt = MIN_DATA(cnt, MIN_DATA(fb_rest, 4));

          for (i = 0; i < cnt; i++)
            {
              font_bit_data = (font_byte_data >> bit_shift) & bit_mask;
              if (font_bit_data & 0x01) /* Is needed to draw? */
                {
                  if (font_bit_data & 0x02) /* Is edge ? */
                    {
                      draw_start_addr[w] = edge_color;
                    }
                  else
                    {
                      draw_start_addr[w] = fg_color;
                    }
                }

              bit_shift += 2;
              w++;
            }
        }
    }
}

/****************************************************************************
 * Included Files
 ****************************************************************************/

void print_borderingfont(FAR const char *msg, uint8_t *canvas, int w, int h,
    int posx, int posy, uint16_t fg, uint16_t bg)
{
  int font_offset;

  while (*msg)
    {
      if (isdigit(*msg))
        {
          font_offset = digit_offset + *msg - '0';
        }
      else if (isupper(*msg))
        {
          font_offset = upper_offset + *msg - 'A';
        }
      else if (islower(*msg))
        {
          font_offset = lower_offset + *msg - 'a';
        }
      else
        {
          font_offset = other_offset;
        }

      draw_borderingfont(canvas, w, h, posx, posy, fg, bg,
                    MonoFont_data[font_offset],
                    MonoFont_eachwidth, MonoFont_height);
      posx += MonoFont_eachwidth;
      msg++;
      if (posx >= w)
        {
          posx = 0;
          posy += MonoFont_height;
          if (posy >= h)
            {
              break;
            }
        }
    }
}

int get_drawfontwidth(void)
{
  return MonoFont_eachwidth;
}

int get_drawfontheight(void)
{
  return MonoFont_height;
}
