/****************************************************************************
 * examples/dsc/include/font_draw.h
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

#ifndef __EXAMPLES_DSC_INCLUDE_FONT_DRAW_H__
#define __EXAMPLES_DSC_INCLUDE_FONT_DRAW_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FONT_COLOR_RED     (0xF800)
#define FONT_COLOR_GREEN   (0x07E0)
#define FONT_COLOR_BLUE    (0x001F)
#define FONT_COLOR_YELLOW  (0xFFE0)
#define FONT_COLOR_WHITE   (0xFFFF)
#define FONT_COLOR_BLACK   (0x0000)
#define FONT_COLOR_GRAY    (0x7BEF) /* 0111_1 011_111 0_1111 */
#define FONT_COLOR_DRKGRAY (0x39E7) /* 0011_1 001_111 0_0111 */

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void draw_borderingfont(uint8_t *frame_buff, int fb_width, int fb_height,
                    int draw_pos_x, int draw_pos_y, uint16_t fg_color,
                    uint16_t edge_color, const uint8_t *font_data,
                    int font_width, int font_height);

void print_borderingfont(FAR const char *msg, uint8_t *canvas, int w, int h,
    int posx, int posy, uint16_t fg, uint16_t bg);

int get_drawfontwidth(void);
int get_drawfontheight(void);

#ifdef __cplusplus
}
#endif

#endif  /* __EXAMPLES_DSC_INCLUDE_FONT_DRAW_H__ */
