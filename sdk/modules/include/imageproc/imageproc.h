/****************************************************************************
 * sdk/modules/include/imageproc/imageproc.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __IMAGEPROC_H__
#define __IMAGEPROC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @defgroup imageproc_funcs Functions
 * @{
 */

/**
 * Structure of rectangle coordinates from left top point to right buttom point.
 */
struct imageproc_rect_s {
  uint16_t x1;  /**< X coordinate of left top point */
  uint16_t y1;  /**< Y coordinate of left top point */
  uint16_t x2;  /**< X coordinate of rignt bottom point */
  uint16_t y2;  /**< Y coordinate of rignt bottom point */
};
typedef struct imageproc_rect_s imageproc_rect_t;

/**
 * Initialize imageproc library
 */

void imageproc_initialize(void);

/**
 * Finalize imageproc library
 */

void imageproc_finalize(void);

/**
 * Convert color format (YUV to RGB)
 *
 * TODO: need more description here
 *
 * @param [in,out] ibuf: image
 * @param [in] hsize: Horizontal size
 * @param [in] vsize: Vertical size
 */

void imageproc_convert_yuv2rgb(uint8_t *ibuf, uint32_t hsize, uint32_t vsize);

/**
 * Convert color format (YUV to grayscale)
 *
 * TODO: need more description here
 *
 * @param [in] ibuf: Input image
 * @param [out] obuf: Output buffer
 * @param [in] hsize: Horizontal size
 * @param [in] vsize: Vertical size
 */

void imageproc_convert_yuv2gray(uint8_t *ibuf, uint8_t *obuf, size_t hsize,
                                size_t vsize);

/**
 * @if Japanese
 *
 * 画像のリサイズ
 *
 * 入力画像 @a ibuf で与えられた画像を、@a ohsize、 @a ovsize で指定された
 * サイズに縮小または拡大し、出力先バッファ @a obuf に出力します。
 *
 * 出力画像に指定可能なサイズは、縦・横それぞれ @a ihsize、 @a ivsize に対して
 * 1/2^n倍〜2^n倍 (n=0..5)となるように設定します。
 *
 * 処理可能なピクセルフォーマットはYUV422またはグレースケールのみとなります。
 * YUV422 (16bpp)の画像を処理する場合は、入出力の横サイズは2の倍数になる必要があります。
 *
 * また、リサイズ可能なサイズは以下の制限事項があります。
 *
 * * 縮小時
 *   + 縦 12ピクセル
 *   + 横 12ピクセル
 * * 拡大時
 *   + 縦 1024ピクセル
 *   + 横 768ピクセル
 *
 * @param [in] ibuf: 入力画像
 * @param [in] ihsize: 入力画像サイズ（横）
 * @param [in] ivsize: 入力画像サイズ（縦）
 * @param [out] obuf: 画像出力先バッファ
 * @param [in] ohsize: 出力画像サイズ（横）
 * @param [in] ovsize: 出力画像サイズ（縦）
 * @param [in] bpp: １ピクセルあたりのビット数（16 or 8）
 *
 * @return 正常終了の場合は0、それ以外の場合はエラーコードを返します。
 *
 * @else
 *
 * Resize image
 *
 * Resize image specified by @a ibuf to @a ohsize, @a ovsize. Processed
 * image will be stored to @a obuf.
 *
 * For @a ohsize and @a ovsize, specify output size calculated by multiply in
 * range 1/2^n to 2^n (n=0..5) against @a ihsize and @a ivsize.
 *
 * This function can be processing for YUV422 color format. So all of specified
 * sizes must be multiple of 2.
 *
 * And there is limitation about output size below.
 *
 * * Shrink
 *   + Horizontal size least 12 pixels
 *   + Vertical size least 12 pixels
 * * Enlarge
 *   + Horizontal size up to 768 pixels
 *   + Vertical size up to 1024 pixels
 *
 * @param [in] ibuf: Input image
 * @param [in] ihsize: Input horizontal size
 * @param [in] ivsize: Input vertical size
 * @param [out] obuf: Output buffer
 * @param [in] ohsize: Output horizontal size
 * @param [in] ovsize: Output vertical size
 * @param [in] bpp: Bits per pixel (16 or 8)
 *
 * @return 0 on success, otherwise error code.
 *
 * @endif
 */

int imageproc_resize(uint8_t *ibuf, uint16_t ihsize, uint16_t ivsize,
                     uint8_t *obuf, uint16_t ohsize, uint16_t ovsize, int bpp);

/**
 * @if Japanese
 *
 * 入力画像の中の矩形の切り出しとリサイズ
 *
 * 入力画像 @a ibuf で与えられた画像を、@a clip_rect で指定された領域を切り出し、かつ、
 * @a ohsize、 @a ovsize で指定されたサイズに縮小または拡大し、出力先バッファ @a obuf に出力します。
 *
 * 出力画像に指定可能なサイズは、縦・横それぞれ @a clip_rectで指定した画像のサイズに対して
 * 1/2^n倍〜2^n倍 (n=0..5)となるように設定します。
 *
 * 処理可能なピクセルフォーマットはYUV422またはグレースケールのみとなります。
 * YUV422 (16bpp)の画像を処理する場合は、入出力の横サイズは2の倍数になる必要があります。
 *
 * また、リサイズ可能なサイズは以下の制限事項があります。
 *
 * * 縮小時
 *   + 縦 12ピクセル
 *   + 横 12ピクセル
 * * 拡大時
 *   + 縦 1024ピクセル
 *   + 横 768ピクセル
 *
 * @param [in] ibuf: 入力画像 (クリップ対象画像)
 * @param [in] ihsize: 入力画像サイズ（横）(クリップ前の画像の幅)
 * @param [in] ivsize: 入力画像サイズ（縦）(クリップ前の画像の高さ)
 * @param [in] rect: 入力画像の矩形領域指定 (クリップ位置の指定)
 * @param [out] obuf: 画像出力先バッファ
 * @param [in] ohsize: 出力画像サイズ（横）
 * @param [in] ovsize: 出力画像サイズ（縦）
 * @param [in] bpp: １ピクセルあたりのビット数（16 or 8）
 * @param [in] clip_rect: Clipping rectangle on input image.
 *
 * @return 正常終了の場合は0、それ以外の場合はエラーコードを返します。
 *
 * @else
 *
 * Clip and Resize image
 *
 * @param [in] ibuf: Input image
 * @param [in] ihsize: Input horizontal size
 * @param [in] ivsize: Input vertical size
 * @param [out] obuf: Output buffer
 * @param [in] ohsize: Output horizontal size
 * @param [in] ovsize: Output vertical size
 * @param [in] bpp: Bits per pixel (16 or 8)
 * @param [in] clip_rect: Clipping rectangle on input image.
 *
 * @return 0 on success, otherwise error code.
 *
 * @endif
 */

int imageproc_clip_and_resize(
  uint8_t *ibuf, uint16_t ihsize, uint16_t ivsize,
  uint8_t *obuf, uint16_t ohsize, uint16_t ovsize,
  int bpp, imageproc_rect_t *clip_rect);

/** @} imageproc_funcs */
/** @} imageproc */

#ifdef __cplusplus
}
#endif

#endif  // __IMAGEPROC_H__
