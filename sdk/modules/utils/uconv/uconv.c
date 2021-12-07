/****************************************************************************
 * modules/utils/uconv/uconv.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <errno.h>
#include "utils/uconv/uconv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_UTILS_UCONV_DEBUG
#  define UCONV_DEBUG(x, ...) printf("%s "x, __func__, ##__VA_ARGS__)
#else
#  define UCONV_DEBUG(x, ...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uconv_utf8_to_ucs2(int src_size, uint8_t *src,
                       int dst_size, uint16_t *dst)
{
  int i     = 0;
  int dst_idx = 0;

  if (src_size <= 0 || dst_size <= 0)
    {
      UCONV_DEBUG("Invalid size or length.\n");
      UCONV_DEBUG("src_size:%d, dst_size:%d.\n", src_size, dst_size);
      return -EINVAL;
    }

  if (!src || !dst)
    {
      UCONV_DEBUG("src or dst is NULL.\n");
      return -EINVAL;
    }

  while ((i < src_size && dst_idx < dst_size))
    {

      /* Copy the 7bit code as it is.
       *
       * - Valid UTF-8 range
       *  1st byte : 0x00 - 0x7F
       */

      if (src[i] <= 0x7F)
        {

          /* - Bit pattern (x is valid bits)
           *  UTF-8 :          0xxxxxxx
           *  UCS2  : 00000000-0xxxxxxx
           */

          dst[dst_idx] = (uint16_t)src[i];
        }
      else if (0xC2 <= src[i] && src[i] <= 0xDF)
        {

          /* 2 byte UTF-8 data.
           *
           * - Valid UTF-8 range
           *  1st byte : 0xC2 - 0xDF
           *  2nd byte : 0x80 - 0xBF
           */

          if (src_size < (i + 2))
            {
               UCONV_DEBUG("Unexpected data : %02X\n", src[i]);
              return -EIO;
            }
          else if (src[i + 1] < 0x80 || 0xBF < src[i + 1])
            {
              UCONV_DEBUG("Unexpected data : %02X%02X\n", src[i], src[i + 1]);
              return -EIO;
            }
          else
            {

              /* - Bit pattern (x and y are valid bits)
               *  UTF-8 : 110xxxxx 10yyyyyy
               *  UCS2  : 00000xxx-xxyyyyyy
               */

              /* The lower 5 bits are shifted and copied. */

              dst[dst_idx] = (src[i] & 0x1F) << 6;
              i++;

              /* The lower 6 bits are copied. */

              dst[dst_idx] |= src[i] & 0x3F;

              /* Only the lower 11 bits are enabled. */

              dst[dst_idx] &= 0x7FF;

            }
        }
      else if (0xE0 <= src[i] && src[i] <= 0xEF)
        {

          /* 3 byte UTF-8 data.
           *
           * - Valid UTF-8 range
           *
           * -- When the 1st byte is 0xE0
           *     2nd byte : 0xA0 - 0xBF
           *     3rd byte : 0x80 - 0xBF
           *
           * -- When the 1st byte range is 0xE1 - 0xEC
           *     2nd byte : 0x80 - 0xBF
           *     3rd byte : 0x80 - 0xBF
           *
           * -- When the 1st byte is 0xED
           *     2nd byte : 0x80 - 0x9F
           *     3rd byte : 0x80 - 0xBF
           *
           * -- When the 1st byte range is 0xEE - 0xEF
           *     2nd byte : 0x80 - 0xBF
           *     3rd byte : 0x80 - 0xBF
           */

          /* Check the range of valid values. */

          if (src_size < (i + 3))
            {
              UCONV_DEBUG("Unexpected data : %02X\n", src[i]);
              return -EIO;
            }
          else if (0xE0 == src[i] &&
              (src[i + 1] < 0xA0 || 0xBF < src[i + 1]))
            {
              UCONV_DEBUG("Unexpected data : %02X%02X%02X\n", src[i], src[i + 1], src[i + 2]);
              return -EIO;
            }
          else if (0xED == src[i] &&
              (src[i + 1] < 0x80 || 0x9F < src[i + 1]))
            {
              UCONV_DEBUG("Unexpected data : %02X%02X%02X\n", src[i], src[i + 1], src[i + 2]);
              return -EIO;
            }
          else if ((src[i + 1] < 0x80 || 0xBF < src[i + 1]))
            {
              UCONV_DEBUG("Unexpected data : %02X%02X%02X\n", src[i], src[i + 1], src[i + 2]);
              return -EIO;
            }
          else
            {
              if ((src[i + 2] < 0x80 || 0xBF < src[i + 2]))
                {
                  UCONV_DEBUG("Unexpected data : %02X%02X%02X\n", src[i], src[i + 1], src[i + 2]);
                  return -EIO;
                }

              /* - Bit pattern (x and y are valid bits)
               *  UTF-8 : 1110xxxx 10yyyyyy 10zzzzzz
               *  UCS2  : xxxxyyyy-yyzzzzzz
               */

              /* The lower 4 bits are shifted and copied. */

              dst[dst_idx]  = (src[i] & 0x0F) << 12;
              i++;
   
              /* The lower 6 bits are shifted and copied. */
   
              dst[dst_idx] |= (src[i] & 0x3F) << 6;
              i++;

              /* The lower 6 bits are copied. */

              dst[dst_idx] |= (src[i] & 0x3F);
            }
        }
      else
        {
          UCONV_DEBUG("Unexpected data : %02X.\n", src[i]);
          return -EIO;
        }

      i++;
      dst_idx++;
    }
  if (i != src_size)
    {
      UCONV_DEBUG("Exceeded the maximum length of dst [%d].\n", dst_idx);
    }

  return dst_idx;
}

int uconv_ucs2_to_utf8(int src_size, uint16_t *src,
                       int dst_size, uint8_t *dst)
{
  int     i     = 0;
  int     dst_idx = 0;

  if (src_size <= 0 || dst_size <= 0)
    {
      UCONV_DEBUG("Invalid size or length.\n");
      UCONV_DEBUG("src_size:%d, dst_size:%d.\n", src_size, dst_size);
      return -EINVAL;
    }

  if (!src || !dst)
    {
      UCONV_DEBUG("src or dst is NULL.\n");
      return -EINVAL;
    }

  for (i = 0; (i < src_size && dst_idx < dst_size); i++)
    {

      /* Copy the 7bit code as it is.
       *
       * - Valid UCS2 range
       *  0x0000 - 0x007F
       */

      if (src[i] <= 0x7F)
        {

          /* - Bit pattern (x is valid bits)
           *  UCS2  : 00000000-0xxxxxxx
           *  UTF-8 :          0xxxxxxx
           */

          dst[dst_idx] = (uint8_t)src[i];
          dst_idx++;
        }
      else if (0x0080 <= src[i] && src[i] <= 0x07FF)
        {

          /* 2 byte UTF-8 data.
           *
           * - Valid UCS2 range
           *  0x0800 - 0x0FFF
           */

          if (dst_size < (dst_idx + 2))
            {
              UCONV_DEBUG("Exceeded the maximum length of dst [%d].\n", (dst_idx + 2));
              break;
            }
          else
            {

              /* - Bit pattern (x and y are valid bits)
               *  UCS2  : 00000xxx-xxyyyyyy
               *  UTF-8 : 110xxxxx 10yyyyyy
               */

              dst[dst_idx]     = 0xC0 | ((src[i] >> 6) & 0x1F);
              dst[dst_idx + 1] = 0x80 | ((uint8_t)src[i] & 0x3F);
              dst_idx += 2;
            }
        }
      else if (0x0800 <= src[i] && src[i] <= 0xFFFF)
        {

          /* 3 byte UTF-8 data.
           *
           * - Valid UCS2 range
           *  0x1000 - 0xFFFF
           */

          if (dst_size < (dst_idx + 3))
            {
              UCONV_DEBUG("Exceeded the maximum length of dst [%d].\n", (dst_idx + 3));
              break;
            }
          else
            {

              /* - Bit pattern (x and y are valid bits)
               *  UCS2  : xxxxyyyy-yyzzzzzz
               *  UTF-8 : 1110xxxx 10yyyyyy 10zzzzzz
               */

              dst[dst_idx]     = 0xE0 | ((src[i] >> 12) & 0x0F);
              dst[dst_idx + 1] = 0x80 | ((src[i] >> 6) & 0x3F);
              dst[dst_idx + 2] = 0x80 | ((uint8_t)src[i] & 0x3F);
              dst_idx += 3;
            }
        }
      else
        {
          UCONV_DEBUG("Unexpected data : %X.\n", src[i]);
          return -EIO;
        }
    }
  if (i != src_size)
    {
      UCONV_DEBUG("Exceeded the maximum length of dst [%d].\n", dst_idx);
    }

  return dst_idx;
}
