/*
    https://github.com/lieff/minimp3
    To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide.
    This software is distributed without any warranty.
    See <http://creativecommons.org/publicdomain/zero/1.0/>.
*/

#include <string.h>

#include "minimp3_ex.h"
#include "minimp3.h"

void mp3dec_skip_id3v1(const unsigned char *buf, size_t *pbuf_size)
{
  const unsigned char *tag;
  unsigned long tag_size;
  size_t buf_size = *pbuf_size;

  if (buf_size >= 128 && !memcmp(buf + buf_size - 128, "TAG", 3))
    {
      buf_size -= 128;
      if (buf_size >= 227 && !memcmp(buf + buf_size - 227, "TAG+", 4))
        {
          buf_size -= 227;
        }
    }

  if (buf_size > 32 && !memcmp(buf + buf_size - 32, "APETAGEX", 8))
    {
      buf_size -= 32;
      tag = buf + buf_size + 8 + 4;
      tag_size = (unsigned long)(tag[3] << 24) |
                           (tag[2] << 16) |
                           (tag[1] << 8)  | tag[0];

      if (buf_size >= tag_size)
        {
          buf_size -= tag_size;
        }
    }

  *pbuf_size = buf_size;
}

size_t mp3dec_skip_id3v2(const unsigned char *buf, size_t buf_size)
{
  if (buf_size >= MINIMP3_ID3_DETECT_SIZE && !memcmp(buf, "ID3", 3) &&
      !((buf[5] & 15) || (buf[6] & 0x80) || (buf[7] & 0x80) ||
        (buf[8] & 0x80) || (buf[9] & 0x80)))
    {
      size_t id3v2size = (((buf[6] & 0x7f) << 21) |
                          ((buf[7] & 0x7f) << 14) |
                          ((buf[8] & 0x7f) << 7)  | (buf[9] & 0x7f)) + 10;
      if ((buf[5] & 16))
        {
          id3v2size += 10; /* footer */
        }

      return id3v2size;
    }

  return 0;
}
