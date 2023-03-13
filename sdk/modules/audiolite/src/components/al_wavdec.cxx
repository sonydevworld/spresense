/****************************************************************************
 * modules/audiolite/src/components/al_wavdec.cxx
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#include <audiolite/al_debug.h>
#include <audiolite/al_wavheader.h>
#include <audiolite/al_eventlistener.h>
#include <audiolite/al_wavdec.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int check_wavheader(al_wavhdr *hdr, int ch, int hz, int bits)
{
  int ret = 0;

  if ((strncmp((const char *)&hdr->riff_chunk_id[0], RIFF_ID, 4)))
    {
      ret = -1;
    }
  else if (strncmp((const char *)&hdr->riff_form_type[0], RIFF_FMT, 4))
    {
      ret = -2;
    }
  else if (strncmp((const char *)&hdr->fmt_chunk_id[0], FMTCNK_ID, 4))
    {
      ret = -3;
    }
  else if (strncmp((const char *)&hdr->data_chunk_id[0], DATACNK_ID, 4))
    {
      ret = -4;
    }
  else if (hdr->fmt_chunk_size != FMTCNK_SZ)
    {
      ret = -5;
    }
  else if (hdr->fmt_wave_format_type != FMT_WAV)
    {
      ret = -6;
    }
  else if (bits != 16)
    {
      ret = -7;
    }
  else if (!(hz == 48000 || hz == 44100 || hz == 96000 || hz == 192000))
    {
      ret = -8;
    }
  else if (!(ch == 1 || ch == 2 || ch == 4 || ch == 8))
    {
      ret = -9;
    }

  return ret;
}

/****************************************************************************
 * Class: audiolite_wavdec
 ****************************************************************************/

int audiolite_wavdec::start_decode()
{
  if (_stream)
    {
      _eof = false;
      ((audiolite_filestream *)_stream)->seek(0);
      return parse_wavhdr();
    }
  else
    {
      return -EAGAIN;
    }
}

int audiolite_wavdec::parse_wavhdr()
{
  int ret = -EINVAL;

  al_wavhdr hdr;

  ret = _stream->read_data(&hdr, sizeof(al_wavhdr), -1);
  if (ret == sizeof(al_wavhdr))
    {
      _chnum = hdr.fmt_channel;
      _samplerate = hdr.fmt_samples_per_sec;
      _bitlen = hdr.fmt_bits_per_sample;

      ret = check_wavheader(&hdr, _chnum, _samplerate, _bitlen);
      if (ret != 0)
        {
          /* Invalid format */

          publish_event(AL_EVENT_ILLIGALSTREAM, (unsigned long)ret);
        }
      else
        {
          ret = OK;
        }
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

void audiolite_wavdec::decode_runner()
{
  bool eof;
  al_dinfo("Entry\n");

  /* Wait for working condition */

  while (_is_thrdrun &&
         (_pool == NULL || _stream == NULL || !_isplay))
    {
      usleep(10 * 1000);
    }

  /* Main Loop */

  while (_is_thrdrun)
    {
      if (_isplay && !_ispause)
        {
          audiolite_mem *mem = _pool->allocate();
          if (mem)
            {
              _stream->receive_data(mem, 0, -1);
              if (!_eof)
                {
                  _outs[0]->push_data(mem);
                }

              eof = mem->is_eof();
              mem->release();

              if (!_eof && eof)
                {
                  _eof = true;
                  publish_event(AL_EVENT_DECODEDONE, 0);
                }
            }
        }
      else
        {
          /* Yeild */

          usleep(10 * 1000);
        }
    }
}
