/****************************************************************************
 * modules/audiolite/src/components/al_wavenc.cxx
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
#include <audiolite/al_eventlistener.h>
#include <audiolite/al_wavenc.h>

/****************************************************************************
 * Class: audiolite_wavenc
 ****************************************************************************/

void *audiolite_wavenc::construct_wavheader(al_wavhdr *hdr)
{
  uint16_t chnum = channels();
  uint16_t fs = samplingrate();
  uint16_t bits = samplebitwidth();

  /* RIFF Chunk */

  memcpy(&hdr->riff_chunk_id[0], RIFF_ID, 4);
  hdr->riff_chunk_size = _crnt_sz + sizeof(al_wavhdr) - 8;
  memcpy(&hdr->riff_form_type[0], RIFF_FMT, 4);

  /* Format Chunk */

  memcpy(&hdr->fmt_chunk_id[0], FMTCNK_ID, 4);
  hdr->fmt_chunk_size = FMTCNK_SZ;
  hdr->fmt_wave_format_type = FMT_WAV;
  hdr->fmt_channel = chnum;
  hdr->fmt_samples_per_sec = fs;
  hdr->fmt_bytes_per_sec = fs * chnum * bits / 8;
  hdr->fmt_block_size = chnum * bits / 8;
  hdr->fmt_bits_per_sample = bits;

  /* Data Chunk */

  memcpy(&hdr->data_chunk_id[0], DATACNK_ID, 4);
  hdr->data_chunk_size = _crnt_sz;

  return (void *)hdr;
}

void audiolite_wavenc::create_fname()
{
  snprintf(&_fname[_prefixlen], 8, "_%02x.wav", _crnt_idx & 0xff);
  _crnt_idx++;
}

void audiolite_wavenc::terminate_wavfile()
{
  al_wavhdr hdr;

  if (_stream)
    {
      mossfw_lock_take(&_lock);
      if (_stream->has_file())
        {
          _stream->seek(0);
          _stream->write_data(construct_wavheader(&hdr),
                              sizeof(al_wavhdr), -1);
          _stream->close();
        }
      mossfw_lock_give(&_lock);
    }
}

void audiolite_wavenc::create_wav_file()
{
  al_wavhdr hdr;
  memset(&hdr, 0, sizeof(al_wavhdr));
  mossfw_lock_take(&_lock);
  create_fname();
  _stream->wfile(_fname);

  /* Write sizeof al_wavhdr to create header field on the file */

  _stream->write_data(&hdr, sizeof(al_wavhdr), -1);
  mossfw_lock_give(&_lock);
}

audiolite_wavenc::audiolite_wavenc() : audiolite_encoder("wavenc"),
                     _max_filelen(-1), _prefixlen(0),
                     _crnt_idx(0), _crnt_sz(0)
{
  _fname[0] = '\0';
  mossfw_lock_init(&_lock);
}

audiolite_wavenc::~audiolite_wavenc()
{
  terminate_wavfile();
}

void audiolite_wavenc::on_data()
{
  int ret;
  int wsize = 0;
  audiolite_memapbuf *mem =
          (audiolite_memapbuf *)pop_data();

  if (mem == NULL)
    {
      return;
    }

  mossfw_lock_take(&_lock);
  while (_stream->has_file() && wsize < mem->get_storedsize())
    {
      ret = _stream->send_data(mem, wsize, -1);
      if (ret <= 0)
        {
          mossfw_lock_give(&_lock);
          publish_event(AL_EVENT_SENDERROR, (unsigned long)ret);
          mem->release();
          return;
        }

      wsize += ret;
    }

  mossfw_lock_give(&_lock);

  mem->release();
  _crnt_sz += wsize;

  if (_max_filelen > 0 && _crnt_sz >= _max_filelen)
    {
      terminate_wavfile();
      create_wav_file();
      _crnt_sz = 0;
    }
}

int audiolite_wavenc::on_starting(audiolite_inputnode *inode,
                audiolite_outputnode *onode)
{
  int ret = -1;
  if (_stream)
    {
      create_wav_file();
      if (_stream->has_file())
        {
          ret = 0;
        }
    }

  return ret;
}

void audiolite_wavenc::on_started(audiolite_inputnode *inode,
                audiolite_outputnode *onode)
{
  audiolite_encoder::on_started(inode, onode);
}

void audiolite_wavenc::on_canceled(audiolite_inputnode *inode,
                 audiolite_outputnode *onode)
{
  audiolite_encoder::on_canceled(inode, onode);
}

void audiolite_wavenc::on_stop(audiolite_inputnode *inode,
             audiolite_outputnode *onode)
{
  al_ddebug("Entry\n");
  terminate_wavfile();
  audiolite_encoder::on_stop(inode, onode);
  al_ddebug("Leave\n");
}

int audiolite_wavenc::set_fileprefix(const char *pfx)
{
  int plen = strlen(pfx);
  if (plen <= (AL_WAVENC_FNAMELEN - 8))
    {
      memcpy(_fname, pfx, plen);
      _prefixlen = plen;
      return OK;
    }

  return -EINVAL;
}
