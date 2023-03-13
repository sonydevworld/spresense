/****************************************************************************
 * modules/include/audiolite/al_wavenc.h
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

#ifndef __INCLUDE_AUDIOLITE_WAVENC_H
#define __INCLUDE_AUDIOLITE_WAVENC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <audiolite/al_wavheader.h>
#include <audiolite/al_encoder.h>
#include <audiolite/al_stream.h>

#define AL_WAVENC_FNAMELEN  (64)

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_wavenc
 ****************************************************************************/

class audiolite_wavenc : public audiolite_encoder
{
  private:
    int _max_filelen;
    int _prefixlen;
    int _crnt_idx;
    int _crnt_sz;
    mossfw_lock_t _lock;

    char _fname[AL_WAVENC_FNAMELEN];

    void *construct_wavheader(al_wavhdr *hdr);
    void create_fname();
    void terminate_wavfile();

    void create_wav_file();

  public:
    audiolite_wavenc();
    ~audiolite_wavenc();

    void on_data();
    int on_starting(audiolite_inputnode *inode,
                    audiolite_outputnode *onode);
    void on_started(audiolite_inputnode *inode,
                    audiolite_outputnode *onode);
    void on_canceled(audiolite_inputnode *inode,
                     audiolite_outputnode *onode);
    void on_stop(audiolite_inputnode *inode,
                 audiolite_outputnode *onode);

    void set_max_filesize(int sz) { _max_filelen = sz; };
    int set_fileprefix(const char *pfx);
};

#endif /* __INCLUDE_AUDIOLITE_WAVENC_H */
