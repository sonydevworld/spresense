/****************************************************************************
 * modules/include/audiolite/al_stream.h
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

#ifndef __INCLUDE_AUDIOLITE_STREAM_H
#define __INCLUDE_AUDIOLITE_STREAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <mossfw/mossfw_lock.h>
#include <audiolite/al_memalloc.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_stream
 ****************************************************************************/

class audiolite_stream
{
  public:
    virtual ~audiolite_stream(){};

    virtual int rfile(const char *fname) = 0;
    virtual int wfile(const char *fname) = 0;

    virtual void close() = 0;
    virtual int filesize() = 0;
    virtual int seek(int size) = 0;
    virtual int seekcur(int size) = 0;
    virtual int seekend(int size) = 0;
    virtual bool has_file() = 0;

    virtual int read_data(void *data, int sz, int toms) = 0;
    virtual int write_data(void *data, int sz, int toms) = 0;
    virtual int receive_data(audiolite_mem *mem, int ofst, int toms) = 0;
    virtual int send_data(audiolite_mem *mem, int ofst, int toms) = 0;
};

/****************************************************************************
 * class: audiolite_filestream
 ****************************************************************************/

class audiolite_filestream : public audiolite_stream
{
  private:
    FILE *_fp;
    bool _self_open;
    mossfw_lock_t _lock;

    int open_file(const char *fname, const char *flg);

  public:
    audiolite_filestream() : _fp(NULL), _self_open(false)
    {
      mossfw_lock_init(&_lock);
    };
    ~audiolite_filestream();

    int rfile(const char *fname);
    int wfile(const char *fname);
    int set_file(FILE *fp);

    void close();
    int filesize();
    int seek(int size);
    int seekcur(int size);
    int seekend(int size);
    bool has_file();

    int read_data(void *data, int sz, int toms);
    int write_data(void *data, int sz, int toms);
    int receive_data(audiolite_mem *mem, int ofst, int toms);
    int send_data(audiolite_mem *mem, int ofst, int toms);
};

#endif  /* __INCLUDE_AUDIOLITE_STREAM_H */
