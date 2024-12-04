/****************************************************************************
 * modules/audiolite/src/base/al_stream.cxx
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

#include <errno.h>
#include <audiolite/al_debug.h>
#include <audiolite/al_stream.h>

/****************************************************************************
 * Class: audiolite_filestream
 ****************************************************************************/

audiolite_filestream::~audiolite_filestream()
{
  mossfw_lock_take(&_lock);
  if (_fp != NULL && _self_open)
    {
      fclose(_fp);
    }
  mossfw_lock_give(&_lock);
}

int audiolite_filestream::open_file(const char *fname, const char *flg)
{
  mossfw_lock_take(&_lock);
  _fp = fopen(fname, flg);
  if (_fp)
    {
      _self_open = true;
    }
  mossfw_lock_give(&_lock);

  return errno;
}

int audiolite_filestream::rfile(const char *fname)
{
  return open_file(fname, "rb");
}

int audiolite_filestream::wfile(const char *fname)
{
  return open_file(fname, "w+b");
}

int audiolite_filestream::set_file(FILE *fp)
{
  if (_fp == NULL)
    {
      _fp = fp;
      return OK;
    }

  return -EALREADY;
}

void audiolite_filestream::close()
{
  mossfw_lock_take(&_lock);
  if (_fp != NULL && _self_open)
    {
      fclose(_fp);
    }

  _fp = NULL;
  _self_open = false;
  mossfw_lock_give(&_lock);
}

int audiolite_filestream::filesize()
{
  if (_fp)
    {
      return ftell(_fp);
    }

  return -ENOENT;
}

int audiolite_filestream::seek(int size)
{
  if (_fp)
    {
      return fseek(_fp, size, SEEK_SET);
    }

  return -ENOENT;
}

int audiolite_filestream::seekcur(int size)
{
  if (_fp)
    {
      return fseek(_fp, size, SEEK_CUR);
    }

  return -ENOENT;
}

int audiolite_filestream::seekend(int size)
{
  if (_fp)
    {
      return fseek(_fp, size, SEEK_END);
    }

  return -ENOENT;
}

int audiolite_filestream::read_data(void *data, int sz, int toms)
{
  if (_fp)
    {
      return fread(data, 1, sz, _fp);
    }

  return -ENOENT;
}

int audiolite_filestream::write_data(void *data, int sz, int toms)
{
  if (_fp)
    {
      if (fwrite(data, sz, 1, _fp) == 1)
        {
          return sz;
        }
    }

  return -ENOENT;
}

bool audiolite_filestream::has_file()
{
  bool ret;
  mossfw_lock_take(&_lock);
  ret = _fp != NULL;
  mossfw_lock_give(&_lock);
  return ret;
}

int audiolite_filestream::receive_data(audiolite_mem *mem,
                                       int ofst, int toms)
{
  char *data = (char *)mem->get_data();
  int sz = mem->get_fullsize() - ofst;
  int ret = 0;

  mem->set_storedsize(0);
  if (_fp && sz > 0)
    {
      ret = fread(&data[ofst], 1, sz, _fp);
      if (feof(_fp))
        {
          mem->set_eof();
          fclose(_fp);
          _fp = NULL;
        }

      if (ret >= 0)
        {
          mem->set_storedsize(ret);
        }
    }

  return ret;
}

int audiolite_filestream::send_data(audiolite_mem *mem,
                                    int ofst, int toms)
{
  char *data = (char *)mem->get_data();
  int sz = mem->get_storedsize() - ofst;
  int ret = -EINVAL;

  if (_fp && sz > 0)
    {
      ret = fwrite(&data[ofst], 1, sz, _fp);
    }

  return ret;
}
