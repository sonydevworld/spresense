/****************************************************************************
 * examples/fft_pwbimu/imu_hydrant.cxx
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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

#include <imufft/imufft_worker_main.h>
#include "imu_hydrant.h"

/****************************************************************************
 * imu_hydrant Class Methods
 ****************************************************************************/

imu_hydrant::imu_hydrant(unsigned int chmap, int taps)
{
  unsigned int b;

  _chmap = chmap;
  _chs = 0;
  for (b = 1; b <= IMUHYDRANT_AZ; b <<= 1)
    {
      if (b & chmap) _chs++;
    }

  for (_taps = FFT_MINTAPS; _taps <= FFT_MAXTAPS; _taps <<= 1)
    {
      if (_taps == taps) break;
    }

  if (_taps > FFT_MAXTAPS)  _taps = 512;

  _mempool = new audiolite_mempoolapbuf;
  _hydrant = new audiolite_outputnode(NULL);
  _cur_buf = NULL;
  _cur_pos = 0;

  _mempool->create_instance(_taps * _chs * sizeof(float), 4);
}

imu_hydrant::~imu_hydrant()
{
  if (_cur_buf)
    {
      _cur_buf->release();
      _cur_buf = NULL;
    }

  if (_mempool)
    {
      delete _mempool;
      _mempool = NULL;
    }

  if (_hydrant)
    {
      delete _hydrant;
      _hydrant = NULL;
    }
}

void imu_hydrant::inject(cxd5602pwbimu_data_t *imu)
{
  unsigned int map = 1;
  float *memdata;

  /* Assumed imu data order is gx,gy,gz,ax,ay,az */

  float *imudata = &imu->gx;

  if (_cur_buf == NULL)
    {
      _cur_buf = (audiolite_memapbuf *)_mempool->allocate();
      _cur_pos = 0;
    }

  memdata = (float *)_cur_buf->get_data();

  while (map <= IMUHYDRANT_AZ)
    {
      if (map & _chmap)
        {
          memdata[_cur_pos++] = *imudata;
        }

      imudata++;
      map <<= 1;
    }

  if (_cur_pos >= _taps * _chs)
    {
      _cur_buf->set_storedsize(_taps * _chs * sizeof(float));
      _hydrant->push_data(_cur_buf);
      _cur_buf->release();
      _cur_buf = NULL;
      _cur_pos = 0;
    }
}

void imu_hydrant::bind(audiolite_component *comp)
{
  _hydrant->bind(comp->get_input());
}

int imu_hydrant::start()
{
  return audiolite_start(_hydrant);
}

void imu_hydrant::stop()
{
  audiolite_stop(_hydrant);
}
