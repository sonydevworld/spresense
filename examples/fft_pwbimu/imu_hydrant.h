/****************************************************************************
 * examples/fft_pwbimu/imu_hydrant.h
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

#ifndef __EXAMPLES_FFT_PWBIMU_IMU_HYDRANT_H
#define __EXAMPLES_FFT_PWBIMU_IMU_HYDRANT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/audiolite.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMUHYDRANT_GX (1 << 0)
#define IMUHYDRANT_GY (1 << 1)
#define IMUHYDRANT_GZ (1 << 2)
#define IMUHYDRANT_AX (1 << 3)
#define IMUHYDRANT_AY (1 << 4)
#define IMUHYDRANT_AZ (1 << 5)

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class imu_hydrant
{
  private:
    audiolite_mempoolapbuf *_mempool;
    audiolite_outputnode   *_hydrant;
    audiolite_memapbuf     *_cur_buf;
    int _cur_pos;

    int _chs;
    int _taps;
    unsigned int _chmap;

  public:
    imu_hydrant(unsigned int chmap, int taps);
    ~imu_hydrant();

    void inject(cxd5602pwbimu_data_t *imu);
    void bind(audiolite_component *);
    int start();
    void stop();
    void unbindall() { _hydrant->unbind(); };

    int taps() { return _taps; };
    int chs()  { return _chs;  };
};

#endif /* __EXAMPLES_FFT_PWBIMU_IMU_HYDRANT_H */
