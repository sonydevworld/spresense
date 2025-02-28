/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/pll/pll_worker_main.h
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

#ifndef __AUDIOLITE_WORKER_USR_PLL_H
#define __AUDIOLITE_WORKER_USR_PLL_H

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define PLL_WORKER_VERSION (1)

#define PLLCMD_START      (1)
#define PLLCMD_STOP       (2)

#define PLLCMD_XFERDATA   (3 << 16)
#define PLLCMD_ENABLEXFER (4)
#define PLLCMD_CANCELXFER (5)
#define PLLCMD_DONE_XFER  (6)

#define PLLCMD_RETCODE_OK (0)
#define PLLCMD_RETCODE_NG (1)

#define PLLXFERDATACMDMASK  (0xffff0000)
#define PLLXFERDATACMD(ofst,len) \
        (PLLCMD_XFERDATA | ((ofst & 0xff) << 8) | (len & 0xff))
#define PLLXFERDATACMD_OFST(d) (((d) & 0xff00) >> 8)
#define PLLXFERDATACMD_LEN(d) ((d) & 0xff)

#define PLLCMD_RESP       (2)

#define SAMPLE_FS   (192000)
#define SAMPLE_BITS (16)
#define SAMPLE_CHS  (2)
#define BLK_SAMPLES (1024)

#define DEFAULT_REFFREQ     (200)
#define DEFAULT_CARRIERFREQ (35000)

#define XFER_DATA_BITS   (128)
#define XFER_DATA_BYTES  ((XFER_DATA_BITS + 7) / 8)

#define PIN_LED0 PIN_I2S1_BCK
#define PIN_LED1 PIN_I2S1_LRCK
#define PIN_LED2 PIN_I2S1_DATA_IN
#define PIN_LED3 PIN_I2S1_DATA_OUT

#endif /* __AUDIOLITE_WORKER_USR_PLL_H */
