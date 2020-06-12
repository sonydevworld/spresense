/****************************************************************************
 * modules/audio/include/apus/dsp_audio_version.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_AUDIO_INCLUDE_APUS_DSP_AUDIO_VERSION_H
#define __MODULES_AUDIO_INCLUDE_APUS_DSP_AUDIO_VERSION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Version rule:
 * (change library).(change of DSP interface).(change of internal processing)
 */

/* Decoder Version. */

#define DSP_AACDEC_VERSION    0x010303    /* 01.03.03 */
#define DSP_MP3DEC_VERSION    0x010303    /* 01.03.03 */
#define DSP_OPUSDEC_VERSION   0x010304    /* 01.03.04 */
#define DSP_WAVDEC_VERSION    0x010403    /* 01.04.03 */

/* Encoder Version. */

#define DSP_MP3ENC_VERSION    0x020103    /* 02.01.03 */
#define DSP_OPUSENC_VERSION   0x010306    /* 01.03.06 */

/* Filter Version. */

#define DSP_MFESRC_VERSION    0x010203    /* 01.02.03 */
#define DSP_SRC_VERSION       0x020104    /* 02.01.04 */
#define DSP_MPPEAX_VERSION    0x010203    /* 01.02.03 */

/* Postfilter Version. */

#define DSP_POSTFLTR_VERSION  0x010101    /* 01.01.01 */

/* Recognizer Version. */

#define DSP_FREQDET_VERSION   0x010202    /* 01.02.02 */
#define DSP_MEASURE_VERSION   0x010202    /* 01.02.02 */
#define DSP_VADWUW_VERSION    0x010203    /* 01.02.03 */

/* Oscillator Version. */

#define DSP_OSC_VERSION       0x000001    /* 00.00.01 */

/* Version and cpu-id Mask. */

#define DSP_VERSION_MASK      0x00FFFFFF  /* 24bits [23-0]  */
#define DSP_CPU_ID_MASK       0xFF000000  /*  8bits [31-24] */

#define DSP_VERSION_GET_VER(data)    ((data) & 0x00FFFFFF)
#define DSP_VERSION_GET_CPU_ID(data) (((data) & 0xFF000000) >> 24)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_APUS_DSP_AUDIO_VERSION_H */

