/****************************************************************************
 * externals/tensorflow/tf_audio_util.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#ifndef __EXTERNALS_TF_WRAPPER_AUDIO_UTIL_H__
#define __EXTERNALS_TF_WRAPPER_AUDIO_UTIL_H__

#include <memutils/simple_fifo/CMN_SimpleFifo.h>
#include <audio/audio_high_level_api.h>

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

#define AUDIO_FRAME_SAMPLE_LENGTH (768)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

typedef void (*fifo_fill_cb_t)(uint32_t size);
typedef void (*annotation_cb_t)(const ErrorAttentionParam *param);

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

bool initailze_audio_captureing(
    CMN_SimpleFifoHandle *fifo_handle,
    annotation_cb_t anno_cb, fifo_fill_cb_t fill_cb,
    uint32_t sampling_rate, uint32_t chnum, uint32_t sample_per_bits);
void finalize_audio_capturing(void);
bool start_recording(CMN_SimpleFifoHandle *handle);
bool stop_recording(void);

#endif  // __EXTERNALS_TF_WRAPPER_AUDIO_UTIL_H__
