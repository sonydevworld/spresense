/****************************************************************************
 * examples/audio_recorder/audio_util.h
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

#ifndef __EXAMPLES_AUDIO_RECORDER_AUDIO_UTIL_H__
#define __EXAMPLES_AUDIO_RECORDER_AUDIO_UTIL_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h> 

#include <memutils/simple_fifo/CMN_SimpleFifo.h>
#include <audio/audio_high_level_api.h>

#include <audio/utilities/wav_containerformat.h>

#ifdef CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC
#include "userproc_command.h"
#endif /* CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC */

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

enum format_type_e
{
  FORMAT_TYPE_RAW = 0,
  FORMAT_TYPE_WAV,
  FORMAT_TYPE_OGG,
  FORMAT_TYPE_NUM
};

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

typedef void (*fifo_fill_cb_t)(uint32_t);
typedef void (*attention_cb_t)(const ErrorAttentionParam *);

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

bool create_high_level_audio(void *, size_t, size_t, void *,
                             fifo_fill_cb_t, attention_cb_t);
bool standby_audio(void);
bool set_recording_mode(uint8_t, format_type_e, uint32_t, uint8_t,
                        uint8_t, const char *);
bool start_recording(void);
void stop_recording(FILE *);
bool delete_high_level_audio(void);
bool set_mic_gain(int16_t);
bool create_wav_header(FILE *);
bool update_wav_file_size(FILE *);
bool write_frames(FILE *);

#endif /* __EXAMPLES_AUDIO_RECORDER_AUDIO_UTIL_H__ */
