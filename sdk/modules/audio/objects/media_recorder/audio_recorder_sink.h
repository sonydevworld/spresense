/****************************************************************************
 * modules/audio/objects/media_recorder/audio_recorder_sink.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_AUDIO_RECORDER_SINK_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_AUDIO_RECORDER_SINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "wien2_common_defs.h"
#include "wien2_internal_packet.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Internal parameters between voice recorder object and its user. */

/* Parameters for initializing sinker of voice recorder
 * that uses RAM to store output data.
 */

struct InitAudioRecRamSinkParam_s
{
public:
  AsRecorderOutputDeviceHdlr output_device_hdlr;
};

/* Parameters for initializing sinker of voice recorder. */

struct InitAudioRecSinkParam_s
{
public:
  InitAudioRecRamSinkParam_s init_audio_ram_sink;
};

/* Data to the sinker of voice recorder. */

struct AudioRecSinkData_s
{
public:
  MemMgrLite::MemHandle mh;
  uint32_t byte_size;
};

class AudioRecorderSink
{
public:
  AudioRecorderSink() {}

  ~AudioRecorderSink() {}

  bool init(const InitAudioRecSinkParam_s &param);
  bool write(const AudioRecSinkData_s &param);
  bool finalize(void);

private:
  AsRecorderOutputDeviceHdlr m_output_device_hdlr;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_AUDIO_RECORDER_SINK_H */
