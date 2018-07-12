/****************************************************************************
 * modules/audio/objects/media_recorder/audio_recorder_sink.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio_recorder_sink.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

bool AudioRecorderSink::init(const InitAudioRecSinkParam_s &param)
{
  m_output_device_hdlr = param.init_audio_ram_sink.output_device_hdlr;
  return true;
}

/*--------------------------------------------------------------------------*/
bool AudioRecorderSink::write(const AudioRecSinkData_s &param)
{
  if (param.byte_size > 0) {
    if (CMN_SimpleFifoGetVacantSize(static_cast<CMN_SimpleFifoHandle *>
        (m_output_device_hdlr.simple_fifo_handler)) < param.byte_size)
      {
        MEDIA_RECORDER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_OVERFLOW);
        return false;
      }

    if (CMN_SimpleFifoOffer(static_cast<CMN_SimpleFifoHandle *>
        (m_output_device_hdlr.simple_fifo_handler),
        static_cast<const void*>(param.mh.getVa()), param.byte_size) == 0)
      {
        MEDIA_RECORDER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_OVERFLOW);
        return false;
      }

    m_output_device_hdlr.callback_function(param.byte_size);
  }
  return true;
}

/*--------------------------------------------------------------------------*/
bool AudioRecorderSink::finalize(void)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE
