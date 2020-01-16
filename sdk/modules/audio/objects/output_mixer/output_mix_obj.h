/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_OUTPUT_MIXER_OUTPUT_MIX_OBJ_H
#define __MODULES_AUDIO_OBJECTS_OUTPUT_MIXER_OUTPUT_MIX_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_message_types.h"
#include "audio/audio_outputmix_api.h"
#include "audio/audio_object_common_api.h"
#include "output_mix_sink_device.h"
#include "wien2_common_defs.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

class OutputMixObjectTask
{
public:
  static void create(AsOutputMixMsgQueId_t msgq_id,
                     AsOutputMixPoolId_t pool_id);

  OutputMixObjectTask(AsOutputMixMsgQueId_t msgq_id,
                      AsOutputMixPoolId_t pool_id);

private:
  AsOutputMixMsgQueId_t m_msgq_id;

  OutputMixToHPI2S m_output_mix_to_hpi2s_0;
  OutputMixToHPI2S m_output_mix_to_hpi2s_1;

  AsOutputMixDevice m_output_device;

  void run();
  int getHandle(MsgPacket* msg);
  void parse(MsgPacket* msg);
  void enableOutputDevice();
  void disableOutputDevice();
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

#endif /* __MODULES_AUDIO_OBJECTS_OUTPUT_MIXER_OUTPUT_MIX_OBJ_H */

