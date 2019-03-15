/****************************************************************************
 * modules/audio/components/common/component_common.h
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

#ifndef WIEN2_COMPONENT_COMMON_H
#define WIEN2_COMPONENT_COMMON_H

#include "memutils/common_utils/common_assert.h"
#include "audio/audio_message_types.h"
#include "memutils/message/Message.h"

#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

__WIEN2_BEGIN_NAMESPACE

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
#define AUDIOUTILS_DSP_DEBUG_DUMP_SIZE  (1948)
#define LOG_ENTRY_NAME                  (8)

struct DebugLogInfo
{
  char name[LOG_ENTRY_NAME];
  void *addr;
};
#endif

template<typename T>
struct DspResult
{
  uint32_t exec_result;
  T        internal_result;
};

template<typename T>
class ComponentCommon
{
public:
  ComponentCommon() {}
  ~ComponentCommon() {}

  bool dsp_boot_check(MsgQueId dsp_dtq, uint32_t *dsp_inf);
  uint32_t dsp_init_check(MsgQueId dsp_dtq, T *internal);
  void dsp_init_complete(MsgQueId dsp_dtq, uint32_t result, T *internal);

private:
};

__WIEN2_END_NAMESPACE

#endif /* WIEN2_COMPONENT_COMMON_H */

