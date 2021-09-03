/****************************************************************************
 * modules/audio/include/audio_state.h
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

#ifndef __MODULES_AUDIO_INCLUDE_AUDIO_STATE_H
#define __MODULES_AUDIO_INCLUDE_AUDIO_STATE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include "debug/dbg_log.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

template <typename T>
class AudioState
{
private:
  T m_state;
  uint32_t m_mod_id;
  char m_name[5];

public:
  AudioState(uint32_t mod_id, FAR const char* name, T state)
    {
      m_mod_id = mod_id;
      strncpy(m_name, name, sizeof(m_name));
      m_name[4] = '\0';
      m_state = state;
    }

  AudioState& operator=(T state)
    {
      AUDIO_LOG_STATE(m_mod_id, "(%04lx) %s[%d>%d]\n",
        ((uint32_t)this & 0xffff), m_name, m_state, state);
      m_state = state;
      return *this;
    }

  bool operator==(T state)
    {
      return (m_state == state);
    }

  bool operator!=(T state)
    {
      return (m_state != state);
    }

  T get()
    {
      return m_state;
    }
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

#endif /* __MODULES_AUDIO_INCLUDE_AUDIO_STATE_H */

