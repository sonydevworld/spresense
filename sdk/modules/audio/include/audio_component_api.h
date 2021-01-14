/****************************************************************************
 * modules/audio/include/audio_component_api.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_AUDIO_INCLUDE_AUDIO_COMPONENT_API_H
#define __MODULES_AUDIO_INCLUDE_AUDIO_COMPONENT_API_H


/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_object_api Audio Object Layer API
 * @{
 *
 * @file       audio_component_common_api.h
 * @brief      CXD5602 Audio Componet Layer API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "apus/apu_cmd.h"

#include "wien2_common_defs.h"
#include "memutils/memory_manager/MemHandle.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
enum ComponentEventType
{
  ComponentInit = 0,
  ComponentExec,
  ComponentFlush,
  ComponentSet,
};

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct ComponentCbParam
{
  ComponentEventType event_type;
  bool               result;
};

typedef bool (*ComponentCallback)(ComponentCbParam*, void*);


struct CustomProcParam
{
  uint8_t  *addr;
  uint32_t size;
};
/****************************************************************************
 * Ocillator Command
 ****************************************************************************/
struct ExecOscParam : Apu::ApuExecOscCmd
{
};

struct SetOscParam : Apu::ApuSetOscCmd
{
};

struct OscCmpltParam
{
  uint32_t event_type;
  bool     result;
  union
  {
    ExecOscParam exec_osc_param;
    SetOscParam  set_osc_param;
  };
};

typedef bool (*OscCompCallback)(OscCmpltParam*, void*);

struct InitOscParam : Apu::ApuInitOscCmd
{
  OscCompCallback  callback;
  void            *instance;
};


/****************************************************************************
 * Initiarize Command
 ****************************************************************************/
struct CommonInitParam
{
  uint32_t samples;
  uint32_t in_fs;
  uint32_t out_fs;
  uint16_t in_bitlength;
  uint16_t out_bitlength;
  uint8_t  ch_num;
};

struct InitComponentParam
{
  uint8_t          cmd_type;
  bool             is_userdraw;

  union
  {
    CommonInitParam  common;
    CustomProcParam  custom;
    InitOscParam     osc;
  };
};


/****************************************************************************
 * Execute Command
 ****************************************************************************/

struct ExecComponentParam
{
  uint8_t          cmd_type;
  AsPcmDataParam   input;
  MemHandle        output;
  union
  {
    CustomProcParam  custom;
    ExecOscParam     osc;
  };
};


/****************************************************************************
 * Flush Command
 ****************************************************************************/
struct FlushComponentParam
{
  uint8_t           cmd_type;
  MemHandle         output;
  union
  {
    CustomProcParam  custom;
  };
};

/****************************************************************************
 * Set Command
 ****************************************************************************/
struct SetComponentParam
{
  uint8_t          cmd_type;
  bool             is_userdraw;

  union
  {
    CustomProcParam  custom;
    SetOscParam      osc;
  };
};


/****************************************************************************
 * Complete Result
 ****************************************************************************/

struct ComponentCmpltParam
{
  bool                 result;
  AsPcmDataParam       output;
  union
  {
    OscCmpltParam      osc;
  };
};

/****************************************************************************
 * Imformation Result
 ****************************************************************************/
struct ComponentInformParam
{
  bool              result;
  uint32_t          inform_req;
  AsRecognitionInfo inform_data;
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

#endif  /* __MODULES_AUDIO_INCLUDE_AUDIO_COMPONENT_API_H */
/**
 * @}
 */

/**
 * @}
 */
