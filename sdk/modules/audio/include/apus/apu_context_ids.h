/****************************************************************************
 * modules/audio/include/apus/apu_context_ids.h
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

#ifndef __MODULES_AUDIO_INCLUDE_APUS_APU_CONTEXT_IDS_H
#define __MODULES_AUDIO_INCLUDE_APUS_APU_CONTEXT_IDS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "wien2_common_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FILE_TYPE_MPPEAX 13
#define FILE_TYPE_MFE    14

/* [07:03] FILETYPE enum defined in dsp_drv_inf.h.
 * [02:00] context types per DSP.
 */

#define DSP_MPPEAX_ID   (FILE_TYPE_MPPEAX << 3)
#define DSP_MFE_ID      (FILE_TYPE_MFE    << 3)

#define DSP_FILETYPE_MASK  0xF8

/* FILETYPE: MPPEAX. */

#define DSP_MPPEAX_CONTEXT_MPPEAX             (DSP_MPPEAX_ID | 0)
#define DSP_MPPEAX_CONTEXT_MFE                (DSP_MPPEAX_ID | 1)
#define DSP_MPPEAX_CONTEXT_1_3_SRC_FOR_MFE    (DSP_MPPEAX_ID | 2)
#define DSP_MPPEAX_CONTEXT_3_SRC_FOR_MFE      (DSP_MPPEAX_ID | 3)
#define DSP_MPPEAX_CONTEXT_1_3_SRC_FOR_MPPEAX (DSP_MPPEAX_ID | 4)
#define DSP_MPPEAX_CONTEXT_EAX                (DSP_MPPEAX_ID | 5)
#define DSP_MPPEAX_CONTEXT_NUM ((DSP_MPPEAX_CONTEXT_EAX + 1) & 7)

/* FILETYPE: MPPEAX. */

#define DSP_MFE_CONTEXT_MFE    (DSP_MFE_ID | 0)
#define DSP_MFE_CONTEXT_NUM    ((DSP_MFE_CONTEXT_MFE + 1) & 7)

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

#endif /* __MODULES_AUDIO_INCLUDE_APUS_APU_CONTEXT_IDS_H */
