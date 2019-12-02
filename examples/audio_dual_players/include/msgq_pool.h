/* This file is generated automatically. */
/****************************************************************************
 * msgq_pool.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#ifndef MSGQ_POOL_H_INCLUDED
#define MSGQ_POOL_H_INCLUDED

#include "msgq_id.h"

extern const MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] =
{
  /* n_drm, n_size, n_num, h_drm, h_size, h_num */

  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0 }, /* MSGQ_NULL */
  { 0xfd3b8, 88, 30, 0xffffffff, 0, 0 }, /* MSGQ_AUD_MGR */
  { 0xfde08, 64, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_APP */
  { 0xfde88, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_DSP0 */
  { 0xfdeec, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_DSP1 */
  { 0xfdf50, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PLY0 */
  { 0xfe040, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PFDSP0 */
  { 0xfe0a4, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PFDSP1 */
  { 0xfe108, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PLY1 */
  { 0xfe1f8, 48, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_OUTPUT_MIX */
  { 0xfe378, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY0 */
  { 0xfe578, 16, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY0_SYNC */
  { 0xfe598, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY1 */
  { 0xfe798, 16, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY1_SYNC */
};

#endif /* MSGQ_POOL_H_INCLUDED */
