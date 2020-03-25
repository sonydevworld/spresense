/****************************************************************************
 * audio_oscillator/audio_synthesizer_melody.h
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

#ifndef __AUDIO_SYNTHESIZER_MELODY
#define __AUDIO_SYNTHESIZER_MELODY

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Musical scale */

#define M_C_4   262   /* Do */
#define M_CC4   277
#define M_D_4   294   /* Re */
#define M_DD4   311
#define M_E_4   330   /* Mi */
#define M_F_4   349   /* Fa */
#define M_FF4   370
#define M_G_4   392   /* So */
#define M_GG4   415
#define M_A_4   440   /* Ra */
#define M_AA4   466
#define M_B_4   494   /* Si */
#define M_C_5   523
#define M_END  9999

/****************************************************************************
 * Private data
 ****************************************************************************/

/* Define beep scale */

static struct {
  uint32_t  fs[8];
}
node[] = {
  {M_C_4,     0,     0,     0,     0,     0,     0,     0},
  {M_D_4,     0,     0,     0,     0,     0,     0,     0},
  {M_E_4,     0,     0,     0,     0,     0,     0,     0},
  {M_F_4,     0,     0,     0,     0,     0,     0,     0},
  {M_E_4,     0,     0,     0,     0,     0,     0,     0},
  {M_D_4,     0,     0,     0,     0,     0,     0,     0},
  {M_C_4,     0,     0,     0,     0,     0,     0,     0},
  {M_C_4,     0,     0,     0,     0,     0,     0,     0},
  {M_E_4, M_C_4,     0,     0,     0,     0,     0,     0},
  {M_F_4, M_D_4,     0,     0,     0,     0,     0,     0},
  {M_G_4, M_E_4,     0,     0,     0,     0,     0,     0},
  {M_A_4, M_F_4,     0,     0,     0,     0,     0,     0},
  {M_G_4, M_E_4,     0,     0,     0,     0,     0,     0},
  {M_F_4, M_D_4,     0,     0,     0,     0,     0,     0},
  {M_E_4, M_C_4,     0,     0,     0,     0,     0,     0},
  {M_E_4, M_C_4,     0,     0,     0,     0,     0,     0},
  {M_C_4, M_E_4, M_C_4,     0,     0,     0,     0,     0},
  {    0, M_F_4, M_D_4,     0,     0,     0,     0,     0},
  {M_C_4, M_G_4, M_E_4,     0,     0,     0,     0,     0},
  {    0, M_A_4, M_F_4,     0,     0,     0,     0,     0},
  {M_C_4, M_G_4, M_E_4,     0,     0,     0,     0,     0},
  {    0, M_F_4, M_D_4,     0,     0,     0,     0,     0},
  {M_C_4, M_E_4, M_C_4,     0,     0,     0,     0,     0},
  {    0, M_E_4, M_C_4,     0,     0,     0,     0,     0},
  {M_C_4, M_C_4, M_E_4, M_C_4,     0,     0,     0,     0},
  {M_D_4,     1, M_F_4, M_D_4,     0,     0,     0,     0},
  {M_E_4, M_C_4, M_G_4, M_E_4,     0,     0,     0,     0},
  {M_F_4,     1, M_A_4, M_F_4,     0,     0,     0,     0},
  {M_E_4, M_C_4, M_G_4, M_E_4,     0,     0,     0,     0},
  {M_D_4,     1, M_F_4, M_D_4,     0,     0,     0,     0},
  {M_C_4, M_C_4, M_E_4, M_C_4,     0,     0,     0,     0},
  {    0,     1, M_E_4, M_C_4,     0,     0,     0,     0},
  {    0, M_C_4, M_C_4, M_E_4, M_C_4,     0,     0,     0},
  {    0, M_D_4,     1, M_F_4, M_D_4,     0,     0,     0},
  {    0, M_E_4, M_C_4, M_G_4, M_E_4,     0,     0,     0},
  {    0, M_F_4,     1, M_A_4, M_F_4,     0,     0,     0},
  {    0, M_E_4, M_C_4, M_G_4, M_E_4,     0,     0,     0},
  {    0, M_D_4,     1, M_F_4, M_D_4,     0,     0,     0},
  {    0, M_C_4, M_C_4, M_E_4, M_C_4,     0,     0,     0},
  {    0,     0,     1, M_E_4, M_C_4,     0,     0,     0},
  {    0,     0, M_C_4, M_C_4, M_E_4, M_C_4,     0,     0},
  {    0,     0, M_D_4,     1, M_F_4, M_D_4,     0,     0},
  {    0,     0, M_E_4, M_C_4, M_G_4, M_E_4,     0,     0},
  {    0,     0, M_F_4,     1, M_A_4, M_F_4,     0,     0},
  {    0,     0, M_E_4, M_C_4, M_G_4, M_E_4,     0,     0},
  {    0,     0, M_D_4,     1, M_F_4, M_D_4,     0,     0},
  {    0,     0, M_C_4, M_C_4, M_E_4, M_C_4,     0,     0},
  {    0,     0,     0,     1, M_E_4, M_C_4,     0,     0},
  {    0,     0,     0, M_C_4, M_C_4, M_E_4, M_C_4,     0},
  {    0,     0,     0, M_D_4,     1, M_F_4, M_D_4,     0},
  {    0,     0,     0, M_E_4, M_C_4, M_G_4, M_E_4,     0},
  {    0,     0,     0, M_F_4,     1, M_A_4, M_F_4,     0},
  {    0,     0,     0, M_E_4, M_C_4, M_G_4, M_E_4,     0},
  {    0,     0,     0, M_D_4,     1, M_F_4, M_D_4,     0},
  {    0,     0,     0, M_C_4, M_C_4, M_E_4, M_C_4,     0},
  {    0,     0,     0,     0,     1, M_E_4, M_C_4,     0},
  {    0,     0,     0,     0, M_C_4, M_C_4, M_E_4, M_C_4},
  {    0,     0,     0,     0, M_D_4,     1, M_F_4, M_D_4},
  {    0,     0,     0,     0, M_E_4, M_C_4, M_G_4, M_E_4},
  {    0,     0,     0,     0, M_F_4,     1, M_A_4, M_F_4},
  {    0,     0,     0,     0, M_E_4, M_C_4, M_G_4, M_E_4},
  {    0,     0,     0,     0, M_D_4,     1, M_F_4, M_D_4},
  {    0,     0,     0,     0, M_C_4, M_C_4, M_E_4, M_C_4},
  {    0,     0,     0,     0,     0,     1, M_E_4, M_C_4},
  {    0,     0,     0,     0,     0, M_C_4, M_C_4, M_E_4},
  {    0,     0,     0,     0,     0, M_D_4,     1, M_F_4},
  {    0,     0,     0,     0,     0, M_E_4, M_C_4, M_G_4},
  {    0,     0,     0,     0,     0, M_F_4,     1, M_A_4},
  {    0,     0,     0,     0,     0, M_E_4, M_C_4, M_G_4},
  {    0,     0,     0,     0,     0, M_D_4,     1, M_F_4},
  {    0,     0,     0,     0,     0, M_C_4, M_C_4, M_E_4},
  {    0,     0,     0,     0,     0,     0,     1, M_E_4},
  {    0,     0,     0,     0,     0,     0, M_C_4, M_C_4},
  {    0,     0,     0,     0,     0,     0, M_D_4,     1},
  {    0,     0,     0,     0,     0,     0, M_E_4, M_C_4},
  {    0,     0,     0,     0,     0,     0, M_F_4,     1},
  {    0,     0,     0,     0,     0,     0, M_E_4, M_C_4},
  {    0,     0,     0,     0,     0,     0, M_D_4,     1},
  {    0,     0,     0,     0,     0,     0, M_C_4, M_C_4},
  {    0,     0,     0,     0,     0,     0,     0,     1},
  {    0,     0,     0,     0,     0,     0,     0, M_C_4},
  {    0,     0,     0,     0,     0,     0,     0, M_D_4},
  {    0,     0,     0,     0,     0,     0,     0, M_E_4},
  {    0,     0,     0,     0,     0,     0,     0, M_F_4},
  {    0,     0,     0,     0,     0,     0,     0, M_E_4},
  {    0,     0,     0,     0,     0,     0,     0, M_D_4},
  {    0,     0,     0,     0,     0,     0,     0, M_C_4},
  {M_END,     0,     0,     0,     0,     0,     0,     0},
},
*p_node;

#endif /* __AUDIO_SYNTHESIZER_MELODY */
