/****************************************************************************
 * audio_recorder/worker/userproc/src/rcfilter.cpp
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

#include "rcfilter.h"

/*--------------------------------------------------------------------*/
/*                                                                    */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
bool RCfilter::init(void)
{
  return true;
}

/*--------------------------------------------------------------------*/
uint32_t RCfilter::exec(int16_t *in, uint32_t insize, int16_t *out, uint32_t outsize)
{
  /* Exec RC filter. */

  int16_t *ls_i = in;
  int16_t *rs_i = ls_i + 1;
  int16_t *ls_o = out;
  int16_t *rs_o = ls_o + 1;

  static int16_t ls_l = 0;
  static int16_t rs_l = 0;

  if (!ls_l && !rs_l)
    {
      ls_l = *ls_i;
      rs_l = *rs_i;
    }

  uint32_t cnt = 0;

  for (cnt = 0; cnt < insize; cnt += 4)
    {
      *ls_o = (ls_l * m_coef / 100) + (*ls_i * (100 - m_coef) / 100);
      *rs_o = (rs_l * m_coef / 100) + (*rs_i * (100 - m_coef) / 100);

      ls_l = *ls_o;
      rs_l = *rs_o;

      ls_i += 2;
      rs_i += 2;
      ls_o += 2;
      rs_o += 2;
    }

  return cnt;
}

/*--------------------------------------------------------------------*/
uint32_t RCfilter::flush(int16_t *out, uint32_t outsize)
{
  return 0;
}

/*--------------------------------------------------------------------*/
bool RCfilter::set(uint32_t coef)
{
  /* Set RC filter coef. */

  m_coef = static_cast<int16_t>(coef);

  return true;
}

