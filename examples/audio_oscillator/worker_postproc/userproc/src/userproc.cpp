/****************************************************************************
 * audio_oscillator/worker_postproc/userproc/src/userproc.cpp
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

#include "userproc.h"

/*--------------------------------------------------------------------*/
/*                                                                    */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
void UserProc::init(InitParam *param)
{
  /* Init signal process. */

  m_channel_num = param->input_channel_num;

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::exec(ExecParam *param)
{
  /* Simple copy of input to output */

  memcpy(param->exec_cmd.output.addr,
         param->exec_cmd.input.addr,
         param->exec_cmd.input.size);

  /* Execute signal process to input audio data. */

  uint16_t  input_ch    = m_channel_num == 1 ? 2 : m_channel_num;
  uint16_t  byte_length = m_bit_length / 8;
  int16_t  *input       = (int16_t *)param->exec_cmd.input.addr;
  int16_t  *output      = (int16_t *)param->exec_cmd.output.addr;
  int32_t   data[MAX_CHANNEL_NUMBER];
  int32_t   data_l;
  int32_t   data_r;
  int32_t   dev;

  /* Currently, the sampling rate is 48 kHz and
   * the bit depth is fixed at 16 bits.
   */

  uint32_t sample_byte = byte_length * input_ch;

  for (uint32_t j = 0; j < param->exec_cmd.input.size / sample_byte; j++)
    {
      for (int i = 0; i < input_ch; i++)
        {
          data[i] = *input++;
        }

      data_l = 0;
      data_r = 0;

      for (int i = 0; i < input_ch; i += 2)
        {
          data_l += data[i];

          if (m_channel_num < 2)
            {
              data_r = 0;
            }
          else
            {
              data_r += data[i + 1];
            }
        }

      if (input_ch > 1)
        {
          dev = input_ch / 2;
        }
      else
        {
          dev = 1;
        }

      *output++ = (data_l / dev);
      *output++ = (data_r / dev);
    }

  /* Set output data size */

  param->exec_cmd.output.size = ((uint8_t *)output - (uint8_t *)param->exec_cmd.output.addr);

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::flush(FlushParam *param)
{
  param->flush_cmd.output.size = 0;

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::set(SetParam *param)
{
  param->result.result_code = CustomprocCommand::ExecOk;
}
