/****************************************************************************
 * audio_oscillator/worker_oscillator/userproc/src/oscillator.cpp
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

#include "oscillator.h"
#include <audio/audio_synthesizer_api.h>

/*--------------------------------------------------------------------*/
/*   Wave Generator Base                                              */
/*--------------------------------------------------------------------*/
void GeneratorBase::init(uint8_t bits/* only 16bits*/ , uint32_t rate, uint8_t channels)
{
  m_theta = 0;
  m_omega = 0;
  m_sampling_rate = rate;
  m_channels = (channels < 2) ? 2 : channels;

}
/*--------------------------------------------------------------------*/
void GeneratorBase::set(uint32_t frequency)
{
  if (frequency != 0)
    {
      m_omega = frequency * 0x7fff / m_sampling_rate;
    }
}

/*--------------------------------------------------------------------*/
q15_t* GeneratorBase::update_sample(q15_t* ptr, q15_t val)
{
  *ptr = val;
  if (m_channels < 2)
    {
      *(ptr+1) = val;
    }
  m_theta = (m_theta + m_omega) & 0x7fff;
  ptr += m_channels;
  return ptr;
}

/*--------------------------------------------------------------------*/
/*  Sin Wave Generator                                                */
/*--------------------------------------------------------------------*/
void SinGenerator::exec(q15_t* ptr, uint16_t samples)
{
  for (int i = 0; i < samples ; i++)
    {
      q15_t val = arm_sin_q15(m_theta);
      ptr = update_sample(ptr,val);
    }
}

/*--------------------------------------------------------------------*/
/*   Rectangle Wave Generator                                         */
/*--------------------------------------------------------------------*/
void RectGenerator::exec(q15_t* ptr, uint16_t samples)
{
  for (int i = 0; i < samples ; i++)
    {
     q15_t val = (m_theta > 0x3fff) ?  0x7fff: 0x8000;
     ptr = update_sample(ptr,val);
    }
}

/*--------------------------------------------------------------------*/
/*  Saw Wave Generator                                                */
/*--------------------------------------------------------------------*/

void SawGenerator::exec(q15_t* ptr, uint16_t samples)
{
  for (int i = 0; i < samples ; i++)
    {
     q15_t val = m_theta;
     ptr = update_sample(ptr,val);
    }
}

/*--------------------------------------------------------------------*/
/*    Envelope Generator                                              */
/*--------------------------------------------------------------------*/
void EnvelopeGenerator::init(uint8_t bits, uint8_t channels)
{
  m_channels = channels;
  m_bits = bits;
  m_cur_coef = 0;
  set(1,1,100,1);

  m_cur_state = Ready_state;
}

/*--------------------------------------------------------------------*/
void EnvelopeGenerator::set(uint16_t attack, uint16_t decay, uint16_t sustain, uint16_t release)
{
  m_a_delta = (float) 0x7fff/(48*attack);  // only 48kHz sampling
  m_s_level = 0x7fff * (uint32_t) sustain / 100;
  m_d_delta = (float) (0x7fff- m_s_level)/(48*decay);  // only 48kHz sampling
  m_r_delta = (float) (m_s_level)/(48*release);  // only 48kHz sampling

  m_cur_state = Attack_state;
}

/*--------------------------------------------------------------------*/
void EnvelopeGenerator::start(void)
{
    m_cur_state = Attack_state;
}

/*--------------------------------------------------------------------*/
void EnvelopeGenerator::stop(void)
{
    m_cur_state = Release_state;
}

/*--------------------------------------------------------------------*/
void EnvelopeGenerator::exec(q15_t* ptr, uint16_t samples)
{
  while (samples > 0)
    {
      switch (m_cur_state)
        {
          case Attack_state:
            samples = attack(&ptr, samples);
            break;
          case Decay_state:
            samples = decay(&ptr, samples);
            break;
          case Sustain_state:
            samples = sustain(&ptr, samples);
            break;
          case Release_state:
            samples = release(&ptr, samples);
            break;
          case Ready_state:
            samples = ready(&ptr, samples);
            break;
          default:
            break;
        }
    }
}

/*--------------------------------------------------------------------*/
uint16_t EnvelopeGenerator::attack(q15_t** top, uint16_t samples)
{
  q15_t *ptr = *top;

  while (samples>0)
    {
      float data = *ptr;
      *ptr = (q15_t)((data * m_cur_coef) / 0x7fff);
      m_cur_coef += m_a_delta;

      samples--;
      ptr += m_channels;

      if (m_cur_coef > (float)0x7fff)
        {
          m_cur_coef = 0x7fff;
          m_cur_state = Decay_state;
          break;
        }
    }

  *top = ptr;

  return samples;
}
/*--------------------------------------------------------------------*/
uint16_t EnvelopeGenerator::decay(q15_t** top, uint16_t samples)
{
  q15_t *ptr = *top;

  while (samples>0)
    {
      float data = *ptr;
      *ptr = (q15_t)((data * m_cur_coef) / 0x7fff);
      m_cur_coef -= m_d_delta;
      samples--;
      ptr += m_channels;

      if (m_cur_coef < m_s_level)
        {
          m_cur_state = Sustain_state;
          break;
        }
    }

  *top = ptr;

  return samples;
}
/*--------------------------------------------------------------------*/
uint16_t EnvelopeGenerator::sustain(q15_t** top, uint16_t samples)
{
  q15_t *ptr = *top;

  while (samples>0)
    {
      float data = *ptr;
      *ptr = (q15_t)((data * m_cur_coef) / 0x7fff);
      samples--;
      ptr += m_channels;
    }

  *top = ptr;

  return samples;
  
}
/*--------------------------------------------------------------------*/
uint16_t EnvelopeGenerator::release(q15_t** top, uint16_t samples)
{
  q15_t *ptr = *top;

  while (samples>0)
    {
      float data = *ptr;
      *ptr = (q15_t)((data * m_cur_coef) / 0x7fff);
      m_cur_coef -= m_r_delta;
      samples--;
      ptr += m_channels;

      if (m_cur_coef < 0)
        {
          m_cur_coef = 0;
          m_cur_state = Ready_state;
        }
    }

  *top = ptr;

  return samples;
}
/*--------------------------------------------------------------------*/
uint16_t EnvelopeGenerator::ready(q15_t** top, uint16_t samples)
{
  q15_t *ptr = *top;

  while (samples > 0)
    {
      *ptr = 0;
      samples--;
      ptr += m_channels;
    }

  *top = ptr;

  return 0;
}

/*--------------------------------------------------------------------*/
/*  Oscillator                                                        */
/*--------------------------------------------------------------------*/

Oscillator::CtrlProc Oscillator::CtrlFuncTbl[Wien2::Apu::ApuEventTypeNum][OscStateNum] =
{
                /* Booted */          /* Ready */           /* Active */
/* boot   */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::illegal },
/* Init   */  { &Oscillator::init,    &Oscillator::init,    &Oscillator::illegal },
/* Exec   */  { &Oscillator::illegal, &Oscillator::exec,    &Oscillator::exec    },
/* Flush  */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::flush   },
/* Set    */  { &Oscillator::illegal, &Oscillator::set,     &Oscillator::set     },
/* tuning */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::illegal },
/* error  */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::illegal }
};

/*--------------------------------------------------------------------*/
void Oscillator::parse(Wien2::Apu::Wien2ApuCmd *cmd)
{
  if (cmd->header.process_mode != Wien2::Apu::OscMode)
    {
      cmd->result.exec_result = Wien2::Apu::ApuExecError;
      return;
    }

  (this->*CtrlFuncTbl[cmd->header.event_type][m_state])(cmd);
}

/*--------------------------------------------------------------------*/
void Oscillator::illegal(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Divide the code so that you can distinguish between data errors */

  cmd->result.exec_result = Wien2::Apu::ApuExecError;
}

/*--------------------------------------------------------------------*/
void Oscillator::init(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Init signal process. */

  /* Data check */
  m_type        = cmd->init_osc_cmd.type;
  m_bit_length  = cmd->init_osc_cmd.bit_length;
  m_channel_num = cmd->init_osc_cmd.channel_num;

  switch (m_type)
    {
      case AsSynthesizerSinWave:
        for (int i = 0; i < MAX_CHANNEL_NUMBER; i++)
          {
            m_wave[i] = &m_sin[i];
          }
        break;
      case AsSynthesizerRectWave:
        for (int i = 0; i < MAX_CHANNEL_NUMBER; i++)
          {
            m_wave[i] = &m_rect[i];
          }
        break;
      case AsSynthesizerSawWave:
        for (int i = 0; i < MAX_CHANNEL_NUMBER; i++)
          {
            m_wave[i] = &m_saw[i];
          }
        break;
      default:
        cmd->result.exec_result = Wien2::Apu::ApuExecError;
        return;
    }

  for (int i = 0; i < m_channel_num; i++)
    {
      m_wave[i]->init(m_bit_length,
                      cmd->init_osc_cmd.sampling_rate,
                      m_channel_num);

      m_envlop[i].init(m_bit_length, m_channel_num);

      /* set */

      m_envlop[i].set(cmd->init_osc_cmd.env.attack,
                      cmd->init_osc_cmd.env.decay,
                      cmd->init_osc_cmd.env.sustain,
                      cmd->init_osc_cmd.env.release);
    }

  m_state = Ready;

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void Oscillator::exec(Wien2::Apu::Wien2ApuCmd *cmd)
{
   /* Execute process to input audio data. */

  q15_t* ptr = (q15_t*)cmd->exec_osc_cmd.buffer.p_buffer;

  /* Byte size per sample.
   * If ch num is 1, but need to extend mono data to L and R ch.
   */
  uint16_t samples = cmd->exec_osc_cmd.buffer.size/m_channel_num/(m_bit_length/8);

  for (int i = 0; i < m_channel_num; i++)
    {
      switch (m_type)
        {
          case AsSynthesizerSinWave:
            m_sin[i].exec((ptr + i), samples);
            break;
          case AsSynthesizerRectWave:
            m_rect[i].exec((ptr + i), samples);
            break;
          case AsSynthesizerSawWave:
            m_saw[i].exec((ptr + i), samples);
            break;
          default:
            m_sin[i].exec((ptr + i), samples);
            break;
        }

      m_envlop[i].exec((ptr + i), samples);
    }

  m_state = Active;

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void Oscillator::flush(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Flush process. */

  m_state = Ready;

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void Oscillator::set(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Set process parameters. */

  uint8_t   ch   = cmd->setparam_osc_cmd.channel_no;
  uint32_t  type = cmd->setparam_osc_cmd.type;

  if (type & Wien2::Apu::OscTypeEnvelope)
    {
      m_envlop[ch].set(cmd->setparam_osc_cmd.env.attack,
                       cmd->setparam_osc_cmd.env.decay,
                       cmd->setparam_osc_cmd.env.sustain,
                       cmd->setparam_osc_cmd.env.release);
    }

  if (type & Wien2::Apu::OscTypeFrequency)
    {
      m_wave[ch]->set(cmd->setparam_osc_cmd.frequency);
      if (cmd->setparam_osc_cmd.frequency == 0)
        {
          m_envlop[ch].stop();
        }
      else
        {
          m_envlop[ch].start();
        }
    }

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}
