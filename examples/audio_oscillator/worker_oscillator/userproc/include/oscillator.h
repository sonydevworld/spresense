/****************************************************************************
 * audio_oscillator/worker_oscillator/userproc/include/oscillator.h
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

#ifndef __OSCILLATOR_H__
#define __OSCILLATOR_H__

#include <wien2_common_defs.h>
#include <apus/apu_cmd.h>

#include <cstdlib>
#include "../../arm-none-eabi/include/math.h"
#include "arm_math.h"

#define MAX_CHANNEL_NUMBER 8

/*--------------------------------------------------------------------*/
/*  Wave Generator                                                */
/*--------------------------------------------------------------------*/
class GeneratorBase
{
public:
  void init(uint8_t, uint32_t, uint8_t);
  void set(uint32_t);

protected:
  q15_t* update_sample(q15_t* ptr, q15_t val);

  uint32_t  m_sampling_rate;  /**< Sampling rate of data */
  q15_t     m_theta;
  q15_t     m_omega;
  uint8_t   m_channels;
};

class SinGenerator : public GeneratorBase
{
public:
  void exec(q15_t*, uint16_t);
};

class RectGenerator : public GeneratorBase
{
public:
  void exec(q15_t*, uint16_t);

};

class SawGenerator : public GeneratorBase
{
public:
  void exec(q15_t*, uint16_t);
};

/*--------------------------------------------------------------------*/
/*  Envelope Generator                                                */
/*--------------------------------------------------------------------*/
class EnvelopeGenerator
{
public:
  void init(uint8_t, /* bit_length */
            uint8_t  /* channel_num */);

  void set(uint16_t, /* Attack :ms */
           uint16_t, /* Decay  :ms */
           uint16_t, /* Sustain:%  */
           uint16_t  /* Release:ms */
           );

  void exec(q15_t*, uint16_t);

  void start(void);

  void stop(void);

private:

  enum EgState
  {
    Attack_state = 0,
    Decay_state,
    Sustain_state,
    Release_state,
    Ready_state,
    EgStateNum
  };

  float   m_cur_coef;
  EgState m_cur_state;

  float m_a_delta;
  float m_d_delta;
  float m_r_delta;
  q15_t m_s_level;

  uint8_t m_channels;
  uint8_t m_bits;

  uint16_t attack(q15_t**, uint16_t);
  uint16_t decay(q15_t**, uint16_t);
  uint16_t sustain(q15_t**, uint16_t);
  uint16_t release(q15_t**, uint16_t);
  uint16_t ready(q15_t**, uint16_t);
};

/*--------------------------------------------------------------------*/
/*  Oscillator                                                        */
/*--------------------------------------------------------------------*/
class Oscillator
{
public:

  Oscillator()
    : m_state(Booted)
  {}

  void parse(Wien2::Apu::Wien2ApuCmd *cmd);

  void illegal(Wien2::Apu::Wien2ApuCmd *cmd);
  void init(Wien2::Apu::Wien2ApuCmd *cmd);
  void exec(Wien2::Apu::Wien2ApuCmd *cmd);
  void flush(Wien2::Apu::Wien2ApuCmd *cmd);
  void set(Wien2::Apu::Wien2ApuCmd *cmd);

private:
  Wien2::WaveMode m_type;
  uint8_t m_channel_num;
  uint8_t m_bit_length;
  GeneratorBase* m_wave[MAX_CHANNEL_NUMBER];
  SinGenerator   m_sin[MAX_CHANNEL_NUMBER];
  RectGenerator  m_rect[MAX_CHANNEL_NUMBER];
  SawGenerator   m_saw[MAX_CHANNEL_NUMBER];
  EnvelopeGenerator m_envlop[MAX_CHANNEL_NUMBER];

  enum OscState
  {
    Booted = 0,
    Ready,
    Active,

    OscStateNum
  };

  OscState m_state;

  typedef void (Oscillator::*CtrlProc)(Wien2::Apu::Wien2ApuCmd *cmd);
  static CtrlProc CtrlFuncTbl[Wien2::Apu::ApuEventTypeNum][OscStateNum];
};

#endif /* __OSCILLATOR_H__ */
