/****************************************************************************
 * modules/include/audiolite/al_eventlistner.h
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#ifndef __INCLUDE_AUDIOLITE_EVENT_LISTENER_H
#define __INCLUDE_AUDIOLITE_EVENT_LISTENER_H

#include <nuttx/config.h>
#include <audiolite/al_component.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AL_EVENT_OVERFLOW          (-1)
#define AL_EVENT_UNDERFLOW         (-2)
#define AL_EVENT_ILLIGALSTREAM     (-3)
#define AL_EVENT_UNSUPPORTFMT      (-4)
#define AL_EVENT_DECODEDONE        (-5)
#define AL_EVENT_STREAMDONE        (-6)
#define AL_EVENT_PLAYSTARTED       (-7)
#define AL_EVENT_PLAYSTOPPED       (-8)
#define AL_EVENT_RECORDSTARTED     (-9)
#define AL_EVENT_RECORDSTOPPED     (-10)
#define AL_EVENT_PLAYPAUSED        (-11)
#define AL_EVENT_PLAYRESUMED       (-12)
#define AL_EVENT_RECORDPAUSED      (-13)
#define AL_EVENT_RECORDRESUMED     (-14)
#define AL_EVENT_DRVERROR          (-15)
#define AL_EVENT_INVALIDSYSPARAM   (-16)
#define AL_EVENT_STOPOUTPUT        (-17)
#define AL_EVENT_STOPINPUT         (-18)
#define AL_EVENT_INITERROR         (-19)
#define AL_EVENT_SENDERROR         (-20)
#define AL_EVENT_MP3FRAMEINFO      (-21)
#define AL_EVENT_MP3DECWORKEREND   (-22)
#define AL_EVENT_UNKNOWN           (-23)
#define AL_EVENT_MP3DECERROR       (-24)
#define AL_EVENT_MP3DEC_WRONGTYPE  (-25)
#define AL_EVENT_WRONGVERSION      (-26)

class audiolite_component;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

const char *audiolite_strevent(int evt);

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * Class: audiolite_eventlistner
 ****************************************************************************/

class audiolite_eventlistener
{
  public:
    virtual ~audiolite_eventlistener(){};
    virtual void on_event(int evt, audiolite_component *cmp,
                                   unsigned long arg){};
};

/****************************************************************************
 * Class: audiolite_simplelistener
 ****************************************************************************/

class audiolite_simplelistener : public audiolite_eventlistener
{
  public:
    void on_event(int evt, audiolite_component *cmp,
                  unsigned long arg)
    {
      printf("AudioLite Event %s is happened : %d\n",
             audiolite_strevent(evt), (int)arg);
    }
};

#endif  /* __INCLUDE_AUDIOLITE_EVENT_LISTENER_H */
