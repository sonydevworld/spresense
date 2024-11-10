/****************************************************************************
 * examples/audiolite_rec2net/event_str.h
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_AUDIOLITE_REC2NET_EVENT_STR_H
#define __EXAMPLES_AUDIOLITE_REC2NET_EVENT_STR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STRINGCASE(e) case AL_EVENT_##e: return #e;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *convert_evtid(int id)
{
  switch (id)
    {
      STRINGCASE(OVERFLOW)
      STRINGCASE(UNDERFLOW)
      STRINGCASE(ILLIGALSTREAM)
      STRINGCASE(UNSUPPORTFMT)
      STRINGCASE(DECODEDONE)
      STRINGCASE(STREAMDONE)
      STRINGCASE(PLAYSTARTED)
      STRINGCASE(PLAYSTOPPED)
      STRINGCASE(RECORDSTARTED)
      STRINGCASE(RECORDSTOPPED)
      STRINGCASE(PLAYPAUSED)
      STRINGCASE(PLAYRESUMED)
      STRINGCASE(RECORDPAUSED)
      STRINGCASE(RECORDRESUMED)
      STRINGCASE(DRVERROR)
      STRINGCASE(INVALIDSYSPARAM)
      STRINGCASE(STOPOUTPUT)
      STRINGCASE(STOPINPUT)
      STRINGCASE(INITERROR)
      STRINGCASE(SENDERROR)
      STRINGCASE(MP3FRAMEINFO)
      STRINGCASE(MP3DECWORKEREND)
      STRINGCASE(UNKNOWN)
      STRINGCASE(MP3DECERROR)
      STRINGCASE(MP3DEC_WRONGTYPE)
      STRINGCASE(WRONGVERSION)
      default:
        return "not event id...";
    }
}

#endif /* __EXAMPLES_AUDIOLITE_REC2NET_EVENT_STR_H */
