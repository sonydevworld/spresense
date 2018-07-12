/****************************************************************************
 * modules/include/asmp/mpsignal.h
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
/**
 * @file mpsignal.h
 */

#ifndef __INCLUDE_ASMP_MPSIGNAL_H
#define __INCLUDE_ASMP_MPSIGNAL_H

/**
 * MP signal numbers
 */

#define MPSIGSTART       1      /**< Task started */
#define MPSIGEXIT        2      /**< Task exited */
#define MPSIGHARDFAULT   3      /**< Hard fault */
#define MPSIGBUSFAULT    4      /**< BUS fault */
#define MPSIGPING        5      /**< Ping */
#define MPSIGREQ         6      /**< Request */
#define MPSIGRESP        7      /**< Response */
#define MPSIGABORT       8      /**< Abort to task */
#define MPSIGSYS         9      /**< Other system reserved */
#define MPSIGDEBUG      10      /**< Debug signal */

typedef struct mpsignal
{
  int8_t   signo;
  uint16_t sigdatas;
  uint32_t sigdatal;
} mpsignal_t;

#endif /* __INCLUDE_ASMP_MPSIGNAL_H */
