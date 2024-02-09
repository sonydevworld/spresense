/****************************************************************************
 * modules/include/audiolite/al_debug.h
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

#ifndef __INCLUDE_AUDIOLITE_DEBUG_H
#define __INCLUDE_AUDIOLITE_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define al_debugmessage(T,...) \
  do { \
      printf("\n" T " %s:%s(%d) : \n" T "   ",    \
             __FILE__, __PRETTY_FUNCTION__, __LINE__); \
      printf(__VA_ARGS__); \
  } while(0)

#ifdef CONFIG_AL_DEBUG_ERR
#define al_derror(...) al_debugmessage("ALD[ERR]", __VA_ARGS__)
#else
#define al_derror(...)
#endif

#ifdef CONFIG_AL_DEBUG_WRN
#define al_dwarn(...) al_debugmessage("ALD[WRN]", __VA_ARGS__)
#else
#define al_dwarn(...)
#endif

#ifdef CONFIG_AL_DEBUG_DBG
#define al_ddebug(...) al_debugmessage("ALD[DBG]", __VA_ARGS__)
#else
#define al_ddebug(...)
#endif

#ifdef CONFIG_AL_DEBUG_INF
#define al_dinfo(...) al_debugmessage("ALD[INF]", __VA_ARGS__)
#else
#define al_dinfo(...)
#endif

#endif /* __INCLUDE_AUDIOLITE_DEBUG_H */
