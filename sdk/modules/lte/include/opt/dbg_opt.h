/****************************************************************************
 * modules/lte/include/opt/dbg_opt.h
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

#ifndef __MODULES_LTE_INCLUDE_OPT_DBG_OPT_H
#define __MODULES_LTE_INCLUDE_OPT_DBG_OPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sdk/debug.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// TODO To be enabled logging macro
#define logdebug(x...)
#define loginfo(x...)
#define lognotice(x...)
#define logwarn(x...)
#define logerr(x...)

#define DBGIF_LEVEL_ERR    0x01
#define DBGIF_LEVEL_WARN   0x02
#define DBGIF_LEVEL_NOTICE 0x04
#define DBGIF_LEVEL_INFO   0x08
#define DBGIF_LEVEL_DEBUG  0x10
#define DBFIF_LEVEL_MASK   0x1F

#define DBGIF_LV_ERR       DBGIF_LEVEL_ERR
#define DBGIF_LV_WARN      DBGIF_LEVEL_WARN
#define DBGIF_LV_NORM      DBGIF_LEVEL_NOTICE
#define DBGIF_LV_INF       DBGIF_LEVEL_INFO
#define DBGIF_LV_DBG       DBGIF_LEVEL_DEBUG

#define DBGIF_LOG(lv, fmt, prm1, prm2, prm3) \
  do { \
    if (((lv) & DBFIF_LEVEL_MASK) >= DBGIF_LEVEL_DEBUG) \
      { \
        logdebug("[DBG] "fmt, prm1, prm2, prm3); \
      } \
    else if (((lv) & DBFIF_LEVEL_MASK) >= DBGIF_LEVEL_INFO) \
      { \
        loginfo("[INF] "fmt, prm1, prm2, prm3); \
      } \
    else if (((lv) & DBFIF_LEVEL_MASK) >= DBGIF_LEVEL_NOTICE) \
      { \
        lognotice("[NRM] "fmt, prm1, prm2, prm3); \
      } \
    else if (((lv) & DBFIF_LEVEL_MASK) >= DBGIF_LEVEL_WARN) \
      { \
        logwarn("[WRN] "fmt, prm1, prm2, prm3); \
      } \
    else if (((lv) & DBFIF_LEVEL_MASK) >= DBGIF_LEVEL_ERR) \
      { \
        logerr("[ERR] "fmt, prm1, prm2, prm3); \
      } \
  } while(0)

#ifdef NDEBUG
#  define DBGIF_ASSERT(asrt, msg) do { \
  if(!(asrt)) { DBGIF_LOG(DBGIF_LV_ERR, #msg, 0, 0, 0); \
  while(1);} } while(0)
#else
#  define DBGIF_ASSERT(assert, msg) ASSERT(assert)
#endif

#endif /* __MODULES_LTE_INCLUDE_OPT_DBG_OPT_H */
