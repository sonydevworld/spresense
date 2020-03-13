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

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdarg.h>
#include <syslog.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DBGIF_LV_ERR       LOG_ERR
#define DBGIF_LV_WARN      LOG_WARNING
#define DBGIF_LV_NORM      LOG_NOTICE
#define DBGIF_LV_INF       LOG_INFO
#define DBGIF_LV_DBG       LOG_DEBUG

static inline const char* dbgif_syslog_prefix(int level)
{
  char *prefix = "[XXX]";

  switch (level)
    {
      case LOG_ERR:
        prefix = "[ERR]";
        break;
      case LOG_WARNING:
        prefix = "[WRN]";
        break;
      case LOG_NOTICE:
        prefix = "[NTC]";
        break;
      case LOG_INFO:
        prefix = "[INF]";
        break;
      case LOG_DEBUG:
        prefix = "[DBG]";
        break;
      default:
        break;
    }

  return prefix;
}

static inline void dbgif_syslog(int level, FAR const IPTR char *fmt, ...)
{
  va_list ap;

  switch (level)
    {
#ifdef CONFIG_LTE_DEBUG_ERROR
      case LOG_ERR:
        break;
#endif
#ifdef CONFIG_LTE_DEBUG_WARN
      case LOG_WARNING:
        break;
#endif
#ifdef CONFIG_LTE_DEBUG_NOTICE
      case LOG_NOTICE:
        break;
#endif
#ifdef CONFIG_LTE_DEBUG_INFO
      case LOG_INFO:
        break;
#endif
#ifdef CONFIG_LTE_DEBUG_DEBUG
      case LOG_DEBUG:
        break;
#endif
      default:
        return;
        break;
    }
  va_start(ap, fmt);
  vsyslog(level, fmt, ap);
  va_end(ap);
}
#define DBGIF_EXTRA_FMT "%s %s:%d "
#define DBGIF_EXTRA_ARG(lv) ,dbgif_syslog_prefix(lv), __FUNCTION__, __LINE__
#define DBGIF_LOG(lv, fmt, ...) dbgif_syslog(lv, DBGIF_EXTRA_FMT fmt DBGIF_EXTRA_ARG(lv), ##__VA_ARGS__)

#ifdef NDEBUG
#  define DBGIF_ASSERT(asrt, msg) do { \
  if(!(asrt)) { DBGIF_LOG(DBGIF_LV_ERR, #msg, 0, 0, 0); \
  while(1);} } while(0)
#else
#  define DBGIF_ASSERT(assert, msg) ASSERT(assert)
#endif

#endif /* __MODULES_LTE_INCLUDE_OPT_DBG_OPT_H */
