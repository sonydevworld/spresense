/****************************************************************************
 * bsp/include/sdk/debug.h
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

#ifndef __BSP_INCLUDE_SDK_DEBUG_H
#define __BSP_INCLUDE_SDK_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <nuttx/compiler.h>

#include <syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_HAVE_FUNCTIONNAME
#  define EXTRA_FMT "%s: "
#  define EXTRA_ARG ,__FUNCTION__
#else
#  define EXTRA_FMT
#  define EXTRA_ARG
#endif

#ifndef __debug_syslog
#  define __debug_syslog syslog
#endif

#ifdef CONFIG_SDK_DEBUG_EMERG
#  define logemerg(format, ...) \
   __debug_syslog(LOG_EMERG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define logemerg(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_ALERT
#  define logalert(format, ...) \
   __debug_syslog(LOG_ALERT, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define logalert(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_CRIT
#  define logcrit(format, ...) \
   __debug_syslog(LOG_CRIT, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define logcrit(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_ERR
#  define logerr(format, ...) \
   __debug_syslog(LOG_ERR, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define logerr(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_WARN
#  define logwarn(format, ...) \
   __debug_syslog(LOG_WARNING, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define logwarn(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_NOTICE
#  define lognotice(format, ...) \
   __debug_syslog(LOG_NOTICE, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define lognotice(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_INFO
#  define loginfo(format, ...) \
   __debug_syslog(LOG_INFO, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define loginfo(x, ...)
#endif

#ifdef CONFIG_SDK_DEBUG_DEBUG
#  define logdebug(format, ...) \
   __debug_syslog(LOG_DEBUG, EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#else
#  define logdebug(x, ...)
#endif

#endif /* __BSP_INCLUDE_SDK_DEBUG_H */
