/****************************************************************************
 * modules/lte/include/util/dbg_if.h
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

#ifndef __MODULES_LTE_INCLUDE_UTIL_DBG_IF_H
#define __MODULES_LTE_INCLUDE_UTIL_DBG_IF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "dbg_opt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef DBGIF_LV_ERR
#  define DBGIF_LV_ERR 0
#endif

#ifndef DBGIF_LV_WARN
#  define DBGIF_LV_WARN 1
#endif

#ifndef DBGIF_LV_NORM
#  define DBGIF_LV_NORM 2
#endif

#ifndef DBGIF_LV_INF
#  define DBGIF_LV_INF 3
#endif

#ifndef DBGIF_LV_DBG
#  define DBGIF_LV_DBG 4
#endif

/* NOTE: This macros forbid line feeds */

#ifndef DBGIF_LOG
#  define DBGIF_LOG(lv, fmt, prm1, prm2, prm3)
#endif

#ifndef DBGIF_ASSERT
#  define DBGIF_ASSERT(asrt, msg)
#endif

/* NOTE: The following macros forbid line feeds */

#define DBGIF_LOG_LV_ERR(fmt, prm1, prm2, prm3)   DBGIF_LOG(DBGIF_LV_ERR, fmt, prm1, prm2, prm3)
#define DBGIF_LOG_LV_WARN(fmt, prm1, prm2, prm3)  DBGIF_LOG(DBGIF_LV_WARN, fmt, prm1, prm2, prm3)
#define DBGIF_LOG_LV_NORM(fmt, prm1, prm2, prm3)  DBGIF_LOG(DBGIF_LV_NORM, fmt, prm1, prm2, prm3)
#define DBGIF_LOG_LV_INF(fmt, prm1, prm2, prm3)   DBGIF_LOG(DBGIF_LV_INF, fmt, prm1, prm2, prm3)
#define DBGIF_LOG_LV_DBG(fmt, prm1, prm2, prm3)   DBGIF_LOG(DBGIF_LV_DBG, fmt, prm1, prm2, prm3)

/* No parameter */

#define DBGIF_LOG_ERROR(fmt)                      DBGIF_LOG_LV_ERR(fmt, NULL, NULL, NULL)
#define DBGIF_LOG_WARNING(fmt)                    DBGIF_LOG_LV_WARN(fmt, NULL, NULL, NULL)
#define DBGIF_LOG_NORMAL(fmt)                     DBGIF_LOG_LV_NORM(fmt, NULL, NULL, NULL)
#define DBGIF_LOG_INFO(fmt)                       DBGIF_LOG_LV_INF(fmt, NULL, NULL, NULL)
#define DBGIF_LOG_DEBUG(fmt)                      DBGIF_LOG_LV_DBG(fmt, NULL, NULL, NULL)

/* One parameter */

#define DBGIF_LOG1_ERROR(fmt, prm1)               DBGIF_LOG_LV_ERR(fmt, prm1, NULL, NULL)
#define DBGIF_LOG1_WARNING(fmt, prm1)             DBGIF_LOG_LV_WARN(fmt, prm1, NULL, NULL)
#define DBGIF_LOG1_NORMAL(fmt, prm1)              DBGIF_LOG_LV_NORM(fmt, prm1, NULL, NULL)
#define DBGIF_LOG1_INFO(fmt, prm1)                DBGIF_LOG_LV_INF(fmt, prm1, NULL, NULL)
#define DBGIF_LOG1_DEBUG(fmt, prm1)               DBGIF_LOG_LV_DBG(fmt, prm1, NULL, NULL)

/* Two parameters */

#define DBGIF_LOG2_ERROR(fmt, prm1, prm2)         DBGIF_LOG_LV_ERR(fmt, prm1, prm2, NULL)
#define DBGIF_LOG2_WARNING(fmt, prm1, prm2)       DBGIF_LOG_LV_WARN(fmt, prm1, prm2, NULL)
#define DBGIF_LOG2_NORMAL(fmt, prm1, prm2)        DBGIF_LOG_LV_NORM(fmt, prm1, prm2, NULL)
#define DBGIF_LOG2_INFO(fmt, prm1, prm2)          DBGIF_LOG_LV_INF(fmt, prm1, prm2, NULL)
#define DBGIF_LOG2_DEBUG(fmt, prm1, prm2)         DBGIF_LOG_LV_DBG(fmt, prm1, prm2, NULL)

/* Three parameters (Max parameters)
 * Not support more than four parameters
 */

#define DBGIF_LOG3_ERROR(fmt, prm1, prm2, prm3)   DBGIF_LOG_LV_ERR(fmt, prm1, prm2, prm3)
#define DBGIF_LOG3_WARNING(fmt, prm1, prm2, prm3) DBGIF_LOG_LV_WARN(fmt, prm1, prm2, prm3)
#define DBGIF_LOG3_NORMAL(fmt, prm1, prm2, prm3)  DBGIF_LOG_LV_NORM(fmt, prm1, prm2, prm3)
#define DBGIF_LOG3_INFO(fmt, prm1, prm2, prm3)    DBGIF_LOG_LV_INF(fmt, prm1, prm2, prm3)
#define DBGIF_LOG3_DEBUG(fmt, prm1, prm2, prm3)   DBGIF_LOG_LV_DBG(fmt, prm1, prm2, prm3)

#endif /* __MODULES_LTE_INCLUDE_UTIL_DBG_IF_H */

