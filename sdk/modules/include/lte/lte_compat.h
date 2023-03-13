/****************************************************************************
 * modules/include/lte/lte_compat.h
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_INCLUDE_LTE_LTE_COMPAT_H
#define __MODULES_INCLUDE_LTE_LTE_COMPAT_H

/* This header file is provided for applications that use the following
 * deprecated definitions. Basically, the correct approach is to replace
 * the parts of the application that use the deprecated definitions
 * without using this header file.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <lte/lte_api.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internet protocol type: IPv4
 * Use LTE_IPTYPE_V4 instead.
 */

#define LTE_APN_IPTYPE_IP          LTE_IPTYPE_V4

/* Internet protocol type: IPv6
 * Use LTE_IPTYPE_V6 instead.
 */

#define LTE_APN_IPTYPE_IPV6        LTE_IPTYPE_V6

/* Internet protocol type: IPv4/v6
 * Use LTE_IPTYPE_V4V6 instead.
 */

#define LTE_APN_IPTYPE_IPV4V6      LTE_IPTYPE_V4V6

/* Enable setting of PIN lock
 * Use LTE_ENABLE instead.
 */

#define LTE_PIN_ENABLE             LTE_ENABLE

/* Disable setting of PIN lock
 * Use LTE_DISABLE instead.
 */

#define LTE_PIN_DISABLE            LTE_DISABLE

/* Digit number of mcc
 * Use LTE_MCC_DIGIT instead.
 */

#define LTE_CELLINFO_MCC_DIGIT     LTE_MCC_DIGIT

/* Max digit number of mnc
 * Use LTE_MNC_DIGIT_MAX instead.
 */

#define LTE_CELLINFO_MNC_DIGIT_MAX LTE_MNC_DIGIT_MAX

/* Digit number of mcc
 * Use LTE_MCC_DIGIT instead.
 */

#define LTE_SIMINFO_MCC_DIGIT      LTE_MCC_DIGIT

/* Max digit number of mnc
 * Use LTE_MNC_DIGIT_MAX instead.
 */

#define LTE_SIMINFO_MNC_DIGIT_MAX  LTE_MNC_DIGIT_MAX

#endif /* __MODULES_INCLUDE_LTE_LTE_COMPAT_H */
