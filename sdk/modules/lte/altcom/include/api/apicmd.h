/****************************************************************************
 * modules/lte/altcom/include/api/apicmd.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_APICMD_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_APICMD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Definition of magic number field */

#define APICMD_MAGICNUMBER              (0xFEEDBAC5)

/* Definition of version field */

#define APICMD_VER                      (0x01)

/* Definitions of command id field */

/* LTE API commands */

#define APICMDID_POWER_ON               (0x0001)
#define APICMDID_ATTACH_NET             (0x0002)
#define APICMDID_DETACH_NET             (0x0003)
#define APICMDID_GET_NETSTAT            (0x0004)
#define APICMDID_DATAON                 (0x0005)
#define APICMDID_DATAOFF                (0x0006)
#define APICMDID_GET_DATASTAT           (0x0007)
#define APICMDID_GET_DATACONFIG         (0x0008)
#define APICMDID_SET_DATACONFIG         (0x0009)
#define APICMDID_GET_APNSET             (0x000A)
#define APICMDID_SET_APN                (0x000B)
#define APICMDID_GET_VERSION            (0x000C)
#define APICMDID_GET_PHONENO            (0x000D)
#define APICMDID_GET_IMSI               (0x000E)
#define APICMDID_GET_IMEI               (0x000F)
#define APICMDID_GET_PINSET             (0x0010)
#define APICMDID_SET_PIN_LOCK           (0x0011)
#define APICMDID_SET_PIN_CODE           (0x0012)
#define APICMDID_ENTER_PIN              (0x0013)
#define APICMDID_GET_LTIME              (0x0014)
#define APICMDID_GET_OPERATOR           (0x0015)
#define APICMDID_GET_SLPMODESET         (0x0016)
#define APICMDID_SET_SLPMODESET         (0x0017)
#define APICMDID_SET_REP_NETSTAT        (0x0018)
#define APICMDID_SET_REP_EVT            (0x0019)
#define APICMDID_SET_REP_QUALITY        (0x001A)
#define APICMDID_SET_REP_CELLINFO       (0x001B)
#define APICMDID_REPORT_NETSTAT         (0x001C)
#define APICMDID_REPORT_EVT             (0x001D)
#define APICMDID_REPORT_QUALITY         (0x001E)
#define APICMDID_REPORT_CELLINFO        (0x001F)
#define APICMDID_GET_EDRX               (0x0020)
#define APICMDID_SET_EDRX               (0x0021)
#define APICMDID_GET_PSM                (0x0022)
#define APICMDID_SET_PSM                (0x0023)
#define APICMDID_GET_CE                 (0x0024)
#define APICMDID_SET_CE                 (0x0025)

/* SOCKET API commands */

#define APICMDID_SOCK_ACCEPT            (0x0080)
#define APICMDID_SOCK_BIND              (0x0081)
#define APICMDID_SOCK_CLOSE             (0x0082)
#define APICMDID_SOCK_CONNECT           (0x0083)
#define APICMDID_SOCK_FCNTL             (0x0084)
#define APICMDID_SOCK_GETADDRINFO       (0x0085)
#define APICMDID_SOCK_GETHOSTBYNAME     (0x0086)
#define APICMDID_SOCK_GETHOSTBYNAMER    (0x0087)
#define APICMDID_SOCK_GETSOCKNAME       (0x0088)
#define APICMDID_SOCK_GETSOCKOPT        (0x0089)
#define APICMDID_SOCK_LISTEN            (0x008A)
#define APICMDID_SOCK_RECV              (0x008B)
#define APICMDID_SOCK_RECVFROM          (0x008C)
#define APICMDID_SOCK_SELECT            (0x008D)
#define APICMDID_SOCK_SEND              (0x008E)
#define APICMDID_SOCK_SENDTO            (0x008F)
#define APICMDID_SOCK_SHUTDOWN          (0x0090)
#define APICMDID_SOCK_SOCKET            (0x0091)
#define APICMDID_SOCK_SETSOCKOPT        (0x0092)
#define APICMDID_ERRIND                 (0xFFFF)

#define APICMDID_MAX                    APICMDID_SOCK_SETSOCKOPT

/* In the case of a response, set 15th bit of the command ID. */

#define APICMDID_CONVERT_RES(cmdid)     (cmdid | 0x8000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * API command Header Format
 * bits    0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
 *         ----------------------------------------------
 *         |            magic number (Higher)           |
 *         ----------------------------------------------
 *         |            magic number (Lower)            |
 *         ----------------------------------------------
 *         |       version       |      sequence id     |
 *         ----------------------------------------------
 *         |                 command id                 |
 *         ----------------------------------------------
 *         |               transaction id               |
 *         ----------------------------------------------
 *         |                 data length                |
 *         ----------------------------------------------
 *         |                  check sum                 |
 *         ----------------------------------------------
 *         |                   reserve                  |
 *         ----------------------------------------------
 ****************************************************************************/

begin_packed_struct struct apicmd_cmdhdr_s
{
  uint32_t magic;
  uint8_t  ver;
  uint8_t  seqid;
  uint16_t cmdid;
  uint16_t transid;
  uint16_t dtlen;
  uint16_t chksum;
  uint16_t reserve;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_APICMD_H */
