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
#define APICMDID_RADIO_ON               (0x0026)
#define APICMDID_RADIO_OFF              (0x0027)
#define APICMDID_ACTIVATE_PDN           (0x0028)
#define APICMDID_DEACTIVATE_PDN         (0x0029)
#define APICMDID_DATA_ALLOW             (0x002A)
#define APICMDID_GET_NETINFO            (0x002B)
#define APICMDID_GET_IMS_CAP            (0x002C)
#define APICMDID_SETREP_NETINFO         (0x002D)
#define APICMDID_REPORT_NETINFO         (0x002E)
#define APICMDID_REPORT_RESTART         (0x002F)
#define APICMDID_ERRINFO                (0x0030)
#define APICMDID_SET_REP_EVT_LTIME      (0x0031)
#define APICMDID_SET_REP_EVT_SIMSTATE   (0x0032)
#define APICMDID_POWER_OFF              (0x0033)
#define APICMDID_GET_SIMINFO            (0x0034)
#define APICMDID_GET_DYNAMICEDRX        (0x0035)
#define APICMDID_GET_DYNAMICPSM         (0x0036)
#define APICMDID_GET_QUALITY            (0x0037)
#define APICMDID_ACTIVATE_PDN_CANCEL    (0x0038)
#define APICMDID_GET_CELLINFO           (0x0039)
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

/* MBEDTLS API commands */

#define APICMDID_TLS_SSL_INIT                   (0x0100)
#define APICMDID_TLS_SSL_FREE                   (0x0101)
#define APICMDID_TLS_SSL_SETUP                  (0x0102)
#define APICMDID_TLS_SSL_HOSTNAME               (0x0103)
#define APICMDID_TLS_SSL_BIO                    (0x0104)
#define APICMDID_TLS_SSL_HANDSHAKE              (0x0105)
#define APICMDID_TLS_SSL_WRITE                  (0x0106)
#define APICMDID_TLS_SSL_READ                   (0x0107)
#define APICMDID_TLS_SSL_CLOSE_NOTIFY           (0x0108)
#define APICMDID_TLS_SSL_VERSION                (0x0109)
#define APICMDID_TLS_SSL_CIPHERSUITE            (0x010A)
#define APICMDID_TLS_SSL_CIPHERSUITE_ID         (0x010B)
#define APICMDID_TLS_SSL_RECORD_EXPANSION       (0x010C)
#define APICMDID_TLS_SSL_VERIFY_RESULT          (0x010D)
#define APICMDID_TLS_SSL_TIMER_CB               (0x010E)
#define APICMDID_TLS_SSL_PEER_CERT              (0x010F)
#define APICMDID_TLS_SSL_BYTES_AVAIL            (0x0110)
#define APICMDID_TLS_CONFIG_INIT                (0x0120)
#define APICMDID_TLS_CONFIG_FREE                (0x0121)
#define APICMDID_TLS_CONFIG_DEFAULTS            (0x0112)
#define APICMDID_TLS_CONFIG_AUTHMODE            (0x0123)
#define APICMDID_TLS_CONFIG_RNG                 (0x0124)
#define APICMDID_TLS_CONFIG_CA_CHAIN            (0x0125)
#define APICMDID_TLS_CONFIG_OWN_CERT            (0x0126)
#define APICMDID_TLS_CONFIG_READ_TIMEOUT        (0x0127)
#define APICMDID_TLS_CONFIG_VERIFY              (0x0128)
#define APICMDID_TLS_CONFIG_VERIFY_CALLBACK     (0x0129)
#define APICMDID_TLS_CONFIG_ALPN_PROTOCOLS      (0x012A)
#define APICMDID_TLS_CONFIG_CIPHERSUITES        (0x012B)
#define APICMDID_TLS_SESSION_INIT               (0x0140)
#define APICMDID_TLS_SESSION_FREE               (0x0141)
#define APICMDID_TLS_SESSION_GET                (0x0142)
#define APICMDID_TLS_SESSION_SET                (0x0143)
#define APICMDID_TLS_X509_CRT_INIT              (0x0150)
#define APICMDID_TLS_X509_CRT_FREE              (0x0151)
#define APICMDID_TLS_X509_CRT_PARSE_FILE        (0x0152)
#define APICMDID_TLS_X509_CRT_PARSE_DER         (0x0153)
#define APICMDID_TLS_X509_CRT_PARSE             (0x0154)
#define APICMDID_TLS_X509_CRT_INFO              (0x0155)
#define APICMDID_TLS_X509_CRT_VERIFY_INFO       (0x0156)
#define APICMDID_TLS_PK_INIT                    (0x0160)
#define APICMDID_TLS_PK_FREE                    (0x0161)
#define APICMDID_TLS_PK_PARSE_KEYFILE           (0x0162)
#define APICMDID_TLS_PK_PARSE_KEY               (0x0163)
#define APICMDID_TLS_PK_CHECK_PAIR              (0x0164)
#define APICMDID_TLS_PK_SETUP                   (0x0165)
#define APICMDID_TLS_PK_INFO_FROM_TYPE          (0x0166)
#define APICMDID_TLS_PK_WRITE_KEY_PEM           (0x0167)
#define APICMDID_TLS_PK_WRITE_KEY_DER           (0x0168)
#define APICMDID_TLS_PK_RSA                     (0x0169)
#define APICMDID_TLS_CTR_DRBG_INIT              (0x0170)
#define APICMDID_TLS_CTR_DRBG_FREE              (0x0171)
#define APICMDID_TLS_CTR_DRBG_SEED              (0x0172)
#define APICMDID_TLS_ENTROPY_INIT               (0x0180)
#define APICMDID_TLS_ENTROPY_FREE               (0x0181)
#define APICMDID_TLS_CIPHER_INIT                (0x0190)
#define APICMDID_TLS_CIPHER_FREE                (0x0191)
#define APICMDID_TLS_CIPHER_INFO_FROM_STRING    (0x0192)
#define APICMDID_TLS_CIPHER_SETUP               (0x0193)
#define APICMDID_TLS_CIPHER_SETKEY              (0x0194)
#define APICMDID_TLS_CIPHER_SET_IV              (0x0195)
#define APICMDID_TLS_CIPHER_UPDATE              (0x0196)
#define APICMDID_TLS_MD_INFO_FROM_TYPE          (0x01A0)
#define APICMDID_TLS_MD_GET_SIZE                (0x01A1)
#define APICMDID_TLS_MD                         (0x01A2)
#define APICMDID_TLS_MD_DIGEST                  (0x01A3)
#define APICMDID_TLS_BASE64_ENCODE              (0x01B0)
#define APICMDID_TLS_SHA1                       (0x01B1)
#define APICMDID_TLS_SSL_EXPORT_SRTP_KEYS       (0x01E0)
#define APICMDID_TLS_SSL_USE_SRTP               (0x01E1)
#define APICMDID_TLS_SSL_SRTP_PROFILE           (0x01E2)
#define APICMDID_TLS_SSL_TURN                   (0x01F0)
#define APICMDID_TLS_MPI_INIT                   (0x0200)
#define APICMDID_TLS_MPI_FREE                   (0x0201)
#define APICMDID_TLS_MPI_READ_STRING            (0x0202)
#define APICMDID_TLS_MPI_WRITE_STRING           (0x0203)
#define APICMDID_TLS_X509_CSR_INIT              (0x0210)
#define APICMDID_TLS_X509_CSR_FREE              (0x0211)
#define APICMDID_TLS_X509_CSR_PARSE_FILE        (0x0212)
#define APICMDID_TLS_X509_CSR_PARSE_DER         (0x0213)
#define APICMDID_TLS_X509_CSR_PARSE             (0x0214)
#define APICMDID_TLS_X509_DN_GETS_CRT           (0x0220)
#define APICMDID_TLS_X509_DN_GETS_CSR           (0x0221)
#define APICMDID_TLS_X509WRITE_CRT_INIT         (0x0230)
#define APICMDID_TLS_X509WRITE_CRT_FREE         (0x0231)
#define APICMDID_TLS_X509WRITE_CRT_DER          (0x0232)
#define APICMDID_TLS_X509WRITE_CRT_PEM          (0x0233)
#define APICMDID_TLS_X509WRITE_CRT_SUBJECT_KEY  (0x0234)
#define APICMDID_TLS_X509WRITE_CRT_ISSUER_KEY   (0x0235)
#define APICMDID_TLS_X509WRITE_CRT_SUBJECT_NAME (0x0236)
#define APICMDID_TLS_X509WRITE_CRT_ISSUER_NAME  (0x0237)
#define APICMDID_TLS_X509WRITE_CRT_VERSION      (0x0238)
#define APICMDID_TLS_X509WRITE_CRT_MD_ALG       (0x0239)
#define APICMDID_TLS_X509WRITE_CRT_SERIAL       (0x023A)
#define APICMDID_TLS_X509WRITE_CRT_VALIDITY     (0x023B)
#define APICMDID_TLS_X509WRITE_CRT_CONSTRAINTS  (0x023C)
#define APICMDID_TLS_X509WRITE_CRT_SUBJECT_ID   (0x023D)
#define APICMDID_TLS_X509WRITE_CRT_AUTHORITY_ID (0x023E)
#define APICMDID_TLS_X509WRITE_CRT_KEY_USAGE    (0x023F)
#define APICMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE (0x0240)
#define APICMDID_TLS_RSA_INIT                   (0x0250)
#define APICMDID_TLS_RSA_FREE                   (0x0251)
#define APICMDID_TLS_RSA_GEN_KEY                (0x0252)

/* SMS API commands */

#define APICMDID_SMS_INIT                       (0x0500)
#define APICMDID_SMS_FIN                        (0x0501)
#define APICMDID_SMS_SEND                       (0x0502)
#define APICMDID_SMS_REPORT_RECV                (0x0503)
#define APICMDID_SMS_DELETE                     (0x0504)
#define APICMDID_SMS_GET_STGEINFO               (0x0505)
#define APICMDID_SMS_GET_LIST                   (0x0506)
#define APICMDID_SMS_READ                       (0x0507)

#define APICMDID_ERRIND                 (0xFFFF)

#define APICMDID_MAX                    APICMDID_SMS_READ

/* In the case of a response, set 15th bit of the command ID. */

#define APICMDID_CONVERT_RES(cmdid)     (cmdid | 0x8000)

#define APICMD_PAYLOAD_SIZE_MAX         (4112)

/* API command header options field */

#define APICMD_OPT_DATA_CHKSUM_EN_SIFT  (0)
#define APICMD_OPT_DATA_CHKSUM_EN_MASK  (1 << APICMD_OPT_DATA_CHKSUM_EN_SIFT)
#define APICMD_OPT_DATA_CHKSUM_ENABLE   (1 << APICMD_OPT_DATA_CHKSUM_EN_SIFT)
#define APICMD_OPT_DATA_CHKSUM_DISABLE  (0 << APICMD_OPT_DATA_CHKSUM_EN_SIFT)

#define APICMD_OPT_DATA_CHKSUM_ENABLED(opt) (((opt) & APICMD_OPT_DATA_CHKSUM_EN_MASK) == APICMD_OPT_DATA_CHKSUM_ENABLE)

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
 *         |                  options                   |
 *         ----------------------------------------------
 *         |              check sum(header part)        |
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
  uint16_t options;
  uint16_t chksum;
} end_packed_struct;

/****************************************************************************
 * API command Footer Format
 * bits    0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
 *         ----------------------------------------------
 *         |                  reserve                   |
 *         ----------------------------------------------
 *         |        check sum(data + footer part)       |
 *         ----------------------------------------------
 ****************************************************************************/

begin_packed_struct struct apicmd_cmdftr_s
{
  uint16_t reserve;
  uint16_t chksum;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_APICMD_H */
