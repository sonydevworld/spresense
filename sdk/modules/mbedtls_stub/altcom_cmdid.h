/****************************************************************************
 * modules/mbedtls_stub/altcom_cmdid.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_MBEDTLS_STUB_ALTCOM_CMDID_H
#define __MODULES_MBEDTLS_STUB_ALTCOM_CMDID_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMDID_UNKNOWN                        (0x0000)

/* MBEDTLS ALTCOM commands */

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
#define APICMDID_TLS_SSL_CIPHERSUITE            (0x010a)
#define APICMDID_TLS_SSL_CIPHERSUITE_ID         (0x010b)
#define APICMDID_TLS_SSL_RECORD_EXPANSION       (0x010c)
#define APICMDID_TLS_SSL_VERIFY_RESULT          (0x010d)
#define APICMDID_TLS_SSL_TIMER_CB               (0x010e)
#define APICMDID_TLS_SSL_PEER_CERT              (0x010f)
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
#define APICMDID_TLS_CONFIG_ALPN_PROTOCOLS      (0x012a)
#define APICMDID_TLS_CONFIG_CIPHERSUITES        (0x012b)
#define APICMDID_TLS_SESSION_INIT               (0x0140)
#define APICMDID_TLS_SESSION_FREE               (0x0141)
#define APICMDID_TLS_SESSION_GET                (0x0142)
#define APICMDID_TLS_SESSION_SET                (0x0143)
#define APICMDID_TLS_SESSION_RESET              (0x0144)
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
#define APICMDID_TLS_MD_INFO_FROM_TYPE          (0x01a0)
#define APICMDID_TLS_MD_GET_SIZE                (0x01a1)
#define APICMDID_TLS_MD                         (0x01a2)
#define APICMDID_TLS_MD_DIGEST                  (0x01a3)
#define APICMDID_TLS_BASE64_ENCODE              (0x01b0)
#define APICMDID_TLS_SHA1                       (0x01b1)
#define APICMDID_TLS_SSL_EXPORT_SRTP_KEYS       (0x01e0)
#define APICMDID_TLS_SSL_USE_SRTP               (0x01e1)
#define APICMDID_TLS_SSL_SRTP_PROFILE           (0x01e2)
#define APICMDID_TLS_SSL_TURN                   (0x01f0)
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
#define APICMDID_TLS_X509WRITE_CRT_SERIAL       (0x023a)
#define APICMDID_TLS_X509WRITE_CRT_VALIDITY     (0x023b)
#define APICMDID_TLS_X509WRITE_CRT_CONSTRAINTS  (0x023c)
#define APICMDID_TLS_X509WRITE_CRT_SUBJECT_ID   (0x023d)
#define APICMDID_TLS_X509WRITE_CRT_AUTHORITY_ID (0x023e)
#define APICMDID_TLS_X509WRITE_CRT_KEY_USAGE    (0x023f)
#define APICMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE (0x0240)
#define APICMDID_TLS_RSA_INIT                   (0x0250)
#define APICMDID_TLS_RSA_FREE                   (0x0251)
#define APICMDID_TLS_RSA_GEN_KEY                (0x0252)

#define APICMDID_TLS_SSL_CMD                    (0x0260)
#define APICMDID_TLS_CONFIG_CMD                 (0x0261)
#define APICMDID_TLS_SESSION_CMD                (0x0262)
#define APICMDID_TLS_X509_CRT_CMD               (0x0263)
#define APICMDID_TLS_PK_CMD                     (0x0264)
#define APICMDID_TLS_CTR_DRBG_CMD               (0x0265)
#define APICMDID_TLS_ENTROPY_CMD                (0x0266)
#define APICMDID_TLS_CIPHER_CMD                 (0x0267)

/* MBEDTLS ALTCOM commands (protocol version 4) */

#define APICMDID_TLS_SSL_CMD_V4                 (0x0140)
#define APICMDID_TLS_CONFIG_CMD_V4              (0x0160)
#define APICMDID_TLS_CONFIG_VERIFY_CALLBACK_V4  (0x0161)
#define APICMDID_TLS_SESSION_CMD_V4             (0x0170)
#define APICMDID_TLS_X509_CRT_CMD_V4            (0x0180)
#define APICMDID_TLS_X509_CRT_INFO_V4           (0x0181)
#define APICMDID_TLS_PK_CMD_V4                  (0x0190)
#define APICMDID_TLS_CTR_DRBG_CMD_V4            (0x01a0)
#define APICMDID_TLS_ENTROPY_CMD_V4             (0x01b0)
#define APICMDID_TLS_CIPHER_CMD_V4              (0x01c0)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint16_t convert_cid2v1(uint16_t cid)
{
  uint16_t ret = APICMDID_UNKNOWN;

 if (cid == APICMDID_TLS_SSL_CMD_V4)
    {
      ret = APICMDID_TLS_SSL_CMD;
    }
  else if (cid == APICMDID_TLS_CONFIG_CMD_V4)
    {
      ret = APICMDID_TLS_CONFIG_CMD;
    }
  else if (cid == APICMDID_TLS_CONFIG_VERIFY_CALLBACK_V4)
    {
      ret = APICMDID_TLS_CONFIG_VERIFY_CALLBACK;
    }
  else if (cid == APICMDID_TLS_SESSION_CMD_V4)
    {
      ret = APICMDID_TLS_SESSION_CMD;
    }
  else if (cid == APICMDID_TLS_X509_CRT_CMD_V4)
    {
      ret = APICMDID_TLS_X509_CRT_CMD;
    }
  else if (cid == APICMDID_TLS_X509_CRT_INFO_V4)
    {
      ret = APICMDID_TLS_X509_CRT_INFO;
    }
  else if (cid == APICMDID_TLS_PK_CMD_V4)
    {
      ret = APICMDID_TLS_PK_CMD;
    }
  else if (cid == APICMDID_TLS_CTR_DRBG_CMD_V4)
    {
      ret = APICMDID_TLS_CTR_DRBG_CMD;
    }
  else if (cid == APICMDID_TLS_ENTROPY_CMD_V4)
    {
      ret = APICMDID_TLS_ENTROPY_CMD;
    }
  else if (cid == APICMDID_TLS_CIPHER_CMD_V4)
    {
      ret = APICMDID_TLS_CIPHER_CMD;
    }

  return ret;
}

#endif /* __MODULES_MBEDTLS_STUB_ALTCOM_CMDID_H */
