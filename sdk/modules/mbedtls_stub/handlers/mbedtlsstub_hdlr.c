/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr.c
 *
 *   Copyright 2021,2022 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <nuttx/modem/alt1250.h>

#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFF_CLEAR_SIZE        16

#define MBEDTLS_INVALID_CTX_ID (0xFFFFFFFF)
#define MBEDTLS_MINIMUM_CTX_ID (0x00000001)
#define MBEDTLS_MAXIMUM_CTX_ID (0x00FFFFFF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct compose_inst_s
{
  uint32_t cmdid;
  compose_handler_t hdlr;
} compose_inst_t;

typedef struct parse_inst_s
{
  uint32_t altcid;
  parse_handler_t hdlr;
} parse_inst_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_mbedtls_ctx_id_tbl[MBEDTLSSTUB_SSL_CTX_MAX] =
{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };


static compose_inst_t g_composehdlrs[] =
{
  {LTE_CMDID_TLS_SSL_INIT, mbedtlsstub_sslinit_pkt_compose},
  {LTE_CMDID_TLS_SSL_FREE, mbedtlsstub_sslfree_pkt_compose},
  {LTE_CMDID_TLS_SSL_SETUP, mbedtlsstub_sslsetup_pkt_compose},
  {LTE_CMDID_TLS_SSL_HOSTNAME, mbedtlsstub_sslhostname_pkt_compose},
  {LTE_CMDID_TLS_SSL_BIO, mbedtlsstub_sslbio_pkt_compose},
  {LTE_CMDID_TLS_SSL_HANDSHAKE, mbedtlsstub_sslhandshake_pkt_compose},
  {LTE_CMDID_TLS_SSL_WRITE, mbedtlsstub_sslwrite_pkt_compose},
  {LTE_CMDID_TLS_SSL_READ, mbedtlsstub_sslread_pkt_compose},
  {LTE_CMDID_TLS_SSL_CLOSE_NOTIFY, mbedtlsstub_sslclosenotif_pkt_compose},
  {LTE_CMDID_TLS_SSL_VERSION, mbedtlsstub_sslver_pkt_compose},
  {LTE_CMDID_TLS_SSL_CIPHERSUITE, mbedtlsstub_sslciphersuite_pkt_compose},
  {LTE_CMDID_TLS_SSL_CIPHERSUITE_ID, mbedtlsstub_sslciphersuiteid_pkt_compose},
  {LTE_CMDID_TLS_SSL_RECORD_EXPANSION, mbedtlsstub_sslrecexp_pkt_compose},
  {LTE_CMDID_TLS_SSL_VERIFY_RESULT, mbedtlsstub_sslvrfyresult_pkt_compose},
  {LTE_CMDID_TLS_SSL_TIMER_CB, mbedtlsstub_ssltimercb_pkt_compose},
  {LTE_CMDID_TLS_SSL_PEER_CERT, mbedtlsstub_sslpeercert_pkt_compose},
  {LTE_CMDID_TLS_SSL_BYTES_AVAIL, mbedtlsstub_sslbytesavail_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_INIT, mbedtlsstub_confinit_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_FREE, mbedtlsstub_conffree_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_DEFAULTS, mbedtlsstub_confdefauts_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_AUTHMODE, mbedtlsstub_confauth_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_RNG, mbedtlsstub_confrng_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_CA_CHAIN, mbedtlsstub_confcachain_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_OWN_CERT, mbedtlsstub_confowncert_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_READ_TIMEOUT, mbedtlsstub_confreadtimeo_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_VERIFY, mbedtlsstub_confvrfy_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_VERIFY_CALLBACK, mbedtlsstub_confvrfycb_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_ALPN_PROTOCOLS, mbedtlsstub_confalpnproto_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_CIPHERSUITES, mbedtlsstub_confciphersuites_pkt_compose},
  {LTE_CMDID_TLS_SESSION_INIT, mbedtlsstub_ssesioninit_pkt_compose},
  {LTE_CMDID_TLS_SESSION_FREE, mbedtlsstub_sessionfree_pkt_compose},
  {LTE_CMDID_TLS_SESSION_GET, mbedtlsstub_sessionget_pkt_compose},
  {LTE_CMDID_TLS_SESSION_SET, mbedtlsstub_sessionset_pkt_compose},
  {LTE_CMDID_TLS_SESSION_RESET, mbedtlsstub_sessionreset_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_INIT, mbedtlsstub_x509crtinit_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_FREE, mbedtlsstub_x509crtfree_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_PARSE_FILE, mbedtlsstub_x509crtparsefile_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_PARSE_DER, mbedtlsstub_x509crtparseder_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_PARSE, mbedtlsstub_x509crtparse_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_INFO, mbedtlsstub_x509crtinfo_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_VERIFY_INFO, mbedtlsstub_x509crtvrfyinfo_pkt_compose},
  {LTE_CMDID_TLS_PK_INIT, mbedtlsstub_pkinit_pkt_compose},
  {LTE_CMDID_TLS_PK_FREE, mbedtlsstub_pkfree_pkt_compose},
  {LTE_CMDID_TLS_PK_PARSE_KEYFILE, mbedtlsstub_pkparsekeyfile_pkt_compose},
  {LTE_CMDID_TLS_PK_PARSE_KEY, mbedtlsstub_pkparsekey_pkt_compose},
  {LTE_CMDID_TLS_PK_CHECK_PAIR, mbedtlsstub_pkcheckpair_pkt_compose},
  {LTE_CMDID_TLS_PK_SETUP, mbedtlsstub_pksetup_pkt_compose},
  {LTE_CMDID_TLS_PK_INFO_FROM_TYPE, mbedtlsstub_pkinfofromtype_pkt_compose},
  {LTE_CMDID_TLS_PK_WRITE_KEY_PEM, mbedtlsstub_pkwritekeypem_pkt_compose},
  {LTE_CMDID_TLS_PK_WRITE_KEY_DER, mbedtlsstub_pkwritekeyder_pkt_compose},
  {LTE_CMDID_TLS_PK_RSA, mbedtlsstub_pkrsa_pkt_compose},
  {LTE_CMDID_TLS_CTR_DRBG_INIT, mbedtlsstub_ctrdrbginit_pkt_compose},
  {LTE_CMDID_TLS_CTR_DRBG_FREE, mbedtlsstub_ctrdrbgfree_pkt_compose},
  {LTE_CMDID_TLS_CTR_DRBG_SEED, mbedtlsstub_ctrdrbgseed_pkt_compose},
  {LTE_CMDID_TLS_ENTROPY_INIT, mbedtlsstub_entropyinit_pkt_compose},
  {LTE_CMDID_TLS_ENTROPY_FREE, mbedtlsstub_entropyfree_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_INIT, mbedtlsstub_cipherinit_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_FREE, mbedtlsstub_cipherfree_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_INFO_FROM_STRING, mbedtlsstub_cipherinfofromstr_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_SETUP, mbedtlsstub_ciphersetup_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_SETKEY, mbedtlsstub_ciphersetkey_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_SET_IV, mbedtlsstub_ciphersetiv_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_UPDATE, mbedtlsstub_cipherupdate_pkt_compose},
  {LTE_CMDID_TLS_MD_INFO_FROM_TYPE, mbedtlsstub_mdinfofromtype_pkt_compose},
  {LTE_CMDID_TLS_MD_GET_SIZE, mbedtlsstub_mdgetsize_pkt_compose},
  {LTE_CMDID_TLS_MD, mbedtlsstub_md_pkt_compose},
  {LTE_CMDID_TLS_MD_DIGEST, mbedtlsstub_mddigest_pkt_compose},
  {LTE_CMDID_TLS_BASE64_ENCODE, mbedtlsstub_base64enc_pkt_compose},
  {LTE_CMDID_TLS_SHA1, mbedtlsstub_sha1_pkt_compose},
  {LTE_CMDID_TLS_SSL_EXPORT_SRTP_KEYS, mbedtlsstub_sslexportsrtpkeys_pkt_compose},
  {LTE_CMDID_TLS_SSL_USE_SRTP, mbedtlsstub_sslusesrtp_pkt_compose},
  {LTE_CMDID_TLS_SSL_SRTP_PROFILE, mbedtlsstub_srtpprofile_pkt_compose},
  {LTE_CMDID_TLS_SSL_TURN, mbedtlsstub_sslturn_pkt_compose},
  {LTE_CMDID_TLS_MPI_INIT, mbedtlsstub_mpiinit_pkt_compose},
  {LTE_CMDID_TLS_MPI_FREE, mbedtlsstub_mpifree_pkt_compose},
  {LTE_CMDID_TLS_MPI_READ_STRING, mbedtlsstub_mpireadstr_pkt_compose},
  {LTE_CMDID_TLS_MPI_WRITE_STRING, mbedtlsstub_mpiwritestr_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_INIT, mbedtlsstub_x509csrinit_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_FREE, mbedtlsstub_x509csrfree_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_PARSE_FILE, mbedtlsstub_x509csrparsefile_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_PARSE_DER, mbedtlsstub_x509csrparseder_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_PARSE, mbedtlsstub_x509csrparse_pkt_compose},
  {LTE_CMDID_TLS_X509_DN_GETS_CRT, mbedtlsstub_x509dngetscrt_pkt_compose},
  {LTE_CMDID_TLS_X509_DN_GETS_CSR, mbedtlsstub_x509dngetscsr_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_INIT, mbedtlsstub_x509writecrtinit_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_FREE, mbedtlsstub_x509writecrtfree_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_DER, mbedtlsstub_x509writecrtder_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_PEM, mbedtlsstub_x509writecrtpem_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_KEY, mbedtlsstub_x509writecrtsubkey_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_KEY, mbedtlsstub_x509writecrtissuerkey_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_NAME, mbedtlsstub_x509writecrtsubname_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_NAME, mbedtlsstub_x509writecrtissuername_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_VERSION, mbedtlsstub_x509writecrtver_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_MD_ALG, mbedtlsstub_x509writecrtmdalg_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SERIAL, mbedtlsstub_x509writecrtserial_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_VALIDITY, mbedtlsstub_x509writecrtvalidity_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_CONSTRAINTS, mbedtlsstub_x509writecrtconst_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_ID, mbedtlsstub_x509writecrtsubid_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_AUTHORITY_ID, mbedtlsstub_x509writecrtauthid_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_KEY_USAGE, mbedtlsstub_x509writecrtkeyusage_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE, mbedtlsstub_x509writecrtnscerttype_pkt_compose},
  {LTE_CMDID_TLS_RSA_INIT, mbedtlsstub_rsainit_pkt_compose},
  {LTE_CMDID_TLS_RSA_FREE, mbedtlsstub_rsafree_pkt_compose},
  {LTE_CMDID_TLS_RSA_GEN_KEY, mbedtlsstub_rsagenkey_pkt_compose},
};

static parse_inst_t g_parsehdlrs[] =
{
  {APICMDID_TLS_SSL_INIT, mbedtlsstub_sslinit_pkt_parse},
  {APICMDID_TLS_SSL_FREE, mbedtlsstub_sslfree_pkt_parse},
  {APICMDID_TLS_SSL_SETUP, mbedtlsstub_sslsetup_pkt_parse},
  {APICMDID_TLS_SSL_HOSTNAME, mbedtlsstub_sslhostname_pkt_parse},
  {APICMDID_TLS_SSL_BIO, mbedtlsstub_sslbio_pkt_parse},
  {APICMDID_TLS_SSL_HANDSHAKE, mbedtlsstub_sslhandshake_pkt_parse},
  {APICMDID_TLS_SSL_WRITE, mbedtlsstub_sslwrite_pkt_parse},
  {APICMDID_TLS_SSL_READ, mbedtlsstub_sslread_pkt_parse},
  {APICMDID_TLS_SSL_CLOSE_NOTIFY, mbedtlsstub_sslclosenotif_pkt_parse},
  {APICMDID_TLS_SSL_VERSION, mbedtlsstub_sslver_pkt_parse},
  {APICMDID_TLS_SSL_CIPHERSUITE, mbedtlsstub_sslciphersuite_pkt_parse},
  {APICMDID_TLS_SSL_CIPHERSUITE_ID, mbedtlsstub_sslciphersuiteid_pkt_parse},
  {APICMDID_TLS_SSL_RECORD_EXPANSION, mbedtlsstub_sslrecexp_pkt_parse},
  {APICMDID_TLS_SSL_VERIFY_RESULT, mbedtlsstub_sslvrfyresult_pkt_parse},
  {APICMDID_TLS_SSL_TIMER_CB, mbedtlsstub_ssltimercb_pkt_parse},
  {APICMDID_TLS_SSL_PEER_CERT, mbedtlsstub_sslpeercert_pkt_parse},
  {APICMDID_TLS_SSL_BYTES_AVAIL, mbedtlsstub_sslbytesavail_pkt_parse},
  {APICMDID_TLS_CONFIG_INIT, mbedtlsstub_confinit_pkt_parse},
  {APICMDID_TLS_CONFIG_FREE, mbedtlsstub_conffree_pkt_parse},
  {APICMDID_TLS_CONFIG_DEFAULTS, mbedtlsstub_confdefauts_pkt_parse},
  {APICMDID_TLS_CONFIG_AUTHMODE, mbedtlsstub_confauth_pkt_parse},
  {APICMDID_TLS_CONFIG_RNG, mbedtlsstub_confrng_pkt_parse},
  {APICMDID_TLS_CONFIG_CA_CHAIN, mbedtlsstub_confcachain_pkt_parse},
  {APICMDID_TLS_CONFIG_OWN_CERT, mbedtlsstub_confowncert_pkt_parse},
  {APICMDID_TLS_CONFIG_READ_TIMEOUT, mbedtlsstub_confreadtimeo_pkt_parse},
  {APICMDID_TLS_CONFIG_VERIFY, mbedtlsstub_confvrfy_pkt_parse},
  {APICMDID_TLS_CONFIG_VERIFY_CALLBACK, mbedtlsstub_confvrfycb_pkt_parse},
  {APICMDID_TLS_CONFIG_ALPN_PROTOCOLS, mbedtlsstub_confalpnproto_pkt_parse},
  {APICMDID_TLS_CONFIG_CIPHERSUITES, mbedtlsstub_confciphersuites_pkt_parse},
  {APICMDID_TLS_SESSION_INIT, mbedtlsstub_ssesioninit_pkt_parse},
  {APICMDID_TLS_SESSION_FREE, mbedtlsstub_sessionfree_pkt_parse},
  {APICMDID_TLS_SESSION_GET, mbedtlsstub_sessionget_pkt_parse},
  {APICMDID_TLS_SESSION_SET, mbedtlsstub_sessionset_pkt_parse},
  {APICMDID_TLS_X509_CRT_INIT, mbedtlsstub_x509crtinit_pkt_parse},
  {APICMDID_TLS_X509_CRT_FREE, mbedtlsstub_x509crtfree_pkt_parse},
  {APICMDID_TLS_X509_CRT_PARSE_FILE, mbedtlsstub_x509crtparsefile_pkt_parse},
  {APICMDID_TLS_X509_CRT_PARSE_DER, mbedtlsstub_x509crtparseder_pkt_parse},
  {APICMDID_TLS_X509_CRT_PARSE, mbedtlsstub_x509crtparse_pkt_parse},
  {APICMDID_TLS_X509_CRT_INFO, mbedtlsstub_x509crtinfo_pkt_parse},
  {APICMDID_TLS_X509_CRT_VERIFY_INFO, mbedtlsstub_x509crtvrfyinfo_pkt_parse},
  {APICMDID_TLS_PK_INIT, mbedtlsstub_pkinit_pkt_parse},
  {APICMDID_TLS_PK_FREE, mbedtlsstub_pkfree_pkt_parse},
  {APICMDID_TLS_PK_PARSE_KEYFILE, mbedtlsstub_pkparsekeyfile_pkt_parse},
  {APICMDID_TLS_PK_PARSE_KEY, mbedtlsstub_pkparsekey_pkt_parse},
  {APICMDID_TLS_PK_CHECK_PAIR, mbedtlsstub_pkcheckpair_pkt_parse},
  {APICMDID_TLS_PK_SETUP, mbedtlsstub_pksetup_pkt_parse},
  {APICMDID_TLS_PK_INFO_FROM_TYPE, mbedtlsstub_pkinfofromtype_pkt_parse},
  {APICMDID_TLS_PK_WRITE_KEY_PEM, mbedtlsstub_pkwritekeypem_pkt_parse},
  {APICMDID_TLS_PK_WRITE_KEY_DER, mbedtlsstub_pkwritekeyder_pkt_parse},
  {APICMDID_TLS_PK_RSA, mbedtlsstub_pkrsa_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_INIT, mbedtlsstub_ctrdrbginit_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_FREE, mbedtlsstub_ctrdrbgfree_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_SEED, mbedtlsstub_ctrdrbgseed_pkt_parse},
  {APICMDID_TLS_ENTROPY_INIT, mbedtlsstub_entropyinit_pkt_parse},
  {APICMDID_TLS_ENTROPY_FREE, mbedtlsstub_entropyfree_pkt_parse},
  {APICMDID_TLS_CIPHER_INIT, mbedtlsstub_cipherinit_pkt_parse},
  {APICMDID_TLS_CIPHER_FREE, mbedtlsstub_cipherfree_pkt_parse},
  {APICMDID_TLS_CIPHER_INFO_FROM_STRING, mbedtlsstub_cipherinfofromstr_pkt_parse},
  {APICMDID_TLS_CIPHER_SETUP, mbedtlsstub_ciphersetup_pkt_parse},
  {APICMDID_TLS_CIPHER_SETKEY, mbedtlsstub_ciphersetkey_pkt_parse},
  {APICMDID_TLS_CIPHER_SET_IV, mbedtlsstub_ciphersetiv_pkt_parse},
  {APICMDID_TLS_CIPHER_UPDATE, mbedtlsstub_cipherupdate_pkt_parse},
  {APICMDID_TLS_MD_INFO_FROM_TYPE, mbedtlsstub_mdinfofromtype_pkt_parse},
  {APICMDID_TLS_MD_GET_SIZE, mbedtlsstub_mdgetsize_pkt_parse},
  {APICMDID_TLS_MD, mbedtlsstub_md_pkt_parse},
  {APICMDID_TLS_MD_DIGEST, mbedtlsstub_mddigest_pkt_parse},
  {APICMDID_TLS_BASE64_ENCODE, mbedtlsstub_base64enc_pkt_parse},
  {APICMDID_TLS_SHA1, mbedtlsstub_sha1_pkt_parse},
  {APICMDID_TLS_SSL_EXPORT_SRTP_KEYS, mbedtlsstub_sslexportsrtpkeys_pkt_parse},
  {APICMDID_TLS_SSL_USE_SRTP, mbedtlsstub_sslusesrtp_pkt_parse},
  {APICMDID_TLS_SSL_SRTP_PROFILE, mbedtlsstub_srtpprofile_pkt_parse},
  {APICMDID_TLS_SSL_TURN, mbedtlsstub_sslturn_pkt_parse},
  {APICMDID_TLS_MPI_INIT, mbedtlsstub_mpiinit_pkt_parse},
  {APICMDID_TLS_MPI_FREE, mbedtlsstub_mpifree_pkt_parse},
  {APICMDID_TLS_MPI_READ_STRING, mbedtlsstub_mpireadstr_pkt_parse},
  {APICMDID_TLS_MPI_WRITE_STRING, mbedtlsstub_mpiwritestr_pkt_parse},
  {APICMDID_TLS_X509_CSR_INIT, mbedtlsstub_x509csrinit_pkt_parse},
  {APICMDID_TLS_X509_CSR_FREE, mbedtlsstub_x509csrfree_pkt_parse},
  {APICMDID_TLS_X509_CSR_PARSE_FILE, mbedtlsstub_x509csrparsefile_pkt_parse},
  {APICMDID_TLS_X509_CSR_PARSE_DER, mbedtlsstub_x509csrparseder_pkt_parse},
  {APICMDID_TLS_X509_CSR_PARSE, mbedtlsstub_x509csrparse_pkt_parse},
  {APICMDID_TLS_X509_DN_GETS_CRT, mbedtlsstub_x509dngetscrt_pkt_parse},
  {APICMDID_TLS_X509_DN_GETS_CSR, mbedtlsstub_x509dngetscsr_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_INIT, mbedtlsstub_x509writecrtinit_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_FREE, mbedtlsstub_x509writecrtfree_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_DER, mbedtlsstub_x509writecrtder_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_PEM, mbedtlsstub_x509writecrtpem_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SUBJECT_KEY, mbedtlsstub_x509writecrtsubkey_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_ISSUER_KEY, mbedtlsstub_x509writecrtissuerkey_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SUBJECT_NAME, mbedtlsstub_x509writecrtsubname_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_ISSUER_NAME, mbedtlsstub_x509writecrtissuername_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_VERSION, mbedtlsstub_x509writecrtver_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_MD_ALG, mbedtlsstub_x509writecrtmdalg_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SERIAL, mbedtlsstub_x509writecrtserial_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_VALIDITY, mbedtlsstub_x509writecrtvalidity_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_CONSTRAINTS, mbedtlsstub_x509writecrtconst_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SUBJECT_ID, mbedtlsstub_x509writecrtsubid_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_AUTHORITY_ID, mbedtlsstub_x509writecrtauthid_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_KEY_USAGE, mbedtlsstub_x509writecrtkeyusage_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE, mbedtlsstub_x509writecrtnscerttype_pkt_parse},
  {APICMDID_TLS_RSA_INIT, mbedtlsstub_rsainit_pkt_parse},
  {APICMDID_TLS_RSA_FREE, mbedtlsstub_rsafree_pkt_parse},
  {APICMDID_TLS_RSA_GEN_KEY, mbedtlsstub_rsagenkey_pkt_parse},
  {APICMDID_TLS_SSL_CMD, mbedtlsstub_sslcmd_pkt_parse},
  {APICMDID_TLS_CONFIG_CMD, mbedtlsstub_configcmd_pkt_parse},
  {APICMDID_TLS_SESSION_CMD, mbedtlsstub_sessioncmd_pkt_parse},
  {APICMDID_TLS_X509_CRT_CMD, mbedtlsstub_x509crtcmd_pkt_parse},
  {APICMDID_TLS_PK_CMD, mbedtlsstub_pkcmd_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_CMD, mbedtlsstub_ctrdrbgcmd_pkt_parse},
  {APICMDID_TLS_ENTROPY_CMD, mbedtlsstub_entropycmd_pkt_parse},
  {APICMDID_TLS_CIPHER_CMD, mbedtlsstub_ciphercmd_pkt_parse},
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t mbedtlsstub_get_mbedtls_ctx_id(enum mbedtlsstub_ctx_id_e id)
{
  uint32_t ret_id = MBEDTLS_INVALID_CTX_ID;

  if (MBEDTLSSTUB_SSL_CTX_MAX <= id)
    {
      return MBEDTLS_INVALID_CTX_ID;
    }

  g_mbedtls_ctx_id_tbl[id]++;
  if (g_mbedtls_ctx_id_tbl[id] > MBEDTLS_MAXIMUM_CTX_ID)
    {
      g_mbedtls_ctx_id_tbl[id] = MBEDTLS_MINIMUM_CTX_ID;
    }
  ret_id = g_mbedtls_ctx_id_tbl[id];

  return ret_id;
}

compose_handler_t alt1250_sslcomposehdlr(uint32_t cmdid,
  FAR uint8_t *payload, size_t size)
{
  int i;
  compose_handler_t ret = NULL;

  for (i = 0; i < ARRAY_SZ(g_composehdlrs); i++)
    {
      if (g_composehdlrs[i].cmdid == cmdid)
        {
          ret = g_composehdlrs[i].hdlr;

          if (size >= BUFF_CLEAR_SIZE)
            {
              /* Clear the common area of 16 bytes from the top to 0
               * to prevent sending garbage data.
               */

              memset(payload, 0, BUFF_CLEAR_SIZE);
            }
        }
    }

  return ret;
}

parse_handler_t alt1250_sslparsehdlr(uint16_t altcid, uint8_t altver)
{
  int i;
  parse_handler_t ret = NULL;

  altcid &= ~ALTCOM_CMDID_REPLY_BIT;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      /* Change the command ID to Version 1 */

      altcid = convert_cid2v1(altcid);
      if (altcid == APICMDID_UNKNOWN)
        {
          return NULL;
        }
    }
#endif

  for (i = 0; i < ARRAY_SZ(g_parsehdlrs); i++)
    {
      if (g_parsehdlrs[i].altcid == altcid)
        {
          ret = g_parsehdlrs[i].hdlr;
        }
    }

  return ret;
}
