/****************************************************************************
 * modules/mbedtls_stub/altcom_mbedtls_hdlr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <nuttx/modem/alt1250.h>
#include <arpa/inet.h>

#include "altcom_mbedtls_hdlr.h"

#include <mbedtls/ssl.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/x509_csr.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/pk_internal.h>
#include <mbedtls/md_internal.h>
#include "vrfy_callback_mgr.h"
#include "altcom_cmdid.h"

#include "lte/lte_api.h"
#include "lte/lapi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SZ
#  define ARRAY_SZ(array) (sizeof(array)/sizeof(array[0]))
#endif

#ifdef CONFIG_LTE_NET_MBEDTLS_DEBUG_MSG
#  define TLS_DEBUG(v, ...) printf("%s: "v, __func__, ##__VA_ARGS__) 
#else
#  define TLS_DEBUG(v, ...)
#endif
#ifdef CONFIG_LTE_NET_MBEDTLS_ERROR_MSG
#  define TLS_ERROR(v, ...) printf("%s: "v, __func__, ##__VA_ARGS__)
#else
#  define TLS_ERROR(v, ...)
#endif

#define SSL_CIPHER_STR_BUF     64
#define BUFF_CLEAR_SIZE        16

#define MBEDTLS_INVALID_CTX_ID (0xFFFFFFFF)
#define MBEDTLS_MINIMUM_CTX_ID (0x00000001)
#define MBEDTLS_MAXIMUM_CTX_ID (0x00FFFFFF)

#define SSL_V_3_0              "SSLv3.0"
#define TLS_V_1_0              "TLSv1.0"
#define TLS_V_1_1              "TLSv1.1"
#define TLS_V_1_2              "TLSv1.2"
#define TLS_UNKNOWN            "unknown"

#define DTLS_V_1_0             "DTLSv1.0"
#define DTLS_V_1_2             "DTLSv1.2"
#define DTLS_UNKNOWN           "unknown (DTLS)"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum mbedtls_ctx_id_e
  {
    SSL_CTX = 0,
    SSL_SESSION_CTX,
    SSL_CONFIG_CTX,
    SSL_X509_CRT_CTX,
    SSL_PK_CTX,
    SSL_CTR_DRBG_CTX,
    SSL_ENTROPY_CTX,
    SSL_CIPHER_CTX,
    SSL_MPI_CTX,
    SSL_X509_CSR_CTX,
    SSL_X509WRITE_CRT_CTX,
    SSL_RSA_CTX,
    SSL_CTX_MAX,
  };

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
 * Private Function Prototypes
 ****************************************************************************/

static int32_t sslinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslsetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslhostname_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslbio_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslhandshake_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslwrite_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslread_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslclosenotif_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslver_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslciphersuite_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslciphersuiteid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslrecexp_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslvrfyresult_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ssltimercb_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslpeercert_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslbytesavail_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t conffree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confdefauts_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confauth_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confrng_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confcachain_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confowncert_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confreadtimeo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confvrfy_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confvrfycb_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confalpnproto_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t confciphersuites_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ssesioninit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sessionfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sessionget_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sessionset_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sessionreset_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtparsefile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtparseder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtparse_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtinfo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509crtvrfyinfo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkparsekeyfile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkparsekey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkcheckpair_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pksetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkinfofromtype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkwritekeypem_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkwritekeyder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t pkrsa_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ctrdrbginit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ctrdrbgfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ctrdrbgseed_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t entropyinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t entropyfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t cipherinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t cipherfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t cipherinfofromstr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ciphersetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ciphersetkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t ciphersetiv_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t cipherupdate_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mdinfofromtype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mdgetsize_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t md_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mddigest_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t base64enc_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sha1_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslexportsrtpkeys_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslusesrtp_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t srtpprofile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslturn_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mpiinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mpifree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mpireadstr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t mpiwritestr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509csrinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509csrfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509csrparsefile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509csrparseder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509csrparse_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509dngetscrt_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509dngetscsr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtpem_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtsubkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtissuerkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtsubname_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtissuername_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtver_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtmdalg_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtserial_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtvalidity_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtconst_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtsubid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtauthid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtkeyusage_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t x509writecrtnscerttype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t rsainit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t rsafree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t rsagenkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid);

static int32_t sslinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslsetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslhostname_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslbio_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslhandshake_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslwrite_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslread_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslclosenotif_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslver_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslciphersuite_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslciphersuiteid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslrecexp_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslvrfyresult_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ssltimercb_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslpeercert_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslbytesavail_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t conffree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confdefauts_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confauth_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confrng_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confcachain_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confowncert_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confreadtimeo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confvrfy_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confvrfycb_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confalpnproto_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t confciphersuites_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ssesioninit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sessionfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sessionget_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sessionset_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtparsefile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtparseder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtparse_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtinfo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtvrfyinfo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkparsekeyfile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkparsekey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkcheckpair_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pksetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkinfofromtype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkwritekeypem_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkwritekeyder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkrsa_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ctrdrbginit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ctrdrbgfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ctrdrbgseed_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t entropyinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t entropyfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t cipherinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t cipherfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t cipherinfofromstr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ciphersetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ciphersetkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ciphersetiv_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t cipherupdate_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mdinfofromtype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mdgetsize_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t md_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mddigest_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t base64enc_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sha1_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslexportsrtpkeys_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslusesrtp_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t srtpprofile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslturn_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mpiinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mpifree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mpireadstr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t mpiwritestr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509csrinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509csrfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509csrparsefile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509csrparseder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509csrparse_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509dngetscrt_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509dngetscsr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtpem_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtsubkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtissuerkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtsubname_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtissuername_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtver_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtmdalg_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtserial_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtvalidity_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtconst_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtsubid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtauthid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtkeyusage_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509writecrtnscerttype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t rsainit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t rsafree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t rsagenkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sslcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t configcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t sessioncmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t x509crtcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t pkcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ctrdrbgcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t entropycmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

static int32_t ciphercmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char* ssl_tls_ver_tbl[] =
{
  SSL_V_3_0,
  TLS_V_1_0,
  TLS_V_1_1,
  TLS_V_1_2,
  DTLS_V_1_0,
  DTLS_V_1_2,
  TLS_UNKNOWN,
  DTLS_UNKNOWN
};

static uint32_t g_mbedtls_ctx_id_tbl[SSL_CTX_MAX] =
{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };


static compose_inst_t g_composehdlrs[] =
{
  {LTE_CMDID_TLS_SSL_INIT, sslinit_pkt_compose},
  {LTE_CMDID_TLS_SSL_FREE, sslfree_pkt_compose},
  {LTE_CMDID_TLS_SSL_SETUP, sslsetup_pkt_compose},
  {LTE_CMDID_TLS_SSL_HOSTNAME, sslhostname_pkt_compose},
  {LTE_CMDID_TLS_SSL_BIO, sslbio_pkt_compose},
  {LTE_CMDID_TLS_SSL_HANDSHAKE, sslhandshake_pkt_compose},
  {LTE_CMDID_TLS_SSL_WRITE, sslwrite_pkt_compose},
  {LTE_CMDID_TLS_SSL_READ, sslread_pkt_compose},
  {LTE_CMDID_TLS_SSL_CLOSE_NOTIFY, sslclosenotif_pkt_compose},
  {LTE_CMDID_TLS_SSL_VERSION, sslver_pkt_compose},
  {LTE_CMDID_TLS_SSL_CIPHERSUITE, sslciphersuite_pkt_compose},
  {LTE_CMDID_TLS_SSL_CIPHERSUITE_ID, sslciphersuiteid_pkt_compose},
  {LTE_CMDID_TLS_SSL_RECORD_EXPANSION, sslrecexp_pkt_compose},
  {LTE_CMDID_TLS_SSL_VERIFY_RESULT, sslvrfyresult_pkt_compose},
  {LTE_CMDID_TLS_SSL_TIMER_CB, ssltimercb_pkt_compose},
  {LTE_CMDID_TLS_SSL_PEER_CERT, sslpeercert_pkt_compose},
  {LTE_CMDID_TLS_SSL_BYTES_AVAIL, sslbytesavail_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_INIT, confinit_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_FREE, conffree_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_DEFAULTS, confdefauts_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_AUTHMODE, confauth_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_RNG, confrng_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_CA_CHAIN, confcachain_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_OWN_CERT, confowncert_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_READ_TIMEOUT, confreadtimeo_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_VERIFY, confvrfy_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_VERIFY_CALLBACK, confvrfycb_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_ALPN_PROTOCOLS, confalpnproto_pkt_compose},
  {LTE_CMDID_TLS_CONFIG_CIPHERSUITES, confciphersuites_pkt_compose},
  {LTE_CMDID_TLS_SESSION_INIT, ssesioninit_pkt_compose},
  {LTE_CMDID_TLS_SESSION_FREE, sessionfree_pkt_compose},
  {LTE_CMDID_TLS_SESSION_GET, sessionget_pkt_compose},
  {LTE_CMDID_TLS_SESSION_SET, sessionset_pkt_compose},
  {LTE_CMDID_TLS_SESSION_RESET, sessionreset_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_INIT, x509crtinit_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_FREE, x509crtfree_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_PARSE_FILE, x509crtparsefile_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_PARSE_DER, x509crtparseder_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_PARSE, x509crtparse_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_INFO, x509crtinfo_pkt_compose},
  {LTE_CMDID_TLS_X509_CRT_VERIFY_INFO, x509crtvrfyinfo_pkt_compose},
  {LTE_CMDID_TLS_PK_INIT, pkinit_pkt_compose},
  {LTE_CMDID_TLS_PK_FREE, pkfree_pkt_compose},
  {LTE_CMDID_TLS_PK_PARSE_KEYFILE, pkparsekeyfile_pkt_compose},
  {LTE_CMDID_TLS_PK_PARSE_KEY, pkparsekey_pkt_compose},
  {LTE_CMDID_TLS_PK_CHECK_PAIR, pkcheckpair_pkt_compose},
  {LTE_CMDID_TLS_PK_SETUP, pksetup_pkt_compose},
  {LTE_CMDID_TLS_PK_INFO_FROM_TYPE, pkinfofromtype_pkt_compose},
  {LTE_CMDID_TLS_PK_WRITE_KEY_PEM, pkwritekeypem_pkt_compose},
  {LTE_CMDID_TLS_PK_WRITE_KEY_DER, pkwritekeyder_pkt_compose},
  {LTE_CMDID_TLS_PK_RSA, pkrsa_pkt_compose},
  {LTE_CMDID_TLS_CTR_DRBG_INIT, ctrdrbginit_pkt_compose},
  {LTE_CMDID_TLS_CTR_DRBG_FREE, ctrdrbgfree_pkt_compose},
  {LTE_CMDID_TLS_CTR_DRBG_SEED, ctrdrbgseed_pkt_compose},
  {LTE_CMDID_TLS_ENTROPY_INIT, entropyinit_pkt_compose},
  {LTE_CMDID_TLS_ENTROPY_FREE, entropyfree_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_INIT, cipherinit_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_FREE, cipherfree_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_INFO_FROM_STRING, cipherinfofromstr_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_SETUP, ciphersetup_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_SETKEY, ciphersetkey_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_SET_IV, ciphersetiv_pkt_compose},
  {LTE_CMDID_TLS_CIPHER_UPDATE, cipherupdate_pkt_compose},
  {LTE_CMDID_TLS_MD_INFO_FROM_TYPE, mdinfofromtype_pkt_compose},
  {LTE_CMDID_TLS_MD_GET_SIZE, mdgetsize_pkt_compose},
  {LTE_CMDID_TLS_MD, md_pkt_compose},
  {LTE_CMDID_TLS_MD_DIGEST, mddigest_pkt_compose},
  {LTE_CMDID_TLS_BASE64_ENCODE, base64enc_pkt_compose},
  {LTE_CMDID_TLS_SHA1, sha1_pkt_compose},
  {LTE_CMDID_TLS_SSL_EXPORT_SRTP_KEYS, sslexportsrtpkeys_pkt_compose},
  {LTE_CMDID_TLS_SSL_USE_SRTP, sslusesrtp_pkt_compose},
  {LTE_CMDID_TLS_SSL_SRTP_PROFILE, srtpprofile_pkt_compose},
  {LTE_CMDID_TLS_SSL_TURN, sslturn_pkt_compose},
  {LTE_CMDID_TLS_MPI_INIT, mpiinit_pkt_compose},
  {LTE_CMDID_TLS_MPI_FREE, mpifree_pkt_compose},
  {LTE_CMDID_TLS_MPI_READ_STRING, mpireadstr_pkt_compose},
  {LTE_CMDID_TLS_MPI_WRITE_STRING, mpiwritestr_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_INIT, x509csrinit_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_FREE, x509csrfree_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_PARSE_FILE, x509csrparsefile_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_PARSE_DER, x509csrparseder_pkt_compose},
  {LTE_CMDID_TLS_X509_CSR_PARSE, x509csrparse_pkt_compose},
  {LTE_CMDID_TLS_X509_DN_GETS_CRT, x509dngetscrt_pkt_compose},
  {LTE_CMDID_TLS_X509_DN_GETS_CSR, x509dngetscsr_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_INIT, x509writecrtinit_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_FREE, x509writecrtfree_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_DER, x509writecrtder_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_PEM, x509writecrtpem_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_KEY, x509writecrtsubkey_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_KEY, x509writecrtissuerkey_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_NAME, x509writecrtsubname_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_NAME, x509writecrtissuername_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_VERSION, x509writecrtver_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_MD_ALG, x509writecrtmdalg_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SERIAL, x509writecrtserial_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_VALIDITY, x509writecrtvalidity_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_CONSTRAINTS, x509writecrtconst_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_ID, x509writecrtsubid_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_AUTHORITY_ID, x509writecrtauthid_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_KEY_USAGE, x509writecrtkeyusage_pkt_compose},
  {LTE_CMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE, x509writecrtnscerttype_pkt_compose},
  {LTE_CMDID_TLS_RSA_INIT, rsainit_pkt_compose},
  {LTE_CMDID_TLS_RSA_FREE, rsafree_pkt_compose},
  {LTE_CMDID_TLS_RSA_GEN_KEY, rsagenkey_pkt_compose},
};

static parse_inst_t g_parsehdlrs[] =
{
  {APICMDID_TLS_SSL_INIT, sslinit_pkt_parse},
  {APICMDID_TLS_SSL_FREE, sslfree_pkt_parse},
  {APICMDID_TLS_SSL_SETUP, sslsetup_pkt_parse},
  {APICMDID_TLS_SSL_HOSTNAME, sslhostname_pkt_parse},
  {APICMDID_TLS_SSL_BIO, sslbio_pkt_parse},
  {APICMDID_TLS_SSL_HANDSHAKE, sslhandshake_pkt_parse},
  {APICMDID_TLS_SSL_WRITE, sslwrite_pkt_parse},
  {APICMDID_TLS_SSL_READ, sslread_pkt_parse},
  {APICMDID_TLS_SSL_CLOSE_NOTIFY, sslclosenotif_pkt_parse},
  {APICMDID_TLS_SSL_VERSION, sslver_pkt_parse},
  {APICMDID_TLS_SSL_CIPHERSUITE, sslciphersuite_pkt_parse},
  {APICMDID_TLS_SSL_CIPHERSUITE_ID, sslciphersuiteid_pkt_parse},
  {APICMDID_TLS_SSL_RECORD_EXPANSION, sslrecexp_pkt_parse},
  {APICMDID_TLS_SSL_VERIFY_RESULT, sslvrfyresult_pkt_parse},
  {APICMDID_TLS_SSL_TIMER_CB, ssltimercb_pkt_parse},
  {APICMDID_TLS_SSL_PEER_CERT, sslpeercert_pkt_parse},
  {APICMDID_TLS_SSL_BYTES_AVAIL, sslbytesavail_pkt_parse},
  {APICMDID_TLS_CONFIG_INIT, confinit_pkt_parse},
  {APICMDID_TLS_CONFIG_FREE, conffree_pkt_parse},
  {APICMDID_TLS_CONFIG_DEFAULTS, confdefauts_pkt_parse},
  {APICMDID_TLS_CONFIG_AUTHMODE, confauth_pkt_parse},
  {APICMDID_TLS_CONFIG_RNG, confrng_pkt_parse},
  {APICMDID_TLS_CONFIG_CA_CHAIN, confcachain_pkt_parse},
  {APICMDID_TLS_CONFIG_OWN_CERT, confowncert_pkt_parse},
  {APICMDID_TLS_CONFIG_READ_TIMEOUT, confreadtimeo_pkt_parse},
  {APICMDID_TLS_CONFIG_VERIFY, confvrfy_pkt_parse},
  {APICMDID_TLS_CONFIG_VERIFY_CALLBACK, confvrfycb_pkt_parse},
  {APICMDID_TLS_CONFIG_ALPN_PROTOCOLS, confalpnproto_pkt_parse},
  {APICMDID_TLS_CONFIG_CIPHERSUITES, confciphersuites_pkt_parse},
  {APICMDID_TLS_SESSION_INIT, ssesioninit_pkt_parse},
  {APICMDID_TLS_SESSION_FREE, sessionfree_pkt_parse},
  {APICMDID_TLS_SESSION_GET, sessionget_pkt_parse},
  {APICMDID_TLS_SESSION_SET, sessionset_pkt_parse},
  {APICMDID_TLS_X509_CRT_INIT, x509crtinit_pkt_parse},
  {APICMDID_TLS_X509_CRT_FREE, x509crtfree_pkt_parse},
  {APICMDID_TLS_X509_CRT_PARSE_FILE, x509crtparsefile_pkt_parse},
  {APICMDID_TLS_X509_CRT_PARSE_DER, x509crtparseder_pkt_parse},
  {APICMDID_TLS_X509_CRT_PARSE, x509crtparse_pkt_parse},
  {APICMDID_TLS_X509_CRT_INFO, x509crtinfo_pkt_parse},
  {APICMDID_TLS_X509_CRT_VERIFY_INFO, x509crtvrfyinfo_pkt_parse},
  {APICMDID_TLS_PK_INIT, pkinit_pkt_parse},
  {APICMDID_TLS_PK_FREE, pkfree_pkt_parse},
  {APICMDID_TLS_PK_PARSE_KEYFILE, pkparsekeyfile_pkt_parse},
  {APICMDID_TLS_PK_PARSE_KEY, pkparsekey_pkt_parse},
  {APICMDID_TLS_PK_CHECK_PAIR, pkcheckpair_pkt_parse},
  {APICMDID_TLS_PK_SETUP, pksetup_pkt_parse},
  {APICMDID_TLS_PK_INFO_FROM_TYPE, pkinfofromtype_pkt_parse},
  {APICMDID_TLS_PK_WRITE_KEY_PEM, pkwritekeypem_pkt_parse},
  {APICMDID_TLS_PK_WRITE_KEY_DER, pkwritekeyder_pkt_parse},
  {APICMDID_TLS_PK_RSA, pkrsa_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_INIT, ctrdrbginit_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_FREE, ctrdrbgfree_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_SEED, ctrdrbgseed_pkt_parse},
  {APICMDID_TLS_ENTROPY_INIT, entropyinit_pkt_parse},
  {APICMDID_TLS_ENTROPY_FREE, entropyfree_pkt_parse},
  {APICMDID_TLS_CIPHER_INIT, cipherinit_pkt_parse},
  {APICMDID_TLS_CIPHER_FREE, cipherfree_pkt_parse},
  {APICMDID_TLS_CIPHER_INFO_FROM_STRING, cipherinfofromstr_pkt_parse},
  {APICMDID_TLS_CIPHER_SETUP, ciphersetup_pkt_parse},
  {APICMDID_TLS_CIPHER_SETKEY, ciphersetkey_pkt_parse},
  {APICMDID_TLS_CIPHER_SET_IV, ciphersetiv_pkt_parse},
  {APICMDID_TLS_CIPHER_UPDATE, cipherupdate_pkt_parse},
  {APICMDID_TLS_MD_INFO_FROM_TYPE, mdinfofromtype_pkt_parse},
  {APICMDID_TLS_MD_GET_SIZE, mdgetsize_pkt_parse},
  {APICMDID_TLS_MD, md_pkt_parse},
  {APICMDID_TLS_MD_DIGEST, mddigest_pkt_parse},
  {APICMDID_TLS_BASE64_ENCODE, base64enc_pkt_parse},
  {APICMDID_TLS_SHA1, sha1_pkt_parse},
  {APICMDID_TLS_SSL_EXPORT_SRTP_KEYS, sslexportsrtpkeys_pkt_parse},
  {APICMDID_TLS_SSL_USE_SRTP, sslusesrtp_pkt_parse},
  {APICMDID_TLS_SSL_SRTP_PROFILE, srtpprofile_pkt_parse},
  {APICMDID_TLS_SSL_TURN, sslturn_pkt_parse},
  {APICMDID_TLS_MPI_INIT, mpiinit_pkt_parse},
  {APICMDID_TLS_MPI_FREE, mpifree_pkt_parse},
  {APICMDID_TLS_MPI_READ_STRING, mpireadstr_pkt_parse},
  {APICMDID_TLS_MPI_WRITE_STRING, mpiwritestr_pkt_parse},
  {APICMDID_TLS_X509_CSR_INIT, x509csrinit_pkt_parse},
  {APICMDID_TLS_X509_CSR_FREE, x509csrfree_pkt_parse},
  {APICMDID_TLS_X509_CSR_PARSE_FILE, x509csrparsefile_pkt_parse},
  {APICMDID_TLS_X509_CSR_PARSE_DER, x509csrparseder_pkt_parse},
  {APICMDID_TLS_X509_CSR_PARSE, x509csrparse_pkt_parse},
  {APICMDID_TLS_X509_DN_GETS_CRT, x509dngetscrt_pkt_parse},
  {APICMDID_TLS_X509_DN_GETS_CSR, x509dngetscsr_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_INIT, x509writecrtinit_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_FREE, x509writecrtfree_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_DER, x509writecrtder_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_PEM, x509writecrtpem_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SUBJECT_KEY, x509writecrtsubkey_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_ISSUER_KEY, x509writecrtissuerkey_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SUBJECT_NAME, x509writecrtsubname_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_ISSUER_NAME, x509writecrtissuername_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_VERSION, x509writecrtver_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_MD_ALG, x509writecrtmdalg_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SERIAL, x509writecrtserial_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_VALIDITY, x509writecrtvalidity_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_CONSTRAINTS, x509writecrtconst_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_SUBJECT_ID, x509writecrtsubid_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_AUTHORITY_ID, x509writecrtauthid_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_KEY_USAGE, x509writecrtkeyusage_pkt_parse},
  {APICMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE, x509writecrtnscerttype_pkt_parse},
  {APICMDID_TLS_RSA_INIT, rsainit_pkt_parse},
  {APICMDID_TLS_RSA_FREE, rsafree_pkt_parse},
  {APICMDID_TLS_RSA_GEN_KEY, rsagenkey_pkt_parse},
  {APICMDID_TLS_SSL_CMD, sslcmd_pkt_parse},
  {APICMDID_TLS_CONFIG_CMD, configcmd_pkt_parse},
  {APICMDID_TLS_SESSION_CMD, sessioncmd_pkt_parse},
  {APICMDID_TLS_X509_CRT_CMD, x509crtcmd_pkt_parse},
  {APICMDID_TLS_PK_CMD, pkcmd_pkt_parse},
  {APICMDID_TLS_CTR_DRBG_CMD, ctrdrbgcmd_pkt_parse},
  {APICMDID_TLS_ENTROPY_CMD, entropycmd_pkt_parse},
  {APICMDID_TLS_CIPHER_CMD, ciphercmd_pkt_parse},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t get_mbedtls_ctx_id(enum mbedtls_ctx_id_e id)
{
  uint32_t ret_id = MBEDTLS_INVALID_CTX_ID;

  if (SSL_CTX_MAX <= id)
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

static const char *get_ssl_tls_version(const int8_t *ver_name)
{
  int i = 0;
  int size = sizeof(ssl_tls_ver_tbl) / sizeof(ssl_tls_ver_tbl[0]);

  for (i = 0; i < size; i++)
    {
      if (strcmp((const char*) ver_name, ssl_tls_ver_tbl[i]) == 0)
        {
          return ssl_tls_ver_tbl[i];
        }
    }

  return TLS_UNKNOWN;
}

static int32_t sslinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_init_s *out =
        (FAR struct apicmd_ssl_init_s *)pktbuf;

      out->ssl = htonl(*id);

      *altcid = APICMDID_TLS_SSL_INIT;
      size = sizeof(struct apicmd_ssl_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_INIT);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_init]ctx id: %ld\n", *id);

  return size;
}

static int32_t sslfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_free_s *out =
        (FAR struct apicmd_ssl_free_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_FREE;
      size = sizeof(FAR struct apicmd_ssl_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
          (struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_FREE);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_free]ctx id: %ld\n", ssl->id);

  return size;
}

static int32_t sslsetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_setup_s *out =
        (FAR struct apicmd_ssl_setup_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_SSL_SETUP;
      size = sizeof(struct apicmd_ssl_setup_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_SETUP);
      out->u.setup.conf = htonl(conf->id);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_setup]ssl id: %ld\n", ssl->id);
  TLS_DEBUG("[ssl_setup]conf id: %ld\n", conf->id);

  return size;
}

static int32_t sslhostname_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR const char *hostname = (FAR const char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_hostname_s *out =
        (FAR struct apicmd_ssl_hostname_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      memset(out->hostname, '\0', APICMD_SSL_HOSTNAME_LEN);
      if (hostname != NULL)
        {
          size_t hostname_len = strnlen(hostname,
            APICMD_SSL_HOSTNAME_LEN-1);
          strncpy((char*)out->hostname, hostname, hostname_len);
        }

      TLS_DEBUG("[ssl_hostname]id: %ld\n", ssl->id);
      TLS_DEBUG("[ssl_hostname]hostname: %s\n", out->hostname);

      *altcid = APICMDID_TLS_SSL_HOSTNAME;
      size = sizeof(struct apicmd_ssl_hostname_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_HOSTNAME);
      memset(out->u.hostname.hostname, '\0', APICMD_SSL_HOSTNAME_LEN);
      if (hostname != NULL)
        {
          size_t hostname_len = strnlen(hostname, APICMD_SSL_HOSTNAME_LEN-1);
          strncpy((char *)out->u.hostname.hostname, hostname, hostname_len);
        }

      TLS_DEBUG("[ssl_hostname]id: %ld\n", ssl->id);
      TLS_DEBUG("[ssl_hostname]hostname: %s\n",
        out->u.hostname.hostname);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t sslbio_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR void *p_bio = (FAR void *)arg[1];
  uint32_t fd = *(uint32_t *)arg[5];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_bio_s *out =
        (FAR struct apicmd_ssl_bio_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      TLS_DEBUG("[ssl_bio]id: %ld\n", ssl->id);

      if (p_bio != NULL)
        {
          out->p_bio = htonl(fd);

          TLS_DEBUG("[ssl_bio]p_bio(fd): %lu\n", fd);
        }
      else
        {
          out->p_bio = 0;
          TLS_DEBUG("[ssl_bio]p_bio: %lu\n", out->p_bio);
        }

      *altcid = APICMDID_TLS_SSL_BIO;
      size = sizeof(struct apicmd_ssl_bio_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_BIO);
      TLS_DEBUG("[ssl_bio]id: %lu\n", ssl->id);

      if (p_bio != NULL)
        {
          out->u.bio.p_bio = htonl(fd);

          TLS_DEBUG("[ssl_bio]p_bio(fd): %lu\n", fd);
        }
      else
        {
          out->u.bio.p_bio = 0;
          TLS_DEBUG("[ssl_bio]p_bio: %lu\n", out->u.bio.p_bio);
        }

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t sslhandshake_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_handshake_s *out =
        (FAR struct apicmd_ssl_handshake_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_HANDSHAKE;
      size = sizeof(struct apicmd_ssl_handshake_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_HANDSHAKE);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_handshake]id: %lu\n", ssl->id);

  return size;
}

static int32_t sslwrite_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  size_t   writelen = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *len = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_write_s *out =
        (FAR struct apicmd_ssl_write_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      writelen = (*len <= APICMD_SSL_WRITE_BUF_LEN)
        ? *len : APICMD_SSL_WRITE_BUF_LEN;
      memcpy(out->buf, buf, writelen);
      out->len = htonl(writelen);

      *altcid = APICMDID_TLS_SSL_WRITE;
      size = sizeof(struct apicmd_ssl_write_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_WRITE);
      writelen = (*len <= APICMD_SSL_WRITE_BUF_LEN)
        ? *len : APICMD_SSL_WRITE_BUF_LEN;
      memcpy(out->u.write.buf, buf, writelen);
      out->u.write.len = htonl(writelen);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_write]ctx id: %lu\n", ssl->id);
  TLS_DEBUG("[ssl_write]write len: %d\n", writelen);

  return size;
}

static int32_t sslread_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  size_t req_buf_len = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR size_t *len = (FAR size_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_read_s *out =
        (FAR struct apicmd_ssl_read_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
        ? *len : APICMD_SSL_READ_BUF_LEN;
      out->len = htonl(req_buf_len);

      *altcid = APICMDID_TLS_SSL_READ;
      size = sizeof(struct apicmd_ssl_read_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_READ);

      req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
        ? *len : APICMD_SSL_READ_BUF_LEN;
      out->u.read.len = htonl(req_buf_len);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_read]ctx id: %lu\n", ssl->id);
  TLS_DEBUG("[ssl_read]read len: %d\n", req_buf_len);

  return size;
}

static int32_t sslclosenotif_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_close_notify_s *out =
        (FAR struct apicmd_ssl_close_notify_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_CLOSE_NOTIFY;
      size = sizeof(struct apicmd_ssl_close_notify_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_CLOSE_NOTIFY);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_close_notify]id: %lu\n", ssl->id);

  return size;
}

static int32_t sslver_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_version_s *out =
        (FAR struct apicmd_ssl_version_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_VERSION;
      size = sizeof(struct apicmd_ssl_version_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_VERSION);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_version]ctx id: %lu\n", ssl->id);

  return size;
}

static int32_t sslciphersuite_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuite_s *out =
        (FAR struct apicmd_ssl_ciphersuite_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_CIPHERSUITE;
      size = sizeof(struct apicmd_ssl_ciphersuite_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_CIPHERSUITE);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_ciphersuite]ctx id: %lu\n", ssl->id);

  return size;
}

static int32_t sslciphersuiteid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const char *ciphersuite_name = (FAR const char *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuite_id_s *out =
        (FAR struct apicmd_ssl_ciphersuite_id_s *)pktbuf;

      memset(out->ciphersuite, '\0', APICMD_SSL_CIPHERSUITE_REQLEN);
      strncpy((char *)out->ciphersuite, ciphersuite_name,
        APICMD_SSL_CIPHERSUITE_REQLEN);

      TLS_DEBUG("[ssl_ciphersuite_id]ciphersuite: %s\n",
        out->ciphersuite);

      *altcid = APICMDID_TLS_SSL_CIPHERSUITE_ID;
      size = sizeof(struct apicmd_ssl_ciphersuite_id_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_CIPHERSUITE_ID);
      memset(out->u.ciphersuite_id.ciphersuite,
        '\0', APICMD_SSL_CIPHERSUITE_REQLEN);
      strncpy((char *)out->u.ciphersuite_id.ciphersuite, ciphersuite_name,
        APICMD_SSL_CIPHERSUITE_REQLEN);

      TLS_DEBUG("[ssl_ciphersuite_id]ciphersuite: %s\n",
        out->u.ciphersuite_id.ciphersuite);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t sslrecexp_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_record_expansion_s *out =
        (FAR struct apicmd_ssl_record_expansion_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_RECORD_EXPANSION;
      size = sizeof(struct apicmd_ssl_record_expansion_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_RECORD_EXPANSION);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_record_expansion]ctx id: %lu\n", ssl->id);

  return size;
}

static int32_t sslvrfyresult_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl = (FAR const mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_verify_result_s *out =
        (FAR struct apicmd_ssl_verify_result_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_VERIFY_RESULT;
      size = sizeof(struct apicmd_ssl_verify_result_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_VERIFY_RESULT);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_verify_result]ctx id: %lu\n", ssl->id);

  return size;
}

static int32_t ssltimercb_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_timer_cb_s *out =
        (FAR struct apicmd_ssl_timer_cb_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_TIMER_CB;
      size = sizeof(struct apicmd_ssl_timer_cb_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_TIMER_CB);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_timer_cb]config id: %lu\n", ssl->id);

  return size;
}

static int32_t sslpeercert_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_peer_cert_s *out =
        (FAR struct apicmd_ssl_peer_cert_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_PEER_CERT;
      size = sizeof(struct apicmd_ssl_peer_cert_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_PEER_CERT);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_peer_cert]ctx id: %lu\n", ssl->id);

  return size;
}

static int32_t sslbytesavail_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_bytes_avail_s *out =
        (FAR struct apicmd_ssl_bytes_avail_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_BYTES_AVAIL;
      size = sizeof(struct apicmd_ssl_bytes_avail_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_BYTES_AVAIL);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ssl_bytes_avail]id: %lu\n", ssl->id);

  return size;
}

static int32_t confinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_CONFIG_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_init_s *out =
        (FAR struct apicmd_config_init_s *)pktbuf;

      out->conf = htonl(*id);

      *altcid = APICMDID_TLS_CONFIG_INIT;
      size = sizeof(struct apicmd_config_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_INIT);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_init]config id: %ld\n", *id);

  return size;
}

static int32_t conffree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_free_s *out =
        (FAR struct apicmd_config_free_s *)pktbuf;

      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_CONFIG_FREE;
      size = sizeof(struct apicmd_config_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_FREE);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_free]config id: %lu\n", conf->id);

  return size;
}

static int32_t confdefauts_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR int *endpoint = (FAR int *)arg[1];
  FAR int *transport = (FAR int *)arg[2];
  FAR int *preset = (FAR int *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_defaults_s *out =
        (FAR struct apicmd_config_defaults_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->endpoint = (int32_t) htonl(*endpoint);
      out->transport = (int32_t) htonl(*transport);
      out->preset = (int32_t) htonl(*preset);

      *altcid = APICMDID_TLS_CONFIG_DEFAULTS;
      size = sizeof(struct apicmd_config_defaults_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_DEFAULTS);
      out->u.defaults.endpoint = (int32_t) htonl(*endpoint);
      out->u.defaults.transport = (int32_t) htonl(*transport);
      out->u.defaults.preset = (int32_t) htonl(*preset);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_defaults]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_defaults]endpoint: %d\n", *endpoint);
  TLS_DEBUG("[config_defaults]transport: %d\n", *transport);
  TLS_DEBUG("[config_defaults]preset: %d\n", *preset);

  return size;
}

static int32_t confauth_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR int *authmode = (FAR int *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_authmode_s *out =
        (FAR struct apicmd_config_authmode_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->authmode = (int32_t) htonl(*authmode);

      *altcid = APICMDID_TLS_CONFIG_AUTHMODE;
      size = sizeof(struct apicmd_config_authmode_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_AUTHMODE);
      out->u.authmode.authmode = (int32_t) htonl(*authmode);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_authmode]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_authmode]authmode: %d\n", *authmode);

  return size;
}

static int32_t confrng_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR void *p_rng = (FAR void *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_rng_s *out =
        (FAR struct apicmd_config_rng_s *)pktbuf;

      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_CONFIG_RNG;
      size = sizeof(struct apicmd_config_rng_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_RNG);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_rng]config id: %lu\n", conf->id);

  if (p_rng != NULL)
    {
      mbedtls_ctr_drbg_context *ctx = (mbedtls_ctr_drbg_context*)p_rng;
      uint32_t id = ctx->id;

      if (altver == ALTCOM_VER1)
        {
        FAR struct apicmd_config_rng_s *out =
          (FAR struct apicmd_config_rng_s *)pktbuf;

          out->p_rng = htonl(id);
        }
      else if (altver == ALTCOM_VER4)
        {
        FAR struct apicmd_configcmd_s *out =
          (FAR struct apicmd_configcmd_s *)pktbuf;

          out->u.rng.p_rng = htonl(id);
        }

      TLS_DEBUG("[config_rng]p_rng(id): %lu\n", id);
    }

  return size;
}

static int32_t confcachain_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR mbedtls_x509_crt *ca_chain = (FAR mbedtls_x509_crt *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ca_chain_s *out =
        (FAR struct apicmd_config_ca_chain_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->ca_chain = (ca_chain != NULL) ? htonl(ca_chain->id) : 0;
      out->ca_crl = 0; // force crl_id zero

      *altcid = APICMDID_TLS_CONFIG_CA_CHAIN;
      size = sizeof(struct apicmd_config_ca_chain_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_CA_CHAIN);
      out->u.ca_chain.ca_chain = (ca_chain != NULL) ? htonl(ca_chain->id) : 0;
      out->u.ca_chain.ca_crl = 0; // force crl_id zero

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_ca_chain]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_ca_chain]crt id: %lu\n", (ca_chain != NULL) ? ca_chain->id : 0);

  return size;
}

static int32_t confowncert_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR mbedtls_x509_crt *own_cert = (FAR mbedtls_x509_crt *)arg[1];
  FAR mbedtls_pk_context *pk_key = (FAR mbedtls_pk_context *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_own_cert_s *out =
        (FAR struct apicmd_config_own_cert_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->own_cert = (own_cert != NULL) ? htonl(own_cert->id) : 0;
      out->pk_key = htonl(pk_key->id);

      *altcid = APICMDID_TLS_CONFIG_OWN_CERT;
      size = sizeof(struct apicmd_config_own_cert_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_OWN_CERT);
      out->u.own_cert.own_cert = (own_cert != NULL) ? htonl(own_cert->id) : 0;
      out->u.own_cert.pk_key = htonl(pk_key->id);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_own_cert]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_own_cert]own_cert: %lu\n", (own_cert != NULL) ? own_cert->id : 0);
  TLS_DEBUG("[config_own_cert]pk_key: %lu\n", pk_key->id);

  return size;
}

static int32_t confreadtimeo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR uint32_t *timeout = (FAR uint32_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_read_timeout_s *out =
        (FAR struct apicmd_config_read_timeout_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->timeout = htonl(*timeout);

      *altcid = APICMDID_TLS_CONFIG_READ_TIMEOUT;
      size = sizeof(struct apicmd_config_read_timeout_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_READ_TIMEOUT);
      out->u.read_timeout.timeout = htonl(*timeout);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(FAR struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_read_timeout]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_read_timeout]timeout: %lu\n", *timeout);

  return size;
}

static int32_t confvrfy_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_verify_s *out =
        (FAR struct apicmd_config_verify_s *)pktbuf;

      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_CONFIG_VERIFY;
      size = sizeof(struct apicmd_config_verify_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_VERIFY);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_verify]config id: %lu\n", conf->id);

  return size;
}

static int32_t confvrfycb_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int *ret = (FAR int *)arg[0];
  FAR uint32_t *flags = (FAR uint32_t *)arg[1];

  FAR struct apicmd_config_verify_callback_s *out =
    (FAR struct apicmd_config_verify_callback_s *)pktbuf;

  size = sizeof(struct apicmd_config_verify_callback_s);

  out->ret_code = htonl(*ret);
  out->flags = htonl(*flags);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_TLS_CONFIG_VERIFY_CALLBACK;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_TLS_CONFIG_VERIFY_CALLBACK_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t confalpnproto_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  char **p;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR const char **protos = (FAR const char **)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_alpn_protocols_s *out =
        (FAR struct apicmd_config_alpn_protocols_s *)pktbuf;

      out->conf = htonl(conf->id);
      p = (char **)protos;

      memset(out->protos1, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos1, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->protos2, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos2, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->protos3, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos3, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->protos4, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos4, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }

      TLS_DEBUG("[config_alpn_protocols]config id: %lu\n", conf->id);
      TLS_DEBUG("[config_alpn_protocols]protos1: %s\n", out->protos1);
      TLS_DEBUG("[config_alpn_protocols]protos2: %s\n", out->protos2);
      TLS_DEBUG("[config_alpn_protocols]protos3: %s\n", out->protos3);
      TLS_DEBUG("[config_alpn_protocols]protos4: %s\n", out->protos4);

      *altcid = APICMDID_TLS_CONFIG_ALPN_PROTOCOLS;
      size = sizeof(struct apicmd_config_alpn_protocols_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      p = (char **)protos;

      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_ALPN_PROTOCOLS);
      memset(out->u.alpn_protocols.protos1,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos1, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->u.alpn_protocols.protos2,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos2, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->u.alpn_protocols.protos3,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos3, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->u.alpn_protocols.protos4,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos4, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      TLS_DEBUG("[config_alpn_protocols]config id: %lu\n", conf->id);
      TLS_DEBUG("[config_alpn_protocols]protos1: %s\n",
        out->u.alpn_protocols.protos1);
      TLS_DEBUG("[config_alpn_protocols]protos2: %s\n",
        out->u.alpn_protocols.protos2);
      TLS_DEBUG("[config_alpn_protocols]protos3: %s\n",
        out->u.alpn_protocols.protos3);
      TLS_DEBUG("[config_alpn_protocols]protos4: %s\n",
        out->u.alpn_protocols.protos4);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t confciphersuites_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int cnt;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR const int *ciphersuites = (FAR const int *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ciphersuites_s *out =
        (FAR struct apicmd_config_ciphersuites_s *)pktbuf;

      out->conf = htonl(conf->id);
      memset(out->ciphersuites,
        0, sizeof(int32_t)*APICMD_CONFIG_CIPHERSUITES_COUNT);

      cnt = 0;
      for (cnt = 0; cnt < APICMD_CONFIG_CIPHERSUITES_COUNT; cnt++)
        {
          if (ciphersuites[cnt] == 0)
            {
              break;
            }

          out->ciphersuites[cnt] = htonl(ciphersuites[cnt]);
        }

      *altcid = APICMDID_TLS_CONFIG_CIPHERSUITES;
      size = sizeof(struct apicmd_config_ciphersuites_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_CIPHERSUITES);
      memset(out->u.ciphersuites.ciphersuites, 0,
        sizeof(int32_t)*APICMD_CONFIG_CIPHERSUITES_COUNT);

      cnt = 0;
      for (cnt = 0; cnt < APICMD_CONFIG_CIPHERSUITES_COUNT; cnt++)
        {
          if (ciphersuites[cnt] == 0)
            {
              break;
            }

          out->u.ciphersuites.ciphersuites[cnt] =
             htonl(ciphersuites[cnt]);
        }

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[config_ciphersuites]config id: %lu\n", conf->id);

  return size;
}

static int32_t ssesioninit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_SESSION_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_init_s *out =
        (FAR struct apicmd_session_init_s *)pktbuf;

      out->session = htonl(*id);

      *altcid = APICMDID_TLS_SESSION_INIT;
      size = sizeof(struct apicmd_session_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->session = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_INIT);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[session_init]session id: %ld\n", *id);

  return size;
}

static int32_t sessionfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_session *session = (FAR mbedtls_ssl_session *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_free_s *out =
        (FAR struct apicmd_session_free_s *)pktbuf;

      out->session = htonl(session->id);

      *altcid = APICMDID_TLS_SESSION_FREE;
      size = sizeof(struct apicmd_session_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->session = htonl(session->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_FREE);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[session_free]session id: %lu\n", session->id);

  return size;
}

static int32_t sessionget_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];
  FAR mbedtls_ssl_session *session = (FAR mbedtls_ssl_session *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_get_s *out =
        (FAR struct apicmd_session_get_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->dst = htonl(session->id);

      *altcid = APICMDID_TLS_SESSION_GET;
      size = sizeof(struct apicmd_session_get_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->session = htonl(session->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_GET);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[session_get]ssl id: %lu\n", ssl->id);
  TLS_DEBUG("[session_get]session id: %lu\n", session->id);

  return size;
}

static int32_t sessionset_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR const mbedtls_ssl_session *session =
    (FAR const mbedtls_ssl_session *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_set_s *out =
        (FAR struct apicmd_session_set_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->session = htonl(session->id);

      *altcid = APICMDID_TLS_SESSION_SET;
      size = sizeof(struct apicmd_session_set_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->session = htonl(session->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_SET);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[session_set]ssl id: %lu\n", ssl->id);
  TLS_DEBUG("[session_set]session id: %lu\n", session->id);

  return size;
}

static int32_t sessionreset_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_RESET);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  TLS_DEBUG("[session_reset]ssl id: %lu\n", ssl->id);

  return size;
}

static int32_t x509crtinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_X509_CRT_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_init_s *out =
        (FAR struct apicmd_x509_crt_init_s *)pktbuf;

      out->crt = htonl(*id);

      *altcid = APICMDID_TLS_X509_CRT_INIT;
      size = sizeof(struct apicmd_x509_crt_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_INIT);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_init]ctx id: %ld\n", *id);

  return size;
}

static int32_t x509crtfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *crt = (FAR mbedtls_x509_crt *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_free_s *out =
        (FAR struct apicmd_x509_crt_free_s *)pktbuf;

      out->crt = htonl(crt->id);

      *altcid = APICMDID_TLS_X509_CRT_FREE;
      size = sizeof(struct apicmd_x509_crt_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(crt->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_FREE);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_free]ctx id: %lu\n", crt->id);

  return size;
}

static int32_t x509crtparsefile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[0];
  FAR const char *path = (FAR const char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_file_s *out =
        (FAR struct apicmd_x509_crt_parse_file_s *)pktbuf;

      out->chain = htonl(chain->id);
      memset(out->path, '\0', APICMD_X509_CRT_PARSE_FILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char *)out->path, path,
            APICMD_X509_CRT_PARSE_FILE_PATH_LEN-1);
        }

      TLS_DEBUG("[x509_crt_parse_file]ctx id: %lu\n", chain->id);
      TLS_DEBUG("[x509_crt_parse_file]path: %s\n", out->path);

      *altcid = APICMDID_TLS_X509_CRT_PARSE_FILE;
      size = sizeof(struct apicmd_x509_crt_parse_file_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(chain->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_PARSE_FILE);
      memset(out->u.parse_file.path,
        '\0', APICMD_X509_CRT_PARSE_FILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char *)out->u.parse_file.path, path,
            APICMD_X509_CRT_PARSE_FILE_PATH_LEN-1);
        }

      TLS_DEBUG("[x509_crt_parse_file]ctx id: %lu\n", chain->id);
      TLS_DEBUG("[x509_crt_parse_file]path: %s\n",
        out->u.parse_file.path);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t x509crtparseder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_der_s *out =
        (FAR struct apicmd_x509_crt_parse_der_s *)pktbuf;

      out->chain = htonl(chain->id);
      *buflen = (*buflen <= APICMD_X509_CRT_PARSE_DER_LEN)
        ? *buflen : APICMD_X509_CRT_PARSE_DER_LEN;
      if (buf != NULL)
        {
          memcpy(out->buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(*buflen);

      *altcid = APICMDID_TLS_X509_CRT_PARSE_DER;
      size = sizeof(struct apicmd_x509_crt_parse_der_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(chain->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_X509_CRT_PARSE_DER);
      *buflen = (*buflen <= APICMD_X509_CRT_PARSE_DER_LEN)
        ? *buflen : APICMD_X509_CRT_PARSE_DER_LEN;
      if (buf != NULL)
        {
          memcpy(out->u.parse_der.buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->u.parse_der.buflen = htonl(*buflen);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_parse_der]ctx id: %lu\n", chain->id);
  TLS_DEBUG("[x509_crt_parse_der]buflen: %d\n", *buflen);

  return size;
}

static int32_t x509crtparse_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  uint32_t lbuflen = 0;
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_s *out =
        (FAR struct apicmd_x509_crt_parse_s *)pktbuf;

      out->chain = htonl(chain->id);

      *altcid = APICMDID_TLS_X509_CRT_PARSE;
      size = sizeof(struct apicmd_x509_crt_parse_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(chain->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_PARSE);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  if (*buflen <= APICMD_X509_CRT_PARSE_BUF_LEN)
    {
      lbuflen = *buflen;
    }
  else
    {
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_s *out =
        (FAR struct apicmd_x509_crt_parse_s *)pktbuf;

      memset(out->buf, '\0', APICMD_X509_CRT_PARSE_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, lbuflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(lbuflen);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      memset(out->u.parse.buf, '\0', APICMD_X509_CRT_PARSE_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->u.parse.buf, buf, lbuflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->u.parse.buflen = htonl(lbuflen);
    }

  TLS_DEBUG("[x509_crt_parse]ctx id: %lu\n", chain->id);
  TLS_DEBUG("[x509_crt_parse]buflen: %lu\n", lbuflen);

  return size;
}

static int32_t x509crtinfo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  uint32_t buflen = 0;
  FAR size_t *size = (FAR size_t *)arg[0];
  FAR const char *prefix = (FAR const char *)arg[1];
  FAR const mbedtls_x509_crt *crt = (FAR const mbedtls_x509_crt *)arg[2];

  FAR struct apicmd_x509_crt_info_s *out =
    (FAR struct apicmd_x509_crt_info_s *)pktbuf;

  out->crt = htonl(crt->id);
  buflen = (*size <= APICMD_X509_CRT_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_INFO_RET_BUF_LEN;
  out->size = htonl(buflen);
  memset(out->prefix, '\0', APICMD_X509_CRT_INFO_PREFIX_LEN);
  if (prefix != NULL)
    {
      strncpy((char*) out->prefix, prefix,
        APICMD_X509_CRT_INFO_PREFIX_LEN-1);
    }
  ret_size = sizeof(struct apicmd_x509_crt_info_s);

  TLS_DEBUG("[x509_crt_info]ctx id: %lu\n", crt->id);
  TLS_DEBUG("[x509_crt_info]size: %d\n", *size);
  TLS_DEBUG("[x509_crt_info]prefix: %s\n", out->prefix);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_TLS_X509_CRT_INFO;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_TLS_X509_CRT_INFO_V4;
    }
  else
    {
      ret_size = -ENOSYS;
    }

  return ret_size;
}

static int32_t x509crtvrfyinfo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  uint32_t buflen = 0;
  FAR size_t *size = (FAR size_t *)arg[0];
  FAR const char *prefix = (FAR const char *)arg[1];
  FAR uint32_t *flags = (FAR uint32_t *)arg[2];

  buflen = (*size <= APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_verify_info_s *out =
        (FAR struct apicmd_x509_crt_verify_info_s *)pktbuf;

      out->size = htonl(buflen);
      memset(out->prefix, '\0', APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN);
      if (prefix != NULL)
        {
          strncpy((char *)out->prefix, prefix,
            APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN-1);
        }
      out->flags = htonl(*flags);
  
      TLS_DEBUG("[x509_crt_verify_info]size: %d\n", *size);
      TLS_DEBUG("[x509_crt_verify_info]prefix: %s\n",
        out->prefix);
      TLS_DEBUG("[x509_crt_verify_info]flags: %ld\n", *flags);

      *altcid = APICMDID_TLS_X509_CRT_VERIFY_INFO;
      ret_size = sizeof(struct apicmd_x509_crt_verify_info_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_VERIFY_INFO);
      out->u.verify_info.size = htonl(buflen);
      memset(out->u.verify_info.prefix,
        '\0', APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN);
      if (prefix != NULL)
        {
          strncpy((char *)out->u.verify_info.prefix, prefix,
            APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN-1);
        }
      out->u.verify_info.flags = htonl(*flags);

      TLS_DEBUG("[x509_crt_verify_info]size: %d\n", *size);
      TLS_DEBUG("[x509_crt_verify_info]prefix: %s\n",
        out->u.verify_info.prefix);
      TLS_DEBUG("[x509_crt_verify_info]flags: %ld\n", *flags);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      ret_size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
    {
      ret_size = -ENOSYS;
    }

  return ret_size;
}

static int32_t pkinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_PK_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_init_s *out =
        (FAR struct apicmd_pk_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_PK_INIT;
      size = sizeof(struct apicmd_pk_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_INIT);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_init]ctx id: %ld\n", *id);

  return size;
}

static int32_t pkfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *crt = (FAR mbedtls_x509_crt *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_free_s *out =
        (FAR struct apicmd_pk_free_s *)pktbuf;

      out->ctx = htonl(crt->id);

      *altcid = APICMDID_TLS_PK_FREE;
      size = sizeof(struct apicmd_pk_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(crt->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_FREE);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_free]ctx id: %lu\n", crt->id);

  return size;
}

static int32_t pkparsekeyfile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR const char *path = (FAR const char *)arg[1];
  FAR const char *password = (FAR const char *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_keyfile_s *out =
        (FAR struct apicmd_pk_parse_keyfile_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      memset(out->path, '\0', APICMD_PK_PARSE_KEYFILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char*) out->path,
            path, APICMD_PK_PARSE_KEYFILE_PATH_LEN-1);
        }
  
      memset(out->pwd, '\0', APICMD_PK_PARSE_KEYFILE_PWD_LEN);
      if (password != NULL)
        {
          strncpy((char*)out->pwd,
            password, APICMD_PK_PARSE_KEYFILE_PWD_LEN-1);
        }
  
      TLS_DEBUG("[pk_parse_keyfile]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[pk_parse_keyfile]path: %s\n", out->path);
      TLS_DEBUG("[pk_parse_keyfile]pwd: %s\n", out->pwd);

      *altcid = APICMDID_TLS_PK_PARSE_KEYFILE;
      size = sizeof(struct apicmd_pk_parse_keyfile_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_PARSE_KEYFILE);
      memset(out->u.parse_keyfile.path, '\0',
        APICMD_PK_PARSE_KEYFILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char *)out->u.parse_keyfile.path, path,
            APICMD_PK_PARSE_KEYFILE_PATH_LEN-1);
        }

      memset(out->u.parse_keyfile.pwd, '\0',
        APICMD_PK_PARSE_KEYFILE_PWD_LEN);
      if (password != NULL)
        {
          strncpy((char *)out->u.parse_keyfile.pwd, password,
            APICMD_PK_PARSE_KEYFILE_PWD_LEN-1);
        }

      TLS_DEBUG("[pk_parse_keyfile]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[pk_parse_keyfile]path: %s\n",
        out->u.parse_keyfile.path);
      TLS_DEBUG("[pk_parse_keyfile]pwd: %s\n",
        out->u.parse_keyfile.pwd);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t pkparsekey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  uint32_t buflen = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR const unsigned char *key = (FAR const unsigned char *)arg[1];
  FAR size_t *keylen = (FAR size_t *)arg[2];
  FAR const unsigned char *pwd = (FAR const unsigned char *)arg[3];
  FAR size_t *pwdlen = (FAR size_t *)arg[4];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_key_s *out =
        (FAR struct apicmd_pk_parse_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      if (*keylen <= APICMD_PK_PARSE_KEY_KEY_LEN)
        {
          buflen = *keylen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->key, '\0', APICMD_PK_PARSE_KEY_KEY_LEN);
      if (key != NULL)
        {
          memcpy(out->key, key, buflen);
        }
      out->keylen = htonl(buflen);

      if (*pwdlen <= APICMD_PK_PARSE_KEY_PWD_LEN)
        {
          buflen = *pwdlen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->pwd, '\0', APICMD_PK_PARSE_KEY_PWD_LEN);
      if (pwd != NULL)
        {
          memcpy(out->pwd, pwd, buflen);
        }
      out->pwdlen = htonl(buflen);

      *altcid = APICMDID_TLS_PK_PARSE_KEY;
      size = sizeof(struct apicmd_pk_parse_key_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_PARSE_KEY);
      if (*keylen <= APICMD_PK_PARSE_KEY_KEY_LEN)
        {
          buflen = *keylen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->u.parse_key.key, '\0', APICMD_PK_PARSE_KEY_KEY_LEN);
      if (key != NULL)
        {
          memcpy(out->u.parse_key.key, key, buflen);
        }
      out->u.parse_key.keylen = htonl(buflen);

      if (*pwdlen <= APICMD_PK_PARSE_KEY_PWD_LEN)
        {
          buflen = *pwdlen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->u.parse_key.pwd, '\0', APICMD_PK_PARSE_KEY_PWD_LEN);
      if (pwd != NULL)
        {
          memcpy(out->u.parse_key.pwd, pwd, buflen);
        }
      out->u.parse_key.pwdlen = htonl(buflen);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_parse_key]ctx id: %lu\n", ctx->id);
  TLS_DEBUG("[pk_parse_key]key: %s\n", key);
  TLS_DEBUG("[pk_parse_key]keylen: %d\n", *keylen);
  TLS_DEBUG("[pk_parse_key]pwd: %s\n", pwd);
  TLS_DEBUG("[pk_parse_key]pwdlen: %d\n", *pwdlen);

  return size;
}

static int32_t pkcheckpair_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_pk_context *pub =
    (FAR const mbedtls_pk_context *)arg[0];
  FAR const mbedtls_pk_context *prv =
    (FAR const mbedtls_pk_context *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_check_pair_s *out =
        (FAR struct apicmd_pk_check_pair_s *)pktbuf;

      out->pub = htonl(pub->id);
      out->prv = htonl(prv->id);

      *altcid = APICMDID_TLS_PK_CHECK_PAIR;
      size = sizeof(struct apicmd_pk_check_pair_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(pub->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_CHECK_PAIR);
      out->u.check_pair.prv = htonl(prv->id);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_check_pair]pub id: %lu\n", pub->id);
  TLS_DEBUG("[pk_check_pair]prv id: %lu\n", prv->id);

  return size;
}

static int32_t pksetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR const struct mbedtls_pk_info_t *info =
    (FAR const struct mbedtls_pk_info_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_setup_s *out =
        (FAR struct apicmd_pk_setup_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->info = htonl(info->id);

      *altcid = APICMDID_TLS_PK_SETUP;
      size = sizeof(struct apicmd_pk_setup_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_SETUP);
      out->u.setup.info = htonl(info->id);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_setup]ctx id: %lu\n", ctx->id);
  TLS_DEBUG("[pk_setup]info id: %lu\n", info->id);

  return size;
}

static int32_t pkinfofromtype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_pk_type_t *pk_type = (FAR mbedtls_pk_type_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_info_from_type_s *out =
        (FAR struct apicmd_pk_info_from_type_s *)pktbuf;

      out->pk_type = htonl(*pk_type);

      *altcid = APICMDID_TLS_PK_INFO_FROM_TYPE;
      size = sizeof(struct apicmd_pk_info_from_type_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->u.info_from_type.pk_type = htonl(*pk_type);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_INFO_FROM_TYPE);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_info_from_type]pk_type id: %d\n", *pk_type);

  return size;
}

static int32_t pkwritekeypem_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  size_t   req_buf_len = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_pem_s *out =
        (FAR struct apicmd_pk_write_key_pem_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;
  
      out->size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_WRITE_KEY_PEM;
      ret_size = sizeof(struct apicmd_pk_write_key_pem_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;
  
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_WRITE_KEY_PEM);
      out->u.write_key_pem.size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      ret_size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      ret_size = -ENOSYS;
    }

  TLS_DEBUG("[pk_write_key_pem]config id: %lu\n", ctx->id);

  return ret_size;
}

static int32_t pkwritekeyder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  size_t   req_buf_len = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_der_s *out =
        (FAR struct apicmd_pk_write_key_der_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;
  
      out->size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_WRITE_KEY_DER;
      ret_size = sizeof(struct apicmd_pk_write_key_der_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;
  
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_WRITE_KEY_DER);
      out->u.write_key_der.size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      ret_size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      ret_size = -ENOSYS;
    }

  TLS_DEBUG("[pk_write_key_der]config id: %lu\n", ctx->id);

  return ret_size;
}

static int32_t pkrsa_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_pk_context *pk =
    (FAR const mbedtls_pk_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_rsa_s *out =
        (FAR struct apicmd_pk_rsa_s *)pktbuf;

      out->pk = htonl(pk->id);

      *altcid = APICMDID_TLS_PK_RSA;
      size = sizeof(struct apicmd_pk_rsa_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(pk->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_RSA);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[pk_rsa]pk id: %lu\n", pk->id);

  return size;
}

static int32_t ctrdrbginit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_CTR_DRBG_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_init_s *out =
        (FAR struct apicmd_ctr_drbg_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_CTR_DRBG_INIT;
      size = sizeof(struct apicmd_ctr_drbg_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmd_s *out =
        (FAR struct apicmd_ctr_drbgcmd_s *)pktbuf;

      out->ctx = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CTR_DRBG_INIT);

      *altcid = APICMDID_TLS_CTR_DRBG_CMD_V4;
      size = sizeof(struct apicmd_ctr_drbgcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ctr_drbg_init]ctx id: %ld\n", *id);

  return size;
}

static int32_t ctrdrbgfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ctr_drbg_context *ctx =
    (FAR mbedtls_ctr_drbg_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_free_s *out =
        (FAR struct apicmd_ctr_drbg_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      *altcid = APICMDID_TLS_CTR_DRBG_FREE;
      size = sizeof(struct apicmd_ctr_drbg_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmd_s *out =
        (FAR struct apicmd_ctr_drbgcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CTR_DRBG_FREE);

      *altcid = APICMDID_TLS_CTR_DRBG_CMD_V4;
      size = sizeof(struct apicmd_ctr_drbgcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ctr_drbg_free]ctx id: %lu\n", ctx->id);

  return size;
}

static int32_t ctrdrbgseed_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ctr_drbg_context *ctx =
    (FAR mbedtls_ctr_drbg_context *)arg[0];
  FAR void *p_entropy = (FAR void *)arg[2];
  FAR const unsigned char *custom = (FAR const unsigned char *)arg[3];
  FAR size_t *len = (FAR size_t *)arg[4];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_seed_s *out =
        (FAR struct apicmd_ctr_drbg_seed_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      TLS_DEBUG("[ctr_drbg_seed]ctx id: %lu\n", ctx->id);

      if (p_entropy != NULL)
        {
          mbedtls_entropy_context *lctx =
            (mbedtls_entropy_context*) p_entropy;
          out->p_entropy = htonl(lctx->id);
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: %lu\n", lctx->id);
        }
      else
        {
          out->p_entropy = 0;
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
        }

      memset(out->custom, 0, APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
      memcpy(out->custom, custom, *len);

      out->len = htonl(*len);

      *altcid = APICMDID_TLS_CTR_DRBG_SEED;
      size = sizeof(struct apicmd_ctr_drbg_seed_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmd_s *out =
        (FAR struct apicmd_ctr_drbgcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      TLS_DEBUG("[ctr_drbg_seed]ctx id: %lu\n", ctx->id);

      out->subcmd_id = htonl(APISUBCMDID_TLS_CTR_DRBG_SEED);

      if (p_entropy != NULL)
        {
          mbedtls_entropy_context *lctx =
            (mbedtls_entropy_context*)p_entropy;
          out->u.seed.p_entropy = htonl(lctx->id);
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: %lu\n", lctx->id);
        }
      else
        {
          out->u.seed.p_entropy = 0;
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
        }

      memset(out->u.seed.custom, 0, APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
      memcpy(out->u.seed.custom, custom, *len);

      out->u.seed.len = htonl(*len);

      *altcid = APICMDID_TLS_CTR_DRBG_CMD_V4;
      size = sizeof(struct apicmd_ctr_drbgcmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[ctr_drbg_seed]len: %zu\n", *len);

  return size;
}

static int32_t entropyinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_ENTROPY_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_entropy_init_s *out =
        (FAR struct apicmd_entropy_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_ENTROPY_INIT;
      size = sizeof(struct apicmd_entropy_init_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_entropycmd_s *out =
        (FAR struct apicmd_entropycmd_s *)pktbuf;

      out->ctx = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_ENTROPY_INIT);

      *altcid = APICMDID_TLS_ENTROPY_CMD_V4;
      size = sizeof(struct apicmd_entropycmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[entropy_init]ctx id: %ld\n", *id);

  return size;
}

static int32_t entropyfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_entropy_context *ctx = (FAR mbedtls_entropy_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_entropy_free_s *out =
        (FAR struct apicmd_entropy_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      *altcid = APICMDID_TLS_ENTROPY_FREE;
      size = sizeof(struct apicmd_entropy_free_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_entropycmd_s *out =
        (FAR struct apicmd_entropycmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_ENTROPY_FREE);

      *altcid = APICMDID_TLS_ENTROPY_CMD_V4;
      size = sizeof(struct apicmd_entropycmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[entropy_free]ctx id: %lu\n", ctx->id);

  return size;
}

static int32_t cipherinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_CIPHER_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_init_s *out =
        (FAR struct apicmd_cipher_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_CIPHER_INIT;
      size = sizeof(struct apicmd_cipher_init_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  TLS_DEBUG("[cipher_init]ctx id: %ld\n", *id);

  return size;
}

static int32_t cipherfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_free_s *out =
        (FAR struct apicmd_cipher_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      *altcid = APICMDID_TLS_CIPHER_FREE;
      size = sizeof(struct apicmd_cipher_free_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  TLS_DEBUG("[cipher_free]ctx id: %lu\n", ctx->id);

  return size;
}

static int32_t cipherinfofromstr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const char *cipher_name = (FAR const char *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_info_from_string_s *out =
        (FAR struct apicmd_cipher_info_from_string_s *)pktbuf;

      memset(out->cipher_name, '\0', APICMD_CIPHER_INFO_NAME_LEN);
      strncpy((char*) out->cipher_name, cipher_name,
        APICMD_CIPHER_INFO_NAME_LEN-1);
      TLS_DEBUG("[cipher_info_from_string]cipher name: %s\n",
        out->cipher_name);

      *altcid = APICMDID_TLS_CIPHER_INFO_FROM_STRING;
      size = sizeof(struct apicmd_cipher_info_from_string_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t ciphersetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
  FAR mbedtls_cipher_info_t *cipher_info =
    (FAR mbedtls_cipher_info_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setup_s *out =
        (FAR struct apicmd_cipher_setup_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->cipher_info = htonl(cipher_info->id);

      TLS_DEBUG("[cipher_setup]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_setup]info id: %lu\n", cipher_info->id);

      *altcid = APICMDID_TLS_CIPHER_SETUP;
      size = sizeof(struct apicmd_cipher_setup_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t ciphersetkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
  FAR const unsigned char *key = (FAR const unsigned char *)arg[1];
  FAR int *key_bitlen = (FAR int *)arg[2];
  FAR mbedtls_operation_t *operation = (FAR mbedtls_operation_t *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setkey_s *out =
        (FAR struct apicmd_cipher_setkey_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->key, 0, APICMD_CIPHER_SETKEY_LEN);
      memcpy(out->key, key, APICMD_CIPHER_SETKEY_LEN);
      out->key_bitlen = htonl(*key_bitlen);
      out->operation = htonl(*operation);

      TLS_DEBUG("[cipher_setkey]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_setkey]key_bitlen: %d\n", *key_bitlen);
      TLS_DEBUG("[cipher_setkey]operation: %d\n", *operation);

      *altcid = APICMDID_TLS_CIPHER_SETKEY;
      size = sizeof(struct apicmd_cipher_setkey_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t ciphersetiv_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_cipher_context_t *ctx = (FAR mbedtls_cipher_context_t *)arg[0];
  FAR const unsigned char *iv = (FAR const unsigned char *)arg[1];
  FAR size_t *iv_len = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_set_iv_s *out =
        (FAR struct apicmd_cipher_set_iv_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->iv, 0, APICMD_CIPHER_SET_IV_LEN);
      memcpy(out->iv, iv, *iv_len);
      out->iv_len = htonl(*iv_len);

      TLS_DEBUG("[cipher_set_iv]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_set_iv]iv_len: %zu\n", *iv_len);

      *altcid = APICMDID_TLS_CIPHER_SET_IV;
      size = sizeof(struct apicmd_cipher_set_iv_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t cipherupdate_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
  FAR const unsigned char *input = (FAR const unsigned char *)arg[1];
  FAR size_t *ilen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_update_s *out =
        (FAR struct apicmd_cipher_update_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      if (*ilen <= APICMD_CIPHER_UPDATE_INPUT_LEN)
        {
          memset(out->input, 0, APICMD_CIPHER_UPDATE_INPUT_LEN);
          memcpy(out->input, input, *ilen);
        }
      else
        {
          return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
        }

      out->ilen = htonl(*ilen);

      TLS_DEBUG("[cipher_update]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_update]ilen: %d\n", *ilen);

      *altcid = APICMDID_TLS_CIPHER_UPDATE;
      size = sizeof(struct apicmd_cipher_update_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t mdinfofromtype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_md_type_t *md_type = (FAR mbedtls_md_type_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_info_from_type_s *out =
        (FAR struct apicmd_md_info_from_type_s *)pktbuf;

      out->md_type = htonl(*md_type);

      *altcid = APICMDID_TLS_MD_INFO_FROM_TYPE;
      size = sizeof(struct apicmd_md_info_from_type_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_MD_INFO_FROM_TYPE);
      out->u.md_info_from_type.md_type = htonl(*md_type);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[md_info_from_type]md_type: %d\n", *md_type);

  return size;
}

static int32_t mdgetsize_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_md_info_t *md_info = (FAR mbedtls_md_info_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_get_size_s *out =
        (FAR struct apicmd_md_get_size_s *)pktbuf;

      out->md_info = htonl(md_info->id);

      *altcid = APICMDID_TLS_MD_GET_SIZE;
      size = sizeof(struct apicmd_md_get_size_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->md_info = htonl(md_info->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_MD_GET_SIZE);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[md_get_size]md_info id: %lu\n", md_info->id);

  return size;
}

static int32_t md_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_md_info_t *md_info =
    (FAR const mbedtls_md_info_t *)arg[0];
  FAR const unsigned char *input = (FAR const unsigned char *)arg[1];
  FAR size_t *ilen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_s *out =
        (FAR struct apicmd_md_s *)pktbuf;

      out->md_info = htonl(md_info->id);

      if (*ilen <= APICMD_MD_INPUT_LEN)
        {
          memset(out->input, 0, APICMD_MD_INPUT_LEN);
          memcpy(out->input, input, *ilen);
        }
      else
        {
          return MBEDTLS_ERR_MD_BAD_INPUT_DATA;
        }

      out->ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_MD;
      size = sizeof(struct apicmd_md_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->md_info = htonl(md_info->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_MD);
      if (*ilen <= APICMD_MD_INPUT_LEN)
        {
          memset(out->u.md.input, 0, APICMD_MD_INPUT_LEN);
          memcpy(out->u.md.input, input, *ilen);
        }
      else
        {
          return MBEDTLS_ERR_MD_BAD_INPUT_DATA;
        }

      out->u.md.ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  TLS_DEBUG("[md]md_info id: %lu\n", md_info->id);

  return size;
}

static int32_t mddigest_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_md_info_t *md_info =
    (FAR const mbedtls_md_info_t *)arg[0];
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[1];

  FAR struct apicmd_md_digest_s *out =
    (FAR struct apicmd_md_digest_s *)pktbuf;

  out->md_info = htonl(md_info->id);
  out->chain = htonl(chain->id);

  size = sizeof(struct apicmd_md_digest_s);

  TLS_DEBUG("[md_digest]md_info id: %lu\n", md_info->id);
  TLS_DEBUG("[md_digest]chain id: %lu\n", chain->id);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_TLS_MD_DIGEST;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t base64enc_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR size_t *dlen = (FAR size_t *)arg[0];
  FAR const unsigned char *src = (FAR const unsigned char *)arg[1];
  FAR size_t *slen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_base64_encode_s *out =
        (FAR struct apicmd_base64_encode_s *)pktbuf;

      memcpy(out->src, src, *slen);
      out->slen = htonl(*slen);
      out->dlen = htonl(*dlen);

      *altcid = APICMDID_TLS_BASE64_ENCODE;
      size = sizeof(struct apicmd_base64_encode_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_BASE64_ENCODE);
      memcpy(out->u.base64_encode.src, src, *slen);
      out->u.base64_encode.slen = htonl(*slen);
      out->u.base64_encode.dlen = htonl(*dlen);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t sha1_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const unsigned char *input = (FAR const unsigned char *)arg[0];
  FAR size_t *ilen = (FAR size_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_sha1_s *out =
        (FAR struct apicmd_sha1_s *)pktbuf;

      memcpy(out->input, input, *ilen);
      out->ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_SHA1;
      size = sizeof(struct apicmd_sha1_s);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_SHA1);
      memcpy(out->u.sha1.input, input, *ilen);
      out->u.sha1.ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t sslexportsrtpkeys_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_export_srtp_keys_s *out =
        (FAR struct apicmd_ssl_export_srtp_keys_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      TLS_DEBUG("[ssl_export_srtp_keys]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_EXPORT_SRTP_KEYS;
      size = sizeof(struct apicmd_ssl_export_srtp_keys_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t sslusesrtp_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_use_srtp_s *out =
        (FAR struct apicmd_ssl_use_srtp_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      TLS_DEBUG("[ssl_use_srtp]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_USE_SRTP;
      size = sizeof(struct apicmd_ssl_use_srtp_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t srtpprofile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_srtp_profile_s *out =
        (FAR struct apicmd_ssl_srtp_profile_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      TLS_DEBUG("[ssl_srtp_profile]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_SRTP_PROFILE;
      size = sizeof(struct apicmd_ssl_srtp_profile_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t sslturn_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR uint16_t *turn_channel = (FAR uint16_t *)arg[1];
  FAR uint32_t *peer_addr = (FAR uint32_t *)arg[2];
  FAR uint16_t *peer_port = (FAR uint16_t *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_turn_s *out =
        (FAR struct apicmd_ssl_turn_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->turn_channel = htons(*turn_channel);
      out->peer_addr = htonl(*peer_addr);
      out->peer_port = htons(*peer_port);

      TLS_DEBUG("[ssl_turn]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_TURN;
      size = sizeof(struct apicmd_ssl_turn_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t mpiinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_MPI_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_init_s *out =
        (FAR struct apicmd_mpi_init_s *)pktbuf;

      out->ctx = htonl(*id);

      TLS_DEBUG("[mpi_init]ctx id: %ld\n", *id);

      *altcid = APICMDID_TLS_MPI_INIT;
      size = sizeof(struct apicmd_mpi_init_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t mpifree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_mpi *X = (FAR mbedtls_mpi *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_free_s *out =
        (FAR struct apicmd_mpi_free_s *)pktbuf;

      out->ctx = htonl(X->id);

      TLS_DEBUG("[mpi_free]ctx id: %lu\n", X->id);

      *altcid = APICMDID_TLS_MPI_FREE;
      size = sizeof(struct apicmd_mpi_free_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t mpireadstr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  uint32_t buflen = 0;
  FAR mbedtls_mpi *X = (FAR mbedtls_mpi *)arg[0];
  FAR int *radix = (FAR int *)arg[1];
  FAR const char *s = (FAR const char *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_read_string_s *out =
        (FAR struct apicmd_mpi_read_string_s *)pktbuf;

      out->ctx = htonl(X->id);
      out->radix = htonl(*radix);
      memset(out->s, '\0', APICMD_MPI_READ_STRING_MPI_LEN);
      if (s != NULL)
        {
          buflen = strlen(s);
          if (buflen > APICMD_MPI_READ_STRING_MPI_LEN)
            {
              return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
            }
          memcpy(out->s, s, buflen);
        }
      else
        {
          return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[mpi_read_string]ctx id: %lu\n", X->id);
      TLS_DEBUG("[mpi_read_string]radix: %d\n", *radix);
      TLS_DEBUG("[mpi_read_string]s: %s\n", s);

      *altcid = APICMDID_TLS_MPI_READ_STRING;
      size = sizeof(struct apicmd_mpi_read_string_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t mpiwritestr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_mpi *X = (FAR const mbedtls_mpi *)arg[0];
  FAR int *radix = (FAR int *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_write_string_s *out =
        (FAR struct apicmd_mpi_write_string_s *)pktbuf;

      out->ctx = htonl(X->id);
      out->radix = htonl(*radix);
      if (*buflen > APICMD_MPI_WRITE_STRING_MPI_LEN)
        {
          return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
        }
      else
        {
          out->buflen = htonl(*buflen);
        }

      TLS_DEBUG("[mpi_write_string]ctx id: %lu\n", X->id);
      TLS_DEBUG("[mpi_write_string]radix: %d\n", *radix);
      TLS_DEBUG("[mpi_write_string]buflen: %d\n", *buflen);

      *altcid = APICMDID_TLS_MPI_WRITE_STRING;
      size = sizeof(struct apicmd_mpi_write_string_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509csrinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_X509_CSR_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_init_s *out =
        (FAR struct apicmd_x509_csr_init_s *)pktbuf;

      out->csr = htonl(*id);

      TLS_DEBUG("[x509_csr_init]csr id: %ld\n", *id);

      *altcid = APICMDID_TLS_X509_CSR_INIT;
      size = sizeof(struct apicmd_x509_csr_init_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509csrfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_free_s *out =
        (FAR struct apicmd_x509_csr_free_s *)pktbuf;

      out->csr = htonl(csr->id);

      TLS_DEBUG("[x509_csr_free]ctx id: %lu\n", csr->id);

      *altcid = APICMDID_TLS_X509_CSR_FREE;
      size = sizeof(struct apicmd_x509_csr_free_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509csrparsefile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
  FAR const char *path = (FAR const char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_file_s *out =
        (FAR struct apicmd_x509_csr_parse_file_s *)pktbuf;

      out->csr = htonl(csr->id);
      memset(out->path, '\0', APICMD_X509_CSR_PARSE_FILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char*) out->path, path,
            APICMD_X509_CSR_PARSE_FILE_PATH_LEN-1);
        }

      TLS_DEBUG("[x509_csr_parse_file]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_csr_parse_file]path: %s\n", path);

      *altcid = APICMDID_TLS_X509_CSR_PARSE_FILE;
      size = sizeof(struct apicmd_x509_csr_parse_file_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509csrparseder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_der_s *out =
        (FAR struct apicmd_x509_csr_parse_der_s *)pktbuf;

      out->csr = htonl(csr->id);
      *buflen = (*buflen <= APICMD_X509_CSR_PARSE_DER_LEN)
        ? *buflen : APICMD_X509_CSR_PARSE_DER_LEN;
      if (buf != NULL)
        {
          memcpy(out->buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(*buflen);

      TLS_DEBUG("[x509_csr_parse_der]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_csr_parse_der]buflen: %d\n", *buflen);

      *altcid = APICMDID_TLS_X509_CSR_PARSE_DER;
      size = sizeof(struct apicmd_x509_csr_parse_der_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509csrparse_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_s *out =
        (FAR struct apicmd_x509_csr_parse_s *)pktbuf;

      out->csr = htonl(csr->id);
      if (*buflen > APICMD_X509_CSR_PARSE_BUF_LEN)
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      memset(out->buf, '\0', APICMD_X509_CSR_PARSE_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(*buflen);

      TLS_DEBUG("[x509_csr_parse]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_csr_parse]buflen: %d\n", *buflen);

      *altcid = APICMDID_TLS_X509_CSR_PARSE;
      size = sizeof(struct apicmd_x509_csr_parse_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509dngetscrt_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  uint32_t buflen = 0;
  FAR char *buf = (FAR char *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR const mbedtls_x509_crt *crt = (FAR const mbedtls_x509_crt *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_crt_s *out =
        (FAR struct apicmd_x509_dn_gets_crt_s *)pktbuf;

      out->crt = htonl(crt->id);
      if (*size <= APICMD_X509_DN_GETS_CRT_BUF_LEN)
        {
          buflen = *size;
        }
      else
            {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      memset(out->buf, '\0', APICMD_X509_DN_GETS_CRT_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->size = htonl(buflen);

      TLS_DEBUG("[x509_dn_gets_crt]ctx id: %lu\n", crt->id);
      TLS_DEBUG("[x509_dn_gets_crt]size: %lu\n", buflen);

      *altcid = APICMDID_TLS_X509_DN_GETS_CRT;
      ret_size = sizeof(struct apicmd_x509_dn_gets_crt_s);
    }
  else
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

static int32_t x509dngetscsr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  uint32_t buflen = 0;
  FAR char *buf = (FAR char *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_csr_s *out =
        (FAR struct apicmd_x509_dn_gets_csr_s *)pktbuf;

      out->csr = htonl(csr->id);
      if (*size <= APICMD_X509_DN_GETS_CSR_BUF_LEN)
        {
          buflen = *size;
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      memset(out->buf, '\0', APICMD_X509_DN_GETS_CSR_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->size = htonl(buflen);

      TLS_DEBUG("[x509_dn_gets_csr]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_dn_gets_csr]size: %lu\n", buflen);

      *altcid = APICMDID_TLS_X509_DN_GETS_CSR;
      ret_size = sizeof(struct apicmd_x509_dn_gets_csr_s);
    }
  else
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

static int32_t x509writecrtinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = get_mbedtls_ctx_id(SSL_X509WRITE_CRT_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_init_s *out =
        (FAR struct apicmd_x509write_crt_init_s *)pktbuf;

      out->ctx = htonl(*id);

      TLS_DEBUG("[x509write_crt_init]ctx id: %ld\n", *id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_INIT;
      size = sizeof(struct apicmd_x509write_crt_init_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_free_s *out =
        (FAR struct apicmd_x509write_crt_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[x509write_crt_free]ctx id: %lu\n", ctx->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_FREE;
      size = sizeof(struct apicmd_x509write_crt_free_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  size_t req_buf_len = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR void *p_rng = (FAR void *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_der_s *out =
        (FAR struct apicmd_x509write_crt_der_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      req_buf_len = (*size <= APICMD_X509WRITE_CRT_DER_BUF_LEN)
        ? *size : APICMD_X509WRITE_CRT_DER_BUF_LEN;
      out->size = htonl(req_buf_len);

      TLS_DEBUG("[x509write_crt_der]config id: %lu\n", ctx->id);

      if (p_rng != NULL)
        {
          mbedtls_ctr_drbg_context *lctx = (mbedtls_ctr_drbg_context*)p_rng;
          uint32_t id = lctx->id;
          out->p_rng = htonl(id);
          TLS_DEBUG("[x509write_crt_der]p_rng(id): %lu\n", id);
        }

      *altcid = APICMDID_TLS_X509WRITE_CRT_DER;
      ret_size = sizeof(struct apicmd_x509write_crt_der_s);
    }
  else
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

static int32_t x509writecrtpem_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  size_t req_buf_len = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR void *p_rng = (FAR void *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_pem_s *out =
        (FAR struct apicmd_x509write_crt_pem_s *)pktbuf;

      out->crt = htonl(ctx->id);

      req_buf_len = (*size <= APICMD_X509WRITE_CRT_PEM_BUF_LEN)
        ? *size : APICMD_X509WRITE_CRT_PEM_BUF_LEN;
      out->size = htonl(req_buf_len);

      TLS_DEBUG("[x509write_crt_pem]config id: %lu\n", ctx->id);

      if (p_rng != NULL)
        {
          mbedtls_ctr_drbg_context *lctx = (mbedtls_ctr_drbg_context*)p_rng;
          uint32_t id = lctx->id;
          out->p_rng = htonl(id);
          TLS_DEBUG("[x509write_crt_pem]p_rng(id): %lu\n", id);
        }

      *altcid = APICMDID_TLS_X509WRITE_CRT_PEM;
      ret_size = sizeof(struct apicmd_x509write_crt_pem_s);
    }
  else
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

static int32_t x509writecrtsubkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR mbedtls_pk_context *key = (FAR mbedtls_pk_context *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_key_s *out =
        (FAR struct apicmd_x509write_crt_subject_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->key = htonl(key->id);

      TLS_DEBUG("[x509write_crt_subject_key]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_subject_key]key id: %lu\n", key->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SUBJECT_KEY;
      size = sizeof(struct apicmd_x509write_crt_subject_key_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtissuerkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR mbedtls_pk_context *key = (FAR mbedtls_pk_context *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_key_s *out =
        (FAR struct apicmd_x509write_crt_issuer_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->key = htonl(key->id);

      TLS_DEBUG("[x509write_crt_issuer_key]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_issuer_key]key id: %lu\n", key->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_ISSUER_KEY;
      size = sizeof(struct apicmd_x509write_crt_issuer_key_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtsubname_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int32_t buflen = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const char *issuer_name = (FAR const char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_name_s *out =
        (FAR struct apicmd_x509write_crt_subject_name_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->subject_name, '\0',
        APICMD_X509WRITE_CRT_SUBJECT_NAME_BUF_LEN);
      if (issuer_name != NULL)
        {
          buflen = strlen(issuer_name);
          if (buflen > APICMD_X509WRITE_CRT_SUBJECT_NAME_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->subject_name, issuer_name, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_subject_name]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_subject_name]subject_name: %s\n", issuer_name);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SUBJECT_NAME;
      size = sizeof(struct apicmd_x509write_crt_subject_name_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtissuername_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int32_t buflen = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const char *issuer_name = (FAR const char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_name_s *out =
        (FAR struct apicmd_x509write_crt_issuer_name_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->issuer_name, '\0', APICMD_X509WRITE_CRT_ISSUER_NAME_BUF_LEN);
      if (issuer_name != NULL)
        {
          buflen = strlen(issuer_name);
          if (buflen > APICMD_X509WRITE_CRT_ISSUER_NAME_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->issuer_name, issuer_name, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_issuer_name]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_issuer_name]issuer_name: %s\n",
        issuer_name);

      *altcid = APICMDID_TLS_X509WRITE_CRT_ISSUER_NAME;
      size = sizeof(struct apicmd_x509write_crt_issuer_name_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtver_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR int *version = (FAR int *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_version_s *out =
        (FAR struct apicmd_x509write_crt_version_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->version = htonl(*version);

      TLS_DEBUG("[x509write_crt_version]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_version]version: %d\n", *version);

      *altcid = APICMDID_TLS_X509WRITE_CRT_VERSION;
      size = sizeof(struct apicmd_x509write_crt_version_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtmdalg_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR mbedtls_md_type_t *md_alg = (FAR mbedtls_md_type_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_md_alg_s *out =
        (FAR struct apicmd_x509write_crt_md_alg_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->md_alg = htonl(*md_alg);

      TLS_DEBUG("[x509write_crt_md_alg]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_md_alg]md_alg: %d\n", *md_alg);

      *altcid = APICMDID_TLS_X509WRITE_CRT_MD_ALG;
      size = sizeof(struct apicmd_x509write_crt_md_alg_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtserial_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const mbedtls_mpi *serial = (FAR const mbedtls_mpi *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_serial_s *out =
        (FAR struct apicmd_x509write_crt_serial_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->serial = htonl(serial->id);

      TLS_DEBUG("[x509write_crt_serial]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_serial]serial id: %lu\n", serial->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SERIAL;
      size = sizeof(struct apicmd_x509write_crt_serial_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtvalidity_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int32_t buflen = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const char *not_before = (FAR const char *)arg[1];
  FAR const char *not_after = (FAR const char *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_validity_s *out =
        (FAR struct apicmd_x509write_crt_validity_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->not_before, '\0', APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN);
      if (not_before != NULL)
        {
          buflen = strlen(not_before);
          if (buflen > APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->not_before, not_before, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      memset(out->not_after, '\0', APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN);
      if (not_after != NULL)
        {
          buflen = strlen(not_after);
          if (buflen > APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->not_after, not_after, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_validity]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_validity]not_before: %s\n", not_before);
      TLS_DEBUG("[x509write_crt_validity]not_after: %s\n", not_after);

      *altcid = APICMDID_TLS_X509WRITE_CRT_VALIDITY;
      size = sizeof(struct apicmd_x509write_crt_validity_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtconst_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR int *is_ca = (FAR int *)arg[1];
  FAR int *max_pathlen = (FAR int *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_basic_constraints_s *out =
        (FAR struct apicmd_x509write_crt_basic_constraints_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->is_ca = htonl(*is_ca);
      out->max_pathlen = htonl(*max_pathlen);

      TLS_DEBUG("[x509write_crt_basic_constraints]ctx id: %lu\n",
        ctx->id);
      TLS_DEBUG("[x509write_crt_basic_constraints]is_ca: %d\n",
        *is_ca);
      TLS_DEBUG("[x509write_crt_basic_constraints]max_pathlen: %d\n",
        *max_pathlen);

      *altcid = APICMDID_TLS_X509WRITE_CRT_CONSTRAINTS;
      size = sizeof(struct apicmd_x509write_crt_basic_constraints_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtsubid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_key_identifier_s *out =
        (FAR struct apicmd_x509write_crt_subject_key_identifier_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[x509write_crt_subject_key_identifier]ctx id: %lu\n",
        ctx->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SUBJECT_ID;
      size = sizeof(struct apicmd_x509write_crt_subject_key_identifier_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtauthid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_authority_key_identifier_s *out =
        (FAR struct apicmd_x509write_crt_authority_key_identifier_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[x509write_crt_authority_key_identifier]ctx id: %lu\n",
        ctx->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_AUTHORITY_ID;
      size = sizeof(struct apicmd_x509write_crt_authority_key_identifier_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtkeyusage_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR unsigned int *key_usage = (FAR unsigned int *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_key_usage_s *out =
        (FAR struct apicmd_x509write_crt_key_usage_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->key_usage = htonl(*key_usage);

      TLS_DEBUG("[x509write_crt_key_usage]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_key_usage]key_usage: %d\n",
        *key_usage);

      *altcid = APICMDID_TLS_X509WRITE_CRT_KEY_USAGE;
      size = sizeof(struct apicmd_x509write_crt_key_usage_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t x509writecrtnscerttype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR unsigned char *ns_cert_type = (FAR unsigned char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_ns_cert_type_s *out =
        (FAR struct apicmd_x509write_crt_ns_cert_type_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->ns_cert_type = *ns_cert_type;

      TLS_DEBUG("[x509write_crt_ns_cert_type]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_ns_cert_type]ns_cert_type: %d\n",
        *ns_cert_type);

      *altcid = APICMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE;
      size = sizeof(struct apicmd_x509write_crt_ns_cert_type_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t rsainit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int *padding = (FAR int *)arg[0];
  FAR int *hash_id = (FAR int *)arg[1];
  FAR int32_t *id = (FAR int32_t *)arg[2];

  *id = get_mbedtls_ctx_id(SSL_RSA_CTX);

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_init_s *out =
        (FAR struct apicmd_rsa_init_s *)pktbuf;

      out->ctx = htonl(*id);
      out->padding = htonl(*padding);
      out->hash_id = htonl(*hash_id);
    
      TLS_DEBUG("[rsa_init]ctx id: %ld\n", *id);
      TLS_DEBUG("[rsa_init]padding: %d\n", *padding);
      TLS_DEBUG("[rsa_init]hash_id: %d\n", *hash_id);

      *altcid = APICMDID_TLS_RSA_INIT;
      size = sizeof(struct apicmd_rsa_init_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t rsafree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_rsa_context *ctx = (FAR mbedtls_rsa_context *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_free_s *out =
        (FAR struct apicmd_rsa_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[rsa_free]ctx id: %lu\n", ctx->id);

      *altcid = APICMDID_TLS_RSA_FREE;
      size = sizeof(struct apicmd_rsa_free_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t rsagenkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_rsa_context *ctx = (FAR mbedtls_rsa_context *)arg[0];
  FAR void *p_rng = (FAR void *)arg[2];
  FAR unsigned int *nbits = (FAR unsigned int *)arg[3];
  FAR int *exponent = (FAR int *)arg[4];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_gen_key_s *out =
        (FAR struct apicmd_rsa_gen_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->nbits = htonl(*nbits);
      out->exponent = htonl(*exponent);

      TLS_DEBUG("[rsa_gen_key]config id: %lu\n", ctx->id);
      TLS_DEBUG("[rsa_gen_key]nbits: %d\n", *nbits);
      TLS_DEBUG("[rsa_gen_key]exponent: %d\n", *exponent);

      if (p_rng != NULL)
        {
          mbedtls_ctr_drbg_context *lctx = (mbedtls_ctr_drbg_context*)p_rng;
          uint32_t id = lctx->id;
          out->p_rng = htonl(id);
          TLS_DEBUG("[rsa_gen_key]p_rng(id): %lu\n", id);
        }

      *altcid = APICMDID_TLS_RSA_GEN_KEY;
      size = sizeof(struct apicmd_rsa_gen_key_s);
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t sslinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_initres_s *in =
        (FAR struct apicmd_ssl_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_freeres_s *in =
        (FAR struct apicmd_ssl_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslsetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_setupres_s *in =
        (FAR struct apicmd_ssl_setupres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_setup res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslhostname_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_hostnameres_s *in =
        (FAR struct apicmd_ssl_hostnameres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_hostname res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslbio_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_biores_s *in =
        (FAR struct apicmd_ssl_biores_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_bio res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslhandshake_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_handshakeres_s *in =
        (FAR struct apicmd_ssl_handshakeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_handshake res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslwrite_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_writeres_s *in =
        (FAR struct apicmd_ssl_writeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_write res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslread_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *len = (FAR size_t *)arg[2];

  req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
    ? *len : APICMD_SSL_READ_BUF_LEN;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_readres_s *in =
        (FAR struct apicmd_ssl_readres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_read res]ret: %ld\n", *ret);
      if (*ret <= 0)
        {
          /* Nothing to do */
        }
      else if ((0 < *ret) && (*ret <= req_buf_len))
        {
          memcpy(buf, in->buf, *ret);
        }
      else
        {
          TLS_ERROR("Unexpected buffer length: %ld\n", *ret);
          return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
        }
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslclosenotif_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_close_notifyres_s *in =
        (FAR struct apicmd_ssl_close_notifyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_close_notify res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslver_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR const char **version = (FAR const char **)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_versionres_s *in =
        (FAR struct apicmd_ssl_versionres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_version res]ret: %ld\n", *ret);
      in->version[APICMD_SSL_VERSION_LEN-1] = '\0';
      *version = get_ssl_tls_version(in->version);
      TLS_DEBUG("[ssl_version res]version: %s\n", *version);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslciphersuite_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR char *cipher_str_buf = (FAR char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuiteres_s *in =
        (FAR struct apicmd_ssl_ciphersuiteres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_ciphersuite res]ret: %ld\n", *ret);

      memset(cipher_str_buf, '\0', SSL_CIPHER_STR_BUF);
      strncpy((char *)cipher_str_buf, (const char *)in->ciphersuite,
        SSL_CIPHER_STR_BUF-1);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslciphersuiteid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  int32_t  ret_cipher_id = 0;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuite_idres_s *in =
        (FAR struct apicmd_ssl_ciphersuite_idres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      ret_cipher_id = ntohl(in->id);
      TLS_DEBUG("[ssl_ciphersuite_id res]ret: %ld\n", *ret);
      TLS_DEBUG("[ssl_ciphersuite_id res]id: %ld\n", ret_cipher_id);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return ret_cipher_id;
}

static int32_t sslrecexp_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_record_expansionres_s *in =
        (FAR struct apicmd_ssl_record_expansionres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_record_expansion res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslvrfyresult_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_verify_resultres_s *in =
        (FAR struct apicmd_ssl_verify_resultres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_verify_result res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t ssltimercb_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_timer_cbres_s *in =
        (FAR struct apicmd_ssl_timer_cbres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_timer_cb res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslpeercert_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_x509_crt *x509_crt = (FAR mbedtls_x509_crt *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_peer_certres_s *in =
        (FAR struct apicmd_ssl_peer_certres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      x509_crt->id = ntohl(in->crt);
      TLS_DEBUG("[ssl_peer_cert res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sslbytesavail_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_bytes_availres_s *in =
        (FAR struct apicmd_ssl_bytes_availres_s *)pktbuf;

      *ret = ntohl( in->avail_bytes);
      TLS_DEBUG("[ssl_bytes_avail res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_initres_s *in =
        (FAR struct apicmd_config_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t conffree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_freeres_s *in =
        (FAR struct apicmd_config_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confdefauts_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_defaultsres_s *in =
        (FAR struct apicmd_config_defaultsres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_defaults res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confauth_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_authmoderes_s *in =
        (FAR struct apicmd_config_authmoderes_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_authmode res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confrng_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_rngres_s *in =
        (FAR struct apicmd_config_rngres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_rng res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confcachain_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ca_chainres_s *in =
        (FAR struct apicmd_config_ca_chainres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_ca_chain res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confowncert_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_own_certres_s *in =
        (FAR struct apicmd_config_own_certres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_own_cert res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confreadtimeo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_read_timeoutres_s *in =
        (FAR struct apicmd_config_read_timeoutres_s *)pktbuf;

      *ret = ntohl( in->ret_code);
      TLS_DEBUG("[config_read_timeout res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confvrfy_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_verifyres_s *in =
        (FAR struct apicmd_config_verifyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_verify res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confvrfycb_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR uint32_t *crt = (FAR uint32_t *)arg[0];
  FAR int32_t *depth = (FAR int32_t *)arg[1];

  FAR struct apicmd_config_verify_callbackres_s *in =
    (FAR struct apicmd_config_verify_callbackres_s *)pktbuf;

  *crt = htonl(in->crt);
  *depth = htonl(in->certificate_depth);

  return 0;
}

static int32_t confalpnproto_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_alpn_protocolsres_s *in =
        (FAR struct apicmd_config_alpn_protocolsres_s *)pktbuf;

      *ret = ntohl( in->ret_code);
      TLS_DEBUG("[config_alpn_protocols res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t confciphersuites_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ciphersuitesres_s *in =
        (FAR struct apicmd_config_ciphersuitesres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_ciphersuites res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t ssesioninit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_initres_s *in =
        (FAR struct apicmd_session_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sessionfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_freeres_s *in =
        (FAR struct apicmd_session_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sessionget_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_getres_s *in =
        (FAR struct apicmd_session_getres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_get res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t sessionset_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_setres_s *in =
        (FAR struct apicmd_session_setres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_set res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t x509crtinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_initres_s *in =
        (FAR struct apicmd_x509_crt_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t x509crtfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_freeres_s *in =
        (FAR struct apicmd_x509_crt_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t x509crtparsefile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_fileres_s *in =
        (FAR struct apicmd_x509_crt_parse_fileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_parse_file res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t x509crtparseder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_derres_s *in =
        (FAR struct apicmd_x509_crt_parse_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_parse_der res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t x509crtparse_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parseres_s *in =
        (FAR struct apicmd_x509_crt_parseres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_parse res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t x509crtinfo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  uint32_t buflen = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];
  FAR struct apicmd_x509_crt_infores_s *in =
    (FAR struct apicmd_x509_crt_infores_s *)pktbuf;

  buflen = (*size <= APICMD_X509_CRT_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_INFO_RET_BUF_LEN;

  *ret = ntohl(in->ret_code);
  *ret = (*ret <= buflen) ? *ret : buflen;
  memcpy(buf, in->buf, *ret);

  TLS_DEBUG("[x509_crt_info res]ret: %ld\n", *ret);

  return 0;
}

static int32_t x509crtvrfyinfo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  uint32_t buflen = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  buflen = (*size <= APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_verify_infores_s *in =
        (FAR struct apicmd_x509_crt_verify_infores_s *)pktbuf;

      *ret = ntohl( in->ret_code);
      *ret = (*ret <= buflen) ? *ret : buflen;
      memcpy(buf, in->buf, *ret);
      TLS_DEBUG("[x509_crt_verify_info res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t pkinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_initres_s *in =
        (FAR struct apicmd_pk_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }

  return 0;
}

static int32_t pkfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_freeres_s *in =
        (FAR struct apicmd_pk_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }

  return 0;
}

static int32_t pkparsekeyfile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_keyfileres_s *in =
        (FAR struct apicmd_pk_parse_keyfileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_parse_keyfile res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }

  return 0;
}

static int32_t pkparsekey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_keyres_s *in =
        (FAR struct apicmd_pk_parse_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_parse_key res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }

  return 0;
}

static int32_t pkcheckpair_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_check_pairres_s *in =
        (FAR struct apicmd_pk_check_pairres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_check_pair res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }

  return 0;
}

static int32_t pksetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_setupres_s *in =
        (FAR struct apicmd_pk_setupres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_setup res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }

  return 0;
}

static int32_t pkinfofromtype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_pk_info_t *pk_info = (FAR mbedtls_pk_info_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_info_from_typeres_s *in =
        (FAR struct apicmd_pk_info_from_typeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          pk_info->id = ntohl(in->pk_info);
        }
      TLS_DEBUG("[pk_info_from_type res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_PK_INFO_FROM_TYPE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }

      pk_info->id = ntohl(in->u.info_from_typeres.pk_info);
    }

  return 0;
}

static int32_t pkwritekeypem_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
    ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_pemres_s *in =
        (FAR struct apicmd_pk_write_key_pemres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          memcpy(buf, in->buf, req_buf_len);
        }
      TLS_DEBUG("[pk_write_key_pem res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_PK_WRITE_KEY_PEM)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }

      memcpy(buf, in->u.write_key_pemres.buf, req_buf_len);
    }

  return 0;
}

static int32_t pkwritekeyder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_derres_s *in =
        (FAR struct apicmd_pk_write_key_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_write_key_der res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
    ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;

  if (*ret <= 0)
    {
      /* Nothing to do */
    }
  else if ((0 < *ret) && (*ret <= req_buf_len))
    {
      FAR struct apicmd_pk_write_key_derres_s *in =
        (FAR struct apicmd_pk_write_key_derres_s *)pktbuf;

        memcpy(buf, in->buf, *ret);
    }
  else
    {
      TLS_ERROR("Unexpected buffer length: %ld\n", *ret);
      return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  return 0;
}

static int32_t pkrsa_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_rsa_context *rsa_context =
    (FAR mbedtls_rsa_context *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_rsares_s *in =
        (FAR struct apicmd_pk_rsares_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          rsa_context->id = ntohl(in->rsa);
        }
      TLS_DEBUG("[pk_rsa res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t ctrdrbginit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_initres_s *in =
        (FAR struct apicmd_ctr_drbg_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctr_drbg_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t ctrdrbgfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_freeres_s *in =
        (FAR struct apicmd_ctr_drbg_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctr_drbg_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t ctrdrbgseed_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_seedres_s *in =
        (FAR struct apicmd_ctr_drbg_seedres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctr_drbg_seed res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t entropyinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_entropy_initres_s *in =
        (FAR struct apicmd_entropy_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[entropy_init res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t entropyfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_entropy_freeres_s *in =
        (FAR struct apicmd_entropy_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[entropy_free res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }

  return 0;
}

static int32_t cipherinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_initres_s *in =
        (FAR struct apicmd_cipher_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_init res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t cipherfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_freeres_s *in =
        (FAR struct apicmd_cipher_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_free res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t cipherinfofromstr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_cipher_info_t *cipher_info =
    (FAR mbedtls_cipher_info_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_info_from_stringres_s *in =
        (FAR struct apicmd_cipher_info_from_stringres_s *)pktbuf;

      *ret = ntohl(in->cipher_info);
      cipher_info->id = *ret;

      TLS_DEBUG("[cipher_info_from_string res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t ciphersetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setupres_s *in =
        (FAR struct apicmd_cipher_setupres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_setup res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t ciphersetkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setkeyres_s *in =
        (FAR struct apicmd_cipher_setkeyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_setkey res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t ciphersetiv_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_set_ivres_s *in =
        (FAR struct apicmd_cipher_set_ivres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_set_iv res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t cipherupdate_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *output = (FAR unsigned char *)arg[1];
  FAR size_t *olen = (FAR size_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_updateres_s *in =
        (FAR struct apicmd_cipher_updateres_s *)pktbuf;
      size_t len = ntohl(in->olen);

      *ret = ntohl(in->ret_code);
      if (*olen < len)
        {
          *ret = MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
        }
      else
        {
          memcpy(output, in->output, len);
        }

      *olen = len;

      TLS_DEBUG("[cipher_update res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t mdinfofromtype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_md_info_t *md_info = (FAR mbedtls_md_info_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_info_from_typeres_s *in =
        (FAR struct apicmd_md_info_from_typeres_s *)pktbuf;

      *ret = ntohl(in->md_info);
      md_info->id = *ret;
      TLS_DEBUG("[md_info_from_type res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_MD_INFO_FROM_TYPE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      *ret = ntohl(in->u.md_info_from_typeres.md_info);
      md_info->id = *ret;
    }

  return 0;
}

static int32_t mdgetsize_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR uint8_t *md_size = (FAR uint8_t *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_get_sizeres_s *in =
        (FAR struct apicmd_md_get_sizeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      *md_size = in->md_size;

      TLS_DEBUG("[md_get_size res]ret: %ld\n", *ret);
      TLS_DEBUG("[md_get_size res]md_size: %d\n", (int) in->md_size);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_MD_GET_SIZE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      *md_size = in->u.md_get_sizeres.md_size;

      TLS_DEBUG("[md_get_size res]ret: %ld\n", *ret);
      TLS_DEBUG("[md_get_size res]md_size: %d\n",
        (int)in->u.md_get_sizeres.md_size);
    }

  return 0;
}

static int32_t md_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *output = (FAR unsigned char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mdres_s *in =
        (FAR struct apicmd_mdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      memcpy(output, in->output, APICMD_MD_OUTPUT_LEN);
      TLS_DEBUG("[md res]ret: %ld\n", *ret);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_MD)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return *ret;
        }

      memcpy(output, in->u.mdres.output, APICMD_MD_OUTPUT_LEN_V4);
    }

  return 0;
}

static int32_t mddigest_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_digestres_s *in =
        (FAR struct apicmd_md_digestres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      memcpy(buf, in->output, APICMD_MD_DIGEST_OUTPUT_LEN);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      memcpy(buf, in->u.md_digestres.output, APICMD_MD_DIGEST_OUTPUT_LEN);
    }

  TLS_DEBUG("[md_digest res]ret: %ld\n", *ret);

  return 0;
}

static int32_t base64enc_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  uint32_t out_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR char *buf = (FAR char *)arg[1];
  FAR size_t *olen = (FAR size_t *)arg[2];
  FAR size_t *dlen = (FAR size_t *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_base64_encoderes_s *in =
        (FAR struct apicmd_base64_encoderes_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      out_len = ntohl(in->olen);
      if (out_len <= *dlen)
        {
          memcpy(buf, in->dst, out_len);
          *olen = out_len;
        }
      else
        {
          TLS_ERROR("Unexpected output length: %lu\n", out_len);
          *olen = 0;
          return -1;
        }
      TLS_DEBUG("[base64_encode res]ret: %ld\n", *ret);
      TLS_DEBUG("[base64_encode res]length: %lu\n", out_len);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_BASE64_ENCODE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      out_len = ntohl(in->u.base64_encoderes.olen);
      if (out_len <= *dlen)
        {
          memcpy(buf, in->u.base64_encoderes.dst, out_len);
          *olen = out_len;
        }
      else
        {
          TLS_ERROR("Unexpected output length: %lu\n", out_len);
          *olen = 0;
          return -1;
        }
      TLS_DEBUG("[base64_encode res]ret: %ld\n", *ret);
      TLS_DEBUG("[base64_encode res]length: %lu\n", out_len);
    }

  return 0;
}

static int32_t sha1_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *output = (FAR unsigned char *)arg[1];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_sha1res_s *in =
        (FAR struct apicmd_sha1res_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      memcpy(output, in->output, APICMD_SHA1_OUTPUT_LEN);
      TLS_DEBUG("[sha1 res]ret: %ld\n", *ret);

    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_SHA1)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      memcpy(output, in->u.sha1res.output, APICMD_SHA1_OUTPUT_LEN);
    }

  return 0;
}

static int32_t sslexportsrtpkeys_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  uint32_t buflen = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR uint16_t *size = (FAR uint16_t*)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_export_srtp_keysres_s *in =
        (FAR struct apicmd_ssl_export_srtp_keysres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      buflen = (*size < APICMD_EXPORT_SRTP_KEY_LEN) ?
        *size : APICMD_EXPORT_SRTP_KEY_LEN;
      memcpy(buf, in->key, buflen);

      TLS_DEBUG("[ssl_export_srtp_keys res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t sslusesrtp_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_use_srtpres_s *in =
        (FAR struct apicmd_ssl_use_srtpres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[ssl_use_srtp res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t srtpprofile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  int32_t profile = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_srtp_profileres_s *in =
        (FAR struct apicmd_ssl_srtp_profileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      profile = ntohl(in->profile);

      TLS_DEBUG("[ssl_srtp_profile res]ret: %ld\n", *ret);
      TLS_DEBUG("[ssl_srtp_profile res]profile: %ld\n", profile);
    }

  return profile;
}

static int32_t sslturn_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_turn_res_s *in =
        (FAR struct apicmd_ssl_turn_res_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[ssl_turn res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t mpiinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_initres_s *in =
        (FAR struct apicmd_mpi_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[mpi_init res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t mpifree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_freeres_s *in =
        (FAR struct apicmd_mpi_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[mpi_free res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t mpireadstr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_read_stringres_s *in =
        (FAR struct apicmd_mpi_read_stringres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[mpi_read_string res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t mpiwritestr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR char *buf = (FAR char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];
  FAR size_t *olen = (FAR size_t *)arg[3];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_write_stringres_s *in =
        (FAR struct apicmd_mpi_write_stringres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      *olen = ntohl(in->olen);
      if (*buflen < *olen)
        {
          *ret = MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL;
        }
      else
        {
          memcpy(buf, in->buf, *olen);
        }

      TLS_DEBUG("[mpi_write_string res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509csrinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_initres_s *in =
        (FAR struct apicmd_x509_csr_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_init res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509csrfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_freeres_s *in =
        (FAR struct apicmd_x509_csr_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_free res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509csrparsefile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_fileres_s *in =
        (FAR struct apicmd_x509_csr_parse_fileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_parse_file res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509csrparseder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_derres_s *in =
        (FAR struct apicmd_x509_csr_parse_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_parse_der res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509csrparse_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parseres_s *in =
        (FAR struct apicmd_x509_csr_parseres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_parse res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509dngetscrt_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_crtres_s *in =
        (FAR struct apicmd_x509_dn_gets_crtres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_dn_gets_crt res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509dngetscsr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_csrres_s *in =
        (FAR struct apicmd_x509_dn_gets_csrres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_dn_gets_csr res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_initres_s *in =
        (FAR struct apicmd_x509write_crt_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_init res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_freeres_s *in =
        (FAR struct apicmd_x509write_crt_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_free res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  req_buf_len = (*size <= APICMD_X509WRITE_CRT_DER_BUF_LEN)
    ? *size : APICMD_X509WRITE_CRT_DER_BUF_LEN;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_derres_s *in =
        (FAR struct apicmd_x509write_crt_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret <= 0)
        {
          /* Nothing to do */
        }
      else if ((0 < *ret) && (*ret <= req_buf_len))
        {
          memcpy(buf, in->buf, *ret);
        }
      else
        {
          TLS_ERROR("Unexpected buffer length: %ld\n", *ret);
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_der res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtpem_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  req_buf_len = (*size <= APICMD_X509WRITE_CRT_PEM_BUF_LEN)
    ? *size : APICMD_X509WRITE_CRT_PEM_BUF_LEN;

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_pemres_s *in =
        (FAR struct apicmd_x509write_crt_pemres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          memcpy(buf, in->buf, req_buf_len);
        }

      TLS_DEBUG("[x509write_crt_pem res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtsubkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_keyres_s *in =
        (FAR struct apicmd_x509write_crt_subject_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_subject_key res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtissuerkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_keyres_s *in =
        (FAR struct apicmd_x509write_crt_issuer_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_issuer_key res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtsubname_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_nameres_s *in =
        (FAR struct apicmd_x509write_crt_subject_nameres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_subject_name res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtissuername_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_nameres_s *in =
        (FAR struct apicmd_x509write_crt_issuer_nameres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_issuer_name res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtver_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_versionres_s *in =
        (FAR struct apicmd_x509write_crt_versionres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_version res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtmdalg_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_md_algres_s *in =
        (FAR struct apicmd_x509write_crt_md_algres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_md_alg res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtserial_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_serialres_s *in =
        (FAR struct apicmd_x509write_crt_serialres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_serial res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtvalidity_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_validityres_s *in =
        (FAR struct apicmd_x509write_crt_validityres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_validity res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtconst_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_basic_constraintsres_s *in =
        (FAR struct apicmd_x509write_crt_basic_constraintsres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_basic_constraints res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtsubid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_key_identifierres_s *in =
        (FAR struct apicmd_x509write_crt_subject_key_identifierres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_subject_key_identifier res]ret: %ld\n",
        *ret);
    }

  return 0;
}

static int32_t x509writecrtauthid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_authority_key_identifierres_s *in =
        (FAR struct apicmd_x509write_crt_authority_key_identifierres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_authority_key_identifier res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtkeyusage_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_key_usageres_s *in =
        (FAR struct apicmd_x509write_crt_key_usageres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_key_usage res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509writecrtnscerttype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_ns_cert_typeres_s *in =
        (FAR struct apicmd_x509write_crt_ns_cert_typeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_ns_cert_type res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t rsainit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_initres_s *in =
        (FAR struct apicmd_rsa_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[rsa_init res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t rsafree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_freeres_s *in =
        (FAR struct apicmd_rsa_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[rsa_free res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t rsagenkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_gen_keyres_s *in =
        (FAR struct apicmd_rsa_gen_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[rsa_gen_key res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t sslcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret_code = (FAR int32_t *)arg[0];
  int32_t ret = 0;

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      ret = -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmdres_s *in =
        (FAR struct apicmd_sslcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);
      TLS_DEBUG("[sslcmd_pkt_parse res]ret: %ld\n", ntohl(in->ret_code));

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_SSL_BYTES_AVAIL)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",subcmd_id);
          return -1;
        }
      if (subcmd_id == APISUBCMDID_TLS_SSL_READ)
        {
          size_t req_buf_len = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *len = (FAR size_t *)arg[2];

          req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
            ? *len : APICMD_SSL_READ_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          if (*ret_code <= 0)
            {
              /* Nothing to do */
            }
          else if ((0 < *ret_code) && (*ret_code <= req_buf_len))
            {
              memcpy(buf, in->u.readres.buf, *ret_code);
            }
          else
            {
              TLS_ERROR("Unexpected buffer length: %ld\n", *ret_code);
              return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
            }

          TLS_DEBUG("[ssl_read res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_VERSION)
        {
          FAR const char **version = (FAR const char **)arg[1];

          *ret_code = ntohl(in->ret_code);

          TLS_DEBUG("[ssl_version res]ret: %ld\n", *ret_code);
          TLS_DEBUG("[ssl_version res]version: %s\n",
            in->u.versionres.version);

          in->u.versionres.version[APICMD_SSL_VERSION_LEN-1] = '\0';
          *version = get_ssl_tls_version(in->u.versionres.version);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_PEER_CERT)
        {
          FAR mbedtls_x509_crt *x509_crt = (FAR mbedtls_x509_crt *)arg[1];

          *ret_code = ntohl(in->ret_code);

          x509_crt->id = ntohl(in->u.peer_certres.crt);

          TLS_DEBUG("[ssl_peer_cert res]ret: %ld\n", ntohl(in->ret_code));

        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_BYTES_AVAIL)
        {
          *ret_code = ntohl( in->u.bytes_availres.avail_bytes);
          TLS_DEBUG("[ssl_bytes_avail res]avail_bytes: %ld\n", *ret_code);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_CIPHERSUITE)
        {
          FAR char *cipher_str_buf = (FAR char *)arg[1];

          *ret_code = ntohl(in->ret_code);
          TLS_DEBUG("[ssl_ciphersuite res]ret: %ld\n", *ret_code);

          memset(cipher_str_buf, '\0', SSL_CIPHER_STR_BUF);
          strncpy((char *)cipher_str_buf,
            (const char *)in->u.ciphersuiteres.ciphersuite,
            SSL_CIPHER_STR_BUF-1);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_CIPHERSUITE_ID)
        {
          *ret_code = ntohl(in->ret_code);
          ret = ntohl(in->u.ciphersuite_idres.id);

          TLS_DEBUG("[ssl_ciphersuite_id res]ret: %ld\n", *ret_code);
          TLS_DEBUG("[ssl_ciphersuite_id res]id: %ld\n", ret);
        }
      else
        {
          *ret_code = ntohl(in->ret_code);
        }
    }

  return ret;
}

static int32_t configcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmdres_s *in =
        (FAR struct apicmd_configcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[configcmd_pkt_parse res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t sessioncmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmdres_s *in =
        (FAR struct apicmd_sessioncmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[sessioncmd_pkt_parse res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t x509crtcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret_code = (FAR int32_t *)arg[0];
  int32_t ret = 0;

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      ret = -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmdres_s *in =
        (FAR struct apicmd_x509_crtcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);
      TLS_DEBUG("[x509crtcmd_pkt_parse res]ret: %ld\n", ntohl(in->ret_code));

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_X509_CRT_VERIFY_INFO)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(subcmd_id));
          return -1;
        }

      if (subcmd_id == APISUBCMDID_TLS_X509_CRT_VERIFY_INFO)
        {
          uint32_t buflen = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *size = (FAR size_t *)arg[2];

          buflen = (*size <= APICMD_X509_CRT_INFO_RET_BUF_LEN)
            ? *size : APICMD_X509_CRT_INFO_RET_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          *ret_code = (*ret_code <= buflen) ? *ret_code : buflen;
          memcpy(buf, in->u.verify_infores.buf, *ret_code);

          TLS_DEBUG("[x509_crt_verify_info res]ret: %lu\n", *ret_code);
        }
      else
        {
          *ret_code = ntohl(in->ret_code);
        }
    }

  return ret;
}

static int32_t pkcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret_code = (FAR int32_t *)arg[0];
  int32_t ret = 0;

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      ret = -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);
      TLS_DEBUG("[pkcmd_pkt_parse res]ret: %ld\n", ntohl(in->ret_code));

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_PK_RSA)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n", subcmd_id);
          return -1;
        }
      if (subcmd_id == APISUBCMDID_TLS_PK_INFO_FROM_TYPE)
        {
          FAR mbedtls_pk_info_t *pk_info = (FAR mbedtls_pk_info_t *)arg[1];

          *ret_code = ntohl(in->ret_code);
          if (*ret_code == 0)
            {
              pk_info->id = ntohl(in->u.info_from_typeres.pk_info);
            }
          TLS_DEBUG("[pk_info_from_type res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_PK_WRITE_KEY_PEM)
        {
          size_t req_buf_len = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *size = (FAR size_t *)arg[2];

          req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
            ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          memcpy(buf, in->u.write_key_pemres.buf, req_buf_len);

          TLS_DEBUG("[pk_write_key_pem res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_PK_WRITE_KEY_DER)
        {
          size_t req_buf_len = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *size = (FAR size_t *)arg[2];

          req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
            ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          if (*ret_code <= 0)
            {
              /* Nothing to do */
            }
          else if ((0 < *ret_code) && (*ret_code <= req_buf_len))
            {
              memcpy(buf, in->u.write_key_derres.buf, *ret_code);
            }
          else
            {
              TLS_ERROR("Unexpected buffer length: %ld\n", *ret_code);
              return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
            }

          TLS_DEBUG("[pk_write_key_der res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_PK_RSA)
        {
          FAR mbedtls_rsa_context *rsa_context =
            (FAR mbedtls_rsa_context *)arg[1];

          *ret_code = ntohl(in->ret_code);
          rsa_context->id = ntohl(in->u.rsares.rsa);

          TLS_DEBUG("[pk_rsa res]ret: %ld\n", ntohl(in->ret_code));
        }
      else
        {
          *ret_code = ntohl(in->ret_code);
        }
    }

  return ret;
}

static int32_t ctrdrbgcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmdres_s *in =
        (FAR struct apicmd_ctr_drbgcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctrdrbgcmd_pkt_parse res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t entropycmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_entropycmdres_s *in =
        (FAR struct apicmd_entropycmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[entropycmd_pkt_parse res]ret: %ld\n", *ret);
    }

  return 0;
}

static int32_t ciphercmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  int32_t ret = 0;

  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      ret = -1;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_SHA1)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n", subcmd_id);
          return -1;
        }
      if (subcmd_id == APISUBCMDID_TLS_MD_INFO_FROM_TYPE)
        {
          ret = mdinfofromtype_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                         bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_MD_GET_SIZE)
        {
          ret = mdgetsize_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                    bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_MD)
        {
          ret = md_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen, bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_MD_DIGEST)
        {
          ret = mddigest_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                   bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_BASE64_ENCODE)
        {
          ret = base64enc_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                    bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SHA1)
        {
          ret = sha1_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen, bitmap);
          TLS_DEBUG("[sha1 res]ret: %ld\n", ntohl(in->ret_code));
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  if (altver == ALTCOM_VER4)
    {
      /* Change the command ID to Version 1 */

      altcid = convert_cid2v1(altcid);
      if (altcid == APICMDID_UNKNOWN)
        {
          return NULL;
        }
    }

  for (i = 0; i < ARRAY_SZ(g_parsehdlrs); i++)
    {
      if (g_parsehdlrs[i].altcid == altcid)
        {
          ret = g_parsehdlrs[i].hdlr;
        }
    }

  return ret;
}
