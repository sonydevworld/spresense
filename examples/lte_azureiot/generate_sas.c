/****************************************************************************
 * generate_sas_token.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <mbedtls/md.h>
#include "netutils/urldecode.h"
#include "netutils/base64.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#define LOGERR(format, ...)   printf(format, ##__VA_ARGS__)
#else
#define LOGERR(format, ...)
#endif

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Public functions
 ****************************************************************************/

int generate_sas_token(char       *sas,
                       size_t      size,
                       const char *url,
                       const char *key,
                       char       *policy,
                       uint32_t    expiry)
{
  /* URL encode */

  char urlencoded_url[128];
  int  urlencoded_url_len = sizeof(urlencoded_url);

  urlencode(url, strlen(url), urlencoded_url, &urlencoded_url_len);

  /* Key base64 decode */

  char    decodec_key[128];
  size_t  decodec_key_len = sizeof(decodec_key);

  if (base64_decode((const unsigned char *)key,
                    strlen(key),
                    (unsigned char *)decodec_key,
                    &decodec_key_len) == NULL)
    {
      LOGERR("Failed: base64_decode()\n");

      return ERROR;
    }


  char  sign_key[400];
  int   sign_key_len = sizeof(sign_key);

  expiry += time(NULL);

  snprintf(sign_key, sign_key_len, "%s\n%d", urlencoded_url, expiry);

  /* Create HMAC hash */

  mbedtls_md_context_t  ctx;
  char                  out_hmac_hash[MBEDTLS_MD_MAX_SIZE];
  int                   ret;

  mbedtls_md_init(&ctx);

  if ((ret = mbedtls_md_setup(&ctx,
                              mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                              1/*hmac*/)) != 0)
    {
      LOGERR("Failed: mbedtls_md_setup() == %d\n", ret);
    }                           
  else if ((ret = mbedtls_md_hmac_starts(&ctx,
                                         (const unsigned char *)decodec_key,
                                         decodec_key_len)) !=0)
    {
      LOGERR("Failed: mbedtls_md_hmac_starts() == %d\n", ret);
    }
  else if ((ret = mbedtls_md_hmac_update(&ctx,
                                         (const unsigned char *)sign_key,
                                         strlen(sign_key))) != 0)
    {
      LOGERR("Failed: mbedtls_md_hmac_update() == %d\n", ret);
    }
  else if ((ret = mbedtls_md_hmac_finish(&ctx,
                         (unsigned char *)out_hmac_hash)) != 0)
    {
      LOGERR("Failed: mbedtls_md_hmac_finish() == %d\n", ret);
    }
  else
    {
      /* Create signature */

      char    signature[256];
      size_t  signature_len = sizeof(signature);

      base64_encode((const unsigned char *)out_hmac_hash,
                    32,
                    (unsigned char *)signature,
                    &signature_len);

      /* URL encode */

      urlencode(signature, signature_len, sign_key, &sign_key_len);

      /*
       * Construct SAS string
       *
       * SAS toke format is shown in below. 
       *
       * SharedAccessSignature sig={signature-string}&se={expiry}&skn={policyName}&sr={URL-encoded-resourceURI}
       */

      snprintf(sas,
               size,
               "SharedAccessSignature sig=%s&se=%d&sr=%s",
               sign_key,
               expiry,
               urlencoded_url);
    }

  return ret;
}
