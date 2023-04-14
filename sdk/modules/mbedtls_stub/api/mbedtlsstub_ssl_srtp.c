/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_ssl_srtp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wireless/lte/lte_ioctl.h>
#include <mbedtls/ssl.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_ssl_export_srtp_keys( mbedtls_ssl_context *ssl, uint8_t* key_buffer, uint16_t key_buffer_size)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, &key_buffer_size};
  FAR void *outarg[] = {&result, key_buffer, &key_buffer_size};

  ret = lapi_req(LTE_CMDID_TLS_SSL_EXPORT_SRTP_KEYS,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);

  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_set_use_srtp(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_USE_SRTP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_get_srtp_profile(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_SRTP_PROFILE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_set_turn(mbedtls_ssl_context *ssl, uint16_t turn_channel, uint32_t peer_addr, uint16_t peer_port)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, &turn_channel, &peer_addr, &peer_port};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_TURN,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

