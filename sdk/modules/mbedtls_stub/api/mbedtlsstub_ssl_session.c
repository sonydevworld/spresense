/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_ssl_session.c
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

#include <sys/ioctl.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <mbedtls/ssl.h>
#include <mbedtls/net_sockets.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_ssl_session_init(mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&session->id};
  FAR void *outarg[] = {&result, session};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_session_free(mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {session};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_get_session(const mbedtls_ssl_context *ssl, mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl, session};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_GET,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_set_session(mbedtls_ssl_context *ssl, const mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, (void *)session};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_SET,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_session_reset(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_RESET,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_session_getctx(mbedtls_ssl_session *session, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_ssl_session);

  if (session && buff && size >= sizeof(mbedtls_ssl_session))
    {
      mbedtls_ssl_session *ctx = (mbedtls_ssl_session *)buff;
      ctx->id = session->id;
    }
  else if (session && buff)
    {
      ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_ssl_session_setctx(mbedtls_ssl_session *session, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (session && buff && size >= sizeof(mbedtls_ssl_session))
    {
      mbedtls_ssl_session *ctx = (mbedtls_ssl_session *)buff;
      session->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_ssl_session_getctxsize(mbedtls_ssl_session *session)
{
  return mbedtls_ssl_session_getctx(NULL, NULL, 0);
}

