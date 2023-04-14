/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_bignum.c
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

#include <stdint.h>
#include <stddef.h>
#include <arpa/inet.h>
#include <nuttx/modem/alt1250.h>
#include <mbedtls/bignum.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_mpi_init.h"
#include "include/apicmd_mpi_free.h"
#include "include/apicmd_mpi_read_string.h"
#include "include/apicmd_mpi_write_string.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_mpiinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_MPI_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
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
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_mpifree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_mpi *X = (FAR mbedtls_mpi *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
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
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_mpireadstr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  uint32_t buflen = 0;
  FAR mbedtls_mpi *X = (FAR mbedtls_mpi *)arg[0];
  FAR int *radix = (FAR int *)arg[1];
  FAR const char *s = (FAR const char *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
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
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_mpiwritestr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR const mbedtls_mpi *X = (FAR const mbedtls_mpi *)arg[0];
  FAR int *radix = (FAR int *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
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
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}


int32_t mbedtlsstub_mpiinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_initres_s *in =
        (FAR struct apicmd_mpi_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[mpi_init res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_mpifree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_freeres_s *in =
        (FAR struct apicmd_mpi_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[mpi_free res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_mpireadstr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mpi_read_stringres_s *in =
        (FAR struct apicmd_mpi_read_stringres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[mpi_read_string res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_mpiwritestr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR char *buf = (FAR char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];
  FAR size_t *olen = (FAR size_t *)arg[3];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
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
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
