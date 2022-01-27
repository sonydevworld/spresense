/****************************************************************************
 * modules/lte_ext/altcom_additional_hdlr.c
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

#include <sys/types.h>
#include <nuttx/modem/alt1250.h>

/****************************************************************************
 * Functions prototypes
 ****************************************************************************/

extern compose_handler_t
    alt1250_sslcomposehdlr(uint32_t, FAR uint8_t *, size_t);

extern parse_handler_t
    alt1250_sslparsehdlr(uint16_t, uint8_t);

extern compose_handler_t
    alt1250_fresetcomposehdlr(uint32_t, FAR uint8_t *, size_t);

extern parse_handler_t
    alt1250_fresetparsehdlr(uint16_t, uint8_t);

/* Below 2 functions are for extension commands for alt1250 driver */

extern compose_handler_t alt1250_extcomposehdlr(uint32_t);
extern parse_handler_t alt1250_extparsehdlr(uint16_t, uint8_t);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** Name: alt1250_additional_parsehdlr */

parse_handler_t alt1250_additional_parsehdlr(uint16_t altcid, uint8_t altver)
{
  parse_handler_t ret = NULL;

#ifdef CONFIG_LTE_NET_MBEDTLS
  if (ret == NULL)
    {
      ret = alt1250_sslparsehdlr(altcid, altver);
    }
#endif

#ifdef CONFIG_LTE_FACTORY_RESET_API
  if (ret == NULL)
    {
      ret = alt1250_fresetparsehdlr(altcid, altver);
    }
#endif

#ifdef CONFIG_HAVE_EXTHANDLER
  if (ret == NULL)
    {
      ret = alt1250_extparsehdlr(altcid, altver);
    }
#endif

  return ret;
}

/** Name: alt1250_additional_composehdlr */

compose_handler_t alt1250_additional_composehdlr(uint32_t cmdid,
    FAR uint8_t *payload, size_t size)
{
  compose_handler_t ret = NULL;

#ifdef CONFIG_LTE_NET_MBEDTLS
  if (ret == NULL)
    {
      ret = alt1250_sslcomposehdlr(cmdid, payload, size);
    }
#endif

#ifdef CONFIG_LTE_FACTORY_RESET_API
  if (ret == NULL)
    {
      ret = alt1250_fresetcomposehdlr(cmdid, payload, size);
    }
#endif

#ifdef CONFIG_HAVE_EXTHANDLER
  if (ret == NULL)
    {
      ret = alt1250_extcomposehdlr(cmdid);
    }
#endif

  return ret;
}
