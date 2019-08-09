/****************************************************************************
 * modules/lte/altcom/api/mbedtls/apicmdhdlr_config_verify_callback.c
 *
 *   Copyright 2018 Sony Corporation
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

#include <string.h>

#include "buffpoolwrapper.h"
#include "apicmd_config_verify_callback.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_select_ext.h"
#include "mbedtls/x509_crt.h"
#include "vrfy_callback_mgr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_VERIFY_CALLBACK_REQ_DATALEN (sizeof(struct apicmd_config_verify_callback_s))

#define CONFIG_VERIFY_CALLBACK_SUCCESS 0
#define CONFIG_VERIFY_CALLBACK_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct config_verify_callback_res_s
{
  int32_t  ret_code;
  uint32_t flags;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t config_verify_callback_response(FAR struct config_verify_callback_res_s *res)
{
  int32_t                                    ret = 0;
  uint16_t                                   reslen = 0;
  FAR struct apicmd_config_verify_callback_s *cmd = NULL;

  cmd = altcom_alloc_cmdbuff(APICMDID_TLS_CONFIG_VERIFY_CALLBACK,
                              CONFIG_VERIFY_CALLBACK_REQ_DATALEN);
  if (cmd == NULL)
    {
      altcom_free_cmd((FAR uint8_t *) cmd);
      altcom_seterrno((int32_t)ALTCOM_ENOMEM);
      return CONFIG_VERIFY_CALLBACK_FAILURE;
    }

  cmd->ret_code = htonl(res->ret_code);
  cmd->flags = htonl(res->flags);

  DBGIF_LOG1_DEBUG("[config_verify_callback]ret_code: %d\n", res->ret_code);
  DBGIF_LOG1_DEBUG("[config_verify_callback]flags: 0x%x\n", res->flags);

  ret = apicmdgw_send((FAR uint8_t *)cmd, NULL,
                      CONFIG_VERIFY_CALLBACK_REQ_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  altcom_free_cmd((FAR uint8_t *) cmd);

  return CONFIG_VERIFY_CALLBACK_SUCCESS;

errout_with_cmdfree:
  altcom_free_cmd((FAR uint8_t *) cmd);
  return CONFIG_VERIFY_CALLBACK_FAILURE;
}

/****************************************************************************
 * Name: config_verify_callback_job
 *
 * Description:
 *   This function is an API callback for config_verify_callback async.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void config_verify_callback_job(FAR void *arg)
{
  mbedtls_x509_crt                              crt;
  int                                           depth = 0;
  uint32_t                                      flags = 0;
  int                                           ret = 0;
  FAR struct apicmd_config_verify_callbackres_s *data;
  FAR struct config_verify_callback_res_s       res;

  data = (FAR struct apicmd_config_verify_callbackres_s *) arg;

  crt.id = htonl(data->crt);
  depth = (int32_t) htonl(data->certificate_depth);

  DBGIF_LOG1_DEBUG("[config_verify_callback_job]crt id: %d\n", crt.id);
  DBGIF_LOG1_DEBUG("[config_verify_callback_job]depth: %d\n", depth);

  ret = invoke_verify_callback(&crt, depth, &flags);

  res.ret_code = ret;
  res.flags = flags;

  config_verify_callback_response(&res);

  /* In order to reduce the number of copies of the receive buffer,
  * bring a pointer to the receive buffer to the worker thread.
  * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdhdlr_config_verify_callback
 *
 * Description:
 *   This function is an API command handler for config_verify_callback async.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_TLS_CONFIG_VERIFY_CALLBACK,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_config_verify_callback(FAR uint8_t *evt, uint32_t evlen)
{
    return apicmdhdlrbs_do_runjob(evt, APICMDID_CONVERT_RES(APICMDID_TLS_CONFIG_VERIFY_CALLBACK),
                                  config_verify_callback_job);
}
