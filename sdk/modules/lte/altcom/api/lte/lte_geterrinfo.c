/****************************************************************************
 * modules/lte/altcom/api/lte/lte_geterrno.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include "lte/lte_api.h"
#include "altcombs.h"
#include "apicmd_errinfo.h"
#include "apicmdhdlrbs.h"
#include "lte_geterrinfo.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: activatepdn_job
 *
 * Description:
 *   This function is notification the api error infomation.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void errinfo_job(FAR void *arg)
{
  struct apicmd_cmddat_errinfo_s *cmd = NULL;
  lte_errinfo_t                   info;

  cmd = (struct apicmd_cmddat_errinfo_s *)arg;

  info.err_indicator   = cmd->indicator;
  info.err_result_code = ntohl(cmd->err_code);
  info.err_no          = ntohl(cmd->err_no);
  memcpy(info.err_string, cmd->err_str, LTE_ERROR_STRING_MAX_LEN);
  info.err_string[LTE_ERROR_STRING_MAX_LEN - 1] = '\0';

  DBGIF_LOG1_INFO("recv errinfo. err_indicator:0x%x\n", info.err_indicator);
  if (info.err_indicator & LTE_ERR_INDICATOR_ERRCODE)
    {
      DBGIF_LOG1_INFO("err_result_code: %d.\n", info.err_result_code);
    }
  if (info.err_indicator & LTE_ERR_INDICATOR_ERRNO)
    {
      DBGIF_LOG1_INFO("err_no: %d.\n", info.err_no);
    }
  if (info.err_indicator & LTE_ERR_INDICATOR_ERRSTR)
    {
      DBGIF_LOG1_INFO("err_string: %s.\n", info.err_string);
    }

  altcombs_set_errinfo(&info);

  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_errinfo
 *
 * Description:
 *   Get ALTCOM last error information.
 *
 * Input Parameters:
 *   info    Pointer of lte error info.
 *
 * Returned Value:
 *   When get success is returned 0.
 *   When get failed return negative value.
 *
 ****************************************************************************/

int32_t lte_get_errinfo(lte_errinfo_t *info)
{
  int32_t ret;

  if (ALTCOM_STATUS_INITIALIZED > altcom_get_status())
    {
      return -EOPNOTSUPP;
    }

  if (!info)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  ret = altcombs_get_errinfo(info);

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_errinfo
 *
 * Description:
 *   This function is an API command handler for Error info notification.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_ERRINFO,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_errinfo(FAR uint8_t *evt, uint32_t evlen)
{
  if (apicmdgw_cmdid_compare(evt, APICMDID_ERRINFO))
    {
      errinfo_job(evt);
      return EVTHDLRC_STARTHANDLE;
    }

  return EVTHDLRC_UNSUPPORTEDEVENT;
}
