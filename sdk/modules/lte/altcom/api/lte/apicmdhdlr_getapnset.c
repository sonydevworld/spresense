/****************************************************************************
 * modules/lte/altcom/api/lte/apicmdhdlr_getapnset.c
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
#include "buffpoolwrapper.h"
#include "apicmd_getapnset.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern get_apnset_cb_t g_getapnset_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getapnset_job
 *
 * Description:
 *   This function is an API callback for get APN set.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getapnset_job(FAR void *arg)
{
  int32_t                                 ret;
  int32_t                                 result;
  FAR struct apicmd_cmddat_getapnsetres_s *data;
  lte_getapnset_t                         apnset;
  get_apnset_cb_t                         callback;
  int32_t                                 cnt;

  data = (FAR struct apicmd_cmddat_getapnsetres_s *)arg;

  ALTCOM_GET_AND_CLR_CALLBACK(ret, g_getapnset_callback, callback);

  if ((ret == 0) && (callback))
    {
      result = (int32_t)data->result;
      apnset.listnum = data->listnum;
      apnset.apnlist = NULL;

      if (APICMD_GETAPNSET_RES_OK == result)
        {
          if (APICMD_GETAPNSET_RES_LIST_MAX_NUM < apnset.listnum)
            {
              DBGIF_LOG1_ERROR("apnset.listnum error:%d\n", apnset.listnum);
              result = APICMD_GETAPNSET_RES_ERR;
            }
          else
            {
              apnset.apnlist =
                BUFFPOOL_ALLOC(sizeof(lte_getapndata_t) * data->listnum);
              DBGIF_ASSERT(apnset.apnlist, "BUFFPOOL_ALLOC error.");
              for (cnt = 0; cnt < data->listnum; cnt++)
                {
                  apnset.apnlist[cnt].session_id = data->apnlist[cnt].session_id;
                  strncpy((FAR char *)apnset.apnlist[cnt].apn,
                    (FAR char *)data->apnlist[cnt].apn,
                    sizeof(apnset.apnlist[cnt].apn));
                  apnset.apnlist[cnt].ip_type   = data->apnlist[cnt].ip_type;
                }
            }
        }

      callback(result, &apnset);
      if (apnset.apnlist)
        {
          (void)BUFFPOOL_FREE((FAR void *)apnset.apnlist);
        }
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdhdlr_getapnset
 *
 * Description:
 *   This function is an API command handler for get APN set result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_APNSET_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getapnset(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_GET_APNSET), getapnset_job);
}
