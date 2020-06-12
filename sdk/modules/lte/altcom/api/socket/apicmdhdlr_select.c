/****************************************************************************
 * modules/lte/altcom/api/socket/apicmdhdlr_select.c
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

#include "buffpoolwrapper.h"
#include "apicmd_select.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_select_ext.h"
#include "cc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: select_job
 ****************************************************************************/

static void select_job(FAR void *arg)
{
  int32_t                       ret;
  int32_t                       ret_code;
  int32_t                       err_code;
  int32_t                       select_id;
  uint16_t                      used_setbit;
  FAR struct apicmd_selectres_s *data;
  FAR altcom_fd_set             *preadset = NULL;
  FAR altcom_fd_set             *pwriteset = NULL;
  FAR altcom_fd_set             *pexceptset = NULL;

  data = (FAR struct apicmd_selectres_s *)arg;

  ret_code    = ntohl(data->ret_code);
  err_code    = ntohl(data->err_code);
  select_id   = ntohl(data->id);
  used_setbit = ntohs(data->used_setbit);

  if (used_setbit & APICMD_SELECT_USED_BIT_READSET)
    {
      preadset = &data->readset;
    }
  if (used_setbit & APICMD_SELECT_USED_BIT_WRITESET)
    {
      pwriteset = &data->writeset;
    }
  if (used_setbit & APICMD_SELECT_USED_BIT_EXCEPTSET)
    {
      pexceptset = &data->exceptset;
    }

  /* Get calback function by select id */

  ret = altcom_select_async_exec_callback(select_id, ret_code, err_code,
                                          preadset, pwriteset, pexceptset);
  if (ret < 0)
    {
      DBGIF_LOG2_DEBUG("altcom_select_async_exec_callback() failed: %d, %d\n", ret, select_id);
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
 * Name: apicmdhdlr_select
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_select(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_CONVERT_RES(APICMDID_SOCK_SELECT),
                                select_job);
}
