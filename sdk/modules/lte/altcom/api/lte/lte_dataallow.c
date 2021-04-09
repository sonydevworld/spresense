/****************************************************************************
 * modules/lte/altcom/api/lte/lte_dataallow.c
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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

#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "evthdlbs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_data_allow_sync
 *
 * Description:
 *   Allow or disallow to data communication for specified PDN.
 *
 * Input Parameters:
 *   session_id        The numeric value of the session ID.
 *   allow             Allow or disallow to data communication
 *                     for all network.
 *   roaming_allow     Allow or disallow to data communication
 *                     for roaming network.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_data_allow_sync(uint8_t session_id, uint8_t allow,
                            uint8_t roaming_allow)
{
  printf("lte_data_allow_sync() is not supported.\n");

  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: lte_data_allow
 *
 * Description:
 *   Allow or disallow to data communication for specified PDN.
 *
 * Input Parameters:
 *   session_id        The numeric value of the session ID.
 *   allow             Allow or disallow to data communication
 *                     for all network.
 *   roaming_allow     Allow or disallow to data communication
 *                     for roaming network.
 *   callback          Callback function to notify that
 *                     configuration has changed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_data_allow(uint8_t session_id, uint8_t allow,
                       uint8_t roaming_allow, data_allow_cb_t callback)
{
  printf("lte_data_allow() is not supported.\n");

  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: apicmdhdlr_dataallow
 *
 * Description:
 *   This function is an API command handler for data allow result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_DATA_ALLOW_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_dataallow(FAR uint8_t *evt, uint32_t evlen)
{
  return EVTHDLRC_UNSUPPORTEDEVENT;
}
