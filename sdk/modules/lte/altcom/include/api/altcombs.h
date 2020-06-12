/****************************************************************************
 * modules/lte/altcom/include/api/altcombs.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOMBS_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOMBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include "dbg_if.h"
#include "lte/lte_api.h"
#include "altcom_status.h"
#include "apicmd_pdn.h"
#include "apicmd_edrx.h"
#include "apicmd_psm.h"
#include "apicmd_quality.h"
#include "apicmd_cellinfo.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altcombs_cb_block
{
  int32_t                       id;
  FAR void                     *cb;
  bool                         removal;
  FAR struct altcombs_cb_block *prev;
  FAR struct altcombs_cb_block *next;
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline int altcombs_check_poweron_status(void)
{
  int ret = 0;
  int stat = altcom_get_status();

  if (stat != ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG1_WARNING("Invalid LTE status %d\n", stat);
      ret = -ENETDOWN;
    }

  return ret;
}

/****************************************************************************
 * Public function prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcombs_add_cbblock
 *
 * Description:
 *   Add callback block to the end of list.
 *
 * Input Parameters:
 *   head  Pointer of callback block list head.
 *   id    Callback block id.
 *   cb    Pointer to callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_add_cbblock(FAR struct altcombs_cb_block **head,
                             int32_t                        id,
                             FAR void                      *cb);

/****************************************************************************
 * Name: altcombs_remove_cbblock
 *
 * Description:
 *   Delete callback block from callback block list.
 *
 * Input Parameters:
 *   head   Pointer to callback block list head.
 *   block  Pointer to callback block.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_remove_cbblock(FAR struct altcombs_cb_block **head,
                                FAR struct altcombs_cb_block  *block);

/****************************************************************************
 * Name: altcombs_search_cbblock
 *
 * Description:
 *   Search callback block by callback id.
 *
 * Input Parameters:
 *   head  Pointer to callback block list head.
 *   id    Target callback block id.
 *
 * Returned Value:
 *   If list is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct altcombs_cb_block *altcombs_search_cbblock(
  FAR struct altcombs_cb_block *head, int32_t id);

/****************************************************************************
 * Name: altcombs_search_cbblock_bycb
 *
 * Description:
 *   Search callback block by callback pointer.
 *
 * Input Parameters:
 *   head  Pointer to callback block list head.
 *   cb    Pointer to callback.
 *
 * Returned Value:
 *   If list is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct altcombs_cb_block *altcombs_search_cbblock_bycb(
  FAR struct altcombs_cb_block *head, FAR void *cb);

/****************************************************************************
 * Name: altcombs_mark_removal_cbblock
 *
 * Description:
 *   Mark as removal callback block.
 *
 * Input Parameters:
 *   block  Pointer to callback block.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_mark_removal_cbblock(FAR struct altcombs_cb_block *block);

/****************************************************************************
 * Name: altcombs_remove_removal_cbblock
 *
 * Description:
 *   Delete callback block marked for removal.
 *
 * Input Parameters:
 *   head   Pointer to callback block list head.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_remove_removal_cbblock(FAR struct altcombs_cb_block **head);

/****************************************************************************
 * Name: altcombs_get_next_cbblock
 *
 * Description:
 *   Get pointer to next callback block.
 *
 * Input Parameters:
 *   block  Pointer to callback block.
 *
 * Returned Value:
 *   If next is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct altcombs_cb_block *altcombs_get_next_cbblock(
  FAR struct altcombs_cb_block *block);

/****************************************************************************
 * Name: altcombs_set_errinfo
 *
 * Description:
 *   Get LTE API last error information.
 *
 * Input Parameters:
 *   info    Pointer of LTE error information.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altcombs_set_errinfo(FAR lte_errinfo_t *info);

/****************************************************************************
 * Name: altcombs_get_errinfo
 *
 * Description:
 *   Get LTE API last error information.
 *
 * Input Parameters:
 *   info    Pointer of LTE error information.
 *
 * Returned Value:
 *   When get success is returned 0.
 *   When get failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_get_errinfo(lte_errinfo_t *info);

/****************************************************************************
 * Name: altcombs_set_pdninfo
 *
 * Description:
 *   Set lte_pdn_t param.
 *
 * Input Parameters:
 *   cmd_pdn    Pointer of api command pdn struct.
 *   lte_pdn    Pointer of lte_pdn_t.
 *
 * Returned Value:
 *   When convert success is returned 0.
 *   When convert failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_set_pdninfo(struct apicmd_pdnset_s *cmd_pdn,
  lte_pdn_t *lte_pdn);

/****************************************************************************
 * Name: altcombs_check_edrx
 *
 * Description:
 *   Check api comand eDRX param.
 *
 * Input Parameters:
 *   set    Pointer of api command eDRX struct.
 *
 * Returned Value:
 *   When check success is returned 0.
 *   When check failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_check_edrx(struct apicmd_edrxset_s *set);

/****************************************************************************
 * Name: altcombs_set_edrx
 *
 * Description:
 *   Set lte_edrx_setting_t param.
 *
 * Input Parameters:
 *   cmd_edrx    Pointer of api command edrx struct.
 *   lte_edrx    Pointer of lte_edrx_setting_t.
 *
 * Returned Value:
 *   When set success is returned 0.
 *   When set failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_set_edrx(struct apicmd_edrxset_s *cmd_edrx,
  lte_edrx_setting_t *lte_edrx);

/****************************************************************************
 * Name: altcombs_check_psm
 *
 * Description:
 *   Check PSM parameter of LTE API.
 *
 * Input Parameters:
 *   api_set  Pointer to PSM parameter of LTE API.
 *
 * Returned Value:
 *   When check success is returned 0.
 *   When check failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_check_psm(FAR lte_psm_setting_t *api_set);

/****************************************************************************
 * Name: altcombs_set_psm
 *
 * Description:
 *   Set to PSM parameter from API command.
 *
 * Input Parameters:
 *   cmd_set    Pointer to PSM parameter of API command.
 *   api_set    Pointer to PSM parameter of LTE API.
 *
 * Returned Value:
 *   When set success is returned 0.
 *   When set failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_set_psm(FAR struct apicmd_cmddat_psm_set_s *cmd_set,
                         FAR lte_psm_setting_t *api_set);

/****************************************************************************
 * Name: altcombs_set_quality
 *
 * Description:
 *   Set lte_quality_t.
 *
 * Input Parameters:
 *   data           Pointer of lte_quality_t.
 *   cmd_quality    Pointer of api command Quality struct.
 *
 * Returned Value:
 *   When check success is returned 0.
 *   When check failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_set_quality(FAR lte_quality_t *data,
          FAR struct apicmd_cmddat_quality_s *cmd_quality);

/****************************************************************************
 * Name: altcombs_set_cellinfo
 *
 * Description:
 *   Set lte_cellinfo_t.
 *
 * Input Parameters:
 *   cmd_cellinfo  Pointer of api command cellinfo struct.
 *   api_cellinfo  Pointer of lte_cellinfo_t.
 *
 * Returned Value:
 *   When check success is returned 0.
 *   When check failed return negative value.
 *
 ****************************************************************************/

void altcombs_set_cellinfo(FAR struct apicmd_cmddat_cellinfo_s *cmd_cellinfo,
                           FAR lte_cellinfo_t *api_cellinfo);

/****************************************************************************
 * Name: altcombs_setup_apicallback
 *
 * Description:
 *   Setup the callback of API.
 *
 * Input Parameters:
 *   id       API command ID.
 *   api_cb   Pointer of API callback to register.
 *   stat_cb  Pointer of state change callback to register.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_setup_apicallback(int32_t id, FAR void *api_cb,
                                   altcom_stat_chg_cb_t stat_cb);

/****************************************************************************
 * Name: altcombs_teardown_apicallback
 *
 * Description:
 *   Teardown the callback of API.
 *
 * Input Parameters:
 *   id       API command ID.
 *   stat_cb  Pointer of state change callback to register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altcombs_teardown_apicallback(int32_t id, altcom_stat_chg_cb_t stat_cb);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOMBS_H */
