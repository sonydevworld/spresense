/****************************************************************************
 * modules/lte/altcom/api/altcombs.c
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

#include <errno.h>
#include <string.h>

#include "lte/lte_api.h"
#include "dbg_if.h"
#include "buffpoolwrapper.h"
#include "altcombs.h"
#include "altcom_callbacks.h"
#include "altcom_status.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOMBS_EDRX_CYCLE_WBS1_MIN      (LTE_EDRX_CYC_512)
#define ALTCOMBS_EDRX_CYCLE_WBS1_MAX      (LTE_EDRX_CYC_262144)
#define ALTCOMBS_EDRX_PTW_WBS1_MIN        (LTE_EDRX_PTW_128)
#define ALTCOMBS_EDRX_PTW_WBS1_MAX        (LTE_EDRX_PTW_2048)
#define ALTCOMBS_PSM_UNIT_T3324_MIN       (LTE_PSM_T3324_UNIT_2SEC)
#define ALTCOMBS_PSM_UNIT_T3324_MAX       (LTE_PSM_T3324_UNIT_6MIN)
#define ALTCOMBS_PSM_UNIT_T3412_MIN       (LTE_PSM_T3412_UNIT_2SEC)
#define ALTCOMBS_PSM_UNIT_T3412_MAX       (LTE_PSM_T3412_UNIT_320HOUR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static lte_errinfo_t g_errinfo = { 0, 0, 0, ""};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcombs_alloc_cbblock
 *
 * Description:
 *   Allocation callback block.
 *
 * Input Parameters:
 *   id  Callback block id.
 *   cb  Pointer to callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns allocated callback block.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

static FAR struct altcombs_cb_block *altcombs_alloc_cbblock(int32_t id,
                                                            FAR void *cb)
{
  FAR struct altcombs_cb_block *block;

  if (!cb)
    {
      return NULL;
    }

  block = (FAR struct altcombs_cb_block *)BUFFPOOL_ALLOC(
            sizeof(struct altcombs_cb_block));
  if (!block)
    {
      DBGIF_LOG_ERROR("Failed to allocate memory\n");
      return NULL;
    }

  block->id      = id;
  block->cb      = cb;
  block->removal = false;
  block->prev    = NULL;
  block->next    = NULL;

  return block;
}

/****************************************************************************
 * Name: altcombs_free_cbblock
 *
 * Description:
 *   Free callback block.
 *
 * Input Parameters:
 *   cb_block   Pointer to callback block.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

static int32_t altcombs_free_cbblock(FAR struct altcombs_cb_block *cb_block)
{
  if (!cb_block)
    {
      return -EINVAL;
    }

  (void)BUFFPOOL_FREE(cb_block);

  return 0;
}

/****************************************************************************
 * Name: check_arrydigitnum
 *
 * Description:
 *   Evaluate the validity of each digit of arrayed numerical value.
 *
 * Input Parameters:
 *  number    Array type number.
 *  digit     @number digit.
 *
 * Returned Value:
 *   Returns true if everything is valid. Otherwise it returns false.
 *
 ****************************************************************************/

static bool altcombs_check_arrydigitnum(FAR uint8_t number[], uint8_t digit)
{
  uint8_t cnt;

  for (cnt = 0; cnt < digit; cnt++)
    {
      if (number[cnt] < APICMD_CELLINFO_DIGIT_NUM_MIN ||
        APICMD_CELLINFO_DIGIT_NUM_MAX < number[cnt])
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Public Functions
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
                             FAR void                      *cb)
{
  FAR struct altcombs_cb_block *block;
  FAR struct altcombs_cb_block *curr_block;

  if ((!cb) || (!head))
    {
      return -EINVAL;
    }

  block = altcombs_alloc_cbblock(id, cb);
  if (!block)
    {
      return -ENOMEM;
    }

  curr_block = *head;

  if (*head)
    {
      /* Search end of block */

      while(curr_block->next)
        {
          curr_block = curr_block->next;
        }
      curr_block->next = block;
      block->prev      = curr_block;
    }
  else
    {
      *head = block;
    }

  return 0;
}

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
                                FAR struct altcombs_cb_block  *block)
{
  FAR struct altcombs_cb_block *curr_block = *head;

  if (!block)
    {
      return -EINVAL;
    }

  while (curr_block)
    {
      if (curr_block->cb == block->cb)
        {
          /* Check begining of the list */

          if (curr_block->prev)
            {
              curr_block->prev->next = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = curr_block->prev;
                }
            }
          else
            {
              *head = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = NULL;
                }
            }
          altcombs_free_cbblock(curr_block);

          return 0;
        }
      curr_block = curr_block->next;
    }

  return -EINVAL;
}

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
  FAR struct altcombs_cb_block *head, int32_t id)
{
  FAR struct altcombs_cb_block *curr_block = head;

  while (curr_block)
    {
      if (curr_block->id == id)
        {
          return curr_block;
        }
      curr_block = curr_block->next;
    }

  return NULL;
}

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
  FAR struct altcombs_cb_block *head, FAR void *cb)
{
  FAR struct altcombs_cb_block *curr_block = head;

  while (curr_block)
    {
      if (curr_block->cb == cb)
        {
          return curr_block;
        }
      curr_block = curr_block->next;
    }

  return NULL;
}

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

int32_t altcombs_mark_removal_cbblock(FAR struct altcombs_cb_block *block)
{
  if (!block)
    {
      return -EINVAL;
    }

  block->removal = true;

  return 0;
}

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

int32_t altcombs_remove_removal_cbblock(FAR struct altcombs_cb_block **head)
{
  FAR struct altcombs_cb_block *curr_block = *head;

  while (curr_block)
    {
      if (curr_block->removal)
        {
          /* Check begining of the list */

          if (curr_block->prev)
            {
              curr_block->prev->next = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = curr_block->prev;
                }
            }
          else
            {
              *head = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = NULL;
                }
            }
          altcombs_free_cbblock(curr_block);
        }
      curr_block = curr_block->next;
    }

  return 0;
}

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
  FAR struct altcombs_cb_block *block)
{
  if (!block)
    {
      return NULL;
    }

  return block->next;
}

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

void altcombs_set_errinfo(FAR lte_errinfo_t *info)
{
  if (info)
    {
      memcpy(&g_errinfo, info, sizeof(lte_errinfo_t));
    }
}

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

int32_t altcombs_get_errinfo(FAR lte_errinfo_t *info)
{
  if (!info)
    {
      return -EINVAL;
    }

  memcpy(info, &g_errinfo, sizeof(lte_errinfo_t));
  return 0;
}

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
  lte_pdn_t *lte_pdn)
{
  int32_t i;

  if (!cmd_pdn || !lte_pdn)
    {
      return -EINVAL;
    }

  lte_pdn->session_id = cmd_pdn->session_id;
  lte_pdn->active = cmd_pdn->activate;
  lte_pdn->apn_type = htonl(cmd_pdn->apntype);
  lte_pdn->ipaddr_num = cmd_pdn->ipaddr_num;
  for (i = 0; i < lte_pdn->ipaddr_num; i++)
    {
      lte_pdn->address[i].ip_type = cmd_pdn->ip_address[i].iptype;
      strncpy((FAR char *)lte_pdn->address[i].address,
              (FAR char *)cmd_pdn->ip_address[i].address,
              LTE_IPADDR_MAX_LEN - 1);
    }

  lte_pdn->ims_register = cmd_pdn->imsregister == APICMD_PDN_IMS_REG ?
    LTE_IMS_REGISTERED : LTE_IMS_NOT_REGISTERED;
  lte_pdn->data_allow = cmd_pdn->dataallow ==
    APICMD_PDN_DATAALLOW_ALLOW ?
    LTE_DATA_ALLOW : LTE_DATA_DISALLOW;
  lte_pdn->data_roaming_allow = cmd_pdn->dararoamingallow ==
    APICMD_PDN_DATAROAMALLOW_ALLOW ?
    LTE_DATA_ALLOW : LTE_DATA_DISALLOW;

  return 0;
}

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

int32_t altcombs_check_edrx(struct apicmd_edrxset_s *set)
{
  if (!set)
    {
      DBGIF_LOG_ERROR("null param\n");
      return -EINVAL;
    }

  if (set->enable < LTE_DISABLE ||
      set->enable > LTE_ENABLE)
    {
      DBGIF_LOG1_ERROR("Invalid enable :%d\n", set->enable);
      return -EINVAL;
    }

  if (LTE_ENABLE == set->enable)
    {
      if (APICMD_EDRX_ACTTYPE_NOTUSE != set->acttype &&
          APICMD_EDRX_ACTTYPE_WBS1 != set->acttype)
        {
          DBGIF_LOG1_ERROR("Invalid acttype :%d\n", set->acttype);
          return -EINVAL;
        }

      if (LTE_EDRX_ACTTYPE_WBS1 == set->acttype)
        {
          if (set->edrx_cycle < ALTCOMBS_EDRX_CYCLE_WBS1_MIN ||
              set->edrx_cycle > ALTCOMBS_EDRX_CYCLE_WBS1_MAX)
            {
              DBGIF_LOG1_ERROR("Invalid cycle :%d\n", set->edrx_cycle);
              return -EINVAL;
            }

          if (set->ptw_val < ALTCOMBS_EDRX_PTW_WBS1_MIN ||
              set->ptw_val > ALTCOMBS_EDRX_PTW_WBS1_MAX)
            {
              DBGIF_LOG1_ERROR("Invalid PTW :%d\n", set->ptw_val);
              return -EINVAL;
            }
        }
    }

  return 0;
}

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
  lte_edrx_setting_t *lte_edrx)
{
  uint8_t edrx_acttype_table[] =
    {
      LTE_EDRX_ACTTYPE_NOTUSE,
      LTE_EDRX_ACTTYPE_ECGSMIOT,
      LTE_EDRX_ACTTYPE_GSM,
      LTE_EDRX_ACTTYPE_IU,
      LTE_EDRX_ACTTYPE_WBS1,
      LTE_EDRX_ACTTYPE_NBS1
    };

  const uint8_t edrx_cycle_wbs1_table[] =
    {
      LTE_EDRX_CYC_512,
      LTE_EDRX_CYC_1024,
      LTE_EDRX_CYC_2048,
      LTE_EDRX_CYC_4096,
      LTE_EDRX_CYC_6144,
      LTE_EDRX_CYC_8192,
      LTE_EDRX_CYC_10240,
      LTE_EDRX_CYC_12288,
      LTE_EDRX_CYC_14336,
      LTE_EDRX_CYC_16384,
      LTE_EDRX_CYC_32768,
      LTE_EDRX_CYC_65536,
      LTE_EDRX_CYC_131072,
      LTE_EDRX_CYC_262144,
    };

  const uint8_t edrx_ptw_wbs1_table[] =
    {
      LTE_EDRX_PTW_128,
      LTE_EDRX_PTW_256,
      LTE_EDRX_PTW_384,
      LTE_EDRX_PTW_512,
      LTE_EDRX_PTW_640,
      LTE_EDRX_PTW_768,
      LTE_EDRX_PTW_896,
      LTE_EDRX_PTW_1024,
      LTE_EDRX_PTW_1152,
      LTE_EDRX_PTW_1280,
      LTE_EDRX_PTW_1408,
      LTE_EDRX_PTW_1536,
      LTE_EDRX_PTW_1664,
      LTE_EDRX_PTW_1792,
      LTE_EDRX_PTW_1920,
      LTE_EDRX_PTW_2048,
    };

  if (!cmd_edrx || !lte_edrx)
    {
      return -EINVAL;
    }

  if (LTE_ENABLE == cmd_edrx->enable)
    {
      if (APICMD_EDRX_ACTTYPE_NOTUSE != cmd_edrx->acttype &&
          APICMD_EDRX_ACTTYPE_WBS1 != cmd_edrx->acttype)
        {
          DBGIF_LOG_ERROR("cmd_edrx erorr\n");
          return -EINVAL;
        }

      if (APICMD_EDRX_ACTTYPE_WBS1 == cmd_edrx->acttype)
        {
          if (ALTCOMBS_EDRX_CYCLE_WBS1_MIN > cmd_edrx->edrx_cycle ||
              ALTCOMBS_EDRX_CYCLE_WBS1_MAX < cmd_edrx->edrx_cycle ||
              ALTCOMBS_EDRX_PTW_WBS1_MIN > cmd_edrx->ptw_val ||
              ALTCOMBS_EDRX_PTW_WBS1_MAX < cmd_edrx->ptw_val)
            {
              DBGIF_LOG_ERROR("cmd_edrx erorr\n");
              return -EINVAL;
            }
        }

      lte_edrx->enable = LTE_ENABLE;
      lte_edrx->act_type = edrx_acttype_table[cmd_edrx->acttype];
      if (APICMD_EDRX_ACTTYPE_WBS1 == cmd_edrx->acttype)
        {
          lte_edrx->edrx_cycle = edrx_cycle_wbs1_table[cmd_edrx->edrx_cycle];
          lte_edrx->ptw_val = edrx_ptw_wbs1_table[cmd_edrx->ptw_val];
        }
    }
  else
    {
      lte_edrx->enable = LTE_DISABLE;
    }

  return 0;
}

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

int32_t altcombs_check_psm(FAR lte_psm_setting_t *api_set)
{
  if (!api_set)
    {
      DBGIF_LOG_ERROR("null param\n");
      return -EINVAL;
    }

  if (LTE_ENABLE == api_set->enable)
    {
      if (api_set->req_active_time.unit < LTE_PSM_T3324_UNIT_2SEC ||
          api_set->req_active_time.unit > LTE_PSM_T3324_UNIT_DEACT)
        {
          DBGIF_LOG1_ERROR("Invalid rat_time unit :%d\n", api_set->req_active_time.unit);
          return -EINVAL;
        }

      if (api_set->req_active_time.time_val < LTE_PSM_TIMEVAL_MIN ||
          api_set->req_active_time.time_val > LTE_PSM_TIMEVAL_MAX)
        {
          DBGIF_LOG1_ERROR("Invalid rat_time time_val :%d\n", api_set->req_active_time.time_val);
          return -EINVAL;
        }

      if (api_set->ext_periodic_tau_time.unit < LTE_PSM_T3412_UNIT_2SEC ||
          api_set->ext_periodic_tau_time.unit > LTE_PSM_T3412_UNIT_DEACT)
        {
          DBGIF_LOG1_ERROR("Invalid tau_time unit :%d\n", api_set->ext_periodic_tau_time.unit);
          return -EINVAL;
        }

      if (api_set->ext_periodic_tau_time.time_val < LTE_PSM_TIMEVAL_MIN ||
          api_set->ext_periodic_tau_time.time_val > LTE_PSM_TIMEVAL_MAX)
        {
          DBGIF_LOG1_ERROR("Invalid tau_time time_val :%d\n", api_set->ext_periodic_tau_time.time_val);
          return -EINVAL;
        }
    }

  return 0;
}

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
                         FAR lte_psm_setting_t *api_set)
{
  if (!cmd_set || !api_set)
    {
      return -EINVAL;
    }

  api_set->enable                         = cmd_set->enable;
  api_set->req_active_time.unit           = cmd_set->rat_time.unit;
  api_set->req_active_time.time_val       = cmd_set->rat_time.time_val;
  api_set->ext_periodic_tau_time.unit     = cmd_set->tau_time.unit;
  api_set->ext_periodic_tau_time.time_val = cmd_set->tau_time.time_val;

  return 0;
}

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
          FAR struct apicmd_cmddat_quality_s *cmd_quality)
{
  data->valid = APICMD_QUALITY_ENABLE == cmd_quality->enability ?
                         LTE_VALID : LTE_INVALID;
  if(data->valid)
    {
      data->rsrp  = ntohs(cmd_quality->rsrp);
      data->rsrq  = ntohs(cmd_quality->rsrq);
      data->sinr  = ntohs(cmd_quality->sinr);
      data->rssi  = ntohs(cmd_quality->rssi);
      if (data->rsrp < APICMD_QUALITY_RSRP_MIN ||
        APICMD_QUALITY_RSRP_MAX < data->rsrp)
        {
          DBGIF_LOG1_ERROR("data.rsrp error:%d\n", data->rsrp);
          data->valid = LTE_INVALID;
        }
      else if (data->rsrq < APICMD_QUALITY_RSRQ_MIN ||
          APICMD_QUALITY_RSRQ_MAX < data->rsrq)
        {
          DBGIF_LOG1_ERROR("data.rsrq error:%d\n", data->rsrq);
          data->valid = LTE_INVALID;
        }
      else if (data->sinr < APICMD_QUALITY_SINR_MIN ||
          APICMD_QUALITY_SINR_MAX < data->sinr)
        {
          DBGIF_LOG1_ERROR("data->sinr error:%d\n", data->sinr);
          data->valid = LTE_INVALID;
        }
      else
        {
          /* Do nothing. */
        }
    }
  return 0;
}

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

void altcombs_set_cellinfo(
          FAR struct apicmd_cmddat_cellinfo_s *cmd_cellinfo,
          FAR lte_cellinfo_t *api_cellinfo)
{
  if (cmd_cellinfo->valid == LTE_VALID)
    {
      if (ntohl(cmd_cellinfo->cell_id) > APICMD_CELLINFO_CELLID_MAX )
        {
          DBGIF_LOG1_ERROR("cmd_cellinfo->cell_id error:%d\n",
                           ntohl(cmd_cellinfo->cell_id));
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (ntohl(cmd_cellinfo->earfcn) > APICMD_CELLINFO_EARFCN_MAX )
        {
          DBGIF_LOG1_ERROR("cmd_cellinfo->earfcn error:%d\n",
                           ntohl(cmd_cellinfo->earfcn));
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (!altcombs_check_arrydigitnum(cmd_cellinfo->mcc, LTE_MCC_DIGIT))
        {
          DBGIF_LOG_ERROR("cmd_cellinfo->mcc error\n");
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (
        cmd_cellinfo->mnc_digit < APICMD_CELLINFO_MNC_DIGIT_MIN ||
        cmd_cellinfo->mnc_digit > LTE_MNC_DIGIT_MAX)
        {
          DBGIF_LOG1_ERROR("cmd_cellinfo->mnc_digit error:%d\n",
                           cmd_cellinfo->mnc_digit);
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (!altcombs_check_arrydigitnum(cmd_cellinfo->mnc,
               cmd_cellinfo->mnc_digit))
        {
          DBGIF_LOG_ERROR("cmd_cellinfo->mnc error\n");
          api_cellinfo->valid = LTE_INVALID;
        }
      else
        {
          api_cellinfo->valid = LTE_VALID;
          api_cellinfo->phycell_id = ntohl(cmd_cellinfo->cell_id);
          api_cellinfo->earfcn     = ntohl(cmd_cellinfo->earfcn);
          memcpy(api_cellinfo->mcc, cmd_cellinfo->mcc, LTE_MCC_DIGIT);
          api_cellinfo->mnc_digit  = cmd_cellinfo->mnc_digit;
          memcpy(api_cellinfo->mnc, cmd_cellinfo->mnc, 
                 cmd_cellinfo->mnc_digit);
        }
    }
  else
    {
      api_cellinfo->valid = LTE_INVALID;
    }
}

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
                                   altcom_stat_chg_cb_t stat_cb)
{
  int32_t ret;
  /* Register API callback */

  ret = altcomcallbacks_chk_reg_cb(api_cb, id);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  /* Register state change callback */

  ret = altcomstatus_reg_statchgcb(stat_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(id);
      return ret;
    }

  return 0;
}

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

void altcombs_teardown_apicallback(int32_t id, altcom_stat_chg_cb_t stat_cb)
{
  /* Unregister callbacks */

  altcomcallbacks_unreg_cb(id);
  altcomstatus_unreg_statchgcb(stat_cb);
}

