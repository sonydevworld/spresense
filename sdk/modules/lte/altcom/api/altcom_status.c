/****************************************************************************
 * modules/lte/altcom/api/lte/altcom_status.c
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
#include "dbg_if.h"
#include "osal.h"
#include "altcombs.h"
#include "altcom_status.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t g_altcom_status = ALTCOM_STATUS_UNINITIALIZED;
static FAR struct altcombs_cb_block *g_statchg_cb_table = NULL;
static sys_mutex_t                  g_table_mtx;
static sys_cremtx_s                 g_mtxparam;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcomstatus_callcb
 *
 * Description:
 *   Call all status change callbacks.
 *
 * Input Parameters:
 *   new_stat   ALTCOM new status.
 *   old_stat   ALTCOM old status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void altcomstatus_callcb(int32_t new_stat, int32_t old_stat)
{
  CODE altcom_stat_chg_cb_t *cbs = NULL;
  CODE altcom_stat_chg_cb_t cb = NULL;
  FAR struct altcombs_cb_block* cb_block = NULL;
  int32_t cnt = 0;
  cb_block = g_statchg_cb_table;
  while (cb_block)
    {
      cbs = (altcom_stat_chg_cb_t *)cb_block->cb_list;
      while (1)
        {
          cb = cbs[cnt];
          if (!cb)
            {
              break;
            }

          cb(new_stat, old_stat);
          cnt++;
        }

      cb_block = cb_block->next;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_get_status
 *
 * Description:
 *   Get altcom status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Return current altcom status.
 *
 ****************************************************************************/

int32_t altcom_get_status(void)
{
  return g_altcom_status;
}

/****************************************************************************
 * Name: altcom_set_status
 *
 * Description:
 *   set altcom status.
 *
 * Input Parameters:
 *   status     altcom status.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcom_set_status(int32_t status)
{
  int32_t prev_stat;
  int32_t ret = 0;

  if (ALTCOM_STATUS_MIN > status ||
      ALTCOM_STATUS_MAX < status)
    {
      return -EINVAL;
    }

  /* status UNINIT >> INIT, Create mutex. */

  if (ALTCOM_STATUS_UNINITIALIZED == g_altcom_status &&
      ALTCOM_STATUS_INITIALIZED == status)
    {
      ret = sys_create_mutex(&g_table_mtx, &g_mtxparam);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("sys_create_mutex() %d.\n", ret);
          return ret;
        }

    }
  else if (ALTCOM_STATUS_UNINITIALIZED != g_altcom_status &&
           ALTCOM_STATUS_UNINITIALIZED == status)
    {
      /* status NOTUNINIT >> UNINIT, Delete mutex */

      ret = sys_delete_mutex(&g_table_mtx);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("sys_delete_mutex() %d.\n", ret);
          return ret;
        }
    }
  else
    {
      ;;
    }

  prev_stat = g_altcom_status;
  DBGIF_LOG2_INFO("LTE library status %d -> %d.\n", prev_stat, status);
  g_altcom_status = status;
  altcomstatus_callcb(g_altcom_status, prev_stat);

  return 0;
}

/****************************************************************************
 * Name: altcomstatus_reg_statchgcb
 *
 * Description:
 *   Registoration altcom status change callbacks.
 *
 * Input Parameters:
 *   cb_list     Status change callback list.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomstatus_reg_statchgcb(void *cb_list)
{
  int32_t ret;
  FAR struct altcombs_cb_block *cb_block = NULL;

  if (ALTCOM_STATUS_UNINITIALIZED == g_altcom_status)
    {
      DBGIF_LOG_WARNING("ALTCOM status not initialized.\n");
      return -EPERM;
    }

  ret = altcombs_alloc_callbacklist(cb_list, &cb_block);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("altcombs_alloc_callbacklist() %d.\n", ret);
      return ret;
    }

  cb_block->cb_list = cb_list;
  cb_block->next = NULL;

  sys_lock_mutex(&g_table_mtx);
  ret = altcombs_add_cblist(&g_statchg_cb_table, cb_block);
  sys_unlock_mutex(&g_table_mtx);

  return ret;
}

/****************************************************************************
 * Name: altcomstatus_unreg_statchgcb
 *
 * Description:
 *   Unregistration altcom status change callbacks.
 *
 * Input Parameters:
 *   cb_list     Status change callback list.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomstatus_unreg_statchgcb(void *cb_list)
{
  int32_t ret;
  FAR struct altcombs_cb_block *cb_block = NULL;

  if (ALTCOM_STATUS_UNINITIALIZED == g_altcom_status)
    {
      DBGIF_LOG_WARNING("ALTCOM status not initialized.\n");
      return -EPERM;
    }

  if (!cb_list)
    {
      DBGIF_LOG_ERROR("null parameter.\n");
      return -EINVAL;
    }

  if (!g_statchg_cb_table)
    {
      DBGIF_LOG_WARNING("Callback list not found.\n");
      return 0;
    }

  sys_lock_mutex(&g_table_mtx);
  cb_block = altcombs_remove_cblist(&g_statchg_cb_table, cb_list);
  sys_unlock_mutex(&g_table_mtx);
  ret = altcombs_free_callbacklist(&cb_block);

  return ret;
}
