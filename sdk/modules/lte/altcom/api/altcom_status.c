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
  CODE altcom_stat_chg_cb_t     cb_func       = NULL;
  FAR struct altcombs_cb_block *cb_block      = NULL;
  int32_t                       ret           = ALTCOM_STATUS_REG_KEEP;

  sys_lock_mutex(&g_table_mtx);

  cb_block = g_statchg_cb_table;

  if (cb_block)
    {
      while (cb_block)
        {
          cb_func = (altcom_stat_chg_cb_t)cb_block->cb;
          if (cb_func)
            {
              ret = cb_func(new_stat, old_stat);
            }
          else
            {
              ret = ALTCOM_STATUS_REG_KEEP;
            }

          if (ret == ALTCOM_STATUS_REG_CLR)
            {
              /* Mark as removal when this loop is over */

              altcombs_mark_removal_cbblock(cb_block);
            }
          cb_block = altcombs_get_next_cbblock(cb_block);
        }

      /* Remove block marked for removal */

      altcombs_remove_removal_cbblock(&g_statchg_cb_table);
    }

  sys_unlock_mutex(&g_table_mtx);
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
  bool    delflag = false;

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

      delflag = true;
    }
  else
    {
      ;;
    }

  prev_stat = g_altcom_status;
  DBGIF_LOG2_INFO("LTE library status %d -> %d.\n", prev_stat, status);
  g_altcom_status = status;
  altcomstatus_callcb(g_altcom_status, prev_stat);

  if (delflag)
    {
      ret = sys_delete_mutex(&g_table_mtx);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("sys_delete_mutex() %d.\n", ret);
          return ret;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: altcomstatus_reg_statchgcb
 *
 * Description:
 *   Registoration altcom status change callbacks.
 *
 * Input Parameters:
 *   cb     Status change callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomstatus_reg_statchgcb(altcom_stat_chg_cb_t cb)
{
  int32_t                       ret;

  if (ALTCOM_STATUS_UNINITIALIZED == g_altcom_status)
    {
      DBGIF_LOG_WARNING("ALTCOM status not initialized.\n");
      return -EPERM;
    }

  sys_lock_mutex(&g_table_mtx);

  ret = altcombs_add_cbblock(&g_statchg_cb_table, 0, cb);

  sys_unlock_mutex(&g_table_mtx);

  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("altcombs_add_cbblock() %d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: altcomstatus_unreg_statchgcb
 *
 * Description:
 *   Unregistration altcom status change callbacks.
 *
 * Input Parameters:
 *   cb     Status change callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomstatus_unreg_statchgcb(altcom_stat_chg_cb_t cb)
{
  int32_t                       ret = 0;
  FAR struct altcombs_cb_block *cb_block = NULL;

  if (ALTCOM_STATUS_UNINITIALIZED == g_altcom_status)
    {
      DBGIF_LOG_WARNING("ALTCOM status not initialized.\n");
      return -EPERM;
    }

  if (!cb)
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

  cb_block = altcombs_search_cbblock_bycb(g_statchg_cb_table, cb);
  if (cb_block)
    {
      altcombs_remove_cbblock(&g_statchg_cb_table, cb_block);
    }
  else
    {
      ret = -EINVAL;
    }

  sys_unlock_mutex(&g_table_mtx);

  return ret;
}
