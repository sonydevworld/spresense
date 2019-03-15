/****************************************************************************
 * modules/lte/altcom/api/altcom_callbacks.c
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

#include "dbg_if.h"
#include "osal.h"
#include "buffpoolwrapper.h"
#include "altcombs.h"
#include "altcom_callbacks.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct altcombs_cb_block *g_lteapi_callback_list = NULL;
static sys_mutex_t                  g_list_mtx;
static sys_cremtx_s                 g_mtxparam;
static bool                         g_isinit = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcomcallbacks_init
 *
 * Description:
 *   Initialize altcom API callbacks.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_init(void)
{
  int32_t ret;

  if (g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks initialized.\n");
      return -EPERM;
    }

  ret = sys_create_mutex(&g_list_mtx, &g_mtxparam);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("sys_create_mutex() %d.\n", ret);
      return ret;
    }

  g_isinit = true;

  return ret;
}

/****************************************************************************
 * Name: altcomcallbacks_fin
 *
 * Description:
 *   Finalize altcom API callbacks.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_fin(void)
{
  int32_t ret;
  FAR struct altcombs_cb_block *cb_block = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks not initialized.\n");
      return -EPERM;
    }

  sys_lock_mutex(&g_list_mtx);

  cb_block = g_lteapi_callback_list;

  while(cb_block)
    {
      altcombs_mark_removal_cbblock(cb_block);
      cb_block = altcombs_get_next_cbblock(cb_block);
    }
  altcombs_remove_removal_cbblock(&g_lteapi_callback_list);

  sys_unlock_mutex(&g_list_mtx);

  ret = sys_delete_mutex(&g_list_mtx);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("sys_delete_mutex() %d.\n", ret);
      return ret;
    }

  g_isinit = false;

  return ret;
}

/****************************************************************************
 * Name: altcomcallbacks_reg_cb
 *
 * Description:
 *   Registration altcom API callback.
 *
 * Input Parameters:
 *   cb_ptr     Pointer of registration callback.
 *   id         Callback function id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_reg_cb(void *cb_ptr, int32_t id)
{
  int32_t                       ret;

  if (!g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks not initialized.\n");
      return -EPERM;
    }

  sys_lock_mutex(&g_list_mtx);

  ret = altcombs_add_cbblock(&g_lteapi_callback_list, id, cb_ptr);

  sys_unlock_mutex(&g_list_mtx);

  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("altcombs_add_cbblock() %d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: altcomcallbacks_unreg_cb
 *
 * Description:
 *   Unregistration altcom API callback.
 *
 * Input Parameters:
 *   id       Target callback function id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_unreg_cb(int32_t id)
{
  int32_t                       ret = 0;
  FAR struct altcombs_cb_block *cb_block = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks not initialized.\n");
      return -EPERM;
    }

  if (!g_lteapi_callback_list)
    {
      DBGIF_LOG_WARNING("Callback list not found.\n");
      return 0;
    }

  sys_lock_mutex(&g_list_mtx);

  cb_block = altcombs_search_cbblock(g_lteapi_callback_list, id);
  if (cb_block)
    {
      altcombs_remove_cbblock(&g_lteapi_callback_list, cb_block);
    }
  else
    {
      ret = -EINVAL;
    }

  sys_unlock_mutex(&g_list_mtx);

  return ret;
}

/****************************************************************************
 * Name: altcomcallbacks_get_cb
 *
 * Description:
 *   Get altcom API callback.
 *
 * Input Parameters:
 *   id    Target callback function id.
 *
 * Returned Value:
 *   Pointer of Callback function. When not registered callbacke return NULL.
 *
 ****************************************************************************/

void *altcomcallbacks_get_cb(int32_t id)
{
  FAR struct altcombs_cb_block *cb_block = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks not initialized.\n");
      return NULL;
    }

  sys_lock_mutex(&g_list_mtx);

  cb_block = altcombs_search_cbblock(g_lteapi_callback_list, id);

  sys_unlock_mutex(&g_list_mtx);

  if (!cb_block)
    {
      return NULL;
    }

  return cb_block->cb;
}

/****************************************************************************
 * Name: altcomcallbacks_get_unreg_cb
 *
 * Description:
 *   Get and unregistration altcom API callback.
 *
 * Input Parameters:
 *   id         Target callback function id.
 *   callback   Pointer of target callback address.
 *              This pointer is out parameter.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_get_unreg_cb(int32_t id, void **callback)
{
  int32_t ret = 0;
  FAR struct altcombs_cb_block *cb_block = NULL;

  *callback = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks not initialized.\n");
      return -EPERM;
    }

  if (!g_lteapi_callback_list)
    {
      DBGIF_LOG_WARNING("Callback list not found.\n");
      return -EPERM;
    }

  sys_lock_mutex(&g_list_mtx);

  cb_block = altcombs_search_cbblock(g_lteapi_callback_list, id);
  if (cb_block)
    {
      *callback = cb_block->cb;

      altcombs_remove_cbblock(&g_lteapi_callback_list, cb_block);
    }
  else
    {
      ret = -EINVAL;
    }

  sys_unlock_mutex(&g_list_mtx);

  return ret;
}

/****************************************************************************
 * Name: altcomcallbacks_chk_reg_cb
 *
 * Description:
 *   Check and registration altcom API callback.
 *
 * Input Parameters:
 *   cb         Pointer of altcom API callback function.
 *   id         Callback function id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_chk_reg_cb(void *cb, int32_t id)
{
  int32_t                       ret = 0;

  if (!g_isinit)
    {
      DBGIF_LOG_WARNING("ALTCOM callbacks not initialized.\n");
      return -EPERM;
    }

  sys_lock_mutex(&g_list_mtx);

  if (altcombs_search_cbblock(g_lteapi_callback_list, id))
    {
      ret = -EALREADY;
    }
  else
    {
      ret = altcombs_add_cbblock(&g_lteapi_callback_list, id, cb);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("altcombs_add_cbblock() %d.\n", ret);
        }
    }

  sys_unlock_mutex(&g_list_mtx);

  return ret;
}
