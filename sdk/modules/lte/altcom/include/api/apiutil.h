/****************************************************************************
 * modules/lte/altcom/include/api/apiutil.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_APIUTIL_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_APIUTIL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include "osal.h"
#include "thrdpool.h"
#include "dbg_if.h"
#include "apicmdgw.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "buffpoolwrapper.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define altcom_lock()            do { sys_disable_dispatch(); } while(0)
#define altcom_unlock()          do { sys_enable_dispatch(); } while(0)

#define ALTCOM_REG_CALLBACK(ret, tgt_callback, new_callback) \
  do \
    { \
      altcom_callback_lock(); \
      if (tgt_callback) \
        { \
          ret = -EBUSY; \
        } \
      else \
        { \
          tgt_callback = new_callback; \
          ret = 0; \
        } \
      altcom_callback_unlock(); \
    } \
  while (0)

#define ALTCOM_GET_AND_CLR_CALLBACK(ret, tgt_callback, old_callback) \
  do \
    { \
      altcom_callback_lock(); \
      if (!tgt_callback) \
        { \
          ret = -EPERM; \
        } \
      else \
        { \
          old_callback = tgt_callback; \
          tgt_callback = NULL; \
          ret = 0; \
        } \
      altcom_callback_unlock(); \
    } \
  while (0)

#define ALTCOM_CLR_CALLBACK(tgt_callback) \
  do \
    { \
      altcom_callback_lock(); \
      tgt_callback = NULL; \
      altcom_callback_unlock(); \
    } \
  while (0)

#define ALTCOM_SOCK_ALLOC_CMDBUFF(buff, id ,len) \
  (((buff) = altcom_alloc_cmdbuff(id, len)) != NULL)

#define ALTCOM_SOCK_ALLOC_RESBUFF(buff, len) \
  ((buff = altcom_alloc_resbuff(len)) != NULL)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern bool        g_lte_initialized;
extern sys_mutex_t g_lte_apicallback_mtx;

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline void altcom_free_cmd(FAR uint8_t *dat)
{
  int32_t freeret = apicmdgw_freebuff(dat);
  if (freeret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_freebuff() failure. ret:%d\n", freeret);
    }
}

static inline int32_t altcom_send_and_free(FAR uint8_t *dat)
{
  int32_t ret = APICMDGW_SEND_ONLY(dat);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to send API command. err=[%d]\n", ret);
    }

  altcom_free_cmd(dat);

  return ret;
}

static inline FAR void *altcom_alloc_cmdbuff(int32_t cmdid, uint16_t len)
{
  FAR void *buff = NULL;

  buff = apicmdgw_cmd_allocbuff(cmdid, len);
  if (!buff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
    }
  return buff;
}

static inline FAR void *altcom_alloc_resbuff(uint16_t len)
{
  FAR void *res = NULL;

  res = BUFFPOOL_ALLOC(len);
  if (!res)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      altcom_seterrno((int32_t)ALTCOM_ENOMEM);
    }

  return res;
}

/****************************************************************************
 * Name: altcom_callback_createlock
 *
 * Description:
 *   Create the lock for access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void altcom_callback_createlock(void)
{
  int32_t      ret;
  sys_cremtx_s param = {0};

  ret = sys_create_mutex(&g_lte_apicallback_mtx, &param);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to create mutex :%d\n", ret);
    }
}

/****************************************************************************
 * Name: altcom_callback_deletelock
 *
 * Description:
 *   Delete the lock for access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void altcom_callback_deletelock(void)
{
  int32_t      ret;

  ret = sys_delete_mutex(&g_lte_apicallback_mtx);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to delete mutex :%d\n", ret);
    }
}

/****************************************************************************
 * Name: altcom_callback_lock
 *
 * Description:
 *   Lock access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void altcom_callback_lock(void)
{
  sys_lock_mutex(&g_lte_apicallback_mtx);
}

/****************************************************************************
 * Name: altcom_callback_unlock
 *
 * Description:
 *   Unock access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void altcom_callback_unlock(void)
{
  sys_unlock_mutex(&g_lte_apicallback_mtx);
}

static inline int32_t altcom_check_initialized_and_set(void)
{
  int32_t ret;

  altcom_lock();
  if (g_lte_initialized)
    {
      ret = -EBUSY;
    }
  else
    {
      g_lte_initialized = true;
      altcom_callback_createlock();
      ret = 0;
    }
  altcom_unlock();

  return ret;
}

static inline void altcom_set_initialized(void)
{
  altcom_lock();
  g_lte_initialized = true;
  altcom_callback_createlock();
  altcom_unlock();
}

static inline int32_t altcom_check_finalized_and_set(void)
{
  int32_t ret;

  altcom_lock();
  if (!g_lte_initialized)
    {
      ret = -EPERM;
    }
  else
    {
      g_lte_initialized = false;
      altcom_callback_deletelock();
      ret = 0;
    }
  altcom_unlock();

  return ret;
}

static inline void altcom_set_finalized(void)
{
  altcom_lock();
  g_lte_initialized = false;
  altcom_callback_deletelock();
  altcom_unlock();
}

static inline bool altcom_isinit(void)
{
  int ret;

  ret = altcombs_check_poweron_status();
  if(ret < 0)
    {
      return false;
    }

  return true;
}

static inline void altcom_sock_free_cmdandresbuff(
  FAR void *cmdbuff, FAR void *resbuff)
{
  if (cmdbuff)
    {
      altcom_free_cmd((FAR uint8_t *)cmdbuff);
    }
  if (resbuff)
    {
      (void)BUFFPOOL_FREE(resbuff);
    }
}

static inline bool altcom_sock_alloc_cmdandresbuff(
  FAR void **buff, int32_t id, uint16_t bufflen,
  FAR void **res, uint16_t reslen)
{
  if (!ALTCOM_SOCK_ALLOC_CMDBUFF(*buff, id, bufflen))
    {
      altcom_seterrno((int32_t)ALTCOM_ENOMEM);
      return false;
    }

  if (!ALTCOM_SOCK_ALLOC_RESBUFF(*res, reslen))
    {
      altcom_free_cmd((FAR uint8_t *)*buff);
      altcom_seterrno((int32_t)ALTCOM_ENOMEM);
      return false;
    }

  return true;
}

static inline void altcom_mbedtls_free_cmdandresbuff(
  FAR void *cmdbuff, FAR void *resbuff)
{
  return altcom_sock_free_cmdandresbuff(cmdbuff, resbuff);
}

static inline bool altcom_mbedtls_alloc_cmdandresbuff(
  FAR void **buff, int32_t id, uint16_t bufflen,
  FAR void **res, uint16_t reslen)
{
  return altcom_sock_alloc_cmdandresbuff(buff, id, bufflen, res, reslen);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_runjob
 *
 * Description:
 *  run job to the worker.
 *
 * Input Parameters:
 *  id   workerid
 *  job  workin job pointer.
 *  arg  job parameters pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns APIUTIL_SUCCESS.
 *   Otherwise APIUTIL_FAILURE is returned.
 *
 ****************************************************************************/

int32_t altcom_runjob(
  int8_t id, CODE thrdpool_jobif_t job, FAR void *arg);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_APIUTIL_H */
