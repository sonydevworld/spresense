/****************************************************************************
 * modules/lte/altcom/api/ltebuilder.c
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
#include "buffpoolwrapper.h"
#include "wrkrid.h"
#include "thrdfctry.h"
#include "evtdispfctry.h"
#include "evtdispid.h"
#include "ltebuilder.h"
#include "hal_altmdm_spi.h"
#include "apicmdgw.h"
#include "altcom_callbacks.h"

#include "apicmdhdlr_enterpin.h"
#include "apicmdhdlr_errind.h"
#include "apicmdhdlr_getce.h"
#include "apicmdhdlr_getedrx.h"
#include "apicmdhdlr_getltime.h"
#include "apicmdhdlr_getpinset.h"
#include "apicmdhdlr_getpsm.h"
#include "apicmdhdlr_imei.h"
#include "apicmdhdlr_imsi.h"
#include "apicmdhdlr_operator.h"
#include "apicmdhdlr_phoneno.h"
#include "lte_power.h"
#include "apicmdhdlr_repcellinfo.h"
#include "apicmdhdlr_repevt.h"
#include "apicmdhdlr_repquality.h"
#include "apicmdhdlr_setce.h"
#include "apicmdhdlr_setedrx.h"
#include "apicmdhdlr_setpin.h"
#include "apicmdhdlr_setpsm.h"
#include "apicmdhdlr_getquality.h"
#include "apicmdhdlr_ver.h"
#include "lte_radio_on.h"
#include "lte_radio_off.h"
#include "lte_activatepdn.h"
#include "lte_deactivatepdn.h"
#include "lte_dataallow.h"
#include "lte_getnetinfo.h"
#include "lte_rep_netinfo.h"
#include "lte_getimscap.h"
#include "lte_getsiminfo.h"
#include "lte_getdynamicedrx.h"
#include "lte_getdynamicpsm.h"
#include "lte_geterrinfo.h"
#include "apicmdhdlr_select.h"
#ifdef CONFIG_LTE_NET_MBEDTLS
#include "apicmdhdlr_config_verify_callback.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICALLBACK_THRD_STACKSIZE     (2048)
#define APICALLBACK_THRD_PRIO          SYS_TASK_PRIO_NORMAL
#define APICALLBACK_THRD_QNUM          (16)  /* tentative */

#define RESTARTCALLBACK_THRD_STACKSIZE (1024)
#define RESTARTCALLBACK_THRD_PRIO      SYS_TASK_PRIO_NORMAL
#define RESTARTCALLBACK_THRD_QNUM      (2)

#define THRDSETLIST_NUM                (2)

#define BLOCKSETLIST_NUM (sizeof(g_blk_settings) / sizeof(g_blk_settings[0]))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static CODE int32_t lte_buildmain(FAR void *arg);
static CODE int32_t lte_destroy(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct buffpool_blockset_s g_blk_settings[] =
{
  {
      16, 64
  },
  {
      32, 48
  },
  {
     128,  4
  },
  {
     512,  6
  },
  {
    2064,  1
  },
#ifdef CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE
  {
    CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE, 1
  },
#endif
  {
    APICMDGW_RECVBUFF_SIZE_MAX, 2
  }
};

static struct evtdisp_s *g_evtdips_obj;

static evthdl_if_t g_apicmdhdlrs[] = 
{
  apicmdhdlr_power,
  apicmdhdlr_ver,
  apicmdhdlr_imei,
  apicmdhdlr_getltime,
  apicmdhdlr_repcellinfo,
  apicmdhdlr_repquality,
  apicmdhdlr_enterpin,
  apicmdhdlr_errindication,
  apicmdhdlr_getpinset,
  apicmdhdlr_imsi,
  apicmdhdlr_operator,
  apicmdhdlr_phoneno,
  apicmdhdlr_setpin,
  apicmdhdlr_repevt,
  apicmdhdlr_getedrx,
  apicmdhdlr_setedrx,
  apicmdhdlr_getpsm,
  apicmdhdlr_setpsm,
  apicmdhdlr_getce,
  apicmdhdlr_setce,
  apicmdhdlr_radioon,
  apicmdhdlr_radiooff,
  apicmdhdlr_activatepdn,
  apicmdhdlr_deactivatepdn,
  apicmdhdlr_dataallow,
  apicmdhdlr_repnetinfo,
  apicmdhdlr_getnetinfo,
  apicmdhdlr_getimscap,
  apicmdhdlr_getsiminfo,
  apicmdhdlr_getdynamicedrx,
  apicmdhdlr_getdynamicpsm,
  apicmdhdlr_errinfo,
  apicmdhdlr_select,
  apicmdhdlr_getquality,
#ifdef CONFIG_LTE_NET_MBEDTLS
  apicmdhdlr_config_verify_callback,
#endif
  EVTDISP_EVTHDLLIST_TERMINATION
};

static FAR struct hal_if_s *g_halif;

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct builder_if_s g_ltebuilder =
{
  .buildmain = lte_buildmain,
  .buildsub1 = NULL,
  .buildsub2 = NULL,
  .buildsub3 = NULL,
  .destroy = lte_destroy
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bufferpool_initialize
 *
 * Description:
 *   Initialize buffer pool for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t bufferpool_initialize(void)
{
  int32_t ret = 0;
  FAR struct buffpool_blockset_s *pset = &g_blk_settings[0];

  if (pset->size)
    {
      ret = buffpoolwrapper_init(g_blk_settings, BLOCKSETLIST_NUM);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("buffpoolwrapper_init() error :%d.\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bufferpool_uninitialize
 *
 * Description:
 *   Uninitialize buffer pool.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t bufferpool_uninitialize(void)
{
  int32_t ret;

  ret = buffpoolwrapper_fin();
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("buffpoolwrapper_fin() error :%d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: workerthread_initialize
 *
 * Description:
 *   Initialize worker thread for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t workerthread_initialize(void)
{
  int32_t                    ret;
  struct thrdfctry_thrdset_s settings[THRDSETLIST_NUM];

  /* worker thread settings for API callback */

  settings[WRKRID_API_CALLBACK_THREAD].id
    = WRKRID_API_CALLBACK_THREAD;
  settings[WRKRID_API_CALLBACK_THREAD].type
    = THRDFCTRY_SEQUENTIAL;
  settings[WRKRID_API_CALLBACK_THREAD].u.seqset.thrdstacksize
    = APICALLBACK_THRD_STACKSIZE;
  settings[WRKRID_API_CALLBACK_THREAD].u.seqset.thrdpriority
    = APICALLBACK_THRD_PRIO;
  settings[WRKRID_API_CALLBACK_THREAD].u.seqset.maxquenum
    = APICALLBACK_THRD_QNUM;

  /* worker thread settings for restart callback */

  settings[WRKRID_RESTART_CALLBACK_THREAD].id
    = WRKRID_RESTART_CALLBACK_THREAD;
  settings[WRKRID_RESTART_CALLBACK_THREAD].type
    = THRDFCTRY_SEQUENTIAL;
  settings[WRKRID_RESTART_CALLBACK_THREAD].u.seqset.thrdstacksize
    = RESTARTCALLBACK_THRD_STACKSIZE;
  settings[WRKRID_RESTART_CALLBACK_THREAD].u.seqset.thrdpriority
    = RESTARTCALLBACK_THRD_PRIO;
  settings[WRKRID_RESTART_CALLBACK_THREAD].u.seqset.maxquenum
    = RESTARTCALLBACK_THRD_QNUM;

  ret = thrdfctry_init(settings, THRDSETLIST_NUM);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("thrdfctry_init() error :%d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: workerthread_uninitialize
 *
 * Description:
 *   Uninitialize worker thread.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t workerthread_uninitialize(void)
{
  int32_t ret;

  ret = thrdfctry_fin();
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("thrdfctry_fin() error :%d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: eventdispatcher_initialize
 *
 * Description:
 *   Initialize event dispatcher for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t eventdispatcher_initialize(void)
{
  int ret = 0;
  struct evtdispfctry_evtdispset_s set[] =
  {
    { EVTDISPID_APICMD_DISP_ID, g_apicmdhdlrs }
  };

  ret = evtdispfctry_init(set, sizeof(set) / sizeof(set[0]));
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("evtdispfctry_init() error.\n");
      return -1;
    }

  g_evtdips_obj = evtdispfctry_get_instance(EVTDISPID_APICMD_DISP_ID);
  if (!g_evtdips_obj)
    {
      DBGIF_LOG1_ERROR("evtdispfctry_get_instance() error. id = \n",
        EVTDISPID_APICMD_DISP_ID);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: eventdispatcher_uninitialize
 *
 * Description:
 *   Uninitialize event dispatcher.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t eventdispatcher_uninitialize(void)
{
  int32_t ret;

  ret = evtdispfctry_fin();
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("evtdisp_delete() error :%d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: halspi_initialize
 *
 * Description:
 *   Initialize HAL altmdm spi I/F for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t halspi_initialize(void)
{
  int32_t ret = 0;

  g_halif = hal_altmdm_spi_create();
  if (!g_halif)
    {
      DBGIF_LOG_ERROR("hal_altmdm_spi_create() error.\n");
      ret = -1;
    }

  lte_power_set_hal_instance(g_halif);

  return ret;
}

/****************************************************************************
 * Name: halspi_uninitialize
 *
 * Description:
 *   Uninitialize HAL altmdm spi I/F.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t halspi_uninitialize(void)
{
  int32_t ret;

  lte_power_set_hal_instance(NULL);

  ret = hal_altmdm_spi_delete(g_halif);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("hal_altmdm_spi_delete() error :%d.\n", ret);
      ret = -1;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdgw_initialize
 *
 * Description:
 *   Initialize HAL altmdm spi I/F for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t apicmdgw_initialize(void)
{
  int32_t ret = 0;
  struct apicmdgw_set_s set;

  if (!g_evtdips_obj || !g_halif)
    {
      ret = -1;
    }

  set.dispatcher = g_evtdips_obj;
  set.halif      = g_halif;
  ret = apicmdgw_init(&set);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("apicmdgw_init() error :%d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdgw_uninitialize
 *
 * Description:
 *   Initialize HAL altmdm spi I/F for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t apicmdgw_uninitialize(void)
{
  int32_t ret;

  ret = apicmdgw_fin();
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("apicmdgw_init() error :%d.\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lte_buildmain
 *
 * Description:
 *   Initialize the resource which is the mandatory used in the library.
 *
 * Input Parameters:
 *   arg  Arguments required for library initialization.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static CODE int32_t lte_buildmain(FAR void *arg)
{
  int32_t ret;

  ret = bufferpool_initialize();
  if (ret < 0)
    {
      goto errout;
    }

  ret = workerthread_initialize();
  if (ret < 0)
    {
      goto errout_with_buffpl;
    }

  ret = eventdispatcher_initialize();
  if (ret < 0)
    {
      goto errout_with_workerthread;
    }

  ret = halspi_initialize();
  if (ret < 0)
    {
      goto errout_with_evtdispatcher;
    }

  ret = apicmdgw_initialize();
  if (ret < 0)
    {
      goto errout_with_halspi;
    }

  ret = altcomcallbacks_init();
  if (ret < 0)
    {
      goto errout_with_apicmdgw;
    }

  return 0;

errout_with_apicmdgw:
  (void)apicmdgw_uninitialize();

errout_with_halspi:
  (void)halspi_uninitialize();

errout_with_evtdispatcher:
  (void)eventdispatcher_uninitialize();
  
errout_with_workerthread:
  (void)workerthread_uninitialize();

errout_with_buffpl:
  (void)bufferpool_uninitialize();

errout:
  return ret;
}

/****************************************************************************
 * Name: lte_destroy
 *
 * Description:
 *   Uninitalize the resources used in the library.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static CODE int32_t lte_destroy(void)
{
  int32_t ret;

  ret = altcomcallbacks_fin();
  if (ret < 0)
    {
      return ret;
    }

  ret = apicmdgw_uninitialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = halspi_uninitialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = eventdispatcher_uninitialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = workerthread_uninitialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = bufferpool_uninitialize();
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}
