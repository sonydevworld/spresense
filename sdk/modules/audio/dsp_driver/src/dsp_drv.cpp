/****************************************************************************
 * modules/audio/dsp_driver/src/dsp_drv.cpp
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

#include <sdk/config.h>
#include <nuttx/compiler.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include "dsp_drv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FULLPATH  64

/* MP object keys. Must be synchronized with worker. */

#define KEY_MQ 2

#ifndef CONFIG_ASMP
#  error "ASMP support library is not enabled!"
#endif

/* If CONFIG_DEBUG_FEATURES is enabled, use _info or _err instead of printf
 * so that the output will be synchronous with the debug output.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG_FEATURES
#    define message(format, ...)    _info(format, ##__VA_ARGS__)
#    define err(format, ...)        _err(format, ##__VA_ARGS__)
#  else
#    define message(format, ...)
#    define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG_FEATURES
#    define message                 _info
#    define err                     _err
#  else
#    define message
#    define err                     printf
#  endif
#endif

#define CXD56_DSP_DRV_SCHED_PRIORITY 200

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Symbols from Auto-Generated Code
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern "C" CODE void *dd_receiver_thread(FAR void *p_instance)
{
  do
    {
      ((DspDrv*)p_instance)->receive();
    }
  while(1);

  return 0;
}

/*--------------------------------------------------------------------------*/
int DspDrv::init(FAR const char  *pfilename,
                 DspDoneCallback p_cbfunc,
                 FAR void        *p_parent_instance,
                 dsp_bin_type_e  bintype)
{
  int   ret;
  int   errout_ret;
  char  *dsp;
  pthread_attr_t attr;
  struct sched_param sch_param;

  m_p_cb_func = p_cbfunc;
  m_p_parent_instance = p_parent_instance;

  if (bintype == DspBinTypeSPK)
    {
      /* Initialize MP task. */

      ret = mptask_init_secure(&m_mptask, pfilename);
      if (ret < 0)
        {
          err("mptask_init_secure() failure. %d\n", ret);
          return DSPDRV_INIT_MPTASK_FAIL;
        }
    }
  else
    {
      /* Initialize MP task. */

      ret = mptask_init(&m_mptask, pfilename);
      if (ret < 0)
        {
          err("mptask_init() failure. %d\n", ret);
          return DSPDRV_INIT_MPTASK_FAIL;
        }
    }

  ret = mptask_assign(&m_mptask);
  if (ret < 0)
    {
      err("mptask_assign() failure. %d\n", ret);
      return DSPDRV_INIT_MPTASK_FAIL;
    }

  /* Initialize MP message queue with asigned CPU ID,
   * and bind it to MP task.
   */

  ret = mpmq_init(&m_mq, KEY_MQ, mptask_getcpuid(&m_mptask));
  if (ret < 0)
    {
      err("mpmq_init() failure. %d\n", ret);
      errout_ret = DSPDRV_INIT_MPMQ_FAIL;
      goto dsp_drv_errout_with_mptask_destroy;
    }

  if (bintype == DspBinTypeELF)
    {
      ret = mptask_bindobj(&m_mptask, &m_mq);
      if (ret < 0)
        {
          err("mptask_bindobj() failure. %d\n", ret);
          return ret;
        }
    }

  /* Create receive thread. */

  (void)pthread_attr_init(&attr);
  sch_param.sched_priority = CXD56_DSP_DRV_SCHED_PRIORITY;
  ret = pthread_attr_setschedparam(&attr, &sch_param);
  if (ret != 0)
    {
      err("pthread_attr_setschedparam() failure. %d\n", ret);
      errout_ret = DSPDRV_INIT_PTHREAD_FAIL;
      goto dsp_drv_errout_with_mpmq_destory;
    }

  ret = pthread_create(&m_thread_id,
                       &attr,
                       dd_receiver_thread,
                       (pthread_addr_t)this);
  if (ret != 0)
    {
      err("pthread_create() failure. %d\n", ret);
      errout_ret = DSPDRV_INIT_PTHREAD_FAIL;
      (void)pthread_attr_destroy(&attr);
      goto dsp_drv_errout_with_mpmq_destory;
    }

  dsp = strrchr(pfilename, '/');
  if (dsp)
    {
      pthread_setname_np(m_thread_id, dsp + 1);
    }
  else
    {
      pthread_setname_np(m_thread_id, pfilename);
    }

  (void)pthread_attr_destroy(&attr);

  /* Run worker. */

  ret = mptask_exec(&m_mptask);
  if (ret < 0)
    {
      err("mptask_exec() failure. %d\n", ret);
      errout_ret = DSPDRV_INIT_MPTASK_FAIL;
      ret = pthread_cancel(m_thread_id);
      DEBUGASSERT(ret == 0);

      ret = pthread_join(m_thread_id, NULL);
      DEBUGASSERT(ret == 0);
    }
  else
    {
      return DSPDRV_NOERROR;
    }

dsp_drv_errout_with_mpmq_destory:
  ret = mpmq_destroy(&m_mq);
  DEBUGASSERT(ret == 0);

dsp_drv_errout_with_mptask_destroy:
  ret = mptask_destroy(&m_mptask, false, NULL);
  DEBUGASSERT(ret == 0);

  return errout_ret;
}

/*--------------------------------------------------------------------------*/
int DspDrv::destroy(bool force)
{

  int ret;

  /* Cancel thread. */

  ret = pthread_cancel(m_thread_id);
  if (ret != 0)
    {
      err("pthread_cancel() failure. %d\n", ret);
      return DSPDRV_INIT_PTHREAD_FAIL;
    }

  ret = pthread_join(m_thread_id, NULL);
  if (ret != 0)
    {
      err("pthread_join() failure. %d\n", ret);
      return DSPDRV_INIT_PTHREAD_FAIL;
    }

  /* Destroy worker. */

  ret = mptask_destroy(&m_mptask, force, NULL);
  if (ret < 0)
    {
      err("mptask_destroy() failure. %d\n", ret);
      return DSPDRV_INIT_MPTASK_FAIL;
    }

  /* Finalize all of MP objects. */

  ret = mpmq_destroy(&m_mq);
  if (ret < 0)
    {
      err("mpmq_destroy() failure. %d\n", ret);
      return DSPDRV_INIT_MPMQ_FAIL;
    }

  return DSPDRV_NOERROR;
}

/*--------------------------------------------------------------------------*/
int DspDrv::send(FAR const DspDrvComPrm_t *p_param)
{
  int     ret;
  uint8_t command;

  command =
    (p_param->process_mode << 4) + (p_param->event_type << 1) + p_param->type;

  /* Send command to worker. */

  ret = mpmq_timedsend(&m_mq, command, p_param->data.value, 1000);
  if (ret < 0)
    {
      err("mpmq_send() failure. %d\n", ret);
      return DSPDRV_INIT_MPMQ_FAIL;
    }

  return DSPDRV_NOERROR;
}

/*--------------------------------------------------------------------------*/
int DspDrv::receive()
{
  int      command;
  uint32_t msgdata;
  bool active = true;

  /* Wait for worker message. */

  while (active)
    {
      command = mpmq_receive(&m_mq, &msgdata);
      if (command < 0)
        {
          err("mpmq_recieve() failure. command(%d) < 0\n", command);
          return DSPDRV_INIT_MPMQ_FAIL;
        }

      DspDrvComPrm_t param;
      param.process_mode = (command >> 4) & 0xf;
      param.event_type   = (command >> 1) & 0x7;
      param.type         = (command >> 0) & 0x1;
      param.data.value   = msgdata;
      m_p_cb_func((FAR void *)&param, m_p_parent_instance);

      if (param.event_type == 7)
        {
          active = false;
        }
    }

  return DSPDRV_NOERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int DD_Load(FAR const char  *filename,
            DspDoneCallback p_cbfunc,
            FAR void        *p_parent_instance,
            FAR void        **dsp_handler,
            dsp_bin_type_e  bintype)
{
  if (filename == NULL)
    {
      return DSPDRV_FILENAME_EMPTY;
    }
  if (p_cbfunc == NULL)
    {
      return DSPDRV_CALLBACK_ERROR;
    }
  if (dsp_handler == NULL)
    {
      return DSPDRV_INVALID_VALUE;
    }
  FAR DspDrv *p_instance = new DspDrv;
  *dsp_handler = p_instance;

  if (p_instance == NULL)
    {
      return DSPDRV_CREATE_FAIL;
    }
  else
    {
      /* Initialize DspDriver. */

      int ret = p_instance->init(filename,
                                 p_cbfunc,
                                 p_parent_instance,
                                 bintype);
      if (ret != DSPDRV_NOERROR)
        {
          delete ((FAR DspDrv*)p_instance);
          return ret;
        }
    }
  return DSPDRV_NOERROR;
}

/*--------------------------------------------------------------------------*/
int DD_Load_Secure(FAR const char  *filename,
                   DspDoneCallback p_cbfunc,
                   FAR void        *p_parent_instance,
                   FAR void        **dsp_handler)
{
  return DD_Load(filename, p_cbfunc, p_parent_instance, dsp_handler, DspBinTypeSPK);
}

/*--------------------------------------------------------------------------*/
int DD_SendCommand(FAR const void           *p_instance,
                   FAR const DspDrvComPrm_t *p_param)
{
  if (p_instance == NULL || p_param == NULL)
    {
      return DSPDRV_INVALID_VALUE;
    }

  int ret = ((FAR DspDrv*)p_instance)->send(p_param);

  return ret;
}

/*--------------------------------------------------------------------------*/
int DD_Unload(FAR const void *p_instance)
{
  if (p_instance == NULL)
    {
      return DSPDRV_INVALID_VALUE;
    }

  int ret = ((FAR DspDrv*)p_instance)->destroy(false);
  if (ret == DSPDRV_NOERROR)
    {
      delete ((FAR DspDrv*)p_instance);
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
int DD_force_Unload(FAR const void *p_instance)
{
  if (p_instance == NULL)
    {
      return DSPDRV_INVALID_VALUE;
    }

  int ret = ((FAR DspDrv*)p_instance)->destroy(true);
  if (ret == DSPDRV_NOERROR)
    {
      delete ((FAR DspDrv*)p_instance);
    }

  return ret;
}

