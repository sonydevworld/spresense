/****************************************************************************
 * modules/lte/altcom/api/lte/lte_power.c
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

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/modem/altmdm.h>

#include "lte/lte_api.h"
#include "wrkrid.h"
#include "apicmd.h"
#include "apiutil.h"
#include "altcom_status.h"
#include "lte_report_restart.h"
#include "apicmd_power.h"
#include "apicmdhdlrbs.h"
#include "altcombs.h"
#include "altcom_callbacks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MODEM_DEVICE_PATH
#  define DEV_PATH CONFIG_MODEM_DEVICE_PATH
#else
#  warning "CONFIG_MODEM_DEVICE_PATH not defined"
#  define DEV_PATH "/dev/altmdm"
#endif

#define POWERON_DATA_LEN     (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poweron_status_chg_cb
 *
 * Description:
 *   Notification status change in processing Power on.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t poweron_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat <= ALTCOM_STATUS_INITIALIZED)
    {
      DBGIF_LOG2_INFO("poweron_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_POWER_ON);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: restart_callback_job
 *
 * Description:
 *   This function is an API callback for power.
 *
 * Input Parameters:
 *  arg    Pointer to input argment.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void restart_callback_job(FAR void *arg)
{
  int32_t             ret;
  FAR uint8_t        *cmdbuff;
  power_control_cb_t  callback;

  /* Allocate API command buffer to send */

  cmdbuff = apicmdgw_cmd_allocbuff(APICMDID_POWER_ON, POWERON_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Send API command to modem */

      ret = altcom_send_and_free(cmdbuff);
    }

  /* If fail, execute the callback */

  if (0 > ret)
    {
      ret = altcomcallbacks_get_unreg_cb(APICMDID_POWER_ON,
                                         (void **)&callback);
      if ((ret == 0) && (callback))
        {
          callback(LTE_RESULT_ERROR);
        }
      else
        {
          DBGIF_LOG_WARNING("callback is NULL.\n");
        }

      lte_power_off();
    }
}

/****************************************************************************
 * Name: restart_callback
 *
 * Description:
 *   This function is an restart callback.
 *
 * Input Parameters:
 *  arg    Pointer to input argment.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void restart_callback(uint32_t state)
{
  int ret;

  if (ALTCOM_STATUS_POWER_ON == altcom_get_status())
    {
      /* When receive reset packet in current status power on,
       * This case is modem reset. */

      /* Call restart callback input modem restart */

      lte_set_report_reason(LTE_RESTART_MODEM_INITIATED);

      /* Abort send apicmd for Release waiting sync API responce. */

      apicmdgw_sendabort();
    }
  else
    {
      lte_set_report_reason(LTE_RESTART_USER_INITIATED);
    }

  altcom_set_status(ALTCOM_STATUS_RESTART_ONGOING);

  /* Call the API callback function in the context of worker thread */

  ret = altcom_runjob(WRKRID_RESTART_CALLBACK_THREAD,
                      restart_callback_job, NULL);
  DBGIF_ASSERT(0 == ret, "Failed to job to worker\n");
}

/****************************************************************************
 * Name: poweroff_callback_job
 *
 * Description:
 *   This function is an API callback for power off.
 *
 * Input Parameters:
 *  arg    Pointer to input argment.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void poweroff_callback_job(FAR void *arg)
{
  int32_t            ret;
  power_control_cb_t callback = NULL;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_POWER_OFF,
                                     (void **)&callback);
  if ((0 == ret) && (callback))
    {
      callback(LTE_RESULT_OK);
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }
}

/****************************************************************************
 * Name: modem_powerctrl
 *
 * Description:
 *   Power on or off the modem.
 *
 * Input Parameters:
 *  on  "power on" or "power off".
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t modem_powerctrl(bool on)
{
  int32_t  ret;
  int      fd;
  int      l_errno;
  int      req;

  /* Open the device */

  fd = open(DEV_PATH, O_WRONLY);
  if (0 > fd)
    {
      DBGIF_LOG2_ERROR("Device %s open failure. %d\n", DEV_PATH, fd);
      ret = fd;
    }
  else
    {

      if (on)
        {
          ret = ioctl(fd, MODEM_IOC_PM_ERR_REGISTERCB,
                      (unsigned long)restart_callback);
          if (0 > ret)
            {
              /* Store errno */

              l_errno = errno;
              ret = -l_errno;

              DBGIF_LOG2_ERROR("Failed to ioctl(0x%08x). %d\n", MODEM_IOC_PM_ERR_REGISTERCB, l_errno);
              
            }
        }

      if (on)
        {
          req = MODEM_IOC_POWERON;
        }
      else
        {
          req = MODEM_IOC_POWEROFF;
        }

      /* Power on the modem */

      ret = ioctl(fd, req, 0);
      if (0 > ret)
        {
          /* Store errno */

          l_errno = errno;
          ret = -l_errno;

          DBGIF_LOG2_ERROR("Failed to ioctl(0x%08x). %d\n", req, l_errno);
          
        }

      close(fd);
    }

  return ret;
}

/****************************************************************************
 * Name: poweron_job
 *
 * Description:
 *   This function is an API callback for power on.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void poweron_job(FAR void *arg)
{
  int32_t                               ret;
  int32_t                               result;
  FAR struct apicmd_cmddat_poweronres_s *data =
      (FAR struct apicmd_cmddat_poweronres_s *)arg;
  power_control_cb_t                    callback;

  altcom_set_status(ALTCOM_STATUS_POWER_ON);

  ret = altcomcallbacks_get_unreg_cb(APICMDID_POWER_ON,
                                     (void **)&callback);
  if ((ret == 0) && (callback))
    {
      result = (int32_t)data->result;

      callback(result);
    }
  else
    {
      DBGIF_LOG_WARNING("callback is NULL.\n");
    }

  /* Call report restart callback */

  lte_do_restartcallback();

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb(poweron_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_power_on
 *
 * Description:
 *   Power on modem.
 *
 * Input Parameters:
 *   viod
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_power_on(void)
{
  int32_t ret;
  int32_t state = altcom_get_status();

  /* Check lte status */

  switch (state)
    {
      case ALTCOM_STATUS_INITIALIZED:
        /* Power on the modem */

        ret = modem_powerctrl(LTE_POWERON);
        if (ret == 0)
          {
            altcom_set_status(ALTCOM_STATUS_RESTART_ONGOING);
          }

        break;
      case ALTCOM_STATUS_RESTART_ONGOING:
        ret = -EINPROGRESS;
        break;
      case ALTCOM_STATUS_POWER_ON:
        ret = -EALREADY;
        break;
      case ALTCOM_STATUS_UNINITIALIZED:
      default:
        ret = -EOPNOTSUPP;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: lte_power_off
 *
 * Description:
 *   Power off modem.
 *
 * Input Parameters:
 *   viod
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_power_off(void)
{
  int32_t ret;
  int32_t state = altcom_get_status();

  /* Check lte status */

  switch (state)
    {
      case ALTCOM_STATUS_INITIALIZED:
        ret = -EALREADY;
        break;
      case ALTCOM_STATUS_RESTART_ONGOING:
      case ALTCOM_STATUS_POWER_ON:
        /* Power off the modem */

        ret = modem_powerctrl(LTE_POWEROFF);
        if (ret == 0)
          {
            altcom_set_status(ALTCOM_STATUS_INITIALIZED);
          }

        break;
      case ALTCOM_STATUS_UNINITIALIZED:
      default:
        ret = -EOPNOTSUPP;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: lte_power_control
 *
 * Description:
 *   Control power on/off of modem.
 *
 * Input Parameters:
 *   on       "power on" or "power off".
 *   callback Callback function to notify that power on/off has been
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_power_control(bool on, power_control_cb_t callback)
{
  int32_t ret;
  int32_t state = altcom_get_status();

  /* Return error if callback is NULL */

  if (!callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  if (state == ALTCOM_STATUS_UNINITIALIZED)
    {
      return -EOPNOTSUPP;
    }

 if (LTE_POWERON == on)
   {
      /* Register API callback */

      ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_POWER_ON);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("Currently API is busy.\n");
          return -EINPROGRESS;
        }

      ret = altcomstatus_reg_statchgcb(poweron_status_chg_cb);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
          altcomcallbacks_unreg_cb(APICMDID_POWER_ON);
          return ret;
        }

     /* Power on the modem */

     ret = lte_power_on();

     /* If fail, there is no opportunity to execute the callback,
      * so clear it here. */

     if (0 > ret)
       {
         /* Clear registered callback */

         altcomcallbacks_unreg_cb(APICMDID_POWER_ON);
         altcomstatus_unreg_statchgcb(poweron_status_chg_cb);
         return ret;
       }
   }
 else
   {
      /* Register API callback */

      ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_POWER_OFF);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("Currently API is busy.\n");
          return -EINPROGRESS;
        }

     /* Power off the modem */

     ret = lte_power_off();
     if (0 > ret)
       {
         return ret;
       }

     /* Call the API callback function in the context of worker thread */

     ret = altcom_runjob(WRKRID_API_CALLBACK_THREAD,
                          poweroff_callback_job, NULL);
     DBGIF_ASSERT(0 == ret, "Failed to job to worker\n");
   }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_power
 *
 * Description:
 *   This function is an API command handler for power on result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_POWER_ON_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_power(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_POWER_ON), poweron_job);
}
