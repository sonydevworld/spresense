/****************************************************************************
 * system/lte_sysctl/lte_sysctl.c
 *
 *   Copyright 2020, 2021, 2023 Sony Semiconductor Solutions Corporation
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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <nuttx/wireless/lte/lte_ioctl.h>

#include "lte/lapi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SZ
#  define ARRAY_SZ(array) (sizeof(array)/sizeof(array[0]))
#endif

#define LTE_SYSCTL_STRTOL_BASE     (10)
#define LTE_SYSCTL_STRTOL_BASE_HEX (16)

#define RAT_KEEP 0
#define NPDN 1

/* APN settings */

#ifdef CONFIG_LTE_SYSCTL_APN_NAME
#  define APP_APN_NAME   CONFIG_LTE_SYSCTL_APN_NAME
#else
#  define APP_APN_NAME   "lte_sysctl_sample_apn"
#endif

#ifdef CONFIG_LTE_SYSCTL_APN_IPTYPE_IPV6
#  define APP_APN_IPTYPE   LTE_IPTYPE_V6
#elif defined CONFIG_LTE_SYSCTL_APN_IPTYPE_IPV4V6
#  define APP_APN_IPTYPE   LTE_IPTYPE_V4V6
#elif defined CONFIG_LTE_SYSCTL_APN_IPTYPE_NON
#  define APP_APN_IPTYPE   LTE_IPTYPE_NON
#else
#  define APP_APN_IPTYPE   LTE_IPTYPE_V4
#endif

#ifdef CONFIG_LTE_SYSCTL_APN_AUTHTYPE_PAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_PAP
#elif defined CONFIG_LTE_SYSCTL_APN_AUTHTYPE_CHAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_CHAP
#else
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_NONE
#endif

#ifdef CONFIG_LTE_SYSCTL_APN_USERNAME
#  define APP_APN_USR_NAME CONFIG_LTE_SYSCTL_APN_USERNAME
#else
#  define APP_APN_USR_NAME ""
#endif

#ifdef CONFIG_LTE_SYSCTL_APN_PASSWD
#  define APP_APN_PASSWD   CONFIG_LTE_SYSCTL_APN_PASSWD
#else
#  define APP_APN_PASSWD   ""
#endif

#define ERR_FMT_NUM "%s: %s failed: %d\n"
#define ERR_FMT_STR "%s: %s failed: %s\n"
#define DAEMON_STAT_FMT "Daemon state : %s\n"
#define APN_STAT_FMT "APN\n"
#define APN_NAME_FMT "  Name: %s\n"
#define APN_TYPE_FMT "  IP type: %s\n"
#define APN_AUTH_FMT "  Authentication: %s\n"
#define APN_USER_FMT "  Username: %s\n"
#define APN_PASS_FMT "  Password: %s\n"
#define RAT_FMT "RAT: %s\n"
#define VER_FMT "VER: %s\n"
#define IMEI_FMT "IMEI: %s\n"
#define LTE_SYSCTL_CMD_START "start"
#define LTE_SYSCTL_CMD_STOP "stop"
#define LTE_SYSCTL_CMD_STAT "stat"
#define LTE_SYSCTL_CMD_FRESET "factoryreset"
#define LTE_SYSCTL_CMD_RAT_CATM1 "M1"
#define LTE_SYSCTL_CMD_RAT_NB "NB"
#define STAT_STOPPED "stopped"
#define STAT_RUNNING "running"
#define STAT_SEARCHING "searching"
#define STAT_CONNECTED "connected"
#define IPV4_STR "IPv4"
#define IPV6_STR "IPv6"
#define IPV4V6_STR "IPv4 and IPv6"
#define NONIP_STR "Non-IP"
#define NONE_STR "NONE"
#define PAP_STR "PAP"
#define CHAP_STR "CHAP"
#define NBIOT_STR "NB-IoT"
#define CATM1_STR "CAT-M1"

#define MATCH_STRING(str1, str2) ((strlen(str1) == strlen(str2)) && \
                                  (strncasecmp(str1, str2, strlen(str2)) == 0))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_sem;
static sem_t g_exclsem = SEM_INITIALIZER(1);
static bool g_vererr = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void restart_callback(uint32_t reason)
{
  if (reason == LTE_RESTART_VERSION_ERROR)
    {
      fprintf(stderr, "ERROR: Modem protocol version mismatch.\n"
              "Please enable the disabled protocol version"
              " and flash the application.\n");
      g_vererr = true;
    }

  sem_post(&g_sem);
}

static void show_usage(FAR const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s command\n", progname);
  fprintf(stderr, " [-a <apn_name>] [-i <ip_type>] [-v <auth_type>]"
                  " [-u <user_name>] [-p <password>] [-r <rat_type>]"
                  " start\n");
  fprintf(stderr, "  -a: APN name\n");
  fprintf(stderr, "  -i: IP type \"V4\", \"V6\", \"V4V6\", \"NON\" or"
                  " 0=IPv4, 1=IPv6, 2=IPv4 and IPv6, 3=Non-IP\n");
  fprintf(stderr, "  -v: Authenticaion type \"NONE\", \"PAP\", \"CHAP\", or"
                  " 0=NONE, 1=PAP, 2=CHAP\n");
  fprintf(stderr, "  -u: User name for authenticaion\n");
  fprintf(stderr, "  -p: Password for authenticaion\n");
  fprintf(stderr, "  -r: Radio Access Technology type M1=CAT-M1,"
                  " NB=NB-IoT\n");
  fprintf(stderr, " stop\n");
  fprintf(stderr, " stat\n");
#ifdef CONFIG_LTE_SYSCTL_FACTORY_RESET
  fprintf(stderr, " factoryreset: Reset parameters to the factory default.\n"
                  "               It takes around 30 sec\n");
#endif
  fprintf(stderr, " [-h]: Show this message\n");
  exit(exitcode);
}

static bool is_edrx_enable(lte_edrx_setting_t *val)
{
  if (val == NULL)
    {
      return false;
    }

  if (val->act_type == LTE_EDRX_ACTTYPE_WBS1 ||
      val->act_type == LTE_EDRX_ACTTYPE_NBS1)
    {
      return val->enable;
    }

  return false;
}

static const char *get_edrx_enable(lte_edrx_setting_t *val)
{
  return is_edrx_enable(val) ? "enable" : "disable";
}

static const char *get_edrx_cycle(lte_edrx_setting_t *val)
{
  static const char *cycle_str[LTE_EDRX_CYC_1048576 + 1] =
    {
      "5.12",     /* LTE_EDRX_CYC_512     */
      "10.24",    /* LTE_EDRX_CYC_1024    */
      "20.48",    /* LTE_EDRX_CYC_2048    */
      "40.96",    /* LTE_EDRX_CYC_4096    */
      "61.44",    /* LTE_EDRX_CYC_6144    */
      "81.92",    /* LTE_EDRX_CYC_8192    */
      "102.40",   /* LTE_EDRX_CYC_10240   */
      "122.88",   /* LTE_EDRX_CYC_12288   */
      "143.36",   /* LTE_EDRX_CYC_14336   */
      "163.84",   /* LTE_EDRX_CYC_16384   */
      "327.68",   /* LTE_EDRX_CYC_32768   */
      "655.36",   /* LTE_EDRX_CYC_65536   */
      "1310.72",  /* LTE_EDRX_CYC_131072  */
      "2621.44",  /* LTE_EDRX_CYC_262144  */
      "5242.88",  /* LTE_EDRX_CYC_524288  */
      "10485.76"  /* LTE_EDRX_CYC_1048576 */
    };

  if (is_edrx_enable(val))
    {
      if (val->edrx_cycle <= LTE_EDRX_CYC_1048576)
        {
          return cycle_str[val->edrx_cycle];
        }
    }

  return "-";
}

static const char *get_edrx_ptw(lte_edrx_setting_t *val)
{
  static const char *ptw_str[LTE_EDRX_PTW_4096 + 1] =
    {
      "1.28",   /* LTE_EDRX_PTW_128  */
      "2.56",   /* LTE_EDRX_PTW_256  */
      "3.84",   /* LTE_EDRX_PTW_384  */
      "5.12",   /* LTE_EDRX_PTW_512  */
      "6.40",   /* LTE_EDRX_PTW_640  */
      "7.68",   /* LTE_EDRX_PTW_768  */
      "8.96",   /* LTE_EDRX_PTW_896  */
      "10.24",  /* LTE_EDRX_PTW_1024 */
      "11.52",  /* LTE_EDRX_PTW_1152 */
      "12.80",  /* LTE_EDRX_PTW_1280 */
      "14.08",  /* LTE_EDRX_PTW_1408 */
      "15.36",  /* LTE_EDRX_PTW_1536 */
      "16.64",  /* LTE_EDRX_PTW_1664 */
      "17.92",  /* LTE_EDRX_PTW_1792 */
      "19.20",  /* LTE_EDRX_PTW_1920 */
      "20.48",  /* LTE_EDRX_PTW_2048 */
      "23.04",  /* LTE_EDRX_PTW_2304 */
      "25.60",  /* LTE_EDRX_PTW_2560 */
      "28.16",  /* LTE_EDRX_PTW_2816 */
      "30.72",  /* LTE_EDRX_PTW_3072 */
      "33.28",  /* LTE_EDRX_PTW_3328 */
      "35.84",  /* LTE_EDRX_PTW_3584 */
      "38.40",  /* LTE_EDRX_PTW_3840 */
      "40.96"   /* LTE_EDRX_PTW_4096 */
    };

  if (is_edrx_enable(val))
    {
      if (val->ptw_val <= LTE_EDRX_PTW_4096)
        {
          return ptw_str[val->ptw_val];
        }
    }

  return "-";
}

static void show_edrxsettings(void)
{
  int ret;
  lte_edrx_setting_t dev_edrx;
  lte_edrx_setting_t nego_edrx;

  ret = lte_get_edrx_sync(&dev_edrx);
  if (ret == 0)
    {
      ret = lte_get_current_edrx_sync(&nego_edrx);
      if (ret == 0)
        {
          fprintf(stderr, "eDRX (device settings): %s\n"
                          "  eDRX cycle:         %s seconds\n"
                          "  Paging time window: %s seconds\n",
                          get_edrx_enable(&dev_edrx),
                          get_edrx_cycle(&dev_edrx),
                          get_edrx_ptw(&dev_edrx));
          fprintf(stderr, "eDRX (negotiated with LTE): %s\n"
                          "  eDRX cycle:         %s seconds\n"
                          "  Paging time window: %s seconds\n",
                          get_edrx_enable(&nego_edrx),
                          get_edrx_cycle(&nego_edrx),
                          get_edrx_ptw(&nego_edrx));
        }
      else
        {
          fprintf(stderr, "eDRX (device settings): %s\n"
                          "  eDRX cycle:         %s seconds\n"
                          "  Paging time window: %s seconds\n",
                          get_edrx_enable(&dev_edrx),
                          get_edrx_cycle(&dev_edrx),
                          get_edrx_ptw(&dev_edrx));
        }
    }
}

static bool is_psm_enable(lte_psm_setting_t *val)
{
  if (val == NULL)
    {
      return false;
    }

  if (val->req_active_time.unit == LTE_PSM_T3324_UNIT_DEACT ||
      val->req_active_time.time_val == 0 ||
      val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_DEACT ||
      val->ext_periodic_tau_time.time_val == 0)
    {
      return false;
    }

  return val->enable;
}

static const char *get_psm_enable(lte_psm_setting_t *val)
{
  return is_psm_enable(val) ? "enable" : "disable";
}

static int get_psm_acttimer(lte_psm_setting_t *val)
{
  /* val->req_active_time.time_val: The range is 0 to 31 seconds */

  if (val->req_active_time.unit == LTE_PSM_T3324_UNIT_2SEC)
    {
      return val->req_active_time.time_val * 2;
    }
  else if (val->req_active_time.unit == LTE_PSM_T3324_UNIT_1MIN)
    {
      return val->req_active_time.time_val * 60;
    }
  else if (val->req_active_time.unit == LTE_PSM_T3324_UNIT_6MIN)
    {
      return val->req_active_time.time_val * 6 * 60;
    }

  return 0;
}

static int get_psm_updatetimer(lte_psm_setting_t *val)
{
  /* val->ext_periodic_tau_time.time_val: The range is 0 to 31 seconds */

  if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_2SEC)
    {
      return val->ext_periodic_tau_time.time_val * 2;
    }
  else if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_30SEC)
    {
      return val->ext_periodic_tau_time.time_val * 30;
    }
  else if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_1MIN)
    {
      return val->ext_periodic_tau_time.time_val * 60;
    }
  else if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_10MIN)
    {
      return val->ext_periodic_tau_time.time_val * 10 * 60;
    }
  else if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_1HOUR)
    {
      return val->ext_periodic_tau_time.time_val * 60 * 60;
    }
  else if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_10HOUR)
    {
      return val->ext_periodic_tau_time.time_val * 10 * 60 * 60;
    }
  else if (val->ext_periodic_tau_time.unit == LTE_PSM_T3412_UNIT_320HOUR)
    {
      return val->ext_periodic_tau_time.time_val * 320 * 60 * 60;
    }

  return 0;
}

static void show_psmsettings(void)
{
  int ret;
  lte_psm_setting_t dev_psm;
  lte_psm_setting_t nego_psm;

  ret = lte_get_psm_sync(&dev_psm);
  if (ret == 0)
    {
      ret = lte_get_current_psm_sync(&nego_psm);
      if (ret == 0)
        {
          fprintf(stderr, "PSM (device settings): %s\n"
                          "  Active timer:          %d seconds\n"
                          "  Periodic update timer: %d seconds\n",
                          get_psm_enable(&dev_psm),
                          get_psm_acttimer(&dev_psm),
                          get_psm_updatetimer(&dev_psm));
          fprintf(stderr, "PSM (negotiated with LTE): %s\n"
                          "  Active timer:          %d seconds\n"
                          "  Periodic update timer: %d seconds\n",
                          get_psm_enable(&nego_psm),
                          get_psm_acttimer(&nego_psm),
                          get_psm_updatetimer(&nego_psm));
        }
      else
        {
          fprintf(stderr, "PSM (device settings): %s\n"
                          "  Active timer:          %d seconds\n"
                          "  Periodic update timer: %d seconds\n",
                          get_psm_enable(&dev_psm),
                          get_psm_acttimer(&dev_psm),
                          get_psm_updatetimer(&dev_psm));
        }
    }
}

static void show_siminfo(void)
{
  int ret;
  char buff[LTE_PHONENO_LEN];
  lte_siminfo_t siminfo;
  int i;

  ret = lte_get_imsi_sync(buff, LTE_IMSI_LEN);
  if (ret == 0)
    {
      fprintf(stderr, "SIM: Inserted\n");
      fprintf(stderr, "  IMSI: %s\n", buff);
    }
  else
    {
      fprintf(stderr, "SIM: Not inserted\n");
      return;
    }

  ret = lte_get_phoneno_sync(buff, LTE_PHONENO_LEN);
  if (ret == 0)
    {
      fprintf(stderr, "  Phone number: %s\n", buff);
    }

  ret = lte_get_siminfo_sync(LTE_SIMINFO_GETOPT_ICCID, &siminfo);
  if (ret == 0)
    {
      fprintf(stderr, "  ICCID: ");
      for (i = 0; i < siminfo.iccid_len - 1; i++)
        {
          fprintf(stderr, "%02X", siminfo.iccid[i]);
        }

      if ((siminfo.iccid[i] & 0x0f) == 0x0f)
        {
          fprintf(stderr, "%X\n", siminfo.iccid[i] >> 4);
        }
      else
        {
          fprintf(stderr, "%02X\n", siminfo.iccid[i]);
        }
    }

  ret = lte_get_siminfo_sync(LTE_SIMINFO_GETOPT_MCCMNC, &siminfo);
  if (ret == 0)
    {
      fprintf(stderr, "  MCC MNC: %c%c%c ",
              siminfo.mcc[0], siminfo.mcc[1], siminfo.mcc[2]);
      if (siminfo.mnc_digit == 2)
        {
          fprintf(stderr, "%c%c\n",
                  siminfo.mnc[0], siminfo.mnc[1]);
        }
      else
        {
          fprintf(stderr, "%c%c%c\n",
                  siminfo.mnc[0], siminfo.mnc[1], siminfo.mnc[2]);
        }
    }

  ret = lte_get_siminfo_sync(LTE_SIMINFO_GETOPT_SPN, &siminfo);
  if (ret == 0)
    {
      if (siminfo.spn_len <= sizeof(siminfo.spn))
        {
          char tail = siminfo.spn[siminfo.spn_len - 1];
          siminfo.spn[siminfo.spn_len - 1] = '\0';
          fprintf(stderr, "  Service provider name: %s%c\n",
                  siminfo.spn, tail);
        }
    }
}

static void show_networkinfo(FAR lte_netinfo_t *netinfo)
{
  int i;
  int ret;
  char oper[LTE_OPERATOR_LEN];

  fprintf(stderr, "Network infomation\n");

  for (i = 0; i < netinfo->pdn_stat[0].ipaddr_num; i++)
    {
      fprintf(stderr, "  IP address[%d]: %s\n", i,
              netinfo->pdn_stat[0].address[i].address);
    }

  ret = lte_get_operator_sync(oper, LTE_OPERATOR_LEN);
  if (ret == 0)
    {
      fprintf(stderr, "  Operator: %s\n", oper);
    }
}

static int save_apnsettings(FAR lte_apn_setting_t *apn)
{
  FAR void *inarg[] =
    {
      apn
    };

  return lapi_req(LTE_CMDID_SAVEAPN, (FAR void *)inarg, ARRAY_SZ(inarg),
                  NULL, 0, NULL);
}

static int get_apnsettings(FAR lte_apn_setting_t *apn)
{
  FAR void *outarg[] =
    {
      apn
    };

  return lapi_req(LTE_CMDID_GETAPN, NULL, 0,
                  (FAR void *)outarg, ARRAY_SZ(outarg), NULL);
}

static int start_daemon(FAR const char *progname, FAR lte_apn_setting_t *apn,
                        uint8_t rat)
{
  int ret;
  lte_version_t version =
    {
      0
    };

  sem_init(&g_sem, 0, 0);

  ret = lte_initialize();
  if (ret < 0)
    {
      if (ret == -EALREADY)
        {
          fprintf(stderr, ERR_FMT_STR, progname, LTE_SYSCTL_CMD_START,
                  "daemon is already running");
        }
      else
        {
          fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_START, -ret);
        }

      goto err_out;
    }

  ret = lte_set_report_restart(restart_callback);
  if (ret < 0)
    {
      fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_START, -ret);

      lte_finalize();
      goto err_out;
    }

  ret = lte_power_on();
  if (ret < 0)
    {
      fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_START, -ret);

      lte_finalize();
      goto err_out;
    }

  sem_wait(&g_sem);

  lte_set_report_restart(NULL);

  if (g_vererr)
    {
      g_vererr = false;
      ret = lte_get_version_sync(&version);
      if (ret < 0)
        {
          fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_START, -ret);
        }
      else
        {
          printf("Modem IC Type : %s\n", version.bb_product);
          printf("      FW Ver. : %s\n", version.np_package);
        }

      lte_finalize();
      goto err_out;
    }

  ret = save_apnsettings(apn);
  if (ret < 0)
    {
      fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_START, -ret);
      lte_finalize();
      goto err_out;
    }

  if (rat != RAT_KEEP)
    {
      ret = lte_set_rat_sync(rat, LTE_ENABLE);
      if (ret < 0)
        {
          if (ret == -ENOTSUP)
            {
              fprintf(stderr, ERR_FMT_STR, progname, LTE_SYSCTL_CMD_START,
                      "RAT changes are not supported"
                      " in the FW version of the modem");
            }
          else
            {
              fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_START,
                      -ret);
            }

          lte_finalize();
          goto err_out;
        }
    }

  sem_destroy(&g_sem);

  return 0;

err_out:
  sem_destroy(&g_sem);
  return ret;
}

static void show_daemon_stat(void)
{
  int ret;
  char imei[LTE_IMEI_LEN];
  lte_pdn_t pdnstat[NPDN] =
    {
      0
    };

  lte_netinfo_t info =
    {
      .pdn_stat = pdnstat
    };

  ret = lte_get_netinfo_sync(NPDN, &info);
  if (ret < 0)
    {
      fprintf(stderr, DAEMON_STAT_FMT, STAT_STOPPED);
    }
  else
    {
      lte_apn_setting_t apn;
      lte_version_t ver;

      if (info.pdn_num != 0)
        {
          fprintf(stderr, DAEMON_STAT_FMT, STAT_CONNECTED);
        }
      else if (info.nw_stat == LTE_NETSTAT_NOT_REG_SEARCHING)
        {
          fprintf(stderr, DAEMON_STAT_FMT, STAT_SEARCHING);
        }
      else
        {
          fprintf(stderr, DAEMON_STAT_FMT, STAT_RUNNING);
        }

      ret = get_apnsettings(&apn);
      if (ret == 0)
        {
          fprintf(stderr, APN_STAT_FMT);
          fprintf(stderr, APN_NAME_FMT, apn.apn);
          if (apn.ip_type == LTE_IPTYPE_V4)
            {
              fprintf(stderr, APN_TYPE_FMT, IPV4_STR);
            }
          else if (apn.ip_type == LTE_IPTYPE_V6)
            {
              fprintf(stderr, APN_TYPE_FMT, IPV6_STR);
            }
          else if (apn.ip_type == LTE_IPTYPE_V4V6)
            {
              fprintf(stderr, APN_TYPE_FMT, IPV4V6_STR);
            }
          else if (apn.ip_type == LTE_IPTYPE_NON)
            {
              fprintf(stderr, APN_TYPE_FMT, NONIP_STR);
            }

          if (apn.auth_type == LTE_APN_AUTHTYPE_NONE)
            {
              fprintf(stderr, APN_AUTH_FMT, NONE_STR);
            }
          else if (apn.auth_type == LTE_APN_AUTHTYPE_PAP)
            {
              fprintf(stderr, APN_AUTH_FMT, PAP_STR);
            }
          else if (apn.auth_type == LTE_APN_AUTHTYPE_CHAP)
            {
              fprintf(stderr, APN_AUTH_FMT, CHAP_STR);
            }

          fprintf(stderr, APN_USER_FMT, apn.user_name);
          fprintf(stderr, APN_PASS_FMT, apn.password);
        }

      ret = lte_get_rat_sync();
      if (ret == LTE_RAT_NBIOT)
        {
          fprintf(stderr, RAT_FMT, NBIOT_STR);
        }
      else
        {
          fprintf(stderr, RAT_FMT, CATM1_STR);
        }

      ret = lte_get_version_sync(&ver);
      if (ret == 0)
        {
          fprintf(stderr, VER_FMT, ver.np_package);
        }

      ret = lte_get_imei_sync(imei, LTE_IMEI_LEN);
      if (ret == 0)
        {
          fprintf(stderr, IMEI_FMT, imei);
        }

      show_edrxsettings();
      show_psmsettings();
      show_siminfo();

      if (info.pdn_num != 0)
        {
          show_networkinfo(&info);
        }
    }
}

#ifdef CONFIG_LTE_SYSCTL_FACTORY_RESET
static int factory_reset(FAR const char *progname)
{
  int ret;
  lte_version_t version =
    {
      0
    };

  sem_init(&g_sem, 0, 0);

  ret = lte_initialize();
  if (ret < 0)
    {
      if (ret == -EALREADY)
        {
          fprintf(stderr, ERR_FMT_STR, progname, LTE_SYSCTL_CMD_FRESET,
                  "Please stop daemon before run factory reset");
        }
      else
        {
          fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_FRESET,
                  -ret);
        }

      goto err_out;
    }

  ret = lte_set_report_restart(restart_callback);
  if (ret < 0)
    {
      fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_FRESET, -ret);

      lte_finalize();
      goto err_out;
    }

  ret = lte_power_on();
  if (ret < 0)
    {
      fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_FRESET, -ret);

      lte_finalize();
      goto err_out;
    }

  sem_wait(&g_sem);

  if (g_vererr)
    {
      g_vererr = false;
      ret = lte_get_version_sync(&version);
      if (ret < 0)
        {
          fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_FRESET,
                  -ret);
        }
      else
        {
          printf("Modem IC Type : %s\n", version.bb_product);
          printf("      FW Ver. : %s\n", version.np_package);
        }

      lte_finalize();
      goto err_out;
    }

  /* Acquire the wakelock during factory reset running */

  ret = lte_acquire_wakelock();
  if (ret <= 0)
    {
      fprintf(stderr, ERR_FMT_STR, progname, LTE_SYSCTL_CMD_FRESET,
              "Failed to acquire a wakelock");

      lte_finalize();
      goto err_out;
    }

  ret = lte_factory_reset_sync();
  if (ret < 0)
    {
      if (ret == -ENOTSUP)
        {
          fprintf(stderr, ERR_FMT_STR, progname, LTE_SYSCTL_CMD_FRESET,
                  "Factory reset is not supported in the FW version of"
                  " the modem");
        }
      else
        {
          fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_FRESET,
                  -ret);
        }

      lte_release_wakelock();
      lte_finalize();
      goto err_out;
    }

  fprintf(stdout, "Factory reset running...\n"
                  "Please do not turn off the device. Factory reset"
                  " takes around 30 sec.\n");

  sem_wait(&g_sem);

  lte_release_wakelock();

  lte_set_report_restart(NULL);

  sem_destroy(&g_sem);

  ret = lte_finalize();
  if (ret < 0)
    {
      fprintf(stderr, ERR_FMT_NUM, progname, LTE_SYSCTL_CMD_FRESET, -ret);

      return ret;
    }

  fprintf(stdout, "Factory reset done.\n");

  return 0;

err_out:
  sem_destroy(&g_sem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int               ret         = 0;
  int               opt         = 0;
  char              *cmd        = NULL;
  char              *rat_str    = NULL;
  uint8_t           rat         = RAT_KEEP;
  long apn_type;
  long ip_type;
  long auth_type;
  lte_apn_setting_t setting_apn =
    {
    };

  setting_apn.apn       = APP_APN_NAME;
  setting_apn.apn_type  = LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA;
  setting_apn.ip_type   = APP_APN_IPTYPE;
  setting_apn.auth_type = APP_APN_AUTHTYPE;
  setting_apn.user_name = APP_APN_USR_NAME;
  setting_apn.password  = APP_APN_PASSWD;

  optind = -1;
  while ((opt = getopt(argc, argv, "a:t:i:v:u:p:r:h")) != -1)
    {
      switch (opt)
        {
          case 'a':
            if (strlen(optarg) >= LTE_APN_LEN)
              {
                fprintf(stderr, "APN name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.apn = optarg;
            break;
          case 't':
            apn_type = strtol(optarg, NULL, LTE_SYSCTL_STRTOL_BASE_HEX);
            if (apn_type != (LTE_APN_TYPE_IA | LTE_APN_TYPE_DEFAULT))
              {
                fprintf(stderr, "Currently supported APN type is 0x%x"
                        ": LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA\n",
                        LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA);
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.apn_type = (uint32_t)apn_type;
            break;
          case 'i':
            if (MATCH_STRING(optarg, "V4"))
              {
                ip_type = LTE_IPTYPE_V4;
              }
            else if (MATCH_STRING(optarg, "V6"))
              {
                ip_type = LTE_IPTYPE_V6;
              }
            else if (MATCH_STRING(optarg, "V4V6"))
              {
                ip_type = LTE_IPTYPE_V4V6;
              }
            else if (MATCH_STRING(optarg, "NON"))
              {
                ip_type = LTE_IPTYPE_NON;
              }
            else
              {
                if (optarg[1] == '\0' && (optarg[0] >= '0' && optarg[0] <= '3'))
                  {
                    ip_type = atoi(optarg);
                  }
                else
                  {
                    fprintf(stderr, "Invalid IP type: %s\n", optarg);
                    show_usage(argv[0], EXIT_FAILURE);
                  }
              }

            setting_apn.ip_type = (uint8_t)ip_type;
            break;
          case 'v':
            if (MATCH_STRING(optarg, "NONE"))
              {
                auth_type = LTE_APN_AUTHTYPE_NONE;
              }
            else if (MATCH_STRING(optarg, "PAP"))
              {
                auth_type = LTE_APN_AUTHTYPE_PAP;
              }
            else if (MATCH_STRING(optarg, "CHAP"))
              {
                auth_type = LTE_APN_AUTHTYPE_CHAP;
              }
            else
              {
                if (optarg[1] == '\0' && (optarg[0] >= '0' && optarg[0] <= '2'))
                  {
                    auth_type = atoi(optarg);
                  }
                else
                  {
                    fprintf(stderr, "Invalid authenticaion type: %s\n", optarg);
                    show_usage(argv[0], EXIT_FAILURE);
                  }
              }

            setting_apn.auth_type = (uint8_t)auth_type;
            break;
          case 'u':
            if (strlen(optarg) >= LTE_APN_USER_NAME_LEN)
              {
                fprintf(stderr, "User name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.user_name = optarg;
            break;
          case 'p':
            if (strlen(optarg) >= LTE_APN_PASSWD_LEN)
              {
                fprintf(stderr, "Password is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.password = optarg;
            break;
          case 'r':
            rat_str = optarg;
            if (MATCH_STRING(rat_str, LTE_SYSCTL_CMD_RAT_CATM1))
              {
                rat = LTE_RAT_CATM;
              }
            else if (MATCH_STRING(rat_str, LTE_SYSCTL_CMD_RAT_NB))
              {
                rat = LTE_RAT_NBIOT;
              }
            else
              {
                fprintf(stderr,
                        "Please set Radio Access Technology: M1 or NB\n");
                show_usage(argv[0], EXIT_FAILURE);
              }
            break;
          case 'h':
            show_usage(argv[0], EXIT_SUCCESS);
            break;
          default:
            show_usage(argv[0], EXIT_FAILURE);
            break;
        }
    }

  if (optind >= argc || argv[1] == NULL)
    {
      fprintf(stderr, "%s: missing required argument(s)\n", argv[0]);
      show_usage(argv[0], EXIT_FAILURE);
    }

  cmd = argv[optind++];

  if (optind < argc)
    {
      fprintf(stderr, "%s: Invalid required argument(s)\n", argv[0]);
      show_usage(argv[0], EXIT_FAILURE);
    }

  if (MATCH_STRING(cmd, LTE_SYSCTL_CMD_START))
    {
      /* Acquire semaphore for exclusive control */

      sem_wait(&g_exclsem);

      ret = start_daemon(argv[0], &setting_apn, rat);

      /* Release semaphore for exclusive control */

      sem_post(&g_exclsem);
    }
  else if (MATCH_STRING(cmd, LTE_SYSCTL_CMD_STOP))
    {
      ret = lte_finalize();
      if (ret < 0)
        {
          if (ret == -EALREADY)
            {
              fprintf(stderr, ERR_FMT_STR, argv[0], cmd,
                      "daemon is not running");
            }
          else
            {
              fprintf(stderr, ERR_FMT_NUM, argv[0], cmd, -ret);
            }

          goto err_out;
        }
    }
  else if (MATCH_STRING(cmd, LTE_SYSCTL_CMD_STAT))
    {
      show_daemon_stat();
    }
#ifdef CONFIG_LTE_SYSCTL_FACTORY_RESET
  else if (MATCH_STRING(cmd, LTE_SYSCTL_CMD_FRESET))
    {
      /* Acquire semaphore for exclusive control */

      sem_wait(&g_exclsem);

      ret = factory_reset(argv[0]);

      /* Release semaphore for exclusive control */

      sem_post(&g_exclsem);
    }
#endif
  else
    {
      fprintf(stderr, "%s: Invalid required argument(s)\n", argv[0]);
      show_usage(argv[0], EXIT_FAILURE);
    }

  return 0;

err_out:
  return ret;
}
