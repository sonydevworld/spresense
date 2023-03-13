/****************************************************************************
 * examples/lte_lwm2mstub/app_parameter.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <fsutils/ini.h>

#include <lte/lte_api.h>
#include <lte/lte_lwm2m.h>

#include "app_parameter.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct param_value_s
{
  const char *param;
  int value;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct param_value_s authtype_param[] =
{
  {"NONE", LTE_APN_AUTHTYPE_NONE},
  {"PAP",  LTE_APN_AUTHTYPE_PAP},
  {"CHAP", LTE_APN_AUTHTYPE_CHAP},
  {NULL,   -1},
};

static struct param_value_s iptype_param[] =
{
  {"V4",    LTE_IPTYPE_V4},
  {"V6",    LTE_IPTYPE_V6},
  {"V4V6",  LTE_IPTYPE_V4V6},
  {"NONIP", LTE_IPTYPE_NON},
  {NULL,   -1},
};

static struct param_value_s truefalse_param[] =
{
  {"true",  1},
  {"false", 0},
  {NULL,   -1},
};

static struct param_value_s rat_param[] =
{
  {"CATM", LTE_RAT_CATM},
  {"NB",   LTE_RAT_NBIOT},
  {NULL,   -1},
};

static struct param_value_s secmode_param[] =
{
  {"PSK",  LWM2MSTUB_SECUREMODE_PSK},
  {"NONE", LWM2MSTUB_SECUREMODE_NOSEC},
  {NULL,   -1},
};

static struct app_parameter_s app_param;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: selectable_paramparse
 ****************************************************************************/

static int selectable_paramparse(const char *value,
                                 struct param_value_s *tbl)
{
  for (; tbl->param; tbl++)
    {
      if (!strcmp(value, tbl->param))
        {
          return tbl->value;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: setup_parsedvalue
 ****************************************************************************/

static int setup_parsedvalue(void *user, const char *section,
                    const char *name, const char *value)
{
  struct app_parameter_s *param = (struct app_parameter_s *)user;
  int parsed_param;

  /* Settings of this application is informed by INI file.
   * The INI file needs to contain below items.
   *
   * [LTE]
   * apn_name = <APN Name>          ;Set APN name according to your SIM
   * rat = <RAT>                    ;Select RAT from CATM or NB
   * auth_type = <Auth Type>        ;Select authentication type from
   *                                ; PAP, CHAP or NONE
   * ip_type = <PI type>            ;Select IP type from V4, V6, V4V6, NONIP
   * user_name = <APN user Name>    ;Set user name according to your SIM
   * passwd = <APN password>        ;Set password according to your SIM
   *
   * [LwM2M]
   * server_uri = <URL>             ; LwM2M Server URL to connect
   * bootstrap = <true or false>    ;Set "true" when you use BootStrap server
   * security_mode = <Secure mode>  ; Select Secure mode from NONE or PSK
   * deviceid = <Device ID>         ; Set Device ID
   * security_key = <Security KeyD> ; Set security key
   * lifetime = <Life time>         ; Life time [sec] of resource /1/x/1.
   *
   */

  if (!strcmp(section, "LTE") && !strcmp(name, "apn_name"))
    {
      strncpy(param->apn_name, value, sizeof(param->apn_name));
    }
  else if (!strcmp(section, "LTE") && !strcmp(name, "rat"))
    {
      param->rat = selectable_paramparse(value, rat_param);
      if (param->rat < 0)
        {
          printf("Unknown RAT selection  %s:%s = %s\n",
                         section, name, value);

          /* return 0 means error */

          return 0;
        }
    }
  else if (!strcmp(section, "LTE") && !strcmp(name, "auth_type"))
    {
      param->auth_type = selectable_paramparse(value, authtype_param);
      if (param->auth_type < 0)
        {
          printf("Unknown Authentication type  %s:%s = %s\n",
                         section, name, value);

          /* return 0 means error */

          return 0;
        }
    }
  else if (!strcmp(section, "LTE") && !strcmp(name, "ip_type"))
    {
      param->ip_type = selectable_paramparse(value, iptype_param);
      if (param->ip_type < 0)
        {
          printf("Unknown IP type  %s:%s = %s\n",
                         section, name, value);

          /* return 0 means error */

          return 0;
        }
    }
  else if (!strcmp(section, "LTE") && !strcmp(name, "user_name"))
    {
      strncpy(param->user_name, value, sizeof(param->user_name));
    }
  else if (!strcmp(section, "LTE") && !strcmp(name, "passwd"))
    {
      strncpy(param->passwd, value, sizeof(param->passwd));
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "epname"))
    {
      strncpy(param->epname, value, sizeof(param->epname));
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "server_uri"))
    {
      strncpy(param->server_uri, value, sizeof(param->server_uri));
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "bootstrap"))
    {
      parsed_param = selectable_paramparse(value, truefalse_param);
      if (parsed_param < 0)
        {
          printf("Unknown bootstrap parameter %s:%s = %s\n",
                         section, name, value);

          /* return 0 means error */

          return 0;
        }

      param->bootstrap = parsed_param ? true : false;
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "security_mode"))
    {
      param->security_mode = selectable_paramparse(value, secmode_param);
      if (param->security_mode < 0)
        {
          printf("Unknown security_mode parameter %s:%s = %s\n",
                         section, name, value);

          /* return 0 means error */

          return 0;
        }
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "deviceid"))
    {
      strncpy(param->device_id, value, sizeof(param->device_id));
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "security_key"))
    {
      strncpy(param->security_key, value, sizeof(param->security_key));
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "lifetime"))
    {
      param->lifetime = atoi(value);
    }
  else if (!strcmp(section, "LwM2M") && !strcmp(name, "time_period"))
    {
      printf("Time period.\n");
      param->time_period = atoi(value);
    }

  /* return 1 means OK (no error) */

  return 1;
}

/****************************************************************************
 * Name: dump_params
 ****************************************************************************/

static void dump_params(struct app_parameter_s *param)
{
  printf("=== Setting parameters from INI file ===\n");
  printf("\n");
  printf("  apn_name = %s\n", param->apn_name);
  printf("  rat = %d\n", param->rat);
  printf("  auth_type = %d\n", param->auth_type);
  printf("  ip_type = %d\n", param->ip_type);
  printf("  user_name = %s\n", param->user_name);
  printf("  password = %s\n", param->passwd);
  printf("\n");
  printf("  server_uri = %s\n", param->server_uri);
  printf("  bootstrap = %s\n", param->bootstrap ? "TRUE" : "FALSE");
  printf("  security_mode = %d\n", param->security_mode);
  printf("  device_id = %s\n", param->device_id);
  printf("  security_key = %s\n\n", param->security_key);
  printf("  time_period = %d\n\n", param->time_period);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: createparam_from_ini
 ****************************************************************************/

struct app_parameter_s *createparam_from_ini(const char *filename)
{
  int ret;
  memset(&app_param, 0, sizeof(app_param));
  app_param.lifetime = 6000; /* As default */

  ret = ini_parse(filename, setup_parsedvalue, &app_param);
  if (ret != 0)
    {
      printf("ini_parse ret = %d\n", ret);
      if (ret < 0)
        {
          printf("Couldn't open : %s\n", filename);
        }

      return NULL;
    }

  /* Check parameters for discrepancies */

  if (app_param.security_mode == LWM2MSTUB_SECUREMODE_PSK &&
     (app_param.device_id[0] == '\0' || app_param.security_key[0] == '\0'))
    {
      printf("Enabled PSK security, but no device ID or security key\n");
      return NULL;
    }

  if (app_param.ip_type == LTE_IPTYPE_NON && app_param.rat != LTE_RAT_NBIOT)
    {
      printf("IP Type is NON-IP, but RAT is not NB-IOT\n");
      return NULL;
    }

  dump_params(&app_param);

  return &app_param;
}

/****************************************************************************
 * Name: get_appparam
 ****************************************************************************/

struct app_parameter_s *get_appparam(void)
{
  return &app_param;
}
