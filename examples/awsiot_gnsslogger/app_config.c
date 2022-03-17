/****************************************************************************
 * examples/awsiot_gnsslogger/app_config.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"

#include "fsutils/ini.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_MQTT_CMDTIMEOUT 20000
#define DEFAULT_MQTT_SSLTIMEOUT 10000
#define DEFAULT_MQTT_KEEPALIVE  600

#define MATCH(s, n) ((strncmp(section, s, strlen(s))==0) && (strncmp(name, n, strlen(n))==0))

#define IS_CONFIG_NOTGOOD(cnf) ( ((cnf)->host_url==NULL) || \
                            ((cnf)->host_port==0) || \
                            ((cnf)->root_ca_path==NULL) || \
                            ((cnf)->crt_path==NULL) || \
                            ((cnf)->privkey_path==NULL) || \
                            ((cnf)->client_id==NULL) )

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: default_app_config()
 *
 * Description:
 *   Set default value on awsiot_app_config.
 ****************************************************************************/

static void default_app_config( awsiot_app_config *confg )
{
  confg->host_url = confg->root_ca_path
    = confg->crt_path = confg->privkey_path = confg->client_id = NULL;
  confg->host_port = 0;
  confg->cmd_timeout = DEFAULT_MQTT_CMDTIMEOUT;
  confg->ssl_timeout = DEFAULT_MQTT_SSLTIMEOUT;
  confg->keepalive_ivtime = DEFAULT_MQTT_KEEPALIVE;
}

/****************************************************************************
 * Name: inihandler()
 *
 * Description:
 *   Called this function back from inih libraries on persing ini file.
 ****************************************************************************/

static int inihandler(  void *priv,
                        const char *section,
                        const char *name,
                        const char *value)
{
  awsiot_app_config *conf = (awsiot_app_config *)priv;

  if (MATCH("host", "url"))
    {
      conf->host_url = strdup(value);
    }
  else if (MATCH("host", "port"))
    {
      conf->host_port = atoi(value);
    }
  else if (MATCH("certification", "rootCA_path"))
    {
      conf->root_ca_path = strdup(value);
    }
  else if (MATCH("certification", "cert_path"))
    {
      conf->crt_path = strdup(value);
    }
  else if (MATCH("certification", "privKEY_path"))
    {
      conf->privkey_path = strdup(value);
    }
  else if (MATCH("mqtt", "client_id"))
    {
      conf->client_id = strdup(value);
    }
  else if (MATCH("mqtt", "cmd_timeout_ms"))
    {
      conf->cmd_timeout = atoi(value);
    }
  else if (MATCH("mqtt", "ssl_timeout_ms"))
    {
      conf->ssl_timeout = atoi(value);
    }
  else if (MATCH("mqtt", "keepalive_interval_sec"))
    {
      conf->keepalive_ivtime = atoi(value);
    }
  else
    {
      return 0; /* unknown section/name, error */
    }

  return 1;
}

/****************************************************************************
 * Name: dump_config()
 *
 * Description:
 *   Dump configuration parameters on stdout.
 ****************************************************************************/

static void dump_config(awsiot_app_config *config)
{
  printf("== Configuration ==\n");
  printf("    Host URL        = %s\n", config->host_url ? config->host_url : "NULL");
  printf("    Host PORT       = %d\n", config->host_port);
  printf("    Root CA Path    = %s\n", config->root_ca_path ? config->root_ca_path : "NULL");
  printf("    Cert Path       = %s\n", config->crt_path ? config->crt_path : "NULL");
  printf("    PrivKEY Path    = %s\n", config->privkey_path ? config->privkey_path : "NULL");
  printf("    MQTT Client ID  = %s\n", config->client_id ? config->client_id : "NULL");
  printf("    MQTT CMD TO     = %d (ms)\n", config->cmd_timeout);
  printf("    MQTT SSL TO     = %d (ms)\n", config->ssl_timeout);
  printf("    MQTT KEEP ALIVE = %d (sec)\n", config->keepalive_ivtime);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_inifile()
 *
 * Description:
 *   Load and parse ini file.
 ****************************************************************************/

int parse_inifile(const char *ini, awsiot_app_config *config)
{
  int ret = 0;
  default_app_config(config);

  if (ini_parse(ini, inihandler, config)==0)
    {
      if (IS_CONFIG_NOTGOOD(config))
        {
          printf("INI file contents is not enough..\n");
          dump_config(config);
          ret = -2;
        }
    }
  else
    {
      printf("Can't load '%s'\n", ini);
      ret = -1;
    }

  dump_config(config);

  return ret;
}

/****************************************************************************
 * Name: delete_config()
 *
 * Description:
 *   Delete some memory which are allocated on parse_inifile()
 ****************************************************************************/

void delete_config(awsiot_app_config *config)
{
  if (config->host_url)
    {
      free(config->host_url);
    }

  if (config->root_ca_path)
    {
      free(config->root_ca_path);
    }

  if (config->crt_path)
    {
      free(config->crt_path);
    }

  if (config->privkey_path)
    {
      free(config->privkey_path);
    }

  if (config->client_id)
    {
      free(config->client_id);
    }

  default_app_config(config);
}
