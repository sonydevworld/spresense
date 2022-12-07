/****************************************************************************
 * examples/lte_lwm2mstub/app_parameter.h
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

#ifndef __EXAMPLES_LTE_LWM2MSTUB_APP_PARAMETER_H
#define __EXAMPLES_LTE_LWM2MSTUB_APP_PARAMETER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <lte/lte_api.h>
#include <lte/lte_lwm2m.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_MAXPARAMLEN (54)

#define MESSAGE_QUEUE_NAME  "m2mstub_mq"
#define MESSAGE_QUEUE_MAX   (1)

#define MESSAGE_ID_LTE_RESTARTED  (1)
#define MESSAGE_ID_LWM2M_OPERATE  (2)
#define MESSAGE_ID_LWM2M_FWUPDATE (3)
#define MESSAGE_ID_LWM2M_OVSTART  (4)
#define MESSAGE_ID_LWM2M_OVSTOP   (5)
#define MESSAGE_ID_LWM2M_VALUE    (6)
#define MESSAGE_ID_ENDAPP         (7)
#define MESSAGE_ID_LWM2M_ACTION   (8)
#define MESSAGE_ID_VERSION        (9)
#define MESSAGE_ID_RECONNECT      (10)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct app_parameter_s
{
  /* LTE setting */

  char apn_name[APP_MAXPARAMLEN];
  int auth_type;
  int ip_type;
  char user_name[APP_MAXPARAMLEN];
  char passwd[APP_MAXPARAMLEN];
  int rat;

  /* LeM2M setting */

  char epname[128];
  char server_uri[LWM2MSTUB_MAX_SERVER_NAME];
  char device_id[LWM2MSTUB_MAX_DEVID];
  char security_key[LWM2MSTUB_MAX_SEQKEY];
  bool bootstrap;
  int security_mode;
  uint32_t lifetime;

  int time_period;
};

struct app_message_s
{
  int msgid;
  union
  {
    int code;
    char token[LWM2MSTUB_MAX_TOKEN_SIZE];
  } arg;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct app_parameter_s *createparam_from_ini(const char *filename);
struct app_parameter_s *get_appparam(void);

#endif  /* __EXAMPLES_LTE_LWM2MSTUB_APP_PARAMETER_H */
