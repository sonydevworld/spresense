/****************************************************************************
 * examples/lte_lwm2mstub/lte_lwm2mstub_main.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#include <lte/lte_api.h>
#include <lte/lte_lwm2m.h>

#include "app_parameter.h"
#include "app_lte_util.h"
#include "app_lwm2m_util.h"
#include "app_message.h"
#include "app_eventtimer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OBJID_DATACONTAINER  (19)
#define APP_INIFILE "/mnt/spif/lwm2m.ini"
#define LWM2M_TMP_BUFF_LEN   (16)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char tmp_buff[LWM2M_TMP_BUFF_LEN];
static uint16_t enableobjs[6] =
{
  1, 2, 3, 4, 5, OBJID_DATACONTAINER
};

static struct lwm2mstub_resource_s datacontainer_res[] =
{
  {
    .res_id = 0,
    .operation = LWM2MSTUB_RESOP_RW,
    .inst_type = LWM2MSTUB_RESINST_SINGLE,
    .data_type = LWM2MSTUB_RESDATA_OPAQUE,
    .handl = LWM2MSTUB_RESOURCE_HANDLENOCARE
  },
};

#define DATACONTAINER_OBJRESNUM \
                (sizeof(datacontainer_res) / sizeof(datacontainer_res[0]))

static volatile int periodic_value = 1234;
static int dummy_value_01 = 0x1122aabb;
static int dummy_value_02 = 0xdeadbeef;

static bool is_observing = false;
static char ov_token[LWM2MSTUB_MAX_TOKEN_SIZE];

static volatile bool is_task_running = false;
static bool is_registerd_once = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* LwM2M callbacks */

/****************************************************************************
 * Name: write_cb
 ****************************************************************************/

static void write_cb(int seq_no, int srv_id,
              struct lwm2mstub_instance_s *inst, char *value, int len)
{
  struct app_message_s msg;

  printf("[CB] Write /%d/%d/%d request size=%d value >>%s<<\n",
          inst->object_id, inst->object_inst, inst->res_id, len, value);

  if (inst->object_id == OBJID_DATACONTAINER && inst->res_id == 0)
    {
      if (!strcmp(value, "01"))
        {
          msg.msgid = MESSAGE_ID_LWM2M_ACTION;
          msg.arg.code = 1;
          send_message(MESSAGE_QUEUE_NAME, &msg);
        }
      else if (!strcmp(value, "02"))
        {
          msg.msgid = MESSAGE_ID_LWM2M_ACTION;
          msg.arg.code = 2;
          send_message(MESSAGE_QUEUE_NAME, &msg);
        }

      lte_m2m_writeresponse(seq_no, inst, LWM2MSTUB_RESP_CHANGED);
    }
  else
    {
      lte_m2m_writeresponse(seq_no, inst, LWM2MSTUB_RESP_NOTALLOW);
    }
}

/****************************************************************************
 * Name: read_cb
 ****************************************************************************/

static void read_cb(int seq_no, int srv_id,
              struct lwm2mstub_instance_s *inst)
{
  int len;

  printf("[CB] Read /%d/%d/%d request\n",
          inst->object_id, inst->object_inst, inst->res_id);

  if (inst->object_id == OBJID_DATACONTAINER && inst->res_id == 0)
    {
      len = snprintf(tmp_buff, LWM2M_TMP_BUFF_LEN, "%08x", periodic_value);
      lte_m2m_readresponse(seq_no, inst,
                           LWM2MSTUB_RESP_CONTENT, tmp_buff, len);
    }
  else
    {
      lte_m2m_readresponse(seq_no, inst,
                           LWM2MSTUB_RESP_NOTALLOW, tmp_buff, 0);
    }
}

/****************************************************************************
 * Name: exec_cb
 ****************************************************************************/

static void exec_cb(int seq_no, int srv_id,
              struct lwm2mstub_instance_s *inst)
{
  printf("[CB] Exec /%d/%d/%d request\n",
          inst->object_id, inst->object_inst, inst->res_id);

  lte_m2m_executeresp(seq_no, inst, LWM2MSTUB_RESP_NOTACCEPT);
}

/****************************************************************************
 * Name: ovstart_cb
 ****************************************************************************/

static void ovstart_cb(int seq_no, int srv_id,
              struct lwm2mstub_instance_s *inst, char *token,
              struct lwm2mstub_ovcondition_s *cond)
{
  struct app_message_s msg;

  printf("[CB] Observe Start /%d/%d/%d request: token=%s\n",
          inst->object_id, inst->object_inst, inst->res_id, token);

  if (inst->object_id == OBJID_DATACONTAINER)
    {
      msg.msgid = MESSAGE_ID_LWM2M_OVSTART;
      memcpy(msg.arg.token, token, LWM2MSTUB_MAX_TOKEN_SIZE);
      send_message(MESSAGE_QUEUE_NAME, &msg);
      lte_m2m_observeresp(seq_no, LWM2MSTUB_RESP_CONTENT);
    }
  else
    {
      lte_m2m_observeresp(seq_no, LWM2MSTUB_RESP_UNAUTH);
    }
}

/****************************************************************************
 * Name: ovstop_cb
 ****************************************************************************/

static void ovstop_cb(int seq_no, int srv_id,
              struct lwm2mstub_instance_s *inst, char *token)
{
  struct app_message_s msg;

  printf("[CB] Observe Stop /%d/%d/%d request: token=%s\n",
          inst->object_id, inst->object_inst, inst->res_id, token);

  if (inst->object_id == OBJID_DATACONTAINER)
    {
      msg.msgid = MESSAGE_ID_LWM2M_OVSTOP;
      send_message(MESSAGE_QUEUE_NAME, &msg);
    }

  lte_m2m_observeresp(seq_no, LWM2MSTUB_RESP_CONTENT);
}

/****************************************************************************
 * Name: serverop_cb
 ****************************************************************************/

static void serverop_cb(int event, int srvid, struct lwm2mstub_instance_s *inst)
{
  struct app_message_s msg;

  printf("Operation : %d  ", event);
  dump_opevent(event);
  printf("TargetObj: %d/%d/%d\n",
          inst->object_id, inst->object_inst, inst->res_id);

  if (event == LWM2MSTUB_OP_REGSUCCESS)
    {
      is_registerd_once = true;
    }

  if (is_registerd_once && event == LWM2MSTUB_OP_OFFLINE)
    {
      msg.msgid = MESSAGE_ID_LWM2M_OPERATE;
      msg.arg.code = event;
      send_message(MESSAGE_QUEUE_NAME, &msg);
    }
}

/****************************************************************************
 * Name: fwupdate_cb
 ****************************************************************************/

static void fwupstate_cb(int event)
{
  printf("FW Update state : %d  ", event);
  dump_fwevent(event);
}

/****************************************************************************
 * Name: is_the_object_same
 ****************************************************************************/

static bool is_the_object_same(void)
{
  struct lwm2mstub_resource_s res;

  if (lte_getm2m_objresourceinfo(OBJID_DATACONTAINER, 1, &res) >= 0)
    {
      if (res.res_id == datacontainer_res[0].res_id &&
          res.operation == datacontainer_res[0].operation &&
          res.inst_type == datacontainer_res[0].inst_type &&
          res.data_type == datacontainer_res[0].data_type )
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: object_datacontainer_setup
 ****************************************************************************/

static void object_datacontainer_setup(void)
{
  int resnum;

  resnum = lte_getm2m_objresourcenum(OBJID_DATACONTAINER);
  if (resnum != DATACONTAINER_OBJRESNUM || !is_the_object_same())
    {
      printf("Setup object %d\n", OBJID_DATACONTAINER);
      lte_setm2m_objectdefinition(OBJID_DATACONTAINER,
                                  DATACONTAINER_OBJRESNUM, datacontainer_res);
      usleep(100 * 1000); /* Wait for update stable */
    }

  dump_objinfo(OBJID_DATACONTAINER);
}

/****************************************************************************
 * Name: lwm2m_setup
 ****************************************************************************/

static int lwm2m_setup(struct app_parameter_s *param)
{
  int ret;
  struct lwm2mstub_serverinfo_s info;

  /* Setup server information */

  info.device_id[0] = '\0';
  info.security_key[0] = '\0';

  strncpy(info.server_uri, param->server_uri, sizeof(info.server_uri));
  strncpy(info.device_id, param->device_id, sizeof(info.device_id));
  strncpy(info.security_key, param->security_key,
                             sizeof(info.security_key));

  info.bootstrap = param->bootstrap;
  info.security_mode = param->security_mode;
  info.nonip = (param->ip_type == LTE_IPTYPE_NON) ? true : false;
  info.lifetime = param->lifetime;

  ret = lte_setm2m_serverinfo(&info, 0);
  if (ret < 0)
    {
      printf("lte_setm2m_serverinfo() failed:%d\n", ret);
      return ERROR;
    }

  usleep(100 * 1000); /* Wait for update stable */

  /* Setup Binary App Data Container object (ID:19) */

  object_datacontainer_setup();

  /* Setup endpoint name */

  lte_setm2m_endpointname(param->epname);
  usleep(100 * 1000); /* Wait for update stable */

  /* Enable objects */

  lte_enablem2m_objects(enableobjs,
                        sizeof(enableobjs) / sizeof(enableobjs[0]));
  usleep(100 * 1000); /* Wait for update stable */

  /* Apply setting */

  lte_apply_m2msetting();
  usleep(500 * 1000); /* Wait for modem is ready */

  return OK;
}

/****************************************************************************
 * Name: connect_to_lwm2mserver
 ****************************************************************************/

static int connect_to_lwm2mserver(struct app_parameter_s *param)
{
  int ret;

  lte_set_report_m2mwrite(write_cb);
  lte_set_report_m2mread(read_cb);
  lte_set_report_m2mexec(exec_cb);
  lte_set_report_m2movstart(ovstart_cb);
  lte_set_report_m2movstop(ovstop_cb);
  lte_set_report_m2moperation(serverop_cb);
  lte_set_report_m2mfwupdate(fwupstate_cb);

  ret = lte_m2m_connection(LWM2MSTUB_CONNECT_REGISTER);
  if (ret != OK)
    {
      printf("lwm2m register failed:%d\n", ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: notify_value
 ****************************************************************************/

static void notify_value(int value)
{
  int len;
  struct lwm2mstub_instance_s inst;

  inst.object_id = OBJID_DATACONTAINER;
  inst.object_inst = 0;
  inst.res_id = 0;
  inst.res_inst = -1;

  len = snprintf(tmp_buff, LWM2M_TMP_BUFF_LEN, "%08x", value);
  printf("Update value as : token: %s, /%d/0/0 %s\n",
                                      ov_token, OBJID_DATACONTAINER, tmp_buff);
  printf("observe update : %d\n",
                   lte_m2m_observeupdate(ov_token, &inst, tmp_buff, len));
}

/****************************************************************************
 * Name: handle_message
 ****************************************************************************/

static bool handle_message(struct app_message_s *msg)
{
  bool ret = true;
  int *tgt;

  switch (msg->msgid)
    {
      case MESSAGE_ID_LWM2M_VALUE:
        periodic_value = msg->arg.code;
        printf("periodic_value = %d\n", periodic_value);
        if (is_observing)
          {
            notify_value(periodic_value);
          }

        break;

      case MESSAGE_ID_LWM2M_ACTION:
        tgt = NULL;
        switch (msg->arg.code)
          {
            case 1:
              tgt = &dummy_value_01;
              break;
            case 2:
              tgt = &dummy_value_02;
              break;
          }

        if (is_observing && tgt)
          {
            notify_value(*tgt);
          }

        break;

      case MESSAGE_ID_LWM2M_OPERATE:
        printf("Client is OFF so update register now.\n");
        lte_m2m_connection(LWM2MSTUB_CONNECT_REREGISTER);
        break;

      case MESSAGE_ID_LWM2M_OVSTART:
        is_observing = true;
        memcpy(ov_token, msg->arg.token, sizeof(ov_token));
        printf("Observe Started. token:%s\n", ov_token);
        break;

      case MESSAGE_ID_LWM2M_OVSTOP:
        is_observing = false;
        break;

      case MESSAGE_ID_ENDAPP:
        ret = false;
        break;

      case MESSAGE_ID_LTE_RESTARTED:
        is_observing = false;
        printf("LTE modem is resetted. This app will be terminated."
               " Please execute this app again.\n");
        ret = false;
        break;

      case MESSAGE_ID_VERSION:
          {
            lte_version_t version;
            lte_get_version_sync(&version);
            printf("Modem IC Type : %s\n", version.bb_product);
            printf("      FW Ver. : %s\n", version.np_package);
          }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: lwm2m_task
 ****************************************************************************/

static int lwm2m_task(int argc, char *argv[])
{
  int ret;
  bool is_running = true;
  struct app_parameter_s *param;
  struct app_message_s msg;

  /* Initializing */

  param = createparam_from_ini(APP_INIFILE);
  if (!param)
    {
      goto error_label_initial;
    }

  if (create_msgqueue(MESSAGE_QUEUE_NAME) != OK)
    {
      printf("Message creation error\n");
      goto error_label_initial;
    }

  if (power_on_ltemodem(param) != OK)
    {
      printf("LTE Poower On error\n");
      goto error_label_queue_unlink;
    }

  printf("Setup LTE Server\n");
  if (lwm2m_setup(param) != OK)
    {
      printf("LwM2M setup error\n");
      goto error_label_lte_finalize;
    }

  printf("Connection LTE network\n");
  if (connect_to_ltenetwork(param) != OK)
    {
      printf("LTE network connection failed\n");
      goto error_label_lte_finalize;
    }

  printf("Connect to LwM2M Server\n");
  if (connect_to_lwm2mserver(param) != OK)
    {
      printf("Connection to LwM2M server failed\n");
      goto error_label_lte_finalize;
    }

  if (timer_setup(param) != OK)
    {
      printf("Timer setup failed.\n");
      goto error_label_lwm2m_disconnect;
    }

  lte_acquire_wakelock();

  /* Main loop for waiting message and handle it */

  while (is_running)
    {
      printf("Wait for next event\n");
      ret = receive_message(MESSAGE_QUEUE_NAME, &msg);
      if (ret == OK)
        {
          is_running = handle_message(&msg);
        }
    }

  lte_release_wakelock();

  timer_unset();

error_label_lwm2m_disconnect:
  lte_m2m_connection(LWM2MSTUB_CONNECT_DEREGISTER);

error_label_lte_finalize:
  printf("LTE Finalize\n");
  lte_finalize();

error_label_queue_unlink:
  printf("Unlink mqueue\n");
  mq_unlink(MESSAGE_QUEUE_NAME);

error_label_initial:
  printf("End this app.\n");
  is_task_running = false;

  return 0;
}

/****************************************************************************
 * Name: m2mapp_update_value
 ****************************************************************************/

static void m2mapp_update_value(int v)
{
  struct app_message_s msg;

  if (v >= 0)
    {
      msg.msgid = MESSAGE_ID_LWM2M_VALUE;
      msg.arg.code = v;
      send_message(MESSAGE_QUEUE_NAME, &msg);
    }
  else
    {
      msg.msgid = MESSAGE_ID_ENDAPP;
      send_message(MESSAGE_QUEUE_NAME, &msg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, char **argv)
{
  int ret;
  struct app_message_s msg;

  if (!is_task_running)
    {
      printf("Launching lwm2m_task...\n");
      ret = task_create("lwm2m_task", 100, 2048, lwm2m_task, NULL);
      if (ret < 0)
        {
          printf("Could not create lwm2m task : [%d]\n", errno);
          return -1;
        }

      printf("Done.\n");
      is_task_running = true;
    }
  else if (argc == 2)
    {
      printf("Updating value : %s\n", argv[1]);
      if (argv[1][0] == 'v')
        {
          msg.msgid = MESSAGE_ID_VERSION;
          msg.arg.code = argv[1][0];
          send_message(MESSAGE_QUEUE_NAME, &msg);
        }
      else if (argv[1][0] == 'r')
        {
          msg.msgid = MESSAGE_ID_RECONNECT;
          msg.arg.code = argv[1][0];
          send_message(MESSAGE_QUEUE_NAME, &msg);
        }
      else
        {
          m2mapp_update_value(atoi(argv[1]));
        }
    }

  return 0;
}

/****************************************************************************
 * Name: increment_value
 ****************************************************************************/

void increment_value(void)
{
  m2mapp_update_value(periodic_value + 1);
}
