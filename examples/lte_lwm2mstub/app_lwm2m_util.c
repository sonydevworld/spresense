/****************************************************************************
 * examples/lte_lwm2mstub/app_lwm2m_util.c
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

#include <stdio.h>
#include <stdlib.h>
#include <lte/lte_lwm2m.h>

#include "app_lwm2m_util.h"

/****************************************************************************
 * Private data
 ****************************************************************************/

static char tmp_buff[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: state_string
 ****************************************************************************/

static const char *state_string(int state)
{
  switch (state)
    {
      case LWM2MSTUB_STATE_NOTREGISTERD :
        return "Not registerd";
      case LWM2MSTUB_STATE_REGISTPENDING :
        return "Pending registration";
      case LWM2MSTUB_STATE_REGISTERD :
        return "Registerd";
      case LWM2MSTUB_STATE_REGISTERFAILED :
        return "Register failed";
      case LWM2MSTUB_STATE_UPDATEPENDING :
        return "Re-Registration pending";
      case LWM2MSTUB_STATE_DEREGISTPENDING :
        return "De-register pending";
      case LWM2MSTUB_STATE_BSHOLDOFF :
        return " Bootstrap hold time out";
      case LWM2MSTUB_STATE_BSREQUESTED :
        return "Bootstrap requested";
      case LWM2MSTUB_STATE_BSONGOING :
        return "Bootstrap on going";
      case LWM2MSTUB_STATE_BSDONE :
        return "Bootstrap is done";
      case LWM2MSTUB_STATE_BSFAILED :
        return "Bootstrap failed";
      default :
        break;
    }

  return "Unknown";
}

/****************************************************************************
 * Name: secmode_string
 ****************************************************************************/

static const char *secmode_string(int mode)
{
  switch (mode)
    {
      case LWM2MSTUB_SECUREMODE_PSK :
        return "PSK";
      case LWM2MSTUB_SECUREMODE_RPK :
        return "RPK";
      case LWM2MSTUB_SECUREMODE_CERT :
        return "Certificate";
      case LWM2MSTUB_SECUREMODE_NOSEC :
        return "Non Secure";
      case LWM2MSTUB_SECUREMODE_CERTEST :
        return "Certificate with EST";
      default:
        break;
    }

  return "Unknown";
}

/****************************************************************************
 * Name: objop_string
 ****************************************************************************/

static const char *objop_string(int op)
{
  switch (op)
    {
      case LWM2MSTUB_RESOP_READ :
        return "READ ";
      case LWM2MSTUB_RESOP_WRITE :
        return "WRITE";
      case LWM2MSTUB_RESOP_RW    :
        return "RDWT ";
      case LWM2MSTUB_RESOP_EXEC  :
        return "EXEC ";
      default:
        break;
    }

  return "Unknown";
}

/****************************************************************************
 * Name: objdata_string
 ****************************************************************************/

static const char *objdata_string(int type)
{
  switch (type)
    {
      case LWM2MSTUB_RESDATA_NONE:
        return  "None";
      case LWM2MSTUB_RESDATA_STRING:
        return  "String";
      case LWM2MSTUB_RESDATA_INT:
        return  "Integer";
      case LWM2MSTUB_RESDATA_UNSIGNED:
        return  "U-Integer";
      case LWM2MSTUB_RESDATA_FLOAT:
        return  "Float";
      case LWM2MSTUB_RESDATA_BOOL:
        return  "Boolean";
      case LWM2MSTUB_RESDATA_OPAQUE:
        return  "Opaque";
      case LWM2MSTUB_RESDATA_TIME:
        return  "Time";
      case LWM2MSTUB_RESDATA_OBJLINK:
        return  "Object-Link";
      default:
        break;
    }

  return "TypeError";
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dump_opevent
 ****************************************************************************/

void dump_opevent(int event)
{
  switch (event)
    {
      case LWM2MSTUB_OP_WRITE:
        printf("Write operation received.\n");
        break;
      case LWM2MSTUB_OP_EXEC:
        printf("Execution operation received.\n");
        break;
      case LWM2MSTUB_OP_WATTR:
        printf("Write attribute operation received.\n");
        break;
      case LWM2MSTUB_OP_DISCOVER:
        printf("Discover operation received.\n");
        break;
      case LWM2MSTUB_OP_READ:
        printf("Read operation received.\n");
        break;
      case LWM2MSTUB_OP_OBSERVE:
        printf("Observe START operation received.\n");
        break;
      case LWM2MSTUB_OP_CANCELOBS:
        printf("Observe STOP operation received.\n");
        break;
      case LWM2MSTUB_OP_OFFLINE:
        printf("Client is Off-line.\n");
        break;
      case LWM2MSTUB_OP_ONLINE:
        printf("Client is On-line.\n");
        break;
      case LWM2MSTUB_OP_SENDNOTICE:
        printf("Notification is sent to server.\n");
        break;
      case LWM2MSTUB_OP_RCVWUP:
        printf("SMS wake up received.\n");
        break;
      case LWM2MSTUB_OP_RCVOBSACK:
        printf("Notification ack is received.\n");
        break;
      case LWM2MSTUB_OP_CLIENTON:
        printf("Client is on.\n");
        break;
      case LWM2MSTUB_OP_CLIENTOFF:
        printf("Client is off.\n");
        break;
      case LWM2MSTUB_OP_FAILNOTIFY:
        printf("Notification confirmation failed.\n");
        break;
      case LWM2MSTUB_OP_BSFINISH:
        printf("Bootstrap is done.\n");
        break;
      case LWM2MSTUB_OP_REGSUCCESS:
        printf("Register is successed.\n");
        break;
      case LWM2MSTUB_OP_REGUPDATED:
        printf("Re-register is successed.\n");
        break;
      case LWM2MSTUB_OP_DEREGSUCCESS:
        printf("De-Register is successed.\n");
        break;
      case LWM2MSTUB_OP_NOSENDNOTICE:
        printf("Notification is not sent to server.\n");
        break;
      default:
        printf("Unknown...\n");
        break;
    }
}

/****************************************************************************
 * Name: dump_fwevent
 ****************************************************************************/

void dump_fwevent(int event)
{
  switch (event)
    {
      case LWM2MSTUB_FWUP_PEND_DL:
        printf("FW update image donwload pending..\n");
        break;
      case LWM2MSTUB_FWUP_PEND_UPD:
        printf("FW update is pending..\n");
        break;
      case LWM2MSTUB_FWUP_COMP_DL:
        printf("FW update image download is done.\n");
        break;
      case LWM2MSTUB_FWUP_FAIL_DL:
        printf("FW update image donwload is failed.\n");
        break;
      case LWM2MSTUB_FWUP_CANCELED:
        printf("FW update is canceld by a server.\n");
        break;
      default:
        printf("Unknown...\n");
        break;
    }
}

/****************************************************************************
 * Name: dump_servinfo
 ****************************************************************************/

void dump_servinfo(void)
{
  struct lwm2mstub_serverinfo_s info;
  char *cc;

  printf("Getting server information\n");
  if (lte_getm2m_serverinfo(&info, 0) >= 0)
    {
      printf("=== Server information ===\n");

      lte_getm2m_endpointname(tmp_buff, sizeof(tmp_buff));
      printf("Endpoint name : %s\n", tmp_buff);

      printf("     Instno : %d\n", info.object_inst);
      printf("     Status : %s\n", state_string(info.state));
      printf("  BootStrap : %s\n", info.bootstrap ? "Yes" : "No");
      printf("     Non-IP : %s\n", info.nonip ? "Yes" : "No");
      printf(" Secur Mode : %s\n", secmode_string(info.security_mode));
      printf(" Server URI : %s\n", info.server_uri);
      printf("  Device ID : ");

      for (cc = info.device_id; *cc; cc++)
        {
          /* lte_getm2m_endpointname() can not get device_id.
           * So this should be empty.
           */

          printf("%02x", *cc);
        }

      printf("\n");

      printf("SecurityKey : ");
      for (cc = info.security_key; *cc; cc++)
        {
          /* lte_getm2m_endpointname() can not get security_key.
           * So this should be empty.
           */

          printf("%02x", *cc);
        }

      printf("\n\n");
    }
  else
    {
      printf("Error on lte_getm2m_serverinfo()\n");
    }
}

/****************************************************************************
 * Name: dump_objinfo
 ****************************************************************************/

void dump_objinfo(uint16_t objid)
{
  int i;
  int resnum;
  struct lwm2mstub_resource_s *reses;

  resnum = lte_getm2m_objresourcenum(objid);
  printf("Object %d has %d resources.\n", objid, resnum);

  if (resnum > 0)
    {
      reses = (struct lwm2mstub_resource_s *)
              malloc(sizeof(struct lwm2mstub_resource_s) * resnum);
      if (reses)
        {
          if (lte_getm2m_objresourceinfo(objid, resnum, reses) >= 0)
            {
              for (i = 0; i < resnum; i++)
                {
                  printf("  ID %d : %s, %s, %s\n", reses[i].res_id,
                          reses[i].inst_type == LWM2MSTUB_RESINST_SINGLE
                            ? "Single" : "Multi ",
                          objop_string(reses[i].operation),
                          objdata_string(reses[i].data_type));
                }

              printf("\n");
            }

          free(reses);
        }
    }
}
