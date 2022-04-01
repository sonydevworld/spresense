/****************************************************************************
 * modules/lte_ext/lapiext/alt1250_ioctl_extend.c
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
#include <nuttx/net/usrsock.h>

#include "alt1250_dbg.h"
#include "alt1250_container.h"
#include "alt1250_socket.h"
#include "alt1250_usockevent.h"
#include "alt1250_postproc.h"
#include "alt1250_devif.h"
#include "alt1250_evt.h"
#include "alt1250_util.h"
#include "alt1250_ioctl_subhdlr.h"
#include "alt1250_usrsock_hdlr.h"

#include "alt1250_factory_reset.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TARGET_FWVERSION "RK_02_01_02_10_108_54"
#define IS_TARGET_FWVERSION(d) (!strncmp(((d)->fw_version), TARGET_FWVERSION, 21))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int send_freset_command(FAR struct alt1250_s *dev,
                               FAR struct alt_container_s *container,
                               int16_t usockid, FAR int32_t *usock_result,
                               int freset_cmdid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *freset_cmd_oargs[1];
static int freset_cmd_result;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int postproc_freset_command(FAR struct alt1250_s *dev,
                                   FAR struct alt_container_s *reply,
                                   FAR struct usock_s *usock,
                                   FAR int32_t *usock_result,
                                   FAR uint8_t *usock_xid,
                                   FAR struct usock_ackinfo_s *ackinfo,
                                   unsigned long arg)
{
  FAR void **resp = CONTAINER_RESPONSE(reply);
  int next_cmd = 0;
  int cmd_result = *((int *)resp[0]);

  if (cmd_result == ALT1250_FRESET_RESLT_ERROR)
    {
      *usock_result = ERROR;
      return REP_SEND_ACK;
    }
  else if (arg == ALT1250_FRESET_CMD_INQUIRE_PRE_RESET)
    {
      if (cmd_result == ALT1250_FRESET_PRE_RESET_NEEDED)
        {
          next_cmd = ALT1250_FRESET_CMD_PRE_RESET;
        }
      else
        {
          next_cmd = ALT1250_FRESET_CMD_FACTORY_RESET;
        }
    }
  else if (arg == ALT1250_FRESET_CMD_PRE_RESET)
    {
      next_cmd = ALT1250_FRESET_CMD_FACTORY_RESET;
    }
  else if (arg == ALT1250_FRESET_CMD_FACTORY_RESET)
    {
      *usock_result = OK;
      return REP_SEND_ACK;
    }

  return send_freset_command(dev, reply, USOCKET_USOCKID(usock),
                              usock_result, next_cmd);
}

static int send_freset_command(FAR struct alt1250_s *dev,
                               FAR struct alt_container_s *container,
                               int16_t usockid, FAR int32_t *usock_result,
                               int freset_cmdid)
{
  FAR void *inparam[2] = {0};

  inparam[0] = (FAR void *) freset_cmdid;
  inparam[1] = (FAR void *) dev->fw_version;

  freset_cmd_oargs[0] = &freset_cmd_result;

  set_container_ids(container, usockid, LTE_CMDID_FACTORY_RESET);
  set_container_argument(container, inparam, ARRAY_SZ(inparam));
  set_container_response(container, freset_cmd_oargs, ARRAY_SZ(freset_cmd_oargs));
  set_container_postproc(container, postproc_freset_command, freset_cmdid);

  return altdevice_send_command(dev->altfd, container, usock_result);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: usockreq_ioctl_extend
 ****************************************************************************/

int usockreq_ioctl_extend(FAR struct alt1250_s *dev,
                          FAR struct usrsock_request_buff_s *req,
                          FAR int32_t *usock_result,
                          FAR uint8_t *usock_xid,
                          FAR struct usock_ackinfo_s *ackinfo)
{
  FAR struct usrsock_request_ioctl_s *request = &req->request.ioctl_req;
  FAR struct lte_ioctl_data_s *ltecmd = &req->req_ioctl.ltecmd;
  FAR struct usock_s *usock = NULL;
  int ret = REP_SEND_ACK_WOFREE;
  FAR struct alt_container_s *container;
  int freset_cmd = 0;

  dbg_alt1250("%s start\n", __func__);

  *usock_result = OK;
  *usock_xid = request->head.xid;

  usock = usocket_search(dev, request->usockid);
  if (usock == NULL)
    {
      dbg_alt1250("Failed to get socket context: %u\n",
                     request->usockid);
      *usock_result = -EBADFD;
      return REP_SEND_ACK_WOFREE;
    }

  container = container_alloc();
  if (container == NULL)
    {
      dbg_alt1250("no container\n");
      return REP_NO_CONTAINER;
    }

  USOCKET_SET_REQUEST(usock, request->head.reqid, request->head.xid);

  /* Special operation for each commands */

  switch (ltecmd->cmdid)
    {
      case LTE_CMDID_FACTORY_RESET:
        {
          if (IS_TARGET_FWVERSION(dev))
            {
              freset_cmd = ALT1250_FRESET_CMD_INQUIRE_PRE_RESET;
            }
          else
            {
              freset_cmd = ALT1250_FRESET_CMD_FACTORY_RESET;
            }

          ret = send_freset_command(dev, container,
                                    USOCKET_USOCKID(usock), 
                                    usock_result, freset_cmd);

          if (IS_NEED_CONTAINER_FREE(ret))
            {
              container_free(container);
            }
        }
        break;

      default:
        {
          *usock_result = -EINVAL;
          container_free(container);
          return REP_SEND_ACK_WOFREE;
        }
        break;
    }

  return ret;
}

