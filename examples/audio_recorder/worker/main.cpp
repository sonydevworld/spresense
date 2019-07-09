/****************************************************************************
 * audio_recorder/worker/userproc/main.cpp
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
#include <errno.h>

#include <stdlib.h>
#include <string.h>

extern "C"
{
#include <asmp/types.h>
#include <asmp/mpshm.h>
#include <asmp/mpmutex.h>
#include <asmp/mpmq.h>
}

#include "userproc.h"
#include <audio/dsp_framework/customproc_dsp_ctrl.h>
#include <audio/dsp_framework/customproc_command_base.h>

#define KEY_MQ 2
#define KEY_SHM   1
#define COMMAND_DATATYPE_ADDRESS 0
#define COMMAND_DATATYPE_VALUE 1

#define MSGID_PROCMODE_SHIFT 4
#define MSGID_DATATYPE_MASK 0x01

#define CRE_MSGID(mode, type) ((mode << MSGID_PROCMODE_SHIFT) | (type & MSGID_DATATYPE_MASK))

static mpmq_t s_mq;

extern "C" {

/*--------------------------------------------------------------------*/
static void reply_to_spu(void *addr)
{
  uint8_t msg_id = 0;
  uint32_t msg_data = 0;

  /* Create message ID */

  msg_id = CRE_MSGID(CustomprocCommand::FilterMode, COMMAND_DATATYPE_ADDRESS);

  /* Message data is address of APU command */

  msg_data = reinterpret_cast<uint32_t>(addr);

  /* Send */

  int ret = mpmq_send(&s_mq, msg_id, msg_data);
  if (ret != 0)
    {
      /* error */
    }
}

};

/*--------------------------------------------------------------------*/
int main()
{
  UserProc userproc_ins;
  CustomprocDspCtrl ctrl_ins(&userproc_ins);

  int ret = 0;

  /* Initialize MP message queue,
   * On the worker side, 3rd argument is ignored.
   */

  ret = mpmq_init(&s_mq, KEY_MQ, 0);
  if (ret != 0)
    {
      /* error */
    }

  /* Reply "boot complete"
   * MsgID is taken as part of message parameters.
   */

  uint8_t msg_id = 0;

  msg_id = CRE_MSGID(CustomprocCommand::CommonMode, COMMAND_DATATYPE_VALUE);

  ret = mpmq_send(&s_mq, msg_id, ret);
  if (ret != 0)
    {
      /* error */
    }

  /* Excution loop */

  while (true)
    {
      /* Receive message from SPU
       * Process is blocked until receive a message
       * (mpmq_receive() is polling message internally)
       */

      int command = 0;
      uint32_t msgdata = 0;

      command = mpmq_receive(&s_mq, &msgdata);

      uint8_t type = command & MSGID_DATATYPE_MASK;

      /* Parse and execute message */

      if (type == COMMAND_DATATYPE_ADDRESS)
        {
          ctrl_ins.parse(reinterpret_cast<CustomprocCommand::CmdBase *>(msgdata));
        }
      else
        {
          /* write error process here. */
        }

      /* Send Reply */

      reply_to_spu(reinterpret_cast<void *>(msgdata));
    }
}

