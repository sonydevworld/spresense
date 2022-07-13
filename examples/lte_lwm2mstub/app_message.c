/****************************************************************************
 * examples/lte_lwm2mstub/app_message.c
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
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>

#include "app_parameter.h"
#include "app_message.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: create_msgqueue
 ****************************************************************************/

int create_msgqueue(const char *qname)
{
  mqd_t mq;
  struct mq_attr attr;

  attr.mq_maxmsg = MESSAGE_QUEUE_MAX;
  attr.mq_msgsize = sizeof(struct app_message_s);
  attr.mq_flags = 0;

  mq = mq_open(qname, O_CREAT, 0666,  &attr);
  if (mq == (mqd_t)-1)
    {
      printf("Message queue open error:%d\n", errno);
      return ERROR;
    }

  mq_close(mq);
  return OK;
}

/****************************************************************************
 * Name: send_message
 ****************************************************************************/

int send_message(const char *qname, struct app_message_s *msg)
{
  int ret = ERROR;
  mqd_t mq;

  mq = mq_open(qname, O_WRONLY);
  if (mq != (mqd_t)-1)
    {
      mq_send(mq, (const char *)msg, sizeof(struct app_message_s), 0);
      mq_close(mq);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: receive_message
 ****************************************************************************/

int receive_message(const char *qname, struct app_message_s *msg)
{
  int ret = ERROR;
  mqd_t mq;

  mq = mq_open(qname, O_RDONLY);
  if (mq != (mqd_t)-1)
    {
      ret = mq_receive(mq, (char *)msg, sizeof(struct app_message_s), 0);
      if (ret == sizeof(struct app_message_s))
        {
          ret = OK;
        }

      mq_close(mq);
    }

  return ret;
}
