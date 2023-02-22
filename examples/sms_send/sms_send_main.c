/****************************************************************************
 * examples/sms_send/sms_send_main.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <nuttx/net/sms.h>
#include <nuttx/wireless/lte/lte_ioctl.h>

#include "utils/uconv/uconv.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * show_usage
 ****************************************************************************/

static void show_usage(FAR const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s command\n", progname);
  fprintf(stderr, " <phone number> <text message> [<enable status report>]\n");
  fprintf(stderr, "   <enable status report> 0: disable, 1: enable \n");

  exit(exitcode);
}

/****************************************************************************
 * send_sms
 ****************************************************************************/

static int send_sms(FAR char *phone_number, FAR char *text_msg,
                    bool is_status_report)
{
  int i;
  int sms_sock;
  int ret;
  FAR struct sms_send_msg_s *sendmsg;
  struct lte_smsreq_s smsreq;

  sms_sock = socket(AF_INET, SOCK_SMS, 0);
  if (sms_sock < 0)
    {
      printf("Failed to open socket:%d\n", errno);
      return ERROR;
    }

  printf("socket open success:%d\n", sms_sock);

  /* Allocate a buffer to store the SMS header and USC2 encoded body. */

  sendmsg = (FAR struct sms_send_msg_s *)malloc(
              sizeof(struct sms_send_msg_s) + (strlen(text_msg) * 2));

  if (sendmsg == NULL)
    {
      printf("Failed to allocate memory.\n");
      close(sms_sock);
      return ERROR;
    }

  /* Determines whether or not to receive a status report message
   * for the SMS to be sent. When status report is enabled, it is
   * possible to check whether the SMS has been delivered to the
   * destination device or not.
   */

  smsreq.smsru.enable = is_status_report;
  ioctl(sms_sock, SIOCSMSENSTREP, &smsreq);

  /* USC2 encoding is required for the phone number. */

  sendmsg->header.destaddrlen = uconv_utf8_to_ucs2(
                                  strlen(phone_number),
                                  (FAR uint8_t *)phone_number,
                                  SMS_MAX_ADDRLEN, sendmsg->header.destaddr);

  /* USC2 encoding is required for the message body. */

  sendmsg->header.datalen = uconv_utf8_to_ucs2(
                              strlen(text_msg), (FAR uint8_t *)text_msg,
                              strlen(text_msg) * 2, sendmsg->data);

  if ((sendmsg->header.destaddrlen < 0) || (sendmsg->header.datalen < 0))
    {
      printf("Failed to encode USC2 for the destination address or body.\n");
      free(sendmsg);
      close(sms_sock);
      return ERROR;
    }

  /* The length encoded by uconv_utf8_to_ucs2() means the number
   * of characters. The unit set in the SMS header is the number
   * of bytes, so double it.
   */

  sendmsg->header.destaddrlen *= 2;
  sendmsg->header.datalen *= 2;


  /* Send SMS to phone number */

  ret = write(sms_sock, sendmsg, sizeof(*sendmsg) + sendmsg->header.datalen);
  if (ret < 0)
    {
      printf("Failed to send SMS: %d\n", errno);
      free(sendmsg);
      close(sms_sock);
      return ERROR;
    }

  printf("Successfully sent SMS to %s\n", phone_number);

  if (is_status_report)
    {
      /* Get the reference ID of the sent SMS.
       * This ID will be set in the received status report. Using this ID,
       * it is possible to identify if the status report is for the SMS
       * just sent.
       */

      ioctl(sms_sock, SIOCSMSGREFID, &smsreq);

      for (i = 0; (i < smsreq.smsru.refid.nrefid) &&
                  (i < SMS_CONCATENATE_MAX); i++)
        {
          printf("Get reference id[%d] = %d\n",
                 i, smsreq.smsru.refid.refid[i]);
        }
    }

  free(sendmsg);
  close(sms_sock);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * sms_send_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  FAR char *phone_number = NULL;
  FAR char *text_msg = NULL;
  bool en_status_report = false;

  /* sms_send <phone number> <text msg> [<enable status report msg>] */

  if (argc > 3)
    {
      phone_number = argv[1];
      text_msg = argv[2];
      en_status_report = (atoi(argv[3]) != 0);
    }
  else if (argc > 2)
    {
      phone_number = argv[1];
      text_msg = argv[2];
    }
  else
    {
      show_usage(argv[0], EXIT_FAILURE);
    }

  /* Send SMS to phone number */

  return send_sms(phone_number, text_msg, en_status_report);
}
