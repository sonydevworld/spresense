/****************************************************************************
 * examples/sms_recv/sms_recv_main.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define MSG_BODY_UCS2_CHARACTERS 32 /* 32 characters in USC2 format: 64byte */

/* UTF-8 may be expressed in 4 bytes, so a buffer equivalent to 4 times
 * the number of characters is required for conversion.
 */

#define MSG_BODY_UTF8_BUFF_LEN (MSG_BODY_UCS2_CHARACTERS * 4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UTF-8 may be expressed in 4 bytes, so a buffer equivalent to 4 times
 * the number of characters is required for conversion.
 */

static char g_address_str[(SMS_MAX_ADDRLEN * 4) + 1];
static char g_body_str[MSG_BODY_UTF8_BUFF_LEN + 1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * read_sms_body
 ****************************************************************************/

static int read_sms_body(int sms_sock, unsigned short bodylen)
{
  unsigned short msgbody_ucs2[MSG_BODY_UCS2_CHARACTERS];
  int rsize;

  printf("message body         : ");

  /* In the case of a concatenated SMS, it is not possible to read
   * the entire text at once, so it must be read repeatedly.
   */

  do
    {
      rsize = read(sms_sock, msgbody_ucs2,
                   MIN(sizeof(msgbody_ucs2), bodylen));
      if (rsize < 0)
        {
          printf("failed to read: %d\n", errno);
          return ERROR;
        }

      /* The messge body is encoded in UCS2, so convert it to UTF-8
       * so that it can be displayed.
       */

      memset(g_body_str, 0, sizeof(g_body_str));
      if (uconv_ucs2_to_utf8(rsize / 2, msgbody_ucs2,
                             sizeof(g_body_str),
                             (FAR uint8_t *)g_body_str) < 0)
        {
          printf("Failed to convert message body\n");
        }
      else
        {
          printf("%s", g_body_str);
        }

      bodylen -= rsize;
    }
  while (bodylen > 0);

  printf("\n");

  return OK;
}

/****************************************************************************
 * read_status_report
 ****************************************************************************/

static int read_status_report(int sms_sock)
{
  struct sms_status_report_s report;
  int rsize;

  rsize = read(sms_sock, &report, sizeof(report));
  if (rsize < 0)
    {
      printf("failed to read: %d\n", errno);
      return ERROR;
    }

   printf("reference ID         : %u\n", report.refid);
   printf("status               : %s\n",
          (report.status == SMS_STATUS_SUCCESS) ? "success" :
          (report.status == SMS_STATUS_FAILED) ? "failed" : "pending");
   printf("discharge time       : %02d/%02d/%02d : %02d:%02d:%02d %s%02d\n",
          report.discharge_time.year, report.discharge_time.mon,
          report.discharge_time.mday, report.discharge_time.hour,
          report.discharge_time.min, report.discharge_time.sec,
          (report.discharge_time.tz >= 0) ? "+" : "",
          report.discharge_time.tz);

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
  int ret;
  int sms_sock;
  int rsize;
  FAR struct sms_recv_msg_header_s msgheader;

  sms_sock = socket(AF_INET, SOCK_SMS, 0);
  if (sms_sock < 0)
    {
      printf("Failed to open socket:%d\n", errno);
      return ERROR;
    }

  printf("socket open success:%d\n", sms_sock);

  while (1)
    {
      /* read message header */

      rsize = read(sms_sock, &msgheader, sizeof(msgheader));
      if (rsize < 0)
        {
          printf("failed to read: %d\n", errno);
          close(sms_sock);
          return ERROR;
        }

      printf("-----------------------------------------------\n");
      printf("sent time            : %02d/%02d/%02d : %02d:%02d:%02d %s%02d\n",
             msgheader.send_time.year, msgheader.send_time.mon,
             msgheader.send_time.mday, msgheader.send_time.hour,
             msgheader.send_time.min, msgheader.send_time.sec,
             (msgheader.send_time.tz >= 0) ? "+" : "",
             msgheader.send_time.tz);
      printf("message type         : %s\n",
             (msgheader.msgtype == SMS_MSG_TYPE_DELIVER) ?
             "Deliver message" : "Status report message");
      printf("source address length: %u\n", msgheader.srcaddrlen);
      printf("message body length  : %u\n", msgheader.datalen);

      if (msgheader.srcaddrlen != 0)
        {
          /* The address is encoded in UCS2, so convert it to UTF-8
           * so that it can be displayed.
           */

          memset(g_address_str, 0, sizeof(g_address_str));
          if (uconv_ucs2_to_utf8(msgheader.srcaddrlen / 2, msgheader.srcaddr,
                                 sizeof(g_address_str),
                                 (FAR uint8_t *)g_address_str) < 0)
            {
              printf("Failed to convert address\n");
            }
          else
            {
              printf("source address       : %s\n", g_address_str);
            }
        }

      /* read message body */

      switch (msgheader.msgtype)
        {
          case SMS_MSG_TYPE_DELIVER:
            ret = read_sms_body(sms_sock, msgheader.datalen);
            if (ret < 0)
              {
                close(sms_sock);
                return ERROR;
              }

            break;

          case SMS_MSG_TYPE_STATUS_REPORT:
            ret = read_status_report(sms_sock);
            if (ret < 0)
              {
                close(sms_sock);
                return ERROR;
              }

            break;

          default:
            printf("Unknown message type: %u\n", msgheader.msgtype);
            close(sms_sock);
            return ERROR;
            break;
        }
    }

  close(sms_sock);

  return OK;
}
