/****************************************************************************
 * examples/lte_azureiot/lte_azureiot_main.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <sdk/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/stat.h>
#include "azureiot_if.h"
#include "lte_connection.h"
#include "mbedtls_if.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_HTTP_STATUS_CODE_OFFSET 9
#define APP_IO_BUFFER_SIZE          1024
#define APP_IO_HOSTNAME_SIZE        64

/*
 * Refer to this URL and describe "IoT Hub Name", "Device ID",
 * and "Primary key" in the file defined here.
 *
 * https://docs.microsoft.com/en-US/azure/iot-hub/iot-hub-create-through-portal
 *
 * Description example:
 *
 *   iot-hub-contoso-one
 *   myDeviceId
 *   HZAww1PN3suNBkailQU1UeEllNB3j0=
 */

#define AZURE_IOT_RESOURCE_FILE     "/mnt/sd0/azureiot/resources.txt"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Root certificates */

static char certificates[] =
/* DigiCert Global Root G2 */
"-----BEGIN CERTIFICATE-----\r\n"
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\r\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\r\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\r\n"
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\r\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\r\n"
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\r\n"
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\r\n"
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\r\n"
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\r\n"
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\r\n"
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\r\n"
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\r\n"
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\r\n"
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\r\n"
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\r\n"
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\r\n"
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\r\n"
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\r\n"
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\r\n"
"MrY=\r\n"
"-----END CERTIFICATE-----\r\n"

/* Baltimore CyberTrust Root */
"-----BEGIN CERTIFICATE-----\r\n"
"MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
"RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
"VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
"DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
"ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
"VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
"mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
"IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
"mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
"XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
"dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
"jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
"BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
"DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
"9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
"jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
"Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
"ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
"R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
"-----END CERTIFICATE-----\r\n"
;

/* Azure resources */

static char IoTHubName[AZURE_IOT_INFO_NAME_MAX_SIZE];
static char DeviceID[AZURE_IOT_INFO_ID_MAX_SIZE];
static char PrimaryKey[AZURE_IOT_INFO_PRIKEY_MAX_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static char *strnstr(const char *str,
                     int         buflen,
                     const char *substr,
                     int         subbuflen)
{
  /* Support "strstr" function overrun */

  char  *cp  = NULL;
  int    len = strnlen(substr, subbuflen);
  int    i   = 0;

  if (len > buflen)
    {
      return NULL;
    }

  for (i = 0; *str && i < buflen; i++, str++)
    {
      if (strncmp(str, substr, len) == 0)
        {
          cp = (char *)str;
          break;
        }
    }

  return cp;
}

static char *strncasestr(const char *str,
                         int         buflen,
                         const char *substr,
                         int         subbuflen)
{
  char  *cp  = NULL;
  int    len = strnlen(substr, subbuflen);
  int    i   = 0;

  if (len > buflen)
    {
      return NULL;
    }

  for (i = 0; *str && i < buflen; i++, str++)
    {
      if (strncasecmp(str, substr, len) == 0)
        {
          cp = (char *)str;
          break;
        }
    }

  return cp;
}

/* ------------------------------------------------------------------------ */
static char *fgets_with_trim(char *target, int size, FILE *fp)
{
  /* Get one line from the text file and delete the line feed code */

  int   comment = 0;

  while (fgets(target, size, fp))
    {
      if (target[0] == '\r' || target[0] == '\n')
        {
          /* Skip invalid lines */

          continue;
        }

      /* Check terminate */

      char *term = target + strnlen(target, size) - 1;

      if (target[0] == '#' || comment)
        {
          if (*term == '\r' || *term == '\n')
            {
              comment = 0;
            }
          else
            {
              comment = 1;
            }

          /* Skip comment lines */

          continue;
        }

      if (*term == '\r' || *term == '\n')
        {
          *term = '\0';
        }
      else
        {
          /* Buffer over */

          target = NULL;
        }

      break;
    }

  return target;
}

/* ------------------------------------------------------------------------ */
static int open_azureiot_file(const char *file_name)
{
  /* Get Azure information from file. */

  FILE* fp  = fopen(file_name, "r");
  int   res = -1;

  if (fp == NULL)
    {
      printf("Fail: Open Azure resource file [%s]\n", file_name);
    }
  else if (fgets_with_trim(IoTHubName, sizeof(IoTHubName), fp) == NULL)
    {
      printf("Fail: IoTHubName buffer over.\n");
    }
  else if (fgets_with_trim(DeviceID, sizeof(DeviceID), fp) == NULL)
    {
      printf("Fail: DeviceID buffer over.\n");
    }
  else if (fgets_with_trim(PrimaryKey, sizeof(PrimaryKey), fp) == NULL)
    {
      printf("Fail: PrimaryKey buffer over.\n");
    }
  else
    {
      res = 0;
    }

  fclose(fp);

  return res;
}

/* ------------------------------------------------------------------------ */
static int get_http_status_code(const char *buffer)
{
  /* Get integer value of HTTP response status code */

  return atoi(&buffer[APP_HTTP_STATUS_CODE_OFFSET]);
}

/* ------------------------------------------------------------------------ */
static int get_http_body_offset(const char *buffer, int buffer_size)
{
  /* Get body position of HTTP response */

  const char  *target = "\r\n\r\n";
  char        *str    = strnstr(buffer, buffer_size, target, strlen(target));

  if (str == NULL)
    {
      return ERROR;
    }

  return str + strlen(target) - buffer;
}

/* ------------------------------------------------------------------------ */
static int check_HTTP_response_status_code(const char *iobuffer)
{
  /* Check HTTP response status code */

  int   status = get_http_status_code(iobuffer);

  if (status < 200 || 300 <= status)
    {
      printf("Fail: HTTP status %d\n", status);

      return ERROR;
    }

  return OK;
}

/* ------------------------------------------------------------------------ */
static int get_http_content_length(const char *buffer, int buffer_size)
{
  /* Get HTTP response body size */

  const char  *target = "content-length:";
  char        *str    = strncasestr(buffer, buffer_size,
                                    target, strlen(target));

  if (str == NULL)
    {
      return ERROR;
    }

  return atoi(str + strlen(target));
}

/* ------------------------------------------------------------------------ */
static int tls_http_respons_read(char *iobuffer, int iobuffer_size)
{
  /* Read all HTTP responses */

  int          ret;
  char        *buf_ptr     = iobuffer;
  int          request_len = iobuffer_size;
  const char  *target      = "\r\n\r\n";

  memset(iobuffer, 0, iobuffer_size);

  do
    {
      ret = tls_read(buf_ptr, request_len);

      if (ret == 0 || ret == iobuffer_size)
        {
          /* Successful */

          break;
        }

     if (ret < 0)
       {
          /* Unsuccessful */

          break;
       }

      if (strnstr(buf_ptr, request_len, target, strlen(target)) == NULL)
        {
          /* Not all HTTP responses have been read */

          buf_ptr     += ret;
          request_len -= ret;
        }
      else
        {
          /* Successful */

          break;
        }
    }
  while (1);

  return ret;
}

/* ------------------------------------------------------------------------ */
static int send_and_recv_message(const char *hostname,
                                 char       *iobuffer,
                                 int         write_len,
                                 int         iobuffer_size)
{
  /* Connect to Azure, send data, receive data */

  int len     = ERROR;
  int ret     = 0;
  int contlen = 0;
  int headlen = 0;
  int remain  = 0;

  if (tls_connect(hostname, certificates, sizeof(certificates)) < 0)
    {
      printf("Fail: Connect\n");
      goto exit_without_disconnect;
    }

  if ((len = tls_write(iobuffer, write_len)) < 0)
    {
      printf("Fail: tls_write()=%X\n", len);
      goto exit;
    }

  if ((len = tls_http_respons_read(iobuffer, iobuffer_size)) < 0)
    {
      printf("Fail: tls_http_respons_read()=%X\n", len);
      goto exit;
    }

  contlen = get_http_content_length(iobuffer, iobuffer_size);

  if (contlen < 0)
    {
      /* Header doesn't have content */

      goto exit;
    }

  headlen = get_http_body_offset(iobuffer, iobuffer_size);

  if (headlen < 0)
    {
      printf("Fail: get_http_body_offset()=%d\n", headlen);
      len = ERROR;
      goto exit;
    }

  remain = headlen + contlen - len;

  while (remain > 0 && len < iobuffer_size)
    {
      ret = tls_read(iobuffer + len, iobuffer_size - len);

      if (ret < 0)
        {
          printf("Fail: tls_read()=%X\n", ret);
          len = ret;
          goto exit;
        }

      len += ret;
      remain -= ret;
    }

exit:
  tls_disconnect();

exit_without_disconnect:
  return len;
}

/* ------------------------------------------------------------------------ */
static int send_message(struct azureiot_info *info, const char *message)
{
  char hostname[APP_IO_HOSTNAME_SIZE];
  char io_buffer[APP_IO_BUFFER_SIZE];
  int  len;

  if (azureiot_get_hostname(info, hostname, sizeof(hostname)) < 0)
    {
      printf("Fail: get_hostname()\n");
      return ERROR;
    }

  len = azureiot_create_sendmsg(info,
                                io_buffer,
                                sizeof(io_buffer),
                                message);

  if (len < 0)
    {
      printf("Fail: create_sendmsg()=%X\n", len);
      return ERROR;
    }

  if (send_and_recv_message(hostname,
                            io_buffer,
                            len,
                            sizeof(io_buffer)) < 0)
    {
      return ERROR;
    }

  /* Check HTTP response status code */

  if (check_HTTP_response_status_code(io_buffer) < 0)
    {
      return ERROR;
    }
  else
    {
      printf("Successful\n");
    }

  return OK;
}

/* ------------------------------------------------------------------------ */
static int recv_message(struct azureiot_info *info,
                        char                 *recv_msg,
                        int                   recv_msg_size)
{
  char hostname[APP_IO_HOSTNAME_SIZE];
  char io_buffer[APP_IO_BUFFER_SIZE];
  int  len;

  if (azureiot_get_hostname(info, hostname, sizeof(hostname)) < 0)
    {
      printf("Fail: get_hostname()\n");

      return ERROR;
    }

  len = azureiot_create_recvmsg(info,
                                io_buffer,
                                sizeof(io_buffer));

  if (len < 0)
    {
      printf("Fail: create_recvmsg()=%d\n", len);

      return ERROR;
    }

  len = send_and_recv_message(hostname,
                              io_buffer,
                              len,
                              sizeof(io_buffer));
  if (len < 0)
    {
      return ERROR;
    }

  /* Check HTTP response status code */

  if (check_HTTP_response_status_code(io_buffer))
    {
      return ERROR;
    }

  int   ofst;

  if ((ofst = get_http_body_offset(io_buffer, len)) < 0)
    {
      printf("Fail: Not body offset=%d\n", ofst);

      return ERROR;
    }

  /* Output received messages */

  len = get_http_content_length(io_buffer, len);

  if (len >= recv_msg_size)
    {
      len = recv_msg_size - 1;
    }

  if (len > 0)
    {
      memcpy(recv_msg, io_buffer + ofst, len);

      recv_msg[len] = '\0';
    }

  /* Delete message */

  if ((len = azureiot_create_deletemsg(info,
                                       io_buffer,
                                       sizeof(io_buffer))) < 0)
    {
      /* Exit because there are no messages */

      printf("No messages\n");

      return ERROR;
    }

  if (send_and_recv_message(hostname,
                            io_buffer,
                            len,
                            sizeof(io_buffer)) < 0)
    {
      return ERROR;
    }

  /* Check HTTP response status code */

  if (check_HTTP_response_status_code(io_buffer))
    {
      return ERROR;
    }
  else
    {
      printf("Successful\n");
    }

  return OK;
}

/* ------------------------------------------------------------------------ */
static int filelength(const char *file_name)
{
  struct stat st;

  if (stat(file_name, &st))
    {
      return ERROR;
    }

  if (!S_ISREG(st.st_mode))
    {
      return ERROR;
    }

  return (int)st.st_size;
}

/* ------------------------------------------------------------------------ */
static int get_fileinfo(struct azureiot_info *info,
                        char                 *io_buffer,
                        int                   io_buffer_size,
                        char                 *hostname,
                        int                   hostname_size,
                        const char           *target)
{
  /* Get AzureStrage SAS URI */

  int  len = azureiot_create_fileinfo_msg(info,
                                          io_buffer,
                                          io_buffer_size,
                                          target);

  if (len < 0)
    {
      printf("Fail: create_fileinfo_msg()=%d\n", len);

      return ERROR;
    }

  if (azureiot_get_hostname(info, hostname, hostname_size) < 0)
    {
      printf("Fail: get_hostname()\n");

      return ERROR;
    }

  return send_and_recv_message(hostname,
                               io_buffer,
                               len,
                               io_buffer_size);
}

/* ------------------------------------------------------------------------ */
static int notify_status(struct azureiot_info *info,
                                char                 *io_buffer,
                                int                   io_buffer_size,
                                char                 *hostname,
                                int                   hostname_size,
                                bool                  is_success,
                                int                   status_code)
{
  /* Create response message */

  int len = azureiot_create_notifymsg(info,
                                      io_buffer,
                                      io_buffer_size,
                                      is_success,
                                      status_code);

  if (len < 0)
    {
      printf("Fail: create_notifymsg()=%d\n", len);

      return ERROR;
    }

  if (azureiot_get_hostname(info, hostname, hostname_size) < 0)
    {
      printf("Fail: get_hostname()\n");

      return ERROR;
    }

  return send_and_recv_message(hostname,
                               io_buffer,
                               len,
                               io_buffer_size);
}

/* ------------------------------------------------------------------------ */
static int upload(struct azureiot_info *info,
                  const char           *src_filename,
                  const char           *dst_filename)
{
  char hostname[APP_IO_HOSTNAME_SIZE];
  char io_buffer[APP_IO_BUFFER_SIZE];
  int  res = ERROR;
  int  len = 0;
  int  file_size;
  bool is_success = false;
  int  status;

  /* Get upload file size */

  file_size = filelength(src_filename);

  if (file_size <= 0)
    {
      /* File open error or file size is 0 */

      printf("File open error or file size is 0.(%d)\n", file_size);

      return ERROR;
    }

  /* Get AzureStrage SAS URI */

  if (get_fileinfo(info,
                   io_buffer,
                   sizeof(io_buffer),
                   hostname,
                   sizeof(hostname),
                   dst_filename) < 0)
    {
      return ERROR;
    }

  /* Create upload message */

  if ((len = azureiot_create_uploadmsg(io_buffer,
                                       sizeof(io_buffer),
                                       hostname,
                                       sizeof(hostname),
                                       file_size)) < 0)
    {
      /* Exit because there are no file information */

      printf("Fail: Not SAS URI\n");

      return ERROR;
    }

  if (tls_connect(hostname, certificates, sizeof(certificates)) != 0)
    {
      printf("Fail connect!\n");

      return ERROR;
    }

  /* Upload start */

  size_t  size;
  size_t  upload_size = 0;

  FILE   *fp = fopen(src_filename, "rb");

  if (fp)
    {
      printf("Upload start\n");

      while (1)
        {
          size = fread(io_buffer + len, 1, sizeof(io_buffer) - len, fp);

          if (size == 0)
            {
              printf("Upload end\n");

              break;
            }

          upload_size += size;

          printf("%d/%d\n", upload_size, file_size);

          if (tls_write(io_buffer, size + len) < 0)
            {
              break;
            }

          len = 0;
        }

      fclose(fp);
    }

  /* Get HTTP respons */

  tls_http_respons_read(io_buffer, sizeof(io_buffer));

  tls_disconnect();

  /* Check HTTP response status code */

  status = get_http_status_code(io_buffer);
  if (status < 200 || 300 <= status)
    {
      printf("Failed\n");
    }
  else
    {
      printf("Successful\n");

      is_success = true;
    }

  /* Notify IoT Hub of a completed file upload */

  if (notify_status(info,
                    io_buffer,
                    sizeof(io_buffer),
                    hostname,
                    sizeof(hostname),
                    is_success,
                    status) < 0)
    {
      printf("Notify failed\n");
    }
  else
    {
      printf("Notify OK\n");

      if (is_success)
        {
          res = OK;
        }
    }

  return res;
}

/* ------------------------------------------------------------------------ */
static int download(struct azureiot_info *info,
                    const char           *src_filename,
                    const char           *dst_filename)
{
  char hostname[APP_IO_HOSTNAME_SIZE];
  char io_buffer[APP_IO_BUFFER_SIZE];
  int  res = ERROR;
  int  len = 0;
  bool is_success = false;
  int  status = 0;

  /* Get AzureStrage SAS URI */

  if (get_fileinfo(info,
                   io_buffer,
                   sizeof(io_buffer),
                   hostname,
                   sizeof(hostname),
                   src_filename) < 0)
    {
      return ERROR;
    }

  /* Download message */

  if ((len = azureiot_create_downloadmsg(io_buffer,
                                         sizeof(io_buffer),
                                         hostname,
                                         sizeof(hostname))) < 0)
    {
      /* Exit because there are no file information */

      printf("Fail: Not SAS URI\n");

      return ERROR;
    }

  /* Open download destination file */

  if (tls_connect(hostname, certificates, sizeof(certificates)) != 0)
    {
      printf("Fail: Connect!\n");

      return ERROR;
    }

  /* Command write to Azure */

  if (tls_write(io_buffer, len) < 0)
    {
      printf("Fail: tls_write()\n");

      goto errout;
    }

  /* Read HTTP response from Azure */

  len = tls_http_respons_read(io_buffer, sizeof(io_buffer));

  if (len < 0)
    {
      printf("Fail: tls_http_respons_read()=%d\n", len);

      goto errout;
    }

  /* Check HTTP response status code */

  status = get_http_status_code(io_buffer);
  if (status < 200 || 300 <= status)
    {
      goto errout;
    }
  else
    {
      is_success = true;
    }

  /* Analyze Azure response */

  int   content_length;
  int   offset;

  /* Get download file size */

  content_length = get_http_content_length(io_buffer, len);

  if (content_length <= 0)
    {
      /* Normal termination if the size of the downloaded file is 0 */

      res = OK;

      goto errout;
    }

  /* Get the offset of the actual data in the download file */

  if ((offset = get_http_body_offset(io_buffer, len)) < 0)
    {
      goto errout;
    }

  /* Ready for download content */

  FILE  *fp = fopen(dst_filename, "wb");

  if (fp == NULL)
    {
      printf("Fail: Open download destination file(%s)!\n", dst_filename);

      goto errout;
    }

  /* First data write */

  len = fwrite(io_buffer + offset, 1, len - offset, fp);

  if (len < 0)
    {
      printf("Fail: fwrite()=%d\n", len);

      goto errout_fclose;
    }

  offset = content_length - len;

  /* Next data read to write */

  printf("Download start\n");

  while (offset)
    {
      len = tls_read(io_buffer, sizeof(io_buffer));

      if (len < 0)
        {
          break;
        }

      offset -= len;

      printf("%d/%d\n", content_length - offset, content_length);

      /* Write received data to file */

      if (fwrite(io_buffer, 1, len, fp) != len)
        {
          /* Error writing something */

          printf("Fail: Writing something\n");
        }
    }

  printf("Download end.\n");

errout_fclose:

  fclose(fp);

errout:

  tls_disconnect();

  /* Notify IoT Hub of a completed file download */
  if (100 <= status && status < 600)
    {
      if (notify_status(info,
                        io_buffer,
                        sizeof(io_buffer),
                        hostname,
                        sizeof(hostname),
                        is_success,
                        status) < 0)
        {
          printf("Notify failed\n");
        }
      else
        {
          printf("Notify OK\n");

          if (is_success)
            {
              res = OK;
            }
        }
    }

  return res;
}

/* ------------------------------------------------------------------------ */
int  usage(void)
{
  printf("usage: lte_azureiot <command> [arg1] [arg1]\n\n");
  printf("These are the commands used:\n\n");
  printf("  send         Send message to AzureIoTHub\n");
  printf("               arg1: Message string to send\n");
  printf("               ex) lte_azureiot send Hello\n");
  printf("\n");
  printf("  recv         Receive message from AzureIoTHub\n");
  printf("               ex) lte_azureiot recv\n");
  printf("\n");
  printf("  upload       Upload file to AzureStorage\n");
  printf("               arg1: Upload file name\n");
  printf("               arg2: AzureStorage file name\n");
  printf("               ex) lte_azureiot upload /mnt/sd0/test.txt test.txt\n");
  printf("\n");
  printf("  download     Download file from AzureStorage\n");
  printf("               arg1: AzureStorage file name\n");
  printf("               arg2: Download file name\n");
  printf("               ex) lte_azureiot download test.txt /mnt/sd0/test.txt \n");
  printf("\n");

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * lte_azureiot_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  /* Option analysis */

  int   select = 0;

  if (argc > 2 && strcmp("send", argv[1]) == 0)
    {
      select = 0;
    }
  else if(argc > 1 && strcmp("recv", argv[1]) == 0)
    {
      select = 1;
    }
  else if(argc > 3 && strcmp("upload", argv[1]) == 0)
    {
      select = 2;
    }
  else if(argc > 3 && strcmp("download", argv[1]) == 0)
    {
      select = 3;
    }
  else
    {
      return usage();
    }

  /* Read Azure resource file */

  if (open_azureiot_file(AZURE_IOT_RESOURCE_FILE) < 0)
    {
      return ERROR;
    }

  /* Set Azure connection parameters */

  struct azureiot_info  info;

  info.IoTHubName = IoTHubName;
  info.DeviceID   = DeviceID;
  info.PrimaryKey = PrimaryKey;

  printf("LTE connect...\n");

  if (app_lte_connect_to_lte()) 
    {
      printf("LTE connect...NG\n\n");

      return ERROR;
    }

  printf("LTE connect...OK\n\n");

  switch (select)
    {
      case 0:
        printf("Device message: %s --> Cloud\n", argv[2]);
        send_message(&info, argv[2]);
        break;

      case 1:
        {
          char recv_msg[64];

          if (recv_message(&info, recv_msg, sizeof(recv_msg)) == 0)
            {
              printf("Recv message: %s\n", recv_msg);
            }
        }
        break;

      case 2:
        printf("Upload device: %s --> cloud: %s\n", argv[2], argv[3]);
        upload(&info, argv[2], argv[3]);
        break;

      case 3:
        printf("Download cloud: %s --> device: %s\n", argv[2], argv[3]);
        download(&info, argv[2], argv[3]);
        break;

      default:
        break;
    }

  printf("\nLTE disconnect...\n");

  app_lte_disconnect_from_lte();

  printf("LTE disconnect...OK\n\n");

  return 0;
}
