/****************************************************************************
 * system/ambient/ambient.c
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
#include <errno.h>
#include <debug.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ambient.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMBIENT_WRITEKEY_SIZE   18
#define AMBIENT_DATA_SIZE       24
#define AMBIENT_NUM_PARAMS      11
#define HTTP_STATUS_CODE_OFFSET 9

#define IOBUFFER_MAXLEN         360
#define CONTENTS_MAXLEN         192

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ambient_ctx_s
{
  int  channel;
  char write_key[AMBIENT_WRITEKEY_SIZE];
  struct
  {
    bool enable;
    char item[AMBIENT_DATA_SIZE];
  }
  data[AMBIENT_NUM_PARAMS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR static const char *ambient_keys[] =
  {
    "\"d1\":\"",
    "\"d2\":\"",
    "\"d3\":\"",
    "\"d4\":\"",
    "\"d5\":\"",
    "\"d6\":\"",
    "\"d7\":\"",
    "\"d8\":\"",
    "\"lat\":\"",
    "\"lng\":\"",
    "\"created\":\""
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_http_status_code(const char *buffer)
{
  /* Get integer value of HTTP response status code */

  return atoi(&buffer[HTTP_STATUS_CODE_OFFSET]);
}

static int check_response_status_code(const char *iobuffer)
{
  /* Check HTTP response status code */

  int   status = get_http_status_code(iobuffer);

  if (status < 200 || 300 <= status)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ambient_create
 ****************************************************************************/

ambient_ctx_t *ambient_create(int channel, FAR const char *write_key)
{
  int i;
  FAR ambient_ctx_t *ctx;

  ctx = (FAR ambient_ctx_t *)malloc(sizeof(ambient_ctx_t));
  if (ctx == NULL)
    {
      return NULL;
    }

  ctx->channel = channel;
  strncpy(ctx->write_key, write_key, AMBIENT_WRITEKEY_SIZE);

  for (i = 0; i < AMBIENT_NUM_PARAMS; i++)
    {
      ctx->data[i].enable = false;
    }

  return ctx;
}

/****************************************************************************
 * Name: ambient_set
 ****************************************************************************/

int ambient_set(FAR ambient_ctx_t *ctx, int field, FAR const char *data)
{
  if (ctx == NULL)
    {
      return -ENXIO;
    }

  --field;

  if (field < 0 || field >= AMBIENT_NUM_PARAMS)
    {
      return -EINVAL;
    }

  if (data == NULL)
    {
      return -EINVAL;
    }

  if (strlen(data) > AMBIENT_DATA_SIZE)
    {
      return -EINVAL;
    }

  ctx->data[field].enable = true;
  strncpy(ctx->data[field].item, data, AMBIENT_DATA_SIZE);

  return OK;
}

/****************************************************************************
 * Name: ambient_set_int
 ****************************************************************************/

int ambient_set_int(FAR ambient_ctx_t *ctx, int field, int data)
{
  char item[AMBIENT_DATA_SIZE];

  snprintf(item, AMBIENT_DATA_SIZE, "%d", data);

  return ambient_set(ctx, field, item);
}

/****************************************************************************
 * Name: ambient_set_double
 ****************************************************************************/

int ambient_set_double(FAR ambient_ctx_t *ctx, int field, double data)
{
  char item[AMBIENT_DATA_SIZE];

  snprintf(item, AMBIENT_DATA_SIZE, "%lf", data);

  return ambient_set(ctx, field, item);
}

/****************************************************************************
 * Name: ambient_clear
 ****************************************************************************/

int ambient_clear(FAR ambient_ctx_t *ctx, int field)
{
  if (ctx == NULL)
    {
      return -ENXIO;
    }

  --field;

  if (field < 0 || field >= AMBIENT_NUM_PARAMS)
    {
      return -EINVAL;
    }

  ctx->data[field].enable = false;

  return OK;
}

/****************************************************************************
 * Name: ambient_send
 ****************************************************************************/

int ambient_send(FAR ambient_ctx_t *ctx)
{
  int i;
  int ret;
  int sockfd;
  struct sockaddr_in addr;
  FAR char *iobuffer = NULL;
  FAR char *body = NULL;
  size_t bodylen = 0;

  if (ctx == NULL)
    {
      return -ENXIO;
    }

  /* Create a new TCP socket */

  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      nerr("ERROR: socket failed: %d\n", errno);
      return ERROR;
    }

  /* Connect the socket to the server */

  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(CONFIG_AMBIENT_PORT);
  addr.sin_addr.s_addr = inet_addr(CONFIG_AMBIENT_IPADDR);

  ninfo("Connecting to %s...\n", CONFIG_AMBIENT_IPADDR);
  ret = connect(sockfd, (struct sockaddr *)&addr,
                         sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      nerr("ERROR: connect failed: %d\n", errno);
      goto errout;
    }

  iobuffer = (FAR char *)zalloc(IOBUFFER_MAXLEN);
  body = (FAR char*)zalloc(CONTENTS_MAXLEN);
  if (!iobuffer || !body)
    {
      nerr("ERROR: Not allocated memory\n");
      ret = -ENOMEM;
      goto errout;
    }

  bodylen += snprintf(&body[bodylen], CONTENTS_MAXLEN - bodylen,
                      "{\"writeKey\":\"%s\",", ctx->write_key);

  for (i = 0; i < AMBIENT_NUM_PARAMS; i++)
    {
      if (ctx->data[i].enable)
        {
          bodylen += snprintf(&body[bodylen], CONTENTS_MAXLEN - bodylen,
                             "%s%s\",", ambient_keys[i], ctx->data[i].item);
        }
    }

  /* Truncate the last character of ',' and overwrite */

  bodylen -= 1;
  bodylen += snprintf(&body[bodylen], CONTENTS_MAXLEN - bodylen, "}\r\n");

  snprintf(iobuffer, IOBUFFER_MAXLEN,
           "POST /api/v2/channels/%d/data HTTP/1.1\r\n"
           "Host: %s\r\n"
           "Content-Length: %d\r\n"
           "Content-Type: application/json\r\n\r\n"
           "%s", ctx->channel, CONFIG_AMBIENT_IPADDR, bodylen, body);

  ninfo("Sending '%s' (%d bytes)\n", iobuffer, strlen(iobuffer));
  ret = send(sockfd, iobuffer, strlen(iobuffer), 0);
  if (ret < 0)
    {
      nerr("ERROR: send failed: %d\n", errno);
      goto errout;
    }

  /* Wait just a little */

  usleep(30 * 1000);

  ret = recv(sockfd, iobuffer, IOBUFFER_MAXLEN - 1, 0);
  if (ret < 0)
    {
      nerr("ERROR: recv failed: %d\n", errno);
      goto errout;
    }

  iobuffer[ret] = '\0';
  ninfo("Received String : (%d bytes)\r\n%s\r\n", ret, iobuffer);

  /* Check HTTP response status code */

  if (check_response_status_code(iobuffer) < 0)
    {
      ret = ERROR;
    }

errout:
  close(sockfd);

  free(iobuffer);
  free(body);

  for (i = 0; i < AMBIENT_NUM_PARAMS; i++)
    {
      ctx->data[i].enable = false;
    }

  return ret;
}

/****************************************************************************
 * Name: ambient_delete
 ****************************************************************************/

int ambient_delete(FAR ambient_ctx_t *ctx)
{
  if (ctx == NULL)
    {
      return -ENXIO;
    }

  free(ctx);
  ctx = NULL;

  return OK;
}
