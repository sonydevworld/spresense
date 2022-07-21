/****************************************************************************
 * examples/websockt_gmocoin/websocket_gmocoin_main.c
 *
 *   Copyright 2021 Sony Group Corporation
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
 * 3. Neither the name of Sony Group Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "client.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* #define WSS_SAMPLE_TRACE */

#ifdef WSS_SAMPLE_TRACE
# define wss_printf(v, ...) printf(v, ##__VA_ARGS__)
#else
# define wss_printf(v, ...)
#endif

#define WSS_GMOCOIN_URI         "wss://api.coin.z.com:443/ws/public/v1"
#define WSS_GMOCOIN_SUBSCRIBE   "{\"command\":\"subscribe\",\"channel\":\"ticker\",\"symbol\":\"%s\"}"
#define WSS_GMOCOIN_UNSUBSCRIBE "{\"command\":\"unsubscribe\",\"channel\":\"ticker\",\"symbol\":\"%s\"}"

#define WSS_GMOCOIN_SYMBOL_SIZE (8)
#define WSS_GMOCOIN_BUFFER_SIZE (256)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

enum wss_state
{
  wss_initialize = 0,
  wss_open,
  wss_message,
  wss_close,
};

enum gmo_command
{
  gmo_none = 0,
  gmo_subscribe,
  gmo_unsubscribe
};

struct wss_command
{
  enum gmo_command command;
  char             symbol[WSS_GMOCOIN_SYMBOL_SIZE + 1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static enum wss_state         state = wss_initialize;
static cwebsocket_client      wss_client;
static cwebsocket_subprotocol wss_protocol;
static struct wss_command     new_command;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * cwebsocet : on open
 ****************************************************************************/

static void websocket_gmocoin_onopen(void *websocket)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  printf("websocket_gmocoin_onopen: fd=%i\n", client->fd);
#endif

  state = wss_open;
}

/****************************************************************************
 * cwebsocket : on message
 ****************************************************************************/

static void websocket_gmocoin_onmessage(void *websocket,
                                  cwebsocket_message *message)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_gmocoin_onmessage: ");
  wss_printf("fd=%i, opcode=%#04x, payload_len=%lld, payload=%s\n",
             client->fd, message->opcode,
             message->payload_len, message->payload);
#endif

  if (state == wss_open)
    {
      state = wss_message;
    }

  printf("%s\n", message->payload);
}

/****************************************************************************
 * cwebsocket : on close
 ****************************************************************************/

static void websocket_gmocoin_onclose(void *websocket, int code,
                                const char *reason)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_gmocoin_onclose: fd=%i, code=%i, reason=%s\n",
             client->fd, code, reason);
#endif

  if (state != wss_initialize)
    {
      state = wss_close;
    }
}

/****************************************************************************
 * cwebsocket : on error
 ****************************************************************************/

static void websocket_gmocoin_onerror(void *websocket, const char *message)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_gmocoin_onerror: fd=%i, message=%s\n",
             client->fd, message);
#endif
}

/****************************************************************************
 * cwebsocket : setup subprotocol
 ****************************************************************************/

static void websocket_gmocoin_new(cwebsocket_subprotocol *protocol)
{
  memset(protocol, 0, sizeof(cwebsocket_subprotocol));
  protocol->name = "websocket";
  protocol->onopen = &websocket_gmocoin_onopen;
  protocol->onmessage = &websocket_gmocoin_onmessage;
  protocol->onclose = &websocket_gmocoin_onclose;
  protocol->onerror = &websocket_gmocoin_onerror;
}

/****************************************************************************
 * wss_task
 ****************************************************************************/

static int wss_task(int argc, FAR char *argv[])
{
  cwebsocket_subprotocol *protocols;
  char buffer[WSS_GMOCOIN_BUFFER_SIZE];
  int ret;

  printf("wss main task started.\n");

  /* setup parameter */

  state = wss_initialize;
  websocket_gmocoin_new(&wss_protocol);
  protocols = &wss_protocol;
  ret = cwebsocket_client_init(&wss_client, &protocols, 1);
  if (ret < 0)
    {
      printf("cwebsocket_client_init failed : %d\n", errno);
      return -1;
    }

  ret = cwebsocket_client_ssl_init(&wss_client,
                     CONFIG_EXAMPLES_WEBSOCKET_GMOCOIN_ROOT_CA_FILE,
                     NULL, NULL, "");
  if (ret < 0)
    {
      printf("cwebsocket_client_ssl_init failed : %d\n", errno);
      return -1;
    }

  /* connect */

  wss_client.uri = WSS_GMOCOIN_URI;
  ret = cwebsocket_client_connect(&wss_client);
  if (ret < 0)
    {
      printf("cwebsocket_client_connect failed : %d\n", errno);
      return -1;
    }

  while (state == wss_initialize)
    {
      sleep(1);
    }

  /* main loop */

  while (state != wss_close)
    {
      if (new_command.command != gmo_none)
        {
          /* create request */

          memset(buffer, 0, WSS_GMOCOIN_BUFFER_SIZE);
          if (new_command.command == gmo_subscribe)
            {
              ret = snprintf(buffer, WSS_GMOCOIN_BUFFER_SIZE,
                             WSS_GMOCOIN_SUBSCRIBE, new_command.symbol);
            }
          else if (new_command.command == gmo_unsubscribe)
            {
              ret = snprintf(buffer, WSS_GMOCOIN_BUFFER_SIZE,
                             WSS_GMOCOIN_UNSUBSCRIBE, new_command.symbol);
            }
          else
            {
              ret = -1;
              errno = EINVAL;
            }

          if (ret <= 0)
            {
              printf("subscribe/unsubscribe format failed : %d\n", errno);
              break;
            }

          /* reset for new data */

          memset(&new_command, 0, sizeof(struct wss_command));

          /* send request */

          ret = cwebsocket_client_write_data(&wss_client, buffer,
                                             strlen(buffer), TEXT_FRAME);
          if (ret < 0)
            {
              printf("cwebsocket_client_write_data failed : %d\n", errno);
              break;
            }
        }

      ret = cwebsocket_client_read_data(&wss_client);
      if ((ret < 0) && (errno != EAGAIN))
        {
          printf("cwebsocket_client_read_data failed : %d\n", errno);
          break;
        }
    }

  /* close */

  cwebsocket_client_close(&wss_client, 1000, "main: run loop complete");
  printf("wss main task closed.\n");
  state = wss_initialize;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if ((argc > 2) && ((strncmp(argv[1], "subscribe", 3) == 0)
                     || (strncmp(argv[1], "unsubscribe", 5) == 0)))
    {
      if (state == wss_initialize)
        {
          ret = task_create("wss_task",
                            CONFIG_EXAMPLES_WEBSOCKET_GMOCOIN_TASK_PRIORITY,
                            CONFIG_EXAMPLES_WEBSOCKET_GMOCOIN_TASK_STACKSIZE,
                            wss_task, NULL);
          if (ret < 0)
            {
              printf("Could not create wss task : [%d]\n", errno);
              return -1;
            }
        }

      /* set new command */

      memset(new_command.symbol, 0, sizeof(new_command.symbol));
      if (strncmp(argv[1], "subscribe", 3) == 0)
        {
          strncpy(new_command.symbol, argv[2], WSS_GMOCOIN_SYMBOL_SIZE);
          new_command.command = gmo_subscribe;
        }
      else if (strncmp(argv[1], "unsubscribe", 5) == 0)
        {
          strncpy(new_command.symbol, argv[2], WSS_GMOCOIN_SYMBOL_SIZE);
          new_command.command = gmo_unsubscribe;
        }
    }

  else if ((argc > 1) && (strncmp(argv[1], "quit", 4) == 0))
    {
      if (state == wss_message)
        {
          state = wss_close;
          printf("Start to quit WebSocket.\n");
        }
      else
        {
          printf("Couldn't quit : illegal state.\n");
        }
    }

  else
    {
      printf("WebSocket sample using GMO coin web service\n");
      printf("  usage : %s command symbol\n", argv[0]);
      printf("          %s quit\n", argv[0]);
      printf("  [command]\n");
      printf("    subscribe   : subscribe services\n");
      printf("    unsubscribe : unsubscribe services\n");
      printf("  [symbol]\n");
      printf("    BTC     : Bitcoin\n");
      printf("    ETH     : Ethereum\n");
      printf("    BCH     : Bitcoin Cash\n");
      printf("    LTC     : Litecoin\n");
      printf("    XRP     : Ripple\n");
      printf("    XEM     : New Economy Movement\n");
      printf("    XLM     : Stellar\n");
      printf("    BTC_JPY : Bitcoin - Yen\n");
      printf("    ETH_JPY : Ethereum - Yen\n");
      printf("    BCH_JPY : Bitcoin Cash - Yen\n");
      printf("    LTC_JPY : Litecoin - Yen\n");
      printf("    XRP_JPY : Ripple - Yen\n");
      printf("\n");
    }

  return 0;
}

