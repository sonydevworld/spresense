/****************************************************************************
 * examples/websocket_bitbank/websocket_bitbank.c
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

#define WSS_BITBANK_URI         "wss://stream.bitbank.cc:443/socket.io/?EIO=3&transport=websocket"
#define WSS_BITBANK_PING        "2"
#define WSS_BITBANK_UPGRADE     "5"
#define WSS_BITBANK_SUBSCRIBE   "42[\"join-room\",\"%s\"]"
#define WSS_BITBANK_KEEP_COUNT  (20)

#define WSS_BITBANK_SYMBOL_SIZE (32)
#define WSS_BITBANK_BUFFER_SIZE (128)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

enum wss_state
{
  wss_initialize = 0,
  wss_open,
  wss_message,
  wss_upgrade,
  wss_close,
};

struct wss_command
{
  char symbol[WSS_BITBANK_SYMBOL_SIZE + 1];
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

static void websocket_bitbank_onopen(void *websocket)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_bitbank_onopen: fd=%i\n", client->fd);
#endif

  state = wss_open;
}

/****************************************************************************
 * cwebsocket : on message
 ****************************************************************************/

static void websocket_bitbank_onmessage(void *websocket,
                                  cwebsocket_message *message)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_bitbank_onmessage: ");
  wss_printf("fd=%i, opcode=%#04x, payload_len=%lld, payload=%s\n",
             client->fd, message->opcode,
             message->payload_len, message->payload);
#endif

  if ((state == wss_open)
      && (message->payload_len == 2)
      && (strncmp(message->payload, "40", 2) == 0))
    {
      state = wss_message;
    }

  if (strncmp(message->payload, "42", 2) == 0)
    {
      printf("%s\n", message->payload);
    }
}

/****************************************************************************
 * cwebsocket : on close
 ****************************************************************************/

static void websocket_bitbank_onclose(void *websocket, int code,
                                      const char *reason)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_bitbank_onclose: fd=%i, code=%i, reason=%s\n",
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

static void websocket_bitbank_onerror(void *websocket, const char *message)
{
#ifdef WSS_SAMPLE_TRACE
  cwebsocket_client *client = (cwebsocket_client *)websocket;
  wss_printf("websocket_bitbank_onerror: fd=%i, message=%s\n",
             client->fd, message);
#endif
}

/****************************************************************************
 * cwebsocket : setup subprotocol
 ****************************************************************************/

static void websocket_bitbank_new(cwebsocket_subprotocol *protocol)
{
  memset(protocol, 0, sizeof(cwebsocket_subprotocol));
  protocol->name = "websocket";
  protocol->onopen = &websocket_bitbank_onopen;
  protocol->onmessage = &websocket_bitbank_onmessage;
  protocol->onclose = &websocket_bitbank_onclose;
  protocol->onerror = &websocket_bitbank_onerror;
}

/****************************************************************************
 * wss_task
 ****************************************************************************/

static int wss_task(int argc, FAR char *argv[])
{
  cwebsocket_subprotocol *protocols;
  char buffer[WSS_BITBANK_BUFFER_SIZE];
  int keep_count;
  int ret;

  printf("wss main task started.\n");

  /* setup parameter */

  state = wss_initialize;
  websocket_bitbank_new(&wss_protocol);
  protocols = &wss_protocol;
  ret = cwebsocket_client_init(&wss_client, &protocols, 1);
  if (ret < 0)
    {
      printf("cwebsocket_client_init failed : %d\n", errno);
      return -1;
    }

  ret = cwebsocket_client_ssl_init(&wss_client,
                     CONFIG_EXAMPLES_WEBSOCKET_BITBANK_ROOT_CA_FILE,
                     NULL, NULL, "");
  if (ret < 0)
    {
      printf("cwebsocket_client_ssl_init failed : %d\n", errno);
      return -1;
    }

  /* connect */

  wss_client.uri = WSS_BITBANK_URI;
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

  keep_count = 0;
  while (state != wss_close)
    {
      if (state == wss_message)
        {
          /* send upgrade request */

          ret = cwebsocket_client_write_data(&wss_client, WSS_BITBANK_UPGRADE,
                                             1, TEXT_FRAME);
          if (ret < 0)
            {
              printf("cwebsocket_client_write_data failed : %d\n", errno);
              break;
            }

          state = wss_upgrade;
        }
      else if ((state == wss_upgrade) && (strlen(new_command.symbol) > 0))
        {
          /* create request */

          memset(buffer, 0, WSS_BITBANK_BUFFER_SIZE);
          ret = snprintf(buffer, WSS_BITBANK_BUFFER_SIZE,
                         WSS_BITBANK_SUBSCRIBE, new_command.symbol);
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
      else if (keep_count > WSS_BITBANK_KEEP_COUNT)
        {
          /* send ping request */

          ret = cwebsocket_client_write_data(&wss_client, WSS_BITBANK_PING,
                                             1, TEXT_FRAME);
          if (ret < 0)
            {
              printf("cwebsocket_client_write_data failed : %d\n", errno);
              break;
            }

          keep_count = 0;
        }

      ret = cwebsocket_client_read_data(&wss_client);
      if ((ret < 0) && (errno != EAGAIN))
        {
          printf("cwebsocket_client_read_data failed : %d\n", errno);
          break;
        }

      keep_count++;
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

  if ((argc > 2) && ((strncmp(argv[1], "ticker", 6) == 0)
                     || (strncmp(argv[1], "transactions", 12) == 0)
                     || (strncmp(argv[1], "depth_diff", 10) == 0)
                     || (strncmp(argv[1], "depth_whole", 11) == 0)))
    {
      if (state == wss_initialize)
        {
          ret = task_create("wss_task",
                            CONFIG_EXAMPLES_WEBSOCKET_BITBANK_TASK_PRIORITY,
                            CONFIG_EXAMPLES_WEBSOCKET_BITBANK_TASK_STACKSIZE,
                            wss_task, NULL);
          if (ret < 0)
            {
              printf("Could not create wss task : [%d]\n", errno);
              return -1;
            }
        }

      /* set new command */

      memset(new_command.symbol, 0, sizeof(new_command.symbol));
      snprintf(new_command.symbol, sizeof(new_command.symbol),
               "%s_%s", argv[1], argv[2]);
    }

  else if ((argc > 1) && (strncmp(argv[1], "quit", 4) == 0))
    {
      if ((state == wss_message) || (state == wss_upgrade))
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
      printf("WebSocket sample using bitbank web service\n");
      printf("  usage : %s channel symbol\n", argv[0]);
      printf("          %s quit\n", argv[0]);
      printf("  [channel]\n");
      printf("    ticker       : subscribe ticker channel\n");
      printf("    transactions : subscribe transactions channel\n");
      printf("    depth_diff   : subscribe depth diff channel\n");
      printf("    depth_whole  : subscribe whole depth channel\n");
      printf("  [symbol]\n");
      printf("    btc_jpy  : Bitcoin - Yen\n");
      printf("    xrp_jpy  : Ripple - Yen\n");
      printf("    xrp_btc  : Ripple - Bitcoin\n");
      printf("    ltc_jpy  : Litecoin - Yen\n");
      printf("    ltc_btc  : Litecoin - Bitcoin\n");
      printf("    eth_jpy  : Ethereum - Yen\n");
      printf("    eth_btc  : Ethereum - Bitcoin\n");
      printf("    mona_jpy : Monacoin - Yen\n");
      printf("    mona_btc : Monacoin - Bitcoin\n");
      printf("    bcc_jpy  : Bitcoin Cash - Yen\n");
      printf("    bcc_btc  : Bitcoin Cash - Bitcoin\n");
      printf("    xlm_jpy  : Stellar - Yen\n");
      printf("    xlm_btc  : Stellar - Bitcoin\n");
      printf("    qtum_jpy : Qtum - Yen\n");
      printf("    qtum_btc : Qtum - Bitcoin\n");
      printf("    bat_jpy  : Basic Attention Token - Yen\n");
      printf("    bat_btc  : Basic Attention Token - Bitcoin\n");
      printf("    omg_jpy  : Omisego - Yen\n");
      printf("    omg_btc  : Omisego - Bitcoin\n");
      printf("    xym_jpy  : Symbol - Yen\n");
      printf("    xym_btc  : Symbol - Bitcoin\n");
      printf("\n");
    }

  return 0;
}

