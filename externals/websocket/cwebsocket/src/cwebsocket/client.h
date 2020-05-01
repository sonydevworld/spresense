/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Jeremy Hahn
 *  Copyright 2019 Sony Corporation
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

#ifndef CWEBSOCKET_CLIENT_H
#define CWEBSOCKET_CLIENT_H

#include <time.h>
#include <sys/time.h>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include "sys/socket.h"
#include "netdb.h"
#include "arpa/inet.h"
#include "common.h"

#ifdef ENABLE_THREADS
#include "pthread.h"
#endif

/**
 * @file  client.h
 * @brief WebSocket API
 */

/** 
 * @ingroup lteiftop
 * @defgroup netwebsocket WebSocket
 * @brief WebSocket interface
 * @{
 */

#define WEBSOCKET_FLAG_AUTORECONNECT (1 << 1)
#define WEBSOCKET_FLAG_PROXY (1 << 5)

#define WEBSOCKET_SUBPROTOCOL_MAX 4

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup websockcomstr Structure for common interface
 * Data structure for common interface
 * @{
 */

typedef struct _cwebsocket {
	int fd;
	int retry;
	char *uri;
	uint8_t flags;
	uint8_t state;
	char *proxy_addr;
	char *proxy_port;
	char **headers;
	int num_headers;
#ifdef ENABLE_SSL
	mbedtls_ssl_context ssl;
	mbedtls_net_context ssl_net_ctx;
	mbedtls_ssl_config conf;
	mbedtls_x509_crt cacert;
	mbedtls_x509_crt clicert;
	mbedtls_pk_context pkey;
	mbedtls_entropy_context entropy;
	mbedtls_ctr_drbg_context ctr_drbg;
#endif
#ifdef ENABLE_THREADS
	pthread_t thread;
	pthread_mutex_t lock;
	pthread_mutex_t write_lock;
#endif
	int code;
	size_t subprotocol_len;
	cwebsocket_subprotocol *subprotocol;
	cwebsocket_subprotocol *subprotocols[WEBSOCKET_SUBPROTOCOL_MAX];
} cwebsocket_client;

typedef struct {
	cwebsocket_client *socket;
	cwebsocket_message *message;
} cwebsocket_client_thread_args;

// "public"

/**
 * @brief Initialized the client connection on a websocket
 *
 * @param[in] websocket : Data structure for common interface
 * @param[in] subprotocols : The array information for sub protocol
 * @param[in] subprotocol_len : Actual size of the subprotocol.
 *
 * @retval On success, 0 is returned. On error, -1 is returned.
 *
 * @detail Initialized the client connection on a websocket 
 *         
 */
int cwebsocket_client_init(cwebsocket_client *websocket, cwebsocket_subprotocol **subprotocols, int subprotocol_len);

/**
 * @brief Initialized the ssl information on a websocket
 *
 * @param[in] websocket : Data structure for common interface
 * @param[in] cert_name : Certificate name in file system
 * @param[in] pers : Personalization data (Device specific identifiers)
 *                      (Can be NULL)
 *
 * @retval On success, 0 is returned. On error, -1 is returned.
 *
 * @detail Initialized the ssl information on a websocket
 *         
 */
int cwebsocket_client_ssl_init(cwebsocket_client *websocket, char *cert_name, char *cli_cert, char *cli_key, char *passphrase);

/**
 * @brief Conneted to the websocket server
 *
 	* @param[in] websocket : Data structure for common interface.(Set the server name to the uri)
 *
 * @retval On success, 0 is returned. On error, -1 is returned.
 *
 * @detail Conneted to the websocket server
 *         
 */
int cwebsocket_client_connect(cwebsocket_client *websocket);
/**
 * @brief Read the packet data from the websocket server 
 *
 * @param[in] websocket : Data structure for common interface.
 *
 * @retval On success, Read() shall return the length of the message in bytes. On error, -1 is returned.
 *         On finish, 0 is returne.
 *
 * @detail Read the packet data from the websocket server 
 *         
 */
int cwebsocket_client_read_data(cwebsocket_client *websocket);

/**
 * @brief Wrote the packet data to the websocket server 
 *
 * @param[in] websocket : Data structure for common interface.
 *
 * @retval On success, Write() shall return the length of the message in bytes. On error, -1 is returned.
 *
 * @detail Wrote the packet data to the websocket server 
 *         
 */
int cwebsocket_client_write_data(cwebsocket_client *websocket, const char *data, uint64_t len, opcode code);

/**
 * @brief Closed the websocket session to server 
 *
 * @param[in] websocket : Data structure for common interface.
 * @param[in] code : Closed code. (Refer to chapter 7.4 of RFC6455.)
 * @param[in] reason : Closed reason.
 *
 * @retval Nothing
 *
 * @detail Closed the websocket session to server 
 *         
 */
 
void cwebsocket_client_close(cwebsocket_client *websocket, uint16_t code, const char *reason);

/**
 * @brief Set the proxy address for websocket
 *
 * @param[in] websocket : Data structure for common interface.
 * @param[in] proxy_addr : Proxy address.
 * @param[in] proxy_port : Proxy port.
 *
 * @retval Nothing
 *
 * @detail Set the proxy address for websocket
 *         
 */
void cwebsocket_client_set_proxy(cwebsocket_client *websocket, char *proxy_addr, char *proxy_port);

/**
 * @brief Released the proxy features for websocket
 *
 * @param[in] websocket : Data structure for common interface.
 *
 * @retval Nothing
 *
 * @detail Released the proxy address for websocket
 *         
 */
void cwebsocket_client_unset_proxy(cwebsocket_client *websocket);

// "private"
void cwebsocket_client_run(cwebsocket_client *websocket);
void cwebsocket_client_listen(cwebsocket_client *websocket);
void cwebsocket_client_parse_uri(cwebsocket_client *websocket, const char *uri, char *hostname, char *port, char *resource, char *querystring);
int cwebsocket_client_handshake_handler(cwebsocket_client *websocket, const char *handshake_response, char *seckey);
int cwebsocket_client_read_handshake(cwebsocket_client *websocket, char *seckey, int flags);
int cwebsocket_client_send_control_frame(cwebsocket_client *websocket, opcode opcode, const char *frame_type, uint8_t *payload, int payload_len);
void cwebsocket_client_create_masking_key(uint8_t *masking_key);
int cwebsocket_client_read(cwebsocket_client *websocket, void *buf, int len);
int cwebsocket_client_write(cwebsocket_client *websocket, void *buf, int len);
 	
void cwebsocket_client_onopen(cwebsocket_client *websocket);
void cwebsocket_client_onmessage(cwebsocket_client *websocket, cwebsocket_message *message);
void cwebsocket_client_onclose(cwebsocket_client *websocket, int code, const char *message);
void cwebsocket_client_onerror(cwebsocket_client *websocket, const char *error);

ssize_t cwebsocket_proxy_client_write(int fd, void *buf, int len);
int cwebsocket_proxy_client_handshake_handler(cwebsocket_client *websocket, const char *handshake_response);
int cwebsocket_proxy_client_read_handshake(cwebsocket_client *websocket);
/**@}*/

#ifdef __cplusplus
}
#endif

#endif
