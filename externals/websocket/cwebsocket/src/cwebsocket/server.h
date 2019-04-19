/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Jeremy Hahn
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

#ifndef cwebsocket_server_SERVER_H_
#define cwebsocket_server_SERVER_H_

#include <ev.h>
#include "common.h"

#ifndef CWS_MAX_CONNECTIONS
	#define CWS_MAX_CONNECTIONS 1000000
#endif

#ifndef CWS_MAX_QUEUED_CONNECTIONS
	#define CWS_MAX_QUEUED_CONNECTIONS 1000000
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	ev_io ev_io_read;
	ev_io ev_io_write;
	int fd;
	uint8_t state;
	pthread_attr_t accept_thread_attr;
	pthread_t accept_thread;
	pthread_t read_thread;
	pthread_t write_thread;
	pthread_t onmessage_thread;
	pthread_mutex_t write_lock;
	cwebsocket_subprotocol *subprotocol;
} cwebsocket_connection;

typedef struct {
	int fd;
	int port;
	uint8_t flags;
	size_t cores;
	size_t connections;
	size_t subprotocol_len;
	pthread_mutex_t lock;
	struct ev_loop *ev_loop_accept;
	struct ev_io ev_io_accept;
	cwebsocket_subprotocol *subprotocols[];
} cwebsocket_server;

typedef struct {
	cwebsocket_connection *connection;
	cwebsocket_message *message;
} cwebsocket_server_thread_args;

cwebsocket_server *websocket_server;

// "public"
void cwebsocket_server_init(int port, cwebsocket_subprotocol *subprotocols[], int subprotocol_len);
int cwebsocket_server_listen();
int cwebsocket_server_accept(struct ev_loop *loop, struct ev_io *watcher, int revents);
int cwebsocket_server_read_handshake(cwebsocket_connection *connection);
int cwebsocket_server_read_handshake_handler(cwebsocket_connection *connection, const char *handshake);
int cwebsocket_server_send_handshake_response(cwebsocket_connection *connection, const char *seckey);
int cwebsocket_server_read_data(cwebsocket_connection *connection);
ssize_t cwebsocket_server_write_data(cwebsocket_connection *connection, const char *data, uint64_t len, opcode code);
int cwebsocket_server_close_connection(cwebsocket_connection *connection, uint16_t code, const char *reason);
int cwebsocket_server_shutdown();

// "private"
void* cwebsocket_server_accept_thread(void *args);
void* cwebsocket_server_read_thread(void *args);
inline ssize_t cwebsocket_server_read(cwebsocket_connection *connection, void *buf, int len);
inline ssize_t cwebsocket_server_write(cwebsocket_connection *connection, void *buf, int len);
inline void cwebsocket_server_onopen(cwebsocket_connection *connection);
inline void cwebsocket_server_onmessage(cwebsocket_connection *connection, cwebsocket_message *message);
inline void cwebsocket_server_onclose(cwebsocket_connection *connection, const char *message);
inline void cwebsocket_server_onerror(cwebsocket_connection *connection, const char *error);

#ifdef __cplusplus
}
#endif

#endif
