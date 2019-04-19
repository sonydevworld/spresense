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

#include "server.h"

int cwebsocket_server_setnonblocking(int fd) {
	int flags;
	flags = fcntl(fd, F_GETFL);
	if (flags < 0) return flags;
	flags |= O_NONBLOCK;
	if(fcntl(fd, F_SETFL, flags) < 0) return -1;
	return 0;
}

void cwebsocket_server_init(int port, cwebsocket_subprotocol *subprotocols[], int subprotocol_len) {
	websocket_server = malloc(sizeof(cwebsocket_server) + (sizeof(cwebsocket_subprotocol) * subprotocol_len));
	memset(websocket_server, 0, sizeof(cwebsocket_server));
	websocket_server->port = port;
	websocket_server->subprotocol_len = subprotocol_len;
	int i;
	for(i=0; i<subprotocol_len; i++) {
		syslog(LOG_DEBUG, "cwebsocket_server_init: initializing subprotocol %s", subprotocols[i]->name);
	    websocket_server->subprotocols[i] = subprotocols[i];
	}
	if(websocket_server->cores <= 0) {
		websocket_server->cores = sysconf(_SC_NPROCESSORS_ONLN);
	}
	syslog(LOG_DEBUG, "cwebsocket_server_init: port=%i, cores=%zd", websocket_server->port, websocket_server->cores);
}

int cwebsocket_server_listen() {

	int reuseaddr = 1;
	struct sockaddr_in srvaddr;
	memset(&srvaddr, 0, sizeof(srvaddr));

	websocket_server->fd = socket(AF_INET, SOCK_STREAM, 0);
	if(websocket_server->fd == -1) {
		syslog(LOG_CRIT, "cwebsocket_server_listen: unable to connect: %s", strerror(errno));
		return -1;
	}

	if(websocket_server->port < 0 || websocket_server->port > 65535) {
		syslog(LOG_CRIT, "cwebsocket_server_listen: invalid port %i", websocket_server->port);
		return -1;
	}

	srvaddr.sin_family = AF_INET;
	srvaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	srvaddr.sin_port = htons(websocket_server->port);

	if(bind(websocket_server->fd, (struct sockaddr*)&srvaddr, sizeof(srvaddr)) == -1) {
		syslog(LOG_CRIT, "cwebsocket_server_listen: unable to bind to socket: %s", strerror(errno));
		return -1;
	}

	if(listen(websocket_server->fd, CWS_MAX_QUEUED_CONNECTIONS) == -1) {
		syslog(LOG_CRIT, "cwebsocket_server_listen: unable to set maximum queued connections: %s", strerror(errno));
		return -1;
	}

	if(setsockopt(websocket_server->fd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) == -1) {
	   syslog(LOG_CRIT, "cwebsocket_server_listen: failed to set SO_REUSEADDR sockopt: %s", strerror(errno));
	   return -1;
	}

	if(cwebsocket_server_setnonblocking(websocket_server->fd) == -1) {
		syslog(LOG_CRIT, "cwebsocket_server_listen: unable to set socket to non-blocking mode: %s", strerror(errno));
		return -1;
	}

	syslog(LOG_DEBUG, "cwebsocket_server_listen: listening for incoming connections...");

	websocket_server->ev_loop_accept = ev_default_loop(0);
	ev_io_init(&websocket_server->ev_io_accept, cwebsocket_server_accept, websocket_server->fd, EV_READ);
	ev_io_start(websocket_server->ev_loop_accept, &websocket_server->ev_io_accept);
	ev_loop(websocket_server->ev_loop_accept, 0);

	syslog(LOG_DEBUG, "cwebsocket_server_listen: completed libev event loop");

	return 0;
}

int cwebsocket_server_accept(struct ev_loop *loop, struct ev_io *watcher, int revents) {

	struct sockaddr_in client_addr;
	socklen_t client_len = sizeof(client_addr);
	cwebsocket_connection *connection = malloc(sizeof(cwebsocket_connection));
	if(connection == NULL) {
		perror("out of memory");
		exit(0);
	}
	memset(connection, 0, sizeof(cwebsocket_connection));
	connection->state |= WEBSOCKET_STATE_CONNECTING;

	if(EV_ERROR & revents) {
		syslog(LOG_ERR, "cwebsocket_server_accept: received invalid event");
		return -1;
	}

	connection->fd = accept(watcher->fd, (struct sockaddr *)&client_addr, &client_len);
	if(connection->fd == -1) {
		syslog(LOG_CRIT, "cwebsocket_server_accept: %s", strerror(errno));
		return -1;
	}

	/*
	if(cwebsocket_server_setnonblocking(connection->fd) == -1) {
		syslog(LOG_CRIT, "cwebsocket_server_accept: %s", strerror(errno));
		return -1;
	}*/

	//pthread_mutex_lock(&websocket_server->lock);
	//websocket_server->connections++;
	//pthread_mutex_unlock(&websocket_server->lock);

	connection->state |= WEBSOCKET_STATE_CONNECTED;
	//syslog(LOG_DEBUG, "cwebsocket_server_accept: connection #%zd accepted on fd %i\n", websocket_server->connections, connection->fd);
	syslog(LOG_DEBUG, "cwebsocket_server_accept: accepted on fd %i\n", connection->fd);

	if(pthread_attr_init(&connection->accept_thread_attr) != 0) {
		syslog(LOG_CRIT, "cwebsocket_server_accept: unable to initialize pthread attribute");
		cwebsocket_server_onerror(connection, "unable to initialize pthread attribute");
		return -1;
	}
	if(pthread_attr_setdetachstate(&connection->accept_thread_attr, PTHREAD_CREATE_DETACHED) != 0) {
		syslog(LOG_CRIT, "cwebsocket_server_accept: unable to set pthread detached state");
		cwebsocket_server_onerror(connection, "unable to set pthread detached state");
		return -1;
	}
	if(pthread_create(&connection->accept_thread, &connection->accept_thread_attr, cwebsocket_server_accept_thread, (void *)connection) == -1) {
		syslog(LOG_ERR, "cwebsocket_server_accept: %s", strerror(errno));
		cwebsocket_server_onerror(connection, strerror(errno));
		return -1;
	}

	return 0;
}

void* cwebsocket_server_accept_thread(void *args) {
	cwebsocket_connection *connection = (cwebsocket_connection *)args;
	if(cwebsocket_server_read_handshake(connection) == -1) {
		return NULL;
	}
	if(pthread_create(&connection->read_thread, NULL, cwebsocket_server_read_thread, (void *)connection) == -1) {
		syslog(LOG_ERR, "cwebsocket_server_accept: %s", strerror(errno));
		return NULL;
	}
	 pthread_attr_destroy(&connection->accept_thread_attr);
	return NULL;
}

void* cwebsocket_server_read_thread(void *args) {
	cwebsocket_connection *connection = (cwebsocket_connection *)args;
	while(connection->state & WEBSOCKET_STATE_OPEN) {
		syslog(LOG_DEBUG, "cwebsocket_server_read_thread: calling cwebsocket_server_read_data");
		cwebsocket_server_read_data(connection);
	}
	syslog(LOG_DEBUG, "cwebsocket_server_read_thread: complete");
	free(connection);
	return NULL;
}

void *cwebsocket_server_onmessage_thread(void *ptr) {
	cwebsocket_server_thread_args *args = (cwebsocket_server_thread_args *)ptr;
	cwebsocket_server_onmessage(args->connection, args->message);
	free(args->message->payload);
	free(args->message);
	free(ptr);
	return NULL;
}

int cwebsocket_server_read_handshake(cwebsocket_connection *connection) {
	char buffer[CWS_HANDSHAKE_BUFFER_MAX] = {0};
	ssize_t bytes_read = read(connection->fd, buffer, CWS_HANDSHAKE_BUFFER_MAX);
	if(bytes_read == -1) {
		syslog(LOG_ERR, "cwebsocket_server_read_handshake: %s", strerror(errno));
		return -1;
	}
	if(bytes_read == 0) {
		syslog(LOG_ERR, "cwebsocket_server_read_handshake: client closed the connection");
		//pthread_mutex_lock(&websocket_server->lock);
		//websocket_server->connections--;
		//pthread_mutex_unlock(&websocket_server->lock);
		return -1;
	}
	syslog(LOG_DEBUG, "cwebsocket_server_read_handshake:\n%s", buffer);
	return cwebsocket_server_read_handshake_handler(connection, buffer);
}

int cwebsocket_server_read_handshake_handler(cwebsocket_connection *connection, const char *handshake) {
	uint8_t flags = 0;
	char *ptr = NULL, *token = NULL, *seckey_response = NULL;
	for(token = strtok((char *)handshake, "\r\n"); token != NULL; token = strtok(NULL, "\r\n")) {
		if(*token == 'G' && *(token+1) == 'E' && *(token+2) == 'T' && *(token+3) == ' ') {
			if(strstr(token, "HTTP/1.1") == NULL && strstr(token, "HTTP/1.0") == NULL) {
				syslog(LOG_ERR, "cwebsocket_server_read_handshake_handler: invalid HTTP version header: %s", token);
				return -1;
			}
		} else {
			ptr = strchr(token, ' ');
			if(ptr == NULL) {
				syslog(LOG_ERR, "cwebsocket_server_read_handshake_handler: invalid HTTP token: %s", token);
				return -1;
			}
			*ptr = '\0';
			if(strcasecmp(token, "Upgrade:") == 0) {
				if(strcasecmp(ptr+1, "websocket") != 0) {
					syslog(LOG_ERR, "cwebsocket_server_read_handshake_handler: invalid upgrade header");
					return -1;
				}
				flags |= CWS_HANDSHAKE_HAS_UPGRADE;
			}
			if(strcasecmp(token, "Connection:") == 0) {
				if(strcasecmp(ptr+1, "upgrade") != 0) {
					syslog(LOG_ERR, "cwebsocket_server_read_handshake_handler: invalid connection header");
					return -1;
				}
				flags |= CWS_HANDSHAKE_HAS_CONNECTION;
			}
			if(strcasecmp(token, "Sec-WebSocket-Key:") == 0) {
				int key_len = strlen(ptr+1);
				char seckey[key_len];
				strcpy(seckey, ptr+1);
				seckey_response = cwebsocket_create_key_challenge_response(seckey);
				flags |= CWS_HANDSHAKE_HAS_KEY;
			}
			if(strcasecmp(token, "Sec-WebSocket-Version:") == 0) {
				if(strcmp(ptr+1, "13") != 0) {
					syslog(LOG_ERR, "cwebsocket_server_read_handshake_handler: unsupported websocket protocol version %s", ptr+1);
					return -1;
				}
				flags |= CWS_HANDSHAKE_HAS_VERSION;
			}
			if(strcasecmp(token, "Sec-WebSocket-Protocol:") == 0) {
				char *client_subprotocol, *client_subprotocols = ptr+1;
				for(client_subprotocol = strtok(client_subprotocols, " "); client_subprotocol != NULL; client_subprotocol = strtok(NULL, " ")) {
					int i;
					for(i=0; i<websocket_server->subprotocol_len; i++) {
						if(strcasecmp(websocket_server->subprotocols[i]->name, client_subprotocol) == 0) {
							connection->subprotocol = websocket_server->subprotocols[i];
						}
					}
				}
			}
		}
	}
	if(((flags & CWS_HANDSHAKE_HAS_UPGRADE) == 0) || ((flags & CWS_HANDSHAKE_HAS_CONNECTION) == 0) ||
			((flags & CWS_HANDSHAKE_HAS_KEY) == 0) || ((flags & CWS_HANDSHAKE_HAS_VERSION) == 0)) {
		// TODO send 400 bad request or 404 not found
		const char *errmsg = "invalid websocket HTTP headers";
		cwebsocket_server_onerror(connection, errmsg);
		cwebsocket_server_close_connection(connection, 1002, errmsg);
		free(seckey_response);
		free(connection);
		return -1;
	}
	if(cwebsocket_server_send_handshake_response(connection, seckey_response) == -1) {
		free(seckey_response);
		return -1;
	}
	free(seckey_response);
	return 0;
}

int cwebsocket_server_send_handshake_response(cwebsocket_connection *connection, const char *seckey) {
	char buf[512];
	snprintf(buf, 512,
	      "HTTP/1.1 101 Switching Protocols\r\n"
	      "Server: cwebsocket/%s\r\n"
	      "Upgrade: WebSocket\r\n"
	      "Connection: Upgrade\r\n"
	      "Sec-WebSocket-Accept: %s\r\n", CWS_VERSION, seckey);
	if(connection->subprotocol != NULL) {
		strcat(buf, "Sec-WebSocket-Protocol: ");
		strcat(buf, connection->subprotocol->name);
		strcat(buf, "\r\n");
	}
	strcat(buf, "\r\n");
	if(write(connection->fd, buf, strlen(buf)) == -1) {
		syslog(LOG_ERR, "cwebsocket_server_send_handshake_response: %s", strerror(errno));
		cwebsocket_server_onerror(connection, strerror(errno));
		return -1;
	}
	connection->state |= WEBSOCKET_STATE_OPEN;
	cwebsocket_server_onopen(connection);
	return 0;
}

int cwebsocket_server_read_data(cwebsocket_connection *connection) {

	/*
	ssize_t byte = 0;
	int header_length = 6;                      // The size of the header (header = everything up until the start of the payload)
	const int header_length_offset = 6;         // The byte which starts the 2 byte header
	const int extended_payload16_end_byte = 8;  // The byte which completes the extended 16-bit payload length bits
	const int extended_payload64_end_byte = 14; // The byte which completes the extended 64-bit payload length bits
	int bytes_read = 0;                         // Current byte counter
	int payload_length = 0;                     // Total length of the payload/data (minus the variable length header)
	int extended_payload_length;                // Stores the extended payload length bits, if present
	uint8_t data[CWS_DATA_BUFFER_MAX];          // Data stream buffer
	cwebsocket_frame frame;                     // WebSocket Data Frame - RFC 6455 Section 5.2
	memset(&frame, 0, sizeof(frame));
	 */

	int header_length = 6, bytes_read = 0;
	const int header_length_offset = 6;
	const int extended_payload16_end_byte = 8;
	const int extended_payload64_end_byte = 14;
	uint64_t payload_length = 0;

	uint8_t *data = malloc(CWS_DATA_BUFFER_MAX);
	if(data == NULL) {
		perror("out of memory");
		exit(-1);
	}
	memset(data, 0, CWS_DATA_BUFFER_MAX);

	cwebsocket_frame frame;
	memset(&frame, 0, sizeof(frame));

	uint64_t frame_size = header_length;
	while(bytes_read < frame_size && (connection->state & WEBSOCKET_STATE_OPEN)) {

		if(bytes_read == CWS_DATA_BUFFER_MAX) {
				syslog(LOG_ERR, "cwebsocket_server_read_data: frame too large. RECEIVE_BUFFER_MAX = %i bytes. bytes_read=%i, header_length=%i",
						CWS_DATA_BUFFER_MAX, bytes_read, header_length);
				cwebsocket_server_onerror(connection, "frame too large");
				return -1;
		}

		ssize_t byte = cwebsocket_server_read(connection, data+bytes_read, 1);

		if(byte == 0) {
		   char *errmsg = "client closed the connection";
		   cwebsocket_server_onerror(connection, errmsg);
		   cwebsocket_server_close_connection(connection, 1006, errmsg);
		   return -1;
		}
		if(byte == -1) {
		   syslog(LOG_ERR, "cwebsocket_server_read_data: error reading frame: %s", strerror(errno));
		   cwebsocket_server_onerror(connection, strerror(errno));
		   cwebsocket_server_close_connection(connection, 1002, strerror(errno));
		   return -1;
		}
		bytes_read++;

		if(bytes_read == header_length_offset) {

		   frame.fin = (data[0] & 0x80) == 0x80;
		   frame.rsv1 = (data[0] & 0x40) == 0x40;
		   frame.rsv2 = (data[0] & 0x20) == 0x20;
		   frame.rsv3 = (data[0] & 0x10) == 0x10;
		   frame.opcode = ((data[0] & 0x08) | (data[0] & 0x04) | (data[0] & 0x02) | (data[0] & 0x01));
		   frame.mask = (data[1] & 0x80) == 0x80;
		   frame.payload_len = (data[1] & 0x7F);

		   if(frame.mask == 0) {
			   cwebsocket_server_close_connection(connection, 1002, "received unmasked client frame");
			   return -1;
		   }
		}

		if(frame.payload_len <= 125) {

			frame.masking_key[0] = data[2];
			frame.masking_key[1] = data[3];
			frame.masking_key[2] = data[4];
			frame.masking_key[3] = data[5];

		    payload_length = frame.payload_len;
			frame_size = header_length + payload_length;
		}
		else if(frame.payload_len == 126 && bytes_read == extended_payload16_end_byte) {

			uint16_t extended_payload_length = 0;
			extended_payload_length |= ((uint8_t) data[2]) << 8;
			extended_payload_length |= ((uint8_t) data[3]) << 0;

			frame.masking_key[0] = data[4];
			frame.masking_key[1] = data[5];
			frame.masking_key[2] = data[6];
			frame.masking_key[3] = data[7];

			header_length += 2;
			payload_length = extended_payload_length;
		}
		else if(frame.payload_len == 127 && bytes_read == extended_payload64_end_byte) {

			header_length += 6;

			uint64_t extended_payload_length = 0;
			extended_payload_length |= ((uint64_t) data[2]) << 56;
			extended_payload_length |= ((uint64_t) data[3]) << 48;
			extended_payload_length |= ((uint64_t) data[4]) << 40;
			extended_payload_length |= ((uint64_t) data[5]) << 32;
			extended_payload_length |= ((uint64_t) data[6]) << 24;
			extended_payload_length |= ((uint64_t) data[7]) << 16;
			extended_payload_length |= ((uint64_t) data[8]) << 8;
			extended_payload_length |= ((uint64_t) data[9]) << 0;

			frame.masking_key[0] = data[10];
			frame.masking_key[1] = data[11];
			frame.masking_key[2] = data[12];
			frame.masking_key[3] = data[13];

			header_length += 6;
			payload_length = extended_payload_length;
		}
	}

	if(frame.fin && frame.opcode == TEXT_FRAME) {

		char *payload = malloc(sizeof(char) * payload_length);
		if(payload == NULL) {
			perror("out of memory");
			exit(-1);
		}
		memcpy(payload, &data[header_length], payload_length);
		payload[payload_length] = '\0';
		free(data);

		int i;
		for(i=0; i<payload_length; i++) {
			payload[i] = payload[i] ^ frame.masking_key[i%4];
		}

		size_t utf8_code_points = 0;
		if(utf8_count_code_points((uint8_t *)payload, &utf8_code_points)) {
			syslog(LOG_ERR, "cwebsocket_server_read_data: received %lld byte malformed utf-8 text payload: %s\n", payload_length, payload);
			cwebsocket_server_onerror(connection, "received malformed utf-8 payload");
			return -1;
		}

		syslog(LOG_DEBUG, "cwebsocket_server_read_data: received %lld byte text payload: %s", payload_length, payload);

		if(connection->subprotocol != NULL && connection->subprotocol->onmessage != NULL) {

			cwebsocket_message *message = malloc(sizeof(cwebsocket_message));
			if(message == NULL) {
				perror("out of memory");
				exit(-1);
			}
			memset(message, 0, sizeof(cwebsocket_message));
			message->opcode = frame.opcode;
			message->payload_len = frame.payload_len;
			message->payload = malloc(payload_length+1);
			if(message->payload == NULL) {
				perror("out of memory");
				exit(-1);
			}
			strncpy(message->payload, (char *)payload, payload_length+1);
			free(payload);

			cwebsocket_server_thread_args *args = malloc(sizeof(cwebsocket_server_thread_args));
			if(args == NULL) {
				perror("out of memory");
				exit(-1);
			}
		    memset(args, 0, sizeof(cwebsocket_server_thread_args));
		    args->connection = connection;
		    args->message = message;

		    if(pthread_create(&connection->onmessage_thread, NULL, cwebsocket_server_onmessage_thread, (void *)args) == -1) {
		    	syslog(LOG_ERR, "cwebsocket_server_read_data: %s", strerror(errno));
		    	cwebsocket_server_onerror(connection, strerror(errno));
		    	return -1;
		    }
		    return bytes_read;
		}

		free(payload);
		syslog(LOG_WARNING, "cwebsocket_server_read_data: onmessage callback not defined");
		return bytes_read;
	}
	else if(frame.fin && frame.opcode == BINARY_FRAME) {

		syslog(LOG_DEBUG, "cwebsocket_server_read_data: received BINARY payload. bytes=%lld", payload_length);

		char payload[payload_length];
		memcpy(payload, &data[header_length], payload_length);

		if(connection->subprotocol->onmessage != NULL) {

			cwebsocket_message *message = malloc(sizeof(cwebsocket_message));
			if(message == NULL) {
				perror("out of memory");
				exit(-1);
			}
			message->opcode = frame.opcode;
			message->payload_len = frame.payload_len;
			message->payload = malloc(sizeof(char) * payload_length);
			if(message->payload == NULL) {
				perror("out of memory");
				exit(-1);
			}
			memcpy(message->payload, payload, payload_length);

			cwebsocket_server_thread_args *args = malloc(sizeof(cwebsocket_server_thread_args));
			if(args == NULL) {
				perror("out of memory");
				exit(-1);
			}
			args->connection = connection;
			args->message = message;

			if(pthread_create(&connection->onmessage_thread, NULL, cwebsocket_server_onmessage_thread, (void *)args) == -1) {
				syslog(LOG_ERR, "cwebsocket_server_read_data: %s", strerror(errno));
				cwebsocket_server_onerror(connection, strerror(errno));
				return -1;
			}
			return bytes_read;
		}
		syslog(LOG_WARNING, "cwebsocket_server_read_data: undefined onmessage callback");
		return bytes_read;
	}
	else if(frame.opcode == CONTINUATION) {
		syslog(LOG_DEBUG, "cwebsocket_server_read_data: received CONTINUATION opcode");
		return -1; // continuations are accounted for in read loop
	}
	else if(frame.opcode == PING) {
		syslog(LOG_DEBUG, "cwebsocket_server_read_data: received PING control frame");
		char payload[payload_length];
		memcpy(payload, &data[header_length], payload_length);
		payload[payload_length] = '\0';
		//return cwebsocket_server_send_control_frame(connection, 0x0A, "PONG", payload);
	}
	else if(frame.opcode == PONG) {
		syslog(LOG_DEBUG, "cwebsocket_server_read_data: received PONG control frame");
		return 0;
	}
	else if(frame.opcode == CLOSE) {
		int code = 0;
		if(payload_length > 2) {
		   code = (data[header_length] << 8) + (data[header_length+1]);
		   header_length += 2;
		   payload_length -= 2;
		}
		uint8_t reason[payload_length];
		memcpy(reason, &data[header_length], sizeof(uint8_t) * payload_length);
		reason[payload_length] = '\0';
		int i;
		for(i=0; i<payload_length; i++) {
			reason[i] = reason[i] ^ frame.masking_key[i%4];
		}
		syslog(LOG_DEBUG, "cwebsocket_server_read_data: received CLOSE control frame. payload_length=%lld, code=%i, reason=%s", payload_length, code, reason);
		cwebsocket_server_close_connection(connection, (uint32_t)code, (const char *) reason);
		return 0;
	}

	char closemsg[50];
	sprintf(closemsg, "received unsupported opcode: %#04x", frame.opcode);
	syslog(LOG_ERR, "cwebsocket_client_read_data: %s", closemsg);
	cwebsocket_server_onerror(connection, closemsg);
	cwebsocket_server_close_connection(connection, 1002, closemsg);
	return -1;
}

ssize_t cwebsocket_server_write_data(cwebsocket_connection *connection, const char *data, uint64_t payload_len, opcode code) {

	if((connection->state & WEBSOCKET_STATE_OPEN) == 0) {
		syslog(LOG_DEBUG, "cwebsocket_server_write_data: websocket closed");
		cwebsocket_server_onerror(connection, "websocket closed");
		return -1;
	}

	uint32_t header_length = 2 + (payload_len > 125 ? 2 : 0) + (payload_len > 0xffff ? 8 : 0);
	uint8_t header[header_length];
	ssize_t bytes_written;

	header[0] = (code | 0x80);

	if(payload_len <= 125) {
		header[1] = payload_len;
	}
	else if(payload_len > 125 && payload_len <= 0xffff) {
		header[1] = 126;
		uint16_t len16 = htons(payload_len);
		memcpy(header+2, &len16, 2);
	}
	else if(payload_len > 0xffff && payload_len <= 0xffffffffffffffffLL) {
		header[1] = 127;
		char len64[8] = htonl64(payload_len);
		memcpy(header+2, &len64, 8);
	}
	else {
		syslog(LOG_CRIT, "cwebsocket_server_write_data: frame too large");
		cwebsocket_server_close_connection(connection, 1009, "frame too large");
		return -1;
	}

	int frame_length = header_length + payload_len;
	char framebuf[frame_length];
	memset(framebuf, 0, frame_length);
	memcpy(framebuf, header, header_length);
	memcpy(&framebuf[header_length], data, payload_len);

	bytes_written = cwebsocket_server_write(connection, framebuf, frame_length);

	if(bytes_written == -1) {
		syslog(LOG_ERR, "cwebsocket_server_write_data: error: %s", strerror(errno));
		cwebsocket_server_onerror(connection, strerror(errno));
		return -1;
	}

	syslog(LOG_DEBUG, "cwebsocket_server_write_data: bytes_written=%zu, frame_length=%i, opcode=%#04x, payload_len=%lld, payload=%s\n",
			bytes_written, frame_length, code, (long long)payload_len, data);

	return bytes_written;
}

int cwebsocket_server_close_connection(cwebsocket_connection *connection, uint16_t code, const char *reason) {
	connection->state |= WEBSOCKET_STATE_CLOSING;
	if(connection->fd > 0) {
		if(close(connection->fd) == -1) {
			syslog(LOG_ERR, "cwebsocket_server_close_connection: unable to close connection");
			return -1;
		}
		connection->fd = 0;
	}
	connection->state |= WEBSOCKET_STATE_CLOSED;
	syslog(LOG_DEBUG, "cwebsocket_server_close_connection: code=%i, reason=%s", code, reason);
	return 0;
}

int cwebsocket_server_shutdown() {
	if(websocket_server->fd > 0) {
		if(close(websocket_server->fd) == -1) {
			syslog(LOG_ERR, "cwebsocket_server_shutdown: unable to close server fd");
			return -1;
		}
	}
	free(websocket_server);
	syslog(LOG_DEBUG, "cwebsocket_server_shutdown: shutting down");
	return 0;
}

inline ssize_t cwebsocket_server_read(cwebsocket_connection *connection, void *buf, int len) {
/*
#ifdef USESSL
	return (websocket->flags & WEBSOCKET_FLAG_SSL) ?
			SSL_read(websocket->ssl, buf, len) :
			read(websocket->socket, buf, len);
#else
*/
	return read(connection->fd, buf, len);
//#endif
}

inline ssize_t cwebsocket_server_write(cwebsocket_connection *connection, void *buf, int len) {
	ssize_t bytes_written;
	pthread_mutex_lock(&connection->write_lock);
	/*
	#ifdef USESSL
		bytes_written = (connection->flags & WEBSOCKET_FLAG_SSL) ?
				SSL_write(connection->ssl, buf, len) :
				write(connection->fd, buf, len);
	#else
	*/
		bytes_written = write(connection->fd, buf, len);
	//#endif
	pthread_mutex_unlock(&connection->write_lock);
	return bytes_written;
}

inline void cwebsocket_server_onopen(cwebsocket_connection *connection) {
	if(connection->subprotocol != NULL && connection->subprotocol->onopen != NULL) {
	   connection->subprotocol->onopen(connection);
	}
}

inline void cwebsocket_server_onmessage(cwebsocket_connection *connection, cwebsocket_message *message) {
	if(connection->subprotocol != NULL && connection->subprotocol->onmessage != NULL) {
	   connection->subprotocol->onmessage(connection, message);
	}
}

inline void cwebsocket_server_onclose(cwebsocket_connection *connection, const char *message) {
	if(connection->subprotocol != NULL && connection->subprotocol->onclose != NULL) {
	   connection->subprotocol->onclose(connection, 1000, message);
	}
}

inline void cwebsocket_server_onerror(cwebsocket_connection *connection, const char *error) {
	if(connection->subprotocol != NULL && connection->subprotocol->onerror != NULL) {
	   connection->subprotocol->onerror(connection, error);
	}
}
