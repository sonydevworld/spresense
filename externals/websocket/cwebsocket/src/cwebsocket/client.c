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

#include "client.h"

int cwebsocket_client_init(cwebsocket_client *websocket, cwebsocket_subprotocol **subprotocols, int subprotocol_len) {
	if (subprotocol_len > WEBSOCKET_SUBPROTOCOL_MAX)
		return -1;
	websocket->fd = 0;
	websocket->retry = 0;
	websocket->uri = '\0';
	websocket->flags = 0;
	websocket->state = WEBSOCKET_STATE_CLOSED;
	websocket->proxy_addr = NULL;
	websocket->proxy_port = NULL;
	websocket->headers = NULL;
	websocket->num_headers = 0;
	websocket->subprotocol_len = subprotocol_len;
	int i;
	for(i=0; i<subprotocol_len; i++) {
		WS_DEBUG("client_init: loading subprotocol %s\n", subprotocols[i]->name);

		websocket->subprotocols[i] = subprotocols[i];
	}
	websocket->subprotocol = websocket->subprotocols[0];
	return 0;
}

void cwebsocket_client_set_proxy(cwebsocket_client *websocket, char *proxy_addr, char *proxy_port)
{
	if (strlen(proxy_addr) > 0 && strlen(proxy_port) > 0){
		websocket->flags |= WEBSOCKET_FLAG_PROXY;
		websocket->proxy_addr = proxy_addr;
		websocket->proxy_port = proxy_port;
	}else {
		WS_DEBUG("client_set_proxy: invalid proxy_addr\n");
	}
}
void cwebsocket_client_unset_proxy(cwebsocket_client *websocket)
{
	if (websocket->flags & WEBSOCKET_FLAG_PROXY){
		websocket->proxy_addr = NULL;
		websocket->proxy_port = NULL;
		websocket->flags |= ~WEBSOCKET_FLAG_PROXY;
	}
}

void cwebsocket_client_parse_uri(cwebsocket_client *websocket, const char *uri,
		char *hostname, char *port, char *resource, char *querystring) {

	if(sscanf(uri, "ws://%[^:]:%[^/]%[^?]%s", hostname, port, resource, querystring) == 4) {
		return;
	}
	else if(sscanf(uri, "ws://%[^:]:%[^/]%s", hostname, port, resource) == 3) {
		strcpy(querystring, "");
		return;
	}
	else if(sscanf(uri, "ws://%[^:]:%[^/]%s", hostname, port, resource) == 2) {
		strncpy(resource, "/", strlen("/") + 1);
		strcpy(querystring, "");
		return;
	}
	else if(sscanf(uri, "ws://%[^/]%s", hostname, resource) == 2) {
		strncpy(port, "80", strlen("80") + 1);
		strcpy(querystring, "");
		return;
	}
	else if(sscanf(uri, "ws://%[^/]", hostname) == 1) {
		strncpy(port, "80", strlen("80") + 1);
		strncpy(resource, "/", strlen("/") + 1);
		strcpy(querystring, "");
		return;
	}
#ifdef ENABLE_SSL
	else if(sscanf(uri, "wss://%[^:]:%[^/]%[^?]%s", hostname, port, resource, querystring) == 4) {
		websocket->flags |= WEBSOCKET_FLAG_SSL;
		return;
	}
	else if(sscanf(uri, "wss://%[^:]:%[^/]%s", hostname, port, resource) == 3) {
		strcpy(querystring, "");
		websocket->flags |= WEBSOCKET_FLAG_SSL;
		return;
	}
	else if(sscanf(uri, "wss://%[^:]:%[^/]%s", hostname, port, resource) == 2) {
		strncpy(resource, "/", strlen("/") + 1);
		strcpy(querystring, "");
		websocket->flags |= WEBSOCKET_FLAG_SSL;
		return;
	}
	else if(sscanf(uri, "wss://%[^/]%s", hostname, resource) == 2) {
		strncpy(port, "443", strlen("443") + 1);
		strcpy(querystring, "");
		websocket->flags |= WEBSOCKET_FLAG_SSL;
		return;
	}
	else if(sscanf(uri, "wss://%[^/]", hostname) == 1) {
		strncpy(port, "443", strlen("443") + 1);
		strncpy(resource, "/", strlen("/") + 1);
		strcpy(querystring, "");
		websocket->flags |= WEBSOCKET_FLAG_SSL;
		return;
	}
#endif
	else if(strstr(uri, "wss://") > 0) {
		WS_DEBUG("client_parse_uri: recompile with SSL support to use a secure connection");
		exit(1);
	}
	else {
		WS_DEBUG("client_parse_uri: invalid websocket URL : %s\n", uri);
		exit(1);
	}
}


int cwebsocket_client_ssl_init(cwebsocket_client *websocket, char *cert_name, char *cli_cert, char *cli_key, char *passphrase) {
#ifdef ENABLE_SSL
	int ret= 0;

	WS_DEBUG("client_connect: using secure (SSL) connection\n");

	mbedtls_net_init( &websocket->ssl_net_ctx );
	mbedtls_ssl_init( &websocket->ssl );
	mbedtls_ssl_config_init( &websocket->conf );
	mbedtls_x509_crt_init( &websocket->cacert );
	mbedtls_ctr_drbg_init( &websocket->ctr_drbg );
	mbedtls_entropy_init( &websocket->entropy );

	if( ( ret = mbedtls_ctr_drbg_seed( &websocket->ctr_drbg, mbedtls_entropy_func,
                                                           &websocket->entropy,
							   (const unsigned char *) "ssl_client1",
							   strlen( "ssl_client1" ) ) ) != 0 )
	{
		WS_DEBUG(" failed\n	! mbedtls_ctr_drbg_seed returned %d\n", ret );
			return -1;
	}
	ret = mbedtls_x509_crt_parse_file( &websocket->cacert, cert_name );
	if( ret != 0 )
	{
		WS_DEBUG("nothing file = -0x%x\n", -ret);
		return -1;
	}

	if( ( ret = mbedtls_ssl_config_defaults( &websocket->conf,
					MBEDTLS_SSL_IS_CLIENT,
					MBEDTLS_SSL_TRANSPORT_STREAM,
					MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
	{
		WS_DEBUG(" failed\n	! mbedtls_ssl_config_defaults returned -0x%x\n\n", ret );
		return -1;
	}

	/* OPTIONAL is not optimal for security,
	 * but makes interop easier in this simplified example */
		mbedtls_ssl_conf_authmode( &websocket->conf, MBEDTLS_SSL_VERIFY_OPTIONAL );
		mbedtls_ssl_conf_ca_chain( &websocket->conf, &websocket->cacert, NULL );
		mbedtls_ssl_conf_rng( &websocket->conf, mbedtls_ctr_drbg_random, &websocket->ctr_drbg );

	if (cli_cert != NULL && cli_key != NULL){
		ret = mbedtls_x509_crt_parse_file(&websocket->clicert, cli_cert);
		if (ret == 0) {
			ret = mbedtls_pk_parse_keyfile(&websocket->pkey, cli_key, passphrase);
			if (ret != 0) {
				WS_DEBUG("Private Key not found\n");
				return -1;
			}
		} else {
			WS_DEBUG("Client certificate not found\n");
			return -1;
		}
		mbedtls_ssl_conf_own_cert(&websocket->conf, &websocket->clicert, &websocket->pkey);
	}
	

	return ret;
#else
	return -1;
#endif
}

int cwebsocket_client_connect(cwebsocket_client *websocket) {

	if(websocket->state & WEBSOCKET_STATE_CONNECTED) {
		WS_DEBUG("client_connect: socket already connected");
		return -1;
	}

	if(websocket->state & WEBSOCKET_STATE_CONNECTING) {
		WS_DEBUG("client_connect: socket already connecting");
		return -1;
	}

	if(websocket->state & WEBSOCKET_STATE_OPEN) {
		WS_DEBUG("client_connect: socket already open");
		return -1;
	}

#ifdef ENABLE_THREADS
	if(pthread_mutex_init(&websocket->lock, NULL) != 0) {
		syslog(LOG_ERR, "cwebsocket_client_connect: unable to initialize websocket mutex: %s\n", strerror(errno));
		cwebsocket_client_onerror(websocket, strerror(errno));
		return -1;
	}
	if(pthread_mutex_init(&websocket->write_lock, NULL) != 0) {
		syslog(LOG_ERR, "cwebsocket_client_connect: unable to initialize websocket write mutex: %s\n", strerror(errno));
		cwebsocket_client_onerror(websocket, strerror(errno));
		return -1;
	}
	pthread_mutex_lock(&websocket->lock);
	websocket->state = WEBSOCKET_STATE_CONNECTING;
	pthread_mutex_unlock(&websocket->lock);
#else
	websocket->state = WEBSOCKET_STATE_CONNECTING;
#endif
#ifdef ENABLE_SSL
	uint32_t flags;
	int ret;
#endif
	char hostname[100];
	char port[6];
	char resource[256];
	char querystring[256];

	memset(hostname, 0, 100);
	memset(port, 0, 6);
	memset(resource, 0, 256);
	memset(querystring, 0, 100);

	cwebsocket_client_parse_uri(websocket, websocket->uri, hostname, port, resource, querystring);

	WS_DEBUG ("client_connect: hostname=%s, port=%s, resource=%s, querystring=%s, secure=%i\n",
			hostname, port, resource, querystring, (websocket->flags & WEBSOCKET_FLAG_SSL));

	char handshake[CWS_HANDSHAKE_BUFFER_MAX+1];
	struct addrinfo hints, *servinfo;
	time_t Tick0 = 0;
	time(&Tick0);

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;

	srand(Tick0);
	char nonce[16];
	static const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVXYZabcdefghijklmnopqrstuvwxyz";
	int i;
	for(i = 0; i < 16; i++) {
		nonce[i] = alphanum[rand() % 61];
	}
	char *seckey = cwebsocket_base64_encode((const unsigned char *)nonce, sizeof(nonce));

	snprintf(handshake, CWS_HANDSHAKE_BUFFER_MAX,
			  "GET %s%s HTTP/1.1\r\n"
			  "Host: %s:%s\r\n"
			  "Upgrade: WebSocket\r\n"
			  "Connection: Upgrade\r\n"
			  "Sec-WebSocket-Key: %s\r\n"
			  "Sec-WebSocket-Version: 13\r\n"
			  ,resource, querystring, hostname, port, seckey);

	if((websocket->headers != NULL) && (websocket->num_headers > 0)) {
		for(i = 0; i < websocket->num_headers; i++) {
			strncat(handshake, websocket->headers[i], (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
			strncat(handshake, "\r\n", (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
		}
	}

	if(websocket->subprotocol_len > 0) {
		strncat(handshake, "Sec-WebSocket-Protocol: ", (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
		for(i=0; i<websocket->subprotocol_len; i++) {
			strncat(handshake, websocket->subprotocols[i]->name, (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
			if(i<websocket->subprotocol_len) {
				strncat(handshake, " ", (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
			}
			else {
				strncat(handshake, "\r\n", (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
			}
		}
		strncat(handshake, "\r\n", (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));
	}

	strncat(handshake, "\r\n", (CWS_HANDSHAKE_BUFFER_MAX - strlen(handshake)));

	
	if(websocket->flags & WEBSOCKET_FLAG_SSL) {
#ifdef ENABLE_SSL
		if( ( ret = mbedtls_ssl_setup( &websocket->ssl, &websocket->conf ) ) != 0 )
		{
			WS_DEBUG( " failed\n	 ! mbedtls_ssl_setup returned -0x%x\n\n", -ret );
			goto fail;
		}
		if( ( ret = mbedtls_ssl_set_hostname( &websocket->ssl, hostname ) ) != 0 )
		{
			WS_DEBUG( " failed\n	 ! mbedtls_ssl_set_hostname returned -0x%x\n\n", -ret );
			goto fail;
		}
		mbedtls_ssl_conf_read_timeout( &websocket->conf, 10000); /* 10 second Timeout */
		if(websocket->flags & WEBSOCKET_FLAG_PROXY) {
			/*  SSL On / Proxy On
				Execute CONNECT command of non-secure message after connecting to proxy server.
			*/
			if( ( ret = mbedtls_net_connect( &websocket->ssl_net_ctx, websocket->proxy_addr,
												 websocket->proxy_port, MBEDTLS_NET_PROTO_TCP ) ) != 0 )
			{
				WS_DEBUG(" failed\n	! mbedtls_net_connect proxy returned %d\n\n", ret );
				goto fail;
			}
			if(websocket->flags & WEBSOCKET_FLAG_PROXY) {
				char proxy_connect[CWS_HANDSHAKE_BUFFER_MAX];
				
				snprintf(proxy_connect, CWS_HANDSHAKE_BUFFER_MAX,
						  "CONNECT %s:%s HTTP/1.1\r\n"
						  "Host: %s\r\n"
						  , hostname, port, hostname);
				strncat(proxy_connect, "\r\n", (CWS_HANDSHAKE_BUFFER_MAX - strlen(proxy_connect)));
				if(write(websocket->ssl_net_ctx.fd, proxy_connect, strlen(proxy_connect)) == -1) {
					WS_DEBUG("proxy_client_write: NG A\n");
					goto fail;
				}
				if(cwebsocket_client_read_handshake(websocket, seckey, websocket->flags) == -1) {
					WS_DEBUG("proxy_client_read_handshake for SSL: %s\n", seckey);
					goto fail;
				}
			}
		}else{
			/*  SSL On / Proxy OFF
				Connecting to websocket server.
			*/
			if( ( ret = mbedtls_net_connect( &websocket->ssl_net_ctx, hostname,
												 port, MBEDTLS_NET_PROTO_TCP ) ) != 0 )
			{
				WS_DEBUG(" failed\n	! mbedtls_net_connect returned %d\n\n", ret );
				goto fail;
			}
		}
			
		mbedtls_ssl_set_bio( &websocket->ssl, &websocket->ssl_net_ctx, mbedtls_net_send, mbedtls_net_recv, mbedtls_net_recv_timeout );

		while( ( ret = mbedtls_ssl_handshake( &websocket->ssl ) ) != 0 )
		{
			if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
			{
				WS_DEBUG(" failed\n	! mbedtls_ssl_handshake returned -0x%x\n\n", -ret );
				goto fail;
			}
		}
		websocket->fd = websocket->ssl_net_ctx.fd;

		/* In real life, we probably want to bail out when ret != 0 */
		if( ( flags = mbedtls_ssl_get_verify_result( &websocket->ssl ) ) != 0 )
		{
			char vrfy_buf[512];

			mbedtls_x509_crt_verify_info( vrfy_buf, sizeof( vrfy_buf ), "  ! ", flags );

			WS_DEBUG("%s\n", vrfy_buf );

		}
		else
			WS_DEBUG(" certificate ok\n");
#endif
	}else {
		if(websocket->flags & WEBSOCKET_FLAG_PROXY) {
			if(getaddrinfo(websocket->proxy_addr, websocket->proxy_port, &hints, &servinfo) != 0 ) {
				freeaddrinfo(servinfo);
				const char *errmsg = "invalid hostname or IP";
				WS_DEBUG("client_proxy_connect: %s", websocket->proxy_addr);
				cwebsocket_client_onerror(websocket, errmsg);
				goto fail;
			} 
		}else {
			if(getaddrinfo(hostname, port, &hints, &servinfo) != 0 ) {
				freeaddrinfo(servinfo);
				const char *errmsg = "invalid hostname or IP";
				WS_DEBUG("client_non_proxy_connect: %s", errmsg);
				cwebsocket_client_onerror(websocket, errmsg);
				goto fail;
			}
		}

		websocket->fd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol);
		if(websocket->fd < 0) {
			freeaddrinfo(servinfo);
	//		cwebsocket_client_onerror(websocket, strerror(errno));
			goto fail;
		}

		if(connect(websocket->fd, servinfo->ai_addr, servinfo->ai_addrlen) != 0 ) {
			freeaddrinfo(servinfo);
	//		cwebsocket_client_onerror(websocket, strerror(errno));
			websocket->state = WEBSOCKET_STATE_CLOSED;
			if(websocket->retry > 0) {
				usleep(websocket->retry * 1000);
				cwebsocket_client_connect(websocket);
			}
			goto fail;
		}
		freeaddrinfo(servinfo);

		int optval = 1;
		if(setsockopt(websocket->fd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof optval) == -1) {
	//		cwebsocket_client_onerror(websocket, strerror(errno));
			goto fail;
		}
		struct timeval tv;
		tv.tv_sec  =  10;  /* 10 second Timeout */
		tv.tv_usec =  0;

		if(setsockopt(websocket->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval)) == -1) {
	//		cwebsocket_client_onerror(websocket, strerror(errno));
			goto fail;
		}

		if(websocket->flags & WEBSOCKET_FLAG_PROXY) {
			/*  SSL Off / Proxy On
					Execute CONNECT command after connecting to proxy server.
			*/
			char proxy_connect[CWS_HANDSHAKE_BUFFER_MAX];
			
			snprintf(proxy_connect, CWS_HANDSHAKE_BUFFER_MAX,
					  "CONNECT %s:%s HTTP/1.1\r\n"
					  "Host: %s\r\n"
					  , hostname, port, hostname);
			strncat(proxy_connect, "\r\n", (CWS_HANDSHAKE_BUFFER_MAX - strlen(proxy_connect)));
			if(write(websocket->fd, proxy_connect, strlen(proxy_connect)) == -1) {
				WS_DEBUG("proxy_client_write: NG\n");
				goto fail;
			}
			if(cwebsocket_client_read_handshake(websocket, seckey, websocket->flags) == -1) {
				WS_DEBUG("proxy_client_read_handshake: %s\n", seckey);
				goto fail;
			}
		}
	}

#ifdef ENABLE_THREADS
	pthread_mutex_lock(&websocket->lock);
	websocket->state = WEBSOCKET_STATE_CONNECTED;
	pthread_mutex_unlock(&websocket->lock);
#else
	websocket->state = WEBSOCKET_STATE_CONNECTED;
#endif

	if(cwebsocket_client_write(websocket, handshake, strlen(handshake)) == -1) {
		WS_DEBUG("client_connect: NG\n");
//		cwebsocket_client_onerror(websocket, strerror(errno));
		goto fail;
	}

	if(cwebsocket_client_read_handshake(websocket, seckey, 0) == -1) {
		WS_DEBUG("client_connect: %s\n", seckey);
//		cwebsocket_client_onerror(websocket, strerror(errno));
		goto fail;
	}

	free(seckey);
#ifdef ENABLE_THREADS
	pthread_mutex_lock(&websocket->lock);
	websocket->state = WEBSOCKET_STATE_OPEN;
	pthread_mutex_unlock(&websocket->lock);
#else
	websocket->state = WEBSOCKET_STATE_OPEN;
#endif

	cwebsocket_client_onopen(websocket);

	return 0;
fail:
	free(seckey);
	cwebsocket_client_close(websocket, 1009, "connect failed");
	return -1;
}

int cwebsocket_client_handshake_handler(cwebsocket_client *websocket, const char *handshake_response, char *seckey) {
	uint8_t flags = 0;
	WS_DEBUG("client_handshake_handler: handshake response: \n%s\n", handshake_response);
	char *ptr = NULL, *token = NULL;
	for(token = strtok((char *)handshake_response, "\r\n"); token != NULL; token = strtok(NULL, "\r\n")) {
		if(*token == 'H' && *(token+1) == 'T' && *(token+2) == 'T' && *(token+3) == 'P') {
			ptr = strchr(token, ' ');
			ptr = strchr(ptr+1, ' ');
			*ptr = '\0';
			if(strcmp(token, "HTTP/1.1 101") != 0 && strcmp(token, "HTTP/1.0 101") != 0) {
				cwebsocket_client_onerror(websocket, "cwebsocket_client_handshake_handler: invalid HTTP status response code");
				return -1;
			}
		} else {
			ptr = strchr(token, ' ');
			if(ptr == NULL) {
				WS_DEBUG("client_handshake_handler: invalid HTTP header sent: %s\n", token);
				cwebsocket_client_onerror(websocket, "invalid HTTP header sent");
				return -1;
			}
			*ptr = '\0';
			if(strcasecmp(token, "Upgrade:") == 0) {

				if(strncasecmp(ptr+1, "websocket", strlen("websocket") -1) != 0) {
					cwebsocket_client_onerror(websocket, "cwebsocket_client_handshake_handler: invalid HTTP upgrade header");
					return -1;
				}
				flags |= CWS_HANDSHAKE_HAS_UPGRADE;
			}
			if(strcasecmp(token, "Connection:") == 0) {
				if(strncasecmp(ptr+1, "upgrade", strlen("upgrade") -1) != 0) {
					cwebsocket_client_onerror(websocket, "cwebsocket_client_handshake_handler: invalid HTTP connection header");
					return -1;
				}
				flags |= CWS_HANDSHAKE_HAS_CONNECTION;
			}
			if(strcasecmp(token, "Sec-WebSocket-Protocol:") == 0) {
				int i;
				for(i=0; i<websocket->subprotocol_len; i++) {
					if(strncasecmp(ptr+1, websocket->subprotocols[i]->name, strlen(websocket->subprotocols[i]->name) -1) == 0) {
						websocket->subprotocol = websocket->subprotocols[i];
						WS_DEBUG("client_handshake_handler: setting subprotocol to %s\n", websocket->subprotocol->name);
					}
				}
			}
			if(strcasecmp(token, "Sec-WebSocket-Accept:") == 0) {
				char* response = cwebsocket_create_key_challenge_response(seckey);
				if(strncmp(ptr+1, response, strlen(response) -1) != 0) {
					//free(seckey);
					if(websocket->subprotocol->onerror != NULL) {
						char errmsg[255];
						sprintf(errmsg, "cwebsocket_client_handshake_handler: Sec-WebSocket-Accept header does not match computed sha1/base64 response. expected=%s, actual=%s", response, ptr+1);
						cwebsocket_client_onerror(websocket, errmsg);
					}
					free(response);
					return -1;
				}
				free(response);
				//free(seckey);
				flags |= CWS_HANDSHAKE_HAS_ACCEPT;
			}
		}
	}
	if(((flags & CWS_HANDSHAKE_HAS_UPGRADE) == 0) || ((flags & CWS_HANDSHAKE_HAS_CONNECTION) == 0) ||
				((flags & CWS_HANDSHAKE_HAS_ACCEPT) == 0)) {
		// TODO send http error code (500?)
		cwebsocket_client_close(websocket, 1002, "invalid websocket HTTP headers");
		return -1;
	}
	WS_DEBUG("client_handshake_handler: handshake successful\n");
	return 0;
}

int cwebsocket_client_read_handshake(cwebsocket_client *websocket, char *seckey, int flags) {

	int byte, tmplen = 0;
	uint32_t bytes_read = 0;
	uint8_t data[CWS_HANDSHAKE_BUFFER_MAX];
	memset(data, 0, CWS_HANDSHAKE_BUFFER_MAX);

	while(bytes_read <= CWS_HANDSHAKE_BUFFER_MAX) {

		byte = cwebsocket_client_read(websocket, data+bytes_read, 1);

		if(byte == 0) return -1;
		if(byte == -1) {
			if (errno == EAGAIN) {
				continue;
			}
			WS_DEBUG("client_read_handshake: %s", strerror(errno));
//			cwebsocket_client_onerror(websocket, strerror(errno));
			return -1;
		}
		if(bytes_read == CWS_HANDSHAKE_BUFFER_MAX) {
			WS_DEBUG("client_read_handshake: handshake response too large. CWS_HANDSHAKE_BUFFER_MAX = %i bytes.", CWS_HANDSHAKE_BUFFER_MAX);
			cwebsocket_client_onerror(websocket, "handshake response too large");
			return -1;
		}
		if((data[bytes_read] == '\n' && data[bytes_read-1] == '\r' && data[bytes_read-2] == '\n' && data[bytes_read-3] == '\r')) {
			break;
		}
		bytes_read++;
	}

	tmplen = bytes_read - 3;
	char buf[tmplen+1];
	memcpy(buf, data, tmplen);
	buf[tmplen] = '\0';
	
	if(flags & WEBSOCKET_FLAG_PROXY) {
		return cwebsocket_proxy_client_handshake_handler(websocket, buf);
	} else {
		return cwebsocket_client_handshake_handler(websocket, buf, seckey);
	}
}

int cwebsocket_proxy_client_handshake_handler(cwebsocket_client *websocket, const char *handshake_response) {
	uint8_t flags = 0;
	WS_DEBUG("proxy_client_handshake_handler: handshake response: \n%s\n", handshake_response);
	char *ptr = NULL, *token = NULL;
	
	/*
		The response code from the proxy server allows only 200.
	*/
	for(token = strtok((char *)handshake_response, "\r\n"); token != NULL; token = strtok(NULL, "\r\n")) {
		if(*token == 'H' && *(token+1) == 'T' && *(token+2) == 'T' && *(token+3) == 'P') {
			ptr = strchr(token, ' ');
			ptr = strchr(ptr+1, ' ');
			*ptr = '\0';
	 		if((websocket->flags & WEBSOCKET_FLAG_PROXY) && (strcmp(token, "HTTP/1.1 200") == 0 || strcmp(token, "HTTP/1.0 200") == 0)) {
				WS_DEBUG("client_handshake_handler: proxy handshake successful\n");
				return 0;
			}
		} else {
			ptr = strchr(token, ' ');
			if(ptr == NULL) {
				WS_DEBUG("client_handshake_handler: invalid HTTP header sent: %s\n", token);
				cwebsocket_client_onerror(websocket, "invalid HTTP header sent");
				return -1;
			}
			*ptr = '\0';
		}
	}
	if(((flags & CWS_HANDSHAKE_HAS_UPGRADE) == 0) || ((flags & CWS_HANDSHAKE_HAS_CONNECTION) == 0) ||
				((flags & CWS_HANDSHAKE_HAS_ACCEPT) == 0)) {
		// TODO send http error code (500?)
		cwebsocket_client_close(websocket, 1002, "invalid websocket HTTP headers");
		return -1;
	}
	WS_DEBUG("client_handshake_handler: handshake successful\n");
	return 0;
}

void cwebsocket_client_listen(cwebsocket_client *websocket) {
	while(websocket->state & WEBSOCKET_STATE_OPEN) {
		WS_DEBUG("client_listen: calling cwebsocket_client_read_data");
		cwebsocket_client_read_data(websocket);
	}
	WS_DEBUG("client_listen: shutting down");
}

#ifdef ENABLE_THREADS
void *cwebsocket_client_onmessage_thread(void *ptr) {
	cwebsocket_client_thread_args *args = (cwebsocket_client_thread_args *)ptr;
	cwebsocket_client_onmessage(args->socket, args->message);
	//free(args->message->payload);
	free(args->message);
	free(ptr);
	return NULL;
}
#endif

int cwebsocket_client_send_control_frame(cwebsocket_client *websocket, opcode code, const char *frame_type, uint8_t *payload, int payload_len) {
	if(websocket->fd <= 0) return -1;
	ssize_t bytes_written;
	int header_len = 6;
	int frame_len = header_len + payload_len;
	uint8_t control_frame[frame_len];
	memset(control_frame, 0, frame_len);
	uint8_t masking_key[4];
	cwebsocket_client_create_masking_key(masking_key);
	control_frame[0] = (code | 0x80);
	control_frame[1] = (payload_len | 0x80);
	control_frame[2] = masking_key[0];
	control_frame[3] = masking_key[1];
	control_frame[4] = masking_key[2];
	control_frame[5] = masking_key[3];
	if(code & CLOSE) {
		if(payload_len >= 2) {
		   if(payload_len > 2) {
			  int i;
			  for(i=0; i<payload_len; i++) {
				  control_frame[header_len+i] = (payload[i] ^ masking_key[i % 4]) & 0xff;
			  }
			  WS_DEBUG("client_send_control_frame: opcode=%#04x, frame_type=%s, payload_len=%i, payload=%s\n",
					  code, frame_type, payload_len, payload + 2);
		   }
		   else {
				WS_DEBUG("client_send_control_frame: opcode=%#04x, frame_type=%s, payload_len=%i, payload=(null)\n",
						code, frame_type, payload_len);
		   }
		}
		else {
			WS_DEBUG("client_send_control_frame: opcode=%#04x, frame_type=%s, payload_len=%i, payload=(null)\n",
					code, frame_type, payload_len);
		}
	}
	else {
		int i;
		for(i=0; i<payload_len; i++) {
			control_frame[header_len+i] = (payload[i] ^ masking_key[i % 4]) & 0xff;
		}
	}
	bytes_written = cwebsocket_client_write(websocket, control_frame, frame_len);
	if(bytes_written == 0) {
		WS_DEBUG("client_send_control_frame: remote host closed the connection\n");
		return 0;
	}
	else if(bytes_written == -1) {
		WS_DEBUG("client_send_control_frame: error sending control frame\n");
//		cwebsocket_client_onerror(websocket, strerror(errno));
		return -1;
	}
	else {
		WS_DEBUG("client_send_control_frame: wrote %zd byte\n", bytes_written);
	}
	return bytes_written;
}

int cwebsocket_client_read_data(cwebsocket_client *websocket) {

	int header_length = 2, bytes_read = 0;
	const int header_length_offset = 2;
	const int extended_payload16_end_byte = 4;
	const int extended_payload64_end_byte = 10;
	uint64_t payload_length = 0;

	uint8_t *data = malloc(CWS_DATA_BUFFER_MAX);
	if(data == NULL) {
		WS_DEBUG("client_read_data: data out of memory");
		cwebsocket_client_close(websocket, 1009, "out of memory");
		return -1;
	}
	memset(data, 0, CWS_DATA_BUFFER_MAX);

	cwebsocket_frame frame;
	memset(&frame, 0, sizeof(frame));

	uint64_t frame_size = header_length;
	while(bytes_read < frame_size && (websocket->state & WEBSOCKET_STATE_OPEN)) {

		if(bytes_read >= CWS_DATA_BUFFER_MAX) {
			WS_DEBUG("client_read_data: frame too large. RECEIVE_BUFFER_MAX = %i bytes. bytes_read=%i, header_length=%i",
					CWS_DATA_BUFFER_MAX, bytes_read, header_length);
			cwebsocket_client_close(websocket, 1009, "frame too large");
			free(data);
			return -1;
		}

		ssize_t byte = cwebsocket_client_read(websocket, data+bytes_read, 1);

		if(byte == 0) {
		   char *errmsg = "server closed the connection";
		   cwebsocket_client_onerror(websocket, errmsg);
		   cwebsocket_client_close(websocket, 1006, errmsg);
		   free(data);
		   return -1;
		}
		if(byte == -1) {
//		   syslog(LOG_ERR, "cwebsocket_client_read_data: error reading frame: %s", strerror(errno));
//		   cwebsocket_client_onerror(websocket, strerror(errno));
		   free(data);
		   return -1;
		}
		bytes_read++;

		if(bytes_read == header_length_offset) {

		   frame.fin = (data[0] & 0x80) == 0x80 ? 1 : 0;
		   frame.rsv1 = (data[0] & 0x40) == 0x40 ? 1 : 0;
		   frame.rsv2 = (data[0] & 0x20) == 0x20 ? 1 : 0;
		   frame.rsv3 = (data[0] & 0x10) == 0x10 ? 1 : 0;
		   frame.opcode = (data[0] & 0x7F);
		   frame.mask = data[1] & 0x80;
		   frame.payload_len = (data[1] & 0x7F);

		   if(frame.mask == 1) {
			   const char *errmsg = "received masked frame from server";
			   WS_DEBUG("client_read_data: %s", errmsg);
			   cwebsocket_client_onerror(websocket, errmsg);
			   free(data);
			   return -1;
		   }

		   payload_length = frame.payload_len;
		   frame_size = header_length + payload_length;
		}

		if(frame.payload_len == 126 && bytes_read == extended_payload16_end_byte) {

			header_length += 2;

			uint16_t extended_payload_length = 0;
			extended_payload_length |= ((uint8_t) data[2]) << 8;
			extended_payload_length |= ((uint8_t) data[3]) << 0;

			payload_length = extended_payload_length;
			frame_size = header_length + payload_length;
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

			payload_length = extended_payload_length;
			frame_size = header_length + payload_length;
		}
	}

	if(frame.fin && frame.opcode == TEXT_FRAME) {

		char *payload = malloc(sizeof(char) * payload_length+1);
		if(payload == NULL) {
			WS_DEBUG("client_read_data: payload out of memory");
			cwebsocket_client_close(websocket, 1009, "out of memory");
			free(data);
			return -1;
		}
		memcpy(payload, &data[header_length], payload_length);
		payload[payload_length] = '\0';
		free(data);
		data = NULL;

		size_t utf8_code_points = 0;
		if(utf8_count_code_points((uint8_t *)payload, &utf8_code_points)) {
			WS_DEBUG("client_read_data: received %lld byte malformed utf8 text payload: %s", payload_length, payload);
			cwebsocket_client_onerror(websocket, "received malformed utf8 payload");
			free(payload);
			return -1;
		}

		WS_DEBUG("client_read_data: received %lld byte text payload: %s", payload_length, payload);

		if(websocket->subprotocol != NULL && websocket->subprotocol->onmessage != NULL) {

#ifdef ENABLE_THREADS
			cwebsocket_message *message = malloc(sizeof(cwebsocket_message));
			if(message == NULL) {
				WS_DEBUG("client_read_data: text message out of memory");
				cwebsocket_client_close(websocket, 1009, "out of memory");
				free(payload);
				return -1;
			}
			memset(message, 0, sizeof(cwebsocket_message));
			message->opcode = frame.opcode;
			message->payload_len = payload_length;
			message->payload = payload;

		    cwebsocket_client_thread_args *args = malloc(sizeof(cwebsocket_client_thread_args));
		    if(args == NULL) {
				WS_DEBUG ("client_read_data: text args out of memory");
				cwebsocket_client_close(websocket, 1009, "out of memory");
				free(payload);
				return -1;
			}
		    memset(args, 0, sizeof(cwebsocket_client_thread_args));
		    args->socket = websocket;
		    args->message = message;

		    if(pthread_create(&websocket->thread, NULL, cwebsocket_client_onmessage_thread, (void *)args) == -1) {
//				syslog(LOG_ERR, "cwebsocket_client_read_data: %s", strerror(errno));
//				cwebsocket_client_onerror(websocket, strerror(errno));
				free(payload);
				free(message);
				free(args);
				return -1;
		    }
		    free(payload);
// freed at created thread
//		    free(message);
//		    free(args);
		    return bytes_read;
#else
			cwebsocket_message message = {0};
			message.opcode = frame.opcode;
			message.payload_len = payload_length;
			message.payload = payload;
			cwebsocket_client_onmessage(websocket, &message);
			free(payload);
			return bytes_read;
#endif
		}

		WS_DEBUG("client_read_data: onmessage callback undefined\n");
		free(payload);
		return bytes_read;
	}
	else if(frame.fin && frame.opcode == BINARY_FRAME) {

		WS_DEBUG("client_read_data: received BINARY payload. bytes=%lld\n", payload_length);

		char *payload = malloc(sizeof(char) * payload_length);
		if(payload == NULL) {
			perror("out of memory");
			free(data);
			exit(-1);
		}
		memcpy(payload, &data[header_length], payload_length);
		free(data);
		data = NULL;

		if(websocket->subprotocol->onmessage != NULL) {

#ifdef ENABLE_THREADS
			cwebsocket_message *message = malloc(sizeof(cwebsocket_message));
			if(message == NULL) {
				WS_DEBUG("client_read_data: binary message out of memory");
				cwebsocket_client_close(websocket, 1009, "out of memory");
				free(payload);
				return -1;
			}
			message->opcode = frame.opcode;
			message->payload_len = payload_length;
			message->payload = payload;

			cwebsocket_client_thread_args *args = malloc(sizeof(cwebsocket_client_thread_args));
			if(args == NULL) {
				WS_DEBUG("client_read_data: binary args out of memory");
				cwebsocket_client_close(websocket, 1009, "out of memory");
				return -1;
			}
			args->socket = websocket;
			args->message = message;

			if(pthread_create(&websocket->thread, NULL, cwebsocket_client_onmessage_thread, (void *)args) == -1) {
//				syslog(LOG_ERR, "cwebsocket_client_read_data: %s", strerror(errno));
//				cwebsocket_client_onerror(websocket, strerror(errno));
				free(payload);
				free(message);
				free(args);
				return -1;
			}
			free(payload);
// freed at created thread
//			free(message);
//			free(args);
			return bytes_read;
#else
			cwebsocket_message message;
			message.opcode = frame.opcode;
			message.payload_len = payload_length;
			message.payload = payload;
			cwebsocket_client_onmessage(websocket, &message);
			free(payload);
			return bytes_read;
#endif
		}
		WS_DEBUG("client_read_data: onmessage callback undefined");
		free(payload);
		return bytes_read;
	}
	else if(frame.opcode == CONTINUATION) {
		WS_DEBUG("client_read_data: onmessage callback undefined");
		free(data);
		return 0;
	}
	else if(frame.opcode == PING) {
		if(frame.fin == 0) {
			cwebsocket_client_close(websocket, 1002, "control message must not be fragmented");
		}
		if(frame.payload_len > 125) {
			cwebsocket_client_close(websocket, 1002, "control frames must not exceed 125 bytes");
			return -1;
		}
		WS_DEBUG("client_read_data: received PING control frame");
		uint8_t payload[payload_length];
		memcpy(payload, &data[header_length], payload_length);
		free(data);
		return cwebsocket_client_send_control_frame(websocket, PONG, "PONG", payload, payload_length);
	}
	else if(frame.opcode == PONG) {
		WS_DEBUG("client_read_data: received PONG control frame");
		free(data);
		return 0;
	}
	else if(frame.opcode == CLOSE) {
		if(frame.payload_len > 125) {
			cwebsocket_client_close(websocket, 1002, "control frames must not exceed 125 bytes");
			free(data);
			return -1;
		}
		int code = 0;
		if(payload_length > 2) {
		   code = (data[header_length] << 8) + (data[header_length+1]);
		   header_length += 2;
		   payload_length -= 2;
		}
		uint8_t payload[payload_length+1];
		memcpy(payload, &data[header_length], (payload_length) * sizeof(uint8_t));
		payload[payload_length] = '\0';
		free(data);
		WS_DEBUG("client_read_data: received CLOSE control frame. payload_length=%lld, code=%i, reason=%s", payload_length, code, payload);
		cwebsocket_client_close(websocket, code, NULL);
		return 0;
	}

	free(data);
	char closemsg[50];
	sprintf(closemsg, "received unsupported opcode: %#04x", frame.opcode);
	WS_DEBUG("client_read_data: %s", closemsg);
	cwebsocket_print_frame(&frame);
	cwebsocket_client_onerror(websocket, closemsg);
	cwebsocket_client_close(websocket, 1002, closemsg);
	return -1;
}

void cwebsocket_client_create_masking_key(uint8_t *masking_key) {
	uint32_t mask_bit;
	time_t Tick0 = 0;

	time(&Tick0);
	srand(Tick0);
	mask_bit = rand();
	memcpy(masking_key, &mask_bit, 4);
}

int cwebsocket_client_write_data(cwebsocket_client *websocket, const char *data, uint64_t payload_len, opcode code) {

	if((websocket->state & WEBSOCKET_STATE_OPEN) == 0) {
		WS_DEBUG("client_write_data: websocket closed");
		cwebsocket_client_onerror(websocket, "websocket closed");
		return -1;
	}

	uint32_t header_length = 6 + (payload_len > 125 ? 2 : 0) + (payload_len > 0xffff ? 6 : 0);
	uint8_t masking_key[4];
	uint8_t header[header_length];
	int bytes_written;

	cwebsocket_client_create_masking_key(masking_key);
	header[0] = (code | 0x80);

	if(payload_len <= 125) {

		header[1] = (payload_len | 0x80);
		header[2] = masking_key[0];
		header[3] = masking_key[1];
		header[4] = masking_key[2];
		header[5] = masking_key[3];
	}
	else if(payload_len > 125 && payload_len <= 0xffff) { // 125 && 65535

		uint16_t len16 = htons(payload_len);
		header[1] = (126 | 0x80);
		memcpy(header+2, &len16, 2);
		header[4] = masking_key[0];
		header[5] = masking_key[1];
		header[6] = masking_key[2];
		header[7] = masking_key[3];
	}
	else if(payload_len > 0xffff && payload_len <= 0xffffffffffffffffLL) {  // 65535 && 18446744073709551615

		char len64[8] = htonl64(payload_len);
		header[1] = (127 | 0x80);
		memcpy(header+2, len64, 8);
		header[10] = masking_key[0];
		header[11] = masking_key[1];
		header[12] = masking_key[2];
		header[13] = masking_key[3];
	}
	else {
		WS_DEBUG("client_write_data: frame too large");
		cwebsocket_client_close(websocket, 1009, "frame too large");
		return -1;
	}

	int frame_length = header_length + payload_len;
	char framebuf[frame_length];
	memset(framebuf, 0, frame_length);
	memcpy(framebuf, header, header_length);
	memcpy(&framebuf[header_length], data, payload_len);

	int i;
	for(i=0; i<payload_len; i++) {
		framebuf[header_length+i] ^= masking_key[i % 4] & 0xff;
	}

	bytes_written = cwebsocket_client_write(websocket, framebuf, frame_length);

	if(bytes_written == -1) {
		WS_DEBUG("client_write_data: error: %d", bytes_written);
//		cwebsocket_client_onerror(websocket, strerror(errno));
		return -1;
	}

	WS_DEBUG("client_write_data: bytes_written=%zu, frame_length=%i, payload_len=%lld, payload=%s\n",
			bytes_written, frame_length, (long long)payload_len, data);

	return bytes_written;
}

void cwebsocket_client_close(cwebsocket_client *websocket, uint16_t code, const char *message) {
	int code32 = 0;

	if(websocket->state == 0) {
		return;
	}

#ifdef ENABLE_SSL
	if((websocket->state & WEBSOCKET_STATE_OPEN) != 0) {
#else
	if((websocket->state & WEBSOCKET_STATE_OPEN) != 0 && websocket->fd > 0) {
#endif

#ifdef ENABLE_THREADS
		pthread_mutex_lock(&websocket->lock);
		websocket->state = WEBSOCKET_STATE_CLOSING;
		pthread_mutex_unlock(&websocket->lock);
#else
		websocket->state = WEBSOCKET_STATE_CLOSING;
#endif

		WS_DEBUG("client_close: code=%i, message=%s\n", code, message);

		if(code > 0) {
			code = code ? htons(code) : htons(1005);
			int message_len = (message == NULL) ? 0 : strlen(message);
			uint8_t close_frame[message_len+2];
			close_frame[0] = code & 0xFF;
			close_frame[1] = (code >> 8);
			code32 = (close_frame[0] << 8) + (close_frame[1]);
			int i;
			for(i=0; i<message_len; i++) {
				close_frame[i+2] = message[i];
			}
			cwebsocket_client_send_control_frame(websocket, CLOSE, "CLOSE", close_frame, message_len+2);
		}
		else {
			cwebsocket_client_send_control_frame(websocket, CLOSE, "CLOSE", NULL, 0);
		}
	}

#ifdef ENABLE_SSL
	if(websocket->flags & WEBSOCKET_FLAG_SSL) {
		WS_DEBUG("ssl_client_close: code=%i, message=%s\n", code, message);

		mbedtls_ssl_close_notify( &websocket->ssl );

		mbedtls_net_free( &websocket->ssl_net_ctx );
		mbedtls_x509_crt_free( &websocket->cacert );
		mbedtls_ssl_free( &websocket->ssl );
		mbedtls_ssl_config_free( &websocket->conf );
		mbedtls_ctr_drbg_free( &websocket->ctr_drbg );
		mbedtls_entropy_free( &websocket->entropy );
	}else {
#endif
		if(shutdown(websocket->fd, SHUT_WR) == -1) {
			WS_DEBUG("cwebsocket_client_close: unable to shutdown websocket: %s", strerror(errno));
		}
		char buf[1];
		while(read(websocket->fd, buf, 1) > 0) { buf[0] = '\0'; }
		if(close(websocket->fd) == -1) {
			WS_DEBUG("cwebsocket_client_close: error closing websocket: %s\n", strerror(errno));
	//		cwebsocket_client_onclose(websocket, 1011, strerror(errno));
		}
#ifdef ENABLE_SSL
	}
#endif
	cwebsocket_client_onclose(websocket, code32, message);

	websocket->fd = 0;

#ifdef ENABLE_THREADS
	pthread_mutex_lock(&websocket->lock);
	websocket->state = WEBSOCKET_STATE_CLOSED;
	pthread_mutex_unlock(&websocket->lock);
	pthread_cancel(websocket->thread);
#else
	websocket->state = WEBSOCKET_STATE_CLOSED;
#endif

	websocket->state = 0;

	if(websocket->flags & WEBSOCKET_FLAG_AUTORECONNECT) {
		cwebsocket_client_connect(websocket);
	}
}

ssize_t cwebsocket_client_read(cwebsocket_client *websocket, void *buf, int len) {
#ifdef ENABLE_SSL
	if (websocket->flags & WEBSOCKET_FLAG_SSL) {
		int ret = mbedtls_ssl_read(&websocket->ssl, buf, len );
		if ( ret == MBEDTLS_ERR_SSL_TIMEOUT ) {
			errno = EAGAIN;
			return -1;
		} else if ( ret < 0 ) {
			WS_DEBUG("cwebsocket_client_read: error websocket: %X\n", -ret);
			errno = EIO;
			return -1;
		} else {
			return ret;
		}
	} else {
		return read(websocket->fd, buf, len);
	}
#else
	return read(websocket->fd, buf, len);
#endif
}

ssize_t cwebsocket_client_write(cwebsocket_client *websocket, void *buf, int len) {
#ifdef ENABLE_THREADS
	ssize_t bytes_written;
	pthread_mutex_lock(&websocket->write_lock);
#ifdef ENABLE_SSL
	bytes_written = (websocket->flags & WEBSOCKET_FLAG_SSL) ?
			mbedtls_ssl_write(&websocket->ssl, buf, len ) :
			write(websocket->fd, buf, len);
#else
	bytes_written = write(websocket->fd, buf, len);
#endif
	pthread_mutex_unlock(&websocket->write_lock);
	return bytes_written;
#else
	#ifdef ENABLE_SSL
		return (websocket->flags & WEBSOCKET_FLAG_SSL) ?
				mbedtls_ssl_write(&websocket->ssl, buf, len) :
				write(websocket->fd, buf, len);
	#else
		return write(websocket->fd, buf, len);
	#endif
#endif
}

void cwebsocket_client_onopen(cwebsocket_client *websocket) {
	if(websocket->subprotocol != NULL && websocket->subprotocol->onopen != NULL) {
		websocket->subprotocol->onopen(websocket);
	}
}

void cwebsocket_client_onmessage(cwebsocket_client *websocket, cwebsocket_message *message) {
	if(websocket->subprotocol != NULL && websocket->subprotocol->onmessage != NULL) {
		websocket->subprotocol->onmessage(websocket, message);
	}
}

void cwebsocket_client_onclose(cwebsocket_client *websocket, int code, const char *message) {
	if(websocket->subprotocol != NULL && websocket->subprotocol->onclose != NULL) {
		websocket->subprotocol->onclose(websocket, code, message);
	}
}

void cwebsocket_client_onerror(cwebsocket_client *websocket, const char *error) {
	if(websocket->subprotocol != NULL && websocket->subprotocol->onerror != NULL) {
		websocket->subprotocol->onerror(websocket, error);
	}
}
