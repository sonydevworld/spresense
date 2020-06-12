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

#include "common.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha1.h"
//#include <pthread.h>


char* cwebsocket_base64_encode(const unsigned char *input, int length) {

	unsigned char buffer[128];
	size_t olen;
	
	mbedtls_base64_encode(buffer, sizeof( buffer ), &olen, input, length );

	if ((olen > 1) && (olen <= 128))
	{
		char *buff = (char *)malloc(olen);
		if (buff != NULL)
		{
			memcpy(buff, buffer, olen-1); 
			buff[olen-1] = '\0'; 
			return buff;
		}
	}
	return NULL;
}

void cwebsocket_print_frame(cwebsocket_frame *frame) {
	WS_DEBUG("print_frame: fin=%i, rsv1=%i, rsv2=%i, rsv3=%i, opcode=%#04x, mask=%i, payload_len=%lld\n",
			frame->fin, frame->rsv1, frame->rsv2, frame->rsv3, frame->opcode, frame->mask, frame->payload_len);
}

char* cwebsocket_create_key_challenge_response(const char *seckey) {
	const char *GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
	const int seckey_len = strlen(seckey);
	const int total_len = seckey_len + 36;
	char sha1buf[total_len];
	memcpy(sha1buf, seckey, seckey_len);
	memcpy(&sha1buf[seckey_len], GUID, 36);
	unsigned char sha1_bytes[20];
	mbedtls_sha1((const unsigned char *)sha1buf, total_len, sha1_bytes);
	return cwebsocket_base64_encode((const unsigned char *)sha1_bytes, sizeof(sha1_bytes));
}

