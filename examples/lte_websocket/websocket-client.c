/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2014 Jeremy Hahn
 *  Copyright 2020 Sony Corporation
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

#include <signal.h>
#include "client.h"
#include "chat_client.h"
#include "websocket_lte_connection.h"

#define WEBSOCKET_CHAT_SERVER         "ws://ruby-websockets-chat.herokuapp.com:80/"
#define WEBSOCKET_CHAT_SERVER_SSL     "wss://ruby-websockets-chat.herokuapp.com:443/"
#define WEBSOCKET_CHAT_SERVER_ROOTCA  "/mnt/sd0/DigiCertHighAssuranceEVRootCA.crt"

#define BUFFER_SIZE  255

static cwebsocket_client websocket_client;
static cwebsocket_subprotocol websocket_protocol;

int main_exit(int exit_status) {
	printf("exiting cwebsocket : %s\n", (exit_status==EXIT_SUCCESS)?"SUCCESS":"FAILURE");
	/* Disconnect from lte */
	app_websocket_disconnect_from_lte();
	return exit_status;
}

void print_program_header(void) {
	printf("\n");
	printf("                      ______                    ______      _____ \n");
	printf(" _________      _________  /_______________________  /________  /_\n");
	printf(" _  ___/_ | /| / /  _ \\_  __ \\_  ___/  __ \\  ___/_  //_/  _ \\  __/\n");
	printf(" / /__ __ |/ |/ //  __/  /_/ /(__  )/ /_/ / /__ _  ,<  /  __/ /_  \n");
	printf(" \\___/ ____/|__/ \\___//_____//____/ \\____/\\___/ /_/|_| \\___/\\__/\n");
	printf("\n");
	printf("                                   WebSocket Client              \n");
	printf("                                   Copyright (c) 2014 Jeremy Hahn\n");
	printf("                                   mail@jeremyhahn.com           \n");
	printf("\n");
}

void print_program_usage(const char *progname) {
	printf("usage: [type] [message] ...\n");
	printf("example: %s chat WebSocket Works!\n", progname);
	printf("example: %s chat-ssl WebSocket Works!\n", progname);
	printf("\n");
	exit(0);
}

int main(int argc, char FAR **argv)
{
	cwebsocket_subprotocol* protocols;
	char buffer[BUFFER_SIZE+1];
	int  cnt;

	print_program_header();

	/* Chat protocol */
	if (strstr(argv[1], "chat") != NULL) {

		/* Check argument */

		if(argc < 4) print_program_usage(argv[0]);

		/* Connect to lte */

		if (app_websocket_connect_to_lte()) {
			return EXIT_FAILURE;
		}

		/* Initialize */

		cwebsocket_subprotocol_chat_client_new(&websocket_protocol);
		protocols = &websocket_protocol;
		cwebsocket_client_init(&websocket_client, &protocols, 1);

		if ((strstr(argv[1], "ssl") != NULL) || (strstr(argv[1], "SSL") != NULL)) {
			websocket_client.uri = WEBSOCKET_CHAT_SERVER_SSL;
			cwebsocket_client_ssl_init(&websocket_client,
				 WEBSOCKET_CHAT_SERVER_ROOTCA, NULL, NULL, "");
		}
		else {
			websocket_client.uri = WEBSOCKET_CHAT_SERVER;
		}

		/* Connect */

		if (cwebsocket_client_connect(&websocket_client) == -1) {
			return main_exit(EXIT_FAILURE);
		}

		/* Create message */

		memset(buffer, 0, BUFFER_SIZE+1);
		strncat(buffer, "{\"handle\":\"", BUFFER_SIZE-strlen(buffer));
		strncat(buffer, argv[2], BUFFER_SIZE-strlen(buffer));
		strncat(buffer, "\",\"text\":\"", BUFFER_SIZE-strlen(buffer));
		for (cnt=3; cnt<argc; cnt++) {
			strncat(buffer, argv[cnt], BUFFER_SIZE-strlen(buffer)-1);
			strncat(buffer, " ", 1);
		}
		strncat(buffer, "\"}", BUFFER_SIZE-strlen(buffer));

		/* Send request and recv responsee */

		cwebsocket_client_write_data(&websocket_client, buffer, strlen(buffer), TEXT_FRAME);
		cwebsocket_client_read_data(&websocket_client);

		/* Close */

		cwebsocket_client_close(&websocket_client, 1000, "main: run loop complete");
		return main_exit(EXIT_SUCCESS);
	}

	else {
		print_program_usage(argv[0]);
	}

	return EXIT_FAILURE;
}
