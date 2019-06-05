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

#include <signal.h>
#include "cwebsocket/client.h"
#include "cwebsocket/subprotocol/echo/echo_client.h"

cwebsocket_client websocket_client;

#define STATE_GET_CASE_COUNT     (1 << 0)
#define STATE_RUNNING_TESTS      (1 << 1)
#define STATE_GENERATNING_REPORT (1 << 2)

uint8_t STATE;
int number_of_tests = 0;

void autobahn_onopen(void *websocket) {
	cwebsocket_client *client = (cwebsocket_client *)websocket;
	syslog(LOG_DEBUG, "autobahn_onopen: fd=%i", client->fd);
}

void autobahn_onmessage(void *websocket, cwebsocket_message *message) {

	cwebsocket_client *client = (cwebsocket_client *)websocket;
	syslog(LOG_DEBUG, "autobahn_onmessage: fd=%i, opcode=%#04x, payload_len=%zu, actual_payload_len=%zd, payload=%s\n",
			client->fd, message->opcode, message->payload_len, strlen(message->payload), message->payload);

	if(STATE & STATE_GET_CASE_COUNT) {
		number_of_tests = atoi(message->payload);
		STATE |= STATE_RUNNING_TESTS;
		syslog(LOG_DEBUG, "autobahn_onmessage: fetched %i test cases", number_of_tests);
	}
	else if(STATE & STATE_RUNNING_TESTS) {
		syslog(LOG_DEBUG, "autobahn_onmessage: echoing data back to server");
		cwebsocket_client_write_data(client, message->payload, strlen(message->payload), message->opcode);
	}
}

void autobahn_onclose(void *websocket, int code, const char *message) {
	cwebsocket_client *client = (cwebsocket_client *)websocket;
	syslog(LOG_DEBUG, "autobahn_onclose: fd=%i, code=%i, message=%s", client->fd, code, message);
}

void autobahn_onerror(void *websocket, const char *message) {
	cwebsocket_client *client = (cwebsocket_client *)websocket;
	syslog(LOG_DEBUG, "autobahn_onerror: fd=%i, message=%s", client->fd, message);
}

cwebsocket_subprotocol* autobahn_testsuite_new() {
	cwebsocket_subprotocol *protocol = malloc(sizeof(cwebsocket_subprotocol));
	memset(protocol, 0, sizeof(cwebsocket_subprotocol));
	protocol->name = "echo.cwebsocket\0";
	protocol->onopen = &autobahn_onopen;
	protocol->onmessage = &autobahn_onmessage;
	protocol->onclose = &autobahn_onclose;
	protocol->onerror = &autobahn_onerror;
	return protocol;
}

int main_exit(int exit_status) {
	syslog(LOG_DEBUG, "exiting cwebsocket");
	closelog();
	return exit_status;
}

void print_program_header() {
	printf("\n");
	printf("                      ______                    ______      _____ \n");
    printf(" _________      _________  /_______________________  /________  /_\n");
    printf(" _  ___/_ | /| / /  _ \\_  __ \\_  ___/  __ \\  ___/_  //_/  _ \\  __/\n");
    printf(" / /__ __ |/ |/ //  __/  /_/ /(__  )/ /_/ / /__ _  ,<  /  __/ /_  \n");
    printf(" \\___/ ____/|__/ \\___//_____//____/ \\____/\\___/ /_/|_| \\___/\\__/\n");
    printf("\n");
    printf("                                   Autobahn Testsuite            \n");
    printf("                                   Copyright (c) 2014 Jeremy Hahn\n");
    printf("                                   mail@jeremyhahn.com           \n");
	printf("\n");
}

int main(int argc, char **argv) {

	print_program_header();

	setlogmask(LOG_UPTO(LOG_DEBUG));
	openlog("cwebsocket", LOG_CONS | LOG_PERROR, LOG_USER);
	syslog(LOG_DEBUG, "starting cwebsocket client");

	STATE |= STATE_GET_CASE_COUNT;

	// Hardcoding the protocol instead of relying on negotiation during handshake
	websocket_client.subprotocol = autobahn_testsuite_new();

	cwebsocket_client_init(&websocket_client, NULL, 0);
	websocket_client.uri = "ws://localhost:9001/getCaseCount";
	cwebsocket_client_connect(&websocket_client);
	cwebsocket_client_read_data(&websocket_client);
	cwebsocket_client_close(&websocket_client, 1000, "received number of tests");
	syslog(LOG_DEBUG, "Total number of tests: %i", number_of_tests);

	STATE = STATE_RUNNING_TESTS;
	int i;
	for(i=1; i<number_of_tests+1; i++) {

		syslog(LOG_DEBUG, "Running test %i", i);

		char uri[255];
		sprintf(uri, "ws://localhost:9001/runCase?case=%i&agent=%s", i, "cwebsocket/0.1a");

		websocket_client.uri = uri;
		if(cwebsocket_client_connect(&websocket_client) == -1) {
			break;
		}
		cwebsocket_client_listen(&websocket_client);
		//cwebsocket_client_close(&websocket_client, 1000, "test complete");
	}

	STATE |= STATE_GENERATNING_REPORT;
	websocket_client.uri = "ws://localhost:9001/updateReports?agent=cwebsocket/0.1a";
	if(cwebsocket_client_connect(&websocket_client) == -1) {
		perror("unable to connect to server to run reports");
		exit(-1);
	}
	cwebsocket_client_read_data(&websocket_client);

	cwebsocket_client_close(&websocket_client, 1000, "disconnecting");
	free(websocket_client.subprotocol);
	return main_exit(EXIT_SUCCESS);
}
