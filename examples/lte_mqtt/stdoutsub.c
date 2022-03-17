/*******************************************************************************
 * Copyright (c) 2012, 2016 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *   http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial contribution
 *    Ian Craggs - change delimiter option from char to string
 *    Al Stockdill-Mander - Version using the embedded C client
 *    Ian Craggs - update MQTTClient function names
 *******************************************************************************/

/*
 
 stdout subscriber
 
 compulsory parameters:
 
  topic to subscribe to
 
 defaulted parameters:
 
	--host localhost
	--port 1883
	--qos 2
	--delimiter \n
	--clientid stdout_subscriber
	
	--userid none
	--password none

 for example:

    stdoutsub topic/of/interest --host iot.eclipse.org

*/
#include <stdio.h>
//#include <memory.h>
#include "MQTTClient.h"
#include "lte_connection.h"

#include <stdio.h>
#include <signal.h>

#include <sys/time.h>


volatile int toStop = 0;


void usage(void)
{
	printf("MQTT stdout subscriber/publisher\n");
	printf("Usage: lte_mqtt topicname <options>, where options are:\n");
	printf("  --host <hostname> (default is localhost)\n");
	printf("  --port <port> (default is 1883)\n");
	printf("  --qos <qos> (default is 2)\n");
	printf("  --delimiter <delim> (default is \\n)\n");
	printf("  --clientid <clientid> (default is hostname+timestamp)\n");
	printf("  --username none\n");
	printf("  --password none\n");
	printf("  --showtopics <on or off> (default is on if the topic has a wildcard, else off)\n");
	printf("  --cafile <file> (path to a file of trusted CA certificate)\n");
	printf("  --cert <file> (path to a file of client certificate)\n");
	printf("  --key <file> (path to a file of client private key)\n");
	printf("  --publish <message>\n");
	exit(-1);
}


void cfinish(int sig)
{
	signal(SIGINT, NULL);
	toStop = 1;
}


struct opts_struct
{
	char* clientid;
	int nodelimiter;
	char* delimiter;
	enum QoS qos;
	char* username;
	char* password;
	char* host;
	int port;
	int showtopics;
	char* cafile;
	char* certfile;
	char* keyfile;
	char* message;
} opts =
{
	(char*)"stdout-subscriber", 0, (char*)"\n", QOS2, NULL, NULL, (char*)"localhost", 1883, 0, NULL
};


void getopts(int argc, char** argv)
{
	int count = 2;
	
	while (count < argc)
	{
		if (strcmp(argv[count], "--qos") == 0)
		{
			if (++count < argc)
			{
				if (strcmp(argv[count], "0") == 0)
					opts.qos = QOS0;
				else if (strcmp(argv[count], "1") == 0)
					opts.qos = QOS1;
				else if (strcmp(argv[count], "2") == 0)
					opts.qos = QOS2;
				else
					usage();
			}
			else
				usage();
		}
		else if (strcmp(argv[count], "--host") == 0)
		{
			if (++count < argc)
				opts.host = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--port") == 0)
		{
			if (++count < argc)
				opts.port = atoi(argv[count]);
			else
				usage();
		}
		else if (strcmp(argv[count], "--clientid") == 0)
		{
			if (++count < argc)
				opts.clientid = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--username") == 0)
		{
			if (++count < argc)
				opts.username = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--password") == 0)
		{
			if (++count < argc)
				opts.password = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--delimiter") == 0)
		{
			if (++count < argc)
				opts.delimiter = argv[count];
			else
				opts.nodelimiter = 1;
		}
		else if (strcmp(argv[count], "--showtopics") == 0)
		{
			if (++count < argc)
			{
				if (strcmp(argv[count], "on") == 0)
					opts.showtopics = 1;
				else if (strcmp(argv[count], "off") == 0)
					opts.showtopics = 0;
				else
					usage();
			}
			else
				usage();
		}
		else if (strcmp(argv[count], "--cafile") == 0)
		{
			if (++count < argc)
				opts.cafile = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--cert") == 0)
		{
			if (++count < argc)
				opts.certfile = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--key") == 0)
		{
			if (++count < argc)
				opts.keyfile = argv[count];
			else
				usage();
		}
		else if (strcmp(argv[count], "--publish") == 0)
		{
			if (++count < argc)
				opts.message = argv[count];
			else
				usage();
		}
		count++;
	}
	
}


void messageArrived(MessageData* md)
{
	MQTTMessage* message = md->message;

	if (opts.showtopics)
		printf("%.*s\t", md->topicName->lenstring.len, md->topicName->lenstring.data);
	if (opts.nodelimiter)
		printf("%.*s", (int)message->payloadlen, (char*)message->payload);
	else
		printf("%.*s%s", (int)message->payloadlen, (char*)message->payload, opts.delimiter);
	fflush(stdout);
}


int main(int argc, char** argv)
{
	int rc = 0;
	unsigned char buf[100];
	unsigned char readbuf[100];
	int use_ssl = 0;
	
	if (argc < 2)
		usage();
	
	char* topic = argv[1];

	/* Initialize global variables for repeated execution */

	toStop = 0;

	opts.clientid    = (char*)"stdout-subscriber";
	opts.nodelimiter = 0;
	opts.delimiter   = (char*)"\n";
	opts.qos         = QOS2;
	opts.username    = NULL;
	opts.password    = NULL;
	opts.host        = (char*)"localhost";
	opts.port        = 1883;
	opts.showtopics  = 0;
	opts.cafile      = NULL;
	opts.certfile    = NULL;
	opts.keyfile     = NULL;
	opts.message     = NULL;

	if (strchr(topic, '#') || strchr(topic, '+'))
		opts.showtopics = 1;
	if (opts.showtopics)
		printf("topic is %s\n", topic);

	getopts(argc, argv);	

	if (app_connect_to_lte())
		return ERROR;

	MQTTSocket n;
	MQTTClient c;

	signal(SIGINT, cfinish);
	signal(SIGTERM, cfinish);

	/* Use MQTT over SSL/TLS if set CA certificate */

	if (opts.cafile != NULL)
	{
		use_ssl = 1;
		n.pRootCALocation = opts.cafile;
		n.pDeviceCertLocation = opts.certfile;
		n.pDevicePrivateKeyLocation = opts.keyfile;
	}
	MQTTSocketInit(&n, use_ssl);
	MQTTSocketConnect(&n, opts.host, opts.port);
	MQTTClientInit(&c, &n, 10000, buf, 100, readbuf, 100);
 
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;       
	data.willFlag = 0;
	data.MQTTVersion = 3;
	data.clientID.cstring = opts.clientid;
	data.username.cstring = opts.username;
	data.password.cstring = opts.password;

	data.keepAliveInterval = 10;
	data.cleansession = 1;
	printf("Connecting to %s %d\n", opts.host, opts.port);
	
	rc = MQTTConnect(&c, &data);
	printf("Connected %d\n", rc);

	if (opts.message)
	{
		/* If there is any message, publish the message on a topic. */

		MQTTMessage pubmsg;

		memset(&pubmsg, 0, sizeof(pubmsg));
		pubmsg.qos = opts.qos;
		pubmsg.retained = 0;
		pubmsg.dup = 0;
		pubmsg.payload = opts.message;
		pubmsg.payloadlen = strlen(opts.message);

		printf("Publishing to %s\n", topic);
		rc = MQTTPublish(&c, topic, &pubmsg);
		printf("Published %d\n", rc);
	}
	else
	{
		printf("Subscribing to %s\n", topic);
		rc = MQTTSubscribe(&c, topic, opts.qos, messageArrived);
		printf("Subscribed %d\n", rc);

		while (!toStop)
		{
			MQTTYield(&c, 1000);
		}

		printf("Stopping\n");
	}

	MQTTDisconnect(&c);
	MQTTSocketDisconnect(&n);
	MQTTSocketFin(&n);

	app_disconnect_from_lte();

	return 0;
}


