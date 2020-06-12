/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 * Copyright 2019 Sony Corporation
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *******************************************************************************/

#if !defined(MQTTSocket_H)
#define MQTTSocket_H

#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "sys/socket.h"
#include "netdb.h"
#include "arpa/inet.h"

#include "mbedtls/config.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/net.h"
#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"

typedef struct Timer Timer;

/**
 * definition of the Timer struct. Platform specific
 */

struct Timer {
	struct timespec start_time;
	unsigned int TimeOut;
};

/**
 * \addtogroup MQTT
 * \{
 */

/**
 * \defgroup Mqtt_socket_if MQTT socket interface
 * MQTT socket interface
 * \{
 */
/**
 * \brief           MQTT socket strutcture
 */
typedef struct MQTTSocket  MQTTSocket ;
struct MQTTSocket 
{
	int my_socket;
	int use_ssl;
	mbedtls_ssl_context *mqtt_context;
	mbedtls_net_context mqtt_net_context;
	char *pRootCALocation;					/**< The Root CA file path in using mbedTLS (full file, not path) */
	char *pDeviceCertLocation;				/**< The device identity certificate file path in using mbedTLS (full file, not path) */
	char *pDevicePrivateKeyLocation;		/**< The device private key file in using mbedTLS (full file, not path) */
	int (*mqttread) (MQTTSocket *, unsigned char*, int, int);
	int (*mqttwrite) (MQTTSocket *, unsigned char*, int, int);
	void (*disconnect) (MQTTSocket *);
};


void TimerInit(Timer*);
char TimerIsExpired(Timer*);
void TimerCountdownMS(Timer*, unsigned int);
void TimerCountdown(Timer*, unsigned int);
int TimerLeftMS(Timer*);

typedef struct Mutex
{
	unsigned char sem;
} Mutex;

void MutexInit(Mutex*);
int MutexLock(Mutex*);
int MutexUnlock(Mutex*);

typedef struct Thread
{
	pthread_t thid;
} Thread;

int ThreadStart(Thread* thread, void (*fn)(void*), void* arg);
int MQTTSocket_read(MQTTSocket*, unsigned char*, int, int);
int MQTTSocket_write(MQTTSocket*, unsigned char*, int, int);


/**
 * Initialize  MQTT socket
 * @param socket  MQTT socket
 * @param use_tls 0:not use mbedTLS / 1:use mbedTLS
 */
void MQTTSocketInit(MQTTSocket *socket, int use_tls);

/**
 * Finalize  MQTT socket
 * @param socket  MQTT socket
 */
void MQTTSocketFin(MQTTSocket *socket);

/**
 *  Connect MQTT socket 
 * @param socket  MQTT socket to be used
 * @param broker  MQTT broker address/hostname to be connected
 * @param port    port that MQTT broker uses
 */
int MQTTSocketConnect(MQTTSocket *socket, char *broker, int port);

/**
 *  Connect MQTT socket 
 * @param socket  MQTT socket to be disconnected
 */
void MQTTSocketDisconnect(MQTTSocket *socket);
/* \} */


/* \} */

void MQTTSslInit(MQTTSocket* n);
void MQTTSslFin(MQTTSocket* n);
int MQTTSslConnect(MQTTSocket* n, char* hostname);
void MQTTSslDisconnect(MQTTSocket* n);
int MQTTSslWrite(MQTTSocket* n, unsigned char *buf, int len, int);
int MQTTSslRead(MQTTSocket* n, unsigned char *buf, int len, int);
#endif
