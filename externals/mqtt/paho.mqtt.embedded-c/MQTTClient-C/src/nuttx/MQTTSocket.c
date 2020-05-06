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
 *    Ian Craggs - convert to FreeRTOS
 *******************************************************************************/

#include "MQTTSocket.h"
#include "errno.h"

/* for NUTTX */
extern int getaddrinfo(const char *nodename, const char *servname,
                const struct addrinfo *hints, struct addrinfo **res);
extern void freeaddrinfo(struct addrinfo *res);

int ThreadStart(Thread* thread, void (*fn)(void*), void* arg)
{
	int rc = 0;

	if( (rc=pthread_create( &thread->thid, NULL, (pthread_startroutine_t)fn, arg )) == 0 )
	{
		pthread_detach(thread->thid);
		return 0;
	}
	else
	{
		return rc;
	}
}


void MutexInit(Mutex* mutex)
{
	sem_init( (sem_t *)&(mutex->sem), 0, 1 );
}

int MutexLock(Mutex* mutex)
{
	return sem_wait((sem_t *)&(mutex->sem));
}

int MutexUnlock(Mutex* mutex)
{
	return sem_post((sem_t *)&(mutex->sem));
}

void TimerCountdownMS(Timer* timer, unsigned int timeout_ms)
{
	timer->TimeOut = timeout_ms;
	clock_gettime( CLOCK_MONOTONIC, &timer->start_time); /* Record the time at which this function was entered. */
}

void TimerCountdown(Timer* timer, unsigned int timeout) 
{
	TimerCountdownMS(timer, timeout * 1000);
}

int TimerLeftMS(Timer* timer) 
{
	struct timespec current_time;
	struct timespec diff_time;
	int difftime_msec = 0;
	clock_gettime( CLOCK_MONOTONIC, &current_time);

	diff_time.tv_sec  = current_time.tv_sec - timer->start_time.tv_sec;
	diff_time.tv_nsec = current_time.tv_nsec - timer->start_time.tv_nsec;

	if(diff_time.tv_nsec < 0)
	{
		diff_time.tv_sec -= 1;
		diff_time.tv_nsec += 1000*1000*1000;
	}

	difftime_msec = diff_time.tv_sec*1000 + diff_time.tv_nsec/(1000*1000);

	if(difftime_msec < timer->TimeOut)
	{
		return timer->TimeOut - difftime_msec;
	}
	else
	{
		return 0;
	}
}

char TimerIsExpired(Timer* timer)
{
	if( TimerLeftMS(timer) )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void TimerInit(Timer* timer)
{
	timer->TimeOut = 0;
        memset(&timer, '\0', sizeof(timer));
}

int MQTTSocket_read(MQTTSocket* n, unsigned char* buffer, int len, int timeout_ms)
{
	Timer timer;
	struct timeval tv;

	int recvLen = 0;

	TimerInit(&timer);
	TimerCountdownMS( &timer, timeout_ms );
	do
	{
		int rc = 0;

		tv.tv_sec = timeout_ms / 1000;  /* 1 second Timeout */
		tv.tv_usec = (timeout_ms % 1000) * 1000;  
		setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		rc = recv(n->my_socket, buffer + recvLen, len - recvLen, 0);
		if (rc > 0)
			recvLen += rc;
		else if ((rc < 0) && (errno != EAGAIN))
		{
			recvLen = rc;
			break;
		}
	} while ((recvLen < len) && (TimerIsExpired(&timer) == 0));

	return recvLen;
}


int MQTTSocket_write(MQTTSocket* n, unsigned char* buffer, int len, int timeout_ms)
{
	Timer timer;
	struct timeval tv;

	int sentLen = 0;

	TimerInit(&timer);
	TimerCountdownMS( &timer, timeout_ms );
	do
	{
		int rc = 0;

		tv.tv_sec = timeout_ms / 1000;  /* 1 second Timeout */
		tv.tv_usec = (timeout_ms % 1000) * 1000;  

		setsockopt(n->my_socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
		rc = send(n->my_socket, buffer + sentLen, len - sentLen, 0);
		if (rc > 0)
		{
			sentLen += rc;
		}
		else if (rc < 0)
		{
			sentLen = rc;
			break;
		}
	} while ((sentLen < len) && (TimerIsExpired(&timer) == 0));

	return sentLen;
}


void MQTTSocketDisconnect(MQTTSocket* n)
{
	if (n->use_ssl) {
		MQTTSslDisconnect(n);
	}
	else {
		close(n->my_socket);
	}
}


void MQTTSocketInit(MQTTSocket* n, int use_ssl)
{
	n->my_socket = 0;
	n->use_ssl = use_ssl;
	if(use_ssl == 0) {
		n->mqttread = MQTTSocket_read;
		n->mqttwrite = MQTTSocket_write;
		n->disconnect = MQTTSocketDisconnect;
	} else {
		MQTTSslInit(n);
	}
}


void MQTTSocketFin(MQTTSocket* n)
{
	if (n->use_ssl) {
		MQTTSslFin(n);
	}
}


int MQTTSocketConnect(MQTTSocket* n, char* addr, int port)
{
	int* sock = &n->my_socket;
	struct sockaddr_in address;
	struct sockaddr_in6 address6;
	int rc = -1;
	struct addrinfo *result = NULL;
	struct addrinfo hints = {0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};
	static struct timeval tv;

	*sock = -1;
	if (addr[0] == '[')
	  ++addr;

	if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0)
	{
		struct addrinfo* res = result;

		/* prefer ip4 addresses */
		while (res)
		{
			if (res->ai_family == AF_INET)
			{
				result = res;
				break;
			}
			res = res->ai_next;
		}

		if (result->ai_family == AF_INET6)
		{
			address6.sin6_port = htons(port);
			address6.sin6_family = AF_INET6;
			address6.sin6_addr = ((struct sockaddr_in6*)(result->ai_addr))->sin6_addr;
		}
		else
		if (result->ai_family == AF_INET)
		{
			address.sin_port = htons(port);
			address.sin_family = AF_INET;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;
	}

	if (rc == 0)
	{
		*sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
		if (*sock != -1)
		{
#if defined(NOSIGPIPE)
			int opt = 1;

			if (setsockopt(*sock, SOL_SOCKET, SO_NOSIGPIPE, (void*)&opt, sizeof(opt)) != 0)
				Log(TRACE_MIN, -1, "Could not set SO_NOSIGPIPE for socket %d", *sock);
#endif
			if (n->use_ssl){
				if (result->ai_family == AF_INET)
					rc = connect(*sock, (struct sockaddr*)&address, sizeof(address));
				else
				if (result->ai_family == AF_INET6)
					rc = connect(*sock, (struct sockaddr*)&address6, sizeof(address6));
				else
					rc = -1;
				if (rc == 0){
					n->mqtt_net_context.fd = *sock;
					rc = MQTTSslConnect(n, addr);
				}

			}
			else {
				if (result->ai_family == AF_INET)
					rc = connect(*sock, (struct sockaddr*)&address, sizeof(address));
				else
				if (result->ai_family == AF_INET6)
					rc = connect(*sock, (struct sockaddr*)&address6, sizeof(address6));
				else
					rc = -1;
			}
		}
		else {
			rc = -1;
		}

		freeaddrinfo(result);
	}
	if (n->my_socket == -1 || rc == -1)
		return rc;

	tv.tv_sec = 1;  /* 1 second Timeout */
	tv.tv_usec = 0;
	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	return n->my_socket;
}

