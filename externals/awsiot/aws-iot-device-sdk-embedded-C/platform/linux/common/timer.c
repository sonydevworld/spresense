/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Copyright 2019 Sony Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**
 * @file timer.c
 * @brief Linux implementation of the timer interface.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "timer_platform.h"

bool has_timer_expired(Timer *timer) {
	if( left_ms(timer) )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void countdown_ms(Timer *timer, uint32_t timeout) {
	timer->TimeOut = timeout;
	clock_gettime( CLOCK_MONOTONIC, &timer->start_time); /* Record the time at which this function was entered. */
}

uint32_t left_ms(Timer *timer) {
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

void countdown_sec(Timer *timer, uint32_t timeout) {
	timer->TimeOut = timeout * 1000;
	clock_gettime( CLOCK_MONOTONIC, &timer->start_time); /* Record the time at which this function was entered. */
}

void init_timer(Timer *timer) {
	memset(timer, 0, sizeof(Timer));
}

#ifdef __cplusplus
}
#endif
