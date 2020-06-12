/****************************************************************************
 * examples/multi_webcamera/multiwebcam_perf.h
 *
 *   Copyright 2019, 2020 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __EXAMPLE_MULTIWEBCAM_PERF_H__
#define __EXAMPLE_MULTIWEBCAM_PERF_H__

#include <nuttx/config.h>

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_PERF

#  include <stdio.h>
#  include <sys/time.h>
#  include <time.h>

#  define PREPARE_PERF_VARIABLES()  struct timeval now, then
#  define SET_INITIAL_TIME()        gettimeofday(&then, NULL)
#  define PRINT_DIFF_TIME(send_bytes) \
  do \
    { \
      int ___time_diff; \
      gettimeofday(&now, NULL); \
      ___time_diff = (((now.tv_sec - then.tv_sec) * 1000) \
                   + ((now.tv_usec - then.tv_usec) / 1000)); \
      then = now; \
      printf("[%d ms] %d bytes (rate: %10.3f bps)\n", \
              ___time_diff, \
              (send_bytes), \
              ((float)(send_bytes * 8 * 1000) / (float)___time_diff)  \
            ); \
    } \
  while(0)

#else

#  define PREPARE_PERF_VARIABLES()
#  define SET_INITIAL_TIME()
#  define PRINT_DIFF_TIME(b)

#endif

#endif  /* __EXAMPLE_MULTIWEBCAM_PERFORM_H__ */
