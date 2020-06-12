/****************************************************************************
 * modules/dnnrt/src-mp/runtime/runtime_client.h
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#ifndef RUNTIME_CLIENT_H
#  define RUNTIME_CLIENT_H

#  include <nuttx/config.h>
#  include <errno.h>
#  include <debug.h>
#  include <nnablart/functions.h>
#  include <nnablart/runtime.h>
#  include <dnnrt/runtime.h>
#  ifdef __arm__
#    include <asmp/asmp.h>
#    include <asmp/mpmq.h>
#    include <asmp/mptask.h>
#  endif

#  ifdef __cplusplus
extern "C"
{
#  endif

#  define dnn_info(x...) _info(x)
#  define dnn_err(x...) _err(x)

#  define DNN_CHECK_NULL_RET(b, r)                                               \
  do {                                                                         \
    if ((b) == NULL) {                                                         \
      dnn_err("Null-check failed. %s()@%s:L%d\n", __FUNCTION__, __FILE__,      \
              __LINE__);                                                       \
      return r;                                                                \
    }                                                                          \
  } while (0)

#  define MP_CPUID_MAX (8)
#  define MAX_MP_CORE (5)       /* only 5 cores available as CPUAFMASK=0x3eu */
#  define MP_MQ_KEY (1)

/* all messages must be acknowledged by set ACK to 1 */
#  define MP_MSG_INIT (0x01)    /* L -> M/S: initialize master/slave */
#  define MP_MSG_CALL_API (0x02)        /* L -> M: call a dnnrt API */
#  define MP_MSG_MALLOC (0x03)  /* M -> L: malloc from kernel heap */
#  define MP_MSG_FREE (0x04)    /* M -> L: free memory to kernel heap */
#  define MP_MSG_REALLOC (0x05) /* M -> L: realloc memory on kernel heap */
#  define MP_MSG_QUIT (0x06)    /* L -> M/S: notify MP program will be
                                 * terminated */
#  define MP_MSG_ACK_BIT (0x40)

#  define ALIGN(x, s)  ((void*)((unsigned int)(((void*)(x)) + ((s) - 1)) & ~((s) - 1)))

#  define _S(a) (sizeof(a) / sizeof(a[0]))

  /* paramter of MP_MSG_MALLOC */
  typedef struct mp_alloc_memory
  {
    size_t bsize;               /* request size of memory */
    void *addr;                 /* returned address */
  } mp_alloc_memory_t;

  /* parameter of MP_MSG_FREE */
  typedef struct mp_free_memory
  {
    void *addr;
  } mp_free_memory_t;

  typedef struct mp_message_buffer
  {
    mp_alloc_memory_t allocmem;
    mp_free_memory_t freemem;
  } mp_message_buffer_t;

  /* paramter of MP_MSG_INIT */
  typedef struct mp_task_init
  {
    cpu_set_t cpu_set;
    void *load_addr;            /* start physical address of each ASMP bank */
    mp_message_buffer_t *msg_buf;       /* buffers for messages sent by master */
    int ret;                    /* return code */
  } mp_task_init_t;

  typedef enum
  {
    DNNRT_API_INIT,
    DNNRT_API_FINI,
    DNNRT_API_RT_INIT,
    DNNRT_API_RT_FINI,
    DNNRT_API_RT_FOWARD,
    DNNRT_API_RT_INPUT_NUM,
    DNNRT_API_RT_INPUT_SIZE,
    DNNRT_API_RT_INPUT_NDIM,
    DNNRT_API_RT_INPUT_SHAPE,
    DNNRT_API_RT_INPUT_VARIABLE,
    DNNRT_API_RT_OUTPUT_NUM,
    DNNRT_API_RT_OUTPUT_SIZE,
    DNNRT_API_RT_OUTPUT_NDIM,
    DNNRT_API_RT_OUTPUT_SHAPE,
    DNNRT_API_RT_OUTPUT_BUFFER,
    DNNRT_API_RT_OUTPUT_VARIABLE,
    DNNRT_API_ASMP_MALLINFO,
  } dnn_api_id_t;

  /* paramter of MP_MSG_CALL_API */
  typedef struct mp_api_call
  {
    dnn_api_id_t api;           /* dnnrt API ID */
    int arg[3];                 /* input arguments */
    int ret;                    /* return code */
  } mp_api_call_t;

  int dnn_mpmgr_load(int slave_num);    /* load MP image */
  int dnn_mpmgr_unload(void);   /* unload MP image */
  int dnn_mpmgr_call_api(int api, int num_args, ...);   /* send API call *
                                                         * request */

#  ifdef __cplusplus
}
#  endif

#endif                          /* RUNTIME_CLIENT_H */
