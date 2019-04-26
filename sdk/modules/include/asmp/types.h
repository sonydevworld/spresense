/****************************************************************************
 * modules/include/asmp/types.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

/**
 * @file types.h
 *
 * @defgroup asmp_types MP Framework specific types
 * @{
 */

#ifndef __INCLUDE_ASMP_TYPES_H
#define __INCLUDE_ASMP_TYPES_H

#include <sys/types.h>
#include <queue.h>

#define MPOBJTYPE_SHM   0x4588  /* MP shared memory object */
#define MPOBJTYPE_MQ    0x2dc6  /* MP message queue object */
#define MPOBJTYPE_MUTEX 0x0686  /* MP mutex object */

/*
 * MP object initializer
 */

#define mpobj_init(obj, t, k) do { (obj)->super.type = MPOBJTYPE_ ## t; (obj)->super.key = (k); } while (0)

/**
 * @typedef cpuid_t
 * CPU ID type
 */

typedef int16_t cpuid_t;

/**
 * @typedef mpobjtype_t
 * MP object type
 */

typedef int16_t mpobjtype_t;

/* This typedef is supplemental */

#ifndef CONFIG_SMP
typedef volatile uint8_t cpu_set_t;
#endif

/*
 * Super class for MP objects
 */

typedef struct mpobj
{
  mpobjtype_t type;
  key_t key;
} mpobj_t;

/** @} */

#endif
