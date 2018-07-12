/****************************************************************************
 * modules/asmp/worker/arch/sysctl.h
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
 * @file sysctl.h
 */

#ifndef __ASMP_WORKER_ARCH_SYSCTL_H
#define __ASMP_WORKER_ARCH_SYSCTL_H

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
 * System control commands
 *
 * XXX: Command assignments are experimental
 */

#define SYSCTL_INVALID 0

#define SYSCTL_MAP       1
#define SYSCTL_UNMAP     2
#define SYSCTL_GETFWSIZE 3
#define SYSCTL_LOADFW    4
#define SYSCTL_UNLOADFW  5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Arguments for SYSCTL_MAP */

typedef struct sysctl_map_s
{
  int cpuid;
  uint32_t virt;
  uint32_t phys;
  uint32_t size;
} sysctl_map_t;

/* Arguments for SYSCTL_UNMAP */

typedef struct sysctl_unmap_s
{
  int cpuid;
  uint32_t virt;
  uint32_t size;
} sysctl_unmap_t;

/* Arguments for SYSCTL_GETFWSIZE */

typedef struct sysctl_getfwsize_s
{
  char filename[32];
} sysctl_getfwsize_t;

/* Arguments for SYSCTL_LOADFW */

typedef struct sysctl_loadfw_s
{
  int cpuid;
  uint32_t addr;
  char filename[32];
} sysctl_loadfw_t;

/* Arguments for SYSCTL_UNLOADFW */

typedef struct sysctl_unloadfw_s
{
  int cpuid;
} sysctl_unloadfw_t;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/**
 * Send system control command
 */

int sysctl(uint8_t id, uint32_t data);

#endif
