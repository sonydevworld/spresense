/****************************************************************************
 * lte_lwm2m/system_device.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <sys/boardctl.h>
#include <sys/utsname.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRV_MANUFACTURER      "SONY SPRESENSE"
#define PRV_MODEL_NUMBER      "CXD5602PWBMAIN1"
#define PRV_MEMORY_TOTAL      (1536 * 1024 / 1000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_utc_offset_sec = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const char *get_manufacture(void)
{
  return PRV_MANUFACTURER;
}

const char *get_model_number(void)
{
  return PRV_MODEL_NUMBER;
}

const char *get_serial_number(void)
{
  uint8_t uid[CONFIG_BOARDCTL_UNIQUEID_SIZE];
  static char boardid[CONFIG_BOARDCTL_UNIQUEID_SIZE * 2 + 1];

  memset(boardid, 0, sizeof(boardid));
  boardctl(BOARDIOC_UNIQUEID, (uintptr_t)uid);
  snprintf(boardid, sizeof(boardid), "%02X%02X%02X%02X%02X",
           uid[0], uid[1], uid[2], uid[3], uid[4]);

  return boardid;
}

const char *get_firmware_version(void)
{
  static struct utsname name;

  uname(&name);

  return name.version;
}

void device_reboot(void)
{
  boardctl(BOARDIOC_RESET, EXIT_SUCCESS);
  exit(1);
}

int get_free_memory(void)
{
  struct mallinfo info;

  info = mallinfo();

  return info.fordblks / 1000;
}

int get_total_memory(void)
{
  return PRV_MEMORY_TOTAL;
}

void set_utc_offset_sec(char * timeoffset)
{
  int hour = 0;
  int min = 0;

  if (3 == strlen(timeoffset))
    {
      sscanf(timeoffset, "%d", &hour);
    }
  else
    {
      sscanf(timeoffset, "%d:%d", &hour, &min);
    }

  g_utc_offset_sec = (hour * 60 + min) * 60;

  return;
}

int get_utc_offset_sec(void)
{
  return g_utc_offset_sec;
}
