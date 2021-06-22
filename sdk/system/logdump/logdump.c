/****************************************************************************
 * system/logdump/logdump.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

#include <sys/stat.h>
#include <fcntl.h>

#include <arch/chip/backuplog.h>
#include <arch/chip/crashdump.h>

#include "logdump.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void logdump_sub(char *name, void *addr, size_t size)
{
  /* Call the dedicated dump routine */

  if (0 == strncmp(name, "crash", 5))
    {
      logdump_crash(addr, size);
    }
  else
    {
      ;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int logdump_main(int argc, char **argv)
#endif
{
  FILE *fp;
  char logfile[64];
  void *addr = NULL;
  size_t size = 0;
  char *name;

  if (argc < 2)
    {
      printf("Usage: logdump <name>\n");
      return ERROR;
    }

  name = argv[1];

  /* Dump from memory */

  up_backuplog_region(name, &addr, &size);

  if ((NULL == addr) || (0 == size))
    {
      printf("No such entry: %s\n", name);
    }
  else
    {
      printf("=== Dump %s at 0x%08lx (%d bytes)\n", name, (uint32_t)addr, size);
      logdump_sub(name, addr, size);
    }

  /* Dump from file */

  snprintf(logfile, 64, CONFIG_SYSTEM_LOGSAVE_MOUNTPOINT"/%s.log", name);

  fp = fopen(logfile, "rb");
  if (fp == NULL)
    {
      printf("No such file: %s\n", logfile);
    }
  else
    {
      struct stat filestat;
      if (stat(logfile, &filestat) != 0)
        {
          fclose(fp);
          return ERROR;
        }

      size = filestat.st_size;

      addr = malloc(size);
      if (!addr)
        {
          printf("Not enough memory\n");
          fclose(fp);
          return ERROR;
        }

      fseek(fp, 0, SEEK_SET);
      fread(addr, size, 1, fp);

      printf("=== Dump %s (%d bytes)\n", logfile, size);
      logdump_sub(name, addr, size);

      free(addr);
      fclose(fp);

      return OK;
    }

  return ERROR;
}
