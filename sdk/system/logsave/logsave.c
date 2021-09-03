/****************************************************************************
 * system/logsave/logsave.c
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
#include <errno.h>
#include <debug.h>

#include <sys/stat.h>
#include <fcntl.h>

#include <arch/chip/backuplog.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int logsave_main(int argc, char **argv)
#endif
{
  FILE *fp;
  char logfile[64];
  void *addr = NULL;
  size_t size = 0;
  char name[8 + 1];
  int ret;

  for (; ;)
    {
      /* Get a log entry */

      ret = up_backuplog_entry(name, &addr, &size);
      if ((-ENOENT == ret) || (NULL == addr) || (0 == size))
        {
          /* No entry and exit */

          break;
        }

      name[8] = '\0';
      snprintf(logfile, 64, CONFIG_SYSTEM_LOGSAVE_MOUNTPOINT"/%s.log", name);

      /* Save the logging data */

      printf("Save at 0x%08lx (%d bytes) into %s\n", (uint32_t)addr, size, logfile);

      fp = fopen(logfile, "ab");
      if (fp == NULL)
        {
          printf("open failed %s\n", logfile);
          break;
        }
      fwrite(addr, size, 1, fp);
      fclose(fp);

      /* Remove the entry */

      up_backuplog_free(name);
    }

  return OK;
}
