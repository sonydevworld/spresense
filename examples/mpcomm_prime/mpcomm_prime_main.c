/****************************************************************************
 * mpcomm_prime/mpcomm_prime_main.c
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
#include <stdlib.h>

#include <sys/stat.h>
#include <sys/mount.h>

#include <nuttx/drivers/ramdisk.h>

#include <mpcomm/supervisor.h>

#include "worker/prime/prime.h"

#ifdef CONFIG_FS_ROMFS
#include "worker/romfs.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_ROMFS
#define SECTORSIZE 512
#define NSECTORS(b) (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define MOUNTPT "/romfs"
#endif

#ifndef MOUNTPT
#define MOUNTPT "/mnt/sd0/BIN"
#endif

#define PRIME_MIN 0
#define PRIME_MAX 1500

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int prime_min;
  int prime_max;
} my_setting_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int romfs_init(void)
{
  int ret = OK;

#ifdef CONFIG_FS_ROMFS
  struct stat buf;

  ret = stat(MOUNTPT, &buf);
  if (ret < 0)
    {
      printf("Registering romdisk at /dev/ram0\n");
      ret = romdisk_register(0, (FAR uint8_t *)romfs_img,
                            NSECTORS(romfs_img_len), SECTORSIZE);
      if (ret < 0)
        {
          printf("ERROR: romdisk_register failed: %d\n", ret);
          exit(1);
        }

      printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
            MOUNTPT, "/dev/ram0");

      ret = mount("/dev/ram0", MOUNTPT, "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          printf("ERROR: mount(%s,%s,romfs) failed: %d\n",
                "/dev/ram0", MOUNTPT, errno);
        }
    }
#endif

  return ret;
}

static void parse_args(int argc, char *argv[], my_setting_t *setting)
{
  int i = 1;

  if (argc == 2)
    {
      setting->prime_min = PRIME_MIN;
      setting->prime_max = atoi(argv[i]);
    }
  else if (argc == 3)
    {
      setting->prime_min = atoi(argv[i++]);
      setting->prime_max = atoi(argv[i]);
    }
  else
    {
      setting->prime_min = PRIME_MIN;
      setting->prime_max = PRIME_MAX;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpcomm_prime_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  my_setting_t setting = {0};
  mpcomm_supervisor_context_t *ctx = NULL;

  parse_args(argc, argv, &setting);

  ret = romfs_init();
  if (ret)
    {
      printf("romfs_init failed due to %d\n", ret);
      return ret;
    }

  ret = mpcomm_supervisor_init(&ctx, MOUNTPT"/PRIME", 4);
  if (ret)
    {
      printf("mpcomm_supervisor_init failed due to %d\n", ret);
      return ret;
    }

  prime_data_t main_task;
  main_task.start = setting.prime_min;
  main_task.end = setting.prime_max;

  ret = mpcomm_supervisor_send_controller(ctx, &main_task);
  if (ret)
    {
      printf("mpcomm_supervisor_send_controller failed due to %d\n", ret);
      return ret;
    }

  ret = mpcomm_supervisor_wait_controller_done(ctx, NULL);
  if (ret)
    {
      printf("mpcomm_supervisor_wait_controller_done failed due to %d\n",
             ret);
      return ret;
    }

  printf("Found %ld primes\n", main_task.result);

  ret = mpcomm_supervisor_deinit(ctx);
  if (ret)
    {
      printf("mpcomm_supervisor_deinit failed due to %d\n", ret);
      return ret;
    }

  return ret;
}
