/****************************************************************************
 * fwupdate/fwupdate_usbcdc_zmodem.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include <sys/stat.h>
#include <fcntl.h>

#include <sys/boardctl.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>

#include "system/zmodem.h"
#include "fwuputils/fwup_client.h"

#include "fwupdate_local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int receive_zmodem(void)
{
  ZMRHANDLE handle;
  FAR const char *devname = CONFIG_EXAMPLES_FWUPDATE_USBCDC_DEVNAME;
  int exitcode = EXIT_FAILURE;
  int ret;
  int fd;
  int retry = 0;

  /* Open the device for read/write access */

  do
    {
      fd = open(devname, O_RDWR);
      if (fd < 0)
        {
          int errcode = errno;

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              retry++;
              sleep(1);
            }
          else
            {
              goto errout;
            }
        }
    }
  while (fd < 0);

  /* Get the Zmodem handle */

  handle = zmr_initialize(fd);
  if (!handle)
    {
      fprintf(stderr, "ERROR: Failed to get Zmodem handle\n");
      goto errout_with_device;
    }

  /* And begin reception of files */

  ret = zmr_receive(handle, CONFIG_EXAMPLES_FWUPDATE_DOWNLOAD_DIR);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: File reception failed: %d\n", ret);
      goto errout_with_zmodem;
    }

  exitcode = EXIT_SUCCESS;

errout_with_zmodem:
  (void)zmr_release(handle);
errout_with_device:
  (void)close(fd);
errout:
  return exitcode;
}

static int usb_cdcacm_setup(void)
{
  struct boardioc_usbdev_ctrl_s ctrl;
  void *handle;
  int ret;

  /* Initialize the USB CDC/ACM serial driver */

  ctrl.usbdev   = BOARDIOC_USBDEV_CDCACM;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = CONFIG_EXAMPLES_FWUPDATE_USBCDC_DEVMINOR;
  ctrl.handle   = &handle;

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  if (ret < 0)
    {
      printf("Already ");
    }

  printf("Registered the CDC/ACM serial driver\n");
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fwupdate_usbcdc_zmodem(void)
{
  int ret = OK;
  char path[32];

  /*
   * USB CDC/ACM setup
   */

  usb_cdcacm_setup();

  /*
   * Use zmodem protocol
   */

  printf("Wait until zmodem data come...\n");
  ret = receive_zmodem();
  if (ret)
    {
      printf("ERROR: receive_zmodem: ret=%d\n", ret);
      return ret;
    }

  /*
   * FW Update Sequence execution
   */

  snprintf(path, sizeof(path), "%s/package.bin",
           CONFIG_EXAMPLES_FWUPDATE_DOWNLOAD_DIR);
  ret = fwupdate_package(path);

  return ret;
}

