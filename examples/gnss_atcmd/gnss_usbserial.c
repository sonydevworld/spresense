/****************************************************************************
 * gnss_atcmd/gnss_usbserial.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_USB

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#include "gnss_usbserial.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRACE_INIT_BITS       (0)
#define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)
#define TRACE_CLASS_BITS      (0)
#define TRACE_TRANSFER_BITS   (0)
#define TRACE_CONTROLLER_BITS (0)
#define TRACE_INTERRUPT_BITS  (0)
#define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|TRACE_CLASS_BITS|\
                                 TRACE_TRANSFER_BITS|TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)
#ifdef CONFIG_CDCACM
#  define USBSER_DEVNAME "/dev/ttyACM0"
#else
#  define USBSER_DEVNAME "/dev/ttyUSB0"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define dumptrace()

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * gnss_usbserial_open
 ****************************************************************************/

 /* this code is refered of usbserial example */
 /* gnss_atcmd_open_usbserial : usbserial initialize */
 /* fds[0]: write file(bulk out), fds[1]: read file(bulk in) */

int gnss_usbserial_open(FAR int *fds)
{
  struct boardioc_usbdev_ctrl_s ctrl;
  FAR void *handle;
  int *wfd;
  int *rfd;
  int ret;

  wfd = &fds[GNSS_SERIAL_WRITE_FD];
  rfd = &fds[GNSS_SERIAL_READ_FD];

  /* Initialize the USB serial driver */

  printf("%s: Registering USB serial driver\n", __func__);

#ifdef CONFIG_CDCACM

  ctrl.usbdev   = BOARDIOC_USBDEV_CDCACM;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &handle;

#else

  ctrl.usbdev   = BOARDIOC_USBDEV_PL2303;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &handle;

#endif

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  if (ret < 0)
    {
      printf("%s: WARN: Maybe already registered the USB serial device: %d\n",
             __func__, -ret);
    }
  else
    {
      printf("%s: Successfully registered the serial driver\n", __func__);
    }

  /* Then, in any event, configure trace data collection as configured */

  usbtrace_enable(TRACE_BITSET);

  /* Open the USB serial device for writing (blocking) */

  do
    {
      printf("%s: Opening USB serial driver\n", __func__);
      *wfd = open(USBSER_DEVNAME, O_WRONLY);
      if (*wfd < 0)
        {
          int errcode = errno;
          printf("%s: ERROR: Failed to open " USBSER_DEVNAME
                 " for writing: %d\n", __func__, errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              printf("%s:        Not connected. Wait and try again.\n",
                     __func__);
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              printf("%s:        Aborting\n", __func__);
              return errcode;
            }
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }

  while (*wfd < 0);

  /* Open the USB serial device for reading (non-blocking) */

  *rfd = open(USBSER_DEVNAME, O_RDONLY|O_NONBLOCK);
  if (*rfd < 0)
    {
      printf("%s: ERROR: Failed to open " USBSER_DEVNAME
             " for reading: %d\n", __func__, errno);
      close(*wfd);
      return *rfd;
    }

  printf("%s: Successfully opened the serial driver\n", __func__);

  return 0;
}

int gnss_usbserial_close(FAR int *fds)
{
  int *wfd;
  int *rfd;

  wfd = &fds[GNSS_SERIAL_WRITE_FD];
  rfd = &fds[GNSS_SERIAL_READ_FD];

  close(*wfd);
  close(*rfd);

  printf("%s: Successfully closed the serial driver\n", __func__);

  return 0;
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_USB */
