/****************************************************************************
 * fwupdate/fwupdate_main.c
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

#include <fwuputils/fwup_client.h>

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

static void show_usage(FAR const char *progname)
{
  printf("\nUsage: %s [-f <filename>]... [-p <pkgname>] [-h]\n\n",
         progname);
  printf("Description:\n");
  printf(" FW Update operation\n");
  printf("Options:\n");
  printf(" -f <filename>: update a file.\n");
  printf(" -p <pkgname> : update a package.\n");
#ifdef CONFIG_EXAMPLES_FWUPDATE_USBCDC_ZMODEM
  printf(" -z : update a package via USB CDC/ACM Zmodem.\n");
#endif
  printf(" -h: Show this message\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * fwupdate_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  int opt = 0;
  char *farg = NULL;
  char *parg = NULL;
#ifdef CONFIG_EXAMPLES_FWUPDATE_USBCDC_ZMODEM
  int zopt = 0;
#endif

  printf("FW Update Example!!\n");

  optind = -1;
  while ((opt = getopt(argc, argv, ":f:p:zh")) != -1)
    {
      switch (opt)
        {
        case 'f':
          farg = optarg;
          break;
        case 'p':
          parg = optarg;
          break;
        case 'z':
#ifdef CONFIG_EXAMPLES_FWUPDATE_USBCDC_ZMODEM
          zopt = 1;
#endif
          break;
        case 'h':
        case ':':
        case '?':
          show_usage(argv[0]);
          return EXIT_FAILURE;
        }
    }

  printf("Free space %u bytes\n", fwup_client_getfreespace());

  if (farg)
    {
      /* Update a FW via FileIO */

      ret = fwupdate_file(farg);
    }
  else if (parg)
    {
      /* Update a FW package via FileIO */

      ret = fwupdate_package(parg);
    }
#ifdef CONFIG_EXAMPLES_FWUPDATE_USBCDC_ZMODEM
  else if (zopt)
    {
      ret = fwupdate_usbcdc_zmodem();
    }
#endif
  else
    {
      show_usage(argv[0]);
    }

  return ret;
}
