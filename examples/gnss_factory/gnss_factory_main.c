/****************************************************************************
 * gnss_factory/gnss_factory_main.c
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
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <arch/chip/gnss.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SLEEP_TIME      1
#define TEST_COUNT      ((60/SLEEP_TIME)*3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: gnss_factory_main()
 *
 * Description:
 *   Execute FACTORY test.
 *   svID can be specified from the console.
 *
 * Input Parameters:
 *   argv[1] - svID.
 *   If argv[1] is null, use the default value specified by kconfig.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int      fd;
  int      ret;
  uint32_t count;
  struct cxd56_gnss_test_info_s setparam =
    { CONFIG_EXAMPLES_GNSS_FACTORY_SVID, 0, 0, 0 };
  struct cxd56_gnss_test_result_s get;

  /* Program start */

  printf("Hello, FACTORY SAMPLE!!\n");

  /**< Specify satellite by svID  */

  if (argc >= 2)
    {
      setparam.satellite = atoi(argv[1]);
    }

  if (setparam.satellite == 0)
    {
      printf("invalid svid!! %d\n", setparam.satellite);
      return -EINVAL;
    }

  /* Get file descriptor to control GNSS. */

  fd = open("/dev/gps", O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      return -ENODEV;
    }

  /* Test start */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_FACTORY_START_TEST,
              (unsigned long)&setparam);
  if (ret < 0)
    {
      printf("start error %d\n", ret);
    }
  else
    {
      printf("start test(svid %d)\n", setparam.satellite);

      for (count = TEST_COUNT; count > 0; count--)
        {
          sleep(SLEEP_TIME);

          /* Get test result */

          ret = ioctl(fd, CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT,
                      (unsigned long)&get);
          if (ret == OK)
            {
              /* Print test result */

              printf("  cn %d, doppler %ld\n", (int)(get.cn * 1000000),
                     (int64_t)(get.doppler * 1000000));
              break;
            }
          else
            {
              printf("  waiting...\n");
            }
        }

      /* Test stop */

      ioctl(fd, CXD56_GNSS_IOCTL_FACTORY_STOP_TEST, 0);
      if (ret == OK)
        {
          printf("test OK\n");
        }
      else
        {
          printf("test ERROR\n");
        }
    }

  /* Release GNSS file descriptor. */

  close(fd);

  printf("End of FACTORY Sample:%d\n", ret);

  return ret;
}

/****************************************************************************
 * Name: gnss_factory_test()
 *
 * Description:
 *   Called to run factory test directly from entrypoint.
 *
 * Input Parameters:
 *   none.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int gnss_factory_test(int argc, char *argv[])
{
  /* Initialize  */

  int ret = boardctl(BOARDIOC_INIT, 0);

  if (ret == OK)
    {
      /* Call factory test */

      ret = gnss_factory_main(argc, argv);
    }

  return ret;
}
