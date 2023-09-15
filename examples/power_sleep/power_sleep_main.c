/****************************************************************************
 * examples/power_sleep/power_sleep_main.c
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
#include <fcntl.h>
#include <sys/ioctl.h>

#include <arch/chip/pm.h>
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALARM_DEVPATH "/dev/rtc0"
#define ALARM_SIGNO 1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int fd;
  time_t seconds;
  struct rtc_setrelative_s setrel;

  if (argc != 2)
    {
      printf("Usage : %s <second>\n", argv[0]);
      return -1;
    }

  seconds = (time_t)atoi(argv[1]);

  /* Turn off the 3.3V power supply on the Spresense main board */

  board_power_control(POWER_LDO_EMMC, false);

  /* Turn off the power of SPI-Flash */

  board_flash_power_control(false);

  /* Turn off the power of TCXO.
   * NOTE: The CPU freqlock mechanism will not lock anything.
   */

  board_xtal_power_control(false);

  /* Clear all boot mask */

  up_pm_clr_bootmask((uint32_t)-1);

  /* Set boot mask RTC and RTC-Alarm0 */

  up_pm_set_bootmask(PM_BOOT_COLD_RTC | PM_BOOT_COLD_RTC_ALM0);

  fd = open(ALARM_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      printf("Could not open %s\n", ALARM_DEVPATH);
      return -1;
    }

  /* Set the alarm expired after the specified time */

  setrel.id      = 0;
  setrel.pid     = getpid();
  setrel.reltime = seconds;

  setrel.event.sigev_notify = SIGEV_SIGNAL;
  setrel.event.sigev_signo  = ALARM_SIGNO;
  setrel.event.sigev_value.sival_int = 0;

  do
    {
      /* If the RTC is not initialized and returns EBUSY,
       * set the RTC alarm repeatedly until it returns OK.
       */

      ret = ioctl(fd, RTC_SET_RELATIVE, (unsigned long)&setrel);
    }
  while ((ret < 0) && (errno == EBUSY));

  close(fd);
  if (ret < 0)
    {
      printf("Could not set alarm..\n");
      return ret;
    }

  /* Select sleep mode */

  if (up_pm_get_bootcause() & (PM_BOOT_COLD_RTC | PM_BOOT_COLD_RTC_ALM0))
    {
      /* Last time was cold sleep, so go to deep sleep then */

      printf("Sleep DEEPLY in %ld seconds...\n", seconds);
      boardctl(BOARDIOC_POWEROFF, 0);
    }
  else
    {
      /* Last time was deep sleep or POR, so go to deep sleep then */

      printf("Sleep COLDLY in %ld seconds...\n", seconds);
      boardctl(BOARDIOC_POWEROFF, 1);
    }

  /* Never reached because the power will be off as deep/cold sleep */

  return 0;
}
