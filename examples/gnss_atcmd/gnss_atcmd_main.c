/****************************************************************************
 * gnss_atcmd/gnss_atcmd_main.c
 *
 *   Copyright 2018,2019 Sony Semiconductor Solutions Corporation
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
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <arch/chip/gnss.h>
#include "gpsutils/cxd56_gnss_nmea.h"
#if defined(CONFIG_EXAMPLES_GNSS_ATCMD_USB)
#include "gnss_usbserial.h"
#endif /* if defined(CONFIG_EXAMPLES_GNSS_ATCMD_USB) */
#include "gnss_atcmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_POLL_FD_NUM          2
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define GNSS_SIG_TERM             18
#define GNSS_SIG_SPECTRUM         19
#define GNSS_SIG_DCREPORT         20
#define GNSS_SIG_SARRLM           21
#define CMD_RBUF_SIZE             128
#define READ_FD                   cmdfds[GNSS_ATCMD_READ_FD]
#define WRITE_FD                  cmdfds[GNSS_ATCMD_WRITE_FD]

#define _USE_STATIC_NMEA_BUF

#ifdef _DEBUG
#define dbg_printf                printf
#else
#define dbg_printf(FMT, ...)
#endif

#if defined(CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM) || \
    defined(CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT) || \
    defined(CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM)
#define USE_ATCMD_SUB_THREAD
#endif

#ifdef USE_ATCMD_SUB_THREAD
#ifndef CONFIG_EXAMPLES_GNSS_ATCMD_SUB_STACKSIZE
#define CONFIG_EXAMPLES_GNSS_ATCMD_SUB_STACKSIZE  1024
#endif
#ifndef CONFIG_EXAMPLES_GNSS_ATCMD_SUB_PRIORITY
#define CONFIG_EXAMPLES_GNSS_ATCMD_SUB_PRIORITY   CONFIG_EXAMPLES_GNSS_ATCMD_PRIORITY
#endif
#endif /* ifdef USE_ATCMD_SUB_THREAD */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_STDINOUT
#define TTYS_NAME "stdin/stdout"
#endif
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_TTYS0
#define TTYS_NAME "/dev/ttyS0"
#endif
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_TTYS1
#define TTYS_NAME "/dev/ttyS1"
#endif
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_TTYS2
#define TTYS_NAME "/dev/ttyS2"
#endif

#ifndef CONFIG_EXAMPLES_GNSS_ATCMD_STDINOUT
#define CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD
#endif /* ifndef CONFIG_EXAMPLES_GNSS_ATCMD_TTYS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int8_t   sing;
  uint8_t  degree;
  uint8_t  minute;
  uint32_t frac;
} ST_DMS;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct gnss_atcmd_info atcmd_info;
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD
static pthread_t              atcmd_tid;
#endif /* ifndef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD */
static struct cxd56_gnss_positiondata_s  posdat;
static int                    cmdfds[2];
static char                   cmd_rbuf[CMD_RBUF_SIZE];
#ifdef _USE_STATIC_NMEA_BUF
static char                   nmea_buf[NMEA_SENTENCE_MAX_LEN];
#endif
#ifdef USE_ATCMD_SUB_THREAD
static sem_t                  syncsem;
static pthread_t              atcmd_sub_tid;
#endif /* ifdef USE_ATCMD_SUB_THREAD */
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
static NMEA_SPECTRUM_DATA     spectrumdat;
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT
static struct cxd56_gnss_dcreport_data_s dcreport;
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT */
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM
static struct cxd56_gnss_gal_sarrlm_s sarrlm;
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int print_nmea(int fd)
{
  int    ret;

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_LAST_GNSS, SEEK_SET);
  if (ret < 0)
    {
      ret = errno;
      printf("lseek error %d\n", ret);
      goto _err1;
    }

  ret = read(fd, &posdat, sizeof(posdat));
  if (ret < 0)
    {
      ret = errno;
      printf("read error %d\n", ret);
      goto _err1;
    }

   NMEA_Output(&posdat);

_err1:
  return ret;
}

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

static int print_spectrum(int fd)
{
  int    ret;

  do
    {
      ret = lseek(fd, CXD56_GNSS_READ_OFFSET_SPECTRUM, SEEK_SET);
      if (ret < 0)
        {
          ret = errno;
          printf("lseek error %d\n", ret);
          goto _err1;
        }

      ret = read(fd, &spectrumdat, sizeof(spectrumdat));
      if (ret < 0)
        {
          ret = errno;
          printf("read error %d\n", ret);
          goto _err1;
        }
      else if (ret == 0)
        {
          goto _err1;
        }

      NMEA_OutputSpectrum(&spectrumdat);
    }
  while (1);

_err1:
  return ret;
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT

static int print_dcreport(int fd)
{
  int    ret;

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_DCREPORT, SEEK_SET);
  if (ret < 0)
    {
      ret = errno;
      printf("lseek error %d\n", ret);
      goto _err1;
    }

  ret = read(fd, &dcreport, sizeof(dcreport));
  if (ret < 0)
    {
      ret = errno;
      printf("read error %d\n", ret);
      goto _err1;
    }

  NMEA_DcReport_Output(&dcreport);

_err1:
  return ret;
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM

static int print_sarrlm(int fd)
{
  int    ret;

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_SARRLM, SEEK_SET);
  if (ret < 0)
    {
      ret = errno;
      printf("lseek error %d\n", ret);
      goto _err1;
    }

  ret = read(fd, &sarrlm, sizeof(sarrlm));
  if (ret < 0)
    {
      ret = errno;
      printf("read error %d\n", ret);
      goto _err1;
    }

  NMEA_GalSarRlm_Output(&sarrlm);

_err1:
  return ret;
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM */

/* output NMEA */

FAR static char *reqbuf(uint16_t size)
{
#ifdef _USE_STATIC_NMEA_BUF
  if (size > sizeof(nmea_buf))
    {
      printf("reqbuf error: oversize %s\n", size);
      return NULL;
    }
  return nmea_buf;
#else
  return malloc(size);
#endif
}

static void freebuf(FAR char *buf)
{
#ifdef _USE_STATIC_NMEA_BUF
#else
  free(buf);
#endif
}

static int outnmea(FAR char *buf)
{
  return gnss_atcmd_printf(WRITE_FD, "%s", buf);
}

static int outbin(FAR char *buf, uint32_t len)
{
  return write(WRITE_FD, buf, (size_t)len);
}

#ifdef USE_ATCMD_SUB_THREAD

static int set_signal(int fd, int signo, uint8_t gnsssig, bool enable)
{
  struct cxd56_gnss_signal_setting_s setting;

  setting.fd      = fd;
  setting.enable  = enable;
  setting.gnsssig = gnsssig;
  setting.signo   = signo;
  setting.data    = NULL;

  return ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
}

static int signal2own(int signo, void *data)
{
  int ret;

#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
  value.sival_ptr = data;
  ret = sigqueue(getpid(), signo, value);
#else
  ret = sigqueue(getpid(), signo, data);
#endif

  return ret;
}

#endif /* ifdef USE_ATCMD_SUB_THREAD */

static FAR void atcmd_emulator(FAR void *arg)
{
  int            fd = -1;
  int            ret;
  int            remain;
  struct pollfd  fds[GNSS_POLL_FD_NUM];
  char *         bufhead;
  NMEA_OUTPUT_CB funcs;
#if !defined(CONFIG_EXAMPLES_GNSS_ATCMD_USB)
  struct termios tio;
  const speed_t  baudRate = B115200;
#endif /* if !defined(CONFIG_EXAMPLES_GNSS_ATCMD_USB) */

  /* program start */

  printf("Start GNSS_ATCMD!!\n");

  fd = open("/dev/gps", O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      ret = -ENODEV;
      goto _err;
    }

  /* Init NMEA library */

  NMEA_InitMask();
  funcs.bufReq  = reqbuf;
  funcs.out     = outnmea;
  funcs.outBin  = outbin;
  funcs.bufFree = freebuf;
  NMEA_RegistOutputFunc(&funcs);

#if defined(CONFIG_EXAMPLES_GNSS_ATCMD_USB)
  /* Open ttyUSB-CDC */

  ret = gnss_usbserial_open(cmdfds);
  if (ret < 0)
    {
      printf("usb open error. %d\n", ret);
      goto _err;
    }
#else
# if defined(CONFIG_EXAMPLES_GNSS_ATCMD_STDINOUT)
  /* Open tty0 using nash */

  cmdfds[GNSS_ATCMD_WRITE_FD] = fileno(stdout);
  cmdfds[GNSS_ATCMD_READ_FD] = fileno(stdin);
# else
  /* Open tty1/tty2 */

  cmdfds[GNSS_ATCMD_WRITE_FD] = cmdfds[GNSS_ATCMD_READ_FD] =
    open(TTYS_NAME, O_RDWR);
# endif
  ret = errno;
  if (cmdfds[GNSS_ATCMD_WRITE_FD] < 0 || cmdfds[GNSS_ATCMD_READ_FD] < 0)
    {
      printf("%s open error. %d\n", TTYS_NAME, ret);
      goto _err;
    }

  /* tty: setup parameters */

  tio.c_cflag += CREAD;  /* Enable receive */
  tio.c_cflag += CLOCAL; /* Local line, no modem control */
  tio.c_cflag += CS8;    /* Data bit 8bit */
  tio.c_cflag += 0;      /* Stop bit 1bit */
  tio.c_cflag += 0;      /* Paritiy none */
  cfsetispeed(&tio, baudRate);
  cfsetospeed(&tio, baudRate);

  /* tty: set to tty device */

  tcsetattr(cmdfds[GNSS_ATCMD_WRITE_FD], TCSANOW, &tio);
  tcsetattr(cmdfds[GNSS_ATCMD_READ_FD], TCSANOW, &tio);

  /* tty: Enable settings */

  ioctl(fd, TCSETS, (unsigned long)&tio);
#endif

  atcmd_info.gnssfd = fd;
  atcmd_info.wfd    = WRITE_FD;
  atcmd_info.rfd    = READ_FD;

  bufhead       = cmd_rbuf;
  remain        = sizeof(cmd_rbuf);

  do
    {
      int   sz;
      int   n;

#ifdef USE_ATCMD_SUB_THREAD
      sem_post(&syncsem);
#endif /* ifdef USE_ATCMD_SUB_THREAD */

      memset(fds, 0, sizeof(fds));
      fds[0].fd     = READ_FD;
      fds[0].events = POLLIN;
      fds[1].fd     = fd;
      fds[1].events = POLLIN;

      ret = poll(fds, GNSS_POLL_FD_NUM, GNSS_POLL_TIMEOUT_FOREVER);
      if (ret <= 0 && errno != EINTR)
        {
          printf("poll error %d,%d,%x,%x\n", ret, errno, fds[0].events,
                  fds[0].revents);
          break;
        }

      if (fds[1].revents & POLLIN)
        {
          print_nmea(fd);
        }

#ifdef USE_ATCMD_SUB_THREAD
      ret = sem_wait(&syncsem);
      if (ret < 0)
        {
          printf("unexpected wait syncsem error%d\n", ret);
          goto _err;
        }
#endif /* ifdef USE_ATCMD_SUB_THREAD */

      if (fds[0].revents & POLLIN)
        {
          sz = read(READ_FD, bufhead, remain);
          if (sz < 0)
            {
              continue;
            }
          remain -= sz;
          if (remain > 0)
            {
              for (n = 0; n < sz; n++, bufhead++)
                {
                  if (*bufhead == '\r' || *bufhead == '\n' || *bufhead == '\0')
                    {
                      break;
                    }
                }

              if (n == sz)
                {
                  continue;
                }

              sz = bufhead - cmd_rbuf + 1;
            }
          else
            {
              sz = sizeof(cmd_rbuf);
              bufhead  = &cmd_rbuf[sz - 1];
            }
          *bufhead = '\0';
          dbg_printf("# %*s\n", sz, cmd_rbuf);
#ifdef _DEBUG
          write(WRITE_FD, cmd_rbuf, sz);
#endif
          ret = gnss_atcmd_exec(&atcmd_info, cmd_rbuf, sz);

          remain  = sizeof(cmd_rbuf);
          bufhead = cmd_rbuf;
        }
    }
  while (ret != -ESHUTDOWN);

#ifdef USE_ATCMD_SUB_THREAD
  sem_post(&syncsem);
#endif /* ifdef USE_ATCMD_SUB_THREAD */

  /* close tty */

#if defined(CONFIG_EXAMPLES_GNSS_ATCMD_USB)
  gnss_usbserial_close(cmdfds);
#elif defined(CONFIG_EXAMPLES_GNSS_ATCMD_TTY1) || \
  defined(CONFIG_EXAMPLES_GNSS_ATCMD_TTY2)
  close(cmdfds[GNSS_ATCMD_READ_FD]);
  close(cmdfds[GNSS_ATCMD_WRITE_FD]);
#endif

_err:
#ifdef USE_ATCMD_SUB_THREAD

  /* Send signal to sub pthread and force exit it */

  signal2own(GNSS_SIG_TERM, NULL);
  pthread_join(atcmd_sub_tid, NULL);
  sem_destroy(&syncsem);
#endif /* ifdef USE_ATCMD_SUB_THREAD */

  /* close GPS device */

  if (fd >= 0)
    {
      ret = close(fd);
      if (ret < 0)
        {
          printf("device close error\n");
        }
    }

  printf("Stop GNSS_ATCMD!!\n");

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD
  pthread_detach(pthread_self());

  /* This pthread has returned, and exit */

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD */
}

#ifdef USE_ATCMD_SUB_THREAD

static FAR void subthread_handler(FAR void *arg)
{
  int      ret;
  int      signo;
  sigset_t mask;

  ret = sem_wait(&syncsem);
  if (ret < 0)
    {
      printf("unexpected wait syncsem error%d\n", ret);
      goto _err;
    }

  /* Init signal */

  sigemptyset(&mask);
  sigaddset(&mask, GNSS_SIG_TERM);

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

  ret =
    set_signal(atcmd_info.gnssfd, GNSS_SIG_SPECTRUM, CXD56_GNSS_SIG_SPECTRUM, 1);
  if (ret < 0)
    {
      printf("GNSS spectrum signal set error\n");
      goto _err;
    }

  sigaddset(&mask, GNSS_SIG_SPECTRUM);

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT

  ret =
    set_signal(atcmd_info.gnssfd, GNSS_SIG_DCREPORT, CXD56_GNSS_SIG_DCREPORT, 1);
  if (ret < 0)
    {
      printf("GNSS DC report signal set error\n");
      goto _err;
    }

  sigaddset(&mask, GNSS_SIG_DCREPORT);

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM

  ret =
    set_signal(atcmd_info.gnssfd, GNSS_SIG_SARRLM, CXD56_GNSS_SIG_SARRLM, 1);
  if (ret < 0)
    {
      printf("GNSS SAR/RLM signal set error\n");
      goto _err;
    }

  sigaddset(&mask, GNSS_SIG_SARRLM);

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM */

  sem_post(&syncsem);

  do
    {
      signo = sigwaitinfo(&mask, NULL);
      if (signo < 0)
        {
          continue;
        }
      else if (signo == GNSS_SIG_TERM)
        {
          goto _exit;
        }

      ret = sem_wait(&syncsem);
      if (ret < 0)
        {
          printf("unexpected wait syncsem error%d\n", ret);
          goto _exit;
        }

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
      if (signo == GNSS_SIG_SPECTRUM)
        {
          print_spectrum(atcmd_info.gnssfd);
        }
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT
      if (signo == GNSS_SIG_DCREPORT)
        {
          print_dcreport(atcmd_info.gnssfd);
        }
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM
      if (signo == GNSS_SIG_SARRLM)
        {
          print_sarrlm(atcmd_info.gnssfd);
        }
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM */

      sem_post(&syncsem);
    }
  while (1);

_exit:
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
  set_signal(atcmd_info.gnssfd, GNSS_SIG_SPECTRUM, CXD56_GNSS_SIG_SPECTRUM, 0);
#endif
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_DCREPORT
  set_signal(atcmd_info.gnssfd, GNSS_SIG_DCREPORT, CXD56_GNSS_SIG_DCREPORT, 0);
#endif
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SARRLM
  set_signal(atcmd_info.gnssfd, GNSS_SIG_SARRLM, CXD56_GNSS_SIG_SARRLM, 0);
#endif
_err:
  printf("GNSS exit subthread handler\n");
}

#endif /* ifdef USE_ATCMD_SUB_THREAD */

/****************************************************************************
 * gnss_atcmd_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  pthread_attr_t           tattr;
  struct sched_param       param;
  int                      ret = 0;

#ifdef USE_ATCMD_SUB_THREAD

  ret = sem_init(&syncsem, 0, 0);
  if (ret < 0)
    {
      _err("Failed to initialize syncsem!\n");
      goto _err;
    }

#endif /* ifdef USE_ATCMD_SUB_THREAD */

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD

  pthread_attr_init(&tattr);
  tattr.stacksize      = CONFIG_EXAMPLES_GNSS_ATCMD_STACKSIZE;
  param.sched_priority = CONFIG_EXAMPLES_GNSS_ATCMD_PRIORITY;
  pthread_attr_setschedparam(&tattr, &param);

  ret = pthread_create(&atcmd_tid, &tattr, (pthread_startroutine_t)atcmd_emulator,
                       (pthread_addr_t)NULL);
  if (ret != 0)
    {
      ret = -ret; /* pthread_create does not modify errno. */
      goto _err;
    }

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD */

#ifdef USE_ATCMD_SUB_THREAD

  pthread_attr_init(&tattr);
  tattr.stacksize      = CONFIG_EXAMPLES_GNSS_ATCMD_SUB_STACKSIZE;
  param.sched_priority = CONFIG_EXAMPLES_GNSS_ATCMD_SUB_PRIORITY;
  pthread_attr_setschedparam(&tattr, &param);

  ret = pthread_create(&atcmd_sub_tid, &tattr, (pthread_startroutine_t)subthread_handler,
                       (pthread_addr_t)NULL);
  if (ret != 0)
    {
      ret = -ret; /* pthread_create does not modify errno. */
    }

#endif /* ifdef USE_ATCMD_SUB_THREAD */

#ifndef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD

  atcmd_emulator(NULL);

#endif /* ifndef CONFIG_EXAMPLES_GNSS_ATCMD_CREATE_EMULATOR_PTHREAD */

_err:
  return ret;
}
