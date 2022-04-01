/****************************************************************************
 * externals/vtun/nuttx/tun_dev.c
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
#include <stdint.h>
#include <arpa/inet.h>
#include <nuttx/net/tun.h>
#include <net/if.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <debug.h>

#include "vtun.h"
#include "lib.h"
#include "compat_nuttx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DEVNAME 8

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct tun_priv_s
{
  int  fd;
  char devname[MAX_DEVNAME];
};

typedef struct pcap_hdr_s
{
  uint32_t magic_number;
  uint16_t version_major;
  uint16_t version_minor;
  int32_t  thiszone;
  uint32_t sigfigs;
  uint32_t snaplen;
  uint32_t network;
} pcap_hdr_t;

typedef struct pcaprec_hdr_s
{
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint32_t incl_len;
  uint32_t orig_len;
} pcaprec_hdr_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct tun_priv_s g_tun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tun_configure
 ****************************************************************************/

static int tun_configure(struct tun_priv_s *tun, int istun)
{
  struct ifreq ifr;
  int errcode;
  int ret;

  tun->fd = open("/dev/tun", O_RDWR);
  if (tun->fd < 0)
    {
      errcode = errno;
      vtun_syslog(LOG_ERR, "ERROR: Failed to open /dev/tun: %d", errcode);
      return -errcode;
    }

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = (istun != 0) ? IFF_TUN : IFF_TAP;

  ret = ioctl(tun->fd, TUNSETIFF, (unsigned long)&ifr);
  if (ret < 0)
    {
      errcode = errno;
      vtun_syslog(LOG_ERR, "ERROR: ioctl TUNSETIFF failed: %d", errcode);
      close(tun->fd);
      return -errcode;
    }

  strncpy(tun->devname, ifr.ifr_name, MAX_DEVNAME);
  vtun_syslog(LOG_ERR, "Created TUN device: %s", tun->devname);

  return 0;
}

static int tun_open_common(char *dev, int istun)
{
  int ret;

  ret = tun_configure(&g_tun_dev, istun);
  if (ret < 0)
    {
      vtun_syslog(LOG_ERR, "ERROR: Failed to create tun: %d", ret);
      return -1;
    }

  return g_tun_dev.fd;
}

static int tun_write_common(int fd, char *buf, int len)
{
  int ret = 0;

  ret = write(fd, buf, len);
  if (ret < 0)
    {
      vtun_syslog(LOG_DEBUG, "%s error(%d)", __FUNCTION__, errno);
    }

  return 0;
}

static int tun_read_common(int fd, char *buf, int len)
{
  int ret = 0;

  ret = read(fd, buf, len);

#ifdef ENABLE_DUMP_PCAP
  if (0 < ret)
    {
      dump_pcap(buf, ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tun_open(char *dev)
{
  return tun_open_common(dev, 1);
}

int tap_open(char *dev)
{
  return tun_open_common(dev, 0);
}

int tun_close(int fd, char *dev)
{
  return close(fd);
}

int tap_close(int fd, char *dev)
{
  return close(fd);
}

/* Read/write frames from TUN device */

int tun_write(int fd, char *buf, int len)
{
  return tun_write_common(fd, buf, len);
}

int tap_write(int fd, char *buf, int len)
{
  return tun_write_common(fd, buf, len);
}

int tun_read(int fd, char *buf, int len)
{
  return tun_read_common(fd, buf, len);
}

int tap_read(int fd, char *buf, int len)
{
  return tun_read_common(fd, buf, len);
}

