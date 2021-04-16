/****************************************************************************
 *  VTun - Virtual Tunnel over TCP/IP network.
 *
 *  Copyright 2021 Sony Corporation
 *
 *  VTun has been derived from VPPP package by Maxim Krasnyansky.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
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

#include "vtun.h"
#include "lib.h"
#include "compat_nuttx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DEVNAME 8

#define PCAP_PATH "/mnt/spif/dump.pcap"

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

#ifdef ENABLE_DUMP_PCAP
static void dump_pcap_init(int istun)
{
  FILE *fp = NULL;
  pcap_hdr_t hdr;

  fp = fopen(PCAP_PATH, "wb");
  if (fp == NULL)
    {
      vtun_syslog(LOG_ERR, "pcap fopen error");
      return;
    }

  memset(&hdr, 0, sizeof(hdr));

  hdr.magic_number = 0xa1b2c3d4;
  hdr.version_major = 2;
  hdr.version_minor = 4;
  hdr.snaplen = 65535;
  hdr.network = (istun != 0) ? 228 : 1; /* 228: DLT_IPV4, 1: DLT_EN10MB */

  fwrite(&hdr, sizeof(hdr), 1, fp);

  fclose(fp);
}

static void dump_pcap(uint8_t *buf, size_t len)
{
  struct pcaprec_hdr_s hdr;
  FILE *fp = NULL;

  memset(&hdr, 0, sizeof(hdr));

  fp = fopen(PCAP_PATH, "ab");
  if (fp == NULL)
    {
      vtun_syslog(LOG_ERR, "fopen error");
      return;
    }

  hdr.incl_len = len;
  hdr.orig_len = len;

  fwrite(&hdr, sizeof(hdr), 1, fp);
  fwrite(buf, len, 1, fp);
  fflush(fp);
  fclose(fp);
}
#endif

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

#ifdef ENABLE_DUMP_PCAP
  dump_pcap_init(istun);
#endif

  return g_tun_dev.fd;
}

static int tun_write_common(int fd, char *buf, int len)
{
  int ret = 0;

#ifdef ENABLE_DUMP_PCAP
  if (0 < len)
    {
      dump_pcap(buf, len);
    }
#endif

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

