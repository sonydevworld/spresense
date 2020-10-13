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

#define MAX_DEVNAME 8

//#define ENABLE_DUMP_PCAP
#define PCAP_PATH "/mnt/spif/dump.pcap"

struct tun_priv_s
{
  int  fd;
  char devname[MAX_DEVNAME];
};

typedef struct pcap_hdr_s {
  uint32_t magic_number;
  uint16_t version_major;
  uint16_t version_minor;
  int32_t  thiszone;
  uint32_t sigfigs;
  uint32_t snaplen;
  uint32_t network;
} pcap_hdr_t;

typedef struct pcaprec_hdr_s {
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint32_t incl_len;
  uint32_t orig_len;
} pcaprec_hdr_t;

struct tun_priv_s g_tun_dev;

#ifdef ENABLE_DUMP_PCAP
static void dump_pcap_init()
{
  FILE* fp = NULL;
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
  hdr.network = 228; // DLT_IPV4

  fwrite(&hdr, sizeof(hdr), 1, fp);

  fclose(fp);
}

static void dump_pcap(uint8_t* buf, size_t len)
{
  struct pcaprec_hdr_s hdr;
  FILE* fp = NULL;

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
 * Name: vtun_set_ipv4addr
 ****************************************************************************/
static int vtun_set_ipv4addr(const char *ifname, const struct in_addr *addr)
{
  int ret = -1;

  if (ifname && addr)
    {
      int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (sockfd >= 0)
        {
          struct sockaddr_in *inaddr;
          struct ifreq req;

          strncpy(req.ifr_name, ifname, IFNAMSIZ);

          inaddr             = (struct sockaddr_in *)&req.ifr_addr;
          inaddr->sin_family = AF_INET;
          inaddr->sin_port   = 0;
          memcpy(&inaddr->sin_addr, addr, sizeof(struct in_addr));

          ret = ioctl(sockfd, SIOCSIFADDR, (unsigned long)&req);
          close(sockfd);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: vtun_set_ipv4netmask
 ****************************************************************************/
static int vtun_set_ipv4netmask(const char *ifname,
                                const struct in_addr *addr)
{
  int ret = -1;

  if (ifname && addr)
    {
      int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (sockfd >= 0)
        {
          struct sockaddr_in *inaddr;
          struct ifreq req;

          strncpy(req.ifr_name, ifname, IFNAMSIZ);

          inaddr             = (struct sockaddr_in *)&req.ifr_addr;
          inaddr->sin_family = AF_INET;
          inaddr->sin_port   = 0;
          memcpy(&inaddr->sin_addr, addr, sizeof(struct in_addr));

          ret = ioctl(sockfd, SIOCSIFNETMASK, (unsigned long)&req);
          close(sockfd);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: vtun_ifup
 ****************************************************************************/
static int vtun_ifup(const char *ifname)
{
  int ret = -1;
  if (ifname)
    {
      int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (sockfd >= 0)
        {
          struct ifreq req;
          memset (&req, 0, sizeof(struct ifreq));

          strncpy(req.ifr_name, ifname, IFNAMSIZ);

          req.ifr_flags |= IFF_UP;

          ret = ioctl(sockfd, SIOCSIFFLAGS, (unsigned long)&req);
          close(sockfd);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tun_configure
 ****************************************************************************/
static int tun_configure(struct tun_priv_s *tun)
{
  struct ifreq ifr;
  int errcode;
  int ret;

  tun->fd = open("/dev/tun", O_RDWR);
  if (tun->fd < 0)
    {
      errcode = errno;
      vtun_syslog(LOG_ERR,"ERROR: Failed to open /dev/tun: %d", errcode);
      return -errcode;
    }

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TUN;

  ret = ioctl(tun->fd, TUNSETIFF, (unsigned long)&ifr);
  if (ret < 0)
    {
      errcode = errno;
      vtun_syslog(LOG_ERR,"ERROR: ioctl TUNSETIFF failed: %d", errcode);
      close(tun->fd);
      return -errcode;
    }

  strncpy(tun->devname, ifr.ifr_name, MAX_DEVNAME);
  vtun_syslog(LOG_ERR, "Created TUN device: %s", tun->devname);

  return 0;
}

/****************************************************************************
 * Name: tun_netconf
 ****************************************************************************/

static int tun_netconf(struct tun_priv_s *tun, char *ipaddr,
                       char *netmask)
{
  int ret;
  struct in_addr *addr;
  char buf[INET_ADDRSTRLEN];

  ret = inet_pton(AF_INET, ipaddr, buf);
  if (ret != 1)
    {
      vtun_syslog(LOG_ERR, "inet_pton error: %s", ipaddr);
    }
  else
    {
      addr = (struct in_addr*) buf;
      ret = vtun_set_ipv4addr(tun->devname, addr);
      if (ret < 0)
        {
          vtun_syslog(LOG_ERR, "ERROR: vtun_set_ipv4addr() failed", ret);
        }
    }

  ret = inet_pton(AF_INET, netmask, buf);
  if (ret != 1)
    {
      vtun_syslog(LOG_ERR, "inet_pton error: %s", netmask);
    }
  else
    {
      addr = (struct in_addr*) buf;
      ret = vtun_set_ipv4netmask(tun->devname, addr);
      if (ret < 0)
        {
          vtun_syslog(LOG_ERR, "ERROR: netlib_set_ipv4netmask() failed", ret);
        }
    }

  vtun_ifup(tun->devname);
  return 0;
}

static int tun_open_common(char *dev, int istun)
{
  int ret;

  ret = tun_configure(&g_tun_dev);
  if (ret < 0)
    {
      vtun_syslog(LOG_ERR, "ERROR: Failed to create tun: %d", ret);
      return -1;
    }

  ret = tun_netconf(&g_tun_dev, CONFIG_EXAMPLES_VTUN_TUN_IP_ADDR,
                    CONFIG_EXAMPLES_VTUN_TUN_NETMASK);
  if (ret < 0)
    {
      vtun_syslog(LOG_ERR, "ERROR: tun_netconf for tun failed: %d", ret);
      close(g_tun_dev.fd);
      return -1;
    }

#ifdef ENABLE_DUMP_PCAP
  dump_pcap_init();
#endif

  return g_tun_dev.fd;
}

int tun_open(char *dev) { return tun_open_common(dev, 1); }
int tap_open(char *dev) { return -1; }

int tun_close(int fd, char *dev) { return close(fd); }
int tap_close(int fd, char *dev) { return -1; }

/* Read/write frames from TUN device */
int tun_write(int fd, char *buf, int len) { return write(fd, buf, len); }
int tap_write(int fd, char *buf, int len) { return -1; }

int tun_read(int fd, char *buf, int len) {
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
int tap_read(int fd, char *buf, int len) { return -1; }
