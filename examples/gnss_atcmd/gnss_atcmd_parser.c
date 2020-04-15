/****************************************************************************
 * gnss_atcmd/gnss_atcmd_parser.c
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

#include <nuttx/config.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <arch/chip/gnss.h>
#include "gpsutils/cxd56_gnss_nmea.h"

#include "gnss_atcmd.h"
#include "gnss_usbserial.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

#define _HEADER_SIZE 4
#define _FOOTER_SIZE 4
#define _HEADERFOOTER_SIZE (_HEADER_SIZE + _FOOTER_SIZE)
#define _WBUF_SIZE 160
#define _READY_POLL_FD_NUM 1
#define _MAX_ARGC 8

/* Header/footer for output binary data */

#define CMD_HEADER_MGC 0xA0
#define CMD_FOOTER_MGC 0xB0
#define CMD_HEADER_CHKSUM 0x80u
#define CMD_HEADER_DT_D2HA 0x40u /* Identify  D2HA packet */
#define CMD_HEADER_RESV 0x30u    /* Identify  D2HA packet */
#define CMD_HEADER_IDMASK 0x0Fu
#define CMD_HEADER_SETUP(head, cs, d2ha, id, len)                       \
  {                                                                     \
    ((uint8_t *)(head))[0] = (uint8_t)CMD_HEADER_MGC;                   \
    ((uint8_t *)(head))[1] = (uint8_t)((cs) ? CMD_HEADER_CHKSUM : 0u) | \
                             ((d2ha) ? CMD_HEADER_DT_D2HA : 0u) |       \
                             ((id)&CMD_HEADER_IDMASK);                  \
    ((uint8_t *)(head))[2] = (uint8_t)((len) >> 8) & 0xffu;             \
    ((uint8_t *)(head))[3] = (uint8_t)((len)) & 0xffu;                  \
  }
#define CMD_FOOTER_SETUP(foot, ret_seq, csum)      \
  {                                                \
    ((uint8_t *)(foot))[0] = ((csum) >> 8) & 0xff; \
    ((uint8_t *)(foot))[1] = ((csum)) & 0xff;      \
    ((uint8_t *)(foot))[2] = (ret_seq)&0xff;       \
    ((uint8_t *)(foot))[3] = CMD_FOOTER_MGC;       \
  }

/***************************************************************************/

struct orbitaldata_info
{
  int req;
  int sattype;
  int datasize;
};

struct atcmd_entry
{
  int (*func)(FAR struct gnss_atcmd_info *info,
              FAR struct atcmd_entry *cmdentry, FAR char *argv[], int argc);
  char *cmd;
  int   argnum;
};

/***************************************************************************
 * Private variables
 ***************************************************************************/

static char wbuf[_WBUF_SIZE];

/***************************************************************************
 * Private functions
 ***************************************************************************/

static void toupper_line(FAR char *line, int size)
{
  int i;
  for (i = 0; i < size; i++)
    {
      line[i] = (char)toupper(line[i]);
    }
}

static int token_line(FAR char *line, char *argv[], int argn)
{
  int   argc = 0;
  char *tp;

  tp = strtok(line, " ");
  while (tp != NULL && argc <= argn)
    {
      argv[argc] = tp;
      argc++;
      tp = strtok(NULL, " ");
    }
  return argc;
}

static uint16_t calc_checksum(FAR char *dp, int16_t sz)
{
  uint32_t cs = 0;

  while (sz > 0)
    {
      cs += (uint32_t)(*dp & 0xff);
      dp++;
      sz--;
    }

  return (uint16_t)(cs & 0xffffu);
}

static int get_data_size(FAR char *buf)
{
  uint32_t *head;
  int       len;

  head = (uint32_t *)buf;

  if (((uint8_t *)head)[0] != (uint8_t)CMD_HEADER_MGC)
    {
      return -EINVAL;
    }

  len = (((uint8_t *)head)[2] << 8) | ((uint8_t *)head)[3];

  return len;
}

static int check_headfoot(FAR char *buf)
{
  uint32_t *head;
  uint32_t *foot;
  int       len;

  len = get_data_size(buf);
  if (len < 0)
    {
      return len;
    }

  head = (uint32_t *)buf;
  foot = (uint32_t *)(buf + _HEADER_SIZE + len);

  if (((uint8_t *)foot)[3] != (uint8_t)CMD_FOOTER_MGC)
    {
      return -EFBIG;
    }

  if (((uint8_t *)head)[1] & (uint8_t)CMD_HEADER_CHKSUM)
    {
      uint16_t cs0;
      uint16_t cs1;
      cs0 = calc_checksum(buf, _HEADER_SIZE + len);
      cs1 = (((uint8_t *)foot)[0] << 8) | ((uint8_t *)foot)[1];
      return (cs0 - cs1) ? -EAGAIN : 0;
    }

  return 0;
}

static int set_headfoot(FAR char *buf, uint16_t sz)
{
  uint32_t *head;
  uint32_t *foot;
  uint16_t  cs;

  head = (uint32_t *)buf;
  foot = (uint32_t *)(buf + _HEADER_SIZE + sz);

  CMD_HEADER_SETUP(head, 1, 0, 0, sz);
  cs = calc_checksum(buf, _HEADER_SIZE + sz);
  CMD_FOOTER_SETUP(foot, 0, cs);

  return 0;
}

static int ready_read(FAR struct gnss_atcmd_info *info,
                      FAR struct atcmd_entry *cmdentry, FAR char *buf,
                      int size, int timeout)
{
  int           ret;
  int           n;
  struct pollfd fds[_READY_POLL_FD_NUM] = {{0}};

  ret = gnss_atcmd_printf(info->wfd, "[%s] Ready\r\n", cmdentry->cmd);

  fds[0].fd     = info->rfd;
  fds[0].events = POLLIN;

  do
    {
      ret = poll(fds, _READY_POLL_FD_NUM, timeout);
      if (ret < 0)
        {
          ret = -errno;
          goto _err;
        }
      else if (ret == 0)
        {
          ret = -ETIMEDOUT;
          goto _err;
        }

      n = read(info->rfd, buf, size);
      if (n < 0)
      {
        ret = -errno;
        break;
      }
      size -= n;
      buf += n;
    }
  while (size > 0);

_err:
  return ret;
}

/***************************************************************************/

static double dms_to_double(const int degree, const int minute,
                            const int second, const int fraction)
{
  return (double)degree + (double)minute / 60.0 + (double)second / 3600.0 +
         (double)fraction / 3600000.0;
}

/***************************************************************************/

static int set_orbitaldata(FAR struct gnss_atcmd_info  *info,
                           FAR struct atcmd_entry      *cmdentry,
                           FAR struct orbitaldata_info *oinfo)
{
  int                             ret;
  int                             bufsize;
  struct cxd56_gnss_orbital_param_s oparam;
  FAR char                       *buf;

  bufsize = oinfo->datasize + _HEADERFOOTER_SIZE;
  buf     = malloc(bufsize);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  ret = ready_read(info, cmdentry, buf, bufsize, 5000);
  if (ret < 0)
    {
      goto _err;
    }

  ret = check_headfoot(buf);
  if (ret < 0)
    {
      goto _err;
    }

  oparam.type = oinfo->sattype;
  oparam.data = (uint32_t *)(buf + _HEADER_SIZE);

  ret = ioctl(info->gnssfd, oinfo->req, (unsigned long)&oparam);
  if (ret < 0)
    {
      ret = -errno;
      goto _err;
    }

_err:
  free(buf);

  return ret;
}

static int get_orbitaldata(FAR struct gnss_atcmd_info  *info,
                           FAR struct atcmd_entry      *cmdentry,
                           FAR struct orbitaldata_info *oinfo)
{
  int                             ret;
  int                             n;
  int                             bufsize;
  struct cxd56_gnss_orbital_param_s oparam;
  FAR char                       *buf;

  bufsize = oinfo->datasize + _HEADERFOOTER_SIZE;
  buf     = malloc(bufsize);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  oparam.type = oinfo->sattype;
  oparam.data = (uint32_t *)(buf + _HEADER_SIZE);

  ret = ioctl(info->gnssfd, oinfo->req, (unsigned long)&oparam);
  if (ret < 0)
    {
      ret = -errno;
      goto _err;
    }

  set_headfoot(buf, oinfo->datasize);

  n = write(info->wfd, buf, bufsize);
  if (n < 0)
    {
      ret = -errno;
      goto _err;
    }

_err:
  free(buf);

  return ret;
}

/****************************************************************************
 * @ commands
 ****************************************************************************/

static int command_bssl(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  uint32_t mask;

  mask = strtol(argv[1], NULL, 0);

  NMEA_SetMask(mask);

  return 0;
}

static int command_bup(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA, 0);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_cepa(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct cxd56_gnss_cep_age_s age;
  int ret;

  ret =
    ioctl(info->gnssfd, CXD56_GNSS_IOCTL_GET_CEP_AGE, (unsigned long)&age);
  if (ret < 0)
    {
      return -errno;
    }
  ret = gnss_atcmd_printf(info->wfd, "%.2f %.2f", age.cepi, age.age);
  if (ret >= 0)
    {
      ret = 0;
    }
  return ret;
}

static int command_cepc(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_CHECK_CEP_DATA, 0);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_cepl(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_CLOSE_CEP_DATA, 0);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_cepo(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_OPEN_CEP_DATA, 0);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gsop(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct cxd56_gnss_ope_mode_param_s ope_mode;
  ope_mode.mode  = atoi(argv[1]);
  ope_mode.cycle = atoi(argv[2]);

  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SET_OPE_MODE,
              (unsigned long)&ope_mode);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gcd(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_COLD);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gsw(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_WARM);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gswp(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_WARM_ACC2);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gsp(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_HOT_ACC);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gspp(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_HOT_ACC2);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gspq(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_HOT_ACC3);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gsr(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_START,
              (unsigned long)CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gstp(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gns(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int          ret = -EINVAL;
  unsigned int sat = strtol(argv[1], NULL, 0);

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM,
              (unsigned long)sat);
  if (ret < 0)
    {
      ret = -errno;
    }

  return ret;
}

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
static int command_gnfs(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct cxd56_gnss_spectrum_control_s control;
  int                                ret;

  control.time   = strtol(argv[1], NULL, 0);
  control.enable = strtol(argv[2], NULL, 0);
  control.point1 = strtol(argv[3], NULL, 0);
  control.step1  = strtol(argv[4], NULL, 0);
  control.point2 = strtol(argv[5], NULL, 0);
  control.step2  = strtol(argv[6], NULL, 0);

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SPECTRUM_CONTROL,
              (unsigned long)&control);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

static int command_gpoe(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct cxd56_gnss_ellipsoidal_position_s pos;

  int degree;
  int minute;
  int second;
  int ret;

  degree = strtol(argv[1], NULL, 0);
  minute = strtol(argv[2], NULL, 0);
  second = strtol(argv[3], NULL, 0);

  pos.latitude = dms_to_double(degree, minute, second, 0.0);

  degree = strtol(argv[4], NULL, 0);
  minute = strtol(argv[5], NULL, 0);
  second = strtol(argv[6], NULL, 0);

  pos.longitude = dms_to_double(degree, minute, second, 0.0);

  pos.altitude = 0.0;

  ret = ioctl(info->gnssfd,
              CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL,
              (unsigned long)&pos);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gtim(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int                        ret = -EINVAL;
  struct cxd56_gnss_datetime_s datetime;

  datetime.date.year = atoi(argv[1]);
  if (datetime.date.year < 1970 || datetime.date.year > 2050)
    {
      goto _err;
    }
  datetime.date.month = atoi(argv[2]);
  if (datetime.date.month < 1 || datetime.date.month > 12)
    {
      goto _err;
    }
  datetime.date.day = atoi(argv[3]);
  if (datetime.date.day < 1 || datetime.date.day > 31)
    {
      goto _err;
    }
  datetime.time.hour = atoi(argv[4]);
  if (datetime.time.hour < 0 || datetime.time.hour > 24)
    {
      goto _err;
    }
  datetime.time.minute = atoi(argv[5]);
  if (datetime.time.minute < 0 || datetime.time.minute > 59)
    {
      goto _err;
    }
  datetime.time.sec = atoi(argv[6]);
  if (datetime.time.sec < 0 || datetime.time.sec > 59)
    {
      goto _err;
    }
  datetime.time.usec = 0;

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SET_TIME,
              (unsigned long)&datetime);
  if (ret < 0)
    {
      ret = -errno;
    }
_err:
  return ret;
}

static int command_gtcx(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int     ret = -EINVAL;
  int32_t offset;

  offset = atoi(argv[1]);

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SET_TCXO_OFFSET,
              (unsigned long)offset);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gptc(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int32_t offset;
  int     len;
  int     n;
  int     ret;

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_GET_TCXO_OFFSET,
              (unsigned long)&offset);
  if (ret < 0)
    {
      ret = -errno;
      goto _err;
    }
  len = snprintf(wbuf + _HEADER_SIZE, _WBUF_SIZE - _HEADER_SIZE,
                 "%d\r\n", offset);
  if (len < 0)
    {
      ret = len;
      goto _err;
    }
  set_headfoot(wbuf, len);
  n = write(info->wfd, wbuf, len + _HEADERFOOTER_SIZE);
  if (n < 0)
    {
      ret = -errno;
      goto _err;
    }
  ret = 0;
_err:
  return ret;
}

static int command_gte(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_FACTORY_STOP_TEST, 0);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gtr(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  struct cxd56_gnss_test_result_s result;
  int                           len;
  int                           n;
  int                           ret;

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT,
              (unsigned long)&result);
  if (ret < 0)
    {
      ret = -errno;
      goto _err;
    }
  len = snprintf(wbuf + _HEADER_SIZE, _WBUF_SIZE - _HEADER_SIZE,
                 "%.1f,%.2f\r\n", result.cn, result.doppler);
  if (len < 0)
    {
      ret = len;
      goto _err;
    }
  set_headfoot(wbuf, len);
  n = write(info->wfd, wbuf, len + _HEADERFOOTER_SIZE);
  if (n < 0)
    {
      ret = -errno;
      goto _err;
    }
  ret = 0;
_err:
  return ret;
}

static int command_gts(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  struct cxd56_gnss_test_info_s sinfo;
  int                         ret;

  sinfo.satellite = atoi(argv[1]);
  sinfo.reserve1  = atoi(argv[2]);
  sinfo.reserve2  = atoi(argv[3]);
  sinfo.reserve3  = atoi(argv[4]);

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_FACTORY_START_TEST,
              (unsigned long)&sinfo);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gemg(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_GET_EPHEMERIS,
    CXD56_GNSS_DATA_GPS,
    CXD56_GNSS_GPS_EPHEMERIS_SIZE,
  };

  return get_orbitaldata(info, cmdentry, &oinfo);
}

static int command_gems(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_SET_EPHEMERIS,
    CXD56_GNSS_DATA_GPS,
    CXD56_GNSS_GPS_EPHEMERIS_SIZE,
  };

  return set_orbitaldata(info, cmdentry, &oinfo);
}

static int command_galg(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_GET_ALMANAC,
    CXD56_GNSS_DATA_GPS,
    CXD56_GNSS_GPS_ALMANAC_SIZE
  };

  return get_orbitaldata(info, cmdentry, &oinfo);
}

static int command_gals(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_SET_ALMANAC,
    CXD56_GNSS_DATA_GPS,
    CXD56_GNSS_GPS_ALMANAC_SIZE
  };

  return set_orbitaldata(info, cmdentry, &oinfo);
}

static int command_lemg(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_GET_EPHEMERIS,
    CXD56_GNSS_DATA_GLONASS,
    CXD56_GNSS_GLONASS_EPHEMERIS_SIZE,
  };

  return get_orbitaldata(info, cmdentry, &oinfo);
}

static int command_lems(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_SET_EPHEMERIS,
    CXD56_GNSS_DATA_GLONASS,
    CXD56_GNSS_GLONASS_EPHEMERIS_SIZE,
  };

  return set_orbitaldata(info, cmdentry, &oinfo);
}

static int command_lalg(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {CXD56_GNSS_IOCTL_GET_ALMANAC,
                                   CXD56_GNSS_DATA_GLONASS,
                                   CXD56_GNSS_GLONASS_ALMANAC_SIZE};
  return get_orbitaldata(info, cmdentry, &oinfo);
}

static int command_lals(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {CXD56_GNSS_IOCTL_SET_ALMANAC,
                                   CXD56_GNSS_DATA_GLONASS,
                                   CXD56_GNSS_GLONASS_ALMANAC_SIZE};
  return set_orbitaldata(info, cmdentry, &oinfo);
}

static int command_qemg(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_GET_EPHEMERIS,
    CXD56_GNSS_DATA_QZSSL1CA,
    CXD56_GNSS_QZSSL1CA_EPHEMERIS_SIZE,
  };

  return get_orbitaldata(info, cmdentry, &oinfo);
}

static int command_qems(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_SET_EPHEMERIS,
    CXD56_GNSS_DATA_QZSSL1CA,
    CXD56_GNSS_QZSSL1CA_EPHEMERIS_SIZE,
  };

  return set_orbitaldata(info, cmdentry, &oinfo);
}

static int command_qalg(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_GET_ALMANAC,
    CXD56_GNSS_DATA_QZSSL1CA,
    CXD56_GNSS_QZSSL1CA_ALMANAC_SIZE,
  };

  return get_orbitaldata(info, cmdentry, &oinfo);
}

static int command_qals(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  struct orbitaldata_info oinfo = {
    CXD56_GNSS_IOCTL_SET_ALMANAC,
    CXD56_GNSS_DATA_QZSSL1CA,
    CXD56_GNSS_QZSSL1CA_ALMANAC_SIZE
  };

  return set_orbitaldata(info, cmdentry, &oinfo);
}

static int command_guse(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int     ret = -EINVAL;
  uint32_t usecase;

  usecase = strtoul(argv[1], NULL, 0);

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SET_USECASE,
              (unsigned long)usecase);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_gguc(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int ret;
  uint32_t usecase;

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_GET_USECASE,
              (unsigned long)&usecase);
  if (ret < 0)
    {
      ret = -errno;
    }
  ret = gnss_atcmd_printf(info->wfd, "0x%08x\r\n", usecase);
  if (ret >= 0)
    {
      ret = 0;
    }
  return ret;
}

static int command_gpps(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  int     ret = -EINVAL;
  uint32_t enable;

  enable = strtoul(argv[1], NULL, 0);

  ret = ioctl(info->gnssfd, CXD56_GNSS_IOCTL_SET_1PPS_OUTPUT,
              (unsigned long)enable);
  if (ret < 0)
    {
      ret = -errno;
    }
  return ret;
}

static int command_ver(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  int ret;
  ret = gnss_atcmd_printf(info->wfd, "00000,0000000,0,0,0\r\n", 0, 0);
  if (ret >= 0)
    {
      ret = 0;
    }
  return ret;
}

static int command_nop(FAR struct gnss_atcmd_info *info,
                       FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                       int argc)
{
  return 0;
}

static int command_aext(FAR struct gnss_atcmd_info *info,
                        FAR struct atcmd_entry *cmdentry, FAR char *argv[],
                        int argc)
{
  return 0;
}

/***************************************************************************/

static struct atcmd_entry atcmd_entry_table[] = {
  {command_bssl, "BSSL", 1},
  {command_bup,  "BUP",  0},
  {command_cepa, "CEPA", 0},
  {command_cepc, "CEPC", 0},
  {command_cepl, "CEPL", 0},
  {command_cepo, "CEPO", 0},
  {command_galg, "GALG", 0},
  {command_gals, "GALS", 0},
  {command_gcd,  "GCD",  0},
  {command_gemg, "GEMG", 0},
  {command_gems, "GEMS", 0},
  {command_gns,  "GNS",  1},
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
  {command_gnfs, "GNFS", 6},
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */
  {command_gpoe, "GPOE", 6},
  {command_gptc, "GPTC", 0},
  {command_gsop, "GSOP", 3},
  {command_gsp,  "GSP",  0},
  {command_gspp, "GSPP", 0},
  {command_gspq, "GSPQ", 0},
  {command_gsr,  "GSR",  0},
  {command_gstp, "GSTP", 0},
  {command_gsw,  "GSW",  0},
  {command_gswp, "GSWP", 0},
  {command_gtim, "GTIM", 6},
  {command_gtcx, "GTCX", 1},
  {command_gte,  "GTE",  0},
  {command_gtr,  "GTR",  0},
  {command_gts,  "GTS",  4},
  {command_lalg, "LALG", 0},
  {command_lals, "LALS", 0},
  {command_lemg, "LEMG", 0},
  {command_lems, "LEMS", 0},
  {command_qalg, "QALG", 0},
  {command_qals, "QALS", 0},
  {command_qemg, "QEMG", 0},
  {command_qems, "QEMS", 0},
  {command_nop,  "AQCK", 1},
  {command_guse, "GUSE", 1},
  {command_gguc, "GGUC", 0},
  {command_gpps, "GPPS", 1},
  {command_nop,  "LEMG", 0},
  {command_nop,  "TRCK", 1},
  {command_ver,  "VER",  0},
  {command_aext, "AEXT", 0},
};

/***************************************************************************
 * Public functions
 ***************************************************************************/

int gnss_atcmd_exec(FAR struct gnss_atcmd_info *info, FAR char *line,
                    int size)
{
  int                 i;
  int                 ret;
  int                 argc;
  char               *cmd_name;
  char               *argv[_MAX_ARGC];
  struct atcmd_entry *cmd_tbl;

  toupper_line(line, size);
  argc = token_line(line, argv, _MAX_ARGC);

  if (argc <= 0 || *argv[0] != '@')
    {
      return 0;
    }

  cmd_name = argv[0] + 1;
  for (i = 0; i < sizeof(atcmd_entry_table) / sizeof(atcmd_entry_table[0]);
       i++)
    {
      cmd_tbl = &atcmd_entry_table[i];
      if (strncmp(cmd_tbl->cmd, argv[0] + 1, 4) != 0)
        {
          continue;
        }
      if (cmd_tbl->argnum != argc - 1)
        {
          ret = -EINVAL;
          goto _err;
        }
      ret = cmd_tbl->func(info, cmd_tbl, argv, argc);
      if (ret == 0)
        {
          ret = gnss_atcmd_printf(info->wfd, "[%s] Done\r\n", cmd_name);
          if (cmd_tbl->func == command_aext)
            {
              ret = -ESHUTDOWN;
            }
          return ret;
        }
      else
        {
          goto _err;
        }
    }

  ret = -ENOENT;

_err:
  return gnss_atcmd_printf(info->wfd, "[%s] Err %d\r\n", cmd_name, ret);
}

int gnss_atcmd_printf(int fd, FAR const IPTR char *fmt, ...)
{
  va_list ap;
  int     n;

  va_start(ap, fmt);

  n = vsnprintf(wbuf, _WBUF_SIZE, fmt, ap);

  va_end(ap);

  if (n < 0)
    {
      goto _err;
    }

  n = write(fd, wbuf, n);

_err:
  return n;
}
