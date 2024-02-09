/****************************************************************************
 * modules/sensing/gnss/cxd5610nmea/gnss_nmea.c
 *
 *   Copyright 2023, 2024 Sony Semiconductor Solutions Corporation
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
#include <string.h>
#include <math.h>
#include <arch/chip/gnss.h>
#include <gpsutils/cxd56_gnss_nmea.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NMEA_SINGLE 0x00000001
#define NMEA_MULTI  0x00000002

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct cxd56_gnss_positiondata2_s posdat_t;
typedef struct cxd56_gnss_dcreport_data_s dcrdat_t;

struct svinfo_s
{
  uint8_t s;
  uint8_t e;
  uint8_t nl;
};

struct subtbl_s
{
  uint8_t *nl;
  uint8_t *array;
};

struct nmea_entry_s
{
  uint32_t mask;
  uint32_t attr;
  int (*func)(char *str, const size_t maxlen, const posdat_t *posdat);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nmea_gga(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_gll(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_gsa(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_gsv(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_gns(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_rmc(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_vtg(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_zda(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_gst(char *str, const size_t maxlen, const posdat_t *posdat);
static int nmea_qsm(char *str, const size_t maxlen, const dcrdat_t *dcrdat);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_nmea_mask;
static NMEA_OUTPUT_CB g_nmea_cb;

static const struct nmea_entry_s g_sentence_tbl[] =
{
  { NMEA_GGA_ON,  NMEA_SINGLE,  nmea_gga },
  { NMEA_GLL_ON,  NMEA_SINGLE,  nmea_gll },
  { NMEA_GSA_ON,  NMEA_MULTI,   nmea_gsa },
  { NMEA_GSV_ON,  NMEA_MULTI,   nmea_gsv },
  { NMEA_GNS_ON,  NMEA_SINGLE,  nmea_gns },
  { NMEA_RMC_ON,  NMEA_SINGLE,  nmea_rmc },
  { NMEA_VTG_ON,  NMEA_SINGLE,  nmea_vtg },
  { NMEA_ZDA_ON,  NMEA_SINGLE,  nmea_zda },
  { NMEA_GST_ON,  NMEA_SINGLE,  nmea_gst },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nmea_set_checksum(char *str, const size_t maxlen)
{
  uint32_t checksum = 0;

  /* Don't include the leading dollar sign in xor calculation */

  if (*str == '$')
    {
      str++;
    }

  while (*str != '\0')
    {
      checksum = checksum ^ (uint32_t)*str;
      str++;
    }

  return snprintf(str, maxlen, "*%02lX\r\n", checksum);
}

static int nmea_set_time(char *str, const size_t maxlen,
                         const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",%02d%02d%02d.%02ld",
                  rcv->time.hour, rcv->time.minute, rcv->time.sec,
                  rcv->time.usec / 10000);
}

static void nmea_deg2nmea(double deg, int *d, double *m, int *sign)
{
  int min_integer;

  *sign = (deg < 0.0) ? -1 : 1;
  deg = fabs(deg);

  *d = (int)deg;
  *m = (deg - *d) * 60.0;

  /* Round minute to the fifth place after the decimal point */

  min_integer = (int)*m;
  *m = *m - min_integer + (5 / 100000.0);
  *m = (int)(*m * 10000.0) / 10000.0;
  *m += min_integer;

  if (*m >= 60.0)
    {
      *d += 1;
      *m = 0.0;
    }
}

static int nmea_set_position(char *str, const size_t maxlen,
                             const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;
  int    latd;
  double latm;
  int    latsign;
  int    lond;
  double lonm;
  int    lonsign;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",,,,");
    }

  nmea_deg2nmea(rcv->latitude, &latd, &latm, &latsign);
  nmea_deg2nmea(rcv->longitude, &lond, &lonm, &lonsign);

  return snprintf(str, maxlen, ",%02d%07.4lf,%c,%03d%07.4lf,%c",
                  latd, latm, (latsign > 0) ? 'N' : 'S',
                  lond, lonm, (lonsign > 0) ? 'E' : 'W');
}

static int nmea_set_fixindicator(char *str, const size_t maxlen,
                                 const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",%1d", rcv->fix_indicator);
}

static int nmea_set_posnum(char *str, const size_t maxlen,
                           const posdat_t *posdat)
{
  int i;
  int posnum = 0;

  for (i = 0; i < posdat->svcount; i++)
    {
      if (posdat->sv[i].stat & (1 << 1))
        {
          posnum++;
        }
    }

  return snprintf(str, maxlen, ",%02d", posnum);
}

static int nmea_set_pdop(char *str, const size_t maxlen,
                         const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->pdop);
}

static int nmea_set_hdop(char *str, const size_t maxlen,
                         const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->hdop);
}

static int nmea_set_vdop(char *str, const size_t maxlen,
                         const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->vdop);
}

static int nmea_set_altitude(char *str, const size_t maxlen,
                             const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->altitude);
}

static int nmea_set_altitude_unit(char *str, const size_t maxlen,
                                  const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",M");
}

static int nmea_set_geoid(char *str, const size_t maxlen,
                          const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->geoid);
}

static int nmea_set_geoid_unit(char *str, const size_t maxlen,
                               const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",M");
}

static int nmea_set_dgps(char *str, const size_t maxlen,
                         const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",,");
}

static int nmea_set_gga(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPGGA");
}

static int nmea_set_gll(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPGLL");
}

static int nmea_set_talkerid(char *str, const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  switch (rcv->svtype)
    {
      case 1:
        str[2] = 'L';
        break;
      case 2:
        str[2] = 'A';
        break;
      case 3:
        str[2] = 'B';
        break;
      case 4:
        str[2] = 'Q';
        break;
      case 5:
        str[2] = 'I';
        break;
      case 6:
        str[2] = 'N';
        break;
      case 0:
      default:
        break;
    }

  return 0;
}

static int nmea_set_status(char *str, const size_t maxlen,
                           const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",%c",
                  (rcv->fix_indicator == 0) ? 'V' : 'A');
}

static int nmea_set_faamode(char *str, const size_t maxlen,
                            const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",%c",
                  (rcv->fix_indicator == 0) ? 'N' :
                  (rcv->fix_indicator == 1) ? 'A' :
                  (rcv->fix_indicator == 2) ? 'D' :
                  (rcv->fix_indicator == 6) ? 'E' : 'A');
}

static int nmea_gga(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_gga(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_time(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_position(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_fixindicator(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_posnum(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_hdop(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_altitude(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_altitude_unit(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_geoid(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_geoid_unit(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_dgps(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_gll(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_gll(&str[len], rem, posdat);
  nmea_set_talkerid(&str[0], posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_position(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_time(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_status(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_faamode(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_set_speed_knots(char *str, const size_t maxlen,
                                const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;
  float knot;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  knot = rcv->velocity / 1.852f;
  return snprintf(str, maxlen, ",%.1f", knot);
}

static int nmea_set_speed_kph(char *str, const size_t maxlen,
                              const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->velocity);
}

static int nmea_set_truecourse(char *str, const size_t maxlen,
                               const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  if (rcv->pos_dataexist == 0)
    {
      return snprintf(str, maxlen, ",");
    }

  return snprintf(str, maxlen, ",%.1f", rcv->direction);
}

static int nmea_set_ddmmyy(char *str, const size_t maxlen,
                           const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",%02d%02d%02d",
                  rcv->date.day, rcv->date.month, rcv->date.year % 100);
}

static int nmea_set_date(char *str, const size_t maxlen,
                         const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",%02d,%02d,%04d",
                  rcv->date.day, rcv->date.month, rcv->date.year);
}

static int nmea_set_magcourse(char *str, const size_t maxlen,
                              const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",");
}

static int nmea_set_magdirection(char *str, const size_t maxlen,
                                 const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",");
}

static int nmea_set_navstatus(char *str, const size_t maxlen,
                              const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",V");
}

static int nmea_set_rmc(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPRMC");
}

static int nmea_rmc(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_rmc(&str[len], rem, posdat);
  nmea_set_talkerid(&str[0], posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_time(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_status(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_position(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_speed_knots(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_truecourse(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_ddmmyy(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_magcourse(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_magdirection(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_faamode(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_navstatus(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_set_localzone(char *str, const size_t maxlen,
                              const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",,");
}

static int nmea_set_zda(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPZDA");
}

static int nmea_zda(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_zda(&str[len], rem, posdat);
  nmea_set_talkerid(&str[0], posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_time(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_date(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_localzone(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_set_vtg(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPVTG");
}

static int nmea_set_truecourse_unit(char *str, const size_t maxlen,
                                    const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",T");
}

static int nmea_set_magcourse_unit(char *str, const size_t maxlen,
                                   const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",M");
}

static int nmea_set_speed_knots_unit(char *str, const size_t maxlen,
                                     const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",N");
}

static int nmea_set_speed_kph_unit(char *str, const size_t maxlen,
                                   const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",K");
}

static int nmea_vtg(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_vtg(&str[len], rem, posdat);
  nmea_set_talkerid(&str[0], posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_truecourse(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_truecourse_unit(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_magcourse(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_magcourse_unit(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_speed_knots(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_speed_knots_unit(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_speed_kph(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_speed_kph_unit(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_faamode(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_set_deviation(char *str, const size_t maxlen,
                              const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;
  float maj = rcv->majdop;
  float min = rcv->mindop;
  float ori = rcv->oridop / 180.0f * M_PI;
  float latvar;
  float lonvar;

  if ((rcv->pos_dataexist == 0) || (rcv->fix_indicator == 0))
    {
      return snprintf(str, maxlen, ",,,,,,,");
    }

  latvar = sqrtf(powf(maj * cosf(ori), 2.0f) + powf(min * sinf(ori), 2.0f));
  lonvar = sqrtf(powf(maj * sinf(ori), 2.0f) + powf(min * cosf(ori), 2.0f));

  return snprintf(str, maxlen,
                  ",%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
                  rcv->hvar, rcv->majdop, rcv->mindop, rcv->oridop,
                  latvar, lonvar, rcv->vvar);
}

static int nmea_set_gst(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPGST");
}

static int nmea_gst(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_gst(&str[len], rem, posdat);
  nmea_set_talkerid(&str[0], posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_time(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_deviation(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_set_gsamode(char *str, const size_t maxlen,
                            const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;

  return snprintf(str, maxlen, ",A,%1d", rcv->pos_fixmode);
}

static int nmea_set_gsa(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPGSA");
}

static int nmea_set_gns(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPGNS");
}

static int nmea_set_gnsmode(char *str, const size_t maxlen,
                            const posdat_t *posdat)
{
  const struct cxd56_gnss_receiver2_s *rcv = &posdat->receiver;
  int i;
  char gps = 'N';
  char gln = 'N';
  char gal = 'N';
  char bds = 'N';
  char qzs = 'N';
  char nav = 'N';

  for (i = 0; i < posdat->svcount; i++)
    {
      if (posdat->sv[i].stat & (1 << 1))
        {
          if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_GPS_L5)
            {
              gps = (rcv->fix_indicator == 2) ? 'D' : 'A';
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_GLN_L1OF)
            {
              gln = (rcv->fix_indicator == 2) ? 'D' : 'A';
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_QZS_L5)
            {
              qzs = (rcv->fix_indicator == 2) ? 'D' : 'A';
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_BDS_B2A)
            {
              bds = (rcv->fix_indicator == 2) ? 'D' : 'A';
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_GAL_E5A)
            {
              gal = (rcv->fix_indicator == 2) ? 'D' : 'A';
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_NAV_L5)
            {
              nav = (rcv->fix_indicator == 2) ? 'D' : 'A';
            }
        }
    }

  return snprintf(str, maxlen, ",%c%c%c%c%c%c",
                  gps, gln, gal, bds, qzs, nav);
}

static int nmea_set_gnstail(char *str, const size_t maxlen,
                            const posdat_t *posdat)
{
  return snprintf(str, maxlen, ",,,V");
}

static int nmea_gns(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_gns(&str[len], rem, posdat);
  nmea_set_talkerid(&str[0], posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_time(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_position(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_gnsmode(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_posnum(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_hdop(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_altitude(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_geoid(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_gnstail(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_set_svid(uint8_t svid, uint8_t sv[], int nl)
{
  int i;

  if (12 < nl)
    {
      return 0;
    }

  for (i = 0; i < nl; i++)
    {
      if (sv[i] == svid)
        {
          return 0;
        }
    }

  sv[nl] = svid;

  return 1;
}

static int nmea_gsa_sub(char *str, const size_t maxlen,
                        const posdat_t *posdat, uint8_t sv[],
                        int nl, uint8_t signalid, uint8_t multi)
{
  int len = 0;
  int i;
  size_t rem = maxlen;

  if ((nl == 0) && (multi != 0))
    {
      return len;
    }

  len += nmea_set_gsa(&str[len], rem, posdat);
  if (multi ^ (1 << signalid))
    {
      str[2] = 'N';
    }

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_gsamode(&str[len], rem, posdat);
  for (i = 0; i < 12; i++)
    {
      rem = (maxlen > len) ? maxlen - len : 0;
      if (i < nl)
        {
          len += snprintf(&str[len], rem, ",%02d", sv[nl - 1 - i]);
        }
      else
        {
          len += snprintf(&str[len], rem, ",");
        }
    }

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_pdop(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_hdop(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_vdop(&str[len], rem, posdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  if (signalid)
    {
      len += snprintf(&str[len], rem, ",%1d", signalid);
    }
  else
    {
      len += snprintf(&str[len], rem, ",");
    }

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_gsa(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  int i;
  uint8_t nl_gps = 0;
  uint8_t nl_gln = 0;
  uint8_t nl_gal = 0;
  uint8_t nl_bds = 0;
  uint8_t nl_qzs = 0;
  uint8_t nl_nav = 0;
  uint8_t gps[12];
  uint8_t gln[12];
  uint8_t gal[12];
  uint8_t bds[12];
  uint8_t qzs[12];
  uint8_t nav[12];
  uint8_t constellation = 0;
  struct subtbl_s subtbl[] = {
    { &nl_gps, gps },
    { &nl_gln, gln },
    { &nl_gal, gal },
    { &nl_bds, bds },
    { &nl_qzs, qzs },
    { &nl_nav, nav },
  };

  for (i = 0; i < posdat->svcount; i++)
    {
      if (posdat->sv[i].stat & (1 << 1))
        {
          if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_GPS_L5)
            {
              constellation |= (1 << 1);
              nl_gps += nmea_set_svid(posdat->sv[i].svid, gps, nl_gps);
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_GLN_L1OF)
            {
              constellation |= (1 << 2);
              nl_gln += nmea_set_svid(posdat->sv[i].svid, gln, nl_gln);
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_QZS_L5)
            {
              constellation |= (1 << 5);
              nl_qzs += nmea_set_svid(posdat->sv[i].svid, qzs, nl_qzs);
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_BDS_B2A)
            {
              constellation |= (1 << 4);
              nl_bds += nmea_set_svid(posdat->sv[i].svid, bds, nl_bds);
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_GAL_E5A)
            {
              constellation |= (1 << 3);
              nl_gal += nmea_set_svid(posdat->sv[i].svid, gal, nl_gal);
            }
          else if (posdat->sv[i].type <= CXD56_GNSS_SIGNAL_NAV_L5)
            {
              constellation |= (1 << 6);
              nl_nav += nmea_set_svid(posdat->sv[i].svid, nav, nl_nav);
            }
        }
    }

  if (constellation == 0)
    {
      len = nmea_gsa_sub(str, maxlen, posdat, gps, nl_gps, 0, constellation);
      if ((len > 0) && g_nmea_cb.out)
        {
          g_nmea_cb.out(str);
        }

      return 0;
    }

  for (i = 0; i < sizeof(subtbl) / sizeof(subtbl[0]); i++)
    {
      len = nmea_gsa_sub(str, maxlen, posdat, subtbl[i].array, *subtbl[i].nl,
                         i + 1, constellation);
      if ((len > 0) && g_nmea_cb.out)
        {
          g_nmea_cb.out(str);
        }
    }

  return 0;
}

static int nmea_set_gsv(char *str, const size_t maxlen,
                        const posdat_t *posdat)
{
  return snprintf(str, maxlen, "$GPGSV");
}

static int nmea_gsv_sub(char *str, const size_t maxlen,
                        const posdat_t *posdat, int total_msgs,
                        int msg_nr, int total_sats, int s, int e,
                        int signalid)
{
  int len = 0;
  int i;
  size_t rem = maxlen;

  len += nmea_set_gsv(&str[len], rem, posdat);
  if (signalid <= CXD56_GNSS_SIGNAL_GPS_L5)
    {
      str[2] = 'P';
    }
  else if (signalid <= CXD56_GNSS_SIGNAL_GLN_L1OF)
    {
      str[2] = 'L';
    }
  else if (signalid <= CXD56_GNSS_SIGNAL_QZS_L5)
    {
      str[2] = 'Q';
    }
  else if (signalid <= CXD56_GNSS_SIGNAL_BDS_B2A)
    {
      str[2] = 'B';
    }
  else if (signalid <= CXD56_GNSS_SIGNAL_GAL_E5A)
    {
      str[2] = 'A';
    }
  else if (signalid <= CXD56_GNSS_SIGNAL_NAV_L5)
    {
      str[2] = 'I';
    }

  rem = (maxlen > len) ? maxlen - len : 0;
  len += snprintf(&str[len], rem, ",%d,%d,%d",
                  total_msgs, msg_nr, total_sats);

  int p;
  for (i = 0, p = e; i < 4; i++, p--)
    {
      rem = (maxlen > len) ? maxlen - len : 0;
      if (s <= p)
        {
          len += snprintf(&str[len], rem,
                          ",%02d", posdat->sv[p].svid);

          rem = (maxlen > len) ? maxlen - len : 0;
          if (posdat->sv[p].elevation > 0)
            {
              len += snprintf(&str[len], rem,
                              ",%d", posdat->sv[p].elevation);
            }
          else
            {
              len += snprintf(&str[len], rem, ",");
            }

          rem = (maxlen > len) ? maxlen - len : 0;
          if (posdat->sv[p].azimuth > 0)
            {
              len += snprintf(&str[len], rem,
                              ",%d", posdat->sv[p].azimuth);
            }
          else
            {
              len += snprintf(&str[len], rem, ",");
            }

          rem = (maxlen > len) ? maxlen - len : 0;
          if (posdat->sv[p].siglevel > 0)
            {
              len += snprintf(&str[len], rem,
                              ",%d", posdat->sv[p].siglevel);
            }
          else
            {
              len += snprintf(&str[len], rem, ",");
            }
        }
      else
        {
          len += snprintf(&str[len], rem, ",,,,");
        }
    }

  rem = (maxlen > len) ? maxlen - len : 0;
  len += snprintf(&str[len], rem,
                  ",%1d", (signalid == CXD56_GNSS_SIGNAL_GPS_L1CA) ? 1 :
                          (signalid == CXD56_GNSS_SIGNAL_GPS_L5) ? 7 :
                          (signalid == CXD56_GNSS_SIGNAL_GLN_L1OF) ? 1 :
                          (signalid == CXD56_GNSS_SIGNAL_QZS_L1CA) ? 1 :
                          (signalid == CXD56_GNSS_SIGNAL_QZS_L1S) ? 4 :
                          (signalid == CXD56_GNSS_SIGNAL_QZS_L5) ? 7 :
                          (signalid == CXD56_GNSS_SIGNAL_BDS_B1ID1) ? 1 :
                          (signalid == CXD56_GNSS_SIGNAL_BDS_B1ID2) ? 1 :
                          (signalid == CXD56_GNSS_SIGNAL_BDS_B1C) ? 3 :
                          (signalid == CXD56_GNSS_SIGNAL_BDS_B2A) ? 5 :
                          (signalid == CXD56_GNSS_SIGNAL_GAL_E1) ? 7 :
                          (signalid == CXD56_GNSS_SIGNAL_GAL_E5A) ? 1 :
                          (signalid == CXD56_GNSS_SIGNAL_NAV_L5) ? 1 : 0);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

static int nmea_gsv(char *str, const size_t maxlen, const posdat_t *posdat)
{
  int len = 0;
  int i;
  struct svinfo_s info[CXD56_GNSS_SIGNAL_SBAS_L1];
  int total_sats;
  int total_msgs;
  int msg_nr;
  int s;
  int e;
  uint8_t type;

  memset(info, 0, sizeof(info));
  for (i = 0; i < sizeof(info) / sizeof(info[0]); i++)
    {
      info[i].s = 255;
    }

  for (i = 0; i < posdat->svcount; i++)
    {
      type = posdat->sv[i].type;
      if (type >= CXD56_GNSS_SIGNAL_SBAS_L1)
        {
          continue;
        }

      if (type == CXD56_GNSS_SIGNAL_BDS_B1ID2)
        {
          type = CXD56_GNSS_SIGNAL_BDS_B1ID1;
        }

      info[type].nl++;
      if (info[type].s > i)
        {
          info[type].s = i;
        }

      if (info[type].e < i)
        {
          info[type].e = i;
        }
    }

  for (i = 0; i < sizeof(info) / sizeof(info[0]); i++)
    {
      total_sats = info[i].nl;
      total_msgs = (total_sats + 3) / 4;
      s = info[i].s;
      e = info[i].e;
      for (msg_nr = 1; msg_nr <= total_msgs; msg_nr++)
        {
          s = e - 3;
          if (s < info[i].s)
            {
              s = info[i].s;
            }

          len = nmea_gsv_sub(str, maxlen, posdat, total_msgs, msg_nr,
                             total_sats, s, e, i);
          if ((len > 0) && g_nmea_cb.out)
            {
              g_nmea_cb.out(str);
            }

          e -= 4;
        }
    }

  return 0;
}

static int nmea_set_qsm(char *str, const size_t maxlen,
                        const dcrdat_t *dcrdat)
{
  return snprintf(str, maxlen, "$QZQSM");
}

static int nmea_set_qsm_svid(char *str, const size_t maxlen,
                             const dcrdat_t *dcrdat)
{
  uint8_t svid = dcrdat->svid;

  return snprintf(str, maxlen, ",%d", svid);
}

static int nmea_set_qsm_msg(char *str, const size_t maxlen,
                            const dcrdat_t *dcrdat)
{
  int len = 0;
  int i;
  size_t rem = maxlen;

  len += snprintf(&str[len], rem, ",");

  for (i = 0; i < 63; i++)
    {
      rem = (maxlen > len) ? maxlen - len : 0;
      if ((i % 2) == 0)
        {
          len += snprintf(&str[len], rem,
                          "%X", dcrdat->sf[i / 2] >> 4);
        }
      else
        {
          len += snprintf(&str[len], rem,
                          "%X", dcrdat->sf[i / 2] & 0xf);
        }
    }

  return len;
}

static int nmea_qsm(char *str, const size_t maxlen, const dcrdat_t *dcrdat)
{
  int len = 0;
  size_t rem = maxlen;

  len += nmea_set_qsm(&str[len], rem, dcrdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_qsm_svid(&str[len], rem, dcrdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_qsm_msg(&str[len], rem, dcrdat);

  rem = (maxlen > len) ? maxlen - len : 0;
  len += nmea_set_checksum(&str[0], rem);

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void NMEA_InitMask2(void)
{
  g_nmea_mask = NMEA_GGA_ON |
                NMEA_GLL_ON |
                NMEA_GSA_ON |
                NMEA_GSV_ON |
                NMEA_RMC_ON |
                NMEA_VTG_ON |
                NMEA_ZDA_ON;
}

int NMEA_RegistOutputFunc2(const NMEA_OUTPUT_CB *func)
{
  if (!func || !func->bufReq || !func->bufFree)
    {
      return -1;
    }

  g_nmea_cb = *func;

  return 0;
}

void NMEA_SetMask2(uint32_t mask)
{
  g_nmea_mask = mask;
}

uint32_t NMEA_GetMask2(void)
{
  return g_nmea_mask;
}

uint16_t NMEA_Output2(const posdat_t *posdat)
{
  int i;
  uint16_t len = 0;
  char *sentence = NULL;
  const struct nmea_entry_s *entry;

  if (g_nmea_cb.bufReq)
    {
      sentence = g_nmea_cb.bufReq(NMEA_SENTENCE_MAX_LEN);
    }

  if (!sentence)
    {
      return len;
    }

  for (i = 0; i < sizeof(g_sentence_tbl) / sizeof(g_sentence_tbl[0]); i++)
    {
      entry = &g_sentence_tbl[i];
      if (g_nmea_mask & entry->mask)
        {
          len = entry->func(sentence, NMEA_SENTENCE_MAX_LEN, posdat);
          if ((len > 0) && g_nmea_cb.out && (entry->attr == NMEA_SINGLE))
            {
              g_nmea_cb.out(sentence);
            }
        }
    }

  return len;
}

uint16_t NMEA_DcReport_Output2(const dcrdat_t *dcrdat)
{
  uint16_t len = 0;
  char *sentence = NULL;

  if (g_nmea_cb.bufReq)
    {
      sentence = g_nmea_cb.bufReq(NMEA_SENTENCE_MAX_LEN);
    }

  if (!sentence)
    {
      return len;
    }

  if (g_nmea_mask & NMEA_QZQSM_ON)
    {
      len = nmea_qsm(sentence, NMEA_SENTENCE_MAX_LEN, dcrdat);
      if ((len > 0) && g_nmea_cb.out)
        {
          g_nmea_cb.out(sentence);
        }
    }

  return len;
}
