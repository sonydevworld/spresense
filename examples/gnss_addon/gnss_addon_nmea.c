/****************************************************************************
 * examples/gnss_addon/gnss_addon_nmea.c
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
#include <stdlib.h>
#include <arch/chip/gnss.h>
#include <gpsutils/cxd56_gnss_nmea.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_nmea_buf[NMEA_SENTENCE_MAX_LEN];
static FILE *g_stream;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NMEA callback functions */

static char *reqbuf(uint16_t size)
{
  return g_nmea_buf;
}

static void freebuf(char *buf)
{
  /* do nothing */
}

static int outnmea(char *buf)
{
  int ret = fprintf(g_stream, "%s", buf);
#ifdef CONFIG_EXAMPLES_GNSS_ADDON_FSYNC_LOGGING
  fsync(fileno(g_stream));
#endif
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int setup_nmea(FILE *stream)
{
  int ret;
  NMEA_OUTPUT_CB funcs;

  NMEA_InitMask2();

  /* Select NMEA sentence */

  NMEA_SetMask2(NMEA_GGA_ON |
                NMEA_GLL_ON |
                NMEA_GSA_ON |
                NMEA_GSV_ON |
                NMEA_GNS_ON |
                NMEA_RMC_ON |
                NMEA_VTG_ON |
                NMEA_QZQSM_ON |
                NMEA_ZDA_ON);

  /* Register callbacks to output NMEA */

  funcs.bufReq  = reqbuf;
  funcs.bufFree = freebuf;
  funcs.out     = outnmea;
  funcs.outBin  = NULL;

  ret = NMEA_RegistOutputFunc2(&funcs);

  /* Select output stream */

  g_stream = stream;

  return ret;
}

int print_nmea(struct cxd56_gnss_positiondata2_s *posdat)
{
  NMEA_Output2(posdat);
  return 0;
}

int print_dcreport(struct cxd56_gnss_dcreport_data_s *dcreport)
{
  static struct cxd56_gnss_dcreport_data_s s_dcreport;

  if (dcreport->svid == 0)
    {
        /* invalid data */

        return 0;
    }

  if (0 == memcmp(&s_dcreport, dcreport, sizeof(s_dcreport)))
    {
        /* not updated */

        return 0;
    }

  memcpy(&s_dcreport, dcreport, sizeof(s_dcreport));

  NMEA_DcReport_Output2(dcreport);
  return 0;
}
