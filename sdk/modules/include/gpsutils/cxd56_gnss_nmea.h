/****************************************************************************
 * modules/include/gpsutils/cxd56_gnss_nmea.h
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

#ifndef __SDK_MODULES_INCLUDE_GPSUTILS_CXD56_GNSS_NMEA_H
#define __SDK_MODULES_INCLUDE_GPSUTILS_CXD56_GNSS_NMEA_H

/**
 * @file gnss_type.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * include files
 *---------------------------------------------------------------------------*/

#include <stdint.h>
#include <arch/chip/gnss.h>

/**
 * @addtogroup gnss
 * @{ */

/**
 * @defgroup gnss_nmea NMEA output library
 * Description of CXD56xx GNSS NMEA converter interface.
 * @{ */


/** Maximum length of one line of NMEA sentence output from this library */

#define NMEA_SENTENCE_MAX_LEN  160

/** Structure of callback functions to manage buffer and output NMEA sentences */

typedef struct
{
  FAR char *(FAR *bufReq)(uint16_t);      /**< Buffer request callback func */
  void(FAR *bufFree)(FAR char *);         /**< Buffer free callback func */
  int(FAR *out)(FAR char *);              /**< Output callback func */
  int(FAR *outBin)(FAR char *, uint32_t); /**< Output callback func for binary */
} NMEA_OUTPUT_CB;

/** Spectrum OUTPUT data */

typedef char NMEA_SPECTRUM_DATA[112];

/** Raw data */

struct nmea_raw_s {
    double lat;
    double lon;
    double alt;
    float vel;
    float dir;
};

/**
 * Initialize NMEA sentence mask
 */

void NMEA_InitMask(void);
 
/**
 * Register output function
 * @param[in] *func : function pointer
 * @retval 0 : success
 * @retval <0 : fail
 */

int NMEA_RegistOutputFunc(FAR const NMEA_OUTPUT_CB *func);

/**
 * Set NMEA sentence mask
 * Give this function by ORing the bits in the table below corresponding to
 * the sentence to be output.
 * If set 0, stopping output.
 * @param[in] mask : 32bit mask value
 *
 * ___
 * ### Mask description
 *
 * - Defult value 0x000000ef.
 * - Bits not listed below are reserved.
 *
 * |bit |sentence|
 * |:----:|:---:|
 * |bit0|GGA|
 * |bit1|GLL|
 * |bit2|GSA|
 * |bit3|GSV|
 * |bit4|GNS|
 * |bit5|RMC|
 * |bit6|VTG|
 * |bit7|ZDA|
 * |bit22|QZQSM|
 *
 */

void NMEA_SetMask(uint32_t mask);

/**
 * Get NMEA sentence mask
 * @retval NMEA sentence mask
 */

uint32_t NMEA_GetMask(void);

/**
 * Output NMEA sentence
 * @param[in] pposdat : Position data output from GNSS
 * @retval >0 : success, output total sentence size
 * @retval <0 : fail
 */

uint16_t NMEA_Output(FAR const struct cxd56_gnss_positiondata_s* pposdat);

/*
 * Output QZSS Satellite Report sentence for Disaster and Crisis Management(DC Report)
 * @param[in] dcrdat : QZSS DC report data
 * @retval >0 : success, output total sentence size
 * @retval <0 : fail
 */
uint16_t NMEA_DcReport_Output(const struct cxd56_gnss_dcreport_data_s* dcrdat);

/**
 * Output Spectrum data as TEXT sentence
 * @param[in] spectrumdat : Spectrum data output from GNSS
 * @retval >0 : success, output total sentence size
 * @retval <0 : fail
 */

uint16_t NMEA_OutputSpectrum(FAR NMEA_SPECTRUM_DATA *spectrumdat);

/*
 * Extract raw data from postion data
 * @param[in] pposdat : Position data output from GNSS
 * @param[out] rawdat : Extracted raw data
 * @retval 0 : success
 * @retval <0 : fail
 */

int NMEA_ExtractRawData(FAR const struct cxd56_gnss_positiondata_s* pposdat,
                        FAR struct nmea_raw_s *rawdat);

/* @} gnss_nmea */
/* @} gnss */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SDK_MODULES_INCLUDE_GPSUTILS_CXD56_GNSS_NMEA_H */
