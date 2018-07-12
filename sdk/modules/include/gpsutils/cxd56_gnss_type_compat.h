/****************************************************************************
 * modules/include/gpsutils/cxd56_gnss_type_compat.h
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

/*
 * NOTICE:
 * This file is an old definition file to maintain compatibility with the old
 * API specification of CXD56_GNSS. Since it will be abolished in the future, 
 * PLEASE REWRITE THE APPLICATION USING THE NEW DEFINITION.
 */

/*
 * NOTICE:
 * This file defines a structure that stores GNSS positioning
 * data of CXD 56xx. The public header file cxd56_gnss_type.h for NuttX
 * of the CXD 56xx SDK has been copied as gd_type.h in the nxloader
 * build system and used. Therefore, if you change the definitions
 * in this file, please synchronize and change the other.
 */

#ifndef __SDK_MODULES_INCLUDE_GPSUTILS_CXD56_GNSS_TYPE_COMPAT_H
#define __SDK_MODULES_INCLUDE_GPSUTILS_CXD56_GNSS_TYPE_COMPAT_H

/**
 * @file cxd56_gnss_type_compat.h
 */

#ifdef __cplusplus
#define EXTERN extern "C"
  extern "C" {
#else
#define EXTERN extern
#endif

/*-----------------------------------------------------------------------------
 * include files
 *---------------------------------------------------------------------------*/

#include <stdint.h>

/**
 * @addtogroup gnss
 * @{ */

/**
 * @defgroup gnss_output_data GNSS position data
 * Description of GNSS output data of postioning.
 * @{ */
 
/**
 * Max number of satellites
 */
#define GD_MAX_SV_NUM 32

/**
 * @name GNSS satellite system
 * GNSS bit fields to positioning
 * @{ */
#define GD_SAT_NONE    (0)       /**< None */
#define GD_SAT_GPS     (1U << 0) /**< GPS */
#define GD_SAT_GLONASS (1U << 1) /**< Glonass */
#define GD_SAT_SBAS    (1U << 2) /**< SBAS */
#define GD_SAT_QZ_L1CA (1U << 3) /**< QZSS/L1CA */
#define GD_SAT_IMES    (1U << 4) /**< IMES */
#define GD_SAT_QZ_SAIF (1U << 5) /**< QZSS/SAIF */
#define GD_SAT_BEIDOU  (1U << 6) /**< BeiDou */
#define GD_SAT_GALILEO (1U << 7) /**< Galileo */
/* @} */

/**
 * @name GNSS positioning type
 * #GD_PVT_RECEIVER::type
 */
/* @{ */
#define GD_PVT_TYPE_NONE 0 /**< Positioning data none */
#define GD_PVT_TYPE_GNSS 1 /**< by GNSS */
#define GD_PVT_TYPE_IMES 2 /**< by IMES */
#define GD_PVT_TYPE_USER 3 /**< API setting */
/* @} */

/**
 * @name GNSS position fix mode
 * #GD_PVT_RECEIVER::posFixMode
 */
/* @{ */
#define GD_PVT_POSFIX_INVALID 1 /**< No measurement */
#define GD_PVT_POSFIX_2D      2 /**< 2D fix */
#define GD_PVT_POSFIX_3D      3 /**< 3D fix */
/* @} */

/**
 * @name GNSS velocity fix mode
 * #GD_PVT_RECEIVER::velFixMode
 */
/* @{ */
#define GD_PVT_VELFIX_INVALID  1 /**< No measurement */
#define GD_PVT_VELFIX_2DVZ     2 /**< 2D VZ fix */
#define GD_PVT_VELFIX_2DOFFSET 3 /**< 2D Offset fix */
#define GD_PVT_VELFIX_3D       4 /**< 3D fix */
#define GD_PVT_VELFIX_1D       5 /**< 1D fix */
#define GD_PVT_VELFIX_PRED     6 /**< Prediction */
/* @} */

/**
 * @name GNSS oribital infomation data type, almanac & ephemeris
 */
/* @{ */
#define GD_DATA_GPS      0 /**< GPS data type */
#define GD_DATA_GLONASS  1 /**< Glonass data type */
#define GD_DATA_QZSSL1CA 2 /**< QZSS/L1CA data type */
/* @} */

/**
 * @name GNSS satellite status
 * #GD_PVT_SV::stat
 */
/* @{ */
#define GD_SV_STAT_NONE          (0)      /**< None */
#define GD_SV_STAT_TRACKING      (1 << 0) /**< Tracking */
#define GD_SV_STAT_POSITIONING   (1 << 1) /**< Positioning */
#define GD_SV_STAT_CALC_VELOCITY (1 << 2) /**< Calc Velocity */
#define GD_SV_STAT_VISIBLE       (1 << 3) /**< Visible */
#define GD_SV_STAT_SUB_CH        (1 << 4) /**< Sub Ch */
/* @} */

/*
 * @name GNSS 1PPS synchronization status (internal use)
 */
/* @{ */
#define GD_PPS_NOT_ADJUSTED    0 /**< not adjusted */
#define GD_PPS_ADJUSTED        1 /**< adjusted */
#define GD_PPS_ADJUSTED_SSDGLN 2 /**< adjusted SSDGLN */
#define GD_PPS_ADJUSTED_SSD    3 /**< adjusted SSD */
#define GD_PPS_ADJUSTED_POS    4 /**< adjusted POS */
#define GD_PPS_DEGRADE2        5 /**< Degrade2 */
#define GD_PPS_DEGRADE         6 /**< Degrade */
#define GD_PPS_COMPLETE        7 /**< Complete */
/* @} */

/*
 * @name GNSS Output interval of carrier phase info.
 */
/* @{ */
#define GD_RTK_INTERVAL_1HZ  1000 /* 1Hz */
#define GD_RTK_INTERVAL_2HZ  500  /* 2Hz */
#define GD_RTK_INTERVAL_5HZ  200  /* 5Hz */
#define GD_RTK_INTERVAL_10HZ 100  /* 10Hz */
#define GD_RTK_INTERVAL_20HZ 50   /* 20H */
/* @} */

/**
 * @name Carrier phase max satellite number 
 */
/* @{ */
#define GD_RTK_MAX_SV_NUM  24
/* @} */

/*
 * @name GNSS Spectrum data size
 */
/* @{ */
/** Spectrum Data Max(adjusted as GD_SPECTRUM_DATA will be 116byte.)  */

#define GD_SPECTRUM_MAXNUM      37

/** Peak Spectrum Data*/

#define GD_PEAK_SPECTRUM_MAXNUM 3
/* @} */
/** @endcond SPZ_INT_API */

/*
 * @name AGPS Measurement tracking data
 */
/* @{ */
#define GD_SUPL_TRK_DATA_SIZE (16)
/* @} */

/*
 * @name PVTLOG Max stored log number
*/
/* @{ */
#define GD_PVTLOG_MAXNUM        170
/* @} */

/*
 * @name PVTLOG notify threshold of the stored data.
*/
/* @{ */
#define GD_PVTLOG_THRESHOLD_FULL            0 /* Limit of the storage size */
#define GD_PVTLOG_THRESHOLD_HALF            1 /* 1/2 of the Storage size */
#define GD_PVTLOG_THRESHOLD_ONE_DATA        2 /* Each log stored */
/* @} */


/**
 * @name assist bit fields
**/
#define GD_PVT_RECEIVER_ASSIST_NONE   (0x00)
#define GD_PVT_RECEIVER_ASSIST_USER   (0x01)
#define GD_PVT_RECEIVER_ASSIST_CEPPOS (0x02)
#define GD_PVT_RECEIVER_ASSIST_CEPVEL (0x04)
#define GD_PVT_RECEIVER_ASSIST_AEPPOS (0x08)
#define GD_PVT_RECEIVER_ASSIST_AEPVEL (0x10)

/**
 * @name GNSS positionig data elements
 */
/* @{ */
/**
 *  Day (UTC)
 */
typedef struct
{
  uint16_t year;  /**< year */
  uint8_t  month; /**< month */
  uint8_t  day;   /**< day */
} GD_DATE;

/**
 * Time (UTC)
 */
typedef struct
{
  uint8_t  hour;   /**< hour */
  uint8_t  minute; /**< minitue */
  uint8_t  sec;    /**< sec */
  uint32_t usec;   /**< usec */
} GD_TIME;

/**
 * Time (GPS)
 */
typedef struct {
  uint32_t dwTOW;     /**< truncated TOW (1 = 6sec, 0 ... 100799) */
  uint16_t wWN;       /**< week number (0 ... 1023) */
  uint8_t  bSec;      /**< offset (0 ... 5) */
  uint8_t  bRollOver; /**< Number of WN Roll Over (0 ... 255) */
  double   dFrac;     /**< fraction */
} GD_WNTOW;

/**
 * GD_PVT_DOP - Dilution Of Precision
 */
typedef struct
{
  float pDop;  /**< Position DOP */
  float hDop;  /**< Horizontal DOP */
  float vDop;  /**< Vertical DOP */
  float tDop;  /**< Time DOP */
  float ewDop; /**< East-West DOP */
  float nsDop; /**< North-South DOP */
} GD_PVT_DOP;

/**
 * GD_PVT_VAR - Variance
 */
typedef struct
{
  float hVar; /**< Horizontal */
  float vVar; /**< Vertical */
} GD_PVT_VAR;
/* @} */

/* Extra data for debugging */

#define GD_PVT_RECEIVER_EXTRA_DATA_SIZE (448)
#define GD_PVT_RECEIVER_EXTRA_DATA \
  uint8_t extra[GD_PVT_RECEIVER_EXTRA_DATA_SIZE]
#define GD_PVT_SV_EXTRA_DATA_SIZE       36
#define GD_PVT_SV_EXTRA_DATA \
  uint8_t extra[GD_PVT_SV_EXTRA_DATA_SIZE]
#define GD_FFT_MAXPOOLNUM   (8)

/**
 * @name GNSS AGPS clear flag
 */
/* @{ */
#define GD_GCLR_EPH  0x00000001 /**< ephemeris */
#define GD_GCLR_ALM  0x00000002 /**< almanac */
#define GD_GCLR_PV   0x00000004 /**< position and velocity */
#define GD_GCLR_TIME 0x00000008 /**< time */
#define GD_GCLR_TCXO 0x00010000 /**< TCXO offset */
#define GD_GCLR_ALL  0xffffffff /**< all of above */
/* @} */

/**
 * @name GNSS output data
 */
/* @{ */

/**
 * GNSS Receiver data
 */
typedef struct
{
  uint8_t type;           /**< [out] Position type; 0:Invalid, 1:GNSS,
                               2:IMES, 3:user set, 4:previous */
  uint8_t dgps;           /**< [out] FALSE:SGPS, TRUE:DGPS */
  uint8_t posFixMode;     /**< [out] 1:Invalid, 2:2D, 3:3D */
  uint8_t velFixMode;     /**< [out] 1:Invalid, 2:2D VZ, 3:2D Offset,
                               4:3D, 5:1D, 6:PRED */
  uint8_t numSv;          /**< [out] Number of visible satellites */
  uint8_t numSvTracking;  /**< [out] Number of tracking satellites */
  uint8_t numSvCalcPos;   /**< [out] Number of satellites
                               to calculate the position */
  uint8_t numSvCalcVel;   /**< [out] Number of satellites
                               to calculate the velocity */
  uint8_t assist;         /**< [out] bit field [7..5]Reserved [4]AEP Velocity [3]AEP Position [2]CEP Velocity [1]CEP Position, [0]user set */
  uint8_t  posDataExist;  /**< [out] 0:none, 1:exist */
  uint16_t svType;        /**< [out] Using sv system, bit field;
                               bit0:GPS, bit1:GLONASS, bit2:SBAS,
                               bit3:QZSS_L1CA, bit4:IMES,
                               bit5:QZSS_L1SAIF, bit6:Beidu,
                               bit7:Galileo */
  uint16_t svPosType;     /**< [out] using sv system, bit field;
                               bit0:GPS, bit1:GLONASS, bit2:SBAS,
                               bit3:QZSS_L1CA, bit4:IMES,
                               bit5:QZSS_L1SAIF, bit6:Beidu,
                               bit7:Galileo */
  uint16_t svVelType;     /**< [out] using sv system, bit field; bit0:GPS,
                               bit0:GPS, bit1:GLONASS, bit2:SBAS,
                               bit3:QZSS_L1CA, bit4:IMES,
                               bit5:QZSS_L1SAIF, bit6:Beidu,
                               bit7:Galileo */
  uint32_t satSel;        /**< [out] Satellite type; 0:Invalid, 1:GNSS,
                               2:IMES, 3:user set, 4:previous */
  int32_t    tcxoOffset;  /**< [out] TCXO offset[Hz] */
  GD_PVT_DOP posDop;      /**< [out] DOPs of Position */
  GD_PVT_DOP velIdx;      /**< [out] Weighted DOPs of Velocity */
  GD_PVT_VAR posAcc;      /**< [out] Accuracy of Position */
  double     latitude;    /**< [out] Latitude [degree] */
  double     longitude;   /**< [out] Longitude [degree] */
  double     altitude;    /**< [out] Altitude [degree] */
  float      velocity;    /**< [out] Velocity [m/s] */
  float      direction;   /**< [out] Direction [degree] */
  GD_DATE    date;         /**< [out] Current day (UTC) */
  GD_TIME    time;         /**< [out] Current time (UTC) */
  GD_DATE    gpsDate;      /**< [out] Current day (GPS) */
  GD_TIME    gpsTime;      /**< [out] Current time (GPS) */
  GD_TIME    recieveTime;  /**< [out] Receive time (UTC) */
  uint32_t   priv;         /**< [out] For internal use */
  GD_PVT_RECEIVER_EXTRA_DATA; /**< [out] Receiver extra data */
} GD_PVT_RECEIVER;

/**
 * GNSS satellite data
 */
typedef struct
{
  uint16_t type;        /**< [out] Using sv system, bit field; bit0:GPS,
                             bit1:GLONASS, bit2:SBAS, bit3:QZSS_L1CA,
                             bit4:IMES, bit5:QZSS_L1SAIF, bit6:Beidu,
                             bit7:Galileo\n
                             same as GD_PVT_RECEIVER::svType */
  uint8_t svid;         /**< [out] Satellite id */
  uint8_t stat;         /**< Using sv info, bit field; bit0:tracking,
                             bit1:positioning, bit2:calculating velocity,
                             bit3:visible satellite */
  uint8_t elevation;    /**< [out] Elevation [degree] */
  int16_t azimuth;      /**< [out] Azimuth [degree] */
  float   sigLevel;     /**< [out] CN */
  GD_PVT_SV_EXTRA_DATA; /**< [out] Sv extra data */
} GD_PVT_SV;

/**
* Positioning data with SV data
*/
typedef struct
{
  uint64_t dataTimestamp;                /**< [out] Timestamp  */
  uint32_t status;                       /**< [out] Positioning data status\n
                                              0 : Valid, <0: Invalid */
  uint32_t        svdataCount;           /**< [out] Sv data count */
  GD_PVT_RECEIVER receiverData;          /**< [out] Receiver data */
  GD_PVT_SV       svData[GD_MAX_SV_NUM]; /**< [out] Sv data array */
} GD_GNSS_POSITION_DATA;
/* @} */

/* SF_EVENT_GNSS_MEASUREMENT_VALUE */
/**
 * SUPL tracking data
 */
typedef struct
{
  uint8_t gnssId;                   /**< [out] sv system\n
                                         GPS:       0x01\n
                                         GLONASS:   0x02\n
                                         SBAS :     0x04\n
                                         QZSS_L1C/A:0x08 */
  uint8_t signalId;                 /**< [out] Always 0 */
  uint8_t svId;                     /**< [out] Satellite Id
                                         GPS:            1-32\n
                                         GLONASS:        1-24\n
                                         SBAS:           120-158\n
                                         QZSS_L1C/A      193-197 */
  uint8_t  cn;                      /**< [out] CN ratio [dBHz] */
  uint8_t  codePhaseAmbiguty;       /**< Currently version not supported. */
  uint8_t  carriorQualityIngicator; /**< Currently version not supported. */
  uint8_t  codePhaseRmsErr;         /**< Currently version not supported. */
  uint8_t  multiPathIndicator;      /**< Currently version not supported. */
  uint32_t codePhase;               /**< [out] Code Phase[ms]\n
                                         scale: 2-21[ms] */
  uint16_t wholeChip;               /**< [out] Chip integer part */
  uint16_t fracChip;                /**< [out] Chip frac part */
  uint32_t adr;                     /**< Currently version not supported. */
  int16_t  doppler;                 /**< [out] Doppler [Hz] */
} GD_SUPL_TRK_DATA;

/**
* SUPL positioning data
*/
typedef struct
{
  double   uncertaintySemiMajor;   /**< [out] Uncertainty semi-major */
  double   uncertaintySemiMinor;   /**< [out] Uncertainty semi-minor */
  double   orientationOfMajorAxis; /**< [out] Orientation of major axis */
  double   uncertaintyAltitude;    /**< [out] Uncertainty Altitude */
  uint32_t tow;                    /**< [out] Time of week [sec]\n
                                        acquisition TOW : 0-604799\n
                                        no acquisition TOW : 0xffffffff */
  float fracSec;                   /**< [out] Under second part[sec]\n
                                        no acquisition TOW : -1 */
  float horizontalAccuracy;        /**< [out] Horizontal accuracy [m]\n
                                        disable : -1 */
  uint16_t refFrame;               /**< Currently version not supported */
  uint8_t  todUnc;                 /**< [out] Acquisition : 1\n
                                        no acquisition : 0 */
  uint8_t numOfSat;                /**< [out] Tracking Sv number */
} GD_SUPL_POS_DATA;

/**
* SUPL Measurement data struct
*/
typedef struct {
  GD_SUPL_POS_DATA suplPosData; /**< [out] Supl positioning data */
  GD_SUPL_TRK_DATA trackingSat[GD_SUPL_TRK_DATA_SIZE];  /**< [out] Tracking satellite data */
}GD_SUPL_MEASUREMENT_DATA;

/**
 * GD_TIME_TAG - Internal time tag
 */
typedef struct
{
	uint32_t msec;              /**< [ms] whole millisecond part */
	uint32_t frac;              /**< Under millisecond part (0 ... cycle-1) */
	uint16_t cycle;             /**< Resolution of 1ms */
} GD_TIME_TAG;

/** 
 * Time and frequency information for RTK
 */
typedef struct
{
  uint64_t    timestamp;  /**< [out] system timestamp */
  uint64_t    timesnow;   /**< [out] system now times */
  GD_WNTOW    wntow;      /**< [out] GPS time */
  GD_DATE     date;       /**< [out] Date (UTC time) */
  GD_TIME     time;       /**< [out] Time (UTC time) */
  GD_TIME_TAG tag;        /**< [out] TimeTag */
  double      clockDrift; /**< [out] [Hz] clock drift @ 1.5GHz (valid only if
                               cdValidity is 1) */
  int8_t  cdValidity;     /**< [out] ClockDrift validity (0: invalid, 1: valid) */
  uint8_t ppsStatus;      /**< [out] 1PPS synchronization status (GD_PPS_XXX, see
                               above) */
  int8_t svDataCount;     /**< [out] Num of svData */
} GD_RTK_INFO;

/**
 * Carrier phase and related data for RTK
 */
typedef struct
{
  double   pseudoRange;  /**< [out] [m] pseudo range */
  double   carrierPhase; /**< [out] [wave number] carrier phase */
  uint32_t gnss;       /**< [out] GNSS type (GD_GNSS_XXX, see above) */
  int8_t   svid;       /**< [out] Satellite id */
  int8_t   fdmCh;      /**< [out] Frequency slot for GLONASS (-7 ... 6) */
  int16_t  cn;         /**< [out] [0.01dBHz] CN */
  int8_t   polarity;   /**< [out] Carrier polarity
                            (0: not inverted, 1: inverted) */
  int8_t lastPreamble; /**< [out] Parity of last premable (0: ok, 1: ng) */
  int8_t lli;          /**< [out] Lock loss indicator
                            (0: no lock loss, 1: lock loss) */
  int8_t ch;           /**< [out] TRK channel number */
  float    c2p;          /**< [out] C2P (0 ... 1.0) */
  float    doppler;      /**< [out] [Hz] Doppler shift */
} GD_RTK_SV;

/**
 * RTK Carrier phase data
 */
typedef struct {
  GD_RTK_INFO infoOut;                 /**< [out] Time and frequency
                                            information */
  GD_RTK_SV svOut[GD_RTK_MAX_SV_NUM];  /**< [out] Carrier phase and related
                                            data */
} GD_RTK_CARRIER_PHASE;

/**
 * Ephemeris data (GPS)
 */
typedef struct
{
  uint64_t timesnow;   /**< [out] system now times */
  uint8_t  ppsStatus;  /**< [out] 1PPS synchronization status */
  double af0;          /**< [out] SV Clock Correction */
  double af1;          /**< [out] SV Clock Correction */
  double af2;          /**< [out] SV Clock Correction */
  double crs;          /**< [out] Amplitude correction term of orbital
                            radius(sin) */
  double deltaN;       /**< [out] Average motion difference [rad] */
  double m0;           /**< [out] Average near point separation at t_oe
                            [rad] */
  double cuc;          /**< [out] Latitude amplitude correction term(cos) */
  double e;            /**< [out] Eccentricity of orbit */
  double cus;          /**< [out] Latitude amplitude correction term(sin) */
  double sqrtA;        /**< [out] Square root of the orbital length radius */
  double cic;          /**< [out] Amplitude correction term of orbital
                            inclination angle(cos) */
  double Omega0;       /**< [out] Rise of ascension at Weekly Epoch [rad] */
  double cis;          /**< [out] Amplitude correction term of orbital
                            inclination angle(sin) */
  double i0;           /**< [out] Orbital inclination angle at t_oe */
  double crc;          /**< [out] Amplitude correction term of orbital
                            radius(cos) */
  double omega;        /**< [out] Perigee argument [rad] */
  double OmegaDOT;     /**< [out] Ascension of ascending node correction
                            [rad] */
  double iDOT;         /**< [out] Orbital inclination angle correction
                            [rad] */
  double accuracy;     /**< [out] nominal URA (User Range Accuracy) [m] */
  double tgd;          /**< [out] Estimated Group Delay Differential */
  GD_WNTOW tocWntow;   /**< [out] toc */
  GD_DATE tocDate;     /**< [out] toc Date */
  GD_TIME tocTime;     /**< [out] toc Time */
  int32_t toe;         /**< [out] Reference time [s] */
  int32_t tow;         /**< [out] Time of Week (truncated) */
  int16_t id;          /**< [out] Satellite id */
  uint8_t iode;        /**< [out] Issue of Data (Ephemeris) Subframe 2 */
  int8_t codesOnL2;    /**< [out] Code(s) on L2 Channel */
  int16_t weekNumber;  /**< [out] Full week number */
  int8_t l2p;          /**< [out] Data Flag for L2 P-Code */
  uint8_t health;      /**< [out] SV Health (6bit for ephemeris / 8bit for
                            almanac) */
  int16_t iodc;        /**< [out] Issue of Data, Clock (IODC) */
  int8_t fitInterval;  /**< [out] Fit interval flag */
} GD_EPH_GPS;

/**
 * Ephemeris data (GLONASS)
 */
typedef struct {
  uint64_t timesnow;/**< [out] system now times */
  uint32_t valid;   /**< [out] valid */
  uint8_t  ppsStatus;/**< [out] 1PPS synchronization status */
  uint8_t  slot;    /**< [out] slot 1...24 (It generates from svid.
                         Usually same as me->n) */
  int8_t   ch;      /**< [out] ch -7...6 */
  uint8_t  P1;      /**< [out] The difference of t_b from the previous
                         frame */
  uint8_t  tk_h;    /**< [out] Current frame first time (hours) */
  uint8_t  tk_m;    /**< [out] Current frame first time (minutes) */
  uint8_t  tk_s;    /**< [out] Current frame first time (seconds) */
  float    xv;      /**< [out] The velocity vector components of t_b */
  float    xa;      /**< [out] The acceleration components of t_b */
  float    xp;      /**< [out] The position of t_b */
  uint8_t  Bn;      /**< [out] The health info */
  uint8_t  P2;      /**< [out] flag of oddness ("1") or evenness ("0") of
                         the value of t_b */
  uint16_t tb;      /**< [out] Reference time t_b (15...1425) */
  uint8_t  Hn_e;    /**< [out] Carrier frequency number
                         (0...31, (25...31)=(7...-1)) */
  float    yv;      /**< [out] The velocity vector components of t_b */
  float    ya;      /**< [out] The acceleration components of t_b */
  float    yp;      /**< [out] The position of t_b */
  uint8_t  P3;      /**< [out] Number of almanacs in the current frame */
  float    gn;      /**< [out] Carrier frequency relative deviation of t_b */
  uint8_t  P;       /**< [out] Origin of tau variable */
  uint8_t  Health;  /**< [out] Health flag */
  float    zv;      /**< [out] The velocity vector components of t_b */
  float    za;      /**< [out] The acceleration components of t_b */
  float    zp;      /**< [out] The position of t_b */
  float    tn;      /**< [out] Correction to the satellite time t_n relative
                         to GLONASS time t_c */
  float    dtn;     /**< [out] Difference in internal delay between L2 and
                         L1 */
  uint8_t  En;      /**< [out] Number of days from when data was uploaded
                         until t_b (0...31) */
  uint8_t  P4;      /**< [out] Flag of ephemeris parameters updating */
  uint8_t  FT;      /**< [out] The URA (index) of t_b */
  uint16_t NT;      /**< [out] Number of days since 1/1 of a leap year */
  uint8_t  n;       /**< [out] Slot number of the signaling satellite
                         (0...31) */
  uint8_t  M;       /**< [out] Satellite type (0...3) */
} GD_EPH_GLN;

/*
 * Spectrum Data
 */
typedef struct
{
  uint8_t  bStatus;       /**< FFT Sampling Point 0-1 */
  uint8_t  bSamplingStep; /**< FFT Sampling Step  1-16 */
  uint8_t  bSizeMode;     /**< FFT Sampling Num   0:1024 1:512 2:256 */
  uint8_t  bDataCount;    /**< Spectrum data Count  */
  uint8_t  bDataNum;      /**< Spectrum data Size   */
  uint8_t  bIfGain;       /**< IfGain  0-15         */
  uint16_t sDataIndex;    /**< Spectrum data Inex   */
  uint16_t sSpectrum[GD_SPECTRUM_MAXNUM];  /**< Spectrum Data Buffer */
  double   dPeak[GD_PEAK_SPECTRUM_MAXNUM]; /**< Peak Spectrum        */
} GD_SPECTRUM_DATA;

/* @} gnss_output_data */

/**
 * @defgroup gnss_pvtlog_data GNSS PVTLog data
 * Description of GNSS PVTLog data structure.
 * @{ */

/********* PVTLog Parameter ***********/
/**
 * Latitude of PVT data
 */
typedef struct {
  uint32_t frac   :14;  /**< Decimal */
  uint32_t minute :6;   /**< Minute */
  uint32_t degree :7;   /**< Degree */
  uint32_t sign   :1;   /**< Sign */
  uint32_t rsv    :4;   /**< Reserved */
} PVT_DATA_LATITUDE;

/**
 * Longitude of PVT data
 */
typedef struct {
  uint32_t frac   :14;  /**< Decimal */
  uint32_t minute :6;   /**< Minute */
  uint32_t degree :8;   /**< Degree */
  uint32_t sign   :1;   /**< Sign */
  uint32_t rsv    :3;   /**< Reserved */
} PVT_DATA_LONGITUDE;

/**
 * Altitude of PVT data
 */
typedef struct {
  uint32_t frac   :4;   /**< Decimal */
  uint32_t rsv1   :12;  /**< Reserved */
  uint32_t meter  :14;  /**< Integer */
  uint32_t sign   :1;   /**< Sign */
  uint32_t rsv2   :1;   /**< Reserved */
} PVT_DATA_ALTITUDE;

/**
 * Velocity of PVT data
 */
typedef struct {
  uint16_t knot   :14;  /**< Integer */
  uint16_t rsv    :2;   /**< Reserved */
} PVT_DATA_VELOCITY;

/**
 * Direction of PVT data
 */
typedef struct {
  uint16_t frac   :4;   /**< Decimal */
  uint16_t degree :9;   /**< Integer */
  uint16_t rsv    :3;   /**< Reserved */
} PVT_DATA_DIRECTION;

/**
 * Time (UTC) of PVT data
 */
typedef struct {
  uint32_t msec   :7;   /**< msec */
  uint32_t rsv1   :1;   /**< Reserved */
  uint32_t sec    :6;   /**< Second */
  uint32_t rsv2   :2;   /**< Reserved */
  uint32_t minute :6;   /**< Minute */
  uint32_t rsv3   :2;   /**< Reserved */
  uint32_t hour   :5;   /**< Hour */
  uint32_t rsv4   :3;   /**< Reserved */
} PVT_DATA_TIME;

/**
 * Date (UTC) of PVT data
 */
typedef struct {
  uint32_t year   :7;   /**< Year */
  uint32_t day    :5;   /**< Day */
  uint32_t month  :4;   /**< Month */
  uint32_t rsv    :16;  /**< Reserved */
} PVT_DATA_DATE;

/**
 * PVTLog save data struct
 */
typedef struct {
  PVT_DATA_LATITUDE   latitude;   /**< Latitude of data   4B */
  PVT_DATA_LONGITUDE  longitude;  /**< Longitude of data  4B */
  PVT_DATA_ALTITUDE   altitude;   /**< Altitude of data   4B */
  PVT_DATA_VELOCITY   velocity;   /**< Velocity of data   2B */
  PVT_DATA_DIRECTION  direction;  /**< Direction of data  2B */
  PVT_DATA_TIME       time;       /**< Time (UTC)         4B */
  PVT_DATA_DATE       date;       /**< Date (UTC)         4B */
} GD_PVTLOG_DATA;

/*
 * PVTLog notification data struct
 */
typedef struct {
  uint32_t        log_count;                  /* [in] Valid log count of log_data */
  GD_PVTLOG_DATA  log_data[GD_PVTLOG_MAXNUM]; /* [in] Stored log data */
} GD_PVTLOG_DATA_PACKT;

/**
 * PVTLog Status Data
 */
typedef struct
{
  uint32_t        log_count;    /**< [in] Saved log count  */
  PVT_DATA_TIME   start_time;   /**< [in] Time (UTC)   4B  */
  PVT_DATA_DATE   start_date;   /**< [in] Date (UTC)   4B  */
  PVT_DATA_TIME   end_time;     /**< [in] Time (UTC)   4B  */
  PVT_DATA_DATE   end_date;     /**< [in] Date (UTC)   4B  */
}GD_PVTLOG_STATUS;

/* @} gnss_pvtlog_data */

/* @} gnss */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SDK_MODULES_INCLUDE_GPSUTILS_CXD56_GNSS_TYPE_COMPAT_H */
