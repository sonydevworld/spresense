// ==========================================================================
/*!
* @file     CXM150x_APITypeDef.h
* @brief    Define type of API
* @date     2021/12/24
*
* Copyright 2021, 2022 Sony Semiconductor Solutions Corporation
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 
* 3. Neither the name of Sony Semiconductor Solutions Corporation nor the names of
* its contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
// =========================================================================

#ifndef __CXM150x_APITYPEDEF_H
#define __CXM150x_APITYPEDEF_H

#include <nuttx/config.h>
#include <stdint.h>

// API version definition
// Maximum string length of API version
#define CXM150x_API_VERSION_MAX_LEN  (11)
// Character string to be displayed in API version display (up to 11 characters)
#define CXM150x_API_VERSION    "3.0.4"

// Buffer for unprocessed event messages (ring buffer)
#define CXM150x_RECEIVE_EVENT_BUF_SIZE      (10)

// Set whether to compile transmission test mode API (valid if not 0)
#ifdef CONFIG_EXTERNALS_ELTRES_TEST_MODE
#define CXM150x_TEST_MODE_API_USE     (1)
#else
#define CXM150x_TEST_MODE_API_USE     (0)
#endif

// Set whether to compile CONTROL FW UPDATE API (valid if not 0)
#ifdef CONFIG_EXTERNALS_ELTRES_CTRL_FWUPDATE
#define CXM150x_CTRL_FW_UPDATE_API_USE     (1)
#else
#define CXM150x_CTRL_FW_UPDATE_API_USE     (0)
#endif

// Set whether to compile GNSS FW UPDATE API (valid if not 0)
#ifdef CONFIG_EXTERNALS_ELTRES_GNSS_FWUPDATE
#define CXM150x_GNSS_FW_UPDATE_API_USE     (1)
#else
#define CXM150x_GNSS_FW_UPDATE_API_USE     (0)
#endif

// command maximum length
#define CXM150x_MAX_COMMAND_LEN       (128)

// Payload length (16 bytes)
#define CXM150x_PAYLOAD_LEN   (16)

// Maximum message length from CXM150x
#define CXM150x_RECEIVE_BUF_SIZE    (0x80)

// Response error value from CXM150x
#define CXM150x_RESPONSE_ERROR    (-1)

// Maximum length of GNSS Time string from CXM500GE
#define CXM150x_RECEIVE_BUF_GNSSTIME_SIZE    (8)

// CXM150x power ON / OFF state constant
typedef enum {
    CXM150x_POWER_OFF = 0,
    CXM150x_POWER_ON,
    CXM150x_POWER_CONTROL_FW_UPDATE,
    CXM150x_POWER_GNSS_FW_UPDATE
}CXM150x_power_state;

// Mode constant of CXM150x
typedef enum {
    CXM150x_MODE_NORMAL = 0,                  // Normal mode
    CXM150x_MODE_LPWA_TEST_MODE = 1,          // RF Test mode
    CXM150x_MODE_EEPROM_WRITE = 5,            // EEPROM write mode
    CXM150x_MODE_GNSS_FIRMWARE_UPDATE = 7     // GNSS FW update
}CXM150x_mode;

// Define the command response
typedef enum {
    RETURN_NG = 0,
    RETURN_OK,
    RETURN_TIMEOUT,
    RETURN_BUSY
} CXM150x_return_code;

// OK or NG in the command response message from CXM150x
typedef enum {
    CXM150x_RESPONSE_NG = 0,
    CXM150x_RESPONSE_OK
}CXM150x_response_ok_ng;

// Set event notification ON / OFF
typedef enum {
    EVENT_OFF = 0,
    EVENT_ON
}CXM150x_event_on_off;

// NMEA checksum check result
typedef enum {
    NMEA_CS_NG = 0,     // Checksum check result NG
    NMEA_CS_OK          // Checksum inspection result OK
} CXM150x_NMEA_checksum_result;


// Define callback function pointer type
typedef void (* CXM150x_CALLBACK_FUNC_POINTER)(void*,uint32_t); 

// Response callback function pointer type definition
typedef void (* CXM150x_CALLBACK_RESPONSE_FUNC_POINTER)(CXM150x_return_code,void*);

// callback function pointer for response parsing
typedef void (* CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER)(uint8_t*,void*); 

// NMEA related definitions
#define NMEA_TAKER_ID_SIZE       (2)
#define NMEA_SENTENCE_ID_SIZE    (3)
#define NMEA_UTC_SIZE            (9)
#define NMEA_LAT_SIZE            (10)
#define NMEA_N_S_SIZE            (1)
#define NMEA_LON_SIZE            (10)
#define NMEA_E_W_SIZE            (1)
#define NMEA_HEIGHT_UNIT_SIZE    (1)
#define NMEA_GEOID_UNIT_SIZE     (1)
#define NMEA_DGPS_DATA_SIZE      (9)
#define NMEA_STATUS_SIZE         (1)
#define NMEA_DATE_SIZE           (7)
#define NMEA_POS_MODE_SIZE       (1)
#define NMEA_CS_SIZE             (2)
#define NMEA_GNS_MODE_SIZE       (2)
#define NMEA_MODE_SIZE           (1)
#define NMEA_T_SIZE              (1)
#define NMEA_M_SIZE              (1)
#define NMEA_N_SIZE              (1)
#define NMEA_K_SIZE              (1)

#define NMEA_GSA_PRN_MAX         (12)

// Reference date and time difference between struct tm and GPS time (struct tm: January 1, 1900 00:00:00 GPS time: January 6, 1980 00:00:00)
#define GPS_FORMAT_BASE_TIME_SEC        (0)    // second
#define GPS_FORMAT_BASE_TIME_MIN        (0)    // minute
#define GPS_FORMAT_BASE_TIME_HOUR       (0)    // hour
#define GPS_FORMAT_BASE_TIME_MDAY       (6)    // day
#define GPS_FORMAT_BASE_TIME_MON        (0)    // month (January = 0)
#define GPS_FORMAT_BASE_TIME_YEAR       (80)   // year-1900
#define GPS_FORMAT_BASE_TIME_WDAY       (0)    // day of week
#define GPS_FORMAT_BASE_TIME_YDAY       (0)    // day of year
#define GPS_FORMAT_BASE_TIME_ISDST      (0)    // daylight saving time flag

// NMEA GGA related
typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint8_t m_utc[NMEA_UTC_SIZE + 1];                    // UTC
    uint8_t m_lat[NMEA_LAT_SIZE + 1];                    // Latitude
    uint8_t m_n_s[NMEA_N_S_SIZE + 1];                    // N: North latitude, S: South latitude
    uint8_t m_lon[NMEA_LON_SIZE + 1];                    // longitude
    uint8_t m_e_w[NMEA_E_W_SIZE + 1];                    // E: East longitude, W: West longitude
    uint32_t m_pos_status;                               // GPS Quality indicator
    uint32_t m_sat_used;                                 // number of satellites used
    float m_hdop;                                        // HDOP (Horizontal Dilution of Precision)
    float m_height;                                      // altitude
    uint8_t m_height_unit[NMEA_HEIGHT_UNIT_SIZE + 1];    // M: Unit of altitude
    float m_geoid;                                       // geoid height
    uint8_t m_geoid_unit[NMEA_GEOID_UNIT_SIZE + 1];      // M: Unit of geoid height
    uint8_t m_dgps_data[NMEA_DGPS_DATA_SIZE + 1];        // DGPS data
    uint32_t m_bs_id;                                    // base station ID
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAGGAInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint8_t m_lat[NMEA_LAT_SIZE + 1];                    // Latitude
    uint8_t m_n_s[NMEA_N_S_SIZE + 1];                    // N: North latitude, S: South latitude
    uint8_t m_lon[NMEA_LON_SIZE + 1];                    // longitude
    uint8_t m_e_w[NMEA_E_W_SIZE + 1];                    // E: East longitude, W: West longitude
    uint8_t m_utc[NMEA_UTC_SIZE + 1];                    // UTC
    uint8_t m_pos_status[NMEA_STATUS_SIZE + 1];          // status
    uint8_t m_mode[NMEA_MODE_SIZE + 1];                  // mode
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAGLLInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint8_t m_utc[NMEA_UTC_SIZE + 1];                    // UTC
    uint8_t m_lat[NMEA_LAT_SIZE + 1];                    // Latitude
    uint8_t m_n_s[NMEA_N_S_SIZE + 1];                    // N: North latitude, S: South latitude
    uint8_t m_lon[NMEA_LON_SIZE + 1];                    // longitude
    uint8_t m_e_w[NMEA_E_W_SIZE + 1];                    // E: East longitude, W: West longitude
    uint8_t m_mode[NMEA_GNS_MODE_SIZE + 1];              // mode
    uint32_t m_sat_used;                                 // number of satellites used
    float m_hdop;                                        // HDOP (Horizontal Dilution of Precision)
    float m_height;                                      // altitude
    float m_geoid;                                       // geoid height
    uint8_t m_dgps_data[NMEA_DGPS_DATA_SIZE + 1];        // DGPS data
    uint32_t m_bs_id;                                    // base station ID
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAGNSInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint8_t m_mode1[NMEA_MODE_SIZE + 1];                 // MODE 1
    uint8_t m_mode2[NMEA_MODE_SIZE + 1];                 // MODE 2 Fix Type
    uint32_t m_prn_num[NMEA_GSA_PRN_MAX];                // satellite PRN number (up to 12)
    float m_pdop;                                        // PDOP (Position Dilution of Precision): Position accuracy decrease
    float m_hdop;                                        // HDOP (Horizontal Dilution of Precision)
    float m_vdop;                                        // VDOP (Vertical Dilution of Precision): Degree of decrease in vertical precision
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAGSAInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint32_t m_msg_total;                                // number of total GSV messages
    uint32_t m_msg_num;                                  // message number
    uint32_t m_sv_total;                                 // satellites that can be used
    uint32_t m_sv_prn_num_1;                             // 1: Satellite PRN number
    uint32_t m_elevation_in_degrees_1;                   // 1: satellite elevation angle
    uint32_t m_azimuth_1;                                // 1: satellite azimuth angle
    uint32_t m_snr_1;                                    // 1: satellite SN ratio
    uint32_t m_sv_prn_num_2;                             // 2: satellite PRN number
    uint32_t m_elevation_in_degrees_2;                   // 2: satellite elevation angle
    uint32_t m_azimuth_2;                                // 2: satellite azimuth angle
    uint32_t m_snr_2;                                    // 2: satellite SN ratio
    uint32_t m_sv_prn_num_3;                             // 3: Satellite PRN number
    uint32_t m_elevation_in_degrees_3;                   // 3: satellite elevation angle
    uint32_t m_azimuth_3;                                // 3: satellite azimuth angle
    uint32_t m_snr_3;                                    // 3: satellite SN ratio
    uint32_t m_sv_prn_num_4;                             // 4: Satellite PRN number
    uint32_t m_elevation_in_degrees_4;                   // 4: satellite elevation angle
    uint32_t m_azimuth_4;                                // 4: satellite azimuth angle
    uint32_t m_snr_4;                                    // 4: satellite SN ratio
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAGSVInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint8_t m_utc[NMEA_UTC_SIZE + 1];                    // UTC
    uint8_t m_pos_status[NMEA_STATUS_SIZE + 1];          // status
    uint8_t m_lat[NMEA_LAT_SIZE + 1];                    // Latitude
    uint8_t m_n_s[NMEA_N_S_SIZE + 1];                    // N: North latitude, S: South latitude
    uint8_t m_lon[NMEA_LON_SIZE + 1];                    // longitude
    uint8_t m_e_w[NMEA_E_W_SIZE + 1];                    // E: East longitude, W: West longitude
    float m_speed;                                       // travel speed
    float m_direction;                                   // Cource over ground
    uint8_t m_date_utc[NMEA_DATE_SIZE];                  // UTC date
    uint32_t m_magnetic_declination;                     // magnetic deviation
    uint32_t m_magnetic_declination_direction;           // Magnetic variation
    uint8_t m_pos_mode[NMEA_POS_MODE_SIZE + 1];          // positioning mode
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEARMCInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    float m_degrees_true;                                // Course (Measured heading)
    uint8_t m_t[NMEA_T_SIZE + 1];                        // Unit of Cource (Measured heading ) : T
    float m_degrees_magnetic;                            // Cource (Measured heading)
    uint8_t m_m[NMEA_M_SIZE + 1];                        // Unit of Cource (Measured heading) : M
    float m_speed_knot;                                  // speed [knot]
    uint8_t m_knot[NMEA_N_SIZE + 1];                     // N: unit of knot
    float m_speed_kph;                                   // speed [km / h]
    uint8_t m_kph[NMEA_K_SIZE + 1];                      // K: unit of km / h
    uint8_t m_mode[NMEA_MODE_SIZE + 1];                  // mode
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAVTGInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint8_t m_utc[NMEA_UTC_SIZE + 1];                    // UTC
    uint32_t m_day;                                      // day
    uint32_t m_month;                                    // month
    uint32_t m_year;                                     // year
    uint32_t m_local_zone_hours;                         // hours (local time zone offset from GMT)
    uint32_t m_local_zone_minutes;                       // minutes (local time zone offset from GMT)
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAZDAInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint32_t m_msg_total;                                // number of all PSGES messages
    uint32_t m_msg_num;                                  // message number
    uint32_t m_prn_num_1;                                // satellite number 1 of the satellite with ephemeris
    uint32_t m_prn_num_2;                                // satellite number 2 of the satellite with ephemeris
    uint32_t m_prn_num_3;                                // satellite number 3 of the satellite with ephemeris
    uint32_t m_prn_num_4;                                // satellite number 4 of the satellite with ephemeris
    uint32_t m_prn_num_5;                                // satellite number 5 of the satellite with ephemeris
    uint32_t m_prn_num_6;                                // satellite number 6 of the satellite with ephemeris
    uint32_t m_prn_num_7;                                // satellite number 7 of the satellite with ephemeris
    uint32_t m_prn_num_8;                                // satellite number 8 of the satellite with ephemeris
    uint32_t m_prn_num_9;                                // satellite number 9 of the satellite with ephemeris
    uint32_t m_prn_num_10;                               // satellite number 10 of the satellite with ephemeris
    uint32_t m_prn_num_11;                               // satellite number 11 of the satellite with ephemeris
    uint32_t m_prn_num_12;                               // satellite number 12 of the satellite with ephemeris
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAPSGESInfo;

typedef struct {
    uint8_t m_talker_id[NMEA_TAKER_ID_SIZE + 1];         // talker ID
    uint8_t m_sentence_id[NMEA_SENTENCE_ID_SIZE + 1];    // sentence ID
    uint32_t m_msg_total;                                // number of all PSLES messages
    uint32_t m_msg_num;                                  // message number
    uint32_t m_prn_num_1;                                // satellite number 1 of the satellite with ephemeris
    uint32_t m_prn_num_2;                                // satellite number 2 of the satellite with ephemeris
    uint32_t m_prn_num_3;                                // satellite number 3 of the satellite with ephemeris
    uint32_t m_prn_num_4;                                // satellite number 4 of the satellite with ephemeris
    uint32_t m_prn_num_5;                                // satellite number 5 of the satellite with ephemeris
    uint32_t m_prn_num_6;                                // satellite number 6 of the satellite with ephemeris
    uint32_t m_prn_num_7;                                // satellite number 7 of the satellite with ephemeris
    uint32_t m_prn_num_8;                                // satellite number 8 of the satellite with ephemeris
    uint32_t m_prn_num_9;                                // satellite number 9 of the satellite with ephemeris
    uint32_t m_prn_num_10;                               // satellite number 10 of the satellite with ephemeris
    uint32_t m_prn_num_11;                               // satellite number 11 of the satellite with ephemeris
    uint32_t m_prn_num_12;                               // satellite number 12 of the satellite with ephemeris
    uint32_t m_cs;                                       // checksum
    CXM150x_NMEA_checksum_result m_cs_correct;         // Checksum check result
} CXM150xNMEAPSLESInfo;

// LPWA Transmit data update time limit UART event
typedef enum {
    TIME_ALARM,
}CXM150xTimeAlarm;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xPower;

typedef struct {
    uint32_t m_num;
} CmdResGetCXM150xPower;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xMode;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xMode;


#define GNSS_FIRMWARE_VERSION_VERSION_LEN       (17)
#define GNSS_FIRMWARE_VERSION_ID1_LEN           (10)
#define GNSS_FIRMWARE_VERSION_ID2_LEN           (10)
typedef struct {
    uint8_t m_version[GNSS_FIRMWARE_VERSION_VERSION_LEN+1];      // FW version
    uint8_t m_id1[GNSS_FIRMWARE_VERSION_ID1_LEN+1];          // Individual identification data 1
    uint8_t m_id2[GNSS_FIRMWARE_VERSION_ID2_LEN+1];          // Individual identification data 2
} CmdResGetCXM150xGNSSFirmwareVersion;

typedef enum {
    GNSS_STATE,
} CXM150xGNSSState;


typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xGNSSStateEvent;

typedef struct {
    int32_t m_num;    
} CmdResGetCXM150xGNSSStateEvent;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xLPWAPayload;

typedef struct {
    uint8_t m_str[CXM150x_PAYLOAD_LEN];
} CmdResGetCXM150xPrevLPWATxData;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xSysStateEvent;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xSysStateEventInfo;

typedef struct {
    uint8_t m_str[CXM150x_PAYLOAD_LEN];
} CmdResGetCXM150xLPWAPayload;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xSysStateEvent;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTxStateEvent;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTxStateEvent;

typedef struct {
    uint8_t m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE + 1];
} CmdResGetCXM150xPrevLPWATxTime;

typedef struct {
    uint8_t m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE + 1];
} CmdResGetCXM150xNextLPWATxTime;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xNextLPWATxPowerOnLimitTime;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xNextLPWATxRenewLimitTime;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xNextLPWATxTerm;

typedef struct {
    uint8_t m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE + 1];
} CmdResGetCXM150xCurrentGNSSTime;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xFetchingTime;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xWaitFetchingTime;

typedef struct {
    uint32_t m_result;
} CmdResResetCXM150x;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xResetEventInfo;


typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xEEPROMData;

typedef struct{
    int32_t m_offset_address;
    int32_t m_val;
}CXM150xEEPROMSetData;

typedef struct {
    uint32_t m_num;
} CmdResGetCXM150xEEPROMData;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTxProfile;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTxProfile;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xGnssStateEventInfo;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xNMEAEvent;

typedef struct {
    uint32_t m_num;
} CmdResGetCXM150xNMEAEvent;

typedef struct {
    uint8_t m_version[CXM150x_API_VERSION_MAX_LEN+1];
} CmdResGetCXM150xAPIVersion;

typedef struct {
    uint8_t m_lat[NMEA_LAT_SIZE + 1];                    // Latitude
    uint8_t m_n_s[NMEA_N_S_SIZE + 1];                    // N: North latitude, S: South latitude
    uint8_t m_lon[NMEA_LON_SIZE + 1];                    // longitude
    uint8_t m_e_w[NMEA_E_W_SIZE + 1];                    // E: East longitude, W: West longitude
} CXM150xGNSSPositionSetData;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xGNSSBackup;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xGNSSPosition;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xGNSSDateTime;

typedef struct {
    uint32_t m_num;
} CmdResGetCXM150xEEPROMDataSequential;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTxPoCEnable;

typedef enum {
    TX_POC_ENABLE_MESSAGE
} CXM150xTxPoCEnableMessage;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xSysToDeepSleep;

typedef enum {
    TO_DEEPSLEEP_EVENT_TYPE_TO_DSLP = 0,
    TO_DEEPSLEEP_EVENT_TYPE_RESET_DSLP
} CXM150xSysToDeepSleepEventType;

typedef struct {
    CXM150xSysToDeepSleepEventType m_type;
    uint32_t m_sleep_time;
} CXM150xSysToDeepSleepInfo;

typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTxDutyEvent;

typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTxDutyEvent;

typedef struct {
    uint32_t m_result;
    uint8_t m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE + 1];
} CXM150xTxDutyEventInfo;

typedef struct {
    uint8_t m_payload_data[CXM150x_PAYLOAD_LEN];
} CXM150xTxPayloadInfo;

// Definition for test mode
#if CXM150x_TEST_MODE_API_USE
// Response data structure definition for test mode
typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTestTxCh;
typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTestTxCh;
typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTestTxMode;
typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTestTxMode;
typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTestTxRun;
typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTestTxRun;
typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTestGPIState;
typedef struct {
    uint32_t m_result;
} CmdResSetCXM150xTestGPOState;
typedef struct {
    int32_t m_num;
} CmdResGetCXM150xTestGPOState;

// Set transmission test operation status
typedef enum {
    CXM150x_TX_TEST_RUN_OFF = 0,
    CXM150x_TX_TEST_RUN_ON
}CXM150xRunState;

// GPIO port state constant
typedef enum {
    CXM150x_GPIO_PORT_STATE_L = 0,
    CXM150x_GPIO_PORT_STATE_H
} CXM150xGPIOPortState;

// GPI port type constant
typedef enum {
    CXM150x_GPI_PORT_TYPE_WAKEUP = 0
} CXM150xGPIPortType;

// GPO port type constant
typedef enum {
    CXM150x_GPO_PORT_TYPE_INT_OUT1 = 1,
    CXM150x_GPO_PORT_TYPE_INT_OUT2 = 2
} CXM150xGPOPortType;

// Transmission test mode type
typedef enum {
    CXM150x_TEST_TX_MODE_1 = 1,   // transmitting unmodulated continuous signals (frequency deviation), LNA power OFF & STM Sleep
    CXM150x_TEST_TX_MODE_2 = 2,   // carrier sense enabled, transmitting random number data modulation signals continuously and repeatedly 
    CXM150x_TEST_TX_MODE_3 = 3,   // Carrier sense is repeated continuously
    CXM150x_TEST_TX_MODE_4 = 4    // carrier sense enabled, transmitting random number data modulation signals continuously and repeatedly wait after FrameSpan time
} CXM150xTestTxMode;

// GPO port setting structure
typedef struct {
    uint32_t m_port_type;       // port type
    uint32_t m_port_state;      // Port state (H or L)
} CXM150xSetGPOState;


#endif

// CONTROL FW UPDATE definition
#if CXM150x_CTRL_FW_UPDATE_API_USE

// CONTROL FW UPDATE command maximum length (including end)
#define CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN          (250)

// Maximum length of character string to be obtained
#define CTRL_FW_UPDATE_RECEIVE_STR_MAX                    (236)

// Maximum length of binary image transmission
#define CTRL_FW_BINARY_IMAGE_MAX_LEN                      (240)

// Maximum length of binary image transmission (235 bytes only for the first time)
#define CTRL_FW_FIRST_BINARY_IMAGE_MAX_LEN                (235)

typedef struct {
    uint8_t m_str[CTRL_FW_UPDATE_RECEIVE_STR_MAX+1];
} CmdResGetCtrlFWUpdateRoutineName;

typedef struct {
    uint32_t m_num;
} CmdResGetCtrlFWUpdateVersion;

typedef struct {
    uint8_t m_str[CTRL_FW_UPDATE_RECEIVE_STR_MAX+1];
} CmdResGetCtrlFWUpdateCompanyName;

typedef struct {
    uint8_t m_str[CTRL_FW_UPDATE_RECEIVE_STR_MAX+1];
} CmdResGetCtrlFWUpdateDeviceName;

typedef struct {
    int32_t m_num;
} CmdResGetCtrlFWUpdateState;

typedef struct {
    uint32_t m_result;
} CmdResSetCtrlFWUpdateEndRequest;

typedef struct {
    uint32_t m_result;
} CmdResSetCtrlFWUpdateData;

typedef struct {
    uint32_t m_result;
} CmdResSetCtrlFWUpdatePower;

// firmware binary image structure
typedef struct {
    uint32_t m_total_image_len;                     // Firmware binary image length Set only for the first time, set to 0 for the second and subsequent times
    uint32_t m_data_len;                            // Length of divided binary image
    uint8_t m_data[CTRL_FW_BINARY_IMAGE_MAX_LEN];   // Binary data
} CXM150xFWUpdateSetData;

// Update routine state constant
typedef enum {
    CXM150x_CTRL_FW_UPDATE_STATE_INIT_END = 0x96,         // initialization completed
    CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATEING = 0xA0,     // Firmware updating
    CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_OK = 0xB4,     // Firmware update succeeded
    CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_NG = 0xD2,     // Firmware update failed
    CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_INVALID = -1   // acquisition failed
} CXM150xCtrlFWUpdateState;

// Firmware Update End Request type definition
typedef enum {
    CXM150x_CTRL_FW_UPDATE_END_REQUEST_TYPE_A = 0,        // Constant indicating the transferred firmware is A side
    CXM150x_CTRL_FW_UPDATE_END_REQUEST_TYPE_B             // Constant indicating the transferred firmware is B side
} CXM150xFWUpdateType;

// Firmware update power setting structure
typedef struct {
    uint32_t m_on_off;                              // Specify CXM150x power ON / OFF
    CXM150xFWUpdateType m_update_type;            // Specify A side update or B side update
} CXM150xFWUpdateSetPower;

#endif  // CXM150x_CTRL_FW_UPDATE_API_USE

// Definition for GNSS FW UPDATE
#if CXM150x_GNSS_FW_UPDATE_API_USE

// GNSS FW UPDATE_Maximum transmission data length
#define CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN          (256)

// GNSS FW UPDATE_ Maximum length of received data
#define CXM150x_GNSS_FW_UPDATE_RECEIVE_MAX_LEN       (128)

// GNSS FW UPDATE_ header file data length
#define CXM150x_GNSS_FW_UPDATE_SEND_HEADER_LEN       (256)

typedef struct {
    uint32_t m_result;
} CmdResGetGNSSFWUpdateModeCheck;

typedef struct {
    uint32_t m_result;
} CmdResSetGNSSFWUpdateHeaderData;

typedef struct {
    uint32_t m_result;
} CmdResSetGNSSFWUpdateCodeData;

typedef struct {
    uint32_t m_result;
} CmdResGetGNSSFWUpdateResult;

// Constant for transmission data position
typedef enum {
    CODE_DATA_FIRST = 0,    // First data
    CODE_DATA_MID = 1,      // Data in the second or later transmissions except the last one
    CODE_DATA_LAST = 2,      // Last data
} CXM150xDataPosFlag;

// firmware binary image structure
typedef struct {
    CXM150xDataPosFlag m_data_pos_flag;
    uint32_t m_data_len;                            // Length of divided binary image
    uint8_t m_data[CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN + 1];   // Binary data ( maximum transmission data and terminator length )
} CXM150xGNSSFWUpdateCodeData;
#endif  // CXM150x_GNSS_FW_UPDATE_API_USE

// NMEA event notification setting constant
typedef enum {
    NMEA_EVENT_ALL_OFF = 0x00,
    NMEA_EVENT_GGA = 0x01,
    NMEA_EVENT_GLL = 0x02,
    NMEA_EVENT_GSA = 0x04,
    NMEA_EVENT_GSV = 0x08,
    NMEA_EVENT_GNS = 0x10,
    NMEA_EVENT_RMC = 0x20,
    NMEA_EVENT_VTG = 0x40,
    NMEA_EVENT_ZDA = 0x80,
    NMEA_EVENT_PSGES = 0x00400000,
    NMEA_EVENT_PSLES = 0x00800000
} NmeaEventType;


// System state event constant definition
typedef enum{
    SYS_STT_IDLE = 0,
    SYS_STT_FETCHING_TIME ,
    SYS_STT_WAIT_FETCHING_TIME ,
    SYS_STT_EPM_FILL ,
    SYS_STT_WAIT_TX_PREPARE ,
    SYS_STT_AF_TX_PREPARE ,
    SYS_STT_AF_WAIT_TX_START ,
    SYS_STT_AF_TX_PROGRESS ,
    SYS_STT_DF_TX_PREPARE ,
    SYS_STT_DF_WAIT_TX_START ,
    SYS_STT_DF_TX_PROGRESS ,
    SYS_STT_EV_TX_COMPLETE ,
    SYS_STT_GNSS_BACKUP ,
    SYS_STT_GNSS_BACKUP_DONE ,
    SYS_STT_PARSE_ERROR
}CXM150xSysState;

// TxState event
typedef enum{
    TX_PREV_STT_OK = 0,
    TX_PREV_STT_CS_OK,
    TX_PREV_STT_CS_NG,
    TX_PREV_STT_NG,
}CXM150xTxState;


// CXM150x version information command response format (Defining what information is stored from what byte to what byte)
#define SYS_GET_VER_FORMAT_HARDWARE_POS    (14)
#define SYS_GET_VER_FORMAT_HARDWARE_LEN    (24)
#define SYS_GET_VER_FORMAT_VER_NAME_LEN    (6)
#define SYS_GET_VER_FORMAT_COMMITID_LEN    (8)
#define SYS_GET_VER_FORMAT_BUILD_DT_LEN    (20)

// Size definition of version information parse result structure
#define CXM150x_FIRMWARE_VERSION_LEN_ID           (SYS_GET_VER_FORMAT_HARDWARE_LEN / 2)   // 1/2 because ASCII data is converted to binary data
#define CXM150x_FIRMWARE_VERSION_LEN_VERSION      (SYS_GET_VER_FORMAT_VER_NAME_LEN + 1)   // +1 byte for '\0'
#define CXM150x_FIRMWARE_VERSION_LEN_BUILD_DATE   (SYS_GET_VER_FORMAT_BUILD_DT_LEN + 1)   // +1 byte for '\0'

// CXM150x firmware version information
typedef struct {
    uint8_t m_id[CXM150x_FIRMWARE_VERSION_LEN_ID];
    uint8_t m_version[CXM150x_FIRMWARE_VERSION_LEN_VERSION];
    uint32_t m_commit_id;
    uint8_t m_build_date[CXM150x_FIRMWARE_VERSION_LEN_BUILD_DATE];
} CmdResGetCXM150xFirmwareVersion;

typedef enum{
    SYS_RESET_POWER     = 0x0C,    // reset by turning on the power
    SYS_RESET_COMMAND   = 0x14    // Reset by reset command
}CXM150xResetInfo;

typedef enum{
    TX_CUR_FRM_TYPE_PERIODIC_1 = 0,
    TX_CUR_FRM_TYPE_PERIODIC_2,
    TX_CUR_FRM_TYPE_EVENT,
    TX_CUR_FRM_TYPE_PERIODIC
}CXM150xTxProfileType;


// Define event callback ID
typedef enum{
    CXM150x_EVENT_CALLBACK_ID_CXM150x_UART_START_INTERRUPT = 0,   //register_CXM150x_uart_start_interrupt
    CXM150x_EVENT_CALLBACK_ID_SYS_STATE_EVENT,            //register_CXM150x_sys_state_event
    CXM150x_EVENT_CALLBACK_ID_TX_STATE_EVENT,             //register_CXM150x_tx_state_event
    CXM150x_EVENT_CALLBACK_ID_GNSS_STATE_EVENT,           //register_CXM150x_GNSS_state_event
    CXM150x_EVENT_CALLBACK_ID_NMEAGGA_EVENT,              //register_CXM150x_NMEAGGA_event
    CXM150x_EVENT_CALLBACK_ID_NMEAGLL_EVENT,              //register_CXM150x_NMEAGLL_event
    CXM150x_EVENT_CALLBACK_ID_NMEAGNS_EVENT,              //register_CXM150x_NMEAGNS_event
    CXM150x_EVENT_CALLBACK_ID_NMEAGSA_EVENT,              //register_CXM150x_NMEAGSA_event
    CXM150x_EVENT_CALLBACK_ID_NMEAGSV_EVENT,              //register_CXM150x_NMEAGSV_event
    CXM150x_EVENT_CALLBACK_ID_NMEARMC_EVENT,              //register_CXM150x_NMEARMC_event
    CXM150x_EVENT_CALLBACK_ID_NMEAVTG_EVENT,              //register_CXM150x_NMEAVTG_event
    CXM150x_EVENT_CALLBACK_ID_NMEAZDA_EVENT,              //register_CXM150x_NMEAZDA_event
    CXM150x_EVENT_CALLBACK_ID_LPWA_START_INTERRUPT,       //register_CXM150x_LPWA_start_interrupt
    CXM150x_EVENT_CALLBACK_ID_TX_START_MESSAGE_EVENT,     //register_CXM150x_tx_start_message_event
    CXM150x_EVENT_CALLBACK_ID_FATAL_MESSAGE_EVENT,        //register_CXM150x_FATAL_message_event
    CXM150x_EVENT_CALLBACK_ID_EVENT_BUFFER_OVERFLOW,      //register_CXM150x_event_buffer_overflow
    CXM150x_EVENT_CALLBACK_ID_NMEAPSGES_EVENT,            //register_CXM150x_NMEAPSGES_event
    CXM150x_EVENT_CALLBACK_ID_NMEAPSLES_EVENT,            //register_CXM150x_NMEAPSLES_event
    CXM150x_EVENT_CALLBACK_ID_POC_ENABLE_MESSAGE_EVENT,   //register_CXM150x_tx_PoC_enable_message_event
    CXM150x_EVENT_CALLBACK_ID_SYS_TO_DEEP_SLEEP_EVENT,    //register_CXM150x_sys_to_deep_sleep_event
    CXM150x_EVENT_CALLBACK_ID_TX_PLD,                     //register_CXM150x_tx_payload_event
    CXM150x_EVENT_CALLBACK_ID_TX_DUTY,                     //register_CXM150x_tx_duty_event
}CXM150x_EVENT_CALLBACK_ID;

// Command response format of boot loader version information (definition of what information is stored from what byte to what byte)
#define SYS_GET_BTVER_FORMAT_VER_NAME_POS    (16)
#define SYS_GET_BTVER_FORMAT_VER_NAME_LEN    (6)
#define SYS_GET_BTVER_FORMAT_BUILD_DT_LEN    (20)

// Size definition of boot loader version information parse result structure
#define CXM150x_BOOTLOADER_VERSION_LEN_VERSION      (SYS_GET_BTVER_FORMAT_VER_NAME_LEN + 1)   // +1 byte for '\0'
#define CXM150x_BOOTLOADER_VERSION_LEN_BUILD_DATE   (SYS_GET_BTVER_FORMAT_BUILD_DT_LEN + 1)   // +1 byte for '\0'

// CXM150x boot loader version information
typedef struct {
    uint8_t m_version[CXM150x_BOOTLOADER_VERSION_LEN_VERSION];
    uint8_t m_build_date[CXM150x_FIRMWARE_VERSION_LEN_BUILD_DATE];
} CmdResGetCXM150xBootloaderVersion;

// FATAL message event information
typedef struct {
    uint8_t m_str[CXM150x_RECEIVE_BUF_SIZE];
} CXM150xFATALMessage;

// buffer overflow
typedef enum {
    EVENT_BUFFER_OVERFLOW
}CXM150xEventBufferOverflow;

// CXM150x WAKEUP High / Low state constant
typedef enum {
    CXM150x_WAKEUP_L = 0,
    CXM150x_WAKEUP_H
}CXM150x_wakeup_state;

#endif // __CXM150x_APITYPEDEF_H

