// ==========================================================================
/*!
* @file     CXM150x_Utility.c
* @brief    utility function
* @date     2021/08/16
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
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "CXM150x_Utility.h"

#define SECONDS_PER_WEEK                (604800)    // 604800=60*60*24*7
#define GET_GPS_FORMAT_UPPER(x)         ((x) >> 20)
#define GET_GPS_FORMAT_LOWER(x)         ((x) & ((1 << 20) - 1))
#define SET_GPS_FORMAT(upper, lower)    (((upper) << 20) + (lower))

#define DATE_BUF_SIZE                   (20)
#define NMEA_FIELD_BUF_SIZE             (32)

typedef enum {
    GGA_SENTENCE_SENTENCE_ID = 0,
    GGA_SENTENCE_UTC,
    GGA_SENTENCE_LAT,
    GGA_SENTENCE_N_S,
    GGA_SENTENCE_LON,
    GGA_SENTENCE_E_W,
    GGA_SENTENCE_POS_STATUS,
    GGA_SENTENCE_SAT_USED,
    GGA_SENTENCE_HDOP,
    GGA_SENTENCE_HEIGHT,
    GGA_SENTENCE_HEIGHT_UNIT,
    GGA_SENTENCE_GEOID,
    GGA_SENTENCE_GEOID_UNIT,
    GGA_SENTENCE_DGPS_DATA,
    GGA_SENTENCE_BS_ID,
    GGA_SENTENCE_CS,
    GGA_SENTENCE_NUM
} nmea_gga_sentence;

typedef enum {
    GLL_SENTENCE_SENTENCE_ID = 0,
    GLL_SENTENCE_LAT,
    GLL_SENTENCE_N_S,
    GLL_SENTENCE_LON,
    GLL_SENTENCE_E_W,
    GLL_SENTENCE_UTC,
    GLL_SENTENCE_POS_STATUS,
    GLL_SENTENCE_MODE,
    GLL_SENTENCE_CS,
    GLL_SENTENCE_NUM
} nmea_gll_sentence;

typedef enum {
    GNS_SENTENCE_SENTENCE_ID = 0,
    GNS_SENTENCE_UTC,
    GNS_SENTENCE_LAT,
    GNS_SENTENCE_N_S,
    GNS_SENTENCE_LON,
    GNS_SENTENCE_E_W,
    GNS_SENTENCE_MODE,
    GNS_SENTENCE_SAT_USED,
    GNS_SENTENCE_HDOP,
    GNS_SENTENCE_HEIGHT,
    GNS_SENTENCE_HEIGHT_M,
    GNS_SENTENCE_GEOID,
    GNS_SENTENCE_GEOID_M,
    GNS_SENTENCE_DGPS_DATA,
    GNS_SENTENCE_BS_ID,
    GNS_SENTENCE_CS,
    GNS_SENTENCE_NUM
} nmea_gns_sentence;

typedef enum {
    GSA_SENTENCE_SENTENCE_ID = 0,
    GSA_SENTENCE_MODE1,
    GSA_SENTENCE_MODE2,
    GSA_SENTENCE_PRN_NUM_1,
    GSA_SENTENCE_PRN_NUM_2,
    GSA_SENTENCE_PRN_NUM_3,
    GSA_SENTENCE_PRN_NUM_4,
    GSA_SENTENCE_PRN_NUM_5,
    GSA_SENTENCE_PRN_NUM_6,
    GSA_SENTENCE_PRN_NUM_7,
    GSA_SENTENCE_PRN_NUM_8,
    GSA_SENTENCE_PRN_NUM_9,
    GSA_SENTENCE_PRN_NUM_10,
    GSA_SENTENCE_PRN_NUM_11,
    GSA_SENTENCE_PRN_NUM_12,
    GSA_SENTENCE_PDOP,
    GSA_SENTENCE_HDOP,
    GSA_SENTENCE_VDOP,
    GSA_SENTENCE_CS,
    GSA_SENTENCE_NUM
} nmea_gsa_sentence;

typedef enum {
    GSV_SENTENCE_SENTENCE_ID = 0,
    GSV_SENTENCE_MSG_TOTAL,
    GSV_SENTENCE_MSG_NUM,
    GSV_SENTENCE_SV_TOTAL,
    GSV_SENTENCE_SV_PRN_NUM_1,
    GSV_SENTENCE_ELEVATION_IN_DEGREES_1,
    GSV_SENTENCE_AZIMUTH_1,
    GSV_SENTENCE_SNR_1,
    GSV_SENTENCE_SV_PRN_NUM_2,
    GSV_SENTENCE_ELEVATION_IN_DEGREES_2,
    GSV_SENTENCE_AZIMUTH_2,
    GSV_SENTENCE_SNR_2,
    GSV_SENTENCE_SV_PRN_NUM_3,
    GSV_SENTENCE_ELEVATION_IN_DEGREES_3,
    GSV_SENTENCE_AZIMUTH_3,
    GSV_SENTENCE_SNR_3,
    GSV_SENTENCE_SV_PRN_NUM_4,
    GSV_SENTENCE_ELEVATION_IN_DEGREES_4,
    GSV_SENTENCE_AZIMUTH_4,
    GSV_SENTENCE_SNR_4,
    GSV_SENTENCE_CS,
    GSV_SENTENCE_NUM
} nmea_gsv_sentence;

typedef enum {
    RMC_SENTENCE_SENTENCE_ID = 0,
    RMC_SENTENCE_UTC,
    RMC_SENTENCE_POS_STATUS,
    RMC_SENTENCE_LAT,
    RMC_SENTENCE_N_S,
    RMC_SENTENCE_LON,
    RMC_SENTENCE_E_W,
    RMC_SENTENCE_SPEED,
    RMC_SENTENCE_DIRECTION,
    RMC_SENTENCE_DATE_UTC,
    RMC_SENTENCE_MAGNETIC_DECLINATION,
    RMC_SENTENCE_MAGNETIC_DECLINATION_DIRECTION,
    RMC_SENTENCE_POS_MODE,
    RMC_SENTENCE_CS,
    RMC_SENTENCE_NUM
} nmea_rmc_sentence;

typedef enum {
    VTG_SENTENCE_SENTENCE_ID = 0,
    VTG_SENTENCE_DEGREES_TRUE,
    VTG_SENTENCE_T,
    VTG_SENTENCE_DEGREES_MAGNETIC,
    VTG_SENTENCE_M,
    VTG_SENTENCE_SPEED_KNOT,
    VTG_SENTENCE_KNOT,
    VTG_SENTENCE_SPEED_KPH,
    VTG_SENTENCE_KPH,
    VTG_SENTENCE_MODE,
    VTG_SENTENCE_CS,
    VTG_SENTENCE_NUM
} nmea_vtg_sentence;

typedef enum {
    ZDA_SENTENCE_SENTENCE_ID = 0,
    ZDA_SENTENCE_UTC,
    ZDA_SENTENCE_DAY,
    ZDA_SENTENCE_MONTH,
    ZDA_SENTENCE_YEAR,
    ZDA_SENTENCE_LOCAL_ZONE_HOURS,
    ZDA_SENTENCE_LOCAL_ZONE_MINUTES,
    ZDA_SENTENCE_CS,
    ZDA_SENTENCE_NUM
} nmea_zda_sentence;

typedef enum {
    PSGES_SENTENCE_SENTENCE_ID = 0,
    PSGES_SENTENCE_MSG_TOTAL,
    PSGES_SENTENCE_MSG_NUM,
    PSGES_SENTENCE_PRN_NUM_1,
    PSGES_SENTENCE_PRN_NUM_2,
    PSGES_SENTENCE_PRN_NUM_3,
    PSGES_SENTENCE_PRN_NUM_4,
    PSGES_SENTENCE_PRN_NUM_5,
    PSGES_SENTENCE_PRN_NUM_6,
    PSGES_SENTENCE_PRN_NUM_7,
    PSGES_SENTENCE_PRN_NUM_8,
    PSGES_SENTENCE_PRN_NUM_9,
    PSGES_SENTENCE_PRN_NUM_10,
    PSGES_SENTENCE_PRN_NUM_11,
    PSGES_SENTENCE_PRN_NUM_12,
    PSGES_SENTENCE_CS,
    PSGES_SENTENCE_NUM
} nmea_psges_sentence;

typedef enum {
    PSLES_SENTENCE_SENTENCE_ID = 0,
    PSLES_SENTENCE_MSG_TOTAL,
    PSLES_SENTENCE_MSG_NUM,
    PSLES_SENTENCE_PRN_NUM_1,
    PSLES_SENTENCE_PRN_NUM_2,
    PSLES_SENTENCE_PRN_NUM_3,
    PSLES_SENTENCE_PRN_NUM_4,
    PSLES_SENTENCE_PRN_NUM_5,
    PSLES_SENTENCE_PRN_NUM_6,
    PSLES_SENTENCE_PRN_NUM_7,
    PSLES_SENTENCE_PRN_NUM_8,
    PSLES_SENTENCE_PRN_NUM_9,
    PSLES_SENTENCE_PRN_NUM_10,
    PSLES_SENTENCE_PRN_NUM_11,
    PSLES_SENTENCE_PRN_NUM_12,
    PSLES_SENTENCE_CS,
    PSLES_SENTENCE_NUM
} nmea_psles_sentence;

// ===========================================================================
//! Calculate NMEA checksum
/*!
 *
 * @param [in] p_nmea_str: NMEA sentence string
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return Calculated checksum (0xFFFFFFFF on error)
*/
// ===========================================================================
static uint32_t calc_nmea_checksum(const uint8_t *p_nmea_str){
    uint32_t ret = 0xFFFFFFFF;
    uint32_t sum = 0;
    
    if(p_nmea_str != NULL){
        // Search for start position ($) of NMEA sentence
        while (*p_nmea_str != '$' && *p_nmea_str != '\0'){
            p_nmea_str++;
        }
        // Start checksum calculation if $ is found
        if(*p_nmea_str == '$'){
            while(1){
                p_nmea_str++;
                
                // If a terminator comes before *, the calculation ends with a checksum error
                if(*p_nmea_str == '\0'){
                    break;
                }
                // If * comes, end checksum calculation
                if(*p_nmea_str == '*'){
                    ret = sum;
                    break;
                }
                // XOR calculation other than '$' and '^'
                if(*p_nmea_str != '$' && *p_nmea_str != '^'){
                    sum ^= *p_nmea_str;
                }
            }
        } else {
            // return NG if the terminator is found before $ is found
            return ret;
        }
    }
    
    return ret;
}

// ===========================================================================
//! Convert GPS time to elapsed seconds
/*!
 *
 * @param [in] p_s: GPS time character string
 * @param [out] p_num: Pointer to a variable that stores the number of seconds elapsed
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t conv_CXM150x_GNSSTime_to_second(uint8_t *p_s, uint32_t *p_num){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint32_t timeword_week;
    uint32_t timeword_sec;
    uint32_t val = 0;

    if (sscanf((char*)p_s, "%lx", &val)) {
        timeword_week = GET_GPS_FORMAT_UPPER(val);
        timeword_sec = GET_GPS_FORMAT_LOWER(val);
        *p_num = ((timeword_week * SECONDS_PER_WEEK) + timeword_sec);
        ret = CXM150x_RESPONSE_OK;
    }
    return ret;
}

// ===========================================================================
//! Convert GPS time to date format
/*!
 *
 * @param [in] p_s: GPS time character string
 * @param [out] p_date: Buffer pointer for storing date string
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t conv_CXM150x_GNSSTime_to_string(uint8_t *p_s, uint8_t *p_date){
    uint32_t ret = CXM150x_RESPONSE_NG;
    struct tm base_time;
    struct tm gps_time;
    time_t gps_time_sec = 0;
    uint32_t timeword_week;
    uint32_t timeword_sec;
    uint32_t val = 0;

    if (sscanf((char*)p_s, "%lx", &val)) {
        memset(&base_time, 0, sizeof(base_time));
        base_time.tm_sec = GPS_FORMAT_BASE_TIME_SEC;
        base_time.tm_min = GPS_FORMAT_BASE_TIME_MIN;
        base_time.tm_hour = GPS_FORMAT_BASE_TIME_HOUR;
        base_time.tm_mday = GPS_FORMAT_BASE_TIME_MDAY;
        base_time.tm_mon = GPS_FORMAT_BASE_TIME_MON;
        base_time.tm_year = GPS_FORMAT_BASE_TIME_YEAR;
        base_time.tm_wday = GPS_FORMAT_BASE_TIME_WDAY;
        base_time.tm_yday = GPS_FORMAT_BASE_TIME_YDAY;
        base_time.tm_isdst = GPS_FORMAT_BASE_TIME_ISDST;

        timeword_week = GET_GPS_FORMAT_UPPER(val);
        timeword_sec = GET_GPS_FORMAT_LOWER(val);
        gps_time_sec = ((timeword_week * SECONDS_PER_WEEK) + timeword_sec);
        gps_time_sec += mktime(&base_time);
        
        if(localtime_r(&gps_time_sec,&gps_time) != NULL){
            strftime((char*)p_date, DATE_BUF_SIZE, "%Y-%m-%d %H:%M:%S", &gps_time);
            ret = CXM150x_RESPONSE_OK;
        } else {
            p_date[0] = '\0';
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA GAA sentences
/*!
 *
 * @param [in] p_nmea_str: GAA sentence string
 * @param [out] p_gga: GAA sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_gga(uint8_t *p_nmea_str, CXM150xNMEAGGAInfo *p_gga){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_gga_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_gga == NULL){
        return ret;
    }
    
    memset(p_gga, 0, sizeof(*p_gga));
    p_base = p_nmea_str;

    for (e_sentence_num = GGA_SENTENCE_SENTENCE_ID; e_sentence_num < GGA_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == GGA_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case GGA_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_gga->m_talker_id, p_gga->m_sentence_id) == 0){
                    memset(p_gga->m_talker_id, 0, sizeof(p_gga->m_talker_id));
                    memset(p_gga->m_sentence_id, 0, sizeof(p_gga->m_sentence_id));
                }
                break;
            case GGA_SENTENCE_UTC:
                // UTC
                if(sscanf((char*)field_str, "%9s", p_gga->m_utc) == 0){
                    memset(p_gga->m_utc, 0, sizeof(p_gga->m_utc));
                }
                break;
            case GGA_SENTENCE_LAT:
                // latitude
                if(sscanf((char*)field_str, "%10s", p_gga->m_lat) == 0){
                    memset(p_gga->m_lat, 0, sizeof(p_gga->m_lat));
                }
                break;
            case GGA_SENTENCE_N_S:
                // North / South latitude
                if(sscanf((char*)field_str, "%1s", p_gga->m_n_s) == 0){
                    memset(p_gga->m_n_s, 0, sizeof(p_gga->m_n_s));
                }
                break;
            case GGA_SENTENCE_LON:
                // longitude
                if(sscanf((char*)field_str, "%10s", p_gga->m_lon) == 0){
                    memset(p_gga->m_lon, 0, sizeof(p_gga->m_lon));
                }
                break;
            case GGA_SENTENCE_E_W:
                // East / West longitude
                if(sscanf((char*)field_str, "%1s", p_gga->m_e_w) == 0){
                    memset(p_gga->m_e_w, 0, sizeof(p_gga->m_e_w));
                }
                break;
            case GGA_SENTENCE_POS_STATUS:
                // GPS Quality indicator
                if(sscanf((char*)field_str, "%ld", &p_gga->m_pos_status) == 0){
                    p_gga->m_pos_status = 0;
                }
                break;
            case GGA_SENTENCE_SAT_USED:
                // Number of satellites used
                if(sscanf((char*)field_str, "%ld", &p_gga->m_sat_used) == 0){
                    p_gga->m_sat_used = 0;
                }
                break;
            case GGA_SENTENCE_HDOP:
                // HDOP
                if(sscanf((char*)field_str, "%f", &p_gga->m_hdop) == 0){
                    p_gga->m_hdop = 0;
                }
                break;
            case GGA_SENTENCE_HEIGHT:
                // Altitude
                if(sscanf((char*)field_str, "%f", &p_gga->m_height) == 0){
                    p_gga->m_height = 0;
                }
                break;
            case GGA_SENTENCE_HEIGHT_UNIT:
                // Altitude unit: M
                if(sscanf((char*)field_str, "%1s", p_gga->m_height_unit) == 0){
                    memset(p_gga->m_height_unit, 0, sizeof(p_gga->m_height_unit));
                }
                break;
            case GGA_SENTENCE_GEOID:
                // geoid height
                if(sscanf((char*)field_str, "%f", &p_gga->m_geoid) == 0){
                    p_gga->m_geoid = 0;
                }
                break;
            case GGA_SENTENCE_GEOID_UNIT:
                // Geoid height unit: M
                if(sscanf((char*)field_str, "%1s", p_gga->m_geoid_unit) == 0){
                    memset(p_gga->m_geoid_unit, 0, sizeof(p_gga->m_geoid_unit));
                }
                break;
            case GGA_SENTENCE_DGPS_DATA:
                // DGPS data
                if(sscanf((char*)field_str, "%9s", p_gga->m_dgps_data) == 0){
                    memset(p_gga->m_dgps_data, 0, sizeof(p_gga->m_dgps_data));
                }
                break;
            case GGA_SENTENCE_BS_ID:
                // base station ID
                if(sscanf((char*)field_str, "%ld", &p_gga->m_bs_id) == 0){
                    p_gga->m_bs_id = 0;
                }
                break;
            case GGA_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_gga->m_cs) == 0){
                    p_gga->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_gga->m_cs){
                        p_gga->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_gga->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA GLL sentences
/*!
 *
 * @param [in] p_nmea_str: GLL sentence string
 * @param [out] p_gll: GLL sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_gll(uint8_t *p_nmea_str, CXM150xNMEAGLLInfo *p_gll){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_gll_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_gll == NULL){
        return ret;
    }
    
    memset(p_gll, 0, sizeof(*p_gll));
    p_base = p_nmea_str;

    for (e_sentence_num = GLL_SENTENCE_SENTENCE_ID; e_sentence_num < GLL_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == GLL_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case GLL_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_gll->m_talker_id, p_gll->m_sentence_id) == 0){
                    memset(p_gll->m_talker_id, 0, sizeof(p_gll->m_talker_id));
                    memset(p_gll->m_sentence_id, 0, sizeof(p_gll->m_sentence_id));
                }
                break;
            case GLL_SENTENCE_LAT:
                // latitude
                if(sscanf((char*)field_str, "%10s", p_gll->m_lat) == 0){
                    memset(p_gll->m_lat,0,sizeof(p_gll->m_lat));
                }
                break;
            case GLL_SENTENCE_N_S:
                // North / South latitude
                if(sscanf((char*)field_str, "%1s", p_gll->m_n_s) == 0){
                    memset(p_gll->m_n_s,0,sizeof(p_gll->m_n_s));
                }
                break;
            case GLL_SENTENCE_LON:
                // longitude
                if(sscanf((char*)field_str, "%10s", p_gll->m_lon) == 0){
                    memset(p_gll->m_lon,0,sizeof(p_gll->m_lon));
                }
                break;
            case GLL_SENTENCE_E_W:
                // East / West longitude
                if(sscanf((char*)field_str, "%1s", p_gll->m_e_w) == 0){
                    memset(p_gll->m_e_w,0,sizeof(p_gll->m_e_w));
                }
                break;
            case GLL_SENTENCE_UTC:
                // UTC
                if(sscanf((char*)field_str, "%9s", p_gll->m_utc) == 0){
                    memset(p_gll->m_utc,0,sizeof(p_gll->m_utc));
                }
                break;
            case GLL_SENTENCE_POS_STATUS:
                // GPS Quality indicator
                if(sscanf((char*)field_str, "%1s", p_gll->m_pos_status) == 0){
                    memset(p_gll->m_pos_status,0,sizeof(p_gll->m_pos_status));
                }
                break;
            case GLL_SENTENCE_MODE:
                // mode
                if(sscanf((char*)field_str, "%1s", p_gll->m_mode) == 0){
                    memset(p_gll->m_mode,0,sizeof(p_gll->m_mode));
                }
                break;
            case GLL_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_gll->m_cs) == 0){
                    p_gll->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_gll->m_cs){
                        p_gll->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_gll->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA GNS sentences
/*!
 *
 * @param [in] p_nmea_str: GNS sentence string
 * @param [out] p_gns: GNS sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_gns(uint8_t *p_nmea_str, CXM150xNMEAGNSInfo *p_gns){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_gns_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_gns == NULL){
        return ret;
    }

    memset(p_gns, 0, sizeof(*p_gns));
    p_base = p_nmea_str;

    for (e_sentence_num = GNS_SENTENCE_SENTENCE_ID; e_sentence_num < GNS_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == GNS_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case GNS_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_gns->m_talker_id, p_gns->m_sentence_id) == 0){
                    memset(p_gns->m_talker_id,0,sizeof(p_gns->m_talker_id));
                    memset(p_gns->m_sentence_id,0,sizeof(p_gns->m_sentence_id));
                }
                break;
            case GNS_SENTENCE_UTC:
                // UTC
                if(sscanf((char*)field_str, "%9s", p_gns->m_utc) == 0){
                    memset(p_gns->m_utc,0,sizeof(p_gns->m_utc));
                }
                break;
            case GNS_SENTENCE_LAT:
                // latitude
                if(sscanf((char*)field_str, "%10s", p_gns->m_lat) == 0){
                    memset(p_gns->m_lat,0,sizeof(p_gns->m_lat));
                }
                break;
            case GNS_SENTENCE_N_S:
                // North / South latitude
                if(sscanf((char*)field_str, "%1s", p_gns->m_n_s) == 0){
                    memset(p_gns->m_n_s,0,sizeof(p_gns->m_n_s));
                }
                break;
            case GNS_SENTENCE_LON:
                // longitude
                if(sscanf((char*)field_str, "%10s", p_gns->m_lon) == 0){
                    memset(p_gns->m_lon,0,sizeof(p_gns->m_lon));
                }
                break;
            case GNS_SENTENCE_E_W:
                // East / West longitude
                if(sscanf((char*)field_str, "%1s", p_gns->m_e_w) == 0){
                    memset(p_gns->m_e_w,0,sizeof(p_gns->m_e_w));
                }
                break;
            case GNS_SENTENCE_MODE:
                // MODE
                if(sscanf((char*)field_str, "%2s", p_gns->m_mode) == 0){
                    memset(p_gns->m_mode,0,sizeof(p_gns->m_mode));
                }
                break;
            case GNS_SENTENCE_SAT_USED:
                // Number of satellites used
                if(sscanf((char*)field_str, "%ld", &p_gns->m_sat_used) == 0){
                    p_gns->m_sat_used = 0;
                }
                break;
            case GNS_SENTENCE_HDOP:
                // HDOP
                if(sscanf((char*)field_str, "%f", &p_gns->m_hdop) == 0){
                    p_gns->m_hdop = 0;
                }
                break;
            case GNS_SENTENCE_HEIGHT:
                // Altitude
                if(sscanf((char*)field_str, "%f", &p_gns->m_height) == 0){
                    p_gns->m_height = 0;
                }
                break;
            case GNS_SENTENCE_HEIGHT_M:
                // skip the item
                break;
            case GNS_SENTENCE_GEOID:
                // geoid height
                if(sscanf((char*)field_str, "%f", &p_gns->m_geoid) == 0){
                    p_gns->m_geoid = 0;
                }
                break;
            case GNS_SENTENCE_GEOID_M:
                // skip the item
                break;
            case GNS_SENTENCE_DGPS_DATA:
                // DGPS data
                if(sscanf((char*)field_str, "%9s", p_gns->m_dgps_data) == 0){
                    memset(p_gns->m_dgps_data,0,sizeof(p_gns->m_dgps_data));
                }
                break;
            case GNS_SENTENCE_BS_ID:
                // base station ID
                if(sscanf((char*)field_str, "%ld", &p_gns->m_bs_id) == 0){
                    p_gns->m_bs_id = 0;
                }
                break;
            case GNS_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_gns->m_cs) == 0){
                    p_gns->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_gns->m_cs){
                        p_gns->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_gns->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA GSA sentence
/*!
 *
 * @param [in] p_nmea_str: GSA sentence string
 * @param [out] p_gsa: GSA sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_gsa(uint8_t *p_nmea_str, CXM150xNMEAGSAInfo *p_gsa){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_gsa_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    uint32_t prn_num_cnt = 0;
    
    if(p_nmea_str == NULL || p_gsa == NULL){
        return ret;
    }

    memset(p_gsa, 0, sizeof(*p_gsa));
    p_base = p_nmea_str;

    for (e_sentence_num = GSA_SENTENCE_SENTENCE_ID; e_sentence_num < GSA_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == GSA_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case GSA_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_gsa->m_talker_id, p_gsa->m_sentence_id) == 0){
                    memset(p_gsa->m_talker_id,0,sizeof(p_gsa->m_talker_id));
                    memset(p_gsa->m_sentence_id,0,sizeof(p_gsa->m_sentence_id));
                }
                break;
            case GSA_SENTENCE_MODE1:
                // MODE1
                if(sscanf((char*)field_str, "%1s", p_gsa->m_mode1) == 0){
                    memset(p_gsa->m_mode1,0,sizeof(p_gsa->m_mode1));
                }
                break;
            case GSA_SENTENCE_MODE2:
                // MODE2
                if(sscanf((char*)field_str, "%1s", p_gsa->m_mode2) == 0){
                    memset(p_gsa->m_mode2,0,sizeof(p_gsa->m_mode2));
                }
                break;
            case GSA_SENTENCE_PRN_NUM_1:
            case GSA_SENTENCE_PRN_NUM_2:
            case GSA_SENTENCE_PRN_NUM_3:
            case GSA_SENTENCE_PRN_NUM_4:
            case GSA_SENTENCE_PRN_NUM_5:
            case GSA_SENTENCE_PRN_NUM_6:
            case GSA_SENTENCE_PRN_NUM_7:
            case GSA_SENTENCE_PRN_NUM_8:
            case GSA_SENTENCE_PRN_NUM_9:
            case GSA_SENTENCE_PRN_NUM_10:
            case GSA_SENTENCE_PRN_NUM_11:
            case GSA_SENTENCE_PRN_NUM_12:
                // Number of satellite PRN numbers
                if(sscanf((char*)field_str, "%ld", &p_gsa->m_prn_num[prn_num_cnt]) == 0){
                    p_gsa->m_prn_num[prn_num_cnt] = 0;
                }
                prn_num_cnt++;
                break;
            case GSA_SENTENCE_PDOP:
                // PDOP
                if(sscanf((char*)field_str, "%f", &p_gsa->m_pdop) == 0){
                    p_gsa->m_pdop = 0;
                }
                break;
            case GSA_SENTENCE_HDOP:
                // HDOP
                if(sscanf((char*)field_str, "%f", &p_gsa->m_hdop) == 0){
                    p_gsa->m_hdop = 0;
                }
                break;
            case GSA_SENTENCE_VDOP:
                // VDOP
                if(sscanf((char*)field_str, "%f", &p_gsa->m_vdop) == 0){
                    p_gsa->m_vdop = 0;
                }
                break;
            case GSA_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_gsa->m_cs) == 0){
                    p_gsa->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_gsa->m_cs){
                        p_gsa->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_gsa->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA GSV sentences
/*!
 *
 * @param [in] p_nmea_str: GSV sentence string
 * @param [out] p_gsv: GSV sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_gsv(uint8_t *p_nmea_str, CXM150xNMEAGSVInfo *p_gsv){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_gsv_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];

    if(p_nmea_str == NULL || p_gsv == NULL){
        return ret;
    }
    
    memset(p_gsv, 0, sizeof(*p_gsv));
    p_base = p_nmea_str;

    for (e_sentence_num = GSV_SENTENCE_SENTENCE_ID; e_sentence_num < GSV_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == GSV_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case GSV_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_gsv->m_talker_id, p_gsv->m_sentence_id) == 0){
                    memset(p_gsv->m_talker_id,0,sizeof(p_gsv->m_talker_id));
                    memset(p_gsv->m_sentence_id,0,sizeof(p_gsv->m_sentence_id));
                }
                break;
            case GSV_SENTENCE_MSG_TOTAL:
                // number of all GSV messages
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_msg_total) == 0){
                    p_gsv->m_msg_total = 0;
                }
                break;
            case GSV_SENTENCE_MSG_NUM:
                // message number
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_msg_num) == 0){
                    p_gsv->m_msg_num = 0;
                }
                break;
            case GSV_SENTENCE_SV_TOTAL:
                // satellites that can be used
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_sv_total) == 0){
                    p_gsv->m_sv_total = 0;
                }
                break;
            case GSV_SENTENCE_SV_PRN_NUM_1:
                // 1: Satellite PRN number
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_sv_prn_num_1) == 0){
                    p_gsv->m_sv_prn_num_1 = 0;
                }
                break;
            case GSV_SENTENCE_ELEVATION_IN_DEGREES_1:
                // 1: satellite elevation angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_elevation_in_degrees_1) == 0){
                    p_gsv->m_elevation_in_degrees_1 = 0;
                }
                break;
            case GSV_SENTENCE_AZIMUTH_1:
                // 1: satellite azimuth angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_azimuth_1) == 0){
                    p_gsv->m_azimuth_1 = 0;
                }
                break;
            case GSV_SENTENCE_SNR_1:
                // 1: satellite SN ratio
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_snr_1) == 0){
                    p_gsv->m_snr_1 = 0;
                }
                break;
            case GSV_SENTENCE_SV_PRN_NUM_2:
                // 2: Satellite PRN number
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_sv_prn_num_2) == 0){
                    p_gsv->m_sv_prn_num_2 = 0;
                }
                break;
            case GSV_SENTENCE_ELEVATION_IN_DEGREES_2:
                // 2: satellite elevation angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_elevation_in_degrees_2) == 0){
                    p_gsv->m_elevation_in_degrees_2 = 0;
                }
                break;
            case GSV_SENTENCE_AZIMUTH_2:
                // 2: satellite azimuth angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_azimuth_2) == 0){
                    p_gsv->m_azimuth_2 = 0;
                }
                break;
            case GSV_SENTENCE_SNR_2:
                // 2: satellite SN ratio
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_snr_2) == 0){
                    p_gsv->m_snr_2 = 0;
                }
                break;
            case GSV_SENTENCE_SV_PRN_NUM_3:
                // 3: Satellite PRN number
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_sv_prn_num_3) == 0){
                    p_gsv->m_sv_prn_num_3 = 0;
                }
                break;
            case GSV_SENTENCE_ELEVATION_IN_DEGREES_3:
                // 3: satellite elevation angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_elevation_in_degrees_3) == 0){
                    p_gsv->m_elevation_in_degrees_3 = 0;
                }
                break;
            case GSV_SENTENCE_AZIMUTH_3:
                // 3: satellite azimuth angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_azimuth_3) == 0){
                    p_gsv->m_azimuth_3 = 0;
                }
                break;
            case GSV_SENTENCE_SNR_3:
                // 3: satellite SN ratio
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_snr_3) == 0){
                    p_gsv->m_snr_3 = 0;
                }
                break;
            case GSV_SENTENCE_SV_PRN_NUM_4:
                // 4: Satellite PRN number
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_sv_prn_num_4) == 0){
                    p_gsv->m_sv_prn_num_4 = 0;
                }
                break;
            case GSV_SENTENCE_ELEVATION_IN_DEGREES_4:
                // 4: satellite elevation angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_elevation_in_degrees_4) == 0){
                    p_gsv->m_elevation_in_degrees_4 = 0;
                }
                break;
            case GSV_SENTENCE_AZIMUTH_4:
                // 4: satellite azimuth angle
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_azimuth_4) == 0){
                    p_gsv->m_azimuth_4 = 0;
                }
                break;
            case GSV_SENTENCE_SNR_4:
                // 4: satellite SN ratio
                if(sscanf((char*)field_str, "%ld", &p_gsv->m_snr_4) == 0){
                    p_gsv->m_snr_4 = 0;
                }
                break;
            case GSV_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_gsv->m_cs) == 0){
                    p_gsv->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_gsv->m_cs){
                        p_gsv->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_gsv->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA RMC sentences
/*!
 *
 * @param [in] p_nmea_str: RMC sentence string
 * @param [out] p_rmc: RMC sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_rmc(uint8_t *p_nmea_str, CXM150xNMEARMCInfo *p_rmc){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_rmc_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_rmc == NULL){
        return ret;
    }

    memset(p_rmc, 0, sizeof(*p_rmc));
    p_base = p_nmea_str;

    for (e_sentence_num = RMC_SENTENCE_SENTENCE_ID; e_sentence_num < RMC_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == RMC_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case RMC_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_rmc->m_talker_id, p_rmc->m_sentence_id) == 0){
                    memset(p_rmc->m_talker_id,0,sizeof(p_rmc->m_talker_id));
                    memset(p_rmc->m_sentence_id,0,sizeof(p_rmc->m_sentence_id));
                }
                break;
            case RMC_SENTENCE_UTC:
                // UTC
                if(sscanf((char*)field_str, "%9s", p_rmc->m_utc) == 0){
                    memset(p_rmc->m_utc,0,sizeof(p_rmc->m_utc));
                }
                break;
            case RMC_SENTENCE_POS_STATUS:
                // status
                if(sscanf((char*)field_str, "%1s", p_rmc->m_pos_status) == 0){
                    memset(p_rmc->m_pos_status,0,sizeof(p_rmc->m_pos_status));
                }
                break;
            case RMC_SENTENCE_LAT:
                // latitude
                if(sscanf((char*)field_str, "%10s", p_rmc->m_lat) == 0){
                    memset(p_rmc->m_lat,0,sizeof(p_rmc->m_lat));
                }
                break;
            case RMC_SENTENCE_N_S:
                // North / South latitude
                if(sscanf((char*)field_str, "%1s", p_rmc->m_n_s) == 0){
                    memset(p_rmc->m_n_s,0,sizeof(p_rmc->m_n_s));
                }
                break;
            case RMC_SENTENCE_LON:
                // longitude
                if(sscanf((char*)field_str, "%10s", p_rmc->m_lon) == 0){
                    memset(p_rmc->m_lon,0,sizeof(p_rmc->m_lon));
                }
                break;
            case RMC_SENTENCE_E_W:
                // East / West longitude
                if(sscanf((char*)field_str, "%1s", p_rmc->m_e_w) == 0){
                    memset(p_rmc->m_e_w,0,sizeof(p_rmc->m_e_w));
                }
                break;
            case RMC_SENTENCE_SPEED:
                // Speed over ground
                if(sscanf((char*)field_str, "%f", &p_rmc->m_speed) == 0){
                    p_rmc->m_speed = 0;
                }
                break;
            case RMC_SENTENCE_DIRECTION:
                // Cource over ground
                if(sscanf((char*)field_str, "%f", &p_rmc->m_direction) == 0){
                    p_rmc->m_direction = 0;
                }
                break;
            case RMC_SENTENCE_DATE_UTC:
                // UTC date
                if(sscanf((char*)field_str, "%6s", p_rmc->m_date_utc) == 0){
                    memset(p_rmc->m_date_utc,0,sizeof(p_rmc->m_date_utc));
                }
                break;
            case RMC_SENTENCE_MAGNETIC_DECLINATION:
                // Magnetic variation
                if(sscanf((char*)field_str, "%ld", &p_rmc->m_magnetic_declination) == 0){
                    p_rmc->m_magnetic_declination = 0;
                }
                break;
            case RMC_SENTENCE_MAGNETIC_DECLINATION_DIRECTION:
                // magnetic variation direction
                if(sscanf((char*)field_str, "%ld", &p_rmc->m_magnetic_declination_direction) == 0){
                    p_rmc->m_magnetic_declination_direction = 0;
                }
                break;
            case RMC_SENTENCE_POS_MODE:
                // positioning mode
                if(sscanf((char*)field_str, "%1s", p_rmc->m_pos_mode) == 0){
                    memset(p_rmc->m_pos_mode,0,sizeof(p_rmc->m_pos_mode));
                }
                break;
            case RMC_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_rmc->m_cs) == 0){
                    p_rmc->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_rmc->m_cs){
                        p_rmc->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_rmc->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze the NMEA VTG sentence
/*!
 *
 * @param [in] p_nmea_str: VTG sentence string
 * @param [out] p_vtg: VTG sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_vtg(uint8_t *p_nmea_str, CXM150xNMEAVTGInfo *p_vtg){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_vtg_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];

    if(p_nmea_str == NULL || p_vtg == NULL){
        return ret;
    }
    
    memset(p_vtg, 0, sizeof(*p_vtg));
    p_base = p_nmea_str;

    for (e_sentence_num = VTG_SENTENCE_SENTENCE_ID; e_sentence_num < VTG_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == VTG_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case VTG_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_vtg->m_talker_id, p_vtg->m_sentence_id) == 0){
                    memset(p_vtg->m_talker_id,0,sizeof(p_vtg->m_talker_id));
                    memset(p_vtg->m_sentence_id,0,sizeof(p_vtg->m_sentence_id));
                }
                break;
            case VTG_SENTENCE_DEGREES_TRUE:
                // Course (Measured heading)
                if(sscanf((char*)field_str, "%f", &p_vtg->m_degrees_true) == 0){
                    p_vtg->m_degrees_true = 0;
                }
                break;
            case VTG_SENTENCE_T:
                // Unit of Cource (Measured heading ) : T
                if(sscanf((char*)field_str, "%1s", p_vtg->m_t) == 0){
                    memset(p_vtg->m_t,0,sizeof(p_vtg->m_t));
                }
                break;
            case VTG_SENTENCE_DEGREES_MAGNETIC:
                // Cource (Measured heading)
                if(sscanf((char*)field_str, "%f", &p_vtg->m_degrees_magnetic) == 0){
                    p_vtg->m_degrees_magnetic = 0;
                }
                break;
            case VTG_SENTENCE_M:
                // Unit of Cource (Measured heading) : M
                if(sscanf((char*)field_str, "%1s", p_vtg->m_m) == 0){
                    memset(p_vtg->m_m,0,sizeof(p_vtg->m_m));
                }
                break;
            case VTG_SENTENCE_SPEED_KNOT:
                // speed [knot]
                if(sscanf((char*)field_str, "%f", &p_vtg->m_speed_knot) == 0){
                    p_vtg->m_speed_knot = 0;
                }
                break;
            case VTG_SENTENCE_KNOT:
                // N: unit of knot
                if(sscanf((char*)field_str, "%1s", p_vtg->m_knot) == 0){
                    memset(p_vtg->m_knot,0,sizeof(p_vtg->m_knot));
                }
                break;
            case VTG_SENTENCE_SPEED_KPH:
                // speed [km / h]
                if(sscanf((char*)field_str, "%f", &p_vtg->m_speed_kph) == 0){
                    p_vtg->m_speed_kph = 0;
                }
                break;
            case VTG_SENTENCE_KPH:
                // speed unit: K
                if(sscanf((char*)field_str, "%1s", p_vtg->m_kph) == 0){
                    memset(p_vtg->m_kph,0,sizeof(p_vtg->m_kph));
                }
                break;
            case VTG_SENTENCE_MODE:
                // Mode
                if(sscanf((char*)field_str, "%1s", p_vtg->m_mode) == 0){
                    memset(p_vtg->m_mode,0,sizeof(p_vtg->m_mode));
                }
                break;
            case VTG_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_vtg->m_cs) == 0){
                    p_vtg->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_vtg->m_cs){
                        p_vtg->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_vtg->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA ZDA sentence
/*!
 *
 * @param [in] p_nmea_str: ZDA sentence string
 * @param [out] p_zda: ZDA sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_zda(uint8_t *p_nmea_str, CXM150xNMEAZDAInfo *p_zda){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_zda_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_zda == NULL){
        return ret;
    }
    
    memset(p_zda, 0, sizeof(*p_zda));
    p_base = p_nmea_str;

    for (e_sentence_num = ZDA_SENTENCE_SENTENCE_ID; e_sentence_num < ZDA_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == ZDA_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case ZDA_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_zda->m_talker_id, p_zda->m_sentence_id) == 0){
                    memset(p_zda->m_talker_id,0,sizeof(p_zda->m_talker_id));
                    memset(p_zda->m_sentence_id,0,sizeof(p_zda->m_sentence_id));
                }
                break;
            case ZDA_SENTENCE_UTC:
                // UTC
                if(sscanf((char*)field_str, "%9s", p_zda->m_utc) == 0){
                    memset(p_zda->m_utc,0,sizeof(p_zda->m_utc));
                }
                break;
            case ZDA_SENTENCE_DAY:
                // Day
                if(sscanf((char*)field_str, "%ld", &p_zda->m_day) == 0){
                    p_zda->m_day = 0;
                }
                break;
            case ZDA_SENTENCE_MONTH:
                // Month
                if(sscanf((char*)field_str, "%ld", &p_zda->m_month) == 0){
                    p_zda->m_month = 0;
                }
                break;
            case ZDA_SENTENCE_YEAR:
                // Year
                if(sscanf((char*)field_str, "%ld", &p_zda->m_year) == 0){
                    p_zda->m_year = 0;
                }
                break;
            case ZDA_SENTENCE_LOCAL_ZONE_HOURS:
                // time (local time zone offset from GMT)
                if(sscanf((char*)field_str, "%ld", &p_zda->m_local_zone_hours) == 0){
                    p_zda->m_local_zone_hours = 0;
                }
                break;
            case ZDA_SENTENCE_LOCAL_ZONE_MINUTES:
                // minute (local time zone offset from GMT)
                if(sscanf((char*)field_str, "%ld", &p_zda->m_local_zone_minutes) == 0){
                    p_zda->m_local_zone_minutes = 0;
                }
                break;
            case ZDA_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_zda->m_cs) == 0){
                    p_zda->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_zda->m_cs){
                        p_zda->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_zda->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA PSGES sentences
/*!
 *
 * @param [in] p_nmea_str: PSGES sentence string
 * @param [out] p_psges: PSGES sentence structure pointer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_psges(uint8_t *p_nmea_str, CXM150xNMEAPSGESInfo *p_psges){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_psges_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_psges == NULL){
        return ret;
    }
    
    memset(p_psges, 0, sizeof(*p_psges));
    p_base = p_nmea_str;

    for (e_sentence_num = PSGES_SENTENCE_SENTENCE_ID; e_sentence_num < PSGES_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == PSGES_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case PSGES_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_psges->m_talker_id, p_psges->m_sentence_id) == 0){
                    memset(p_psges->m_talker_id,0,sizeof(p_psges->m_talker_id));
                    memset(p_psges->m_sentence_id,0,sizeof(p_psges->m_sentence_id));
                }
                break;
            case PSGES_SENTENCE_MSG_TOTAL:
                // number of all PSGES messages
                if(sscanf((char*)field_str, "%ld", &p_psges->m_msg_total) == 0){
                    p_psges->m_msg_total = 0;
                }
                break;
            case PSGES_SENTENCE_MSG_NUM:
                // message number
                if(sscanf((char*)field_str, "%ld", &p_psges->m_msg_num) == 0){
                    p_psges->m_msg_num = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_1:
                // satellite number 1 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_1) == 0){
                    p_psges->m_prn_num_1 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_2:
                // satellite number 2 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_2) == 0){
                    p_psges->m_prn_num_2 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_3:
                // satellite number 3 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_3) == 0){
                    p_psges->m_prn_num_3 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_4:
                // satellite number 4 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_4) == 0){
                    p_psges->m_prn_num_4 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_5:
                // satellite number 5 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_5) == 0){
                    p_psges->m_prn_num_5 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_6:
                // satellite number 6 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_6) == 0){
                    p_psges->m_prn_num_6 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_7:
                // satellite number 7 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_7) == 0){
                    p_psges->m_prn_num_7 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_8:
                // satellite number 8 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_8) == 0){
                    p_psges->m_prn_num_8 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_9:
                // satellite number 9 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_9) == 0){
                    p_psges->m_prn_num_9 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_10:
                // satellite number 10 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_10) == 0){
                    p_psges->m_prn_num_10 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_11:
                // satellite number 11 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_11) == 0){
                    p_psges->m_prn_num_11 = 0;
                }
                break;
            case PSGES_SENTENCE_PRN_NUM_12:
                // satellite number 12 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psges->m_prn_num_12) == 0){
                    p_psges->m_prn_num_12 = 0;
                }
                break;
            case PSGES_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_psges->m_cs) == 0){
                    p_psges->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_psges->m_cs){
                        p_psges->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_psges->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Analyze NMEA PSLES sentence
/*!
 *
 * @param [in] p_nmea_str: PSLES sentence string
 * @param [out] p_psles: Pointer to PSLES sentence structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
uint32_t parse_nmea_sentence_psles(uint8_t *p_nmea_str, CXM150xNMEAPSLESInfo *p_psles){
    uint32_t ret = CXM150x_RESPONSE_NG;
    uint8_t *p_base;
    uint8_t *p_sent;
    uint32_t len;
    nmea_psles_sentence e_sentence_num;
    uint8_t field_str[NMEA_FIELD_BUF_SIZE];
    
    if(p_nmea_str == NULL || p_psles == NULL){
        return ret;
    }
    
    memset(p_psles, 0, sizeof(*p_psles));
    p_base = p_nmea_str;

    for (e_sentence_num = PSLES_SENTENCE_SENTENCE_ID; e_sentence_num < PSLES_SENTENCE_NUM; e_sentence_num++) {
        memset(field_str, '\0', sizeof(field_str));
        p_sent = (uint8_t*)strpbrk((char*)p_base, ",*");
        if (p_sent == NULL) {
            if (e_sentence_num == PSLES_SENTENCE_CS) {
                if(strlen((char*)p_base) < NMEA_CS_SIZE + 2){
                    // If the length is not 4(2digit of the checksum, '\r', '\n'), break due to format error
                    break;
                }
                strncpy((char*)field_str, (char*)p_base, NMEA_CS_SIZE);
                len = NMEA_CS_SIZE;
                field_str[len] = '\0';
            } else {
                break;
            }
        } else {
            len = (uint32_t)(p_sent - p_base);
            if (len < NMEA_FIELD_BUF_SIZE) {
                strncpy((char*)field_str, (char*)p_base, len);
                field_str[len] = '\0';
            } else {
                // do nothing 
            }
            p_base += len + 1;
        }

        if (len) {
            switch (e_sentence_num) {
            case PSLES_SENTENCE_SENTENCE_ID:
                // talker ID, sentence ID
                if(sscanf((char*)field_str, "| $%2s%3s", p_psles->m_talker_id, p_psles->m_sentence_id) == 0){
                    memset(p_psles->m_talker_id,0,sizeof(p_psles->m_talker_id));
                    memset(p_psles->m_sentence_id,0,sizeof(p_psles->m_sentence_id));
                }
                break;
            case PSLES_SENTENCE_MSG_TOTAL:
                // number of all PSLES messages
                if(sscanf((char*)field_str, "%ld", &p_psles->m_msg_total) == 0){
                    p_psles->m_msg_total = 0;
                }
                break;
            case PSLES_SENTENCE_MSG_NUM:
                // message number
                if(sscanf((char*)field_str, "%ld", &p_psles->m_msg_num) == 0){
                    p_psles->m_msg_num = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_1:
                // satellite number 1 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_1) == 0){
                    p_psles->m_prn_num_1 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_2:
                // satellite number 2 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_2) == 0){
                    p_psles->m_prn_num_2 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_3:
                // satellite number 3 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_3) == 0){
                    p_psles->m_prn_num_3 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_4:
                // satellite number 4 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_4) == 0){
                    p_psles->m_prn_num_4 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_5:
                // satellite number 5 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_5) == 0){
                    p_psles->m_prn_num_5 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_6:
                // satellite number 6 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_6) == 0){
                    p_psles->m_prn_num_6 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_7:
                // satellite number 7 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_7) == 0){
                    p_psles->m_prn_num_7 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_8:
                // satellite number 8 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_8) == 0){
                    p_psles->m_prn_num_8 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_9:
                // satellite number 9 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_9) == 0){
                    p_psles->m_prn_num_9 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_10:
                // satellite number 10 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_10) == 0){
                    p_psles->m_prn_num_10 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_11:
                // satellite number 11 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_11) == 0){
                    p_psles->m_prn_num_11 = 0;
                }
                break;
            case PSLES_SENTENCE_PRN_NUM_12:
                // satellite number 12 of the satellite with ephemeris
                if(sscanf((char*)field_str, "%ld", &p_psles->m_prn_num_12) == 0){
                    p_psles->m_prn_num_12 = 0;
                }
                break;
            case PSLES_SENTENCE_CS:
                // Checksum
                if(sscanf((char*)field_str, "%lx", &p_psles->m_cs) == 0){
                    p_psles->m_cs = 0;
                } else{
                    if(calc_nmea_checksum(p_nmea_str) == p_psles->m_cs){
                        p_psles->m_cs_correct = NMEA_CS_OK;
                    } else {
                        p_psles->m_cs_correct = NMEA_CS_NG;
                    }
                }
                ret = CXM150x_RESPONSE_OK;
                break;
            default:
                break;
            }
        }
    }
    return ret;
}

// ===========================================================================
//! Judge whether Error is included in the contents of command response
/*!
 *
 * @param [in] str: Response message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return judgment result
*/
// ===========================================================================
#define CXM150x_COMMAND_ERROR_STR     "Error"
int32_t CXM150x_chk_response_error(uint8_t *str){
    if(strstr((char*)str,CXM150x_COMMAND_ERROR_STR)){
        return CXM150x_RESPONSE_NG;
    }
    
    return CXM150x_RESPONSE_OK;
}

// ===========================================================================
//! Get last word from message
/*! ex. from "<SYS MODE SET 00\r\n", extract only 00
 *
 * @param [in] msg: Response message
 * @param [out] word: The extracted word
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return processing result
*/
// ===========================================================================
int32_t CXM150x_get_last_word(uint8_t *msg,uint8_t *word){
    if(msg == NULL || word == NULL){
        return CXM150x_RESPONSE_NG;
    }

    uint8_t *spc = (uint8_t*)strrchr((char*)msg,' ');
    uint8_t *r = (uint8_t*)strrchr((char*)msg,'\r');
    int32_t len = 0;
    
    if(spc == NULL || r == NULL){
        return CXM150x_RESPONSE_NG;
    }
    
    spc += 1;           // copy from the character following the space
    len = (int32_t)(r-spc);

    if(len < 0){
        return CXM150x_RESPONSE_NG;
    }
    
    strncpy((char*)word,(char*)spc,len);
    word[len] = '\0';
    
    return CXM150x_RESPONSE_OK;
}

// ===========================================================================
//! Determine if the last word ends with OK
/*!
 *
 * @param [in] msg: Response message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return judgment result
*/
// ===========================================================================
int32_t CXM150x_check_last_ok_ng(uint8_t *msg){
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    if(CXM150x_get_last_word(msg,(uint8_t*)buf) == CXM150x_RESPONSE_NG){
        return CXM150x_RESPONSE_NG;
    }
    
    if(strstr((char*)buf,"OK")){
        return CXM150x_RESPONSE_OK;
    }
    return CXM150x_RESPONSE_NG;
}

// ===========================================================================
//! Determine if the last word ends with ON or OFF
/*!
 *
 * @param [in] msg: Response message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return judgment result
*/
// ===========================================================================
int32_t CXM150x_check_last_on_off(uint8_t *msg){
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    if(CXM150x_get_last_word(msg,(uint8_t*)buf) == CXM150x_RESPONSE_NG){
        return EVENT_ON;
    }
    
    if(strstr((char*)buf,"OFF")){
        return EVENT_OFF;
    }
    return EVENT_ON;
}

// ===========================================================================
//! Convert ASCII data to binary data
/*!
 *
 * @param [in] ascii: ASCII data
 * @param [in] ascii_len: ASCII data length
 * @param [out] bin: Binary data storage destination
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
void CXM150x_ascii_to_bin(uint8_t *ascii,uint8_t *bin,uint32_t ascii_len){
    for(uint32_t i=0;i<ascii_len;i+=2){
        uint8_t buf[3] = "";
        buf[0] = ascii[i];
        buf[1] = ascii[i+1];
        uint32_t cnv = 0;
        if(sscanf((char*)buf,"%lx",&cnv) != 0){
            bin[i/2] = (uint8_t)cnv;
        } else {
            bin[i/2] = 0;
        }
    }
}

// ===========================================================================
//! Convert last word to uint32_t
/*!
 *
 * @param [in] msg: message from CXM150x
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return convert result
*/
// ===========================================================================
uint32_t CXM150x_get_last_uint32(uint8_t *msg){
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint32_t ret = 0;

    if(CXM150x_get_last_word(msg,(uint8_t*)buf) == CXM150x_RESPONSE_NG){
        return 0xFFFFFFFF;
    }
    
    if(sscanf((char*)buf,"%ld",&ret) == 0){
        return 0xFFFFFFFF;
    }
    
    return ret;
}


