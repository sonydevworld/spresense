// ==========================================================================
/*!
* @file     CXM150x_Utility.h
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

#ifndef __CXM150x_UTILITY_H
#define __CXM150x_UTILITY_H

#include "CXM150x_APITypeDef.h"

uint32_t conv_CXM150x_GNSSTime_to_second(uint8_t *p_s, uint32_t *p_num);
uint32_t conv_CXM150x_GNSSTime_to_string(uint8_t *p_s, uint8_t *p_date);
uint32_t parse_nmea_sentence_gga(uint8_t *p_nmea_str, CXM150xNMEAGGAInfo *p_gga);
uint32_t parse_nmea_sentence_gll(uint8_t *p_nmea_str, CXM150xNMEAGLLInfo *p_gll);
uint32_t parse_nmea_sentence_gns(uint8_t *p_nmea_str, CXM150xNMEAGNSInfo *p_gns);
uint32_t parse_nmea_sentence_gsa(uint8_t *p_nmea_str, CXM150xNMEAGSAInfo *p_gsa);
uint32_t parse_nmea_sentence_gsv(uint8_t *p_nmea_str, CXM150xNMEAGSVInfo *p_gsv);
uint32_t parse_nmea_sentence_rmc(uint8_t *p_nmea_str, CXM150xNMEARMCInfo *p_rmc);
uint32_t parse_nmea_sentence_vtg(uint8_t *p_nmea_str, CXM150xNMEAVTGInfo *p_vtg);
uint32_t parse_nmea_sentence_zda(uint8_t *p_nmea_str, CXM150xNMEAZDAInfo *p_zda);
uint32_t parse_nmea_sentence_psges(uint8_t *p_nmea_str, CXM150xNMEAPSGESInfo *p_psges);
uint32_t parse_nmea_sentence_psles(uint8_t *p_nmea_str, CXM150xNMEAPSLESInfo *p_psles);
int32_t CXM150x_chk_response_error(uint8_t *str);
int32_t CXM150x_get_last_word(uint8_t *msg,uint8_t *word);
int32_t CXM150x_check_last_ok_ng(uint8_t *msg);
int32_t CXM150x_check_last_on_off(uint8_t *msg);
void CXM150x_ascii_to_bin(uint8_t *ascii,uint8_t *bin,uint32_t ascii_len);
uint32_t CXM150x_get_last_uint32(uint8_t *msg);

#endif // __CXM150x_UTILITY_H

