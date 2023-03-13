// ==========================================================================
/*!
* @file     CXM150x_GNSS.c
* @brief    CXM150x control API (GNSS group command)
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
#include <stdlib.h>

#include "CXM150x_GNSS.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Port.h"
#include "CXM150x_Utility.h"


extern CXM150xGNSSState *g_gnss_state_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_GNSS_state_callback_func_p;
extern CXM150xNMEAGGAInfo *g_nmea_gga_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAGGA_callback_func_p;
extern CXM150xNMEAGLLInfo *g_nmea_gll_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAGLL_callback_func_p;
extern CXM150xNMEAGNSInfo *g_nmea_gns_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAGNS_callback_func_p;
extern CXM150xNMEAGSAInfo *g_nmea_gsa_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAGSA_callback_func_p;
extern CXM150xNMEAGSVInfo *g_nmea_gsv_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAGSV_callback_func_p;
extern CXM150xNMEARMCInfo *g_nmea_rmc_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEARMC_callback_func_p;
extern CXM150xNMEAVTGInfo *g_nmea_vtg_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAVTG_callback_func_p;
extern CXM150xNMEAZDAInfo *g_nmea_zda_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAZDA_callback_func_p;
extern CXM150xNMEAPSGESInfo *g_nmea_psges_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAPSGES_callback_func_p;
extern CXM150xNMEAPSLESInfo *g_nmea_psles_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_NMEAPSLES_callback_func_p;

// ===========================================================================
//! Parse GNSS version information
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_GNSS_firmware_version(uint8_t *response,void *res_buf){
    CmdResGetCXM150xGNSSFirmwareVersion *res = res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        //Error checking
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Parse to <FW version>, <version ID1>, <version ID2>
            memset(res,'\0',sizeof(CmdResGetCXM150xGNSSFirmwareVersion));
            uint8_t *st_pos = NULL;
            uint8_t *ed_pos = NULL;
            int32_t len = 0;

            //version
            st_pos = &response[15];
            ed_pos = (uint8_t*)strchr((char*)st_pos,(int)' ');
            if(ed_pos == NULL){
                return;
            }
            len = (int)ed_pos - (int)st_pos;
            len = len < GNSS_FIRMWARE_VERSION_VERSION_LEN ? len : GNSS_FIRMWARE_VERSION_VERSION_LEN;
            memcpy(res->m_version,st_pos,len);
            
            // Parse ID1
            st_pos = ed_pos+1;
            ed_pos = (uint8_t*)strchr((char*)st_pos,(int)' ');
            if(ed_pos == NULL){
                return;
            }
            len = (int)ed_pos - (int)st_pos;
            len = len < GNSS_FIRMWARE_VERSION_ID1_LEN ? len : GNSS_FIRMWARE_VERSION_ID1_LEN;
            memcpy(res->m_id1,st_pos,len);
            
            //ID2
            st_pos = ed_pos+1;
            ed_pos = (uint8_t*)strchr((char*)st_pos,(int)'\r');
            if(ed_pos == NULL){
                return;
            }
            len = (int)ed_pos - (int)st_pos;
            len = len < GNSS_FIRMWARE_VERSION_ID2_LEN ? len : GNSS_FIRMWARE_VERSION_ID2_LEN;
            memcpy(res->m_id2,st_pos,len);
        } else {
            memset(res,'\0',sizeof(CmdResGetCXM150xGNSSFirmwareVersion));
        }
    }
}
// ===========================================================================
//! Get GNSS version information
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_GNSS_firmware_version(void *param,CmdResGetCXM150xGNSSFirmwareVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS VER GET
    //> GNSS VER GET <FW version> <version ID1> <version ID2>

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_VER,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_GNSS_firmware_version,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_GNSS_firmware_version(response,res);

    return RETURN_OK;
}



// ===========================================================================
//! Register GNSS operation status event notification callback function
/*!
 *
 * @param [in] info: Event data structure
 * @param [in] func: Event notification callback function pointer
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_GNSS_state_callback_func_p: GNSS operation state event notification callback function pointer
 *        [out] g_gnss_state_info: Pointer to GNSS operation state event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_GNSS_state_event(CXM150xGNSSState *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_GNSS_state_callback_func_p;
    g_gnss_state_info = info;
    g_GNSS_state_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Parse the setting result of GNSS operation status event notification
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_set_GNSS_state_event(uint8_t *response,void *res_buf){
    CmdResSetCXM150xGNSSStateEvent *res = (CmdResSetCXM150xGNSSStateEvent*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! GNSS operation status event notification setting
/*!
 *
 * @param [in] on_off: Event notification setting value
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_GNSS_state_event(uint32_t on_off,CmdResSetCXM150xGNSSStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS STT SET_EVT ON
    //> GNSS STT SET_EVT OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    if(on_off == EVENT_ON){
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_STT,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_ON);
    } else {
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_STT,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_OFF);
    }
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_GNSS_state_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_set_GNSS_state_event(response,res);

    return RETURN_OK;
}


// ===========================================================================
//! Parse the setting result of GNSS operation status event notification
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_GNSS_state_event(uint8_t *response,void *res_buf){
    CmdResGetCXM150xGNSSStateEvent *res = (CmdResGetCXM150xGNSSStateEvent*)res_buf;

    // Parse response message from CXM150x
    if(res != NULL){
        //Error checking
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Get the last word out of the message
            uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,last_word) == CXM150x_RESPONSE_OK){
                if(!strcmp((char*)last_word,CXM150x_COMMAND_ON)){
                    res->m_num = EVENT_ON;
                } else if(!strcmp((char*)last_word,CXM150x_COMMAND_OFF)) {
                    res->m_num = EVENT_OFF;
                } else {
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            } else {
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }

}

// ===========================================================================
//! Get GNSS operation status event notification settings
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_GNSS_state_event(void *param,CmdResGetCXM150xGNSSStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS STT GET_EVT
    //> GNSS STT GET_EVT ON

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_STT,CXM150x_COMMAND_GET_EVT);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_GNSS_state_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_get_GNSS_state_event(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the acquisition result of GNSS operation status event
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_GNSS_state_event_info(uint8_t *response,void *res_buf){
    CmdResGetCXM150xGnssStateEventInfo *res = (CmdResGetCXM150xGnssStateEventInfo*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,last_word) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)last_word,"0x%02lx",&res->m_num) == 0){
                    res->m_num = CXM150x_RESPONSE_NG;
                }
            } else {
                res->m_num = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! GNSS operation status event acquisition
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_GNSS_state_event_info(void *param, CmdResGetCXM150xGnssStateEventInfo *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS STT GET
    //> GNSS STT GET 0x01

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_STT,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_GNSS_state_event_info,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_get_GNSS_state_event_info(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Register GGA sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAGGA_callback_func_p: GAA sentence event notification callback function pointer
 *        [out] g_nmea_gga_info: Pointer to GGA sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGGA_event(CXM150xNMEAGGAInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAGGA_callback_func_p;
    g_nmea_gga_info = info;
    g_NMEAGGA_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register GLL sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAGLL_callback_func_p: GLL sentence event notification callback function pointer
 *        [out] g_nmea_gll_info: Pointer to GLL sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGLL_event(CXM150xNMEAGLLInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAGLL_callback_func_p;
    g_nmea_gll_info = info;
    g_NMEAGLL_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register GNS sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAGNS_callback_func_p: GNS sentence event notification callback function pointer
 *        [out] CXM150xNMEAGNSInfo: Pointer to GNS sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGNS_event(CXM150xNMEAGNSInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAGNS_callback_func_p;
    g_nmea_gns_info = info;
    g_NMEAGNS_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register GSA sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAGSA_callback_func_p: GSA sentence event notification callback function pointer
 *        [out] g_nmea_gsa_info: Pointer to GSA sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGSA_event(CXM150xNMEAGSAInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAGSA_callback_func_p;
    g_nmea_gsa_info = info;
    g_NMEAGSA_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register GSV sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAGSV_callback_func_p: GSV sentence event notification callback function pointer
 *        [out] g_nmea_gsv_info: Pointer to GSV sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGSV_event(CXM150xNMEAGSVInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAGSV_callback_func_p;
    g_nmea_gsv_info = info;
    g_NMEAGSV_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register RMC sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEARMC_callback_func_p: RMC sentence event notification callback function pointer
 *        [out] g_nmea_rmc_info: Pointer to RMC sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEARMC_event(CXM150xNMEARMCInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEARMC_callback_func_p;
    g_nmea_rmc_info = info;
    g_NMEARMC_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register VTG sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAVTG_callback_func_p: VTG sentence event notification callback function pointer
 *        [out] g_nmea_vtg_info: Pointer to VTG sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAVTG_event(CXM150xNMEAVTGInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAVTG_callback_func_p;
    g_nmea_vtg_info = info;
    g_NMEAVTG_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register the ZDA sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAZDA_callback_func_p: ZDA sentence event notification callback function pointer
 *        [out] g_nmea_zda_info: Pointer to ZDA sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAZDA_event(CXM150xNMEAZDAInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAZDA_callback_func_p;
    g_nmea_zda_info = info;
    g_NMEAZDA_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register PSGES sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAPSGES_callback_func_p: PSGES sentence event notification callback function pointer
 *        [out] g_nmea_psges_info: Pointer to PSGES sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAPSGES_event(CXM150xNMEAPSGESInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAPSGES_callback_func_p;
    g_nmea_psges_info = info;
    g_NMEAPSGES_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register PSLES sentence event notification callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: Event data structure
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_NMEAPSLES_callback_func_p: PSLES sentence event notification callback function pointer
 *        [out] g_nmea_psles_info: Pointer to PSLES sentence event structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAPSLES_event(CXM150xNMEAPSLESInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_NMEAPSLES_callback_func_p;
    g_nmea_psles_info = info;
    g_NMEAPSLES_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Extract operation status from GNSS operation status event notification
/*!
 *
 * @param [in] msg: Message received from CXM150x
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return GNSS operation status
*/
// ===========================================================================
CXM150xGNSSState conv_gnss_stt_message_to_code(uint8_t *msg){
    uint32_t state = 0;
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    // Get the last word out of the message
    if(CXM150x_get_last_word(msg,buf) == CXM150x_RESPONSE_OK){
        if(sscanf((char*)buf, "%lx", &state) == 0){
            return (CXM150xGNSSState)0x00;
        }
    }
    return (CXM150xGNSSState)state;
}

// ===========================================================================
//! Parse NMEA event notification setting result
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_set_NMEA_event(uint8_t *response,void *res_buf){
    CmdResSetCXM150xNMEAEvent *res = (CmdResSetCXM150xNMEAEvent*)res_buf;
    
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! NMEA event notification settings
/*!
 *
 * @param [in] param: System mode setting value
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_NMEA_event(uint32_t param, CmdResSetCXM150xNMEAEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS HOST_FW_SENT SET_EVT 0x000C
    //> GNSS HOST_FW_SENT SET_EVT OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";

    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s 0x%08lX\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_HOST_FW_SENT,CXM150x_COMMAND_SET_EVT,param);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_NMEA_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_set_NMEA_event(response,res);

    return RETURN_OK;
}


// ===========================================================================
//! Parse NMEA event notification setting acquisition result
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_NMEA_event(uint8_t *response,void *res_buf){
    CmdResGetCXM150xNMEAEvent *res = (CmdResGetCXM150xNMEAEvent*)res_buf;
    
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Get the last word out of the message
            uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,last_word) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)last_word,"0x%08lX",&res->m_num) == 0){
                    res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
                }
            } else {
                res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Get NMEA event notifications
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_NMEA_event(void* param, CmdResGetCXM150xNMEAEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS HOST_FW_SENT GET_EVT
    //> GNSS HOST_FW_SENT GET_EVT 0x0000

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";

    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_HOST_FW_SENT,CXM150x_COMMAND_GET_EVT);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_NMEA_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_get_NMEA_event(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse GNSS position coordinate setting result
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_set_CXM150x_GNSS_position(uint8_t *response,void *res_buf){
    CmdResSetCXM150xGNSSPosition *res = (CmdResSetCXM150xGNSSPosition*)res_buf;
    
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! GNSS position coordinate setting
/*!
 *
 * @param [in] param: Position coordinate structure
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_GNSS_position (CXM150xGNSSPositionSetData *param, CmdResSetCXM150xGNSSPosition *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS GPOE SET 3525.6911,N,13922.2240,E
    //> GNSS GPOE SET OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    if(param == NULL){
        return RETURN_NG;
    }
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s,%s,%s,%s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_GPOE,CXM150x_COMMAND_SET,param->m_lat,param->m_n_s,param->m_lon,param->m_e_w);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_GNSS_position,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_set_CXM150x_GNSS_position(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse GNSS time information setting result
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_set_CXM150x_GNSS_datetime(uint8_t *response,void *res_buf){
    CmdResSetCXM150xGNSSDateTime *res = (CmdResSetCXM150xGNSSDateTime*)res_buf;
    
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! GNSS time information setting
/*!
 *
 * @param [in] param: Time information
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_GNSS_datetime (uint8_t *param, CmdResSetCXM150xGNSSDateTime *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< GNSS GTIM SET 20170901150130
    //> GNSS GTIM SET OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    if(param == NULL){
        return RETURN_NG;
    }

    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_GNSS_GTIM,CXM150x_COMMAND_SET,param);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_GNSS_datetime,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }

    res_check_set_CXM150x_GNSS_datetime(response,res);

    return RETURN_OK;
}




