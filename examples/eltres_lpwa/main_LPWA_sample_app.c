// ==========================================================================
/*!
* @file     main_LPWA_sample_app.c
* @brief    LPWA sample application
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

/* Includes ------------------------------------------------------------------*/
/*Host Microcomputer dependence include BEGIN */

#include <nuttx/config.h>
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

/*Host Microcomputer dependence include END   */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <assert.h>
#include "CXM150x_APITypeDef.h"
#include "CXM150x_GNSS.h"
#include "CXM150x_SYS.h"
#include "CXM150x_TIME.h"
#include "CXM150x_TX.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Utility.h"
#include "CXM150x_Port.h"
#include "main_LPWA_sample_app.h"

// Sample application name definition
#define SAMPLE_APP_NAME   "main_LPWA_sample_app"
// Sample application version definition
#define SAMPLE_APP_VER    "3.0.3"

// Set to a value other than 0 to enable the GNSS backup function
#define GNSS_BACKUP_USE                     (1)

// Set to a value other than 0 to enable PoC format send(unencrypted message)
#define TX_POC_USE                          (0)

// Set to a value other than 0 to enable TX Duty event
#define TX_DUTY_USE                          (0)

// If the transmission interval is longer than CXM150x_POWER_OFF_INTERVAL_SEC, power off CXM150x (unit is sec)
#define CXM150x_POWER_OFF_INTERVAL_SEC    (5*60)     // 5 min

// Calculate remaining time until next CXM150x power on Offset value
#define EEPROM_POW_ENABLE_REMAIN_OFFSET         (0x0218)
// Periodic1 valid flag
#define EEPROM_DF1_ENABLE                       (0x0400)
// Periodic2 valid flag
#define EEPROM_DF2_ENABLE                       (0x0500)
// Event transmission valid flag
#define EEPROM_EF_ENABLE                        (0x0600)
// AR frame transmission delay time
#define AR_FRAME_DELAY                          (40)
// GGA positioning quality status invalid
#define GGA_FIX_QUALITY_INVALID                 (0)

#if GNSS_BACKUP_USE
// Define GNSS backup interval (unit is seconds)
// Skip backup command issue if elapsed time since last backup is less than defined seconds
#define GNSS_BACKUP_INTERVAL_SEC                (60*60)

// Length of time character string set in GNSS (14 bytes in YYYYMMDDhhmmss format)
#define GNSS_DATETIME_LEN                       (14)

// Last backup time
time_t g_last_gnss_backup_time = 0;

// Timeout for waiting for backup command (unit is msec)
#define WAIT_BACKUP_DONE_MAX_TICK_COUNT         (30000)

// Leap second correction (Leap second as of May 2019)
#define GNSS_DATETIME_ADJUST_LEAP_SECOND        (18)

#endif

// Difference between the base date in the RTC and struct tm
#define RTC_STRUCT_TM_BASE_TIME_DIFF_YEAR            (100)  // RTC is based on 2000, struct tm is based on 1900, so the difference is 100 years
#define RTC_STRUCT_TM_BASE_TIME_DIFF_MONTH           (1)    // RTC starts in month 1, struct tm starts in month 0
#define RTC_STRUCT_TM_BASE_TIME_DIFF_WEEK            (1)    // week of RTC starts at 1, week of struct tm starts at 0

// Signal number for RTC alarm notification
#define RTC_SIGNO                               (1)

#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)

// INT_BUTTON pin assignment
#define INT_BUTTON1_PIN   PIN_SPI4_SCK
#define INT_BUTTON2_PIN   PIN_SPI4_MISO
#define INT_BUTTON3_PIN   PIN_SPI4_MOSI
#define INT_BUTTON4_PIN   PIN_SPI4_CS_X
#define POWER_SDCARD      PMIC_GPO(5)

#endif

// Flag ON / OFF definition
typedef enum {
    FLAG_OFF = 0,
    FLAG_ON
}FlagOnOff;

static uint32_t get_current_tm(void);
static void set_rtc_alarm(int32_t hh,int32_t mm,int32_t ss);
static void wait_gps_data(void);
static void wait_profile_change(void);
static void set_rtc_time(void);
#if GNSS_BACKUP_USE
static time_t get_rtc_time(void);
static void set_GNSS_backup_info(void);
static void GNSS_backup_exec(void);
#endif

// buffer for event information
static CXM150xSysState g_sys_stt_info;
static CXM150xNMEAGGAInfo g_nmea_gga_info_buf;
static CXM150xFATALMessage g_fatalmessage_info;
static CXM150xEventBufferOverflow g_buffer_overflow_info;
static CXM150xTxPoCEnableMessage g_tx_poc_enable_message_info;
#if TX_DUTY_USE
static CXM150xTxDutyEventInfo g_duty_event_info;
#endif

// Payload buffer for LPWA transmission
static uint8_t g_payload_data_all0[CXM150x_PAYLOAD_LEN] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
// Push button interrupt flag
static uint8_t g_push_btn_flag = FLAG_ON;

// Event transmission completion flag
static uint8_t g_ev_tx_complete_flag = FLAG_OFF;
// INT_OUT1 interrupt flag
static uint8_t g_int1_callback_flg = FLAG_OFF;

// transmission completion flag
static uint8_t g_tx_comlete_flg = FLAG_OFF;

// profile change completion flag
static uint8_t g_profile_change_comlete_flg = FLAG_OFF;

// CXM150x power ON offset time
static uint32_t g_pow_enable_remain_offset = 0;

// Event transmission valid flag
static uint32_t g_profile_event_enable = FLAG_OFF;

// periodec valid flag
static uint32_t g_periodec_enable = FLAG_OFF;

// RTC interrupt flag
static uint32_t g_rtc_callback_flg = FLAG_OFF;

// GNSS is avairable if CXM150x status is "EPM_FILL" or later state
static uint32_t g_gnss_ready_flg = FLAG_OFF;

#if GNSS_BACKUP_USE
// GNSS backup completion wait flag
static uint32_t g_gnss_backup_done_flag = FLAG_OFF;
#endif

// GNSS operation state event information buffer
static CXM150xGNSSState g_gnss_stt_info;
// GNSS acquisition timeout flag
static uint8_t g_gnss_timeout_flag = FLAG_OFF;
// GNSS Sleep flag
static uint8_t g_gnss_sleep_flag = FLAG_OFF;
// GGA event information reception flag
static uint8_t g_gga_event_callback_flg = FLAG_OFF;

// ===========================================================================
//! Push button interrupt callback function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_push_btn_flag: Push button interrupt flag
 *
 * @return none
*/
// ===========================================================================
#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
static void push_btn_CXM150x(void){
    if(g_push_btn_flag == FLAG_OFF){
        printf("push_btn_callback!\r\n");
        g_push_btn_flag = FLAG_ON;
    }
}
#endif

// ===========================================================================
//! Callback function when a system state event occurs
/*!
 *
 * @param [in] info: Response data structure
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] g_sys_stt_info: Response data structure
 *        [out] g_gnss_ready_flg: GNSS ready flag
 *        [out] g_tx_comlete_flg: transmission completion flag
 *        [out] g_ev_tx_complete_flag: Event transmission completion flag
 *        [out] g_gnss_timeout_flag: GNSS acquisition timeout flag
 *        [out] g_gnss_backup_done_flag: GNSS backup completion wait flag
 * @return none
*/
// ===========================================================================
static void sys_stt_event_callback(void *info,uint32_t id){

    if(g_sys_stt_info == SYS_STT_IDLE){
        printf("sys_stt_event_callback:code=%d(IDLE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_FETCHING_TIME){
        printf("sys_stt_event_callback:code=%d(FETCHING_TIME)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_WAIT_FETCHING_TIME){
        printf("sys_stt_event_callback:code=%d(WAIT_FETCHING_TIME)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_EPM_FILL){
        printf("sys_stt_event_callback:code=%d(EPM_FILL)\r\n",g_sys_stt_info);
        // Turn on GNSS ready flag and transmission completion flag
        g_gnss_ready_flg = FLAG_ON;
        g_tx_comlete_flg = FLAG_ON;
    } else if(g_sys_stt_info == SYS_STT_WAIT_TX_PREPARE){
        printf("sys_stt_event_callback:code=%d(WAIT_TX_PREPARE)\r\n",g_sys_stt_info);
        g_profile_change_comlete_flg = FLAG_ON;
    } else if(g_sys_stt_info == SYS_STT_AF_TX_PREPARE){
        printf("sys_stt_event_callback:code=%d(AF_TX_PREPARE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_AF_WAIT_TX_START){
        printf("sys_stt_event_callback:code=%d(AF_WAIT_TX_START)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_AF_TX_PROGRESS){
        printf("sys_stt_event_callback:code=%d(AF_TX_PROGRESS)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_DF_TX_PREPARE){
        printf("sys_stt_event_callback:code=%d(DF_TX_PREPARE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_DF_WAIT_TX_START){
        printf("sys_stt_event_callback:code=%d(DF_WAIT_TX_START)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_DF_TX_PROGRESS){
        printf("sys_stt_event_callback:code=%d(DF_TX_PROGRESS)\r\n",g_sys_stt_info);
        printf("*** START LPWA TX: Stop get GNSS! ***\r\n");
        // Turn on GNSS acquisition timeout flag
        g_gnss_timeout_flag = FLAG_ON;
    } else if(g_sys_stt_info == SYS_STT_EV_TX_COMPLETE){
        printf("sys_stt_event_callback:code=%d(SYS_STT_EV_TX_COMPLETE)\r\n",g_sys_stt_info);
        // Turn on the event transmission completion flag
        g_ev_tx_complete_flag = FLAG_ON;
    } else if(g_sys_stt_info == SYS_STT_GNSS_BACKUP){
        printf("sys_stt_event_callback:code=%d(SYS_STT_GNSS_BACKUP)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_GNSS_BACKUP_DONE){
        printf("sys_stt_event_callback:code=%d(SYS_STT_GNSS_BACKUP_DONE)\r\n",g_sys_stt_info);
#if GNSS_BACKUP_USE
        // Turn on the GNSS backup completion wait flag
        g_gnss_backup_done_flag = FLAG_ON;
#endif
    } else{
        printf("sys_stt_event_callback:code=%d(PURSE_ERROR)\r\n",g_sys_stt_info);
    }

}

// ===========================================================================
//! Callback function when a GNSS operation state event occurs
/*!
 *
 * @param [in] info: GNSS operation state event information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] g_gnss_stt_info: GNSS operation state event information buffer
 *        [out] g_gnss_sleep_flag: GNSS Sleep flag
 * @return none
*/
// ===========================================================================
static void gnss_stt_event_callback(void *info,uint32_t id){

    if(g_gnss_stt_info & 0x80){   // Sleeping
        g_gnss_sleep_flag = FLAG_ON;
        printf("*** GNSS_SLEEP ***\r\n");
    } else {
        g_gnss_sleep_flag = FLAG_OFF;
    }
}

// ===========================================================================
//! Callback function when a GGA event occurs
/*!
 *
 * @param [in] info: GGA information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_gga_event_callback_flg: GGA event information reception flag
 * @return none
*/
// ===========================================================================
static void nmea_gga_event_callback(void *info,uint32_t id){
    CXM150xNMEAGGAInfo *gga_inf = (CXM150xNMEAGGAInfo*)info;
    if(gga_inf->m_utc[0] == '\0'){
        printf("GGA: not ready\r\n");
    } else {
        printf("GGA: time[%s] lat[%s %s] lng[%s %s] cs_correct[%d]\r\n",gga_inf->m_utc,gga_inf->m_n_s,gga_inf->m_lat,gga_inf->m_e_w,gga_inf->m_lon,g_nmea_gga_info_buf.m_cs_correct);
        g_gga_event_callback_flg = FLAG_ON;
    }
    
}

// ===========================================================================
//! Callback function for INT_OUT1 interrupt
/*!
 *
 * @param [in] msg: NULL fixed
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_int1_callback_flg: INT_OUT1 callback flag
 *
 * @return none
*/
// ===========================================================================
static void int1_callback(void* msg,uint32_t id){
    printf("int1_callback\r\n");
    g_int1_callback_flg = FLAG_ON;
}

// ===========================================================================
//! Callback function for INT_OUT2 interrupt
/*!
 *
 * @param [in] msg: NULL fixed
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void int2_callback(void* msg,uint32_t id){
    printf("int2_callback\r\n");
}

// ===========================================================================
//! Callback function when FATAL message event occurs
/*!
 *
 * @param [in] info: Abnormal event information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void fatal_message_event_callback(void *info,uint32_t id){
    CXM150xFATALMessage *fatal_info = (CXM150xFATALMessage*)info;
    printf("FATAL Message Event: %s\r\n",fatal_info->m_str);
}

// ===========================================================================
//! resume CXM150x
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_gnss_ready_flg: GNSS ready flag
 *        [out] g_rtc_callback_flg: RTC interrupt flag
 * @return none
*/
// ===========================================================================
static void resume_CXM150x(void){
    
    // Get power supply status
    CmdResGetCXM150xPower pwstate;
    get_CXM150x_power(NULL,&pwstate,NULL);
    
    // resume if the power is off
    if(pwstate.m_num == CXM150x_POWER_OFF){
        // Power ON and set normal mode
        g_gnss_ready_flg = FLAG_OFF;
        board_gpio_intconfig(ELTRES_PIN_INT_OUT1, INT_RISING_EDGE, false, (xcpt_t)wrapper_CXM150x_int_out1);
        board_gpio_int(ELTRES_PIN_INT_OUT1, true);

        CmdResSetCXM150xPower res_set_power;
        memset(&res_set_power,0,sizeof(res_set_power));
        set_CXM150x_power(CXM150x_POWER_ON,&res_set_power,NULL);
#if GNSS_BACKUP_USE
        // Set information for GNSS assist function (BUP)
        set_GNSS_backup_info();
#endif
#if TX_POC_USE
        CmdResSetCXM150xTxPoCEnable res_set_tx_poc_enable;
        memset(&res_set_tx_poc_enable,0,sizeof(res_set_tx_poc_enable));
        set_CXM150x_tx_PoC_enable(NULL,&res_set_tx_poc_enable,NULL);
#endif
#if TX_DUTY_USE
        CmdResSetCXM150xTxDutyEvent res_set_tx_duty_event;
        memset(&res_set_tx_duty_event,0,sizeof(res_set_tx_duty_event));
        set_CXM150x_tx_duty_event(EVENT_ON,&res_set_tx_duty_event,NULL);
#endif
        CmdResSetCXM150xMode res_set_mode;
        memset(&res_set_mode,0,sizeof(res_set_mode));
        set_CXM150x_mode(CXM150x_MODE_NORMAL,&res_set_mode,NULL);
        if(g_push_btn_flag == FLAG_OFF){
            // GPS data acquisition
            wait_gps_data();
            
            g_rtc_callback_flg = FLAG_OFF;
        }
    } else {
        while(get_CXM150x_Rx_message_count()>0){
            analyse_CXM150x_Rx();
        }
    }
}

// ===========================================================================
//! Change transmission profile, execute sleep process
/*!
 *
 * @param [in] next_time: Next transmission profile
 * @param [out] none
 * @par Global variable
 *        [in] g_pow_enable_remain_offset: CXM150x power ON offset time
 *        [out] g_gnss_ready_flg: GNSS ready flag
 *        [out] g_int1_callback_flg:  INT_OUT1 interrupt flag 
 *        [out] g_nmea_gga_info_buf: GGA information buffer
 * @return none
*/
// ===========================================================================
static void power_off_check(void){
    uint32_t c_time = 0;
    uint32_t next_time = 0;
    struct tm c_tm;
    struct tm n_tm;
    CmdResGetCXM150xNextLPWATxTerm res_next_term;
    CmdResGetCXM150xTxProfile res_profile;
    CXM150xTxProfileType current_profile_type;

    // If Periodic is invalid, judgment is unnecessary because only event transmission is performed
    if(g_periodec_enable == FLAG_OFF){
        printf("+++++time check start+++++\r\n");
        set_rtc_time();
#if GNSS_BACKUP_USE
        GNSS_backup_exec();
#endif
        printf("event only mode\r\n");
        printf("CXM150x power off\r\n");
        board_gpio_int(ELTRES_PIN_INT_OUT1, false);

        CmdResSetCXM150xPower res_set_power;
        memset(&res_set_power,0,sizeof(res_set_power));
        set_CXM150x_power(CXM150x_POWER_OFF,&res_set_power,NULL);
        g_gnss_ready_flg = FLAG_OFF;
        g_int1_callback_flg = FLAG_OFF;
        printf("+++++time check end  +++++\r\n");
        
        return;
    }

    // get current profile
    if(get_CXM150x_tx_profile(NULL,&res_profile,NULL) == RETURN_OK){
        current_profile_type = (CXM150xTxProfileType)res_profile.m_num;
    } else {
        current_profile_type = TX_CUR_FRM_TYPE_EVENT;
    }
    
    if(current_profile_type == TX_CUR_FRM_TYPE_EVENT){
        // If profile switching is required
        CmdResSetCXM150xTxProfile res_tx_profile;
        memset(&res_tx_profile,0,sizeof(res_tx_profile));
        set_CXM150x_tx_profile(TX_CUR_FRM_TYPE_PERIODIC,&res_tx_profile,NULL);

        // After changing profiles, wait for transition to EPM_FILL or WAIT_TX_PREPARE
        g_profile_change_comlete_flg = FLAG_OFF;
        g_gnss_ready_flg = FLAG_OFF;
        wait_profile_change();
    }
    
    // get current time
    c_time = get_current_tm();
    (void)localtime_r(&c_time,&c_tm);

    printf("+++++time check start+++++\r\n");
    get_CXM150x_next_LPWA_tx_term(NULL,&res_next_term,NULL);
    next_time = c_time + res_next_term.m_num;
    (void)localtime_r(&next_time,&n_tm);
    printf("current time:%02d:%02d.%02d(utc) next time:%02d:%02d.%02d(utc)\r\n",c_tm.tm_hour,c_tm.tm_min,c_tm.tm_sec,n_tm.tm_hour,n_tm.tm_min,n_tm.tm_sec);

    // Judge whether to turn off the power of CXM150x (Calculate the time it takes to turn off the power and return to normal)
    int32_t interval = next_time - c_time - g_pow_enable_remain_offset;
    printf("next interval:%ld sec\r\n",interval);
    if(interval > CXM150x_POWER_OFF_INTERVAL_SEC){
        printf("CXM150x power off\r\n");
        set_rtc_time();
#if GNSS_BACKUP_USE
        GNSS_backup_exec();
#endif
        board_gpio_int(ELTRES_PIN_INT_OUT1, false);
        CmdResSetCXM150xPower res_set_power;
        memset(&res_set_power,0,sizeof(res_set_power));
        set_CXM150x_power(CXM150x_POWER_OFF,&res_set_power,NULL);
        g_gnss_ready_flg = FLAG_OFF;
        g_int1_callback_flg = FLAG_OFF;
        set_rtc_alarm(0,0,interval);
        memset(&g_nmea_gga_info_buf,0,sizeof(g_nmea_gga_info_buf));
    } else {
        printf("CXM150x power not change\r\n");
    }
    
    printf("+++++time check end  +++++\r\n");
}

// ===========================================================================
//! Wait until GNSS is available
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_gnss_ready_flg: GNSS ready flag
 *        [out] none
 * @return none
*/
// ===========================================================================
static void wait_gps_data(void){
    if(g_gnss_ready_flg == FLAG_OFF){
        printf("-wait GNSS ready-\r\n");
        // Set system state event to ON
        register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
        CmdResSetCXM150xSysStateEvent res_sys_stt;
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);

        while(g_gnss_ready_flg == FLAG_OFF){
            analyse_CXM150x_Rx();
        }
        
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_OFF,&res_sys_stt,NULL);
        printf("-GNSS ready-\r\n");
    }
}

// ===========================================================================
//! Wait profile change
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_profile_change_comlete_flg: profile change completion flag
 *        [out] none
 * @return none
*/
// ===========================================================================
static void wait_profile_change(void){
    if(g_profile_change_comlete_flg == FLAG_OFF && g_gnss_ready_flg == FLAG_OFF){
        printf("-wait profile change-\r\n");
        // Set system state event to ON
        register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
        CmdResSetCXM150xSysStateEvent res_sys_stt;
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);

        while(g_profile_change_comlete_flg == FLAG_OFF && g_gnss_ready_flg == FLAG_OFF){
            analyse_CXM150x_Rx();
        }
        
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_OFF,&res_sys_stt,NULL);
        printf("-profile change complete-\r\n");
    }
}

// ===========================================================================
//! Set payload data and wait for transmission completion
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_payload_data_all0: Default payload data
 *        [in] g_tx_comlete_flg: transmission completion flag
 *        [in] g_ev_tx_complete_flag: Event transmission completion flag
 *        [in] g_gnss_timeout_flag: GNSS acquisition timeout flag
 *        [in] g_gnss_sleep_flag: GNSSSleep flag
 *        [out] g_nmea_gga_info_buf: GGA information event buffer
 *        [out] g_gnss_stt_info: GNSS operation state event information buffer
 * @return none
*/
// ===========================================================================
static void data_send(void){
    g_tx_comlete_flg = FLAG_OFF;

    // Payload settings
    uint8_t payload_data[CXM150x_PAYLOAD_LEN];
    memset(payload_data,0,sizeof(payload_data));
    CmdResSetCXM150xLPWAPayload res_LPWAPayload;
    memset(&res_LPWAPayload,0,sizeof(res_LPWAPayload));

    // GGA information Event ON
    memset(&g_nmea_gga_info_buf,'\0',sizeof(g_nmea_gga_info_buf));
    register_CXM150x_NMEAGGA_event(&g_nmea_gga_info_buf,nmea_gga_event_callback);
    CmdResSetCXM150xNMEAEvent res_set_gnss;
    memset(&res_set_gnss,0,sizeof(res_set_gnss));
    set_CXM150x_NMEA_event(NMEA_EVENT_GGA,&res_set_gnss,NULL);
        
    //GNSS EVENT ON
    memset(&g_gnss_stt_info,'\0',sizeof(g_gnss_stt_info));
    register_CXM150x_GNSS_state_event(&g_gnss_stt_info, gnss_stt_event_callback);
    CmdResSetCXM150xGNSSStateEvent res_set_gnss_stt;
    memset(&res_set_gnss_stt,0,sizeof(res_set_gnss_stt));
    set_CXM150x_GNSS_state_event(EVENT_ON,&res_set_gnss_stt, NULL);

    g_gnss_timeout_flag = FLAG_OFF;
    //wait GNSS wakeup
    while((g_gnss_timeout_flag == FLAG_OFF) && (g_gnss_sleep_flag == FLAG_ON)){
        analyse_CXM150x_Rx();
    }
    
    // Keep updating the payload until the LPWA transmission starts or GNSS module goes to sleep
    // The payload is updated each time a GGA is received, to increase the accuracy of the location information by repetition of positioning.
    // If you don't need location accuracy, you only need to set the payload once.
    while((g_gnss_timeout_flag == FLAG_OFF) && (g_gnss_sleep_flag == FLAG_OFF)){
        g_gga_event_callback_flg = FLAG_OFF; 
            
        // Wait for GGA information reception
        while((g_gga_event_callback_flg == FLAG_OFF) && (g_gnss_sleep_flag == FLAG_OFF)){
            analyse_CXM150x_Rx();
        }
        if(g_gnss_sleep_flag == FLAG_OFF){
            if(g_gnss_timeout_flag == FLAG_OFF){
                // payload setting
                if((g_nmea_gga_info_buf.m_pos_status != GGA_FIX_QUALITY_INVALID) && (g_nmea_gga_info_buf.m_cs_correct == NMEA_CS_OK)){
                    // If the GPS is in the positioning state, set the latitude to the first 8 digits of 16 bytes and the longitude to the remaining 8 digits in ASCII
                    strncpy((char*)payload_data,(char*)g_nmea_gga_info_buf.m_lat,8);
                    strncpy((char*)&payload_data[8],(char*)g_nmea_gga_info_buf.m_lon,8);
                    set_CXM150x_LPWA_payload(payload_data,&res_LPWAPayload,NULL);
                }else{
                    // If GGA data has not been received, set a fixed message
                    set_CXM150x_LPWA_payload(g_payload_data_all0,&res_LPWAPayload,NULL);
                }
            }else{
                // Because update time limit for next LPWA transmit data has passed, do not set the payload here
            }
        }
    }

    set_CXM150x_GNSS_state_event(EVENT_OFF,&res_set_gnss_stt, NULL);
    set_CXM150x_NMEA_event(NMEA_EVENT_ALL_OFF,&res_set_gnss,NULL);

    // Wait for transmission completion
    while(g_tx_comlete_flg == FLAG_OFF && g_ev_tx_complete_flag == FLAG_OFF){
        analyse_CXM150x_Rx();
    }
}

// ===========================================================================
//! RTC timer set function
/*!
 *
 * @param [in] hh: RTC wakeup time (hour)
 * @param [in] mm: RTC wakeup time (minutes)
 * @param [in] ss: RTC wakeup time (seconds)
 * @param [out] none
 * @par Global variable
 *        [in] g_rtc_callback_flg: RTC interrupt flag
 *        [out] none
 * @return none
*/
// ===========================================================================
static void set_rtc_alarm(int32_t hh,int32_t mm,int32_t ss){
    struct rtc_setrelative_s setrel;

    int fd = open("/dev/rtc0", O_WRONLY);

    setrel.id      = 0;
    setrel.pid     = getpid();
    setrel.reltime = (time_t)(hh*60*60 + mm*60 + ss);

    setrel.event.sigev_notify = SIGEV_SIGNAL;
    setrel.event.sigev_signo  = RTC_SIGNO;
    setrel.event.sigev_value.sival_int = 0;

    ioctl(fd, RTC_SET_RELATIVE, (unsigned long)((uintptr_t)&setrel));
    g_rtc_callback_flg = FLAG_OFF;
    close(fd);

    mm += ss / 60;
    ss %= 60;
    hh += mm / 60;
    mm %= 60;
    printf("set_rtc_alarm %02ld:%02ld:%02ld later\r\n",hh,mm,ss);
}

// ===========================================================================
//! Current time acquisition function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return current time
*/
// ===========================================================================
static uint32_t get_current_tm(void){
    // Get current time
    CmdResGetCXM150xCurrentGNSSTime res;
    while(1){
        get_CXM150x_current_GNSS_time(NULL,&res,NULL);
        if(res.m_str[0] != '\0'){
            break;
        } else {
            sleep(1);
        }
    }
    
    uint32_t sec = 0;
    conv_CXM150x_GNSSTime_to_second(res.m_str,&sec);
    
    // Convert the number of seconds elapsed since GPS reference date and time obtained by above API to the number of seconds elapsed since 00:00:00 on January 1, 1900
    // Convert by adding the number of seconds elapsed from 00:00:00 on January 1, 1900 to 00:00:00 on January 6, 1980, which is the base date of GPS time
    struct tm base_time;
    base_time.tm_sec = GPS_FORMAT_BASE_TIME_SEC;
    base_time.tm_min = GPS_FORMAT_BASE_TIME_MIN;
    base_time.tm_hour = GPS_FORMAT_BASE_TIME_HOUR;
    base_time.tm_mday = GPS_FORMAT_BASE_TIME_MDAY;
    base_time.tm_mon = GPS_FORMAT_BASE_TIME_MON;
    base_time.tm_year = GPS_FORMAT_BASE_TIME_YEAR;
    base_time.tm_wday = GPS_FORMAT_BASE_TIME_WDAY;
    base_time.tm_yday = GPS_FORMAT_BASE_TIME_YDAY;
    base_time.tm_isdst = GPS_FORMAT_BASE_TIME_ISDST;
    sec += mktime(&base_time);

    // This time format is GNSS time so substract leap second to convert it to UTC time format
    sec -= GNSS_DATETIME_ADJUST_LEAP_SECOND;

    return sec;
}

// ===========================================================================
//! Set RTC timer
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return current time
*/
// ===========================================================================
static void set_rtc_time(void){
    uint32_t sec = get_current_tm();

    struct timespec ts;
    ts.tv_sec = sec;
    ts.tv_nsec = 0;
    clock_settime(CLOCK_REALTIME, &ts);

    struct tm s_tm;
    localtime_r(&ts.tv_sec, &s_tm);
    printf("local time:%02d:%02d.%02d(utc)\r\n",s_tm.tm_hour,s_tm.tm_min,s_tm.tm_sec);
    printf("RTC init:%04d-%02d-%02d %02d:%02d:%02d\r\n",
           s_tm.tm_year + 1900, s_tm.tm_mon + 1, s_tm.tm_mday, s_tm.tm_hour, s_tm.tm_min, s_tm.tm_sec);
    
}

#if GNSS_BACKUP_USE
// ===========================================================================
//! Get the time set in RTC in time_t format
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return current time (time_t format)
*/
// ===========================================================================
static time_t get_rtc_time(void){
    int ret;
    struct timespec ts;
    // Get the time set in RTC
    ret = clock_gettime(CLOCK_REALTIME, &ts);
    
    if(ret == 0){
        struct tm s_tm;

        localtime_r(&ts.tv_sec, &s_tm);
        printf("get_rtc_time:%04d-%02d-%02d %02d:%02d:%02d\r\n",s_tm.tm_year+1900,s_tm.tm_mon+1,s_tm.tm_mday,s_tm.tm_hour,s_tm.tm_min,s_tm.tm_sec);
        
        return ts.tv_sec;
    } else {
        //Acquisition failure
        printf("get_rtc_time invalid.\r\n");
        return 0;
    }
}

// ===========================================================================
//! Read the UTC time set in RTC and get it in character string format
/*!
 *
 * @param [in] none
 * @param [out] Current time character string (YYYYMMDDhhmmss format)
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void get_utc_time_str(uint8_t *str){
    time_t rtc_time = get_rtc_time();
    if(rtc_time > 0){
        // Successful acquisition
        struct tm utc_tm;
        (void)localtime_r(&rtc_time,&utc_tm);
        strftime((char*)str, GNSS_DATETIME_LEN+1, "%Y%m%d%H%M%S", &utc_tm);
    } else {
        //Acquisition failure
        str[0] = '\0';
        printf("get_utc_time_str invalid.\r\n");
    }
}

// ===========================================================================
//! When CXM150x is powered on, set time information and location information to enable hot start using the backup data of GNSS(BUP)
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void set_GNSS_backup_info(void){
/*
    // Set location information
    // After GNSS BACKUP, if you move a long distance before starting, the effect of hot start may not be obtained
    // In that case, get the location information by another way and set it to the GNSS by following method
    
    CmdResSetCXM150xGNSSPosition res_set_position;
    memset(&res_set_position,0,sizeof(res_set_position));
    CXM150xGNSSPositionSetData pos_data;
    strncpy((char*)pos_data.m_lat,"3525.6911",NMEA_LAT_SIZE);
    pos_data.m_lat[NMEA_LAT_SIZE] = '\0';
    strncpy((char*)pos_data.m_n_s,"N",NMEA_N_S_SIZE);
    pos_data.m_n_s[NMEA_N_S_SIZE] = '\0';
    strncpy((char*)pos_data.m_lon,"13922.2240",NMEA_LON_SIZE);
    pos_data.m_lon[NMEA_LON_SIZE] = '\0';
    strncpy((char*)pos_data.m_e_w,"E",NMEA_E_W_SIZE);
    pos_data.m_e_w[NMEA_E_W_SIZE] = '\0';
    set_CXM150x_GNSS_position(&pos_data,&res_set_position,NULL);
*/
    
    // Set GNSS time information
    CmdResSetCXM150xGNSSDateTime res_gnss_datetime;
    memset(&res_gnss_datetime,0,sizeof(res_gnss_datetime));
    uint8_t rtc_time_str[GNSS_DATETIME_LEN+1] = "";
    get_utc_time_str(rtc_time_str);
    
    if(rtc_time_str[0] != '\0'){
        // Set time (with leap second) for CXM150x
        set_CXM150x_GNSS_datetime(rtc_time_str,&res_gnss_datetime,NULL);
    }
}

// ===========================================================================
//! Execute GNSS information backup
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_gnss_backup_done_flag: GNSS information backup completion wait flag
 *        [out] g_last_gnss_backup_time: Last GNSS information backup time
 * @return none
*/
// ===========================================================================
static void GNSS_backup_exec(void){
    
    // Do not backup if GNSS_BACKUP_INTERVAL_SEC seconds have not elapsed since last backup time
    time_t c_tm = get_rtc_time();
    if(c_tm != 0 && c_tm - g_last_gnss_backup_time >= GNSS_BACKUP_INTERVAL_SEC){
        // Execute backup processing
        printf("-GNSS_backup_exec start-\r\n");
        
        // Start waiting for GNSS backup completion
        CmdResSetCXM150xSysStateEvent res_sys_stt;
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
        set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);

        // Issue GNSS backup command
        g_gnss_backup_done_flag = FLAG_OFF;
        CmdResSetCXM150xGNSSBackup res_gnss_backup;
        memset(&res_gnss_backup,0,sizeof(res_gnss_backup));
        set_CXM150x_GNSS_backup(NULL,&res_gnss_backup,NULL);

        uint32_t wait_start_tick_count = wrapper_CXM150x_get_tick();
        while(g_gnss_backup_done_flag == FLAG_OFF){
            analyse_CXM150x_Rx();
            // timeout check
            uint32_t current_tick_count = wrapper_CXM150x_get_tick();
            if(current_tick_count - wait_start_tick_count > WAIT_BACKUP_DONE_MAX_TICK_COUNT){
                printf("wait backup done timeout.\r\n");
                break;
            }
        }
        
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_OFF,&res_sys_stt,NULL);
        printf("-GNSS_backup_exec end-\r\n");
        
        if(g_gnss_backup_done_flag == FLAG_ON){
            // Update the previous backup time only when a backup completion event is received
            g_last_gnss_backup_time = get_rtc_time();
            struct tm s_tm;
            (void)localtime_r(&g_last_gnss_backup_time,&s_tm);
            printf("GNSS backup time renew:%04d-%02d-%02d %02d:%02d:%02d\r\n",s_tm.tm_year+1900,s_tm.tm_mon+1,s_tm.tm_mday,s_tm.tm_hour,s_tm.tm_min,s_tm.tm_sec);
        }
    } else {
        // Skip backup processing because it is less than the specified interval
        printf("GNSS_backup_exec skip(%ldsec/%dsec)\r\n",(c_tm - g_last_gnss_backup_time),GNSS_BACKUP_INTERVAL_SEC);
    }
}
#endif

// ===========================================================================
//! check message count
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return FLAG_ON if message exist
*/
// ===========================================================================
static uint32_t check_message_count(void){
    // Get power supply status
    CmdResGetCXM150xPower pwstate;
    get_CXM150x_power(NULL,&pwstate,NULL);
    
    // resume if the power is off
    if(pwstate.m_num == CXM150x_POWER_OFF){
        return FLAG_OFF;
    } else {
        if(get_CXM150x_Rx_message_count()>0){
            return FLAG_ON;
        } else {
            return FLAG_OFF;
        }
    }
}

// ===========================================================================
//! RTC wakeup callback function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_rtc_callback_flg: RTC interrupt flag
 * @return none
*/
// ===========================================================================
static void rtc_callback(void){
    printf("rtc_callback\r\n");
    g_rtc_callback_flg = FLAG_ON;
}

// ===========================================================================
//! RTC alarm handler
/*!
 *
 * @param[in]    signo    : signal number
 * @param[in]    info     : signal information
 * @param[in]    ucontext : signal context
 * @return none
*/
// ===========================================================================
static void rtc_alarm_handler(int signo, FAR siginfo_t *info, FAR void *ucontext)
{
  rtc_callback();
}

// ===========================================================================
//! Event buffer overflow callback function
/*!
 *
 * @param [in] info: Event data information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void event_buffer_overflow_callback(void *info,uint32_t id){
    printf("event buffer overflow\r\n");
}

// ===========================================================================
//! tx PoC enable message callback function
/*!
 *
 * @param [in] info: Event data information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void tx_poc_enable_message_callback(void *info,uint32_t id){
#if TX_POC_USE
    printf("TX PoC enable message callback\r\n");
#else
    //If TX PoC enable message callback occurs when TX PoC is disable, turn off the power due to a fatal error.
    printf("FATAL errot :An unencrypted message\r\n");
    assert(0);
#endif
}

#if TX_DUTY_USE
// ===========================================================================
//! tx Duty event callback function
/*!
 *
 * @param [in] info: Event data information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void tx_duty_event_callback(void *info,uint32_t id){
    CXM150xTxDutyEventInfo *tx_duty_event_info = (CXM150xTxDutyEventInfo*)info;
    if(tx_duty_event_info->m_result == CXM150x_RESPONSE_OK){
        printf("TX DUTY EVENT OK,%s\r\n",tx_duty_event_info->m_str);
    }else{
        printf("TX DUTY EVENT NG,%s\r\n",tx_duty_event_info->m_str);
    }
}
#endif

// ===========================================================================
//! Display sample app and API version
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void disp_version(void){
    CmdResGetCXM150xAPIVersion api_ver_inf;
    get_CXM150x_api_version(NULL,&api_ver_inf,NULL);
    printf("%s:Ver.%s, API:Ver.%s\r\n",SAMPLE_APP_NAME,SAMPLE_APP_VER,api_ver_inf.m_version);
}

// ===========================================================================
//! LPWA sample application main function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_push_btn_flag: Push button interrupt flag
 *        [in] g_int1_callback_flg:  INT_OUT1 interrupt flag 
 *        [in] g_ev_tx_complete_flag: Event transmission completion flag
 *        [in] g_sys_stt_info: system state event information
 *        [in] g_buffer_overflow_info: Response data structure
 *        [out] g_pow_enable_remain_offset: CXM150x power ON offset time
 *        [out] g_periodec_enable: Periodic enable flag
 *        [out] g_profile_event_enable: Event transmission enable flag
 * @return exit code
*/
// ===========================================================================
int main_LPWA_sample_app(void){
    // Display version information
    disp_version();
    ///////////////////////////////////////////////////////////
    // mask the INT2 interrupt
    board_gpio_int(ELTRES_PIN_INT_OUT2, false);
    ///////////////////////////////////////////////////////////

    /* Register alarm signal handler */

    struct sigaction act;
    sigset_t set;

    sigemptyset(&set);
    sigaddset(&set, RTC_SIGNO);
    sigprocmask(SIG_UNBLOCK, &set, NULL);

    act.sa_sigaction = rtc_alarm_handler;
    act.sa_flags     = SA_SIGINFO;

    sigfillset(&act.sa_mask);
    sigdelset(&act.sa_mask, RTC_SIGNO);

    sigaction(RTC_SIGNO, &act, NULL);

    // FATAL message event callback setting
    register_CXM150x_FATAL_message_event(&g_fatalmessage_info,fatal_message_event_callback);
    register_CXM150x_event_buffer_overflow(&g_buffer_overflow_info,event_buffer_overflow_callback);
    register_CXM150x_tx_PoC_enable_message_event(&g_tx_poc_enable_message_info,tx_poc_enable_message_callback);

    // INT_OUT interrupt setting
    board_gpio_intconfig(ELTRES_PIN_INT_OUT1, INT_RISING_EDGE, false, (xcpt_t)wrapper_CXM150x_int_out1);
    board_gpio_int(ELTRES_PIN_INT_OUT1, true);

    board_gpio_intconfig(ELTRES_PIN_INT_OUT2, INT_RISING_EDGE, false, (xcpt_t)wrapper_CXM150x_int_out2);
    board_gpio_int(ELTRES_PIN_INT_OUT2, true);

#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
    // Button interrupt setting
    board_gpio_config(INT_BUTTON1_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON1_PIN, INT_FALLING_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON1_PIN, true);

    board_gpio_config(INT_BUTTON2_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON2_PIN, INT_FALLING_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON2_PIN, true);

    board_gpio_config(INT_BUTTON3_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON3_PIN, INT_FALLING_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON3_PIN, true);

    board_gpio_config(INT_BUTTON4_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON4_PIN, INT_FALLING_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON4_PIN, true);
#endif
    // Power ON and set normal mode
    CmdResSetCXM150xPower res_set_power;
    memset(&res_set_power,0,sizeof(res_set_power));
    set_CXM150x_power(CXM150x_POWER_ON,&res_set_power,NULL);
#if TX_POC_USE
    CmdResSetCXM150xTxPoCEnable res_set_tx_poc_enable;
    memset(&res_set_tx_poc_enable,0,sizeof(res_set_tx_poc_enable));
    set_CXM150x_tx_PoC_enable(NULL,&res_set_tx_poc_enable,NULL);
#endif
#if TX_DUTY_USE
    CmdResSetCXM150xTxDutyEvent res_set_tx_duty_event;
    memset(&res_set_tx_duty_event,0,sizeof(res_set_tx_duty_event));
    set_CXM150x_tx_duty_event(EVENT_ON,&res_set_tx_duty_event,NULL);

    // tx duty callback setting
    register_CXM150x_tx_duty_event(&g_duty_event_info,tx_duty_event_callback);
#endif

    CmdResSetCXM150xMode res_set_mode;
    memset(&res_set_mode,0,sizeof(res_set_mode));
    set_CXM150x_mode(CXM150x_MODE_NORMAL,&res_set_mode,NULL);

    // Get various EEPROM settings
    CmdResGetCXM150xEEPROMData eep_data;

    // CXM150x power ON offset time
    get_CXM150x_EEPROM_data(EEPROM_POW_ENABLE_REMAIN_OFFSET,&eep_data,NULL);
    g_pow_enable_remain_offset = eep_data.m_num + AR_FRAME_DELAY;

    // Periodi 1 and 2 valid setting
    uint32_t periodic1_eanable;
    uint32_t periodic2_eanable;
    get_CXM150x_EEPROM_data(EEPROM_DF1_ENABLE,&eep_data,NULL);
    periodic1_eanable = eep_data.m_num;
    
    get_CXM150x_EEPROM_data(EEPROM_DF2_ENABLE,&eep_data,NULL);
    periodic2_eanable = eep_data.m_num;
    
    // If both Periodic1 and Periodic2 are set to disabled, then Periodic is disabled
    if(periodic1_eanable == FLAG_OFF && periodic2_eanable == FLAG_OFF){
        g_periodec_enable = FLAG_OFF;
    } else {
        g_periodec_enable = FLAG_ON;
    }

    // event transmission enable setting
    get_CXM150x_EEPROM_data(EEPROM_EF_ENABLE,&eep_data,NULL);
    g_profile_event_enable = eep_data.m_num;

    printf("*** pow_enable_remain_offset:%ld\r\n",g_pow_enable_remain_offset);
    printf("*** periodic1_eanable:%ld  periodic2_eanable:%ld\r\n",periodic1_eanable,periodic2_eanable);
    printf("*** periodic  enable:%ld\r\n",g_periodec_enable);
    printf("*** event     enable:%ld\r\n",g_profile_event_enable);

    // int2 interrupt callback setting
    register_CXM150x_uart_start_interrupt(NULL,int2_callback);

    // GPS data acquisition
    wait_gps_data();

    // RTC settings
    set_rtc_time();

    // Register INT_OUT1 interrupt function
    register_CXM150x_LPWA_start_interrupt(NULL,int1_callback);

    // Power off judgment
    power_off_check();

    g_push_btn_flag = FLAG_OFF;

    printf("*** push button to event send ***\r\n");

    /* Infinite loop */
    while(1){
        // Sleep processing if no interrupt
        if(g_int1_callback_flg == FLAG_OFF && g_push_btn_flag == FLAG_OFF && check_message_count() == FLAG_OFF
           && g_rtc_callback_flg == FLAG_OFF){
            // TBD: Implement sleep function
            sleep(1);
            continue;
        }

        // resume CXM150x
        resume_CXM150x();

        if(g_push_btn_flag == FLAG_ON){
            if(g_profile_event_enable == FLAG_OFF){
                printf("Event profile disable\r\n");
                g_push_btn_flag = FLAG_OFF;
                // GPS data acquisition
                wait_gps_data();
                // Power off judgment
                power_off_check();

                continue;
            }
            printf("*** event send start! ***\r\n");
            g_ev_tx_complete_flag = FLAG_OFF;

            // profile change (event)
            CmdResSetCXM150xTxProfile res_tx_profile;
            set_CXM150x_tx_profile(TX_CUR_FRM_TYPE_EVENT,&res_tx_profile,NULL);

            // After changing profiles, wait for transition to EPM_FILL or WAIT_TX_PREPARE
            g_profile_change_comlete_flg = FLAG_OFF;
            g_gnss_ready_flg = FLAG_OFF;
            wait_profile_change();
            
            // Set system state event to ON
            register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
            CmdResSetCXM150xSysStateEvent res_sys_stt;
            memset(&res_sys_stt,0,sizeof(res_sys_stt));
            set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);

            while(1){
                analyse_CXM150x_Rx();
                if(g_ev_tx_complete_flag == FLAG_OFF && g_int1_callback_flg == FLAG_ON){
                    g_int1_callback_flg = FLAG_OFF;

                    // Send data
                    data_send();
                }else if(g_ev_tx_complete_flag == FLAG_ON){
                    break;
                }
            }

            // Set system state event to OFF
            memset(&res_sys_stt,0,sizeof(res_sys_stt));
            set_CXM150x_sys_state_event(EVENT_OFF,&res_sys_stt,NULL);

            // Power off judgment
            power_off_check();

            g_push_btn_flag = FLAG_OFF;
            g_int1_callback_flg = FLAG_OFF;
            printf("*** event send end! ***\r\n");
        } else if(g_int1_callback_flg == FLAG_ON){
            printf("*** normal send start! ***\r\n");
            g_ev_tx_complete_flag = FLAG_OFF;

            // Set system state event to ON
            register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
            CmdResSetCXM150xSysStateEvent res_sys_stt;
            memset(&res_sys_stt,0,sizeof(res_sys_stt));
            set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);

            // Send data
            data_send();

            // Set system state event to OFF
            memset(&res_sys_stt,0,sizeof(res_sys_stt));
            set_CXM150x_sys_state_event(EVENT_OFF,&res_sys_stt,NULL);

            // If there is an event transmission interrupt, do not judge the power OFF here
            if(g_push_btn_flag == FLAG_OFF){
                power_off_check();
            }

            g_int1_callback_flg = FLAG_OFF;
            printf("*** normal send end! ***\r\n");
        }
    }
}

