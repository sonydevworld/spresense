/****************************************************************************
 * modules/include/sensing/tap.h
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
/**
 * @file tap.h
 */

#ifndef __INCLUDE_SENSING_TAP_H
#define __INCLUDE_SENSING_TAP_H

/**
 * @defgroup tap_lib Tap Library
 * @{
 */

/* --------------------------------------------------------------------------
  Included Files
   -------------------------------------------------------------------------- */

#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
  Pre-Processor Definitions
   -------------------------------------------------------------------------- */
/**
 * @defgroup tap_lib_defs Defines
 * @{
 */

  
/**
 * @def TAP_BUF_LEN
 * @brief Maximum number of data of accel sensor to save
 */
#define TAP_BUF_LEN                   16

/**
 * @def D_SA_STATUS_OK
 * @brief [Error Code] OK
 */
#define D_SA_STATUS_OK                0

/**
 * @def D_SA_STATUS_E_INVALID_ARGS
 * @brief [Error Code] Invalid argument
 */
#define D_SA_STATUS_E_INVALID_ARGS    -1

/**
 * @def D_SA_STATUS_E_UNEXPECTED
 * @brief [Error Code] Unexpected error
 */
#define D_SA_STATUS_E_UNEXPECTED      -8

/**
 * @def SEC_PER_US
 * @brief second -> microsecond
 */
#define SEC_PER_US                    1000000

 /**
 * @def NS_PER_US
 * @brief nanosecond -> microsecond
 */
#define NS_PER_US                     1000

/** @} tap_lib_defs */

/* --------------------------------------------------------------------------
  Enumerations
   -------------------------------------------------------------------------- */
/**
 * @defgroup tap_lib_datatypes Data types
 * @{
 */


/**
 * @enum E_TAP_STATE
 * @brief tap status
 */
typedef enum {
  E_TAP_STATE_IDLE = 0,     /**< When No tap detected.  */
  E_TAP_STATE_TAP           /**< When tap detected.     */
} E_TAP_STATE;

/* --------------------------------------------------------------------------
  Command Structures
   -------------------------------------------------------------------------- */

/**
 * @struct ST_TAP_OPEN
 * @brief Coefficient required for tap detection
 */
typedef struct
{
  float tap_period;  /**< (microsec) Validity period to detect the tap series */
  float peak_thres;  /** (G) The minimum value of vibration 
                      * tapping shall be deemed. range 0.0 - 4.0
                      */
  float long_thres;  /** (G) The maximum vibration indicates 
                      * that vibration of the tap. range 0.0 - 4.0
                      */
  int stab_frame;    /** (64Hz frame num) time of PEAK_THRES -> LONG_THRES. 
                      * range 0 - 32
                      */
} ST_TAP_OPEN;

/**
 * @struct ST_TAP_ACCEL
 * @brief accel data
 */
typedef struct
{
  float accel_x;    /**< (G) X axis standard gravity acceleration. */
  float accel_y;    /**< (G) Y axis standard gravity acceleration. */
  float accel_z;    /**< (G) Z axis standard gravity acceleration. */
} ST_TAP_ACCEL;

/** @} tap_lib_datatypes */

/*--------------------------------------------------------------------
    Tap Class
  --------------------------------------------------------------------*/
/**
 * @defgroup tap_lib_funcs Functions
 * @{
 */

class TapClass {

public:

  /* public methods */

  int open(ST_TAP_OPEN*);
  int close(void);
  int write(ST_TAP_ACCEL*);
  int write(ST_TAP_ACCEL*, uint64_t);

  TapClass();
  ~TapClass(){};

private:

  /* private members */

  uint64_t    mTapPeriod;  /**< (umec) Validity period to detect the tap series */
  float       mPeakThres;  /* (G) The minimum value of vibration tapping
                       *   shall be deemed. range 0.0 - 4.0
                       */
  float       mLongThres;  /* (G) The maximum vibration indicates that 
                       *   vibration of the tap. range 0.0 - 4.0
                       */
  int         mStabFrame;  /* (64Hz frame num) time of PEAK_THRES -> LONG_THRES. 
                       *   range 0 - 32
                       */

  int          mTapCnt;          /**< Detect tap Count. */
  E_TAP_STATE  mState;           /**< Holds IDLE or TAP state */

  float        mR[TAP_BUF_LEN];  /**< Set sqrt value */
  float        mX[TAP_BUF_LEN];  /**< Accel Data(x)  */
  float        mY[TAP_BUF_LEN];  /**< Accel Data(y)  */
  float        mZ[TAP_BUF_LEN];  /**< Accel Data(z)  */

  int          mIndex;           /**< Storage location of the mR to be referenced */
  int          mDetectionCount;  /**< last detection. */
  int          mStab;            /* When it becomes larger than mStabFrame,
	                                * tap detection 
                                  */

  uint64_t     mStartTime;        /**< Time to use for continuous tap detection. */

  /* private methods */
  float calcR(int i0, int j0);
  bool detect(float x, float y, float z);
  float getIndex(int idx);

};

/*--------------------------------------------------------------------
    External Interface
  --------------------------------------------------------------------*/

/**
 * @brief Create TapClass instance. 
 * return Address for instance of TapClass
 *
 */
TapClass* TapCreate(void);

/**
 * @brief     Set coefficients necessary for parameter 
 *            initialization and tap detection.
 * @param[in] ins : instance address of TapClass
 * @return    result of process.
 */
int TapOpen(FAR TapClass *ins, FAR ST_TAP_OPEN *OpenParam);

/**
 * @brief     None.
 * @param[in] ins : instance address of TapClass
 * @return    D_SA_STATUS_OK
 */
int TapClose(FAR TapClass *ins);

/**
 * @brief     Detect tap
 * @param[in] ins : instance address of TapClass
 * @param[in] accelData : Accel Data
 * @return    tapcnt or error code
 */
int TapWrite(FAR TapClass *ins, FAR ST_TAP_ACCEL *accelData);

/**
 * @brief     Detect tap
 * @param[in] ins : instance address of TapClass
 * @param[in] accelData : Accel Data
 * @param[in] time_stamp : Time Stamp
 * @return    tapcnt or error code
 */
int TapWrite_timestamp(FAR TapClass *ins, FAR ST_TAP_ACCEL *accelData, 
                       uint64_t time_stamp);

/** @} tap_lib_funcs */
/** @} tap_lib */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SENSING_TAP_H */
