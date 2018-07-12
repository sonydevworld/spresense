/****************************************************************************
 * modules/sensing/tap/tap.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <debug.h>
#include "sensing/tap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define TAP_DETECTION_COUNT 8

/* tap parameter min,max */

#define TAP_PEAK_THRES_MIN  0.0F
#define TAP_PEAK_THRES_MAX  4.0F
#define TAP_LONG_THRES_MIN  0.0F
#define TAP_LONG_THRES_MAX  4.0F
#define TAP_STAB_FRAMES_MIN 0
#define TAP_STAB_FRAMES_MAX 32


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: TapCreate
 *
 * Description:
 *   Create a TapClass.
 *
 * Input Parameters:
 *   -
 *
 * Returned Value:
 *   TapClass*   Object of TapClass.
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
TapClass* TapCreate(void)
{
  return new TapClass();
}

/****************************************************************************
 * Name: TapOpen
 *
 * Description:
 *   TapClass::open() call.
 *
 * Input Parameters:
 *   TapClass*           Object of TapClass.
 *   ST_TAP_OPEN*     Coefficient required for tap detection
 *
 * Returned Value:
 *   TapClass::open() result
 *     D_SA_STATUS_E_INVALID_ARGS   Parameter error
 *     D_SA_STATUS_OK               OK
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapOpen(FAR TapClass *ins, FAR ST_TAP_OPEN *OpenParam)
{
  int ret = 0;

  ret = ins->open(OpenParam);

  return ret;
}

/****************************************************************************
 * Name: TapClose
 *
 * Description:
 *   TapClass::close() call.
 *
 * Input Parameters:
 *   TapClass*           Object of TapClass.
 *
 * Returned Value:
 *   TapClass::close() result
 *     D_SA_STATUS_OK               OK
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapClose(FAR TapClass *ins)
{
  int ret = 0;

  ret = ins->close();
  delete ins;

  return ret;
}

/****************************************************************************
 * Name: TapWrite
 *
 * Description:
 *   TapClass::write() call.
 *
 * Input Parameters:
 *   TapClass*           Object of TapClass.
 *   ST_SA_ACCEL_DATA*   Accel Data(x,y,z)
 *
 * Returned Value:
 *   TapClass::write() result
 *     D_SA_STATUS_E_INVALID_ARGS   Parameter error
 *     D_SA_STATUS_E_UNEXPECTED     Unexpected error
 *     tapcnt                       number of taps
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapWrite_timestamp(FAR TapClass *ins, FAR ST_TAP_ACCEL *accelData, 
                       uint64_t time_stamp)
{
  int ret = 0;

  ret = ins->write(accelData, time_stamp);

  return ret;
}

/****************************************************************************
 * Name: TapWrite
 *
 * Description:
 *   TapClass::write() call.
 *
 * Input Parameters:
 *   TapClass*           Object of TapClass.
 *   ST_SA_ACCEL_DATA*   Accel Data(x,y,z)
 *
 * Returned Value:
 *   TapClass::write() result
 *     D_SA_STATUS_E_INVALID_ARGS   Parameter error
 *     D_SA_STATUS_E_UNEXPECTED     Unexpected error
 *     tapcnt                       number of taps
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapWrite(FAR TapClass *ins, FAR ST_TAP_ACCEL *accelData)
{
  int ret = 0;

  ret = ins->write(accelData);

  return ret;
}

/****************************************************************************
 *Tap Class
 ****************************************************************************/
/****************************************************************************
 * Name: TapClass
 *
 * Description:
 *   constructor
 *
 * Input Parameters:
 *   -
 *
 * Returned Value:
 *   -
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
TapClass::TapClass()
{
  mTapCnt     = 0;
  mState      = E_TAP_STATE_IDLE;

  for (int i = 0; i < TAP_BUF_LEN; i++)
    {
      mX[i] = 0;
      mY[i] = 0;
      mZ[i] = 0;
      mR[i] = 0;
    }

  mIndex          = 0;
  mDetectionCount = 0;
  mStab           = 0;
}

/****************************************************************************
 * Name: open
 *
 * Description:
 *   Set Coefficient required for tap detection
 *
 * Input Parameters:
 *   ST_TAP_OPEN*     Coefficient required for tap detection
 *
 * Returned Value:
 *   D_SA_STATUS_E_INVALID_ARGS   Parameter error
 *   D_SA_STATUS_OK               OK
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapClass::open(ST_TAP_OPEN *OpenParam)
{
  _info("TapClass::open() called.\n");

  /* Param Check */

  if (OpenParam->peak_thres < TAP_PEAK_THRES_MIN ||
    OpenParam->peak_thres > TAP_PEAK_THRES_MAX)
    {
      _err("[ERROR] peak_thres : %f\n", OpenParam->peak_thres);
      return D_SA_STATUS_E_INVALID_ARGS;
    }

  if (OpenParam->long_thres < TAP_LONG_THRES_MIN ||
    OpenParam->long_thres > TAP_LONG_THRES_MAX)
    {
      _err("[ERROR] long_thres : %f\n", OpenParam->long_thres);
      return D_SA_STATUS_E_INVALID_ARGS;
    }

  if (OpenParam->stab_frame < TAP_STAB_FRAMES_MIN ||
    OpenParam->stab_frame > TAP_STAB_FRAMES_MAX)
    {
      _err("[ERROR] stab_frame : %d\n", OpenParam->stab_frame);
      return D_SA_STATUS_E_INVALID_ARGS;
    }

  /* Set param */

  mTapPeriod  = OpenParam->tap_period;
  mPeakThres  = OpenParam->peak_thres;
  mLongThres  = OpenParam->long_thres;
  mStabFrame  = OpenParam->stab_frame;
  mTapCnt     = 0;
  mState      = E_TAP_STATE_IDLE;

  for (int i = 0; i < TAP_BUF_LEN; i++)
    {
      mX[i] = 0;
      mY[i] = 0;
      mZ[i] = 0;
      mR[i] = 0;
    }

  mIndex          = 0;
  mDetectionCount = 0;
  mStab           = 0;

  return D_SA_STATUS_OK;
}

/****************************************************************************
 * Name: close
 *
 * Description:
 *   -
 *
 * Input Parameters:
 *   -
 *
 * Returned Value:
 *   D_SA_STATUS_OK               OK
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapClass::close(void)
{
  _info("TapClass::close() called.\n");
  return D_SA_STATUS_OK;
}

/****************************************************************************
 * Name: TapWrite
 *
 * Description:
 *   TapClass::write() call.
 *
 * Input Parameters:
 *   TapClass*           Object of TapClass.
 *   ST_SA_ACCEL_DATA*   Accel Data(x,y,z)
 *
 * Returned Value:
 *   TapClass::write() result
 *     D_SA_STATUS_E_INVALID_ARGS   Parameter error
 *     D_SA_STATUS_E_UNEXPECTED     Unexpected error
 *     tapcnt                       number of taps
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapClass::write(ST_TAP_ACCEL* accelData)
{
  _info("TapClass::write(acc) called.\n");
  
  bool              detectflg     = false;
  int               tapcnt        = 0;
  uint64_t          elapsedTime   = 0;
  uint64_t          endTime       = 0;
  struct   timespec ts;

  _info("accel_x %.3f accel_y %.3f accel_z %.3f \n",
        accelData->accel_x, accelData->accel_y, accelData->accel_z);

  if (NULL == accelData)
    {

      /* No AccelData */

      _err("accelData or algorithmResult is NULL\n");
      return D_SA_STATUS_E_INVALID_ARGS;
    }

  /* Time acquisition */

  if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
    {
      _err("clock_gettime() failed\n");
      return D_SA_STATUS_E_UNEXPECTED;
    }

  endTime = (ts.tv_sec * SEC_PER_US) + (ts.tv_nsec / NS_PER_US);

  /* Tap detection judgment */

  detectflg = detect(accelData->accel_x, accelData->accel_y, accelData->accel_z);

  /* State determination */
  
  switch (mState){
  case E_TAP_STATE_IDLE:
    if(true == detectflg)
      {

        /* Detect Tap */

        mTapCnt++;

        /* Transition to tap state */

        mState = E_TAP_STATE_TAP;

        /* Time update */

        mStartTime = endTime;
      }
    else
      {

        /* Notify number of taps */

        tapcnt = mTapCnt;
        mTapCnt = 0;
      }
    break;

  case E_TAP_STATE_TAP:
    
    /* Calculate the time difference (Unit: microseconds) */

    elapsedTime = endTime - mStartTime;
    if (true == detectflg)
      {

        /* Detect Tap */

        if (elapsedTime > mTapPeriod)
          {

            /* Notify the number of previous taps once and update it to 1 */

            tapcnt = mTapCnt;
            mTapCnt = 1;

            /* Time update */

            mStartTime = endTime;
          } 
        else
          {

            /* In time */
            
            mTapCnt++;

            /* Time update */
            
            mStartTime = endTime;
          }
      }
    else
      {
        if (elapsedTime > mTapPeriod)
          {
        
            /* TimeOut */

            tapcnt  = mTapCnt;
            mTapCnt = 0;
            mState  = E_TAP_STATE_IDLE;
          }
      }
    break;
  default:
    break;
  }

  /* Return number of taps */
  return tapcnt;
}

/****************************************************************************
 * Name: write
 *
 * Description:
 *   TapClass::write() call.
 *
 * Input Parameters:
 *   TapClass*           Object of TapMngClass.
 *   ST_SA_ACCEL_DATA*   Accel Data(x,y,z)
 *
 * Returned Value:
 *   TapClass::write() result
 *     D_SA_STATUS_E_INVALID_ARGS   Parameter error
 *     D_SA_STATUS_E_UNEXPECTED     Unexpected error
 *     tapcnt                       number of taps
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
int TapClass::write(ST_TAP_ACCEL *accelData, uint64_t time_stamp)
{
  bool detectflg         = false;
  int tapcnt             = 0;
  uint64_t elapsedTime   = 0;
  uint64_t endTime       = time_stamp;

  _info("accel_x %.3f accel_y %.3f accel_z %.3f timestamp %llu \n",
       accelData->accel_x, accelData->accel_y, accelData->accel_z, time_stamp);

  if (NULL == accelData)
    {

      /* No AccelData */

      _err("accelData or algorithmResult is NULL\n");
      return D_SA_STATUS_E_INVALID_ARGS;
    }

  /* Tap detection judgment */

  detectflg = detect(accelData->accel_x, accelData->accel_y, accelData->accel_z);

  /* State determination */
  
  switch (mState){
  case E_TAP_STATE_IDLE:
    if (true == detectflg)
      {

        /* Detect Tap */

        mTapCnt++;

        /* Transition to tap state */
        
        mState = E_TAP_STATE_TAP;
        
        /* Time update */
        
        mStartTime = endTime;
      }
    else
      {

        /* Notify number of taps */

        tapcnt = mTapCnt;
        mTapCnt = 0;
      }
    break;

  case E_TAP_STATE_TAP:
    
    /* Calculate the time difference (Unit: microseconds) */

    elapsedTime = endTime - mStartTime;
    if (true == detectflg)
      {

        /* Detect Tap */

        if (elapsedTime > mTapPeriod)
          {

            /* Notify the number of previous taps once and update it to 1 */

            tapcnt = mTapCnt;
            mTapCnt = 1;

            /* Time update */

            mStartTime = endTime;
          } 
        else
          {

            /* In time */
            
            mTapCnt++;

            /* Time update */
            
            mStartTime = endTime;
          }
      }
    else
      {
        if (elapsedTime > mTapPeriod)
          {
        
            /* TimeOut */

            tapcnt  = mTapCnt;
            mTapCnt = 0;
            mState  = E_TAP_STATE_IDLE;
          }
      }
    break;
  default:
    break;
  }

  return tapcnt;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: calcR
 *
 * Description:
 *   
 *
 * Input Parameters:
 *   i0   - 0
 *   j0   - detection count
 *
 * Returned Value:
 *   Sqrt Result.
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
float TapClass::calcR(int i0, int j0)
{
  int i    = getIndex(i0);
  int j    = getIndex(j0);
  float dx = mX[i] - mX[j];
  float dy = mY[i] - mY[j];
  float dz = mY[i] - mY[j];
  float r  = sqrt(dx * dx + dy * dy + dz * dz);

  return r;
}

/****************************************************************************
 * Name: detect
 *
 * Description:
 *   It judges whether it detects tap.
 *
 * Input Parameters:
 *   x   - accel data(x)
 *   y   - accel data(y)
 *   z   - accel data(z)
 *
 * Returned Value:
 *   true   - detect tap
 *   false  - not detect tap
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
bool TapClass::detect(float x, float y, float z)
{

  int index = mIndex;
  if (++mIndex == TAP_BUF_LEN)
    {
      mIndex = 0;
    }

  mX[index] = x;
  mY[index] = y;
  mZ[index] = z;
  mR[index] = sqrt(x * x + y * y + z * z);

  if (mDetectionCount == 0)
    {
      if (mR[index] > mPeakThres)
        {
          mDetectionCount = TAP_DETECTION_COUNT;
        }
      return false;
    }

  mDetectionCount--;
  if (mR[index] > mPeakThres)
    {
      return false;
    }

  if (calcR(0, TAP_DETECTION_COUNT - mDetectionCount) > mLongThres)
    {
      mStab = 0;
      return false;
    }
  if (++mStab <= mStabFrame)
    {
      return false;
    }

  mDetectionCount = 0;
  return true;
}

/****************************************************************************
 * Name: getIndex
 *
 * Description:
 *  Get index necessary for tap detection.
 *
 * Input Parameters:
 *   idx
 *
 * Returned Value:
 *   Index
 *
 * Assumptions/Limitations:
 *   -
 *
 ****************************************************************************/
float TapClass::getIndex(int idx)
{
  int i = mIndex - idx - 1;

  if (i < 0)
    {
      i += TAP_BUF_LEN;
    }

  return i;
}
