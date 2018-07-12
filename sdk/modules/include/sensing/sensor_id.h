/****************************************************************************
 * modules/include/sensing/sensor_id.h
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

#ifndef __INCLUDE_SENSING_SENSOR_ID_H
#define __INCLUDE_SENSING_SENSOR_ID_H

/*
ToDo
Ultimately, should be generated with Config tool etc.
Or, the application user lists oneself.
*/

enum SensorClientID
{
  selfID = 0,
  accelID,        /* 1 */
  accel1ID,       /* 2 */
  magID,          /* 3 */
  pressureID,     /* 4 */
  lightID,        /* 5 */
  pulseID,        /* 6 */
  tempID,         /* 7 */
  gyroID,         /* 8 */
  gnssID,         /* 9 */
  stepcounterID,  /* 10 */
  tramID,         /* 11 */
  gestureID,      /* 12 */
  compassID,      /* 13 */
  barometerID,    /* 14 */
  tramliteID,     /* 15 */
  vadID,          /* 16 */
  wuwsrID,        /* 17 */
  adcID,          /* 18 */
  reserve19ID,    /* 19 */
  app0ID,         /* 20 */
  app1ID,         /* 21 */
  app2ID,         /* 22 */
  app3ID,         /* 23 */
  NumOfSensorClientID
};



#endif /* __INCLUDE_SENSING_SENSOR_ID_H */

