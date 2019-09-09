/****************************************************************************
 * modules/include/sensing/logical_sensor/physical_command.h
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

#ifndef __INCLUDE_SENSING_PHYSICAL_COMMAND_H
#define __INCLUDE_SENSING_PHYSICAL_COMMAND_H

/**
 * @defgroup physical Physical sensor API
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/**
 * @file physical_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
/**
 * @struct ThreeAxisSample
 * @brief Structure of physical sensor sample data using 3 axis.
 */
typedef struct {

  float ax;  /**<
              * Acceleration sensor:
              * X axis standard gravity acceleration. The unit is [G].
              * Magnetometer sensor:
              * Northward component, X. The unit is [uT].
              */
  float ay;  /**<
              * Acceleration sensor:
              * Y axis standard gravity acceleration. The unit is [G].
              * Magnetometer sensor:
              * Northward component, Y. The unit is [uT].
              */
  float az;  /**<
              * Acceleration sensor:
              * Z axis standard gravity acceleration. The unit is [G].
              * Magnetometer sensor:
              * Northward component, Z. The unit is [uT].
              */
} ThreeAxisSample;

/*--------------------------------------------------------------------------*/
/**
 * @struct ThreeAxesSampleData
 * @brief the frame of 3 axis data.
 */
typedef struct
{
  uint32_t time_stamp;    /**< Time stamp at update.[ms]                       */
  uint16_t sampling_rate; /**< Sampling frequency of acceleration data sample. */
  uint16_t sample_num;    /**< 3 axis table number of samples.                 */
  FAR ThreeAxisSample *p_data; /**< 3 axis table sample data.                  */
} ThreeAxisSampleData;

/*--------------------------------------------------------------------------*/
/**
 * @struct BarSampleData
 * @brief the frame of Barometer data.
 */
typedef struct
{
  uint32_t time_stamp;    /**< Time stamp at update.[ms]                    */
  uint16_t sampling_rate; /**< Sampling frequency of Barometer data sample. */
  uint16_t sample_num;    /**< Barometer number of samples.                 */
  FAR uint32_t *p_data;   /**< Barometer sample data.                       */
} BarSampleData;

/*--------------------------------------------------------------------------*/
/**
 * @struct GnssData
 * @brief the struct of GNSS data for the execute command.
 */
typedef struct
{
  double   raw_latitude;  /**<
                           * Unfiltered latitude.
                           * The unit is [degrees].
                           */
  double   raw_longitude; /**<
                           * Longitude not filtered. The unit is [degree].
                           */
  double   latitude;      /**< Filtered latitude. The unit is [degrees]. */
  double   longitude;     /**< Filtered longitude. The unit is [degrees]. */
  float    direction;     /**<
                           * Indicates the direction. The unit is [degree].
                           */
  float    velocity;      /**< Indicates speed. The unit is [m/s]. */
  uint32_t time_stamp;    /**< Time stamp at update [ms]. */
  uint8_t  pos_fix_mode;  /**< GNSS position compensation mode. */
  uint8_t  vel_fix_mode;  /**< GNSS speed compensation mode. */
} GnssSampleData;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_PHYSICAL_COMMAND_H */

