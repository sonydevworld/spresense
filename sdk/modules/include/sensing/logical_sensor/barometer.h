/****************************************************************************
 * modules/include/sensing/logical_sensor/barometer.h
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

#ifndef __INCLUDE_SENSING_BAROMETER_H
#define __INCLUDE_SENSING_BAROMETER_H

/**
 * @defgroup logical_barometer Barometer API 
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <asmp/mpmq.h>
#include <asmp/mptask.h>
#include <nuttx/sensors/bmp280.h>
#include "memutils/memory_manager/MemHandle.h"
#include "sensing/sensor_api.h"
#include "sensing/sensor_id.h"
#include "memutils/s_stl/queue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BAROMETER_PRESSURE_SAMPLING_FREQUENCY    8  /**< Pressure sensor sampling frequency[Hz] */
#define BAROMETER_PRESSURE_WATERMARK_NUM         40 /**< Pressure sample data watermark */
#define BAROMETER_TEMPERATURE_SAMPLING_FREQUENCY 8  /**< Temperature sensor sampling frequency[Hz] */
#define BAROMETER_TEMPERATURE_WATERMARK_NUM      40 /**< Temperature sample data watermark */

#define PRESSURE_SAMPLING_FREQUENCY BAROMETER_PRESSURE_SAMPLING_FREQUENCY
#define PRESSURE_WATERMARK_NUM      BAROMETER_PRESSURE_WATERMARK_NUM

#define TEMPERATURE_SAMPLING_FREQUENCY  BAROMETER_TEMPERATURE_SAMPLING_FREQUENCY
#define TEMPERATURE_WATERMARK_NUM       BAROMETER_TEMPERATURE_WATERMARK_NUM

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct BarometerTempData
 * @brief Input temperature data to DSP.
 */

typedef struct
{
  uint32_t data[BAROMETER_PRESSURE_WATERMARK_NUM];
} BarometerTempData;

/**
 * @struct BarometerPressData
 * @brief Input pressure data to DSP.
 */

typedef struct
{
  uint32_t data[BAROMETER_TEMPERATURE_WATERMARK_NUM];
} BarometerPressData;

/*--------------------------------------------------------------------*/
/*  Barometer Class                                                   */
/*--------------------------------------------------------------------*/

class BarometerClass
{
public:

  /* public methods */
  int open(void);
  int close(void);
  int start(void);
  int stop(void);
  int write(sensor_command_data_mh_t*);
  void setAdjustParam(struct bmp280_press_adj_s* param);
  void setAdjustParam(struct bmp280_temp_adj_s* param);
        
  BarometerClass(SensorClientID id)
    : m_id(id), isReceivedPressureData(false), isReceivedTemperatureData(false)
  {
  };

  ~BarometerClass(){};

private:

  /* private members */

  SensorClientID m_id;

  struct bmp280_press_adj_s press_adj;
  struct bmp280_temp_adj_s temp_adj;

  bool isReceivedPressureData;
  bool isReceivedTemperatureData;
  MemMgrLite::MemHandle  pressureDate;
  MemMgrLite::MemHandle  temperatureData;
  
  void compemsate(void);
  uint32_t compensatePressure(int32_t adc_P, int32_t comp_T);
  int32_t compensateTemperature(int32_t adc_T);

};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Create BarometerClass instance. 
 * @return Address for instance of BarometerClass
 *
 */
BarometerClass* BarometerCreate(void);

/**
 * @brief     Open BarometerClass.
 * @param[in] ins : instance address of BarometerClass
 * @return    result of process.
 */
int BarometerOpen(BarometerClass* ins);

/**
 * @brief     Close BarometerClass.
 * @param[in] ins : instance address of BarometerClass
 * @return    result of process.
 */
int BarometerClose(BarometerClass* ins);

/**
 * @brief     Start Barometer.
 * @param[in] ins : instance address of BarometerClass
 * @return    result of process.
 */
int BarometerStart(BarometerClass* ins);

/**
 * @brief     Stop Barometer.
 * @param[in] ins : instance address of BarometerClass
 * @return    result of process.
 */
int BarometerStop(BarometerClass* ins);

/**
 * @brief     Send data to BarometerClass.
 * @param[in] ins : instance address of BarometerClass
 * @param[in] command : command including data to send
 * @return    result of process
 */
int BarometerWrite(BarometerClass* ins, sensor_command_data_mh_t* command);

/**
 * @brief     Set sensor predefined adjustment values for pressure.
 * @param[in] param : adjustment values
 * @return    
 */
void BarometerSetPressureAdjustParam(struct bmp280_press_adj_s* param);

/**
 * @brief     Set sensor predefined adjustment values for temperature.
 * @param[in] param : adjustment values
 * @return    
 */
void BarometerSetTemperatureAdjustParam(struct bmp280_temp_adj_s* param);

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_BAROMETER_H */

