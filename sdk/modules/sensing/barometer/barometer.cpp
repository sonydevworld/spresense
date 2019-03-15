/****************************************************************************
 * modules/sensing/barometer/barometer.cpp
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

#include <sys/time.h>
#include <debug.h>

#include "sensing/logical_sensor/barometer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int BarometerClass::open(void)
{
  return 0;
}

/*--------------------------------------------------------------------*/
int BarometerClass::close(void)
{
  return 0;
}

/*--------------------------------------------------------------------*/
int BarometerClass::start(void)
{
  /* Pressure and Temperture sensor Power ON request. */

  sensor_command_power_t packet;

  /* Create command. */

  packet.header.code   = SetPower;
  packet.header.size   = 4; /*tentative*/
  packet.self          = barometerID;
  packet.subscriptions = (0x01 << pressureID) | (0x01 << tempID);

  SS_SendSensorSetPower(&packet);

  return 0;
}

/*--------------------------------------------------------------------*/
int BarometerClass::stop(void)
{
  /* Sensors Power OFF request. */

  sensor_command_power_t packet;

  /* Create command. */

  packet.header.code   = ClearPower;
  packet.header.size   = 4; /*tentative*/
  packet.self          = barometerID;
  packet.subscriptions = (0x01 << pressureID) | (0x01 << tempID);

  SS_SendSensorClearPower(&packet);

  return 0;
}

/*--------------------------------------------------------------------*/
int BarometerClass::write(sensor_command_data_mh_t* command)
{
  switch (command->self)
    {
      case pressureID:
        {
          this->pressureDate = command->mh;
          this->isReceivedPressureData = true;
        }
        break;

      case tempID:
        {
          this->temperatureData = command->mh;
          this->isReceivedTemperatureData = true;
        }
        break;

      default:
        break;
    }

  if (this->isReceivedPressureData && this->isReceivedTemperatureData)
    {
      this->compemsate();

      sensor_command_data_mh_t packet;
      packet.header.size = 0;
      packet.header.code = SendData;
      packet.self        = barometerID;
      packet.time        = command->time;
      packet.fs          = BAROMETER_PRESSURE_SAMPLING_FREQUENCY;
      packet.size        = BAROMETER_PRESSURE_WATERMARK_NUM;
      packet.mh          = this->pressureDate;

      SS_SendSensorDataMH(&packet);

      this->pressureDate.freeSeg();
      this->temperatureData.freeSeg();
      this->isReceivedPressureData = false;
      this->isReceivedTemperatureData = false;
    }
  
  return 0;
}

/*--------------------------------------------------------------------*/
void BarometerClass::compemsate(void)
{
  int32_t* p_temp = (int32_t*)this->temperatureData.getVa();
  uint32_t* p_pres = (uint32_t*)this->pressureDate.getVa();
  
  for (int i = 0; i < BAROMETER_PRESSURE_WATERMARK_NUM; i++, p_temp++, p_pres++)
    {
      int t = this->compensateTemperature(*p_temp);
      *p_pres = this->compensatePressure(*p_pres, t);
    }
 }

/*--------------------------------------------------------------------*/
void BarometerClass::setAdjustParam(struct bmp280_press_adj_s* param)
{
  this->press_adj = *param;
}
  
/*--------------------------------------------------------------------*/
void BarometerClass::setAdjustParam(struct bmp280_temp_adj_s* param)
{
  this->temp_adj = *param;
}
  
/****************************************************************************
 * Name: compensateTemperature
 *
 * Description:
 *   calculate compensated tempreture
 *
 * Input Parameters:
 *   adc_T - uncompensated value of tempreture.
 *
 * Returned Value:
 *   result of compensated tempreture.
 *   to get in 0.01 degree Centigrade, calulate as below.
 *   (T * 5 + 128) >> 8 [0.01 C]
 *
 ****************************************************************************/

int32_t BarometerClass::compensateTemperature(int32_t adc_T)
{
  int32_t var1;
  int32_t var2;
  int32_t T;

  var1 = ((((adc_T >> 3) - ((int32_t)temp_adj.dig_T1 << 1))) *
          ((int32_t)temp_adj.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)temp_adj.dig_T1)) *
          ((adc_T >> 4) - ((int32_t)temp_adj.dig_T1))) >> 12) *
            ((int32_t)temp_adj.dig_T3)) >> 14;

  T = var1 + var2;

  return T;
}

/****************************************************************************
 * Name: compensatePressure
 *
 * Description:
 *   calculate compensated pressure
 *
 * Input Parameters:
 *   adc_P - uncompensated value of pressure.
 *   comp_T - compensated value of temperature.
 *
 * Returned Value:
 *   result of compensated pressure.
 *
 ****************************************************************************/

uint32_t BarometerClass::compensatePressure(int32_t adc_P, int32_t comp_T)
{
  int32_t var1;
  int32_t var2;
  uint32_t p;

  var1 = (((int32_t)comp_T) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)press_adj.dig_P6);
  var2 = var2 + ((var1 * ((int32_t)press_adj.dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)press_adj.dig_P4) << 16);
  var1 = (((press_adj.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) +
          ((((int32_t)press_adj.dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)press_adj.dig_P1)) >> 15);

  /* avoid exception caused by division by zero */

  if (var1 == 0)
    {
      return 0;
    }

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

  if (p < 0x80000000)
    {
      p = (p << 1) / ((uint32_t)var1);
    }
  else
    {
      p = (p / (uint32_t)var1) * 2;
    }

  var1 = (((int32_t)press_adj.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)press_adj.dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + press_adj.dig_P7) >> 4));

  return p;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

BarometerClass* BarometerCreate(void)
{
  return new BarometerClass(barometerID);
}

int BarometerOpen(FAR BarometerClass *ins)
{
  int ret;

  ret = ins->open();

  return ret;
}

int BarometerClose(FAR BarometerClass *ins)
{
  int ret;

  ret = ins->close();
  delete ins;

  return ret; 
}

int BarometerStart(FAR BarometerClass *ins)
{
  int ret;

  ret = ins->start();

  return ret; 
}

int BarometerStop(FAR BarometerClass *ins)
{
  int ret;

  ret = ins->stop();

  return ret; 
}

int BarometerWrite(FAR BarometerClass *ins, FAR sensor_command_data_mh_t *command)
{
  int ret;

  ret = ins->write(command);

  return ret;
}

