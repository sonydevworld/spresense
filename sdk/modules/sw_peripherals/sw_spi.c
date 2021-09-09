/****************************************************************************
 * modules/sw_peripherals/sw_spi.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include <sw_peripherals/sw_spi.h>

/****************************************************************************
 * Private functions
 ****************************************************************************/

static void set_pin_default(FAR struct swspi_s *dev)
{
  /* Now mode0 is suppoeted only */

  SWSPI_SELECT(dev, 0, false);
  board_gpio_write(dev->sck, 0);
}

static bool exchange_one_bit(FAR struct swspi_s *dev, bool bit)
{
  int recv_bit;

  /* Now mode0 is suppoered only. */

  /* Clock Low */

  board_gpio_write(dev->mosi, bit ? 1 : 0); /* Data set on MOSI */
  up_udelay(dev->delay_us);

  /* Clock High */

  board_gpio_write(dev->sck, 1); /* Latch timing */
  up_udelay(dev->delay_us);
  recv_bit = board_gpio_read(dev->miso); /* Latch MIOSO */

  /* Clock Low */

  board_gpio_write(dev->sck, 0); /* Shift timing */

  return (recv_bit != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** SWSPI_SETUP */

void SWSPI_SETUP(FAR struct swspi_s *dev, int cs_pin,
    int sck_pin, int mosi_pin, int miso_pin)
{
  dev->cs   = cs_pin;
  dev->sck  = sck_pin;
  dev->mosi = mosi_pin;
  dev->miso = miso_pin;

  /* Output pin setting
   * Mode:GPIO, Direction:output, Drivability:high drive, Pull:None
   */

  board_gpio_config(cs_pin,   0 /* GPIO */, false, true, PIN_FLOAT);
  board_gpio_config(sck_pin,  0 /* GPIO */, false, true, PIN_FLOAT);
  board_gpio_config(mosi_pin, 0 /* GPIO */, false, true, PIN_FLOAT);

  /* Input pin setting
   * Mode:GPIO, Direction:input, Drivability:high drive, Pull:None
   */

  board_gpio_config(miso_pin, 0 /* GPIO */, true, true, PIN_FLOAT);

  /* Default parameter settings */

  dev->mode = SPIDEV_MODE0;
  dev->bits = 8;
  dev->delay_us = 1;

  set_pin_default(dev);
}

/** SWSPI_SELECT */

void SWSPI_SELECT(FAR struct swspi_s *dev,
    uint32_t devid, bool selected)
{
  up_udelay(dev->delay_us);
  board_gpio_write(dev->cs, selected ? 0 : 1);
  up_udelay(dev->delay_us);
}

/** SWSPI_SETMODE */

void SWSPI_SETMODE(FAR struct swspi_s *dev, enum spi_mode_e mode)
{
  /* Do nothing, just MODE0 is supported. */
}

/** SWSPI_SETBITS */

void SWSPI_SETBITS(FAR struct swspi_s *dev, int bits)
{
  /* Do nothing, just 8 bit is supported. */
}

/** SWSPI_SETFREQUENCY */

uint32_t SWSPI_SETFREQUENCY(FAR struct swspi_s *dev, uint32_t freq)
{
  /* Support only 1Mbps now */

  return 1000000;
}

/** SWSPI_EXCHANGE */

void SWSPI_EXCHANGE(FAR struct swspi_s *dev,
    FAR const void *txbuff, FAR void *rxbuff, size_t nwords)
{
  size_t words;
  uint32_t bitmask;
  uint8_t dummy_tx = 0;
  uint8_t dummy_rx;

  uint8_t *odata;
  uint8_t *idata;
 
  odata = txbuff != NULL ? (uint8_t *)txbuff : &dummy_tx;
  idata = rxbuff != NULL ? (uint8_t *)rxbuff : &dummy_rx;

  /* TODO: now 8 bits per word is supported only. */

  for (words = 0; words < nwords; words++)
    {
      /* MSB first only */

      *idata = 0;
      for (bitmask = (1<<(dev->bits - 1)); bitmask != 0; bitmask >>= 1)
        {
          *idata |= exchange_one_bit(dev, (*odata & bitmask) ? true : false)
            ? bitmask : 0;
        }

      if (txbuff != NULL)
        {
          odata++;
        }

      if (rxbuff != NULL)
        {
          idata++;
        }
    }
}
