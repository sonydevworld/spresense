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
#include <nuttx/spi/spi_bitbang.h>
#include <nuttx/kmalloc.h>

#include <sw_peripherals/sw_spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SPI_BITBANG_VARWIDTH
#define BIT_PER_WORD(dev)  ((dev)->nbit)
#else
#define BIT_PER_WORD(dev)  (8)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void swspi_select(FAR struct spi_bitbang_s *dev, uint32_t devid,
    bool selected);
static uint32_t swspi_setfrequency(FAR struct spi_bitbang_s *priv,
    uint32_t frequency);
static void swspi_setmode(FAR struct spi_bitbang_s *dev,
    enum spi_mode_e mode);
static uint16_t swspi_exchange(FAR struct spi_bitbang_s *priv,
    uint16_t dataout);
static uint8_t swspi_status(FAR struct spi_bitbang_s *priv, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int swspi_cmddata(FAR struct spi_bitbang_s *priv, uint32_t devid,
    bool cmd);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct swspi_s
{
  int cs;
  int sck;
  int mosi;
  int miso;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_bitbang_ops_s swspi_ops =
{
  swspi_select,
  swspi_setfrequency,
  swspi_setmode,
  swspi_exchange,
  swspi_status,
#ifdef CONFIG_SPI_CMDDATA
  swspi_cmddata,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
/****************************************************************************
 * Name: set_pin_default
 ****************************************************************************/

static void set_pin_default(FAR struct swspi_s *dev)
{
  /* Now mode0 is suppoeted only */

  board_gpio_write(dev->cs, 1);
  board_gpio_write(dev->sck, 0);
}

/****************************************************************************
 * Name: exchange_one_bit
 ****************************************************************************/

static bool exchange_one_bit(FAR struct swspi_s *dev, bool bit)
{
  int recv_bit;

  /* Now mode0 is suppoered only. */

  /* Clock Low */

  board_gpio_write(dev->mosi, bit ? 1 : 0); /* Data set on MOSI */

  /* Clock High */

  board_gpio_write(dev->sck, 1); /* Latch timing */
  recv_bit = board_gpio_read(dev->miso); /* Latch MIOSO */

  /* Clock Low */

  board_gpio_write(dev->sck, 0); /* Shift timing */

  return (recv_bit != 0);
}

/****************************************************************************
 * Name: swspi_select
 ****************************************************************************/

static void swspi_select(FAR struct spi_bitbang_s *priv,
    uint32_t devid, bool selected)
{
  FAR struct swspi_s *dev = (FAR struct swspi_s *)priv->priv;

  up_udelay(1);
  board_gpio_write(dev->cs, selected ? 0 : 1);
  up_udelay(1);
}

/****************************************************************************
 * Name: swspi_setmode
 ****************************************************************************/

static void swspi_setmode(FAR struct spi_bitbang_s *dev,
    enum spi_mode_e mode)
{
  /* Do nothing, just MODE0 is supported. */
}

/****************************************************************************
 * Name: swspi_setfrequency
 ****************************************************************************/

static uint32_t swspi_setfrequency(FAR struct spi_bitbang_s *dev,
    uint32_t freq)
{
  /* Support only 1Mbps now */

  return 1000000;
}

/****************************************************************************
 * Name: swspi_exchange
 ****************************************************************************/

static uint16_t swspi_exchange(FAR struct spi_bitbang_s *dev,
  uint16_t dataout)
{
  uint16_t rxdata;
  uint16_t bitmask;

  rxdata = 0;
  for (bitmask = (1<<(BIT_PER_WORD(dev) - 1)); bitmask != 0; bitmask >>= 1)
    {
      rxdata |= exchange_one_bit((struct swspi_s *)dev->priv,
          (dataout & bitmask) ? true : false)
        ? bitmask : 0;
    }

  return rxdata;
}

/****************************************************************************
 * Name: swspi_status
 ****************************************************************************/

static uint8_t swspi_status(FAR struct spi_bitbang_s *priv, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: swspi_cmddata
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int swspi_cmddata(FAR struct spi_bitbang_s *priv, uint32_t devid,
    bool cmd)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: create_swspi
 ****************************************************************************/

FAR struct spi_dev_s *create_swspi(int cs_pin,
    int sck_pin, int mosi_pin, int miso_pin)
{
  FAR struct spi_dev_s *dev = NULL;
  FAR struct swspi_s *priv;

  priv = (FAR struct swspi_s *)kmm_zalloc(sizeof(struct swspi_s));
  if (priv)
    {
      priv->cs   = cs_pin;
      priv->sck  = sck_pin;
      priv->mosi = mosi_pin;
      priv->miso = miso_pin;

      dev = spi_create_bitbang(&swspi_ops, (FAR void *)priv);
      if (dev)
        {
          /* Output pin setting
           * Mode:GPIO, Direction:output, Drivability:high drive, Pull:None
           */

          board_gpio_config(cs_pin,   0 , false, true, PIN_FLOAT);
          board_gpio_config(sck_pin,  0 , false, true, PIN_FLOAT);
          board_gpio_config(mosi_pin, 0 , false, true, PIN_FLOAT);

          /* Input pin setting
           * Mode:GPIO, Direction:input, Drivability:high drive, Pull:None
           */

          board_gpio_config(miso_pin, 0 , true, true, PIN_FLOAT);

          /* Default parameter settings */

          set_pin_default(priv);
        }
      else
        {
          kmm_free(priv);
        }
    }

  return dev;
}

/****************************************************************************
 * Name: destroy_swspi
 ****************************************************************************/

void destroy_swspi(FAR struct spi_dev_s *spi)
{
  FAR struct spi_bitbang_s *dev = (FAR struct spi_bitbang_s *)spi;

  if (dev)
    {
      if (dev->priv)
        {
          kmm_free(dev->priv);
        }
      spi_destroy_bitbang(spi);
    }
}
