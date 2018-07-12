/****************************************************************************
 * bsp/src/audio/cxd56_audio_pin.c
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

#include <sdk/config.h>
#include "cxd56_pinconfig.h"
#include "audio/cxd56_audio_config.h"
#include "audio/cxd56_audio_pin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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

void audio_pin_setmclk(void)
{
  CXD56_PIN_CONFIGS(PINCONFS_MCLK);
}

/*--------------------------------------------------------------------------*/
void audio_pin_unsetmclk(void)
{
  CXD56_PIN_CONFIGS(PINCONFS_MCLK_GPIO);
}

/*--------------------------------------------------------------------------*/
void audio_pin_setpdm(void)
{
  if (CXD56_AUDIO_CFG_PDM_DS == CXD56_AUDIO_CFG_LOEMI_2MA)
    {
      CXD56_PIN_CONFIGS(PINCONFS_PDM_NORM);
    }
  else
    {
      CXD56_PIN_CONFIGS(PINCONFS_PDM_HIGH);
    }
}

/*--------------------------------------------------------------------------*/
void audio_pin_unsetpdm(void)
{
  CXD56_PIN_CONFIGS(PINCONFS_PDM_GPIO);
}

/*--------------------------------------------------------------------------*/
void audio_pin_seti2s(void)
{
#ifdef CONFIG_CXD56_I2S0
  /* I2S1 device setting */

  if (CXD56_AUDIO_CFG_I2S1_MODE == CXD56_AUDIO_CFG_I2S_MODE_MASTER)
    {
      if (CXD56_AUDIO_CFG_PDM_DS == CXD56_AUDIO_CFG_LOEMI_4MA)
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_HIGH);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_NORM);
        }
    }
  else
    {
      if (CXD56_AUDIO_CFG_PDM_DS == CXD56_AUDIO_CFG_LOEMI_2MA)
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_HIGH);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_NORM);
        }
    }
#endif

#ifdef CONFIG_CXD56_I2S1
  /* I2S2 device setting */

  if (CXD56_AUDIO_CFG_I2S2_MODE == CXD56_AUDIO_CFG_I2S_MODE_MASTER)
    {
      if (CXD56_AUDIO_CFG_PDM_DS == CXD56_AUDIO_CFG_LOEMI_4MA)
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_HIGH);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_NORM);
        }
    }
  else
    {
      if (CXD56_AUDIO_CFG_PDM_DS == CXD56_AUDIO_CFG_LOEMI_2MA)
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_HIGH);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_NORM);
        }
    }
#endif
}

/*--------------------------------------------------------------------------*/
void audio_pin_unseti2s(void)
{
#ifdef CONFIG_CXD56_I2S0
  /* I2S0 device setting */

  CXD56_PIN_CONFIGS(PINCONFS_I2S0_GPIO);

#endif

#ifdef CONFIG_CXD56_I2S1

  /* I2S1 device setting */
  CXD56_PIN_CONFIGS(PINCONFS_I2S1_GPIO);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cxd56_audio_pin_set(void)
{
  /* Select MCLK. */

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  audio_pin_setmclk();
#endif

  /* Select PDM_CLK, PDM_IN, PDM_OUT. */

  audio_pin_setpdm();
}

/*--------------------------------------------------------------------------*/
void cxd56_audio_pin_unset(void)
{
  /* Select GPIO(P1x_00). */

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  audio_pin_unsetmclk();
#endif

  /* Select GPIO(P1y_00/01/02). */

  audio_pin_unsetpdm();

  /* Select GPIO(P1v_00/01/02/03, P1w_00/01/02/03). */

  audio_pin_unseti2s();
}

/*--------------------------------------------------------------------------*/
void cxd56_audio_pin_i2s_set(void)
{
  /* Select I2S0/1_BCK, I2S0/1_LRCK, I2S0/1_DATA_IN, I2S0/1_DATA_OUT. */

  audio_pin_seti2s();
}

/*--------------------------------------------------------------------------*/
void cxd56_audio_pin_i2s_unset(void)
{
 /* Select GPIO(P1v_00/01/02/03, P1w_00/01/02/03). */

  audio_pin_unseti2s();
}

