/****************************************************************************
 * sdk/modules/imageproc/imageproc.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdio.h>

#include <fcntl.h>
#include <time.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <imageproc/imageproc.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

/****************************************************************************
 * CXD5602 Register Address definitions.
 ****************************************************************************/

#define CXD56_ROT_BASE      (CXD56_ADSP_BASE + 0x02101400)
#define ROT_INTR_STATUS     (CXD56_ROT_BASE  + 0x0000)
#define ROT_INTR_ENABLE     (CXD56_ROT_BASE  + 0x0004)
#define ROT_INTR_DISABLE    (CXD56_ROT_BASE  + 0x0008)
#define ROT_INTR_CLEAR      (CXD56_ROT_BASE  + 0x000C)
#define ROT_SET_DIRECTION   (CXD56_ROT_BASE  + 0x0014)
#define ROT_SET_SRC_HSIZE   (CXD56_ROT_BASE  + 0x0018)
#define ROT_SET_SRC_VSIZE   (CXD56_ROT_BASE  + 0x001C)
#define ROT_SET_SRC_ADDRESS (CXD56_ROT_BASE  + 0x0020)
#define ROT_SET_SRC_PITCH   (CXD56_ROT_BASE  + 0x0024)
#define ROT_SET_DST_ADDRESS (CXD56_ROT_BASE  + 0x0028)
#define ROT_SET_DST_PITCH   (CXD56_ROT_BASE  + 0x002C)
#define ROT_CONV_CTRL       (CXD56_ROT_BASE  + 0x0034)
#define ROT_RGB_ALIGNMENT   (CXD56_ROT_BASE  + 0x0038)
#define ROT_COMMAND         (CXD56_ROT_BASE  + 0x0010)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t rot_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int intr_handler_ROT(int irq, FAR void *context, FAR void *arg)
{
  putreg32(1, ROT_INTR_CLEAR);
  putreg32(0, ROT_INTR_ENABLE);
  putreg32(1, ROT_INTR_DISABLE);

  sem_post(&rot_sem);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imageproc_initialize(void)
{
  sem_init(&rot_sem, 0, 0);

  putreg32(1, ROT_INTR_CLEAR);
  putreg32(0, ROT_INTR_ENABLE);
  putreg32(1, ROT_INTR_DISABLE);

  irq_attach(CXD56_IRQ_ROT, intr_handler_ROT, NULL);
  up_enable_irq(CXD56_IRQ_ROT);
}

void imageproc_finalize(void)
{
  up_disable_irq(CXD56_IRQ_ROT);
  irq_detach(CXD56_IRQ_ROT);

  sem_post(&rot_sem);
  sem_destroy(&rot_sem);
}

void imageproc_convert_yuv2rgb(uint8_t * ibuf, uint32_t hsize, uint32_t vsize)
{
  /*
   * Image processing hardware want to be set horizontal/vertical size to
   * actual size - 1.
   */

  --hsize;
  --vsize;

  putreg32(1, ROT_INTR_ENABLE);
  putreg32(0, ROT_INTR_DISABLE);
  putreg32(0, ROT_SET_DIRECTION);

  putreg32(hsize, ROT_SET_SRC_HSIZE);
  putreg32(vsize, ROT_SET_SRC_VSIZE);
  putreg32((uint32_t)(uintptr_t)ibuf,  ROT_SET_SRC_ADDRESS);

  putreg32(hsize, ROT_SET_SRC_PITCH);
  putreg32((uint32_t)(uintptr_t)ibuf,  ROT_SET_DST_ADDRESS);

  putreg32(hsize, ROT_SET_DST_PITCH);

  putreg32(1, ROT_CONV_CTRL);
  putreg32(0, ROT_RGB_ALIGNMENT);
  putreg32(1, ROT_COMMAND);

  sem_wait(&rot_sem);

  return;
}

void imageproc_convert_yuv2gray(uint8_t *ibuf, uint8_t *obuf, size_t hsize, size_t vsize)
{
  uint16_t *p_src = (uint16_t *) ibuf;
  size_t ix;
  size_t iy;

  for (iy = 0; iy < vsize; iy++)
    {
      for (ix = 0; ix < hsize; ix++)
        {
          *obuf++ = (uint8_t) ((*p_src++ & 0xff00) >> 8);
        }
    }
}

