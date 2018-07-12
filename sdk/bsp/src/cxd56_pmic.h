/****************************************************************************
 * bsp/src/cxd56_pmic.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_PMIC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_PMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_CXD56_PMIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMIC Register Definitions */

#define PMIC_REG_DDC_ANA1   (0x26)
#define PMIC_REG_CNT_USB2   (0x81)

/* PMIC DDC_ANA1 Definitions */

#define PMIC_PM_HIZ     (2u << 4)
#define PMIC_PM_DEF     (0u << 4)
#define PMIC_IOST_DEF   (2u << 2)
#define PMIC_IOMAX_DEF  (2u << 0)

/* PMIC CNT_USB2 Definitions */

#define PMIC_SET_CHGOFF (1u << 2)

/* PMIC Interrupt Status Definitions */

#define PMIC_INT_ALARM  (1u << 0)
#define PMIC_INT_WKUPS  (1u << 1)
#define PMIC_INT_WKUPL  (1u << 2)
#define PMIC_INT_VSYS   (1u << 3)

/* PMIC GPO Channel Definitions */

#define PMIC_GPO0       (1u << 0)
#define PMIC_GPO1       (1u << 1)
#define PMIC_GPO2       (1u << 2)
#define PMIC_GPO3       (1u << 3)
#define PMIC_GPO4       (1u << 4)
#define PMIC_GPO5       (1u << 5)
#define PMIC_GPO6       (1u << 6)
#define PMIC_GPO7       (1u << 7)

/* PMIC LoadSwitch Channel Definitions */

#define PMIC_LOADSW2    (1u << 2)
#define PMIC_LOADSW3    (1u << 3)
#define PMIC_LOADSW4    (1u << 4)

/* PMIC DDC/LDO Channel Definitions */

#define PMIC_DDC_IO     (1u << 0)
#define PMIC_LDO_EMMC   (1u << 1)
#define PMIC_DDC_ANA    (1u << 2)
#define PMIC_LDO_ANA    (1u << 3)
#define PMIC_DDC_CORE   (1u << 4)
#define PMIC_LDO_PERI   (1u << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_pmic_get_interrupt_status
 *
 * Description:
 *   Get Raw Interrupt Status register. And furthermore, if status is set,
 *   then clear the interrupt. Register's decription is below:
 *
 *     7   6   5   4    3     2     1     0
 *   +---------------+-----+-----+-----+-----+
 *   | x   x   x   x |VSYS |WKUPL|WKUPS|ALARM| target of status
 *   +---------------+-----+-----+-----+-----+
 *
 * Input Parameter:
 *   status - a pointer to the polled value of interrupt status
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   status - return the value of interrupt status register
 *
 ****************************************************************************/

int cxd56_pmic_get_interrupt_status(uint8_t *status);

/****************************************************************************
 * Name: cxd56_pmic_set_gpo_reg
 *
 * Description:
 *   Set GPO register. Register's decription is below:
 *
 *     7   6   5   4   3   2   1   0
 *   +---+---+---+---+---+---+---+---+
 *   |CH3|CH2|CH1|CH0|CH3|CH2|CH1|CH0| target of setbit0/clrbit0
 *   +---+---+---+---+---+---+---+---+
 *   +---+---+---+---+---+---+---+---+
 *   |CH7|CH6|CH5|CH4|CH7|CH6|CH5|CH4| target of setbit1/clrbit1
 *   +---+---+---+---+---+---+---+---+
 *   |<- 0: Hi-Z   ->|<- 0: Low    ->|
 *   |<- 1: Output ->|<- 1: High   ->|
 *
 * Input Parameter:
 *   setbitX - set bit that 1 is set
 *   clrbitX - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbitX - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo_reg(uint8_t *setbit0, uint8_t *clrbit0,
                           uint8_t *setbit1, uint8_t *clrbit1);

/****************************************************************************
 * Name: cxd56_pmic_set_gpo
 *
 * Description:
 *   Set High/Low to the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset - GPO Channel number(s)
 *   value - true if output high, false if output low.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo(uint8_t chset, bool value);

/****************************************************************************
 * Name: cxd56_pmic_set_gpo_hiz
 *
 * Description:
 *   Set Hi-Z to the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset - GPO Channel number(s)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo_hiz(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_get_gpo
 *
 * Description:
 *   Get the value from the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset : GPO Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are high. Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_gpo(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_set_loadswitch_reg
 *
 * Description:
 *   Set LoadSwitch register. Register's decription is below:
 *
 *     7   6   5   4   3   2   1   0
 *   +---+---+---+---+---+---+---+---+
 *   | - | - | - |CH4|CH3|CH2| 1 | 1 | target of setbit/clrbit
 *   +---+---+---+---+---+---+---+---+
 *               |<- 0: Off->|
 *               |<- 1: On ->|
 *
 * Input Parameter:
 *   setbit - set bit that 1 is set
 *   clrbit - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbit - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_loadswitch_reg(uint8_t *setbit, uint8_t *clrbit);

/****************************************************************************
 * Name: cxd56_pmic_set_loadswitch
 *
 * Description:
 *   Set On/Off to the specified LoadSwitch channel(s)
 *
 * Input Parameter:
 *   chset - LoadSwitch Channel number(s)
 *   value - true if set on, false if set off.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_loadswitch(uint8_t chset, bool value);

/****************************************************************************
 * Name: cxd56_pmic_get_loadswitch
 *
 * Description:
 *   Get the value from the specified LoadSwitch channel(s)
 *
 * Input Parameter:
 *   chset - LoadSwitch Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are on. Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_loadswitch(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_set_ddc_ldo_reg
 *
 * Description:
 *   Set DDC/LDO register. Register's decription is below:
 *
 *      7    6    5    4    3    2    1    0
 *   +----+----+----+----+----+----+----+----+
 *   |    |    |LDO |DDC |LDO |DDC |LDO |DDC | target of setbit/clrbit
 *   | -  | -  |PERI|CORE|ANA |ANA |EMMC|IO  |
 *   +----+----+----+----+----+----+----+----+
 *             |<-  0: Off                 ->|
 *             |<-  1: On                  ->|
 *
 * Input Parameter:
 *   setbit - set bit that 1 is set
 *   clrbit - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbit - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_ddc_ldo_reg(uint8_t *setbit, uint8_t *clrbit);

/****************************************************************************
 * Name: cxd56_pmic_set_ddc_ldo
 *
 * Description:
 *   Set On/Off to the specified DDC/LDO channel(s)
 *
 * Input Parameter:
 *   chset - DDC/LO Channel number(s)
 *   value - true if set on, false if set off.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_ddc_ldo(uint8_t chset, bool value);

/****************************************************************************
 * Name: cxd56_pmic_get_ddc_ldo
 *
 * Description:
 *   Get the value from the specified DDC/LDO channel(s)
 *
 * Input Parameter:
 *   chset - DDC/LDO Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are on. Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_ddc_ldo(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_read
 *
 * Description:
 *   Read the value from the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to read buffer
 *   size - byte count of read
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_read(uint8_t addr, void *buf, uint32_t size);

/****************************************************************************
 * Name: cxd56_pmic_write
 *
 * Description:
 *   Write the value to the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to write buffer
 *   size - byte count of write
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_write(uint8_t addr, void *buf, uint32_t size);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_CXD56_PMIC */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_PMIC_H */
