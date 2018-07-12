/****************************************************************************
 * bsp/src/cxd56_pmic.c
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
#include <sdk/debug.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include <arch/chip/pm.h>
#include "cxd56_pmic.h"

#ifdef CONFIG_CXD56_PMIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum pmic_cmd_type_e
{
  /* basic */
  PMIC_CMD_READ = 0x00,
  PMIC_CMD_WRITE,
  PMIC_CMD_GPO,
  PMIC_CMD_LOADSW,
  PMIC_CMD_DDCLDO,
  PMIC_CMD_INTSTATUS,
  PMIC_CMD_SETPREVSYS,
  PMIC_CMD_GETPREVSYS,
  PMIC_CMD_SETVSYS,
  PMIC_CMD_GETVSYS,
  /* charger */
  PMIC_CMD_GAUGE = 0x10,
  PMIC_CMD_GET_USB_PORT_TYPE,
  PMIC_CMD_GET_CHG_STATE,
  PMIC_CMD_SET_CHG_VOLTAGE,
  PMIC_CMD_GET_CHG_VOLTAGE,
  PMIC_CMD_SET_CHG_CURRENT,
  PMIC_CMD_GET_CHG_CURRENT,
  PMIC_CMD_SET_CHG_TEMPERATURE_MODE,
  PMIC_CMD_GET_CHG_TEMPERATURE_MODE,
  PMIC_CMD_SET_RECHG_VOLTAGE,
  PMIC_CMD_GET_RECHG_VOLTAGE,
  PMIC_CMD_PRESET_CHG_CURRENT,
  PMIC_CMD_CHG_START,
  PMIC_CMD_CHG_STOP,
  PMIC_CMD_CHG_PAUSE,
  PMIC_CMD_CHG_ENABLE,
  PMIC_CMD_CHG_DISABLE,
};

extern int PM_PmicControl(int cmd, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_CXD56_PMIC_INT
static pmic_notify_t g_pmic_notify[PMIC_NOTIFY_MAX];
static struct work_s g_irqwork;
#endif /* CONFIG_CXD56_PMIC_INT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_CXD56_PMIC_INT
/************************************************************************************
 * Name: is_notify_registerd
 *
 * Description:
 *   Return whether any notification is registered or not
 *
 ************************************************************************************/

static bool is_notify_registerd(void)
{
  int i;

  for (i = PMIC_NOTIFY_ALARM; i < PMIC_NOTIFY_MAX; i++)
    {
      if (g_pmic_notify[i])
        {
          return true;
        }
    }
  return false;
}

/************************************************************************************
 * Name: pmic_int_worker
 *
 * Description:
 *   Work queue for pmic interrupt
 *
 ************************************************************************************/

static void pmic_int_worker(void *arg)
{
  int        i;
  uint8_t    stat;
  irqstate_t flags;
  int        irq = CXD56_IRQ_PMIC;

  /* Get interrupt cause with clear and call the registered callback */

  cxd56_pmic_get_interrupt_status(&stat);

  for (i = PMIC_NOTIFY_ALARM; i < PMIC_NOTIFY_MAX; i++)
    {
      if ((stat & (1 << i)) && g_pmic_notify[i])
        {
          g_pmic_notify[i](arg);
        }
    }

  /* Prevent from the race condition with up_pmic_set_notify() */

  flags = enter_critical_section();

  /* After processing of each notification, enable the pmic interrupt again */

  if (is_notify_registerd())
    {
      up_enable_irq(irq);
    }

  leave_critical_section(flags);
}

/************************************************************************************
 * Name: pmic_int_handler
 *
 * Description:
 *   Interrupt handler for pmic interrupt
 *
 ************************************************************************************/

static int pmic_int_handler(int irq, void *context)
{
  int ret;

  /* Schedule the callback to occur on the low-priority worker thread */

  DEBUGASSERT(work_available(&g_irqwork));
  ret = work_queue(LPWORK, &g_irqwork, pmic_int_worker, NULL, 0);
  if (ret < 0)
    {
      logerr("ERROR: work_queue failed: %d\n", ret);
    }

  /* Disable any further pmic interrupts */

  up_disable_irq(irq);

  return OK;
}
#endif /* CONFIG_CXD56_PMIC_INT */

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

int cxd56_pmic_get_interrupt_status(uint8_t *status)
{
  return PM_PmicControl(PMIC_CMD_INTSTATUS, status);
}

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
                           uint8_t *setbit1, uint8_t *clrbit1)
{
  struct pmic_gpo_arg_s
    {
      uint8_t *setbit0;
      uint8_t *clrbit0;
      uint8_t *setbit1;
      uint8_t *clrbit1;
    }
  arg =
    {
      .setbit0 = setbit0,
      .clrbit0 = clrbit0,
      .setbit1 = setbit1,
      .clrbit1 = clrbit1,
    };

  return PM_PmicControl(PMIC_CMD_GPO, &arg);
}

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

int cxd56_pmic_set_gpo(uint8_t chset, bool value)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;

  /* Set GPO0~3 */

  set = chset & 0xf;
  if (set)
    {
      if (value)
        {
          setbit0 = (set << 4) | set;
        }
      else
        {
          setbit0 = (set << 4);
          clrbit0 = set;
        }
    }

  /* Set GPO4~7 */

  set = (chset >> 4) & 0xf;
  if (set)
    {
      if (value)
        {
          setbit1 = (set << 4) | set;
        }
      else
        {
          setbit1 = (set << 4);
          clrbit1 = set;
        }
    }

  return cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);
}

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

int cxd56_pmic_set_gpo_hiz(uint8_t chset)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;

  /* Set GPO0~3 */

  set = chset & 0xf;
  if (set)
    {
      clrbit0 = (set << 4) | set;
    }

  /* Set GPO4~7 */

  set = (chset >> 4) & 0xf;
  if (set)
    {
      clrbit1 = (set << 4) | set;
    }

  return cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);
}

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

bool cxd56_pmic_get_gpo(uint8_t chset)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;

  cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);

  set = ((setbit1 & 0xf) << 4) | (setbit0 & 0xf);

  /* If all of the specified chset is high, return true */

  if ((set & chset) == chset)
    {
      return true;
    }
  return false;
}

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

int cxd56_pmic_set_loadswitch_reg(uint8_t *setbit, uint8_t *clrbit)
{
  struct pmic_loadswitch_arg_s
    {
      uint8_t *setbit;
      uint8_t *clrbit;
    }
  arg =
    {
      .setbit = setbit,
      .clrbit = clrbit,
    };

  return PM_PmicControl(PMIC_CMD_LOADSW, &arg);
}

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

int cxd56_pmic_set_loadswitch(uint8_t chset, bool value)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  if (value)
    {
      setbit = chset;
    }
  else
    {
      clrbit = chset;
    }
  return cxd56_pmic_set_loadswitch_reg(&setbit, &clrbit);
}

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

bool cxd56_pmic_get_loadswitch(uint8_t chset)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  cxd56_pmic_set_loadswitch_reg(&setbit, &clrbit);

  return ((setbit & chset) == chset);
}

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

int cxd56_pmic_set_ddc_ldo_reg(uint8_t *setbit, uint8_t *clrbit)
{
  struct pmic_ddc_ldo_arg_s
    {
      uint8_t *setbit;
      uint8_t *clrbit;
    }
  arg =
    {
      .setbit = setbit,
      .clrbit = clrbit,
    };

  return PM_PmicControl(PMIC_CMD_DDCLDO, &arg);
}

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

int cxd56_pmic_set_ddc_ldo(uint8_t chset, bool value)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  if (value)
    {
      setbit = chset;
    }
  else
    {
      clrbit = chset;
    }
  return cxd56_pmic_set_ddc_ldo_reg(&setbit, &clrbit);
}

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

bool cxd56_pmic_get_ddc_ldo(uint8_t chset)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  cxd56_pmic_set_ddc_ldo_reg(&setbit, &clrbit);

  return ((setbit & chset) == chset);
}

#ifdef CONFIG_CXD56_PMIC_INT
/****************************************************************************
 * Name: up_pmic_set_notify
 *
 * Description:
 *   Register a callback for pmic interrupt
 *
 * Input Parameter:
 *   kind - A kind of pmic interrupt defined as pmic_notify_e
 *   cb - A callback function for a kind of pmic interrupt
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int up_pmic_set_notify(int kind, pmic_notify_t cb)
{
  static int is_first = 1;
  irqstate_t flags;
  int        irq = CXD56_IRQ_PMIC;
  int        delayed_work = 0;

  if ((kind < PMIC_NOTIFY_ALARM) || (PMIC_NOTIFY_MAX <= kind))
    {
      return -EINVAL;
    }

  /* attach interrupt handler only for the first time */

  if (is_first)
    {
      irq_attach(irq, pmic_int_handler);
      is_first = 0;
    }

  flags = enter_critical_section();

  g_pmic_notify[kind] = cb;

  if (is_notify_registerd())
    {
      /* If up_enable_irq() is called when interrupt is pending,
       * an assertion may occur because workqueue is not prepared yet.
       * As the workaround, call worker directly and in the worker
       * both interrupt clear and enable processing are performed.
       */

      delayed_work = 1;
    }
  else
    {
      up_disable_irq(irq);
    }

  leave_critical_section(flags);

  if (delayed_work)
    {
      pmic_int_worker(NULL);
    }

  return 0;
}
#endif /* CONFIG_CXD56_PMIC_INT */

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

int cxd56_pmic_read(uint8_t addr, void *buf, uint32_t size)
{
  struct pmic_trans_arg_s
    {
      uint8_t  addr;
      void     *buf;
      uint32_t size;
    }
  arg =
    {
      .addr = addr,
      .buf = buf,
      .size = size,
    };

  return PM_PmicControl(PMIC_CMD_READ, &arg);
}

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

int cxd56_pmic_write(uint8_t addr, void *buf, uint32_t size)
{
  struct pmic_trans_arg_s
    {
      uint8_t  addr;
      void     *buf;
      uint32_t size;
    }
  arg =
    {
      .addr = addr,
      .buf = buf,
      .size = size,
    };

  return PM_PmicControl(PMIC_CMD_WRITE, &arg);
}

#endif /* CONFIG_CXD56_PMIC */
