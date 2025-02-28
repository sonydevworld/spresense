#include <asmp/gpio.h>
#include <arch/chip/pin.h>
#include <arm_internal.h>
#include <hardware/cxd5602_memorymap.h>
#include <hardware/cxd5602_topreg.h>

/* Input enable Definitions */

#define PINCONF_IN_EN_SHIFT     (0)
#define PINCONF_IN_EN_MASK      (1u << PINCONF_IN_EN_SHIFT)

#define PINCONF_INPUT_ENABLE    (1u << PINCONF_IN_EN_SHIFT)
#define PINCONF_INPUT_DISABLE   (0u << PINCONF_IN_EN_SHIFT)

#define PINCONF_INPUT_ENABLED(p) (((p) & PINCONF_IN_EN_MASK) == PINCONF_INPUT_ENABLE)

/* GPIO register Definitions */

#define GPIO_OUTPUT_EN_SHIFT    (16)
#define GPIO_OUTPUT_EN_MASK     (1u << GPIO_OUTPUT_EN_SHIFT)
#define GPIO_OUTPUT_ENABLE      (0u << GPIO_OUTPUT_EN_SHIFT)
#define GPIO_OUTPUT_DISABLE     (1u << GPIO_OUTPUT_EN_SHIFT)
#define GPIO_OUTPUT_ENABLED(v)  (((v) & GPIO_OUTPUT_EN_MASK) == GPIO_OUTPUT_ENABLE)
#define GPIO_OUTPUT_SHIFT       (8)
#define GPIO_OUTPUT_MASK        (1u << GPIO_OUTPUT_SHIFT)
#define GPIO_OUTPUT_HIGH        (1u << GPIO_OUTPUT_SHIFT)
#define GPIO_OUTPUT_LOW         (0u << GPIO_OUTPUT_SHIFT)
#define GPIO_INPUT_SHIFT        (0)
#define GPIO_INPUT_MASK         (1u << GPIO_INPUT_SHIFT)

static uint32_t get_gpio_regaddr(uint32_t pin)
{
  uint32_t base;

  base = (pin < PIN_IS_CLK) ? 1 : 7;

  return CXD56_TOPREG_GP_I2C4_BCK + ((pin - base) * 4);
}

static void cxd56_gpio_write_hiz(uint32_t pin)
{   
  uint32_t regaddr;
  uint32_t regval;

  regaddr = get_gpio_regaddr(pin);
  regval = GPIO_OUTPUT_DISABLE;
  putreg32(regval, regaddr);
}

static bool cxd56_gpio_read(uint32_t pin)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t shift;
  uint32_t ioreg;
  uint32_t ioval;

  regaddr = get_gpio_regaddr(pin);
  regval = getreg32(regaddr);

  ioreg = CXD56_TOPREG_IO_RTC_CLK_IN + (pin * 4);
  ioval = getreg32(ioreg);

  if (PINCONF_INPUT_ENABLED(ioval))
    {
      shift = GPIO_INPUT_SHIFT;
    }
  else if (GPIO_OUTPUT_ENABLED(regval))
    {
      shift = GPIO_OUTPUT_SHIFT;
    }
  else
    {
      shift = GPIO_INPUT_SHIFT;
    }

  return ((regval & (1 << shift)) != 0);
}

static void cxd56_gpio_write(uint32_t pin, bool value)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = get_gpio_regaddr(pin);

  if (value)
    {
      regval = GPIO_OUTPUT_ENABLE | GPIO_OUTPUT_HIGH;
    }
  else
    {
      regval = GPIO_OUTPUT_ENABLE | GPIO_OUTPUT_LOW;
    }

  putreg32(regval, regaddr);
}

void board_gpio_write(uint32_t pin, int value)
{ 
  if (value < 0)
    {
      cxd56_gpio_write_hiz(pin);
    }
  else
    {
      cxd56_gpio_write(pin, (value > 0));
    }
}

int board_gpio_read(uint32_t pin)
{
  return (int)cxd56_gpio_read(pin);
}
