
#include <nuttx/config.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>
#include <arch/chip/pin.h>

#define PIN_SIG PIN_UART2_TXD

int main(int argc, FAR char *argv[])
{
  irqstate_t flags;

  board_gpio_config(PIN_SIG, 0, false, true, PIN_FLOAT);
  board_gpio_write(PIN_SIG, 0);

  /* Completely prohibit interrupt, to make
   * the interval as accurate as possible.
   */

  flags = enter_critical_section();

  while (1)
    {
      board_gpio_write(PIN_SIG, 0);
      up_mdelay(900);
      board_gpio_write(PIN_SIG, 1);
      up_mdelay(100);
    }

  leave_critical_section(flags);
  return 0;
}
