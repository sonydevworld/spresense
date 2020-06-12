#include <stdio.h>
#include <stdint.h>

#include <nuttx/board.h>

#include "spresense/src/spresense.h"
#include "asmp/asmp.h"

void board_late_initialize(void)
{
  cxd56_bringup();

#ifdef CONFIG_ASMP
  asmp_initialize();
#endif  
}
