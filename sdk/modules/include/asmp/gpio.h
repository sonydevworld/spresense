#ifndef __INCLUDE_ASMP_GPIO_H
#define __INCLUDE_ASMP_GPIO_H

#include <stdint.h>
#include <arch/chip/pin.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

void board_gpio_write(uint32_t pin, int value);
int board_gpio_read(uint32_t pin);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ASMP_GPIO_H */
