/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_uart_manager.c
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

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_BCM20706_BT_INTERNAL_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_BCM20706_BT_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS_ONE_SLOT   0.625

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  TRANS_TYPE_CLASSIC = 1,
  TRNAS_TYPE_BLE,
} BT_TRANS_TYPE;

typedef enum
{
  ADDR_TYPE_PUBLIC = 0,
  ADDR_TYPE_RANDOM,
} BT_ADDR_TYPE;

/****************************************************************************
 * Public Functions prototype
 ****************************************************************************/

int btSetBtAddress(BT_ADDR *addr);
int btSetBtName(char *name);
int btSetPairingEnable(uint8_t isEnable);

int bcm20706_bt_common_register(void);

#ifdef CONFIG_BCM20706_A2DP
int bcm20706_bt_a2dp_register(void);
#endif

#ifdef CONFIG_BCM20706_AVRCP
int bcm20706_bt_avrcp_register(void);
#endif

#ifdef CONFIG_BCM20706_HFP
int bcm20706_bt_hfp_register(void);
#endif

#ifdef CONFIG_BCM20706_SPP
int bcm20706_bt_spp_register(void);
#endif

#ifdef CONFIG_BCM20706_LE
int bcm20706_ble_common_register(void);

#ifdef CONFIG_BCM20706_LE_GATT
int bcm20706_ble_gatt_register(void);
#endif
#endif

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_BCM20706_BT_INTERNAL_H */

