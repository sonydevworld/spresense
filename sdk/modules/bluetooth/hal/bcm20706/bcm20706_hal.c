/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_hal.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <bluetooth/hal/bt_if.h>

#include "bcm20706_bt_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcm20706_probe(void)
{
  int ret = BT_SUCCESS;

  ret = bcm20706_bt_common_register();

#ifdef CONFIG_BCM20706_A2DP
  if (ret == BT_SUCCESS)
    {
      ret = bcm20706_bt_a2dp_register();
    }
#endif

#ifdef CONFIG_BCM20706_AVRCP
  if (ret == BT_SUCCESS)
    {
      ret = bcm20706_bt_avrcp_register();
    }
#endif

#ifdef CONFIG_BCM20706_HFP
  if (ret == BT_SUCCESS)
    {
      ret = bcm20706_bt_hfp_register();
    }
#endif

#ifdef CONFIG_BCM20706_SPP
  if (ret == BT_SUCCESS)
    {
      ret = bcm20706_bt_spp_register();
    }
#endif

#ifdef CONFIG_BCM20706_LE
  if (ret == BT_SUCCESS)
    {
      ret = bcm20706_ble_common_register();
    }
#ifdef CONFIG_BCM20706_LE_GATT
  if (ret == BT_SUCCESS)
    {
      ret = bcm20706_ble_gatt_register();
    }
#endif
#endif

  return ret;
}
