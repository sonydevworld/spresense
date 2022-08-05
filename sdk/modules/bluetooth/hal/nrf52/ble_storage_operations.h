/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_storage_operations.h
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_BLUETOOTH_HAL_NRF52_BT_STORAGE_OPERATION_H
#define __MODULES_BLUETOOTH_HAL_NRF52_BT_STORAGE_OPERATION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <stdint.h>
/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct BSO_KeyPair_Tag
{
    uint32_t key;
    const void* value;
    uint32_t size;
} BSO_KeyPair;

// APIs should be called in a single thread, not thread safe
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: BSO_Init
 *
 * Description:
 *   Bluetooth Storage Operation Initialize.
 *   Open a bluetooth database file and check.
 *
 ****************************************************************************/

int BSO_Init(void* data);

/****************************************************************************
 * Name: BSO_Sync
 *
 * Description:
 *   Bluetooth Storage Operation Sync.
 *   Sync a bluetooth database file between file and program.
 *
 ****************************************************************************/

void BSO_Sync(void);

/****************************************************************************
 * Name: BSO_Finalize
 *
 * Description:
 *   Bluetooth Storage Operation Finalize.
 *   Finalize a bluetooth database file operation.
 *
 ****************************************************************************/

int BSO_Finalize(void* data);

/****************************************************************************
 * Name: BSO_GetRegistryValue
 *
 * Description:
 *   Bluetooth Storage Operation Get registoty value.
 *   Get a registry value by key.
 *
 ****************************************************************************/

int BSO_GetRegistryValue(uint32_t key, void* value, uint32_t size);

/****************************************************************************
 * Name: BSO_SetRegistryValueList
 *
 * Description:
 *   Bluetooth Storage Operation Set registoty value list.
 *   Set a registry value by list of key.
 *
 ****************************************************************************/

int BSO_SetRegistryValueList(BSO_KeyPair* list, uint32_t pairsNum);

/****************************************************************************
 * Name: BSO_SetRegistryValue
 *
 * Description:
 *   Bluetooth Storage Operation Set registoty value.
 *   Set a registry value by key.
 *
 ****************************************************************************/

int BSO_SetRegistryValue(uint32_t key, const void* value, uint32_t size);

/****************************************************************************
 * Name: BSO_DeleteRegistryKey
 *
 * Description:
 *   Bluetooth Storage Operation Delete registoty value.
 *   Delete a registry value by key.
 *
 ****************************************************************************/

int BSO_DeleteRegistryKey(uint32_t key);

/****************************************************************************
 * Name: BSO_CleanRegistry
 *
 * Description:
 *   Bluetooth Storage Operation cleanup.
 *   Clean up a registry value.
 *
 ****************************************************************************/

int BSO_CleanRegistry(void);

/****************************************************************************
 * Name: BSO_GenerateRegistryKey
 *
 * Description:
 *   Bluetooth Storage Operation generate registration key.
 *   Generate a registry key for set value.
 *
 ****************************************************************************/

uint32_t BSO_GenerateRegistryKey(const char* keyName, uint32_t attr);

#endif /* __MODULES_BLUETOOTH_HAL_NRF52_BT_STORAGE_OPERATION_H */
