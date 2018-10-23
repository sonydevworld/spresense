/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/manager/bt_storage_manager.h
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

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_MANAGER_BT_STORAGE_MANAGER_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_MANAGER_BT_STORAGE_MANAGER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef FILE BSO_FILE;

typedef struct BSO_KeyPair_Tag
{
    uint32_t key;
    const void* value;
    uint32_t size;
} BSO_KeyPair;

typedef enum BSO_MODE_Tag
{
    BSO_MODE_READ = 1,
    BSO_MODE_WRITE,
    BSO_MODE_APPEND,
    BSO_MODE_MAX
} BSO_MODE;

typedef enum BSO_SEEK_Tag
{
    BSO_SEEK_START = 0,
    BSO_SEEK_CUR,
    BSO_SEEK_END
} BSO_SEEK;

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

/****************************************************************************
 * Name: BSO_FileRead
 *
 * Description:
 *   Bluetooth Storage Operation file read.
 *   Read a file for loading database.
 *
 ****************************************************************************/

int BSO_FileRead(void* buf, uint32_t* size, BSO_FILE* handle);

/****************************************************************************
 * Name: BSO_FileWrite
 *
 * Description:
 *   Bluetooth Storage Operation file write.
 *   Write a file for saving database.
 *
 ****************************************************************************/

int BSO_FileWrite(void* buf, uint32_t size, BSO_FILE* handle);

/****************************************************************************
 * Name: BSO_FileOpen
 *
 * Description:
 *   Bluetooth Storage Operation file open.
 *   Open a file for read database.
 *
 ****************************************************************************/

BSO_FILE* BSO_FileOpen(const char* fileName, int32_t mode);

/****************************************************************************
 * Name: BSO_FileClose
 *
 * Description:
 *   Bluetooth Storage Operation file close.
 *   Close a database file.
 *
 ****************************************************************************/

int BSO_FileClose(BSO_FILE* handle);

/****************************************************************************
 * Name: BSO_FileSeek
 *
 * Description:
 *   Bluetooth Storage Operation file seek.
 *   Seek a database file.
 *
 ****************************************************************************/

int BSO_FileSeek(BSO_FILE* handle, int32_t offset, int whence);

/****************************************************************************
 * Name: BSO_FileFlush
 *
 * Description:
 *   Bluetooth Storage Operation file flush.
 *   Flush a database file.
 *
 ****************************************************************************/

int BSO_FileFlush(BSO_FILE* handle);

/****************************************************************************
 * Name: BSO_FileDelete
 *
 * Description:
 *   Bluetooth Storage Operation file delete.
 *   Delete a database file.
 *
 ****************************************************************************/

int BSO_FileDelete(const char* fileName);

/****************************************************************************
 * Name: BSO_FileRename
 *
 * Description:
 *   Bluetooth Storage Operation file rename.
 *   Rename a database file.
 *
 ****************************************************************************/

int BSO_FileRename(const char* oldName, const char* newName);

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_MANAGER_BT_STORAGE_MANAGER_H */
