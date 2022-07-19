/****************************************************************************
 * modules/bluetooth/hal/nrf52/bt_storage_manager.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <crc32.h>
#include <errno.h>

#include "queue.h" /* TODO: replace to nuttx/include/queue.h */
#include "ble_storage_operations.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
#define BSO_FILE_PATH      "/var/hostif"

#define BSO_FILE_PATH_NAME BSO_FILE_PATH"/%s"
#define BSO_REGISTRY_DB    "bso_registry_db"
*/
#define REGDB_VALUE_MAX     60
#define REGDB_KEY_NAME_MAX  4
/*
#define BSO_FILE_NAME_MAX  64
#define BSO_FILE_PATH_MAX  128
*/
#define BITS_PER_BYTE      8

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define FLAG_INITIALIZED (1ul << 0)

#define BSO_REG_KEY_READONLY (1u << 15)
#define BSO_REG_KEY_ATTR_MASK (BSO_REG_KEY_READONLY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct BSO_REGDB_RECORD_Tag
{
  uint32_t key;
  uint8_t value[REGDB_VALUE_MAX];
} BSO_REGDB_RECORD;

typedef struct BSO_REGDB_ENTRY
{
  TAILQ_ENTRY(BSO_REGDB_ENTRY) entry;
  BSO_REGDB_RECORD record;
} BSO_REGDB_ENTRY;

typedef struct BSO_CONTEXT_Tag
{
  uint32_t flags;
  TAILQ_HEAD(, BSO_REGDB_ENTRY) regDbHead;
} BSO_CONTEXT;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static BSO_CONTEXT gBsoContext;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int initRegistryDatabase(BSO_CONTEXT* ctx)
{
    TAILQ_INIT(&ctx->regDbHead);

    return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: BSO_Init
 *
 * Description:
 *   Bluetooth Storage Operation Initialize.
 *   Open a bluetooth database file and check.
 *
 ****************************************************************************/


int BSO_Init(void* data)
{
  (void)data;
  int ret          = 0;
  BSO_CONTEXT* ctx = &gBsoContext;

  if (ctx->flags & FLAG_INITIALIZED)
    {
      return 0;
    }

  ret = initRegistryDatabase(ctx);

  if (!ret)
    {
      ctx->flags |= FLAG_INITIALIZED;
    }

  return ret;
}

/****************************************************************************
 * Name: BSO_Sync
 *
 * Description:
 *   Bluetooth Storage Operation Sync.
 *   Sync a bluetooth database file between file and program.
 *
 ****************************************************************************/

void BSO_Sync(void)
{
  /* TODO: impl after persistent storage is ready */
}

/****************************************************************************
 * Name: BSO_Finalize
 *
 * Description:
 *   Bluetooth Storage Operation Finalize.
 *   Finalize a bluetooth database file operation.
 *
 ****************************************************************************/

int BSO_Finalize(void* data)
{
  (void)data;
  int ret                = 0;

  BSO_CONTEXT* ctx       = &gBsoContext;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return 0;
    }

  ret = BSO_CleanRegistry();

  ctx->flags = 0;

  return ret;
}

/****************************************************************************
 * Name: BSO_GetRegistryValue
 *
 * Description:
 *   Bluetooth Storage Operation Get registoty value.
 *   Get a registry value by key.
 *
 ****************************************************************************/

int BSO_GetRegistryValue(uint32_t key, void* value, uint32_t size)
{
  BSO_CONTEXT* ctx       = &gBsoContext;
  BSO_REGDB_ENTRY* entry = NULL;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (size > REGDB_VALUE_MAX)
    {
      return -EINVAL;
    }

  TAILQ_FOREACH(entry, &ctx->regDbHead, entry)
    {
      if (key == entry->record.key)
        {
          memcpy(value, entry->record.value, MIN(size, REGDB_VALUE_MAX));
          return 0;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: BSO_SetRegistryValueList
 *
 * Description:
 *   Bluetooth Storage Operation Set registoty value list.
 *   Set a registry value by list of key.
 *
 ****************************************************************************/

int BSO_SetRegistryValueList(BSO_KeyPair* list, uint32_t pairsNum)
{
  BSO_CONTEXT* ctx       = &gBsoContext;
  BSO_REGDB_ENTRY* entry = NULL;
  uint32_t i             = 0;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (!list)
    {
      return -EINVAL;
    }

  for (i = 0; i < pairsNum; ++i, ++list)
    {
      if (list->size > REGDB_VALUE_MAX)
        {
          return -EINVAL;
        }

      TAILQ_FOREACH(entry, &ctx->regDbHead, entry)
        {
          if (list->key == entry->record.key)
            {
              if (list->key & BSO_REG_KEY_READONLY)
                {
                  return -EACCES;
                }

              memcpy(entry->record.value, list->value, MIN(list->size, REGDB_VALUE_MAX));

              break;
            }
        }

      if (!entry)
        {
          entry = (BSO_REGDB_ENTRY*)malloc(sizeof(BSO_REGDB_ENTRY));

          if (!entry)
            {
              return -ENOSPC;
            }

          memset(entry, 0, sizeof(BSO_REGDB_ENTRY));
          entry->record.key = list->key;
          memcpy(entry->record.value, list->value, MIN(list->size, REGDB_VALUE_MAX));
          TAILQ_INSERT_TAIL(&ctx->regDbHead, entry, entry);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: BSO_SetRegistryValue
 *
 * Description:
 *   Bluetooth Storage Operation Set registoty value.
 *   Set a registry value by key.
 *
 ****************************************************************************/

int BSO_SetRegistryValue(uint32_t key, const void* value, uint32_t size)
{
  BSO_KeyPair kp = {key, value, size};
  return BSO_SetRegistryValueList(&kp, 1);
}

/****************************************************************************
 * Name: BSO_DeleteRegistryKey
 *
 * Description:
 *   Bluetooth Storage Operation Delete registoty value.
 *   Delete a registry value by key.
 *
 ****************************************************************************/

int BSO_DeleteRegistryKey(uint32_t key)
{
  BSO_CONTEXT* ctx       = &gBsoContext;
  BSO_REGDB_ENTRY* entry = NULL;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  TAILQ_FOREACH(entry, &ctx->regDbHead, entry)
    {
      if (key == entry->record.key)
        {
          break;
        }
    }

  if (entry)
    {
      TAILQ_REMOVE(&ctx->regDbHead, entry, entry);
      free(entry);
      entry = NULL;
    }
  else
    {
      return -ENOENT;
    }

  return 0;
}

/****************************************************************************
 * Name: BSO_CleanRegistry
 *
 * Description:
 *   Bluetooth Storage Operation cleanup.
 *   Clean up a registry value.
 *
 ****************************************************************************/

int BSO_CleanRegistry(void)
{
  BSO_CONTEXT* ctx       = &gBsoContext;
  BSO_REGDB_ENTRY* entry = NULL;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  while (!TAILQ_EMPTY(&ctx->regDbHead))
    {
      entry = TAILQ_FIRST(&ctx->regDbHead);
      TAILQ_REMOVE(&ctx->regDbHead, entry, entry);

      free(entry);
      entry = NULL;
    }

  return 0;
}

/****************************************************************************
 * Name: BSO_GenerateRegistryKey
 *
 * Description:
 *   Bluetooth Storage Operation generate registration key.
 *   Generate a registry key for set value.
 *
 ****************************************************************************/

uint32_t BSO_GenerateRegistryKey(const char* keyName, uint32_t attr)
{
  uint32_t key = 0;
  uint32_t len = strnlen(keyName, REGDB_KEY_NAME_MAX);
  uint32_t i   = 0;

  for (i = 0; i < len; ++i)
    {
      key |= ((uint32_t)keyName[i] << (i * BITS_PER_BYTE));
    }

  key |= (attr & BSO_REG_KEY_ATTR_MASK);

  return key;
}
