/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_storage_manager.c
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

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <crc32.h>
#include <errno.h>
#include <debug.h>

#include "queue.h" /* TODO: replace to nuttx/include/queue.h */
#include "manager/bt_storage_manager.h"
#include "bt_debug.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BSO_FILE_PATH      "/mnt/spif"

#define BSO_FILE_PATH_NAME BSO_FILE_PATH"/%s"
#define BSO_REGISTRY_DB    "bso_registry_db"
#define REGDB_VALUE_MAX     160
#define REGDB_KEY_NAME_MAX  4
#define REGDB_REC_CNT_LEN   4
#define REGDB_CRC_LEN       4

#define BSO_FILE_NAME_MAX  64
#define BSO_FILE_PATH_MAX  128

#define BITS_PER_BYTE      8

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define FLAG_INITIALIZED (1ul << 0)

#define BSO_REG_KEY_READONLY (1u << 15)
#define BSO_REG_KEY_ATTR_MASK (BSO_REG_KEY_READONLY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  BSO_REGDB_CRC_OK = 0,
  BSO_REGDB_CRC_NG
} BSO_REGDB_CRC_STAT;

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

typedef struct BSO_FILE_ENTRY
{
  TAILQ_ENTRY(BSO_FILE_ENTRY) entry;
  BSO_FILE* fp;
  uint32_t mode;
  char fileName[BSO_FILE_NAME_MAX];
} BSO_FILE_ENTRY;

typedef struct BSO_CONTEXT_Tag
{
  uint32_t flags;
  TAILQ_HEAD(, BSO_REGDB_ENTRY) regDbHead;
  BSO_FILE* regDbFile;
  TAILQ_HEAD(, BSO_FILE_ENTRY) fpHead;
} BSO_CONTEXT;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static BSO_CONTEXT gBsoContext;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int delRegDb(void)
{
  BSO_CONTEXT* ctx                 = &gBsoContext;
  BSO_FILE* fp                     = ctx->regDbFile;
  char pathBuf[BSO_FILE_PATH_MAX]  = {0};

  snprintf(pathBuf, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, BSO_REGISTRY_DB);
  fp = fopen(pathBuf, "w");
  if (!fp)
    {
      btdbg("registry db delete failed\n");
      return -ENOSPC;
    }
  fclose(fp);

  return 0;
}

static int addEntryToRegDbList(BSO_REGDB_ENTRY* entry, BSO_CONTEXT* ctx)
{
  BSO_FILE* fp = ctx->regDbFile;
  size_t readRec = 0;

  memset(entry, 0, sizeof(BSO_REGDB_ENTRY));
  readRec = fread(&entry->record, sizeof(BSO_REGDB_RECORD), 1, fp);
  btdbg("fread_______entry->record.key = %ld, entry->revord.value = %02x:%02x:%02x:%02x:%02x:%02x\n",\
          entry->record.key,\
          entry->record.value[5],\
          entry->record.value[4],\
          entry->record.value[3],\
          entry->record.value[2],\
          entry->record.value[1],\
          entry->record.value[0]);

  if (ferror(fp))
    {
      free(entry);
      entry = NULL;
      btdbg("entry read error.\n");
      clearerr(fp);
      return -EIO;
    }

  if (feof(fp) || readRec <= 0)
    {
      free(entry);
      entry = NULL;
      return 0;
    }

  TAILQ_INSERT_TAIL(&ctx->regDbHead, entry, entry);

  return (int)entry;
}

static int populateRegistryEntries(BSO_CONTEXT* ctx)
{
  BSO_REGDB_ENTRY* entry = NULL;
  int ret                = 0;

  do
    {
      entry = (BSO_REGDB_ENTRY*)malloc(sizeof(BSO_REGDB_ENTRY));

      if (!entry)
        {
          btdbg("alloc memory for BSO_REGDB_ENTRY failed.\n");
          return -ENOMEM;
        }
    }
  while ((ret = addEntryToRegDbList(entry, ctx)));

  return ret;
}

static uint32_t calcRegDbCrc(const int* const dbFd, const uint32_t recordCount)
{
  uint32_t crc         = 0xffffffff;
  ssize_t readSize     = 0;
  uint32_t recCnt      = recordCount;
  BSO_REGDB_RECORD buf = {0};
  uint32_t i           = 0;

  if (!recCnt)
    {
      /* TODO: check return values */
      (void)lseek(*dbFd, -(REGDB_REC_CNT_LEN + REGDB_CRC_LEN), SEEK_END);
      (void)read(*dbFd, &recCnt, REGDB_REC_CNT_LEN);
      (void)lseek(*dbFd, 0, SEEK_SET);
    }

  for (i = 0;
     ((readSize = read(*dbFd, &buf, sizeof(buf))) > 0) && (i < recCnt);
     ++i)
    {
      crc = crc32part((uint8_t*)&buf, readSize, crc);
    }

  /* TODO: check return values */
  (void)lseek(*dbFd, 0, SEEK_SET);

  return ~crc;
}

static int verifyRegDbCrc(const int* const dbFd)
{
  uint32_t currCrc = 0;
  uint32_t origCrc = 0xffffffff;

  (void)lseek(*dbFd, 0, SEEK_SET);

  currCrc = calcRegDbCrc(dbFd, 0);

  /* TODO: check return values */
  (void)lseek(*dbFd, -(REGDB_CRC_LEN), SEEK_END);
  (void)read(*dbFd, &origCrc, REGDB_CRC_LEN);
  (void)lseek(*dbFd, 0, SEEK_SET);

  return ((currCrc == origCrc) ? BSO_REGDB_CRC_OK : BSO_REGDB_CRC_NG);
}

static int initRegistryDatabase(BSO_CONTEXT* ctx)
{
  int  ret                        = 0;
  char pathBuf[BSO_FILE_PATH_MAX] = {0};
  int regDbFd                     = -1;

  TAILQ_INIT(&ctx->regDbHead);

  snprintf(pathBuf, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, BSO_REGISTRY_DB);

  regDbFd = open(pathBuf, O_CREAT | O_APPEND | O_RDWR);

  if (-1 == regDbFd)
    {
      btdbg("access %s failed.\n", pathBuf);
      return -EIO;
    }

  if (BSO_REGDB_CRC_OK == verifyRegDbCrc(&regDbFd))
    {
      ctx->regDbFile = fdopen(regDbFd, "r");

      if (!ctx->regDbFile) {
        btdbg("open %s failed.\n", pathBuf);
        close(regDbFd);
        delRegDb();
        return -EIO;
      }

      ret = populateRegistryEntries(ctx);
      fclose(ctx->regDbFile);
    }
  else
    {
      close(regDbFd);
      delRegDb();
      btdbg("crc failed\n");
    }

  return ret;
}

static int initFileStorage(BSO_CONTEXT* ctx)
{
  TAILQ_INIT(&ctx->fpHead);
  return 0;
}

static void addRegDbCrc(const char* dbFile, const uint32_t recordCount)
{
  uint32_t crc = 0;
  int fd       = -1;

  fd = open(dbFile, O_RDWR);
  if (fd < 0)
    {
      btdbg("ERROR: Failed to open %s: %d\n", dbFile, errno);
      return;
    }

  /* TODO: check return values */
  (void)lseek(fd, 0, SEEK_SET);
  crc = calcRegDbCrc(&fd, recordCount);
  (void)lseek(fd, 0, SEEK_END);

  (void)write(fd, &recordCount, sizeof(recordCount));
  (void)write(fd, &crc, sizeof(crc));
  (void)close(fd);
}

static void syncRegistryDatabase(BSO_CONTEXT* ctx)
{
  BSO_FILE* fp           = NULL;
  BSO_REGDB_ENTRY* entry = NULL;
  uint32_t recordCount   = 0;
  size_t recordSize      = sizeof(BSO_REGDB_RECORD);
  const size_t NITEMS    = 1;
  int ret                = 0;

  char pathBuf[BSO_FILE_PATH_MAX]  = {0};

  snprintf(pathBuf, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, BSO_REGISTRY_DB);
  fp = fopen(pathBuf, "w");

  TAILQ_FOREACH(entry, &ctx->regDbHead, entry)
    {
      ret = fwrite(&entry->record, recordSize, NITEMS, fp);
      if (ferror(fp) || (NITEMS != ret))
        {
          btdbg("sync fwrite failed.\n");
          goto error;
        }
      ++recordCount;
    }

  if (fflush(fp))
    {
      btdbg("sync fflush failed.\n");
      goto error;
    }

  fclose(fp);
  addRegDbCrc(pathBuf, recordCount);
  return;

error:
  fclose(fp);
  delRegDb();
  btdbg("registry database sync failed.\n");
}

static void syncFileStorage(BSO_CONTEXT* ctx)
{
  BSO_FILE_ENTRY* entry = NULL;

  TAILQ_FOREACH(entry, &ctx->fpHead, entry)
    {
      if (fflush(entry->fp))
        {
          btdbg("file storage sync failed.\n");
        }
    }
}

static int cleanRegistry(void)
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

static int isFileOpened(BSO_CONTEXT* ctx, const char* fileName)
{
  BSO_FILE_ENTRY* entry = NULL;

  TAILQ_FOREACH(entry, &ctx->fpHead, entry)
    {
      if (!strncmp(entry->fileName, fileName, BSO_FILE_NAME_MAX))
        {
          return 1;
        }
    }

  return 0;
}

static int checkFileName(const char* fileName)
{
  if ((!fileName)
      || (BSO_FILE_NAME_MAX == strnlen(fileName, BSO_FILE_NAME_MAX)))
    {
      return -EINVAL;
    }

  return 0;
}

static int isHandleOpened(BSO_CONTEXT* ctx, BSO_FILE* fp)
{
  BSO_FILE_ENTRY* entry = NULL;

  TAILQ_FOREACH(entry, &ctx->fpHead, entry)
    {
      if (fp == entry->fp)
        {
          return 1;
        }
    }

  return 0;
}

static uint32_t getFileMode(BSO_CONTEXT* ctx, BSO_FILE* handle)
{
  BSO_FILE_ENTRY* entry = NULL;

  TAILQ_FOREACH(entry, &ctx->fpHead, entry)
  {
    if (handle == entry->fp)
      {
        return entry->mode;
      }
  }

  return BSO_MODE_MAX;
}

static int getWhence(int whence)
{
  return ((BSO_SEEK_START == whence) ? SEEK_SET :
      (BSO_SEEK_CUR   == whence) ? SEEK_CUR :
      (BSO_SEEK_END   == whence) ? SEEK_END :
      -EINVAL);
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

  ret = (!(ret = initRegistryDatabase(ctx))) ? initFileStorage(ctx) : ret;
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
  BSO_CONTEXT* ctx = &gBsoContext;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return;
    }

  syncRegistryDatabase(ctx);
  syncFileStorage(ctx);
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
  int ret = 0;

  BSO_CONTEXT* ctx          = &gBsoContext;
  BSO_FILE_ENTRY* fileEntry = NULL;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return 0;
    }

  syncRegistryDatabase(ctx);
  TAILQ_FOREACH(fileEntry, &ctx->fpHead, entry)
    {
      fclose(fileEntry->fp);
    }

  ret = cleanRegistry();
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
  uint32_t changeCount   = 0;

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

              ++changeCount;

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

          ++changeCount;
        }
    }

  if (changeCount)
    {
      syncRegistryDatabase(ctx);
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

      syncRegistryDatabase(ctx);
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
  int ret = 0;
  ret = cleanRegistry();
  ret |= delRegDb();
  return ret;
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

/****************************************************************************
 * Name: BSO_FileRead
 *
 * Description:
 *   Bluetooth Storage Operation file read.
 *   Read a file for loading database.
 *
 ****************************************************************************/

int BSO_FileRead(void* buf, uint32_t* size, BSO_FILE* handle)
{
  BSO_CONTEXT* ctx = &gBsoContext;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (!handle)
    {
      return -EBADF;
    }

  if (!isHandleOpened(ctx, handle))
    {
      return -ENFILE;
    }

  *size = fread(buf, *size, 1, handle) * (*size);

  return 0;
}

/****************************************************************************
 * Name: BSO_FileWrite
 *
 * Description:
 *   Bluetooth Storage Operation file write.
 *   Write a file for saving database.
 *
 ****************************************************************************/

int BSO_FileWrite(void* buf, uint32_t size, BSO_FILE* handle)
{
  BSO_CONTEXT* ctx  = &gBsoContext;
  uint32_t fileMode = BSO_MODE_MAX;
  const size_t NITEMS    = 1;
  int ret                = 0;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (!handle)
    {
      return -EBADF;
    }

  if (!isHandleOpened(ctx, handle))
    {
      return -ENFILE;
    }

  fileMode = getFileMode(ctx, handle);

  if ((BSO_MODE_MAX == fileMode)
      || (BSO_MODE_READ == fileMode))
    {
      return -EACCES;
    }

  ret = fwrite(buf, size, NITEMS, handle);

  if (ferror(handle) || (NITEMS != ret))
    {
      return -ENOSPC;
    }

  return 0;
}

/****************************************************************************
 * Name: BSO_FileOpen
 *
 * Description:
 *   Bluetooth Storage Operation file open.
 *   Open a file for read database.
 *
 ****************************************************************************/

BSO_FILE* BSO_FileOpen(const char* fileName, int32_t mode)
{
  BSO_CONTEXT* ctx                = &gBsoContext;
  BSO_FILE_ENTRY* entry           = NULL;
  char pathBuf[BSO_FILE_PATH_MAX] = {0};

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return NULL;
    }

  if (checkFileName(fileName))
    {
      return NULL;
    }

  if (isFileOpened(ctx, fileName))
    {
      return NULL;
    }

  entry = (BSO_FILE_ENTRY*)malloc(sizeof(BSO_FILE_ENTRY));
  if (entry)
    {
      memset(entry, 0, sizeof(BSO_FILE_ENTRY));
      snprintf(pathBuf, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, fileName);
      entry->fp = fopen(pathBuf,
          ((BSO_MODE_READ   == mode) ? "r" :
           (BSO_MODE_WRITE  == mode) ? "w" :
           (BSO_MODE_APPEND == mode) ? "a" :
           "inval"));
      if (!entry->fp)
        {
          free(entry);
          entry = NULL;
          return NULL;
        }

      entry->mode = mode;
      strncpy(entry->fileName, fileName, BSO_FILE_NAME_MAX);

      TAILQ_INSERT_TAIL(&ctx->fpHead, entry, entry);

      return entry->fp;
    }

  return NULL;
}

/****************************************************************************
 * Name: BSO_FileClose
 *
 * Description:
 *   Bluetooth Storage Operation file close.
 *   Close a database file.
 *
 ****************************************************************************/

int BSO_FileClose(BSO_FILE* handle)
{
  BSO_CONTEXT* ctx      = &gBsoContext;
  BSO_FILE_ENTRY* entry = NULL;
  int ret               = 0;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (!handle)
    {
      return -EBADF;
    }

  if (!isHandleOpened(ctx, handle))
    {
      return -ENFILE;
    }

  TAILQ_FOREACH(entry, &ctx->fpHead, entry)
    {
      if (handle == entry->fp)
        {
          break;
        }
    }

  if (entry)
    {
      TAILQ_REMOVE(&ctx->fpHead, entry, entry);
      if (ferror(handle))
      {
        // FIXME: return all errors as -ENOSPC
        ret = -ENOSPC;
      }
      fclose(handle);
      free(entry);
      entry = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: BSO_FileSeek
 *
 * Description:
 *   Bluetooth Storage Operation file seek.
 *   Seek a database file.
 *
 ****************************************************************************/

int BSO_FileSeek(BSO_FILE* handle, int32_t offset, int whence)
{
  BSO_CONTEXT* ctx = &gBsoContext;
  int w            = -EINVAL;
  int64_t currPos  = 0;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (!handle)
    {
      return -EBADF;
    }

  if (!isHandleOpened(ctx, handle))
    {
      return -ENFILE;
    }

  w = getWhence(whence);

  if (-EINVAL == w)
    {
      return w;
    }

  if (fseek(handle, offset, w))
    {
      return -EFAULT;
    }

  if ((currPos = ftell(handle)) < 0)
    {
      return -EFAULT;
    }

  return (int)currPos;
}

/****************************************************************************
 * Name: BSO_FileFlush
 *
 * Description:
 *   Bluetooth Storage Operation file flush.
 *   Flush a database file.
 *
 ****************************************************************************/

int BSO_FileFlush(BSO_FILE* handle)
{
  BSO_CONTEXT* ctx = &gBsoContext;

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (!handle)
    {
      return -EBADF;
    }

  if (!isHandleOpened(ctx, handle))
    {
      return -ENFILE;
    }

  fflush(handle);

  if (ferror(handle))
    {
      // FIXME: return all errors as -ENOSPC
      return -ENOSPC;
    }

  return 0;
}

/****************************************************************************
 * Name: BSO_FileDelete
 *
 * Description:
 *   Bluetooth Storage Operation file delete.
 *   Delete a database file.
 *
 ****************************************************************************/

int BSO_FileDelete(const char* fileName)
{
  BSO_CONTEXT* ctx                = &gBsoContext;
  char pathBuf[BSO_FILE_PATH_MAX] = {0};

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (isFileOpened(ctx, fileName))
    {
      return -ETXTBSY;
    }

  snprintf(pathBuf, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, fileName);

  if (unlink(pathBuf))
    {
      // FIXME: return most of errors as -ENOSPC
      return ((ENOENT == errno) ? -errno : -ENOSPC);
    }

  return 0;
}

/****************************************************************************
 * Name: BSO_FileRename
 *
 * Description:
 *   Bluetooth Storage Operation file rename.
 *   Rename a database file.
 *
 ****************************************************************************/

int BSO_FileRename(const char* oldName, const char* newName)
{
  BSO_CONTEXT* ctx                = &gBsoContext;
  char pathOld[BSO_FILE_PATH_MAX] = {0};
  char pathNew[BSO_FILE_PATH_MAX] = {0};

  if (!(ctx->flags & FLAG_INITIALIZED))
    {
      return -ENXIO;
    }

  if (checkFileName(newName))
    {
      return -EINVAL;
    }

  if (isFileOpened(ctx, oldName))
    {
      return -ETXTBSY;
    }

  snprintf(pathNew, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, newName);
  snprintf(pathOld, BSO_FILE_PATH_MAX, BSO_FILE_PATH_NAME, oldName);

  if (rename(pathOld, pathNew))
    {
      // FIXME: return most of errors as -ENOSPC
      return (((EEXIST == errno) || (ENOENT == errno)) ? -errno : -ENOSPC);
    }

  return 0;
}
