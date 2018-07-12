/****************************************************************************
 * modules/include/fwuputils/fwup_manager.h
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

#ifndef __APPS_INCLUDE_FWUPUTILS_FWUP_MANAGER_H
#define __APPS_INCLUDE_FWUPUTILS_FWUP_MANAGER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FWUP_MSGQ_NAME "fwup_msgq"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum fw_type_e
{
  FW_APP,
  FW_SYS,
  FW_UPDATER,
  FW_SBL,
};

enum fwup_cmd_e
{
  FWUP_INIT,
  FWUP_MSGSYNC,
  FWUP_DOWNLOAD,
  FWUP_UPDATE,
  FWUP_SUSPEND,
  FWUP_RESUME,
  FWUP_ABORT,
};

struct fwup_msg_dl_param_s
{
  enum fw_type_e    fwtype;
  uint32_t          fwsize;
  void              *data;
  uint32_t          size;
};

struct fwup_msg_sync_param_s
{
  sem_t             *sem;
};

struct fwup_msg_s
{
  enum fwup_cmd_e   cmd;
  union {
    struct fwup_msg_dl_param_s dlparam;
    struct fwup_msg_sync_param_s syncparam;
  } u;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  fwup_initialize
 *
 * Description:
 *   Initialize for FW Update Manager
 *
 ****************************************************************************/

int fwup_initialize(void);

/****************************************************************************
 * Name:  fwup_finalize
 *
 * Description:
 *   Finalize for FW Update Manager
 *
 ****************************************************************************/

int fwup_finalize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_FWUPUTILS_FWUP_MANAGER_H */
