/****************************************************************************
 * modules/lte/altcom/api/lte/lte_initialize.c
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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

#include <stdint.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "apiutil.h"
#include "ltebuilder.h"
#include "director.h"
#include "dbg_if.h"
#include "altcombs.h"
#include "altcom_status.h"

#include "lte/lte_daemon.h"
#include "lte/altcom/altcom_api.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

bool        g_lte_initialized = false;
sys_mutex_t g_lte_apicallback_mtx;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_initialize
 *
 * Description:
 *   Initialize the LTE library resouces.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_initialize(void)
{
  int32_t ret;

  ret = lte_daemon_init(NULL);

  return ret;
}

/****************************************************************************
* Name: altcom_initialize
*
* Description:
*   Initialize the LTE library resouces.
*
* Input Parameters:
*   None
*
* Returned Value:
*   On success, 0 is returned.
*   On failure, negative value is returned.
*
****************************************************************************/

int32_t altcom_initialize(void)
{
  int32_t ret;
  int32_t status;

  status = altcom_get_status();
  if (status != ALTCOM_STATUS_UNINITIALIZED)
    {
      return -EALREADY;
    }

  ret = director_construct(&g_ltebuilder, NULL);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("director_construct() error. %d", ret);
    }
  else
    {
      altcom_set_status(ALTCOM_STATUS_INITIALIZED);
    }

  return ret;
}
