/****************************************************************************
 * modules/include/sensing/tap_manager.h
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
/**
 * @file tap_manager.h
 */

#ifndef __INCLUDE_SENSING_TAP_MANAGER_H
#define __INCLUDE_SENSING_TAP_MANAGER_H

/**
 * @defgroup tap_mng Tap Manager
 * @{
 */

/* --------------------------------------------------------------------------
  Included Files
   -------------------------------------------------------------------------- */
#include <sys/time.h>
#include "sensing/tap.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef void (FAR *tap_mng_out_cbs)(int tap_cnt);

/**
 * @defgroup tap_mng_datatypes Data types
 * @{
 */

/* --------------------------------------------------------------------------
  Command Structures
   -------------------------------------------------------------------------- */
/**
 * @struct tap_mng_start_param
 * @brief tap manager initial setting value
 */
struct tap_mng_start_param
{
  ST_TAP_OPEN     tap_prm;  /**< When No tap detected.                    */
  tap_mng_out_cbs cbs;      /**< Callback function to get number of taps. */
};
/** @} tap_mng_datatypes */

/**
 * @defgroup tap_mng_funcs Functions
 * @{
 */

/*--------------------------------------------------------------------
    External Interface
  --------------------------------------------------------------------*/

/**
 * @brief     Initial processing. Create task.
 * @return    result of process.
 */
int TapMngInit(void);

/**
 * @brief     Start Accel sensor and detect tap.
 * @param[in]  sta_prm : Coefficient required for tap detection
 * @param[out] ctl_id : Id identifying generated TapClass
 * @return    result of process.
 */
int TapMngStart(struct tap_mng_start_param *sta_prm, int *ctl_id);

/**
 * @brief     Stop the Accel sensor and stop tap detection.
 * @param[in] ctl_id : ctl_id of the tap to stop
 * @return    result of process.
 */
int TapMngStop(int ctl_id);

/**
 * @brief     Finalize processing. 
 * @return    result of process.
 */
int TapMngFin(void);

/** @} tap_mng_funcs */
/** @} tap_mng */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SENSING_TAP_MANAGER_H */
