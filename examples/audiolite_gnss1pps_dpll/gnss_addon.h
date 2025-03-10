/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/gnss_addon.h
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_GNSS_ADDON_H
#define __EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_GNSS_ADDON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_STATE_NOTLOCK  (0)
#define GNSS_STATE_LOCKED   (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct gnss_ctrl_s
{
  int devfd;
  int state;
  int is_running;
  pthread_t thd;
  pthread_mutex_t lock;
  pthread_cond_t  cond;
};
typedef struct gnss_ctrl_s gnss_ctrl_t;

#  ifdef __cplusplus
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int gnss_setup(gnss_ctrl_t *gnss);
int gnss_wait_statechange(gnss_ctrl_t *gnss, int cur_state);
void gnss_cleanup(gnss_ctrl_t *gnss);

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

#endif /* __EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_GNSS_ADDON_H */
