/****************************************************************************
 * modules/lte/altcom/api/lte/lte_callback.c
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
#include "lte/lte_api.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

power_control_cb_t    g_lte_power_callback        = NULL;
attach_net_cb_t       g_attach_net_callback       = NULL;
detach_net_cb_t       g_detach_net_callback       = NULL;
get_netstat_cb_t      g_getnetstat_callback       = NULL;
data_on_cb_t          g_dataon_callback           = NULL;
data_off_cb_t         g_dataoff_callback          = NULL;
get_datastat_cb_t     g_getdatastat_callback      = NULL;
get_dataconfig_cb_t   g_getdataconfig_callback    = NULL;
set_dataconfig_cb_t   g_setdataconfig_callback    = NULL;
get_apnset_cb_t       g_getapnset_callback        = NULL;
set_apn_cb_t          g_setapn_callback           = NULL;
get_ver_cb_t          g_getver_callback           = NULL;
get_phoneno_cb_t      g_getphoneno_callback       = NULL;
get_imsi_cb_t         g_getimsi_callback          = NULL;
get_imei_cb_t         g_getimei_callback          = NULL;
get_pinset_cb_t       g_getpinset_callback        = NULL;
set_pinenable_cb_t    g_pinenable_callback        = NULL;
change_pin_cb_t       g_changepin_callback        = NULL;
enter_pin_cb_t        g_enterpin_callback         = NULL;
get_localtime_cb_t    g_getltime_callback         = NULL;
get_operator_cb_t     g_getoperator_callback      = NULL;
get_edrx_cb_t         g_getedrx_callback          = NULL;
set_edrx_cb_t         g_setedrx_callback          = NULL;
get_psm_cb_t          g_getpsm_callback           = NULL;
set_psm_cb_t          g_setpsm_callback           = NULL;
get_ce_cb_t           g_getce_callback            = NULL;
set_ce_cb_t           g_setce_callback            = NULL;
netstat_report_cb_t   g_netstat_report_callback   = NULL;
simstat_report_cb_t   g_simstat_report_callback   = NULL;
localtime_report_cb_t g_localtime_report_callback = NULL;
quality_report_cb_t   g_quality_callback          = NULL;
cellinfo_report_cb_t  g_cellinfo_callback         = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_callback_init
 *
 * Description:
 *   Delete the registered callback and initialize it.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void lte_callback_init(void)
{
  g_lte_power_callback        = NULL;
  g_attach_net_callback       = NULL;
  g_detach_net_callback       = NULL;
  g_getnetstat_callback       = NULL;
  g_dataon_callback           = NULL;
  g_dataoff_callback          = NULL;
  g_getdatastat_callback      = NULL;
  g_getdataconfig_callback    = NULL;
  g_setdataconfig_callback    = NULL;
  g_getapnset_callback        = NULL;
  g_setapn_callback           = NULL;
  g_getver_callback           = NULL;
  g_getphoneno_callback       = NULL;
  g_getimsi_callback          = NULL;
  g_getimei_callback          = NULL;
  g_getpinset_callback        = NULL;
  g_pinenable_callback        = NULL;
  g_changepin_callback        = NULL;
  g_enterpin_callback         = NULL;
  g_getltime_callback         = NULL;
  g_getoperator_callback      = NULL;
  g_getedrx_callback          = NULL;
  g_setedrx_callback          = NULL;
  g_getpsm_callback           = NULL;
  g_setpsm_callback           = NULL;
  g_getce_callback            = NULL;
  g_setce_callback            = NULL;
  g_netstat_report_callback   = NULL;
  g_simstat_report_callback   = NULL;
  g_localtime_report_callback = NULL;
  g_quality_callback          = NULL;
  g_cellinfo_callback         = NULL;
}
