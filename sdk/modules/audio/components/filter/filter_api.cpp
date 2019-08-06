/****************************************************************************
 * modules/audio/components/filter/filter_api.cpp
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

#include "filter_api.h"
#include "packing_component.h"
#include "audio/audio_message_types.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

#ifdef CONFIG_AUDIOUTILS_MFE
static MFEComponent *sp_mfe_instance = NULL;
#endif

extern "C" {

/*--------------------------------------------------------------------
  C Interface
  --------------------------------------------------------------------*/

uint32_t AS_filter_activate(FilterComponentType type,
                            const char *path,
                            MsgQueId apu_dtq,
                            PoolId apu_pool_id,
                            uint32_t *dsp_inf,
                            FilterCompCallback callback,
                            FilterComponent **pp_ins)
{
  switch (type)
    {
#ifdef CONFIG_AUDIOUTILS_MFE
      case MicFrontEnd:
        {
          *pp_ins = new MFEComponent(apu_dtq);
          if (*pp_ins == NULL)
            {
              FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
              return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
            }

          sp_mfe_instance = static_cast<MFEComponent *>(*pp_ins);

          (*pp_ins)->setCallBack(callback);
          return (*pp_ins)->activate_apu(path, dsp_inf);
        }
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case MediaPlayerPostAsSub:
        {
          *pp_ins = new MPPComponent(apu_dtq);
          if (*pp_ins == NULL)
            {
              FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
              return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
            }

          (*pp_ins)->setCallBack(callback);

          /* In this case, don't actvite MPP component because loading DSP is
           * done by MFE Component. So only notify own instance to MFE instead. */

#ifdef CONFIG_AUDIOUTILS_MFE
          mfe_register_sub_instance(static_cast<MPPComponent *>(*pp_ins), sp_mfe_instance);
#endif
          return AS_ECODE_OK;
        }

      case MediaPlayerPost:
        *pp_ins = new MPPComponent(apu_dtq);
        if (*pp_ins == NULL)
          {
            FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
            return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
          }

        (*pp_ins)->setCallBack(callback);
        return (*pp_ins)->activate_apu(path, dsp_inf);
#endif
      default:
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }
}

/*--------------------------------------------------------------------*/
bool AS_filter_deactivate(FilterComponent *p_ins, FilterComponentType type)
{
  if (type == MediaPlayerPostAsSub)
    {
      delete p_ins;
      return true;
    }

  if (p_ins->deactivate_apu())
    {
#ifdef CONFIG_AUDIOUTILS_MFE
      if (type == MicFrontEnd)
        {
          sp_mfe_instance = NULL;
        }
#endif

      delete p_ins;
      return true;
    }

  return false;
}

/*--------------------------------------------------------------------*/
uint32_t AS_filter_init(InitFilterParam *param, uint32_t *dsp_inf, FilterComponent *p_ins)
{
  return p_ins->init_apu(param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_filter_exec(ExecFilterParam *param, FilterComponent *p_ins)
{
  return p_ins->exec_apu(param);
}

/*--------------------------------------------------------------------*/
bool AS_filter_stop(StopFilterParam *param, FilterComponent *p_ins)
{
  return p_ins->flush_apu(param);
}

#ifndef CONFIG_AUDIOUTILS_RECORDER
/*--------------------------------------------------------------------*/
bool AS_filter_setparam(SetFilterParam *param, FilterComponent *p_ins)
{
  return p_ins->setparam_apu(param);
}

/*--------------------------------------------------------------------*/
bool AS_filter_tuning(TuningFilterParam *param, FilterComponent *p_ins)
{
  return p_ins->tuning_apu(param);
}
#endif

/*--------------------------------------------------------------------*/
bool AS_filter_recv_done(FilterComponent *p_ins)
{
  return p_ins->recv_done();
}

} /* extern "C" */

__WIEN2_END_NAMESPACE

