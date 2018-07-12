/****************************************************************************
 * modules/audio/components/filter/filter_component.cpp
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

#include "filter_component.h"
#include "common/audio_internal_message_types.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

#ifdef CONFIG_AUDIOUTILS_SRC
static SRCComponent *sp_src_instance = NULL;
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
static MFEComponent *sp_mfe_instance = NULL;
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
static MPPComponent *sp_mpp_instance = NULL;
#endif

extern "C" {

/*--------------------------------------------------------------------
  C Interface
  --------------------------------------------------------------------*/

uint32_t AS_filter_activate(FilterComponentType type,
                            const char *path,
                            MsgQueId apu_dtq,
                            PoolId apu_pool_id,
                            uint32_t *dsp_inf)
{
  switch (type)
    {
#ifdef CONFIG_AUDIOUTILS_SRC
      case SRCOnly:
        if (sp_src_instance == NULL)
          {
            sp_src_instance = new SRCComponent(apu_dtq, apu_pool_id);
            if (sp_src_instance == NULL)
              {
                FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
                return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
              }
          }

        return sp_src_instance->activate_apu(sp_src_instance, path, dsp_inf);
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
      case MfeOnly:
        if (sp_mfe_instance == NULL)
          {
            sp_mfe_instance = new MFEComponent(apu_dtq);
            if (sp_mfe_instance == NULL)
              {
                FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
                return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
              }
          }

        if (sp_mpp_instance == NULL)
          {
            sp_mpp_instance = new MPPComponent(apu_dtq);
            if (sp_mpp_instance == NULL)
              {
                FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
                return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
              }
          }

        return sp_mfe_instance->activate_apu(sp_mfe_instance,
                                             sp_mpp_instance,
                                             dsp_inf);
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case MppEax:
        if (sp_mfe_instance == NULL)
          {
            sp_mfe_instance = new MFEComponent(apu_dtq);
            if (sp_mfe_instance == NULL)
              {
                FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
                return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
              }
          }

        if (sp_mpp_instance == NULL)
          {
            sp_mpp_instance = new MPPComponent(apu_dtq);
            if (sp_mpp_instance == NULL)
              {
                FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
                return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
              }
          }
        {
          bool result = sp_mfe_instance->activate_apu(sp_mfe_instance,
                                                      dsp_inf);

          result &= sp_mpp_instance->activate_apu(sp_mpp_instance,
                                                  path,
                                                  dsp_inf);
          return result;
        }
#endif
      default:
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }
}

/*--------------------------------------------------------------------*/
bool AS_filter_deactivate(FilterComponentType type)
{
  switch (type)
    {
#ifdef CONFIG_AUDIOUTILS_SRC
      case SRCOnly:
        if(sp_src_instance->deactivate_apu())
          {
            delete sp_src_instance;
            sp_src_instance = NULL;
            return true;
          }
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
      case MfeOnly:
        if(sp_mfe_instance->deactivate_apu())
          {
            delete sp_mfe_instance;
            sp_mfe_instance = NULL;
            delete sp_mpp_instance;
            sp_mpp_instance = NULL;
            return true;
          }
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case MppEax:
        {
          bool result = sp_mfe_instance->deactivate_apu();
          result &= sp_mpp_instance->deactivate_apu();
          if(result)
            {
              delete sp_mfe_instance;
              sp_mfe_instance = NULL;
              delete sp_mpp_instance;
              sp_mpp_instance = NULL;
              return true;
            }
        }
#endif
      default:
        break;
    }

  return false;
}

/*--------------------------------------------------------------------*/
uint32_t AS_filter_init(FilterComponentParam param, uint32_t *dsp_inf)
{
  switch (param.filter_type)
    {
#ifdef CONFIG_AUDIOUTILS_SRC
      case Apu::SRC:
        sp_src_instance->setCallBack(param.callback);
        return sp_src_instance->init_apu(param.init_src_param, dsp_inf);
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
      case Apu::MFE:
        sp_mfe_instance->setCallBack(param.callback);
        return sp_mfe_instance->init_apu(param.init_mfe_param, dsp_inf);
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case Apu::XLOUD:
        sp_mpp_instance->setCallBack(param.callback);
        return sp_mpp_instance->init_apu(param.init_xloud_param, dsp_inf);
#endif
      default:
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }
}

/*--------------------------------------------------------------------*/
bool AS_filter_exec(FilterComponentParam param)
{
  switch (param.filter_type)
    {
#ifdef CONFIG_AUDIOUTILS_SRC
      case Apu::SRC:
        sp_src_instance->setCallBack(param.callback);
        return sp_src_instance->exec_apu(param.exec_src_param);
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
      case Apu::MFE:
        sp_mfe_instance->setCallBack(param.callback);
        return sp_mfe_instance->exec_apu(param.exec_mfe_param);
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case Apu::XLOUD:
        sp_mpp_instance->setCallBack(param.callback);
        return sp_mpp_instance->exec_apu(param.exec_xloud_param);
#endif
      default:
        break;
    }

  return false;
}

/*--------------------------------------------------------------------*/
bool AS_filter_stop(FilterComponentParam param)
{
  switch (param.filter_type)
    {
#ifdef CONFIG_AUDIOUTILS_SRC
      case Apu::SRC:
        return sp_src_instance->flush_apu(param.stop_src_param);
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
      case Apu::MFE:
        sp_mfe_instance->setCallBack(param.callback);
        return sp_mfe_instance->flush_apu();
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case Apu::XLOUD:
        sp_mpp_instance->setCallBack(param.callback);
        return sp_mpp_instance->flush_apu();
#endif
      default:
        break;
    }

  return false;
}

#ifndef CONFIG_AUDIOUTILS_RECORDER
/*--------------------------------------------------------------------*/
bool AS_filter_setparam(FilterComponentParam param)
{
  switch (param.filter_type)
    {
#ifdef CONFIG_AUDIOUTILS_MFE
      case Apu::MFE:
        sp_mfe_instance->setCallBack(param.callback);
        return sp_mfe_instance->setparam_apu(param.set_mfe_param);
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case Apu::XLOUD:
        sp_mpp_instance->setCallBack(param.callback);
        return sp_mpp_instance->setparam_apu(param.set_mpp_param);
#endif
      default:
        break;
    }

  return false;
}

/*--------------------------------------------------------------------*/
bool AS_filter_tuning(FilterComponentParam param)
{
  switch (param.filter_type)
    {
#ifdef CONFIG_AUDIOUTILS_MFE
      case Apu::MFE:
        sp_mfe_instance->setCallBack(param.callback);
        return sp_mfe_instance->tuning_apu(param.tuning_mfe_param);
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
      case Apu::XLOUD:
        sp_mpp_instance->setCallBack(param.callback);
        return sp_mpp_instance->tuning_apu(param.tuning_mpp_param);
#endif
      default:
        break;
    }

  return false;
}
#endif

/*--------------------------------------------------------------------*/
bool AS_filter_recv_done(void)
{
#ifdef CONFIG_AUDIOUTILS_SRC
  return sp_src_instance->recv_done();
#endif
  return false;
}

} /* extern "C" */

__WIEN2_END_NAMESPACE

