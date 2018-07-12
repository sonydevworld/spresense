/****************************************************************************
 * modules/audio/components/postfilter/postfilter_api.cpp
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

#include "postfilter_api.h"

extern "C"
{
/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
uint32_t AS_postfilter_init(const InitPostfilterParam *param,
                            void *p_instance,
                            uint32_t *dsp_inf)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL || dsp_inf == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Execute */

  return ((PostfilterBase *)p_instance)->init_apu(*param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_exec(const ExecPostfilterParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  return ((PostfilterBase *)p_instance)->exec_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_flush(const FlushPostfilterParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  return ((PostfilterBase *)p_instance)->flush_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_recv_done(void *p_instance, PostfilterCmpltParam *output)
{
  /* Parameter check */

  if (p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  if (output == NULL)
    {
      return ((PostfilterBase *)p_instance)->recv_done();
    }
  else
    {
      return ((PostfilterBase *)p_instance)->recv_done(output);
    }
}

/*--------------------------------------------------------------------*/
uint32_t AS_postfilter_activate(void **p_instance,
                                MemMgrLite::PoolId apu_pool_id,
                                MsgQueId apu_mid,
                                uint32_t *dsp_inf,
                                bool through)
{
  if (p_instance == NULL || dsp_inf == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Reply pointer of self instance, which is used for API call. */

  *p_instance = (through) ?
                    (void*)(new PostfilterThrough())
                  : (void*)(new PostfilterComponent(apu_pool_id,apu_mid));

  if (*p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  return ((PostfilterBase *)*p_instance)->activate(dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_deactivate(void *p_instance)
{

  if ((PostfilterBase *)p_instance != NULL)
    {
      ((PostfilterBase *)p_instance)->deactivate();
      delete (PostfilterBase*)p_instance;
      return true;
    }

  POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);

  return false;
}
} /* extern "C" */

