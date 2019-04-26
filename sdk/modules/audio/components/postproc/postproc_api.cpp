/****************************************************************************
 * modules/audio/components/postproc/postproc_api.cpp
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

#include "postproc_api.h"

extern "C"
{
/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
uint32_t AS_postproc_init(const InitCustomProcParam *param,
                          void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Execute */

  return ((CustomProcBase *)p_instance)->init(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postproc_exec(const ExecCustomProcParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  return ((CustomProcBase *)p_instance)->exec(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postproc_flush(const FlushCustomProcParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  return ((CustomProcBase *)p_instance)->flush(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postproc_setparam(const SetCustomProcParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  return ((CustomProcBase *)p_instance)->set(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postproc_recv_done(void *p_instance, CustomProcCmpltParam *cmplt)
{
  /* Parameter check */

  if (p_instance == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute */

  if (cmplt == NULL)
    {
      return ((CustomProcBase *)p_instance)->recv_done();
    }
  else
    {
      return ((CustomProcBase *)p_instance)->recv_done(cmplt);
    }
}

/*--------------------------------------------------------------------*/
uint32_t AS_postproc_activate(void **p_instance,
                              MemMgrLite::PoolId apu_pool_id,
                              MsgQueId apu_mid,
                              CustomProcCallback callback,
                              const char *dsp_name,
                              void *p_requester,
                              uint32_t *dsp_inf,
                              ProcType type)
{
  if (p_instance == NULL || dsp_inf == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Reply pointer of self instance, which is used for API call. */

  switch (type)
    {
      case ProcTypeUserDefFilter:
        *p_instance = (void*)(new UserCustomComponent(apu_pool_id,apu_mid));
        break;

      default:
        *p_instance = (void*)(new ThruProcComponent());
        break;
    }

  if (*p_instance == NULL)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  return ((CustomProcBase *)*p_instance)->activate(callback, dsp_name, p_requester, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_postproc_deactivate(void *p_instance)
{

  if ((CustomProcBase *)p_instance != NULL)
    {
      ((CustomProcBase *)p_instance)->deactivate();
      delete (CustomProcBase*)p_instance;
      return true;
    }

  POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);

  return false;
}
} /* extern "C" */

