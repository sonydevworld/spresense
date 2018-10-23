/****************************************************************************
 * modules/lte/altcom/evtdisp/director.c
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

#include "dbg_if.h"
#include "director.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: director_construct
 *
 * Description:
 *   Construction of object using builder interface.
 *
 * Input Parameters:
 *   bldif  builder interface.
 *   arg  initialize argment.
 *
 * Returned Value:
 *   result.
 *
 ****************************************************************************/

int32_t director_construct(FAR struct builder_if_s *builder, FAR void *arg)
{
  int32_t ret;

  if (!builder || !builder->buildmain)
    {
      DBGIF_LOG_ERROR("NULL parameter.");
      return -EINVAL;
    }

  ret = builder->buildmain(arg);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("buildmain() [errno=%d]\n", ret);
      return ret;
    }

  if (builder->buildsub1)
    {
      ret = builder->buildsub1(arg);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("buildoption() [errno=%d]\n", ret);
          return ret;
        }
    }

  if (builder->buildsub2)
    {
      ret = builder->buildsub2(arg);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("buildoption() [errno=%d]\n", ret);
          return ret;
        }
    }

  if (builder->buildsub3)
    {
      ret = builder->buildsub3(arg);
      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("buildoption() [errno=%d]\n", ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: director_destruct
 *
 * Description:
 *   Destruction of objects using builder interface.
 *
 * Input Parameters:
 *   bldif  builder interface.
 *
 * Returned Value:
 *   result.
 *
 ****************************************************************************/

int32_t director_destruct(FAR struct builder_if_s *builder)
{
  int32_t ret;

  if (!builder || !builder->destroy)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  ret = builder->destroy();
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("destroy() [errno=%d]\n", ret);
      return ret;
    }

  return ret;
}
