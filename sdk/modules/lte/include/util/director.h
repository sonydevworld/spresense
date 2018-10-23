/****************************************************************************
 * modules/lte/include/util/director.h
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
 
#ifndef __MODULES_LTE_INCLUDE_UTIL_DIRECTOR_H
#define __MODULES_LTE_INCLUDE_UTIL_DIRECTOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include "osal.h"
#include "builder_if.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 Public Function Prototypes
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

int32_t director_construct(FAR struct builder_if_s *builder, FAR void *arg);

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

int32_t director_destruct(FAR struct builder_if_s *builder);

#endif /* __MODULES_LTE_INCLUDE_UTIL_DIRECTOR_H */

