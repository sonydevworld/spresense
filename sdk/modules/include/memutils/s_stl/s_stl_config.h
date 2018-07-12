/****************************************************************************
 * modules/include/memutils/s_stl/s_stl_config.h
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

#ifndef STL_CONFIG_INCLUDED
#define STL_CONFIG_INCLUDED

#include "memutils/common_utils/common_types.h"

/*-------------------------------------------------------------------
	Simple STL Definitions.
  -------------------------------------------------------------------*/
#define __STL_BEGIN_NAMESPACE	namespace s_std {
#define __STL_END_NAMESPACE	}
#define __USING_S_STL		using namespace s_std

#define __M_STL_BEGIN_NAMESPACE	namespace ms_std {
#define __M_STL_END_NAMESPACE	}
#define __USING_MS_STL		using namespace ms_std

#define FULL		1
#define EMPTY		0

/*#define NULL		(0)*/
#define NULL_T		(reinterpret_cast<T>(0))
#define NULL_NODE	(reinterpret_cast<Node<T>*>(0))


/*-------------------------------------------------------------------
	Select Data Structure (Algorithm).
  -------------------------------------------------------------------*/
/* Data Structure List */
#define LIST  1
#define HEAP  2
#define SHEAP 3
#define PHEAP 4

/* Priority Queue Data Structure */
#define PQUEUE_STR LIST


#include "memutils/common_utils/common_assert.h"

/*-------------------------------------------------------------------
	User Definitions. (for Exception)
  -------------------------------------------------------------------*/
#ifndef _fatal
#include <stdio.h>

//#define _fatal	while(1)
#define _fatal	D_ASSERT(!"Fatal!")
#endif

#endif // STL_CONFIG_INCLUDED

