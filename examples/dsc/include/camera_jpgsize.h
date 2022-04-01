/****************************************************************************
 * examples/dsc/include/camera_jpgsize.h
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_DSC_INCLUDE_CAMERA_JPGSIZE_H__
#define __EXAMPLES_DSC_INCLUDE_CAMERA_JPGSIZE_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "camera_parambase.h"

/****************************************************************************
 * Public Class
 ****************************************************************************/

struct camera_pair_param_name
{
  int param1;
  int param2;
  const char *name;
};

class camera_jpgsize : public camera_parambase<struct camera_pair_param_name>
{
  public:
    camera_jpgsize(camera_pair_param_name *cpn, int num, int initval);
    virtual ~camera_jpgsize();

    virtual int exec_item(int fd, int idx);
    virtual const char *get_valuename();

    int jpg_width();
    int jpg_height();
};

#endif  /* __EXAMPLES_DSC_INCLUDE_CAMERA_JPGSIZE_H__ */
