/****************************************************************************
 * examples/dsc/include/camera_parambase.h
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

#ifndef __EXAMPLES_DSC_INCLUDE_CAMERA_PARAMBASE_H__
#define __EXAMPLES_DSC_INCLUDE_CAMERA_PARAMBASE_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

#include "camera_menu.h"
#include "camera_ctrl.h"

/****************************************************************************
 * Public Class
 ****************************************************************************/

template<class T>
class camera_parambase : public cam_menuitem
{
  protected:
    T *param;
    int param_num;

  public:
    camera_parambase(const char *name,
                  T *cpn, int num, int initval);
    virtual ~camera_parambase();

    virtual int exec();
    virtual int exec_item(int fd, int idx) = 0;
    virtual const char *get_valuename() = 0;
};

/****************************************************************************
 * Template Class Methods
 ****************************************************************************/

template<class T>
camera_parambase<T>::camera_parambase(const char *name,
                             T *cpn, int num, int initval)
  : cam_menuitem(name, initval), param(cpn), param_num(num)
{
}

template<class T>
camera_parambase<T>::~camera_parambase()
{
}

template<class T>
int camera_parambase<T>::exec()
{
  int i;
  int ret;
  int fd = cam_menuitem::get_camfd();

  printf("Called exec()\n");

  /* Next supported item will be set */

  for (i = item_value + 1, ret = -1; i < param_num && ret < 0; i++)
    {
      ret = exec_item(fd, i);
      if (ret >= 0)
        {
          printf("[%d] Successed!!!\n", i);
          item_value = i;
        }
    }

  if (ret < 0)
    {
      /* Lap-round */

      for (i = 0; i < item_value && ret < 0; i++)
        {
          ret = exec_item(fd, i);
          if (ret >= 0)
            {
              printf("[%d] Successed!!!\n", i);
              item_value = i;
            }
        }
    }

  return MENU_EXEC_DONE;
}

#endif  /* __EXAMPLES_DSC_INCLUDE_CAMERA_PARAMBASE_H__ */
