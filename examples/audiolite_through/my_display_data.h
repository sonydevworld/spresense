/****************************************************************************
 * examples/audiolite_through/my_display_data.h
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_AUDIOLITE_THROUGH_MY_DISPLAY_DATA_H
#define __EXAMPLES_AUDIOLITE_THROUGH_MY_DISPLAY_DATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>

#ifdef CONFIG_EXAMPLES_AUDIOLITE_THROUGH_WATCHDATA
#include <stdio.h>

#include <audiolite/audiolite.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: my_display_data
 ****************************************************************************/

class my_display_data : public audiolite_component
{
  public:
    void on_data()
    {
      /* Get comming data */

      audiolite_memapbuf *mem = (audiolite_memapbuf *)pop_data(0);

      if (mem)
        {
          /* If the data is available, let's see the data */

          int16_t *data = (int16_t *)mem->get_data();
          int samples = mem->get_storedsize() /
                        mem->get_channels() / sizeof(int16_t);

          /* Display top 2 (L and R)  samples */

          printf("%d : %6d %6d\n", samples, data[0], data[1]);

          /* Pass the data to later block. */

          push_data(mem);

          /* Release the memory it after finishing use */

          mem->release();
        }
    }
};

#endif /* CONFIG_EXAMPLES_AUDIOLITE_THROUGH_WATCHDATA */

#endif /* __EXAMPLES_AUDIOLITE_THROUGH_MY_DISPLAY_DATA_H */
