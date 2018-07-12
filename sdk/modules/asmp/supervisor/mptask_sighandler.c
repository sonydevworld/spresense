/****************************************************************************
 * modules/asmp/supervisor/mptask_sighandler.c
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

#include <sdk/config.h>

#include <asmp/types.h>
#include <asmp/mpsignal.h>

#include "mptask.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mptask_sighandler(int8_t signo, uint16_t sigdata, uint32_t data,
                      FAR void *userdata)
{
  FAR mptask_t *task = (FAR mptask_t *)userdata;

  switch (signo)
    {
      case MPSIGSTART:
        /* Start signal
         * Direction: W -> S, Data: None
         */
        {
          if (task_is_init(task))
            {
              task_set_exec(task);
              mptask_semgive(&task->wait);
              mpinfo("SIGSTART\n");
            }
          else
            {
              mperr("Caught invalid signal %d.\n", signo);
            }
          break;
        }

      case MPSIGEXIT:
        /* Exit signal
         * Direction: W -> S, Data: exit status
         */
        {
          if (task_is_exec(task))
            {
              task_set_exit(task);
              task_set_exit_status(task, data);
              mptask_semgive(&task->wait);
              mpinfo("SIGEXIT %d\n", (int)data);
            }
          break;
        }

      case MPSIGHARDFAULT:
        /* Raise Hardfault exception
         * Direction: W -> S, Data: Register dump memory address
         */
        break;

      case MPSIGBUSFAULT:
        /* Raise Busfault exception
         * Direction: W -> S, Data: Register dump memory address
         */
        break;

      case MPSIGPING:
        /* Ping to other CPU
         * Direction: both, Data: Ping Number
         */
        break;

      case MPSIGREQ:
        break;

      case MPSIGRESP:
      case MPSIGABORT:
        /* Aboart Request
         * Direction: S -> W, Data: None
         */

      case MPSIGSYS:
        /* System Message
         * Direction: S -> W, Data: any
         */

      case MPSIGDEBUG:
        /* Debug signal
         * Direction S -> W, Data: any
         */

      default:
        {
          mperr("Caught invalid signal %d.\n", signo);
          break;
        }
    }

  return OK;
}
