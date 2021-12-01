/****************************************************************************
 * mpcomm_prime/worker/mpcomm/mpcomm_main.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <math.h>
#include <string.h>

#include <mpcomm/mpcomm.h>

#include "prime.h"
#include "asmp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int split_load(int start, int end, prime_data_t *tasks, int task_num)
{
  int i;
  int total_load = end - start;
  int load = (int)ceilf((float)total_load / task_num);
  int offset = start;

  wk_memset(tasks, 0, sizeof(prime_data_t) * task_num);

  for (i = 0; i < task_num; ++i)
    {
      tasks[i].start = offset;
      offset += load;
      tasks[i].end = offset;

      if ((end - offset) < load)
        {
          load = end - offset;
        }
    }

  return 0;
}

static void controller_user_func(void *data)
{
  int i;

  prime_data_t *main_task = (prime_data_t *)data;
  int task_num = mpcomm_get_helpers_num() + 1;

  prime_data_t split_tasks[MPCOMM_MAX_HELPERS + 1];

  split_load(main_task->start, main_task->end, split_tasks, task_num);

  for (i = 0; i < task_num; ++i)
    {
      if (i < mpcomm_get_helpers_num())
        {
          mpcomm_send_helper(i, MEM_V2P(&split_tasks[i]));
        }
      else
        {
          split_tasks[i].result = find_primes(split_tasks[i].start,
                                              split_tasks[i].end);
        }
    }

  mpcomm_wait_helpers_done();

  main_task->result = 0;

  for (i = 0; i < task_num; ++i)
    {
      main_task->result += split_tasks[i].result;
    }
}

static void helper_user_func(void *data)
{
  prime_data_t *task = (prime_data_t *)data;

  task->result = find_primes(task->start, task->end);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  return mpcomm_main(controller_user_func, helper_user_func);
}
