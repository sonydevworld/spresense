/****************************************************************************
 * examples/tf_example/tc_example_main.c
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

#include <nuttx/config.h>
#include <stdio.h>

#include "tensorflow/lite/micro/spresense/debug_log_callback.h"

#ifndef CONFIG_EXTERNALS_TENSORFLOW
#error "tf_example require external library of tensorflow lite micro." \
  " Please enable it as set EXTERNALS_TENSORFLOW in configuration."
#endif

#ifdef CONFIG_EXTERNALS_TENSORFLOW_EXAMPLE_HELLOWORLD
#include "examples/hello_world/main_functions.h"
#define EXAMPLE_NAME  "Hello world"
#endif

#ifdef CONFIG_EXTERNALS_TENSORFLOW_EXAMPLE_MICROSPEECH
#include "examples/micro_speech/main_functions.h"
#define EXAMPLE_NAME  "Micro Speech"
#endif

#ifdef CONFIG_EXTERNALS_TENSORFLOW_EXAMPLE_PERSONDETECTION
#include "examples/person_detection/main_functions.h"
#define EXAMPLE_NAME  "Person Detection"
#endif

#ifndef EXAMPLE_NAME
#error "To use tf_example, you need to have one of the Tensorflow samples" \
  " selected in the configuration."
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void debug_log_printf(const char *s)
{
  printf(s);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  printf("Start Tensorflow example : " EXAMPLE_NAME "\n");

  /* Register callback for printing debug log */

  RegisterDebugLogCallback(debug_log_printf);

  setup();

  while (1)
    {
      loop();
    }

  return 0;
}
