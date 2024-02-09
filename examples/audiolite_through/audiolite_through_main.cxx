/****************************************************************************
 * examples/audiolite_through/audiolite_through_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

#include <audiolite/audiolite.h>

#include "my_display_data.h"

/****************************************************************************
 * Privete Class
 ****************************************************************************/

/* For Event receiving */

class my_throughlistener : public audiolite_eventlistener
{
  public:
    volatile bool playing;

  public:
    my_throughlistener() : playing(false){};

    void on_event(int evt, audiolite_component *cmp,
                  unsigned long arg)
    {
      printf("AudioThrough Event %d is happened : %d\n", evt, (int)arg);

      if (evt == AL_EVENT_STOPINPUT ||
          evt == AL_EVENT_STOPOUTPUT)
        {
          printf("Through is done\n");
          playing = false;
        }
    }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
int main(int argc, FAR char *argv[])
{
  my_throughlistener lsn;

  audiolite_inputcomp *aindev = new audiolite_inputcomp;
  audiolite_outputcomp *aoutdev = new audiolite_outputcomp;
  audiolite_mempoolapbuf *mempool = new audiolite_mempoolapbuf;

#ifdef CONFIG_EXAMPLES_AUDIOLITE_THROUGH_WATCHDATA
  // This my_display_data class is defined in my_display_data.h file.
  my_display_data *disp = new my_display_data;
#endif

  /* To Create below structure.
   *
   *      +-------------------------------------+
   *      | my_throughlistener to listen events |
   *      +-------------------------------------+
   *           ^              ^               ^
   *           |              |               |
   *       +-------+    +------------+    +--------+
   *       | Audio |    |   Display  |    | Audio  |
   * MIC-> | Input | -> |    Data    | -> | Output | -> Speaker
   *       |       |    | (optional) |    |        |
   *       +-------+    +------------+    ---------+
   *           ^
   *           |
   *        +------+
   *        |Memroy|
   *        | Pool |
   *        +------+
   */

  /* Setup system parameter as
   *   Output Sampling rate : 48K
   *   Output bitwidth for each sample : 16 bits
   *   Output channels : 2 channel
   */

  audiolite_set_systemparam(48000, 16, 2);

  /* Set listener to listen system events */

  audiolite_set_evtlistener(&lsn);

  /* Setup memory pool to read MP3 data from the file
   * as 4096bytes x 8blocks.
   */

  mempool->create_instance(4096, 8);

  /* Set memory pools on Audio Input */

  aindev->set_mempool(mempool);

  /* Connect Audio Input to Audio Output
   * In a option, if you enabled EXAMPLES_AUDIOLITE_THROUGH_WATCHDATA
   * of this example config, my_display_data is set between them.
   */

#ifdef CONFIG_EXAMPLES_AUDIOLITE_THROUGH_WATCHDATA
  aindev->bind(disp);
  disp->bind(aoutdev);
#else
  aindev->bind(aoutdev);
#endif

  /* Let's Start */

  printf("Start Through data from MIC to Speaker in 10 sec\n");
  aindev->start();

  /* Wait for 10sec.
   * You can speak on the mic and it will sound on the speaker during 10sec.
   */

  for (int i = 0; i < 10; i++)
    {
      printf("."); fflush(stdout);
      sleep(1);
    }

  printf("\n");

  /* Stop playing */

  printf("Stop Through\n");
  aindev->stop();

  /* Unconnect all connections */

  aindev->unbindall();

  printf("Delete instances\n");
  audiolite_eventdestroy();

#ifdef CONFIG_EXAMPLES_AUDIOLITE_THROUGH_WATCHDATA
  delete disp;
#endif
  delete aindev;
  delete aoutdev;
  delete mempool;

  return 0;
}
