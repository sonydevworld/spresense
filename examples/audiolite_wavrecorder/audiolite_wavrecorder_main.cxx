/****************************************************************************
 * examples/audiolite_wavrecorder/audiolite_wavrecorder_main.cxx
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#include "event_str.h"

/****************************************************************************
 * Privete Class
 ****************************************************************************/

/* For Event receiving */

class my_wenclistener : public audiolite_eventlistener
{
  public:
    void on_event(int evt, audiolite_component *cmp,
                  unsigned long arg)
    {
      printf("Event %s is happened : %d\n", convert_evtid(evt),
                                            (int)arg);
    }
};

/* For Monitoring receiving data.
 * To make original component, need to ihnerit audiolite_component
 */

class my_interceptor : public audiolite_component
{
  public:
    void on_data()
    {
      /* This method is called when data is comming. */

      /* Get comming data */

      audiolite_memapbuf *mem = (audiolite_memapbuf *)pop_data();

      if (mem)
        {
          /* If the data is available, let's see the data */

          int16_t *data = (int16_t *)mem->get_data();
          int samples = mem->get_storedsize() /
                        mem->get_channels() / sizeof(int16_t);

          /* Display top 4 samples on both L and R */

          printf("DL %d : %6d %6d\n", samples, data[0], data[1]);

          /* Pass the data to later block. */

          push_data(mem);

          /* Release the memory it after finishing use */

          mem->release();
        }
    }
};


/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
int main(int argc, FAR char *argv[])
{
  int ret;
  my_wenclistener lsn;

  if (argc != 2)
    {
      printf("Usage nsh> %s <Recording wav file prefix "
             "like \"/mnt/sd0/rec\">\n", argv[0]);
      return -1;
    }

  /* To Create below structure.
   *
   * +----------------------------------+
   * | my_wenclistener to listen events |
   * +----------------------------------+
   *     ^            ^           ^
   *     |            |           |
   * +-------+    +-------+    +-----+    +--------+
   * | Audio |    | Data  |    | WAV |    | File   |
   * | Input | -> | Monito| -> | Enc | -> | Stream |
   * +-------+    +-------+    +-----+    ---------+
   *       ^
   *       |
   *      +------+
   *      |Memroy|
   *      | Pool |
   *      +------+
   */

  audiolite_filestream *fstream = new audiolite_filestream;
  audiolite_mempoolapbuf *mempool = new audiolite_mempoolapbuf;
  audiolite_inputcomp *aindev = new audiolite_inputcomp;
  audiolite_wavenc *wavenc = new audiolite_wavenc;
  my_interceptor *intercept = new my_interceptor;

  /* Setup system parameter as
   *   Output Sampling rate : 48K
   *   Output bitwidth for each sample : 16 bits
   *   Output channels : 2 channel
   */

  audiolite_set_systemparam(48000, 16, 2);

  /* Set listener to listen system events */

  audiolite_set_evtlistener(&lsn);

  /* Setup memory pool to receive audio data from the Input device
   * as 4096bytes x 32blocks.
   */

  mempool->create_instance(4096, 32);

  /* Setup WAV Encoder */

  /* Set file stream into WAV Encoder */

  wavenc->set_stream(fstream);

  /* Set recording file name prefix */

  wavenc->set_fileprefix(argv[1]);

  /* Audio Input device setting */

  /* Set memory pool */

  aindev->set_mempool(mempool);

  /* Connect Audio input device to later blocks
   *     aindev -> intercept -> wavenc
   */

  aindev->bind(intercept);
  intercept->bind(wavenc);

  /* Let's Record */

  printf("Start Recording 10 sec : %s_00.wav\n", argv[1]);
  ret = aindev->start();
  if (ret != OK)
    {
      printf("Start error..: %d\n", ret);
      goto app_error;
    }

  /* If recording time is 10 sec,
   * Let's wait 10 sec.
   */

  for (int i = 0; i < 10; i++)
    {
      printf("."); fflush(stdout);
      sleep(1);
    }

  printf("\n");

  /* Stop Recording */

  printf("Stop record\n");
  aindev->stop();

app_error:

  /* Clean up */

  aindev->unbindall();
  intercept->unbindall();

  printf("Delete instances\n");

  delete aindev;
  delete wavenc;
  delete mempool;
  delete intercept;
  delete fstream;

  printf("Delete system event handler\n");

  audiolite_eventdestroy();

  return 0;
}
