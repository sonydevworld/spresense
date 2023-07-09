/****************************************************************************
 * examples/audiolite_wavplayer/audiolite_wavplayer_main.c
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

class my_wdeclistener : public audiolite_eventlistener
{
  public:
    volatile bool playing;

  public:
    my_wdeclistener() : playing(false){};

    void on_event(int evt, audiolite_component *cmp,
                  unsigned long arg)
    {
      printf("WAV Dec Event %s is happened : %d\n", convert_evtid(evt),
                                            (int)arg);

      if (evt == AL_EVENT_DECODEDONE ||
          evt == AL_EVENT_STOPOUTPUT)
        {
          printf("Decode is done\n");
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
  int ret;
  my_wdeclistener lsn;

  if (argc != 2)
    {
      printf("Usage nsh> %s <playing wav file name>\n", argv[0]);
      return -1;
    }

  /* To Create below structure.
   *
   * +----------------------------------+
   * | my_wdeclistener to listen events |
   * +----------------------------------+
   *     ^            ^           ^
   *     |            |           |
   * +--------+    +-----+    +-------+
   * | File   |    | WAV |    | Audio |
   * | Stream | -> | Dec | -> | Output|
   * +--------+    +-----+    --------+
   *                ^
   *                |
   *      +------+  |
   *      |Memroy|  |
   *      | Pool |--+
   *      +------+
   */

  audiolite_filestream *fstream = new audiolite_filestream;
  audiolite_mempoolapbuf *mempool = new audiolite_mempoolapbuf;
  audiolite_outputcomp *aoutdev = new audiolite_outputcomp;
  audiolite_wavdec *wavdec = new audiolite_wavdec;

  /* File open on filestream as read mode */

  if (fstream->rfile(argv[1]) != OK)
    {
      printf("File open error : %d\n", errno);
      return -1;
    }

  /* Setup system parameter as
   *   Output Sampling rate : 48K
   *   Output bitwidth for each sample : 16 bits
   *   Output channels : 2 channel
   */

  audiolite_set_systemparam(48000, 16, 2);

  /* Set listener to listen system events */

  audiolite_set_evtlistener(&lsn);

  /* Setup memory pool to read WAV data from the file
   * as 4096bytes x 8blocks.
   */

  mempool->create_instance(4096, 8);

  /* Setup WAV Decorder */

  /* Set memory pools on WAV Decorder */

  wavdec->set_mempool(mempool);

  /* Set file stream on WAV Decorder */

  wavdec->set_stream(fstream);

  /* Connect WAV output to Audio Output device */

  wavdec->bind(aoutdev);

  /* Let's play */

  printf("Start player : %s\n", argv[1]);
  lsn.playing = true;
  ret = wavdec->start();
  if (ret != OK)
    {
      printf("Start error..: %d\n", ret);
      goto app_error;
    }

  /* Wait for finishing as receiving AL_EVENT_DECODEDONE
   * in my_wdeclistener.
   */

  while (lsn.playing)
    {
      usleep(10 * 1000);
    }

  /* Stop playing */

  printf("Stop player\n");
  wavdec->stop();

app_error:

  /* Clean up */

  wavdec->unbindall();
  printf("Delete instances\n");

  delete fstream;
  delete wavdec;
  delete aoutdev;
  delete mempool;

  printf("Delete system event handler\n");

  audiolite_eventdestroy();

  return 0;
}
