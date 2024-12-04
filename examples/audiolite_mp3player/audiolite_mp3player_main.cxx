/****************************************************************************
 * examples/audiolite_mp3player/audiolite_mp3player_main.c
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
#include <poll.h>
#include <fcntl.h>

#include <audiolite/audiolite.h>

#include "event_str.h"

/****************************************************************************
 * Privete Class
 ****************************************************************************/

/* For Event receiving */

class my_mp3listener : public audiolite_eventlistener
{
  public:
    volatile bool playing;

  public:
    my_mp3listener() : playing(false){};

    void on_event(int evt, audiolite_component *cmp,
                  unsigned long arg)
    {
      printf("Event %s is happened : %d\n", convert_evtid(evt),
                                            (int)arg);
      if (evt == AL_EVENT_STOPOUTPUT)
        {
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
  my_mp3listener lsn;
  int volume = 1000;
  struct pollfd pfd;

  /* Argument check */

  if (argc < 2)
    {
      printf("Usage nsh> %s <playing mp3 file name> (<vol>)\n",
             argv[0]);
      return -1;
    }

  if (argc >= 3)
    {
      volume = atoi(argv[2]);
      volume = (volume < 0)    ?    0 :
               (volume > 1000) ? 1000 : volume;
    }

  /* To Create below structure.
   *
   * +---------------------------------+
   * | my_mp3listener to listen events |
   * +---------------------------------+
   *     ^            ^           ^
   *     |            |           |
   * +--------+    +-----+    +-------+
   * | File   |    | MP3 |    | Audio |
   * | Stream | -> | Dec | -> | Output|
   * +--------+    +-----+    --------+
   *                ^   ^
   *                |   |
   *      +------+  |   |  +------+
   *      |Memroy|  |   |  |Memory|
   *      | Pool |--+   +--| Pool |
   *      +------+         +------+
   */

  audiolite_filestream *fstream = new audiolite_filestream;
  audiolite_mempoolapbuf *imempool = new audiolite_mempoolapbuf;
  audiolite_mempoolapbuf *omempool = new audiolite_mempoolapbuf;
  audiolite_outputcomp *aoutdev = new audiolite_outputcomp();
  audiolite_mp3dec *mp3 = new audiolite_mp3dec;

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

  /* Setup memory pool to read MP3 data from the file
   * as 4096bytes x 8blocks.
   */

  imempool->create_instance(4096, 8);

  /* Setup memory pool to stored decoded data
   * as 4096bytes x 16blocks.
   */

  omempool->create_instance(4096, 16);

  /* Setup MP3 Decorder */

  /* Set memory pools on MP3 Decorder */

  mp3->set_mempool(imempool);
  mp3->set_outputmempool(omempool);

  /* Set file stream on MP3 Decorder */

  mp3->set_stream(fstream);

  /* Connect MP3 output to Audio Output device */

  mp3->bind(aoutdev);

  /* Set volume */

  aoutdev->set_volume(volume);

  /* Let's play */

  printf("Start player : %s\n", argv[1]);
  lsn.playing = true;
  ret = mp3->start();
  if (ret != OK)
    {
      printf("Start error..: %d\n", ret);
      goto app_error;
    }

  /* Wait for finishing as receiving AL_EVENT_DECODEDONE
   * in my_mp3listener.
   */

  while (lsn.playing)
    {
      pfd.fd = fileno(stdin);
      pfd.events = POLLIN;

      poll(&pfd, 1, 10 /* ms */);
      if (pfd.revents & POLLIN)
        {
          if (getchar() == 'q')
            {
              break;
            }
        }
    }

  /* Stop playing */

  printf("Stop player\n");
  mp3->stop();

app_error:

  /* Clean up */

  mp3->unbindall();

  printf("Delete instances\n");

  delete fstream;
  delete mp3;
  delete aoutdev;
  delete imempool;
  delete omempool;

  printf("Delete system event handler\n");

  audiolite_eventdestroy();

  return 0;
}
