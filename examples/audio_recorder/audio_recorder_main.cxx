/****************************************************************************
 * examples/audio_recorder/audio_recorder_main.cxx
 *
 *   Copyright 2018-2021 Sony Semiconductor Solutions Corporation
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
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <sys/stat.h>

#include "audio_util.h"

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Recorder mode definition
 ****************************************************************************/

/* Codec definition.
 *   Select what you want and uncomment it.
 */

//const uint8_t codec_type = AS_CODECTYPE_MP3;
const uint8_t codec_type = AS_CODECTYPE_LPCM;

/* Container format definition.
 *   Select what you want and uncomment it.
 */

const format_type_e format_type = FORMAT_TYPE_WAV;
//const format_type_e format_type = FORMAT_TYPE_RAW;

/* Do not support WAV & MP3 */

static_assert(!((codec_type == AS_CODECTYPE_MP3) && (format_type == FORMAT_TYPE_WAV)));

/* Sampling rate definition.
 *   Select what you want and uncomment it.
 */

//const uint32_t sampling_rate = AS_SAMPLINGRATE_16000;
const uint32_t sampling_rate = AS_SAMPLINGRATE_48000;
//const uint32_t sampling_rate = AS_SAMPLINGRATE_192000;

/* Use microphone channel number defintion.
 *   Select what you want and uncomment it.
 *
 *   [Analog microphone]
 *       Maximum number: 4
 *       The channel number is MONO/STEREO/4CH
 *   [Digital microphone]
 *       Maximum number: 8
 *       The channel number is MONO/STEREO/4CH/8CH
 */

//const uint8_t channel_number = AS_CHANNEL_MONO;
const uint8_t channel_number = AS_CHANNEL_STEREO;
//const uint8_t channel_number = AS_CHANNEL_4CH;
//const uint8_t channel_number = AS_CHANNEL_8CH;

/* Bit length definition.
 *   Select what you want and uncomment it.
 */

const uint8_t bit_length = AS_BITLENGTH_16;
//const uint8_t bit_length = AS_BITLENGTH_24;

/* Default microphone gain */

const uint32_t default_mic_gain = 0;

/* The path of recorded files */

#define RECFILE_ROOTPATH "/mnt/sd0/REC"

/* Simple FIFO definition */

#define READ_SIMPLE_FIFO_BYTES (768 * (bit_length / 8) * channel_number)
#define SIMPLE_FIFO_FRAME_NUM 60
#define SIMPLE_FIFO_BUF_BYTES  (READ_SIMPLE_FIFO_BYTES * SIMPLE_FIFO_FRAME_NUM)

static uint32_t fifo_memory[SIMPLE_FIFO_BUF_BYTES / sizeof(uint32_t)];
static uint32_t write_buf[READ_SIMPLE_FIFO_BYTES / sizeof(uint32_t)];

/* file write buffer size. */

#define STDIO_BUFFER_SIZE 4096

/* Length of recording file name */

#define MAX_PATH_LENGTH 128

/* Recording time(sec). */

#define RECORDER_REC_TIME 10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DIR *dirp;
static FILE *fd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void outputDeviceCallback(uint32_t size)
{
  /* do nothing */
}

static bool app_open_output_file(void)
{
  static char fname[MAX_PATH_LENGTH];

  struct tm        *cur_time;
  struct timespec   cur_sec;
  const char       *ext;

  /* Decide file extension according to file format. */

  if ((codec_type == AS_CODECTYPE_MP3) && (format_type == FORMAT_TYPE_RAW))
    {
      ext = "mp3";
    }
  else if (codec_type == AS_CODECTYPE_LPCM)
    {
      if (format_type == FORMAT_TYPE_WAV)
        {
          ext = "wav";
        }
      else
        {
          ext = "raw";
        }
    }
  else if ((codec_type == AS_CODECTYPE_OPUS) && (format_type == FORMAT_TYPE_RAW))
    {
      ext = "opus";
    }
  else
    {
      printf("Unsupported format\n");
      return false;
    }

  /* Use date time as recording file name. */

  clock_gettime(CLOCK_REALTIME, &cur_sec);
  cur_time = gmtime(&cur_sec.tv_sec);

  snprintf(fname,
           MAX_PATH_LENGTH,
           "%s/%04d%02d%02d_%02d%02d%02d.%s",
           RECFILE_ROOTPATH,
           cur_time->tm_year + 1900,
           cur_time->tm_mon + 1,
           cur_time->tm_mday,
           cur_time->tm_hour,
           cur_time->tm_min,
           cur_time->tm_sec,
           ext);

  fd = fopen(fname, "w");
  if (fd == 0)
    {
      printf("open err(%s)\n", fname);
      return false;
    }

  setvbuf(fd, NULL, _IOLBF, STDIO_BUFFER_SIZE);
  printf("Record data to %s.\n", &fname[0]);

  if (format_type == FORMAT_TYPE_WAV)
    {
      if (!create_wav_header(fd))
        {
          printf("Error: create_wav_header() failure.\n");
          return false;
        }
    }

  return true;
}

static void app_close_output_file(void)
{
  fclose(fd);
}

static void app_attention_callback(const ErrorAttentionParam *attparam)
{
  printf("Attention!! %s L%d ecode %d subcode %ld\n",
          attparam->error_filename,
          attparam->line_number,
          attparam->error_code,
          attparam->error_att_sub_code);
}

static bool app_start_recorder(void)
{
  if (!app_open_output_file())
    {
      return false;
    }

  if (!start_recording())
    {
      app_close_output_file();
      return false;
    }

  return true;
}

static void app_stop_recorder(void)
{
  stop_recording(fd);

  if (format_type == FORMAT_TYPE_WAV)
    {
      if (!update_wav_file_size(fd))
        {
          printf("Error: app_write_wav_header() failure.\n");
        }
    }

  app_close_output_file();
}

static bool app_open_file_dir(const char *name)
{
  int ret;

  dirp = opendir("/mnt");
  if (!dirp)
    {
      printf("opendir err(errno:%d)\n", errno);
      return false;
    }

  ret = mkdir(name, 0777);
  if (ret != 0)
    {
      if (errno != EEXIST)
        {
          printf("mkdir err(errno:%d)\n", errno);
          return false;
        }
    }

  return true;
}

static bool app_close_file_dir(void)
{
  closedir(dirp);

  return true;
}

void app_recorde_process(uint32_t rec_time)
{
  /* Timer Start */

  time_t start_time;
  time_t cur_time;

  time(&start_time);

  do
    {
      /* Check the FIFO every 5 ms and fill if there is space. */

      usleep(5 * 1000);
      if (!write_frames(fd))
        {
         app_close_output_file();
         return;
        }
    } while((time(&cur_time) - start_time) < rec_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  printf("Start AudioRecorder example\n");

  /* Create & initialize AudioSubSystem. */

  if (!create_high_level_audio(fifo_memory, sizeof(fifo_memory), READ_SIMPLE_FIFO_BYTES,
                               write_buf, outputDeviceCallback, app_attention_callback))
    {
      printf("Error: create_high_level_audio() failure.\n");
      return 1;
    }

  /* Open directory of recording file. */

  if (!app_open_file_dir(RECFILE_ROOTPATH))
    {
      printf("Error: app_open_file_dir() failure.\n");
      goto ErrorReturn;
    }

  /* Set the initial gain of the microphone to be used.
   * Parameter can be set from -7850 to 210 as from -78.5db to 21.0db
   */

  if (!set_mic_gain(default_mic_gain))
    {
      printf("Error: set_mic_gain() failure.\n");
      goto ErrorReturn;
    }

  /* Set recorder operation mode. */

  if (!set_recording_mode(codec_type,
                          format_type,
                          sampling_rate,
                          bit_length,
                          channel_number,
                          "PREPROC"))
    {
      printf("Error: set_recording_mode() failure.\n");
      goto ErrorReturn;
    }

  /* Start recorder operation. */

  if (!app_start_recorder())
    {
      printf("Error: app_start_recorder() failure.\n");
      standby_audio();
      goto ErrorReturn;
    }

  /* Running... */

  printf("Running time is %d sec\n", RECORDER_REC_TIME);

  app_recorde_process(RECORDER_REC_TIME);

  /* Stop recorder operation. */

  app_stop_recorder();

  /* Close directory of recording file. */

  if (!app_close_file_dir())
    {
      printf("Error: app_close_contents_dir() failure.\n");
      goto ErrorReturn;
    }

  /* Return the state of AudioSubSystem before voice_call operation. */

  if (!standby_audio())
    {
      printf("Error: standby_audio() failure.\n");
      goto ErrorReturn;
    }

  /* finalize and delete the AudioSubSystem with all resource. */

  if (!delete_high_level_audio())
    {
      printf("Error: delete_high_level_audio() failure.\n");
      goto ErrorReturn;
    }

  printf("Exit AudioRecorder example\n");

  return 0;

ErrorReturn:

  /* finalize and delete the AudioSubSystem with all resource. */

  if (!delete_high_level_audio())
    {
      printf("Error: delete_high_level_audio() failure.\n");
      return 1;
    }

  printf("Exit AudioRecorder example\n");

  return 1;
}
