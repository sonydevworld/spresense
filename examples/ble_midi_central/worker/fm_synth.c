/****************************************************************************
 * modules/audiolite/worker/mp3dec/sprmp3_msghandler.c
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

#include <string.h>

#include "synth_worker_comm.h"

#define KEYCH_NUM  (6)
/* #define NO_KEY    (0xF0) */
#define NO_KEY    (0xF00)
#define CARRIER_LEVEL (1.f / 8.f)

struct fmsound_s
{
  fmsynth_sound_t snd[3][KEYCH_NUM];

  /* For 1st sound */

  fmsynth_op_t op1[KEYCH_NUM];
  fmsynth_eg_t eg1[KEYCH_NUM];

  /* For 2nd sound */

  fmsynth_op_t op2[KEYCH_NUM];
  fmsynth_eg_t eg2[KEYCH_NUM];

  /* For 3rd sound */

  fmsynth_op_t op3[KEYCH_NUM];
  fmsynth_eg_t eg3[KEYCH_NUM];

  fmsynth_op_t op3sub[KEYCH_NUM];
  fmsynth_eg_t eg3sub[KEYCH_NUM];

  int key_map[KEYCH_NUM];
  int key_bitmap;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct memblk_s
{
  int16_t *addr;
  int size;
} memblks[MEMBLOCK_NUM];
int mem_bitmap;

midi_wtask_t g_mp3dec_task;
struct fmsound_s sound;
int cur_snd;
int send_counter;

static const float musicmidi_scale[] =
{
  /* Octave 0 */

  -1.f, -1.f, -1.f,
  -1.f, -1.f, -1.f,
  -1.f, -1.f, -1.f,
  27.5f, 29.13523509f, 30.86770631f,

  /* Octave 1 */

  32.70319563f, 34.64782883f, 36.70809593f,
  38.89087289f, 41.20344452f, 43.65352881f,
  46.2493027f, 48.99942933f, 51.913087f,
  55.f, 58.27047017f, 61.73541262f,

  /* Octave 2 */

  65.40639126f, 69.29565765f, 73.41619185f,
  77.78174577f, 82.40688903f, 87.30705762f,
  92.4986054f, 97.99885866f, 103.826174f,
  110.f, 116.5409403f, 123.4708252f,

  /* Octave 3 */

  130.8127825f, 138.5913153f, 146.8323837f,
  155.5634915f, 164.8137781f, 174.6141152f,
  184.9972108f, 195.9977173f, 207.652348f,
  220.f, 233.0818807f, 246.9416505f,

  /* Octave 4 */

  261.625565f, 277.1826306f, 293.6647674f,
  311.1269831f, 329.6275561f, 349.2282305f,
  369.9944216f, 391.9954347f, 415.304696f,
  440.f, 466.1637614f, 493.8833009f,

  /* Octave 5 */

  523.2511301f, 554.3652612f, 587.3295348f,
  622.2539662f, 659.2551123f, 698.456461f,
  739.9888432f, 783.9908693f, 830.6093921f,
  880.f, 932.3275227f, 987.7666018f,

  /* Octave 6 */

  1046.50226f, 1108.730522f, 1174.65907f,
  1244.507932f, 1318.510225f, 1396.912922f,
  1479.977686f, 1567.981739f, 1661.218784f,
  1760.f, 1864.655045f, 1975.533204f,

  /* Octave 7 */

  2093.00452f, 2217.461045f, 2349.318139f,
  2489.015865f, 2637.020449f, 2793.825844f,
  2959.955373f, 3135.963477f, 3322.437568f,
  3520.f, 3729.310091f, 3951.066407f,

  /* Octave 8 */

  4186.009041f, -1.f, -1.f,
  -1.f, -1.f, -1.f,
  -1.f, -1.f, -1.f,
  -1.f, -1.f, -1.f,
};

static void init_sound(void)
{
  int i;
  fmsynth_eglevels_t level;

  cur_snd = 0;
  fmsynth_initialize(48000);

  /* For sound 1 */

  level.attack.level       = 1.0f;
  level.attack.period_ms   = 40;
  level.decaybrk.level     = 0.3f;
  level.decaybrk.period_ms = 200;
  level.decay.level        = 0.1f;
  level.decay.period_ms    = 100;
  level.sustain.level      = 0.1f;
  level.sustain.period_ms  = 100;
  level.release.level      = 0.f;
  level.release.period_ms  = 0;

  for (i = 0; i < KEYCH_NUM; i++)
    {
      create_fmsynthsnd(&sound.snd[0][i]);
      create_fmsynthop(&sound.op1[i], &sound.eg1[i]);

      fmsynthop_set_envelope(&sound.op1[i], &level);
      /*fmsynthop_select_opfunc(&sound.op1[i], FMSYNTH_OPFUNC_SQUARE);*/
      fmsynthop_select_opfunc(&sound.op1[i], FMSYNTH_OPFUNC_SIN);
      
      fmsynthsnd_set_operator(&sound.snd[0][i], &sound.op1[i]);
      fmsynthsnd_set_volume(&sound.snd[0][i], CARRIER_LEVEL);
    }

  for (i = 1; i < KEYCH_NUM; i++)
    {
      fmsynthsnd_add_subsound(&sound.snd[0][0], &sound.snd[0][i]);
    }

  /* For sound 2 */

  level.attack.level       = 1.0f;
  level.attack.period_ms   = 40;
  level.decaybrk.level     = 0.3f;
  level.decaybrk.period_ms = 200;
  level.decay.level        = 0.1f;
  level.decay.period_ms    = 100;
  level.sustain.level      = 0.1f;
  level.sustain.period_ms  = 100;
  level.release.level      = 0.f;
  level.release.period_ms  = 0;

  for (i = 0; i < KEYCH_NUM; i++)
    {
      create_fmsynthsnd(&sound.snd[1][i]);
      create_fmsynthop(&sound.op2[i], &sound.eg2[i]);
      fmsynthop_set_envelope(&sound.op2[i], &level);
      fmsynthop_select_opfunc(&sound.op2[i], FMSYNTH_OPFUNC_SIN);
      fmsynthop_bind_feedback(&sound.op2[i], &sound.op2[i], 0.6f);

      fmsynthsnd_set_operator(&sound.snd[1][i], &sound.op2[i]);
      fmsynthsnd_set_volume(&sound.snd[1][i], CARRIER_LEVEL);
    }

  for (i = 1; i < KEYCH_NUM; i++)
    {
      fmsynthsnd_add_subsound(&sound.snd[1][0], &sound.snd[1][i]);
    }

  /* For sound 3 */

  level.attack.level       = 1.0f;
  level.attack.period_ms   = 50;
  level.decaybrk.level     = 0.6f;
  level.decaybrk.period_ms = 180;
  level.decay.level        = 0.4f;
  level.decay.period_ms    = 100;
  level.sustain.level      = 0.1f;
  level.sustain.period_ms  = 50;
  level.release.level      = 0.f;
  level.release.period_ms  = 0;

  for (i = 0; i < KEYCH_NUM; i++)
    {
      create_fmsynthsnd(&sound.snd[2][i]);
      create_fmsynthop(&sound.op3[i], &sound.eg3[i]);
      create_fmsynthop(&sound.op3sub[i], &sound.eg3sub[i]);

      fmsynthop_set_envelope(&sound.op3[i], &level);
      fmsynthop_select_opfunc(&sound.op3[i], FMSYNTH_OPFUNC_SIN);

      fmsynthop_set_soundfreqrate(&sound.op3sub[i], 3.7f);
      fmsynthop_select_opfunc(&sound.op3sub[i], FMSYNTH_OPFUNC_SIN);

      fmsynthop_cascade_subop(&sound.op3[i], &sound.op3sub[i]);

      fmsynthsnd_set_operator(&sound.snd[2][i], &sound.op3[i]);
      fmsynthsnd_set_volume(&sound.snd[2][i], CARRIER_LEVEL);
    }

  for (i = 1; i < KEYCH_NUM; i++)
    {
      fmsynthsnd_add_subsound(&sound.snd[2][0], &sound.snd[2][i]);
    }

  for (i = 0; i < KEYCH_NUM; i++)
    {
      sound.key_map[i] = NO_KEY;
    }

  sound.key_bitmap = 0;
}

/****************************************************************************
 * Private functions
 ****************************************************************************/

static int search_next_buff(void)
{
  int i;

  for (i = 0; i < MEMBLOCK_NUM; i++)
    {
      if (mem_bitmap & (1 << i))
        {
          break;
        }
    }

  return i;
}

static int render_buffer(void)
{
  int blkidx;
  struct memblk_s *mem;

  blkidx = search_next_buff();

  if (blkidx < MEMBLOCK_NUM)
    {
      mem = &memblks[blkidx];
      mem->size = fmsynth_rendering(&sound.snd[cur_snd][0],
                                   (int16_t *)mem->addr,
                                   mem->size / sizeof(int16_t),
                                   2, NULL, 0);
    }

  return blkidx;
}

static int send_buffer(int blkidx)
{
  int ret = -1;
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;
  struct memblk_s *mem;

  if (blkidx < MEMBLOCK_NUM)
    {
      mem = &memblks[blkidx];

      hdr.grp  = MIDI_COMM_MESSAGE_WORKER;
      /* hdr.type = MIDI_COMM_MSGTYPE_ASYNC; */
      hdr.code = MIDI_COMM_MSGCODE_DATA;
      hdr.opt  = (unsigned char)send_counter;
      /* hdr.opt  = (unsigned char)sound.key_bitmap; */
      hdr.type  = (unsigned char)mem_bitmap;

      opt.addr = (unsigned char *)mem->addr;
      opt.size = mem->size;

      mem_bitmap &= ~(1 << blkidx);
      
      ret = midi_send_message(&g_mp3dec_task, hdr, &opt);
      send_counter++;
    }

  return ret;
}

static void store_mem(unsigned char *addr, int size)
{
  int blkidx;

  for (blkidx = 0; blkidx < MEMBLOCK_NUM; blkidx++)
    {
      if (!(mem_bitmap & (1 << blkidx)))
        {
          memblks[blkidx].addr = (int16_t *)addr;
          memblks[blkidx].size = size;
          mem_bitmap |= (1 << blkidx);
          return;
        }
    }
}

static int search_emptych(void)
{
  int ret;

  for (ret = 0; ret < KEYCH_NUM; ret++)
    {
      if (sound.key_map[ret] == NO_KEY)
        {
          break;
        }
    }

  return ret;
}

static int search_samech(int key)
{
  int ret;

  for (ret = 0; ret < KEYCH_NUM; ret++)
    {
      if (sound.key_map[ret] == key)
        {
          break;
        }
    }

  return ret;
}

static void handle_note(midi_comm_msgopt_t *opt)
{
  int ch;

  if (opt->on_xoff)
    {
      ch = search_emptych();
      if (ch < KEYCH_NUM)
        {
          fmsynthsnd_set_soundfreq(&sound.snd[cur_snd][ch],
                                   musicmidi_scale[opt->note_id]);
          sound.key_map[ch] = opt->note_id;
          sound.key_bitmap |= (1 << ch);
        }
    }
  else
    {
      ch = search_samech(opt->note_id);
      if (ch < KEYCH_NUM)
        {
          fmsynthop_stop(sound.snd[cur_snd][ch].operators);
          sound.key_map[ch] = NO_KEY;
          sound.key_bitmap &= ~(1 << ch);
        }
    }
}

static void handle_sound(unsigned char snd)
{
  if (snd < 3 && cur_snd != snd)
    {
      fmsynthsnd_stop(&sound.snd[cur_snd][0]);
      cur_snd = snd;
    }
}

static void handle_mute(void)
{
}

/*** name: handle_message */

static void handle_message(midi_comm_msghdr_t hdr,
                           midi_comm_msgopt_t *opt)
{
  if (hdr.grp == MIDI_COMM_MESSAGE_HOST)
    {
      switch (hdr.code)
        {
          case MIDI_COMM_MSGCODE_SETUP:
            break;

          case MIDI_COMM_MSGCODE_DATA:
            store_mem(opt->addr, opt->size);
            break;

          case MIDI_COMM_MSGCODE_NOTE:
            handle_note(opt);
            break;

          case MIDI_COMM_MSGCODE_SOUND:
            handle_sound(hdr.opt);
            break;

          case MIDI_COMM_MSGCODE_MUTE:
            handle_mute();
            break;
        }
    }
  else
    {
      hdr.opt = MIDI_COMM_MSGCODEERR_UNKNOWN;
      midi_send_message(&g_mp3dec_task, hdr, opt);
    }
}

static int send_bootmsg(void)
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  hdr.grp  = MIDI_COMM_MESSAGE_WORKER;
  /* hdr.type = MIDI_COMM_MSGTYPE_ASYNC; */
  hdr.type  = (unsigned char)mem_bitmap;
  hdr.code = MIDI_COMM_MSGCODE_BOOT;
  /* hdr.opt  = AL_WORKER_VERSION_0; */
  hdr.opt  = (unsigned char)send_counter;

  send_counter++;
  return midi_send_message(&g_mp3dec_task, hdr, &opt);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*** name: sprmp3_pollmessage */

static void sprmp3_pollmessage(int block)
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  do
    {
      hdr = midi_receive_message(&g_mp3dec_task, &opt, block);
      if (hdr.u32 != MIDI_COMM_NO_MSG)
        {
          handle_message(hdr, &opt);
        }
    }
  while (!block && hdr.u32 != MIDI_COMM_NO_MSG);
}

int main(void)
{
  initialize_midiworker(&g_mp3dec_task, "", false);
  init_sound();

  mem_bitmap = 0;
  send_counter = 0;

  send_bootmsg();

  while (1)
    {
      if ((mem_bitmap & 0x0f) == 0)
        {
          sprmp3_pollmessage(1);
        }
      else
        {
          sprmp3_pollmessage(0);
          send_buffer(render_buffer());
        }
    }
}
