#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <audiolite/audiolite.h>

#include "synth_worker_comm.h"

#define FMSYNTH_WORKER "/mnt/spif/fm_synth"

static midi_wtask_t g_synthwkr;

static int send_adata(midi_wtask_t *wkr, int16_t *addr)
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  hdr.grp  = MIDI_COMM_MESSAGE_HOST;
  hdr.type = MIDI_COMM_MSGTYPE_ASYNC;
  hdr.code = MIDI_COMM_MSGCODE_DATA;
  hdr.opt  = 0;

  opt.addr = (unsigned char *)midiworker_addr_convert(addr);
  opt.size = 1024 * sizeof(int16_t);

  return midi_send_message(wkr, hdr, &opt);
}

extern "C"
int send_code(bool on_xoff, uint8_t code, uint8_t vel)
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  hdr.grp  = MIDI_COMM_MESSAGE_HOST;
  hdr.type = MIDI_COMM_MSGTYPE_ASYNC;
  hdr.code = MIDI_COMM_MSGCODE_NOTE;
  hdr.opt  = 0;

  opt.note_id = code;
  opt.on_xoff = on_xoff ? 1 : 0;
  opt.vel = vel;

  return midi_send_message(&g_synthwkr, hdr, &opt);
}

extern "C"
int send_sound(unsigned char snd)
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  hdr.grp  = MIDI_COMM_MESSAGE_HOST;
  hdr.type = MIDI_COMM_MSGTYPE_ASYNC;
  hdr.code = MIDI_COMM_MSGCODE_SOUND;
  hdr.opt  = snd;

  return midi_send_message(&g_synthwkr, hdr, &opt);
}

static audiolite_mempoolapbuf *mempool;
static audiolite_outputcomp *aoutdev;
static audiolite_outputnode *pcmnode;

void init_audio_output(void)
{
  mempool = new audiolite_mempoolapbuf;
  aoutdev = new audiolite_outputcomp;
  pcmnode = new audiolite_outputnode(NULL);

  audiolite_set_systemparam(48000, 16, 2);

  mempool->create_instance(2048, 4);

  pcmnode->bind(aoutdev->get_input());

  audiolite_start(pcmnode);
}

static int bootup_worker(midi_wtask_t *wkr, const char *path)
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  printf("Boot up worker : %s\n", path);

  if (initialize_midiworker(wkr, path, false) != 0)
    {
      printf("[ERROR] Worker %s init failed\n", path);
      return -1;
    }

  hdr = midi_receive_message(wkr, &opt, true);
  if (hdr.code != MIDI_COMM_MSGCODE_BOOT)
    {
      printf("[ERROR] Boot msg not received\n");
      printf("        Received code:%d\n", hdr.code);
      return -1;
    }

  return 0;
};

/* #define WITH_FIRFILTER */

extern "C"
void play_midi(void)
{
  int i;
  audiolite_memapbuf *mem;

  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;

  if (bootup_worker(&g_synthwkr, FMSYNTH_WORKER) != 0)
    {
      printf("[ERROR] Fatal error to boot worker...\n");
      return;
    }

  init_audio_output();

  printf("Inject audio buffer into audio driver.\n");


  for (i = 0; i < 4; i++)
    {
      mem = (audiolite_memapbuf *)mempool->allocate();
      mem->set_fs(48000);
      mem->set_channels(2);
      mem->set_storedsize(2048);
      memset(mem->get_data(), 0, 2048);

      printf("Push init data : %08x\n", (unsigned int)mem);
      pcmnode->push_data(mem);
      mem->release();
    }

  printf("Get 2 empty buffer \n");

  mem = (audiolite_memapbuf *)mempool->allocate();
  mem->set_fs(48000);
  mem->set_channels(2);
  mem->set_storedsize(2048);
  printf("   Twe : %08x \n", (unsigned int)mem);

  send_adata(&g_synthwkr, (int16_t *)mem->get_data());

  printf("Loop\n");
  while (1)
    {
      /* Rendering audio data */

      hdr = midi_receive_message(&g_synthwkr, &opt, true);
      pcmnode->push_data(mem);
      mem->release();

      mem = (audiolite_memapbuf *)mempool->allocate();
      mem->set_fs(48000);
      mem->set_channels(2);
      mem->set_storedsize(2048);

      /* Send data */

      send_adata(&g_synthwkr, (int16_t *)mem->get_data());
    }
}
