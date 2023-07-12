/****************************************************************************
 * modules/include/audiolite/alworker_comm.h
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

#ifndef __SYNTH_WORKER_COMM_H
#define __SYNTH_WORKER_COMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#ifndef __linux__
#include <asmp/mpmq.h>
#ifndef BUILD_TGT_ASMPWORKER
#  include <asmp/mptask.h>
#endif
#endif

#include <audioutils/fmsynth.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIDI_COMM_MQ_NAMESEND  (2)

#define MIDI_COMM_MESSAGE_HOST   (0)
#define MIDI_COMM_MESSAGE_WORKER (1)

#define MIDI_COMM_MSGTYPE_NONE  (0)
#define MIDI_COMM_MSGTYPE_ASYNC (1)
#define MIDI_COMM_MSGTYPE_SYNC  (2)

#define MIDI_COMM_MSGCODE_SETUP (0)
#define MIDI_COMM_MSGCODE_DATA  (1)
#define MIDI_COMM_MSGCODE_NOTE  (2)
#define MIDI_COMM_MSGCODE_SOUND (3)
#define MIDI_COMM_MSGCODE_MUTE  (4)
#define MIDI_COMM_MSGCODE_BOOT  (200)

#define MIDI_COMM_MSGCODEERR_OK      (0)
#define MIDI_COMM_MSGCODEERR_UNKNOWN (1)

#define MIDI_COMM_ERR_SUCCESS      (0)
#define MIDI_COMM_ERR_WORKERINIT   (-1)
#define MIDI_COMM_ERR_WORKERASSIGN (-2)
#define MIDI_COMM_ERR_SENDMQCREATE (-3)
#define MIDI_COMM_ERR_SENDMQBIND   (-4)
#define MIDI_COMM_ERR_RECVMQCREATE (-5)
#define MIDI_COMM_ERR_EXECWORKER   (-6)

#define MIDI_WORKER_VERSION_0 (0)

#define MIDI_COMM_NO_MSG (0xffffffff)

#define MIDI_MSGBUF_DEPTH_POW  (4)
#define MIDI_MSGBUF_DEPTH      (1 << MIDI_MSGBUF_DEPTH_POW)

#define MEMBLOCK_NUM  (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

union midi_comm_msghdr_u
{
  struct
    {
      unsigned char grp;
      unsigned char type;
      unsigned char code;
      unsigned char opt;
    };
  uint32_t u32;
};
typedef union midi_comm_msghdr_u midi_comm_msghdr_t;

union midi_comm_msgopt_u
{
  /* For output */

  struct
    {
        unsigned char *addr;
        unsigned int size;
    };

  /* For input */

  struct
    {
      int note_id;
      int on_xoff;
      int vel;
    };

  /* Set up */

  struct
    {
      int fs;
      fmsynth_eglevels_t *lvl;
    };
};
typedef union midi_comm_msgopt_u midi_comm_msgopt_t;

struct midi_msg_s
{
  midi_comm_msghdr_t hdr;
  midi_comm_msgopt_t opt;
};
typedef struct midi_msg_s midi_msg_t;

struct midi_wtask_s
{
#ifndef BUILD_TGT_ASMPWORKER
  mptask_t wtask;
  bool is_spk; 
#endif
  mpmq_t mqsend;
  struct midi_msg_s msg[MIDI_MSGBUF_DEPTH];
  int msg_index;
};
typedef struct midi_wtask_s midi_wtask_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void *midiworker_addr_convert(void *a);
int initialize_midiworker(midi_wtask_t *inst, const char *dspfname, bool is_spk);
midi_comm_msghdr_t midi_receive_message(midi_wtask_t *inst,
                                    midi_comm_msgopt_t *opt, int block);
midi_comm_msghdr_t midi_receive_messageto(midi_wtask_t *inst,
                                      midi_comm_msgopt_t *opt, int ms);
int midi_send_message(midi_wtask_t *inst,
                    midi_comm_msghdr_t hdr, midi_comm_msgopt_t *opt);
int finalize_midiworker(midi_wtask_t *inst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SYNTH_WORKER_COMM_H */
