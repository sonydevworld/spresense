/****************************************************************************
 * examples/ble_midi_central/midi_parser.h
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

/* BLE MIDI specification : 
 *    https://www.hangar42.nl/wp-content/uploads/2017/10/BLE-MIDI-spec.pdf
 */

/* MIDI code reference :
 *    https://www.midi.org/specifications-old/item/table-1-summary-of-midi-message
 *    https://nagasm.org/ASL/midi03/index.html
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "led_indicator.h"
#include "midi_parser.h"
#include "midi_play.h"

/****************************************************************************
 * Pre-processor Definisions
 ****************************************************************************/

#define MIDIPARSE_STATE_NORMAL    (0)
#define MIDIPARSE_STATE_RUNNING   (1)
#define MIDIPARSE_STATE_EXCLUSIVE (2)

#define MSGTYPE(d) ((d) & 0xF0)
#define CHNUM(d)   ((d) & 0x0F)

#define MSGTYPE_NOTE_OFF     (0x80)
#define MSGTYPE_NOTE_ON      (0x90)
#define MSGTYPE_AFTRTOUCH    (0xA0)
#define MSGTYPE_CTLCHANGE    (0xB0)
#define MSGTYPE_PRGCHANGE    (0xC0)
#define MSGTYPE_CHAFTRTOUCH  (0xD0)

#define CONTROLLER_NUM_MAX   (120)
#define CHMODEMSG_ALLSNDOFF  (120)
#define CHMODEMSG_RSTALLCTL  (121)
#define CHMODEMSG_LCLCTL     (122)
#define CHMODEMSG_ALLNOTEOFF (123)
#define CHMODEMSG_OMNIOFF    (124)
#define CHMODEMSG_OMNION     (125)
#define CHMODEMSG_MONOON     (126)
#define CHMODEMSG_POLYON     (127)

#define ALLOFF_PARAM         (0)
#define LCLCTL_PARAM_OFF     (0)
#define LCLCTL_PARAM_ON      (127)

#define MSGTYPE_PITCHBEND    (0xE0)
#define MSGTYPE_SYSEXCL      (0xF0)
#define MSGTYPE_SYSEXCLSTART (0xF0)
#define MSGTYPE_TIMECODE     (0xF1)
#define MSGTYPE_SONGPOS      (0xF2)
#define MSGTYPE_SONGSEL      (0xF3)
#define MSGTYPE_TUNEREQ      (0xF6)
#define MSGTYPE_SYSEXCLEND   (0xF7)
#define MSGTYPE_CLOCK        (0xF8)
#define MSGTYPE_START        (0xFA)
#define MSGTYPE_CONTINUE     (0xFB)
#define MSGTYPE_STOP         (0xFC)
#define MSGTYPE_ACTSENS      (0xFE)
#define MSGTYPE_SYSRESET     (0xFF)

#define MIDIHDR_TSHIGH_FIELD (0x3F)
#define MIDI_TSLOW_FIELD     (0x7F)

#define IS_NOTMIDIDATA(d) ((d) & 0x80)
#define IS_MIDIDATA(d) (!IS_NOTMIDIDATA(d))

/* #define DEBUG_MIDIPARSER */

#ifdef DEBUG_MIDIPARSER
#define dbg_midiparser  printf
#else
#define dbg_midiparser(...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct parser_state_s;

typedef void (*msghdlr_t)(uint8_t **msg, int *len,
                          struct parser_state_s *state);

struct parser_state_s
{
  int state;
  uint16_t tmstmp;
  uint8_t header;
  uint8_t chnl;
  uint8_t msgid;
  msghdlr_t running_hdlr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void msghdlr_noteoff(uint8_t **msg, int *len,
                            struct parser_state_s *state);
static void msghdlr_noteon(uint8_t **msg, int *len,
                           struct parser_state_s *state);
static void msghdlr_aftrtouch(uint8_t **msg, int *len,
                              struct parser_state_s *state);
static void msghdlr_ctlchange(uint8_t **msg, int *len,
                              struct parser_state_s *state);
static void msghdlr_prgchange(uint8_t **msg, int *len,
                              struct parser_state_s *state);
static void msghdlr_chaftertouch(uint8_t **msg, int *len,
                                 struct parser_state_s *state);
static void msghdlr_pitchbend(uint8_t **msg, int *len,
                              struct parser_state_s *state);
static void msghdlr_sysexclusive(uint8_t **msg, int *len,
                                 struct parser_state_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct parser_state_s g_state;

const struct msghdlr_parser_s
{
  uint16_t msg_type;
  msghdlr_t hdlr;
} g_msghdlr_list[] =
{
  { MSGTYPE_NOTE_OFF,    msghdlr_noteoff      },
  { MSGTYPE_NOTE_ON,     msghdlr_noteon       },
  { MSGTYPE_AFTRTOUCH,   msghdlr_aftrtouch    },
  { MSGTYPE_CTLCHANGE,   msghdlr_ctlchange    },
  { MSGTYPE_PRGCHANGE,   msghdlr_prgchange    },
  { MSGTYPE_CHAFTRTOUCH, msghdlr_chaftertouch },
  { MSGTYPE_PITCHBEND,   msghdlr_pitchbend    },
  { MSGTYPE_SYSEXCL,     msghdlr_sysexclusive },
};
#define MSGLIST_SIZE  (sizeof(g_msghdlr_list) / sizeof(g_msghdlr_list[0]))

static int g_sndno = 0;
static int g_note_on_count;

static const char *g_notestr[] = { "C", "C#", "D", "D#", "E", "F",
                                   "F#", "G", "G#", "A", "A#", "B" };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_note(bool on, int key, int vel)
{
  int oct = key / 12;
  int note = key % 12;

  printf("[%s] O%x %s (%d)\n", on ? "ON " : "OFF",
                               oct, g_notestr[note], vel);
}

/* Execution parsed command functions
 * These functions are kinds of porting layer.
 * If other application wants to use this MIDI parser,
 * re-write these exec_xxxxx() functions to do for the application.
 */

static void exec_note_off(uint16_t ts, uint8_t ch, uint8_t key,
                          uint8_t velocity)
{
  if (g_note_on_count > 0)
    {
      g_note_on_count--;
      set_led_indicator(g_note_on_count);
    }

  send_code(false, key, velocity);
  print_note(false, key, velocity);
}

static void exec_note_on(uint16_t ts, uint8_t ch, uint8_t key,
                         uint8_t velocity)
{
  if (g_note_on_count < 7)
    {
      g_note_on_count++;
      set_led_indicator(g_note_on_count);
    }

  send_code(true, key, velocity);
  print_note(true, key, velocity);
}

static void exec_control_change(uint16_t ts, uint8_t ch, uint8_t key,
                                uint8_t pressure)
{
  int soundno = pressure >= 0x70 ? 2 : pressure <= 0x10 ? 0 : 1;

  if (g_sndno != soundno)
    {
      printf("[Change soud] %d\n", soundno);
      send_sound((unsigned char)soundno);
      g_sndno = soundno;
    }
}

static void exec_poly_key_press(uint16_t ts, uint8_t ch, uint8_t key,
                                uint8_t pressure)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, ch %d, key %d, pressure %d\n",
                  __func__, ts, ch, key, pressure);
}

static void exec_all_sound_off(uint16_t ts, uint8_t ch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_reset_all_controll(uint16_t ts, uint8_t ch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_local_controll(uint16_t ts, uint8_t ch, int on_xoff)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_all_notesoff(uint16_t ts, uint8_t ch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_omni_off(uint16_t ts, uint8_t ch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_omni_on(uint16_t ts, uint8_t ch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_mono_mode(uint16_t ts, uint8_t ch, uint8_t mode)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_polymode_on(uint16_t ts, uint8_t ch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : ts %d, ch %d\n", __func__, ts, ch);
}

static void exec_program_change(uint16_t ts, uint8_t ch, uint8_t prog)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, ch %d, prog %d\n", __func__, ts, ch, prog);
}

static void exec_channel_pressure(uint16_t ts, uint8_t ch,
                                  uint8_t pressure)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, ch %d, pressure %d\n",
                  __func__, ts, ch, pressure);
}

static void exec_pitch_bend(uint16_t ts, uint8_t ch, uint16_t pitch)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, ch %d, pitch %d\n", __func__, ts, ch, pitch);
}

static void exec_timecode_quarter(uint16_t ts, uint8_t typ, uint8_t val)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, typ %d, val %d\n", __func__, ts, typ, val);
}

static void exec_song_posision(uint16_t ts, uint8_t pos)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, pos %d\n", __func__, ts, pos);
}

static void exec_song_select(uint16_t ts, uint8_t sel)
{
  /* Do nothing for this application */

  dbg_midiparser("%s : TS %d, sel %d\n", __func__, ts, sel);
}

static void exec_tune_request(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void exec_timing_clock(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void exec_start_current_seq(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void exec_continue_seq(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void exec_stop_seq(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void exec_active_sense(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void exec_system_reset(uint16_t ts)
{
  /* Do nothing for this application */

  dbg_midiparser("%s() : ts %d\n", __func__, ts);
}

static void update_timestamp(struct parser_state_s *state, uint8_t ts)
{
  state->tmstmp = (uint16_t)(((g_state.header & MIDIHDR_TSHIGH_FIELD) << 7) +
                                (ts & MIDI_TSLOW_FIELD));
}

static void msghdlr_noteoff(uint8_t **msg, int *len,
                            struct parser_state_s *state)
{
  uint8_t key;
  uint8_t velocity;

  while ((*len) >= 2 && IS_MIDIDATA(**msg))
    {
      key = **msg;
      (*msg)++;
      velocity = **msg;
      (*msg)++;
      *len -= 2;

      exec_note_off(state->tmstmp, state->chnl, key, velocity);
    }
}

static void msghdlr_noteon(uint8_t **msg, int *len,
                           struct parser_state_s *state)
{
  uint8_t key;
  uint8_t velocity;

  while ((*len) >= 2 && IS_MIDIDATA(**msg))
    {
      key = **msg;
      (*msg)++;
      velocity = **msg;
      (*msg)++;
      *len -= 2;

      if (velocity == 0)
        {
          exec_note_off(state->tmstmp, state->chnl, key, velocity);
        }
      else
        {
          exec_note_on(state->tmstmp, state->chnl, key, velocity);
        }
    }
}

static void msghdlr_aftrtouch(uint8_t **msg, int *len,
                              struct parser_state_s *state)
{
  uint8_t key;
  uint8_t pressure;

  while ((*len) >= 2 && IS_MIDIDATA(**msg))
    {
      key = **msg;
      (*msg)++;
      pressure = **msg;
      (*msg)++;
      *len -= 2;

      exec_poly_key_press(state->tmstmp, state->chnl, key, pressure);
    }
}

static void msghdlr_ctlchange(uint8_t **msg, int *len,
                              struct parser_state_s *state)
{
  uint8_t key;
  uint8_t pressure;

  while ((*len) >= 2 && IS_MIDIDATA(**msg))
    {
      key = **msg;
      (*msg)++;
      pressure = **msg;
      (*msg)++;
      *len -= 2;

      if (key < CONTROLLER_NUM_MAX)
        {
          exec_control_change(state->tmstmp, state->chnl, key,
                              pressure);
        }
      else
        {
          switch (key)
            {
              case CHMODEMSG_ALLSNDOFF:
                if (pressure == 0)
                  {
                    exec_all_sound_off(state->tmstmp, state->chnl);
                  }
                return;

              case CHMODEMSG_RSTALLCTL:
                exec_reset_all_controll(state->tmstmp, state->chnl);
                return;

              case CHMODEMSG_LCLCTL:
                if (pressure == 0)
                  {
                    exec_local_controll(state->tmstmp, state->chnl, 0);
                  }
                else if (pressure == 127)
                  {
                    exec_local_controll(state->tmstmp, state->chnl, 1);
                  }
                return;

              case CHMODEMSG_ALLNOTEOFF:
                if (pressure == 0)
                  {
                    exec_all_notesoff(state->tmstmp, state->chnl);
                  }
                return;

              case CHMODEMSG_OMNIOFF:
                if (pressure == 0)
                  {
                    exec_omni_off(state->tmstmp, state->chnl);
                  }
                return;

              case CHMODEMSG_OMNION:
                if (pressure == 0)
                  {
                    exec_omni_on(state->tmstmp, state->chnl);
                  }
                return;

              case CHMODEMSG_MONOON:
                if (pressure == 0)
                  {
                    exec_mono_mode(state->tmstmp, state->chnl,
                                   pressure);
                  }
                return;

              case CHMODEMSG_POLYON:
                if (pressure == 0)
                  {
                    exec_polymode_on(state->tmstmp, state->chnl);
                  }
                return;
            }
        }
    }
}

static void msghdlr_prgchange(uint8_t **msg, int *len,
                              struct parser_state_s *state)
{
  uint8_t prog;

  while ((*len) >= 1 && IS_MIDIDATA(**msg))
    {
      prog = **msg;
      (*msg)++;
      (*len)--;

      exec_program_change(state->tmstmp, state->chnl, prog);
    }
}

static void msghdlr_chaftertouch(uint8_t **msg, int *len,
                                 struct parser_state_s *state)
{
  uint8_t pressure;

  while ((*len) >= 1 && IS_MIDIDATA(**msg))
    {
      pressure = **msg;
      (*msg)++;
      (*len)--;

      exec_channel_pressure(state->tmstmp, state->chnl, pressure);
    }
}

static void msghdlr_pitchbend(uint8_t **msg, int *len,
                              struct parser_state_s *state)
{
  uint16_t pitch;

  while ((*len) >= 2 && IS_MIDIDATA(**msg))
    {
      pitch = **msg;
      (*msg)++;
      pitch = pitch + ((uint16_t)(**msg) << 7);
      (*msg)++;
      *len -= 2;

      exec_pitch_bend(state->tmstmp, state->chnl, pitch);
    }
}

static void sysexcl_timecode(uint8_t **msg, int *len, uint16_t tmstmp)
{
  uint8_t typ;
  uint8_t val;

  typ = ((**msg) & 0x70) >> 4;
  val = (**msg) % 0x0F;
  (*msg)++;
  (*len)--;

  exec_timecode_quarter(tmstmp, typ, val);
}

static void sysexcl_songpos(uint8_t **msg, int *len, uint16_t tmstmp)
{
  uint16_t pos;

  pos = **msg;
  (*msg)++;
  pos = pos + ((uint16_t)(**msg) << 7);
  (*msg)++;
  *len -= 2;

  exec_song_posision(tmstmp, pos);
}

static void sysexcl_songsel(uint8_t **msg, int *len, uint16_t tmstmp)
{
  uint8_t sel = **msg;

  (*msg)++;
  (*len)--;

  exec_song_select(tmstmp, sel);
}

static void msghdlr_sysexclusive(uint8_t **msg, int *len,
                                 struct parser_state_s *state)
{
  switch (state->msgid)
    {
      case MSGTYPE_SYSEXCLSTART:
        state->state = MIDIPARSE_STATE_EXCLUSIVE;
        return;

      case MSGTYPE_TIMECODE:
        sysexcl_timecode(msg, len, state->tmstmp);
        return;

      case MSGTYPE_SONGPOS:
        sysexcl_songpos(msg, len, state->tmstmp);
        return;

      case MSGTYPE_SONGSEL:
        sysexcl_songsel(msg, len, state->tmstmp);
        return;

      case MSGTYPE_TUNEREQ:
        exec_tune_request(state->tmstmp);
        return;

      case MSGTYPE_SYSEXCLEND:
        state->state = MIDIPARSE_STATE_NORMAL;
        return;

      case MSGTYPE_CLOCK:
        exec_timing_clock(state->tmstmp);
        return;

      case MSGTYPE_START:
        exec_start_current_seq(state->tmstmp);
        return;

      case MSGTYPE_CONTINUE:
        exec_continue_seq(state->tmstmp);
        return;

      case MSGTYPE_STOP:
        exec_stop_seq(state->tmstmp);
        return;

      case MSGTYPE_ACTSENS:
        exec_active_sense(state->tmstmp);
        return;

      case MSGTYPE_SYSRESET:
        exec_system_reset(state->tmstmp);
        return;
    }
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void init_midiparser(void)
{
  g_sndno = 0;
  g_note_on_count = 0;
  memset(&g_state, 0, sizeof(g_state));
}

void parse_midicmd(uint8_t *rcvdata, int len)
{
  int i;

  if (len <= 0)
    {
      /* Zero length.... */

      return;
    }

  g_state.running_hdlr = NULL;
  g_state.msgid = 0;

  /* First byte must be header */

  if (IS_NOTMIDIDATA(*rcvdata))
    {
      g_state.header = *rcvdata;
      rcvdata++;
      len--;

      dbg_midiparser("HEADER : %02x\n", g_state.header);

      while (len)
        {
          dbg_midiparser("Rest len : %d\n", len);

          if (g_state.state == MIDIPARSE_STATE_EXCLUSIVE)
            {
              /* Skip data until END of Exclusive */

              dbg_midiparser("In EXCLUSIVE\n");

              while (len && *rcvdata != MSGTYPE_SYSEXCLEND)
                {
                  if (IS_NOTMIDIDATA(*rcvdata))
                    {
                      update_timestamp(&g_state, *rcvdata);
                    }

                  rcvdata++;
                  len--;
                }

              /* Is the end of Exclusive ? */

              if (len && *rcvdata == MSGTYPE_SYSEXCLEND)
                {
                  g_state.state = MIDIPARSE_STATE_NORMAL;
                  rcvdata++;
                  len--;
                }

              dbg_midiparser("Rest len at end of exclusive: %d\n", len);
            }
          else /* Normal case */
            {
              dbg_midiparser("In NORMAL : %02x\n", *rcvdata);

              if (IS_NOTMIDIDATA(*rcvdata)) /* Should Timestamp */
                {
                  update_timestamp(&g_state, *rcvdata);
                  rcvdata++;
                  len--;

                  dbg_midiparser("In NORMAL midi msg : %02x\n", *rcvdata);

                  if (len && IS_NOTMIDIDATA(*rcvdata)) /* MIDI Message */
                    {
                      g_state.msgid = *rcvdata;
                      rcvdata++;
                      len--;

                      for (i = 0; i < MSGLIST_SIZE; i++)
                        {
                          if (MSGTYPE(g_state.msgid) == g_msghdlr_list[i].msg_type)
                            {
                              g_state.chnl = CHNUM(g_state.msgid);
                              g_state.msgid = g_state.msgid;
                              g_state.running_hdlr = g_msghdlr_list[i].hdlr;

                              dbg_midiparser("Found handlr : %02x\n", g_state.msgid);

                              g_msghdlr_list[i].hdlr(&rcvdata, &len, &g_state);
                              break;
                            }
                        }

                      if (i == MSGLIST_SIZE)
                        {
                          /* Irregular case... */

                          dbg_midiparser("Not found handlr : %02x\n", g_state.msgid);

                          return;
                        }
                    }
                  else if (len) /* Running mode case */
                    {
                      dbg_midiparser("Running mode\n");
                      if (g_state.running_hdlr == NULL)
                        {
                          /* Irregular case... */

                          dbg_midiparser("Not found handlr in running\n");

                          return;
                        }

                      g_state.state = MIDIPARSE_STATE_RUNNING;
                      g_state.running_hdlr(&rcvdata, &len, &g_state);
                      g_state.state = MIDIPARSE_STATE_NORMAL;
                    }
                }
              else
                {
                  /* Irregular case... */

                  dbg_midiparser("Irregular....\n");
                  return;
                }
            } /* else Normal case */
        } /* while (len) */
    } /* if (IS_NOTMIDIDATA(*rcvdata)) */

  return;
}
