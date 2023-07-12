#ifndef __MIDI_PLAY_H
#define __MIDI_PLAY_H

#include <stdint.h>
#include <stdbool.h>

void play_midi(void);
int send_code(bool on_xoff, uint8_t code, uint8_t vel);
int send_sound(unsigned char snd);

#endif  /* __MIDI_PLAY_H */
