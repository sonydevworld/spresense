#ifndef __EXAMPLES_BLEMIDI_PLAYER_MIDI_PARSER_H
#define __EXAMPLES_BLEMIDI_PLAYER_MIDI_PARSER_H

#include <stdint.h>

void init_midiparser(void);
void parse_midicmd(uint8_t *rcvdata, int len);

#endif /* __EXAMPLES_BLEMIDI_PLAYER_MIDI_PARSER_H */
