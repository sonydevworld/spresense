
Usage of audio_through
===========================

Usage
---------------------------

Select options in below.

- [CXD56xx Configuration]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio manager] <= Y
- [Memory manager] <= Y
    [Memory Utilities]
      [Memory manager] <= Y
      [Message] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio through example] <= Y

Or use audio_through default configuration

$ ./tools/config.py examples/audio_through

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

Execute
--------------------------

Type 'audio_through' on nsh.
nsh>audio_through

Combinations of the following data paths are executed.

 +---+-------+---------+
 |#  | IN    | OUT     |
 +---+-------+---------+
 |1  | Mic   | Speaker |
 +---+-------+---------+
 |2  | Mic   | I2S     |
 +---+-------+---------+
 |3  | I2S   | I2S     |
 +---+-------+---------+
 |4  | I2S   | Speaker |
 +---+-------+---------+
 |5  | Mic   | I2S     |
 |   | I2s   | Speaker |
 +---+-------+---------+

