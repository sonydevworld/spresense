
Usage of audio_through
===========================

Usage
---------------------------

Select options in below.

Å°SDK
- [CXD56xx Configuration Options]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio manager] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio through example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
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

