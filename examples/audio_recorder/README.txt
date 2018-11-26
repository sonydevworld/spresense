
Usage of audio_recorder
===========================

Usage
---------------------------

Select options in below.

- [CXD56xx Configuration]
    [SDIO SD Card] <= Y
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio Recorder] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio recorder example] <= Y

Or use audio_recorder default configuration

$ ./tools/config.py examples/audio_recorder

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MP3ENC', 'SRC'
in directory sdk/modules/audio/dsp.
Store worker binary in the path specified by option.
 - Default path
    worker binary : /mnt/sd0/BIN

Recording data is written to the optional path.
 - Default path
    contents      : /mnt/sd0/REC

Execute
--------------------------

Type 'recorder' on nsh.
nsh>recorder

Audio from the microphone is recorded in the WAV file for 10 seconds.


