
Usage of audio_recorder
===========================

Usage
---------------------------

Select options in below.

Å°SDK
- [CXD56xx Configuration Options]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio Recorder] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio recorder example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MP3ENC', 'OPUSENC', 'SRC'
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


