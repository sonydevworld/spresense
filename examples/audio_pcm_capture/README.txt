
Usage of audio_pcm_capture
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
    [Audio pcm_capture example] <= Y

Or use audio_pcm_capture default configuration

$ ./tools/config.py examples/audio_pcm_capture

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

Execute
--------------------------

Type 'audio_pcm_capture' on nsh.
nsh> audio_pcm_capture

PCM capture with the following settings.

Sampling rate 48KHz
Channel number STEREO
Bit length 16 bits
Recording time 10 (seconds)

Change settings with arguments.

Example)
Change capture time to 20 seconds.

nsh> audio_pcm_capture 20

Example)
192k, monaural, 30 seconds, capture.

nsh> audio_pcm_capture 192k 1ch 30

Please refer to the help about setting items.

nsh> audio_pcm_capture help

The sample code outputs capture data.
Please modify according to the application.

