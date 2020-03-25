
Usage of audio_oscillator
===========================

In this example, the oscillator DSP creates up to eight different frequency
waveforms. Next, the post-processing DSP synthesizes this waveform and
outputs it to the speaker.
The oscillator DSP can also apply an envelope to each waveform.

Usage
---------------------------

Select options in below.

- [CXD56xx Configuration]
    [SDIO SD Card] <= Y
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio Synthesizer] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio oscillator example] <= Y

Or use audio_oscillator default configuration

$ ./tools/config.py examples/audio_oscillator

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'OSCPROC', 'POSTPROC'
in directory worker_postproc, worker_postproc.
Store worker binary in the path specified by option.
 - Default path
    worker binary : /mnt/sd0/BIN

Execute
--------------------------

Type 'audio_oscillator' on nsh.
nsh>audio_oscillator

