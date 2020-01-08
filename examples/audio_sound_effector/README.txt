
Usage of audio_sound_effector
===========================

This sample application shows how to output to speaker from microphone input
audio data with low latency.

In this example, a simple RC filter will be applied to the microphone sound
and output to the speakers in real time.

Usage
---------------------------

Select options in below.

- [CXD56xx Configuration]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Mic Front End] <= Y
      [Output Mixer] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio capture (use object if)example] <= Y

Or use audio_sound_effector default configuration

$ ./tools/config.py examples/audio_sound_effector

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

Execute
--------------------------

Type 'audio_sound_effector' on nsh.
nsh>audio_sound_effector

