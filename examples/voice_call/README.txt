
Usage of voice_call
===========================

Usage
---------------------------

Select options in below.

Å°SDK
- [CXD56xx Configuration Options]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Voice Call] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Voice call example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MFESRC.espk' in directory proprietary/*/bin.
Install 'MFESRC.espk' to system.

Execute
--------------------------

Type 'voice_call' on nsh.
nsh>voice_call

Run the following process for 30 seconds.
 - Microphone input is output to I2S.
 - I2S input is output to Speaker.

