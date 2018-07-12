
Usage of voice_command
===========================

Usage
---------------------------

Select options in below.

Å°SDK
- [CXD56xx Configuration Options]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Voice Command] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Voice command example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MFESRC.espk', 'VADWUW.espk'
in directory proprietary/*/bin.
Install 'MFESRC.espk' and 'VADWUW.espk' to system.

Execute
--------------------------

Type 'voice_command' on nsh.
nsh>voice_command

Run the following process for 30 seconds.
 - Microphone input is output to I2S.
 - I2S input is output to Speaker.
 - When recognizing the voice input in the microphone,
   it displays "VAD RISE->".
 - When the voice input disappears,it displays "->VAD FALL".
 - When recognizing the keyword "Hello Sony" input in the microphone,
   it displays "*** Found Command ***".

