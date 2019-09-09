
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


NOTE

  Build preprocess DSP and deploy
  --------------------------
  
  If you would like to use preprocess DSP.
  Set config "EXAMPLES_AUDIO_RECORDER_USEPREPROC" = "y" before do make.

  When EXAMPLES_AUDIO_RECORDER_USEPREPROC is set and build this example,
  DSP binary for preprocess will out at "worker/src/PREPROC".
  Please place PREPROC binary file to SD card(/mnt/sd0/BIN).

  The codes of DSP is placed on "worker/userproc/src(or include)".
  You can edit them and it will be built with framework code in sdk(*)
  and embedded to DSP binary.
  (*)The framework codes are at "sdk/module/audio/components/usercustom/dsp_framework".


Execute
--------------------------

Type 'audio_recorder' on nsh.
nsh>audio_recorder

Audio from the microphone is recorded in the WAV file for 10 seconds.


