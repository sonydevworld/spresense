
Usage of audio_recorder_objif
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
    [Audio recorder_objif example] <= Y

Or use audio_recorder_objif default configuration

$ ./tools/config.py examples/audio_recorder_objif

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
  Set config "EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC" = "y" before do make.

  When EXAMPLES_AUDIO_RECORDER_OBJIF_USEPREPROC is set and build this example,
  DSP binary for preprocess will out at "worker/src/PREPROC".
  Please place PREPROC binary file to SD card(/mnt/sd0/BIN).

  The codes of DSP is placed on "worker/userproc/src(or include)".
  You can edit them and it will be built with framework code in sdk(*)
  and embedded to DSP binary.
  (*)The framework codes are at "sdk/module/audio/components/usercustom/dsp_framework".


Execute
--------------------------

Type 'audio_recorder_objif' on nsh.
nsh>audio_recorder_objif

The following settings will be recorded.

Codec type LPCM
Sampling rate 48KHz
Channel number STEREO
Bit length 16 bits
Recording time 10 (seconds)

Change settings with arguments.

Example)
Change the codec to MP3.

nsh> audio_recorder_objif MP3.

Example)
Change the recording time to 20 seconds.

nsh> audio_recorder_objif 20

Example)
LPCM, 192k, monaural, 30 seconds, recording.

nsh> audio_recorder_objif pcm 192 1ch 30

Please refer to the help about setting items.

nsh> audio_recorder_objif Help

Note: Some combinations of settings can not be recorded.

Thank you.

