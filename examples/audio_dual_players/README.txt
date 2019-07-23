
Usage of audio_dual_players
===========================

Usage
---------------------------

Select options in below.

- [CXD56xx Configuration]
    [SDIO SD Card] <= Y
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio Player] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio dual players example] <= Y

Or use audio_dual_players default configuration

$ ./tools/config.py examples/audio_dual_players


Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MP3DEC', 'WAVDEC'
in directory sdk/modules/audio/dsp.
Store worker binary and play contents in the path specified by option.

The default storage location in example is microSD card(formatted with FAT32).
Create 'BIN', 'AUDIO' directories on Top of microSD card,

Store worker binary in 'BIN' directory.
Store contents to be played back in 'AUDIO' directory.

Nuttx and windows have the following configuration.

 - Nuttx configuration
    /mnt/sd0/BIN
    /mnt/sd0/AUDIO

 - Windows configuration(Case where microSD card is mounted on 'D' drive)
    D:\BIN
    D:\AUDIO

As a sample of contents, there is Sound0.mp3 iand Sound1.mp3 in examples/audio_player/AUDIO.
(sampling frequency of 48000, bit length 16, codec MP3)

The file list when using these samples is as follows.

 - Nuttx file list
    /mnt/sd0/BIN/MP3DEC
    /mnt/sd0/BIN/WAVDEC
    /mnt/sd0/AUDIO/Sound0.mp3
    /mnt/sd0/AUDIO/Sound1.mp3

 - Windows file list(Case where microSD card is mounted on 'D' drive)
    D:\BIN\MP3DEC
    D:\BIN\WAVDEC
    D:\AUDIO\Sound0.mp3
    D:\AUDIO\Sound1.mp3


Execute
--------------------------

Type 'audio_dual_players' on nsh.
nsh>audio_dual_players

The contents of Sound0.mp3 and Sound1.mp3 will be played simultaneously.

The playback time (seconds) can be specified in the command argument.

Ex)
10 seconds playback.

nsh>audio_dual_players 10

Note: Due to memory limitations, there are the following restrictions.
  Content 1 up to sampling rate 192 kHz, 2 ch, 24 bit.
  Content 2 up to sampling rate 192 kHz, 2 ch, 16 bit.
  If the sampling rate of one content is 192 kHz,
  the other sampling rate must also be 192 kHz.
