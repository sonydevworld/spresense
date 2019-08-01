
Usage of audio_player
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
        [Playlist manager] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio player example] <= Y

Or use audio_player default configuration

$ ./tools/config.py examples/audio_player

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MP3DEC', 'WAVDEC'
in directory sdk/modules/audio/dsp.
Store worker binary, playlist and play contents in the path specified by option.

The default storage location in example is microSD card(formatted with FAT32).
Create 'BIN', 'PLAYLIST', 'AUDIO' directories on Top of microSD card, 

Store worker binary in 'BIN' directory.
Store playlist in 'PLAYSLIT' directory.
Store contents to be played back in 'AUDIO' directory.

Nuttx and windows have the following configuration.

 - Nuttx configuration
    /mnt/sd0/BIN
    /mnt/sd0/PLAYLIST
    /mnt/sd0/AUDIO

 - Windows configuration(Case where microSD card is mounted on 'D' drive)
    D:\BIN
    D:\PLAYLIST
    D:\AUDIO


As a sample of Playlist, there is TRACK_DB.CSV in examples/audio_player/PLAYLIST.
Also, as a sample of contents, there is Sound.mp3 in examples/audio_player/AUDIO.
(Sound.mp3 is a sampling frequency of 48000, bit length 16, codec MP3 file.)

The file list when using these samples is as follows.

 - Nuttx file list
    /mnt/sd0/BIN/MP3DEC
    /mnt/sd0/BIN/WAVDEC
    /mnt/sd0/PLAYLIST/TRACK_DB.CSV
    /mnt/sd0/AUDIO/Sound.mp3

 - Windows file list(Case where microSD card is mounted on 'D' drive)
    D:\BIN\MP3DEC
    D:\BIN\WAVDEC
    D:\PLAYLIST\TRACK_DB.CSV
    D:\AUDIO\Sound.mp3


Execute
--------------------------

Type 'audio_player' on nsh.
nsh>audio_player

The first content of playlist will be played for 10 seconds.

If you do not select playlist manager as option,
"Sound.mp3" file will be played.
