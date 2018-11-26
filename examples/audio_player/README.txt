
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
 - Default path
    worker binary : /mnt/sd0/BIN
    play list     : /mnt/sd0/PLAYLIST
    contents      : /mnt/sd0/AUDIO

As a sample of Playlist, there is TRACK_DB.CSV in the PLAYLIST folder.
Also, as a sample of contents, there is Sound.mp 3 in the AUDIO folder.
Sound.mp3 is a sampling frequency of 48000, bit length 16, codec MP3 file.

Execute
--------------------------

Type 'player' on nsh.
nsh>player

The first content of playlist will be played for 10 seconds.

If you do not select playlist manager as option,
"Sound.mp3" file will be played.
