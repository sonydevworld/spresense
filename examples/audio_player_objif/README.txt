
Usage of audio_player_objif
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
    [Audio player (use object if)example] <= Y

Or use audio_player_objif default configuration

$ ./tools/config.py examples/audio_player_objif

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

Execute
--------------------------

Type 'audio_player_objif' on nsh.
nsh>audio_player_objif

The first content of playlist will be played for 10 seconds.

