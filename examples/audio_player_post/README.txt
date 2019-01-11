
Usage of audio_player_post
===========================

Usage
---------------------------

Select options in below.

SDK
- [CXD56xx Configuration Options]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio Player] <= Y
      [Playlist manager] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio player example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'AACDEC', 'MP3DEC', 'OPUSDEC'
in directory sdk/modules/audio/dsp.
Store worker binary, playlist and play contents in the path specified by option.
 - Default path
    worker binary : /mnt/sd0/bin
    play list     : /mnt/sd0/playlist
    contents      : /mnt/sd0/audio

Execute
--------------------------

Type 'player' on nsh.
nsh>player

The first content of playlist will be played for 10 seconds.

