================
   README.txt
================

Playlist class provides playlist function for audio player.
Basic usage is written in this document.

_/_/ Usage

(1) First, You have to prepare "Playlist-file" which written in
    provided format as below. File name has no restriction.

    ----------------------
    filename,author,album,ch-num,bit-length,sampling-rate,codec,flag 

    filename      : Audio file name. Can spcify with directory.
    author        : Author name. You can filter by this.
    album         : Album name. You can filter by this.
    ch-num        : 1 or 2.
    bit-length    : 16 or 24.
    sampling-rate : 8000, 16000, 24000, 32000, 44100, 48000, 64000, 88200, 96000 or 192000 
    flag          : Don't care
    ----------------------

    For example,

    <MyPlaylistFile.csv>

    track-001.mp3,Anyone,1stAlbum,2,16,44100,mp3
    track-002.mp3,Anyone,1stAlbum,2,16,44100,mp3
    track-003.mp3,Anyone,1stAlbum,2,16,44100,mp3
    music-001.aac,EveryOne,LastAlbum,2,16,48000,aac
    music-002.aac,EveryOne,LastAlbum,2,16,48000,aac
    music-003.aac,EveryOne,LastAlbum,2,16,48000,aac
    mydir/sound-001.aac,EveryOne,LastAlbum,2,16,48000,aac
    mydir/sound-002.aac,EveryOne,LastAlbum,2,16,48000,aac
    mydir/sound-003.aac,EveryOne,LastAlbum,2,16,48000,aac

(2) Put "Playlist-file" in any path.

    path/to/playlist/MyPlaylistFile.csv

(3) You should bulit in playlist class in your application.

(4) Set playlist file name and path.

    - set "Playlist-file" name by Constractor

        Playlist::init("MyPlaylistFile.csv");

    - set path to "Playlist-file" by init() function.

        Playlist::init("path/to/playlist/");

(5) Get track information.

    - Next track

        Playlist::getNextTrack(&trak_info);

    - Previous track

        Playlist::getPrevTrack(&trak_info);

_/_/_/ Functions

  Fucntions of Playlist Class are written in Playlist.h 

