/****************************************************************************
 * modules/audio/playlist/playlist.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef MODULES_AUDIO_PLAYLIST_PLAYLIST_H
#define MODULES_AUDIO_PLAYLIST_PLAYLIST_H

#include "memutils/s_stl/queue.h"
#include "audio/audio_high_level_api.h"

struct Track
{
  bool is_played;
  char title[64];
  char author[64];
  char album[64];

  uint8_t   channel_number;  /* Channel number. */
  uint8_t   bit_length;      /* Bit length.     */
  uint32_t  sampling_rate;   /* Sampling rate.  */
  uint8_t   codec_type;      /* Codec type.     */
};

class Playlist
{
public:
  enum PlayMode
  {
    PlayModeNormal = 0, /* Normal play mode. */
    PlayModeShuffle,    /* Shuffle play mode. */
    NumOfPlayMode,
  };

  enum RepeatMode
  {
    RepeatModeOff = 0, /* Play all track at once. */
    RepeatModeOn,      /* Play all track repeat. */
    NumOfRepeatMode,
  };

  enum ListType
  {
    ListTypeAllTrack = 0, /* All track list. */
    ListTypeArtist,       /* Artist categolized track list. */
    ListTypeAlbum,        /* Album categolized track list. */
    ListTypeUser,         /* User defined track list. */
    NumOfListType,
  };

  Playlist(FAR const char* file_name) :
    m_play_mode(PlayModeNormal),
    m_repeat_mode(RepeatModeOff),
    m_list_type(ListTypeAllTrack),
    m_play_idx(-1),
    m_track_db_fp(NULL)
  {
    strncpy(m_track_db_file_name, file_name, sizeof(m_track_db_file_name));
    memset(m_playlist_path, 0, sizeof(m_playlist_path));
  }

  ~Playlist()
  {
    close();
  }

  bool init(const char *playlist_path);
  bool setPlayMode(PlayMode play_mode);
  bool setRepeatMode(RepeatMode repeat_mode);
  bool select(ListType type, FAR const char *key_str);
  bool updatePlaylist(ListType type, FAR const char *key_str);
  bool addTrack(FAR const char *key_str, int track_no);
  bool removeTrack(FAR const char *key_str, uint32_t remove_pos);
  bool updateTrackDb(const char *audiofile_root_path);
  bool deleteAll(void);
  bool deleteOne(ListType type, FAR const char *key_str);
  bool getNextTrack(FAR Track *track);
  bool getPrevTrack(FAR Track *track);
  bool restart(void);

private:
  bool open(FAR const char *mode);
  bool close(void);
  bool readLine(FAR char *line, uint32_t line_size);
  bool isTargetTrack(ListType       type,
                     FAR const char *key_str,
                     FAR Track      *track);
  bool loadAliasList(void);
  bool shuffleList(int idx_top);
  bool parseTrackInfo(FAR Track *track, FAR char *line, uint32_t line_size);
  bool getFileName(ListType       type,
                   FAR const char *key_str,
                   FAR char       *file_name,
                   uint8_t        max_length);

  static const int  FileNameMaxLength = 128;
  static const int  LineMaxLength     = 256;

  char       m_playlist_path[FileNameMaxLength];
  PlayMode   m_play_mode;
  RepeatMode m_repeat_mode;
  ListType   m_list_type;
  int        m_play_idx;
  char       m_list_key[64];
  char       m_line_buffer[LineMaxLength];
  char       m_track_db_file_name[FileNameMaxLength];
  FAR FILE   *m_track_db_fp;

  s_std::Queue<uint32_t, 256> m_alias_list;
};

#endif /* MODULES_AUDIO_PLAYLIST_PLAYLIST_H */

