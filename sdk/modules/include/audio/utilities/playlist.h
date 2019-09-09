/****************************************************************************
 * modules/include/audio/utilities/playlist.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_PLAYLIST_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_PLAYLIST_H

#include "memutils/s_stl/queue.h"
#include "audio/audio_high_level_api.h"

/* Track information */

struct Track
{
  /*! \brief Track title (file name) */

  char title[64];

  /*! \brief Author name */

  char author[64];

  /*! \brief Album name */

  char album[64];

  /*! \brief Number of channels */

  uint8_t   channel_number;

  /*! \brief Bit length of the track */

  uint8_t   bit_length;

  /*! \brief Sampling rate of the track */

  uint32_t  sampling_rate;

  /*! \brief Codec type of the track */

  uint8_t   codec_type;
};

/* Playlist class definition */

class Playlist
{
public:
  enum PlayMode
  {
    /*! \brief Normal play mode. */

    PlayModeNormal = 0,

    /*! \brief Shuffle play mode. */

    PlayModeShuffle,

    NumOfPlayMode,
  };

  enum RepeatMode
  {
    /*! \brief Play all track at once. */

    RepeatModeOff = 0,

    /*! \brief Play all track repeat. */

    RepeatModeOn,

    NumOfRepeatMode,
  };

  enum ListType
  {
    /*! \brief All track (which is written in playlist file) list. */

    ListTypeAllTrack = 0,

    /*! \brief Artist categorized track list. */

    ListTypeArtist,

    /*! \brief Album categorized track list. */

    ListTypeAlbum,

    /*! \brief User defined track list. */

    ListTypeUser,

    NumOfListType,
  };

  /**
   * @brief Playlist Constructor
   *
   * @param[in] file_name: Playlist file name.
   */
   
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

  /**
   * @brief Playlist Destructor
   */

  ~Playlist()
  {
    close();
  }

  /**
   * @brief Init playlist
   *
   * @param[in] playlist_path: Path to playlist file
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool init(const char *playlist_path);

  /**
   * @brief Set play mode
   *
   * @param[in] play_mode: PlayModeNormal, PlayModeShuffle
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool setPlayMode(PlayMode play_mode);

  /**
   * @brief Set repeat mode
   *
   * @param[in] repeat_mode: RepeatModeOff, RepeatModeOn
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool setRepeatMode(RepeatMode repeat_mode);

  /**
   * @brief Select playlist to play
   * @note If type is ListTypeAllTrack, key_str is not cared.
   *
   * @param[in] type:    ListTypeAllTrack, ListTypeArtist, ListTypeAlbum, ListTypeUser
   * @param[in] key_str: Key string to select playlist. Author name, album name, etc...
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool select(ListType type, FAR const char *key_str);

  /**
   * @brief Update playlist
   * @details Create or update playlist. Target playlist should be selected by parameters.
   * @note If type is ListTypeAllTrack, key_str is not cared.
   *
   * @param[in] type:    ListTypeAllTrack, ListTypeArtist, ListTypeAlbum, ListTypeUser
   * @param[in] key_str: Key string to filter playlist. Author name, album name, etc...
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool updatePlaylist(ListType type, FAR const char *key_str);

  /**
   * @brief Add track to playlist
   * @note Target playlist is always user defined playlist(ListTypeUser).
   *
   * @param[in] key_str:  Key string to filter playlist. Author name, album name, etc...
   * @param[in] track_no: Track number of which would like to add to playlist.
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool addTrack(FAR const char *key_str, int track_no);

  /**
   * @brief Remove track from playlist
   * @note Target playlist is always user defined playlist(ListTypeUser).
   *
   * @param[in] key_str:  Key string to filter playlist. Author name, album name, etc...
   * @param[in] remove_pos: Track number of which would like to remove from playlist.
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool removeTrack(FAR const char *key_str, uint32_t remove_pos);

  /**
   * @brief Update track database
   * @details Create or update all track playlist by tracks in path/to/.
   *
   * @param[in] audiofile_root_path: Path to audio data file.
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool updateTrackDb(const char *audiofile_root_path);

  /**
   * @brief Delete all playlist
   * @note Delete playlist which is created internally.
   *       track database(***.csv) will not be deleted.
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool deleteAll(void);

  /**
   * @brief Delete playlist
   *
   * @param[in] type:    ListTypeAllTrack, ListTypeArtist, ListTypeAlbum, ListTypeUser
   * @param[in] key_str: Key string to filter playlist. Author name, album name, etc...
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool deleteOne(ListType type, FAR const char *key_str);

  /**
   * @brief Get next track
   * @details Get next track in playlist which is selected by select().
   *
   * @param[out] track: Track information
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool getNextTrack(FAR Track *track);

  /**
   * @brief Get previous track
   * @details Get previous track in playlist which is selected by select().
   *
   * @param[out] track: Track information
   *
   * @retval     true  : success
   * @retval     false : failure
   */

  bool getPrevTrack(FAR Track *track);

  /**
   * @brief Restart playlist 
   * @details Restart from top of playlist.
   *
   * @retval     true  : success
   * @retval     false : failure
   */

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

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_PLAYLIST_H */

