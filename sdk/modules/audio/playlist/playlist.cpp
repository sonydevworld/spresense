/****************************************************************************
 * modules/audio/playlist/playlist.cpp
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

#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>

#include <audio/utilities/playlist.h>

/*--------------------------------------------------------------------------*/
bool Playlist::init(const char *playlist_path)
{
  /* Check argument */

  if (playlist_path == NULL)
    {
      return false;
    }

  /* Set playlist file path */

  snprintf(m_playlist_path, sizeof(m_playlist_path), "%s", playlist_path);

  /* Open track database. */

  this->open("r");

  /* Create alias list. */

  this->updatePlaylist(ListTypeAllTrack, "");

  /* Load alias list. */

  if (!this->loadAliasList())
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::open(FAR const char *mode)
{
  char absolute_path[FileNameMaxLength];

  /* Check argument */

  if (mode == NULL)
    {
      return false;
    }

  snprintf(absolute_path,
           sizeof(absolute_path),
           "%s/%s", m_playlist_path,
           this->m_track_db_file_name);
  this->m_track_db_fp = fopen(absolute_path, mode);

  if (this->m_track_db_fp == NULL)
    {
      printf("Track db(playlist) %s open error. check paths and files!\n",
             absolute_path);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::close(void)
{
  if (this->m_track_db_fp != NULL)
  {
    if (fclose(this->m_track_db_fp) != 0)
      {
        return false;
      }
  }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::setPlayMode(PlayMode play_mode)
{
  _info("Set play mode [%d]\n", play_mode);

  this->m_play_mode = play_mode;
  this->m_play_idx = -1;

  if (play_mode == PlayModeShuffle)
    {
      this->shuffleList(0);
    }
  else
    {
      if (!this->loadAliasList())
        {
          return false;
        }
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::setRepeatMode(RepeatMode repeat_mode)
{
  _info("Set repeat mode [%d]\n", repeat_mode);

  this->m_repeat_mode = repeat_mode;
  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::select(ListType type, FAR const char *key_str)
{
  /* Check argument */

  if (key_str == NULL)
    {
      return false;
    }

  this->m_list_type = type;
  strncpy(this->m_list_key, key_str, sizeof(this->m_list_key));

  /* Load alist list from file. */

  if (!this->loadAliasList())
    {
      return false;
    }

  /* If shuffle mode, shuffle a list. */

  if (this->m_play_mode == PlayModeShuffle)
    {
      this->shuffleList(0);
    }

  return true;

}

/*--------------------------------------------------------------------------*/
bool Playlist::getNextTrack(FAR Track *track)
{
  /* Check argument */

  if (track == NULL)
    {
      return false;
    }

  if (this->m_alias_list.empty())
    {
      _err("no playlist.\n");
      return false;
    }

  /* Get track info, according to active list. */

  if (this->m_play_idx >= (this->m_alias_list.size() - 1))
    {
      if (this->m_repeat_mode == RepeatModeOn)
        {
          this->m_play_idx = -1;

          if (this->m_play_mode == PlayModeShuffle)
            {
              this->shuffleList(0);
            }
        }
      else
        {
          return false;
        }
    }

  /* Clear EOF indicator. (Calling fseek() dows not clear them.) */

  clearerr(this->m_track_db_fp);

  /* Increment index. */

  this->m_play_idx++;

  /* Move a file pointer of track database to the head of next track. */

  int seek_rst = fseek(this->m_track_db_fp,
                       this->m_alias_list.at(this->m_play_idx),
                       SEEK_SET);
  if (seek_rst != 0)
    {
      this->m_play_idx--;
      return false;
    }


  /* Get track info. */

  char line[LineMaxLength] = { '\0' };
  bool ret = this->readLine(line, sizeof(line));
  if (ret != true)
    {
      this->m_play_idx--;
      return false;
    }

  return this->parseTrackInfo(track, line, sizeof(line));
}

/*--------------------------------------------------------------------------*/
bool Playlist::getPrevTrack(Track *track)
{
  /* Check argument */

  if (track == NULL)
    {
      return false;
    }

  if (this->m_alias_list.empty())
    {
      _err("no playlist.\n");
      return false;
    }

  /* Get track info, according to active list. */

  if (this->m_play_idx <= 0)
    {
      if (this->m_repeat_mode == RepeatModeOn)
        {
          this->m_play_idx = this->m_alias_list.size();

          if (this->m_play_mode == PlayModeShuffle)
            {
              this->shuffleList(0);
            }
        }
      else
        {
          return false;
        }
    }

  /* Clear EOF indicator. (Calling fseek() dows not clear them.) */

  clearerr(this->m_track_db_fp);

  /* Decrement index. */

  this->m_play_idx--;

  /* Move a file pointer of track database to the head of next track. */

  int seek_rst = fseek(this->m_track_db_fp,
                       this->m_alias_list.at(this->m_play_idx),
                       SEEK_SET);
  if (seek_rst != 0)
    {
      this->m_play_idx++;
      return false;
    }

  /* Get track info. */

  char line[LineMaxLength] =
    {
      '\0'
    };
  bool ret = this->readLine(line, sizeof(line));
  if (ret != true)
    {
      this->m_play_idx++;
      return false;
    }

  return this->parseTrackInfo(track, line, sizeof(line));
}

/*--------------------------------------------------------------------------*/
bool Playlist::restart(void)
{
  _info("Restart playlist\n");

  this->m_play_idx = -1;

  if (this->m_play_mode == PlayModeShuffle)
    {
      this->shuffleList(0);
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::updatePlaylist(ListType type, FAR const char *key_str)
{
  /* Check argument */

  if (key_str == NULL)
    {
      return false;
    }

  if (type == ListTypeUser)
    {
      _err("User define list cannnot not be updated automatically.\n");
      return true;
    }

  /* Open list corresponding to type. */

  char file_name[FileNameMaxLength];
  this->getFileName(type, key_str, file_name, sizeof(file_name));
  FAR FILE *list_fp = fopen(static_cast<FAR const char*>(file_name), "w");
  if (list_fp == NULL)
    {
      printf("%s cannot opened.\n", file_name);
      return false;
    }

  /* Move file pointer to top of file. */

  if (fseek(this->m_track_db_fp, 0, SEEK_SET) != 0)
    {
      printf("Cannot move file pointer. file_name=%s\n", file_name);
      fclose(list_fp);
      return false;
    }

  while (true)
    {
      /* Data for insertion. */

      fpos_t fp_offset = 0;
      fgetpos(this->m_track_db_fp, &fp_offset);

      char line[LineMaxLength] =
        {
          '\0'
        };
      bool ret = this->readLine(line, sizeof(line));
      if (ret != true)
        {
          break;
        }

      Track track;
      this->parseTrackInfo(&track, line, sizeof(line));

      /* Check, and write to alias list. */

      if (this->isTargetTrack(type, key_str, &track))
        {
          size_t wsize = fwrite(&fp_offset, sizeof(fp_offset), 1, list_fp);
          if (wsize != 1)
            {
              printf("File write error. [%d]\n", wsize);
            }
        }
    }

  /* Close list. */

  fclose(list_fp);

  /* Move file pointer back to top of file. */

  fseek(this->m_track_db_fp, 0, SEEK_SET);

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::addTrack(FAR const char *key_str, int track_no)
{
  _info("Add track to [%s, %d]\n", key_str, track_no);

  char     file_name[FileNameMaxLength] =
    {
      '\0'
    };
  FAR FILE *fp;
  uint32_t data = 0;
  uint32_t seek_offset = sizeof(data) * track_no;
  int      read_size = 0;

  /* Check argument */

  if (key_str == NULL)
    {
      return false;
    }

  /* Open AllList and read data. */

  this->getFileName(ListTypeAllTrack, "", file_name, sizeof(file_name));
  fp = fopen(file_name, "r");
  if (fp == NULL)
    {
      return false;
    }

  struct stat file_stat =
    {
      0
    };
  if (stat(file_name, &file_stat) != 0)
    {
      fclose(fp);
      return false;
    }

  if (static_cast<uint32_t>(file_stat.st_size) <= seek_offset)
    {
      _err("Track no %d is not exist.\n", track_no);
      fclose(fp);
      return false;
    }

  fseek(fp, seek_offset, SEEK_SET);

  read_size = fread(&data, 1, sizeof(data), fp);

  fclose(fp);

  /* Open UserList and write data. */

  if (read_size == sizeof(data))
    {
      this->getFileName(ListTypeUser, key_str, file_name, sizeof(file_name));
      fp = fopen(file_name, "a");
      if (fp == NULL)
        {
          return false;
        }

      fwrite(&data, sizeof(data), 1, fp);
      fclose(fp);
    }
  else
    {
      _warn("designated track is not exit [%s]\n", key_str);
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::removeTrack(FAR const char *key_str, uint32_t remove_pos)
{
  _info("remove track from [%s : pos %ld]\n", key_str, remove_pos);

  /* Check argument */

  if (key_str == NULL)
    {
      return false;
    }

  char file_name_org[FileNameMaxLength];
  this->getFileName(ListTypeUser,
                    key_str,
                    file_name_org,
                    sizeof(file_name_org));
  FAR FILE *fp_org = fopen(file_name_org, "r");
  if (fp_org == NULL)
    {
      return false;
    }

  char file_name_tmp[FileNameMaxLength];
  snprintf(file_name_tmp, sizeof(file_name_tmp), "%s_tmp", file_name_org);
  FAR FILE *fp_tmp = fopen(file_name_tmp, "w");
  if (fp_tmp == NULL)
    {
      fclose(fp_org);
      return false;
    }

  /* Copy track from original to tmp except to be removed. */

  for (uint32_t idx = 0; ; idx++)
    {
      uint32_t data;
      fread(&data, 1, sizeof(data), fp_org);

      if (feof(fp_org) != 0)
        {
          break;
        }

      if (idx != remove_pos)
        {
          fwrite(&data, sizeof(data), 1, fp_tmp);
        }
    }

  fclose(fp_org);
  fclose(fp_tmp);

  /* Delete original and rename tmp to original. */

  if (unlink(file_name_org) != 0)
    {
      printf("Cannot delete file. %s\n", file_name_org);
    }

  if (rename(file_name_tmp, file_name_org) != 0)
    {
      printf("Cannot rename file. %s -> %s\n", file_name_tmp, file_name_org);
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::updateTrackDb(const char *audiofile_root_path)
{
  /* Check argument */

  if (audiofile_root_path == NULL)
    {
      return false;
    }

  /* Reopen track database with write mode. */

  this->close();
  this->open("w");

  FAR DIR *dir_descriptor = opendir(audiofile_root_path);
  if (dir_descriptor == NULL)
    {
      printf("Cannot open folder.\n");
    }
  else
    {
      while (true)
        {
          FAR struct dirent *dir_ent = readdir(dir_descriptor);
          if (dir_ent == NULL)
            {
              printf("track database is created.\n");
              break;
            }

          if (DTYPE_FILE == dir_ent->d_type)
            {
              /* A parameter which is shown below is provisional value.
               * [artist][album][ch num][bit length][sampling freq]
               * Because, to know there value, it need to analys file.
               * But Application cannnot do it. *SDK has, but no API.
               * If provisional value set, mp3 will be played clearly,
               * but playing a wav will be occur noise.
               */

              char line[LineMaxLength];

              /* Make terminate of string to use strncpy(). */

              dir_ent->d_name[sizeof(dir_ent->d_name) - 1] = '\0';

              char fname[FileNameMaxLength] =
                {
                  '\0'
                };
              memset(fname, 0, sizeof(fname));
              strncpy(fname, dir_ent->d_name, sizeof(fname) - 1);

              FAR char *codec;
              codec = strtok(dir_ent->d_name, ".");
              codec = strtok(NULL, ".");

              if (codec == NULL)
                {
                  _err("There was no delimiter.\n");
                  break;
                }
              snprintf(line, sizeof(line),
                       "%s,unknown artist,unknown album,2,16,44100,%s,0\r\n",
                       fname,
                       codec);

              size_t wsize = fwrite(const_cast<char*>(line),
                                    strnlen(line, sizeof(line)),
                                    1,
                                    this->m_track_db_fp);
              if (wsize != 1)
                {
                  printf("File write error. [%d]\n", wsize);
                }

              printf("create line %s.\n", line);
            }
        }

      if (closedir(dir_descriptor) != 0)
        {
          _err("FS_Closedir error.\n");
        }
    }

  /* Reopen track database with read mode. */

  this->close();
  this->open("r");

  /* Delete all playlist. */

  this->deleteAll();

  /* Update playlist(type All). */

  this->updatePlaylist(ListTypeAllTrack, "");

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::deleteAll(void)
{
  FAR DIR *dir_descriptor = opendir(m_playlist_path);
  if (dir_descriptor == NULL)
    {
      printf("Cannot open folder.\n");
      return false;
    }

  while (true)
    {
      struct dirent *dir_ent = readdir(dir_descriptor);
      if (dir_ent == NULL)
        {
          break;
        }

      if (DTYPE_FILE == dir_ent->d_type)
        {
          /* Make terminate of string to use strlen(). */

          int idx = sizeof(this->m_track_db_file_name) - 1;
          this->m_track_db_file_name[idx] = '\0';

          /* Exclude track database file from deletion. */

          int ret = strncmp(dir_ent->d_name,
                            this->m_track_db_file_name,
                            strlen(this->m_track_db_file_name));
          if (ret != 0)
            {
              if (unlink(dir_ent->d_name) != 0)
                {
                  printf("Cannot delete.\n");
                  closedir(dir_descriptor);
                  return false;
                }
            }
        }
    }

  closedir(dir_descriptor);

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::deleteOne(ListType type, FAR const char *key_str)
{
  /* Check argument */

  if (key_str == NULL)
    {
      return false;
    }

  if (type == ListTypeAllTrack)
    {
      _err(" List type All cannnot be deleted.\n");
      return true;
    }

  char file_name[FileNameMaxLength];
  this->getFileName(type, key_str, file_name, sizeof(file_name));
  if (unlink(file_name) != 0)
    {
      printf("Cannot delete. %d %s\n", errno, file_name);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::isTargetTrack(ListType       type,
                             FAR const char *key_str,
                             FAR Track      *track)
{
  bool rtcd;
  int  cmp_rst;

  /* Check arguments */

  if (key_str == NULL || track == NULL)
    {
      return false;
    }

  switch (type)
    {
      case ListTypeAllTrack:
        rtcd = true;
        break;

      case ListTypeArtist:
        cmp_rst = strncmp(track->author, key_str, sizeof(track->author));
        rtcd = ((cmp_rst == 0) ? true : false);
        break;

      case ListTypeAlbum:
        cmp_rst = strncmp(track->album, key_str, sizeof(track->album));
        rtcd = ((cmp_rst == 0) ? true : false);
        break;

      case ListTypeUser:
        rtcd = false;
        break;

      default:
        rtcd = true;
        break;
    }

  return rtcd;
}

/*--------------------------------------------------------------------------*/
bool Playlist::loadAliasList(void)
{
  /* Open AliasList. */

  char file_name[FileNameMaxLength + 1];
  if (!this->getFileName(this->m_list_type,
                         this->m_list_key,
                         file_name,
                         sizeof(file_name)))
    {
      return false;
    }

  FAR FILE* list_fp = fopen(static_cast<const char*>(file_name), "r");
  if (list_fp == NULL)
    {
      _err("list for [%s] is not exist.\n", this->m_list_key);
      return false;
    }

  /* Clear list. */

  this->m_alias_list.clear();

  /* Read alias data from file, and push_back to vector. */

  while(true)
    {
      uint32_t read_data;
      fread(&read_data, sizeof(read_data), 1, list_fp);
      if (feof(list_fp) != 0)
        {
          break;
        }

      this->m_alias_list.push(read_data);
    }

  /* Correct index. */

  if (this->m_play_idx >= (this->m_alias_list.size() - 1))
    {
      this->m_play_idx = -1;
    }

  /* Close list. */

  fclose(list_fp);

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::shuffleList(int idx_top)
{
  uint32_t tmp;

  for (int idx_src = idx_top; idx_src < this->m_alias_list.size(); idx_src++)
    {
      int idx_dst = rand() % (this->m_alias_list.size() - idx_top);
      idx_dst += idx_top;

      tmp = this->m_alias_list.at(idx_src);
      this->m_alias_list.writable_at(idx_src) =
        this->m_alias_list.at(idx_dst);
      this->m_alias_list.writable_at(idx_dst) = tmp;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::readLine(FAR char *line, uint32_t line_size)
{
  if (this->m_track_db_fp == NULL)
    {
      _err("file not opened.\n");
      return false;
    }

  /* Check argument */

  if (line == NULL)
    {
      return false;
    }

  FAR char *tmp = this->m_line_buffer;
  int read_size;
  int idx;

  read_size = fread(tmp,
                    1,
                    sizeof(this->m_line_buffer),
                    this->m_track_db_fp);

  if (0 == read_size)
    {
      return false;
    }

  for (idx = 0; idx < read_size; idx++)
    {
      if (*tmp == 0x0d)
        {
          *tmp = '\0';
        }
      else if (*tmp == 0x0a)
        {
          *tmp = '\0';
          break;
        }
      else
        {
        }

      tmp++;
    }

  /* If read 1 line complete, set file pointer back to top of row. */

  fseek(this->m_track_db_fp, -((read_size - 1) - idx), SEEK_CUR);

  /* Make terminate of string to use strncpy(). */

  this->m_line_buffer[sizeof(this->m_line_buffer) - 1] = '\0';

  strncpy(line, this->m_line_buffer, line_size - 1);

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::parseTrackInfo(FAR Track *track,
                              FAR char  *line,
                              uint32_t  line_size)
{
  /* Check arguments */

  if (track == NULL || line == NULL)
    {
      return false;
    }

  memset(track, 0, sizeof(Track));

  /* Make terminate of string to use strtok(). */

  char token_buffer[line_size + 1];
  strncpy(token_buffer, line, line_size);
  token_buffer[line_size] = '\0';

  /* Get track name. */

  strncpy(track->title, strtok(token_buffer, ","), sizeof(track->title) - 1);

  /* Get author. */

  FAR char *tp = strtok(NULL, ",");
  if (tp == NULL)
    {
      return false;
    }
  strncpy(track->author, tp, sizeof(track->author) - 1);

  /* Get album. */

  tp = strtok(NULL, ",");
  if (tp == NULL)
    {
      return false;
    }
  strncpy(track->album, tp, sizeof(track->album) - 1);

  /* Get channel number. */

  tp = strtok(NULL, ",");
  if (tp == NULL)
    {
      return false;
    }
  int ch_num = atoi(tp);
  if (ch_num == 1)
    {
      track->channel_number = AS_CHANNEL_MONO;
    }
  else if (ch_num == 2)
    {
      track->channel_number = AS_CHANNEL_STEREO;
    }
  else
    {
      return false;
    }

  /* Get bit length. */

  tp = strtok(NULL, ",");
  if (tp == NULL)
    {
      return false;
    }
  int length = atoi(tp);
  if (length == 16)
    {
      track->bit_length = AS_BITLENGTH_16;
    }
  else if (length == 24)
    {
      track->bit_length = AS_BITLENGTH_24;
    }
  else
    {
      return false;
    }

  /* Get sampling rate. */

  tp = strtok(NULL, ",");
  if (tp == NULL)
    {
      return false;
    }
  int rate = atoi(tp);
  if (rate == 0)
    {
      track->sampling_rate = AS_SAMPLINGRATE_AUTO;
    }
  else if (rate == 8000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_8000;
    }
  else if (rate == 16000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_16000;
    }
  else if (rate == 24000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_24000;
    }
  else if (rate == 32000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_32000;
    }
  else if (rate == 44100)
    {
      track->sampling_rate = AS_SAMPLINGRATE_44100;
    }
  else if (rate == 48000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_48000;
    }
  else if (rate == 64000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_64000;
    }
  else if (rate == 88200)
    {
      track->sampling_rate = AS_SAMPLINGRATE_88200;
    }
  else if (rate == 96000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_96000;
    }
  else if (rate == 176400)
    {
      track->sampling_rate = AS_SAMPLINGRATE_176400;
    }
  else if (rate == 192000)
    {
      track->sampling_rate = AS_SAMPLINGRATE_192000;
    }
  else
    {
      return false;
    }

  /* Get codec type. */

  tp = strtok(NULL, ",");
  if (tp == NULL)
    {
      return false;
    }
  char codec[16] = { '\0' };
  strncpy(codec, tp, sizeof(codec) - 1);
  if ((strncmp(codec, "wav", sizeof(codec)) == 0) ||
      (strncmp(codec, "WAV", sizeof(codec)) == 0))
    {
      track->codec_type = AS_CODECTYPE_WAV;
    }
  else if ((strncmp(codec, "mp3", sizeof(codec)) == 0) ||
           (strncmp(codec, "MP3", sizeof(codec)) == 0))
    {
      track->codec_type = AS_CODECTYPE_MP3;
    }
  else if ((strncmp(codec, "aac", sizeof(codec)) == 0) ||
           (strncmp(codec, "AAC", sizeof(codec)) == 0))
    {
      track->codec_type = AS_CODECTYPE_AAC;
    }
  else if ((strncmp(codec, "opus", sizeof(codec)) == 0) ||
           (strncmp(codec, "OPUS", sizeof(codec)) == 0))
    {
      track->codec_type = AS_CODECTYPE_OPUS;
    }
  else
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool Playlist::getFileName(ListType       type,
                           FAR const char *key_str,
                           FAR char       *file_name,
                           uint8_t        max_length)
{
  const char prefix[] = "alias_list_";

  /* Check arguments */

  if (key_str == NULL || file_name == NULL || max_length < 1)
    {
      return false;
    }

  switch (type)
    {
      case ListTypeAllTrack:
          snprintf(file_name,
                   max_length - 1,
                   "%s/%s%s.bin",
                   m_playlist_path,
                   prefix,
                   "alltrack");
          break;

      case ListTypeArtist:
          snprintf(file_name,
                   max_length - 1,
                   "%s/%s%s%s.bin",
                   m_playlist_path,
                   prefix,
                   "artist_",
                   key_str);
          break;

      case ListTypeAlbum:
          snprintf(file_name,
                   max_length - 1,
                   "%s/%s%s%s.bin",
                   m_playlist_path,
                   prefix,
                   "album_",
                   key_str);
          break;

      case ListTypeUser:
          snprintf(file_name,
                   max_length - 1,
                   "%s/%s%s%s.bin",
                   m_playlist_path,
                   prefix,
                   "user_",
                   key_str);
          break;

      default:
          snprintf(file_name,
                   max_length - 1,
                   "%s/%s%s.bin",
                   m_playlist_path,
                   prefix,
                   "alltrack");
          break;
    }

  /* Add termination code */

  file_name[max_length - 1] = '\0';

  return true;
}
