/****************************************************************************
 * modules/include/audio/utilities/audio_wav_containerformat_parser.h
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_PARSER_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_PARSER_H

typedef void* handel_wav_parser;

/* format */

#define WAVE_FORMAT_UNKNOWN                 0x0000 /* Microsoft Corporation */
#define WAVE_FORMAT_PCM                     0x0001 /* Microsoft PCM format */
#define WAVE_FORMAT_MS_ADPCM                0x0002 /* Microsoft ADPCM */
#define WAVE_FORMAT_IEEE_FLOAT              0x0003 /* Micrososft 32 bit float format */
#define WAVE_FORMAT_VSELP                   0x0004 /* Compaq Computer Corporation */
#define WAVE_FORMAT_IBM_CVSD                0x0005 /* IBM Corporation */
#define WAVE_FORMAT_ALAW                    0x0006 /* Microsoft Corporation */
#define WAVE_FORMAT_MULAW                   0x0007 /* Microsoft Corporation */
#define WAVE_FORMAT_OKI_ADPCM               0x0010 /* OKI */
#define WAVE_FORMAT_IMA_ADPCM               0x0011 /* Intel Corporation */
#define WAVE_FORMAT_MEDIASPACE_ADPCM        0x0012 /* Videologic */
#define WAVE_FORMAT_SIERRA_ADPCM            0x0013 /* Sierra Semiconductor Corp */
#define WAVE_FORMAT_G723_ADPCM              0x0014 /* Antex Electronics Corporation */
#define WAVE_FORMAT_DIGISTD                 0x0015 /* DSP Solutions, Inc. */
#define WAVE_FORMAT_DIGIFIX                 0x0016 /* DSP Solutions, Inc. */
#define WAVE_FORMAT_DIALOGIC_OKI_ADPCM      0x0017 /* Dialogic Corporation */
#define WAVE_FORMAT_MEDIAVISION_ADPCM       0x0018 /* Media Vision, Inc. */
#define WAVE_FORMAT_CU_CODEC                0x0019 /* Hewlett-Packard Company */
#define WAVE_FORMAT_YAMAHA_ADPCM            0x0020 /* Yamaha Corporation of America */
#define WAVE_FORMAT_SONARC                  0x0021 /* Speech Compression */
#define WAVE_FORMAT_DSPGROUP_TRUESPEECH     0x0022 /* DSP Group, Inc */
#define WAVE_FORMAT_ECHOSC1                 0x0023 /* Echo Speech Corporation */
#define WAVE_FORMAT_AUDIOFILE_AF36          0x0024 /* Audiofile, Inc. */
#define WAVE_FORMAT_APTX                    0x0025 /* Audio Processing Technology */
#define WAVE_FORMAT_AUDIOFILE_AF10          0x0026 /* Audiofile, Inc. */
#define WAVE_FORMAT_PROSODY_1612            0x0027 /* Aculab plc */
#define WAVE_FORMAT_LRC                     0x0028 /* Merging Technologies S.A. */
#define WAVE_FORMAT_DOLBY_AC2               0x0030 /* Dolby Laboratories */
#define WAVE_FORMAT_GSM610                  0x0031 /* Microsoft Corporation */
#define WAVE_FORMAT_MSNAUDIO                0x0032 /* Microsoft Corporation */
#define WAVE_FORMAT_ANTEX_ADPCME            0x0033 /* Antex Electronics Corporation */
#define WAVE_FORMAT_CONTROL_RES_VQLPC       0x0034 /* Control Resources Limited */
#define WAVE_FORMAT_DIGIREAL                0x0035 /* DSP Solutions, Inc. */
#define WAVE_FORMAT_DIGIADPCM               0x0036 /* DSP Solutions, Inc. */
#define WAVE_FORMAT_CONTROL_RES_CR10        0x0037 /* Control Resources Limited */
#define WAVE_FORMAT_NMS_VBXADPCM            0x0038 /* Natural MicroSystems */
#define WAVE_FORMAT_ROLAND_RDAC             0x0039 /* Roland */
#define WAVE_FORMAT_ECHOSC3                 0x003A /* Echo Speech Corporation */
#define WAVE_FORMAT_ROCKWELL_ADPCM          0x003B /* Rockwell International */
#define WAVE_FORMAT_ROCKWELL_DIGITALK       0x003C /* Rockwell International */
#define WAVE_FORMAT_XEBEC                   0x003D /* Xebec Multimedia Solutions Limited */
#define WAVE_FORMAT_G721_ADPCM              0x0040 /* Antex Electronics Corporation */
#define WAVE_FORMAT_G728_CELP               0x0041 /* Antex Electronics Corporation */
#define WAVE_FORMAT_MSG723                  0x0042 /* Microsoft Corporation */
#define WAVE_FORMAT_MPEG                    0x0050 /* Microsoft Corporation */
#define WAVE_FORMAT_RT24                    0x0052 /* InSoft Inc. */
#define WAVE_FORMAT_PAC                     0x0053 /* InSoft Inc. */
#define WAVE_FORMAT_MPEGLAYER3              0x0055 /* MPEG 1 Layer 3 */
#define WAVE_FORMAT_LUCENT_G723             0x0059 /* Lucent Technologies */
#define WAVE_FORMAT_CIRRUS                  0x0060 /* Cirrus Logic */
#define WAVE_FORMAT_ESPCM                   0x0061 /* ESS Technology */
#define WAVE_FORMAT_VOXWARE                 0x0062 /* Voxware Inc. */
#define WAVE_FORMAT_CANOPUS_ATRAC           0x0063 /* Canopus, Co., Ltd. */
#define WAVE_FORMAT_G726_ADPCM              0x0064 /* APICOM */
#define WAVE_FORMAT_G722_ADPCM              0x0065 /* APICOM */
#define WAVE_FORMAT_DSAT                    0x0066 /* Microsoft Corporation */
#define WAVE_FORMAT_DSAT_DISPLAY            0x0067 /* Microsoft Corporation */
#define WAVE_FORMAT_VOXWARE_BYTE_ALIGNED    0x0069 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_AC8             0x0070 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_AC10            0x0071 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_AC16            0x0072 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_AC20            0x0073 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_RT24            0x0074 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_RT29            0x0075 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_RT29HW          0x0076 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_VR12            0x0077 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_VR18            0x0078 /* Voxware Inc. */
#define WAVE_FORMAT_VOXWARE_TQ40            0x0079 /* Voxware Inc. */
#define WAVE_FORMAT_SOFTSOUND               0x0080 /* Softsound, Ltd. */
#define WAVE_FORMAT_VOXARE_TQ60             0x0081 /* Voxware Inc. */
#define WAVE_FORMAT_MSRT24                  0x0082 /* Microsoft Corporation */
#define WAVE_FORMAT_G729A                   0x0083 /* AT&T Laboratories */
#define WAVE_FORMAT_MVI_MV12                0x0084 /* Motion Pixels */
#define WAVE_FORMAT_DF_G726                 0x0085 /* DataFusion Systems */
#define WAVE_FORMAT_DF_GSM610               0x0086 /* DataFusion Systems */
#define WAVE_FORMAT_ONLIVE                  0x0089 /* OnLive! Technologies, Inc. */
#define WAVE_FORMAT_SBC24                   0x0091 /* Siemens Business Communications Systems */
#define WAVE_FORMAT_DOLBY_AC3_SPDIF         0x0092 /* Sonic Foundry */
#define WAVE_FORMAT_ZYXEL_ADPCM             0x0097 /* ZyXEL Communications, Inc. */
#define WAVE_FORMAT_PHILIPS_LPCBB           0x0098 /* Philips Speech Processing */
#define WAVE_FORMAT_PACKED                  0x0099 /* Studer Professional Audio AG */
#define WAVE_FORMAT_RHETOREX_ADPCM          0x0100 /* Rhetorex, Inc. */
#define WAVE_FORMAT_IBM_MULAW               0x0101 /* IBM mu-law format */
#define WAVE_FORMAT_IBM_ALAW                0x0102 /* IBM a-law format */
#define WAVE_FORMAT_IBM_ADPCM               0x0103 /* IBM AVC Adaptive Differential PCM format */
#define WAVE_FORMAT_VIVO_G723               0x0111 /* Vivo Software */
#define WAVE_FORMAT_VIVO_SIREN              0x0112 /* Vivo Software */
#define WAVE_FORMAT_DIGITAL_G723            0x0123 /* Digital Equipment Corporation */
#define WAVE_FORMAT_CREATIVE_ADPCM          0x0200 /* Creative Labs, Inc. */
#define WAVE_FORMAT_CREATIVE_FASTSPEECH8    0x0202 /* Creative Labs, Inc. */
#define WAVE_FORMAT_CREATIVE_FASTSPEECH10   0x0203 /* Creative Labs, Inc. */
#define WAVE_FORMAT_QUARTERDECK             0x0220 /* Quarterdeck Corporation */
#define WAVE_FORMAT_FM_TOWNS_SND            0x0300 /* Fujitsu Corporation */
#define WAVE_FORMAT_BZV_DIGITAL             0x0400 /* Brooktree Corporation */
#define WAVE_FORMAT_VME_VMPCM               0x0680 /* AT&T Labs, Inc. */
#define WAVE_FORMAT_OLIGSM                  0x1000 /* Ing C. Olivetti & C., S.p.A. */
#define WAVE_FORMAT_OLIADPCM                0x1001 /* Ing C. Olivetti & C., S.p.A. */
#define WAVE_FORMAT_OLICELP                 0x1002 /* Ing C. Olivetti & C., S.p.A. */
#define WAVE_FORMAT_OLISBC                  0x1003 /* Ing C. Olivetti & C., S.p.A. */
#define WAVE_FORMAT_OLIOPR                  0x1004 /* Ing C. Olivetti & C., S.p.A. */
#define WAVE_FORMAT_LH_CODEC                0x1100 /* Lernout & Hauspie */
#define WAVE_FORMAT_NORRIS                  0x1400 /* Norris Communications, Inc. */
#define WAVE_FORMAT_SOUNDSPACE_MUSICOMPRESS 0x1500 /* AT&T Labs, Inc. */
#define WAVE_FORMAT_DVM                     0x2000 /* FAST Multimedia AG */
#define WAVE_FORMAT_EXTENSIBLE              0xFFFE

/* Required chunk. */

#define CHUNKID_RIFF     0x46464952  /* RIFF */
#define SUBCHUNKID_FMT   0x20746D66  /* fmt  */
#define SUBCHUNKID_DATA  0x61746164  /* data */

/* Option chunk. */

#define SUBCHUNKID_JUNK  0x4B4E554A  /* JUNK */
#define SUBCHUNKID_LIST  0x5453494C  /* LIST */
#define SUBCHUNKID_ID3   0x20336469  /* id3  */
#define SUBCHUNKID_FACT  0x74636166  /* fact */
#define SUBCHUNKID_PLST  0x74736C70  /* plst */
#define SUBCHUNKID_CUE   0x20657563  /* cue  */
#define SUBCHUNKID_LABL  0x6C62616C  /* labl */
#define SUBCHUNKID_NOTE  0x65746F6E  /* note */
#define SUBCHUNKID_LTXT  0x7478746C  /* ltxt */
#define SUBCHUNKID_SMPL  0x6C706D73  /* smpl */
#define SUBCHUNKID_INST  0x74736E69  /* inst */
#define SUBCHUNKID_BEXT  0x74786562  /* bext */
#define SUBCHUNKID_IXML  0x4C4D5869  /* iXML */
#define SUBCHUNKID_QLTY  0x79746C71  /* qlty */
#define SUBCHUNKID_MEXT  0x7478656D  /* mext */
#define SUBCHUNKID_LEVL  0x6C76656C  /* levl */
#define SUBCHUNKID_LINK  0x6B6E696C  /* link */
#define SUBCHUNKID_AXML  0x6C6D7861  /* axml */
#define SUBCHUNKID_CONT  0x746E6F63  /* cont */

/* For line buffer mode. */

#define STDIO_BUFFER_SIZE 4096

#define MAX_CHUNK_LIST 128

struct chunk_s
{
  uint32_t chunk_id;
  int32_t  size;
};
typedef struct chunk_s chunk_t;

struct chunk_list_s
{
  uint8_t  cnt;
  chunk_t  chunk[MAX_CHUNK_LIST];
};
typedef struct chunk_list_s chunk_list_t;


struct riff_chunk_s
{
  struct chunk_s  chunk;
  uint32_t        type;
};
typedef struct riff_chunk_s riff_chunk_t;

struct fmt_chunk_s
{
  uint16_t  format;
  uint16_t  channel;
  uint32_t  rate;
  uint32_t  avgbyte;
  uint16_t  block;
  uint16_t  bit;
  uint16_t  extended_size;
};
typedef struct fmt_chunk_s fmt_chunk_t;

struct handel_wav_parser_s
{
  chunk_list_t  chunk_list;
  uint32_t      chunk_offset[MAX_CHUNK_LIST];
  uint32_t      data_offset;
  uint32_t      cur_offset;
  uint32_t      file_size;
  uint32_t      data_size;
  uint32_t      read_size;
  FILE          *fd;
};
typedef struct handel_wav_parser_s handel_wav_parser_t;


class WavContainerFormatParser
{
public:
  WavContainerFormatParser() {}
  ~WavContainerFormatParser() {}

  /*
   * Parse WAV container
   *
   *   Parse WAV container and return handle, and "fmt" chunk.
   *   The "fmt" chunk include sampling rate, bit length, ch num... and more.
   *
   */

  handel_wav_parser parseChunk(const char *file_path, fmt_chunk_t *fmt);

  /*
   * Get Chunk List
   *
   *   Get list of chunks which is included in designated WAV file.
   *
   */

  bool getChunkList(handel_wav_parser handle, chunk_list_t *list);

  /*
   * Get Chunk
   *
   *   Get chunk by chunk id.
   *
   */

  bool getChunk(handel_wav_parser handle, uint32_t chunk_id, int8_t *buffer);

  /*
   * Get Data Chunk
   *
   *   Get Data chunk.
   *
   */

  int32_t getDataChunk(handel_wav_parser handle, uint16_t format, int8_t *buffer, uint32_t size);

  /*
   * Reset Parser
   *
   *   Close WAV file and free internal memory area.
   *
   */

  void resetParser(handel_wav_parser handle);

private:
};

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_PARSER_H */
