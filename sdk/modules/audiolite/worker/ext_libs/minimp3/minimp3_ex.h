/*
    https://github.com/lieff/minimp3
    To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide.
    This software is distributed without any warranty.
    See <http://creativecommons.org/publicdomain/zero/1.0/>.
*/
#ifndef MINIMP3_EXT_H
#define MINIMP3_EXT_H

#include <stddef.h>
#include "minimp3.h"

/* flags for mp3dec_ex_open_* functions */
#define MP3D_SEEK_TO_BYTE   0      /* mp3dec_ex_seek seeks to byte in stream */
#define MP3D_SEEK_TO_SAMPLE 1      /* mp3dec_ex_seek precisely seeks to sample using index (created during duration calculation scan or when mp3dec_ex_seek called) */
#define MP3D_DO_NOT_SCAN    2      /* do not scan whole stream for duration if vbrtag not found, mp3dec_ex_t::samples will be filled only if mp3dec_ex_t::vbr_tag_found == 1 */
#define MP3D_ALLOW_MONO_STEREO_TRANSITION  4
#define MP3D_FLAGS_MASK 7

/* compile-time config */
#define MINIMP3_PREDECODE_FRAMES 2 /* frames to pre-decode and skip after seek (to fill internal structures) */
/*#define MINIMP3_SEEK_IDX_LINEAR_SEARCH*/ /* define to use linear index search instead of binary search on seek */
#define MINIMP3_IO_SIZE (4*1024) /* io buffer size for streaming functions, must be greater than MINIMP3_BUF_SIZE */
#define MINIMP3_BUF_SIZE (4*1024) /* buffer which can hold minimum 10 consecutive mp3 frames (~16KB) worst case */
/*#define MINIMP3_SCAN_LIMIT (256*1024)*/ /* how many bytes will be scanned to search first valid mp3 frame, to prevent stall on large non-mp3 files */
#define MINIMP3_ENABLE_RING 0      /* WIP enable hardware magic ring buffer if available, to make less input buffer memmove(s) in callback IO mode */

/* return error codes */
#define MP3D_E_PARAM   -1
#define MP3D_E_MEMORY  -2
#define MP3D_E_IOERROR -3
#define MP3D_E_USER    -4  /* can be used to stop processing from callbacks without indicating specific error */
#define MP3D_E_DECODE  -5  /* decode error which can't be safely skipped, such as sample rate, layer and channels change */

#define MINIMP3_ID3_DETECT_SIZE 10

typedef struct
{
    mp3d_sample_t *buffer;
    size_t samples; /* channels included, byte size = samples*sizeof(mp3d_sample_t) */
    int channels, hz, layer, avg_bitrate_kbps;
} mp3dec_file_info_t;

typedef struct
{
    const uint8_t *buffer;
    size_t size;
} mp3dec_map_info_t;

typedef struct
{
    uint64_t sample;
    uint64_t offset;
} mp3dec_frame_t;

typedef struct
{
    mp3dec_frame_t *frames;
    size_t num_frames, capacity;
} mp3dec_index_t;

typedef size_t (*MP3D_READ_CB)(void *buf, size_t size, void *user_data);
typedef int (*MP3D_SEEK_CB)(uint64_t position, void *user_data);

typedef struct
{
    MP3D_READ_CB read;
    void *read_data;
    MP3D_SEEK_CB seek;
    void *seek_data;
} mp3dec_io_t;

typedef struct
{
    mp3dec_t mp3d;
    mp3dec_map_info_t file;
    mp3dec_io_t *io;
    mp3dec_index_t index;
    uint64_t offset, samples, detected_samples, cur_sample, start_offset, end_offset;
    mp3dec_frame_info_t info;
    mp3d_sample_t buffer[MINIMP3_MAX_SAMPLES_PER_FRAME];
    size_t input_consumed, input_filled;
    int is_file, flags, vbr_tag_found, indexes_built;
    int free_format_bytes;
    int buffer_samples, buffer_consumed, to_skip, start_delay;
    int last_error;
} mp3dec_ex_t;

typedef int (*MP3D_ITERATE_CB)(void *user_data, const uint8_t *frame, int frame_size, int free_format_bytes, size_t buf_size, uint64_t offset, mp3dec_frame_info_t *info);
typedef int (*MP3D_PROGRESS_CB)(void *user_data, size_t file_size, uint64_t offset, mp3dec_frame_info_t *info);

#ifdef __cplusplus
extern "C" {
#endif

void mp3dec_skip_id3v1(const unsigned char *buf, size_t *pbuf_size);
size_t mp3dec_skip_id3v2(const unsigned char *buf, size_t buf_size);

/* detect mp3/mpa format */
int mp3dec_detect_buf(const uint8_t *buf, size_t buf_size);
int mp3dec_detect_cb(mp3dec_io_t *io, uint8_t *buf, size_t buf_size);
/* decode whole buffer block */
int mp3dec_load_buf(mp3dec_t *dec, const uint8_t *buf, size_t buf_size, mp3dec_file_info_t *info, MP3D_PROGRESS_CB progress_cb, void *user_data);
int mp3dec_load_cb(mp3dec_t *dec, mp3dec_io_t *io, uint8_t *buf, size_t buf_size, mp3dec_file_info_t *info, MP3D_PROGRESS_CB progress_cb, void *user_data);
/* iterate through frames */
int mp3dec_iterate_buf(const uint8_t *buf, size_t buf_size, MP3D_ITERATE_CB callback, void *user_data);
int mp3dec_iterate_cb(mp3dec_io_t *io, uint8_t *buf, size_t buf_size, MP3D_ITERATE_CB callback, void *user_data);
/* streaming decoder with seeking capability */
int mp3dec_ex_open_buf(mp3dec_ex_t *dec, const uint8_t *buf, size_t buf_size, int flags);
int mp3dec_ex_open_cb(mp3dec_ex_t *dec, mp3dec_io_t *io, int flags);
void mp3dec_ex_close(mp3dec_ex_t *dec);
int mp3dec_ex_seek(mp3dec_ex_t *dec, uint64_t position);
size_t mp3dec_ex_read_frame(mp3dec_ex_t *dec, mp3d_sample_t **buf, mp3dec_frame_info_t *frame_info, size_t max_samples);
size_t mp3dec_ex_read(mp3dec_ex_t *dec, mp3d_sample_t *buf, size_t samples);
/* stdio versions of file detect, load, iterate and stream */
int mp3dec_detect(const char *file_name);
int mp3dec_load(mp3dec_t *dec, const char *file_name, mp3dec_file_info_t *info, MP3D_PROGRESS_CB progress_cb, void *user_data);
int mp3dec_iterate(const char *file_name, MP3D_ITERATE_CB callback, void *user_data);
int mp3dec_ex_open(mp3dec_ex_t *dec, const char *file_name, int flags);

#ifdef __cplusplus
}
#endif
#endif /*MINIMP3_EXT_H*/
