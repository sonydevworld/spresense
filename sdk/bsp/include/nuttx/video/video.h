/****************************************************************************
 * bsp/include/nuttx/video/video.h
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

#ifndef __BSP_INCLUDE_NUTTX_VIDEO_VIDEO_H
#define __BSP_INCLUDE_NUTTX_VIDEO_VIDEO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/**
 * @defgroup video Video driver
 * @{ */
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define _VIDEOIOCBASE   (0x1000)

#define _VIDEOIOC(nr)       _IOC(_VIDEOIOCBASE,nr)

/**
 * @defgroup video_ioctl IOCTL commands
 * @{
 */

/**
 * Set the data format.
 *
 * @param[in] arg
 * Address pointing to struct #v4l2_format
 */

#define VIDIOC_S_FMT                  _VIDEOIOC(0x0001)

/**
 * Initiate user pointer I/O
 *
 * @param[in] arg
 * Address pointing to struct #v4l2_requestbuffers
 */

#define VIDIOC_REQBUFS                _VIDEOIOC(0x0002)

/**
 * Enqueue an empty buffer
 * 
 * @param[in] arg
 * Address pointing to struct #v4l2_buffer
 */

#define VIDIOC_QBUF                   _VIDEOIOC(0x0003)

/**
 * Dequeue a filled buffer
 *
 * @param[out] arg
 * Address pointing to struct #v4l2_buffer
 */

#define VIDIOC_DQBUF                  _VIDEOIOC(0x0004)

/**
 * Activate video device
 *
 * @param[in] arg
 * Address pointing to enum #v4l2_buf_type 
 */

#define VIDIOC_STREAMON               _VIDEOIOC(0x0005)

/** @} video_ioctl */

#define VIDEOIOC_CHG_IMGSNS_STATE     _VIDEOIOC(0x0081)
#define VIDEOIOC_SET_CAP_PARAM        _VIDEOIOC(0x0082)
#define VIDEOIOC_CAP_FRAME            _VIDEOIOC(0x0083)
#define VIDEOIOC_CONTI_CAP            _VIDEOIOC(0x0084)
#define VIDEOIOC_SET_IMGSNS_PARAM     _VIDEOIOC(0x0085)
#define VIDEOIOC_SET_IMGSNS_PARAM_ALL _VIDEOIOC(0x0086)
#define VIDEOIOC_WR_IMGSNS_REG        _VIDEOIOC(0x0087)
#define VIDEOIOC_RD_IMGSNS_REG        _VIDEOIOC(0x0088)
#define VIDEOIOC_DO_HALF_REL          _VIDEOIOC(0x0089)
#define VIDEOIOC_GET_AUTO_PARAM       _VIDEOIOC(0x008A)

/**
 * @defgroup video_defs Defines
 * @{
 */

#define VIDEO_HSIZE_QVGA        (320)
#define VIDEO_VSIZE_QVGA        (240)
#define VIDEO_HSIZE_VGA         (640)
#define VIDEO_VSIZE_VGA         (480)
#define VIDEO_HSIZE_QUADVGA     (1280)
#define VIDEO_VSIZE_QUADVGA     (960)
#define VIDEO_HSIZE_HD          (1280)
#define VIDEO_VSIZE_HD          (720)
#define VIDEO_HSIZE_FULLHD      (1920)
#define VIDEO_VSIZE_FULLHD      (1080)
#define VIDEO_HSIZE_5M          (2560)
#define VIDEO_VSIZE_5M          (1920)
#define VIDEO_HSIZE_3M          (2048)
#define VIDEO_VSIZE_3M          (1536)

/**  Four-character-code (FOURCC) */
#define v4l2_fourcc(a, b, c, d)\
  ((uint32_t)(a)        | ((uint32_t)(b) << 8) | \
  ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define v4l2_fourcc_be(a, b, c, d)    (v4l2_fourcc(a, b, c, d) | (1 << 31))

#define V4L2_PIX_FMT_UYVY v4l2_fourcc('U', 'Y', 'V', 'Y') /**< 16  YUV 4:2:2 */
#define V4L2_PIX_FMT_JPEG v4l2_fourcc('J', 'P', 'E', 'G') /**< JFIF JPEG     */

/** @} video_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup video_datatypes Data types
 * @{
 */

/** Buffer type. Currently, support only V4L2_BUF_TYPE_VIDEO_CAPTURE. */ 

enum v4l2_buf_type {
  V4L2_BUF_TYPE_VIDEO_CAPTURE        = 1,    /**< single-planar
                                                  video capture stream */
  V4L2_BUF_TYPE_VIDEO_OUTPUT         = 2,    /**< single-planar
                                                  video output stream */
  V4L2_BUF_TYPE_VIDEO_OVERLAY        = 3,    /**< video overlay */
  V4L2_BUF_TYPE_VBI_CAPTURE          = 4,    /**< raw VBI capture stream */
  V4L2_BUF_TYPE_VBI_OUTPUT           = 5,    /**< raw VBI output stream */
  V4L2_BUF_TYPE_SLICED_VBI_CAPTURE   = 6,    /**< sliced VBI capture stream */
  V4L2_BUF_TYPE_SLICED_VBI_OUTPUT    = 7,    /**< sliced VBI output stream */
  V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY = 8,    /**< video output overlay  */
  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9,    /**< multi-planar
                                                  video capture stream */
  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  = 10,   /**< multi-planar
                                                  video output stream */
  V4L2_BUF_TYPE_SDR_CAPTURE          = 11,   /**< Software Defined Radio
                                                  capture stream */
  V4L2_BUF_TYPE_SDR_OUTPUT           = 12,   /**< Software Defined Radio
                                                  output stream */
  V4L2_BUF_TYPE_META_CAPTURE         = 13,   /**< metadata capture */
  V4L2_BUF_TYPE_PRIVATE              = 0x80, /**< Deprecated, do not use */
};

/** Memory I/O method. Currently, support only V4L2_MEMORY_USERPTR. */

enum v4l2_memory {
  V4L2_MEMORY_MMAP         = 1,  /**< memory mapping I/O */
  V4L2_MEMORY_USERPTR      = 2,  /**< user pointer I/O  */
  V4L2_MEMORY_OVERLAY      = 3,  /**< overlay I/O */
  V4L2_MEMORY_DMABUF       = 4,  /**< DMA shared buffer I/O */
};

/** Field order. Currently, support only V4L2_FIELD_ANY */

enum v4l2_field {
  V4L2_FIELD_ANY           = 0, /**< driver can choose from none, */
  V4L2_FIELD_NONE          = 1, /**< this device has no fields ... */
  V4L2_FIELD_TOP           = 2, /**< top field only */
  V4L2_FIELD_BOTTOM        = 3, /**< bottom field only */
  V4L2_FIELD_INTERLACED    = 4, /**< both fields interlaced */
  V4L2_FIELD_SEQ_TB        = 5, /**< both fields sequential into one */
  V4L2_FIELD_SEQ_BT        = 6, /**< same as above + bottom-top order */
  V4L2_FIELD_ALTERNATE     = 7, /**< both fields alternating into */
  V4L2_FIELD_INTERLACED_TB = 8, /**< both fields interlaced, top field */
  V4L2_FIELD_INTERLACED_BT = 9, /**< both fields interlaced, top field */
};

/** @struct v4l2_requestbuffers
 *  @brief  parameter of ioctl(VIDIOC_REQBUFS) 
 */

struct v4l2_requestbuffers {
  uint32_t count;       /**< The number of buffers requested */
  uint32_t type;        /**< enum #v4l2_buf_type */
  uint32_t memory;      /**< enum #v4l2_memory */
  uint32_t reserved[2];
};
typedef struct v4l2_requestbuffers v4l2_requestbuffers_t;

struct v4l2_timecode {
  uint32_t type;
  uint32_t flags;
  uint8_t  frames;
  uint8_t  seconds;
  uint8_t  minutes;
  uint8_t  hours;
  uint8_t  userbits[4];
};
typedef struct v4l2_timecode v4l2_timecode_t;

struct v4l2_plane {
  uint32_t        bytesused;
  uint32_t        length;
  union {
    uint32_t      mem_offset;
    unsigned long userptr;
    int           fd;
  } m;
  uint32_t        data_offset;
  uint32_t        reserved[11];
};
typedef struct v4l2_plane v4l2_plane_t;

/** @struct v4l2_buffer
 *  @brief  Parameter of ioctl(VIDIOC_QBUF) and ioctl(VIDIOC_DQBUF). \n
 *          Currently, support only index, type, bytesused, memory, 
 *          m.userptr, and length.
 */

struct v4l2_buffer {
  uint32_t             index;     /**< buffer id */ 
  uint32_t             type;      /**< enum #v4l2_buf_type */
  uint32_t             bytesused; /**< Driver sets the image size */
  uint32_t             flags;     /**< buffer flags */
  uint32_t             field;     /**< the field order of the image */
  struct v4l2_timecode timecode;  /**< frame timecode */
  uint32_t             sequence;  /**< frame sequence number */
  uint32_t             memory;    /**< enum #v4l2_memory */
  union {
    uint32_t           offset;
    unsigned long      userptr;   /**< address of buffer */
    struct v4l2_plane  *planes;
    int                fd;
  } m;
  uint32_t             length;    /**< user set the buffer size */
  uint32_t             reserved2;
  uint32_t             reserved;
};
typedef struct v4l2_buffer v4l2_buffer_t;

/** Single-planar format structure.
 *  Currently, support only pixelformat and field
 */

struct v4l2_pix_format {
  uint32_t  width;        /**< Image width in pixels */
  uint32_t  height;       /**< Image height in pixels */
  uint32_t  pixelformat;  /**< The pixel format or type of compression.
                                V4L2_PIX_FMT_UYVY or V4L2_PIX_FMT_JPEG */
  uint32_t  field;        /**< enum #v4l2_field */
  uint32_t  bytesperline; /**< for padding, zero if unused */
  uint32_t  sizeimage;    /**< Size in bytes of the buffer
                                to hold a complete image */
  uint32_t  colorspace;   /**< Image colorspace */
  uint32_t  priv;         /**< private data, depends on pixelformat */
  uint32_t  flags;        /**< format flags (V4L2_PIX_FMT_FLAG_*) */
  union {
    uint32_t ycbcr_enc;   /**< enum v4l2_ycbcr_encoding */
    uint32_t hsv_enc;     /**< enum v4l2_hsv_encoding */
  };
  uint32_t  quantization; /**< enum v4l2_quantization */
  uint32_t  xfer_func;    /**< enum v4l2_xfer_func */
};
typedef struct v4l2_pix_format v4l2_pix_format_t;

/** @struct v4l2_format
 *  @brief  parameter of ioctl(VIDIOC_S_FMT) 
 */

struct v4l2_format {
  uint32_t  type;               /**< enum #v4l2_buf_type. */
  union {
    struct v4l2_pix_format pix; /**< image format */
  } fmt;
};
typedef struct v4l2_format v4l2_format_t;

/** @} video_datatypes */

/** @} video */

#endif /* __BSP_INCLUDE_NUTTX_VIDEO_VIDEO_H */
