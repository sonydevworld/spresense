/****************************************************************************
 * modules/include/digital_filter/edge_detection.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

/**
 * @file edge_detection.h
 */

#ifndef __INCLUDE_EDGE_DETECTION_H
#define __INCLUDE_EDGE_DETECTION_H

/**
 * @defgroup edge_detection Edge Detection
 * @{
 *
 * Edge detection utility library.
 */

#include <nuttx/config.h>

#include <stdint.h>

#define EDGE_DETECT_TYPE_RISE (0) /**< Edge detection type for rising edge */
#define EDGE_DETECT_TYPE_FALL (1) /**< Edge detection type for falling edge */

/**
 * @defgroup edge_detection_datatype Data Types
 * @{
 */

/**
 * @struct edge_detectf_s
 *
 * Instance of edge detection for float data.
 */

struct edge_detectf_s {
  int type;
  uint32_t prev_width;
  uint32_t width;
  float *boundary;
  float *sub_buf;
  float *remain_data;
};

/**
 * @typedef edge_detectionf_s
 * Instance type of edge detection with float
 */

typedef struct edge_detectf_s edge_detectf_t;

/**
 * @struct edge_detects_s
 *
 * Instance of edge detection for int16_t data.
 */

struct edge_detects_s {
  int type;
  uint32_t prev_width;
  uint32_t width;
  int16_t *boundary;
  int16_t *sub_buf;
  int16_t *remain_data;
};

/**
 * @typedef edge_detections_s
 * Instance type of edge detection with int16_t
 */

typedef struct edge_detects_s edge_detects_t;

/** @} edge_detection_datatype */

#  ifdef __cplusplus
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/**
 * @defgroup edge_detection_func Functions
 * @{
 */

/**
 * Create edge_detect instance for float data.
 *
 * @param [in] boundary_data: Edge boundary definition data array.
 * @param [in] datelen: Length of boundary_data.
 * @param [in] keep_width: Number of samples to keep the last value.
 * @param [in] type: Type of edge. @ref EDGE_DETECT_TYPE_RISE 
 * or @ref EDGE_DETECT_TYPE_FALL.
 *
 * @return instance of edge_detect. Return NULL when any error is occured.
 */

edge_detectf_t *edge_detection_createf(float *boundary_data, int datalen,
    uint32_t keep_width, int type);

/**
 * Delete edge_detect instance of float data.
 *
 * @param [in] detector: Instance of edge_detect
 */

void edge_detection_deletef(edge_detectf_t *detector);

/**
 * Detect edge from float data array.
 *
 * @param [in] detector: Instance of edge_detect
 * @param [in] input: Data array to detect edge.
 * @param [in] len: Length of data array.
 *
 * @return 0 or positive value is offset position of detected edge.
 *         Negative value when no edge detected.
 */

int edge_detectf(edge_detectf_t *detector, float *input, uint32_t len);

/**
 * Reset an edge detection instance for reflesh.
 *
 * @param [in] detector: Instance of edge_detect
 *
 * @return Return 0 when it success. Negative value is returned when
 *         any error is occured.
 */

int edge_detection_resetf(edge_detectf_t *detector);

/**
 * Create edge_detect instance for int16_t data.
 *
 * @param [in] boundary_data: Edge boundary definition data array.
 * @param [in] datelen: Length of boundary_data.
 * @param [in] keep_width: Number of samples to keep the last value.
 * @param [in] type: Type of edge. @ref EDGE_DETECT_TYPE_RISE 
 * or @ref EDGE_DETECT_TYPE_FALL.
 *
 * @return instance of edge_detect. Return NULL when any error is occured.
 */

edge_detects_t *edge_detection_creates(int16_t *boundary_data, int datalen,
    uint32_t keep_width, int type);

/**
 * Delete edge_detect instance of int16_t data.
 *
 * @param [in] detector: Instance of edge_detect
 */

void edge_detection_deletes(edge_detects_t *detector);

/**
 * Detect edge from int16_t data array.
 *
 * @param [in] detector: Instance of edge_detect
 * @param [in] input: Data array to detect edge.
 * @param [in] len: Length of data array.
 *
 * @return 0 or positive value is offset position of detected edge.
 *         Negative value when no edge detected.
 */

int edge_detects(edge_detects_t *detector, int16_t *input, uint32_t len);

/**
 * Reset an edge detection instance for reflesh.
 *
 * @param [in] detector: Instance of edge_detect
 *
 * @return Return 0 when it success. Negative value is returned when
 *         any error is occured.
 */

int edge_detection_resets(edge_detects_t *detector);

/** @} edge_detection_func */

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

/** @} edge_detection */

#endif  /* __INCLUDE_EDGE_DETECTION_H */
