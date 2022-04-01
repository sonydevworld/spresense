/****************************************************************************
 * modules/digital_filter/edge_detection.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <arm_math.h>

#include <digital_filter/edge_detection.h>

#define SBIT(x) (1<<(x))

/****************************************************************************
 * Private functions
 ****************************************************************************/

static int check_the_edges(edge_detects_t *detector)
{
  uint32_t i;
  uint32_t prevw = detector->prev_width;
  uint32_t width = detector->width;
  int16_t *sub = detector->sub_buf;

  if (detector->type == EDGE_DETECT_TYPE_RISE)
    {

      /* First data should be lower than lowvalue
       * So the value sould be lower than 0.
       */

      for (i = 0; i < prevw; i++)
        {
          if (!(sub[i] & SBIT(15)))  /* is positive */
            {
              return 0;
            }
        }

      /* Other data should be higher than highvalue
       * So all substraction value is higher than 0.
       */

      for (; i < width; i++)
        {
          if (sub[i] & SBIT(15)) /* is negative */
            {
              return 0;
            }
        }
    }
  else /* Case of Falling type edge */
    {
      /* First data should be higher than ops_device
       * So the value sould be lower than 0.
       */

      for (i = 0; i < prevw; i++)
        {
          if (sub[i] & SBIT(15)) /* is negative */
            {
              return 0;
            }
        }

      /* Other data should be higher than highvalue
       * So all substraction value is higher than 0.
       */

      for (; i < width; i++)
        {
          if (!(sub[i] & SBIT(15))) /* is positive */
            {
              return 0;
            }
        }
    }

  /* Detect the edge */

  return 1;
}

static int check_the_edgef(edge_detectf_t *detector)
{
  uint32_t i;
  uint32_t prevw = detector->prev_width;
  uint32_t width = detector->width;
  int32_t *sub = (int32_t *)detector->sub_buf;

  if (detector->type == EDGE_DETECT_TYPE_RISE)
    {

      /* First data should be lower than lowvalue
       * So the value sould be lower than 0.
       */

      for (i = 0; i < prevw; i++)
        {
          if (!(sub[i] & SBIT(31))) /* is positive */
            {
              return 0;
            }
        }

      /* Other data should be higher than highvalue
       * So all substraction value is higher than 0.
       */

      for (; i < width; i++)
        {
          if (sub[i] & SBIT(31))  /* is negative */
            {
              return 0;
            }
        }
    }
  else /* Case of Falling type edge */
    {
      /* First data should be higher than ops_device
       * So the value sould be lower than 0.
       */

      for (i = 0; i < prevw; i++)
        {
          if (sub[i] & SBIT(31))  /* is negative */
            {
              return 0;
            }
        }

      /* Other data should be higher than highvalue
       * So all substraction value is higher than 0.
       */

      for (; i < width; i++)
        {
          if (!(sub[i] & SBIT(31)))  /* is positive */
            {
              return 0;
            }
        }
    }

  /* Detect the edge */

  return 1;
}

/****************************************************************************
 * Public functions
 ****************************************************************************/

#ifdef CONFIG_DIFITAL_FILTER_EDGE_DETECT

/**
 * Functions of type int16_t 
 */

/** edge_detection_creates() */

edge_detects_t *edge_detection_creates(int16_t *boundary_data,
    int datalen, uint32_t keep_width, int type)
{
  int i;
  edge_detects_t *ret;

  if ((datalen < 1) || (boundary_data == NULL)
      || ((type != EDGE_DETECT_TYPE_RISE) && (type != EDGE_DETECT_TYPE_FALL)))
    {
      return NULL;
    }

  ret = (edge_detects_t *)malloc(sizeof(edge_detects_t));
  if (ret == NULL)
    {
      return ret;
    }

  ret->boundary = (int16_t *)malloc(sizeof(int16_t) * (datalen + keep_width));
  if (ret->boundary == NULL)
    {
      free(ret);
      return NULL;
    }

  if ((datalen + keep_width) > 1)
    {
      ret->remain_data = (int16_t *)malloc(
          sizeof(int16_t) * (datalen + keep_width - 1));
      if (ret->remain_data == NULL)
        {
          free(ret->boundary);
          free(ret);
          return NULL;
        }
    }
  else
    {
      ret->remain_data = NULL;
    }

  ret->sub_buf = (int16_t *)malloc(sizeof(int16_t) * (datalen + keep_width));
  if (ret->sub_buf == NULL)
    {
      if (ret->remain_data)
        {
          free(ret->remain_data);
        }
      free(ret->boundary);
      free(ret);
      return NULL;
    }

  ret->prev_width = datalen - 1;
  ret->width = datalen + keep_width;
  ret->type = type;

  /* initialize boundary data */

  for (i = 0; i < datalen; i++)
    {
      ret->boundary[i] = boundary_data[i];
    }
  for (i = 0; i < keep_width; i++)
    {
      ret->boundary[i + datalen] = boundary_data[datalen - 1];
    }

  /* initialize remain data */

  if (ret->remain_data != NULL)
    {
      for (i = 0; i < (ret->width - 1); i++)
        {
          ret->remain_data[i] = boundary_data[0];
        }
    }

  return ret;
}

/** edge_detection_deletes() */

void edge_detection_deletes(edge_detects_t *detector)
{
  if (detector)
    {
      if (detector->boundary)
        {
          free(detector->boundary);
        }
      if (detector->remain_data)
        {
          free(detector->remain_data);
        }
      if (detector->sub_buf)
        {
          free(detector->sub_buf);
        }
      free(detector);
    }
}

/** edge_detection_resets() */

int edge_detection_resets(edge_detects_t *detector)
{
  int i;

  if (detector == NULL)
    {
      return -1;
    }

  /* initialize remain data */

  if (detector->remain_data != NULL)
    {
      for (i = 0; i < (detector->width - 1); i++)
        {
          detector->remain_data[i] = detector->boundary[0];
        }
    }

  return 0;
}

/** edge_detects() */

int edge_detects(edge_detects_t *detector, int16_t *input, uint32_t len)
{
  int i;
  int j;
  int ret;
  uint32_t width;
  int16_t *bound;
  int16_t *remain;
  int16_t *sub;

  if (detector == NULL || input == NULL || len < detector->width)
    {
      return -EINVAL;
    }

  width = detector->width;
  bound = detector->boundary;
  remain = detector->remain_data;
  sub = detector->sub_buf;

  /* At first, find edge with remained data */

  if (remain != NULL)
    {
      for (i = 0; i < (width - 1); i++)
        {
          arm_sub_q15(&remain[i], bound, sub, width - i - 1);
          arm_sub_q15(input, &bound[width - i - 1], &sub[width - i - 1], i + 1);
          if (check_the_edges(detector))
            {
              ret = i;

              /* Update remain data for next scan. */

              if (i < (width - 2))
                {
                  for (j = 0; j < (width - i - 2); j++)
                    {
                      remain[j] = remain[j + i + 1];
                    }
                  for (i = 0; j < width - 1; j++, i++)
                    {
                      remain[j] = input[i];
                    }
                }
              else
                {
                  for (j = 0; j < (width - 1); j++)
                    {
                      remain[j] = input[j];
                    }
                }
              return ret;
            }
        }
    }

  /* And now input data will be checked */

  for (i = 0; i <= len - width; i++)
    {
      arm_sub_q15(&input[i], bound, sub, width + 1);
      if (check_the_edges(detector))
        {
          ret = i + width - 1;

          /* Update remain data for next scan. */
          
          i++;

          for (j = 0; j < (width - 1); j++)
            {
              remain[j] = input[i + j];
            }
          return ret;
        }
    }

  /* End of input data is not checked yet.
   * so save it in remain buffer for checking with next data.
   */

  for (i = 0; i < (width - 1); i++)
    {
      remain[i] = input[len - width + 1 + i];
    }

  return -1;
}

/**
 * Functions of type float 
 */

/** edge_detection_createf() */

edge_detectf_t *edge_detection_createf(float *boundary_data,
    int datalen, uint32_t keep_width, int type)
{
  int i;
  edge_detectf_t *ret;

  if ((datalen < 1) || (boundary_data == NULL)
      || ((type != EDGE_DETECT_TYPE_RISE) && (type != EDGE_DETECT_TYPE_FALL)))
    {
      return NULL;
    }

  ret = (edge_detectf_t *)malloc(sizeof(edge_detectf_t));
  if (ret == NULL)
    {
      return ret;
    }

  ret->boundary = (float *)malloc(sizeof(float) * (datalen + keep_width));
  if (ret->boundary == NULL)
    {
      free(ret);
      return NULL;
    }

  if ((datalen + keep_width) > 1)
    {
      ret->remain_data = (float *)malloc(
          sizeof(float) * (datalen + keep_width - 1));
      if (ret->remain_data == NULL)
        {
          free(ret->boundary);
          free(ret);
          return NULL;
        }
    }
  else
    {
      ret->remain_data = NULL;
    }

  ret->sub_buf = (float *)malloc(sizeof(float) * (datalen + keep_width));
  if (ret->sub_buf == NULL)
    {
      if (ret->remain_data)
        {
          free(ret->remain_data);
        }
      free(ret->boundary);
      free(ret);
      return NULL;
    }

  ret->prev_width = datalen - 1;
  ret->width = datalen + keep_width;
  ret->type = type;

  /* initialize boundary data */

  for (i = 0; i < datalen; i++)
    {
      ret->boundary[i] = boundary_data[i];
    }
  for (i = 0; i < keep_width; i++)
    {
      ret->boundary[i + datalen] = boundary_data[datalen - 1];
    }

  /* initialize remain data */

  if (ret->remain_data != NULL)
    {
      for (i = 0; i < (ret->width - 1); i++)
        {
          ret->remain_data[i] = boundary_data[0];
        }
    }

  return ret;
}

/** edge_detection_deletef() */

void edge_detection_deletef(edge_detectf_t *detector)
{
  if (detector)
    {
      if (detector->boundary)
        {
          free(detector->boundary);
        }
      if (detector->remain_data)
        {
          free(detector->remain_data);
        }
      if (detector->sub_buf)
        {
          free(detector->sub_buf);
        }
      free(detector);
    }
}

/** edge_detection_resetf() */

int edge_detection_resetf(edge_detectf_t *detector)
{
  int i;

  if (detector == NULL)
    {
      return -1;
    }

  /* initialize remain data */

  if (detector->remain_data != NULL)
    {
      for (i = 0; i < (detector->width - 1); i++)
        {
          detector->remain_data[i] = detector->boundary[0];
        }
    }

  return 0;
}

/** edge_detectf() */

int edge_detectf(edge_detectf_t *detector, float *input, uint32_t len)
{
  int i;
  int j;
  int ret;
  uint32_t width;
  float *bound;
  float *remain;
  float *sub;

  if (detector == NULL || input == NULL || len < detector->width)
    {
      return -EINVAL;
    }

  width = detector->width;
  bound = detector->boundary;
  remain = detector->remain_data;
  sub = detector->sub_buf;

  /* At first, find edge with remained data */

  if (remain != NULL)
    {
      for (i = 0; i < (width - 1); i++)
        {
          arm_sub_f32(&remain[i], bound, sub, width - i - 1);
          arm_sub_f32(input, &bound[width - i - 1], &sub[width - i - 1], i + 1);
          if (check_the_edgef(detector))
            {
              ret = i;

              /* Update remain data for next scan. */

              if (i < (width - 2))
                {
                  for (j = 0; j < (width - i - 2); j++)
                    {
                      remain[j] = remain[j + i + 1];
                    }
                  for (i = 0; j < width - 1; j++, i++)
                    {
                      remain[j] = input[i];
                    }
                }
              else
                {
                  for (j = 0; j < (width - 1); j++)
                    {
                      remain[j] = input[j];
                    }
                }
              return ret;
            }
        }
    }

  /* And now input data will be checked */

  for (i = 0; i <= len - width; i++)
    {
      arm_sub_f32(&input[i], bound, sub, width + 1);
      if (check_the_edgef(detector))
        {
          ret = i + width - 1;

          /* Update remain data for next scan. */

          i++;

          for (j = 0; j < (width - 1); j++)
            {
              remain[j] = input[i + j];
            }
          return ret;
        }
    }

  /* End of input data is not checked yet.
   * so save it in remain buffer for checking with next data.
   */

  for (i = 0; i < (width - 1); i++)
    {
      remain[i] = input[len - width + 1 + i];
    }

  return -1;
}

#endif  /* CONFIG_DIFITAL_FILTER_EDGE_DETECT */
