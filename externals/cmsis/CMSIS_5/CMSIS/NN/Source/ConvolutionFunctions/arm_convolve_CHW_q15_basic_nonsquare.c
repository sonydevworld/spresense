/*
 * Copyright (C) 2010-2018 Arm Limited or its affiliates. All rights reserved.
 * Copyright 2018 Sony Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* ----------------------------------------------------------------------
 * Title:        arm_convolve_CHW_q15_basic_nonsquare.c
 * Author:       Sony Corporation
 * Description:  Sony Corporation added this file to 5.4.0
 *               to support the CHW tensor layout
 * $Date:        14. September 2018
 * -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_nnfunctions_nnabla.h"

/**
 *  @ingroup groupNN
 */

/**
 * @addtogroup NNConv
 * @{
 */

  /**
   * @brief Basic Q15 version of CHW convolution (non-sqaure shape)
   * @param[in]       Im_in         pointer to input tensor
   * @param[in]       dim_im_in_x   input tensor dimention x
   * @param[in]       dim_im_in_y   input tensor dimention y
   * @param[in]       ch_im_in      number of input tensor channels
   * @param[in]       wt            pointer to kernel weights
   * @param[in]       ch_im_out     number of filters, i.e., output tensor channels
   * @param[in]       dim_kernel_x  filter kernel size x
   * @param[in]       dim_kernel_y  filter kernel size y
   * @param[in]       padding_x     padding sizes x
   * @param[in]       padding_y     padding sizes y
   * @param[in]       stride_x      convolution stride x
   * @param[in]       stride_y      convolution stride y
   * @param[in]       bias          pointer to bias
   * @param[in]       bias_shift    amount of left-shift for bias
   * @param[in]       out_shift     amount of right-shift for output
   * @param[in,out]   Im_out        pointer to output tensor
   * @param[in]       dim_im_out_x  output tensor dimension x
   * @param[in]       dim_im_out_y  output tensor dimension y
   * @param[in,out]   bufferA       pointer to buffer space for input
   * @param[in,out]   bufferB       pointer to buffer space for output
   * @return     The function returns <code>ARM_MATH_SUCCESS</code> 
   *
   * @details
   *
   * <b>Buffer size:</b>
   *
   * bufferA size: ch_im_in*dim_kernel_x*dim_kernel_y
   *
   * bufferB size: 0
   *
   * This basic version is designed to work for any input tensor and weight
   * dimension. 
   */

arm_status
arm_convolve_CHW_q15_basic_nonsquare(const q15_t * Im_in,
                                     const uint16_t dim_im_in_x,
                                     const uint16_t dim_im_in_y,
                                     const uint16_t ch_im_in,
                                     const q15_t * wt,
                                     const uint16_t ch_im_out,
                                     const uint16_t dim_kernel_x,
                                     const uint16_t dim_kernel_y,
                                     const uint16_t padding_x,
                                     const uint16_t padding_y,
                                     const uint16_t stride_x,
                                     const uint16_t stride_y,
                                     const q15_t * bias,
                                     const uint16_t bias_shift,
                                     const uint16_t out_shift,
                                     q15_t * Im_out, 
                                     const uint16_t dim_im_out_x, 
                                     const uint16_t dim_im_out_y, 
                                     q15_t * bufferA, 
                                     q7_t * bufferB)
{

    /* Run the following code for Cortex-M4 and Cortex-M7 */

    int16_t   i_out_y, i_out_x, i_ker_y, i_ker_x;
    int16_t   i_ker_x_begin, i_ker_y_begin;
    int16_t   i_ker_x_end, i_ker_y_end;
    int16_t   single_in_map_size = dim_im_in_x * dim_im_in_y;
    int16_t   kernel_size_2d = dim_kernel_x * dim_kernel_y;

    uint16_t  im2col_out_pixel_index = 0;
    q15_t    *pBuffer = bufferA;
    q15_t    *im_buffer = bufferA;
    const q15_t *pA;
    int       i;

    /* This part implements the im2col function */
    for (i_out_y = 0; i_out_y < dim_im_out_y; i_out_y++)
    {
        for (i_out_x = 0; i_out_x < dim_im_out_x; i_out_x++)
        {
#define USE_CHW_IN_COL 
#ifdef USE_CHW_IN_COL

            i_ker_y_begin = i_out_y * stride_y - padding_y;
            i_ker_y_end = i_ker_y_begin + dim_kernel_y;

            for (i_ker_y = i_ker_y_begin; i_ker_y < i_ker_y_end; i_ker_y++)
            {
                i_ker_x_begin = i_out_x * stride_x - padding_x;
                i_ker_x_end = i_ker_x_begin + dim_kernel_x;

                for (i_ker_x = i_ker_x_begin; i_ker_x < i_ker_x_end; i_ker_x++)
                {
                    q15_t *pDest = pBuffer + (i_ker_y - i_ker_y_begin) * dim_kernel_x + (i_ker_x - i_ker_x_begin);
                    q15_t *pDestEnd = pDest + ch_im_in * kernel_size_2d;

                    if (i_ker_y < 0 || i_ker_y >= dim_im_in_y || i_ker_x < 0 || i_ker_x >= dim_im_in_x)
                    {
                        /* Out of bound zero values */
                        for (; pDest < pDestEnd;)
                        {
                            *pDest = 0;
                            pDest += kernel_size_2d;
                        }
                    } else
                    {
                        const q15_t *pSrc = Im_in + i_ker_y * dim_im_in_x + i_ker_x;
                        for (; pDest < pDestEnd;)
                        {
                            *pDest = *pSrc;
                            pSrc += single_in_map_size;
                            pDest += kernel_size_2d;
                        }
                    }                        
                }
            }
#else
            // HWC in columns
            for (i_ker_y = i_out_y * stride_y - padding_y; i_ker_y < i_out_y * stride_y - padding_y + dim_kernel_y; i_ker_y++)
            {
                for (i_ker_x = i_out_x * stride_x - padding_x; i_ker_x < i_out_x * stride_x - padding_x + dim_kernel_x; i_ker_x++)
                {
                    if (i_ker_y < 0 || i_ker_y >= dim_im_in_y || i_ker_x < 0 || i_ker_x >= dim_im_in_y)
                    {
                        /* Filling 0 for out-of-bound paddings */
                        /* arm_fill_q15(0, pBuffer, ch_im_in); */
                        memset(pBuffer, 0, sizeof(q15_t)*ch_im_in);
                    } else
                    {
                        /* load CHW patch to HWC column */
                        const q15_t *pSrc = Im_in + i_ker_y * dim_im_in_x + i_ker_x;
                        for (int16_t ch_idx = 0; ch_idx < ch_im_in; ch_idx++) 
                        {
                            pBuffer[ch_idx++] = *pSrc;
                            pSrc += single_in_map_size;
                        }
                    }

                    pBuffer += ch_im_in;
                }
            }
#endif

            pA = wt;
            q15_t  *pOut = Im_out++;
            int16_t map_size_out = dim_im_out_x * dim_im_out_y;
            for (i = 0; i < ch_im_out; i++)
            {
                q31_t     sum = 0;
                q15_t    *pB = im_buffer;
                uint16_t  colCnt = (ch_im_in * dim_kernel_x * dim_kernel_y) >> 2;

                if (bias) 
                {
                    sum = ((q31_t)bias[i] << bias_shift) + NN_ROUND(out_shift);
                }

                while (colCnt)
                { 
                    q31_t     inA1 = *__SIMD32(pA)++;
                    q31_t     inB1 = *__SIMD32(pB)++;
                    q31_t     inA2 = *__SIMD32(pA)++;
                    q31_t     inB2 = *__SIMD32(pB)++;

                    sum = __SMLAD(inA1, inB1, sum);
                    sum = __SMLAD(inA2, inB2, sum);

                    colCnt--;
                }
                colCnt = (ch_im_in * dim_kernel_x * dim_kernel_y) & 0x3;
                while (colCnt)
                {
                    q15_t     inA1 = *pA++;
                    q15_t     inB1 = *pB++;
                    sum += inA1 * inB1;
                    colCnt--;
                }
                *pOut = (q15_t) __SSAT((sum >> out_shift), 16);
                pOut += map_size_out;
            }

            /* counter reset */
            pBuffer = im_buffer;
            im2col_out_pixel_index++;
        }
    }

    /* Return to application */
    return ARM_MATH_SUCCESS;
}

/**
 * @} end of NNConv group
 */
