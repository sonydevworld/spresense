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
 * Title:        arm_convolve_CHW_f32_basic_nonsquare.c
 * Author:       Sony Corporation
 * Description:  Sony Corporation added this file to 5.4.0 for these reasons:
 *                - support float version of convolution
 *                - support the CHW tensor layout
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
   * @brief Basic float32 version of CHW convolution (non-sqaure shape)
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
arm_convolve_CHW_f32_basic_nonsquare(const float * Im_in,
                                     const uint16_t dim_im_in_x,
                                     const uint16_t dim_im_in_y,
                                     const uint16_t ch_im_in,
                                     const float * wt,
                                     const uint16_t ch_im_out,
                                     const uint16_t dim_kernel_x,
                                     const uint16_t dim_kernel_y,
                                     const uint16_t padding_x,
                                     const uint16_t padding_y,
                                     const uint16_t stride_x,
                                     const uint16_t stride_y,
                                     const float * bias,
                                     float * Im_out, 
                                     const uint16_t dim_im_out_x, 
                                     const uint16_t dim_im_out_y, 
                                     float * bufferA, 
                                     float * bufferB)
{

    /* Run the following code for Cortex-M4 and Cortex-M7 */

    int16_t   i_out_y, i_out_x, i_ker_y, i_ker_x;
    int16_t   i_ker_x_begin, i_ker_y_begin;
    int16_t   i_ker_x_end, i_ker_y_end;
    int16_t   single_in_map_size = dim_im_in_x * dim_im_in_y;
    int16_t   kernel_size_2d = dim_kernel_x * dim_kernel_y;

    uint16_t  im2col_out_pixel_index = 0;
    float    *pBuffer = bufferA;
    float    *im_buffer = bufferA;
    const float *pA;
    int       i;

    /* This part implements the im2col function */
    for (i_out_y = 0; i_out_y < dim_im_out_y; i_out_y++)
    {
        for (i_out_x = 0; i_out_x < dim_im_out_x; i_out_x++)
        {
            i_ker_y_begin = i_out_y * stride_y - padding_y;
            i_ker_y_end = i_ker_y_begin + dim_kernel_y;

            for (i_ker_y = i_ker_y_begin; i_ker_y < i_ker_y_end; i_ker_y++)
            {
                i_ker_x_begin = i_out_x * stride_x - padding_x;
                i_ker_x_end = i_ker_x_begin + dim_kernel_x;

                for (i_ker_x = i_ker_x_begin; i_ker_x < i_ker_x_end; i_ker_x++)
                {
                    float *pDest = pBuffer + (i_ker_y - i_ker_y_begin) * dim_kernel_x + (i_ker_x - i_ker_x_begin);
                    float *pDestEnd = pDest + ch_im_in * kernel_size_2d;

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
                        const float *pSrc = Im_in + i_ker_y * dim_im_in_x + i_ker_x;
                        for (; pDest < pDestEnd;)
                        {
                            *pDest = *pSrc;
                            pSrc += single_in_map_size;
                            pDest += kernel_size_2d;
                        }
                    }                        
                }
            }

            pA = wt;
            float  *pOut = Im_out++;
            int16_t map_size_out = dim_im_out_x * dim_im_out_y;
            for (i = 0; i < ch_im_out; i++)
            {
                float     sum = 0;
                float    *pB = im_buffer;
                uint16_t  colCnt = (ch_im_in * dim_kernel_x * dim_kernel_y) >> 2;

                if (bias) 
                {
                    sum = bias[i];
                }

                while (colCnt)
                { 
                    float   inA1 = *pA++;
                    float   inB1 = *pB++;
                    float   inA2 = *pA++;
                    float   inB2 = *pB++;

                    sum += inA1 * inB1;
                    sum += inA2 * inB2;

                    inA1 = *pA++;
                    inB1 = *pB++;
                    inA2 = *pA++;
                    inB2 = *pB++;
                    
                    sum += inA1 * inB1;
                    sum += inA2 * inB2;

                    colCnt--;
                }
                colCnt = (ch_im_in * dim_kernel_x * dim_kernel_y) & 0x3;
                while (colCnt)
                {
                    float     inA1 = *pA++;
                    float     inB1 = *pB++;
                    sum += inA1 * inB1;
                    colCnt--;
                }
                *pOut = sum;
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
