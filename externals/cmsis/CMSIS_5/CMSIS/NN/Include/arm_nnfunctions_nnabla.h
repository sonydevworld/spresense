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
 * Title:        arm_nnfunctions_nnabla.h
 * Author:       Sony Corporation
 * Description:  Sony Corporation added this file to 5.4.0
 *               to add the following function prototypes:
 *               - arm_convolve_CHW_f32_basic_nonsquare()
 *               - arm_convolve_CHW_q15_basic_nonsquare()
 *               - arm_convolve_CHW_q7_basic_nonsquare()
 *               - arm_nn_CHW_mat_mult_kernel_q7_q15()
 * $Date:        14. September 2018
 * -------------------------------------------------------------------- */

#ifndef _ARM_NNFUNCTIONS_CHW_H
#define _ARM_NNFUNCTIONS_CHW_H

#include "arm_nnsupportfunctions.h"
#include "arm_nn_tables.h"

#define USE_INTRINSIC

//#define ARM_NN_TRUNCATE /* This config the rounding model to floor or round to the nearest int */

#ifdef __cplusplus
extern    "C"
{
#endif

  /**
   * @brief Basic float32 convolution function (non-sqaure shape)
   * @param[in]       Im_in        pointer to input tensor
   * @param[in]       dim_im_in_x  input tensor dimention x
   * @param[in]       dim_im_in_y  input tensor dimention y
   * @param[in]       ch_im_in     number of input tensor channels
   * @param[in]       wt           pointer to kernel weights
   * @param[in]       ch_im_out    number of filters, i.e., output tensor channels
   * @param[in]       dim_kernel_x filter kernel size x
   * @param[in]       dim_kernel_y filter kernel size y
   * @param[in]       padding_x    padding size x
   * @param[in]       padding_y    padding size y
   * @param[in]       stride_x     convolution stride x
   * @param[in]       stride_y     convolution stride y
   * @param[in]       bias         pointer to bias
   * @param[in,out]   Im_out       pointer to output tensor
   * @param[in]       dim_im_out_x output tensor dimension x
   * @param[in]       dim_im_out_y output tensor dimension y
   * @param[in,out]   bufferA      pointer to buffer space for input
   * @param[in,out]   bufferB      pointer to buffer space for output
   * @return     The function returns <code>ARM_MATH_SUCCESS</code>
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
                                        float * bufferB);


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
                                         q7_t * bufferB);    

  /**
   * @brief Basic Q7 version of CHW convolution (non-sqaure shape)
   * @param[in]       Im_in        pointer to input tensor
   * @param[in]       dim_im_in_x  input tensor dimention x
   * @param[in]       dim_im_in_y  input tensor dimention y
   * @param[in]       ch_im_in     number of input tensor channels
   * @param[in]       wt           pointer to kernel weights
   * @param[in]       ch_im_out    number of filters, i.e., output tensor channels
   * @param[in]       dim_kernel_x filter kernel size x
   * @param[in]       dim_kernel_y filter kernel size y
   * @param[in]       padding_x    padding size x
   * @param[in]       padding_y    padding size y
   * @param[in]       stride_x     convolution stride x
   * @param[in]       stride_y     convolution stride y
   * @param[in]       bias         pointer to bias
   * @param[in]       bias_shift   amount of left-shift for bias
   * @param[in]       out_shift    amount of right-shift for output
   * @param[in,out]   Im_out       pointer to output tensor
   * @param[in]       dim_im_out_x output tensor dimension x
   * @param[in]       dim_im_out_y output tensor dimension y
   * @param[in,out]   bufferA      pointer to buffer space for input
   * @param[in,out]   bufferB      pointer to buffer space for output
   * @return     The function returns <code>ARM_MATH_SUCCESS</code>
   */

    arm_status
    arm_convolve_CHW_q7_basic_nonsquare(const q7_t * Im_in,
                                        const uint16_t dim_im_in_x,
                                        const uint16_t dim_im_in_y,
                                        const uint16_t ch_im_in,
                                        const q7_t * wt,
                                        const uint16_t ch_im_out,
                                        const uint16_t dim_kernel_x,
                                        const uint16_t dim_kernel_y,
                                        const uint16_t padding_x,
                                        const uint16_t padding_y,
                                        const uint16_t stride_x,
                                        const uint16_t stride_y,
                                        const q7_t * bias,
                                        const uint16_t bias_shift,
                                        const uint16_t out_shift,
                                        q7_t * Im_out,
                                        const uint16_t dim_im_out_x,
                                        const uint16_t dim_im_out_y,
                                        q15_t * bufferA,
                                        q7_t * bufferB);

  /**
   * @brief Matrix-multiplication function for convolution with CHW output
   * @param[in]       pA          pointer to operand A
   * @param[in]       pInBuffer   pointer to operand B, always conssists of 2 vectors
   * @param[in]       ch_im_out   numRow of A
   * @param[in]       numCol_A    numCol of A
   * @param[in]       out_stride  output buffer channel stride 
   * @param[in]       bias_shift  amount of left-shift for bias
   * @param[in]       out_shift   amount of right-shift for output
   * @param[in]       bias        the bias
   * @param[in,out]   pOut        pointer to output
   * @return     The function returns the incremented output pointer
   *
   * @details
   *
   * This function does the matrix multiplication with weight matrix
   * and 2 columns from im2col. 
   */

    q7_t     *arm_nn_CHW_mat_mult_kernel_q7_q15(const q7_t * pA,
                                                const q15_t * pInBuffer,
                                                const uint16_t ch_im_out,
                                                const uint16_t numCol_A,
                                                const uint16_t out_stride,
                                                const uint16_t bias_shift,
                                                const uint16_t out_shift, 
                                                const q7_t * bias, 
                                                q7_t * pOut);

#ifdef __cplusplus
}
#endif
#endif
