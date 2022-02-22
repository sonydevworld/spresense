/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_rfft_fast_init_f32.c
 * Description:  Split Radix Decimation in Frequency CFFT Floating point processing function
 *
 * $Date:        23 April 2021
 * $Revision:    V1.9.0
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2021 ARM Limited or its affiliates. All rights reserved.
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

#include "dsp/transform_functions.h"
#include "arm_common_tables.h"
#include "arm_const_structs.h"

#define FFTINIT(EXT,SIZE)                                           \
  Sint->bitRevLength = arm_cfft_sR_##EXT##_len##SIZE.bitRevLength;  \
  Sint->pBitRevTable = arm_cfft_sR_##EXT##_len##SIZE.pBitRevTable;  \
  Sint->pTwiddle = arm_cfft_sR_##EXT##_len##SIZE.pTwiddle;

/**
  @ingroup groupTransforms
 */

/**
  @addtogroup RealFFT
  @{
 */

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_16) && defined(ARM_TABLE_BITREVIDX_FLT_16) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_32))

/**
  @brief         Initialization function for the 32pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */

arm_status arm_rfft_32_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 16U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,16);

  S->fftLenRFFT = 32U;
  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_32;

  return ARM_MATH_SUCCESS;
}
#endif 

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_32) && defined(ARM_TABLE_BITREVIDX_FLT_32) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_64))

/**
  @brief         Initialization function for the 64pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */

arm_status arm_rfft_64_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 32U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,32);

  S->fftLenRFFT = 64U;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_64;

  return ARM_MATH_SUCCESS;
}
#endif 

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_64) && defined(ARM_TABLE_BITREVIDX_FLT_64) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_128))

/**
  @brief         Initialization function for the 128pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */

arm_status arm_rfft_128_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 64U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,64);

  S->fftLenRFFT = 128;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_128;

  return ARM_MATH_SUCCESS;
}
#endif 

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_128) && defined(ARM_TABLE_BITREVIDX_FLT_128) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_256))

/**
  @brief         Initialization function for the 256pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
*/

arm_status arm_rfft_256_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 128U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,128);

  S->fftLenRFFT = 256U;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_256;

  return ARM_MATH_SUCCESS;
}
#endif 

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_256) && defined(ARM_TABLE_BITREVIDX_FLT_256) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_512))

/**
  @brief         Initialization function for the 512pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */

arm_status arm_rfft_512_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 256U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,256);

  S->fftLenRFFT = 512U;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_512;

  return ARM_MATH_SUCCESS;
}
#endif 

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_512) && defined(ARM_TABLE_BITREVIDX_FLT_512) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_1024))
/**
  @brief         Initialization function for the 1024pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */

arm_status arm_rfft_1024_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 512U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,512);

  S->fftLenRFFT = 1024U;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_1024;

  return ARM_MATH_SUCCESS;
}
#endif

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_1024) && defined(ARM_TABLE_BITREVIDX_FLT_1024) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_2048))
/**
  @brief         Initialization function for the 2048pt floating-point real FFT.
  @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */
arm_status arm_rfft_2048_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 1024U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,1024);

  S->fftLenRFFT = 2048U;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_2048;

  return ARM_MATH_SUCCESS;
}
#endif

#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_2048) && defined(ARM_TABLE_BITREVIDX_FLT_2048) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_4096))
/**
* @brief         Initialization function for the 4096pt floating-point real FFT.
* @param[in,out] S  points to an arm_rfft_fast_instance_f32 structure
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : an error is detected
 */

arm_status arm_rfft_4096_fast_init_f32( arm_rfft_fast_instance_f32 * S ) {

  if( !S ) return ARM_MATH_ARGUMENT_ERROR;

  /* Initialize parameters without arm_cfft_init_f32 to reduce code size */

  arm_cfft_instance_f32 * Sint;

  Sint = &(S->Sint);

  /*  Initialise the FFT length */
  Sint->fftLen = 2048U;

  /*  Initialise the bit reversal table modifier */
  FFTINIT(f32,2048);

  S->fftLenRFFT = 4096U;

  S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_4096;

  return ARM_MATH_SUCCESS;
}
#endif 

/**
  @brief         Initialization function for the floating-point real FFT.
  @param[in,out] S       points to an arm_rfft_fast_instance_f32 structure
  @param[in]     fftLen  length of the Real Sequence
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : Operation successful
                   - \ref ARM_MATH_ARGUMENT_ERROR : <code>fftLen</code> is not a supported length

  @par           Description
                   The parameter <code>fftLen</code> specifies the length of RFFT/CIFFT process.
                   Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096.
  @par
                   This Function also initializes Twiddle factor table pointer and Bit reversal table pointer.
 */

arm_status arm_rfft_fast_init_f32(
  arm_rfft_fast_instance_f32 * S,
  uint16_t fftLen)
{
  typedef arm_status(*fft_init_ptr)( arm_rfft_fast_instance_f32 *);
  fft_init_ptr fptr = 0x0;

  switch (fftLen)
  {
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_2048) && defined(ARM_TABLE_BITREVIDX_FLT_2048) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_4096))
  case 4096U:
    fptr = arm_rfft_4096_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_1024) && defined(ARM_TABLE_BITREVIDX_FLT_1024) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_2048))
  case 2048U:
    fptr = arm_rfft_2048_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_512) && defined(ARM_TABLE_BITREVIDX_FLT_512) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_1024))
  case 1024U:
    fptr = arm_rfft_1024_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_256) && defined(ARM_TABLE_BITREVIDX_FLT_256) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_512))
  case 512U:
    fptr = arm_rfft_512_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_128) && defined(ARM_TABLE_BITREVIDX_FLT_128) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_256))
  case 256U:
    fptr = arm_rfft_256_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_64) && defined(ARM_TABLE_BITREVIDX_FLT_64) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_128))
  case 128U:
    fptr = arm_rfft_128_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_32) && defined(ARM_TABLE_BITREVIDX_FLT_32) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_64))
  case 64U:
    fptr = arm_rfft_64_fast_init_f32;
    break;
#endif
#if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FFT_TABLES) || (defined(ARM_TABLE_TWIDDLECOEF_F32_16) && defined(ARM_TABLE_BITREVIDX_FLT_16) && defined(ARM_TABLE_TWIDDLECOEF_RFFT_F32_32))
  case 32U:
    fptr = arm_rfft_32_fast_init_f32;
    break;
#endif
  default:
    return ARM_MATH_ARGUMENT_ERROR;
  }

  if( ! fptr ) return ARM_MATH_ARGUMENT_ERROR;
  return fptr( S );

}

/**
  @} end of RealFFT group
 */
