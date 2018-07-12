/****************************************************************************
 * bsp/include/arch/chip/cisif.h
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

#ifndef __BSP_INCLUDE_ARCH_CHIP_CISIF_H
#define __BSP_INCLUDE_ARCH_CHIP_CISIF_H

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef void (*notify_callback_t)(uint8_t code, uint32_t size, uint32_t addr);
typedef void (*comp_callback_t)(uint8_t code, uint8_t last_frame, uint32_t size, uint32_t addr);

enum format_cisif_e
{
  FORMAT_CISIF_YUV,
  FORMAT_CISIF_JPEG,
  FORMAT_CISIF_INTERLEAVE,
  FORMAT_CISIF_MAX,
};

typedef enum format_cisif_e format_cisif_t;

struct cisif_init_yuv_param_s
{
  uint16_t          hsize;
  uint16_t          vsize;
  uint32_t          notify_size;
  notify_callback_t notify_func;
  comp_callback_t   comp_func;
};

typedef struct cisif_init_yuv_param_s cisif_init_yuv_param_t;

struct cisif_init_jpeg_param_s
{
  uint32_t notify_size;
  notify_callback_t notify_func;
  comp_callback_t comp_func;
};

typedef struct cisif_init_jpeg_param_s cisif_init_jpeg_param_t;

struct cisif_sarea_s
{
  uint8_t *strg_addr;
  uint32_t strg_size;
  uint32_t capnum;
  uint32_t interval;
};

typedef struct cisif_sarea_s cisif_sarea_t;

struct cisif_param_s
{
  format_cisif_t          format;
  cisif_init_yuv_param_t  yuv_param;
  cisif_init_jpeg_param_t jpg_param;
  cisif_sarea_t           sarea;
};

typedef struct cisif_param_s cisif_param_t;

struct cisif_bank_sarea_s
{
  uint8_t *strg_addr_0;
  uint8_t *strg_addr_1;
  uint32_t strg_size;
};

typedef struct cisif_bank_sarea_s cisif_bank_sarea_t;

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int cxd56_cisifinit(void);
int cxd56_cisiffinalize(void);
int cxd56_cisifcaptureframe(
  cisif_param_t *param,
  cisif_sarea_t *yuv_sarea,
  cisif_sarea_t *jpg_sarea);
int cxd56_cisifstartmonitoring(
  cisif_param_t      *param,
  cisif_bank_sarea_t *yuv_area,
  cisif_bank_sarea_t *jpg_area);
int cxd56_cisifstopmonitoring(void);
int cxd56_cisifcontinuouscapture(
  cisif_param_t *param,
  cisif_sarea_t *sarea);

#  undef EXTERN
#  if defined(__cplusplus)
}
#  endif

#endif /* __ASSEMBLY__ */

#endif /* __BSP_INCLUDE_ARCH_CHIP_CISIF_H */
