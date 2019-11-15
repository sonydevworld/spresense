/****************************************************************************
 * modules/audio/dsp_driver/include/dsp_drv.h
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

#ifndef __MODULES_AUDIO_DSP_DRIVER_INCLUDE_DSP_DRV_H
#define __MODULES_AUDIO_DSP_DRIVER_INCLUDE_DSP_DRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <asmp/asmp.h>
#include <asmp/mptask.h>
#include <asmp/mpmq.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSP_COM_DATA_TYPE_STRUCT_ADDRESS  0
#define DSP_COM_DATA_TYPE_32BIT_VALUE     1

typedef enum {
  DSPDRV_NOERROR = 0,       /* Dsp driver load success.                */
  DSPDRV_FILENAME_EMPTY,    /* No file name.                           */
  DSPDRV_CALLBACK_ERROR,    /* Callback function value type error.     */
  DSPDRV_INVALID_VALUE,     /* Invalid argument.                       */
  DSPDRV_CREATE_FAIL,       /* Failed to Create DspDrv Class instance. */
  DSPDRV_INIT_MPTASK_FAIL,  /* Failed on DspDrv Init mptask sequence.  */
  DSPDRV_INIT_MPMQ_FAIL,    /* Failed on DspDrv Init mpmq sequence.    */
  DSPDRV_INIT_PTHREAD_FAIL  /* Failed on DspDrv Init pthread sequence. */
} dspdrv_errorcode_e;

typedef enum
{
  DspBinTypeELF = 0,   /* ELF format.                        */
  DspBinTypeELFwoBind, /* ELF format without ASMP bind area. */
  DspBinTypeSPK        /* SPK format.                        */
} dsp_bin_type_e;

struct DspDrvComPrm_s
{
  uint32_t process_mode:4;  /* Mode of process. A use case is a case where
                             * DSP has 2 or more processes. */
  uint32_t event_type:3;    /* Type of event. Set event type such as INIT,
                             * EXEC or Flash. */
  uint32_t type:1;          /* Type of data. Set a whether data is a
                             * pointer(0) or a value(1). */
  union
  {
    FAR void *pParam;     /* Set address of parameter. */
    uint32_t value;       /* Set value of parameter. */
  } data;
};
typedef struct DspDrvComPrm_s DspDrvComPrm_t;

/* Type of callback function. */

typedef void (*DspDoneCallback)(FAR void *, FAR void *);

#ifdef __cplusplus
class DspDrv
{
public:
  int init(FAR const char *pfilename,
           DspDoneCallback p_cbfunc,
           FAR void        *p_parent_instance,
           dsp_bin_type_e  bintype);
  int destroy(bool force);
  int send(FAR const DspDrvComPrm_t *p_param);
  int receive();

  DspDrv() {}
  ~DspDrv() {}

private:
  mptask_t  m_mptask;
  mpmq_t    m_mq;

  DspDoneCallback m_p_cb_func;
  pthread_t m_thread_id;

  FAR void *m_p_parent_instance;
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern int DD_Load(FAR const char  *filename,
                   DspDoneCallback p_cbfunc,
                   FAR void        *p_parent_instance,
                   FAR void        **dsp_handler,
                   dsp_bin_type_e  bintype);

extern int DD_Load_Secure(FAR const char  *filename,
                          DspDoneCallback p_cbfunc,
                          FAR void        *p_parent_instance,
                          FAR void        **dsp_handler);

extern int DD_SendCommand(FAR const void           *p_instance,
                          FAR const DspDrvComPrm_t *p_param);

extern int DD_Unload(FAR const void *p_instance);

extern int DD_force_Unload(FAR const void *p_instance);

#endif /* __MODULES_AUDIO_DSP_DRIVER_INCLUDE_DSP_DRV_H */
