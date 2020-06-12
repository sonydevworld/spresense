/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv_api.h
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

#ifndef __MODULES_AUDIO_DMA_CONTROLLER_BCA_DRV_H
#define __MODULES_AUDIO_DMA_CONTROLLER_BCA_DRV_H

#include <arch/chip/audio.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** BaseBand driver common return code */
typedef enum
{
  E_AS_OK,                                /* 0 */
  /* Power OFF/ON */
  E_AS_PWON_NG,                           /* 1 */
  /* Power OFF/ON chk */
  E_AS_POWER_OFF_CHK_ERR,                 /* 2 */
  E_AS_IN_POWER_ON_CHK_ERR,               /* 3 */
  E_AS_OUT_POWER_ON_CHK_ERR,              /* 4 */
  E_AS_CS_POWER_ON_CHK_ERR,               /* 5 */
  E_AS_DEQ_POWER_ON_CHK_ERR,              /* 6 */
  E_AS_DNC_POWER_ON_CHK_ERR,              /* 7 */
  E_AS_MIC_POWER_ON_CHK_ERR,              /* 8 */
  E_AS_OUTSEL_POWER_ON_CHK_ERR,           /* 9 */
  E_AS_I2S_POWER_ON_CHK_ERR,              /* 10 */
  E_AS_SETVOL_POWER_ON_OUT_CHK_ERR,       /* 11 */
  E_AS_MUTE_POWER_ON_OUT_CHK_ERR,         /* 12 */
  E_AS_UNMUTE_POWER_ON_OUT_CHK_ERR,       /* 13 */
  E_AS_SETBEEP_POWER_ON_OUT_CHK_ERR,      /* 14 */
  E_AS_BEEPDIS_POWER_ON_OUT_CHK_ERR,      /* 15 */
  E_AS_BEEPENA_POWER_ON_OUT_CHK_ERR,      /* 16 */
  E_AS_SET_DP_POWER_ON_CHK_ERR,           /* 17 */
  E_AS_CLR_DP_POWER_ON_CHK_ERR,           /* 18 */
  E_AS_CLRALL_DP_POWER_ON_CHK_ERR,        /* 19 */
  E_AS_ACTDMAC_POWER_ON_CHK_ERR,          /* 20 */
  E_AS_INITDMAC_POWER_ON_CHK_ERR,         /* 21 */
  E_AS_STARTDMAC_POWER_ON_CHK_ERR,        /* 22 */
  E_AS_READDMAC_POWER_ON_CHK_ERR,         /* 23 */
  E_AS_WRITEDMAC_POWER_ON_CHK_ERR,        /* 24 */
  E_AS_STOPDMAC_POWER_ON_CHK_ERR,         /* 25 */
  E_AS_GETREADYCMDDMAC_POWER_ON_CHK_ERR,  /* 26 */
  E_AS_DEACTDMAC_POWER_ON_CHK_ERR,        /* 27 */

  /* PowerManager ON chk */
  E_AS_PM_ADONIS_PWON_CHK_ERR,            /* 28 */
  E_AS_PM_AUDIO_PWON_CHK_ERR,             /* 29 */
  /* Audio Codec ID */
  E_AS_AC_ID_NG,                          /* 30 */
  /* AcaPulco ID */
  E_AS_ACAPULCO_ID_NG,                    /* 31 */
  /* BaseBandConfig */
  E_AS_AD_DATA_DS_PARAM,                  /* 32 */
  E_AS_ALC_SPC_SEL_PARAM,                 /* 33 */
  E_AS_ALC_KNEE_PARAM,                    /* 34 */
  E_AS_ALC_TARGET_PARAM,                  /* 35 */
  E_AS_SPC_LIMIT_PARAM,                   /* 36 */
  E_AS_CIC_INPUT_SEL_PARAM,               /* 37 */
  E_AS_CLK_MODE_PARAM,                    /* 38 */
  E_AS_DMIC_CLK_DS_PARAM,                 /* 39 */
  E_AS_GPO_DS_PARAM,                      /* 40 */
  E_AS_I2S_DATA_PATH_PARAM,               /* 41 */
  E_AS_I2S_DEVICE_1_PARAM,                /* 42 */
  E_AS_I2S_FORMAT_1_PARAM,                /* 43 */
  E_AS_I2S_DEVICE_2_PARAM,                /* 44 */
  E_AS_I2S_FORMAT_2_PARAM,                /* 45 */
  E_AS_I2S_LOWEMI_PARAM,                  /* 46 */
  E_AS_LOAD_NG,                           /* 47 */
  E_AS_MCLK_DS_PARAM,                     /* 48 */
  E_AS_MIC_BIAS_SEL_PARAM,                /* 49 */
  E_AS_MIC_CHANNEL_SEL_PARAM,             /* 50 */
  E_AS_PDM_LOWEMI_PARAM,                  /* 51 */
  E_AS_XTAL_CLKMODE_PARAM,                /* 52 */
  E_AS_XTAL_SEL_PARAM,                    /* 53 */
  E_AS_BBENABLE_NG,                       /* 54 */
  /* BEEP */
  E_AS_SETBEEP_OUTDEV_ERR,                /* 55 */
  E_AS_BEEPENA_OUTDEV_ERR,                /* 56 */
  E_AS_BEEPDIS_OUTDEV_ERR,                /* 57 */
  E_AS_BEEP_FREQ_PARAM,                   /* 58 */
  E_AS_BEEP_VOL_PARAM,                    /* 59 */
  /* I2S */
  E_AS_I2S_RATE_PARAM,                    /* 60 */
  E_AS_SRC_BYPASS_ERR,                    /* 61 */
  /* Codec Volume */
  E_AS_SETVOL_OUTDEV_ERR,                 /* 62 */
  E_AS_MUTE_OUTDEV_ERR,                   /* 63 */
  E_AS_UNMUTE_OUTDEV_ERR,                 /* 64 */
  E_AS_CODEC_VOL_NULL,                    /* 65 */
  E_AS_CODEC_VOL_ID_PARAM,                /* 66 */
  E_AS_CODEC_VOL_PARAM,                   /* 67 */
  /* Audio Codec ClearStereo */
  E_AS_CS_EN_PARAM,                       /* 68 */
  E_AS_CS_SIGN_PARAM,                     /* 69 */
  E_AS_CS_VOL_PARAM,                      /* 70 */
  /* DMAC IF */
  E_AS_DMAC_ACTIVATED_ERR,                /* 71 */
  E_AS_DMAC_BUSY,                         /* 72 */
  E_AS_DMAC_CRE_TASK_ERR,                 /* 73 */
  E_AS_DMAC_DEACTIVATED_ERR,              /* 74 */
  E_AS_DMAC_DEL_TASK_ERR,                 /* 75 */
  E_AS_DMAC_ERR_START,                    /* 76 */
  E_AS_DMAC_ID_PARAM,                     /* 77 */
  E_AS_DMAC_MSG_SEND_ERR,                 /* 78 */
  E_AS_DMAC_SAMPLING_FMT_PARAM,           /* 79 */
  E_AS_DMAC_SIZE_MAX_ERR,                 /* 80 */
  E_AS_DMAC_SIZE_MIN_ERR,                 /* 81 */
  E_AS_DMAC_TRANS_ADDR_NULL,              /* 82 */
  E_AS_INITDMAC_NULL,                     /* 83 */
  E_AS_READDMAC_NULL,                     /* 84 */
  E_AS_WRITEDMAC_NULL,                    /* 85 */
  E_AS_GETREADYCMD_RESULT_NULL,           /* 86 */

  /* DEQ */
  E_AS_DEQ_UNSUPPORT,                     /* 87 */
  /* DNC */
  E_AS_DNC_UNSUPPORT,                     /* 88 */
  E_AS_DNC_SEL_PARAM,                     /* 89 */
  /* MIC */
  E_AS_MICGAIN_PARAM,                     /* 90 */
  E_AS_MICMODE_PARAM,                     /* 91 */
  E_AS_MICGAIN_NULL,                      /* 92 */
  /* Output Device */
  E_AS_OUT_DEVICE_SEL_PARAM,              /* 93 */
  /* Audio Codec data path */
  E_AS_PATH_SEL_COMBINATION_NG,           /* 94 */
  E_AS_PATH_SEL_DMACID_ERR,               /* 95 */
  E_AS_PATH_SEL_DMACID_NULL,              /* 96 */
  E_AS_PATH_SEL_DMACID_PARAM,             /* 97 */
  E_AS_PATH_SEL_FROM_PARAM,               /* 98 */
  E_AS_PATH_SEL_TO_PARAM,                 /* 99 */
  E_AS_PATH_SEL_MIC_DMA_CHANNEL_PARAM,    /* 100 */
  E_AS_PATH_SEL_NOUSE_ERR,                /* 101 */
  E_AS_PATH_SEL_NULL,                     /* 102 */
  E_AS_PATH_SEL_USED_ERR                  /* 103 */
} E_AS;

/** DMAC interrupt notify code */
typedef enum
{
  E_AS_DMA_INT_CMPLT,  /* nomal end */
  E_AS_DMA_INT_ERR,    /* illegal end */
  E_AS_DMA_INT_ERR_BUS /* bus error */
} E_AS_DMA_INT;

#define DMAC_MAX_SIZE     4096
#define DMAC_MIN_SIZE_POL 32
#define DMAC_MIN_SIZE_INT 240


#define AS_DMAC_BYTE_WT_24BIT 4
#define AS_DMAC_BYTE_WT_16BIT 2

#define AS_DMAC_CMD_BUF_STATUS_EMPTY 3

#define BUSY_LOOP(cnt)                    \
{                                         \
  volatile unsigned int tmp_value = 1;    \
  int loop = 0;                           \
  for (loop=0 ; loop<(int)(cnt) ; loop++) \
    {                                     \
      tmp_value++;                        \
    }                                     \
}

typedef enum
{
  AS_DMATRNSCMPLT_NONE,
  AS_DMATRNSCMPLT_OK,
  AS_DMATRNSCMPLT_MAX_ENTRY
} asDmacTrnsCmpltResult;

typedef enum
{
  AS_DMATRNSSTOP_INCMPLT,
  AS_DMATRNSSTOP_CMPLT,
  AS_DMATRNSSTOP_MAX_ENTRY
} asDmacTrnsStopStatus;

/** DMAC transfer result code */
typedef enum
{
  E_AS_BB_DMA_OK,          /* nomal end */
  E_AS_BB_DMA_ILLEGAL,     /* illegal end */
  E_AS_BB_DMA_ERR_INT,     /* interrupt error */
  E_AS_BB_DMA_UNDERFLOW,   /* underflow */
  E_AS_BB_DMA_OVERFLOW,    /* overflow */
  E_AS_BB_DMA_ERR_REQUEST, /* request error */
  E_AS_BB_DMA_PARAM,       /* parameter error */
  E_AS_BB_DMA_ERR_START,   /* transfer start error */
  E_AS_BB_DMA_ERR_BUS      /* bus error */
} E_AS_BB;

/** DMAC State */
typedef enum
{
  AS_DMA_STATE_BOOTED,   /* BOOTED */
  AS_DMA_STATE_STOP,     /* STOP */
  AS_DMA_STATE_READY,    /* READY */
  AS_DMA_STATE_PREPARE,  /* PREPARE */
  AS_DMA_STATE_RUN,      /* RUN */
  AS_DMA_STATE_FLUSH,    /* FLUSH */
  AS_DMA_STATE_ERROR,    /* ERROR */
  AS_DMA_STATE_TERMINATE,/* TERMINATE */
  AS_DMA_STATE_MAX_ENTRY /* MAX ENTRY */
} asDmaState;

/** AS_ErrorCb callback function parameter */
typedef struct AudioDrvDmaError_
{
  cxd56_audio_dma_t dmac_id; /* [in] Error DMAC ID */
  E_AS_BB     status;  /* [in] Error fact */
  asDmaState  state;   /* [in] DMAC state */
} AudioDrvDmaError;

/** AS_DmaDoneCb callback function parameter */
typedef struct AudioDrvDmaResult_
{
  E_AS_BB     result;  /* [in] DMAC transfer result */
  bool        endflg;  /* [in] DMAC transfer end data flg */
  cxd56_audio_dma_t dmac_id; /* [in] DMAC ID */
  uint32_t    addr1;   /* [in] DMAC transfer data address1 */
  uint16_t    size1;   /* [in] DMAC transfer data size1 */
  uint32_t    addr2;   /* [in] DMAC transfer data address2 */
  uint16_t    size2;   /* [in] DMAC transfer data size2 */
} AudioDrvDmaResult;

/** DMAC transfer error callback function */
typedef void (* AS_ErrorCb)(AudioDrvDmaError *pParam);
/** DMAC transfer done callback function */
typedef void (* AS_DmaDoneCb)(AudioDrvDmaResult *pParam);

/** #AS_InitDmac function parameter */
typedef struct
{
  cxd56_audio_dma_t      dmacId; /* [in] DMAC ID */
  cxd56_audio_samp_fmt_t format; /* [in] data format */
  uint8_t                ch_num; /* [in] transfer channel num */
  AS_ErrorCb   p_error_func;     /* [in] DMAC transfer error callback */
  AS_DmaDoneCb p_dmadone_func;   /* [in] DMAC transfer done callback */
  bool         fade_en;          /* [in] auto fade mode, TRUE:ENABLE */
} asInitDmacParam;

/**
 * @brief Init DMAC
 *
 * @param[in] asInitDmacParam* Init DMAC parameter
 *
 * @retval E_AS return code
 */
E_AS AS_InitDmac(asInitDmacParam *pInitDmacParam);

/**
 * @brief Start DMAC
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 *
 * @retval E_AS return code
 */
E_AS AS_StartDmac(cxd56_audio_dma_t dmacId);


/** #AS_ReadDmac and #AS_WriteDmac function parameter */
typedef struct
{
  cxd56_audio_dma_t dmacId;   /* [in] DMAC ID */
  uint32_t    addr;     /* [in] DMAC transfer data address */
  uint16_t    size;     /* [in] DMAC transfer data size */
  uint32_t    addr2;    /* [in] DMAC transfer data address2 */
  uint16_t    size2;    /* [in] DMAC transfer data size2 */
  bool        validity; /* [in] Frame validity (true:valid) */
} asReadDmacParam, asWriteDmacParam;

/**
 * @brief Read DMAC
 *
 * @param[in] asReadDmacParam* Read DMAC parameter
 *
 * @retval E_AS return code
 */
E_AS AS_ReadDmac(asReadDmacParam *pReadDmacParam);

/**
 * @brief Write DMAC
 *
 * @param[in] asWriteDmacParam* Write DMAC parameter
 *
 * @retval E_AS return code
 */
E_AS AS_WriteDmac(asWriteDmacParam *pWriteDmacParam);


/** Select DMAC stop mode */
typedef enum
{
  AS_DMASTOPMODE_NORMAL,    /* NORMAL */
  AS_DMASTOPMODE_IMMEDIATE, /* IMMEDIATE */
  AS_DMASTOPMODE_MAX_ENTRY  /* MAX ENTRY */
} asDmacStopMode;

/**
 * @brief Stop DMAC
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 * @param[in] asDmacStopMode Stop mode
 *
 * @retval E_AS return code
 */
E_AS AS_StopDmac(cxd56_audio_dma_t dmacId, asDmacStopMode stopMode);

/**
 * @brief Get numbers of the DMAC ready command
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 * @param[out] uint32_t* Numbers of the DMAC ready command
 *
 * @retval E_AS return code
 */
E_AS AS_GetReadyCmdNumDmac(cxd56_audio_dma_t dmacId, uint32_t *pResult);

/**
 * @brief Regist DMA callback from interrupt handler
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 * @param[in] cxd56_audio_dma_cb_t DMA interrupt handler callback function pointer
 *
 * @retval E_AS return code
 */
E_AS AS_RegistDmaIntCb(cxd56_audio_dma_t dmacId, cxd56_audio_dma_cb_t p_dmaIntCb);

/**
 * @brief Notify DMA completed
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 * @param[in] code type of interrupt
 *
 * @retval E_AS return code
 */
E_AS AS_NotifyDmaCmplt(cxd56_audio_dma_t dmacId, E_AS_DMA_INT code);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __MODULES_AUDIO_DMA_CONTROLLER_BCA_DRV_H */
