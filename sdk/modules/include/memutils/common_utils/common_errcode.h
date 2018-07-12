/****************************************************************************
 * modules/include/memutils/common_utils/common_errcode.h
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

#ifndef __SONY_APPS_INCLUDE_MEMUTILS_COMMON_ERRCODE_H
#define __SONY_APPS_INCLUDE_MEMUTILS_COMMON_ERRCODE_H

typedef unsigned int err_t;

/***********************************************************************
 *
 *  [type]
 *
 *   D31   D30  D29  D28  D27  D26  D25  D24  D23  D22  D21  D20  D19  D18  D17  D16
 *  +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *  |REQ |  ERR LEVEL   |    ERR Code       |   ERR SCode_S     |ERR SCode_E/SubCode|
 *  +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *  |        ALL 0                                                                  |
 *  +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *
 *    D31: MSG_TYPE_REQUEST
 *         0x0: OK
 *         0x1: NG
 *
 *    D30-D28: ERR LEVEL
 *         0x0:NoError
 *         0x1:Exception
 *         0x2:Fatal
 *         0x3:Error
 *         0x4:Warning
 *         0x5:Information
 *
 *    D27-D24: ERR Code
 *         0x1:Resource
 *         0x2:Reference
 *         0x3:OS
 *         0x4:Data
 *         0x5:Address
 *         0x6:Event
 *         0x7:Device
 *         0x8:Timeout
 *         0xf:Others
 *
 *    D23-D20: ERR SCode_S(System Call)
 *         0x0:Memory/SEM
 *         0x1:Que/MPF
 *         0x2:Stk/MTX
 *         0x3:PQue/DTQ
 *
 *    D19-D16: ERR SCode_E(Event)
 *         0x0:Create
 *         0x1:Full/Read/Give/Send/Lock
 *         0x2:Empty/Free/Take/Recv/UnLk
 *         0x3:Big
 *         0x4:Small
 *
 *   D19-D16:ERR SubCode
 *         0x0:Data Value
 *         0x1:Data Size
 **********************************************************************
 */

/***********************************************************************
 * Defintions
 ***********************************************************************
 */
#define ERR_OK      0x00000000
#define ERR_NG      0x80000000

/*---------------------------------------------------------------------
 * Error Level
 *   0:NoError
 *   1:Exception
 *   2:Fatal
 *   3:Error
 *   4:Warning
 *   5:Information
 * ---------------------------------------------------------------------
 */
#define ERR_LV_NOERR    0
#define ERR_LV_EXC      1
#define ERR_LV_FATAL    2
#define ERR_LV_ERR      3
#define ERR_LV_WARNING  4
#define ERR_LV_INFO     5

/*---------------------------------------------------------------------
 * Error Code (Category)
 *   0x1:Resource
 *   0x2:Reference
 *   0x3:OS
 *   0x4:Data
 *   0x5:Address
 *   0x6:Event
 *   0x7:Device
 *   0x8:Timeout
 *   0xf:Others
 * ---------------------------------------------------------------------
 */
#define ERR_CODE_RES    0x1
#define ERR_CODE_REF    0x2
#define ERR_CODE_OS     0x3
#define ERR_CODE_DATA   0x4
#define ERR_CODE_ADR    0x5
#define ERR_CODE_EVT    0x6
#define ERR_CODE_DEV    0x7
#define ERR_CODE_TOUT   0x8
#define ERR_CODE_OTHR   0xf

/*---------------------------------------------------------------------
 * Error SubCode System Call
 *   0x00:Memory/SEM
 *   0x01:Que/MPF
 *   0x02:Stk/MTX
 *   0x03:PQue/DTQ
 * ---------------------------------------------------------------------
 */
#define ERR_SCAL_MEM    0x00
#define ERR_SCAL_QUE    0x01
#define ERR_SCAL_STK    0x02
#define ERR_SCAL_PQUE   0x03

#define ERR_SCAL_SEM    0x00
#define ERR_SCAL_MPF    0x01
#define ERR_SCAL_MTX    0x02
#define ERR_SCAL_DTQ    0x03

#define ERR_SCAL_INPUT  0x00 /* For Adr */
#define ERR_SCAL_OUTPUT 0x01 /* For Adr */

#define ERR_SCAL_EER    0x00 /* For Dev */
#define ERR_SCAL_NRDY   0x01 /* For Dev */

/*---------------------------------------------------------------------
 * Error SubCode Error Event
 *   0x00:Create
 *   0x01:Full/Read/Give/Send/Lock
 *   0x02:Empty/Free/Take/Recv/UnLk
 *   0x03:Big
 *   0x04:Small
 *   0x05:Busy
 * ---------------------------------------------------------------------
 */
#define ERR_EVT_FULL    0x01
#define ERR_EVT_EMP     0x02
#define ERR_EVT_BIG     0x03
#define ERR_EVT_SMALL   0x04
#define ERR_EVT_BUSY    0x05

#define ERR_EVT_READ    0x01
#define ERR_EVT_FREE    0x02

#define ERR_EVT_CRE     0x00

#define ERR_EVT_GIVE    0x01
#define ERR_EVT_TAKE    0x02

#define ERR_EVT_SEND    0x01
#define ERR_EVT_RECV    0x02

#define ERR_EVT_LOCK    0x01
#define ERR_EVT_UNLK    0x02

#define ERR_EVT_GET     0x01
#define ERR_EVT_RELEASE 0x02

#define ERR_EVT_ERR     0x00 /* For Dev */

/*---------------------------------------------------------------------
 * Error SubCode
 *   0x00:Data Value
 *   0x01:Data Size
 * ---------------------------------------------------------------------
 */
#define ERR_SCODE_VAL    0x0000
#define ERR_SCODE_SIZE   0x0001

/***********************************************************************
 * Integrated Definitions
 ***********************************************************************
 */
/*** Resource ***/
#define ERR_MEM_FULL    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_MEM < SCAL_NUM) | (ERR_EVT_FULL << EVT_NUM) )
#define ERR_MEM_EMPTY    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_MEM < SCAL_NUM) | (ERR_EVT_EMP << EVT_NUM) )
#define ERR_MEM_BIG    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_MEM < SCAL_NUM) | (ERR_EVT_BIG << EVT_NUM) )
#define ERR_MEM_SMALL    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_MEM < SCAL_NUM) | (ERR_EVT_SMALL << EVT_NUM) )
#define ERR_MEM_BUSY     (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_MEM < SCAL_NUM) | (ERR_EVT_BUSY << EVT_NUM) )

#define ERR_QUE_FULL    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_QUE << SCAL_NUM) | (ERR_EVT_FULL << EVT_NUM) )
#define ERR_QUE_EMP    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES<<CODE_NUM) | \
        (ERR_SCAL_QUE << SCAL_NUM) | (ERR_EVT_EMP << EVT_NUM) )

#define ERR_STK_FULL    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_STK << SCAL_NUM) | (ERR_EVT_FULL << EVT_NUM) )
#define ERR_STK_EMP    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_STK << SCAL_NUM) | (ERR_EVT_EMP << EVT_NUM) )

#define ERR_PQUE_FULL    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_PQUE << SCAL_NUM) | (ERR_EVT_FULL << EVT_NUM) )
#define ERR_PQUE_EMP    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_RES << CODE_NUM) | \
        (ERR_SCAL_PQUE << SCAL_NUM) | (ERR_EVT_EMP << EVT_NUM) )

/*** Reference ***/
#define ERR_MEM_READ    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_REF << CODE_NUM) | \
        (ERR_SCAL_MEM << SCAL_NUM) | (ERR_EVT_READ << EVT_NUM) )
#define ERR_MEM_FREE    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_REF << CODE_NUM) | \
        (ERR_SCAL_MEM << SCAL_NUM) | (ERR_EVT_FREE << EVT_NUM) )

#define ERR_QUE_READ    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_REF << CODE_NUM) | \
        (ERR_SCAL_QUE << SCAL_NUM) | (ERR_EVT_READ << EVT_NUM) )
#define ERR_QUE_FREE    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_REF << CODE_NUM) | \
        (ERR_SCAL_QUE << SCAL_NUM) | (ERR_EVT_FREE << EVT_NUM) )

/*** OS ***/
#define ERR_SEM_CRE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_SEM << SCAL_NUM) | (ERR_EVT_CRE << EVT_NUM) )
#define ERR_SEM_GIVE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_SEM << SCAL_NUM) | (ERR_EVT_GIVE << EVT_NUM) )
#define ERR_SEM_TAKE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_SEM << SCAL_NUM) | (ERR_EVT_TAKE << EVT_NUM) )

#define ERR_MPF_CRE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_MPF << SCAL_NUM) | (ERR_EVT_CRE << EVT_NUM) )
#define ERR_MPF_GET    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_MPF << SCAL_NUM) | (ERR_EVT_GET << EVT_NUM) )
#define ERR_MPF_RELEASE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_MPF << SCAL_NUM) | (ERR_EVT_RELEASE << EVT_NUM) )

#define ERR_MTX_CRE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_MTX << SCAL_NUM) | (ERR_EVT_CRE << EVT_NUM) )
#define ERR_MTX_LOCK    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_MTX << SCAL_NUM) | (ERR_EVT_LOCK << EVT_NUM) )
#define ERR_MTX_UNLK    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_MTX << SCAL_NUM) | (ERR_EVT_UNLK << EVT_NUM) )

#define ERR_DTQ_CRE    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_DTQ << SCAL_NUM) | (ERR_EVT_CRE << EVT_NUM) )
#define ERR_DTQ_SEND    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_DTQ << SCAL_NUM) | (ERR_EVT_SEND << EVT_NUM) )
#define ERR_DTQ_RECV    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_OS << CODE_NUM) | \
        (ERR_SCAL_DTQ << SCAL_NUM) | (ERR_EVT_RECV << EVT_NUM) )

/*** Data ***/
#define ERR_DATA    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_DATA << CODE_NUM) )
#define ERR_DATA_VAL    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_DATA << CODE_NUM) | \
        (ERR_SCODE_VAL << SCODE_NUM) )
#define ERR_DATA_SIZE    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_DATA << CODE_NUM) | \
        (ERR_SCODE_SIZE << SCODE_NUM) )

/*** Address ***/
#define ERR_ADR_INPUT    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_ADR << CODE_NUM) | \
        (ERR_SCAL_INPUT << SCAL_NUM) )
#define ERR_ADR_OUTPUT    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_ADR << CODE_NUM) | \
        (ERR_SCAL_OUTPUT << SCAL_NUM) )
#define ERR_ADR_ALIGN    (ERR_NG | (ERR_LV_WARNING << LV_NUM) | (ERR_CODE_ADR << CODE_NUM) | \
        (ERR_SCAL_INPUT << SCAL_NUM) )

/*** Event ***/
#define ERR_EVT_ILG    (ERR_NG | (ERR_LV_FATAL << LV_NUM) | (ERR_CODE_EVT << CODE_NUM) )
#define ERR_EVT_BAD    (ERR_NG | (ERR_LV_ERR   << LV_NUM) | (ERR_CODE_EVT << CODE_NUM) )

/*** Device ***/
#define ERR_DEV_EXE    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_DEV << CODE_NUM) | \
        (ERR_SCAL_EER << SCAL_NUM) )
#define ERR_DEV_NRDY    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_DEV << CODE_NUM) | \
        (ERR_SCAL_NRDY << SCAL_NUM) )

/*** Timeout ***/
#define ERR_TOUT    (ERR_NG | (ERR_LV_ERR << LV_NUM) | (ERR_CODE_TOUT << CODE_NUM) )

/* ErrorCode Wrapper */
#define ERR_ARG     ERR_DATA_VAL
#define ERR_STS     ERR_EVT_BAD

/*** Other ***/
/***********************************************************************
 * Macros
 ***********************************************************************
 */
#define  isError(x)      (x>>31)
#define  getErrLv(x)     ((x<<1)>>28)
#define  getErrCode(x)   ((x<<4)>>28)
#define  getErrSCal(x)   ((x<<8)>>28)
#define  getErrEvt(x)    ((x<<16)>>28)

#define  setErrLv(x,y)   ((x | (y << LV_NUM)
#define  setErrCode(x)   ((x | (y << CODE_NUM)
#define  setErrSCal(x)   ((x | (y << SCAL_NUM)
#define  setErrEvt(x)    ((x | (y << EVT_NUM)

#define  LV_NUM       28
#define  CODE_NUM     24
#define  SCAL_NUM     20
#define  EVT_NUM      16
#define  SCODE_NUM    16

#endif /* __SONY_APPS_INCLUDE_MEMUTILS_COMMON_ERRCODE_H */
