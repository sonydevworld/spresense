/****************************************************************************
 * modules/include/audio/audio_message_types.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_MESSAGE_TYPE_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_MESSAGE_TYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/message_type.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/************************************************************************
 *
 *  request / response
 *
 ************************************************************************
 */

#define MSG_TYPE_AUD_RES  (MSG_TYPE_RESPONSE | MSG_TYPE_USER_AUDIO_UTIL)
#define MSG_TYPE_AUD_REQ  (MSG_TYPE_REQUEST  | MSG_TYPE_USER_AUDIO_UTIL)

/************************************************************************
 *
 *  category
 *
 ************************************************************************
 */

#define MSG_CAT_AUD_MNG           (MSG_SET_CATEGORY(0x0))
#define MSG_CAT_AUD_ISR           (MSG_SET_CATEGORY(0x1))
#define MSG_CAT_AUD_PLY           (MSG_SET_CATEGORY(0x2))
#define MSG_CAT_AUD_SEF           (MSG_SET_CATEGORY(0x3))
#define MSG_CAT_AUD_RCG           (MSG_SET_CATEGORY(0x4))
#define MSG_CAT_AUD_MRC           (MSG_SET_CATEGORY(0x5))
#define MSG_CAT_AUD_SNK           (MSG_SET_CATEGORY(0x6))
#define MSG_CAT_AUD_MIX           (MSG_SET_CATEGORY(0x7))
#define MSG_CAT_AUD_MIX_SEF       (MSG_SET_CATEGORY(0x8))
#define MSG_CAT_AUD_CAP           (MSG_SET_CATEGORY(0x9))
#define MSG_CAT_AUD_MFE           (MSG_SET_CATEGORY(0xA))
#define MSG_CAT_AUD_SYN           (MSG_SET_CATEGORY(0xB))
#define MSG_CAT_AUD_BB            (MSG_SET_CATEGORY(0xC))

/************************************************************************
 *
 * Object Base Sub Type
 *
 ************************************************************************
 */

#define MSG_OBJ_SUBTYPE_ACT      (0x00)
#define MSG_OBJ_SUBTYPE_DEACT    (0x01)
#define MSG_OBJ_SUBTYPE_INIT     (0x02)
#define MSG_OBJ_SUBTYPE_START    (0x03)
#define MSG_OBJ_SUBTYPE_STOP     (0x04)
#define MSG_OBJ_SUBTYPE_EXEC     (0x05)
#define MSG_OBJ_SUBTYPE_SET      (0x06)
#define MSG_OBJ_SUBTYPE_NUM      (MSG_OBJ_SUBTYPE_SET + 1)

/************************************************************************
 *
 *    MSG_CAT_AUD_MNG: Audio Manager Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |  MSG_CAT_MNG  | MSG_SUB_TYPE      | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_MNG_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_MNG)
#define MSG_AUD_MNG_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_MNG)

#define MSG_AUD_MGR_CMD_PLAYER           (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_MGR_CMD_SETREADY         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_MGR_CMD_SETBASEBAND      (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_MGR_CMD_SETPLAYER        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_MGR_CMD_SETRECOGNIZER    (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_MGR_CMD_RECOGNIZER       (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x05))
#define MSG_AUD_MGR_CMD_SETRECORDER      (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x06))
#define MSG_AUD_MGR_CMD_RECORDER         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x07))
#define MSG_AUD_MGR_CMD_SOUNDFX          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x08))
#define MSG_AUD_MGR_CMD_MFE              (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x09))
#define MSG_AUD_MGR_CMD_MPP              (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0a))
#define MSG_AUD_MGR_CMD_GETSTATUS        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0b))
#define MSG_AUD_MGR_CMD_SETMICMAP        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0c))
#define MSG_AUD_MGR_CMD_INITMICGAIN      (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0d))
#define MSG_AUD_MGR_CMD_INITDEQPARAM     (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0e))
#define MSG_AUD_MGR_CMD_INITOUTPUTSELECT (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0f))
#define MSG_AUD_MGR_CMD_INITDNCPARAM     (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x10))
#define MSG_AUD_MGR_CMD_INITCLEARSTEREO  (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x11))
#define MSG_AUD_MGR_CMD_SETVOLUME        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x12))
#define MSG_AUD_MGR_CMD_SETVOLUMEMUTE    (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x13))
#define MSG_AUD_MGR_CMD_SETBEEP          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x14))
#define MSG_AUD_MGR_CMD_POWERON          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x15))
#define MSG_AUD_MGR_CMD_POWEROFF         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x16))
#define MSG_AUD_MGR_CMD_OUTPUTMIXER      (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x17))
#define MSG_AUD_MGR_CMD_SETTHROUGH       (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x18))
#define MSG_AUD_MGR_CMD_SETTHROUGHPATH   (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x19))
#define MSG_AUD_MGR_CMD_SETRENDERINGCLK  (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x1a))
#define MSG_AUD_MGR_CMD_INVALID          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x1b))
#define MSG_AUD_MGR_SETSPDRVMODE         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x1c))

#define LAST_AUD_MGR_MSG     (MSG_AUD_MGR_SETSPDRVMODE + 1)
#define AUD_MGR_MSG_NUM      (LAST_AUD_MGR_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_MGR_CALL_ATTENTION  (MSG_AUD_MNG_RES | MSG_SET_SUBTYPE(0x01))

/************************************************************************
 *
 *    MSG_CAT_ISR:  Interrupt (response)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_ISR| MSG_SUB_TYPE      | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_ISR    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_ISR)

#define MSG_ISR_DMA_CMPLT  (MSG_ISR | MSG_SET_SUBTYPE(0x01))
#define MSG_ISR_DMA_ERROR  (MSG_ISR | MSG_SET_SUBTYPE(0x02))
#define MSG_ISR_AC_INTRPT  (MSG_ISR | MSG_SET_SUBTYPE(0x03))
#define MSG_ISR_BUS_ERROR  (MSG_ISR | MSG_SET_SUBTYPE(0x04))
#define MSG_ISR_APU0       (MSG_ISR | MSG_SET_SUBTYPE(0x10))
#define MSG_ISR_APU1       (MSG_ISR | MSG_SET_SUBTYPE(0x11))
#define MSG_ISR_APU2       (MSG_ISR | MSG_SET_SUBTYPE(0x12))

/************************************************************************
 *
 *    MSG_CAT_AUD_PLY: Audio Player Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_PLY|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_PLY_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_PLY)
#define MSG_AUD_PLY_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_PLY)

#define MSG_AUD_PLY_CMD_ACT             (MSG_AUD_PLY_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_PLY_CMD_INIT            (MSG_AUD_PLY_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_PLY_CMD_PLAY            (MSG_AUD_PLY_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_PLY_CMD_STOP            (MSG_AUD_PLY_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_PLY_CMD_DEACT           (MSG_AUD_PLY_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_PLY_CMD_SETGAIN         (MSG_AUD_PLY_REQ | MSG_SET_SUBTYPE(0x05))

#define LAST_AUD_PLY_MSG    (MSG_AUD_PLY_CMD_SETGAIN + 1)
#define AUD_PLY_MSG_NUM     (LAST_AUD_PLY_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_PLY_CMD_NEXT_REQ        (MSG_AUD_PLY_RES | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_PLY_CMD_DEC_DONE        (MSG_AUD_PLY_RES | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_PLY_CMD_DEC_SET_DONE    (MSG_AUD_PLY_RES | MSG_SET_SUBTYPE(0x02))

#define LAST_AUD_PLY_RST_MSG    (MSG_AUD_PLY_CMD_DEC_SET_DONE + 1)
#define AUD_PLY_RST_MSG_NUM     (LAST_AUD_PLY_RST_MSG & MSG_TYPE_SUBTYPE)

/************************************************************************
 *
 *    MSG_CAT_AUD_BB: Audio Baseband Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_BB |   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_BB_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_BB)
#define MSG_AUD_BB_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_BB)

#define MSG_AUD_BB_CMD_ACT          (MSG_AUD_BB_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_BB_CMD_DEACT        (MSG_AUD_BB_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_BB_CMD_INIT         (MSG_AUD_BB_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_BB_CMD_RUN          (MSG_AUD_BB_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_BB_CMD_STOP         (MSG_AUD_BB_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_BB_CMD_CMPLT        (MSG_AUD_BB_REQ | MSG_SET_SUBTYPE(0x05))

#define LAST_AUD_BB_MSG     (MSG_AUD_BB_CMD_CMPLT + 1)
#define AUD_BB_MSG_NUM      (LAST_AUD_BB_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_BB_RST      (MSG_AUD_BB_RES | MSG_SET_SUBTYPE(0x00))

/************************************************************************
 *
 *    MSG_CAT_AUD_RCG: sound recognition Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_RCG|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_RCG_REQ           (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_RCG)
#define MSG_AUD_RCG_RES           (MSG_TYPE_AUD_RES | MSG_CAT_AUD_RCG)

#define MSG_AUD_RCG_ACT           (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_ACT))
#define MSG_AUD_RCG_DEACT         (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_DEACT))
#define MSG_AUD_RCG_INIT          (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_INIT))
#define MSG_AUD_RCG_START         (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_START))
#define MSG_AUD_RCG_STOP          (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_STOP))
#define MSG_AUD_RCG_EXEC          (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_EXEC))
#define MSG_AUD_RCG_INITRCGPROC   (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(0x06))
#define MSG_AUD_RCG_SETRCGPROC    (MSG_AUD_RCG_REQ | MSG_SET_SUBTYPE(0x07))

#define LAST_AUD_RCG_REQ_MSG      (MSG_AUD_RCG_SETRCGPROC)
#define AUD_RCG_REQ_MSG_NUM       (MSG_GET_SUBTYPE(LAST_AUD_RCG_REQ_MSG) + 1)

#define MSG_AUD_RCG_RCG_CMPLT     (MSG_AUD_RCG_RES | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_RCG_FIND_TRIGGER  (MSG_AUD_RCG_RES | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_RCG_FIND_COMMAND  (MSG_AUD_RCG_RES | MSG_SET_SUBTYPE(0x02))

#define LAST_AUDRCG_RES_MSG       (MSG_AUD_RCG_RCG_CMPLT)
#define AUD_RCG_RES_MSG_NUM       (MSG_GET_SUBTYPE(LAST_AUDRCG_RES_MSG) + 1)

/************************************************************************
 *
 *    MSG_CAT_AUD_MRC: Media recorder Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_MRC|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_MRC_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_MRC)
#define MSG_AUD_MRC_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_MRC)

#define MSG_AUD_MRC_CMD_ACTIVATE    (MSG_AUD_MRC_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_ACT))
#define MSG_AUD_MRC_CMD_DEACTIVATE  (MSG_AUD_MRC_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_DEACT))
#define MSG_AUD_MRC_CMD_INIT        (MSG_AUD_MRC_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_INIT))
#define MSG_AUD_MRC_CMD_START       (MSG_AUD_MRC_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_START))
#define MSG_AUD_MRC_CMD_STOP        (MSG_AUD_MRC_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_STOP))
#define MSG_AUD_MRC_CMD_ENCODE      (MSG_AUD_MRC_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_EXEC))

#define LAST_AUD_MRC_MSG    (MSG_AUD_MRC_CMD_ENCODE)
#define AUD_MRC_MSG_NUM     (MSG_GET_SUBTYPE(LAST_AUD_MRC_MSG) + 1)

#define MSG_AUD_MRC_RST_FILTER      (MSG_AUD_MRC_RES | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_MRC_RST_ENC         (MSG_AUD_MRC_RES | MSG_SET_SUBTYPE(0x01))

#define LAST_AUD_MRC_RST_MSG    (MSG_AUD_MRC_RST_ENC)
#define AUD_MRC_RST_MSG_NUM     (MSG_GET_SUBTYPE(LAST_AUD_MRC_RST_MSG) + 1)

/************************************************************************
 *
 *    MSG_CAT_AUD_SNK: Media recorder data sink Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_SNK|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_SNK_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_SNK)
#define MSG_AUD_SNK_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_SNK)

#define MSG_AUD_SNK_INIT   (MSG_AUD_SNK_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_SNK_DATA   (MSG_AUD_SNK_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_SNK_STOP   (MSG_AUD_SNK_REQ | MSG_SET_SUBTYPE(0x02))

#define LAST_AUD_SNK_MSG   (MSG_AUD_SNK_STOP + 1)
#define AUD_SNK_MSG_NUM    (LAST_AUD_SNK_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_SNK_RST    (MSG_AUD_SNK_RES | MSG_SET_SUBTYPE(0x00))

/************************************************************************
 *
 *    MSG_CAT_AUD_MIX: Output mix Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_MIX|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_MIX_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_MIX)
#define MSG_AUD_MIX_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_MIX)

#define MSG_AUD_MIX_CMD_ACT         (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_ACT))
#define MSG_AUD_MIX_CMD_DEACT       (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_DEACT))
#define MSG_AUD_MIX_CMD_INIT        (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_INIT))
#define MSG_AUD_MIX_CMD_DATA        (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_MIX_CMD_CLKRECOVERY (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_MIX_CMD_INITMPP     (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(0x05))
#define MSG_AUD_MIX_CMD_SETMPP      (MSG_AUD_MIX_REQ | MSG_SET_SUBTYPE(0x06))


#define LAST_AUD_MIX_MSG             MSG_AUD_MIX_CMD_SETMPP
#define AUD_MIX_MSG_NUM    (MSG_GET_SUBTYPE(LAST_AUD_MIX_MSG) + 1)

#define MSG_AUD_MIX_RST_PSTFLT_DONE (MSG_AUD_MIX_RES | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_MIX_RST_RENDER_DONE (MSG_AUD_MIX_RES | MSG_SET_SUBTYPE(0x01))

#define LAST_AUD_MIX_RST_MSG    (MSG_AUD_MIX_RST_RENDER_DONE)
#define AUD_MIX_RST_MSG_NUM     (MSG_GET_SUBTYPE(LAST_AUD_MIX_RST_MSG) + 1)

/************************************************************************
 *
 *    MSG_CAT_AUD_SEF: Sound-effect Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_SEF| MSG_SUB_TYPE      | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_SEF_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_SEF)
#define MSG_AUD_SEF_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_SEF)

#define MSG_AUD_SEF_CMD_ACT          (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_SEF_CMD_DEACT        (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_SEF_CMD_INIT         (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_SEF_CMD_START        (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_SEF_CMD_STOP         (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_SEF_CMD_INPUT        (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x05))
#define MSG_AUD_SEF_CMD_FILTER_DATA  (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x06))
#define MSG_AUD_SEF_CMD_DMA_OUT_DONE (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x07))
#define MSG_AUD_SEF_CMD_SETPARAM     (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x08))
#define MSG_AUD_SEF_CMD_CMPLT        (MSG_AUD_SEF_REQ | MSG_SET_SUBTYPE(0x09))

#define LAST_AUD_SEF_MSG    (MSG_AUD_SEF_CMD_CMPLT + 1)
#define AUD_SEF_MSG_NUM     (LAST_AUD_SEF_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_SEF_RST     (MSG_AUD_SEF_RES | MSG_SET_SUBTYPE(0x00))

/************************************************************************
 *
 *    MSG_CAT_AUD_CAP: Audio Capture Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_CAP|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_CAP_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_CAP)
#define MSG_AUD_CAP_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_CAP)

#define MSG_AUD_CAP_CMD_ACT          (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_CAP_CMD_DEACT        (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_CAP_CMD_INIT         (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_CAP_CMD_RUN          (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_CAP_CMD_STOP         (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_CAP_CMD_SETMICGAIN   (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x05))
#define MSG_AUD_CAP_CMD_CMPLT        (MSG_AUD_CAP_REQ | MSG_SET_SUBTYPE(0x06))

#define LAST_AUD_CAP_MSG     (MSG_AUD_CAP_CMD_CMPLT + 1)
#define AUD_CAP_MSG_NUM      (LAST_AUD_CAP_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_CAP_RST      (MSG_AUD_CAP_RES | MSG_SET_SUBTYPE(0x00))

/************************************************************************
 *
 *    MSG_CAT_AUD_MFE: Mic Frontend Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_MFE|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_MFE_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_MFE)
#define MSG_AUD_MFE_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_MFE)

#define MSG_AUD_MFE_PRM_INITPREPROC  (0x00)
#define MSG_AUD_MFE_PRM_SETPREPROC   (0x01)
#define MSG_AUD_MFE_PRM_SETMICGAIN   (0x02)

#define MSG_AUD_MFE_CMD_ACT          (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_ACT))
#define MSG_AUD_MFE_CMD_DEACT        (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_DEACT))
#define MSG_AUD_MFE_CMD_INIT         (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_INIT))
#define MSG_AUD_MFE_CMD_START        (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_START))
#define MSG_AUD_MFE_CMD_STOP         (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_STOP))
#define MSG_AUD_MFE_CMD_INITPREPROC  (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_SET) | MSG_AUD_MFE_PRM_INITPREPROC)
#define MSG_AUD_MFE_CMD_SETPREPROC   (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_SET) | MSG_AUD_MFE_PRM_SETPREPROC)
#define MSG_AUD_MFE_CMD_SETMICGAIN   (MSG_AUD_MFE_REQ | MSG_SET_SUBTYPE(MSG_OBJ_SUBTYPE_SET) | MSG_AUD_MFE_PRM_SETMICGAIN)

#define LAST_AUD_MFE_MSG    (MSG_AUD_MFE_CMD_SETMICGAIN)
#define AUD_MFE_MSG_NUM     (MSG_GET_SUBTYPE(LAST_AUD_MFE_MSG) + 1)

#define LAST_AUD_MFE_PRM    (MSG_AUD_MFE_PRM_SETMICGAIN)
#define AUD_MFE_PRM_NUM     (MSG_GET_PARAM(LAST_AUD_MFE_PRM) + 1)

#define MSG_AUD_MFE_RST_CAPTURE_DONE (MSG_AUD_MFE_RES | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_MFE_RST_CAPTURE_ERR  (MSG_AUD_MFE_RES | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_MFE_RST_PREPROC      (MSG_AUD_MFE_RES | MSG_SET_SUBTYPE(0x02))

#define LAST_AUD_MFE_RST_MSG    (MSG_AUD_MFE_RST_PREPROC)
#define AUD_MFE_RST_MSG_NUM     (MSG_GET_SUBTYPE(LAST_AUD_MFE_RST_MSG) + 1)

/************************************************************************
 *
 *    MSG_CAT_AUD_SYN: Synthesizer Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUDIO |MSG_CAT_AUD_SYN|   MSG_SUB_TYPE    | MSG_PARAM |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_SYN_REQ    (MSG_TYPE_AUD_REQ | MSG_CAT_AUD_SYN)
#define MSG_AUD_SYN_RES    (MSG_TYPE_AUD_RES | MSG_CAT_AUD_SYN)

#define MSG_AUD_SYN_CMD_ACT             (MSG_AUD_SYN_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_SYN_CMD_INIT            (MSG_AUD_SYN_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_SYN_CMD_START           (MSG_AUD_SYN_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_SYN_CMD_STOP            (MSG_AUD_SYN_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_SYN_CMD_DEACT           (MSG_AUD_SYN_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_SYN_CMD_SET             (MSG_AUD_SYN_REQ | MSG_SET_SUBTYPE(0x05))

#define LAST_AUD_SYN_MSG    (MSG_AUD_SYN_CMD_SET + 1)
#define AUD_SYN_MSG_NUM     (LAST_AUD_SYN_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_SYN_CMD_NEXT_REQ        (MSG_AUD_SYN_RES | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_SYN_CMD_DONE            (MSG_AUD_SYN_RES | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_SYN_CMD_SET_DONE        (MSG_AUD_SYN_RES | MSG_SET_SUBTYPE(0x02))

#define LAST_AUD_SYN_RST_MSG    (MSG_AUD_SYN_CMD_SET_DONE + 1)
#define AUD_SYN_RST_MSG_NUM     (LAST_AUD_SYN_RST_MSG & MSG_TYPE_SUBTYPE)



/************************************************************************
 *
 *  responce from AudioSubSystem
 *
 ************************************************************************
 */

#define MSG_AUD_MGR_RST             (MSG_TYPE_AUD_RES)
#define MSG_AUD_MGR_FIND_TRIGGER    (MSG_AUD_RCG_FIND_TRIGGER)
#define MSG_AUD_MGR_FIND_COMMAND    (MSG_AUD_RCG_FIND_COMMAND)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_INCLUDE_AUDIO_AUDIO_MESSAGE_TYPE_H */
