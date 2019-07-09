/****************************************************************************
 * modules/include/audio/audio_command_id.h
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

#ifndef __INCLUDE_AUDIO_AUDIO_COMMAND_ID_H
#define __INCLUDE_AUDIO_AUDIO_COMMAND_ID_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_command_id Audio Command ID
 * @{
 *
 * @file       audio_command_id.h
 * @brief      CXD5602 Audio Command ID
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*--- Category Code --------------------------------------------------------*/

/** @name Command Category code */
/** @{ */

#define AUDCMD_CATEGORY_GENERAL      0x00
#define AUDCMD_CATEGORY_EFFECTOR     0x10
#define AUDCMD_CATEGORY_PLAYER       0x20
#define AUDCMD_CATEGORY_RECORDER     0x30
#define AUDCMD_CATEGORY_RECOGNITION  0x40
#define AUDCMD_CATEGORY_BASEBAND     0x50
#define AUDCMD_CATEGORY_THROUGH      0x60
#define AUDCMD_CATEGORY_TRANSITION   0x70

#define AUDCMD_CATEGORY_ERRNOTIFY    0xF0

/** @} */

/*--------------------------------------------------------------------------*/
/*--- General Function Code ------------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command General Function code */
/** @{ */

/*! \brief Command Code: GetStatus */

#define AUDCMD_GETSTATUS        (AUDCMD_CATEGORY_GENERAL | 0x01)

/*! \brief Command Code: InitAttentions */

#define AUDCMD_INITATTENTIONS   (AUDCMD_CATEGORY_GENERAL | 0x02)

/** @} */

/** @name General Result code */
/** @{ */

/*! \brief Result Code: NotifyStatus */

#define AUDRLT_NOTIFYSTATUS         AUDCMD_GETSTATUS

/*! \brief Result Code: InitAttentions */

#define AUDRLT_INITATTENTIONSCMPLT  AUDCMD_INITATTENTIONS

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Effector Function Code -----------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Effector Function code */
/** @{ */

/*! \brief Command Code: EnablePreProc */

#define  AUDCMD_SETMFETYPE    (AUDCMD_CATEGORY_EFFECTOR | 0x01)

/*! \brief Command Code: InitMFE */

#define  AUDCMD_INITMFE       (AUDCMD_CATEGORY_EFFECTOR | 0x02)

/*! \brief Command Code: SetMFE */

#define  AUDCMD_SETMFE        (AUDCMD_CATEGORY_EFFECTOR | 0x03)

/*! \brief Command Code: InitMPP */

#define  AUDCMD_INITMPP       (AUDCMD_CATEGORY_EFFECTOR | 0x04)

/*! \brief Command Code: SetMPPParam */

#define  AUDCMD_SETMPPPARAM   (AUDCMD_CATEGORY_EFFECTOR | 0x05)

/** @} */

/** @name Effector Result code */
/** @{ */

/*! \brief Result Code: EnableProProcessCmplt */

#define  AUDRLT_ENPREPROCCMPLT  AUDCMD_SETMFETYPE

/*! \brief Result Code: InitMFECmplt */

#define  AUDRLT_INITMFECMPLT    AUDCMD_INITMFE

/*! \brief Result Code: SetMFECmplt */

#define  AUDRLT_SETMFECMPLT     AUDCMD_SETMFE

/*! \brief Result Code: InitMPPCmplt */

#define  AUDRLT_INITMPPCMPLT    AUDCMD_INITMPP

/*! \brief Result Code: SetMPPParamCmplt */

#define  AUDRLT_SETMPPCMPLT     AUDCMD_SETMPPPARAM

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Player Function Code -------------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Player Function code */
/** @{ */

/*! \brief Command Code: InitPlayer */

#define AUDCMD_INITPLAYER     (AUDCMD_CATEGORY_PLAYER | 0x01)

/*! \brief Command Code: PlayPlayer */

#define AUDCMD_PLAYPLAYER     (AUDCMD_CATEGORY_PLAYER | 0x02)

/*! \brief Command Code: StopPlayer */

#define AUDCMD_STOPPLAYER     (AUDCMD_CATEGORY_PLAYER | 0x03)

/*! \brief Command Code: ClkRecovery */

#define AUDCMD_CLKRECOVERY    (AUDCMD_CATEGORY_PLAYER | 0x04)

/*! \brief Command Code: SetGain */

#define AUDCMD_SETGAIN        (AUDCMD_CATEGORY_PLAYER | 0x05)

/** @} */

/** @name Player Result code */
/** @{ */

/*! \brief Result Code: InitPlayCmplt */

#define  AUDRLT_INITPLAYERCMPLT      AUDCMD_INITPLAYER

/*! \brief Result Code: PlayCmplt */

#define  AUDRLT_PLAYCMPLT            AUDCMD_PLAYPLAYER

/*! \brief Result Code: StopPlayCmplt */

#define  AUDRLT_STOPCMPLT            AUDCMD_STOPPLAYER

/*! \brief Result Code: ClkRecoveryComplete */

#define  AUDRLT_CLKRECOVERY_CMPLT    AUDCMD_CLKRECOVERY

/*! \brief Result Code: SetGainComplete */

#define  AUDRLT_SETGAIN_CMPLT        AUDCMD_SETGAIN

/*! \brief Result Code: SendPfCommandComplete */

#define  AUDRLT_SENDPFCMD_CMPLT      AUDCMD_SENDPOSTCMD

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Recorder Function Code -----------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Recorder Function code */
/** @{ */

/*! \brief Command Code: InitRecorder */

#define AUDCMD_INITREC    (AUDCMD_CATEGORY_RECORDER | 0x01)

/*! \brief Command Code: StartRec */

#define AUDCMD_STARTREC   (AUDCMD_CATEGORY_RECORDER | 0x02)

/*! \brief Command Code: StopRec */

#define AUDCMD_STOPREC    (AUDCMD_CATEGORY_RECORDER | 0x03)

/** @} */

/** @name Recoder Result code */
/** @{ */

/*! \brief Result Code: RecCmplt */

#define AUDRLT_RECCMPLT      AUDCMD_STARTREC

/*! \brief Result Code: StopRecCmplt */

#define AUDRLT_STOPRECCMPLT  AUDCMD_STOPREC

/*! \brief Result Code: InitRecCmplt */

#define AUDRLT_INITRECCMPLT  AUDCMD_INITREC

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Recognition Function Code --------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Recognition Function code */
/** @{ */

/*! \brief Command Code: StartRecognizer */

#define AUDCMD_STARTRECOGNIZER  (AUDCMD_CATEGORY_RECOGNITION | 0x01)

/*! \brief Command Code: StopRecognizer */

#define AUDCMD_STOPRECOGNIZER   (AUDCMD_CATEGORY_RECOGNITION | 0x02)

/*! \brief command Code: InitRecognizer */

#define AUDCMD_INITRECOGNIZER   (AUDCMD_CATEGORY_RECOGNITION | 0x03)

/*! \brief command Code: InitRecognitionProcessDSP */

#define AUDCMD_INITRECOGNIZERPROC (AUDCMD_CATEGORY_RECOGNITION | 0x04)

/*! \brief command Code: SetRecognitionProcessDSP */

#define AUDCMD_SETRECOGNIZERPROC  (AUDCMD_CATEGORY_RECOGNITION | 0x05)

/** @} */

/* result code */

/** @name Result code */
/** @{ */

/*! \brief Result Code: StartRecognizerCmplt */

#define  AUDRLT_STARTRECOGNIZERCMPLT  AUDCMD_STARTRECOGNIZER

/*! \brief Result Code: StopRecognizerCmplt */

#define  AUDRLT_STOPRECOGNIZERCMPLT   AUDCMD_STOPRECOGNIZER

/*! \brief Result Code: InitRecognizerCmplt */

#define  AUDRLT_INITRECOGNIZERCMPLT   AUDCMD_INITRECOGNIZER

/*! \brief command Code: InitRecognitionProcessDSPCmplt */

#define  AUDRLT_INITRECOGNIZERPROCCMPLT AUDCMD_INITRECOGNIZERPROC

/*! \brief command Code: SetRecognitionProcessDSPCmplt */

#define  AUDRLT_SETRECOGNIZERPROCCMPLT  AUDCMD_SETRECOGNIZERPROC

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Baseband Function Code -----------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Baseband Function code */
/** @{ */

/*! \brief Command Code: StartBB (__not supported__) */

#define AUDCMD_STARTBB          (AUDCMD_CATEGORY_BASEBAND | 0x01)

/*! \brief Command Code: StopBB (__not supported__) */

#define AUDCMD_STOPBB           (AUDCMD_CATEGORY_BASEBAND | 0x02)

/*! \brief Command Code: InitMicGain */

#define AUDCMD_INITMICGAIN      (AUDCMD_CATEGORY_BASEBAND | 0x03)

/*! \brief Command Code: InitI2SParam (__not supported__) */

#define AUDCMD_INITI2SPARAM     (AUDCMD_CATEGORY_BASEBAND | 0x04)

/*! \brief Command Code: InitDEQParam (__not supported__) */

#define AUDCMD_INITDEQPARAM     (AUDCMD_CATEGORY_BASEBAND | 0x05)

/*! \brief Command Code: InitOutputSelect */

#define AUDCMD_INITOUTPUTSELECT (AUDCMD_CATEGORY_BASEBAND | 0x06)

/*! \brief Command Code: InitDNCParam (__not supported__) */

#define AUDCMD_INITDNCPARAM     (AUDCMD_CATEGORY_BASEBAND | 0x07)

/*! \brief Command Code: InitClearStereo (__not supported__) */

#define AUDCMD_INITCLEARSTEREO  (AUDCMD_CATEGORY_BASEBAND | 0x08)

/*! \brief Command Code: SetVolume */

#define AUDCMD_SETVOLUME        (AUDCMD_CATEGORY_BASEBAND | 0x09)

/*! \brief Command Code: SetVolumeMute */

#define AUDCMD_SETVOLUMEMUTE    (AUDCMD_CATEGORY_BASEBAND | 0x0a)

/*! \brief Command Code: SetBeep */

#define AUDCMD_SETBEEPPARAM     (AUDCMD_CATEGORY_BASEBAND | 0x0b)

/*! \brief Command Code: SetRenderingClk */

#define AUDCMD_SETRENDERINGCLK  (AUDCMD_CATEGORY_BASEBAND | 0x0c)

/*! \brief Command Code: SetSpDrvMode */

#define AUDCMD_SETSPDRVMODE     (AUDCMD_CATEGORY_BASEBAND | 0x0d)

/*! \brief Command Code: SetMicMap */

#define AUDCMD_SETMICMAP        (AUDCMD_CATEGORY_BASEBAND | 0x0e)

/** @} */

/** @name Baseband Result code */
/** @{ */

/*! \brief Result Code: StartBBCmplt (__not supported__) */

#define AUDRLT_STARTBBCMPLT             AUDCMD_STARTBB

/*! \brief Result Code: StopBBCmplt (__not supported__) */

#define AUDRLT_STOPBBCMPLT              AUDCMD_STOPBB

/*! \brief Result Code: InitMicGainCmplt */

#define AUDRLT_INITMICGAINCMPLT         AUDCMD_INITMICGAIN

/*! \brief Result Code: InitMicGainCmplt */

#define AUDRLT_SETMICMAPCMPLT           AUDCMD_SETMICMAP

/*! \brief Result Code: InitI2SCmplt (__not supported__) */

#define AUDRLT_INITI2SPARAMCMPLT        AUDCMD_INITI2SPARAM

/*! \brief Result Code: InitDEQCmplt (__not supported__) */

#define AUDRLT_INITDEQPARAMCMPLT        AUDCMD_INITDEQPARAM

/*! \brief Result Code: InitOutputSelectCmplt */

#define AUDRLT_INITOUTPUTSELECTCMPLT    AUDCMD_INITOUTPUTSELECT

/*! \brief Result Code: InitDNCCmplt (__not supported__) */

#define AUDRLT_INITDNCPARAMCMPLT        AUDCMD_INITDNCPARAM

/*! \brief Result Code: InitClearStereoCmplt (__not supported__) */

#define AUDRLT_INITCLEARSTEREOCMPLT     AUDCMD_INITCLEARSTEREO

/*! \brief Result Code: SetVolumeCmplt */

#define AUDRLT_SETVOLUMECMPLT           AUDCMD_SETVOLUME

/*! \brief Result Code: SetVolumeMuteCmplt */

#define AUDRLT_SETVOLUMEMUTECMPLT       AUDCMD_SETVOLUMEMUTE

/*! \brief Result Code: SetBeepCmplt */

#define AUDRLT_SETBEEPCMPLT             AUDCMD_SETBEEPPARAM

/*! \brief Result Code: SetRenderingClkCmplt */

#define AUDRLT_SETRENDERINGCLKCMPLT     AUDCMD_SETRENDERINGCLK

/*! \brief Result Code: SetSpDrvModeCmplt */

#define AUDRLT_SETSPDRVMODECMPLT        AUDCMD_SETSPDRVMODE

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Through Function Code ------------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Through Function code */
/** @{ */

/*! \brief Command Code: SetThroughPath */

#define AUDCMD_SETTHROUGHPATH   (AUDCMD_CATEGORY_THROUGH | 0x01)

/** @} */

/** @name Through Result code */
/** @{ */

/*! \brief Result Code: SetThroughPathCmplt */

#define AUDRLT_SETTHROUGHPATHCMPLT       AUDCMD_SETTHROUGHPATH

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Transition Function Code ---------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Command Transition Function code */
/** @{ */

/*! \brief Command Code: PowerOn */

#define AUDCMD_POWERON              (AUDCMD_CATEGORY_TRANSITION | 0x01)

/*! \brief Command Code: SetPowerOffStatus */

#define AUDCMD_SETPOWEROFFSTATUS    (AUDCMD_CATEGORY_TRANSITION | 0x02)

/*! \brief Command Code: SetBaseBandStatus (__not supported__) */

#define AUDCMD_SETBASEBANDSTATUS    (AUDCMD_CATEGORY_TRANSITION | 0x03)

/*! \brief Command Code: SetPlayerStatus */

#define AUDCMD_SETPLAYERSTATUS      (AUDCMD_CATEGORY_TRANSITION | 0x04)

/*! \brief Command Code: SetRecorderStatus */

#define AUDCMD_SETRECORDERSTATUS    (AUDCMD_CATEGORY_TRANSITION | 0x05)

/*! \brief Command Code: SetReadyStartus */

#define AUDCMD_SETREADYSTATUS       (AUDCMD_CATEGORY_TRANSITION | 0x06)

/*! \brief Command Code: SetThroughStartus */

#define AUDCMD_SETTHROUGHSTATUS     (AUDCMD_CATEGORY_TRANSITION | 0x07)

/*! \brief Command Code: SetPlayerStatus */

#define AUDCMD_SETPLAYERSTATUSPOST  (AUDCMD_CATEGORY_TRANSITION | 0x08)

/*! \brief Command Code: SetRecognizerStatus */

#define AUDCMD_SETRECOGNIZERSTATUS  (AUDCMD_CATEGORY_TRANSITION | 0x09)

/** @} */

/** @name Transition Result code */
/** @{ */

/*! \brief Result Code: StatusChanged */

#define AUDRLT_STATUSCHANGED        (AUDCMD_CATEGORY_TRANSITION | 0x0f)

/** @} */

/*--------------------------------------------------------------------------*/
/*--- Error Notification Code ----------------------------------------------*/
/*--------------------------------------------------------------------------*/

/** @name Error Notification code */
/** @{ */

/*! \brief Result Code: ErrorResponse */

#define AUDRLT_ERRORRESPONSE          (AUDCMD_CATEGORY_ERRNOTIFY | 0x01)

/*! \brief Result Code: ErrorAttention */

#define AUDRLT_ERRORATTENTION         (AUDCMD_CATEGORY_ERRNOTIFY | 0x02)

/** @} */

#endif /* __INCLUDE_AUDIO_AUDIO_COMMAND_ID_H */

/**
 * @}
 */
/**
 * @}
 */

