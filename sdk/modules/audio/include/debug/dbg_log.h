/****************************************************************************
 * modules/audio/include/debug/dbg_log.h
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

#ifndef __MODULES_AUDIO_INCLUDE_DEBUG_DEBUG_LOG_H
#define __MODULES_AUDIO_INCLUDE_DEBUG_DEBUG_LOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <sdk/config.h>
#include <debug.h>
#include "audio/audio_high_level_api.h"
#include "attention.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_AUDIOUTILS_EVENTLOG
#define AUDIO_LOG_EVENT(id, fmt, ...) syslog(LOG_DEBUG, \
                                             "<%2d>" fmt, \
                                             id, \
                                             ##__VA_ARGS__)
#else
#define AUDIO_LOG_EVENT(id, fmt, ...)
#endif

#ifdef CONFIG_AUDIOUTILS_STATELOG
#define AUDIO_LOG_STATE(id, fmt, ...) syslog(LOG_DEBUG, \
                                             "<%2ld>" fmt, \
                                             id, \
                                             ##__VA_ARGS__)
#else
#define AUDIO_LOG_STATE(id, fmt, ...)
#endif

#ifdef CONFIG_AUDIOUTILS_DETAILLOG
#define AUDIO_LOG_DETAIL(id, fmt, ...) syslog(LOG_DEBUG, \
                                              "<%2d>" fmt, \
  id, ##__VA_ARGS__)
#else
#define AUDIO_LOG_DETAIL(id, fmt, ...)
#endif

#define DBG_LOGF_FATAL(id, sid, code)  FATAL_ATTENTION(id, sid, code)
#define DBG_LOGF_ERROR(id, sid, code)  ERROR_ATTENTION(id, sid, code)
#define DBG_LOGF_WARN(id, sid, code)   WARNING_ATTENTION(id, sid, code)
#define DBG_LOGF_DEBUG(id, sid, code)  _info("DBG : %d %d %d %s(L%d)\n", \
                                       id, sid, code, __FILE__, __LINE__)

#define MANAGER_FATAL(code) DBG_LOGF_FATAL(AS_MODULE_ID_AUDIO_MANAGER, 0, code)
#define MANAGER_ERR(code)   DBG_LOGF_ERROR(AS_MODULE_ID_AUDIO_MANAGER, 0, code)
#define MANAGER_WARN(code)  DBG_LOGF_WARN(AS_MODULE_ID_AUDIO_MANAGER, 0, code)
#define MANAGER_INF(code)   DBG_LOGF_DEBUG(AS_MODULE_ID_AUDIO_MANAGER, 0, code)

#define MEDIA_PLAYER_REG_ATTCB(att_cb)  ATTENTION_CB_REGISTER( \
                                          AS_MODULE_ID_PLAYER_OBJ, \
                                          att_cb)
#define MEDIA_PLAYER_UNREG_ATTCB() ATTENTION_CB_UNREGISTER( \
                                     AS_MODULE_ID_PLAYER_OBJ)

#define MEDIA_PLAYERS_FATAL(sub_mod_id, code)  DBG_LOGF_FATAL( \
                                                AS_MODULE_ID_PLAYER_OBJ, \
                                                sub_mod_id, \
                                                code)
#define MEDIA_PLAYERS_ERR(sub_mod_id, code)    DBG_LOGF_ERROR( \
                                                AS_MODULE_ID_PLAYER_OBJ, \
                                                sub_mod_id, \
                                                code)
#define MEDIA_PLAYERS_WARN(sub_mod_id, code)   DBG_LOGF_WARN( \
                                                AS_MODULE_ID_PLAYER_OBJ, \
                                                sub_mod_id, \
                                                code)
#define MEDIA_PLAYERS_INF(sub_mod_id, code)    DBG_LOGF_DEBUG( \
                                                AS_MODULE_ID_PLAYER_OBJ, \
                                                sub_mod_id, \
                                                code)
#define MEDIA_PLAYER_DBG(fmt, ...)  AUDIO_LOG_EVENT( \
                                      AS_MODULE_ID_PLAYER_OBJ, \
                                      fmt, \
                                      ##__VA_ARGS__)
#define MEDIA_PLAYER_VDBG(fmt, ...) AUDIO_LOG_DETAIL( \
                                      AS_MODULE_ID_PLAYER_OBJ, \
                                      fmt, \
                                      ##__VA_ARGS__)

#define OUTPUT_MIX_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER( \
                                       AS_MODULE_ID_OUTPUT_MIX_OBJ, \
                                       att_cb)
#define OUTPUT_MIX_UNREG_ATTCB() ATTENTION_CB_UNREGISTER( \
                                       AS_MODULE_ID_OUTPUT_MIX_OBJ)

#define OUTPUT_MIX_FATAL(code)   DBG_LOGF_FATAL( \
                                   AS_MODULE_ID_OUTPUT_MIX_OBJ, \
                                   0, \
                                   code)
#define OUTPUT_MIX_ERR(code)     DBG_LOGF_ERROR( \
                                   AS_MODULE_ID_OUTPUT_MIX_OBJ, \
                                   0, \
                                   code)
#define OUTPUT_MIX_WARN(code)    DBG_LOGF_WARN( \
                                   AS_MODULE_ID_OUTPUT_MIX_OBJ, \
                                   0, \
                                   code)
#define OUTPUT_MIX_INF(code)     DBG_LOGF_DEBUG( \
                                   AS_MODULE_ID_OUTPUT_MIX_OBJ, \
                                   0, \
                                   code)
#define OUTPUT_MIX_DBG(fmt, ...) AUDIO_LOG_EVENT( \
                                   AS_MODULE_ID_OUTPUT_MIX_OBJ, \
                                   fmt, \
                                   ##__VA_ARGS__)

#define RENDERER_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_RENDERER_CMP, \
                                                         att_cb)
#define RENDERER_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_RENDERER_CMP)

#define RENDERER_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_RENDERER_CMP, \
                                              0, \
                                              code)
#define RENDERER_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_RENDERER_CMP, \
                                              0, \
                                              code)
#define RENDERER_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_RENDERER_CMP, \
                                             0, \
                                             code)
#define RENDERER_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_RENDERER_CMP, \
                                              0, \
                                              code)
#define RENDERER_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_RENDERER_CMP, \
                                               fmt, \
                                               ##__VA_ARGS__)

#define MEDIA_RECORDER_REG_ATTCB(att_cb)  ATTENTION_CB_REGISTER( \
                                            AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                            att_cb)
#define MEDIA_RECORDER_UNREG_ATTCB()  ATTENTION_CB_UNREGISTER( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ)

#define MEDIA_RECORDER_FATAL(code)    DBG_LOGF_FATAL( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                        0, \
                                        code)
#define MEDIA_RECORDER_ERR(code)      DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                        0, \
                                        code)
#define MEDIA_RECORDER_WARN(code)     DBG_LOGF_WARN( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                        0, \
                                        code)
#define MEDIA_RECORDER_INF(code)      DBG_LOGF_DEBUG( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                        0, \
                                        code)
#define MEDIA_RECORDER_DBG(fmt, ...)  AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                        fmt, \
                                        ##__VA_ARGS__)
#define MEDIA_RECORDER_VDBG(fmt, ...) AUDIO_LOG_DETAIL( \
                                        AS_MODULE_ID_MEDIA_RECORDER_OBJ, \
                                        fmt, \
                                        ##__VA_ARGS__)

#define MIC_FRONTEND_REG_ATTCB(att_cb)  ATTENTION_CB_REGISTER( \
                                            AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                            att_cb)
#define MIC_FRONTEND_UNREG_ATTCB()  ATTENTION_CB_UNREGISTER( \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ)

#define MIC_FRONTEND_FATAL(code)    DBG_LOGF_FATAL( \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                        0, \
                                        code)
#define MIC_FRONTEND_ERR(code)      DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                        0, \
                                        code)
#define MIC_FRONTEND_WARN(code)     DBG_LOGF_WARN( \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                        0, \
                                        code)
#define MIC_FRONTEND_INF(code)      DBG_LOGF_DEBUG( \
                                        0, \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                        code)
#define MIC_FRONTEND_DBG(fmt, ...)  AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                        fmt, \
                                        ##__VA_ARGS__)
#define MIC_FRONTEND_VDBG(fmt, ...) AUDIO_LOG_DETAIL( \
                                        AS_MODULE_ID_MIC_FRONTEND_OBJ, \
                                        fmt, \
                                        ##__VA_ARGS__)

#define DECODER_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_DECODER_CMP, \
                                                        att_cb)
#define DECODER_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_DECODER_CMP)

#define DECODER_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_DECODER_CMP, \
                                             0, \
                                             code)
#define DECODER_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_DECODER_CMP, \
                                             0, \
                                             code)
#define DECODER_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_DECODER_CMP, \
                                            0, \
                                            code)
#define DECODER_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_DECODER_CMP, \
                                             0, \
                                             code)
#define DECODER_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_DECODER_CMP, \
                                              fmt, \
                                              ##__VA_ARGS__)

#define DECODER_VDBG(fmt, ...) AUDIO_LOG_DETAIL(AS_MODULE_ID_DECODER_CMP, \
                                                fmt, \
                                                ##__VA_ARGS__)

#define ENCODER_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_ENCODER_CMP, \
                                                        att_cb)
#define ENCODER_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_ENCODER_CMP)

#define ENCODER_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_ENCODER_CMP, \
                                             0, \
                                             code)
#define ENCODER_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_ENCODER_CMP, \
                                             0, \
                                             code)
#define ENCODER_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_ENCODER_CMP, \
                                            0, \
                                            code)
#define ENCODER_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_ENCODER_CMP, \
                                             0, \
                                             code)
#define ENCODER_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_ENCODER_CMP, \
                                              fmt, \
                                              ##__VA_ARGS__)

#define CAPTURE_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_CAPTURE_CMP, \
                                                        att_cb)
#define CAPTURE_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_CAPTURE_CMP)

#define CAPTURE_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_CAPTURE_CMP, \
                                             0, \
                                             code)
#define CAPTURE_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_CAPTURE_CMP, \
                                             0, \
                                             code)
#define CAPTURE_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_CAPTURE_CMP, \
                                            0, \
                                            code)
#define CAPTURE_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_CAPTURE_CMP, \
                                             0, \
                                             code)
#define CAPTURE_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_CAPTURE_CMP, \
                                              fmt, \
                                              ##__VA_ARGS__)

#define FILTER_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_FILTER_CMP, \
                                                       att_cb)
#define FILTER_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_FILTER_CMP)

#define FILTER_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_FILTER_CMP, \
                                            0, \
                                            code)
#define FILTER_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_FILTER_CMP, \
                                            0, \
                                            code)
#define FILTER_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_FILTER_CMP, \
                                           0, \
                                           code)
#define FILTER_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_FILTER_CMP, \
                                            0, \
                                            code)
#define FILTER_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_FILTER_CMP, \
                                             fmt, \
                                             ##__VA_ARGS__)

#define POSTPROC_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_POSTPROC_CMP, \
                                                           att_cb)
#define POSTPROC_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_POSTPROC_CMP)

#define POSTPROC_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_POSTPROC_CMP, \
                                              0, \
                                              code)
#define POSTPROC_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_POSTPROC_CMP, \
                                              0, \
                                              code)
#define POSTPROC_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_POSTPROC_CMP, \
                                             0, \
                                             code)
#define POSTPROC_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_POSTPROC_CMP, \
                                              0, \
                                              code)
#define POSTPROC_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_POSTPROC_CMP, \
                                             fmt, \
                                             ##__VA_ARGS__)

#define SOUNDFX_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_SOUND_EFFECT_OBJ, \
                                                        att_cb)
#define SOUNDFX_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_SOUND_EFFECT_OBJ)

#define SOUNDFX_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_SOUND_EFFECT_OBJ, \
                                             0, \
                                             code)
#define SOUNDFX_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_SOUND_EFFECT_OBJ, \
                                             0, \
                                             code)
#define SOUNDFX_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_SOUND_EFFECT_OBJ, \
                                            0, \
                                            code)
#define SOUNDFX_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_SOUND_EFFECT_OBJ, \
                                             0, \
                                             code)
#define SOUNDFX_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_SOUND_EFFECT_OBJ, \
                                              fmt, \
                                              ##__VA_ARGS__)

#define RECOGNIZER_OBJ_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER( \
                                            AS_MODULE_ID_RECOGNITION_OBJ, \
                                            att_cb)
#define RECOGNIZER_OBJ_UNREG_ATTCB() ATTENTION_CB_UNREGISTER( \
                                            AS_MODULE_ID_RECOGNITION_OBJ)

#define RECOGNIZER_OBJ_FATAL(code)   DBG_LOGF_FATAL( \
                                        AS_MODULE_ID_RECOGNITION_OBJ, \
                                        0, \
                                        code)
#define RECOGNIZER_OBJ_ERR(code)     DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_RECOGNITION_OBJ, \
                                        0, \
                                        code)
#define RECOGNIZER_OBJ_WARN(code)    DBG_LOGF_WARN( \
                                        AS_MODULE_ID_RECOGNITION_OBJ, \
                                        0, \
                                        code)
#define RECOGNIZER_OBJ_INF(code)     DBG_LOGF_DEBUG( \
                                        AS_MODULE_ID_RECOGNITION_OBJ, \
                                        0, \
                                        code)
#define RECOGNIZER_OBJ_DBG(fmt, ...) AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_RECOGNITION_OBJ, \
                                        fmt, \
                                        ##__VA_ARGS__)
#define SYNTHESIZER_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER( \
                                        AS_MODULE_ID_SYNTHESIZER_OBJ, \
                                        att_cb)
#define SYNTHESIZER_OBJ_ERR(code)     DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_SYNTHESIZER_OBJ, \
                                        0, \
                                        code)
#define SYNTHESIZER_OBJ_WARN(code)    DBG_LOGF_WARN( \
                                        AS_MODULE_ID_SYNTHESIZER_OBJ, \
                                        0, \
                                        code)
#define SYNTHESIZER_OBJ_DBG(fmt, ...) AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_SYNTHESIZER_OBJ, \
                                        fmt, \
                                        ##__VA_ARGS__)
#define SYNTHESIZER_OBJ_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER( \
                                            AS_MODULE_ID_RECOGNITION_OBJ, \
                                            att_cb)
#define SYNTHESIZER_OBJ_UNREG_ATTCB() ATTENTION_CB_UNREGISTER( \
                                        AS_MODULE_ID_SYNTHESIZER_OBJ)


#define RECOGNIZER_CMP_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER( \
                                            AS_MODULE_ID_RECOGNITION_CMP, \
                                            att_cb)
#define RECOGNIZER_CMP_UNREG_ATTCB() ATTENTION_CB_UNREGISTER( \
                                            AS_MODULE_ID_RECOGNITION_CMP)

#define RECOGNIZER_CMP_FATAL(code)   DBG_LOGF_FATAL( \
                                        AS_MODULE_ID_RECOGNITION_CMP, \
                                        0, \
                                        code)
#define RECOGNIZER_CMP_ERR(code)     DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_RECOGNITION_CMP, \
                                        0, \
                                        code)
#define RECOGNIZER_CMP_WARN(code)    DBG_LOGF_WARN( \
                                        AS_MODULE_ID_RECOGNITION_CMP, \
                                        0, \
                                        code)
#define RECOGNIZER_CMP_INF(code)     DBG_LOGF_DEBUG( \
                                        AS_MODULE_ID_RECOGNITION_CMP, \
                                        0, \
                                        code)
#define RECOGNIZER_CMP_DBG(fmt, ...) AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_RECOGNITION_CMP, \
                                        fmt, \
                                        ##__VA_ARGS__)

#define OSCILLATOR_CMP_ERR(code)      DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_OSCILLATOR_CMP, \
                                        0, \
                                        code)
#define OSCILLATOR_CMP_WARN(code)     DBG_LOGF_WARN( \
                                        AS_MODULE_ID_OSCILLATOR_CMP, \
                                        0, \
                                        code)
#define OSCILLATOR_CMP_INF(code)      DBG_LOGF_DEBUG( \
                                        AS_MODULE_ID_OSCILLATOR_CMP, \
                                        0, \
                                        code)
#define OSCILLATOR_CMP_DBG(fmt, ...)  AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_OSCILLATOR_CMP, \
                                        fmt, \
                                        ##__VA_ARGS__)

#define CUSTOM_CMP_ERR(code)          DBG_LOGF_ERROR( \
                                        AS_MODULE_ID_CUSTOM_CMP, \
                                        0, \
                                        code)
#define CUSTOM_CMP_WARN(code)         DBG_LOGF_WARN( \
                                        AS_MODULE_ID_CUSTOM_CMP, \
                                        0, \
                                        code)
#define CUSTOM_CMP_INF(code)          DBG_LOGF_DEBUG( \
                                        AS_MODULE_ID_CUSTOM_CMP, \
                                        0, \
                                        code)
#define CUSTOM_CMP_DBG(fmt, ...)      AUDIO_LOG_EVENT( \
                                        AS_MODULE_ID_CUSTOM_CMP, \
                                        fmt, \
                                        ##__VA_ARGS__)

#define DMAC_REG_ATTCB(att_cb) ATTENTION_CB_REGISTER(AS_MODULE_ID_AUDIO_DRIVER, att_cb)
#define DMAC_UNREG_ATTCB() ATTENTION_CB_UNREGISTER(AS_MODULE_ID_AUDIO_DRIVER)
#define DMAC_FATAL(code)   DBG_LOGF_FATAL(AS_MODULE_ID_AUDIO_DRIVER, 0, code)
#define DMAC_ERR(code)     DBG_LOGF_ERROR(AS_MODULE_ID_AUDIO_DRIVER, 0, code)
#define DMAC_WARN(code)    DBG_LOGF_WARN(AS_MODULE_ID_AUDIO_DRIVER, 0, code)
#define DMAC_INF(code)     DBG_LOGF_DEBUG(AS_MODULE_ID_AUDIO_DRIVER, 0, code)
#define DMAC_DBG(fmt, ...) AUDIO_LOG_EVENT(AS_MODULE_ID_AUDIO_DRIVER, \
                                           fmt, \
                                           ##__VA_ARGS__)

#define logerr             auderr

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_DEBUG_DEBUG_LOG_H */
