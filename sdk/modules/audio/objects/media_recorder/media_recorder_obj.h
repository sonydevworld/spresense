/****************************************************************************
 * modules/audio/objects/media_recorder/media_recorder_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_high_level_api.h"
#include "audio/audio_message_types.h"
#include "audio_recorder_sink.h"
#include "components/capture/capture_component.h"
#include "audio_state.h"

#include "components/filter/filter_component.h"
#include "components/filter/src_filter_component.h"
#include "components/filter/packing_component.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_PRESET_NUM        4
#define CAPTURE_PCM_BUF_QUE_SIZE  7
#define OUTPUT_DATA_QUE_SIZE      5
#define FALT_HANDLE_ID            0xFF

/****************************************************************************
 * Public Types
 ****************************************************************************/

class MediaRecorderObjectTask {
public:
  static void create(AsRecorderMsgQueId_t msgq_id,
                     AsRecorderPoolId_t pool_id);

  /* When MP3 encode and fs is 16kHz, 22.05kHz, 24kHz, sample num of
   * 1au(access unit) is 1152/2 = 576 (It depend on MPEG2 compliant).
   * Therefore, at first, value is (#1)"SampleNumPerFrame[m_codec_type] / 2".
   * And sample num of captured and SRC filterd data is to be 576,
   * return ((#1) * 48000 / m_sampling_rate(Hz)).
   *
   * The process below is only for fs is 48kHz, 16kHz.
   * To correspontd to 32000Hz, 44100Hz..., need conversion process to 
   * sample num per 1au to be 1152.
   */
  static uint32_t getPcmCaptureSample(uint8_t codec_type, uint32_t fs)
    {
      if (codec_type > AS_CODECTYPE_LPCM)
        {
          return 0;
        }

      AudioCodec intr_codec_def[] =
        {
          AudCodecMP3,
          AudCodecLPCM,
          AudCodecAAC,
          AudCodecOPUS,
          AudCodecAAC,
          AudCodecLPCM,
        };

      AudioCodec intr_codec = intr_codec_def[codec_type]; 

      if (intr_codec == AudCodecMP3 && fs < 32000)
        {
          return (SampleNumPerFrame[intr_codec] / 2 * 48000 /
            fs);
        }
      else if (intr_codec == AudCodecOPUS)
        {
          /* 20ms. */

          return ((fs / 50) * (48000 / fs));
        }
        return SampleNumPerFrame[intr_codec];
    }

private:
  MediaRecorderObjectTask(AsRecorderMsgQueId_t msgq_id,
                          AsRecorderPoolId_t pool_id):
    m_msgq_id(msgq_id),
    m_pool_id(pool_id),
    m_state(AS_MODULE_ID_MEDIA_RECORDER_OBJ, "", RecorderStateInactive),
    m_pcm_bit_width(AudPcm16Bit),
    m_sampling_rate(48000),
    m_codec_type(InvalidCodecType),
    m_output_device(AS_SETRECDR_STS_OUTPUTDEVICE_EMMC),
    m_p_output_device_handler(NULL),
    m_filter_instance(NULL)
  {}

  enum RecorderState_e
  {
    RecorderStateInactive = 0,
    RecorderStateReady,
    RecorderStateActive,
    RecorderStateStopping,
    RecorderStateErrorStopping,
    RecorderStateWaitStop,
    RecorderStateNum
  };

  AsRecorderMsgQueId_t m_msgq_id;
  AsRecorderPoolId_t   m_pool_id;

  AudioState<RecorderState_e> m_state;
  AudioPcmBitWidth m_pcm_bit_width;
  int32_t m_sampling_rate;
  int32_t m_max_output_pcm_size;
  AudioCodec m_codec_type;
  AsSetRecorderStsOutputDevice m_output_device;
  AsRecorderOutputDeviceHdlr* m_p_output_device_handler;
  int8_t  m_complexity;
  int32_t m_bit_rate;
  AudioRecorderSink m_rec_sink;

  FilterComponent *m_filter_instance;

  typedef void (MediaRecorderObjectTask::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_VRC_MSG_NUM][RecorderStateNum];
  static MsgProc RsltProcTbl[AUD_VRC_RST_MSG_NUM][RecorderStateNum];

  typedef s_std::Queue<AsPcmDataParam, CAPTURE_PCM_BUF_QUE_SIZE> CnvInQueue;
  CnvInQueue m_cnv_in_que;

  typedef s_std::Queue<MemMgrLite::MemHandle, OUTPUT_DATA_QUE_SIZE>
    OutputBufMhQueue;
  OutputBufMhQueue m_output_buf_mh_que;

  s_std::Queue<AsRecorderEvent, 1> m_external_cmd_que;

  MediaRecorderCallback m_callback;

  void run();
  void parse(MsgPacket *);

  void reply(AsRecorderEvent evtype,
             MsgType msg_type,
             uint32_t result);

  void illegal(MsgPacket *);
  void activate(MsgPacket *);
  void deactivate(MsgPacket *);

  void init(MsgPacket *);

  void startOnReady(MsgPacket *msg);

  void illegalReqEnc(MsgPacket *msg);
  void reqEncOnActive(MsgPacket *msg);

  void flushOnReady(MsgPacket *msg);
  void flushOnActive(MsgPacket *msg);
  void flushOnStop(MsgPacket *msg);
  void flushOnErrorStop(MsgPacket *msg);
  void flushOnWait(MsgPacket *msg);

  void illegalFilterDone(MsgPacket *);
  void filterDoneOnActive(MsgPacket *);
  void filterDoneOnStop(MsgPacket *);
  void filterDoneOnErrorStop(MsgPacket *);

  void illegalEncDone(MsgPacket *);
  void encDoneOnActive(MsgPacket *);
  void encDoneOnStop(MsgPacket *);
  void encDoneOnErrorStop(MsgPacket *);

  bool execEnc(AsPcmDataParam *pcm);
  bool stopEnc(void);

  bool setExternalCmd(AsRecorderEvent ext_event);
  AsRecorderEvent getExternalCmd(void);
  uint32_t checkExternalCmd(void);

  uint32_t loadCodec(AudioCodec, char *, int32_t, int32_t, uint32_t *);
  bool unloadCodec(void);
  uint32_t initEnc(AsInitRecorderParam *param);

  MemMgrLite::MemHandle getOutputBufAddr();

  bool holdCnvInBuf(AsPcmDataParam in)
    {
      if (!m_cnv_in_que.push(in))
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return false;
        }

      return true;
    }

  bool freeCnvInBuf()
    {
      if (!m_cnv_in_que.pop())
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
          return false;
        }

      return true;
    }

  bool holdOutputBuf(MemMgrLite::MemHandle mh)
    {
      if (!m_output_buf_mh_que.push(mh))
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return false;
        }

      return true;
    }

  bool freeOutputBuf()
    {
      if (!m_output_buf_mh_que.pop())
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
          return false;
        }

      return true;
    }


  uint32_t isValidActivateParam(const AsActivateRecorderParam& cmd);
  uint32_t isValidInitParam(const RecorderCommand& cmd);
  uint32_t isValidInitParamMP3(const RecorderCommand& cmd);
  uint32_t isValidInitParamLPCM(const RecorderCommand& cmd);
  uint32_t isValidInitParamOPUS(const RecorderCommand& cmd);
  bool writeToDataSinker(const MemMgrLite::MemHandle& mh, uint32_t byte_size);

  bool checkAndSetMemPool();
  bool isNeedUpsampling(int32_t sampling_rate);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H */

