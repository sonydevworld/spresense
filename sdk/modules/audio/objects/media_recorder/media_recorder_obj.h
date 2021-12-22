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
#include "audio/utilities/frame_samples.h"
#include "audio_recorder_sink.h"
#include "components/capture/capture_component.h"
#include "audio_state.h"

#include "../object_base.h"

#include "components/encoder/encoder_component.h"
#include "components/customproc/thruproc_component.h"
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

class MediaRecorderObject:ObjectBase
{
public:

  static void create(AsObjectParams_t*);
  static void destory() {
    get_instance()->~MediaRecorderObject();
    get_instance()->m_is_created = false;
  }
  static MediaRecorderObject *get_adr(void)
  {
    static MediaRecorderObject s_inst;
    return &s_inst;
  }

  static MediaRecorderObject *get_instance(void)
  {
    MediaRecorderObject* s_inst = get_adr();
    if(s_inst->m_is_created){
      return s_inst;
    }
    return 0;
  }

  static pthread_t get_pid(void)
  {
    return (get_instance() == 0) ? INVALID_PROCESS_ID : get_instance()->m_pid;
  }

  static void set_pid(pthread_t id)
  {
    if (get_instance() != 0) { get_instance()->m_pid = id; }
  }

  static pthread_t get_msgq_id(void)
  {
    return (get_instance() == 0) ? 0 : get_instance()->m_msgq_id.self;
  }

  typedef struct
  {
    ComponentEventType event_type;
    bool                result;
  } FilterDoneCmd;


private:

  MediaRecorderObject()
    : ObjectBase() {}

  MediaRecorderObject(AsObjectMsgQueId_t msgq_id, AsObjectPoolId_t pool_id)
    : ObjectBase(AS_MODULE_ID_MEDIA_RECORDER_OBJ, msgq_id, pool_id),
    m_channel_num(2),
    m_pcm_bit_width(AudPcmFormatInt16),
    m_sampling_rate(48000),
    m_codec_type(InvalidCodecType),
    m_output_device(AS_SETRECDR_STS_OUTPUTDEVICE_RAM),
    m_p_output_device_handler(NULL),
    m_filter_instance(NULL)
  {
    /* Create instance of components when MediaRecorderObj is created.
     * Don't new and delete while MediaRecorderObj is active to avoid
     * heap memory area leak.
     */

    m_src_instance = new SRCComponent(m_pool_id.cmp, m_msgq_id.cmp);
    m_packing_instance = new PackingComponent();
    m_thruproc_instance = new ThruProcComponent();
  }

  ~MediaRecorderObject()
  {
    delete m_src_instance;
    delete m_packing_instance;
    delete m_thruproc_instance;
  }

  int8_t m_channel_num;
  AudioPcmFormat m_pcm_bit_width;
  int32_t m_sampling_rate;
  int32_t m_max_output_pcm_size;
  AudioCodec m_codec_type;
  AsSetRecorderStsOutputDevice m_output_device;
  AsRecorderOutputDeviceHdlr* m_p_output_device_handler;
  int8_t  m_complexity;
  int32_t m_bit_rate;
  AudioRecorderSink m_rec_sink;

  ComponentBase *m_filter_instance;
  SRCComponent *m_src_instance;
  PackingComponent *m_packing_instance;
  ThruProcComponent *m_thruproc_instance;

  typedef void (MediaRecorderObject::*MsgProc)(MsgPacket *);
  static MsgProc RsltProcTbl[AUD_MRC_RST_MSG_NUM][StateNum];

  typedef s_std::Queue<AsPcmDataParam, CAPTURE_PCM_BUF_QUE_SIZE> CnvInQueue;
  CnvInQueue m_cnv_in_que;

  typedef s_std::Queue<MemMgrLite::MemHandle, OUTPUT_DATA_QUE_SIZE>
    OutputBufMhQueue;
  OutputBufMhQueue m_output_buf_mh_que;

  s_std::Queue<AsRecorderEvent, 1> m_external_cmd_que;

  MediaRecorderCallback m_callback;

  void parseResult(MsgPacket *msg);

  void reply(AsRecorderEvent evtype,
             MsgType msg_type,
             uint32_t result);

  void illegal(MsgPacket *);
  void illegalReqEnc(MsgPacket *);

  void activateOnBooted(MsgPacket *);
  void deactivateOnReady(MsgPacket *);
  void initOnReady(MsgPacket *);
  void startOnReady(MsgPacket *);

  void stopOnReady(MsgPacket *);
  void stopOnActive(MsgPacket *);
  void stopOnStopping(MsgPacket *);
  void stopOnErrorStopping(MsgPacket *);
  void stopOnWaitStop(MsgPacket *);

  void execOnActive(MsgPacket *);
  void execOnBooted(MsgPacket *msg) { illegalReqEnc(msg); }
  void execOnReady(MsgPacket *msg) { illegalReqEnc(msg); }
  void execOnPreActive(MsgPacket *msg) { illegalReqEnc(msg); }
  void execOnStopping(MsgPacket *msg) { illegalReqEnc(msg); }
  void execOnErrorStopping(MsgPacket *msg) { illegalReqEnc(msg); }
  void execOnWaitStop(MsgPacket *msg) { illegalReqEnc(msg); }

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

  bool enqueEncInBuf(AsPcmDataParam in)
    {
      if (!m_cnv_in_que.push(in))
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return false;
        }

      return true;
    }

  bool dequeEncInBuf()
    {
      if (!m_cnv_in_que.pop())
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
          return false;
        }

      return true;
    }

  bool enqueEncOutBuf(MemMgrLite::MemHandle mh)
    {
      if (!m_output_buf_mh_que.push(mh))
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return false;
        }

      return true;
    }

  bool dequeEncOutBuf()
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

