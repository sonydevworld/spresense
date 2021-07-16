/****************************************************************************
 * modules/audio/objects/front_end/front_end_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_FRONT_END_FRONT_END_OBJ_H
#define __MODULES_AUDIO_OBJECTS_FRONT_END_FRONT_END_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_message_types.h"
#include "audio/audio_frontend_api.h"
#include "audio_state.h"

#include "../object_base.h"

#include "components/capture/capture_component.h"
#include "components/customproc/usercustom_component.h"
#include "components/customproc/thruproc_component.h"
#include "components/filter/src_filter_component.h"

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

struct MicFrontendObjPreProcDoneCmd
{
  ComponentEventType event_type;
  bool                result;
};

struct MicFrontendObjSendDoneCmd
{
  bool identifier;
  bool is_end;
};

/* Class definition */

class MicFrontEndObject:ObjectBase
{
public:

  static void create(AsObjectParams_t*);
  static void destory() { get_instance()->m_is_created = false; }

  static MicFrontEndObject *get_adr(void)
  {
    static MicFrontEndObject s_inst;
    return &s_inst;
  }

  static MicFrontEndObject *get_instance(void)
  {
    MicFrontEndObject* s_inst = get_adr();
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

private:
  MicFrontEndObject(void)
    : ObjectBase()
    , m_preproc_type(AsMicFrontendPreProcInvalid)
    , m_callback(NULL) {}
//    , m_notify_path(AsNotifyPathCallback) {}


  MicFrontEndObject(AsObjectMsgQueId_t msgq_id, AsObjectPoolId_t pool_id)
    : ObjectBase(AS_MODULE_ID_MIC_FRONTEND_OBJ, msgq_id, pool_id)
    , m_preproc_type(AsMicFrontendPreProcInvalid)
    , m_channel_num(2)
    , m_pcm_bit_width(AudPcmFormatInt16)
    , m_samples_per_frame(768)
    , m_cap_bytes(2)  /* This value depends on the value of
                       * m_pcm_bit_width.
                       */
    , m_capture_hdlr(MAX_CAPTURE_COMP_INSTANCE_NUM)
    , m_capture_req(0)
    , m_preproc_req(0)
    , m_p_preproc_instance(NULL)
    , m_callback(NULL)
  {
    memset(m_dsp_path, 0, sizeof(m_dsp_path));
  }

  AsMicFrontendDataPath m_pcm_data_path;
  AsDataDest m_pcm_data_dest;
  AsMicFrontendPreProcType m_preproc_type;
  char m_dsp_path[AS_PREPROCESS_FILE_PATH_LEN];

  int8_t  m_channel_num;
  uint8_t m_pcm_bit_width;
  uint32_t m_samples_per_frame;
  int8_t  m_cap_bytes;
  int32_t m_max_output_size;
  int32_t m_max_capture_size;
  CaptureDevice m_input_device;
  CaptureComponentHandler m_capture_hdlr;
  uint32_t m_capture_req;
  uint32_t m_preproc_req;

  ComponentBase *m_p_preproc_instance;

  typedef void (MicFrontEndObject::*MsgProc)(MsgPacket *);
  static MsgProc MsgParamTbl[AUD_MFE_PRM_NUM][StateNum];
  static MsgProc RsltProcTbl[AUD_MFE_RST_MSG_NUM][StateNum];
  s_std::Queue<AsMicFrontendEvent, 1> m_external_cmd_que;

  MicFrontendCallback m_callback;

  void parseResult(MsgPacket *msg);

  void reply(AsMicFrontendEvent evtype,
             MsgType msg_type,
             uint32_t result);

  void illegal(MsgPacket *);

  void activateOnBooted(MsgPacket *msg);
  void deactivateOnReady(MsgPacket *msg);
  void initOnReady(MsgPacket *msg);
  void startOnReady(MsgPacket *msg);
  void stopOnActive(MsgPacket *msg);
  void stopOnErrorStopping(MsgPacket *msg);
  void stopOnWaitStop(MsgPacket *msg);
  void setOnReady(MsgPacket *msg) { set(msg); }
  void setOnActive(MsgPacket *msg) { set(msg); }
  void setOnErrorStoppping(MsgPacket *msg) { set(msg); }
  void set(MsgPacket *msg);

  void initPreproc(MsgPacket *msg);
  void setPreproc(MsgPacket *msg);
  void setMicGain(MsgPacket *);

  void illegalPreprocDone(MsgPacket *);
  void preprocDoneOnActive(MsgPacket *);
  void preprocDoneOnStop(MsgPacket *);
  void preprocDoneOnErrorStop(MsgPacket *);
  void preprocDoneOnWaitStop(MsgPacket *);

  void illegalCaptureDone(MsgPacket *);
  void captureDoneOnActive(MsgPacket *);
  void captureDoneOnStop(MsgPacket *);
  void captureDoneOnErrorStop(MsgPacket *);
  void captureDoneOnWaitStop(MsgPacket *msg);

  void illegalCaptureError(MsgPacket *);
  void captureErrorOnActive(MsgPacket *);
  void captureErrorOnStop(MsgPacket *);
  void captureErrorOnErrorStop(MsgPacket *);
  void captureErrorOnWaitStop(MsgPacket *);

  uint32_t loadComponent(AsMicFrontendPreProcType type, char *dsp_path);
  uint32_t unloadComponent(void);

  bool startCapture();

  bool setExternalCmd(AsMicFrontendEvent ext_event);
  AsMicFrontendEvent getExternalCmd(void);
  uint32_t checkExternalCmd(void);

  uint32_t activateParamCheck(const AsActivateFrontendParam& cmd);
  uint32_t initParamCheck(const MicFrontendCommand& cmd);

  bool execPreProc(MemMgrLite::MemHandle inmh, uint32_t sample);
  bool flushPreProc(void);

  bool sendData(AsPcmDataParam& data);
  bool sendDummyEndData(void);

  bool getInputDeviceHdlr(void);
  bool delInputDeviceHdlr(void);

  bool checkAndSetMemPool();
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

#endif /* __MODULES_AUDIO_OBJECTS_FRONT_END_FRONT_END_OBJ_H */

