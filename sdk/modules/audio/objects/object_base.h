/****************************************************************************
 * modules/audio/objects/object_base.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_AUDIO_OBJECTS_OBJECT_BASE_OBJECT_BASE_H
#define __MODULES_AUDIO_OBJECTS_OBJECT_BASE_OBJECT_BASE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/s_stl/queue.h"
#include "audio/audio_object_common_api.h"
#include "audio/audio_message_types.h"
#include "audio_state.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef struct
{
  /*! \brief [in] Message queue id of self */

  uint8_t self;

  /*! \brief [in] Message queue id of application */

  uint8_t from;

  /*! \brief [in] Message queue id of component */

  uint8_t cmp;

} AsObjectMsgQueId_t;


typedef struct
{
  /*! \brief [in] Memory pool id of input data */

  MemMgrLite::PoolId input;

  /*! \brief [in] Memory pool id of output data */

  MemMgrLite::PoolId output;

  /*! \brief [in] Memory pool id of component data */

  MemMgrLite::PoolId cmp;

} AsObjectPoolId_t;


typedef struct
{

  AsObjectMsgQueId_t msgq_id;

  AsObjectPoolId_t   pool_id;

} AsObjectParams_t;

class ObjectBase
{

public:
/*  static void create(AsObjectMsgQueId_t msgq_id,
                     AsObjectPoolId_t   pool_id);*/

  /* Main loop */

  void run();

protected:

  /* Object state */

  typedef enum
  {
    Booted = 0,
    Ready,
    PreActive,
    Active,
    Stopping,
    ErrorStopping,
    WaitStop,

    StateNum
  } ObjectState;

  AudioState<ObjectState> m_state;

  bool               m_is_created;
  AsObjectMsgQueId_t m_msgq_id;
  AsObjectPoolId_t   m_pool_id;
  pthread_t          m_pid;

  typedef void (ObjectBase::*MsgProc)(MsgPacket *);
  MsgProc m_event_tbl[MSG_OBJ_SUBTYPE_NUM][StateNum];

  /* Constructor */
  ObjectBase()
    : m_state(0 , "", Booted)
    , m_is_created(false)
  {
    /* Event table */

    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][Booted]          = &ObjectBase::activateOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][Ready]           = &ObjectBase::activateOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][PreActive]       = &ObjectBase::activateOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][Active]          = &ObjectBase::activateOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][Stopping]        = &ObjectBase::activateOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][ErrorStopping]   = &ObjectBase::activateOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_ACT][WaitStop]        = &ObjectBase::activateOnWaitStop;

    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][Booted]        = &ObjectBase::deactivateOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][Ready]         = &ObjectBase::deactivateOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][PreActive]     = &ObjectBase::deactivateOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][Active]        = &ObjectBase::deactivateOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][Stopping]      = &ObjectBase::deactivateOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][ErrorStopping] = &ObjectBase::deactivateOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_DEACT][WaitStop]      = &ObjectBase::deactivateOnWaitStop;

    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][Booted]         = &ObjectBase::initOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][Ready]          = &ObjectBase::initOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][PreActive]      = &ObjectBase::initOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][Active]         = &ObjectBase::initOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][Stopping]       = &ObjectBase::initOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][ErrorStopping]  = &ObjectBase::initOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_INIT][WaitStop]       = &ObjectBase::initOnWaitStop;

    m_event_tbl[MSG_OBJ_SUBTYPE_START][Booted]        = &ObjectBase::startOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_START][Ready]         = &ObjectBase::startOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_START][PreActive]     = &ObjectBase::startOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_START][Active]        = &ObjectBase::startOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_START][Stopping]      = &ObjectBase::startOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_START][ErrorStopping] = &ObjectBase::startOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_START][WaitStop]      = &ObjectBase::startOnWaitStop;

    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][Booted]         = &ObjectBase::execOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][Ready]          = &ObjectBase::execOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][PreActive]      = &ObjectBase::execOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][Active]         = &ObjectBase::execOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][Stopping]       = &ObjectBase::execOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][ErrorStopping]  = &ObjectBase::execOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_EXEC][WaitStop]       = &ObjectBase::execOnWaitStop;

    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][Booted]         = &ObjectBase::stopOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][Ready]          = &ObjectBase::stopOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][PreActive]      = &ObjectBase::stopOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][Active]         = &ObjectBase::stopOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][Stopping]       = &ObjectBase::stopOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][ErrorStopping]  = &ObjectBase::stopOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_STOP][WaitStop]       = &ObjectBase::stopOnWaitStop;

    m_event_tbl[MSG_OBJ_SUBTYPE_SET][Booted]          = &ObjectBase::setOnBooted;
    m_event_tbl[MSG_OBJ_SUBTYPE_SET][Ready]           = &ObjectBase::setOnReady;
    m_event_tbl[MSG_OBJ_SUBTYPE_SET][PreActive]       = &ObjectBase::setOnPreActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_SET][Active]          = &ObjectBase::setOnActive;
    m_event_tbl[MSG_OBJ_SUBTYPE_SET][Stopping]        = &ObjectBase::setOnStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_SET][ErrorStopping]   = &ObjectBase::setOnErrorStopping;
    m_event_tbl[MSG_OBJ_SUBTYPE_SET][WaitStop]        = &ObjectBase::setOnWaitStop;

  }

  ObjectBase(uint32_t module_id, AsObjectMsgQueId_t msg_id, AsObjectPoolId_t pool_id)
    : m_state(module_id , "", Booted)
    , m_is_created(true)
    , m_msgq_id(msg_id)
    , m_pool_id(pool_id)
    , m_pid(INVALID_PROCESS_ID)
  {
  }

  /* Destructor */

  virtual ~ObjectBase() {}

  /* Parse Event */

  void parse(MsgPacket *msg);

  /* Illegal Event */

  virtual void illegal(MsgPacket *msg) = 0;

  /* Class methods */

  virtual void activateOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void activateOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void activateOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void activateOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void activateOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void activateOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void activateOnWaitStop(MsgPacket *msg) { illegal(msg); }

  virtual void deactivateOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void deactivateOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void deactivateOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void deactivateOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void deactivateOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void deactivateOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void deactivateOnWaitStop(MsgPacket *msg) { illegal(msg); }

  virtual void initOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void initOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void initOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void initOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void initOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void initOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void initOnWaitStop(MsgPacket *msg) { illegal(msg); }

  virtual void startOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void startOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void startOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void startOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void startOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void startOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void startOnWaitStop(MsgPacket *msg) { illegal(msg); }

  virtual void execOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void execOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void execOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void execOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void execOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void execOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void execOnWaitStop(MsgPacket *msg) { illegal(msg); }

  virtual void stopOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void stopOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void stopOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void stopOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void stopOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void stopOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void stopOnWaitStop(MsgPacket *msg) { illegal(msg); }

  virtual void setOnBooted(MsgPacket *msg) { illegal(msg); }
  virtual void setOnReady(MsgPacket *msg) { illegal(msg); }
  virtual void setOnPreActive(MsgPacket *msg) { illegal(msg); }
  virtual void setOnActive(MsgPacket *msg) { illegal(msg); }
  virtual void setOnStopping(MsgPacket *msg) { illegal(msg); }
  virtual void setOnErrorStopping(MsgPacket *msg) { illegal(msg); }
  virtual void setOnWaitStop(MsgPacket *msg) { illegal(msg); }

  /* Pure abstract functions.
   * They should be implimented by the class which inherit ObjcetBase.
   */

  virtual void parseResult(MsgPacket *msg) = 0;

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

#endif /* __MODULES_AUDIO_OBJECTS_OBJECT_BASE_OBJECT_BASE_H */
