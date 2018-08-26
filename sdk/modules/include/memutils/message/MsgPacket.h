/****************************************************************************
 * modules/include/memutils/message/MsgPacket.h
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

#ifndef MSG_PACKET_H_INCLUDED
#define MSG_PACKET_H_INCLUDED

#include <new>			/* placement new */
#include <stdio.h>		/* printf */
#include "memutils/common_utils/common_types.h"	/* MIN, uintN_t */
#include "memutils/common_utils/common_assert.h"	/* D_ASSERT */
//#include "SpinLock.h"		/* MEMORY_BARRIER */
#include "memutils/message/type_holder.h"	/* TypeHolder */

#ifdef USE_MULTI_CORE
#include "get_cpu_id.h"		/* GET_CPU_ID */
#else
#define GET_CPU_ID()		(0)
#define MEMORY_BARRIER() 	{;}
#endif

/* Various type definition. */

typedef uint16_t MsgType;   /* ID of message type. */
typedef uint16_t MsgQueId;  /* ID of message queue. */
typedef uint8_t  MsgCpuId;  /* CPU-ID of message. */
typedef uint8_t  MsgFlags;  /* Flag of message. */

enum MsgPri
{
  /* Priority of message. */

	MsgPriNormal,
	MsgPriHigh,
	NumMsgPri /* Number of priority. */
};

/* Message parameter type match check */
#define MSG_PARAM_TYPE_MATCH_CHECK	false

/*****************************************************************
 * メッセージパケットヘッダクラス
 *****************************************************************/
class MsgPacketHeader {
public:
  /* Basically, each flag is exclusive to MessageLib internal. */

  /* Flag is empty. */

	static const MsgFlags MsgFlagNull	= 0x00; /* Flag is empty. */

  /* Wait for parameter writing. */

	static const MsgFlags MsgFlagWaitParam	= 0x80;

  /* Parameter is formatted with type. */

	static const MsgFlags MsgFlagTypedParam = 0x40;
	MsgPacketHeader(MsgType type, MsgQueId reply, MsgFlags flags, uint16_t size = 0) :
		m_type(type),
		m_reply(reply),
		m_src_cpu(GET_CPU_ID()),
		m_flags(flags),
		m_param_size(size) {}

	MsgType  getType() const { return m_type; }
	MsgQueId getReply() const { return m_reply; }
	MsgCpuId getSrcCpu() const { return m_src_cpu; }
	MsgFlags getFlags() const { return m_flags; }
	uint16_t getParamSize() const { return m_param_size; }
	void     popParamNoDestruct() { m_param_size = 0; }

protected:
	bool isSelfCpu() const { return GET_CPU_ID() == getSrcCpu(); }
	bool isTypedParam() const { return (m_flags & MsgFlagTypedParam) != 0; }

protected:
	MsgType		m_type;
	MsgQueId	m_reply;
	MsgCpuId	m_src_cpu;
	MsgFlags	m_flags;
	uint16_t	m_param_size;
}; /* class MsgPacketHeader */

/*****************************************************************
 * Class indicating that the message parameter does not exist
 *****************************************************************/
class MsgNullParam {};

/*****************************************************************
 * Class indicating that it is an address range parameter
 *****************************************************************/
class MsgRangedParam {
public:
	MsgRangedParam(const void* param, size_t param_size) :
		m_param(param),
		m_param_size(param_size) {}
	const void*	getParam() const { return m_param; }
	size_t		getParamSize() const { return m_param_size; }

private:
	const void*	m_param;
	size_t		m_param_size;
};

/*****************************************************************
 * Message Packet Class
 * In the instance copy of this class,
 * note that the parameters are not copied
 *****************************************************************/
class MsgPacket : public MsgPacketHeader {
public:
	template<typename T>
	T moveParam() {
		T param = peekParam<T>();
		popParam<T>();
		return param;
	}

	template<typename T>
	const T& peekParam() const {
		D_ASSERT2(sizeof(T) == getParamSize(), AssertParamLog(AssertIdSizeError, sizeof(T), getParamSize()));
		if (isTypeCheckEnable()) {
			return reinterpret_cast<const TypeHolderBase*>(&m_param[0])->template get<T>();
		} else {
			return *reinterpret_cast<const T*>(&m_param[0]);
		}
	}

	template<typename T>
	const T& peekParamOther() const {
		D_ASSERT2(sizeof(T) <= getParamSize(), AssertParamLog(AssertIdSizeError, sizeof(T), getParamSize()));
		return peekParamAny<T>();
	}

	template<typename T>
	void popParam() {
		D_ASSERT2(sizeof(T) == getParamSize(), AssertParamLog(AssertIdSizeError, sizeof(T), getParamSize()));
		if (isTypeCheckEnable()) {
			TypeHolderBase* p = reinterpret_cast<TypeHolderBase*>(&m_param[0]);
			D_ASSERT2(p->template is_type<T>(),
				AssertParamLog(AssertIdTypeUnmatch, (uint32_t)p->id(), (uint32_t)GET_TYPE_ID(T)));
			p->~TypeHolderBase();
		} else {
			reinterpret_cast<T*>(&m_param[0])->~T();
		}
		m_param_size = 0;
	}

	void dump() const {
		printf("T:%04x, R:%04x, C:%02x, F:%02x, S:%04x, P:",
			m_type, m_reply, m_src_cpu, m_flags, m_param_size);
		for (uint16_t i = 0; i < MIN(m_param_size, 16); ++i)
			printf("%02x ", m_param[i]);
		printf("\n");
	}

protected:
	friend class MsgLib;
	friend class MsgQueBlock;
	friend class MsgLog;

	MsgPacket(MsgType type, MsgQueId reply, MsgFlags flags) :
		MsgPacketHeader(type, reply, flags) {}

	void setParam(const MsgNullParam& /* param */, bool /* type_check */) {}

	template<typename T>
	void setParam(const T& param, bool type_check) {
		if (type_check) {
			m_flags |= MsgFlagTypedParam;
			new (&m_param[0]) TypeHolder<T>(param);
		} else {
			new (&m_param[0]) T(param);
		}
		m_param_size = sizeof(T);
		MEMORY_BARRIER();
		m_flags &= ~MsgFlagWaitParam; /* Clear the parameter write wait flag. */
	}

	void setParam(const MsgRangedParam& param, bool /* type_check */) {
		D_ASSERT((param.getParam() != NULL) && (0 < param.getParamSize()));
		memcpy(&m_param[0], param.getParam(), param.getParamSize());
		m_param_size = param.getParamSize();
		MEMORY_BARRIER();
		m_flags &= ~MsgFlagWaitParam; /* Clear the parameter write wait flag. */
	}

	bool isTypeCheckEnable() const { return MSG_PARAM_TYPE_MATCH_CHECK && isTypedParam(); }

  /* Reference parameters with arbitrary types without error checking. */

	template<typename T>
	const T& peekParamAny() const {
		if (isTypeCheckEnable()) {
			return reinterpret_cast<const TypeHolderBase*>(&m_param[0])->template get_any<T>(false);
		} else {
			return *reinterpret_cast<const T*>(&m_param[0]);
		}
	}

  /* For dump log only.
   * Since there is no size check,
   * there is a possibility of returning garbage.
   */

	uint32_t peekParamHead() const { return peekParamAny<uint32_t>(); }

protected:
	uint8_t		m_param[0];
}; /* class MsgPacket */

#endif /* MSG_PACKET_H_INCLUDED */
