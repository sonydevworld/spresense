/****************************************************************************
 * modules/include/memutils/message/MsgQue.h
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

#ifndef MSG_QUE_H_INCLUDED
#define MSG_QUE_H_INCLUDED

#include "memutils/common_utils/common_types.h"	/* drm_t, uint16_t */
#include "memutils/common_utils/common_assert.h"	/* D_ASSERT2 */
#include "memutils/message/BasicQueue.h"		/* BasicQueue */
#include "memutils/message/MsgPacket.h"		/* MsgPacket, MsgPacketHeader */

/* Message area fill value after message poped */
#define MSG_FILL_VALUE_AFTER_POP	0x0

/*****************************************************************
 * Message queue class
 * Make it protected inheritance to hide the base class template function
 *****************************************************************/

class MsgQue : protected BasicQueue<drm_t, uint16_t, uint16_t, MSG_FILL_VALUE_AFTER_POP> {
	typedef BasicQueue<drm_t, uint16_t, uint16_t, MSG_FILL_VALUE_AFTER_POP>	Base;
public:
	MsgQue() : Base() {}

	MsgQue(drm_t data_area, uint16_t elem, uint16_t num) :
		Base(data_area, elem, num) {}

  /* Make certain functions public only. */

	using Base::init;
	using Base::is_init;
	using Base::elem_size;
	using Base::capacity;
	using Base::size;
	using Base::rest;
	using Base::empty;
	using Base::full;
	using Base::dump;
	using Base::clear;
	using Base::pop;

  /* Insert a message packet header at the end of the queue
   * and return that address.
   */

	MsgPacket* pushHeader(const MsgPacketHeader& header) {
		return (push(header) == true) ? backMsg() : NULL;
	}

	MsgPacket* frontMsg() { return &front<MsgPacket>(); }
	MsgPacket* backMsg()  { return &back<MsgPacket>(); }
}; /* class MsgQue */

#endif /* MSG_QUE_H_INCLUDED */
