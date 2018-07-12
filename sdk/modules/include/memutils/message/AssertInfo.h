/****************************************************************************
 * modules/include/memutils/message/AssertInfo.h
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

#ifndef ASSERT_INFO_H_INCLUDED
#define ASSERT_INFO_H_INCLUDED

#ifdef _ITRON4
/*
 * SAVESZ の値は、kernel_id.h の_KERNEL_FLOATING_POINT_ の定義により決定される
 * そのため、kernel_id.h が mips.h および mips_regs.h よりも先にincludeされる必要がある
 */
#include "kernel.h"
#ifdef MCB4357
/*
 * SailOSのMCB4357_1.01版では、SAVESZが未定義なので同様の意味合いのSAVESIZE
 * (kernel/cortex-mx/stack.c)の値を参照して、ここで定義する
 */
#if (defined(__TARGET_FPU_VFP) || defined(__ARMVFP__))
#define	SAVESZ		(196)
#else
#define	SAVESZ		(64)
#endif
#endif /* MCB4357 */
#endif /* _ITRON4 */

#define	SAVESZ		(196)


#include "memutils/common_utils/common_types.h"
//#include "dmp_id.h"

#ifndef EVA
#define DBG_P(...)	printf("Assertion information: " __VA_ARGS__)
#else
#define DBG_P(...)
#endif

/* dmp_layout.confにDMP_ASSERT_INFOエントリが未定義の場合は、空文で定義する */
#ifndef DMP_ASSERT_INFO_NUM
#define DMP_ASSERT_INFO_SEQ_LOG(p)
#endif

/* このIDにより、LogAnalyzerの表示方法を切り替える */
enum AssertLogId {
	AssertIdLocation	= 0,
	AssertIdException	= 1,
	AssertIdOsIllegal	= 2,
	AssertIdOsStackIllegal	= 3,
	AssertIdFence		= 4,
	AssertIdBadParam	= 5,
	AssertIdTypeUnmatch	= 6,
	AssertIdSizeError	= 7,
	AssertIdBadMsgQueState	= 8,
	AssertIdMemSegLeak	= 9,
	AssertIdOther		= 10,
}; /* enum AssertLogId */

/*****************************************************************
 * アサート情報の基底クラス
 *****************************************************************/
struct AssertInfoBase {
	uint8_t		m_log_id;
	uint8_t		m_task_id;
	uint16_t	m_body_size;	/* 後続データ長 */
public:
	AssertInfoBase(AssertLogId log_id, uint16_t log_size) :
		m_log_id(log_id),
//		m_task_id(DMP_GET_TASK_ID),
		m_task_id(0),
		m_body_size(log_size - sizeof(*this)) {}
}; /* struct AssertInfoBase */

/*****************************************************************
 * アサート位置を記録するクラス
 *****************************************************************/
struct AssertLocationLog : public AssertInfoBase {
	uint32_t	m_line;
	uint32_t	m_ret_addr;
	char		m_filename[128];
public:
	AssertLocationLog(const char* filename, int line, void* ret_addr) :
		AssertInfoBase(AssertIdLocation, sizeof(*this)),
		m_line(line),
		m_ret_addr(reinterpret_cast<uint32_t>(ret_addr)),
		m_filename()
	{
		size_t n = strlen(filename);
		if (n < sizeof(m_filename)) {
			strcpy(m_filename, filename);
		} else {
			strcpy(m_filename, filename + n - sizeof(m_filename) + 1);
		}
		DBG_P("TaskID=%d, file=%s, line=%d, return addr=%08x\n", m_task_id, m_filename, m_line, m_ret_addr);
		DMP_ASSERT_INFO_SEQ_LOG(*this);
	}
}; /* struct AssertLocationLog */

#ifdef _ITRON4
const uint32_t RegSaveSize = SAVESZ;	/* arm=64, mips=160 or 296, cortex=64 or 196 */
#else  /* #ifdef _ITRON4 */
#ifdef FPU_REG_STORE
const uint32_t RegSaveSize = 284;	/* (31 + 4 + 36) * 4; */
#else  /* #ifdef FPU_REG_STORE */
const uint32_t RegSaveSize = 140;	/* (31 + 4) * 4 */
#endif /* #ifdef FPU_REG_STORE */
#endif /* #ifdef _ITRON4 */

/*****************************************************************
 * 例外情報を記憶するクラス
 *****************************************************************/
struct AssertExceptionLog : public AssertInfoBase {
	uint32_t	m_cause;
	uint32_t	m_epc;
	uint32_t	m_sr;
	uint32_t	m_bad_vaddr;
	uint32_t	m_user_sp;
	uint8_t		m_uStk[256];
	uint8_t		m_kStk[RegSaveSize];
public:
	AssertExceptionLog(uint32_t cause, uint32_t epc, uint32_t sr, uint32_t bad_vaddr,
			const uint32_t* uStk = NULL, const uint32_t* kStk = NULL) :
		AssertInfoBase(AssertIdException, sizeof(*this)),
		m_cause(cause),
		m_epc(epc),
		m_sr(sr),
		m_bad_vaddr(bad_vaddr),
		m_user_sp(reinterpret_cast<uint32_t>(uStk))
	{
		if (uStk) {
			memcpy(m_uStk, uStk, sizeof(m_uStk));
		} else {
			/* スタックフレームのアドレスが指定されない場合は、有効サイズを減らす */
			m_body_size -= sizeof(m_uStk);
		}
		if (kStk) {
			memcpy(m_kStk, kStk, sizeof(m_kStk));
		} else {
			m_body_size -= sizeof(m_kStk);
		}
		DBG_P("TaskID=%d, Cause=%08x, EPC=%08x, SR=%08x\n", m_task_id, m_cause, m_epc, m_sr);
		DMP_ASSERT_INFO_SEQ_LOG(*this);
	}
}; /* struct AssertExceptionLog */

/*****************************************************************
 * フェンスチェックエラー時のパラメタとフェンス内容を記憶するクラス
 *****************************************************************/
struct AssertFenceLog : public AssertInfoBase {
	uint32_t*	m_addr;
	uint32_t	m_data[16];	/* Stack/Heap Fenceは16bytes. Pool Fenceは64bytes */
public:
	explicit AssertFenceLog(uint32_t* addr) :
		AssertInfoBase(AssertIdFence, sizeof(*this)),
		m_addr(addr)
	{
		for (size_t i = 0; i < COUNT_OF(m_data); i++) {
			m_data[i] = addr[i];
		}
		DBG_P("BadAddr=%08x, Data=%08x, %08x, %08x, %08x\n", m_addr, m_data[0], m_data[1], m_data[2], m_data[3]);
		DMP_ASSERT_INFO_SEQ_LOG(*this);
	}
}; /* struct AssertFenceLog */

/*****************************************************************
 * アサート時のパラメタを記憶するクラス
 *****************************************************************/
struct AssertParamLog : public AssertInfoBase {
	uint32_t	m_param[4];
public:
	AssertParamLog(AssertLogId id, uint32_t p0, uint32_t p1 = 0, uint32_t p2 = 0, uint32_t p3 = 0) :
		AssertInfoBase(id, sizeof(*this))
	{
		m_param[0] = p0; m_param[1] = p1; m_param[2] = p2; m_param[3] = p3;
		DBG_P("TaskID=%d, AssertID=%u, param=%08x, %08x, %08x, %08x\n", m_task_id, id, p0, p1, p2, p3);
		DMP_ASSERT_INFO_SEQ_LOG(*this);
	}
}; /* struct AssertParamLog */

#undef DBG_P

#endif /* ASSERT_INFO_H_INCLUDED */
