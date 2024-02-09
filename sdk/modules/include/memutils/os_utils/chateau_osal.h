/****************************************************************************
 * modules/include/memutils/os_utils/chateau_osal.h
 *
 *   Copyright 2018, 2023 Sony Semiconductor Solutions Corporation
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

#ifndef CHATEAU_OSAL_H_INCLUDED
#define CHATEAU_OSAL_H_INCLUDED

#include "memutils/common_utils/common_macro.h"
#include "memutils/common_utils/common_assert.h"

/* TODO(Ohashi): Chateau の下記ヘッダに置き換えたい */
/* #include "chateau_macro.h"
   #include "chateau_assert.h"
   #include "chateau_types.h" */

#if defined(_ITRON4)
#if defined(_MERLOT)
#include "kernel.h"
#else /* _MERLOT */
#include "itron.h"
#include "kernel_itron.h"
#include "kernel_SailOS.h"
#endif /* _MERLOT */

#define TIME_POLLING	TMO_POL
#define TIME_FOREVER	(unsigned)TMO_FEVR

typedef ID		Chateau_sem_handle_t;
typedef	void          (*Chateau_task_func_t)(void *arg);
typedef ID		Chateau_task_id_t;
typedef Chateau_task_id_t	Chateau_task_handle_t;	/* ITRONは、Task ID = Taskハンドル */
typedef PRI		Chateau_task_pri_t;

/* Task */
static INLINE Chateau_task_handle_t Chateau_CreateTask(Chateau_task_func_t func,
						       void* arg, size_t stack_size,
						       Chateau_task_pri_t pri) {
	T_CTSK	cfg;
	cfg.tskatr = TA_HLNG | TA_ACT;
	cfg.exinf = (VP_INT)arg;
	cfg.task = (FP)func;
	cfg.itskpri = pri;
	cfg.stksz = stack_size;
	cfg.stk = NULL;
	ID tid = acre_tsk(&cfg);
	F_ASSERT(tid > 0);
	return tid;
}

static INLINE Chateau_task_id_t Chateau_GetTaskId() {
	Chateau_task_id_t tid = 0;
	(void)get_tid(&tid);	/* 割込み禁止状態で呼ぶとエラー */
	return tid;
}

static INLINE Chateau_task_handle_t Chateau_GetTaskHandle() {
	return Chateau_GetTaskId();
}

static INLINE Chateau_task_pri_t Chateau_GetTaskPriority() {
	Chateau_task_pri_t pri; F_ASSERT(get_pri(TSK_SELF, &pri) == E_OK); return pri;
}
#define Chateau_SetTaskPriority(pri)	F_ASSERT(chg_pri(TSK_SELF, pri) == E_OK)
#define Chateau_SuspendTask(h)		F_ASSERT(sus_tsk(h)	== E_OK)
#define Chateau_ResumeTask(h)		F_ASSERT(rsm_tsk(h)	== E_OK)
#define Chateau_SleepTask(ms)		tslp_tsk(ms)
#define Chateau_DelayTask(ms)		dly_tsk(ms)
#define Chateau_YieldTask()             F_ASSERT(rot_rdq(TPRI_SELF) == E_OK)
#define Chateau_IsTaskContext()		(!sns_ctx())

#define Chateau_GetSystemTime(n) do{ SYSTIM s = 0; get_tim(&s); (n) = (unsigned)s; }while(0)
#define Chateau_GetInterruptMask()		sns_loc()
#define Chateau_LockInterrupt(pContext)		loc_cpu()
#define Chateau_LockInterruptIsr(pContext)	iloc_cpu()
#define Chateau_UnlockInterrupt(pContext)	unl_cpu()
#define Chateau_UnlockInterruptIsr(pContext)	iunl_cpu()
#define Chateau_EnableInterrupt(intno)		ena_int(intno)	
#define Chateau_DisableInterrupt(intno)		dis_int(intno)

/* Semaphore */
#define Chateau_CreateSemaphore(pH, ini, max)					\
	do{ T_CSEM _cfg_ = { TA_TFIFO, ini, max };			\
		F_ASSERT((*pH = acre_sem(&_cfg_)) > 0); }while(0)
#define Chateau_DeleteSemaphore(h)	F_ASSERT(del_sem(h)	   == E_OK)
#define Chateau_WaitSemaphore(h)	F_ASSERT(wai_sem(h)	   == E_OK)
#define Chateau_PollingWaitSemaphore(h)	(pol_sem(h)		   == E_OK)
#define Chateau_TimedWaitSemaphore(h, ms)	(twai_sem(h, (ms)) == E_OK)
#define Chateau_SignalSemaphore(h)	(Chateau_IsTaskContext() ?  Chateau_SignalSemaphoreTask(h) : Chateau_SignalSemaphoreIsr(h))
#define Chateau_SignalSemaphoreTask(h)	F_ASSERT(sig_sem(h)	   == E_OK)
#define Chateau_SignalSemaphoreIsr(h)	F_ASSERT(isig_sem(h)	   == E_OK)
static INLINE size_t Chateau_CountSemaphore(Chateau_sem_handle_t h) {
	T_RSEM s; F_ASSERT(ref_sem(h, &s) == E_OK); return s.semcnt;
}

/* Cyclic */
#define Chateau_StartCyclicHandler(h) F_ASSERT(sta_cyc(h) == E_OK)
#define Chateau_StopCyclicHandler(h) F_ASSERT(stp_cyc(h) == E_OK)

#elif defined(_POSIX)
#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <semaphore.h>
#include <time.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include "memutils/os_utils/os_wrapper.h"
#define Chateau_DelayTask(ms)  (usleep((ms)*1000))
#define Chateau_EnableInterrupt(irq) up_enable_irq(irq)
#define Chateau_DisableInterrupt(irq) up_disable_irq(irq)

#define Chateau_GetInterruptMask() (0)
#define Chateau_IsTaskContext() (getpid() != 0)

#define Chateau_LockInterrupt(pContext)					\
    do {                                                                \
        up_irq_disable();                                               \
        sched_lock();                                                   \
        (void)pContext;                                                   \
    } while(0)
#define Chateau_LockInterruptIsr(pContext)                              \
    do {                                                                \
        up_irq_disable();                                               \
        sched_lock();                                                   \
        (void)pContext;                                                   \
    } while(0)
#define Chateau_UnlockInterrupt(pContext)                               \
    do {                                                                \
        sched_unlock();                                                 \
        up_irq_enable();                                               \
        (void)pContext;                                                   \
    } while(0)
#define Chateau_UnlockInterruptIsr(pContext)                            \
    do {                                                                \
        sched_unlock();                                                 \
        up_irq_enable();                                               \
        (void)pContext;                                                   \
    } while(0)

/* 取り敢えず仮実装 */
#define TIME_FOREVER	(unsigned)TMO_FEVR
typedef sem_t	Chateau_sem_handle_t;
#define Chateau_CreateSemaphore(pH, ini, max)   \
  do {                                          \
    int ret = sem_init(pH, ini, max);           \
    F_ASSERT(ret == 0);                         \
  } while (0)
#define Chateau_DeleteSemaphore(h)              \
  do {                                          \
    int ret = sem_destroy(&h);                  \
    F_ASSERT(ret == 0);                         \
  } while (0)
#define Chateau_SignalSemaphore(h)              \
  do {                                          \
    int ret = sem_post(&h);                     \
    F_ASSERT(ret == 0);                         \
  } while (0)
#define Chateau_SignalSemaphoreTask(h)  Chateau_SignalSemaphore(h)
#define Chateau_SignalSemaphoreIsr(h)   Chateau_SignalSemaphore(h)
#define Chateau_TimedWaitSemaphore(h, tm) (nxsem_timedwait_uninterruptible(&h, &tm) == 0)
#define Chateau_WaitSemaphore(h)        (nxsem_wait_uninterruptible(&h) == 0)

#elif defined(NO_OS)
/* 取り敢えず仮実装 */
typedef short	Chateau_sem_handle_t;
#include "arm_nosys.h"
#define Chateau_LockInterruptIsr(pContext)	loc_cpu()
#define Chateau_UnlockInterruptIsr(pContext)	unl_cpu()
#define Chateau_IsTaskContext()			1
#define Chateau_GetInterruptMask()		sns_loc()
#define Chateau_LockInterrupt(h)		loc_cpu()
#define Chateau_UnlockInterrupt(h)		unl_cpu()
#define Chateau_SleepTask(ms) \
	do{ volatile unsigned _n_ = (ms) * CYCLE_PER_MS; while (_n_--) ; }while(0)
#define Chateau_DelayTask(ms)		Chateau_SleepTask(ms)
#else
#error "Unkown OS"
#endif
#endif /* CHATEAU_OSAL_H_INCLUDED */
