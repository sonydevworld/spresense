/****************************************************************************
 * modules/include/memutils/os_utils/os_wrapper.h
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

#ifndef _OS_WRAPPER_H_
#define _OS_WRAPPER_H_

#if !defined(OS_TYPE_FREERTOS) && !defined(_POSIX)
#include "../os/merlot/osal/uitron/os_wrapper_rename.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#ifdef OS_TYPE_FREERTOS

#include <freertos_inc.h>

typedef unsigned int	SYS_CpuId;
typedef int16_t			SYS_Id;
typedef uint32_t		SYS_FlagPattern;
typedef struct t_msg {
    struct t_msg *next;
} SYS_Message;
typedef int				SYS_Timeout;
typedef unsigned int	SYS_WaitMode;
typedef unsigned int	SYS_Time;
typedef unsigned int	SYS_RelativeTime;
typedef uint32_t		SYS_Attribute;
typedef unsigned int	SYS_Status;
typedef TaskHandle_t	SYS_Task;
typedef TimerHandle_t	SYS_CyclicHandler;
typedef TimerHandle_t	SYS_AlarmHandler;
typedef void			*SYS_CyclicHandlerInfo;
typedef void			*SYS_AlarmHandlerInfo;

#elif !defined(_POSIX) /* OS_TYPE_FREERTOS */

#include <itron.h>
#include <refcnt_mpf.h>

typedef unsigned int	SYS_CpuId;
typedef ID		SYS_Id;
typedef FLGPTN	SYS_FlagPattern;
typedef T_MSG	SYS_Message;
typedef TMO		SYS_Timeout;
typedef MODE	SYS_WaitMode;
typedef SYSTIM	SYS_Time;
typedef RELTIM	SYS_RelativeTime;
typedef ATR		SYS_Attribute;
typedef STAT	SYS_Status;
typedef ID		SYS_Task;
typedef ID		SYS_CyclicHandler;
typedef ID		SYS_AlarmHandler;
typedef void	*SYS_CyclicHandlerInfo;
typedef void	*SYS_AlarmHandlerInfo;

#endif /* OS_TYPE_FREERTOS */

/* Data structure for refer series service call */
#if !defined(_POSIX) /* _POSIX */

typedef struct {
	void *arg;
	void (*function)(void *arg);
	const char *name;
	int priority;
	unsigned long stackSize;
} SYS_CreateTaskParams;

typedef struct {
	SYS_Attribute attribute;
	unsigned int initialCount;
	unsigned int maxCount;
} SYS_CreateSemaphoreParams;

typedef struct {
	SYS_Attribute attribute;
	SYS_FlagPattern initialPattern;
} SYS_CreateFlagParams;

typedef struct {
	SYS_Attribute attribute;
} SYS_CreateMailboxParams;

typedef struct {
	unsigned int blockCount;
	unsigned int blockSize;
	void *pool;
} SYS_CreateMemoryPoolParams;

typedef struct {
	void *arg;
	void (*handler)(SYS_CyclicHandlerInfo info);
	const char *name;
	SYS_RelativeTime cycle;
} SYS_CreateCyclicHandlerParams;

typedef struct {
	void *arg;
	void (*handler)(SYS_AlarmHandlerInfo info);
	const char *name;
} SYS_CreateAlarmHandlerParams;

typedef struct TaskStatus {
	SYS_Status status;
	int priority;
} SYS_TaskStatus;

typedef struct SemaphoreStatus {
	unsigned int semaphoreCount;	/* Current count */
} SYS_SemaphoreStatus;

typedef struct FlagStatus {
	SYS_FlagPattern flagPattern;	/* Current pattern */
} SYS_FlagStatus;

typedef struct MailboxStatus {
	SYS_Message *message;		/* Next receive message */
} SYS_MailboxStatus;

typedef struct MemoryPoolStatus {
	unsigned int freeBlocks;
} SYS_MemoryPoolStatus;

typedef struct CyclicHandlerStatus {
	SYS_Status status;
} SYS_CyclicHandlerStatus;

typedef struct AlarmHandlerStatus {
	SYS_Status status;
} SYS_AlarmHandlerStatus;

/* Task Management Functions */
int SYS_CreateTask(SYS_Task *desc, const SYS_CreateTaskParams *params);
int SYS_DeleteTask(SYS_Task desc);
int SYS_ChangeTaskPriority(SYS_Task desc, int prio);
int SYS_GetTaskPriority(SYS_Task desc, int *prio);
int SYS_ReferTask(SYS_Task desc, SYS_TaskStatus *taskStatus);

/* Task Dependent Synchronization Functions */
int SYS_SleepTask(SYS_Timeout timeout);
int SYS_SuspendTask(SYS_Task desc);
int SYS_ResumeTask(SYS_Task desc);
int SYS_DelayTask(SYS_RelativeTime delay);

/* Synchronization and Communication Functions */
int SYS_CreateSemaphore(SYS_Id id, const SYS_CreateSemaphoreParams *params);
int SYS_DeleteSemaphore(SYS_Id id);
int SYS_SignalSemaphore(SYS_Id id);
int SYS_WaitSemaphore(SYS_Id id, SYS_Timeout timeout);
int SYS_ReferSemaphore(SYS_Id id, SYS_SemaphoreStatus *semaphoreStatus);

int SYS_CreateFlag(SYS_Id id, const SYS_CreateFlagParams *params);
int SYS_DeleteFlag(SYS_Id id);
int SYS_SetFlag(SYS_Id id, SYS_FlagPattern setPattern);
int SYS_ClearFlag(SYS_Id id, SYS_FlagPattern clearPattern);
int SYS_WaitFlag(SYS_Id id, SYS_FlagPattern waitPattern, SYS_WaitMode waitMode, SYS_FlagPattern *flagPattern, SYS_Timeout timeout);
int SYS_ReferFlag(SYS_Id id, SYS_FlagStatus *flagStatus);

int SYS_CreateMailbox(SYS_Id id, const SYS_CreateMailboxParams *params);
int SYS_DeleteMailbox(SYS_Id id);
int SYS_SendMailbox(SYS_Id id, SYS_Message *message);
int SYS_ReceiveMailbox(SYS_Id id, SYS_Message **message, SYS_Timeout timeout);
int SYS_ReferMailbox(SYS_Id id, SYS_MailboxStatus *mailboxStatus);

#if defined(CPU_CLUSTER_M0P) && defined(CONFIG_OS_PM_SUPPORT)
int SYS_SignalSemaphorePowerCare(SYS_Id id);
int SYS_WaitSemaphorePowerCare(SYS_Id id, SYS_Timeout timeout);
int SYS_ReferSemaphorePowerCare(SYS_Id id, SYS_SemaphoreStatus *semaphoreStatus);

int SYS_SetFlagPowerCare(SYS_Id id, SYS_FlagPattern setPattern);
int SYS_ClearFlagPowerCare(SYS_Id id, SYS_FlagPattern clearPattern);
int SYS_WaitFlagPowerCare(SYS_Id id, SYS_FlagPattern waitPattern, SYS_WaitMode waitMode, SYS_FlagPattern *flagPattern, SYS_Timeout timeout);
int SYS_ReferFlagPowerCare(SYS_Id id, SYS_FlagStatus *flagStatus);

int SYS_SendMailboxPowerCare(SYS_Id id, SYS_Message *message);
int SYS_ReceiveMailboxPowerCare(SYS_Id id, SYS_Message **message, SYS_Timeout timeout);
int SYS_ReferMailboxPowerCare(SYS_Id id, SYS_MailboxStatus *mailboxStatus);

#endif /* defined(CPU_CLUSTER_M0P) && defined(CONFIG_OS_PM_SUPPORT) */

/* Memory Pool Management Fucntions */
void *SYS_AllocMemory(unsigned int size);
void SYS_FreeMemory(void *ptr);
void *SYS_AllocSharedMemory(unsigned int size);
void SYS_FreeSharedMemory(void *ptr);
int SYS_CreateMemoryPool(SYS_Id id, const SYS_CreateMemoryPoolParams *params);
int SYS_DeleteMemoryPool(SYS_Id id);
int SYS_GetMemoryBlock(SYS_Id id, void **block, SYS_Timeout timeout);
int SYS_AcquireMemoryBlock(SYS_Id id, void *block);
int SYS_ReleaseMemoryBlock(SYS_Id id, void *block);
int SYS_ReferMemoryPool(SYS_Id id, SYS_MemoryPoolStatus *status);

/* Time Management Functions */
int SYS_GetTime(SYS_Time *time);
int SYS_CreateCyclicHandler(SYS_CyclicHandler *desc, const SYS_CreateCyclicHandlerParams *params);
int SYS_DeleteCyclicHandler(SYS_CyclicHandler desc);
int SYS_StartCyclicHandler(SYS_CyclicHandler desc);
int SYS_StopCyclicHandler(SYS_CyclicHandler desc);
int SYS_GetCyclicHandlerArg(SYS_CyclicHandlerInfo info, void **arg);
int SYS_ReferCyclicHandler(SYS_CyclicHandler desc, SYS_CyclicHandlerStatus *status);
int SYS_CreateAlarmHandler(SYS_AlarmHandler *desc, const SYS_CreateAlarmHandlerParams *params);
int SYS_DeleteAlarmHandler(SYS_AlarmHandler desc);
int SYS_StartAlarmHandler(SYS_AlarmHandler desc, SYS_RelativeTime time);
int SYS_StopAlarmHandler(SYS_AlarmHandler desc);
int SYS_GetAlarmHandlerArg(SYS_AlarmHandlerInfo info, void **arg);
int SYS_ReferAlarmHandler(SYS_AlarmHandler desc, SYS_AlarmHandlerStatus *status);

/* System State Management Functions */
int SYS_GetTaskDescriptor(SYS_Task *desc);
int SYS_YieldTask(void);
int SYS_DisableDispatch(void);
int SYS_EnableDispatch(void);
bool SYS_IsDispatchDisabled(void);
int SYS_DisableInterrupt(int irqNum);
int SYS_EnableInterrupt(int irqNum);
void SYS_SaveInterruptMask(uint32_t *mask);
void SYS_RestoreInterruptMask(uint32_t mask);
bool SYS_GetInterruptMask(void);
bool SYS_IsInTaskContext(void);
int SYS_GetCpuId(void);
int SYS_GetCpuClusterId(void);

#endif /* _POSIX */

#ifdef _POSIX /* _POSIX */
int SYS_DisableDispatch(void);
int SYS_EnableDispatch(void);
int SYS_GetCpuId(void);
#endif /* _POSIX */

/* UINT tskstat */
#define TTS_RUN		(0x0001)
#define TTS_RDY		(0x0002)
#define TTS_WAI 	(0x0004)

/* TMP tmout */
#define TMO_POL		(0)
#define TMO_FEVR	(-1)

/* UINT wfmode */
#define TWF_ANDW	(0x0000)
#define TWF_ORW		(0x0001)

#define	TA_CLR		(0x0004)

#define TSK_SELF	(0)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _OS_WRAPPER_H_ */
