/****************************************************************************
 * modules/include/asmp/mptask.h
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
/**
 * @file mptask.h
 */

#ifndef __INCLUDE_ASMP_MPTASK_H
#define __INCLUDE_ASMP_MPTASK_H

/**
 * @defgroup mptask MP task
 *
 * @details
 * MP task APIs controlling worker or any other processes running on other CPUs
 * such as pthreads.
 *
 * @{
 */

#include <sys/types.h>
#include <asmp/types.h>
#include <semaphore.h>
#include <stdbool.h>

#ifndef __DOCUMENT__
#ifndef CONFIG_SMP

/* void CPU_ZERO(FAR cpu_set_t *set); */

#  define CPU_ZERO(s) do { *(s) = 0; } while (0)

/* void CPU_SET(int cpu, FAR cpu_set_t *set); */

#  define CPU_SET(c,s) do { *(s) |= (1 << (c)); } while (0)

/* void CPU_CLR(int cpu, FAR cpu_set_t *set); */

#  define CPU_CLR(c,s) do { *(s) &= ~(1 << (c)); } while (0)

/* int  CPU_ISSET(int cpu, FAR const cpu_set_t *set); */

#  define CPU_ISSET(c,s) ((*(s) & (1 << (c))) != 0)

/* int CPU_COUNT(FAR const cpu_set_t *set); */

#  define CPU_COUNT(s) sched_cpu_count(s)

/* void CPU_AND(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#  define CPU_AND(d,s1,s2) do { *(d) = *(s1) & *(s2); } while (0)

/* void CPU_OR(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *             FAR const cpu_set_t *srcset2);
 */

#  define CPU_OR(d,s1,s2) do { *(d) = *(s1) | *(s2); } while (0)

/* void CPU_XOR(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#  define CPU_XOR(d,s1,s2) do { *(d) = *(s1) ^ *(s2); } while (0)

/* int CPU_EQUAL(FAR const cpu_set_t *set1, FAR const cpu_set_t *set2); */

#  define CPU_EQUAL(s1,s2) (*(s2) == *(s2))

/* REVISIT: Variably sized CPU sets are not supported */
/* FAR cpu_set_t *CPU_ALLOC(int num_cpus); */

#  define CPU_ALLOC(n) (FAR cpu_set_t *)malloc(sizeof(cpu_set_t));

/* void CPU_FREE(cpu_set_t *set); */

#  define CPU_FREE(s) free(s)

/* size_t CPU_ALLOC_SIZE(int num_cpus); */

#  define CPU_ALLOC_SIZE(n) sizeof(cpu_set_t)

/* void CPU_ZERO_S(size_t setsize, FAR cpu_set_t *set); */

#  define CPU_ZERO_S(n,s) CPU_ZERO_S(s)

/* void CPU_SET_S(int cpu, size_t setsize, FAR cpu_set_t *set); */

#  define CPU_SET_S(c,n,s) CPU_SET(c,s)

/* void CPU_CLR_S(int cpu, size_t setsize, FAR cpu_set_t *set); */

#  define CPU_CLR_S(c,n,s) CPU_CLR(c,s)

/* int CPU_ISSET_S(int cpu, size_t setsize, FAR const cpu_set_t *set); */

#  define CPU_ISSET_S(c,n,s) CPU_ISSET(c,s)

/* int CPU_COUNT_S(size_t setsize, FAR const cpu_set_t *set); */

#  define CPU_COUNT_S(n,s) CPU_COUNT(s)

/* void CPU_AND_S(size_t setsize, FAR cpu_set_t *destset,
 *                FAR const cpu_set_t *srcset1,
 *                FAR const cpu_set_t *srcset2);
 */

#  define CPU_AND_S(n,d,s1,s2) CPU_AND(d,s1,s2)

/* void CPU_OR_S(size_t setsize, FAR cpu_set_t *destset,
 *              FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#  define CPU_OR_S(n,d,s1,s2) CPU_OR(d,s1,s2)

/* void CPU_XOR_S(size_t setsize, FAR cpu_set_t *destset,
 *                FAR const cpu_set_t *srcset1,
 *                FAR const cpu_set_t *srcset2);
 */

#  define CPU_XOR_S(n,d,s1,s2) CPU_XOR(d,s1,s2)

/* int CPU_EQUAL_S(size_t setsize, FAR const cpu_set_t *set1,
 *                 FAR const cpu_set_t *set2);
 */

#  define CPU_EQUAL_S(n,s1,s2) CPU_EQUAL(s1,s2)

#endif
#endif

#define NMPBINDS 8

/**
 * @def mptask_bindobj
 * Simple wrapper for mptask_bind()
 *
 * @ingroup mptask_funcs
 */

#define mptask_bindobj(t, o) mptask_bind((t), (mpobj_t *)(o))

/**
 * @defgroup mptask_defs Defines
 * @{
 */
/**
 * @enum mptask_state
 * @details Represent the MP task status
 * @image html mptask_statechart.png
 *
 * @typedef mptask_state_t
 */
typedef enum mptask_state
{
  STATE_INIT = 0,               /**< Worker initialized */
  STATE_EXEC,                   /**< Worker is running */
  STATE_PAUSED,                 /**< Worker is paused */
  STATE_EXIT,                   /**< Worker is finished */
} mptask_state_t;

/** @} mptask_defs */

/**
 * @defgroup mptask_datatypes Data types
 * @{
 */

/**
 * MP task attribute structure.
 *
 * Do not refer directly.
 */

typedef struct mptask_attr
{
  int8_t status;                /**< Running state */
  int8_t flags;                 /**< Attribute flags */
  cpu_set_t affinity;           /**< Bit set of permitted CPUs */
  uint32_t exit_status;         /**< Worker exit status */
} mptask_attr_t;

/**
 * @typedef mpbindobj_t
 * @brief Bind object type
 * @note This type is internal use only
 */

typedef struct mpbindobj
{
  key_t       key;
  mpobjtype_t type;
  uint32_t    value;
} mpbindobj_t;

typedef struct unified_binary
{
  int nr_offs;       /* Number of offsets has been stored */
  uint8_t offset[5]; /* Offset table, except CPU 0 */
  uint8_t size[5];   /* Size table, except CPU 0 */
} unified_binary_t;

typedef struct binary_info
{
  uint32_t loadaddr;
} binary_info_t;

/**
 * @typedef mptask_t
 * @brief MP task object
 * @note This type is internal use only
 */

typedef struct mptask
{
  int               fd;
  off_t             filelen;
  uintptr_t         loadaddr;
  size_t            loadsize;
  cpu_set_t         cpuids;
  int               groupid;
  mptask_attr_t     attr;

  /* filename is used when secure loading, and bounds is used for normal binary.
   * So I made them into union for prevent increasing the object size.
   */

  union {
    char              filename[32];
    mpbindobj_t       bounds[NMPBINDS];
  };

  int               nbounds;
  sem_t             wait;

  union {
    unified_binary_t  ubin;     /* Unified binary */
    binary_info_t     bin[5];   /* binary */
  };
} mptask_t;

/** @} mptask_datatypes */

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/
/**
 * @defgroup mptask_funcs Functions
 * @{
 */
/**
 * Initialize MP task
 *
 * Initialize MP task object. User must be call this function before using other
 * MP task APIs.
 *
 * @param [in,out] task: MP task object
 * @param [in] filename: Worker program file name
 *
 * @return On success, mptask_init() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOENT: No such file or directory
 */

int mptask_init(mptask_t *task, const char *filename);

/**
 * Initialize MP task for encrypted firmware
 *
 * Initialize MP task object as encrypted firmware. User must be call this
 * function before using other MP task APIs.
 *
 * @param [in,out] task: MP task object
 * @param [in] filename: Worker encrypted firmware name
 *
 * @return On success, mptask_init_secure() returns 0. On error, it returns an
 * error number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOENT: No such file or directory
 */

int mptask_init_secure(mptask_t *task, const char *filename);

/**
 * Initialize MP task attribute data
 *
 * mptask_atta_init() initialize MP task attribute data.
 *
 * @param [in,out] attr: MP task attribute
 *
 * @return On success, mptask_attr_init() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mptask_attr_init(mptask_attr_t *attr);

/**
 * Destroy MP task
 *
 * mptask_destroy() finishes MP task. User must be call when MP task finished.
 * mptask_destroy() waits MP task exit by mptask_join() if MP task still running.
 *
 * @param [in,out] task: MP task object.
 * @param [in] force: If true, shutdown MP task forcibly.
 * @param [out] exit_status: Exit status of MP task
 *
 * @return On success, mptask_destroy() returns 0.
 */

int mptask_destroy(mptask_t *task, bool force, int *exit_status);

/**
 * Bind MP object to task
 *
 * mptask_bind() binds MP objects (MP message queue, MP mutex and MP shared memory) to
 * MP task. Bound objects will be notified when the worker is bring up.
 * User can use mptask_bindobj() instead of mptask_bind().
 *
 * @param [in,out] task: MP task object.
 * @param [in] obj: Bind MP object
 * 
 * @return On success, mptask_bind() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOENT: Bind object is full
 */

int mptask_bind(mptask_t *task, mpobj_t *obj);

/**
 * Set MP task attribute
 *
 * mptask_setattr() replaces MP task attribute by @a attr.
 * This function is useful to change CPU affinity.
 *
 * @param [in,out] task: MP task object.
 * @param [in] attr: MP task attribute.
 *
 * @return On success, mptask_setattr() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mptask_setattr(mptask_t *task, const mptask_attr_t *attr);

/**
 * Get MP task attribute
 *
 * mptask_getattr() obtaining current MP task attribute.
 *
 * @param [in,out] task: MP task object.
 * @param [out] attr: Current MP task attribute.
 *
 * @return On success, mptask_setattr() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mptask_getattr(mptask_t *task, mptask_attr_t *attr);

/**
 * Assign CPU for MP task
 *
 * mptask_assign() assigns CPU for running MP task. This function automatically call
 * in mptask_exec() when not assigned.
 *
 * @param [in,out] task: MP task object.
 *
 * @return On success, mptask_setattr() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOENT: CPU can't be assigned
 */

int mptask_assign(mptask_t *task);

/**
 * Assign CPUs for MP task
 *
 * mptask_assign_cpus() assigns CPU for running MP task.
 * This function is for secure binary only.
 *
 * @param [in,out] task: MP task object.
 * @param [in] ncpus: Number of CPUs to be assigned.
 *
 * @return On success, mptask_setattr() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOENT: CPU can't be assigned
 */

int mptask_assign_cpus(mptask_t *task, int ncpus);

/**
 * Get assigned CPU ID
 *
 * mptask_getcpuid() returns assigned CPU ID for @a task.
 *
 * @param [in,out] task: MP task object.
 *
 * @return On success, mptask_getcpuid() returns CPU ID (3-7). On error, it returns
 * an error number.
 * @retval -ENOENT: CPU not assigned yet
 */

cpuid_t mptask_getcpuid(mptask_t *task);

/**
 * Get assigned sub core ID
 *
 * mptask_getsubcoreid() returns assigned sub core ID for @a task.
 *
 * @param [in,out] task: MP task object.
 *
 * @return On success, mptask_getcpuid() returns sub core ID (1-5). On error, it returns
 * an error number.
 * @retval -ENOENT: CPU not assigned yet
 */

cpuid_t mptask_getsubcoreid(mptask_t *task);

/**
 * Get assigned CPU ID list
 *
 * mptask_getcpuid() returns assigned CPU ID list for @a task. This API is used for
 * secure binary.
 * CPU ID list can be test with CPU_ISSET() macro.
 *
 * @param [in,out] task: MP task object.
 * @param [out] cpuids: Assigned CPU ID list
 *
 * @return On success, mptask_getcpuid() returns CPU ID list. On error, it returns
 * an error number.
 * @retval -ENOENT: CPU not assigned yet
 */

int mptask_getcpuidset(mptask_t *task, cpu_set_t *cpuids);

/**
 * Execute MP task
 *
 * mptask_exec() load worker ELF or any other ELF programs, and execute it on assigned
 * CPU. If user not assigned by mptask_assign(), then automatically assigned CPU from
 * attribute.
 *
 * @param [in,out] task: MP task object.
 *
 * @return On success, mptask_exec() returns 0. On error, it returns an error number.
 * @retval -EINVAL: Invalid argument
 * @retval -EPERM: @a task not initialized
 * @retval -ENOENT: CPU can't be assigned
 * @retval -ENOMEM: No memory space left
 * @retval -ESPIPE: Insufficient space in file for section header table
 */

int mptask_exec(mptask_t *task);

/**
 * Wait for task exit
 *
 * mptask_join() waits for MP task finish.
 *
 * @param [in,out] task: MP task object
 * @param [out] exit_status: Exit status of MP task
 *
 * @return On success, mptask_join() returns 0. On error, it returns an error number.
 * @retval -EINVAL: Invalid argument
 * @retval -EPERM: MP task already not running
 */

int mptask_join(mptask_t *task, int *exit_status);

#ifdef SDK_EXPERIMENTAL

/**
 * Pause MP task (experimental)
 *
 * mptask_pause() stop task temporary, and can be restart by mptask_restart().
 * This function is stop the running CPU's clock, so it reduce power consumption.
 *
 * @param [in,out] task: MP task object
 *
 * @return 0 on success, mptask_pause() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -EPERM: MP task not running
 */

int mptask_pause(mptask_t *task);

/**
 * Restart paused MP task (experimental)
 *
 * mptask_restart() restart paused task by mptask_pause().
 *
 * @param [in,out] task: MP task object
 *
 * @return 0 on success, mptask_pause() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -EPERM: MP task not pausing
 */

int mptask_restart(mptask_t *task);

#endif /* SDK_EXPERIMENTAL */

/** @} mptask_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} mptask */

#endif /* __INCLUDE_ASMP_MPTASK_H */
