/****************************************************************************
 * modules/audiolite/worker/common/alworker_commfw.h
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#ifndef __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_H
#define __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <asmp/stdio.h>
#include <nuttx/queue.h>

#include "audiolite/alworker_comm.h"
#include "alworker_commfw_config.h"
#include "alworker_memblk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALWORKER_DECMODE_JUSTDECODE (0)
#define ALWORKER_DECMODE_ALLMEMORY  (1)

/* MACROs Related on Callback settings */

#define SET_STOPMSG(s, c)   do { (s)->on_stopmsg = c; }while(0)
#define SET_PLAYMSG(s, c)   do { (s)->on_playmsg = c; }while(0)
#define SET_TERMMSG(s, c)   do { (s)->on_termmsg = c; }while(0)
#define SET_PROCESS(s, c)   do { (s)->on_process = c; }while(0)
#define SET_PARAMMSG(s, c)  do { (s)->on_parammsg = c; }while(0)

#ifndef NOTUSE_STARTING
#  define SET_STARTING(s, c)  do { (s)->on_starting = c; }while(0)
#else
#  define SET_STARTING(s, c)
#endif

#ifndef NOTUSE_STOPPING
#  define SET_STOPPING(s, c)  do { (s)->on_stopping = c; }while(0)
#else
#  define SET_STOPPING(s, c)
#endif

#ifndef NOTUSE_SYSPAUSE
#  define SET_PAUSEMSG(s, c) do { (s)->on_pausemsg = c; }while(0)
#  define SET_PAUSING(s, c)  do { (s)->on_pausing = c; }while(0)
#else
#  define SET_PAUSEMSG(s, c)
#  define SET_PAUSING(s, c)
#endif

#ifndef NOTUSE_SYSDBG
#  define SET_DBGMSG(s, c)  do { (s)->on_dbgmsg = c; }while(0)
#else
#  define SET_DBGMSG(s, c)
#endif

#ifndef NOTUSE_ORGMSG
#  define SET_USRMSG(s, c)  do { (s)->on_usrorgmsg = c; }while(0)
#else
#  define SET_USRMSG(s, c)
#endif

#ifndef NOTUSE_INSTGAIN
#  define SET_GAINMSG(s, c)  do { (s)->on_gainmsg = c; }while(0)
#else
#  define SET_GAINMSG(s, c)
#endif

/* MACROs Related on Framework Instance and memblk queue */

#define FREE_MEMBLK(m, i) alworker_free(m, (alworker_insthead_t *)i)

#if CONF_WORKER_IMEMMAX > 0
#  define INPUT_MEM_INSTANCE \
      sq_queue_t ifreeq;  \
      sq_queue_t iavailq; \
      memblk_t   imemblk[CONF_WORKER_IMEMMAX];
#  define SET_IMEMINJECT(s, c)  do { (s)->on_imeminject = c; }while(0)
#  define IN_FREEQ(instobj)   (&(instobj)->ifreeq)
#  define IN_AVAILQ(instobj)  (&(instobj)->iavailq)
#  define TAKE_FREE_IMEM(instobj)     \
          ((memblk_t *)sq_remfirst(IN_FREEQ(instobj)))
#  define PUSH_FREE_IMEM(e, instobj)  \
          sq_addlast((sq_entry_t *)e, IN_FREEQ(instobj))
#  define PEEK_IMEM(instobj)    ((memblk_t *)sq_peek(IN_AVAILQ(instobj)))
#  define TAKE_IMEM(instobj)    ((memblk_t *)sq_remfirst(IN_AVAILQ(instobj)))
#  define PUSH_IMEM(e, instobj) sq_addlast((sq_entry_t *)e, IN_AVAILQ(instobj))
#  define IMEM_NUM(instobj)     sq_count(IN_AVAILQ(instobj))
#else
#  define INPUT_MEM_INSTANCE
#  define SET_IMEMINJECT(s, c)
#  define IN_FREEQ(instobj)         (NULL)
#  define IN_AVAILQ(instobj)        (NULL)
#  define TAKE_FREE_IMEM(instobj)   ((memblk_t *)NULL)
#  define PUSH_FREE_IMEM(e, instobj)
#  define PEEK_IMEM(instobj)        ((memblk_t *)NULL)
#  define TAKE_IMEM(instobj)        ((memblk_t *)NULL)
#  define PUSH_IMEM(e, instobj)
#  define IMEM_NUM(instobj)         (0)
#endif

#if CONF_WORKER_OMEMMAX > 0
#  define OUTPUT_MEM_INSTANCE \
      sq_queue_t ofreeq;  \
      sq_queue_t oavailq; \
      memblk_t   omemblk[CONF_WORKER_OMEMMAX];
#  define SET_OMEMINJECT(s, c)  do { (s)->on_omeminject = c; }while(0)
#  define OUT_FREEQ(instobj)  (&(instobj)->ofreeq)
#  define OUT_AVAILQ(instobj) (&(instobj)->oavailq)
#  define TAKE_FREE_OMEM(instobj)     \
          ((memblk_t *)sq_remfirst(OUT_FREEQ(instobj)))
#  define PUSH_FREE_OMEM(e, instobj)  \
          sq_addlast((sq_entry_t *)e, OUT_FREEQ(instobj))
#  define PEEK_OMEM(instobj)    ((memblk_t *)sq_peek(OUT_AVAILQ(instobj)))
#  define TAKE_OMEM(instobj)    ((memblk_t *)sq_remfirst(OUT_AVAILQ(instobj)))
#  define PUSH_OMEM(e, instobj) sq_addlast((sq_entry_t *)e, OUT_AVAILQ(instobj))
#  define OMEM_NUM(instobj)     sq_count(OUT_AVAILQ(instobj))
#else
#  define OUTPUT_MEM_INSTANCE
#  define SET_OMEMINJECT(s, c)
#  define OUT_FREEQ(instobj)        (NULL)
#  define OUT_AVAILQ(instobj)       (NULL)
#  define TAKE_FREE_OMEM(instobj)   ((memblk_t *)NULL)
#  define PUSH_FREE_OMEM(e, instobj)
#  define PEEK_OMEM(instobj)        ((memblk_t *)NULL)
#  define TAKE_OMEM(instobj)        ((memblk_t *)NULL)
#  define PUSH_OMEM(e, instobj)
#  define OMEM_NUM(instobj)         (0)
#endif

#define ALWORKERCOMMFW_INSTANCE \
    struct {  \
      unsigned char state;  \
      INPUT_MEM_INSTANCE  \
      OUTPUT_MEM_INSTANCE \
    };

/* MACROs Related on State */

#define COMMFW_BLOCK_WAITMSG            (0x80)

#define COMMFW_STATE_STOPPED      (0 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_STARTING     (1)
#define COMMFW_STATE_PROCESS      (2)
#define COMMFW_STATE_WAIT_IMEM    (3 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_WAIT_OMEM    (4 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_PAUSING      (5)
#define COMMFW_STATE_WAIT_P_IMEM  (6 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_WAIT_P_OMEM  (7 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_PAUSED       (8 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_STOPPING     (9)
#define COMMFW_STATE_WAIT_S_IMEM  (10 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_WAIT_S_OMEM  (11 | COMMFW_BLOCK_WAITMSG)
#define COMMFW_STATE_TERM         (12)

#define IS_BLOCKMSG(i)  ((i)->state & COMMFW_BLOCK_WAITMSG)
#define IS_WORKER_RUNNING(i) ((i)->state != COMMFW_STATE_TERM)

#ifdef ALWORKER_COMMFW_DEBUG_STATE
#  define COMMFW_STATECHG(i,s)  do { \
     printf("[WORKER} State %d -> %d\n", (i)->state, COMMFW_STATE_##s); \
     ((i)->state = COMMFW_STATE_##s); \
   }while(0)
#else
#  define COMMFW_STATECHG(i,s)  ((i)->state = COMMFW_STATE_##s)
#endif

#define COMMFW_STATECHG_STOPPED(i)      COMMFW_STATECHG(i, STOPPED)
#define COMMFW_STATECHG_STARTING(i)     COMMFW_STATECHG(i, STARTING)
#define COMMFW_STATECHG_PROCESS(i)      COMMFW_STATECHG(i, PROCESS)
#define COMMFW_STATECHG_WAIT_IMEM(i)    COMMFW_STATECHG(i, WAIT_IMEM)
#define COMMFW_STATECHG_WAIT_OMEM(i)    COMMFW_STATECHG(i, WAIT_OMEM)
#define COMMFW_STATECHG_PAUSING(i)      COMMFW_STATECHG(i, PAUSING)
#define COMMFW_STATECHG_WAIT_P_IMEM(i)  COMMFW_STATECHG(i, WAIT_P_IMEM)
#define COMMFW_STATECHG_WAIT_P_OMEM(i)  COMMFW_STATECHG(i, WAIT_P_OMEM)
#define COMMFW_STATECHG_PAUSED(i)       COMMFW_STATECHG(i, PAUSED)
#define COMMFW_STATECHG_STOPPING(i)     COMMFW_STATECHG(i, STOPPING)
#define COMMFW_STATECHG_WAIT_S_IMEM(i)  COMMFW_STATECHG(i, WAIT_S_IMEM)
#define COMMFW_STATECHG_WAIT_S_OMEM(i)  COMMFW_STATECHG(i, WAIT_S_OMEM)
#define COMMFW_STATECHG_TERM(i)         COMMFW_STATECHG(i, TERM)

#define COMMFW_IS_STATE(i, s)       ((i)->state == COMMFW_STATE_##s)

#define IS_STATE_STOPPED(i)      COMMFW_IS_STATE(i, STOPPED)
#define IS_STATE_STARTING(i)     COMMFW_IS_STATE(i, STARTING)
#define IS_STATE_PROCESS(i)      COMMFW_IS_STATE(i, PROCESS)
#define IS_STATE_WAIT_IMEM(i)    COMMFW_IS_STATE(i, WAIT_IMEM)
#define IS_STATE_WAIT_OMEM(i)    COMMFW_IS_STATE(i, WAIT_OMEM)
#define IS_STATE_PAUSING(i)      COMMFW_IS_STATE(i, PAUSING)
#define IS_STATE_WAIT_P_IMEM(i)  COMMFW_IS_STATE(i, WAIT_P_IMEM)
#define IS_STATE_WAIT_P_OMEM(i)  COMMFW_IS_STATE(i, WAIT_P_OMEM)
#define IS_STATE_PAUSED(i)       COMMFW_IS_STATE(i, PAUSED)
#define IS_STATE_STOPPING(i)     COMMFW_IS_STATE(i, STOPPING)
#define IS_STATE_WAIT_S_IMEM(i)  COMMFW_IS_STATE(i, WAIT_S_IMEM)
#define IS_STATE_WAIT_S_OMEM(i)  COMMFW_IS_STATE(i, WAIT_S_OMEM)

/* Return code of process state */

#define AL_COMMFW_RET_OK      (0)
#define AL_COMMFW_RET_NOIMEM  (1)
#define AL_COMMFW_RET_NOOMEM  (2)
#define AL_COMMFW_RET_STAY    (3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct alworker_commfw_insthead_s
{
  unsigned char state;
  INPUT_MEM_INSTANCE
  OUTPUT_MEM_INSTANCE
};
typedef struct alworker_commfw_insthead_s alworker_insthead_t;

struct alworker_commfw_callbacks_s
{
  void (*on_stopmsg)(int state, void *arg);
  int (*on_playmsg)(int state, void *arg, al_comm_msgopt_t *opt);
  void (*on_termmsg)(void *arg,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
  int (*on_process)(void *arg);
  int (*on_parammsg)(int state, void *arg,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
#if CONF_WORKER_IMEMMAX > 0
  int (*on_imeminject)(int state, void *arg);
#endif
#if CONF_WORKER_OMEMMAX > 0
  int (*on_omeminject)(int state, void *arg);
#endif
#ifndef NOTUSE_STARTING
  int (*on_starting)(void *arg);
#endif
#ifndef NOTUSE_STOPPING
  int (*on_stopping)(void *arg);
#endif
#ifndef NOTUSE_SYSPAUSE
  int (*on_pausemsg)(int state, void *arg);
  int (*on_pausing)(void *arg);
#endif
#ifndef NOTUSE_SYSDBG
  int (*on_dbgmsg)(int state, void *arg,
                   al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
#endif
#ifndef NOTUSE_ORGMSG
  int (*on_usrorgmsg)(int state, void *arg,
                       al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
#endif
#ifndef NOTUSE_INSTGAIN
  int (*on_gainmsg)(int state, void *arg, float gain);
#endif
};
typedef struct alworker_commfw_callbacks_s alcommfw_cbs_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* alworker_commfw_initialize()
 * Innitiaizing worker side instance of asmp.
 *
 * @Return Code: Always AL_COMM_ERR_SUCCESS
 */

int alworker_commfw_initialize(alworker_insthead_t *inst);

/* alworker_commfw_get_cbtable()
 * Get callback table.
 */

alcommfw_cbs_t *alworker_commfw_get_cbtable(void);

/* alworker_commfw_pollmessage()
 * Handle messages from main-core.
 *
 * @param [in] block : Set 1 to block this function returned until
 *                     comming next message from main-core.
 *                     0 means just check if a message is came or not.
 * @param [in] arg   :
 *                         to contain a input memory.
 */

void alworker_commfw_pollmessage(alworker_insthead_t *arg);

void alworker_commfw_waitresp(alworker_insthead_t *inst,
                              al_comm_msghdr_t snd);

/* alworker_commfw_msgloop()
 * Message loop for this framework.
 * When this is called, start waiting message from HOST.
 * And call registered callbacks depending on the state and a message.
 * In this loop, state transition is also done.
 *
 * @param [in] arg : Instance of this framework.
 */

void alworker_commfw_msgloop(alworker_insthead_t *arg);

/* Send messages from worker */

int alworker_send_frameinfo(int id, int chs, int hz, int layer, int rate);
int alworker_send_framedone(int id);
int alworker_send_errormsg(int id, int errcode);
int alworker_send_bootmsg(int version, void *d);
int alworker_send_usrcmd(al_comm_msgopt_t *opt);
int alworker_resp_usrcmd(int code, al_comm_msgopt_t *opt);
int alworker_send_debug(alworker_insthead_t *inst, unsigned char hdr_opt);
int alworker_release_imem(alworker_insthead_t *inst, int id, memblk_t *memblk);
int alworker_release_omem(alworker_insthead_t *inst, memblk_t *memblk, int mode);
int alworker_free(memblk_t *mem, alworker_insthead_t *inst);

#endif /* __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_H */
