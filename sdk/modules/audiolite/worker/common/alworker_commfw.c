/****************************************************************************
 * modules/audiolite/worker/mp3dec/sprmp3_msghandler.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <errno.h>
#include <asmp/stdio.h>
#include <asmp/delay.h>
#include <nuttx/queue.h>
#include <alworker_memblk.h>

#include "audiolite/alworker_comm.h"
#include "alworker_commfw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef ALWORKER_COMMFW_DEBUG
#  define ALWORKER_TRACE()  printf("[WORKER] %s(%d)\n", __func__, __LINE__)
#  define ALWORKER_DMSG(...)  do { \
     printf("[WORKER] %s(%d) : ", __func__, __LINE__); \
     printf(__VA_ARGS__); \
   }while(0)
#else
#  define ALWORKER_TRACE()
#  define ALWORKER_DMSG(...)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

static al_wtask_t g_worker_task;
static alcommfw_cbs_t g_cbs;

/****************************************************************************
 * Private functions
 ****************************************************************************/

/*** name: release_allmem */

static void release_allmem(alworker_insthead_t *inst)
{
  memblk_t *mem;
#if CONF_WORKER_IMEMMAX > 0
  while ((mem = TAKE_IMEM(inst)) != NULL)
    {
      alworker_release_imem(inst, 0, mem);
    }
#endif

#if CONF_WORKER_OMEMMAX > 0
  while ((mem = TAKE_OMEM(inst)) != NULL)
    {
      alworker_release_omem(inst, mem, ALWORKER_DECMODE_ALLMEMORY);
    }
#endif
}

/*** name: handle_system_msg */

static int handle_system_msg(alworker_insthead_t *inst,
                             al_comm_msghdr_t hdr,
                             al_comm_msgopt_t *opt)
{
  unsigned char ret = AL_COMM_MSGCODEERR_UNKNOWN;

  switch (hdr.code)
    {
      case AL_COMM_MSGCODESYS_STOP:
        if (!(IS_STATE_STOPPED(inst) || IS_STATE_STOPPING(inst)))
          {
            ret = AL_COMM_MSGCODEERR_OK;
            if (g_cbs.on_stopmsg)
              {
                g_cbs.on_stopmsg(inst->state, inst);
              }

            COMMFW_STATECHG_STOPPING(inst);
          }
        else
          {
            ret = AL_COMM_MSGCODEERR_INVALIDSTATE;
          }

        break;

      case AL_COMM_MSGCODESYS_PLAY:
        ret = AL_COMM_MSGCODEERR_INVALIDSTATE;
        if (IS_STATE_STOPPED(inst)  ||
            IS_STATE_STOPPING(inst) ||
            IS_STATE_PAUSING(inst)  ||
            IS_STATE_PAUSED(inst))
          {
            ret = AL_COMM_MSGCODEERR_OK;
            if (g_cbs.on_playmsg)
              {
                ret = g_cbs.on_playmsg(inst->state, inst, opt);
              }

            if (ret == AL_COMM_MSGCODEERR_OK)
              {
                COMMFW_STATECHG_STARTING(inst);
              }
          }

        break;

      case AL_COMM_MSGCODESYS_TERM:
        ret = AL_COMM_MSGCODEERR_OK;
        if (g_cbs.on_termmsg)
          {
            g_cbs.on_termmsg(inst, hdr, opt);
          }

        release_allmem(inst);
        COMMFW_STATECHG_TERM(inst);

        break;

      case AL_COMM_MSGCODESYS_PARAM:
        if (g_cbs.on_parammsg)
          {
            ret = g_cbs.on_parammsg(inst->state, inst, hdr, opt);
          }

        break;

#ifndef NOTUSE_SYSPAUSE
      case AL_COMM_MSGCODESYS_PAUSE:
        ret = AL_COMM_MSGCODEERR_INVALIDSTATE;
        if (IS_STATE_PROCESS(inst))
          {
            ret = AL_COMM_MSGCODEERR_OK;
            if (g_cbs.on_pausemsg)
              {
                ret = g_cbs.on_pausemsg(inst->state, inst);
              }

            if (ret == AL_COMM_MSGCODEERR_OK)
              {
                COMMFW_STATECHG_PAUSING(inst);
              }
          }

        break;
#endif

#ifndef NOTUSE_SYSDBG
      case AL_COMM_MSGCODESYS_DBG:
        if (g_cbs.on_dbgmsg)
          {
            ret = g_cbs.on_dbgmsg(inst->state, inst, hdr, opt);
          }

        break;
#endif
    }

  return ret;
}

/*** name: handle_outputmem_msg */

static int handle_outputmem_msg(alworker_insthead_t *inst,
                                al_comm_msghdr_t hdr,
                                al_comm_msgopt_t *opt)
{
#if CONF_WORKER_OMEMMAX > 0
  unsigned char ret = AL_COMM_MSGCODEERR_OK;
  memblk_t *memblk;

  if (hdr.code == AL_COMM_MSGCODEMEM_INJECT)
    {
      if (opt->addr && opt->size > 0)
        {
          memblk = TAKE_FREE_OMEM(inst);
          if (memblk)
            {
              memblk_initout(memblk, (char *)opt->addr, opt->size);
              memset(memblk->addr, 0, memblk->size);
              PUSH_OMEM(memblk, inst);

              if (g_cbs.on_omeminject)
                {
                  ret = g_cbs.on_omeminject(inst->state, inst);
                }

              if (ret == AL_COMM_MSGCODEERR_OK)
                {
                  if (IS_STATE_WAIT_OMEM(inst))
                    {
                      COMMFW_STATECHG_PROCESS(inst);
                    }
                  else if (IS_STATE_WAIT_P_OMEM(inst))
                    {
                      COMMFW_STATECHG_PAUSING(inst);
                    }
                  else if (IS_STATE_WAIT_S_OMEM(inst))
                    {
                      COMMFW_STATECHG_STOPPING(inst);
                    }
                }

              ret = AL_COMM_MSGCODEERR_OK; /* OK for no error return */
            }
          else
            {
              ret = AL_COMM_MSGCODEERR_OVFLOW;
            }
        }
      else
        {
          ret = AL_COMM_MSGCODEERR_INVALIDADDR;
        }
    }
  else
    {
      ret = AL_COMM_MSGCODEERR_UNKNOWN;
    }
#else
  unsigned char ret = AL_COMM_MSGCODEERR_UNKNOWN;
#endif

  return ret;
}

/*** name: handle_inputmem_msg */

static int handle_inputmem_msg(alworker_insthead_t *inst,
                               al_comm_msghdr_t hdr,
                               al_comm_msgopt_t *opt)
{
#if CONF_WORKER_IMEMMAX > 0
  unsigned char ret = AL_COMM_MSGCODEERR_OK;
  memblk_t *memblk;

  if (hdr.code == AL_COMM_MSGCODEMEM_INJECT)
    {
      if (opt->addr && opt->size > 0)
        {
          memblk = TAKE_FREE_IMEM(inst);
          if (memblk)
            {
              ALWORKER_TRACE();
              memblk_initin(memblk, (char *)opt->addr, opt->size, opt->eof);
              memblk->filled = opt->size; /* memory have been filled up */
              PUSH_IMEM(memblk, inst);

              if (g_cbs.on_imeminject)
                {
                  ret = g_cbs.on_imeminject(inst->state, inst);
                }

              if (ret == AL_COMM_MSGCODEERR_OK)
                {
                  /* State change */

                  if (IS_STATE_WAIT_IMEM(inst))
                    {
                      ALWORKER_TRACE();
                      COMMFW_STATECHG_PROCESS(inst);
                    }
                  else if (IS_STATE_WAIT_P_IMEM(inst))
                    {
                      ALWORKER_TRACE();
                      COMMFW_STATECHG_PAUSING(inst);
                    }
                  else if (IS_STATE_WAIT_S_IMEM(inst))
                    {
                      ALWORKER_TRACE();
                      COMMFW_STATECHG_STOPPING(inst);
                    }
                }

              ret = AL_COMM_MSGCODEERR_OK; /* OK for no error return */
            }
          else
            {
              ret = AL_COMM_MSGCODEERR_OVFLOW;
            }
        }
      else
        {
          ret = AL_COMM_MSGCODEERR_INVALIDADDR;
        }
    }
  else
    {
      ret = AL_COMM_MSGCODEERR_UNKNOWN;
    }
#else
  unsigned char ret = AL_COMM_MSGCODEERR_UNKNOWN;
#endif

  return ret;
}

/*** name: handle_instance_msg */

static int handle_instance_msg(alworker_insthead_t *inst,
                                al_comm_msghdr_t hdr,
                                al_comm_msgopt_t *opt)
{
  unsigned char ret = AL_COMM_MSGCODEERR_UNKNOWN;

  switch (hdr.code)
    {
#ifndef NOTUSE_INSTGAIN
      case AL_COMM_MSGCODEINST_GAIN:
        if (g_cbs.on_gainmsg)
          {
            ret = g_cbs.on_gainmsg(inst->state, inst, opt->gain);
          }

        break;
#endif
    }

  return ret;
}

/*** name: handle_message */

static void handle_message(alworker_insthead_t *inst,
                           al_comm_msghdr_t hdr,
                           al_comm_msgopt_t *opt)
{
  int ret = AL_COMM_MSGCODEERR_UNKNOWN;

  ALWORKER_DMSG("Receive G:%d C:%d\n", hdr.grp, hdr.code);

  switch (hdr.grp)
    {
      case AL_COMM_MESSAGE_SYS:
        ret = handle_system_msg(inst, hdr, opt);
        break;

      case AL_COMM_MESSAGE_FMEM:
        ret = handle_inputmem_msg(inst, hdr, opt);
        break;

      case AL_COMM_MESSAGE_OMEM:
        ret = handle_outputmem_msg(inst, hdr, opt);
        break;

      case AL_COMM_MESSAGE_INST:
        ret = handle_instance_msg(inst, hdr, opt);
        break;

      default:
#ifndef NOTUSE_ORGMSG
        if (g_cbs.on_usrorgmsg)
          {
            ret = g_cbs.on_usrorgmsg(inst->state, inst, hdr, opt);
          }
#else
        ret = AL_COMM_MSGCODEERR_UNKNOWN;
#endif
        break;
    }

  if (hdr.type == AL_COMM_MSGTYPE_SYNC || ret != AL_COMM_MSGCODEERR_OK)
    {
      hdr.type = AL_COMM_MSGTYPE_RESP;
      hdr.opt = ret;
      al_send_message(&g_worker_task, hdr, opt);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*** name: alworker_commfw_get_cbtable */

alcommfw_cbs_t *alworker_commfw_get_cbtable(void)
{
  memset(&g_cbs, 0, sizeof(g_cbs));
  return &g_cbs;
}

/*** name: alworker_commfw_initialize */

int alworker_commfw_initialize(alworker_insthead_t *head)
{
  int i;

#if CONF_WORKER_IMEMMAX > 0
  sq_init(IN_FREEQ(head));
  sq_init(IN_AVAILQ(head));
  for (i = 0; i < CONF_WORKER_IMEMMAX; i++)
    {
      PUSH_FREE_IMEM(&head->imemblk[i], head);
    }
#endif

#if CONF_WORKER_OMEMMAX > 0
  sq_init(OUT_FREEQ(head));
  sq_init(OUT_AVAILQ(head));
  for (i = 0; i < CONF_WORKER_OMEMMAX; i++)
    {
      PUSH_FREE_OMEM(&head->omemblk[i], head);
    }
#endif

  COMMFW_STATECHG_STOPPED(head);
  return initialize_alworker(&g_worker_task, "", false);
}

/*** name: alworker_commfw_pollmessage */

void alworker_commfw_pollmessage(alworker_insthead_t *inst)
{
  int block;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  do
    {
      block = IS_BLOCKMSG(inst);
      hdr = al_receive_message(&g_worker_task, &opt, block);
      if (hdr.u32 != AL_COMM_NO_MSG)
        {
          handle_message(inst, hdr, &opt);
        }
    }
  while (!block && hdr.u32 != AL_COMM_NO_MSG);
}

void alworker_commfw_waitresp(alworker_insthead_t *inst,
                              al_comm_msghdr_t snd)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  while (1)
    {
      hdr = al_receive_message(&g_worker_task, &opt, 1);
      if (hdr.u32 != AL_COMM_NO_MSG)
        {
          if (hdr.type == AL_COMM_MSGTYPE_RESP)
            {
              if (hdr.grp != snd.grp || hdr.code != snd.code)
                {
                  printf("[Worker CommFW] Wrang Resp G(exp:%02x, act:%02x), "
                         "C(exp:%02x, act:%02x\n",
                         snd.grp, hdr.grp, snd.code, hdr.code);
                }

              break;
            }

          handle_message(inst, hdr, &opt);
        }
    }
}

void alworker_commfw_msgloop(alworker_insthead_t *inst)
{
  int ret;

  while (IS_WORKER_RUNNING(inst))
    {
      alworker_commfw_pollmessage(inst);

      switch (inst->state)
        {
          case COMMFW_STATE_STARTING:
#ifndef NOTUSE_STARTING
            if (g_cbs.on_starting)
              {
                ret = g_cbs.on_starting((void *)inst);
                if (ret == AL_COMM_MSGCODEERR_OK)
                  {
                    COMMFW_STATECHG_PROCESS(inst);
                  }
              }
            else
#endif
              {
                COMMFW_STATECHG_PROCESS(inst);
              }

            break;

          case COMMFW_STATE_PROCESS:
            if (g_cbs.on_process)
              {
                ret = g_cbs.on_process((void *)inst);
                switch (ret)
                  {
                    case AL_COMMFW_RET_NOIMEM:
                      COMMFW_STATECHG_WAIT_IMEM(inst);
                      break;

                    case AL_COMMFW_RET_NOOMEM:
                      COMMFW_STATECHG_WAIT_OMEM(inst);
                      break;

                    case AL_COMMFW_RET_OK:
                    case AL_COMMFW_RET_STAY:
                    default:

                      /* Do nothing */

                      break;
                  }
              }

            break; /* end of case COMMFW_STATE_PROCESS */

#ifndef NOTUSE_SYSPAUSE
          case COMMFW_STATE_PAUSING:
            if (g_cbs.on_pausing)
              {
                ret = g_cbs.on_pausing((void *)inst);
                switch (ret)
                  {
                    case AL_COMMFW_RET_NOIMEM:
                      COMMFW_STATECHG_WAIT_P_IMEM(inst);
                      break;

                    case AL_COMMFW_RET_NOOMEM:
                      COMMFW_STATECHG_WAIT_P_OMEM(inst);
                      break;

                    case AL_COMMFW_RET_OK:
                      COMMFW_STATECHG_PAUSED(inst);
                      break;

                    case AL_COMMFW_RET_STAY:
                    default:

                      /* Do nothing */

                      break;
                  }
              }
            else
              {
                /* No pausing handler means always ready to go to PAUSED state */

                COMMFW_STATECHG_PAUSED(inst);
              }

            break; /* end of case COMMFW_STATE_PAUSING */
#endif

          case COMMFW_STATE_STOPPING:
#ifndef NOTUSE_STOPPING
            if (g_cbs.on_stopping)
              {
                ret = g_cbs.on_stopping((void *)inst);
                switch (ret)
                  {
                    case AL_COMMFW_RET_NOIMEM:
                      COMMFW_STATECHG_WAIT_S_IMEM(inst);
                      break;

                    case AL_COMMFW_RET_NOOMEM:
                      COMMFW_STATECHG_WAIT_S_OMEM(inst);
                      break;

                    case AL_COMMFW_RET_OK:
                      COMMFW_STATECHG_STOPPED(inst);
                      break;

                    case AL_COMMFW_RET_STAY:
                    default:

                      /* Do nothing */

                      break;
                  }
              }
            else
#endif
              {
                /* No stopping handler means always ready to go to STOPPED state */

                COMMFW_STATECHG_STOPPED(inst);
              }

            break; /* end of case COMMFW_STATE_STOPPING */

          default:
            break;
        } /* end of switch (inst->state) */
    } /* while (IS_WORKER_RUNNING(inst)) */
}

/*** name: send_frameinfo */

int alworker_send_frameinfo(int id, int chs, int hz, int layer, int rate)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_INST;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEINST_INFO;
  hdr.opt  = id;

  opt.dec_chs = chs;
  opt.dec_hz = hz;
  opt.dec_layer = layer;
  opt.dec_kbps = rate;

  return al_send_message(&g_worker_task, hdr, &opt);
}

/*** name: send_framedone */

int alworker_send_framedone(int id)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_INST;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEINST_DONE;
  hdr.opt  = id;

  return al_send_message(&g_worker_task, hdr, &opt);
}

/*** name: send_errormsg */

int alworker_send_errormsg(int id, int errcode)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODESYS_ERR;
  hdr.opt  = id;

  opt.errcode = errcode;

  return al_send_message(&g_worker_task, hdr, &opt);
}

/*** name: send_bootmsg */

int alworker_send_bootmsg(int version, void *d)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODESYS_BOOT;
  hdr.opt  = version;
  opt.addr = (unsigned char *)alworker_addr_convert(d);

  return al_send_message(&g_worker_task, hdr, &opt);
}

int alworker_send_usrcmd(al_comm_msgopt_t *opt)
{
  al_comm_msghdr_t hdr;

  hdr.grp  = AL_COMM_MESSAGE_USER;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = 0;
  hdr.opt  = 0;

  return al_send_message(&g_worker_task, hdr, opt);
}

int alworker_resp_usrcmd(int code, al_comm_msgopt_t *opt)
{
  al_comm_msghdr_t hdr;

  hdr.grp  = AL_COMM_MESSAGE_USER;
  hdr.type = AL_COMM_MSGTYPE_RESP;
  hdr.code = code;
  hdr.opt  = 0;

  return al_send_message(&g_worker_task, hdr, opt);
}

int alworker_send_debug(alworker_insthead_t *inst, unsigned char hdr_opt)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_SYNC;
  hdr.code = AL_COMM_MSGCODESYS_DBG;
  hdr.opt  = hdr_opt;

  al_send_message(&g_worker_task, hdr, &opt);
  alworker_commfw_waitresp(inst, hdr);
  return OK;
}

/*** name: release_framemem */

int alworker_release_imem(alworker_insthead_t *inst, int id, memblk_t *memblk)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_FMEM;
  hdr.type = AL_COMM_MSGTYPE_SYNC;
  hdr.code = AL_COMM_MSGCODEMEM_RELEASE;
  hdr.opt  = (unsigned char)id;

  opt.addr = (unsigned char *)memblk->addr;
  opt.size = memblk->size;
  opt.eof  = memblk->eof;

  al_send_message(&g_worker_task, hdr, &opt);
  alworker_commfw_waitresp(inst, hdr);
  return OK;
}

/*** name: deliver_outpcm */

int alworker_release_omem(alworker_insthead_t *inst, memblk_t *memblk, int mode)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_OMEM;
  hdr.type = AL_COMM_MSGTYPE_SYNC;
  hdr.code = AL_COMM_MSGCODEMEM_RELEASE;
  hdr.opt  = AL_COMM_MSGCODEERR_OK;

  opt.addr = (unsigned char *)memblk->addr;
  opt.size = (mode == ALWORKER_DECMODE_JUSTDECODE) ?
             memblk->filled : memblk->size;
  opt.eof = memblk->eof;

  al_send_message(&g_worker_task, hdr, &opt);
  alworker_commfw_waitresp(inst, hdr);
  return OK;
}

int alworker_free(memblk_t *mem, alworker_insthead_t *inst)
{
  int ret = -EINVAL;

  switch (mem->type)
    {
#if CONF_WORKER_IMEMMAX > 0
      case MEMBLK_TYPE_INPUT:
        ret = alworker_release_imem(inst, 0, mem);
        PUSH_FREE_IMEM(mem, inst);
        break;
#endif

#if CONF_WORKER_OMEMMAX > 0
      case MEMBLK_TYPE_OUTPUT:
        ret = alworker_release_omem(inst, mem, ALWORKER_DECMODE_JUSTDECODE);
        PUSH_FREE_OMEM(mem, inst);
        break;
#endif
    }

  return ret;
}
