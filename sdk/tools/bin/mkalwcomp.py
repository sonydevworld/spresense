#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/bin/mkalwcomp.py
#
#   Copyright 2024 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import os
import sys
import re
import shutil

############################################################################
### This tool's description

TOOL_DESCRIPTION = '''
Generate a new audiolite worker component and worker sckelton code
'''

EPILOG = '''Generate a audiolite component class in specified directory
with worker sckelton code stored in a sub directory created by this tool.
'''

############################################################################
### Command line options

WORKER_OPT_STRT = 's'
WORKER_OPT_STOP = 'x'
WORKER_OPT_INFO = 'i'
WORKER_OPT_PAUS = 'p'
WORKER_OPT_GAIN = 'g'
WORKER_OPT_USER = 'u'
WORKER_OPT_DEBG = 'd'
WORKER_OPT_DEFT = 'f'
WORKER_OPT_IMEM = 'm'
WORKER_OPT_OMEM = 'o'
WORKER_OPTS = WORKER_OPT_STRT + WORKER_OPT_STOP + WORKER_OPT_INFO + \
              WORKER_OPT_PAUS + WORKER_OPT_GAIN + WORKER_OPT_USER + \
              WORKER_OPT_DEBG + WORKER_OPT_DEFT + WORKER_OPT_IMEM + \
              WORKER_OPT_OMEM

worker_opt_help = \
'''Worker code options, choose one or more in ''' + str(list(WORKER_OPTS)) + '''
    ''' + WORKER_OPT_STRT + ''' : enable starting state support
    ''' + WORKER_OPT_STOP + ''' : enable stopping state support
    ''' + WORKER_OPT_INFO + ''' : enable system information support
    ''' + WORKER_OPT_PAUS + ''' : enable pause support
    ''' + WORKER_OPT_GAIN + ''' : enable gain message support
    ''' + WORKER_OPT_USER + ''' : enable user original message handle
    ''' + WORKER_OPT_DEBG + ''' : enable debug message handle
    ''' + WORKER_OPT_DEFT + ''' : enable start/stop/term message handle
    ''' + WORKER_OPT_IMEM + ''' : enable input memory injection message handle
    ''' + WORKER_OPT_OMEM + ''' : enable output memory injection message handle

'''

COMP_OPT_ERRO = 'e'
COMP_OPT_INFO = 'i'
COMP_OPT_DONE = 'f'
COMP_OPT_TERM = 't'
COMP_OPT_IMEM = 'm'
COMP_OPT_OMEM = 'o'
COMP_OPT_USER = 'u'
COMP_OPT_DEBG = 'd'
COMP_OPTS = COMP_OPT_ERRO + COMP_OPT_INFO + COMP_OPT_DONE + COMP_OPT_TERM + \
            COMP_OPT_IMEM + COMP_OPT_OMEM + COMP_OPT_USER + COMP_OPT_DEBG

comp_opt_help = \
'''Component code options, choose one or more in ''' + str(list(COMP_OPTS)) + '''
    ''' + COMP_OPT_ERRO + ''' : handle error message
    ''' + COMP_OPT_INFO + ''' : handle information message
    ''' + COMP_OPT_DONE + ''' : handle done message
    ''' + COMP_OPT_TERM + ''' : handle term message
    ''' + COMP_OPT_IMEM + ''' : handle release input mem message
    ''' + COMP_OPT_OMEM + ''' : handle release output mem message
    ''' + COMP_OPT_USER + ''' : handle user original message
    ''' + COMP_OPT_DEBG + ''' : handle debug message

'''

############################################################################
### Worker gitignore

WORKER_GITIGNORE_TEMPLATE = \
'''{workername}
*.elf
*.spk
'''

############################################################################
### Worker Configuration File Template

WORKER_CONFIG_TEMPLATE = \
'''#ifndef __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_CONFIG_H
#define __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_CONFIG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** Definition of NOTUSE_XXXXXX
 *
 * If you want to handle messages below,
 * comment out to enable it.
 */

{nouse_strt}
{nouse_stop}
{nouse_paus}
{nouse_gain}
{nouse_user}
{nouse_debg}

/** Definition of Memory block QUEUE size
 *
 * Input side memory block size will defined as CONF_WORKER_IMEMMAX.
 * Output side memory block size will defined as CONF_WORKER_OMEMMAX.
 */

#define CONF_WORKER_IMEMMAX ({imem})
#define CONF_WORKER_OMEMMAX ({omem})

#endif /* __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_CONFIG_H */
'''

CONFIG_DEF_GAIN = '#define NOTUSE_INSTGAIN'
CONFIG_DEF_USER = '#define NOTUSE_ORGMSG'
CONFIG_DEF_STRT = '#define NOTUSE_STARTING'
CONFIG_DEF_STOP = '#define NOTUSE_STOPPING'
CONFIG_DEF_DEBG = '#define NOTUSE_SYSDBG'
CONFIG_DEF_INFO = '#define NOTUSE_SYSPARAM'
CONFIG_DEF_PAUS = '#define NOTUSE_SYSPAUSE'

############################################################################
### Worker Header File Template

WORKER_MAIN_HEADER = '''#ifndef __AUDIOLITE_WORKER_USR_{nameupper}_H
#define __AUDIOLITE_WORKER_USR_{nameupper}_H

/* Add a common definition between worker and the audiolite component here */

#define {nameupper}_WORKER_VERSION (1)

#endif /* __AUDIOLITE_WORKER_USR_{nameupper}_H */
'''

############################################################################
### Worker Source File Template

WORKER_MAIN_SRC = '''/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <asmp/stdio.h>
#include <alworker_commfw.h>
#include "{workername}_worker_main.h"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct my_worker_instance_s
{{
  /* ALWORKERCOMMFW_INSTANCE should be on top of your instance */

  ALWORKERCOMMFW_INSTANCE;

  /* Add specific items for your application here */
}};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct my_worker_instance_s g_instance;
static memblk_t *imem_inuse = NULL;
static memblk_t *omem_inuse = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* on_process():
 *  This is called when the state is in PROCESS.
 *
 *   Return code controls state change.
 *     AL_COMMFW_RET_OK     : Stay the state to process next work.
 *     AL_COMMFW_RET_NOIMEM : Change to WAIT_IMEM for waiting next input.
 *     AL_COMMFW_RET_NOOMEM : Change to WAIT_OMEM for waiting next output.
 */

static int on_process(void *arg)
{{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your original code here.
   * This sckelton code implements just copy input memory to output memory. */

  /* If no memory, get new memory block from queue */

  if (imem_inuse == NULL)
    {{
      imem_inuse = TAKE_IMEM(inst);
    }}

  if (omem_inuse == NULL)
    {{
      omem_inuse = TAKE_OMEM(inst);
    }}

  /* Copy data from input memory to output memory */

  if (imem_inuse && omem_inuse)
    {{
      memblk_fillup(omem_inuse, imem_inuse);
    }}

  /* Release IMEM if all data is used */

  if (imem_inuse && memblk_is_empty(imem_inuse))
    {{
      FREE_MEMBLK(imem_inuse, inst);
      imem_inuse = NULL;
    }}

  /* Release OMEM if it is filled up */

  if (omem_inuse && memblk_is_full(omem_inuse))
    {{
      FREE_MEMBLK(omem_inuse, inst);
      omem_inuse = NULL;
    }}

  return AL_COMMFW_RET_OK;
}}

{on_gain}{on_user}{on_debg}{on_paus}''' + \
'''{on_omem}{on_imem}{on_info}''' + \
'''{on_stop}{on_strt}{on_term}''' + \
'''{on_stpm}{on_play}''' + \
'''/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{{
  alcommfw_cbs_t *cbs = alworker_commfw_get_cbtable();

  if (alworker_commfw_initialize((alworker_insthead_t *)&g_instance) != OK)
    {{
      return 0;
    }}

  /* Set callbacks to handle host message and
   * state processing
   */

  SET_PROCESS(cbs, on_process);
{set_play}{set_stop}{set_term}{set_strt}{set_sping}{set_info}{set_imem}{set_omem}{set_paus}{set_debg}{set_user}{set_gain}
  /* Send boot message to host to notice this worker is ready */

  alworker_send_bootmsg({nameupper}_WORKER_VERSION, NULL);

  /* Start message loop.
   * This function returns receive SYSTERM message from a host.
   */

  alworker_commfw_msgloop((alworker_insthead_t *)&g_instance);

  return 0;
}}
'''

SET_PLAYMSG  = '  SET_PLAYMSG(cbs, on_playmsg);\n'
SET_STOPMSG  = '  SET_STOPMSG(cbs, on_stopmsg);\n'
SET_TERMMSG  = '  SET_TERMMSG(cbs, on_termmsg);\n'
SET_STARTING = '  SET_STARTING(cbs, on_starting);\n'
SET_STOPPING = '  SET_STOPPING(cbs, on_stopping);\n'
SET_PARAM    = '  SET_PARAMMSG(cbs, on_parammsg);\n'
SET_IMEM     = '  SET_IMEMINJECT(cbs, on_imeminject);\n'
SET_OMEM     = '  SET_OMEMINJECT(cbs, on_omeminject);\n'
SET_PAUSE    = '  SET_PAUSEMSG(cbs, on_pausemsg);\n' + \
               '  SET_PAUSING(cbs, on_pausing);\n'
SET_DBGMSG   = '  SET_DBGMSG(cbs, on_dbgmsg);\n'
SET_USRMSG   = '  SET_USRMSG(cbs, on_usrmsg);\n'
SET_GAIN     = '  SET_GAINMSG(cbs, on_gainmsg);\n'

ON_PLAYF = '''/* on_playmsg():
 *  This is called when AL_COMM_MSGCODESYS_PLAY is received from a host.
 *
 *  Return AL_COMMFW_RET_OK, to go to STARTING state,
 *  Non AL_COMMFW_RET_OK makes return a message with a returned code
 *  to the host.
 */

static int on_playmsg(int state, void *arg, al_comm_msgopt_t *opt)
{
  /*
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;
  */

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_STPMF = '''/* on_stopmsg():
 *  This is called when AL_COMM_MSGCODESYS_STOP is received from a host.
 *  After return from this function, state will go to STOPPING.
 */

static void on_stopmsg(int state, void *arg)
{
  /*
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;
  */

  /* Add your code here */
}

'''

ON_TERMF = '''/* on_termmsg():
 *  This is called when AL_COMM_MSGCODESYS_TERM is received from a host.
 *  After return from this function, this worker will be destroied.
 */

static void on_termmsg(void *arg,
                       al_comm_msghdr_t hdr,
                       al_comm_msgopt_t *opt)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */
}

'''

ON_STARTF = '''/* on_starting():
 *  This is called when the state is in STARTING.
 *
 *   This callback is for preparing for processing.
 *   Return AL_COMMFW_RET_OK to go to PROCESS state,
 *   other return code makes the state stay.
 */

static int on_starting(void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_STOPF = '''/* on_stopping():
 *  This is called when the state is in STOPPING.
 *
 *   Return code controls state change.
 *     AL_COMMFW_RET_OK     : Change to STOPPED. (Complete stopping process)
 *     AL_COMMFW_RET_NOIMEM : Change to WAIT_IMEM for waiting next input.
 *     AL_COMMFW_RET_NOOMEM : Change to WAIT_OMEM for waiting next output.
 *     AL_COMMFW_RET_STAY   : Stay the state to continue stopping process.
 */

static int on_stopping(void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_INFOF = '''/* on_parammsg():
 *  This is called when AL_COMM_MSGCODESYS_PARAM is received from a host.
 *
 *   If this returns not AL_COMMFW_RET_OK, the ret code will send to
 *   the host as error code.
 */

static int on_parammsg(int state, void *arg,
                       al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

############################################################################
### Worker Memory Injection callbask code

ON_IMEMF = '''/* on_imeminject():
 *  This is called when AL_COMM_MESSAGE_FMEM is received from a host.
 *
 *  Return AL_COMMFW_RET_OK makes state forward
 */

static int on_imeminject(int state, void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_OMEMF = '''/* on_omeminject():
 *  This is called when AL_COMM_MESSAGE_OMEM is received from a host.
 *
 *  Return AL_COMMFW_RET_OK makes state forward
 */

static int on_omeminject(int state, void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

############################################################################
### Worker Optional message handdler template

ON_PAUSF = '''/* on_pausemsg():
 *  This is called when AL_COMM_MSGCODESYS_PAUSE is received from a host.
 *
 *  Return AL_COMMFW_RET_OK, to go to PAUSING state,
 *   Non AL_COMMFW_RET_OK makes return a message with a returned code
 *   to the host.
 */

static int on_pausemsg(int state, void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

/* on_pausing():
 *  This is called when the state is in PAUSING.
 *
 *   Return code controls state change.
 *     AL_COMMFW_RET_OK     : Change to STOPPED. (Complete pausing process)
 *     AL_COMMFW_RET_NOIMEM : Change to WAIT_IMEM for waiting next input.
 *     AL_COMMFW_RET_NOOMEM : Change to WAIT_OMEM for waiting next output.
 *     AL_COMMFW_RET_STAY   : Stay the state to continue stopping process.
 */

static int on_pausing(void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_DEBGF = '''/* on_dbgmsg():
 *  This is called when AL_COMM_MSGCODESYS_DBG is received from a host.
 *
 *   If this returns not AL_COMMFW_RET_OK, the ret code will send to
 *   the host as error code.
 */

static int on_dbgmsg(int state, void *arg,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_USERF = '''/* on_usrmsg():
 *  This is called when not standard message id is received from a host.
 *
 *   If this returns not AL_COMMFW_RET_OK, the ret code will send to
 *   the host as error code.
 */

static int on_usrmsg(int state, void *arg,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

ON_GAINF = '''/* on_gainmsg():
 *  This is called when AL_COMM_MSGCODEINST_GAIN is received from a host.
 *
 *   If this returns not AL_COMMFW_RET_OK, the ret code will send to
 *   the host as error code.
 */

static int on_gainmsg(int state, void *arg, float gain)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  /* Add your code here */

  return AL_COMMFW_RET_OK;
}

'''

############################################################################
### Worker Makefile Template

MAKEFILE_TEMPLATE = '''include $(APPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

ALWORKER_COMMON = $(SDKDIR)/modules/audiolite/worker/common

BUILD_EXECUTE = 1

# ALWORKER_USE_CMSIS = 1
# ALWORKER_USE_RESAMPLER = 1

BIN = {workername}
SPK = $(BIN).spk

CSRCS = {workername}_worker_main.c

CFLAGS +=
LDLIBPATH =
LDLIBS =

VPASH_DIRS =

INCDIRS =

include $(ALWORKER_COMMON)/mkfiles/alworker.mk
'''

############################################################################
### Template Audiolite component for this worker (header)

COMPONENT_HEADER_TEMPLATE = '''#ifndef __INCLUDE_AUDIOLITE_USER_{nameupper}_H
#define __INCLUDE_AUDIOLITE_USER_{nameupper}_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/al_workercomp.h>
#include <audiolite/al_workercmd.h>
#include <{workername}/{workername}_worker_main.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class alusr_{workername} : public audiolite_workercomp
{{
  protected:
    class {workername}_msglistener : public audiolite_stdworker_msglistener
    {{
      public:
        virtual ~{workername}_msglistener(){{}};
        void bootup(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                    int version, void *d);''' + \
'''{cp_erro}{cp_debg}{cp_info}{cp_done}{cp_term}{cp_imem}{cp_omem}{cp_user}
    }};

    {workername}_msglistener _msglsnr;
    audiolite_mempoolapbuf *_outmempool;

  public:
    alusr_{workername}();
    virtual ~alusr_{workername}();
}};

#endif /* __INCLUDE_AUDIOLITE_USER_{nameupper}_H */
'''

CP_ERROM = '''
        void error(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                   int id, int ercode);
'''

CP_DEBGM = '''
        void debug(audiolite_workercomp *wcomp, al_wtask_t *wtask, int code);
'''

CP_INFOM = '''
        void info(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                  int id, int chs, int fs, int layer, int rate);
'''

CP_DONEM = '''
        void done(audiolite_workercomp *wcomp, al_wtask_t *wtask, int id);
'''

CP_TERMM = '''
        void term(audiolite_workercomp *wcomp);
'''

CP_IMEMM = '''
        audiolite_memapbuf *release_inmem(audiolite_workercomp *wcomp,
                                          al_wtask_t *wtask,
                                          audiolite_memapbuf *mem, int size);
'''

CP_OMEMM = '''
        audiolite_memapbuf *release_outmem(audiolite_workercomp *wcomp,
                                           al_wtask_t *wtask,
                                           audiolite_memapbuf *mem);
'''

CP_USERM = '''
        void usermsg(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
'''

INIT_OUTMEMPOOL = '''  _outmempool = new audiolite_mempoolapbuf;
  _outmempool->create_instance(4096, 8);
  set_mempool(_outmempool);
'''

COMPONENT_CXX_TEMPLATE = '''#include <audiolite/audiolite.h>
#include <alusr_{workername}.h>

/****************************************************************************
 * alusr_{workername} Class Methods
 ****************************************************************************/

alusr_{workername}::alusr_{workername}() :
  audiolite_workercomp("{workername}", {imem}, {omem}), _outmempool(NULL)
{{
  set_msglistener(&_msglsnr);
{init_outmempool}}}

alusr_{workername}::~alusr_{workername}()
{{
  set_mempool(NULL);
  if (_outmempool)
    delete _outmempool;
}}

/****************************************************************************
 * Message Listener Class Methods
 ****************************************************************************/

void alusr_{workername}::{workername}_msglistener::bootup(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int version, void *d)
{{
  // This method is called just after receiving boot-up
  // message from the worker

  if (version == {nameupper}_WORKER_VERSION)
    {{
      alworker_send_systemparam(wtask, wcomp->channels(),
                                       wcomp->samplingrate(),
                                       wcomp->samplebitwidth());
      alworker_send_start(wtask);
    }}
  else
    {{
      wcomp->publish_event(AL_EVENT_WRONGVERSION, version);
    }}
}}'''

COMP_CXX_ERRO = '''

void alusr_{workername}::{workername}_msglistener::error(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int id, int ercode)
{{
}}'''

COMP_CXX_DEBG = '''

void alusr_{workername}::{workername}_msglistener::debug(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int code)
{{
}}'''

COMP_CXX_INFO = '''

void alusr_{workername}::{workername}_msglistener::info(
     audiolite_workercomp *wcomp, al_wtask_t *wtask,
     int id, int chs, int fs, int layer, int rate)
{{
}}'''

COMP_CXX_DONE = '''

void alusr_{workername}::{workername}_msglistener::done(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int id)
{{
}}'''

COMP_CXX_TERM = '''

void alusr_{workername}::{workername}_msglistener::term(
     audiolite_workercomp *wcomp)
{{
}}'''

COMP_CXX_IMEM = '''

audiolite_memapbuf *alusr_{workername}::{workername}_msglistener::release_inmem(
      audiolite_workercomp *wcomp, al_wtask_t *wtask,
      audiolite_memapbuf *mem, int size)
{{
  return NULL;
}}'''

COMP_CXX_OMEM = '''

audiolite_memapbuf *alusr_{workername}::{workername}_msglistener::release_outmem(
      audiolite_workercomp *wcomp, al_wtask_t *wtask,
      audiolite_memapbuf *mem)
{{
  push_data(wcomp, mem);
  return NULL;
}}'''

COMP_CXX_USER = '''

void alusr_{workername}::{workername}_msglistener::usermsg(
      audiolite_workercomp *wcomp, al_wtask_t *wtask,
      al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{{
}}'''

COMPONENT_MK_BEFORE_APP = '''CXXSRCS += alusr_{workername}.cxx

'''

COMPONENT_MK_AFTER_APP = '''
build_{workername}_worker:
\t@$(MAKE) -C {workername} TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

clean_{workername}_worker:
\t@$(MAKE) -C {workername} TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

$(OBJS): build_{workername}_worker

clean:: clean_{workername}_worker
'''

############################################################################
### Template Audiolite component for this worker (cxx source)

def create_worker_makefile(wkdir, wkname):
  worker_make = wkdir + '/Makefile'
  with open(worker_make, "w") as f:
    f.write(MAKEFILE_TEMPLATE.format(workername=wkname))
    print(' [G] Generate ' + worker_make)
  return True

def code_commentout(en, code):
  return '/* ' + code + ' */' if en else code

def code_gen(en, code):
  return code if en else ''

def create_component(tgtdir, wkname, imemq, omemq, opts):
  nameupper = wkname.upper()
  init_outmempool = INIT_OUTMEMPOOL if omemq != 0 else ''

  comp_hfname        = tgtdir + '/' + 'alusr_' + wkname + '.h'
  comp_cxfname       = tgtdir + '/' + 'alusr_' + wkname + '.cxx'
  makefile_name      = tgtdir + '/Makefile'
  keep_makefile_name = tgtdir + '/Makefile_org'

  with open(comp_hfname, "w") as f:
    f.write(COMPONENT_HEADER_TEMPLATE.format(
              nameupper = nameupper,
              workername = wkname,
              cp_erro = code_gen(opts[COMP_OPT_ERRO], CP_ERROM),
              cp_info = code_gen(opts[COMP_OPT_INFO], CP_INFOM),
              cp_done = code_gen(opts[COMP_OPT_DONE], CP_DONEM),
              cp_term = code_gen(opts[COMP_OPT_TERM], CP_TERMM),
              cp_imem = code_gen(opts[COMP_OPT_IMEM] and imemq != 0, CP_IMEMM),
              cp_omem = code_gen(opts[COMP_OPT_OMEM] and omemq != 0, CP_OMEMM),
              cp_user = code_gen(opts[COMP_OPT_USER], CP_USERM),
              cp_debg = code_gen(opts[COMP_OPT_DEBG], CP_DEBGM)))

    print(' [G] Generate ' + comp_hfname)

  with open(comp_cxfname, "w") as f:
    cxx_template = COMPONENT_CXX_TEMPLATE + \
                   code_gen(opts[COMP_OPT_ERRO], COMP_CXX_ERRO) + \
                   code_gen(opts[COMP_OPT_INFO], COMP_CXX_INFO) + \
                   code_gen(opts[COMP_OPT_DONE], COMP_CXX_DONE) + \
                   code_gen(opts[COMP_OPT_TERM], COMP_CXX_TERM) + \
                   code_gen(opts[COMP_OPT_IMEM] and imemq != 0, COMP_CXX_IMEM) + \
                   code_gen(opts[COMP_OPT_OMEM] and omemq != 0, COMP_CXX_OMEM) + \
                   code_gen(opts[COMP_OPT_USER], COMP_CXX_USER) + \
                   code_gen(opts[COMP_OPT_DEBG], COMP_CXX_DEBG)

    f.write(cxx_template.format(nameupper = nameupper,
                                workername = wkname,
                                init_outmempool = init_outmempool,
                                imem = imemq, omem = omemq))
    print(' [G] Generate ' + comp_cxfname)

  if not os.path.exists(makefile_name):
    print('Skip : No ' + makefile_name)
    print('       Skip modification of the Makefile')
    return

  print(' [C] Save ' + makefile_name + ' to ' + keep_makefile_name)
  shutil.move(makefile_name, keep_makefile_name)
  with open(keep_makefile_name, "r") as rf:
    with open(makefile_name, "w") as wf:
      lines = rf.readlines()
      add_content = True

      for l in lines:
        if add_content:
          if 'include' in l and 'Application.mk' in l:
            wf.write(COMPONENT_MK_BEFORE_APP.format(workername = wkname))
            wf.write(l)
            wf.write(COMPONENT_MK_AFTER_APP.format(workername = wkname))
            add_content = False
            print(' [M] Modify ' + makefile_name)
          else:
            wf.write(l)
        else:
          wf.write(l)

def create_worker_source(wkdir, wkname, imemq, omemq, wkopt):
  nouse_paus = code_commentout(wkopt[WORKER_OPT_PAUS], CONFIG_DEF_PAUS)
  nouse_debg = code_commentout(wkopt[WORKER_OPT_DEBG], CONFIG_DEF_DEBG)
  nouse_gain = code_commentout(wkopt[WORKER_OPT_GAIN], CONFIG_DEF_GAIN)
  nouse_user = code_commentout(wkopt[WORKER_OPT_USER], CONFIG_DEF_USER)
  nouse_strt = code_commentout(wkopt[WORKER_OPT_STRT], CONFIG_DEF_STRT)
  nouse_stop = code_commentout(wkopt[WORKER_OPT_STOP], CONFIG_DEF_STOP)

  nameupper = wkname.upper()

  config_fname  = wkdir + '/' + 'alworker_commfw_config.h'
  worker_hfname = wkdir + '/' + wkname + '_worker_main.h'
  worker_mfname = wkdir + '/' + wkname + '_worker_main.c'
  ignore_fname  = wkdir + '/' + '.gitignore'

  # Create .gitignore file
  with open(ignore_fname, "w") as f:
    f.write(WORKER_GITIGNORE_TEMPLATE.format(workername = wkname))
    print(' [G] Generate ' + ignore_fname)

  # Create config file
  with open(config_fname, "w") as f:
    f.write(WORKER_CONFIG_TEMPLATE.format(nouse_paus = nouse_paus,
                                          nouse_debg = nouse_debg,
                                          nouse_gain = nouse_gain,
                                          nouse_user = nouse_user,
                                          nouse_strt = nouse_strt,
                                          nouse_stop = nouse_stop,
                                          imem = imemq, omem = omemq))
    print(' [G] Generate ' + config_fname)

  # Create skelton source header
  with open(worker_hfname, "w") as f:
    f.write(WORKER_MAIN_HEADER.format(nameupper = nameupper))
    print(' [G] Generate ' + worker_hfname)

  # Create skelton source file
  with open(worker_mfname, "w") as f:
    f.write(WORKER_MAIN_SRC.format(workername = wkname,
              nameupper = nameupper,
              on_play   = code_gen(wkopt[WORKER_OPT_DEFT], ON_PLAYF),
              on_stpm   = code_gen(wkopt[WORKER_OPT_DEFT], ON_STPMF),
              on_term   = code_gen(wkopt[WORKER_OPT_DEFT], ON_TERMF),
              on_info   = code_gen(wkopt[WORKER_OPT_INFO], ON_INFOF),
              on_strt   = code_gen(wkopt[WORKER_OPT_STRT], ON_STARTF),
              on_stop   = code_gen(wkopt[WORKER_OPT_STOP], ON_STOPF),
              on_gain   = code_gen(wkopt[WORKER_OPT_GAIN], ON_GAINF),
              on_paus   = code_gen(wkopt[WORKER_OPT_PAUS], ON_PAUSF),
              on_user   = code_gen(wkopt[WORKER_OPT_USER], ON_USERF),
              on_omem   = code_gen(wkopt[WORKER_OPT_OMEM], ON_OMEMF),
              on_imem   = code_gen(wkopt[WORKER_OPT_IMEM], ON_IMEMF),
              on_debg   = code_gen(wkopt[WORKER_OPT_DEBG], ON_DEBGF),
              set_play  = code_gen(wkopt[WORKER_OPT_DEFT], SET_PLAYMSG),
              set_stop  = code_gen(wkopt[WORKER_OPT_DEFT], SET_STOPMSG),
              set_term  = code_gen(wkopt[WORKER_OPT_DEFT], SET_TERMMSG),
              set_info  = code_gen(wkopt[WORKER_OPT_INFO], SET_PARAM),
              set_strt  = code_gen(wkopt[WORKER_OPT_STRT], SET_STARTING),
              set_sping = code_gen(wkopt[WORKER_OPT_STOP], SET_STOPPING),
              set_gain  = code_gen(wkopt[WORKER_OPT_GAIN], SET_GAIN),
              set_paus  = code_gen(wkopt[WORKER_OPT_PAUS], SET_PAUSE),
              set_user  = code_gen(wkopt[WORKER_OPT_USER], SET_USRMSG),
              set_imem  = code_gen(wkopt[WORKER_OPT_IMEM], SET_IMEM),
              set_omem  = code_gen(wkopt[WORKER_OPT_OMEM], SET_OMEM),
              set_debg  = code_gen(wkopt[WORKER_OPT_DEBG], SET_DBGMSG)))
    print(' [G] Generate ' + worker_mfname)

def generate_workercomp(wkname, basedir, imemq, omemq, wk_opt, cp_opt):
  wkdir = basedir + '/' + wkname
  comp_hname  = basedir + '/alusr_' + wkname + '.h'
  comp_cxname = basedir + '/alusr_' + wkname + '.cxx'

  if not os.path.exists(basedir):
    print('Error : No exists : ' + basedir + ' directory.', file=sys.stderr)
    return False

  if os.path.exists(wkdir):
    print('Error : ' + wkdir + ' already exists.', file=sys.stderr)
    return False

  if os.path.exists(comp_hname) or os.path.exists(comp_cxname):
    print('Error : ' + comp_cxname + ' class already exists.', file=sys.stderr)
    return False

  try:
    os.mkdir(wkdir)
  except:
    print('Error : Could not make direcotry : ' + wkdir, file=sys.stderr)
    return False

  create_worker_makefile(wkdir, wkname)
  create_worker_source(wkdir, wkname, imemq, omemq, wk_opt)
  create_component(basedir, wkname, imemq, omemq, cp_opt)

  return True

class char_check(object):
  def __init__(self, strs):
    self.strs = strs
    slist = list(strs)
    self.opt = dict(zip(slist, [False] * len(slist)))

  def __contains__(self, val):
    strs = list(val)
    for s in strs:
      if s in self.opt:
        self.opt[s] = True
      else:
        return False
    return True

  def __iter__(self):
    return iter((self.strs))

if __name__ == '__main__':
  import argparse

  worker_optchk = char_check(WORKER_OPTS)
  comp_optchk = char_check(COMP_OPTS)

  parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                   description = TOOL_DESCRIPTION,
                                   epilog = EPILOG)

  parser.add_argument('workername', metavar='<worker name>',
                      type = str, help = 'New woreker name\n')
  parser.add_argument('-d', '--dir', type=str, default = '.',
                      help = 'Execution directory (default is current directory)\n')
  parser.add_argument('-w', '--worker', type=str, choices = worker_optchk,
                      help = worker_opt_help)
  parser.add_argument('-c', '--component', type=str, choices = comp_optchk,
                      help = comp_opt_help)
  parser.add_argument('-i', '--imem', type=int, choices = range(9),
                      default = 8, help='Input queue depth (default 8)\n')
  parser.add_argument('-o', '--omem', type=int, choices = range(9),
                      default = 8, help='Input queue depth (default 8)\n')
  opts = parser.parse_args()

  print('=== Generating audiolite worker component in ' + opts.dir)
  if generate_workercomp(opts.workername, opts.dir, opts.imem, opts.omem,
                         worker_optchk.opt, comp_optchk.opt):
    print('Done in successfly')
  else:
    print('Error...')

