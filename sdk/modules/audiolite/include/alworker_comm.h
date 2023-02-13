/****************************************************************************
 * modules/audiolite/include/alworker_comm.h
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

#ifndef __AUDIOLITE_INCLUDE_ALWORKER_COMM_H
#define __AUDIOLITE_INCLUDE_ALWORKER_COMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AL_COMM_MESSAGE_NONE   (0)
#define AL_COMM_MESSAGE_SYS    (1)
#define AL_COMM_MESSAGE_FMEM   (2)
#define AL_COMM_MESSAGE_OMEM   (3)
#define AL_COMM_MESSAGE_INST   (4)

#define AL_COMM_MSGTYPE_NONE  (0)
#define AL_COMM_MSGTYPE_ASYNC (1)
#define AL_COMM_MSGTYPE_SYNC  (2)

#define AL_COMM_MSGCODESYS_NONE  (0)
#define AL_COMM_MSGCODESYS_STOP  (1)
#define AL_COMM_MSGCODESYS_PAUSE (2)
#define AL_COMM_MSGCODESYS_PLAY  (3)
#define AL_COMM_MSGCODESYS_BOOT  (4)
#define AL_COMM_MSGCODESYS_TERM  (5)
#define AL_COMM_MSGCODESYS_PARAM (6)
#define AL_COMM_MSGCODESYS_DBG   (7)

#define AL_COMM_MSGCODEINST_NONE  (0)
#define AL_COMM_MSGCODEINST_START (1)
#define AL_COMM_MSGCODEINST_STOP  (2)
#define AL_COMM_MSGCODEINST_DONE  (3)
#define AL_COMM_MSGCODEINST_GAIN  (4)
#define AL_COMM_MSGCODEINST_INFO  (5)

#define AL_COMM_MSGCODEMEM_NONE    (0)
#define AL_COMM_MSGCODEMEM_INJECT  (1)
#define AL_COMM_MSGCODEMEM_RELEASE (2)

#define AL_COMM_MSGCODEERR_OK            (0)
#define AL_COMM_MSGCODEERR_TOOSHORT      (1)
#define AL_COMM_MSGCODEERR_ILLIGALFRAME  (2)
#define AL_COMM_MSGCODEERR_NOFRAME       (3)
#define AL_COMM_MSGCODEERR_UNKNOWN       (4)
#define AL_COMM_MSGCODEERR_INVALIDADDR   (5)
#define AL_COMM_MSGCODEERR_OVFLOW        (6)
#define AL_COMM_MSGCODEERR_INVALIDINST   (7)
#define AL_COMM_MSGCODEERR_MULTIFRAME    (8)

#define AL_COMM_ERR_SUCCESS      (0)
#define AL_COMM_ERR_WORKERINIT   (-1)
#define AL_COMM_ERR_WORKERASSIGN (-2)
#define AL_COMM_ERR_SENDMQCREATE (-3)
#define AL_COMM_ERR_SENDMQBIND   (-4)
#define AL_COMM_ERR_RECVMQCREATE (-5)
#define AL_COMM_ERR_EXECWORKER   (-6)

#define AL_COMM_NO_MSG (0xffffffff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

union al_comm_msghdr_u
{
  struct
    {
      unsigned char grp;
      unsigned char type;
      unsigned char code;
      unsigned char opt;
    };
  uint32_t u32;
};
typedef union al_comm_msghdr_u al_comm_msghdr_t;

union al_comm_msgopt_u
{
  struct
    {
        unsigned char *addr;
        unsigned int size;
        int eof;
#ifdef __linux__
        unsigned char body[4096];
#endif
    };
  struct
    {
      int chs;
      int hz;
      int mode;
    };
  int errcode;
  float gain;
};
typedef union al_comm_msgopt_u al_comm_msgopt_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void *alworker_addr_convert(void *a);
int initialize_alworker(char *dspfname);
al_comm_msghdr_t al_receive_message(al_comm_msgopt_t *opt, int block);
int al_send_message(al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
int finalize_alworker(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AUDIOLITE_INCLUDE_ALWORKER_COMM_H */
