/****************************************************************************
 * gnss_atcmd/gnss_atcmd.h
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

#ifndef __EXAMPLES_GNSS_ATCMD_GNSS_ATCMD_H
#define __EXAMPLES_GNSS_ATCMD_GNSS_ATCMD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gnss_usbserial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_ATCMD_WRITE_FD GNSS_SERIAL_WRITE_FD
#define GNSS_ATCMD_READ_FD  GNSS_SERIAL_READ_FD

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct gnss_atcmd_info
{
  int gnssfd;
  int wfd;
  int rfd;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern int gnss_atcmd_exec(FAR struct gnss_atcmd_info *info, FAR char *line,
                           int size);
extern int gnss_atcmd_printf(int fd, FAR const IPTR char *fmt, ...);

#endif /* __EXAMPLES_GNSS_ATCMD_GNSS_ATCMD_H */
