/****************************************************************************
 * modules/lte_ext/lapiext/alt1250_factory_reset.h
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#ifndef __ALT1250_FACTORY_RESET_H
#define __ALT1250_FACTORY_RESET_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command definitions */

#define ALT1250_FRESET_CMD_INQUIRE_PRE_RESET 0x1201
#define ALT1250_FRESET_CMD_PRE_RESET         0x1202
#define ALT1250_FRESET_CMD_FACTORY_RESET     0x1203

/* Result code definitions */

#define ALT1250_FRESET_PRE_RESET_NEEDED      0x1204
#define ALT1250_FRESET_PRE_RESET_UNNEEDED    0x1205
#define ALT1250_FRESET_RESLT_OK              0x1206
#define ALT1250_FRESET_RESLT_ERROR           0x1207

#endif /* __ALT1250_FACTORY_RESET_H */
