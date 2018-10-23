/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bt_debug.h
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
 * @file       bt_debug.h
 */
#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_BT_DEBUG_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_BT_DEBUG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BLUETOOTH
# define btdbg(format, ...)    _err(format, ##__VA_ARGS__)
# define btlldbg(format, ...)  lldbg(format, ##__VA_ARGS__)
# define btvdbg(format, ...)   vdbg(format, ##__VA_ARGS__)
# define btllvdbg(format, ...) llvdbg(format, ##__VA_ARGS__)
#else
# define btdbg(format, ...)  printf(format, ##__VA_ARGS__)
# define btlldbg(format, ...)  printf(format, ##__VA_ARGS__)
# define btvdbg(format, ...)  printf(format, ##__VA_ARGS__)
# define btllvdbg(format, ...)  printf(format, ##__VA_ARGS__)
#endif

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_BT_DEBUG_H */
