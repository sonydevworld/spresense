/****************************************************************************
 * modules/include/memutils/message/cache.h
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

#ifndef CACHE_H_INCLUDED
#define CACHE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define cache_init()
#define cache_sync()
#define Icache_clear(a,b)
#define Icache_clear_sync(a,b)
#define Icache_all_clear()
#define Icache_all_clear_sync()
#define Dcache_clear(a,b)
#define Dcache_clear_sync(a,b)
#define Dcache_all_clear()
#define Dcache_all_clear_sync()
#define Dcache_flush(a,b)
#define Dcache_flush_sync(a,b)
#define Dcache_flush_clear(a,b)
#define Dcache_flush_clear_sync(a,b)
#define Dcache_all_flush_clear()
#define Dcache_all_flush_clear_sync()
/* for DoorBell handler */
#define Dcache_all_flush_clear_sync_on_isr()

#define Dcache_unaligned_clear(a,b)
#define Dcache_unaligned_clear_sync(a,b)
#define Dcache_unaligned_safe_clear(a,b)
#define Dcache_unaligned_safe_clear_sync(a,b)
#define Dcache_unaligned_flush(a,b)
#define Dcache_unaligned_flush_sync(a,b)
#define Dcache_unaligned_flush_clear(a,b)
#define Dcache_unaligned_flush_clear_sync(a,b)

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */
#endif /* CACHE_H_INCLUDED */
