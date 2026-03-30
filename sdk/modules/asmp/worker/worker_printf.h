/****************************************************************************
 * modules/asmp/worker/worker_printf.h
 *
 *   Copyright 2026 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_ASMP_WORKER_WORKER_PRINTF_H
#define __MODULES_ASMP_WORKER_WORKER_PRINTF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CXD5602_WORKER
#  include <asmp/stdio.h>
#  include "arch/lowputc.h"
#else
/* For debugging on main core or linux */
#  ifdef PSEUDO_SPRINTF
#    define lowputc line_putc
int line_putc(char c);
void linebuf_init(void);
char *line_buff(void);
void dump_line(void);
#  else
#    include <nuttx/arch.h>
#    define lowputc up_putc
#  endif
#endif

#ifdef CONFIG_ASMP_WORKER_PRINTF_FMT_FLOAT
typedef unsigned long mantissa_t;
typedef long          expo_t;
typedef float         myfloat_t;
#  define FLOAT_UMEMBER(__a)    ((__a)->value.u)
#  define EXPO_BITS             (0xff)
#  define MANTISSA_BITS         (0x7fffff)
#  define MASK_EXPO(__a)        ((FLOAT_UMEMBER(__a) >> 23) & EXPO_BITS)
#  define MASK_MANTISSA(__a)    (FLOAT_UMEMBER(__a) & MANTISSA_BITS)
#else
typedef unsigned long long mantissa_t;
typedef long long          expo_t;
typedef double             myfloat_t;
#  define FLOAT_UMEMBER(__a)    ((__a)->value.ll)
#  define EXPO_BITS             (0x7fffULL)
#  define MANTISSA_BITS         (0xfffffffffffffULL)
#  define MASK_EXPO(__a)        ((FLOAT_UMEMBER(__a) >> 52) & EXPO_BITS)
#  define MASK_MANTISSA(__a)    (FLOAT_UMEMBER(__a) & MANTISSA_BITS)
#endif

#define ARGPARSE_FLAG_ZFILL   (1<<0)
#define ARGPARSE_FLAG_LEFT    (1<<1)
#define ARGPARSE_FLAG_PLUS    (1<<2)
#define ARGPARSE_FLAG_NEGVAL  (1<<3)
#define ARGPARSE_FLAG_LONG    (1<<4)
#define ARGPARSE_FLAG_LL      (1<<5)
#define ARGPARSE_FLAG_SPACE   (1<<6)
#define ARGPARSE_FLAG_UNSIGN  (1<<7)

#define VALPARSE_TYPE_CHAR   1
#define VALPARSE_TYPE_SINT   2
#define VALPARSE_TYPE_UINT   3
#define VALPARSE_TYPE_HEXL   4
#define VALPARSE_TYPE_HEXH   5
#define VALPARSE_TYPE_PTR    6
#define VALPARSE_TYPE_STR    7
#define VALPARSE_TYPE_FLOAT  8
#define VALPARSE_TYPE_EFLOAT 9

#define DEFAULT_PRECISION 6

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct arg_parse_s
{
  unsigned int flag;  /* bit map of ARGPARSE_FLAG_XXXX */
  int width;          /* Field width */
  int precision;      /* precision digits */
  int type;           /* Parsed type as VALPARSE_TYPE_XXXX */
  union
  {
    unsigned char c;        /* for %c */
    unsigned int  u;        /* for %p, %d, %u, %x, %X, %ld, %lu, %lx, %lX */
    unsigned long long ll;  /* for %lld, %llu, %llx, %llX */
#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
    myfloat_t d;            /* for %f, %e */
#endif
    char *cp;               /* for %s */
  } value;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int lowputs(const char *str);
int worker_print_hex(struct arg_parse_s *arg, char h);
int worker_print_lhex(struct arg_parse_s *arg, char h);
int worker_print_int(struct arg_parse_s *arg);
int worker_print_longlong(struct arg_parse_s *arg);
#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
int worker_print_float(struct arg_parse_s *arg);
int worker_print_floate(struct arg_parse_s *arg);
#endif

#endif /* __MODULES_ASMP_WORKER_WORKER_PRINTF_H */
