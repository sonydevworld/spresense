/****************************************************************************
 * modules/asmp/worker/worker_print_values.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include "worker_printf.h"

#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
#  ifndef CONFIG_ASMP_WORKER_PRINTF_USEPOWTBL
#    include <math.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions for Debugging
 ****************************************************************************/

#if 0
#  include <stdio.h>
#  define T() printf("%s(%d)\n", __func__, __LINE__)
#  define TM(...) do {                        \
    printf("%s(%d): ", __func__, __LINE__); \
    printf(__VA_ARGS__);                    \
  } while(0)

#  define PRINT_FLAG(__a) do { \
    if (FLAG_CHECK((__a)->flag, ZFILL))   printf("Z"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, LEFT))    printf("M"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, PLUS))    printf("P"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, NEGVAL))  printf("N"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, LONG))    printf("l"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, LL))      printf("L"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, SPACE))   printf("S"); else printf("_");  \
    if (FLAG_CHECK((__a)->flag, UNSIGN))  printf("U"); else printf("_");  \
    printf("\n"); \
  } while(0)
#else
#define T()
#define TM(...)
#define PRINT_FLAG(__a)
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VALUE_BUFSZ (128)

#ifndef MAX
#  define MAX(__a, __b) ((__a) > (__b)) ? (__a) : (__b)
#endif

#ifndef MIN
#  define MIN(__a, __b) ((__a) > (__b)) ? (__b) : (__a)
#endif

#ifndef nitems
#  define nitems(__t) (sizeof(__t)/sizeof((__t)[0]))
#endif

#define FLAG_CHECK(__ff, __f) ((__ff) & ARGPARSE_FLAG_ ## __f)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
#  ifdef CONFIG_ASMP_WORKER_PRINTF_USEPOWTBL
static const double pow10_table[] =
{
  1.0,                   /* pow10(0) */
  10.0,                  /* pow10(1) */
  100.0,                 /* pow10(2) */
  1000.0,                /* pow10(3) */
  10000.0,               /* pow10(4) */
  100000.0,              /* pow10(5) */
  1000000.0,             /* pow10(6) */
  10000000.0,            /* pow10(7) */
  100000000.0,           /* pow10(8) */
  1000000000.0,          /* pow10(9) */
  10000000000.0,         /* pow10(10) */
  100000000000.0,        /* pow10(11) */
  1000000000000.0,       /* pow10(12) */
  10000000000000.0,      /* pow10(13) */
  100000000000000.0,     /* pow10(14) */
  1000000000000000.0,    /* pow10(15) */
  10000000000000000.0,   /* pow10(16) */
  100000000000000000.0,  /* pow10(17) */
  1000000000000000000.0, /* pow10(18) */
  10000000000000000000.0 /* pow10(19) */
};

static const double pow01_table[] =
{
  1.0,                  /* pow10(0) */
  0.1,                  /* pow10(-1) */
  0.01,                 /* pow10(-2) */
  0.001,                /* pow10(-3) */
  0.0001,               /* pow10(-4) */
  0.00001,              /* pow10(-5) */
  0.000001,             /* pow10(-6) */
  0.0000001,            /* pow10(-7) */
  0.00000001,           /* pow10(-8) */
  0.000000001,          /* pow10(-9) */
  0.0000000001,         /* pow10(-10) */
  0.00000000001,        /* pow10(-11) */
  0.000000000001,       /* pow10(-12) */
  0.0000000000001,      /* pow10(-13) */
  0.00000000000001,     /* pow10(-14) */
  0.000000000000001,    /* pow10(-15) */
  0.0000000000000001,   /* pow10(-16) */
  0.00000000000000001,  /* pow10(-17) */
  0.000000000000000001, /* pow10(-18) */
  0.0000000000000000001 /* pow10(-19) */
};
#  endif
#endif

static char g_tmpbuf[VALUE_BUFSZ];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
static double local_pow10(double v, int p)
{
#  ifdef CONFIG_ASMP_WORKER_PRINTF_USEPOWTBL
  int max = nitems(pow10_table);
  const double *tbl = pow10_table;

  if (p < 0)
    {
      p = -p;
      max = nitems(pow01_table);
      tbl = pow01_table;
    }

  while (p > max)
    {
      v *= tbl[max - 1];
      p -= max;
    }

  return v * tbl[p];
#  else
  return v * pow(10.0, p);
#  endif /* #ifdef CONFIG_ASMP_WORKER_PRINTF_USEPOWTBL */
}
#endif /* #ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN */

static int rlowputs(const char *st, const char *ed)
{
  int ret = 0;
  while (st > ed)
    {
      st--;
      lowputc(*st);
      ret++;
    }

  return ret;
}

static int draw_sign(int flg, char *buf)
{
       if (FLAG_CHECK(flg, NEGVAL))                           { *buf = '-'; }
  else if (FLAG_CHECK(flg, PLUS) && !FLAG_CHECK(flg, UNSIGN)) { *buf = '+'; }
  else if (FLAG_CHECK(flg, SPACE ))                           { *buf = ' '; }
  else { return 0; }

  return 1;
}

static int draw_valuestr(int pos, unsigned int flag, int width, bool en)
{
  char *st;
  char sign_char;

  if (width > pos)
    {
      if (!FLAG_CHECK(flag, LEFT) && FLAG_CHECK(flag, ZFILL) && en)
        {
          st = &g_tmpbuf[pos];
          if (draw_sign(flag, &sign_char) != 0)
            {
              lowputc(sign_char);
              pos++;
            }

          while (width > pos)
            {
              lowputc('0');
              pos++;
            }
          rlowputs(st, g_tmpbuf);
        }
      else /* of if (!FLAG_CHECK(flag, LEFT) && FLAG_CHECK(flag, ZFILL)) */
        {
          pos += draw_sign(flag, &g_tmpbuf[pos]);
          st = &g_tmpbuf[pos];
          if (FLAG_CHECK(flag, LEFT))
            {
              rlowputs(st, g_tmpbuf);
              while (width > pos)
                {
                  lowputc(' ');
                  pos++;
                }
            }
          else
            {
              while (width > pos)
                {
                  lowputc(' ');
                  pos++;
                }
              rlowputs(st, g_tmpbuf);
            }
        }
    }
  else /* of if (width > pos) */
    {
      pos += draw_sign(flag, &g_tmpbuf[pos]);
      rlowputs(&g_tmpbuf[pos], g_tmpbuf);
    }

  return pos;
}

static int draw_value_int(int pos, unsigned int flag, int prec, int width)
{
  while (prec > pos)
    {
      g_tmpbuf[pos++] = '0';
    }

  return draw_valuestr(pos, flag, width, (prec < 0));
}

static int draw_value_hex(int pos, unsigned int flag, int prec, int width)
{
  char *st;
  char pad_char;

  /* check number of digits */

  while (prec > pos)
    {
      g_tmpbuf[pos++] = '0';
    }

  /* Width treatment */

  if (width > pos)
    {
      if (FLAG_CHECK(flag, LEFT))
        {
          pos += draw_sign(flag, &g_tmpbuf[pos]);
          rlowputs(&g_tmpbuf[pos], g_tmpbuf);
          while (width > pos)
            {
              lowputc(' ');
              pos++;
            }
        }
      else
        {
          pad_char = (FLAG_CHECK(flag, ZFILL) && (prec < 0)) ? '0' : ' ';
          st = &g_tmpbuf[pos];
          while (width > pos)
            {
              lowputc(pad_char);
              pos++;
            }
          rlowputs(st, g_tmpbuf);
        }
    }
  else
    {
      rlowputs(&g_tmpbuf[pos], g_tmpbuf);
    }

  return pos;
}

#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
static int render_float(int pos, uint64_t *intp, uint64_t *fracp, int prec)
{
  /* Render Fractional part */

  if (prec > 0)
    {
      while (prec--)
        {
          g_tmpbuf[pos++] = '0' + (*fracp % 10);
          *fracp /= 10;
        }

      g_tmpbuf[pos++] = '.';
    }

  /* Render Integer part */

  if (*intp == 0)
    {
      g_tmpbuf[pos++] = '0';
    }
  else
    {
      while (*intp)
        {
          g_tmpbuf[pos++] = '0' + (int)(*intp % 10);
          *intp /= 10;
        }
    }

  return pos;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* print_hex(): print hexiagonal value */

int worker_print_hex(struct arg_parse_s *arg, char h)
{
  int pos = 0;
  unsigned int val;

  if (arg->value.u == 0)
    {
      g_tmpbuf[pos++] = '0';
    }
  else
    {
      for (; arg->value.u; arg->value.u >>= 4)
        {
          val = arg->value.u & 0xf;
          g_tmpbuf[pos++] = val >= 10 ? val + h - 10 : val + '0';
        }
    }

  return draw_value_hex(pos, arg->flag, arg->precision, arg->width);
}

/* worker_print_lhex(): print hexiagonal value */

int worker_print_lhex(struct arg_parse_s *arg, char h)
{
  int pos = 0;
  unsigned int val;

  if (arg->value.ll == 0)
    {
      g_tmpbuf[pos++] = '0';
    }
  else
    {
      for (; arg->value.ll; arg->value.ll >>= 4)
        {
          val = arg->value.ll & 0xf;
          g_tmpbuf[pos++] = val >= 10 ? val + h - 10 : val + '0';
        }
    }

  return draw_value_hex(pos, arg->flag, arg->precision, arg->width);
}

/* worker_print_int(): print integer value */

int worker_print_int(struct arg_parse_s *arg)
{
  int pos = 0;

  if (arg->value.u == 0)
    {
      g_tmpbuf[pos++] = '0';
    }
  else
    {
      while (arg->value.u)
        {
          g_tmpbuf[pos++] = '0' + (arg->value.u % 10);
          arg->value.u /= 10;
        }
    }

  return draw_value_int(pos, arg->flag, arg->precision, arg->width);
}

/* worker_print_longlong(): print 64bit integer value */

int worker_print_longlong(struct arg_parse_s *arg)
{
  int pos = 0;

  if (arg->value.ll == 0)
    {
      g_tmpbuf[pos++] = '0';
    }
  else
    {
      while (arg->value.ll)
        {
          g_tmpbuf[pos++] = '0' + (arg->value.ll % 10);
          arg->value.ll /= 10;
        }
    }

  return draw_value_int(pos, arg->flag, arg->precision, arg->width);
}

#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
/* worker_print_float(): print floating value */

int worker_print_float(struct arg_parse_s *arg)
{
  int        pos;
  mantissa_t mantissa;
  expo_t     expo;
  uint64_t   intpart;
  uint64_t   fracpart;
  double     tmppow;

  pos = 0;

  expo     = MASK_EXPO(arg);
  mantissa = MASK_MANTISSA(arg);

  if (expo == EXPO_BITS)
    {
      return lowputs( mantissa                      ? "nan" :
                      FLAG_CHECK(arg->flag, NEGVAL) ? "-inf": "inf");
    }

  if (arg->value.d > (double)UINT64_MAX)
    {
      return lowputs(FLAG_CHECK(arg->flag, NEGVAL) ? "-inf": "inf");
    }

  /* Max precision length */

  if (arg->precision < 0) arg->precision = 6;
  arg->precision = MIN(arg->precision, 15);

  /* Split integer part and fractional part */

  tmppow = local_pow10(1.0, arg->precision);
  intpart = (uint64_t)arg->value.d;
  fracpart = (uint64_t)(((arg->value.d - (myfloat_t)intpart) * tmppow) + 0.5);
  if (fracpart >= tmppow)
    {
      intpart += 1;
      fracpart -= tmppow;
    }

  pos = render_float(pos, &intpart, &fracpart, arg->precision);
  return draw_valuestr(pos, arg->flag, arg->width, true);
}

/* worker_print_floate(): print floating value as exp style */

int worker_print_floate(struct arg_parse_s *arg)
{
  int pos;
  int exp10;
  mantissa_t mantissa;
  expo_t     expo;
  myfloat_t  dtmp;
  uint64_t intpart;
  uint64_t fracpart;
  double tmppow;
  char esign;

  expo     = MASK_EXPO(arg);
  mantissa = MASK_MANTISSA(arg);

  if (expo == EXPO_BITS)
    {
      return lowputs( mantissa                      ? "nan" :
                      FLAG_CHECK(arg->flag, NEGVAL) ? "-inf": "inf");
    }

  /* Max precision length */

  if (arg->precision < 0) arg->precision = 6;
  arg->precision = MIN(arg->precision, 15);

  esign = ' ';
  exp10 = 0;
  dtmp = arg->value.d;
  if (dtmp == 0.0)
    {
      intpart = 0;
      fracpart = 0;
      esign = '+';
    }
  else
    {
      /* Normalize */

      if (dtmp >= 10.0)
        {
          while (dtmp >= 10.0)
            {
              dtmp *= 0.1;
              exp10++;
            }

          esign = '+';
        }
      else if (dtmp < 1.0)
        {
          while (dtmp < 1.0)
            {
              dtmp *= 10.0;
              exp10++;
            }

          esign = '-';
        }

      tmppow = local_pow10(1.0, arg->precision);
      intpart = (uint64_t)dtmp;
      fracpart = (uint64_t)(((dtmp - (myfloat_t)intpart) * tmppow) + 0.5);
      if (fracpart >= tmppow)
        {
          intpart += 1;
          fracpart -= tmppow;
        }
    }

  /* Draw Mantissa part */

  pos = 0;
  for (; exp10; exp10 /= 10) g_tmpbuf[pos++] = '0'+ (exp10 % 10);
  while (pos < 2) g_tmpbuf[pos++] = '0';  /* 2 digits guarantee */
  g_tmpbuf[pos++] = esign;
  g_tmpbuf[pos++] = 'e';

  pos = render_float(pos, &intpart, &fracpart, arg->precision);
  return draw_valuestr(pos, arg->flag, arg->width, true);
}
#endif
