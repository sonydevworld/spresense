/****************************************************************************
 * modules/asmp/worker/worker_printf.c
 *
 *   Copyright 2024,2026 Sony Semiconductor Solutions Corporation
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
#include <stdint.h>
#include <stdarg.h>

#include "worker_printf.h"

/****************************************************************************
 * Pri-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((void *)0)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* type_specify(): parse type in string */

static int type_specify(const char c)
{
  switch (c)
    {
      case  'c': return VALPARSE_TYPE_CHAR;
      case  'd': return VALPARSE_TYPE_SINT;
      case  'u': return VALPARSE_TYPE_UINT;
      case  'x': return VALPARSE_TYPE_HEXL;
      case  'X': return VALPARSE_TYPE_HEXH;
      case  'p': return VALPARSE_TYPE_PTR;
      case  's': return VALPARSE_TYPE_STR;
#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
      case  'f': return VALPARSE_TYPE_FLOAT;
      case  'e': return VALPARSE_TYPE_EFLOAT;
#endif
      default: return 0;
    }
}

/* parse_percent_value(): parse after '%' */

static const char *parse_percent_value(const char *fmt, struct arg_parse_s *arg)
{
  const char *p = fmt;

  arg->flag = 0;
  arg->width = 0;
  arg->precision = -1;
  arg->type = 0;

  /* Check flags just after '%'
   * For light implementation, some format errors are avoided.
   * (Like '--', '++', etc..)
   */

  while (1)
    {
      switch (*p)
        {
          case '-': arg->flag |= ARGPARSE_FLAG_LEFT;  p++; continue;
          case '+': arg->flag |= ARGPARSE_FLAG_PLUS;  p++; continue;
          case '0': arg->flag |= ARGPARSE_FLAG_ZFILL; p++; continue;
          case ' ': arg->flag |= ARGPARSE_FLAG_SPACE; p++; continue;
          default: break;
        }
      break;
    }

  /* Parse specific width of integer part */

  if (*p >= '0' && *p <= '9')
    {
      while (*p >= '0' && *p <= '9')
        {
          arg->width = arg->width * 10 + (*p - '0');
          p++;
        }
    }

  /* Parse specific width of precision part */

  if (*p == '.')
    {
      p++;
      arg->precision = 0;

      if (!(*p >= '0' && *p <= '9') && !(*p == 'f'))
        {
          return NULL;
        }

      while (*p >= '0' && *p <= '9')
        {
          arg->precision =
            arg->precision * 10 + (*p - '0');
          p++;
        }
    }

  /* Check 'l' and 'll' */

  if (*p == 'l')
    {
      p++;
      if (*p == 'l')
        {
          arg->flag |= ARGPARSE_FLAG_LL;
          p++;
        }
      else
        {
          arg->flag |= ARGPARSE_FLAG_LONG;
        }
    }

  /* Parse type like %d, %s, ... */

  arg->type = type_specify(*p);
  if (!arg->type)
    {
      return NULL;
    }

  /* Check format error */

  if ((arg->flag & ARGPARSE_FLAG_LL) &&
      !(arg->type == VALPARSE_TYPE_SINT ||
        arg->type == VALPARSE_TYPE_UINT ||
        arg->type == VALPARSE_TYPE_HEXL ||
        arg->type == VALPARSE_TYPE_HEXH))
    {
      return NULL;
    }

  if ((arg->flag & ARGPARSE_FLAG_LONG) &&
      !(arg->type == VALPARSE_TYPE_SINT ||
        arg->type == VALPARSE_TYPE_UINT ||
        arg->type == VALPARSE_TYPE_HEXL ||
        arg->type == VALPARSE_TYPE_HEXH))
    {
      return NULL;
    }

  return p;
}

/* worker_vprintf(): parse and print string and variables */

static int worker_vprintf(const char *fmt, va_list va)
{
  int total_len = 0;
  int val;
  struct arg_parse_s arg;
  const char *tmp;

  while (*fmt != '\0')
    {
      switch (*fmt)
        {
          case '%':
            fmt++;
            if (*fmt == '%')
              {
                lowputc(*fmt); total_len++;
              }
            else
              {
                arg.type = 0;
                tmp = parse_percent_value(fmt, &arg);
                if (tmp == NULL)
                  {
                    /* Format error is detected.
                     * Operate as normal string.
                     */

                    lowputc('%');
                    total_len++;
                  }
                else
                  {
                    /* Actual print argument */

                    switch (arg.type)
                      {
                        case VALPARSE_TYPE_CHAR:
                          arg.value.c = va_arg(va, int);
                          lowputc(arg.value.c);
                          total_len++;
                          break;

                        case VALPARSE_TYPE_SINT:
                          if (arg.flag & ARGPARSE_FLAG_LL)
                            {
                              int64_t llv = va_arg(va, int64_t);
                              if (llv < 0)
                                {
                                  arg.flag |= ARGPARSE_FLAG_NEGVAL;
                                  arg.value.ll = - llv;
                                }
                              else
                                {
                                  arg.value.ll = llv;
                                }

                              total_len += worker_print_longlong(&arg);
                            }
                          else
                            {
                              val = va_arg(va, int);
                              if (val < 0)
                                {
                                  arg.flag |= ARGPARSE_FLAG_NEGVAL;
                                  arg.value.u = - val;
                                }
                              else
                                {
                                  arg.value.u = val;
                                }

                              total_len += worker_print_int(&arg);
                            }

                          break;

                        case VALPARSE_TYPE_UINT:
                          arg.flag |= ARGPARSE_FLAG_UNSIGN;
                          if (arg.flag & ARGPARSE_FLAG_LL)
                            {
                              arg.value.ll = va_arg(va, unsigned long long);
                              total_len += worker_print_longlong(&arg);
                            }
                          else
                            {
                              arg.value.u = va_arg(va, unsigned int);
                              total_len += worker_print_int(&arg);
                            }
                          break;

                        case VALPARSE_TYPE_HEXH:
                        case VALPARSE_TYPE_HEXL:
                          if (arg.flag & ARGPARSE_FLAG_LL)
                            {
                              arg.value.ll = va_arg(va, unsigned long long);
                              total_len += worker_print_lhex(&arg,
                                            arg.type == VALPARSE_TYPE_HEXH ? 'A' : 'a');
                            }
                          else
                            {
                              arg.value.u = va_arg(va, unsigned int);
                              total_len += worker_print_hex(&arg,
                                            arg.type == VALPARSE_TYPE_HEXH ? 'A' : 'a');
                            }
                          break;


                        case VALPARSE_TYPE_PTR:
                          arg.value.u = va_arg(va, unsigned int);
                          if (arg.value.u)
                            {
                              arg.flag |= ARGPARSE_FLAG_ZFILL;
                              arg.width = 8;
                              total_len += worker_print_hex(&arg, 'a');
                            }
                          else
                            {
                              total_len += lowputs("(null)");
                            }

                          break;

                        case VALPARSE_TYPE_STR:
                          arg.value.cp = va_arg(va, char *);
                          total_len += lowputs(arg.value.cp);
                          break;

#ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN
                        case VALPARSE_TYPE_FLOAT:
                          arg.value.d = va_arg(va, double);

                          if (arg.value.d < 0)
                            {
                              arg.flag |= ARGPARSE_FLAG_NEGVAL;
                              arg.value.d = - arg.value.d;
                            }

                          total_len += worker_print_float(&arg);
                          break;

                        case VALPARSE_TYPE_EFLOAT:
                          arg.value.d = va_arg(va, double);

                          if (arg.value.d < 0)
                            {
                              arg.flag |= ARGPARSE_FLAG_NEGVAL;
                              arg.value.d = - arg.value.d;
                            }

                          total_len += worker_print_floate(&arg);
                          break;
#endif /* #ifdef CONFIG_ASMP_WORKER_PRINTF_FLOATEN */
                      } /* switch (arg.type) */

                    fmt = tmp;
                  } /* else of if (tmp == NULL) */
              } /* else of if (*fmt == '%') */

            break; /* case '%': */

          default:
            lowputc(*fmt);
            total_len++;
            break;
        } /* switch (*fmt) */

      fmt++;
    }

  return total_len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* For debug use only */

#ifdef CXD5602_WORKER
int printf(const char *fmt, ...)
#else
/* For test on main core or linux */
int my_printf(const char *fmt, ...)
#endif
{
  int ret;
  va_list va;

  va_start(va, fmt);
  ret = worker_vprintf(fmt, va);
  va_end(va);

  return ret;
}

int lowputs(const char *str)
{
  int ret = 0;

  while (*str)
    {
      lowputc(*str++);
      ret++;
    }

  return ret + 1;
}

#ifdef CXD5602_WORKER
int putchar(char c)
{
  lowputc(c);
  return (int)c;
}

int puts(const char *str)
{
  int ret;
  ret = lowputs(str);
  lowputc('\n');

  return ret + 1;
}
#endif
