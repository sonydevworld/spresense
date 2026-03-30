/****************************************************************************
 * modules/asmp/worker/printf.c
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#include <stdarg.h>
#include <asmp/stdio.h>
#include "arch/lowputc.h"

/****************************************************************************
 * Pri-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((void *)0)
#endif

#define INT_DIGITTOP (1000000000)

#define ARGPARSE_FLAG_ZFILL   (1<<0)
#define ARGPARSE_FLAG_MINUS   (1<<1)
#define ARGPARSE_FLAG_PLUS    (1<<2)
#define ARGPARSE_FLAG_LEFT    (1<<3)
#define ARGPARSE_FLAG_NEGVAL  (1<<4)

#define VALPARSE_TYPE_CHAR  1
#define VALPARSE_TYPE_SINT  2
#define VALPARSE_TYPE_UINT  3
#define VALPARSE_TYPE_HEXL  4
#define VALPARSE_TYPE_HEXH  5
#define VALPARSE_TYPE_PTR   6
#define VALPARSE_TYPE_STR   7

#define PARSE_PHASE_INITIAL 0
#define PARSE_PHASE_PLSMIN  1
#define PARSE_PHASE_ZFILL   2
#define PARSE_PHASE_DIGIT   3
#define PARSE_PHASE_DONE    4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arg_parse_s
{
  unsigned int flag;  /* bit map of ARGPARSE_FLAG_XXXX */
  int digit_specify;  /* Store digit value. e.x. %8x -> 8 */
  int type;           /* Parsed type as VALPARSE_TYPE_XXXX */
  union
  {
    unsigned char c;  /* for %c */
    unsigned int u;   /* for %p, %d, %u, %x, %X */
    char *cp;         /* for %s */
  } value;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lowputs(const char *str)
{
  int ret = 0;
  while (*str)
    {
      lowputc(*str++);
      ret++;
    }

  return ret;
}

/* print_hex(): print hexiagonal value */

static int print_hex(struct arg_parse_s *arg, char h)
{
  int tmp;
  int val;
  int pos = 0;
  unsigned int mask = 0xf0000000;
  unsigned int shift = 28;
  char value_buf[12];
  int pad_len;
  char pad_char;

  tmp = 0;
  for (mask = 0xf0000000, shift = 28; mask; mask >>= 4, shift -= 4)
    {
      val = (arg->value.u & mask) >> shift;
      if (tmp || val != 0)
        {
          value_buf[pos++] = val >= 10 ? val + h - 10 : val + '0';
          tmp = 1;
        }
    }

  if (pos == 0)
    {
      value_buf[pos++] = '0';
    }

  if (arg->digit_specify)
    {
      arg->digit_specify = arg->digit_specify > 10 ? 10 : arg->digit_specify;
      pad_char = arg->flag & ARGPARSE_FLAG_ZFILL ? '0' : ' ';
      if (arg->digit_specify > pos)
        {
          pad_len = arg->digit_specify - pos;
          for (tmp = pos - 1; tmp >= 0; tmp--)
            {
              value_buf[tmp + pad_len] = value_buf[tmp];
            }

          for (tmp = 0; tmp < pad_len; tmp++)
            {
              value_buf[tmp] = pad_char;
            }

          pos += pad_len;
        }
    }

  value_buf[pos] = '\0';
  lowputs(value_buf);

  return pos;
}

/* print_int(): print integer value */

static int print_int(struct arg_parse_s *arg)
{
  int val;
  int tmp = 0;
  int pos = 0;
  unsigned int digit = INT_DIGITTOP;
  char value_buf[12];
  int pad_len;
  char pad_char;

  if (arg->flag & ARGPARSE_FLAG_PLUS || arg->flag & ARGPARSE_FLAG_NEGVAL)
    {
      value_buf[pos++] = arg->flag & ARGPARSE_FLAG_NEGVAL ? '-' : '+';
    }

  while (digit)
    {
      val = arg->value.u / digit;
      if (tmp || val != 0)  /* tmp indicates print is started or not. */
        {
          val = (val >= 10) ? 9 : val;
          value_buf[pos++] = val + '0';
          tmp = 1;
        }

      arg->value.u = arg->value.u - (val * digit);
      digit = digit / 10;
    }

  if (pos == 0)
    {
      value_buf[pos++] = '0';
    }

  if (arg->digit_specify)
    {
      arg->digit_specify = arg->digit_specify > 10 ? 10 : arg->digit_specify;
      pad_char = arg->flag & ARGPARSE_FLAG_ZFILL ? '0' : ' ';
      if (arg->digit_specify > pos)
        {
          pad_len = arg->digit_specify - pos;
          if (arg->flag & ARGPARSE_FLAG_MINUS)
            {
              for (; pad_len; pad_len--)
                {
                  value_buf[pos++] = pad_char;
                }
            }
          else
            {
              for (tmp = pos - 1; tmp >= 0; tmp--)
                {
                  value_buf[tmp + pad_len] = value_buf[tmp];
                }

              for (tmp = 0; tmp < pad_len; tmp++)
                {
                  value_buf[tmp] = pad_char;
                }
            }

          pos += pad_len;
        }
    }

  value_buf[pos] = '\0';
  lowputs(value_buf);

  return pos;
}

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
      default: return 0;
    }
}

/* parse_percent_value(): parse after '%' */

static const char *parse_percent_value(const char *fmt, struct arg_parse_s *arg)
{
  int parse_phase = PARSE_PHASE_INITIAL;
  int pos = 0;

  arg->flag = 0;
  arg->digit_specify = 0;

  while (!(arg->type = type_specify(fmt[pos])))
    {
      switch (parse_phase)
        {
          case PARSE_PHASE_INITIAL:
            switch (fmt[pos])
              {
                case '-':
                  if (arg->flag & ARGPARSE_FLAG_MINUS)
                    {
                      /* Double specified */

                      return NULL;
                    }

                  arg->flag |= ARGPARSE_FLAG_MINUS;
                  if (arg->flag & ARGPARSE_FLAG_PLUS)
                    {
                      parse_phase = PARSE_PHASE_PLSMIN;
                    }

                  pos++;

                  break;

                case '+':
                  if (arg->flag & ARGPARSE_FLAG_PLUS)
                    {
                      /* Double specified */

                      return NULL;
                    }

                  arg->flag |= ARGPARSE_FLAG_PLUS;
                  if (arg->flag & ARGPARSE_FLAG_MINUS)
                    {
                      parse_phase = PARSE_PHASE_PLSMIN;
                    }

                  pos++;

                  break;

                case '0':
                  arg->flag |= ARGPARSE_FLAG_ZFILL;
                  parse_phase = PARSE_PHASE_ZFILL;
                  pos++;
                  break;

                default:
                  if (fmt[pos] >= '1' && fmt[pos] <= '9')
                    {
                      parse_phase = PARSE_PHASE_DIGIT;

                      /* no pos increment because digit detect in DIGIT phase */
                    }
                  else
                    {
                      return NULL;
                    }

                  break;
              } /* switch (fmt[pos]) */

            break;

          case PARSE_PHASE_PLSMIN :
            switch (fmt[pos])
              {
                case '0':
                  arg->flag |= ARGPARSE_FLAG_ZFILL;
                  parse_phase = PARSE_PHASE_ZFILL;
                  pos++;
                  break;

                default:
                  if (fmt[pos] >= '1' && fmt[pos] <= '9')
                    {
                      parse_phase = PARSE_PHASE_DIGIT;

                    /* no pos increment because digit parse is done
                     * in PARSE_PHASE_DIGIT
                     */
                    }
                  else
                    {
                      return NULL;
                    }
              }

            break;

          case PARSE_PHASE_ZFILL  :
            if (fmt[pos] >= '1' && fmt[pos] <= '9')
              {
                parse_phase = PARSE_PHASE_DIGIT;

                /* no pos increment because digit parse is done
                 * in PARSE_PHASE_DIGIT
                 */
              }
            else
              {
                return NULL;
              }

            break;

          case PARSE_PHASE_DIGIT  :
            arg->digit_specify = fmt[pos] - '0';
            pos++;
            while (fmt[pos] >= '0' && fmt[pos] <= '9')
              {
                arg->digit_specify =
                  arg->digit_specify * 10 + fmt[pos] - '0';
                pos++;
              }

            parse_phase = PARSE_PHASE_DONE;
            break;

          case PARSE_PHASE_DONE  :
            /* Should not enter here */

            return NULL;
        }
    } /* while (!(arg->type = is_type_specify(fmt[pos]))) */

  return &fmt[pos];
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
                    fmt--;
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

                          total_len += print_int(&arg);
                          break;

                        case VALPARSE_TYPE_UINT:
                          arg.value.u = va_arg(va, unsigned int);
                          total_len += print_int(&arg);
                          break;

                        case VALPARSE_TYPE_HEXL:
                          arg.value.u = va_arg(va, unsigned int);
                          total_len += print_hex(&arg, 'a');
                          break;

                        case VALPARSE_TYPE_HEXH:
                          arg.value.u = va_arg(va, unsigned int);
                          total_len += print_hex(&arg, 'A');
                          break;

                        case VALPARSE_TYPE_PTR:
                          arg.value.u = va_arg(va, unsigned int);
                          if (arg.value.u)
                            {
                              arg.flag |= ARGPARSE_FLAG_ZFILL;
                              arg.digit_specify = 8;
                              total_len += print_hex(&arg, 'a');
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
                      } /* switch (arg.type) */

                    fmt = tmp;
                  } /* else of if (tmp == NULL) */
              } /* else of if (*fmt == '%') */

            break; /* case '%': */

          case '\n':
            lowputc('\r');
            lowputc('\n');
            total_len++;
            break;

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

int printf(const char *fmt, ...)
{
  int ret;
  va_list va;

  va_start(va, fmt);
  ret = worker_vprintf(fmt, va);
  va_end(va);

  return ret;
}

int puts(const char *str)
{
  return lowputs(str);
}

int putchar(const char c)
{
  lowputc(c);

  return (int)c;
}
