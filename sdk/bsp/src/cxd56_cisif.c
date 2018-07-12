/****************************************************************************
 * bsp/src/cxd56_cisif.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"

#include "cxd56_clock.h"
#include "chip/cxd56_cisif.h"
#include "cxd56_cisif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* To see the interrupt timing of Vsync */
/* #define CISIF_INTR_TRACE */
/* #define CISIF_DBG_CONTI_CAP */

#define YUV_VSIZE_MIN (64)
#define YUV_HSIZE_MIN (96)
#define YUV_VSIZE_MAX (360)
#define YUV_HSIZE_MAX (480)

#define JPG_INT_ALL   (JPG_ERR_STATUS_INT | \
                       JPG_MEM_OVF_INT    | \
                       JPG_FIFO_OVF_INT   | \
                       JPG_AXI_TRERR_INT  | \
                       JPG_MARKER_ERR_INT | \
                       JPG_AXI_TRDN_INT)

#define YCC_INT_ALL   (YCC_MEM_OVF_INT    | \
                       YCC_FIFO_OVF_INT   | \
                       YCC_AXI_TRERR_INT  | \
                       YCC_MARKER_ERR_INT | \
                       SIZE_UNDER_INT     | \
                       SIZE_OVER_INT      | \
                       YCC_AXI_TRDN_INT)

#define SAREA_SINGLE  (0)
#define SAREA_BANK    (1)
#define SAREA_FLAT    (2)

#ifdef CONFIG_CXD56_CISIF_DEBUG
#  define ciferr    _err
#  define cifwarn   _warn
#  define cifinfo   _info
#else
#  define ciferr(x...)
#  define cifwarn(x...)
#  define cifinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum state_e
{
  STATE_STANDBY,
  STATE_READY,
  STATE_CAPTURE,
  STATE_CONTI_CAPTURE,
  STATE_START_MONITORING,
  STATE_MONITORING,
  STATE_STOP,
};

typedef enum state_e state_t;

enum type_cisif_e
{
  TYPE_CISIF_YUV,
  TYPE_CISIF_JPEG,
  TYPE_CISIF_MAX,
};

typedef void (*intc_func_table)(uint8_t code);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static state_t g_state = STATE_STANDBY;
static uint8_t *g_ycc_storage_addr[2] = { NULL, NULL };
static uint8_t *g_jpg_storage_addr[2] = { NULL, NULL };
static uint8_t g_bank = 0;

notify_callback_t g_notify_callback_func[TYPE_CISIF_MAX];
comp_callback_t   g_comp_callback_func[TYPE_CISIF_MAX];

static uint32_t g_jpg_sarea_addr = 0;
static uint32_t g_jpg_sarea_size = 0;
static uint32_t g_conti_max_capnum = 0;
static uint32_t g_conti_capnum = 0;
static uint32_t g_conti_interval = 0;
static uint32_t g_conti_vcnt   = 0;
static uint32_t g_conti_lastframe = 0;

#ifdef CISIF_INTR_TRACE
static uint32_t g_cisif_vint_count = 0;
static uint32_t g_cisif_vint_count_max = 0;
static uint32_t g_cisif_time_start;
static uint32_t g_cisif_time_stop;
#endif /* CISIF_INTR_TRACE */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cisif_vs_int(uint8_t code);
static void cisif_ycc_axi_trdn_int(uint8_t code);
static void cisif_ycc_nstorage_int(uint8_t code);
static void cisif_jpg_axi_trdn_int(uint8_t code);
static void cisif_jpg_nstorage_int(uint8_t code);
static void cisif_ycc_err_int(uint8_t code);
static void cisif_jpg_err_int(uint8_t code);

static void     cisif_reg_write(uint16_t reg, uint32_t val);
static uint32_t cisif_reg_read(uint16_t reg);

static int cisif_check_param(cisif_param_t *p);
static int cisif_set_yuv_param(cisif_param_t *p);
static int cisif_set_jpg_param(cisif_param_t *p);

static int cisif_check_sarea(int type, void *s);;
static int cisif_set_yuv_sarea(int type, void *s);
static int cisif_set_jpg_sarea(int type, void *s);

int cisif_intc_handler(int irq, FAR void *context, FAR void *arg);

const intc_func_table g_intcomp_func[] =
  {
    cisif_vs_int,            /* VS_INT */
    NULL,                    /* EOY_INT */
    NULL,                    /* SOY_INT */
    NULL,                    /* EOI_INT */
    NULL,                    /* SOI_INT */
    NULL,                    /* YCC_VACT_END_INT */
    NULL,                    /* JPG_VACT_END_INT */
    cisif_ycc_axi_trdn_int,  /* YCC_AXI_TRDN_INT */
    cisif_ycc_nstorage_int,  /* YCC_NSTORAGE_INT */
    NULL,                    /* YCC_DAREA_END_INT */
    cisif_jpg_axi_trdn_int,  /* JPG_AXI_TRDN_INT */
    cisif_jpg_nstorage_int,  /* JPG_NSTORAGE_INT */
    NULL,                    /* JPG_DAREA_END_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    NULL,                    /* VLATCH_INT */
    cisif_ycc_err_int,       /* SIZE_OVER_INT */
    cisif_ycc_err_int,       /* SIZE_UNDER_INT */
    cisif_ycc_err_int,       /* YCC_MARKER_ERR_INT */
    cisif_ycc_err_int,       /* YCC_AXI_TRERR_INT */
    cisif_ycc_err_int,       /* YCC_FIFO_OVF_INT */
    cisif_ycc_err_int,       /* YCC_MEM_OVF_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    cisif_jpg_err_int,       /* JPG_MARKER_ERR_INT */
    cisif_jpg_err_int,       /* JPG_AXI_TRERR_INT */
    cisif_jpg_err_int,       /* JPG_FIFO_OVF_INT */
    cisif_jpg_err_int,       /* JPG_MEM_OVF_INT */
    cisif_jpg_err_int,       /* JPG_ERR_STATUS_INT */
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef CISIF_INTR_TRACE
static uint64_t cisif_get_msec_time(void)
{
  struct timespec tp;

  if (clock_gettime(CLOCK_REALTIME, &tp))
    {
      return 0;
    }
  return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}
static void cisif_trace_time_start(void)
{
  g_cisif_time_start = (uint32_t)cisif_get_msec_time();
}
static void cisif_trace_time_stop(char *str)
{
  g_cisif_time_stop = (uint32_t)cisif_get_msec_time();
  printf("%s:%d[ms]\n",str,(uint32_t)(g_cisif_time_stop - g_cisif_time_start));
}
void cisif_intrtrace_start(int max)
{
  g_cisif_vint_count_max = max;
  g_cisif_vint_count = 0;
  cisif_trace_time_start();
}
#endif /* CISIF_INTR_TRACE */

/*******************************************************************************
 * cisif_vs_int
 *******************************************************************************/
static void cisif_vs_int(uint8_t code)
{
#ifdef CISIF_INTR_TRACE
  if(g_cisif_vint_count < g_cisif_vint_count_max)
    {
#ifdef CISIF_DBG_CONTI_CAP
      printf("VCNT:%d,NUM:%d,LAST:%d ",
             g_conti_vcnt, g_conti_capnum, g_conti_lastframe);
#endif /* CISIF_DBG_CONTI_CAP */
      cisif_trace_time_stop("cisif_vs_int");
      cisif_trace_time_start();
      g_cisif_vint_count++;
    }
  else
    {
      g_cisif_vint_count_max = 0;
    }
#endif /* CISIF_INTR_TRACE */

  switch (g_state)
    {
      case STATE_STANDBY:
        cifinfo("invalid state\n");
        break;

      case STATE_READY:
        break;

      case STATE_CAPTURE:
        g_state = STATE_STOP;
        cisif_reg_write(CISIF_DIN_ENABLE, 1);
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      case STATE_CONTI_CAPTURE:
        if (g_conti_capnum != g_conti_max_capnum)
          {
            g_conti_vcnt++;
          }
        else
          {
            g_state = STATE_READY;
          }
        break;

      case STATE_START_MONITORING:
        g_state = STATE_MONITORING;
        cisif_reg_write(CISIF_DIN_ENABLE, 1);
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      case STATE_MONITORING:
        g_bank ^= 1;
        if (g_ycc_storage_addr[g_bank] != NULL)
          {
            cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)g_ycc_storage_addr[g_bank]);
          }
        if (g_jpg_storage_addr[g_bank] != NULL)
          {
            cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)g_jpg_storage_addr[g_bank]);
          }
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      case STATE_STOP:
        g_state = STATE_READY;
        g_bank ^= 1;
        cisif_reg_write(CISIF_DIN_ENABLE, 0);
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      default:
        cifinfo("invalid state\n");
        break;
    }

}

/*******************************************************************************
 * cisif_ycc_axi_trdn_int
 *******************************************************************************/
static void cisif_ycc_axi_trdn_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_ycc_axi_trdn_int");
#endif /* CISIF_INTR_TRACE */

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  if (g_state == STATE_READY)
    {
      g_comp_callback_func[TYPE_CISIF_YUV](0, 1, size, (uint32_t)g_ycc_storage_addr[g_bank ^ 1]);
    }
  else
    {
      g_comp_callback_func[TYPE_CISIF_YUV](0, 0, size, (uint32_t)g_ycc_storage_addr[g_bank ^ 1]);
    }

  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_ycc_nstorage_int
 *******************************************************************************/
static void cisif_ycc_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_notify_callback_func[TYPE_CISIF_YUV](0, size, (uint32_t)g_ycc_storage_addr[g_bank ^ 1]);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, size);
}

/*******************************************************************************
 * cisif_jpg_axi_trdn_int
 *******************************************************************************/
static void cisif_jpg_axi_trdn_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
#ifdef CISIF_DBG_CONTI_CAP
  printf("VCNT:%d,NUM:%d,LAST:%d ",
         g_conti_vcnt, g_conti_capnum, g_conti_lastframe);
#endif /* CISIF_DBG_CONTI_CAP */

  cisif_trace_time_stop("cisif_jpg_axi_trdn_int");
#endif /* CISIF_INTR_TRACE */
  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  if (g_state == STATE_READY)
    {
      g_comp_callback_func[TYPE_CISIF_JPEG](
        0, 1, size, (uint32_t)g_jpg_storage_addr[g_bank ^ 1]);
    }
  else if (g_state == STATE_CONTI_CAPTURE)
    {
      if (g_conti_vcnt > g_conti_interval)
        {
          g_conti_capnum++;

          if (g_conti_capnum >= g_conti_max_capnum)
            {
              cisif_reg_write(CISIF_DIN_ENABLE, 0);
              cisif_reg_write(CISIF_EXE_CMD, 1);

              g_conti_lastframe = 1;
              g_comp_callback_func[TYPE_CISIF_JPEG](
                0, g_conti_lastframe, size, g_jpg_sarea_addr);

            }
          else
            {
              cisif_reg_write(CISIF_JPG_START_ADDR, g_jpg_sarea_addr + size);
              cisif_reg_write(CISIF_JPG_DAREA_SIZE, g_jpg_sarea_size - size);
              cisif_reg_write(CISIF_EXE_CMD, 1);

              g_comp_callback_func[TYPE_CISIF_JPEG](
                0, g_conti_lastframe, size, g_jpg_sarea_addr);

              g_jpg_sarea_addr += size;
              g_jpg_sarea_size -= size;
            }

          g_conti_vcnt = 0;
        }
    }
  else
    {
      g_comp_callback_func[TYPE_CISIF_JPEG](
        0, 0, size, (uint32_t)g_jpg_storage_addr[g_bank ^ 1]);
    }

  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);

}

/*******************************************************************************
 * cisif_jpg_nstorage_int
 *******************************************************************************/
static void cisif_jpg_nstorage_int(uint8_t code)
{
  uint32_t size;
  uint32_t addr;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  if (g_state != STATE_CONTI_CAPTURE)
    {
      addr = (uint32_t)g_jpg_storage_addr[g_bank ^ 1];
    }
  else
    {
      addr = g_jpg_sarea_addr;
    }
  g_notify_callback_func[TYPE_CISIF_JPEG](0, size, addr);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, size);
}

/*******************************************************************************
 * cisif_ycc_err_int
 *******************************************************************************/
static void cisif_ycc_err_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_ycc_err_int");
#endif /* CISIF_INTR_TRACE */

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_comp_callback_func[TYPE_CISIF_YUV](code, 1, size, (uint32_t)g_ycc_storage_addr[g_bank ^ 1]);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_jpg_err_int
 *******************************************************************************/
static void cisif_jpg_err_int(uint8_t code)
{
  uint32_t size;
  uint32_t addr;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_jpg_err_int");
#endif /* CISIF_INTR_TRACE */

  if (g_state != STATE_CONTI_CAPTURE)
    {
      addr = (uint32_t)g_jpg_storage_addr[g_bank ^ 1];
    }
  else
    {
      addr = g_jpg_sarea_addr;
    }

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  g_comp_callback_func[TYPE_CISIF_JPEG](code, 1, size, addr);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_intc_handler
 *******************************************************************************/
int cisif_intc_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t value;
  uint32_t enable;
  uint8_t  index;

  value = cisif_reg_read(CISIF_INTR_STAT);
  cisif_reg_write(CISIF_INTR_CLEAR, value & ALL_CLEAR_INT);
  cifinfo("int stat %08x\n", value);

  enable = cisif_reg_read(CISIF_INTR_ENABLE);
  value = (value & enable);

  for (index = 0; index < sizeof(g_intcomp_func) / sizeof(g_intcomp_func[0]); index++)
    {
      if ((value & (1 << index)) != 0)
        {
          g_intcomp_func[index](index);
        }

    }

  return OK;
}

/*******************************************************************************
 * cisif_reg_write
 *******************************************************************************/
static void cisif_reg_write(uint16_t reg, uint32_t val)
{
  putreg32(val, CXD56_CISIF_BASE + reg);
}

/*******************************************************************************
 * cisif_reg_read
 *******************************************************************************/
static uint32_t cisif_reg_read(uint16_t reg)
{
  return getreg32(CXD56_CISIF_BASE + reg);
}

/****************************************************************************
 * cisif_check_param
 ****************************************************************************/
static int cisif_check_param(cisif_param_t *p)
{
  if (p == NULL)
    {
      return -EINVAL;
    }

  switch (p->format)
    {
      case FORMAT_CISIF_YUV:
        if (p->yuv_param.comp_func == NULL)
          {
            return -EINVAL;
          }
        break;

      case FORMAT_CISIF_JPEG:
        if (p->jpg_param.comp_func == NULL)
          {
            return -EINVAL;
          }
        break;

      case FORMAT_CISIF_INTERLEAVE:
        if (p->yuv_param.comp_func == NULL ||
            p->jpg_param.comp_func == NULL)
          {
            return -EINVAL;
          }
        break;

      default:
        return -EINVAL;
    }

  /* YUV input enable */
  if (p->yuv_param.comp_func != NULL)
    {
      if (p->yuv_param.hsize < YUV_HSIZE_MIN ||
          p->yuv_param.hsize > YUV_HSIZE_MAX ||
          p->yuv_param.vsize < YUV_VSIZE_MIN ||
          p->yuv_param.vsize > YUV_VSIZE_MAX)
        {
          return -EINVAL;
        }

      if (p->yuv_param.notify_func != NULL)
        {
          if (p->yuv_param.notify_size == 0)
            {
              return -EINVAL;
            }
        }
    }

  /* JPEG input enable */
  if (p->jpg_param.comp_func != NULL)
    {
      if (p->jpg_param.notify_func != NULL)
        {
          if (p->jpg_param.notify_size == 0)
            {
              return -EINVAL;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * cisif_check_sarea
 ****************************************************************************/
static int cisif_check_sarea(int type, void *s)
{
  if (s == NULL)
    {
      return -EINVAL;
    }

  if (type == SAREA_SINGLE)
    {
      cisif_sarea_t *ss = (cisif_sarea_t *)s;

      if (ss->strg_addr == NULL ||
          ss->strg_size == 0 ||
          (uint32_t)ss->strg_addr % 32 != 0)
        {
          return -EINVAL;
        }
    }
  else
    {
      cisif_bank_sarea_t *sb = (cisif_bank_sarea_t *)s;

      if (sb->strg_addr_0 == NULL ||
          sb->strg_addr_1 == NULL ||
          sb->strg_size   == 0    ||
          (uint32_t)sb->strg_addr_0 % 32 != 0 ||
          (uint32_t)sb->strg_addr_1 % 32 != 0)
        {
          return -EINVAL;
        }
    }

  return OK;
}


/****************************************************************************
 * cisif_set_yuvparam
 ****************************************************************************/
static int cisif_set_yuv_param(cisif_param_t *p)
{
  uint32_t act_size = 0;

  act_size = (p->yuv_param.vsize & 0x1FF) << 16;
  act_size |= p->yuv_param.hsize & 0x1FF;

  cisif_reg_write(CISIF_ACT_SIZE, act_size);
  cisif_reg_write(CISIF_CIS_SIZE, act_size);
  /* must align 32 bytes */
  cisif_reg_write(CISIF_YCC_NSTRG_SIZE, (p->yuv_param.notify_size&0xffffffe0));

  g_notify_callback_func[TYPE_CISIF_YUV] = p->yuv_param.notify_func;
  g_comp_callback_func[TYPE_CISIF_YUV]   = p->yuv_param.comp_func;

  return OK;
}

/****************************************************************************
 * cisif_set_yuvsarea
 ****************************************************************************/
static int cisif_set_yuv_sarea(int type, void *s)
{
  if (type == SAREA_SINGLE)
    {
      cisif_sarea_t *ss = (cisif_sarea_t *)s;
      /* must align 32 bytes */
      cisif_reg_write(CISIF_YCC_DAREA_SIZE, (ss->strg_size&0xffffffe0));
      cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)ss->strg_addr);

      g_ycc_storage_addr[0] = (uint8_t *)ss->strg_addr;
      g_ycc_storage_addr[1] = NULL;
    }
  else
    {
      cisif_bank_sarea_t *sb = (cisif_bank_sarea_t *)s;
      /* must align 32 bytes */
      cisif_reg_write(CISIF_YCC_DAREA_SIZE, (sb->strg_size&0xffffffe0));
      cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)sb->strg_addr_0);

      g_ycc_storage_addr[0] = (uint8_t *)sb->strg_addr_0;
      g_ycc_storage_addr[1] = (uint8_t *)sb->strg_addr_1;
    }

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_param
 ****************************************************************************/
static int cisif_set_jpg_param(cisif_param_t *p)
{
  /* must align 32 bytes */
  cisif_reg_write(CISIF_JPG_NSTRG_SIZE, (p->jpg_param.notify_size&0xffffffe0));

  g_notify_callback_func[TYPE_CISIF_JPEG] = p->jpg_param.notify_func;
  g_comp_callback_func[TYPE_CISIF_JPEG]   = p->jpg_param.comp_func;

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/
static int cisif_set_jpg_sarea(int type, void *s)
{
  if (type == SAREA_BANK)
    {
      cisif_bank_sarea_t *sb = (cisif_bank_sarea_t *)s;
      /* must align 32 bytes */
      cisif_reg_write(CISIF_JPG_DAREA_SIZE, (sb->strg_size&0xffffffe0));
      cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)sb->strg_addr_0);

      g_jpg_storage_addr[0] = (uint8_t *)sb->strg_addr_0;
      g_jpg_storage_addr[1] = (uint8_t *)sb->strg_addr_1;
    }
  else
    {
      cisif_sarea_t *ss = (cisif_sarea_t *)s;
      /* must align 32 bytes */
      cisif_reg_write(CISIF_JPG_DAREA_SIZE, (ss->strg_size&0xffffffe0));
      cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)ss->strg_addr);

      if (type == SAREA_SINGLE)
        {
          g_jpg_storage_addr[0] = (uint8_t *)ss->strg_addr;
          g_jpg_storage_addr[1] = NULL;
        }
      else
        {
          g_jpg_sarea_addr = (uint32_t)ss->strg_addr;
          g_jpg_sarea_size = ss->strg_size;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * cxd56_cisifinit
 ****************************************************************************/
int cxd56_cisifinit(void)
{
  if (g_state != STATE_STANDBY)
    {
      return -EPERM;
    }

  /* enable CISIF clock */
  cxd56_img_cisif_clock_enable();

  /* disable CISIF interrupt */
  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);

  /* attach interrupt handler */
  irq_attach(CXD56_IRQ_CISIF, cisif_intc_handler, NULL);
  /* enable CISIF irq  */
  up_enable_irq(CXD56_IRQ_CISIF);

#ifdef CISIF_INTR_TRACE
  cisif_reg_write(CISIF_INTR_ENABLE, VS_INT);
#endif /* CISIF_INTR_TRACE */

  g_state = STATE_READY;

  return OK;
}

/****************************************************************************
 * cxd56_cisiffinalize
 ****************************************************************************/
int cxd56_cisiffinalize(void)
{
  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  /* disable CISIF irq  */
  up_disable_irq(CXD56_IRQ_CISIF);
  /* detach interrupt handler */
  irq_detach(CXD56_IRQ_CISIF);

  /* disable CISIF interrupt */
  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);

  /* disable CISIF clock */
  cxd56_img_cisif_clock_disable();

  g_state = STATE_STANDBY;

  return OK;
}

/****************************************************************************
 * cxd56_cisifcaptureframe
 ****************************************************************************/
int cxd56_cisifcaptureframe(
  cisif_param_t *param,
  cisif_sarea_t *yuv_sarea,
  cisif_sarea_t *jpg_sarea)
{
  uint32_t cisif_mode;
  uint32_t interrupts = VS_INT;
  int ret;

  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  ret = cisif_check_param(param);
  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);

  switch (param->format)
    {
      case FORMAT_CISIF_YUV:
        ret = cisif_check_sarea(SAREA_SINGLE, yuv_sarea);
        if (ret != OK)
          {
            return ret;
          }

        cisif_set_yuv_param(param);
        cisif_set_yuv_sarea(SAREA_SINGLE, yuv_sarea);

        cisif_mode = MODE_YUV_TRS_EN;
        interrupts |= YCC_INT_ALL;
        break;

      case FORMAT_CISIF_JPEG:
        ret = cisif_check_sarea(SAREA_SINGLE, jpg_sarea);
        if (ret != OK)
          {
            return ret;
          }

        cisif_set_jpg_param(param);
        cisif_set_jpg_sarea(SAREA_SINGLE, jpg_sarea);

        cisif_mode = MODE_JPG_TRS_EN;
        interrupts |= JPG_INT_ALL;
        break;

      case FORMAT_CISIF_INTERLEAVE:
        ret = cisif_check_sarea(SAREA_SINGLE, yuv_sarea);
        if (ret != OK)
          {
            return ret;
          }

        ret = cisif_check_sarea(SAREA_SINGLE, jpg_sarea);
        if (ret != OK)
          {
            return ret;
          }

        cisif_set_yuv_param(param);
        cisif_set_yuv_sarea(SAREA_SINGLE, jpg_sarea);
        cisif_set_jpg_param(param);
        cisif_set_jpg_sarea(SAREA_SINGLE, jpg_sarea);

        cisif_mode = MODE_INTLEV_TRS_EN;
        interrupts |= YCC_INT_ALL | JPG_INT_ALL;
        break;

      default:
        return -EINVAL;
    }

  g_bank  = 0;
  g_state = STATE_CAPTURE;

  if (g_notify_callback_func[TYPE_CISIF_YUV] != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (g_notify_callback_func[TYPE_CISIF_JPEG] != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisif_reg_write(CISIF_MODE, cisif_mode);
  cisif_reg_write(CISIF_INTR_CLEAR, interrupts);
  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

#ifdef CISIF_INTR_TRACE
  if (g_cisif_vint_count_max == 0)
    {
      g_cisif_vint_count = 0;
      cisif_intrtrace_start(10);
    }
#endif /* CISIF_INTR_TRACE */

  return OK;
}

/******************************************************************************
 * cxd56_cisifstartmonitoring
 *****************************************************************************/
int cxd56_cisifstartmonitoring(
  cisif_param_t *param,
  cisif_bank_sarea_t *yuv_area,
  cisif_bank_sarea_t *jpg_area)
{
  uint32_t cisif_mode;
  uint32_t interrupts = VS_INT;
  int ret;

  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  ret = cisif_check_param(param);
  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);

  switch (param->format)
    {
      case FORMAT_CISIF_YUV:
        ret = cisif_check_sarea(SAREA_BANK, yuv_area);
        if (ret != OK)
          {
            return ret;
          }

        cisif_set_yuv_param(param);
        cisif_set_yuv_sarea(SAREA_BANK, yuv_area);

        cisif_mode = MODE_YUV_TRS_EN;
        interrupts |= YCC_INT_ALL;
        break;

      case FORMAT_CISIF_JPEG:
        ret = cisif_check_sarea(SAREA_BANK, jpg_area);
        if (ret != OK)
          {
            return ret;
          }

        cisif_set_jpg_param(param);
        cisif_set_jpg_sarea(SAREA_BANK, jpg_area);

        cisif_mode = MODE_JPG_TRS_EN;
        interrupts |= JPG_INT_ALL;
        break;

      case FORMAT_CISIF_INTERLEAVE:
        ret = cisif_check_sarea(SAREA_BANK, yuv_area);
        if (ret != OK)
          {
            return ret;
          }

        ret = cisif_check_sarea(SAREA_BANK, jpg_area);
        if (ret != OK)
          {
            return ret;
          }

        cisif_set_yuv_param(param);
        cisif_set_yuv_sarea(SAREA_BANK, yuv_area);
        cisif_set_jpg_param(param);
        cisif_set_jpg_sarea(SAREA_BANK, jpg_area);

        cisif_mode = MODE_INTLEV_TRS_EN;
        interrupts |= YCC_INT_ALL | JPG_INT_ALL;
        break;

      default:
        return -EINVAL;
    }

  g_bank  = 0;
  g_state = STATE_START_MONITORING;

  if (g_notify_callback_func[TYPE_CISIF_YUV] != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (g_notify_callback_func[TYPE_CISIF_JPEG] != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisif_reg_write(CISIF_MODE, cisif_mode);
  cisif_reg_write(CISIF_INTR_CLEAR, interrupts);
  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

  return OK;
}

/******************************************************************************
 * cxd56_cisifstopmonitoring
 *****************************************************************************/
int cxd56_cisifstopmonitoring(void)
{
  if (g_state != STATE_MONITORING)
    {
      return -EPERM;
    }

  g_state = STATE_STOP;

  return OK;
}

/****************************************************************************
 * cxd56_cisifcontinuouscapture
 ****************************************************************************/
int cxd56_cisifcontinuouscapture(cisif_param_t *param, cisif_sarea_t *sarea)
{
  int ret;
  uint32_t interrupts = VS_INT | JPG_INT_ALL;

  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  if (param->format != FORMAT_CISIF_JPEG)
    {
      return -EPERM;
    }

  ret = cisif_check_param(param);
  if (ret != OK)
    {
      return ret;
    }

  ret = cisif_check_sarea(SAREA_SINGLE, sarea);
  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);

  cisif_set_jpg_param(param);
  cisif_set_jpg_sarea(SAREA_FLAT, sarea);

  g_conti_max_capnum = sarea->capnum;
  g_conti_interval   = sarea->interval;
  g_conti_capnum     = 0;
  g_conti_vcnt       = sarea->interval;
  g_conti_lastframe  = 0;

  g_state = STATE_CONTI_CAPTURE;

  if (g_notify_callback_func[TYPE_CISIF_JPEG] != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

#ifdef CISIF_DBG_CONTI_CAP
  printf("cxd56_cisifcontinuouscapture\n");
#endif /* CISIF_DBG_CONTI_CAP */

  cisif_reg_write(CISIF_MODE, MODE_JPG_TRS_EN);
  cisif_reg_write(CISIF_INTR_CLEAR, interrupts);
  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

#ifdef CISIF_INTR_TRACE
  if (g_cisif_vint_count_max == 0)
    {
      g_cisif_vint_count = 0;
      cisif_intrtrace_start(10);
    }
#endif /* CISIF_INTR_TRACE */

  cisif_reg_write(CISIF_DIN_ENABLE, 1);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}
