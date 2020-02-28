/****************************************************************************
 * modules/sensing/tap/tap_manager.cpp
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
#include <errno.h>
#include <debug.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <assert.h>
#include <sched.h>
#include <signal.h>
#include "sensing/tap_manager.h"
#include <nuttx/sensors/bmi160.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define TAP_MNG_ACC_DRIVER                 "/dev/accel0"
#define TAP_MNG_ACC_SAMPLING_FREQ          (64) /* 64Hz      */
#define TAP_MNG_ACC_WATERMARK_NUM          (4)  /* 4 samples */
#define TAP_MNG_FIFO_NUM                   (2)
#define TAP_MNG_FIFO_SIZE                  (sizeof(tap_mng_three_axis_s) \
                                            * TAP_MNG_ACC_SAMPLING_FREQ \
                                            * TAP_MNG_FIFO_NUM)
#define TAP_MNG_TAP_LIB_THREAD_STACK_SIZE  (2048)
#define TAP_MNG_TASK_NAME                  "tap_mng_task"
#define TAP_MNG_TASK_PRIORITY              (100)
#define TAP_MNG_TASK_STACK_SIZE            (2048)
#define TAP_MNG_LOCK(lock)                 do { \
                                             int r; \
                                             r = sem_wait(&lock); \
                                             if (r == 0) \
                                             { \
                                               break; \
                                             } \
                                           } while(1)
#define TAP_MNG_UNLOCK(lock)               do { \
                                             sem_post(&lock); \
                                           } while(0)
#define TAP_MNG_NODE_LOCK()                TAP_MNG_LOCK(g_tap_mng_node_lock)
#define TAP_MNG_NODE_UNLOCK()              TAP_MNG_UNLOCK(g_tap_mng_node_lock)
#define TAP_MNG_CMD_LOCK()                 TAP_MNG_LOCK(g_tap_mng_acccmd_lock)
#define TAP_MNG_CMD_UNLOCK()               TAP_MNG_UNLOCK(g_tap_mng_acccmd_lock)
#define TAP_MNG_STS_LOCK()                 do { sched_lock(); } while(0)
#define TAP_MNG_STS_UNLOCK()               do { sched_unlock(); } while(0)
#define TAP_MNG_ACC_SIGNAL                 SIGUSR1
#define TAP_MNG_CMD_SIGNAL                 SIGUSR2
#define TAP_MNG_ACC_INIT                   1
#define TAP_MNG_ACC_START                  2
#define TAP_MNG_ACC_STOP                   3
#define TAP_MNG_ACC_CLOSE                  4
#define TAP_MNG_ACCEL_CONVERT(func)        (float)((float)((func) * 2.0) / 32768.0)

/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef enum
{
  E_TAP_MNG_STS_UNINIT,
  E_TAP_MNG_STS_INIT
} E_TAP_MNG_STATUS;

struct tap_mng_node
{
  int                 ctl_id;
  TapClass            *tap;
  tap_mng_out_cbs     cbs;
  struct tap_mng_node *previous;
  struct tap_mng_node *next;
};

struct tap_mng_three_axis_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct tap_mng_acc_data_buf
{
  struct    tap_mng_three_axis_s acc_data[(TAP_MNG_ACC_SAMPLING_FREQ * TAP_MNG_FIFO_NUM)];
  uint64_t  time_stamp;
};

struct tap_mng_inlibdata
{
  ST_TAP_ACCEL  accel;
  uint64_t      timestamp;
};

static sem_t                 g_tap_mng_node_lock;
static sem_t                 g_tap_mng_acccmd_lock;
static sem_t                 g_tap_mng_acccmd_complete;
static E_TAP_MNG_STATUS      g_sts                     = E_TAP_MNG_STS_UNINIT;
static struct tap_mng_node   *g_head                   = NULL;
static int                   g_ctl_id                  = 1;
static struct scutimestamp_s g_ts                      = {0};
static int                   g_tap_mng_task_id         = 0;
static int                   g_tap_mng_acccmd_id       = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
static int TapMngsendAcccmd(int acc_cmd)
{
  int          ret = 0;
  union sigval value;

  TAP_MNG_CMD_LOCK();

  if (g_tap_mng_task_id != 0)
    {
      if (acc_cmd != TAP_MNG_ACC_INIT)
        {
          g_tap_mng_acccmd_id = acc_cmd;

          /* send signal */

#ifdef CONFIG_CAN_PASS_STRUCTS
          value.sival_ptr = NULL;
          (void)sigqueue(g_tap_mng_task_id, TAP_MNG_CMD_SIGNAL, value);
#else
          (void)sigqueue(g_tap_mng_task_id, TAP_MNG_CMD_SIGNAL, NULL);
#endif

        }
      /* waiting for command complete */

      while(1)
        {
          ret = sem_wait(&g_tap_mng_acccmd_complete);
          if ((ret != 0) && (errno == EINTR))
            {
              continue;
            }
          else
            {
              break;
            }
        }
    }
  else
    {
      ret = -ESRCH;
    }

  TAP_MNG_CMD_UNLOCK();

  return ret;
}

static int TapMngsendAcccmdResp()
{
  int ret = 0;

  /* notify command complete */

  ret = sem_post(&g_tap_mng_acccmd_complete);

  return ret;
}

#ifdef CONFIG_CXD56_SCU
static int TapMngSetScu(int fd)
{
  int                 ret = 0;
  struct scufifo_wm_s wm  = {0};

  /* Set FIFO size to 6 bytes * 64 Hz * 2 = 768 */

  ret = ioctl(fd, SCUIOC_SETFIFO, TAP_MNG_FIFO_SIZE);
  if (0 > ret)
    {
      _err("L%d SETFIFO failed. %d\n", __LINE__, ret);
      goto exit;
    }

  /* Set sequencer sampling rate 64 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 3) = 64
   */

  ret = ioctl(fd, SCUIOC_SETSAMPLE, 3);
  if (0 > ret)
    {
      _err("L%d SETSAMPLE failed. %d\n", __LINE__, ret);
      goto exit;
    }

  wm.signo     = TAP_MNG_ACC_SIGNAL;
  wm.ts        = &g_ts;
  wm.watermark = TAP_MNG_ACC_WATERMARK_NUM;

  ret = ioctl(fd, SCUIOC_SETWATERMARK, (unsigned long)(uintptr_t)&wm);
  if (0 > ret)
    {
      _err("L%d SETWATERMARK failed. %d\n", __LINE__, ret);
      goto exit;
    }

exit:
  return ret;
}
#endif

static void TapMngTapLibRun(void)
{
  struct tap_mng_node         *p_node      = NULL;
  int                         fd           = -1;
  int                         tapCnt       = 0;
  int                         icnt         = 0;
  int                         ret          = 0;
  int                         rsize        = 0;
  int                         acc_data_num = 0;
  struct tap_mng_acc_data_buf *data        = NULL;
  struct tap_mng_three_axis_s *ta          = NULL;
  struct tap_mng_inlibdata    indata       = {0};
  sigset_t                    set          = {0};
  struct siginfo              siginfo      = {0};
  struct timespec             ts           = {0};
  struct timespec             timeout      = {0};

  /* allocate read buffer */

  data = (struct tap_mng_acc_data_buf *)malloc(sizeof(struct tap_mng_acc_data_buf));
  ASSERT(NULL != data);

  sigemptyset(&set);
  sigaddset(&set, TAP_MNG_ACC_SIGNAL);
  sigaddset(&set, TAP_MNG_CMD_SIGNAL);

  timeout.tv_sec = 2;
  timeout.tv_nsec = 0;

  /* send responce ACC_INIT complete */

  TapMngsendAcccmdResp();

  while(1)
    {
      /* waiting signal */

      if (fd < 0)
        {
          /* SCU has stopped in this case */

          ret = sigtimedwait(&set, &siginfo, NULL);
        }
      else
        {
          /* SCU has started in this case */

          ret = sigtimedwait(&set, &siginfo, &timeout);
        }
      if (ret < 0)
        {
          int errcode = errno;

          if ((errcode != EINTR) && (errcode != EAGAIN))
            {
              _err("L%d ERROR: sigtimedwait() failed:%d %s\n",
                  __LINE__, errcode, strerror(errcode));
              goto close;
            }

          _warn("L%d Timeout!!\n", __LINE__);
          continue;
        }

      /* received signal from SCU */

      if (siginfo.si_signo == TAP_MNG_ACC_SIGNAL)
        {
          if (fd < 0)
            {
              /* not opened yet */

              _err("L%d ERROR: unexpected signal receive\n", __LINE__);
              continue;
            }
          /* read max FIFO size from SCU */

          rsize = read(fd, (char *)&data->acc_data, sizeof(data->acc_data));
          if (0 > rsize)
            {
              _err("L%d ERROR: read() failed: %d\n", __LINE__, errno);
              goto close;
            }
          if (0 != (rsize % sizeof(tap_mng_three_axis_s)))
            {
              _err("L%d ERROR: read() unexpected size: %ld\n", __LINE__,
                  (long)rsize);
              goto close;
            }
          else
            {
              acc_data_num = (rsize / sizeof(tap_mng_three_axis_s));
              if (acc_data_num > TAP_MNG_ACC_WATERMARK_NUM)
                {
                  _err("L%d acc_data_num:%d\n", __LINE__, acc_data_num);
                }
            }

          /* set timestamp */

          clock_gettime(CLOCK_MONOTONIC, &ts);

          data->time_stamp = (ts.tv_sec * SEC_PER_US) + (ts.tv_nsec / NS_PER_US);

          ta = (struct tap_mng_three_axis_s *)&data->acc_data;
          for (icnt = 0; icnt < acc_data_num; icnt++, ta++)
            {
              if (ta->x && ta->y && ta->z)
                {
                  memset(&indata, 0, sizeof(struct tap_mng_inlibdata));
                  indata.accel.accel_x = TAP_MNG_ACCEL_CONVERT(ta->x);
                  indata.accel.accel_y = TAP_MNG_ACCEL_CONVERT(ta->y);
                  indata.accel.accel_z = TAP_MNG_ACCEL_CONVERT(ta->z);

                  indata.timestamp = data->time_stamp - (1000000 / TAP_MNG_ACC_SAMPLING_FREQ) * (acc_data_num - icnt + 1);

                  TAP_MNG_NODE_LOCK();

                  if (NULL == g_head)
                    {
                      _err("L%d g_head is NULL \n", __LINE__);
                      TAP_MNG_NODE_UNLOCK();
                      continue;
                    }

                  p_node = g_head;
                  do
                    {
                      /* Tap Library call */
                      tapCnt = TapWrite_timestamp(p_node->tap, &indata.accel, indata.timestamp);
                      if (tapCnt > 0)
                        {
                          p_node->cbs(tapCnt);
                        }
                      p_node = p_node->next;
                    } while (NULL != p_node);

                  TAP_MNG_NODE_UNLOCK();
                }
            }
        }
      /* receive signal from tap manager api */

      else if (siginfo.si_signo == TAP_MNG_CMD_SIGNAL)
        {
          /* receive start command */

          if (g_tap_mng_acccmd_id == TAP_MNG_ACC_START)
            {
              /* Open Accel */

              fd = open(TAP_MNG_ACC_DRIVER, O_RDONLY);
              ASSERT(0 < fd);

#ifdef CONFIG_CXD56_SCU
              ret = TapMngSetScu(fd);
              ASSERT(0 == ret);

              ret = ioctl(fd, SCUIOC_START, 0);
              if (0 > ret)
                {
                  _err("L%d START failed. %d\n", __LINE__, ret);
                  goto close;
                }
#endif
              TapMngsendAcccmdResp();
            }
          /* receive stop command */

          else if (g_tap_mng_acccmd_id == TAP_MNG_ACC_STOP)
            {
              if (fd >= 0)
                {
#ifdef CONFIG_CXD56_SCU
                  (void) ioctl(fd, SCUIOC_STOP, 0);
#endif
                  close(fd);
                  fd = -1;
                }

              TapMngsendAcccmdResp();
            }
          /* receive close command */

          else  if (g_tap_mng_acccmd_id == TAP_MNG_ACC_CLOSE)
            {
              if (fd >= 0)
                {
#ifdef CONFIG_CXD56_SCU
                  (void) ioctl(fd, SCUIOC_STOP, 0);
#endif
                  close(fd);
                  fd = -1;
                }

              TapMngsendAcccmdResp();

              break;
            }
          else
            {
              _err("L%d ERROR: unexpected command received:%d\n",
                  __LINE__, g_tap_mng_acccmd_id);
            }
        }
      else
        {
          _err("L%d ERROR: unexpected signal received:%d\n",
              __LINE__, siginfo.si_signo);
        }
    }
close:
  if (fd >= 0)
    {
      close(fd);
      fd = -1;
    }
  if (data)
    {
      free(data);
    }
  return;
}

static struct tap_mng_node *TapMngFindNode(int ctl_id)
{
  struct tap_mng_node *p_node = g_head;

  while (NULL != p_node)
    {
      if (p_node->ctl_id == ctl_id)
        {
          break;
        }
      p_node = p_node->next;
    }

  return p_node;
}

static void TapMngAddList(struct tap_mng_node *add_node)
{
  if (g_head)
    {
      add_node->next   = g_head;
      g_head->previous = add_node;
      g_head           = add_node;
    } 
  else
    {
      g_head           = add_node;
    }
}

static bool TapMngListExist(void)
{
  bool exist = false;

  if (g_head)
    {
      exist = true;
    }

  return exist;
}

static struct tap_mng_node *TapMngCreateNode(struct tap_mng_start_param *sta_prm)
{
  int                 ret         = 0;
  struct tap_mng_node *p_new_node = NULL;

  p_new_node = (struct tap_mng_node *)malloc(sizeof(struct tap_mng_node));

  if (NULL == p_new_node)
    {
      _err("L%d p_new_node == NULL\n", __LINE__);
      return p_new_node;
    }

  memset(p_new_node, 0, sizeof(struct tap_mng_node));

  p_new_node->tap = TapCreate();
  if (NULL == p_new_node->tap)
    {
      _err("L%d p_new_node->tap == NULL\n", __LINE__);
      free(p_new_node);
      p_new_node = NULL;
      goto exit;
    }

  ret = TapOpen(p_new_node->tap, &sta_prm->tap_prm);
  if (0 != ret)
    {
      _err("L%d TapOpen error (%d)\n", __LINE__, ret);
      TapClose(p_new_node->tap);
      free(p_new_node);
      p_new_node = NULL;
      goto exit;
    }

  p_new_node->cbs    = sta_prm->cbs;
  p_new_node->ctl_id = g_ctl_id++;

exit:
  return p_new_node;
}

static void TapMngDeleteNode(struct tap_mng_node *del_node)
{
  if (del_node)
    {
      if (NULL != del_node->previous)
        {
          del_node->previous->next = del_node->next;
        } 
      else
        {
          g_head = del_node->next;
        }

      if (NULL != del_node->next)
        {
          del_node->next->previous = del_node->previous;
        }

      TapClose(del_node->tap);
      free(del_node);
    }
}

static void TapMngDeleteAllNode(void)
{
  while (g_head)
    {
      TapMngDeleteNode(g_head);
    }
}

static int TapMngAccInit(void)
{
  int ret  = 0;

  ret = TapMngsendAcccmd(TAP_MNG_ACC_INIT);

  return ret;
}

static int TapMngAccStart(void)
{
  int ret  = 0;

  ret = TapMngsendAcccmd(TAP_MNG_ACC_START);

  return ret;
}

static int TapMngAccStop(void)
{
  int ret  = 0;

  ret = TapMngsendAcccmd(TAP_MNG_ACC_STOP);

  return ret;
}

static int TapMngAccClose(void)
{
  int ret  = 0;

  ret = TapMngsendAcccmd(TAP_MNG_ACC_CLOSE);

  return ret;
}

static void TapMngMain(void)
{
  _err("L%d start\n", __LINE__);

  TapMngTapLibRun();

  g_tap_mng_task_id = 0;

  /* delete self task */

  task_delete(0);

  _err("L%d end\n", __LINE__);
}

/* Initialize */
int TapMngInit(void)
{
  int ret = 0;

  TAP_MNG_STS_LOCK();

  if (E_TAP_MNG_STS_UNINIT != g_sts)
    {
      TAP_MNG_STS_UNLOCK();

      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  g_sts = E_TAP_MNG_STS_INIT;
  TAP_MNG_STS_UNLOCK();

  /* Create lock variables */

  sem_init(&g_tap_mng_node_lock, 0, 1);
  sem_init(&g_tap_mng_acccmd_lock, 0, 1);
  sem_init(&g_tap_mng_acccmd_complete, 0, 0);

  /* Create TAP manager task */

  ret = task_create(TAP_MNG_TASK_NAME, TAP_MNG_TASK_PRIORITY, 
                    TAP_MNG_TASK_STACK_SIZE, (main_t)TapMngMain, NULL);
  if (0 > ret)
    {
      _err("L%d task_create() %s\n", __LINE__, strerror( errno ));

      g_sts = E_TAP_MNG_STS_UNINIT;
    }
  else
    {
      g_tap_mng_task_id = ret;

      /* wait for init TAP manager task */

      ret = TapMngAccInit();
      if (0 != ret)
        {
          _err("L%d ERROR: TapMngAccInit() failed: %d %s\n",
              __LINE__,errno, strerror( errno ));
          g_sts = E_TAP_MNG_STS_UNINIT;
          goto exit;
        }
    }

exit:
  return ret;
}

/* Start */
int TapMngStart(struct tap_mng_start_param *sta_prm, int *ctl_id)
{
  int                 ret         = 0;
  bool                list_exist = false;
  struct tap_mng_node *p_new_node = NULL;

  TAP_MNG_STS_LOCK();

  if (E_TAP_MNG_STS_UNINIT == g_sts)
    {
      TAP_MNG_STS_UNLOCK();

      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  TAP_MNG_STS_UNLOCK();

  if (NULL == sta_prm || NULL == sta_prm->cbs)
    {
      _err("L%d sta_prm == NULL\n", __LINE__);
      ret = -EIO;
      goto exit;
    }

  TAP_MNG_NODE_LOCK();

  /* Check list exist or not */

  list_exist = TapMngListExist();

  /* Create Node */

  p_new_node = TapMngCreateNode(sta_prm);
  if (NULL == p_new_node)
    {
      ret = -ENOMEM;
      TAP_MNG_NODE_UNLOCK();
      goto exit;
    }

  /* Insert Node into list */

  TapMngAddList(p_new_node);

  *ctl_id = p_new_node->ctl_id;

  TAP_MNG_NODE_UNLOCK();

  if (false == list_exist)
    {
      /* Start Accel */

      ret = TapMngAccStart();
      if (0 != ret)
        {
          TapMngDeleteNode(p_new_node);

          _err("L%d ERROR: TapMngAccStart() failed: %d %s\n",
              __LINE__,errno, strerror( errno ));
          goto exit;
        }
    }

exit:
  return ret;
}

/* Stop */
int TapMngStop(int ctl_id)
{
  int                 ret            = 0;
  bool                list_exist     = false;
  struct tap_mng_node *p_tapmng_node = NULL;

  TAP_MNG_STS_LOCK();

  if (E_TAP_MNG_STS_UNINIT == g_sts)
    {
      TAP_MNG_STS_UNLOCK();

      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  TAP_MNG_NODE_LOCK();

  /* Find Node from contol id */

  p_tapmng_node = TapMngFindNode(ctl_id);
  if (NULL != p_tapmng_node)
    {
      /* Remove Node from list */

      TapMngDeleteNode(p_tapmng_node);
    }
  else
    {
      _err("L%d ctl_id error == %d\n", __LINE__, ctl_id);
      ret = -EPERM;
      TAP_MNG_NODE_UNLOCK();
      goto exit;
    }

  /* Check list exist or not */

  list_exist = TapMngListExist();

  TAP_MNG_NODE_UNLOCK();

  if (false == list_exist)
    {
      /* Stop Accel */

      ret = TapMngAccStop();
      if (0 != ret)
        {
          _err("L%d ERROR: TapMngAccStop()failed: %d %s\n",
              __LINE__,errno, strerror( errno ));
          goto exit;
        }
    }

exit:
  return ret;
}

/* Finalize */
int TapMngFin(void)
{
  int  ret  = 0;

  TAP_MNG_STS_LOCK();

  if (E_TAP_MNG_STS_UNINIT == g_sts)
    {
      TAP_MNG_STS_UNLOCK();

      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  TAP_MNG_STS_UNLOCK();

  /* Close Accel */

  ret = TapMngAccClose();
  if (0 != ret)
    {
      _err("L%d ERROR: TapMngAccClose()failed: %d %s\n",
          __LINE__,errno, strerror( errno ));
      goto exit;
    }

  TAP_MNG_NODE_LOCK();

  TapMngDeleteAllNode();

  TAP_MNG_NODE_UNLOCK();

  /* Delete lock variables */

  sem_destroy(&g_tap_mng_node_lock);
  sem_destroy(&g_tap_mng_acccmd_lock);
  sem_destroy(&g_tap_mng_acccmd_complete);

  g_sts = E_TAP_MNG_STS_UNINIT;

exit:
  return ret;
}
