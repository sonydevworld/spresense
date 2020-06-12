/****************************************************************************
 * modules/lte/altcom/gw/apicmdgw.c
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

#include <stdlib.h>
#include <string.h>

#include "dbg_if.h"
#include "buffpoolwrapper.h"
#include "apicmdgw.h"
#include "apicmd_errind.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of Main task stack size. */

#define APICMDGW_MAIN_TASK_STACK_SIZE (4096)

/* Length of main task name. */

#define APICMDGW_MAX_MAIN_TASK_NAME_LEN (16)

#define APICMDGW_HDR_CHKSUM_LEN         (14)
#define APICMDGW_FTR_CHKSUM_LEN         (2)

#define APICMDGW_APICMDHDR_LEN          (sizeof(struct apicmd_cmdhdr_s))
#define APICMDGW_APICMDFTR_LEN          (sizeof(struct apicmd_cmdftr_s))

#define APICMDGW_HDR_ERR_VER            (-1)
#define APICMDGW_HDR_ERR_CHKSUM         (-2)
#define APICMDGW_DATA_ERR_CHKSUM        (-3)

#define APICMDGW_GET_SEQID              (++g_seqid_counter)
#define APICMDGW_GET_CMDID(hdr_ptr) \
  (ntohs(((FAR struct apicmd_cmdhdr_s *)hdr_ptr)->cmdid))
#define APICMDGW_GET_DATA_PTR(hdr_ptr) \
  (((FAR uint8_t *)(hdr_ptr) + APICMDGW_APICMDHDR_LEN))
#define APICMDGW_GET_HDR_PTR(data_ptr) \
  (((FAR uint8_t *)(data_ptr) - APICMDGW_APICMDHDR_LEN))
#define APICMDGW_GET_TRANSID(hdr_ptr) \
  (ntohs(((FAR struct apicmd_cmdhdr_s *)hdr_ptr)->transid))
#define APICMDGW_GET_DATA_LEN(hdr_ptr) \
  (ntohs(((FAR struct apicmd_cmdhdr_s *)hdr_ptr)->dtlen))
#define APICMDGW_GET_OPT(hdr_ptr) \
  (ntohs(((FAR struct apicmd_cmdhdr_s *)hdr_ptr)->options))
#define APICMDGW_GET_FTR_PTR(hdr_ptr) \
  (((FAR uint8_t *)(hdr_ptr) + APICMDGW_APICMDHDR_LEN \
    + APICMDGW_GET_DATA_LEN(hdr_ptr)))

#define APICMDGW_GET_RESCMDID(cmdid) (cmdid | 0x01 << 15)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct apicmdgw_blockinf_s
{
  FAR uint8_t                     *recvbuff;
  FAR uint16_t                    *recvlen;
  uint16_t                        cmdid;
  uint16_t                        transid;
  uint16_t                        bufflen;
  sys_thread_cond_t               waitcond;
  sys_mutex_t                     waitcondmtx;
  int32_t                         result;
  FAR struct apicmdgw_blockinf_s  *next;
};

struct apicmd_hdr_opts_s
{
  bool datachksum_en;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool                           g_isinit        = false;
static FAR struct apicmdgw_blockinf_s *g_blkinfotbl   = NULL;
static sys_mutex_t                    g_blkinfotbl_mtx;
static sys_task_t                     g_rcvtask;
static uint8_t                        g_seqid_counter = 0;
static sys_thread_cond_t              g_delwaitcond;
static sys_mutex_t                    g_delwaitcondmtx;
static FAR struct hal_if_s            *g_hal_if       = NULL;
static FAR struct evtdisp_s           *g_evtdisp      = NULL;
static sys_cremtx_s                   g_mtxparam;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdgw_createtransid
 *
 * Description:
 *   Create transaction ID.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Non-zero value is returned.
 *
 ****************************************************************************/

static uint16_t apicmdgw_createtransid(void)
{
  static uint16_t transid = 0;

  transid++;
  if (!transid)
    {
      transid++;
    }

  return transid;
}

/****************************************************************************
 * Name: apicmdgw_createchksum
 *
 * Description:
 *   Create api command checksum.
 *
 * Input Parameters:
 *   ptr       Pointer to calculating checksum.
 *
 * Returned Value:
 *   Returns checksum value.
 *
 ****************************************************************************/

static uint16_t apicmdgw_createchksum(FAR uint8_t *ptr, uint16_t len)
{
  uint32_t ret     = 0x00;
  uint16_t calctmp = 0x00;
  uint16_t i;
  int      is_odd  = len & 0x01;

  for (i = 0; i < (len & 0xFFFE); i += sizeof(uint16_t))
    {
      calctmp = *((uint16_t *)(ptr + i));
      ret += ntohs(calctmp);
    }

  if (is_odd)
    {
      ret += *(ptr + i) << 8;
    }

  ret = ~((ret & 0xFFFF) + (ret >> 16));
  DBGIF_LOG1_DEBUG("create check sum. chksum = %04x.\n", (uint16_t)ret);

  return (uint16_t)ret;
}

/****************************************************************************
 * Name: apicmdgw_createhdrchksum
 *
 * Description:
 *   Create api command header checksum.
 *
 * Input Parameters:
 *   hdr       api command header.
 *
 * Returned Value:
 *   Returns checksum value.
 *
 ****************************************************************************/

static uint16_t apicmdgw_createhdrchksum(FAR uint8_t *hdr)
{
  return apicmdgw_createchksum(hdr, APICMDGW_HDR_CHKSUM_LEN);
}

/****************************************************************************
 * Name: apicmdgw_createdtchksum
 *
 * Description:
 *   Create api command data + footer checksum.
 *
 * Input Parameters:
 *   hdr       api command header.
 *
 * Returned Value:
 *   Returns checksum value.
 *
 ****************************************************************************/

static uint16_t apicmdgw_createdtchksum(FAR uint8_t *hdr)
{
  FAR struct apicmd_cmdhdr_s *hdr_ptr = (FAR struct apicmd_cmdhdr_s *)hdr;
  uint16_t                    len;

  len = APICMDGW_GET_DATA_LEN(hdr_ptr) + APICMDGW_FTR_CHKSUM_LEN;
  return apicmdgw_createchksum(APICMDGW_GET_DATA_PTR(hdr), len);
}

/****************************************************************************
 * Name: apicmdgw_checkheader
 *
 * Description:
 *   Check api command header.
 *
 * Input Parameters:
 *   evt       Api command header pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise APICMDGW_HDR_ERR value is returned.
 *
 ****************************************************************************/

static int32_t apicmdgw_checkheader(FAR uint8_t *evt)
{
  FAR struct apicmd_cmdhdr_s *hdr   = NULL;
  uint16_t                   chksum = 0;

  hdr = (FAR struct apicmd_cmdhdr_s *)evt;
  if (hdr->ver != APICMD_VER)
    {
      DBGIF_LOG2_ERROR("version mismatch [sender:0x%x, receiver:0x%x]\n",
        hdr->ver, APICMD_VER);
      return APICMDGW_HDR_ERR_VER;
    }

  if (APICMD_PAYLOAD_SIZE_MAX < APICMDGW_GET_DATA_LEN(hdr))
    {
      DBGIF_LOG1_ERROR("Data length error. [data len:%d]\n", APICMDGW_GET_DATA_LEN(hdr));
      return APICMDGW_HDR_ERR_VER;
    }

  chksum = apicmdgw_createhdrchksum((FAR uint8_t *)hdr);
  if (chksum != ntohs(hdr->chksum))
    {
      DBGIF_LOG2_ERROR("header checksum error [header:0x%04x, calculation:0x%04x]\n", ntohs(hdr->chksum), chksum);
      return APICMDGW_HDR_ERR_CHKSUM;
    }

  DBGIF_LOG2_INFO("Receive header[cmd_id:0x%04x, data len:0x%04x]\n",ntohs(hdr->cmdid), ntohs(hdr->dtlen));

  return 0;
}

/****************************************************************************
 * Name: apicmdgw_checkdata
 *
 * Description:
 *   Check api command data.
 *
 * Input Parameters:
 *   evt       Api command header pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise APICMDGW_DATA_ERR_CHKSUM value is returned.
 *
 ****************************************************************************/

static int32_t apicmdgw_checkdata(FAR uint8_t *evt,
                                  FAR struct apicmd_hdr_opts_s *opts)
{
  FAR struct apicmd_cmdhdr_s *hdr   = NULL;
  FAR struct apicmd_cmdftr_s *ftr   = NULL;
  uint16_t                   chksum = 0;

  hdr = (FAR struct apicmd_cmdhdr_s *)evt;

  if (opts->datachksum_en)
    {
      chksum = apicmdgw_createdtchksum((FAR uint8_t *)hdr);

      ftr = (FAR struct apicmd_cmdftr_s *)APICMDGW_GET_FTR_PTR(hdr);

      if (chksum != ntohs(ftr->chksum))
        {
          DBGIF_LOG2_ERROR("data checksum error [footer:0x%04x, calculation:0x%04x]\n", ntohs(ftr->chksum), chksum);
          return APICMDGW_DATA_ERR_CHKSUM;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: apicmdgw_parse_hdr_options
 *
 * Description:
 *   Perse options value from header field.
 *
 * Input Parameters:
 *   hdr   API command header pointer.
 *   opts  Pointer to a structure that summarizes the options.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void apicmdgw_parse_hdr_options(FAR uint8_t *hdr,
                                       FAR struct apicmd_hdr_opts_s *opts)
{
  opts->datachksum_en = false;

  if (APICMD_OPT_DATA_CHKSUM_ENABLED(APICMDGW_GET_OPT(hdr)))
    {
      opts->datachksum_en = true;
    }
}

/****************************************************************************
 * Name: apicmdgw_errind
 *
 * Description:
 *  handling process error.
 *
 * Input Parameters:
 *  evt  handling process function pointer.
 *  evlen  length of event.
 *
 * Returned Value: 
 *  function pointer handling result.
 *
 ****************************************************************************/

void apicmdgw_errind(FAR struct apicmd_cmdhdr_s *evthdr)
{
  FAR struct apicmd_cmddat_errind_s *errind = NULL;
  int32_t ret;

  errind = (FAR struct apicmd_cmddat_errind_s *)
              apicmdgw_cmd_allocbuff(APICMDID_ERRIND,
                sizeof(struct apicmd_cmddat_errind_s));
  DBGIF_ASSERT(NULL != errind, "apicmdgw_cmd_allocbuff()\n");

  errind->ver     = evthdr->ver;
  errind->seqid   = evthdr->seqid;
  errind->cmdid   = htons(evthdr->cmdid);
  errind->transid = htons(evthdr->transid);
  errind->dtlen   = htons(evthdr->dtlen);
  errind->chksum  = htons(evthdr->chksum);

  ret = APICMDGW_SEND_ONLY((uint8_t *)errind);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("APICMDGW_SEND_ONLY() failed:%d\n", ret);
    }

  DBGIF_ASSERT(0 == apicmdgw_freebuff((uint8_t *)errind),
    "apicmdgw_freebuff()\n");
}

/****************************************************************************
 * Name: apicmdgw_errhandle
 *
 * Description:
 *  handling process error.
 *
 * Input Parameters:
 *  evt  handling process function pointer.
 *  evlen  length of event.
 *
 * Returned Value: 
 *  function pointer handling result.
 *
 ****************************************************************************/

void apicmdgw_errhandle(FAR struct apicmd_cmdhdr_s *evthdr)
{
  DBGIF_LOG_ERROR("dispatch error\n");
  DBGIF_LOG1_ERROR("version:0x%x\n", evthdr->ver);
  DBGIF_LOG1_ERROR("sequence ID:0x%x\n", evthdr->seqid);
  DBGIF_LOG1_ERROR("command ID:0x%x\n", ntohs(evthdr->cmdid));
  DBGIF_LOG1_ERROR("transaction ID:0x%x\n", ntohs(evthdr->transid));
  DBGIF_LOG1_ERROR("data length:0x%x\n", ntohs(evthdr->dtlen));
  DBGIF_LOG1_ERROR("options:0x%x\n", ntohs(evthdr->options));
  DBGIF_LOG1_ERROR("check sum:0x%x\n", ntohs(evthdr->chksum));
}

/****************************************************************************
 * Name: apicmdgw_addtable
 *
 * Description:
 *   Add wait table fot waittablelist.
 *
 * Input Parameters:
 *   tbl    waittable.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void apicmdgw_addtable(FAR struct apicmdgw_blockinf_s *tbl)
{
  sys_lock_mutex(&g_blkinfotbl_mtx);

  if (!g_blkinfotbl)
    {
      g_blkinfotbl = tbl;
    }
  else
    {
      tbl->next    = g_blkinfotbl;
      g_blkinfotbl = tbl;
    }

  sys_unlock_mutex(&g_blkinfotbl_mtx);
}

/****************************************************************************
 * Name: apicmdgw_remtable
 *
 * Description:
 *   Remove wait table fot waittablelist.
 *
 * Input Parameters:
 *   tbl    waittable.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void apicmdgw_remtable(FAR struct apicmdgw_blockinf_s *tbl)
{
  FAR struct apicmdgw_blockinf_s *tmptbl;

  DBGIF_ASSERT(g_blkinfotbl, "table list is null.\n");

  sys_lock_mutex(&g_blkinfotbl_mtx);

  tmptbl = g_blkinfotbl;
  if (tmptbl == tbl)
    {
      g_blkinfotbl = g_blkinfotbl->next;
    }
  else
    {
      while(tmptbl->next)
        {
          if (tmptbl->next == tbl)
            {
              tmptbl->next = tmptbl->next->next;
              tmptbl = tbl;
              break;
            }

          tmptbl = tmptbl->next;
        }
    }

  DBGIF_ASSERT(tmptbl == tbl, "Can not find a table from the table list.");
  sys_delete_thread_cond_mutex(&tmptbl->waitcond, &tmptbl->waitcondmtx);
  BUFFPOOL_FREE(tmptbl);

  sys_unlock_mutex(&g_blkinfotbl_mtx);
}

/****************************************************************************
 * Name: apicmdgw_writetable
 *
 * Description:
 *   Get wait table for waittablelist and write data.
 *
 * Input Parameters:
 *   transid    Transaction id.
 *   cmdid      Api command id.
 *   data       Write data.
 *   datalen    @data length.
 *
 * Returned Value:
 *   If get wait table for wait table list success, return true. 
 *   Otherwise false is returned.
 *
 ****************************************************************************/

static bool apicmdgw_writetable(uint16_t cmdid,
  uint16_t transid, FAR uint8_t *data, uint16_t datalen)
{
  int32_t                        ret;
  bool                           result = false;
  FAR struct apicmdgw_blockinf_s *tbl   = NULL;

  sys_lock_mutex(&g_blkinfotbl_mtx);

  tbl = g_blkinfotbl;
  while(tbl)
    {
      if (tbl->transid == transid && tbl->cmdid == cmdid)
        {
          result = true;
          break;
        }

      tbl = tbl->next;
    }

  if (tbl)
    {
      if (datalen <= tbl->bufflen)
        {
          tbl->result = 0;
          memcpy(tbl->recvbuff, data, datalen);
          *(tbl->recvlen) = datalen;
        }
      else
        {
          tbl->result = -ENOSPC;
          DBGIF_LOG2_ERROR("Unexpected length. datalen: %d, bufflen: %d\n", datalen, tbl->bufflen);
        }

      ret = sys_signal_thread_cond(&tbl->waitcond, &tbl->waitcondmtx);
      DBGIF_ASSERT(0 == ret, "sys_signal_thread_cond().\n");
    }

  sys_unlock_mutex(&g_blkinfotbl_mtx);

  return result;
}

/****************************************************************************
 * Name: apicmdgw_relcondwaitall
 *
 * Description:
 *   Release all waiting tasks.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void apicmdgw_relcondwaitall(void)
{
  int32_t ret;
  FAR struct apicmdgw_blockinf_s *tmptbl;

  sys_lock_mutex(&g_blkinfotbl_mtx);

  tmptbl = g_blkinfotbl;
  while(tmptbl)
    {
      ret = sys_signal_thread_cond(&tmptbl->waitcond, &tmptbl->waitcondmtx);
      DBGIF_ASSERT(0 == ret, "sys_signal_thread_cond().\n");

      tmptbl = tmptbl->next;
    }

  sys_unlock_mutex(&g_blkinfotbl_mtx);
}

/****************************************************************************
 * Name: apicmdgw_recvtask
 *
 * Description:
 *   Main process of receiving API command gateway
 *
 * Input Parameters:
 *   arg     Option parameter.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/

static void apicmdgw_recvtask(void *arg)
{
  #define APICMDGW_RECV_MAGICNUMBER_LEN    (1)
  #define APICMDGW_RECV_STATUS_MAGICNUMBER (1)
  #define APICMDGW_RECV_STATUS_HEADER      (2)
  #define APICMDGW_RECV_STATUS_DATA        (3)
  #define APICMDGW_MAGICNUMBER_LENGTH      (4)
  #define APICMDGW_HEADER_REMAIN_LNGTH     (12)

  int32_t                        ret;
  uint32_t                       magicnum  = 0;
  uint8_t                        recvsts   = APICMDGW_RECV_STATUS_MAGICNUMBER;
  FAR uint8_t                    *rcvbuff  = NULL;
  FAR uint8_t                    *evtbuff  = NULL;
  FAR uint8_t                    *rcvptr   = NULL;
  uint16_t                       rcvlen    = APICMDGW_RECV_MAGICNUMBER_LEN;
  uint16_t                       totallen  = 0;
  uint16_t                       i         = 0;
  uint16_t                       datalen   = 0;
  struct apicmd_hdr_opts_s       opts      = {0};

  #define APICMDGW_RECV_STATUS_INIT() \
    { \
      recvsts  = APICMDGW_RECV_STATUS_MAGICNUMBER; \
      rcvptr   = rcvbuff; \
      rcvlen   = APICMDGW_RECV_MAGICNUMBER_LEN; \
      totallen = 0; \
      DBGIF_LOG_DEBUG("APICMDGW_RECV_STATUS_MAGICNUMBER\n");\
    }

  rcvbuff = (uint8_t *)g_hal_if->allocbuff(
              g_hal_if, APICMDGW_RECVBUFF_SIZE_MAX);
  DBGIF_ASSERT(rcvbuff, "g_hal_atunsolevt->allocbuff()\n");
  rcvptr = rcvbuff;

  while (true)
    {
      if (rcvlen)
        {
          if (APICMDGW_RECVBUFF_SIZE_MAX < totallen + rcvlen)
            {
              memset(rcvbuff, 0, APICMDGW_RECVBUFF_SIZE_MAX);
              APICMDGW_RECV_STATUS_INIT();
            }

          ret = g_hal_if->recv(g_hal_if, rcvptr, rcvlen);
        }
      else
        {
          ret = 0;
        }
      if (0 > ret)
        {
          if (-ECONNABORTED == ret)
            {
              DBGIF_LOG_NORMAL("recv() abort\n");
              break;
            }
          else
            {
              DBGIF_LOG1_ERROR("recv() [errno=%d]\n", ret);
              APICMDGW_RECV_STATUS_INIT();
              continue;
            }
        }
      else
        {
          totallen += ret;
          switch (recvsts)
            {
              case APICMDGW_RECV_STATUS_MAGICNUMBER:
                {
                  if (APICMDGW_MAGICNUMBER_LENGTH <= totallen)
                    {
                      for (i = 0;
                        i <= totallen - APICMDGW_MAGICNUMBER_LENGTH; i++)
                        {
                          memcpy(&magicnum, &rcvbuff[i],
                            APICMDGW_MAGICNUMBER_LENGTH);
                          if (APICMD_MAGICNUMBER == ntohl(magicnum))
                            {
                              memmove(rcvbuff, &rcvbuff[i],
                                APICMDGW_MAGICNUMBER_LENGTH);
                              memset(rcvbuff + APICMDGW_MAGICNUMBER_LENGTH, 0,
                                &rcvbuff[i] - rcvbuff);
                              totallen = APICMDGW_MAGICNUMBER_LENGTH;

                              recvsts = APICMDGW_RECV_STATUS_HEADER;
                              rcvptr  = rcvbuff + APICMDGW_MAGICNUMBER_LENGTH;
                              rcvlen  = APICMDGW_HEADER_REMAIN_LNGTH;
                              DBGIF_LOG_DEBUG("APICMDGW_RECV_STATUS_HEADER\n");
                              break;
                            }
                        }
                    }
                  if (APICMDGW_RECV_STATUS_HEADER != recvsts)
                    {
                      rcvptr++;
                      rcvlen = APICMDGW_RECV_MAGICNUMBER_LEN;
                    }
                }
                break;
              case APICMDGW_RECV_STATUS_HEADER:
                {
                  if (totallen == APICMDGW_APICMDHDR_LEN)
                    {
                      if (0 == apicmdgw_checkheader(rcvbuff))
                        {
                          apicmdgw_parse_hdr_options(rcvbuff, &opts);

                          datalen = APICMDGW_GET_DATA_LEN(rcvbuff);
                          datalen += APICMDGW_APICMDFTR_LEN;

                          recvsts = APICMDGW_RECV_STATUS_DATA;
                          rcvptr = rcvbuff + APICMDGW_APICMDHDR_LEN;
                          rcvlen = datalen;
                          DBGIF_LOG_DEBUG("APICMDGW_RECV_STATUS_DATA\n");
                        }
                      else
                        {
                          apicmdgw_errind(
                            (FAR struct apicmd_cmdhdr_s *)rcvbuff);
                          APICMDGW_RECV_STATUS_INIT();
                          g_hal_if->reset_modem(g_hal_if);
                        }
                    }
                  else
                    {
                      rcvptr += ret;
                      rcvlen = APICMDGW_APICMDHDR_LEN - totallen;
                    }
                }
                break;
              case APICMDGW_RECV_STATUS_DATA:
                {
                  if (totallen == APICMDGW_APICMDHDR_LEN + datalen)
                    {
                      if (0 == apicmdgw_checkdata(rcvbuff, &opts))
                        {
                          if (!apicmdgw_writetable(
                            APICMDGW_GET_CMDID(rcvbuff),
                            APICMDGW_GET_TRANSID(rcvbuff),
                            APICMDGW_GET_DATA_PTR(rcvbuff),
                            APICMDGW_GET_DATA_LEN(rcvbuff)))
                            {
                              evtbuff =
                                (uint8_t *)g_hal_if->allocbuff(g_hal_if,
                                                               totallen);
                              DBGIF_ASSERT(evtbuff, "BUFFPOOL_ALLOC() error.\n");
                              memcpy(evtbuff, rcvbuff, totallen);

                              ret = g_evtdisp->dispatch(g_evtdisp,
                                APICMDGW_GET_DATA_PTR(evtbuff),
                                APICMDGW_GET_DATA_LEN(evtbuff));
                              if (0 > ret)
                                {
                                  apicmdgw_errhandle(
                                    (FAR struct apicmd_cmdhdr_s *)evtbuff);
                                  g_hal_if->freebuff(g_hal_if, evtbuff);
                                  DBGIF_LOG1_ERROR("dispatch() [errno=%d]\n",ret);
                                }
                            }
                        }
                      else
                        {
                          apicmdgw_errind(
                            (FAR struct apicmd_cmdhdr_s *)rcvbuff);
                          g_hal_if->reset_modem(g_hal_if);
                        }

                      APICMDGW_RECV_STATUS_INIT();
                    }
                  else
                    {
                      rcvptr += ret;
                      rcvlen = APICMDGW_APICMDHDR_LEN + datalen - totallen;
                    }
                }
                break;
              default:
                DBGIF_LOG1_ERROR("recvsts = %d\n", recvsts);
                APICMDGW_RECV_STATUS_INIT();
                continue;
            }
        }
    }

  if (rcvbuff)
    {
      g_hal_if->freebuff(g_hal_if, (void *)rcvbuff);
    }

  ret = sys_signal_thread_cond(&g_delwaitcond, &g_delwaitcondmtx);
  DBGIF_ASSERT(0 == ret, "sys_signal_thread_cond().\n");

  ret = sys_delete_task(SYS_OWN_TASK);
  DBGIF_ASSERT(0 == ret, "sys_delete_task()\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdgw_init
 *
 * Description:
 *   Initialize api command gateway instance.
 *
 * Input Parameters:
 *   halif    hal_if_s pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_init(FAR struct apicmdgw_set_s *set)
{
  int32_t       ret;
  sys_cretask_s taskset;
  char          thname[] = "apicmdgw_main";

  if (!set || !set->halif || !set->dispatcher)
    {
      DBGIF_LOG_ERROR("null parameter.\n");
      return -EINVAL;
    }

  if (g_isinit)
    {
      DBGIF_LOG_ERROR("apicmdgw is initialized.\n");
      return -EPERM;
    }

  g_hal_if  = set->halif;
  g_evtdisp = set->dispatcher;

  ret = sys_create_thread_cond_mutex(&g_delwaitcond, &g_delwaitcondmtx);
  DBGIF_ASSERT(0 == ret, "sys_create_thread_cond_mutex().\n");

  taskset.function   = apicmdgw_recvtask;
  taskset.name       = (FAR int8_t *)thname;
  taskset.priority   = SYS_TASK_PRIO_HIGH;
  taskset.stack_size = APICMDGW_MAIN_TASK_STACK_SIZE;

  ret = sys_create_mutex(&g_blkinfotbl_mtx, &g_mtxparam);
  DBGIF_ASSERT(0 == ret, "sys_create_mutex().\n");

  ret = sys_create_task(&g_rcvtask, &taskset);
  DBGIF_ASSERT(0 == ret, "sys_create_task().\n");

  g_isinit = true;

  return ret;
}

/****************************************************************************
 * Name: apicmdgw_fin
 *
 * Description:
 *   Finalize api command gateway instance.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_fin(void)
{
  int32_t ret;

  if (!g_isinit)
    {
      DBGIF_LOG_ERROR("apicmdgw is finalized.\n");
      return -EPERM;
    }

  g_isinit = false;

  sys_lock_mutex(&g_delwaitcondmtx);

  ret = g_hal_if->abortrecv(g_hal_if);
  DBGIF_ASSERT(0 <= ret, "abortrecv()\n");

  ret = sys_thread_cond_wait(&g_delwaitcond, &g_delwaitcondmtx);
  DBGIF_ASSERT(0 <= ret, "sys_wait_thread_cond()\n");

  sys_unlock_mutex(&g_delwaitcondmtx);

  sys_delete_thread_cond_mutex(&g_delwaitcond, &g_delwaitcondmtx);
  apicmdgw_relcondwaitall();

  ret = sys_delete_mutex(&g_blkinfotbl_mtx);
  DBGIF_ASSERT(0 == ret, "sys_delete_mutex().\n");

  g_hal_if       = NULL;
  g_evtdisp      = NULL;

  return ret;
}

/****************************************************************************
 * Name: apicmdgw_send
 *
 * Description:
 *   Send api command.
 *   And wait to response for parameter of timeout_ms value when
 *   parameter of respbuff set valid buffer.
 *   Non wait to response when parameter of respbuff set NULL.
 *
 * Input Parameters:
 *   cmd         Send command payload pointer.
 *   respbuff    Response buffer.
 *   bufflen     @respbuff length.
 *   resplen     Response length.
 *   timeout_ms  Response wait timeout value (msec).
 *               When use SYS_TIMEO_FEVR to waiting non timeout.
 *
 * Returned Value:
 *   On success, the length of the sent command in bytes is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_send(FAR uint8_t *cmd, FAR uint8_t *respbuff,
    uint16_t bufflen, FAR uint16_t *resplen, int32_t timeout_ms)
{
  int32_t                         ret;
  uint32_t                        sendlen;
  FAR struct apicmd_cmdhdr_s      *hdr_ptr;
  FAR struct apicmd_cmdftr_s      *ftr_ptr;
  FAR struct apicmdgw_blockinf_s  *blocktbl = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_ERROR("apicmd gw in not initialized.\n");
      return -EPERM;
    }

  if (!cmd || (respbuff && !resplen))
    {
      DBGIF_LOG_ERROR("Invalid argument.\n");
      return -EINVAL;
    }

  hdr_ptr = (FAR struct apicmd_cmdhdr_s *)APICMDGW_GET_HDR_PTR(cmd);

  sendlen = ntohs(hdr_ptr->dtlen) + APICMDGW_APICMDHDR_LEN
              + APICMDGW_APICMDFTR_LEN;

  ftr_ptr = (FAR struct apicmd_cmdftr_s *)APICMDGW_GET_FTR_PTR(hdr_ptr);
  ftr_ptr->chksum = htons(apicmdgw_createdtchksum((FAR uint8_t *)hdr_ptr));

  if (respbuff)
    {
      blocktbl = (FAR struct apicmdgw_blockinf_s *)
        BUFFPOOL_ALLOC(sizeof(struct apicmdgw_blockinf_s));
      if (!blocktbl)
        {
          DBGIF_LOG_ERROR("BUFFPOOL_ALLOC() failed.\n");
          return -ENOSPC;
        }

      /* Set wait table. */

      blocktbl->transid  = APICMDGW_GET_TRANSID(hdr_ptr);
      blocktbl->recvbuff = respbuff;
      blocktbl->bufflen  = bufflen;
      blocktbl->cmdid    =
        APICMDGW_GET_RESCMDID(APICMDGW_GET_CMDID(hdr_ptr));
      blocktbl->recvlen  = resplen;
      ret = sys_create_thread_cond_mutex(&blocktbl->waitcond,
                                         &blocktbl->waitcondmtx);
      if (0 > ret)
        {
          BUFFPOOL_FREE(blocktbl);
          return ret;
        }

      apicmdgw_addtable(blocktbl);

      sys_lock_mutex(&blocktbl->waitcondmtx);

      g_hal_if->lock(g_hal_if);
      ret = g_hal_if->send(g_hal_if, (FAR uint8_t *)hdr_ptr, sendlen);
      g_hal_if->unlock(g_hal_if);

      if (0 > ret)
        {
          DBGIF_LOG_ERROR("hal_if->send() failed.\n");
        }
      else
        {
          /* Wait until the response data is received or timeout. */

          ret = sys_thread_cond_timedwait(&blocktbl->waitcond,
                                          &blocktbl->waitcondmtx, timeout_ms);
          if (0 > ret)
            {
              ret = -ETIMEDOUT;
            }
          else
            {
              if (0 > blocktbl->result)
                {
                  ret = blocktbl->result;
                }

              if (!g_isinit)
                {
                  ret = -ECONNABORTED;
                }
            }
        }
      sys_unlock_mutex(&blocktbl->waitcondmtx);

      apicmdgw_remtable(blocktbl);
    }
  else
    {
      /* Send only */

      g_hal_if->lock(g_hal_if);
      ret = g_hal_if->send(g_hal_if, (FAR uint8_t *)hdr_ptr, sendlen);
      g_hal_if->unlock(g_hal_if);

      if (0 > ret)
        {
          DBGIF_LOG_ERROR("hal_if->send() failed.\n");
          return ret;
        }
    }

  if (0 <= ret)
    {
      ret = ntohs(hdr_ptr->dtlen);
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdgw_sendabort
 *
 * Description:
 *   Abort api command send, And release sync command response waiting.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, the length of the sent command in bytes is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_sendabort(void)
{
  int32_t                        ret = 0;
  FAR struct apicmdgw_blockinf_s *tbl = NULL;

  sys_lock_mutex(&g_blkinfotbl_mtx);

  tbl = g_blkinfotbl;
  while (tbl)
    {
      tbl->result = -ENETDOWN;
      tbl = tbl->next;
    }

  sys_unlock_mutex(&g_blkinfotbl_mtx);
  apicmdgw_relcondwaitall();

  return ret;
}

/****************************************************************************
 * Name: apicmdgw_cmd_allocbuff
 *
 * Description:
 *   Allocate buffer for API command to be sent. The length to be allocated
 *   is the sum of the data length and header length.
 *   And this function is make api command header in allocated buffer.
 *
 * Input Parameters:
 *   cmdid    Api command id.
 *   len      Length of data field.
 *
 * Returned Value:
 *   If succeeds allocate buffer, start address of the data field
 *   is returned. Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR uint8_t *apicmdgw_cmd_allocbuff(uint16_t cmdid, uint16_t len)
{
  FAR struct apicmd_cmdhdr_s *buff = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_ERROR("apicmd gw in not initialized.\n");
      return NULL;
    }

  if (APICMD_PAYLOAD_SIZE_MAX < len)
    {
      DBGIF_LOG1_ERROR("Over max API command data size. len:%d\n", len);
      return NULL;
    }

  buff = (FAR struct apicmd_cmdhdr_s *)g_hal_if->allocbuff(
    g_hal_if, len + APICMDGW_APICMDHDR_LEN + APICMDGW_APICMDFTR_LEN);
  if (!buff)
    {
      DBGIF_LOG_ERROR("hal_if->allocbuff failed.\n");
      return NULL;
    }

  /* Make header. */

  buff->magic   = htonl(APICMD_MAGICNUMBER);
  buff->ver     = APICMD_VER;
  buff->seqid   = APICMDGW_GET_SEQID;
  buff->cmdid   = htons(cmdid);
  buff->transid = htons(apicmdgw_createtransid());
  buff->dtlen   = htons(len);
  buff->options = htons(APICMD_OPT_DATA_CHKSUM_ENABLE);
  buff->chksum  = htons(apicmdgw_createhdrchksum((FAR uint8_t *)buff));

  return APICMDGW_GET_DATA_PTR(buff);
}

/****************************************************************************
 * Name: apicmdgw_reply_allocbuff
 *
 * Description:
 *   Allocate buffer for API command to be sent. The length to be allocated
 *   is the sum of the data length and header length.
 *   And this function is make api resopnse command header in allocated buffer.
 *
 * Input Parameters:
 *   cmd      Replyning to api command payload pointer.
 *   len      Length of data field.
 *
 * Returned Value:
 *   If succeeds allocate buffer, start address of the data field
 *   is returned. Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR uint8_t *apicmdgw_reply_allocbuff(FAR const uint8_t *cmd, uint16_t len)
{
  FAR struct apicmd_cmdhdr_s *buff   = NULL;
  FAR struct apicmd_cmdhdr_s *evthdr = NULL;

  if (!g_isinit)
    {
      DBGIF_LOG_ERROR("apicmd gw in not initialized.\n");
      return NULL;
    }

  if (!cmd)
    {
      DBGIF_LOG_ERROR("Invalid argument.\n");
      return NULL;
    }

  if (APICMD_PAYLOAD_SIZE_MAX < len)
    {
      DBGIF_LOG1_ERROR("Over max API command data size. len:%d\n", len);
      return NULL;
    }

  buff = (FAR struct apicmd_cmdhdr_s *)g_hal_if->allocbuff(
    g_hal_if, len + APICMDGW_APICMDHDR_LEN + APICMDGW_APICMDFTR_LEN);
  if (!buff)
    {
      DBGIF_LOG_ERROR("hal_if->allocbuff failed.\n");
      return NULL;
    }

  /* Make reply header. */

  evthdr = (FAR struct apicmd_cmdhdr_s *)APICMDGW_GET_HDR_PTR(cmd);
  buff->magic   = htonl(APICMD_MAGICNUMBER);
  buff->ver     = APICMD_VER;
  buff->seqid   = APICMDGW_GET_SEQID;
  buff->cmdid   = htons(
    APICMDGW_GET_RESCMDID(APICMDGW_GET_CMDID(evthdr)));
  buff->transid = evthdr->transid;
  buff->dtlen   = htons(len);
  buff->options = htons(APICMD_OPT_DATA_CHKSUM_ENABLE);
  buff->chksum  = htons(apicmdgw_createhdrchksum((FAR uint8_t *)buff));

  return APICMDGW_GET_DATA_PTR(buff);
}

/****************************************************************************
 * Name: apicmdgw_freebuff
 *
 * Description:
 *   Free allocated buffer.
 *
 * Input Parameters:
 *   buff  Pointer to data field.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno in errno.h is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_freebuff(FAR uint8_t *buff)
{
  if (!g_isinit)
    {
      DBGIF_LOG_ERROR("apicmd gw in not initialized.\n");
      return -EPERM;
    }

  if (!buff)
    {
      DBGIF_LOG_INFO("freebuff target buffer is null.\n");
      return 0;
    }

  return g_hal_if->freebuff(g_hal_if,
    (FAR void *)APICMDGW_GET_HDR_PTR(buff));
}

/****************************************************************************
 * Name: apicmdgw_cmdid_compare
 *
 * Description:
 *   Compare to receive event command id and waiting command id.
 *
 * Input Parameters:
 *   cmd      Receive command payload pointer.
 *   cmdid    Waiting command id.
 *
 * Returned Value:
 *   If the process succeeds, it returns true.
 *   Otherwise false is returned.
 *
 ****************************************************************************/

bool apicmdgw_cmdid_compare(FAR uint8_t *cmd, uint16_t cmdid)
{
  uint16_t rcvid;

  if (!cmd)
    {
      return false;
    }

  rcvid = APICMDGW_GET_CMDID(APICMDGW_GET_HDR_PTR(cmd));
  if (rcvid == cmdid)
    {
      return true;
    }

  return false;
}
