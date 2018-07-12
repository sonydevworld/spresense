/****************************************************************************
 * modules/memutils/message/include/MsgWrapper.h
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

#ifndef MSG_WRAPPER_H_INCLUDED
#define MSG_WRAPPER_H_INCLUDED

#ifdef _ITRON4
#include "kernel_id.h"	/* TNUM_TICK */
const uint32_t TickPrecision = TNUM_TICK;
#else
const uint32_t TickPrecision = 1;
#endif
#include "memutils/message/Message.h"

/*****************************************************************
 * メッセージラッパクラス (OS有無などによりCPU毎にカスタマイズが必要)
 *****************************************************************/
class MsgWrapper {
public:
  /* メッセージパケットの送信(タスクコンテキスト、パラメタなし) */
  static err_t sendWithTimeout(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply, size_t ms)
    {
      /* 送信失敗かつタイムアウト時間が指定されていれば、リトライを行う */
      err_t ret = MsgLib::send(dest, pri, type, reply);
      if (ret != ERR_OK && ms > 0)
        {
          err_t        err_code;
          MsgQueBlock* que;
          err_code = MsgLib::referMsgQueBlock(dest, &que);
          if (err_code != ERR_OK)
            {
              return err_code;
            }
          DMP_MSGLIB_WARN(MsgRetryLog('b', type, dest, pri, reply, GET_CPU_ID(), que->getNumMsg(pri), 0, 0x00));
          for (size_t i = 0; ms == TIME_FOREVER || i < ms; i += TickPrecision)
            {
              SleepTask(1);
              ret = MsgLib::send(dest, pri, type, reply);
              if (ret == ERR_OK)
                {
                  break;
                }
            }
          DMP_MSGLIB_WARN(MsgRetryLog((ret == ERR_OK) ? 'n' : 'e',
            type, dest, pri, reply, GET_CPU_ID(), que->getNumMsg(pri), 0, 0x00));
        }
      return ret;
    }

  /* メッセージパケットの送信(タスクコンテキスト、パラメタあり) */
  template<typename T>
  static err_t sendWithTimeout(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply, const T& param, size_t ms)
    {
      /* 送信失敗かつタイムアウト時間が指定されていれば、リトライを行う */
      err_t ret = MsgLib::send(dest, pri, type, reply, param);
      if (ret != ERR_OK && ms > 0)
        {
          err_t        err_code;
          MsgQueBlock* que;
          err_code = MsgLib::referMsgQueBlock(dest, &que);
          if (err_code != ERR_OK)
            {
              return err_code;
            }
          DMP_MSGLIB_WARN(MsgRetryLog('b', type, dest, pri, reply,
            GET_CPU_ID(), que->getNumMsg(pri), sizeof(T), reinterpret_cast<const uint32_t&>(param)));
          for (size_t i = 0; ms == TIME_FOREVER || i < ms; i += TickPrecision)
            {
              SleepTask(1);
              ret = MsgLib::send(dest, pri, type, reply, param);
              if (ret == ERR_OK)
                {
                  break;
                }
            }
          DMP_MSGLIB_WARN(MsgRetryLog((ret == ERR_OK) ? 'n' : 'e', type, dest, pri, reply,
            GET_CPU_ID(), que->getNumMsg(pri), sizeof(T), reinterpret_cast<const uint32_t&>(param)));
        }
      return ret;
    }
}; /* class MsgWrapper */

#endif /* MSG_WRAPPER_H_INCLUDED */
