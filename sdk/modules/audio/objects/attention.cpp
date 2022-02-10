/****************************************************************************
 * modules/audio/manager/attention.cpp
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

#include <syslog.h>

#include "audio/audio_high_level_api.h"
#include "attention.h"
#include "audio/audio_message_types.h"

struct AttCbInfo
{
  AudioAttentionCb att_cb;

  AttCbInfo()
    : att_cb(NULL)
    {}
};

static AttCbInfo s_attcb_table[AS_MODULE_ID_NUM];

extern MsgQueId AS_GetSelfDtq(void);
extern bool AS_IsValidDtq(MsgQueId id);

extern "C"
{

void _RegisterAttentionCb(uint32_t module_id, AudioAttentionCb att_cb)
{
  if (module_id < AS_MODULE_ID_NUM)
    {
      s_attcb_table[module_id].att_cb = att_cb;
    }
}

void _UnregisterAttentionCb(uint32_t module_id)
{
  if (module_id < AS_MODULE_ID_NUM)
    {
      s_attcb_table[module_id].att_cb = NULL;
    }
}

#ifdef ATTENTION_USE_FILENAME_LINE

void _Attention(uint8_t module_id,
                uint8_t sub_module_id,
                uint8_t attention_id,
                uint8_t sub_code,
                FAR const char* file_name,
                uint16_t line)
{
#  ifndef CONFIG_AUDIOUTILS_ATTENTIONLOG_DISABLE
  syslog(LOG_ERR,
         "Attention: module[%d][%d] attention id[%d]/code[%d] (%s L%d)\n",
         module_id,
         sub_module_id,
         attention_id,
         sub_code,
         file_name,
         line);
#  endif /* CONFIG_AUDIOUTILS_ATTENTIONLOG_DISABLE */

  ErrorAttentionParam info =
    {
      0,            /* reserve */
      attention_id, /* attention id */
      0,            /* cpu id */
      sub_module_id,/* sub module id */
      module_id,    /* module id */
      sub_code,     /* attention code */
      0,            /* reserve */
      line,         /* line number */
      0,            /* task id */
      0,            /* reserve */
      0             /* dummy(file name) */
    };

  snprintf(static_cast<char*>(info.error_filename), ATTENTION_FILE_NAME_LEN, "%s", file_name);

#ifdef CONFIG_AUDIOUTILS_MANAGER
  MsgQueId msgq_aud_mgr = AS_GetSelfDtq();

  if (AS_IsValidDtq(msgq_aud_mgr))
    {
      err_t er = MsgLib::send(msgq_aud_mgr,
                              MsgPriNormal,
                              MSG_AUD_MGR_CALL_ATTENTION,
                              0,
                              info);
      F_ASSERT(er == ERR_OK);
    }
  else
#endif
    {
      if (s_attcb_table[module_id].att_cb != NULL)
        {
          /* Callback attention */

          s_attcb_table[module_id].att_cb(&info);
        }
      else
        {
          /* There is nothing to do. */ 
        }
    }
}

#else /* ATTENTION_USE_FILENAME_LINE */

void _Attention(uint8_t module_id,
                uint8_t sub_module_id,
                uint8_t attention_id,
                uint8_t sub_code)
{
#  ifndef CONFIG_AUDIOUTILS_ATTENTIONLOG_DISABLE
  syslog(LOG_ERR,
         "Attention: module[%d][%d] attention id[%d]/code[%d]\n",
         module_id,
         sub_module_id,
         attention_id,
         sub_code);
#  endif /* CONFIG_AUDIOUTILS_ATTENTIONLOG_DISABLE */

  ErrorAttentionParam info =
    {
      0,            /* reserve */
      attention_id, /* attention id */
      0,            /* cpu id */
      sub_module_id,/* sub module id */
      module_id,    /* module id */
      sub_code,     /* attention code */
      0,            /* reserve */
      0,            /* line number */
      0,            /* task id */
      0,            /* reserve */
      0             /* dummy(file name) */
    };

  MsgQueId msgq_aud_mgr = AS_GetSelfDtq();

  if (AS_IsValidDtq(msgq_aud_mgr))
    {
      err_t er = MsgLib::send(msgq_aud_mgr,
                              MsgPriNormal,
                              MSG_AUD_MGR_CALL_ATTENTION,
                              0,
                              info);
      F_ASSERT(er == ERR_OK);
    }
  else
    {
      if (s_attcb_table[module_id].att_cb != NULL)
        {
          /* Callback attention */

          s_attcb_table[module_id].att_cb(&info);
        }
      else
        {
          /* There is nothing to do. */ 
        }
    }
}

#endif /* ATTENTION_USE_FILENAME_LINE */

} /* extern "C" */
