/****************************************************************************
 * modules/audiolite/src/base/al_evthandler.cxx
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#include <errno.h>
#include <audiolite/al_debug.h>
#include <audiolite/al_evthandler.h>
#include <audiolite/al_component.h>

#define DATA_TYPE  (MOSSFW_DATA_TYPE_CHAR + \
                    MOSSFW_DATA_TYPEGRP_V1 + \
                    MOSSFW_DATA_TYPENAME_AUDIO + \
                    MOSSFW_DATA_TYPEARRAY_ARRAY)

#define STRINGCASE(e) case AL_EVENT_##e: return #e;

/****************************************************************************
 * class: audiolite_evthandler
 ****************************************************************************/

/***********************************************
 * Static Member Variables
 ***********************************************/

SINGLETON_INST(audiolite_evthandler);

/***********************************************
 * Public Class audiolite_evthandler Methods
 ***********************************************/

audiolite_evthandler::audiolite_evthandler(int memnum)
    : _fs(48000), _chnum(2), _bitwidth(16), _listen(NULL)
{
  _pool = new audiolite_mempoolsysmsg;
  _pool->create_instance(memnum);

  _receiver = mossfw_input_create(DATA_TYPE, 4);
  _accepter = mossfw_output_create(DATA_TYPE);

#ifdef __linux__
  _op = mossfw_callback_op_create(audiolite_evthandler::event_handler,
                                  (unsigned long)this, true);
#else
  _op = mossfw_callback_op_create_attr(audiolite_evthandler::event_handler,
                                  (unsigned long)this, true,
                                  CONFIG_AL_EVTHDLR_PRIORITY,
                                  CONFIG_AL_EVTHDLR_STACKSIZE);
#endif
  if (_op && _op->async)
    {
      pthread_setname_np(_op->async->tid, "al_evthdlr");
    }

  mossfw_set_waitcondition(_receiver, 1, _op);
  mossfw_bind_inout(_accepter, _receiver, 0);
}

audiolite_evthandler::~audiolite_evthandler()
{
  mossfw_callback_op_delete(_op);
  mossfw_output_delete(_accepter);
  mossfw_input_delete(_receiver);
  delete _pool;
}

int audiolite_evthandler::event_handler(mossfw_callback_op_t *op,
                                    unsigned long arg)
{
  audiolite_evthandler *thiz = (audiolite_evthandler *)arg;
  audiolite_sysmsg *msg =
        (audiolite_sysmsg *)mossfw_get_delivereddata_array(thiz->_receiver,
                                                           0, NULL);
  if (msg)
    {
      if (thiz->_listen)
        {
          thiz->_listen->on_event(msg->get_evtid(),
                                  (audiolite_component *)msg->get_issuer(),
                                  msg->get_arg());
        }

      msg->release();
    }

  return 0;
}

int audiolite_evthandler::publish_event(int evtid, audiolite_component *issuer,
                                    unsigned long arg)
{
  int ret = -EAGAIN;

  audiolite_sysmsg *msg = (audiolite_sysmsg *)_pool->allocate();
  if (msg)
    {
      msg->set_msgcontent(evtid, issuer, arg);
      mossfw_deliver_dataarray(_accepter, (mossfw_data_t *)msg);
      msg->release();
      ret = OK;
    }

  return ret;
}

int audiolite_evthandler::set_systemparam(int fs, int bitwidth, int chnum)
{
  _fs = fs;
  _chnum = chnum;
  _bitwidth = bitwidth;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void audiolite_set_evtlistener(audiolite_eventlistener *l)
{
  audiolite_evthandler::get_instance()->set_evtlistener(l);
}

int audiolite_set_systemparam(int fs, int chnum, int bitwidth)
{
  return audiolite_evthandler::get_instance()
                               ->set_systemparam(fs, chnum, bitwidth);
}

void audiolite_eventdestroy(void)
{
  audiolite_evthandler::terminate_instance();
}

const char *audiolite_strevent(int evt)
{
  switch (evt)
    {
      STRINGCASE(OVERFLOW)
      STRINGCASE(UNDERFLOW)
      STRINGCASE(ILLIGALSTREAM)
      STRINGCASE(UNSUPPORTFMT)
      STRINGCASE(DECODEDONE)
      STRINGCASE(STREAMDONE)
      STRINGCASE(PLAYSTARTED)
      STRINGCASE(PLAYSTOPPED)
      STRINGCASE(RECORDSTARTED)
      STRINGCASE(RECORDSTOPPED)
      STRINGCASE(PLAYPAUSED)
      STRINGCASE(PLAYRESUMED)
      STRINGCASE(RECORDPAUSED)
      STRINGCASE(RECORDRESUMED)
      STRINGCASE(DRVERROR)
      STRINGCASE(INVALIDSYSPARAM)
      STRINGCASE(STOPOUTPUT)
      STRINGCASE(STOPINPUT)
      STRINGCASE(INITERROR)
      STRINGCASE(SENDERROR)
      STRINGCASE(MP3FRAMEINFO)
      STRINGCASE(MP3DECWORKEREND)
      STRINGCASE(UNKNOWN)
      STRINGCASE(MP3DECERROR)
      STRINGCASE(MP3DEC_WRONGTYPE)
      STRINGCASE(WRONGVERSION)
      default:
        return "not event id...";
    }
}
