/****************************************************************************
 * modules/audiolite/src/components/al_mp3dec.cxx
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

#include <nuttx/config.h>

#include <audiolite/al_debug.h>
#include <audiolite/al_mp3dec.h>
#include <audiolite/al_workercmd.h>
#include <audiolite/al_eventlistener.h>

#include <audiolite/sprmp3dec_qsize.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_AUDIO_LITE_MP3DEC_SUBCORE_SPK
#define MP3WORKERBIN_NAME "mp3dec"
#define MP3WORKERBIN_SPK true
#else
#define MP3WORKERBIN_NAME CONFIG_AIDO_LITE_MP3DEC_SUBCORE_ELFNAME
#define MP3WORKERBIN_SPK false
#endif

/****************************************************************************
 * Class: audiolite_mp3dec
 ****************************************************************************/

void audiolite_mp3dec::decode_runner()
{
  /* Wait for working condition */

  while (_is_thrdrun &&
        (_omempool == NULL || _stream == NULL || !_isplay))
    {
      usleep(10 * 1000);
    }

  while (_is_thrdrun)
    {
      if (_isplay)
        {
          audiolite_memapbuf *mem = 
              (audiolite_memapbuf *)_omempool->allocate();

          if (mem)
            {
              _outq.push(mem);
              alworker_inject_omem(_worker.getwtask(), mem);

              /* No need to release mem here.
               * It will be done after finishing decode.
               */
            }
        }
      else
        {
          /* Yeild */

          usleep(10 * 1000);
        }
    }
}

int audiolite_mp3dec::handle_mesage(al_comm_msghdr_t hdr,
                                    al_comm_msgopt_t *opt, void *arg)
{
  int ret = 0;
  audiolite_mp3dec *thiz = (audiolite_mp3dec *)arg;
  audiolite_memapbuf *mem;

  if (CHECK_HDR(hdr, INST, INST_DONE))
    {
      al_dinfo("DEC DONE\n");
      thiz->publish_event(AL_EVENT_DECODEDONE, 0);
    }
  else if (CHECK_HDR(hdr, FMEM, MEM_RELEASE))
    {
      al_dinfo("FMEM Release\n");
      mem = thiz->_inq.pop();
      if (mem)
        {
          if (thiz->_frame_eof)
            {
              mem->release();
            }
          else
            {
              thiz->_stream->receive_data(mem, 0, -1);
              if (mem->is_eof())
                {
                  if (mem->get_storedsize() == 0)
                    {
                      al_derror("EOF buffer with 0 byte\n");
                      mem->set_storedsize(1);
                    }
                  thiz->_frame_eof = true;
                }

              thiz->_inq.push(mem);
              alworker_inject_imem(thiz->_worker.getwtask(), mem);
            }
        }
    }
  else if (CHECK_HDR(hdr, OMEM, MEM_RELEASE))
    {
      al_dinfo("OMEM Release\n");
      mem = thiz->_outq.pop();
      if (mem)
        {
          mem->set_storedsize(opt->size);
          thiz->_outs[0]->push_data(mem);
          mem->release();
        }
    }
  else if (CHECK_HDR(hdr, INST, INST_INFO))
    {
      uint32_t param;
      uint16_t *hparam = (uint16_t *)&param;

      hparam[0] = opt->chs;
      hparam[1] = opt->hz / 1000;

      thiz->publish_event(AL_EVENT_MP3FRAMEINFO, param);
    }
  else if (CHECK_HDR(hdr, SYS, SYS_TERM))
    {
      al_dinfo("TERM\n");
      thiz->publish_event(AL_EVENT_MP3DECWORKEREND, 0);
      ret = -1;
    }
  else if (CHECK_HDR(hdr, SYS, SYS_PARAM))
    {
      /* Just reply of parameter */
    }
  else if (CHECK_HDR(hdr, SYS, SYS_PLAY))
    {
      /* Just reply of Start play */
    }
  else if (CHECK_HDR(hdr, SYS, SYS_BOOT))
    {
      al_dinfo("BOOT\n");
      if (hdr.type != AL_WORKER_TYPE_MP3DEC)
        {
          thiz->publish_event(AL_EVENT_MP3DEC_WRONGTYPE, 0);
          return 0;
        }

      if (hdr.opt != AL_WORKER_VERSION_0)
        {
          thiz->publish_event(AL_EVENT_MP3DEC_WRONGVER, 0);
          return 0;
        }

      alworker_send_systemparam(thiz->_worker.getwtask(),
                                thiz->channels(),
                                thiz->samplingrate(), 0);

      for (int i = 0;
           !thiz->_frame_eof && i < thiz->_inq.get_qsize();
           i++)
        {
          mem = (audiolite_memapbuf *)thiz->_pool->allocate();
          if (mem)
            {
              thiz->_stream->receive_data(mem, 0, -1);

              if (mem->is_eof())
                {
                  if (mem->get_storedsize() == 0)
                    {
                      al_derror("EOF buffer with 0 byte\n");
                      mem->set_storedsize(1);
                    }
                  thiz->_frame_eof = true;
                }

              thiz->_inq.push(mem);
              alworker_inject_imem(thiz->_worker.getwtask(), mem);
            }
        }

      for (int i = 0; i < thiz->_outq.get_qsize(); i++)
        {
          mem = (audiolite_memapbuf *)thiz->_omempool->allocate();
          if (mem)
            {
              thiz->_outq.push(mem);
              alworker_inject_omem(thiz->_worker.getwtask(), mem);
            }
        }

      alworker_send_startframe(thiz->_worker.getwtask());
      alworker_send_start(thiz->_worker.getwtask());
    }
  else if (CHECK_HDR(hdr, SYS, SYS_ERR))
    {
      al_derror("Error code : %d\n", opt->errcode);

      /* It should send AL_EVENT_MP3DECERROR but the worker
       * send error command when the stream is done, because of
       * worker issue. So now send Decode End.
       */

      thiz->publish_event(AL_EVENT_DECODEDONE,
                          (unsigned long)opt->errcode);
    }
  else
    {
      thiz->publish_event(AL_EVENT_MP3DECUNKNOWNEVT, hdr.u32);
    }
  
  return ret;
}

audiolite_mp3dec::audiolite_mp3dec() : audiolite_decoder("mp3decomem",
                                       CONFIG_ALMP3DEC_INJECTPRIO,
                                       CONFIG_ALMP3DEC_INJECTSTACK),
                  _omempool(NULL), _worker(),
                  _inq(SPRMP3_FRAMEMEM_QSIZE / 2),
                  _outq(SPRMP3_OUTMEM_QSIZE / 2), _frame_eof(false)
{
  _worker.set_msghandler(audiolite_mp3dec::handle_mesage, this);
}

audiolite_mp3dec::~audiolite_mp3dec()
{
  stop_decode();
}

int audiolite_mp3dec::start_decode()
{
  int ret = -EINVAL;

  if (_stream && _pool && _omempool)
    {
      _frame_eof = false;
      _stream->seek(0);
      _pool->enable_pool();
      _omempool->enable_pool();
      _inq.enable();
      _outq.enable();
      ret = _worker.bringup_worker(MP3WORKERBIN_NAME,
                                   MP3WORKERBIN_SPK,
                                   "mp3decsvr", CONFIG_MP3DECSVR_PRIO,
                                    CONFIG_MP3DECSVR_STACKSZ);
    }

  return ret;
}

int audiolite_mp3dec::stop_decode()
{
  if (_pool)
    {
      _pool->disable_pool();
    }
  _inq.disable();
  _worker.terminate_worker();
  _outq.disable();

  return OK;
}
