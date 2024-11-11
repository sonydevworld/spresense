/****************************************************************************
 * modules/audiolite/src/base/al_decoder.cxx
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

#include <audiolite/al_debug.h>
#include <audiolite/al_decoder.h>

/****************************************************************************
 * Class: audiolite_decoder
 ****************************************************************************/

audiolite_decoder::audiolite_decoder(const char *name,
                                     int prio, int stack_sz)
                 : audiolite_component(0, 1), _stream(NULL),
                   _omempool(NULL), _prio(prio), _stacksz(stack_sz),
                   _tid(-1), _tname(name), _isplay(false), _ispause(false),
                   _is_thrdrun(false)
{
}

audiolite_decoder::~audiolite_decoder()
{
  stop_thread();
}

int audiolite_decoder::start_thread(const char *name)
{
  int ret;

  _is_thrdrun = true;
  if (_prio >= 0 && _stacksz >= 0)
    {
      ret = mossfw_create_thread_attr(&_tid,
                                      audiolite_decoder::inject_worker,
                                      this,
                                      _prio, _stacksz);
    }
  else
    {
      ret = mossfw_create_thread(&_tid,
                                 audiolite_decoder::inject_worker,
                                 this);
    }

  if (ret == 0)
    {
      pthread_setname_np(_tid, _tname);
    }
  else
    {
      _tid = -1;
      _is_thrdrun = false;
    }

  return ret;
}

void audiolite_decoder::stop_thread()
{
  if (_tid >= 0)
    {
      _is_thrdrun = false;
      if (_pool != NULL)
        {
          _pool->disable_pool();
        }

      if (_omempool)
        {
          _omempool->disable_pool();
        }

      mossfw_thread_join(&_tid);
      _tid = -1;
    }
}

int audiolite_decoder::start()
{
  int ret = -EAGAIN;

  al_ddebug("Entry\n");
  if (!_isplay && _pool)
    {
      ret = audiolite_start(_outs[0]);
      al_ddebug("result alstart: %d\n", ret);
      if (ret == OK)
        {
          _pool->enable_pool();
          ret = start_thread(_tname);
          al_ddebug("result start thread: %d\n", ret);
          if (ret == 0)
            {
              ret = start_decode();
              al_ddebug("result start decode: %d\n", ret);
              if (ret != 0)
                {
                  audiolite_stop(_outs[0]);
                }
              else
                {
                  _isplay = true;
                }
            }
          else
            {
              audiolite_stop(_outs[0]);
            }
        }
    }

  al_ddebug("Leave\n");
  return ret;
}

void audiolite_decoder::stop()
{
  if (_tid >= 0 && _isplay)
    {
      stop_thread();
      audiolite_stop(_outs[0]);
      _isplay  = false;
      _ispause = false;
      stop_decode();
    }
}

void audiolite_decoder::pause()
{
  int ret;

  if (_tid >= 0 && _isplay)
    {
      ret = pause_decode();
      if (ret == 0)
        {
          _ispause = true;
        }
    }
}

int audiolite_decoder::resume()
{
  int ret = -EAGAIN;

  if (_tid >= 0 && _isplay && _ispause)
    {
      ret = resume_decode();
      if (ret == 0)
        {
          _ispause = false;
        }
    }

  return ret;
}

void *audiolite_decoder::inject_worker(void *arg)
{
  audiolite_decoder *dec = (audiolite_decoder *)arg;
  dec->decode_runner();
  return NULL;
}
