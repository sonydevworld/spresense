#include <string.h>
#include <audiolite/al_thread.h>

audiolite_thread::audiolite_thread(audiolite_runnable_if *runnable,
                                   void *runnable_arg,
                                   int prio, int stacksz,
                                   const char *thdname)
  : _runnable(runnable), _runnable_arg(runnable_arg),
    _tid(-1), _prio(prio), _stacksz(stacksz), _is_thrdrun(false)
{
  if (thdname)
    {
      strncpy(_tname, thdname, AL_THREAD_NAMEMAX - 1);
    }
  else
    {
      _tname[0] = '\0';
    }
}

audiolite_thread::~audiolite_thread()
{
  stop();
}

bool audiolite_thread::set_runnable(audiolite_runnable_if *runnable,
                                    void *runnable_arg)
{
  if (!_is_thrdrun)
    {
      _runnable = runnable;
      _runnable_arg = runnable_arg;
      return true;
    }

  return false;
}

bool audiolite_thread::start()
{
  int ret = false;
  if (!_is_thrdrun && _runnable)
    {
      _is_thrdrun = true;
      ret = mossfw_create_thread_attr(&_tid,
                                      audiolite_thread::running_thread,
                                      this,
                                      _prio, _stacksz);
      if (ret == 0)
        {
          ret = true;
          if (_tname[0])
            {
              pthread_setname_np(_tid, _tname);
            }
        }
      else
        {
          _tid = -1;
          _is_thrdrun = false;
        }
    }

  return ret;
}

void audiolite_thread::stop()
{
  if (_is_thrdrun)
    {
      _is_thrdrun = false;
      _runnable->before_stop(_runnable_arg);
      mossfw_thread_join(&_tid);
      _tid = -1;
    }
}

void *audiolite_thread::running_thread(void *arg)
{
  int ret;
  audiolite_thread *thiz = (audiolite_thread *)arg;

  while (thiz->_is_thrdrun &&
         !thiz->_runnable->before_start(thiz->_runnable_arg))
    {
      usleep(10 * 1000); /* Wait and yeild */
    }

  ret = 1;
  while (thiz->_is_thrdrun && ret)
    {
      ret = thiz->_runnable->run(thiz->_runnable_arg);
    }

  return NULL;
}
