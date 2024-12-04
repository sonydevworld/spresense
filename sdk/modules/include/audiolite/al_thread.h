#ifndef __INCLUDE_AUDIOLITE_THREAD_H
#define __INCLUDE_AUDIOLITE_THREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <mossfw/mossfw_component.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AL_THREAD_NAMEMAX (32)

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class audiolite_runnable_if
{
  public:

    /* Override before_start() method if there is any conditions before
     * start run the thread.
     * Retrun true if all conditions are good enough,
     * return false if it is not.
     */

    virtual bool before_start(void *arg)
    {
      return true;
    };

    /* runnable returns 0 when it wants to finish the thread loop.
     * Returning  othre value makes keep continue the loop.
     */

    virtual int run(void *arg) = 0;

    /* Override before_stop() method to do some before thread is stopped. */

    virtual void before_stop(void *arg) {};
};

class audiolite_thread
{
  protected:
    audiolite_runnable_if *_runnable;
    void *_runnable_arg;

    mossfw_thread_t _tid;
    int _prio;
    int _stacksz;
    char _tname[AL_THREAD_NAMEMAX];
    volatile bool _is_thrdrun;

    static void *running_thread(void *arg);

  public:
    audiolite_thread(audiolite_runnable_if *runnable, void *runnable_arg,
                     int prio, int stacksz, const char *thdname=NULL);

    ~audiolite_thread();

    bool set_runnable(audiolite_runnable_if *runnable, void *runnable_arg);
    bool start();
    void stop();

    bool is_running() { return _is_thrdrun; };
};

#endif // __INCLUDE_AUDIOLITE_THREAD_H
