#ifndef __INCLUDE_AUDIOLITE_WORKERCOMP_H
#define __INCLUDE_AUDIOLITE_WORKERCOMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/al_component.h>
#include <audiolite/al_worker.h>
#include <audiolite/al_thread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WORKERCOMP_SPKNAMESZ (32)

/****************************************************************************
 * Class Pre-definitions
 ****************************************************************************/

class audiolite_workercomp;

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class audiolite_stdworker_msglistener
{
  public:
    virtual void bootup(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                        int version, void *d)
    {
    };

    virtual void error(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                       int id, int ercode)
    {
    };

    virtual void debug(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                       int code)
    {
    };

    virtual void info(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                      int id, int chs, int fs, int layer, int rate)
    {
    };

    virtual void done(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                      int id)
    {
    };

    virtual void term(audiolite_workercomp *wcomp)
    {
    };

    virtual audiolite_memapbuf *release_inmem(audiolite_workercomp *wcomp,
                                              al_wtask_t *wtask,
                                              audiolite_memapbuf *mem,
                                              int size)
    {
      return NULL;
    };

    virtual audiolite_memapbuf *release_outmem(audiolite_workercomp *wcomp,
                                               al_wtask_t *wtask,
                                               audiolite_memapbuf *mem)
    {
      push_data(wcomp, mem);
      return NULL;
    };

    virtual void usermsg(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                         al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
    {
    };

    void push_data(audiolite_workercomp *wcomp, audiolite_memapbuf *mem);
    int get_inqsize(audiolite_workercomp *wcomp);
    int get_outqsize(audiolite_workercomp *wcomp);
    audiolite_memapbuf *allocate(audiolite_workercomp *wcomp);
};

class audiolite_workercomp : public audiolite_component, audiolite_runnable_if
{
  protected:
    char _workerspk[WORKERCOMP_SPKNAMESZ];
    audiolite_worker _worker;
    audiolite_workermemq _inq;
    audiolite_workermemq _outq;
    bool _worker_booted;
    bool _worker_terminated;
    audiolite_stdworker_msglistener *_lsnr;
    audiolite_thread _thd;

    static int handle_message(al_comm_msghdr_t hdr,
                              al_comm_msgopt_t *opt, void *arg);

    int start_worker();
    void stop_worker();

  public:
    audiolite_workercomp(const char *workername,
                         int inqsz,
                         int outqsz,
                         int inputnum = 1,
                         int outputnum = 1,
                         int depth = 16,
                         int prio = 105,
                         int stack_sz = 2048);

    virtual ~audiolite_workercomp();

    void set_msglistener(audiolite_stdworker_msglistener *lsnr)
    {
      _lsnr = lsnr;
    }

    /* Inherited methods from audiolite_component */

    virtual void on_data();
    virtual int on_starting(audiolite_inputnode *inode,
                            audiolite_outputnode *onode);
    virtual void on_canceled(audiolite_inputnode *inode,
                            audiolite_outputnode *onode);
    virtual void on_stopping(audiolite_inputnode *inode,
                             audiolite_outputnode *onode);

    /* Inherited methods from audiolite_stdworker_msglistener */

    virtual bool before_start(void *arg);
    virtual int run(void *arg);
    virtual void before_stop(void *arg);

    friend class audiolite_stdworker_msglistener;
};

#endif /* __INCLUDE_AUDIOLITE_WORKERCOMP_H */
