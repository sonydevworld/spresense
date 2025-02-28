#ifndef __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_CONFIG_H
#define __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_CONFIG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** Definition of NOTUSE_XXXXXX
 *
 * If you want to handle messages below,
 * comment out to enable it.
 */

#define NOTUSE_STARTING
#define NOTUSE_STOPPING
#define NOTUSE_SYSPAUSE
#define NOTUSE_INSTGAIN
/* #define NOTUSE_ORGMSG */
#define NOTUSE_SYSDBG

/** Definition of Memory block QUEUE size
 *
 * Input side memory block size will defined as CONF_WORKER_IMEMMAX.
 * Output side memory block size will defined as CONF_WORKER_OMEMMAX.
 */

#define CONF_WORKER_IMEMMAX (8)
#define CONF_WORKER_OMEMMAX (8)

#endif /* __AUDIOLITE_WORKER_COMMON_ALWORKER_COMMFW_CONFIG_H */
