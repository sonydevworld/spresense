/****************************************************************************
 * modules/sensing/transport_mode/tram_state_transition.cpp
 *
 *   Copyright 2018,2021 Sony Semiconductor Solutions Corporation
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
#include <debug.h>

#include "tram_state_transition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Tramsport mode debug feature. */

#ifdef CONFIG_SENSING_TRAM_DEBUG_ERROR
#  define tram_err(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define tram_err(fmt, ...)
#endif
#ifdef CONFIG_SENSING_TRAM_DEBUG_INFO
#  define tram_info(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define tram_info(fmt, ...)
#endif

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

static int changeStateIlligal(FAR TramClass *owner);
static int changeStateUninitToMs(FAR TramClass *owner);
static int changeStateMSToCMD(FAR TramClass *owner);
static int changeStateCMDToMS(FAR TramClass *owner);
static int changeStateCMDToTMI(FAR TramClass *owner);
static int changeStateTMIToCMD(FAR TramClass *owner);
static int changeStateToUninit(FAR TramClass *owner);

typedef int (*changeState)(FAR TramClass *owner);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SENSING_TRAM_DEBUG_INFO
static const char* s_state_strings[] =
{
  "UNINITIALIZED",
  "MS",
  "CMD",
  "TMI",
};
#endif

static changeState s_state_table[TRAM_STATE_NUM][TRAM_STATE_NUM] =
{
/* UNINITIALIZED */

  {                        /* Curretn State:  */
    changeStateIlligal,    /*   UNINITIALIZED */
    changeStateToUninit,   /*   MS            */
    changeStateToUninit,   /*   CMD           */
    changeStateToUninit    /*   TMI           */
  },

/* MS */

  {                        /* Curretn State:  */
    changeStateUninitToMs, /*   UNINITIALIZED */
    changeStateIlligal,    /*   MS            */
    changeStateCMDToMS,    /*   CMD           */
    changeStateIlligal     /*   TMI           */
  },

/* CMD */

  {                        /* Curretn State:  */
    changeStateIlligal,    /*   UNINITIALIZED */
    changeStateMSToCMD,    /*   MS            */
    changeStateIlligal,    /*   CMD           */
    changeStateTMIToCMD    /*   TMI           */
  },

/* TMI */

  {                        /* Curretn State:  */
    changeStateIlligal,    /*   UNINITIALIZED */
    changeStateIlligal,    /*   MS            */
    changeStateCMDToTMI,   /*   CMD           */
    changeStateIlligal     /*   TMI           */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int changeStateIlligal(FAR TramClass *owner)
{
  return -1;
}

/*------------------------------------------------------------*/
static int changeStateUninitToMs(FAR TramClass *owner)
{
  int ret = 0;

  owner->set_state(TRAM_STATE_MS);
  owner->set_power(0x01 << accelID);

  return ret;
}

/*------------------------------------------------------------*/
static int changeStateToUninit(FAR TramClass *owner)
{
  int ret = 0;

  owner->set_state(TRAM_STATE_UNINITIALIZED);
  owner->clear_power((0x01 << accelID) |
                     (0x01 << magID) |
                     (0x01 << barometerID));

  return ret;
}

/*------------------------------------------------------------*/
static int changeStateMSToCMD(FAR TramClass *owner)
{
  int ret = 0;

  owner->set_state(TRAM_STATE_CMD);
  owner->send_notification(ChangeScuSettings);

  return ret;
}

/*------------------------------------------------------------*/
static int changeStateCMDToMS(FAR TramClass *owner)
{
  int ret = 0;

  owner->set_state(TRAM_STATE_MS);
  owner->send_notification(ChangeScuSettings);

  return ret;
}

/*------------------------------------------------------------*/
static int changeStateCMDToTMI(FAR TramClass *owner)
{
  int ret = 0;

  owner->set_state(TRAM_STATE_TMI);
  owner->set_power((0x01 << magID) | (0x01 << barometerID));

  return ret;
}

/*------------------------------------------------------------*/
static int changeStateTMIToCMD(FAR TramClass *owner)
{
  int ret = 0;

  owner->set_state(TRAM_STATE_CMD);
  owner->clear_power((0x01 << magID) | (0x01 << barometerID));

  return ret;
}

/****************************************************************************
 * External Interface
 ****************************************************************************/

int TramStateTransitionSetState(FAR TramClass *owner, tram_state_e state)
{
  int ret;

  tram_info("current state: %d, next state: %d\n", owner->get_state(), state);

  ret = (*s_state_table[state][owner->get_state()])(owner);
  if (ret == 0)
    {
      if (state > TRAM_STATE_UNINITIALIZED)
        {
          tram_info("State: %s\n", s_state_strings[state]);
        }
    }
  else
    {
      tram_err("TRAM invalid state: %d, current: %d\n",
               state,
               owner->get_state());
    }

  return ret;
}
