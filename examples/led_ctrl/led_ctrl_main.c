/****************************************************************************
 * examples/led_ctrl/led_ctrl_main.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sched.h>
#include <semaphore.h>

#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin assignment to LEDs */

#define PIN_LED0 PIN_I2S1_BCK
#define PIN_LED1 PIN_I2S1_LRCK
#define PIN_LED2 PIN_I2S1_DATA_IN
#define PIN_LED3 PIN_I2S1_DATA_OUT

#define SET_LEDPTN(ctl, onptn, on, offptn, off)  do {  \
    (ctl)->on_ptn   = (onptn);  \
    (ctl)->off_ptn  = (offptn); \
    (ctl)->on_time  = (on);     \
    (ctl)->off_time = (off);    \
  } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct led_ctrl_s
{
  int           on_time;    // Unit : 100ms
  int           off_time;   // Unit : 100ms
  unsigned char on_ptn;
  unsigned char off_ptn;
  sem_t         lock;
  bool          initialized;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct led_ctrl_s g_ledctrl =
{
  0, 0, 0x00, 0x00, {0}, false
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**** name: init_leds() */

static void init_leds(void)
{
  /* Set output to LED0 pin */

  board_gpio_write(PIN_LED0, -1);
  board_gpio_config(PIN_LED0, 0, false, true, PIN_FLOAT);

  /* Set output to LED1 pin */

  board_gpio_write(PIN_LED1, -1);
  board_gpio_config(PIN_LED1, 0, false, true, PIN_FLOAT);

  /* Set output to LED2 pin */

  board_gpio_write(PIN_LED2, -1);
  board_gpio_config(PIN_LED2, 0, false, true, PIN_FLOAT);

  /* Set output to LED3 pin */

  board_gpio_write(PIN_LED3, -1);
  board_gpio_config(PIN_LED3, 0, false, true, PIN_FLOAT);
}

/**** name: set_leds() */

static void set_leds(int ptn)
{
  board_gpio_write(PIN_LED0, (ptn & 0x01) ? 1 : 0);
  board_gpio_write(PIN_LED1, (ptn & 0x02) ? 1 : 0);
  board_gpio_write(PIN_LED2, (ptn & 0x04) ? 1 : 0);
  board_gpio_write(PIN_LED3, (ptn & 0x08) ? 1 : 0);
}

static int led_daemon(int argc, char *argv[])
{
  int tim = 0;
  bool on_xoff = true;

  while (1)
    {
      sem_wait(&g_ledctrl.lock);
      if (g_ledctrl.on_time < 0)
        {
          set_leds(g_ledctrl.on_ptn);
          SET_LEDPTN(&g_ledctrl, 0, 0, 0, 0);
          g_ledctrl.initialized = false;
          sem_post(&g_ledctrl.lock);
          break;
        }
      else if (g_ledctrl.on_time == 0)
        {
          /* Always ON so always reset time count */

          tim = 0;
        }
      else if (on_xoff == false && tim >= g_ledctrl.off_time)
        {
          /* To turn on LEDs */

          set_leds(g_ledctrl.on_ptn);
          on_xoff = true;
          tim = 0;
        }
      else if (on_xoff == true && tim >= g_ledctrl.on_time)
        {
          /* To turn off LEDs */

          set_leds(g_ledctrl.off_ptn);
          on_xoff = false;
          tim = 0;
        }

      sem_post(&g_ledctrl.lock);

      usleep(100 * 1000); /* Resolution of control LED is 100ms */
      tim++;
    }

  printf("Finish task\n");
  return 0;
}

static unsigned char conv_ptn(char *argptn)
{
  int i;
  int max = strlen(argptn);
  unsigned char ptn = 0;

  max = (max > 4) ? 4 : max;

  ptn = 0;
  for (i = 0; i < max; i++)
    {
      if (argptn[i] == '1')
        {
          ptn <<= 1;
          ptn |= 1;
        }
      else if (argptn[i] == '0')
        {
          ptn <<= 1;
        }
      else
        {
          break;
        }
    }

  return ptn;
}

static void print_usage(const char *cmd)
{
  printf("Usage: %s <on pattern>\n", cmd);
  printf("       %s <on pattern> <Blink time>\n", cmd);
  printf("       %s <on pattern> <Blink time> <Off pattern>\n", cmd);
  printf("       %s <on pattern> <On time> <Off pattern> <Off time>\n", cmd);
  printf("       %s help\n", cmd);
  printf(" The unit of time is 100ms, it can be set from 1 to 10\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  unsigned char onptn, offptn;
  int ontim, offtim;

  if (g_ledctrl.initialized == false)
    {
      init_leds();
      sem_init(&g_ledctrl.lock, 0, 1);
      ret = task_create("led_daemon", 100, 1024, led_daemon, NULL);
      if (ret < 0)
        {
          printf("Could not create led_daemon [err = %d]\n", errno);
          return -1;
        }

      g_ledctrl.initialized = true;
    }

  switch (argc)
    {
      case 2: /* Continuous Lighting or Terminate Daemon */
        if (!strncmp(argv[1], "help", 5)) /* Display Help */
          {
            print_usage(argv[0]);
            return -1;
          }

        onptn = conv_ptn(argv[1]);
        ontim = 0;
        offptn = 0;
        offtim = 0;
        break;
      case 3: /* Blink ON pattern */
        onptn  = conv_ptn(argv[1]);
        ontim  = atoi(argv[2]);
        ontim = (ontim > 10) ? 10 : ontim;
        offptn = 0;
        offtim = ontim;
        break;
      case 4: /* Blink ON pattern with specific Off patern */
        onptn  = conv_ptn(argv[1]);
        ontim  = atoi(argv[2]);
        ontim = (ontim > 10) ? 10 : ontim;
        offptn = conv_ptn(argv[3]);
        offtim = ontim;
        break;
      case 5: /* Blink On/Off pattern with specific time period */
        onptn  = conv_ptn(argv[1]);
        ontim  = atoi(argv[2]);
        ontim = (ontim > 10) ? 10 : ontim;
        offptn = conv_ptn(argv[3]);
        offtim = atoi(argv[4]);
        offtim = (offtim > 10) ? 10 : offtim;
        offtim = (offtim < 0)  ?  0 : offtim;
        break;
      default:
        print_usage(argv[0]);
        return -1;
    }

  sem_wait(&g_ledctrl.lock);
  SET_LEDPTN(&g_ledctrl, onptn, ontim, offptn, offtim);
  sem_post(&g_ledctrl.lock);
  if (ontim <= 0) /* Reflect value immediately */
    {
      set_leds(onptn);
    }

  return 0;
}
