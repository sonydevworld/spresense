/****************************************************************************
 * prime/prime_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/drivers/ramdisk.h>

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <asmp/asmp.h>
#include <asmp/mpmq.h>
#include <asmp/mpmutex.h>
#include <asmp/mpshm.h>
#include <asmp/mptask.h>

#include "shared.h"

#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include <sys/time.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_ROMFS
#include "worker/romfs.h"

#define SECTORSIZE 512
#define NSECTORS(b) (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define MOUNTPT "/romfs"
#endif

#ifndef MOUNTPT
#define MOUNTPT "/mnt/sd0/BIN"
#endif

#define MSG_ID_SAYHELLO 1

/* The receive timeout in milliseconds. */
#define RCV_TIMEOUT_MS 50

#if CONFIG_NFILE_DESCRIPTORS < 1
#error                                                                         \
    "You must provide file descriptors via CONFIG_NFILE_DESCRIPTORS in your configuration file"
#endif

#define message(format, ...) printf(format, ##__VA_ARGS__)
#define err(format, ...) fprintf(stderr, format, ##__VA_ARGS__)

#define LED0 PIN_I2S1_BCK
#define LED1 PIN_I2S1_LRCK
#define LED2 PIN_I2S1_DATA_IN
#define LED3 PIN_I2S1_DATA_OUT

#define ON true
#define OFF false

#define N_LEDS 4
static int leds[N_LEDS] = {LED1, LED2, LED0, LED3};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char fullpath[128];
static int cores = N_CPU;
static int nrange = PRIME_LIMIT;
/****************************************************************************
 * Symbols from Auto-Generated Code
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void led(int led, bool state) { board_gpio_write(led, state); }

static void led_init(int pin) {

    board_gpio_write(pin, -1);
    board_gpio_config(pin, 0, 0, true, PIN_FLOAT);
    led(pin, OFF);
}

/* Just an LED progress bar. */
static void led_progress(void) {
    static int ledx = 0;
    static bool turn_on = true;

    if (ledx < N_LEDS) {
	if (turn_on) {
	    led(leds[ledx], ON);
	} else {
	    led(leds[ledx], OFF);
	}
	ledx++;
    } else {
	if (turn_on) {
	    turn_on = false;
	} else {
	    turn_on = true;
	}
	ledx = 0;
    }
}

static inline void time_measure_start(struct timeval *start_time) {
    gettimeofday(start_time, NULL);
}

/* Return diff in ms. */
static inline uint32_t time_measure_end(struct timeval *start_time) {
    struct timeval end_time, result;
    gettimeofday(&end_time, NULL);
    timersub(&end_time, start_time, &result);
    message("\n================================"
	    "\nTime consumed: %lds %ldus\n"
	    "================================\n",
	    result.tv_sec, result.tv_usec);
    return (result.tv_sec * 1000) + (result.tv_usec / 1000);
}

#include "shared.c"

static uint32_t run_singlecore_prime(void) {

    uint32_t primes_found;
    uint32_t start;
    uint32_t n, time2;
    int i;
    struct timeval start_time;
    message("\nRunning same exercise on a single core:\n");

    time_measure_start(&start_time);
    for (i = 0; i < cores; i++) {

	start = i * nrange;
	n = nrange;
	message("[%d] prime calc start % 6d length %d - ", i, start, n);
	primes_found = find_primes(start, n + start);
	printf("found %d primes\n", primes_found);
	led_progress();
    }
    time2 = time_measure_end(&start_time);
    led_progress();
    return time2;
}

static int run_multicore_prime(const char *filename) {
    mptask_t mptask[N_CPU];
    mpmutex_t mutex;
    mpshm_t shm;
    mpmq_t mq[N_CPU];
    uint32_t msgdata;
    int ret, wret;
    char *buf;
    shared_data_t *shared;
    int i;
    struct timeval start_time;
    uint32_t time1;

    /* Initialize MP mutex and bind it to MP task */

    ret = mpmutex_init(&mutex, KEY_MUTEX);
    if (ret < 0) {
	err("mpmutex_init() failure. %d\n", ret);
	return ret;
    }

    ret = mpshm_init(&shm, KEY_SHM, 1024);
    if (ret < 0) {
	err("mpshm_init() failure. %d\n", ret);
	return ret;
    }

    /* Map shared memory to virtual space */

    buf = mpshm_attach(&shm, 0);
    if (!buf) {
	err("mpshm_attach() failure.\n");
	return ret;
    }
    shared = (shared_data_t *)buf;
    message("Shared mem attached @ %08x\n", (uintptr_t)buf);
    memset(buf, 0, 1024);
    memset(shared, 0, sizeof(*shared));
    message("\n==== Starting multicore ===============\n");

    for (i = 0; i < cores; i++) {
	/* Initialize MP task */

	ret = mptask_init(&mptask[i], filename);
	if (ret != 0) {
	    err("mptask_init() failure. %d\n", ret);
	    return ret;
	}

	ret = mptask_assign(&mptask[i]);
	if (ret != 0) {
	    err("mptask_asign() failure. %d\n", ret);
	    return ret;
	}

	message("[%d] Init task, CPU id: %d - ", i,
		mptask_getcpuid(&mptask[i]));

	ret = mptask_bindobj(&mptask[i], &mutex);
	if (ret < 0) {
	    err("mptask[0]_bindobj(mutex) failure. %d\n", ret);
	    return ret;
	}

	/* Initialize MP message queue with assigned CPU ID, and bind it to MP
	 * task
	 */

	ret = mpmq_init(&mq[i], KEY_MQ, mptask_getcpuid(&mptask[i]));
	if (ret < 0) {
	    err("mpmq_init() failure. %d\n", ret);
	    return ret;
	}
	ret = mptask_bindobj(&mptask[i], &mq[i]);
	if (ret < 0) {
	    err("mptask_bindobj(mq) failure. %d\n", ret);
	    return ret;
	}

	/* Initialize MP shared memory and bind it to MP task */
	ret = mptask_bindobj(&mptask[i], &shm);
	if (ret < 0) {
	    err("mptask_binobj(shm) failure. %d\n", ret);
	    return ret;
	}

	/* Run worker */
	message("task started\n");
	ret = mptask_exec(&mptask[i]);
	if (ret < 0) {
	    err("mptask_exec() failure. %d\n", ret);
	    return ret;
	}
	led_progress();
    }
    message("\nSending messages:\n");

    time_measure_start(&start_time);
    for (i = 0; i < cores; i++) {
	/* Send command to worker */
	shared->prime[i].start = i * nrange;
	shared->prime[i].n = nrange;

	message("[%d] prime calc range start % 6d lenght %d\n", i,
		shared->prime[i].start, shared->prime[i].n);

	/* This message will notify each instance of the worker that
	 it can start to process its task.
	 The message will contain a variable i to notify each instance what
	 index it
	 should use in the shared data array.
	 */
	ret = mpmq_send(&mq[i], MSG_ID_SAYHELLO, i);
	if (ret < 0) {
	    err("mpmq_send() failure. %d\n", ret);
	    return ret;
	}
	led_progress();
    }

    message("\n==== Receiving response ============\n");
    int done = 0;
    do {
	for (i = 0; i < cores; i++) {

	    /* Wait for response from each worker, but since we don't know what
	     order
	     the responses will come in.
	     We wait with a timeout and iterate through all message queues
	     untill all
	     workers have responded.
	     */

	    ret = mpmq_timedreceive(&mq[i], &msgdata, RCV_TIMEOUT_MS);

	    if (ret == -ETIMEDOUT) {
		message(".");
		led_progress();
		continue;
	    } else if (ret < 0) {
		err("mpmq_recieve() failure. %d\n", ret);
		return ret;
	    } else {
		message("\n");
		led_progress();
		done++;
	    }

	    mpmutex_lock(&mutex);
	    printf("[%d] found %d primes\n", i, shared->prime[i].primes_found);
	    mpmutex_unlock(&mutex);

	    /* Lock mutex for synchronize with worker after it's started */
	}
    } while (done < cores);

    time1 = time_measure_end(&start_time);

    /* Destroy workers */

    for (i = 0; i < cores; i++) {
	wret = -1;
	ret = mptask_destroy(&mptask[i], false, &wret);
	led_progress();
	if (ret < 0) {
	    err("mptask_destroy() failure. %d\n", ret);
	    return ret;
	}
	/* Finalize all of MP objects */
    }

    mpshm_detach(&shm);
    mpshm_destroy(&shm);
    mpmutex_destroy(&mutex);
    /* Destroy message queue. */
    for (i = 0; i < cores; i++) {
	mpmq_destroy(&mq[i]);
	led_progress();
    }

    return time1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mount_romfs(void) {
#ifdef CONFIG_FS_ROMFS
    int ret = 0;
    struct stat buf;

    ret = stat(MOUNTPT, &buf);
    if (ret < 0) {
	message("Registering romdisk at /dev/ram0\n");
	ret = romdisk_register(0, (FAR uint8_t *)romfs_img,
			       NSECTORS(romfs_img_len), SECTORSIZE);
	if (ret < 0) {
	    err("ERROR: romdisk_register failed: %d\n", ret);
	    exit(1);
	}

	message("Mounting ROMFS filesystem at target=%s with source=%s\n",
		MOUNTPT, "/dev/ram0");

	ret = mount("/dev/ram0", MOUNTPT, "romfs", MS_RDONLY, NULL);
	if (ret < 0) {
	    err("ERROR: mount(%s,%s,romfs) failed: %s\n", "/dev/ram0", MOUNTPT,
		errno);
	}
    }
#endif
    return ret;
}

/****************************************************************************
 * Name: prime_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
    int ret = 0;
    ret = mount_romfs();
    uint32_t time1, time2;

    led_init(LED0);
    led_init(LED1);
    led_init(LED2);
    led_init(LED3);

    if (argc > 1) {
	cores = atoi(argv[1]);
	if (cores == 0 || cores > N_CPU) {
	    cores = N_CPU;
	}
	if (argc > 2) {
	    nrange = atoi(argv[2]);
	}
    }
    snprintf(fullpath, 128, "%s/%s", MOUNTPT, "hello");
    message("Will use %d cores.\n", cores);

    time1 = run_multicore_prime(fullpath);
    time2 = run_singlecore_prime();
    message(
	"Multicore (%dCPUs) execution time \t%dms\nSinglecore execution time "
	"\t\t%dms\nRatio "
	"\t\t\t\t\t%d%%\n",
	cores, time1, time2, (100 * time2) / time1);
    return ret;
}
