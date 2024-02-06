/*****************************************************************************
 * examples/simple_audio_through/simple_audio_through_main.c
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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
 *****************************************************************************/

/* Summary of this Application:
 * This sample application indicates how to use audio device driver directory.
 * It works by throughing audio data from mic to a speaker.
 *
 *           --------------       --------------
 *           |            |       |            |
 *   MIC --->| MIC device |----->>| SPK device |------> Speaker
 *           |   driver   |       |   driver   |
 *           |            |       |            |
 *           --------------       --------------
 */

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <nuttx/audio/audio.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define CHANNEL_NUM (2)
#define SAMPLE_RATE (48000)
#define BITWIDTH    (16)

#define NUM_APB (8)
#define SZ_APB (4096)

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* Audio buffers */

static struct ap_buffer_s apbs[NUM_APB];  /* Audio buffer containers */
static uint8_t buff[NUM_APB][SZ_APB];     /* Actual memory for audio data */

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/* init_apbs() : Initialize Audio Buffer */

static void init_apbs(void)
{
  int i;
  for (i = 0; i < NUM_APB; i++)
    {
      apbs[i].nmaxbytes = SZ_APB;   /* Buffer memory size */
      apbs[i].nbytes = 0;           /* Actual memory size which is stored */
      apbs[i].curbyte = 0;          /* Current position to next use */
      apbs[i].samp = &buff[i][0];   /* Buffer memory address */
      nxmutex_init(&apbs[i].lock);  /* Initialize the lock */
    }
}

/* open_devfile() : Open Audio device file */

static int open_devfile(FAR const char *devpath)
{
  return open(devpath, O_RDWR | O_CLOEXEC);
}

/* create_messageq() : Create message queue to get notify from the driver */

static mqd_t create_messageq(FAR const char *mqname)
{
  mqd_t mq;
  struct mq_attr attr;

  attr.mq_maxmsg = 12;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags = 0;
  mq = mq_open(mqname, O_RDWR | O_CREAT, 0644, &attr);

  return mq;
}

/* register_messageq() : Set the message queue to the audio driver */

static int register_messageq(int fd, mqd_t mq)
{
  return ioctl(fd, AUDIOIOC_REGISTERMQ, (unsigned long)mq);
}

/* configure() : Set-up the audio configuration samplerate,
 *               channel num and bit width.
 */

static int configure(int fd, int type, int chnum, int fs, int bps)
{
  struct audio_caps_desc_s cap;

  cap.caps.ac_len = sizeof(struct audio_caps_s);
  cap.caps.ac_type = type;
  cap.caps.ac_channels = chnum;
  cap.caps.ac_chmap = 0;
  cap.caps.ac_controls.hw[0] = fs & 0xffff;
  cap.caps.ac_controls.b[2] = bps;
  cap.caps.ac_controls.b[3] = (fs >> 16) & 0xff;

  return ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)(uintptr_t)&cap);
}

/* set_volgain() : Set output volume / mic gain */

static int set_volgain(int fd, bool is_vol, int volgain)
{
  struct audio_caps_desc_s cap_desc;

  cap_desc.caps.ac_len            = sizeof(struct audio_caps_s);
  cap_desc.caps.ac_type           = AUDIO_TYPE_FEATURE;
  cap_desc.caps.ac_format.hw      = is_vol ? AUDIO_FU_VOLUME :
                                             AUDIO_FU_INP_GAIN;
  cap_desc.caps.ac_controls.hw[0] = volgain;

  return ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)(uintptr_t)&cap_desc);
}

/* enqueue_buffer() : Enqueue a audio buffer */

static int enqueue_buffer(int fd, FAR struct ap_buffer_s *apb)
{
  struct audio_buf_desc_s desc;

  desc.numbytes = apb->nbytes;
  desc.u.buffer = apb;

  return ioctl(fd, AUDIOIOC_ENQUEUEBUFFER,
                  (unsigned long)(uintptr_t)&desc);
}

/* start_dma(): Start DMA from/to MEMORY. */

static int start_dma(int fd)
{
  return ioctl(fd, AUDIOIOC_START, 0);
}

/* stop_dma(): Stop a started DMA. */

static int stop_dma(int fd)
{
  return ioctl(fd, AUDIOIOC_STOP, 0);
}

/* cleanup_messageq(): Clean up message queue to empty. */

static void cleanup_messageq(mqd_t mq)
{
  int qnum = 0;
  struct audio_msg_s msg;
  struct mq_attr attr;

  if (!mq_getattr(mq, &attr))
    {
      qnum = (int)attr.mq_curmsgs;
    }

  while (qnum--)
    {
      mq_receive(mq, (FAR char *)&msg, sizeof(msg), NULL);
    }
}

/* handle_msg(): Message handle from a driver via message queue. */

static int handle_msg(mqd_t mq, int curfd, int nextfd, bool is_mic)
{
  int ret = 0;
  ssize_t size;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;

  /* Receiving message from a audio driver */

  size = mq_receive(mq, (FAR char *)&msg, sizeof(msg), NULL);

  if (size == sizeof(msg)) /* Size check, just in case */
    {
      switch (msg.msg_id)
        {
          case AUDIO_MSG_DEQUEUE:

            /* Audio data buffer is de-queued from a driver.
             * In input case, the buffer contains PCM data
             * from the mic. In output device case, the buffer
             * contains data which is finished playing.
             */

            /* Received buffer is set into next driver
             * From mic to spk, from spk (used memory) to mic.
             */

            apb = (FAR struct ap_buffer_s *)msg.u.ptr;

            if (is_mic) /* If it is mic, show first 4 samples */
              {
                int16_t *data = (FAR int16_t *)apb->samp;
                printf("0: %7d, %7d, %7d, %7d\n",
                       data[0], data[1], data[2], data[3]);
              }

            enqueue_buffer(nextfd, apb);
            break;

          case AUDIO_MSG_COMPLETE:

            /* Stopped DMA */

            ret = -1;
            break;

          case AUDIO_MSG_UNDERRUN:

            /* Underrun is happened on the driver */

            ret = -2;
            break;

          case AUDIO_MSG_IOERROR:

            /* Any driver error is happened */

            ret = -3;
            break;
        }
    }

  return ret;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int main(int argc, char **argv)
{
  int ret;
  int i;

  int micfd   = -1; /* MIC device file descriptor */
  int spkfd   = -1; /* Speaker device file descriptor */
  int stdfd   = -1; /* file descriptor of standard input(keyboard) */
  mqd_t micmq = -1; /* Message queue to communicate mic device */
  mqd_t spkmq = -1; /* Message queue to communicate speaker device */

  /* For polling message from devices: mic, speaker and standard input */

  struct pollfd fds[3];

  /* Initialize Audio buffers */

  init_apbs();

  /* Open audio device files */

  micfd = open_devfile("/dev/audio/pcm_in0");
  spkfd = open_devfile("/dev/audio/pcm0");
  if (micfd < 0 || spkfd < 0)
    {
      printf("Error to open device file\n");
      goto end_app;
    }

  /* Open message queue for each audio devices.
   * Message queue path should be just unique.
   */

  micmq = create_messageq("/tmp/micmq");
  spkmq = create_messageq("/tmp/spkmq");
  if (micmq < 0 || spkmq < 0)
    {
      printf("Error to create message queue\n");
      goto end_app;
    }

  /* Make sure the message queue empty */

  cleanup_messageq(micmq);
  cleanup_messageq(spkmq);

  /* Set the message queue to each drivers */

  register_messageq(micfd, micmq);
  register_messageq(spkfd, spkmq);

  /* Configure audio devices */

  configure(micfd, AUDIO_TYPE_INPUT,  CHANNEL_NUM, SAMPLE_RATE, BITWIDTH);
  configure(spkfd, AUDIO_TYPE_OUTPUT, CHANNEL_NUM, SAMPLE_RATE, BITWIDTH);

  /* Set volume to maximum (max is 1000) */

  set_volgain(micfd, false, 1000);
  set_volgain(spkfd, true,  1000);

  /* Get standard input file descriptor to exit this app */

  stdfd = fileno(stdin);

  /* Set all audio buffers to mic device at first */

  for (i = 0; i < NUM_APB; i++)
    {
      enqueue_buffer(micfd, &apbs[i]);
    }

  /* Start DMA to start playing */

  start_dma(micfd);
  start_dma(spkfd);

  /* Main loop for handling message from devices */

  while (true)
    {
      fds[0].fd     = stdfd;  /* Set wait target : standard input */
      fds[0].events = POLLIN; /* Set for what it is waiting for (receiving) */
      fds[1].fd     = micmq;  /* Set wait target : mic device */
      fds[1].events = POLLIN; /* Set for what it is waiting for (receiving) */
      fds[2].fd     = spkmq;  /* Set wait target : speaker device */
      fds[2].events = POLLIN; /* Set for what it is waiting for (receiving) */

      /* Wait for a device which is ready to receive */

      ret = poll(fds, 3, -1);

      /* Check if the standard input is ready to read data from keyboard */

      if (fds[0].revents & POLLIN)
        {
          /* Get received charactor and if it is the key 'q', exit the loop */

          ret = getchar();
          if (ret == 'q')
            {
              break;
            }
        }

      /* Check if the mic device has a message */

      if (fds[1].revents & POLLIN)
        {
          /* Handle a message from the MIC device */

          ret = handle_msg(micmq, micfd, spkfd, true);
          if (ret < 0)  /* If any error is happened */
            {
              break;  /* Exit the loop */
            }
        }

      /* Check if the speaker device has a message */

      if (fds[2].revents & POLLIN)
        {
          handle_msg(spkmq, spkfd, micfd, false);
          if (ret < 0)  /* If any error is happened */
            {
              break;  /* Exit the loop */
            }
        }
    }

  stop_dma(micfd);
  stop_dma(spkfd);

end_app:

  if (micfd >= 0) close(micfd);
  if (spkfd >= 0) close(spkfd);
  if (micmq >= 0) mq_close(micmq);
  if (spkmq >= 0) mq_close(spkmq);

  /* Delete Message queue files */

  mq_unlink("/tmp/micmq");
  mq_unlink("/tmp/spkmq");

  return OK;
}
