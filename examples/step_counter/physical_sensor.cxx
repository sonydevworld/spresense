/****************************************************************************
 * step_counter/physical_sensor.cxx
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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <arch/chip/scu.h>

#include "physical_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MQ_REQ_MAX_NUM  1
#define MQ_RES_MAX_NUM  1

/* For task */

#define TASK_ARG_MAX_DEC_ADDR_SIZE 11
#define TASK_ARG_NUM               2
#define TASK_PRIORITY              110
#define TASK_STACK_SIZE            1024

/* For error */

#define err(format, ...)    fprintf(stderr, format, ##__VA_ARGS__)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

inline bool physical_sensor_send(
  mqd_t mq_request_des,
  FAR PhysicalSensorClass::physical_sensor_request_t *msg)
{
  while (mq_send(mq_request_des,
                 (FAR const char*)msg,
                 sizeof(PhysicalSensorClass::physical_sensor_request_t), 0) < 0)
    {
      if (errno == EINTR)
        {
          /* Interrupted signal. retry */

          continue;
        }
      err("mq_send() failure. errno %d\n", errno);
      return false;
    }
  return true;
}

/*--------------------------------------------------------------------------*/
inline bool physical_sensor_receive(
  mqd_t mq_response_des,
  FAR PhysicalSensorClass::physical_sensor_response_t *msg)
{
  while (mq_receive(mq_response_des,
                    (FAR char*)msg,
                    sizeof(PhysicalSensorClass::physical_sensor_response_t), 0) < 0)
    {
      if (errno == EINTR)
        {
          /* Interrupted signal. retry */

          continue;
        }
      err("mq_receive() failure. errno %d\n", errno);
      return false;
    }
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR physical_sensor_t *PhysicalSensorCreate(
  pysical_event_handler_t handler,
  FAR void *entry_function,
  FAR const char *dev_name)
{
  /* Open request message queue. */

  struct mq_attr att_info;
  att_info.mq_maxmsg  = MQ_REQ_MAX_NUM;
  att_info.mq_msgsize =
    sizeof(PhysicalSensorClass::physical_sensor_request_t);
  att_info.mq_flags   = 0;

  char mq_name_request[PHYSICAL_SENSOR_MAX_MQ_NAME];
  snprintf(mq_name_request,
           PHYSICAL_SENSOR_MAX_MQ_NAME,
           "%s_mq_req",
           dev_name);

  mqd_t mq_request = mq_open(mq_name_request, 
                             (O_RDWR|O_CREAT|O_NONBLOCK),
                             0666,
                             &att_info);
  if (mq_request < 0)
    {
      err("PhysicalSensorCreate() is failure."
          " Failed by mq_open(request). errno = %d\n", errno);
      return (physical_sensor_t *)NULL;
    }

  /* Open response message queue. */

  att_info.mq_maxmsg  = MQ_RES_MAX_NUM;
  att_info.mq_msgsize =
    sizeof(PhysicalSensorClass::physical_sensor_response_t);
  att_info.mq_flags   = 0;

  char mq_name_response[PHYSICAL_SENSOR_MAX_MQ_NAME];
  snprintf(mq_name_response,
           PHYSICAL_SENSOR_MAX_MQ_NAME,
           "%s_mq_res",
           dev_name);

  mqd_t mq_response = mq_open(mq_name_response,
                              (O_RDWR|O_CREAT),
                              0666,
                              &att_info);
  if (mq_response < 0)
    {
      err("PhysicalSensorCreate() is failure."
          " Failed by mq_open(response). errno = %d\n", errno);
      mq_close(mq_request);
      mq_unlink(mq_name_request);
      return (physical_sensor_t *)NULL;
    }

  /* Allocate information area for sensor control by instance. */

  physical_sensor_t *sensor = (physical_sensor_t *)malloc(sizeof(physical_sensor_t));
  memset(reinterpret_cast<void *>(sensor), 0, sizeof(physical_sensor_t));

  /* Set control information. */

  sensor->mq_request_des  = mq_request;
  snprintf(sensor->mq_request_name,
           PHYSICAL_SENSOR_MAX_MQ_NAME,
           "%s",
            mq_name_request);
  sensor->mq_response_des = mq_response;
  snprintf(sensor->mq_response_name,
           PHYSICAL_SENSOR_MAX_MQ_NAME,
           "%s",
            mq_name_response);
  sensor->handler   = handler;
  sensor->thread_id = NULL_TASK_PROCESS_ID;

  /* Start physical sensor process. */

  pthread_attr_t attr;
  struct sched_param sch_param;
  pthread_t thread_id;

  pthread_attr_init(&attr);
  sch_param.sched_priority = TASK_PRIORITY;
  attr.stacksize           = TASK_STACK_SIZE;
  pthread_attr_setschedparam(&attr, &sch_param);

  int ret = pthread_create(&thread_id,
                           &attr,
                           (pthread_startroutine_t)entry_function,
                           (pthread_addr_t)sensor);
  if (ret < 0)
    {
      err("PhysicalSensorCreate() is failure. Failed by pthread_create().\n");
      goto errout_create_task;
    }

  /* Wait response. */

  PhysicalSensorClass::physical_sensor_response_t response_msg;
  if (!physical_sensor_receive(sensor->mq_response_des, &response_msg))
    {
      err("PhysicalSensorCreate() is failure. Failed by receiving process.\n");
      goto errout_receive_response;
    }

  /* Error check. */

  if (response_msg.msg_type != PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_CREATE)
    {
      /* Fatal error occured. */

      err("PhysicalSensorCreate() is failure. Message type is illegal.\n");
      goto errout_receive_response;
    }

  if (response_msg.result < 0)
    {
      err("PhysicalSensorCreate() is failure. result = %d\n",
          response_msg.result);
      goto errout_receive_response;
    }

  return sensor;

errout_receive_response:
  pthread_cancel(thread_id);
  pthread_join(thread_id, NULL);

errout_create_task:
  mq_close(mq_response);
  mq_close(mq_request);
  mq_unlink(mq_name_request);
  mq_unlink(mq_name_response);

  return (physical_sensor_t *)NULL;
}

/*--------------------------------------------------------------------------*/
int PhysicalSensorOpen(FAR physical_sensor_t *sensor, FAR void *param)
{
  if (sensor == NULL)
    {
      return PHYSICAL_SENSOR_ERR_CODE_PARAM;
    }

  /* Send open request. */

  PhysicalSensorClass::physical_sensor_request_t request_msg;
  request_msg.msg_type = PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_OPEN;
  request_msg.param    = param;

  if (!physical_sensor_send(sensor->mq_request_des, &request_msg))
    {
      err("PhysicalSensorOpen() is failure. Failed by sending process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_SEND;
    }

  /* Wait response. */

  PhysicalSensorClass::physical_sensor_response_t response_msg;
  if (!physical_sensor_receive(sensor->mq_response_des, &response_msg))
    {
      err("PhysicalSensorOpen() is failure. Failed by receiving process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_RECEIVE;
    }

  /* Error check. */

  if (response_msg.msg_type != PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_OPEN)
    {
      /* Fatal error occured. */

      err("PhysicalSensorOpen() is failure. Message type is illegal.\n");
      return PHYSICAL_SENSOR_ERR_CODE_FATAL;
    }

  if (response_msg.result < 0)
    {
      err("PhysicalSensorOpen() is failure. result = %d\n",
          response_msg.result);
      return PHYSICAL_SENSOR_ERR_CODE_IOCTL;
    }

  return PHYSICAL_SENSOR_ERR_CODE_OK;
}

/*--------------------------------------------------------------------------*/
int PhysicalSensorStart(FAR physical_sensor_t *sensor)
{
  if (sensor == NULL)
    {
      return PHYSICAL_SENSOR_ERR_CODE_PARAM;
    }

  /* Send start request. */

  PhysicalSensorClass::physical_sensor_request_t request_msg;
  request_msg.msg_type = PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_START;

  if (!physical_sensor_send(sensor->mq_request_des, &request_msg))
    {
      err("PhysicalSensorStart() is failure. Failed by sending process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_SEND;
    }

  /* Wait response. */

  PhysicalSensorClass::physical_sensor_response_t response_msg;
  if (!physical_sensor_receive(sensor->mq_response_des, &response_msg))
    {
      err("PhysicalSensorStart() is failure. Failed by receiving process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_RECEIVE;
    }

  /* Error check. */

  if (response_msg.msg_type != PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_START)
    {
      /* Fatal error occured. */

      err("PhysicalSensorStart() is failure. Message type is illegal.\n");
      return PHYSICAL_SENSOR_ERR_CODE_FATAL;
    }

  if (response_msg.result < 0)
    {
      err("PhysicalSensorStart() is failure. result = %d\n",
          response_msg.result);
      return PHYSICAL_SENSOR_ERR_CODE_IOCTL;
    }

  return PHYSICAL_SENSOR_ERR_CODE_OK;
}

/*--------------------------------------------------------------------------*/
int PhysicalSensorStop(FAR physical_sensor_t *sensor)
{
  if (sensor == NULL)
    {
      return PHYSICAL_SENSOR_ERR_CODE_PARAM;
    }

  /* Send stop request. */

  PhysicalSensorClass::physical_sensor_request_t request_msg;
  request_msg.msg_type = PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_STOP;
  if (!physical_sensor_send(sensor->mq_request_des, &request_msg))
    {
      err("PhysicalSensorStop() is failure. Failed by sending process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_SEND;
    }

  /* Wait response. */

  PhysicalSensorClass::physical_sensor_response_t response_msg;
  if (!physical_sensor_receive(sensor->mq_response_des, &response_msg))
    {
      err("PhysicalSensorStop() is failure. Failed by receiving process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_RECEIVE;
    }

  /* Error check. */

  if (response_msg.msg_type != PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_STOP)
    {
      /* Fatal error occured. */

      err("PhysicalSensorStop() is failure. Message type is illegal.\n");
      return PHYSICAL_SENSOR_ERR_CODE_FATAL;
    }

  if (response_msg.result < 0)
    {
      err("PhysicalSensorStop() is failure. result = %d\n",
          response_msg.result);
      return PHYSICAL_SENSOR_ERR_CODE_IOCTL;
    }

  return PHYSICAL_SENSOR_ERR_CODE_OK;
}

/*--------------------------------------------------------------------------*/
int PhysicalSensorClose(FAR physical_sensor_t *sensor)
{
  if (sensor == NULL)
    {
      return PHYSICAL_SENSOR_ERR_CODE_PARAM;
    }

  /* Send close request. */

  PhysicalSensorClass::physical_sensor_request_t request_msg;
  request_msg.msg_type = PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_CLOSE;
  if (!physical_sensor_send(sensor->mq_request_des, &request_msg))
    {
      err("PhysicalSensorClose() is failure. Failed by sending process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_SEND;
    }

  /* Wait response. */

  PhysicalSensorClass::physical_sensor_response_t response_msg;
  if (!physical_sensor_receive(sensor->mq_response_des, &response_msg))
    {
      err("PhysicalSensorClose() is failure. Failed by receiving process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_RECEIVE;
    }

  /* Error check. */

  if (response_msg.msg_type != PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_CLOSE)
    {
      /* Fatal error occured. */

      err("PhysicalSensorClose() is failure. Message type is illegal.\n");
      return PHYSICAL_SENSOR_ERR_CODE_FATAL;
    }

  if (response_msg.result < 0)
    {
      err("PhysicalSensorClose() is failure. result = %d\n",
          response_msg.result);
      return PHYSICAL_SENSOR_ERR_CODE_IOCTL;
    }

  return PHYSICAL_SENSOR_ERR_CODE_OK;
}

/*--------------------------------------------------------------------------*/
int PhysicalSensorDestroy(FAR physical_sensor_t *sensor)
{
  if (sensor == NULL)
    {
      return PHYSICAL_SENSOR_ERR_CODE_PARAM;
    }

  /* Send destroy request. */

  PhysicalSensorClass::physical_sensor_request_t request_msg;
  request_msg.msg_type = PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_DESTROY;
  if (!physical_sensor_send(sensor->mq_request_des, &request_msg))
    {
      err("PhysicalSensorDestroy() is failure. Failed by sending process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_SEND;
    }

  /* Wait response. */

  PhysicalSensorClass::physical_sensor_response_t response_msg;
  if (!physical_sensor_receive(sensor->mq_response_des, &response_msg))
    {
      err("PhysicalSensorDestroy() is failure. Failed by receiving process.\n");
      return PHYSICAL_SENSOR_ERR_CODE_RECEIVE;
    }

  if (response_msg.msg_type != PhysicalSensorClass::PHYSICAL_SENSOR_MSG_TYPE_DESTROY)
    {
      /* Fatal error occured. */

      err("PhysicalSensorDestroy() is failure. Message type is illegal.\n");
      return PHYSICAL_SENSOR_ERR_CODE_FATAL;
    }

  if (response_msg.result < 0)
    {
      err("PhysicalSensorDestroy() is failure. result = %d\n",
          response_msg.result);
      return PHYSICAL_SENSOR_ERR_CODE_IOCTL;
    }

  /* Wait finish of sensor process. */

  if (sensor->thread_id != NULL_TASK_PROCESS_ID)
    {
      FAR void *thread_return;
      pthread_cancel(sensor->thread_id);
      pthread_join(sensor->thread_id, &thread_return);

      /* Check return code. */

      int ret = *(reinterpret_cast<int *>(thread_return));
      if (ret != PHYSICAL_SENSOR_ERR_CODE_OK)
        {
          return ret;
        }
    }

  /* Close message queue. */

  mq_close(sensor->mq_request_des);
  mq_close(sensor->mq_response_des);
  mq_unlink(sensor->mq_request_name);
  mq_unlink(sensor->mq_request_name);

  /* Free information of sensor. */

  free(sensor);

 return PHYSICAL_SENSOR_ERR_CODE_OK;
}

/****************************************************************************
 * PhysicalSensorClass
 ****************************************************************************/

void PhysicalSensorClass::create()
{
  /* Setup signal for receiving data. */

  sigemptyset(&m_sigset);

  /* Update status to "Idle". */

  m_status = PHYSICAL_SENSOR_STATE_IDLE;

  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = PHYSICAL_SENSOR_MSG_TYPE_CREATE;
  response_msg.result   = PHYSICAL_SENSOR_ERR_CODE_OK;
  if (!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::add_signal(const int sig_no)
{
  if (m_status == PHYSICAL_SENSOR_STATE_END)
    {
      return;
    }
  sigaddset(&m_sigset, sig_no);
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::delete_signal(const int sig_no)
{
  if (m_status == PHYSICAL_SENSOR_STATE_END)
    {
      return;
    }
  sigdelset(&m_sigset, sig_no);
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::run()
{
  int sig_no;
  struct siginfo sig_info;
  struct timespec timeout;
  physical_sensor_request_t msg;

  /* Set the latency of the signal from the SCU.
   * The SCU sends a signal every second,
   * but it wishes to receive the event, so it is 500 ms.
   */

  timeout.tv_sec  = 0;
  timeout.tv_nsec = 500000000;

  while(m_status != PHYSICAL_SENSOR_STATE_END)
    {
      /* Check signal from SCU. */

      sig_no = sigtimedwait(&m_sigset, &sig_info, &timeout);
      if (sig_no >= 0)
        {
          if (m_status == PHYSICAL_SENSOR_STATE_RUNNING)
            {
              receive_signal(sig_no, &sig_info);
            }
        }

      /* Check request from application. */

      bool is_request = receive(msg);
      if (is_request)
        {
          parse(msg);
        }
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::msg_open(physical_sensor_request_t &msg)
{
  /* Open device driver of physical sensor. */

  int ret = open_sensor();
  if (ret < 0)
    {
      err("open_sensor() is failure. error %d\n", ret);
      goto send_open_response;
    }

  /* Setup physical sensor. */

  ret = setup_sensor(msg.param);
  if (ret < 0)
    {
      close_sensor();
      goto send_open_response;
    }

  /* Setup sensor control unit. */

  ret = setup_scu(msg.param);
  if (ret < 0)
    {
      close_sensor();
      goto send_open_response;
    }

  /* Update status to "Ready". */

  m_status = PHYSICAL_SENSOR_STATE_READY;

send_open_response:

  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = msg.msg_type;
  response_msg.result   = ret;
  if (!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::msg_start(physical_sensor_request_t &msg)
{
  /* Start sequencer. */

  int ret = start_sensor();
  if (ret < 0)
    {
      err("start_sensor() is failure. error %d\n", ret);
      goto send_start_response;
    }

  /* Update status to "Run"*/

  m_status = PHYSICAL_SENSOR_STATE_RUNNING;

send_start_response:

  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = msg.msg_type;
  response_msg.result   = ret;
  if (!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::msg_stop(physical_sensor_request_t &msg)
{
  int ret = stop_sensor();
  if (ret < 0)
    {
      err("stop_sensor() is failure. error = %d\n", ret);
      goto send_stop_response;
    }

  /* Update status to "Ready"*/

  m_status = PHYSICAL_SENSOR_STATE_READY;

send_stop_response:

  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = msg.msg_type;
  response_msg.result   = ret;
  if (!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::msg_close(physical_sensor_request_t &msg)
{
  /* Close device driver of physical sensor. */

  int ret = close_sensor();
  if (ret < 0)
    {
      err("close_sensor() is failure. error %d\n", ret);
      goto send_destroy_response;
    }

  /* Update status to "Idle". */

  m_status = PHYSICAL_SENSOR_STATE_IDLE;

send_destroy_response:

  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = msg.msg_type;
  response_msg.result   = ret;
  if (!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::msg_destroy(physical_sensor_request_t &msg)
{
  /* Update status to "End". */

  m_status = PHYSICAL_SENSOR_STATE_END;

  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = msg.msg_type;
  response_msg.result   = PHYSICAL_SENSOR_ERR_CODE_OK;
  if (!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
void PhysicalSensorClass::msg_illegal(physical_sensor_request_t &msg)
{
  /* Send response. */

  physical_sensor_response_t response_msg;
  response_msg.msg_type = msg.msg_type;
  response_msg.result   = -1;

  if(!send(response_msg))
    {
      err("PhysicalSensorClass::send() is failure.\n");
    }
}

/*--------------------------------------------------------------------------*/
PhysicalSensorClass::msg_process PhysicalSensorClass::msg_process_table[PHYSICAL_SENSOR_MSG_TYPE_NUM][PHYSICAL_SENSOR_STATE_NUM] =
{
  /* Message type: create. */

  {                                    /* status:  */
    &PhysicalSensorClass::msg_illegal, /*   Idle   */
    &PhysicalSensorClass::msg_illegal, /*   Ready  */
    &PhysicalSensorClass::msg_illegal, /*   Run    */
    &PhysicalSensorClass::msg_illegal, /*   End    */
  },

  /* Message type: open. */

  {                                    /* status:  */
    &PhysicalSensorClass::msg_open,    /*   Idle   */
    &PhysicalSensorClass::msg_illegal, /*   Ready  */
    &PhysicalSensorClass::msg_illegal, /*   Run    */
    &PhysicalSensorClass::msg_illegal, /*   End    */
  },

  /* Message type: start. */

  {                                    /* status:  */
    &PhysicalSensorClass::msg_illegal, /*   Idle   */
    &PhysicalSensorClass::msg_start,   /*   Ready  */
    &PhysicalSensorClass::msg_illegal, /*   Run    */
    &PhysicalSensorClass::msg_illegal, /*   End    */
  },

  /* Message type: stop. */

  {                                    /* status:  */
    &PhysicalSensorClass::msg_illegal, /*   Idle   */
    &PhysicalSensorClass::msg_illegal, /*   Ready  */
    &PhysicalSensorClass::msg_stop,    /*   Run    */
    &PhysicalSensorClass::msg_illegal, /*   End    */
  },

  /* Message type: close. */

  {                                    /* status:  */
    &PhysicalSensorClass::msg_illegal, /*   Idle   */
    &PhysicalSensorClass::msg_close,   /*   Ready  */
    &PhysicalSensorClass::msg_illegal, /*   Run    */
    &PhysicalSensorClass::msg_illegal, /*   End    */
  },

  /* Message type: destroy. */

  {                                    /* status:  */
    &PhysicalSensorClass::msg_destroy, /*   Idle   */
    &PhysicalSensorClass::msg_illegal, /*   Ready  */
    &PhysicalSensorClass::msg_illegal, /*   Run    */
    &PhysicalSensorClass::msg_illegal, /*   End    */
  }
};

void PhysicalSensorClass::parse(physical_sensor_request_t &msg)
{
  (this->*msg_process_table[msg.msg_type][m_status])(msg);
}
