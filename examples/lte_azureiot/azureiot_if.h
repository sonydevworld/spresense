/****************************************************************************
 * examples/lte_azureiot/azureiot_if.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_AZUREIOT_IF_H
#define __EXAMPLES_AZUREIOT_IF_H

/****************************************************************************
 * Public types
 ****************************************************************************/

/* Azure connection information */

struct azureiot_info
{
  const char *IoTHubName;  /**< Azure Iot Hub name    */
  const char *DeviceID;    /**< Azure Iot device ID */
  const char *PrimaryKey;  /**< Device primary key    */
};

/****************************************************************************
 * Public functions
 ****************************************************************************/

/*!
 * \brief      Get Azure hostname
 * \param[in]  info         : Information for Azure connection
 * \param[out] hostname     : Ito Hub host name
 * \param[in]  hostname_size: Buffer size
 * \retval <=0 NG
 */

int azureiot_get_hostname(struct azureiot_info *info,
                          char                 *hostname,
                          int                   hostname_size);

/*!
 * \brief      Create http command to send message to Azure
 * \param[in]  info         : Information for Azure connection
 * \param[out] request      : Command creation buffer
 * \param[in]  request_size : Buffer size
 * \param[out] hostname     : Ito Hub host name
 * \param[in]  hostname_size: Buffer size
 * \param[in]  post_message : Send message
 * \retval <=0 NG
 */

int azureiot_create_sendmsg(struct azureiot_info *info,
                            char                 *request,
                            int                   request_size,
                            const char           *post_message);

/*!
 * \brief      Create http command to receive message from Azure
 * \param[in]  info         : Information for Azure connection
 * \param[out] request      : Command creation buffer
 * \param[in]  request_size : Buffer size
 * \param[out] hostname     : Ito Hub host name
 * \param[in]  hostname_size: Buffer size
 * \retval <=0 NG
 */

int azureiot_create_recvmsg(struct azureiot_info *info,
                            char                 *request,
                            int                   request_size);

/*!
 * \brief      Create http command to delete messages received from Azure
 *             from Azure queue
 * \param[in]  info          : Information for Azure connection
 * \param[in]  io_buffer     : Received data obtained by azureiot_create_recvmsg
 *                             request
 * \param[out] io_buffer     : Command creation buffer
 * \param[in]  io_buffer_size: Buffer size
 * \param[out] hostname      : Iot Hub host name
 * \param[in]  hostname_size : Buffer size
 * \retval <=0 NG
 */

int azureiot_create_deletemsg(struct azureiot_info *info,
                              char                 *io_buffer,
                              int                   io_buffer_size);

/*!
 * \brief      Create http command to acquire file information of storage
 *             linked to Azure
 * \param[in]  info         : Information for Azure connection
 * \param[out] request      : Command creation buffer
 * \param[in]  request_size : Buffer size
 * \param[out] hostname     : Iot Hub host name
 * \param[in]  hostname_size: Buffer size
 * \param[in]  file_name: File name of storage
 * \retval <=0 NG
 */

int azureiot_create_fileinfo_msg(struct azureiot_info *info,
                                 char                 *request,
                                 int                   request_size,
                                 const char           *file_name);

/*!
 * \brief      Create http command to upload file to Azure
 * \param[in]  io_buffer       : Received data obtained by
                                 azureiot_create_fileinfo_msg request
 * \param[out] io_buffer       : Command creation buffer
 * \param[in]  io_buffer_size  : Buffer size
 * \param[out] hostname        : Strage host name
 * \param[in]  hostname_size   : Buffer size
 * \param[in]  upload_file_size: Upload file size
 * \retval <=0 NG
 */

int azureiot_create_uploadmsg(char *io_buffer,
                              int   io_buffer_size,
                              char *str_hostname,
                              int   str_hostname_size,
                              int   upload_file_size);

/*!
 * \brief      Create http command to download file from Azure
 * \param[in]  io_buffer     : Received data obtained by
                              azureiot_create_fileinfo_msg request
 * \param[out] io_buffer     : Command creation buffer
 * \param[in]  io_buffer_size: Buffer size
 * \param[out] hostname      : Strage host name
 * \param[in]  hostname_size : Buffer size
 * \retval <=0 NG
 */

int azureiot_create_downloadmsg(char *io_buffer,
                                int   io_buffer_size,
                                char *str_hostname,
                                int   str_hostname_size);

#endif /* __EXAMPLES_AZUREIOT_IF_H */
