/****************************************************************************
 * system/include/system/xmodem.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#ifndef __APPS_INCLUDE_SYSTEM_XMODEM_H
#define __APPS_INCLUDE_SYSTEM_XMODEM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XM_ERR_CANCEL      (-1) /* canceld by remote */
#define XM_ERR_NOSYNC      (-2) /* no sync */
#define XM_ERR_RETRYOUT    (-3) /* too many retry error */
#define XM_ERR_XMIT        (-4) /* xmit error */
#define XM_ERR_XMIT_NOACK  (-5) /* no ack xmit error */
#define XM_ERR_INVAL       (-6) /* invalid parameter */
#define XM_ERR_NOSPC       (-7) /* no space left on device */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Opaque handles returned by initialization functions */

typedef void *XMHANDLE;

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmodemHandleInit
 *
 * Description:
 *   Initialize the xmodem handle.
 *
 * Input Parameters:
 *   fd - The file descriptor of communication device.
 *
 * Returned Value:
 *   If successful, returns a non-NULL value.
 *   Otherwise it will return a NULL value.
 *
 ****************************************************************************/

XMHANDLE xmodemHandleInit(int fd);

/****************************************************************************
 * Name: xmodemHandleRelease
 *
 * Description:
 *   Release the xmodem handle.
 *
 * Input Parameters:
 *   handle - The handle to use xmodem.
 *
 * Returned Value:
 *   If successful, the zero will be returned.
 *   Otherwise it will return a negative value.
 *
 ****************************************************************************/

int xmodemHandleRelease(XMHANDLE handle);

/****************************************************************************
 * Name: xmodemReceive
 *
 * Description:
 *   Receive date sent from the remote peer.
 *
 * Input Parameters:
 *   handle  - The handle to use xmodem.
 *   dest    - The destination buffer address to receive.
 *   destsz  - The size of destination buffer address.
 *
 * Returned Value:
 *   If successful, the received data size will be returned.
 *   Otherwise it will return a negative value.
 *
 ****************************************************************************/

int xmodemReceive(XMHANDLE handle, unsigned char *dest, int destsz);

/****************************************************************************
 * Name: xmodemTransmit
 *
 * Description:
 *   Send date to the remote peer.
 *
 * Input Parameters:
 *   handle  - The handle to use xmodem.
 *   src     - The source buffer address to send.
 *   srcsz   - The size of source buffer address.
 *
 * Returned Value:
 *   If successful, the sent data size will be returned.
 *   Otherwise it will return a negative value.
 *
 ****************************************************************************/

int xmodemTransmit(XMHANDLE handle, unsigned char *src, int srcsz);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __APPS_INCLUDE_SYSTEM_XMODEM_H */
