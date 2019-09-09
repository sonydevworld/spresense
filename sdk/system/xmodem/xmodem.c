/*	
 * Copyright 2001-2019 Georges Menie (www.menie.org)
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* this code needs standard functions memcpy() and memset()
   and input/output functions _inbyte() and _outbyte().

   the prototypes of the input/output functions are:
     int _inbyte(unsigned short timeout); // msec timeout
     void _outbyte(int c);

 */

//#include "crc16.h"

#include <sdk/config.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <crc16.h>

#include "system/xmodem.h"

#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define CTRLZ 0x1A

#define DLY_1S 1000
#define MAXRETRANS 25

#define MAXBUFFSZ  1030 /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */

#define xm_timerstop(h) xm_timerstart(h,0)

#ifdef CONFIG_DEBUG_XMODEM
#  define   xmdbg(format, ...)   fprintf(stderr, format, ##__VA_ARGS__)
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define xmdbg(x...)
#  else
#    define xmdbg                (void)
#  endif
#endif

struct xmhandle_s
{
  int          fd;
  timer_t      timer;
  unsigned int flags;
};

static int xm_timerinit(struct xmhandle_s *handle)
{
	int ret;
	int errorcode;

	ret = timer_create(CLOCK_REALTIME, NULL, &handle->timer);
	if (ret < 0) {
		errorcode = errno;
		xmdbg("ERROR: Failed to create a timer: %d\n", errorcode);
		return -errorcode;
	}

	return 0;
}

static int xm_timerstart(struct xmhandle_s *handle, unsigned int msec)
{
	struct itimerspec todelay;
	int ret;
	int errorcode;

	/* Start, restart, or stop the timer */

	todelay.it_interval.tv_sec  = 0;   /* Nonrepeating */
	todelay.it_interval.tv_nsec = 0;
	todelay.it_value.tv_sec     = msec / 1000;
	todelay.it_value.tv_nsec    = (msec % 1000) * 1000 * 1000;

	ret = timer_settime(handle->timer, 0, &todelay, NULL);
	if (ret < 0) {
		errorcode = errno;
		xmdbg("ERROR: Failed to set the timer: %d\n", errorcode);
		return -errorcode;
	}

	return 0;
}

static int xm_timerrelease(struct xmhandle_s *handle)
{
	int ret;
	int errorcode;

	ret = timer_delete(handle->timer);
	if (ret < 0) {
		errorcode = errno;
		xmdbg("ERROR: Failed to delete a timer: %d\n", errorcode);
		return -errorcode;
	}

	return 0;
}

static int _inbyte(struct xmhandle_s *handle, unsigned short timeout)
{
	char c;
	ssize_t nread;
	int errorcode;

	xm_timerstart(handle, timeout);

	nread = read(handle->fd, &c, sizeof(c));

	xm_timerstop(handle);

	if (nread < 0) {
		errorcode = errno;
		if (errorcode != EINTR) {
			xmdbg("ERROR: Failed to read device: %d\n", errorcode);
		}
		return -errorcode;
	}

	return c;
}

static int _outbyte(struct xmhandle_s *handle, char data)
{
	ssize_t nwrite;
	char c = data;
	int errorcode;

	nwrite = write(handle->fd, &c, sizeof(c));
	if (nwrite < 0) {
		errorcode = errno;
		xmdbg("ERROR: Failed to write device: %d\n", errorcode);
		return -errorcode;
	}

	return nwrite;
}


static int check(int crc, const unsigned char *buf, int sz)
{
	if (crc) {
		//unsigned short crc = crc16_ccitt(buf, sz);
		unsigned short ccrc = crc16(buf, sz);
		unsigned short tcrc = (buf[sz]<<8)+buf[sz+1];
		//if (crc == tcrc)
		if (ccrc == tcrc)
			return 1;
	}
	else {
		int i;
		unsigned char cks = 0;
		for (i = 0; i < sz; ++i) {
			cks += buf[i];
		}
		if (cks == buf[sz])
		return 1;
	}

	return 0;
}

//static void flushinput(void)
static void flushinput(struct xmhandle_s *handle)
{
	//while (_inbyte(((DLY_1S)*3)>>1) >= 0)
	while (_inbyte(handle, ((DLY_1S)*3)>>1) >= 0)
		;
}

XMHANDLE xmodemHandleInit(int fd)
{
	struct xmhandle_s *handle;
	int ret;

	handle = (struct xmhandle_s*)malloc(sizeof(struct xmhandle_s));
	if (!handle) {
		xmdbg("ERROR: Failed to allocate memory\n");
		return NULL;
	}
	memset(handle, 0, sizeof(struct xmhandle_s));

	handle->fd = fd;

	ret = xm_timerinit(handle);
	if (ret < 0) {
		free(handle);
		return NULL;
	}

	return (XMHANDLE)handle;
}

int xmodemHandleRelease(XMHANDLE handle)
{
	struct xmhandle_s *hdl = handle;

	if (hdl) {
		xm_timerrelease(hdl);
		free(hdl);
	}

	return 0;
}

//int xmodemReceive(unsigned char *dest, int destsz)
int xmodemReceive(XMHANDLE handle, unsigned char *dest, int destsz)
{
//	unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	unsigned char *xbuff;
	unsigned char *p;
	int bufsz, crc = 0;
	unsigned char trychar = 'C';
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry, retrans = MAXRETRANS;
	struct xmhandle_s *hdl = handle;

	if (!hdl) {
		return XM_ERR_INVAL; /* invalid parameter */
	}
	xbuff = (unsigned char *)malloc(MAXBUFFSZ);
	if (!xbuff) {
		return XM_ERR_NOSPC; /* no space left on device */
	}
	for(;;) {
		for( retry = 0; retry < 16; ++retry) {
			//if (trychar) _outbyte(trychar);
			if (trychar) _outbyte(hdl, trychar);
			//if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
			if ((c = _inbyte(hdl, (DLY_1S)<<1)) >= 0) {
				switch (c) {
				case SOH:
					bufsz = 128;
					goto start_recv;
				case STX:
					bufsz = 1024;
					goto start_recv;
				case EOT:
					//flushinput();
					flushinput(hdl);
					//_outbyte(ACK);
					_outbyte(hdl, ACK);
					free(xbuff);
					return len; /* normal end */
				case CAN:
					//if ((c = _inbyte(DLY_1S)) == CAN) {
					if ((c = _inbyte(hdl, DLY_1S)) == CAN) {
						//flushinput();
						flushinput(hdl);
						//_outbyte(ACK);
						_outbyte(hdl, ACK);
						free(xbuff);
						//return -1; /* canceled by remote */
						return XM_ERR_CANCEL; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		if (trychar == 'C') { trychar = NAK; continue; }
		//flushinput();
		flushinput(hdl);
		//_outbyte(CAN);
		//_outbyte(CAN);
		//_outbyte(CAN);
		_outbyte(hdl, CAN);
		_outbyte(hdl, CAN);
		_outbyte(hdl, CAN);
		free(xbuff);
		//return -2; /* sync error */
		return XM_ERR_NOSYNC; /* sync error */

	start_recv:
		if (trychar == 'C') crc = 1;
		trychar = 0;
		p = xbuff;
		*p++ = c;
		for (i = 0;  i < (bufsz+(crc?1:0)+3); ++i) {
			//if ((c = _inbyte(DLY_1S)) < 0) goto reject;
			if ((c = _inbyte(hdl, DLY_1S)) < 0) goto reject;
			*p++ = c;
		}

		if (xbuff[1] == (unsigned char)(~xbuff[2]) && 
			(xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno-1) &&
			check(crc, &xbuff[3], bufsz)) {
			if (xbuff[1] == packetno)	{
				register int count = destsz - len;
				if (count > bufsz) count = bufsz;
				if (count > 0) {
					memcpy (&dest[len], &xbuff[3], count);
					len += count;
				}
				++packetno;
				retrans = MAXRETRANS+1;
			}
			if (--retrans <= 0) {
				//flushinput();
				flushinput(hdl);
				//_outbyte(CAN);
				//_outbyte(CAN);
				//_outbyte(CAN);
				_outbyte(hdl, CAN);
				_outbyte(hdl, CAN);
				_outbyte(hdl, CAN);
				free(xbuff);
				//return -3; /* too many retry error */
				return XM_ERR_RETRYOUT; /* too many retry error */
			}
			//_outbyte(ACK);
			_outbyte(hdl, ACK);
			continue;
		}
	reject:
		//flushinput();
		flushinput(hdl);
		//_outbyte(NAK);
		_outbyte(hdl, NAK);
	}
}

//int xmodemTransmit(unsigned char *src, int srcsz)
int xmodemTransmit(XMHANDLE handle, unsigned char *src, int srcsz)
{
//	unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	unsigned char *xbuff;
	int bufsz, crc = -1;
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry;
	struct xmhandle_s *hdl = handle;

	if (!hdl) {
		return XM_ERR_INVAL; /* invalid parameter */
	}
	xbuff = malloc(MAXBUFFSZ);
	if (!xbuff) {
		return XM_ERR_NOSPC; /* no space left on device */
	}

	for(;;) {
		for( retry = 0; retry < 16; ++retry) {
			//if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
			if ((c = _inbyte(hdl, (DLY_1S)<<1)) >= 0) {
				switch (c) {
				case 'C':
					crc = 1;
					goto start_trans;
				case NAK:
					crc = 0;
					goto start_trans;
				case CAN:
					//if ((c = _inbyte(DLY_1S)) == CAN) {
					if ((c = _inbyte(hdl, DLY_1S)) == CAN) {
						//_outbyte(ACK);
						_outbyte(hdl, ACK);
						//flushinput();
						flushinput(hdl);
						free(xbuff);
						//return -1; /* canceled by remote */
						return XM_ERR_CANCEL; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		//_outbyte(CAN);
		//_outbyte(CAN);
		//_outbyte(CAN);
		_outbyte(hdl, CAN);
		_outbyte(hdl, CAN);
		_outbyte(hdl, CAN);
		//flushinput();
		flushinput(hdl);
		free(xbuff);
		//return -2; /* no sync */
		return XM_ERR_NOSYNC; /* no sync */

		for(;;) {
		start_trans:
			xbuff[0] = SOH; bufsz = 128;
			xbuff[1] = packetno;
			xbuff[2] = ~packetno;
			c = srcsz - len;
			if (c > bufsz) c = bufsz;
			//if (c >= 0) {
			if (c > 0) {
				//memset (&xbuff[3], 0, bufsz);
				memset (&xbuff[3], CTRLZ, bufsz);
				//if (c == 0) {
				//	xbuff[3] = CTRLZ;
				//}
				//else {
					memcpy (&xbuff[3], &src[len], c);
				//	if (c < bufsz) xbuff[3+c] = CTRLZ;
				//}
				if (crc) {
					//unsigned short ccrc = crc16_ccitt(&xbuff[3], bufsz);
					unsigned short ccrc = crc16(&xbuff[3], bufsz);
					xbuff[bufsz+3] = (ccrc>>8) & 0xFF;
					xbuff[bufsz+4] = ccrc & 0xFF;
				}
				else {
					unsigned char ccks = 0;
					for (i = 3; i < bufsz+3; ++i) {
						ccks += xbuff[i];
					}
					xbuff[bufsz+3] = ccks;
				}
				for (retry = 0; retry < MAXRETRANS; ++retry) {
					for (i = 0; i < bufsz+4+(crc?1:0); ++i) {
						//_outbyte(xbuff[i]);
						_outbyte(hdl, xbuff[i]);
					}
					//if ((c = _inbyte(DLY_1S)) >= 0 ) {
					if ((c = _inbyte(hdl, DLY_1S)) >= 0 ) {
						switch (c) {
						case ACK:
							++packetno;
							len += bufsz;
							goto start_trans;
						case CAN:
							//if ((c = _inbyte(DLY_1S)) == CAN) {
							if ((c = _inbyte(hdl, DLY_1S)) == CAN) {
								//_outbyte(ACK);
								_outbyte(hdl, ACK);
								//flushinput();
								flushinput(hdl);
								free(xbuff);
								//return -1; /* canceled by remote */
								return XM_ERR_CANCEL; /* canceled by remote */
							}
							break;
						case NAK:
						default:
							break;
						}
					}
				}
				//_outbyte(CAN);
				//_outbyte(CAN);
				//_outbyte(CAN);
				_outbyte(hdl, CAN);
				_outbyte(hdl, CAN);
				_outbyte(hdl, CAN);
				//flushinput();
				flushinput(hdl);
				free(xbuff);
				//return -4; /* xmit error */
				return XM_ERR_XMIT; /* xmit error */
			}
			else {
				for (retry = 0; retry < 10; ++retry) {
					//_outbyte(EOT);
					_outbyte(hdl, EOT);
					//if ((c = _inbyte((DLY_1S)<<1)) == ACK) break;
					if ((c = _inbyte(hdl, (DLY_1S)<<1)) == ACK) break;
				}
				//flushinput();
				flushinput(hdl);
				free(xbuff);
				//return (c == ACK)?len:-5;
				return (c == ACK)?len:XM_ERR_XMIT_NOACK;
			}
		}
	}
}

#ifdef TEST_XMODEM_RECEIVE
int main(void)
{
	int st;

	printf ("Send data using the xmodem protocol from your terminal emulator now...\n");
	/* the following should be changed for your environment:
	   0x30000 is the download address,
	   65536 is the maximum size to be written at this address
	 */
	st = xmodemReceive((char *)0x30000, 65536);
	if (st < 0) {
		printf ("Xmodem receive error: status: %d\n", st);
	}
	else  {
		printf ("Xmodem successfully received %d bytes\n", st);
	}

	return 0;
}
#endif
#ifdef TEST_XMODEM_SEND
int main(void)
{
	int st;

	printf ("Prepare your terminal emulator to receive data now...\n");
	/* the following should be changed for your environment:
	   0x30000 is the download address,
	   12000 is the maximum size to be send from this address
	 */
	st = xmodemTransmit((char *)0x30000, 12000);
	if (st < 0) {
		printf ("Xmodem transmit error: status: %d\n", st);
	}
	else  {
		printf ("Xmodem successfully transmitted %d bytes\n", st);
	}

	return 0;
}
#endif
