/****************************************************************************
 * modules/include/memutils/common_utils/common_types.h
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

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

/***********************************************************************
 *
 *      Common Environment
 *
 ***********************************************************************
 */

/*-------------------------------------------------------------------
 *      Standard Libraries
 *-------------------------------------------------------------------*/
#if (defined(__CC_ARM) && (__ARMCC_VERSION >= 5050041))
#include <stddef.h> /* for NULL */
#else
#include <sys/types.h>
#endif

/* ushort, uint is defined in <sys/types.h> */

typedef unsigned char	uchar;
typedef unsigned long	ulong;

/* Since ARMCC 5.05 doesn't have sys/types.h and ushort/uint isn't defined */

#if defined(_WIN32) || (defined(__CC_ARM) && (__ARMCC_VERSION >= 5050041))
typedef unsigned short	ushort;
typedef unsigned int	uint;
#endif

/*
 * fixed size types
 */

#include <stdint.h>

typedef uint32_t	drm_t;		/* DRM address. 0 origin byte addressing */
static const uint32_t	INVALID_DRM = 0xffffffffU;	/* Invalid DRM address */

#if (!defined(_ITRON_H_) && !defined(_POSIX))
#  ifndef TRUE
#    define TRUE  1
#  else
S_ASSERT(TRUE == 1); /* ERROR: TRUE is not 1 */
#  endif


#  ifndef FALSE
#    define FALSE  0
#  else
S_ASSERT(FALSE == 0);  /* ERROR: FALSE is not 0 */
#  endif
#endif /* _ITRON_H_  && _POSIX */

#define ON		(1)
#define OFF		(0)

#define MIN(a,b)	(((a)<(b))?(a):(b))
#define MAX(a,b)	(((a)>(b))?(a):(b))
#define ABS(x)  	(((x) < 0) ? ((~(x))+1) : (x))
#define AVE(x, y)	(((x)>>1)+((y)>>1))
#define LOW16(x)	(0xffff & (x))
#define HIGH16(x)	(0xffff & ((x)>>16))
#define CLIPPING(val, min, max) {\
				(val) = MAX((val), (min));		\
				(val) = MIN((val), (max));		\
}

#define WORD_BIT	32
#define LONG_BIT	WORD_BIT
#define HWORD_BIT	16
#define SHORT_BIT	HWORD_BIT
#define BYTE_BIT	8

#define bit_sizeof(x)	(sizeof(x)*CHAR_BIT)

/*** BITS Macro ***/
/*	width  : 0~31
	offset : 0~31
	val    : uint(type)
	spec   : Take out bits from a "val".
*/
#define BITS(width,offset,val)	((((uint)(1 << (width))-1) << (offset)) & (val))


#define BIT(n)	( (uint) 1 << (n) )
#define BIT0	BIT(0)
#define BIT1	BIT(1)
#define BIT2	BIT(2)
#define BIT3	BIT(3)
#define BIT4	BIT(4)
#define BIT5	BIT(5)
#define BIT6	BIT(6)
#define BIT7	BIT(7)
#define BIT8	BIT(8)
#define BIT9	BIT(9)
#define BIT10	BIT(10)
#define BIT11	BIT(11)
#define BIT12	BIT(12)
#define BIT13	BIT(13)
#define BIT14	BIT(14)
#define BIT15	BIT(15)
#define BIT16	BIT(16)
#define BIT17	BIT(17)
#define BIT18	BIT(18)
#define BIT19	BIT(19)
#define BIT20	BIT(20)
#define BIT21	BIT(21)
#define BIT22	BIT(22)
#define BIT23	BIT(23)
#define BIT24	BIT(24)
#define BIT25	BIT(25)
#define BIT26	BIT(26)
#define BIT27	BIT(27)
#define BIT28	BIT(28)
#define BIT29	BIT(29)
#define BIT30	BIT(30)
#define BIT31	BIT(31)

#define MASK01	0x00000001
#define MASK02	0x00000003
#define MASK03	0x00000007
#define MASK04	0x0000000f
#define MASK05	0x0000001f
#define MASK06	0x0000003f
#define MASK07	0x0000007f
#define MASK08	0x000000ff
#define MASK09	0x000001ff
#define MASK10	0x000003ff
#define MASK11	0x000007ff
#define MASK12	0x00000fff
#define MASK14	0x00003fff
#define MASK16	0x0000ffff
#define MASK32	0xffffffff

#define ALL32_0	0x00000000
#define ALL32_1	0xffffffff

#define RSHIFT(x, y)	( (x) >> (y) )
#define LSHIFT(x, y)	( (x) << (y) )

/***********************************************************************
 * Add Static Assert
 ***********************************************************************
 */

#define STATIC_ASSERT(e) \
	do { char static_assert[(e)? 1:-1]; (void)static_assert[0]; } while (0)

/***********************************************************************
 *
 *      I/O Read, Write Macros
 *
 ***********************************************************************
 */
#ifdef ON_UNIX

#include "dummy_io.h"
#define IO_OUT(x,y)			dummy_io_out((uint)&(x), (uint)(y))
#define IO_IN(x,y) 			dummy_io_in((uint)&(x),(uint*)&(y))
#define WAIT_READY(x)
#define TWAIT_READY(x,t,r)
#define IO_REG_OUT(x, y)		dummy_io_out((uint)(x), (uint)(y))
#define IO_REG_IN(x, y) 		dummy_io_in((uint)(x),(uint*)&(y))
#define WAIT_REG_READY(x,y)
#define TWAIT_REG_READY(x,y,t,r)
#define TWAIT_REG_BUSY_END(x,y,t,r)
#define IS_REG_READY(x,y)       	(1)
#define IO_REG_SWAP_IN(x, y)		{ dummy_io_in((uint)(x), (uint*)&(y)); \
				(y) = ((((uint)(y) & 0xFF000000) >> 24) |				\
					   (((uint)(y) & 0x00FF0000) >> 8) |				\
					   (((uint)(y) & 0x0000FF00) << 8) |				\
					   (((uint)(y) & 0x000000FF) << 24)); }
#define IO_REG_SWAP_OUT(x, y) 		{ dummy_io_out((uint)(x), ((((uint)(y) & 0xFF000000) >> 24) | \
															   (((uint)(y) & 0x00FF0000) >> 8) | \
															   (((uint)(y) & 0x0000FF00) << 8) | \
															   (((uint)(y) & 0x000000FF) << 24))); }
#define IO_REG16_OUT(x, y)		dummy_io16_out((uint)(x), (ushort)(y))
#define IO_REG16_IN(x, y) 		dummy_io16_in((uint)(x),(ushort*)&(y))
#define DUMMY_IO_INIT_MAP(x, y)		dummy_io_init_map((const io_value*)(x), (uint)(y))
#define DUMMY_IO_MODIFY_MAP(x, y)	dummy_io_modify_map((uint)(x), (uint)(y))
#define DUMMY_IO_READ_MAP(x, y)		dummy_io_read_map((uint)(x), (uint*)&(y));

#elif defined (ISS_ALLEGRO)

#include "dummy_io.h"
#define IO_OUT(x,y)			dummy_io_out((uint)&(x), (uint)(y))
#define IO_IN(x,y) 			dummy_io_in((uint)&(x),(uint*)&(y))
#define WAIT_READY(x)
#define TWAIT_READY(x,t,r)
#define IO_REG_OUT(x, y)		dummy_io_out((uint)(x), (uint)(y))
#define IO_REG_IN(x, y) 		dummy_io_in((uint)(x),(uint*)&(y))
#define WAIT_REG_READY(x,y)
#define TWAIT_REG_READY(x,y,t,r)
#define TWAIT_REG_BUSY_END(x,y,t,r)
#define IS_REG_READY(x,y)       	(1)
#define IO_REG_SWAP_IN(x, y)		{ dummy_io_in((uint)(x), (uint*)&(y)); \
				(y) = ((((uint)(y) & 0xFF000000) >> 24) |				\
					   (((uint)(y) & 0x00FF0000) >> 8) |				\
					   (((uint)(y) & 0x0000FF00) << 8) |				\
					   (((uint)(y) & 0x000000FF) << 24)); }
#define IO_REG_SWAP_OUT(x, y) 		{ dummy_io_out((uint)(x), ((((uint)(y) & 0xFF000000) >> 24) | \
															   (((uint)(y) & 0x00FF0000) >> 8) | \
															   (((uint)(y) & 0x0000FF00) << 8) | \
															   (((uint)(y) & 0x000000FF) << 24))); }
#define IO_REG16_OUT(x, y)		dummy_io16_out((uint)(x), (ushort)(y))
#define IO_REG16_IN(x, y) 		dummy_io16_in((uint)(x),(ushort*)&(y))
#define DUMMY_IO_INIT_MAP(x, y)		dummy_io_init_map((const io_value*)(x), (uint)(y))
#define DUMMY_IO_MODIFY_MAP(x, y)	dummy_io_modify_map((uint)(x), (uint)(y))
#define DUMMY_IO_READ_MAP(x, y)		dummy_io_read_map((uint)(x), (uint*)&(y));

#else
#ifdef allegro
#include "allegro_regs.h"
#endif /* allegro */

#define IO_OUT(x,y)			( (x) = (y) ) /* OUT = write */
#define IO_IN( x,y)			( (y) = (x) ) /* IN  = read */
#define WAIT_READY(x)			{ while(!(x)); }
#define TWAIT_READY(x,t,r)		{ int _i_; (r) = FALSE; STATIC_ASSERT((t) < 20); \
				for(_i_ = 0; _i_ < (t); _i_++){ if(x){(r) = TRUE; break;}};}
#define IO_REG_OUT(x, y)		( *((volatile uint *)(x)) = (y) )
#define IO_REG_IN(x, y) 		( (y) = *((volatile uint *)(x)) )
#define WAIT_REG_READY(x,y)		{ while(!(*((volatile uint *)(x))&(y))); }
#define TWAIT_REG_READY(x,y,t,r)	{ int _i_; (r) = FALSE; STATIC_ASSERT((t) < 20); \
				for(_i_ = 0; _i_ < (t); _i_++){ if(*((volatile uint *)(x))&(y)){(r) = TRUE; break;}};}
#define TWAIT_REG_BUSY_END(x,y,t,r)	{ int _i_; (r) = FALSE; STATIC_ASSERT((t) < 20); \
				for(_i_ = 0; _i_ < (t); _i_++){ if(!(*((volatile uint *)(x))&(y))){(r) = TRUE; break;}};}
#define IS_REG_READY(x,y)       	(*((volatile uint *)(x))&(y))
#define IO_REG_SWAP_IN(x, y)		{ __wsbw__(y,*((const volatile uint *)(x))); }
#define IO_REG_SWAP_OUT(x, y) 		{ __wsbw__(*((volatile uint *)(x)),(y)); }
#define IO_REG16_OUT(x, y)		( *((volatile ushort *)(x)) = (y) )
#define IO_REG16_IN(x, y) 		( (y) = *((volatile ushort *)(x)) )
#define DUMMY_IO_INIT_MAP(x, y)
#define DUMMY_IO_MODIFY_MAP(x, y)
#define DUMMY_IO_READ_MAP(x, y)

#endif /* ON_UNIX */

/***********************************************************************
 *
 *      Address conversion
 *
 ***********************************************************************
 */
#define ADL2BYTE(x)	((x)<<3)
#define BYTE2ADL(x)	((x)>>3)

#endif /* COMMON_TYPES_H */

