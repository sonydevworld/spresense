/****************************************************************************
 * modules/memutils/simple_fifo/src/CMN_SimpleFifo.c
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

#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

static inline void __DMB(void) { asm volatile ("dmb"); }
static inline void __DSB(void) { asm volatile ("dsb"); }

#include "memutils/simple_fifo/CMN_SimpleFifo.h"

#if 0
#if !defined(__DSB)
#warning disabling DSB
/*!
 *  @brief Dummy Data Sync Barrier instruction.
 *
 *  It is enabled only if not defined.
 */
#define	__DSB()		do {} while (0)
#endif
#if !defined(__DMB)
#warning disabling DMB
/*!
 *  @brief Dummy Data Memory Barrier instruction.
 *  
 *  It is enabled only if not defined.
 */
#define	__DMB()		do {} while (0)
#endif
#endif

/*!
 *  @brief Index based PeekHandle for internal use.
 */
typedef struct {
    size_t m_newRp; //!< new RP to remove the data.
    size_t m_idxChunk[2]; //!< Buffer index to the region.
    size_t m_szChunk[2];  //!< Data size of the region.
} PeekHandleInternal;

/*!
 * @brief Calculate occupied size.
 */
static inline size_t getOccupiedSize(size_t sz, size_t wp, size_t rp) {
    size_t szOccupied = 0;
    if (wp == rp) {
        szOccupied = 0;
    } else if (wp < rp) {
        szOccupied = wp + sz - rp;
    } else {
        szOccupied = wp - rp;
    }
    return szOccupied;
}

/*!
 * @brief Calculate vacant size. Split region is allowed.
 */
static inline size_t getVacantSize(size_t sz, size_t wp, size_t rp) {
    size_t szVacant = 0;
    if (wp == rp) {
        szVacant = sz - 1;
    } else if (wp < rp) {
        szVacant = rp - wp - 1;
    } else {
        szVacant = sz - (wp - rp) - 1;
    }
    return szVacant;
}

/*!
 * @brief Calculate CONTINUOUS vacant size. Split region is NOT allowed.
 */
static inline size_t getVacantSizeContinuous(size_t sz, size_t wp, size_t rp) {
    size_t szVacant = 0;
    if (wp == rp) {
        szVacant = sz - wp;
		if (rp == 0) {
            --szVacant;
        }
    } else if (wp < rp) {
        szVacant = rp - wp - 1;
    } else {
		szVacant = sz - wp;
		if (rp == 0) {
            --szVacant;
        }
    }
    return szVacant;
}

/*!
 * @brief Internal peek implementation.
 */
static size_t peek(
        const CMN_SimpleFifoHandle* pHandle0,
        PeekHandleInternal* pPeekHandle,
        size_t sz) {
    assert(pHandle0 != NULL);
    assert(pPeekHandle != NULL);
    
    volatile const CMN_SimpleFifoHandle* pHandle = pHandle0;
    const size_t rp = pHandle->m_rp;
    const size_t wp = pHandle->m_wp;
    const size_t bufsz = pHandle->m_size;

    pPeekHandle->m_idxChunk[0] = pPeekHandle->m_idxChunk[1] = 0;
    pPeekHandle->m_szChunk[0] = pPeekHandle->m_szChunk[1] = 0;
    pPeekHandle->m_newRp = pHandle->m_rp;
    if (getOccupiedSize(bufsz, wp, rp) < sz) {
        // no enough data in the queue
        return 0;
    }
    if (rp <= wp) {
        pPeekHandle->m_idxChunk[0] = rp;
        pPeekHandle->m_szChunk[0] = sz;
        pPeekHandle->m_newRp = rp + sz;
        if (bufsz <= pPeekHandle->m_newRp) {
            pPeekHandle->m_newRp = 0;
        }
    } else {
        size_t szRegion1= bufsz - rp;
        if (sz < szRegion1) {
            szRegion1 = sz;
        }
        pPeekHandle->m_idxChunk[0] = rp;
        pPeekHandle->m_szChunk[0] = szRegion1;
        pPeekHandle->m_newRp = rp + szRegion1;
        if (bufsz <= pPeekHandle->m_newRp) {
            pPeekHandle->m_newRp = 0;
        }
        if (szRegion1 < sz) {
            pPeekHandle->m_idxChunk[1] = 0;
            pPeekHandle->m_szChunk[1] = sz - szRegion1;
            pPeekHandle->m_newRp = sz - szRegion1;
            if (bufsz <= pPeekHandle->m_newRp) {
                pPeekHandle->m_newRp = 0;
            }
        }
    }
    return sz;
}

/*!
 * @brief Alignment size.
 */
#define WORD_SIZE (4)

/*!
 * @brief Macro to check if the address is aligned to the WORD_SIZE
 *
 * @param[in] addr Address to check. 
 */
#define NOT_ALIGNED(addr) ((uintptr_t)(addr) & (WORD_SIZE - 1))

static void* defaultCopier(
        void* pCopierFuncExtInfo,
        void* dst0,
        const void* src0,
        size_t len) {
    memcpy(dst0, src0, len);
    return dst0;
}

int CMN_SimpleFifoInitialize(
        CMN_SimpleFifoHandle* pHandle0,
        void* pFifoBuffer,
        size_t szFifoBuffer,
        void* pExtInfo) {
    volatile CMN_SimpleFifoHandle* pHandle = pHandle0;
    if (pHandle == NULL
        || NOT_ALIGNED(pHandle)
        || pFifoBuffer == NULL
        || NOT_ALIGNED(pFifoBuffer)
        || szFifoBuffer <= 1) {
        return -1;
    }
    pHandle->m_pBuf = (uint8_t*)pFifoBuffer;
    pHandle->m_size = szFifoBuffer;
    pHandle->m_pExtInfo = pExtInfo;
    pHandle->m_wp = pHandle->m_rp = 0;
    __DSB();
    return 0;
}

size_t CMN_SimpleFifoOffer(
        CMN_SimpleFifoHandle* pHandle,
        const void* pElement,
        size_t sz) {
    return CMN_SimpleFifoOfferWithSpecificCopier(
            pHandle,
            pElement,
            sz,
            defaultCopier,
            NULL);
}

size_t CMN_SimpleFifoOfferWithSpecificCopier(
        CMN_SimpleFifoHandle* pHandle0,
        const void* pElement,
        size_t sz,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo) {
    assert(pHandle0 != NULL);
    assert(pElement != NULL);
    assert(copierFunc != NULL);

    volatile CMN_SimpleFifoHandle* pHandle = pHandle0;
    const size_t rp = pHandle->m_rp;
    const size_t wp = pHandle->m_wp;
    const size_t bufsz = pHandle->m_size;
    size_t newWp = wp;

    if (getVacantSize(bufsz, wp, rp) < sz) {
        // no enough space
        return 0;
    }

    if (wp < rp) {
        (*copierFunc)(pCopierFuncExtInfo, &pHandle->m_pBuf[wp], pElement, sz);
        newWp = wp + sz;
    } else {
        size_t szRegion1= bufsz - wp;
        if (sz < szRegion1) {
            szRegion1 = sz;
        }
        (*copierFunc)(pCopierFuncExtInfo, &pHandle->m_pBuf[wp], &(((uint8_t*)pElement)[0]), szRegion1);
		newWp = wp + szRegion1;
		assert(newWp <= bufsz);
		if (bufsz <= newWp) {
			newWp = 0;
		}
        if (szRegion1 < sz) {
            (*copierFunc)(pCopierFuncExtInfo, &pHandle->m_pBuf[0], &(((uint8_t*)pElement)[szRegion1]), sz - szRegion1);
            newWp = sz - szRegion1;
        }
    }
    __DMB();
    pHandle->m_wp = newWp;
    __DSB();
    return sz;
}

size_t CMN_SimpleFifoOfferContinuous(
        CMN_SimpleFifoHandle* pHandle,
        const void* pElement,
        size_t sz,
        int fallback,
        size_t* pGap) {
    return CMN_SimpleFifoOfferContinuousWithSpecificCopier(
            pHandle,
            pElement,
            sz,
            fallback,
            pGap,
            defaultCopier,
            NULL);
}

size_t CMN_SimpleFifoOfferContinuousWithSpecificCopier(
        CMN_SimpleFifoHandle* pHandle0,
        const void* pElement,
        size_t sz,
        int fallback,
        size_t* pGap,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo) {
    assert(pHandle0 != NULL);
    assert(pElement != NULL);
    assert(pGap != NULL);
    assert(copierFunc != NULL);

    volatile CMN_SimpleFifoHandle* pHandle = pHandle0;
    const size_t rp = pHandle->m_rp;
    const size_t wp = pHandle->m_wp;
    const size_t bufsz = pHandle->m_size;
    size_t newWp = wp;

    // try to insert in the continuous region just after wp
    if (sz <= getVacantSizeContinuous(bufsz, wp, rp)) {
        *pGap = 0;
		(*copierFunc)(pCopierFuncExtInfo, &pHandle->m_pBuf[wp], pElement, sz);
		newWp = wp + sz;
    	assert(newWp <= bufsz);
		if (bufsz <= newWp) {
			newWp = 0;
		}
		__DMB();
		pHandle->m_wp = newWp;
		__DSB();
		return sz;
    }

    // try to wrap and insert in the continous region at the beginning of the buffer
    if ((rp <= wp)
        && (sz <= (getVacantSize(bufsz, wp, rp) - getVacantSizeContinuous(bufsz, wp, rp)))) {
        *pGap = bufsz - wp;
        (*copierFunc)(pCopierFuncExtInfo, &pHandle->m_pBuf[0], pElement, sz);
        __DMB();
        pHandle->m_wp = sz;
        __DSB();
        return sz;
    }

    // no enough continuous space.
    *pGap = 0;
    if (fallback) {
        return CMN_SimpleFifoOfferWithSpecificCopier(pHandle0, pElement, sz, copierFunc, pCopierFuncExtInfo);
    }
    return 0; // no fallback and fail
}

size_t CMN_SimpleFifoPoll(
        CMN_SimpleFifoHandle* pHandle,
        void* pElement,
        size_t sz) {
    return CMN_SimpleFifoPollWithSpecificCopier(pHandle, pElement, sz, defaultCopier, NULL);
}

size_t CMN_SimpleFifoPollWithSpecificCopier(
        CMN_SimpleFifoHandle* pHandle0,
        void* pElement,
        size_t sz,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo) {
    assert(pHandle0 != NULL);
    assert(copierFunc != NULL);

    volatile CMN_SimpleFifoHandle* pHandle = pHandle0;
    PeekHandleInternal peekHandle;
    size_t ret = peek(pHandle0, &peekHandle, sz);
    if (ret) {
        if (pElement != NULL) {
            if (0 < peekHandle.m_szChunk[0]) {
                (*copierFunc)(
                        pCopierFuncExtInfo,
                        &(((uint8_t*)pElement)[0]),
                        &pHandle->m_pBuf[peekHandle.m_idxChunk[0]],
                        peekHandle.m_szChunk[0]);
                if (0 < peekHandle.m_szChunk[1]) {
                    (*copierFunc)(
                            pCopierFuncExtInfo,
                            &(((uint8_t*)pElement)[peekHandle.m_szChunk[0]]),
                            &pHandle->m_pBuf[peekHandle.m_idxChunk[1]],
                            peekHandle.m_szChunk[1]);
                }
            }
        }
        __DMB();
        pHandle->m_rp = peekHandle.m_newRp;
        __DSB();
    }
    return ret;
}                

size_t CMN_SimpleFifoPeekWithOffset(
        const CMN_SimpleFifoHandle* pHandle0,
        CMN_SimpleFifoPeekHandle* pPeekHandle,
        size_t sz,
        size_t offset) {
    assert(pHandle0 != NULL);
    assert(pPeekHandle != NULL);

    volatile const CMN_SimpleFifoHandle* pHandle = pHandle0;
    PeekHandleInternal peekHandle;
    size_t fetchSz = sz + offset;
    size_t ret = peek(pHandle0, &peekHandle, fetchSz);

    if (ret == fetchSz) {
        if (0 < offset) {
            if (0 <= sz) {
                if (offset < peekHandle.m_szChunk[0]) {
                    peekHandle.m_szChunk[0] -= offset;
                    peekHandle.m_idxChunk[0] += offset;
                } else {
                    const size_t offsetInSecondChunk = offset - peekHandle.m_szChunk[0];
                    peekHandle.m_szChunk[0]  = peekHandle.m_szChunk[1] - offsetInSecondChunk;
                    peekHandle.m_idxChunk[0] = peekHandle.m_idxChunk[1] + offsetInSecondChunk;
                    peekHandle.m_szChunk[1] = 0;
                    peekHandle.m_idxChunk[1] = 0;
                }
            }
            ret -= offset;
        }
        pPeekHandle->m_szChunk[0] = peekHandle.m_szChunk[0];
        pPeekHandle->m_pChunk[0] =
            peekHandle.m_szChunk[0] <= 0 ?
            NULL : &pHandle->m_pBuf[peekHandle.m_idxChunk[0]];
        pPeekHandle->m_szChunk[1] = peekHandle.m_szChunk[1];
        pPeekHandle->m_pChunk[1] =
            peekHandle.m_szChunk[1] <= 0 ?
            NULL : &pHandle->m_pBuf[peekHandle.m_idxChunk[1]];
    } else {
        pPeekHandle->m_szChunk[0] = 0;
        pPeekHandle->m_pChunk[0] = NULL;
        pPeekHandle->m_szChunk[1] = 0;
        pPeekHandle->m_pChunk[1] = NULL;
    }
    return ret;
}

/*
size_t CMN_SimpleFifoPeek(
        const CMN_SimpleFifoHandle* pHandle0,
        CMN_SimpleFifoPeekHandle* pPeekHandle,
        size_t sz) {
    return CMN_SimpleFifoPeekWithOffset(pHandle0, pPeekHandle, sz, 0);
}
*/

void CMN_SimpleFifoClear(
        CMN_SimpleFifoHandle* pHandle0) {
    assert(pHandle0 != NULL);

    volatile CMN_SimpleFifoHandle* pHandle = pHandle0;
    pHandle->m_wp = pHandle->m_rp = 0;
    __DSB();
}

size_t CMN_SimpleFifoGetVacantSize(
        const CMN_SimpleFifoHandle* pHandle0) {
    assert(pHandle0 != NULL);

    volatile const CMN_SimpleFifoHandle* pHandle = pHandle0;
    return getVacantSize(pHandle->m_size, pHandle->m_wp, pHandle->m_rp);
}

size_t CMN_SimpleFifoGetOccupiedSize(
        const CMN_SimpleFifoHandle* pHandle0) {
    assert(pHandle0 != NULL);

    volatile const CMN_SimpleFifoHandle* pHandle = pHandle0;
    return getOccupiedSize(pHandle->m_size, pHandle->m_wp, pHandle->m_rp);
}

void* CMN_SimpleFifoGetExtInfo(
        const CMN_SimpleFifoHandle* pHandle0) {
    assert(pHandle0 != NULL);

    volatile const CMN_SimpleFifoHandle* pHandle = pHandle0;
    return pHandle->m_pExtInfo;
}

size_t CMN_SimpleFifoGetDataSizeOfPeekHandle(
        const CMN_SimpleFifoPeekHandle* pPeekHandle) {
    assert(pPeekHandle != NULL);

    const int NUM_OF_CHUNK = sizeof(pPeekHandle->m_pChunk) / sizeof(pPeekHandle->m_pChunk[0]);
    size_t sz = 0;
    int i = 0;
    for (i = 0; i < NUM_OF_CHUNK; ++i) {
        if (pPeekHandle->m_pChunk[i] == NULL
            || pPeekHandle->m_szChunk[i] <= 0) {
            break;
        }
        sz += pPeekHandle->m_szChunk[i];
    }
    return sz;
}

size_t CMN_SimpleFifoCopyFromPeekHandle(
        const CMN_SimpleFifoPeekHandle* pPeekHandle,
        void* pDest,
        size_t sz) {
    return CMN_SimpleFifoCopyFromPeekHandleWithSpecificCopier(
            pPeekHandle,
            pDest,
            sz,
            defaultCopier,
            NULL);
}
    
size_t CMN_SimpleFifoCopyFromPeekHandleWithSpecificCopier(
        const CMN_SimpleFifoPeekHandle* pPeekHandle,
        void* pDest,
        size_t sz,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo) {
    assert(pPeekHandle != NULL);
    assert(pDest != NULL);
    assert(copierFunc != NULL);

    const int NUM_OF_CHUNK = sizeof(pPeekHandle->m_pChunk) / sizeof(pPeekHandle->m_pChunk[0]);
    size_t szRemain = sz;
    uint8_t* pPtr = (uint8_t*)pDest;
    int i = 0;
    for (i = 0; i < NUM_OF_CHUNK; ++i) {
        if (szRemain <= 0
            || pPeekHandle->m_pChunk[i] == NULL
            || pPeekHandle->m_szChunk[i] <= 0) {
            break;
        }
        size_t szCopy = pPeekHandle->m_szChunk[i];
        if (szRemain < szCopy) {
            szCopy = szRemain;
        }
        (*copierFunc)(pCopierFuncExtInfo, pPtr, pPeekHandle->m_pChunk[i], szCopy);
        pPtr += szCopy;
        szRemain -= szCopy;
    }
    return sz - szRemain;
}
