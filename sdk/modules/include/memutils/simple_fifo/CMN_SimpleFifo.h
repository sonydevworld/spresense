/****************************************************************************
 * modules/include/memutils/simple_fifo/CMN_SimpleFifo.h
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

/**
 * @defgroup memutils Memory Utility Libraries
 * @{
 */



#if !defined(CMN_SIMPLE_FIFO_H)
#define CMN_SIMPLE_FIFO_H

/* API Documents creater with Doxgen */

/**
* @ingroup  driver
* @defgroup memuils_simple_fifo Simple Fifo API
* @brief    
* @{
*/

/**
 * @file       CMN_SimpleFifo.h
 * @brief      CXD5602 Sipmle Fifo API
 * @author     CXD5602 Media SW Team
 * @note       
 * @attention  
 */

#ifdef __cplusplus
 extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/*!
 * @brief FIFO handle which holds data required to manipulate FIFO.
 * 
 * Clients MUST NOT access to the internal members directly. To refer
 * ExtInfo, use CMN_SimpleFifoGetExtInfo()
 *
 * @sa CMN_SimpleFifoGetExtInfo
 */
typedef struct {
    uint8_t* m_pBuf; //!< Internal ring buffer.
    void* m_pExtInfo; //!< Info set to the fifo. CMN_SimpleFifo APIs never refer/modify it.  Use CMN_SimpleFifoGetExtInfo() to refer the info.
    size_t m_size; //!< Size of the internal ring buffer. Available size if m_size - 1 because one element is used as a separator of RP and WP.
    size_t m_wp; //!< Write Pointer. Index of the m_pBuf.
    size_t m_rp; //!< Read Pointer. Index of the m_pBuf.
} CMN_SimpleFifoHandle;

/*!
 * @brief Data handle used for CMN_SimpleFifoPeek().
 * 
 * Data retrieved with any variations of peek API may be splited in to
 * 2 pieces because of data allocation in the FIFO internal
 * buffer. Index 0 indicates the first piece and 1 is the second piece
 * if any.
 *
 * Use CMN_SimpleFifoCopyFromPeekHandle() or
 * CMN_SimpleFifoCopyFromPeekHandleWithSpecificCopier() to copy data
 * from a peek handle.
 *
 * @sa CMN_SimpleFifoCopyFromPeekHandle
 * @sa CMN_SimpleFifoCopyFromPeekHandleWithSpecificCopier
 */
typedef struct {
    uint8_t* m_pChunk[2]; //!< Address to the region.
    size_t m_szChunk[2]; //!< Data Size of the region.
} CMN_SimpleFifoPeekHandle;

/*!
 * @brief Definition of data copier function.
 *  
 * Clients can set specific copier function to some APIs.
 *
 * Specific copier might be special copier function like
 * DMA-controller-involved-copier or special-logic-copier.
 *
 * Otherwise, normal version APIs that utilize embedded memory copier
 * should be enough.
 * 
 * @param[in] pCopierFuncExtInfo Info passed to the copier function.
 * @param[in] pDest Address of the destination region.
 * @param[in] pSrc Address of the source region.
 * @param[in] sz Number of bytes to copy.
 * @return Pointer to the destination region.
 */
typedef void* (*CMN_SimpleFifoCopierFunc)(
        void* pCopierFuncExtInfo,
        void* pDest,
        const void* pSrc,
        size_t sz);

/*!
 * @name Manupilation
 */
//@{
/*!
 * @brief Initialize FIFO.
 * 
 * Clients should prepare all memory regions required for the
 * FIFO. This function receives memory regions and sets up them
 * for FIFO use.
 *
 * On failures, any data pointed with the pHandle and the pFifoBuffer
 * are kept untouched.
 * 
 * @param[in] pHandle Pointer to a memory region used for FIFO
 *            management. It should be word aligned address. NULL is
 *            NOT allowed.
 *            - API call Failure
 *                - Not word aligned
 *                - NULL
 *            - Assertion Failure
 *
 * @param[in] pFifoBuffer Pointer to a memory region used for FIFO
 *            data buffer. It should be word aligned address. NULL is
 *            NOT allowed.
 *            - API call Failure
 *                - Not word aligned
 *                - NULL
 *            - Assertion Failure
 *
 * @param[in] szFifoBuffer Size of memory region pointed with
 *            pFifoBuffer. It must be equal to or greater than
 *            2. Avaiable size for FIFO data is szFifoBuffer - 1.
 *            If the value is 1 or less, the API call fails.
 *            - API call Failure
 *                - 1 or less
 *            - Assertion Failure
 *
 * @param[in] pExtInfo Data shared through the FIFO. Any
 *            CMN_SimpleFifo APIs never refer to and modify this. NULL
 *            is allowed.
 *
 * @return Result of the API call. 
 *           - 0 on success
 *           - -1 on failures. 
 */
int CMN_SimpleFifoInitialize(
        CMN_SimpleFifoHandle* pHandle,
        void* pFifoBuffer,
        size_t szFifoBuffer,
        void* pExtInfo);
//@}

/*!
 * @name Offer(Insert)
 */
//@{

/*!
 * @brief Insert data to the FIFO using the default copier(memcpy like).
 *
 * If all data cannot be stored, FIFO is kept untouched and the API
 * call fails.
 *  
 * @param[in] pHandle Pointer to the control block of the FIFO in which
 *            data will be stored. NULL is NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pElement Pointer to the data to be stored. NULL is NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] sz Sise of data to be stored.
 * 
 * @return 
 *         - On success, size of stored data
 *         - On failure, 0
 */
size_t CMN_SimpleFifoOffer(
        CMN_SimpleFifoHandle* pHandle,
        const void* pElement,
        size_t sz);

/*!
 * @brief Insert data to the FIFO using the specified copier.
 *
 * Specific copier might be special copier function like
 * DMA-controller-involved-copier or special-logic-copier.
 *
 * If all data cannot be stored, FIFO is kept untouched.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO in
 *            which data will be stored. NULL is NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pElement Pointer to the data to be stored. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] sz Size of the data to be stored.
 * 
 * @param[in] copierFunc Pointer to copier function. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pCopierFuncExtInfo An argument passed to the
 *            copierFunc. CMN_SimpleFifo never refer to the pointer.
 *
 * @return
 *         - On success, size of stored data
 *         - On failure, 0
 */
size_t CMN_SimpleFifoOfferWithSpecificCopier(
        CMN_SimpleFifoHandle* pHandle,
        const void* pElement,
        size_t sz,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo);

/*!
 * @brief Insert data to the FIFO with Continuous Region Option using the default copier(memcpy like).
 *
 * It will try to store data in one (continuous) region first. Client
 * can specify a preferred behavior for the case that it's impossible
 * to store the data in one region.
 *
 * If all data cannot be stored after all, FIFO is kept untouched.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO in
 *            which data will be stored. NULL is NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pElement Pointer to the data to be stored. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] sz Size of the data to be stored.
 * 
 * @param[in] fallback Specify behavior if it's impossible to store
 *            the data in one region. Fallback is to allow to store in
 *            separated regions
 *            - 0 to allow fallback.
 *            - Values other than 0 to NOT allow fallback.
 *
 * @param[out] pGap Size of the gap inserted prior to the store
 *             data. FIFO readers should skip the gap to obtain the
 *             data. If the API call fails, cleared with 0. NULL is
 *             NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @return
 *         - On success, size of stored data
 *         - On failure, 0
 */
size_t CMN_SimpleFifoOfferContinuous(
        CMN_SimpleFifoHandle* pHandle,
        const void* pElement,
        size_t sz,
        int fallback,
        size_t* pGap);

/*!
 *  @brief Insert data to the FIFO with Continuous Region Option using
 *         the specified copier.
 *
 * It will try to store data in one (continuous) region. Client can
 * specify the preferred behavior if it's impossible to store the data
 * in one region.
 *
 * Specific copier might be special copier function like
 * DMA-controller-involved-copier or special-logic-copier.
 *  
 * If all data cannot be stored, FIFO is kept untouched.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO in
 *            which data will be stored. NULL is NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pElement Pointer to the data to be stored. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] sz Size of the data to be stored.
 * 
 * @param[in] fallback Specify behavior if it's impossible to store
 *            the data in one region. Fallback is to allow to store in
 *            separated regions
 *            - 0 to allow fallback.
 *            - Values other than 0 to NOT allow fallback.
 *
 * @param[out] pGap Size of the gap inserted prior to the store
 *             data. FIFO readers should skip the gap to obtain the
 *             data. If the API call fails, cleared with 0. NULL is
 *             NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 *
 * @param[in] copierFunc Pointer to copier function. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pCopierFuncExtInfo An argument passed to the
 *            copierFunc. CMN_SimpleFifo never refer to the pointer.
 *
 * @return
 *         - On success, size of stored data
 *         - On failure, 0
 */
size_t CMN_SimpleFifoOfferContinuousWithSpecificCopier(
        CMN_SimpleFifoHandle* pHandle,
        const void* pElement,
        size_t sz,
        int fallback,
        size_t* pGap,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo);
//@}

/*!
 * @name Refer and Remove
 */
//@{
/*!
 * @brief Retrieves data on the head of FIFO and removes it from the
 *        FIFO using the default copier(memcpy like).
 *
 * On failures, the FIFO is kept untouched.
 *        
 * @param[in] pHandle Pointer to the control block of the FIFO from
 *            which data will be retrieved. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @param[out] pElement Pointer to the memory in which retrieved data
 *             is stored. Client should prepare enough memory. NULL is
 *             allowed. If NULL is specified, retrieved data is
 *             just dicarded.
 *
 * @param[in] sz Size of data to retrieve.
 * 
 * @return
 *  - On success, size of data retrieved.
 *  - On failure, 0 is returned.
 */
size_t CMN_SimpleFifoPoll(
        CMN_SimpleFifoHandle* pHandle,
        void* pElement,
        size_t sz);

/*!
 * @brief Retrieves data on the head of FIFO and removes it from the
 *        FIFO using the specified copier.
 *
 * Specific copier might be special copier function like
 * DMA-controller-involved-copier or special-logic-copier.
 *
 * On failures, the FIFO is kept untouched.
 *        
 * @param[in] pHandle Pointer to the control block of the FIFO from
 *            which data will be retrieved. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @param[out] pElement Pointer to the memory in which retrieved data
 *             is stored. Client should prepare enough memory. NULL is
 *             allowed. If NULL is specified, retrieved data is
 *             just dicarded.
 *
 * @param[in] sz Size of data to retrieve.
 * 
 * @param[in] copierFunc Pointer to copier function. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pCopierFuncExtInfo An argument passed to the
 *            copierFunc. CMN_SimpleFifo never refer to the pointer.
 *
 * @return
 *  - On success, size of data retrieved.
 *  - On failure, 0 is returned.
 */

size_t CMN_SimpleFifoPollWithSpecificCopier(
        CMN_SimpleFifoHandle* pHandle,
        void* pElement,
        size_t sz,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo);
//@}

/*!
 * @name Refer and NOT Remove
 */
//@{
/*! @brief Retrieves data on the offset from head of the FIFO but does not remove
 *         from the FIFO.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO from
 *            which data will be retrieved. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @param[out] pPeekHandle Pointer to the memory in which retrieved
 *             peek info is stored. Client must prepare enough
 *             memory. On failure, cleared with values meaning
 *             empty. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @param[in] sz Size of data to retrieve.
 * 
 * @param[in] offset Offest from RP(read point) of pHandle.
 * 
 * @return
 *  - On success, size of data retrieved.
 *  - On failure, 0 is returned.
 */
size_t CMN_SimpleFifoPeekWithOffset(
        const CMN_SimpleFifoHandle* pHandle,
        CMN_SimpleFifoPeekHandle* pPeekHandle,
        size_t sz,
        size_t offset);
//@}

/*!
 * @name Refer and NOT Remove
 */
//@{
/*! @brief Retrieves data on the head of the FIFO but does not remove
 *         from the FIFO.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO from
 *            which data will be retrieved. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @param[out] pPeekHandle Pointer to the memory in which retrieved
 *             peek info is stored. Client must prepare enough
 *             memory. On failure, cleared with values meaning
 *             empty. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @param[in] sz Size of data to retrieve.
 * 
 * @return
 *  - On success, size of data retrieved.
 *  - On failure, 0 is returned.
 */
static inline size_t CMN_SimpleFifoPeek(
        const CMN_SimpleFifoHandle* pHandle,
        CMN_SimpleFifoPeekHandle* pPeekHandle,
        size_t sz) {
    return CMN_SimpleFifoPeekWithOffset(pHandle, pPeekHandle, sz, 0);
}

//@}

/*!
 * @name Manupilation
 */
//@{
/*!
 * @brief Reset the RP/WP of the FIFO.
 *
 * FIFO gets empty but all data in the buffer still remain.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO. NULL is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 */
void CMN_SimpleFifoClear(
        CMN_SimpleFifoHandle* pHandle);
//@}

/*!
 * @name Query and Utility
 */
//@{
/*!
 * @brief Get vacant size of the FIFO.
 * 
 * The vacant size is the size which is available to store data in the
 * FIFO.
 *
 * @param[in] pHandle Pointer to the control block of the FIFO. NULL
 *            is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 *
 * @return Vacant size of the FIFO.
 */ 
size_t CMN_SimpleFifoGetVacantSize(
        const CMN_SimpleFifoHandle* pHandle);

/*!
 * @brief Get occupied size of the FIFO.
 *
 * The occupied size is the size of data already stored in the FIFO.
 * 
 * @param[in] pHandle Pointer to the control block of the FIFO. NULL
 *            is NOT allowed.
 *            - Assertion Failure
 *                - NULL
 * @return Occupied size of the FIFO.
 */ 
size_t CMN_SimpleFifoGetOccupiedSize(
        const CMN_SimpleFifoHandle* pHandle);

/*!
 * @brief Get the value of the ExtInfo set to the FIFO.
 * 
 * @param[in] pHandle Pointer to the control block of the FIFO. NULL
 *            is NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @return Value of the ExtInfo set to the FIFO.
 */
void* CMN_SimpleFifoGetExtInfo(
        const CMN_SimpleFifoHandle* pHandle);

/*!
 * @brief Get total size of data in peek handle.
 * 
 * @param[in] pPeekHandle Pointer to the peek handle to calc. NULL is
 *            NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *              
 * @return Size of data in peek handle.
 */
size_t CMN_SimpleFifoGetDataSizeOfPeekHandle(
        const CMN_SimpleFifoPeekHandle* pPeekHandle);

/*!
 * @brief Copy data pointed by PeekHandle using default copier(memcpy like).
 * 
 * @param[in] pPeekHandle Pointer to the source peek handle. NULL is
 *            NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[out] pDest Pointer to the destination region. NULL is NOT
 *             allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] sz Maximum size to copy in byte.
 *              
 * @return Number of copied byte.
 */
size_t CMN_SimpleFifoCopyFromPeekHandle(
        const CMN_SimpleFifoPeekHandle* pPeekHandle,
        void* pDest,
        size_t sz);

/*!
 * @brief Copy data pointed by PeekHandle using specific copier.
 *
 * Specific copier might be special copier function like
 * DMA-controller-involved-copier or special-logic-copier.
 * 
 * @param[in] pPeekHandle Pointer to the source peek handle.  NULL is
 *            NOT allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[out] pDest Pointer to the destination region. NULL is NOT
 *             allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] sz Maximum size to copy in byte.
 *              
 * @param[in] copierFunc Pointer to copier function. NULL is NOT
 *            allowed.
 *            - Assertion Failure
 *              - NULL
 *
 * @param[in] pCopierFuncExtInfo An argument passed to the
 *            copierFunc. CMN_SimpleFifo never refer to the pointer.
 *
 * @return Number of copied byte.
 */
size_t CMN_SimpleFifoCopyFromPeekHandleWithSpecificCopier(
        const CMN_SimpleFifoPeekHandle* pPeekHandle,
        void* pDest,
        size_t sz,
        CMN_SimpleFifoCopierFunc copierFunc,
        void* pCopierFuncExtInfo);
//@}

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* CMN_SIMPLE_FIFO_H */

/*
 * @mainpage SimpleFifo API manual
 *
 * SimpleFifo is a FIFO library.
 *
 * Basically it supports one-writer and one-reader access without any
 * exclusive control. Some exclusive access control is required to
 * support multi-writer/reader outside of this library.
 *
 * This library supports access from multi-processor inserting proper
 * data-sync-barriers and data-memory-barriers. It makes sure the
 * order of update data contents and WP/RP for the purpose.
 *
 * @section how-to-use How to use
 * To use this API, include header file CMN_SimpleFifo.h. In CXD5602 build system, add the following line in your C/C++ code.
 * @code
 * #include <common/CMN_SimpleFifo.h>
 * @endcode
 * 
 * See \ref sample for more details.
 *
 * @section api API
 * See CMN_SimpleFifo.h.
 *
 * @section sample Sample Code
 * Error check is omitted for explanatory purposes.
 * 
 * @code
 * #include <stdio.h>
 * #include <stdlib.h>
 *
 * #include <common/CMN_SimpleFifo.h>
 * 
 * // This is just for explanation.
 * // If you want to use normal memcpy() for FIFO operations,
 * // do not use WithSpecificCopier version API but use normal
 * //relavant API which includes memcpy() like copier.
 * static void* myOwnCopier(
 *         void* pCopierFuncExtInfo,
 *         void* pDest,
 *         const void* pSrc,
 *         size_t sz) {
 *     return memcpy(pDest, pSrc, sz);
 * }
 * 
 * int main() {
 *     //
 *     // Setup FIFO
 *     //
 *     // We assign 256byte buffer to the FIFO. So we can store 255(= 256-1)byte in maximum.
 *     const size_t SIZE = 256;
 *     CMN_SimpleFifoHandle* pHandle = malloc(sizeof(CMN_SimpleFifoHandle));
 *     void* pFifoBuffer = malloc(SIZE);
 *     if (pHandle == NULL
 *         || pFifoBuffer == NULL
 *         || CMN_SimpleFifoInitialize(pHandle, pFifoBuffer, SIZE, NULL) != 0) {
 *         printf("FIFO create error\n");
 *         return -1;
 *     }
 * 
 *     const uint8_t src[] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm'};
 *     uint8_t dst[128];
 * 
 *     //
 *     // The simplest usage example.
 *     //
 *     CMN_SimpleFifoOffer(pHandle, src, 2); // write
 *     CMN_SimpleFifoPoll(pHandle, dst, 2);  // read
 * 
 *     //
 *     // Continuous Region Option usage example.
 *     //
 *     //   write in a continuous region
 *     size_t gap = 0;
 *     CMN_SimpleFifoOfferContinuous(pHandle, src, 2, 0, &gap); 
 *     //   skip gap if any
 *     if (0 < gap) {
 *         CMN_SimpleFifoPoll(pHandle, NULL, gap);
 *     }
 *     //   read 
 *     CMN_SimpleFifoPoll(pHandle, dst, 2);
 * 
 *     //
 *     // Peek usage example.
 *     //
 *     CMN_SimpleFifoOfferContinuous(pHandle, src, 2, 0, &gap);
 *     if (0 < gap) {
 *         CMN_SimpleFifoPoll(pHandle, NULL, gap); // skip gap if any
 *     }
 *     CMN_SimpleFifoPeekHandle peekHandle;
 *     CMN_SimpleFifoPeek(pHandle, &peekHandle, 2);
 *     size_t szPeekData = CMN_SimpleFifoGetDataSizeOfPeekHandle(&peekHandle);
 *     if (sizeof(dst) / sizeof(dst[0]) < szPeekData) {
 *         szPeekData = sizeof(dst) / sizeof(dst[0]);
 *     }
 *     CMN_SimpleFifoCopyFromPeekHandle(&peekHandle, dst, szPeekData);
 * 
 *     //
 *     // Specific copier usage example.
 *     //
 *     CMN_SimpleFifoOfferWithSpecificCopier(pHandle, src, 2, myOwnCopier, NULL); // write
 *     CMN_SimpleFifoPollWithSpecificCopier(pHandle, dst, 2, myOwnCopier, NULL);  // read
 * 
 *     //
 *     //  dispose the FIFO.
 *     //
 *     CMN_SimpleFifoClear(pHandle);
 *     free(pHandle);
 *     free(pFifoBuffer);
 * 
 *     return 0;
 * }
 * @endcode
 * 
 * @section buffer Internal Buffer Status
 * Here is an example of internal buffer status.
 *
 * @image html SimpleFifoBufferStatus.png
 * 
 */

/**
 * @}
 */
