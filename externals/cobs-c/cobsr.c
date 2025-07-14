/*
 * cobsr.c
 *
 * Consistent Overhead Byte Stuffing--Reduced (COBS/R)
 */

#include <stdlib.h>
#include "cobsr.h"


/*****************************************************************************
 * Defines
 ****************************************************************************/

#ifndef FALSE
#define FALSE       (0)
#endif

#ifndef TRUE
#define TRUE        (!FALSE)
#endif


/*****************************************************************************
 * Functions
 ****************************************************************************/

/* COBS/R-encode a string of input bytes, which may save one byte of output.
 *
 * dst_buf_ptr:    The buffer into which the result will be written
 * dst_buf_len:    Length of the buffer into which the result will be written
 * src_ptr:        The byte string to be encoded
 * src_len         Length of the byte string to be encoded
 *
 * returns:        A struct containing the success status of the encoding
 *                 operation and the length of the result (that was written to
 *                 dst_buf_ptr)
 */
cobsr_encode_result cobsr_encode(void * dst_buf_ptr, size_t dst_buf_len,
                                 const void * src_ptr, size_t src_len)
{
    cobsr_encode_result result              = { 0u, COBSR_ENCODE_OK };
    const uint8_t *     src_read_ptr        = src_ptr;
    const uint8_t *     src_end_ptr         = src_ptr + src_len;
    uint8_t *           dst_buf_start_ptr   = dst_buf_ptr;
    uint8_t *           dst_buf_end_ptr     = dst_buf_ptr + dst_buf_len;
    uint8_t *           dst_code_write_ptr  = dst_buf_ptr;
    uint8_t *           dst_write_ptr       = dst_code_write_ptr + 1u;
    uint8_t             src_byte            = 0u;
    uint8_t             search_len          = 1u;


    /* First, do a NULL pointer check and return immediately if it fails. */
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL))
    {
        result.status = COBSR_ENCODE_NULL_POINTER;
        return result;
    }

    if (src_len != 0u)
    {
        /* Iterate over the source bytes */
        for (;;)
        {
            /* Check for running out of output buffer space */
            if (dst_write_ptr >= dst_buf_end_ptr)
            {
                result.status |= COBSR_ENCODE_OUT_BUFFER_OVERFLOW;
                break;
            }

            src_byte = *src_read_ptr++;
            if (src_byte == 0u)
            {
                /* We found a zero byte */
                *dst_code_write_ptr = search_len;
                dst_code_write_ptr = dst_write_ptr++;
                search_len = 1u;
                if (src_read_ptr >= src_end_ptr)
                {
                    break;
                }
            }
            else
            {
                /* Copy the non-zero byte to the destination buffer */
                *dst_write_ptr++ = src_byte;
                search_len++;
                if (src_read_ptr >= src_end_ptr)
                {
                    break;
                }
                if (search_len == 0xFF)
                {
                    /* We have a long string of non-zero bytes, so we need
                     * to write out a length code of 0xFF. */
                    *dst_code_write_ptr = search_len;
                    dst_code_write_ptr = dst_write_ptr++;
                    search_len = 1u;
                }
            }
        }
    }

    /* We've reached the end of the source data (or possibly run out of output buffer)
     * Finalise the remaining output. In particular, write the code (length) byte.
     *
     * For COBS/R, the final code (length) byte is special: if the final data byte is
     * greater than or equal to what would normally be the final code (length) byte,
     * then replace the final code byte with the final data byte, and remove the final
     * data byte from the end of the sequence. This saves one byte in the output.
     *
     * Update the pointer to calculate the final output length.
     */
    if (dst_code_write_ptr >= dst_buf_end_ptr)
    {
        /* We've run out of output buffer to write the code byte. */
        result.status |= COBSR_ENCODE_OUT_BUFFER_OVERFLOW;
        dst_write_ptr = dst_buf_end_ptr;
    }
    else
    {
        if (src_byte < search_len)
        {
            /* Encoding same as plain COBS */
            *dst_code_write_ptr = search_len;
        }
        else
        {
            /* Special COBS/R encoding: length code is final byte,
             * and final byte is removed from data sequence. */
            *dst_code_write_ptr = src_byte;
            dst_write_ptr--;
        }
    }

    /* Calculate the output length, from the value of dst_code_write_ptr */
    result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);

    return result;
}


/* Decode a COBS/R byte string.
 *
 * dst_buf_ptr:    The buffer into which the result will be written
 * dst_buf_len:    Length of the buffer into which the result will be written
 * src_ptr:        The byte string to be decoded
 * src_len         Length of the byte string to be decoded
 *
 * returns:        A struct containing the success status of the decoding
 *                 operation and the length of the result (that was written to
 *                 dst_buf_ptr)
 */
cobsr_decode_result cobsr_decode(void * dst_buf_ptr, size_t dst_buf_len,
                                 const void * src_ptr, size_t src_len)
{
    cobsr_decode_result result              = { 0u, COBSR_DECODE_OK };
    const uint8_t *     src_read_ptr        = src_ptr;
    const uint8_t *     src_end_ptr         = src_ptr + src_len;
    uint8_t *           dst_buf_start_ptr   = dst_buf_ptr;
    uint8_t *           dst_buf_end_ptr     = dst_buf_ptr + dst_buf_len;
    uint8_t *           dst_write_ptr       = dst_buf_ptr;
    size_t              remaining_input_bytes;
    size_t              remaining_output_bytes;
    uint8_t             num_output_bytes;
    uint8_t             src_byte;
    uint8_t             i;
    uint8_t             len_code;


    /* First, do a NULL pointer check and return immediately if it fails. */
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL))
    {
        result.status = COBSR_DECODE_NULL_POINTER;
        return result;
    }

    if (src_len != 0u)
    {
        for (;;)
        {
            len_code = *src_read_ptr++;
            if (len_code == 0u)
            {
                result.status |= COBSR_DECODE_ZERO_BYTE_IN_INPUT;
                break;
            }

            /* Calculate remaining input bytes */
            remaining_input_bytes = (size_t)(src_end_ptr - src_read_ptr);

            if ((len_code - 1u) < remaining_input_bytes)
            {
                num_output_bytes = (uint8_t)(len_code - 1u);

                /* Check length code against remaining output buffer space */
                remaining_output_bytes = (size_t)(dst_buf_end_ptr - dst_write_ptr);
                if (num_output_bytes > remaining_output_bytes)
                {
                    result.status |= COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                    num_output_bytes = (uint8_t)remaining_output_bytes;
                }

                for (i = num_output_bytes; i != 0u; i--)
                {
                    src_byte = *src_read_ptr++;
                    if (src_byte == 0u)
                    {
                        result.status |= COBSR_DECODE_ZERO_BYTE_IN_INPUT;
                    }
                    *dst_write_ptr++ = src_byte;
                }

                /* Add a zero to the end */
                if (len_code != 0xFFu)
                {
                    if (dst_write_ptr >= dst_buf_end_ptr)
                    {
                        result.status |= COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                        break;
                    }
                    *dst_write_ptr++ = '\0';
                }
            }
            else
            {
                /* We've reached the last length code, so write the remaining
                 * bytes and then exit the loop. */

                num_output_bytes = (uint8_t)remaining_input_bytes;

                /* Check length code against remaining output buffer space */
                remaining_output_bytes = (size_t)(dst_buf_end_ptr - dst_write_ptr);
                if (num_output_bytes > remaining_output_bytes)
                {
                    result.status |= COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                    num_output_bytes = (uint8_t)remaining_output_bytes;
                }

                for (i = num_output_bytes; i != 0u; i--)
                {
                    src_byte = *src_read_ptr++;
                    if (src_byte == 0u)
                    {
                        result.status |= COBSR_DECODE_ZERO_BYTE_IN_INPUT;
                    }
                    *dst_write_ptr++ = src_byte;
                }

                /* Write final data byte, if applicable for COBS/R encoding. */
                if ((len_code - 1u) > remaining_input_bytes)
                {
                    if (dst_write_ptr >= dst_buf_end_ptr)
                    {
                        result.status |= COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                    }
                    else
                    {
                        *dst_write_ptr++ = len_code;
                    }
                }

                /* Exit the loop */
                break;
            }
        }

        result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);
    }

    return result;
}
