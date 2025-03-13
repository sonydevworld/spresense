/*****************************************************************************
 *
 * \file test-cobsr.c
 *
 * \brief Unit Tests for COBS/R
 *
 ****************************************************************************/


/*****************************************************************************
 * Includes
 ****************************************************************************/

#include "cobsr.h"

#include "unity.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>         /* For memset() */


/*****************************************************************************
 * Defines
 ****************************************************************************/

#ifndef FALSE
#define FALSE       (0)
#endif

#ifndef TRUE
#define TRUE        (!FALSE)
#endif

#ifndef dimof
#define dimof(X)    (sizeof(X)/sizeof((X)[0]))
#endif

#ifndef MIN
#define MIN(A, B)   (((A) < (B)) ? (A) : (B))
#endif

/*****************************************************************************
 * Tables
 ****************************************************************************/

typedef struct
{
    const uint8_t * data_ptr;
    size_t          data_len;
    const uint8_t * encoded_ptr;
    size_t          encoded_len;
    const char *    description_ptr;
} data_info;

static const data_info cobsr_tests[] =
{
    { "", 0,
        "\x01", 1, "Empty" },
    { "1", 1,
        "1", 1, "1 large non-zero byte" },
    { "\x02", 1,
        "\x02", 1, "1 borderline large non-zero byte" },
    { "\x01", 1,
        "\x02\x01", 2, "1 small non-zero byte" },
    { "12345", 5,
        "51234", 5, "5 non-zero bytes" },
    { "12345\x00""6789", 10,
        "\x06""123459678", 10, "Zero in middle" },
    { "\x00""12345\x00\x06\x07\x08\x01", 11,
        "\x01\x06""12345\x05""\x06\x07\x08\x01", 12, "Zero at start and middle" },
    { "12345\x00""6789\x00", 11,
        "\x06""12345\x05""6789\x01", 12, "Zero at start and end" },
    { "\x00", 1,
        "\x01\x01", 2, "1 zero byte" },
    { "\x00\x00", 2,
        "\x01\x01\x01", 3, "2 zero bytes" },
    { "\x00\x00\x00", 3,
        "\x01\x01\x01\x01", 4, "3 zero bytes" },
};

/*****************************************************************************
 * Helper Functions
 ****************************************************************************/

static void print_hex(const uint8_t * src_ptr, size_t src_len)
{
    size_t              i;


    for (i = 0; i < src_len; i++)
    {
        printf("%02X ", *src_ptr++);
    }
}

static uint8_t non_zero_byte_next(uint8_t non_zero_byte)
{
    if (non_zero_byte >= 'A' && non_zero_byte < 'Z')
    {
        ++non_zero_byte;
    }
    else if (non_zero_byte == 'Z')
    {
        non_zero_byte = 'a';
    }
    if (non_zero_byte >= 'a' && non_zero_byte < 'z')
    {
        ++non_zero_byte;
    }
    else if (non_zero_byte == 'z')
    {
        non_zero_byte = '0';
    }
    else if (non_zero_byte >= '0' && non_zero_byte < '9')
    {
        ++non_zero_byte;
    }
    else if (non_zero_byte == '9')
    {
        non_zero_byte = 'A';
    }
    else
    {
        non_zero_byte = 'A';
    }
    return non_zero_byte;
}

static uint8_t non_zero_byte_fill(void * p_dst, uint8_t start_value, size_t len)
{
    uint8_t * p_dst_u8 = p_dst;
    uint8_t non_zero_byte = start_value ? start_value : 'A';
    while (len--)
    {
        *p_dst_u8++ = non_zero_byte;
        non_zero_byte = non_zero_byte_next(non_zero_byte);
    }
    return non_zero_byte;
}

/*****************************************************************************
 * Test Functions
 ****************************************************************************/

void test_COBSR_ENCODE_DST_BUF_LEN_MAX(void)
{
    TEST_ASSERT_EQUAL_size_t(1, COBSR_ENCODE_DST_BUF_LEN_MAX(0));
    TEST_ASSERT_EQUAL_size_t(2, COBSR_ENCODE_DST_BUF_LEN_MAX(1));
    TEST_ASSERT_EQUAL_size_t(3, COBSR_ENCODE_DST_BUF_LEN_MAX(2));

    TEST_ASSERT_EQUAL_size_t(254, COBSR_ENCODE_DST_BUF_LEN_MAX(253));
    TEST_ASSERT_EQUAL_size_t(255, COBSR_ENCODE_DST_BUF_LEN_MAX(254));
    TEST_ASSERT_EQUAL_size_t(257, COBSR_ENCODE_DST_BUF_LEN_MAX(255));
    TEST_ASSERT_EQUAL_size_t(258, COBSR_ENCODE_DST_BUF_LEN_MAX(256));

    TEST_ASSERT_EQUAL_size_t(509, COBSR_ENCODE_DST_BUF_LEN_MAX(507));
    TEST_ASSERT_EQUAL_size_t(510, COBSR_ENCODE_DST_BUF_LEN_MAX(508));
    TEST_ASSERT_EQUAL_size_t(512, COBSR_ENCODE_DST_BUF_LEN_MAX(509));
    TEST_ASSERT_EQUAL_size_t(513, COBSR_ENCODE_DST_BUF_LEN_MAX(510));
}

void test_cobsr_fixed(void)
{
    uint8_t             encode_out[COBSR_ENCODE_DST_BUF_LEN_MAX(1000)];
    uint8_t             decode_out[1000];
    cobsr_encode_result encode_result;
    cobsr_decode_result decode_result;
    size_t              i;


    for (i = 0; i < dimof(cobsr_tests); i++)
    {
        memset(encode_out, 'A', sizeof(encode_out));

        encode_result = cobsr_encode(encode_out, sizeof(encode_out), cobsr_tests[i].data_ptr, cobsr_tests[i].data_len);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(cobsr_tests[i].encoded_len, encode_result.out_len, cobsr_tests[i].description_ptr);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(cobsr_tests[i].encoded_ptr, encode_out, cobsr_tests[i].encoded_len, cobsr_tests[i].description_ptr);

        decode_result = cobsr_decode(decode_out, sizeof(decode_out), encode_out, encode_result.out_len);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(cobsr_tests[i].data_len, decode_result.out_len, cobsr_tests[i].description_ptr);
        if (cobsr_tests[i].data_len)
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(cobsr_tests[i].data_ptr, decode_out, cobsr_tests[i].data_len, cobsr_tests[i].description_ptr);
        }
    }
}

void test_cobsr_zeros(void)
{
    uint8_t             data[530];
    uint8_t             expected_encode[COBSR_ENCODE_DST_BUF_LEN_MAX(530)];
    size_t              expected_encode_len;
    uint8_t             encode_out[COBSR_ENCODE_DST_BUF_LEN_MAX(530)];
    uint8_t             decode_out[530];
    char                msg[100];
    cobsr_encode_result encode_result;
    cobsr_decode_result decode_result;
    size_t              i;

    memset(data, 0u, sizeof(data));
    memset(expected_encode, 0x01u, sizeof(expected_encode));

    for (i = 1u; i < dimof(data); i++)
    {
        snprintf(msg, sizeof(msg), "i = %zu", i);
        expected_encode_len = i + 1u;
        encode_result = cobsr_encode(encode_out, sizeof(encode_out), data, i);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(expected_encode_len, encode_result.out_len, msg);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(expected_encode, encode_out, encode_result.out_len, msg);

        decode_result = cobsr_decode(decode_out, sizeof(decode_out), encode_out, encode_result.out_len);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(i, decode_result.out_len, msg);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(data, decode_out, i, msg);
    }
}

void test_cobsr_255(void)
{
    uint8_t             data[530];
    uint8_t             expected_encode[COBSR_ENCODE_DST_BUF_LEN_MAX(530)];
    size_t              expected_encode_len;
    uint8_t             encode_out[COBSR_ENCODE_DST_BUF_LEN_MAX(530)];
    uint8_t             decode_out[530];
    char                msg[100];
    cobsr_encode_result encode_result;
    cobsr_decode_result decode_result;
    size_t              i;

    memset(data, 0xFFu, sizeof(data));
    memset(expected_encode, 0xFFu, sizeof(expected_encode));

    for (i = 1u; i < dimof(data); i++)
    {
        snprintf(msg, sizeof(msg), "i = %zu", i);
        expected_encode_len = i + ((i - 1u) / 254u);
        encode_result = cobsr_encode(encode_out, sizeof(encode_out), data, i);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(expected_encode_len, encode_result.out_len, msg);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(expected_encode, encode_out, encode_result.out_len, msg);

        decode_result = cobsr_decode(decode_out, sizeof(decode_out), encode_out, encode_result.out_len);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(i, decode_result.out_len, msg);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(data, decode_out, i, msg);
    }
}

void test_cobsr_nonzeros(void)
{
    uint8_t *           p_dst;
    uint8_t             data[530];
    uint8_t             expected_encode[COBSR_ENCODE_DST_BUF_LEN_MAX(530)];
    size_t              expected_encode_len;
    uint8_t             encode_out[COBSR_ENCODE_DST_BUF_LEN_MAX(530)];
    uint8_t             decode_out[530];
    char                msg[100];
    cobsr_encode_result encode_result;
    cobsr_decode_result decode_result;
    size_t              i;
    size_t              chunk_len;
    size_t              j;
    uint8_t             non_zero_byte;

    non_zero_byte_fill(data, 'A', sizeof(data));
    for (i = 1u; i < dimof(data); i++)
    {
        snprintf(msg, sizeof(msg), "i = %zu", i);
        p_dst = expected_encode;
        expected_encode_len = 0;
        non_zero_byte = 'A';
        for (j = 0, chunk_len = 0;
            j < i;
            j += chunk_len)
        {
            chunk_len = MIN(254, i - j);
            *p_dst++ = (uint8_t)(chunk_len + 1u);
            non_zero_byte = non_zero_byte_fill(p_dst, non_zero_byte, chunk_len);
            p_dst += chunk_len;
            expected_encode_len += chunk_len + 1u;
        }
        if (expected_encode[expected_encode_len - 1u] >= chunk_len + 1u)
        {
            expected_encode[expected_encode_len - chunk_len - 1u] = expected_encode[expected_encode_len - 1u];
            expected_encode_len--;
        }

        encode_result = cobsr_encode(encode_out, sizeof(encode_out), data, i);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(expected_encode_len, encode_result.out_len, msg);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(expected_encode, encode_out, encode_result.out_len, msg);

        decode_result = cobsr_decode(decode_out, sizeof(decode_out), encode_out, encode_result.out_len);
        TEST_ASSERT_EQUAL_size_t_MESSAGE(i, decode_result.out_len, msg);
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(data, decode_out, i, msg);
    }
}

/**
 * \brief Common set up code for each unit test
 */
void setUp( void )
{
}

/**
 * \brief Common take-down code for the end of each unit test
 */
void tearDown( void )
{
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_COBSR_ENCODE_DST_BUF_LEN_MAX);
    RUN_TEST(test_cobsr_fixed);
    RUN_TEST(test_cobsr_zeros);
    RUN_TEST(test_cobsr_255);
    RUN_TEST(test_cobsr_nonzeros);

    return UNITY_END();

    return 0;
}
