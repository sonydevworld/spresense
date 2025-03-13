#!/usr/bin/python3

"""
Consistent Overhead Byte Stuffing--Reduced (COBS/R)

Unit Tests specific to the C implementation details.
In particular, test output buffer overflow detection.

This version is for Python 3.x.
"""

from array import array
import ctypes
import sys
import unittest

import cobsr_wrapper


class CSpecificTests(unittest.TestCase):
    predefined_encodings = [
        [ b"",                                      b"\x01"                                                         ],
        [ b"\x01",                                  b"\x02\x01"                                                     ],
        [ b"\x02",                                  b"\x02"                                                         ],
        [ b"\x03",                                  b"\x03"                                                         ],
        [ b"\x7E",                                  b"\x7E"                                                         ],
        [ b"\x7F",                                  b"\x7F"                                                         ],
        [ b"\x80",                                  b"\x80"                                                         ],
        [ b"\xD5",                                  b"\xD5"                                                         ],
        [ b"1",                                     b"1"                                                            ],
        [ b"\x05\x04\x03\x02\x01",                  b"\x06\x05\x04\x03\x02\x01"                                     ],
        [ b"12345",                                 b"51234"                                                        ],
        [ b"12345\x00\x04\x03\x02\x01",             b"\x0612345\x05\x04\x03\x02\x01"                                ],
        [ b"12345\x006789",                         b"\x06123459678"                                                ],
        [ b"\x0012345\x006789",                     b"\x01\x06123459678"                                            ],
        [ b"12345\x006789\x00",                     b"\x0612345\x056789\x01"                                        ],
        [ b"\x00",                                  b"\x01\x01"                                                     ],
        [ b"\x00\x00",                              b"\x01\x01\x01"                                                 ],
        [ b"\x00\x00\x00",                          b"\x01\x01\x01\x01"                                             ],
        [ bytearray(range(1, 254)),                 b"\xfe" + bytearray(range(1, 254))                              ],
        [ bytearray(range(1, 255)),                 b"\xff" + bytearray(range(1, 255))                              ],
        [ bytearray(range(1, 256)),                 b"\xff" + bytearray(range(1, 255)) + b"\xff"                    ],
        [ bytearray(range(0, 256)),                 b"\x01\xff" + bytearray(range(1, 255)) + b"\xff"                ],
        [ bytearray(range(2, 256)),                 b"\xff" + bytearray(range(2, 255))                              ],
    ]

    def test_encode_null_pointer(self):
        for (test_string, _expected_encoded_string) in self.predefined_encodings:
            test_bytes = bytes(cobsr_wrapper._get_buffer_view(test_string))
            out_buffer_len = cobsr_wrapper.encode_size_max(len(test_bytes))
            out_buffer = ctypes.create_string_buffer(out_buffer_len)

            # First, check that output buffer NULL pointer generates error.
            ret_val = cobsr_wrapper.encode_cfunc(None, out_buffer_len, test_bytes, len(test_bytes))
            self.assertTrue(ret_val.status & cobsr_wrapper.CobsrEncodeStatus.NULL_POINTER)
            self.assertEqual(ret_val.out_len, 0)

            # Second, check that input buffer NULL pointer generates error.
            ret_val = cobsr_wrapper.encode_cfunc(out_buffer, out_buffer_len, None, len(test_bytes))
            self.assertTrue(ret_val.status & cobsr_wrapper.CobsrEncodeStatus.NULL_POINTER)
            self.assertEqual(ret_val.out_len, 0)

    def test_decode_null_pointer(self):
        for (_expected_decoded_string, encoded_string) in self.predefined_encodings:
            out_buffer_len = cobsr_wrapper.decode_size_max(len(encoded_string))
            out_buffer = ctypes.create_string_buffer(out_buffer_len)

            # First, check that output buffer NULL pointer generates error.
            ret_val = cobsr_wrapper.decode_cfunc(None, out_buffer_len, encoded_string, len(encoded_string))
            self.assertTrue(ret_val.status & cobsr_wrapper.CobsrDecodeStatus.NULL_POINTER)
            self.assertEqual(ret_val.out_len, 0)

            # Second, check that input buffer NULL pointer generates error.
            ret_val = cobsr_wrapper.decode_cfunc(out_buffer, out_buffer_len, None, len(encoded_string))
            self.assertTrue(ret_val.status & cobsr_wrapper.CobsrDecodeStatus.NULL_POINTER)
            self.assertEqual(ret_val.out_len, 0)

    def test_encode_output_overflow(self):
        for (test_string, expected_encoded_string) in self.predefined_encodings:
            test_bytes = bytes(cobsr_wrapper._get_buffer_view(test_string))
            try:
                # Sanity check that the expected encode string length is not
                # bigger than the computed maximum.
                self.assertTrue(len(expected_encoded_string) <= cobsr_wrapper.encode_size_max(len(test_bytes)))

                real_out_buffer_len = cobsr_wrapper.encode_size_max(len(test_bytes)) + 100

                for out_buffer_len in range(0, real_out_buffer_len + 1):

                    out_buffer = ctypes.create_string_buffer(b'\xAA' * real_out_buffer_len, real_out_buffer_len)

                    ret_val = cobsr_wrapper.encode_cfunc(out_buffer, out_buffer_len, test_bytes, len(test_bytes))
                    actual_encoded = out_buffer[:ret_val.out_len]

                    # Check that the output length is never larger than the output buffer size
                    self.assertTrue(ret_val.out_len <= out_buffer_len)

                    # Check that the function never writes bytes past the end of the buffer
                    self.assertEqual(out_buffer[out_buffer_len:], b'\xAA' * (real_out_buffer_len - out_buffer_len))

                    # Check that the function never writes byte past where is claims to have written
                    # (not strictly a requirement, but a nice-to-have if possible)
#                    self.assertEqual(out_buffer[ret_val.out_len:], b'\xAA' * (real_out_buffer_len - ret_val.out_len))

                    if out_buffer_len < len(expected_encoded_string):
                        # Check that the output buffer overflow error status is flagged
                        self.assertTrue((ret_val.status & cobsr_wrapper.CobsrEncodeStatus.OUT_BUFFER_OVERFLOW) != 0)

                        # Check that the output buffer is filled up to the limit
                        # (not strictly a requirement, but a nice-to-have if possible)
#                        self.assertEqual(ret_val.out_len, out_buffer_len)

                        # Check that the data that _is_ put in the output buffer is valid
                        # (not strictly a requirement, but a nice-to-have if possible)
                        actual_decoded = cobsr_wrapper.decode(actual_encoded)
                        self.assertTrue(test_bytes.startswith(actual_decoded),
                                        "for %s, encode buffer length %d, got %s" % (repr(test_string), out_buffer_len, repr(actual_decoded)))

                    if (out_buffer_len >= len(expected_encoded_string) + 1 or
                        out_buffer_len >= cobsr_wrapper.encode_size_max(len(test_bytes))):

                        # Check that the output buffer overflow error status is NOT flagged
                        self.assertTrue((ret_val.status & cobsr_wrapper.CobsrEncodeStatus.OUT_BUFFER_OVERFLOW) == 0)

                    if (ret_val.status & cobsr_wrapper.CobsrEncodeStatus.OUT_BUFFER_OVERFLOW) == 0:
                        # Check that the correct encoded value is returned
                        self.assertEqual(actual_encoded, expected_encoded_string)

            except AssertionError:
                print >> sys.stderr, "For test string %s" % repr(test_string)
                raise

    def test_decode_output_overflow(self):
        for (expected_decoded_string, encoded_string) in self.predefined_encodings:
            try:
                # Sanity check that the expected decode string length is not
                # bigger than the computed maximum.
                self.assertTrue(len(expected_decoded_string) <= cobsr_wrapper.decode_size_max(len(encoded_string)))

                real_out_buffer_len = cobsr_wrapper.decode_size_max(len(encoded_string)) + 100

                for out_buffer_len in range(0, real_out_buffer_len + 1):

                    out_buffer = ctypes.create_string_buffer(b'\xAA' * real_out_buffer_len, real_out_buffer_len)

                    ret_val = cobsr_wrapper.decode_cfunc(out_buffer, out_buffer_len, encoded_string, len(encoded_string))
                    actual_decoded = out_buffer[:ret_val.out_len]

                    # Check that the output length is never larger than the output buffer size
                    self.assertTrue(ret_val.out_len <= out_buffer_len)

                    # Check that the function never writes bytes past the end of the buffer
                    self.assertEqual(out_buffer[out_buffer_len:], b'\xAA' * (real_out_buffer_len - out_buffer_len))

                    # Check that the function never writes byte past where is claims to have written
                    # (not strictly a requirement, but a nice-to-have if possible)
                    self.assertEqual(out_buffer[ret_val.out_len:], b'\xAA' * (real_out_buffer_len - ret_val.out_len))

                    if out_buffer_len < len(expected_decoded_string):
                        # Check that the output buffer overflow error status is flagged
                        self.assertTrue((ret_val.status & cobsr_wrapper.CobsrDecodeStatus.OUT_BUFFER_OVERFLOW) != 0)

                        # Check that the output buffer is filled up to the limit
                        # (not strictly a requirement, but a nice-to-have if possible)
                        self.assertEqual(ret_val.out_len, out_buffer_len)

                        # Check that the data that _is_ put in the output buffer is valid
                        # (not strictly a requirement, but a nice-to-have if possible)
                        self.assertTrue(expected_decoded_string.startswith(actual_decoded),
                                        "for %s, decode buffer length %d, got %s" % (repr(expected_decoded_string), out_buffer_len, repr(actual_decoded)))

                    if out_buffer_len >= len(expected_decoded_string):
                        # Check that the output buffer overflow error status is NOT flagged
                        self.assertTrue((ret_val.status & cobsr_wrapper.CobsrDecodeStatus.OUT_BUFFER_OVERFLOW) == 0)

                    if (ret_val.status & cobsr_wrapper.CobsrDecodeStatus.OUT_BUFFER_OVERFLOW) == 0:
                        # Check that the correct encoded value is returned
                        self.assertEqual(out_buffer[:ret_val.out_len], expected_decoded_string)

            except AssertionError:
                print >> sys.stderr, "For test string %s" % repr(expected_decoded_string)
                raise


def runtests():
    unittest.main()


if __name__ == '__main__':
    runtests()
