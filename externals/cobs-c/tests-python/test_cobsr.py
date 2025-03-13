#!/usr/bin/python3
"""
Consistent Overhead Byte Stuffing/Reduced (COBS/R)

Unit Tests

This version is for Python 3.x.
"""

from array import array
import random
import unittest

#from .. import cobsr as cobsr
#from ..cobsr import _cobsr_py as cobsr
import cobsr_wrapper as cobsr


def infinite_non_zero_generator():
    while True:
        for i in range(1,50):
            for j in range(1,256, i):
                yield j

def non_zero_generator(length):
    non_zeros = infinite_non_zero_generator()
    for i in range(length):
        yield next(non_zeros)

def non_zero_bytes(length):
    return b''.join(bytes([i]) for i in non_zero_generator(length))


class PredefinedEncodingsTests(unittest.TestCase):
    predefined_encodings = [
        [ b"",                                  b"\x01"                                                         ],
        [ b"\x01",                              b"\x02\x01"                                                     ],
        [ b"\x02",                              b"\x02"                                                         ],
        [ b"\x03",                              b"\x03"                                                         ],
        [ b"\x7E",                              b"\x7E"                                                         ],
        [ b"\x7F",                              b"\x7F"                                                         ],
        [ b"\x80",                              b"\x80"                                                         ],
        [ b"\xD5",                              b"\xD5"                                                         ],
        [ b"\xFE",                              b"\xFE"                                                         ],
        [ b"\xFF",                              b"\xFF"                                                         ],
        [ b"a\x02",                             b"\x03a\x02"                                                    ],
        [ b"a\x03",                             b"\x03a"                                                        ],
        [ b"a\xFF",                             b"\xFFa"                                                        ],
        [ b"\x05\x04\x03\x02\x01",              b"\x06\x05\x04\x03\x02\x01"                                     ],
        [ b"12345",                             b"51234"                                                        ],
        [ b"12345\x00\x04\x03\x02\x01",         b"\x0612345\x05\x04\x03\x02\x01"                                ],
        [ b"12345\x006789",                     b"\x06123459678"                                                ],
        [ b"\x0012345\x006789",                 b"\x01\x06123459678"                                            ],
        [ b"12345\x006789\x00",                 b"\x0612345\x056789\x01"                                        ],
        [ b"\x00",                              b"\x01\x01"                                                     ],
        [ b"\x00\x00",                          b"\x01\x01\x01"                                                 ],
        [ b"\x00\x00\x00",                      b"\x01\x01\x01\x01"                                             ],
        [ bytes(bytearray(range(1, 254))),      bytes(b"\xfe" + bytearray(range(1, 254)))                       ],
        [ bytes(bytearray(range(1, 255))),      bytes(b"\xff" + bytearray(range(1, 255)))                       ],
        [ bytes(bytearray(range(1, 256))),      bytes(b"\xff" + bytearray(range(1, 255)) + b"\xff")             ],
        [ bytes(bytearray(range(0, 256))),      bytes(b"\x01\xff" + bytearray(range(1, 255)) + b"\xff")         ],
        [ bytes(bytearray(range(2, 256))),      bytes(b"\xff" + bytearray(range(2, 255)))                       ],
    ]

    def test_predefined_encodings(self):
        for (test_string, expected_encoded_string) in self.predefined_encodings:
            encoded = cobsr.encode(test_string)
            self.assertEqual(encoded, expected_encoded_string)

    def test_decode_predefined_encodings(self):
        for (test_string, expected_encoded_string) in self.predefined_encodings:
            decoded = cobsr.decode(expected_encoded_string)
            self.assertEqual(test_string, decoded)


class PredefinedDecodeErrorTests(unittest.TestCase):
    decode_error_test_strings = [
        b"\x00",
        b"\x051234\x00",
        b"\x0512\x004",
    ]

    def test_predefined_decode_error(self):
        for test_encoded in self.decode_error_test_strings:
            with self.assertRaises(cobsr.DecodeError):
                cobsr.decode(test_encoded)


class ZerosTest(unittest.TestCase):
    def test_zeros(self):
        for length in range(520):
            test_string = b'\x00' * length
            encoded = cobsr.encode(test_string)
            expected_encoded = b'\x01' * (length + 1)
            self.assertEqual(encoded, expected_encoded, "encoding zeros failed for length %d" % length)
            decoded = cobsr.decode(encoded)
            self.assertEqual(decoded, test_string, "decoding zeros failed for length %d" % length)


class NonZerosTest(unittest.TestCase):
    def simple_encode_non_zeros_only(self, in_bytes):
        out_list = []
        for i in range(0, len(in_bytes), 254):
            data_block = in_bytes[i: i+254]
            out_list.append(bytes([ len(data_block) + 1 ]))
            out_list.append(data_block)
        return b''.join(out_list)

    def cobsr_encode_final_non_zeros_only(self, in_bytes):
        out_list = []
        for i in range(0, len(in_bytes), 254):
            data_block = in_bytes[i: i+254]
            out_list.append(bytes([ len(data_block) + 1 ]))
            out_list.append(data_block)
        last_block = out_list[-1]
        last_char_value = last_block[-1]
        if last_char_value >= len(last_block) + 1:
            del(out_list[-2:])
            out_list.append(bytes([ last_char_value ]))
            out_list.append(last_block[:-1])
        return b''.join(out_list)

    def test_non_zeros(self):
        for length in range(1, 1000):
            test_string = non_zero_bytes(length)
            encoded = cobsr.encode(test_string)
            expected_encoded = self.cobsr_encode_final_non_zeros_only(test_string)
            self.assertEqual(encoded, expected_encoded,
                             "encoded != expected_encoded for length %d\nencoded: %s\nexpected_encoded: %s" %
                             (length, repr(encoded), repr(expected_encoded)))

    def test_non_zeros_and_trailing_zero(self):
        for length in range(1, 1000):
            non_zeros_string = non_zero_bytes(length)
            test_string = non_zeros_string + b'\x00'
            encoded = cobsr.encode(test_string)
            if (len(non_zeros_string) % 254) == 0:
                expected_encoded = self.simple_encode_non_zeros_only(non_zeros_string) + b'\x01\x01'
            else:
                expected_encoded = self.simple_encode_non_zeros_only(non_zeros_string) + b'\x01'
            self.assertEqual(encoded, expected_encoded,
                             "encoded != expected_encoded for length %d\nencoded: %s\nexpected_encoded: %s" %
                             (length, repr(encoded), repr(expected_encoded)))


class RandomDataTest(unittest.TestCase):
    NUM_TESTS = 5000
    MAX_LENGTH = 2000

    def test_random(self):
        try:
            for _test_num in range(self.NUM_TESTS):
                length = random.randint(0, self.MAX_LENGTH)
                test_string = bytes(random.randint(0,255) for x in range(length))
                encoded = cobsr.encode(test_string)
                self.assertTrue(b'\x00' not in encoded,
                                "encoding contains zero byte(s):\noriginal: %s\nencoded: %s" % (repr(test_string), repr(encoded)))
                self.assertTrue(len(encoded) <= len(test_string) + 1 + (len(test_string) // 254),
                                "encoding too big:\noriginal: %s\nencoded: %s" % (repr(test_string), repr(encoded)))
                decoded = cobsr.decode(encoded)
                self.assertEqual(decoded, test_string,
                                 "encoding and decoding random data failed:\noriginal: %s\ndecoded: %s" % (repr(test_string), repr(decoded)))
        except KeyboardInterrupt:
            pass


class InputTypesTest(unittest.TestCase):
    predefined_encodings = [
        [ b"",                                  b"\x01"                                                         ],
        [ b"\x01",                              b"\x02\x01"                                                     ],
        [ b"\x02",                              b"\x02"                                                         ],
        [ b"\x7F",                              b"\x7F"                                                         ],
        [ b"\x80",                              b"\x80"                                                         ],
        [ b"\xD5",                              b"\xD5"                                                         ],
        [ b"1",                                 b"1"                                                            ],
        [ b"\x05\x04\x03\x02\x01",              b"\x06\x05\x04\x03\x02\x01"                                     ],
        [ b"12345",                             b"51234"                                                        ],
        [ b"12345\x00\x04\x03\x02\x01",         b"\x0612345\x05\x04\x03\x02\x01"                                ],
        [ b"12345\x006789",                     b"\x06123459678"                                                ],
    ]

    def test_unicode_string(self):
        """Test that Unicode strings are not encoded or decoded.
        They should raise a TypeError."""
        for (test_string, expected_encoded_string) in self.predefined_encodings:
            unicode_test_string = test_string.decode('latin')
            with self.assertRaises(TypeError):
                cobsr.encode(unicode_test_string)
            unicode_encoded_string = expected_encoded_string.decode('latin')
            with self.assertRaises(TypeError):
                cobsr.decode(unicode_encoded_string)

    def test_bytearray(self):
        """Test that bytearray objects can be encoded or decoded."""
        for (test_string, expected_encoded_string) in self.predefined_encodings:
            bytearray_test_string = bytearray(test_string)
            encoded = cobsr.encode(bytearray_test_string)
            self.assertEqual(encoded, expected_encoded_string)
            bytearray_encoded_string = bytearray(expected_encoded_string)
            decoded = cobsr.decode(bytearray_encoded_string)
            self.assertEqual(decoded, test_string)

    def test_array_of_bytes(self):
        """Test that array of bytes objects (array('B', ...)) can be encoded or decoded."""
        for (test_string, expected_encoded_string) in self.predefined_encodings:
            array_test_string = array('B', test_string)
            encoded = cobsr.encode(array_test_string)
            self.assertEqual(encoded, expected_encoded_string)
            array_encoded_string = array('B', expected_encoded_string)
            decoded = cobsr.decode(array_encoded_string)
            self.assertEqual(decoded, test_string)

    def test_array_of_half_words(self):
        """Test that array of half-word objects (array('H', ...)) are not encoded or decoded.
        They should raise a BufferError."""
        # Array typecodes for types with size greater than 1.
        # Don't include 'u' for Unicode because it expects a Unicode initialiser.
        typecodes = [ 'H', 'h', 'i', 'I', 'l', 'L', 'f', 'd' ]
        for typecode in typecodes:
            array_test_string = array(typecode, [ 49, 50, 51, 52, 53 ])
            with self.assertRaises(BufferError):
                cobsr.encode(array_test_string)
            array_encoded_string = array(typecode, [6, 49, 50, 51, 52, 53 ])
            with self.assertRaises(BufferError):
                cobsr.decode(array_encoded_string)


def runtests():
    unittest.main()


if __name__ == '__main__':
    runtests()
