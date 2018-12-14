examples/tlstest
^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and check mbedTLS functions using badssl.com.

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  $ make buildkernel KERNCONF=release

  This application can be used by tlstest default config.

  $ ./tools/config.py examples/tlstest
  $ make

  Execute under nsh:

  Type 'tlstest <number>' on nsh like this.
  nsh> tlstest 10

  Some test sequense is exist.
    Library test     : 00 - 04
    Certificate test : 10 - 29
    ClientCert test  : 30 - 31
    CipherSuite test : 32 - 39
    KeyExchange test : 40 - 46
    Protocol test    : 47 - 48
    Transparent test : 50
    Upgrade test     : 51 - 55
    UI test          : 56 - 58
    KnownBad test    : 60 - 68
    SSL LABS test    : 70 - 77

  Some test sequence will not satisfy because of mbedTLS implementation.
    15, 16, 19, 24, 25, 31, 44, 45

