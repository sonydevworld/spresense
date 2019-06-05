examples/lte_http_get
^^^^^^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and obtain the requested file from the HTTP server using the GET method.
  The obtained file is output to standard output.

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  $ make buildkernel KERNCONF=release

  This application can be used by lte_http_get default config.

  $ ./tools/config.py examples/lte_http_get
  $ make

  The web client also support HTTPS.
  HTTPS can be used by adding the mbed TLS configuration as follows:

  $ ./tools/config.py examples/lte_http_get feature/externals_mbedtls
    or                                      ^^^^^^^^^^^^^^^^^^^^^^^^^
  $ ./tools/config.py examples/lte_http_get feature/lte_stub_mbedtls
                                            ^^^^^^^^^^^^^^^^^^^^^^^^
  Execute under nsh:

  Type 'lte_http_get <url>' on nsh like this.
  nsh> lte_http_get http://example.com/index.html
    or
  nsh> lte_http_get https://example.com/index.html

