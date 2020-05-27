examples/lte_http_get
^^^^^^^^^^^^^^^^^^^^^
******************************************************************************
* Description
******************************************************************************

  This application is a sample that connect to the LTE network
  and obtain the requested file from the HTTP server using the GET method.
  The obtained file is output to standard output.

  Supported LTE modem is ALT1250.

******************************************************************************
* Build kernel and SDK
******************************************************************************

  $ ./tools/config.py examples/lte_http_get

    The web client also support HTTPS.
    HTTPS can be used by adding the mbed TLS configuration as follows:

    $ ./tools/config.py examples/lte_http_get feature/externals_mbedtls
      or                                      ^^^^^^^^^^^^^^^^^^^^^^^^^
    $ ./tools/config.py examples/lte_http_get feature/lte_stub_mbedtls
                                              ^^^^^^^^^^^^^^^^^^^^^^^^

    Recommended to use synchronous LTE API.
    Default configuration use synchronous LTE API.
    Asynchronous LTE API can be used by selecting a configuration as follows:

    $ tools/config.py -m
      Application Configuration
        Spresense SDK
          Example
            [*] HTTP GET method using LTE example
              API call type Selection --->
                (X) Use Asynchronous API

    <Setting APN>
    Set the APN of the carrier according to the your environment.

    $ tools/config.py -m
      Application Configuration
        Spresense SDK
          Example
            [*] HTTP GET method using LTE example
              Access Point Name
              IP type Selection
              Authentication type Selection
              Username used for authentication
              Password used for authentication

  $ make

******************************************************************************
* Execute Example
******************************************************************************

  Execute under nsh:

  Type 'lte_http_get <url>' on nsh like this.
  nsh> lte_http_get http://example.com/index.html
    or
  nsh> lte_http_get https://example.com/index.html

