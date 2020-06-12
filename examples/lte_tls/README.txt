examples/lte_tls
^^^^^^^^^^^^^^^^

  This application is a sample of TLS data communication over LTE network.
  In this sample,
     Connect to LTE network,
     establish TLS connection by using mbedTLS over LTE connection,
     send HTTP POST, and receive HTTP response over TLS connection,
     and disconnect.

  The procedure for LTE connection is the same as one of lte_http_get example.

  Build kernel and SDK:

  This application can be used by lte_tls default config.

  $ ./tools/config.py examples/lte_tls
  $ make

  Execute under nsh:

  Type 'lte_tls <url that start from "https://">' on nsh like this.
  nsh> lte_tls https://example.com/index.html

