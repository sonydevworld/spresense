examples/lte_lwm2m
^^^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and CoAP access from the LwM2M server.
  Original sample file is :
    externals/lwm2m/wakaama/examples/client/lwm2mclient.c

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  This application can be used by lte_lwm2m default config.

  $ ./tools/config.py examples/lte_lwm2m
  $ make

  Execute under nsh:

  Type 'lte_lwm2m <options>' on nsh like this.
  nsh> lte_lwm2m -h selftest.iot.nokia.com -p 5683 -4

