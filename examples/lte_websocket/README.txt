examples/lte_websocket
^^^^^^^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and echo access to the WebSocket server.
  Original sample file is :
    externals/websocket/cwebsocket/src/websocket-client.c

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  $ make buildkernel KERNCONF=release

  This application can be used by lte_websocket default config.

  $ ./tools/config.py examples/lte_websocket
  $ make

  Execute under nsh:

  Type 'lte_websocket <url>' on nsh like this.
  nsh> lte_websocket ws://echo.websocket.org

