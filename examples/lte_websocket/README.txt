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

  Type 'lte_websocket <subprotocol> <messages>' on nsh like this.
  nsh> lte_websocket echo WebSocket Works!

  Examples:

  To test WebSocket echo subprotocol, type on nsh like this.
    nsh> lte_websocket echo WebSocket Works! 
       uri         : ws://echo.websocket.org:80/ 
       subprotocol : echo
       messages    : WebScoket Works!
 
  To test Secure WebSocket echo subprotocol, type on nsh like this.
    nsh> lte_websocket echo-ssl WebSocket Works! 
       uri         : wss://echo.websocket.org:443/
       subprotocol : echo
       messages    : WebScoket Works!
 
  To test WebSocket chat subprotocol, type on nsh like this.
    nsh> lte_websocket chat WebSocket Works! 
       uri         : ws://ruby-websockets-chat.herokuapp.com:80/
       subprotocol : chat
       handle      : WebScoket
       messages    : Works!
 
  To test Secure WebSocket chat subprotocol, type on nsh like this.
    nsh> lte_websocket chat-ssl WebSocket Works! 
       uri         : wss://ruby-websockets-chat.herokuapp.com:443/
       subprotocol : chat
       handle      : WebScoket
       messages    : Works!
 
  Appropriate uri of each subprotocol is set automatically.
  When use Secure WebSocket (wss), need to put root CA files in place.

