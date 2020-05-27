examples/lte_websocket
^^^^^^^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and echo access to the WebSocket server.
  Original sample file is :
    externals/websocket/cwebsocket/src/websocket-client.c

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  This application can be used by lte_websocket default config.

  $ ./tools/config.py examples/lte_websocket
  $ make

  Execute under nsh:

  Type 'lte_websocket <subprotocol> <messages>' on nsh like this.
  nsh> lte_websocket echo WebSocket Works!

  Appropriate uri of each subprotocol is set automatically.
  When use Secure WebSocket (wss), need to put root CA files in place.


  2 subprotocols, echo and chat, are supported on this example.
  And for each subprotocol has each server for this example.

  Echo subprotocol server <echo.websocket.org> will reply a message as the same as sending.
  Chat subprotocol server <ruby-websockets-chat.herokuapp.com> will reply a message
  of handle and text, and will show the result at https://ruby-websockets-chat.herokuapp.com/
  by using web browser.


  Examples:

  [Echo subprotocol]

  To test WebSocket echo subprotocol, type on nsh like this.
    nsh> lte_websocket echo WebSocket Works! 

  When it works well, output following message with some debug messages.
    "exiting cwebsocket : SUCCESS"  
  When it faced on some errors, output following message with some debug messages.
    "exiting cwebsocket : FAILURE"  

  This test uses the following parameters.
    uri         : ws://echo.websocket.org:80/ 
    subprotocol : echo
    messages    : WebScoket Works!


  [Echo subprotocol (secure)]

  To test Secure WebSocket echo subprotocol, type on nsh like this.
    nsh> lte_websocket echo-ssl WebSocket Works! 

  When it works well, output following message with some debug messages.
    "exiting cwebsocket : SUCCESS"  
  When it faced on some errors, output following message with some debug messages.
    "exiting cwebsocket : FAILURE"  

  This test uses the following parameters.
    uri         : wss://echo.websocket.org:443/
    subprotocol : echo
    messages    : WebScoket Works!
 

  [Chat subprotocol]

  To test WebSocket chat subprotocol, type on nsh like this.
    nsh> lte_websocket chat WebSocket Works! 

  When it works well, output following message with some debug messages.
    "exiting cwebsocket : SUCCESS"  
  When it faced on some errors, output following message with some debug messages.
    "exiting cwebsocket : FAILURE"  

  This test uses the following parameters.
    uri         : ws://ruby-websockets-chat.herokuapp.com:80/
    subprotocol : chat
    handle      : WebScoket
    text        : Works!
 

  [Chat subprotocol (secure)]

  To test Secure WebSocket chat subprotocol, type on nsh like this.
    nsh> lte_websocket chat-ssl WebSocket Works! 

  When it works well, output following message with some debug messages.
    "exiting cwebsocket : SUCCESS"  
  When it faced on some errors, output following message with some debug messages.
    "exiting cwebsocket : FAILURE"  

  This test uses the following parameters.
    uri         : wss://ruby-websockets-chat.herokuapp.com:443/
    subprotocol : chat
    handle      : WebScoket
    text        : Works!
 
