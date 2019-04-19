[logo]: https://github.com/jeremyhahn/cwebsocket/raw/master/websocket.png "cwebsocket"

# ![alt text][logo] cwebsocket

###### High performance websocket client/server

The goal of cwebsocket is to provide a portable, high performance websocket client/server, especially on low power embedded systems.

cwebsocket is currently in a development state. You may encounter bugs. Report them for a timely fix.

Successful tests have been conducted on the following architectures:

1. [x86](http://en.wikipedia.org/wiki/X86)
2. [x86_64](http://en.wikipedia.org/wiki/X86-64)
3. [ARM](http://en.wikipedia.org/wiki/ARM_architecture)

cwebsocket is compliant with the following standards:

1. [ANSI C](http://en.wikipedia.org/wiki/ANSI_C)
2. [POSIX](http://en.wikipedia.org/wiki/C_POSIX_library)
3. [RFC 6455](http://tools.ietf.org/html/rfc6455)

### Dependencies

1. autoconf
2. automake
3. libtool
4. libssl-dev
5. libev-dev

### Build

By defaults, cwebsocket is built with multi-threading and SSL support. 

To build, run:

	./autogen.sh
	./configure
	make
	sudo make install

To built without multi-threading:

	./configure --enable-threads=no

To build without SSL:

	./configure --enable-ssl=no

### Client

The websocket client is able to connect and exchange data with any RFC 6455 compliant server.

	./websocket-client ws://echo.websocket.org
	./websocket-client wss://echo.websocket.org

### TODO

1. More testing on various embedded devices
2. Implement pluggable sub-protocols (socketio, WAMP, custom)
3. Implement pluggable extensions on the client per RFC (section 9)
4. Get a basic websocket server developed

