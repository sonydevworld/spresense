AUTOMAKE_OPTIONS = foreign subdir-objects
ACLOCAL_AMFLAGS = -I m4

COMMON = src/cwebsocket/utf8.h src/cwebsocket/utf8.c src/cwebsocket/common.h src/cwebsocket/common.c
ECHO_CLIENT = src/cwebsocket/subprotocol/echo/echo_client.h src/cwebsocket/subprotocol/echo/echo_client.c
ECHO_SERVER = src/cwebsocket/subprotocol/echo/echo_server.h src/cwebsocket/subprotocol/echo/echo_server.c
WEBSOCKET_CLIENT = src/cwebsocket/client.h src/cwebsocket/client.c
WEBSOCKET_SERVER = src/cwebsocket/server.h src/cwebsocket/server.c

bin_PROGRAMS = websocket-client websocket-server websocket-testsuite
websocket_client_SOURCES = $(ECHO_CLIENT) $(COMMON) $(WEBSOCKET_CLIENT) src/websocket-client.c
websocket_server_SOURCES = $(ECHO_SERVER) $(COMMON) $(WEBSOCKET_SERVER) src/websocket-server.c
websocket_testsuite_SOURCES = $(COMMON) $(WEBSOCKET_CLIENT) src/websocket-testsuite.c

noinst_LIBRARIES = libwsclient.a libwsserver.a
libwsclient_a_SOURCES = $(ECHO_CLIENT) $(COMMON) $(WEBSOCKET_CLIENT)
libwsserver_a_SOURCES = $(ECHO_SERVER) $(COMMON) $(WEBSOCKET_SERVER)

so:
	gcc -shared -fPIC $(ECHO_CLIENT) $(WEBSOCKET_CLIENT) -o libwsclient.so
	gcc -shared -fPIC $(ECHO_SERVER) $(WEBSOCKET_SERVER) -o libwsserver.so

clean:
	rm -rf compile ltmain.sh libtool config.sub config.guess libwsclient.* libwsserver.* .gitignore vgcore.* autoscan.log config.h.in config.h config.cache configure install-sh aclocal.m4 autom4te.cache/ config.log config.status Debug/ RPI\ Debug Release depcomp .deps/ m4/ Makefile Makefile.in missing stamp-h1 *.o src/*.o *~ src/.deps/ src/.dirstamp src/cwebsocket/.deps/ src/cwebsocket/.dirstamp src/cwebsocket/client.o src/cwebsocket/common.o src/cwebsocket/server.o src/cwebsocket/subprotocol/echo/.deps/ src/cwebsocket/subprotocol/echo/.dirstamp src/cwebsocket/subprotocol/echo/echo_client.o src/cwebsocket/subprotocol/echo/echo_server.o src/cwebsocket/utf8.o websocket-client websocket-server websocket-testsuite
