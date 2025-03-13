
#######################################
# The list of libraries we are building seperated by spaces.
# The 'lib_' indicates that these build products will be installed
# in the $(libdir) directory. For example /usr/lib
lib_LTLIBRARIES = lib@PACKAGE_NAME@.la

#######################################
# Build information for each library

library_include_at_parsedir=$(includedir)/@PACKAGE_NAME@
library_include_at_parse_HEADERS = cobs.h cobsr.h
lib@PACKAGE_NAME@_la_SOURCES = cobs.c cobsr.c
lib@PACKAGE_NAME@_la_LDFLAGS = -version-info @LIB_SO_VERSION@

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = @PACKAGE_NAME@.pc


#######################################
# Tests

TESTS = test-cobs test-cobsr

check_PROGRAMS = test-cobs test-cobsr

test_cobs_SOURCES = tests-c/test-cobs.c tests-c/unity/unity.c
test_cobs_CFLAGS = -I${srcdir}/tests-c/unity
test_cobs_LDADD = lib@PACKAGE_NAME@.la

test_cobsr_SOURCES = tests-c/test-cobsr.c tests-c/unity/unity.c
test_cobsr_CFLAGS = -I${srcdir}/tests-c/unity
test_cobsr_LDADD = lib@PACKAGE_NAME@.la
