############################################################################
# externals/mbedtls/library/Makefile
#
#   Copyright 2018 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

BIN = libmbedtls$(LIBEXT)

CSRCS =  aes.c aesni.c arc4.c asn1parse.c asn1write.c
CSRCS += base64.c bignum.c blowfish.c
CSRCS += camellia.c ccm.c certs.c cipher.c cipher_wrap.c ctr_drbg.c
CSRCS += debug.c des.c dhm.c
CSRCS += ecdh.c ecdsa.c ecjpake.c ecp.c ecp_curves.c entropy.c entropy_poll.c error.c
CSRCS += gcm.c havege.c hmac_drbg.c
CSRCS += md2.c md4.c md5.c md.c md_wrap.c memory_buffer_alloc.c net_sockets.c oid.c
CSRCS += padlock.c pem.c pk.c pkcs11.c pkcs12.c pkcs5.c pkparse.c pk_wrap.c pkwrite.c platform.c
CSRCS += ripemd160.c  rsa.c
CSRCS += sha1.c sha256.c sha512.c ssl_cache.c ssl_ciphersuites.c ssl_cli.c ssl_cookie.c ssl_srv.c ssl_ticket.c ssl_tls.c
CSRCS += threading.c timing.c version.c version_features.c
CSRCS += x509.c x509_create.c x509_crl.c x509_crt.c x509_csr.c x509write_crt.c x509write_csr.c xtea.c 

VPATH = library
ROOTDEPPATH = --dep-path library

include $(APPDIR)/Application.mk
