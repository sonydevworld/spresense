/****************************************************************************
 * externals/vtun/compat_nuttx.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __EXTERNALS_VTUN_COMPAT_NUTTX_H__
#define __EXTERNALS_VTUN_COMPAT_NUTTX_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <mbedtls/blowfish.h>
#include <mbedtls/cipher.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* openssl */

#define BF_KEY mbedtls_blowfish_context
#define BF_ENCRYPT 1
#define BF_DECRYPT 0
#define EVP_CIPHER_CTX mbedtls_cipher_context_t
#define EVP_CIPHER mbedtls_cipher_type_t
#define ENGINE void

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

unsigned char *MD5(char *msg, size_t msg_len, char *out);
void RAND_bytes(char *buf, size_t buf_len);
void BF_set_key(BF_KEY *key, int len, const unsigned char *data);
void BF_ecb_encrypt(char *in, char *out,
        BF_KEY *key, int enc);
void EVP_CIPHER_CTX_init(EVP_CIPHER_CTX *a);
int EVP_CIPHER_CTX_cleanup(EVP_CIPHER_CTX *a);
int EVP_EncryptInit_ex(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *type,
        ENGINE *impl, char *key, char *iv);
int EVP_DecryptInit_ex(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *type,
        ENGINE *impl, char *key, char *iv);
int EVP_CIPHER_CTX_set_padding(EVP_CIPHER_CTX *x, int padding);
int EVP_EncryptUpdate(EVP_CIPHER_CTX *ctx, char *out,
         int *outl, const char *in, int inl);
int EVP_DecryptUpdate(EVP_CIPHER_CTX *ctx, char *out,
         int *outl, const char *in, int inl);
int EVP_CIPHER_CTX_set_key_length(EVP_CIPHER_CTX *x, int keylen);
const EVP_CIPHER *EVP_aes_256_ecb(void);
const EVP_CIPHER *EVP_aes_128_ecb(void);
const EVP_CIPHER *EVP_bf_ecb(void);
const EVP_CIPHER *EVP_aes_256_ofb(void);
const EVP_CIPHER *EVP_aes_256_cfb(void);
const EVP_CIPHER *EVP_aes_256_cbc(void);
const EVP_CIPHER *EVP_aes_128_ofb(void);
const EVP_CIPHER *EVP_aes_128_cfb(void);
const EVP_CIPHER *EVP_aes_128_cbc(void);
const EVP_CIPHER *EVP_bf_ofb(void);
const EVP_CIPHER *EVP_bf_cfb(void);
const EVP_CIPHER *EVP_bf_cbc(void);

int getpriority(int which, id_t who);
int setpriority(int which, id_t who, int prio);

int update_vtun_state(int enable);

pid_t setsid(void);

ssize_t vtun_udp_readv(int fd, const struct iovec *iov, int iovcnt);
#define readv(fd, iov, iovcnt) vtun_udp_readv(fd, iov, iovcnt)

#endif /* __EXTERNALS_VTUN_COMPAT_NUTTX_H__ */
