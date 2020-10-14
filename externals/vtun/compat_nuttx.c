#include <sys/types.h>
#include <mbedtls/md5.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/blowfish.h>
#include <syslog.h>
#include <sys/uio.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "vtun.h"
#include "lib.h"
#include "linkfd.h"

#include "compat_nuttx.h"

static int session_established = 0;

void openlog(const char *ident, int option, int facility)
{}

void closelog(void)
{}

#ifndef CONFIG_PSEUDOFS_SOFTLINKS
int link(const char *path1, const char *path2)
{
  return 0;
}
#endif

int getpriority(int which, id_t who)
{
  return 0;
}

int setpriority(int which, id_t who, int prio)
{
  session_established = (prio == LINKFD_PRIO) ? 1 : 0;
  return 0;
}

unsigned char *MD5(char *msg, size_t msg_len, unsigned char* out)
{
  static mbedtls_md5_context ctx;
  static unsigned char tmp[16];

  if (out == NULL)
    {
      out = tmp;
    }

  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts(&ctx);
  mbedtls_md5_update(&ctx, msg, msg_len);
  mbedtls_md5_finish(&ctx, out);
  mbedtls_md5_free(&ctx);

  return out;
}

static int vtun_entropy(void *rng_state, unsigned char *output, size_t len)
{
  struct timespec ts;
  static const size_t NSEC_LEN = sizeof(ts.tv_nsec);
  size_t offset = 0;
  int ret;

  if (output == NULL)
    {
      return -1;
    }

  while (offset < len)
    {
      ret = clock_gettime(CLOCK_REALTIME, &ts);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"clock_gettime() error: %d", errno);
          return -1;
        }

      if (NSEC_LEN + offset <= len)
        {
          memcpy(&output[offset], &ts.tv_nsec, NSEC_LEN);
          offset += NSEC_LEN;
        }
      else
        {
          memcpy(&output[offset], &ts.tv_nsec, len - offset);
          offset += len - offset;
        }
    }
  return 0;
}

void RAND_bytes(char *buf, size_t buf_len)
{
  mbedtls_ctr_drbg_context ctx;
  int ret;

  if (buf == NULL)
    {
      return;
    }

  mbedtls_ctr_drbg_init(&ctx);
  mbedtls_ctr_drbg_seed(&ctx, vtun_entropy, NULL, NULL, 0);
  if (ret != 0)
    {
      vtun_syslog(LOG_ERR,"mbedtls_ctr_drbg_seed() returned -0x%04X", -ret);
    }
  mbedtls_ctr_drbg_random(&ctx, buf, buf_len);
  mbedtls_ctr_drbg_free(&ctx);
}

void BF_set_key(BF_KEY *key, int len, const unsigned char *data)
{
  mbedtls_blowfish_init(key);
  mbedtls_blowfish_setkey(key, data, len * 8);
}

void BF_ecb_encrypt(const unsigned char *in, unsigned char *out,
        BF_KEY *key, int enc)
{
  int ret;
  int mode;

  if (in == NULL || out == NULL || key == NULL)
    {
      return;
    }

  if (enc == BF_ENCRYPT)
    {
      mode = MBEDTLS_BLOWFISH_ENCRYPT;
    }
  else
    {
      mode = MBEDTLS_BLOWFISH_DECRYPT;
    }

  ret = mbedtls_blowfish_crypt_ecb(key, mode, in, out);
  if (ret != 0)
    {
      vtun_syslog(LOG_ERR, "mbedtls_blowfish_crypt_ecb failed.");
    }
}

void EVP_CIPHER_CTX_init(EVP_CIPHER_CTX *a)
{
  mbedtls_cipher_init(a);
}
int EVP_CIPHER_CTX_cleanup(EVP_CIPHER_CTX *a)
{
  mbedtls_cipher_free(a);
}

int EVP_EncryptInit_ex(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *type,
        ENGINE *impl, unsigned char *key, unsigned char *iv)
{
  int ret;
  size_t keybitlen;

  if (ctx == NULL)
    {
      return 0;
    }

  if (type != NULL)
    {
      ret = mbedtls_cipher_setup(ctx,
                                 mbedtls_cipher_info_from_type(*type));
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_setup() returned -0x%04X", -ret);
          return 0;
        }
    }

  if (key != NULL)
    {
      keybitlen = mbedtls_cipher_get_key_bitlen(ctx);

      ret = mbedtls_cipher_setkey(ctx, key,
                                  keybitlen, MBEDTLS_ENCRYPT);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_setkey() returned -0x%04X", -ret);
          return 0;
        }
    }

  if (iv != NULL)
    {
      size_t ivlen = 0;

      ivlen = mbedtls_cipher_get_iv_size(ctx);
      if (ivlen != 0)
        {
          ret = mbedtls_cipher_set_iv(ctx, iv, ivlen);
          if (ret != 0)
            {
              vtun_syslog(LOG_ERR,"mbedtls_cipher_set_iv() returned -0x%04X", -ret);
              return 0;
            }
        }

      ret = mbedtls_cipher_reset(ctx);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_reset() returned -0x%04X", -ret);
          return 0;
        }

    }

  return 1;
}

int EVP_DecryptInit_ex(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *type,
        ENGINE *impl, unsigned char *key, unsigned char *iv)
{
  int ret;
  size_t keybitlen;

  if (ctx == NULL)
    {
      return 0;
    }

  if (type != NULL)
    {
      ret = mbedtls_cipher_setup(ctx,
                                 mbedtls_cipher_info_from_type(*type));
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_setup() returned -0x%04X", -ret);
          return 0;
        }
    }

  if (key != NULL)
    {
      keybitlen = mbedtls_cipher_get_key_bitlen(ctx);

      ret = mbedtls_cipher_setkey(ctx, key,
                                  keybitlen, MBEDTLS_DECRYPT);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_setkey() returned -0x%04X", -ret);
          return 0;
        }
    }

  if (iv != NULL)
    {
      size_t ivlen = 0;

      ivlen = mbedtls_cipher_get_iv_size(ctx);
      if (ivlen != 0)
        {
          ret = mbedtls_cipher_set_iv(ctx, iv, ivlen);
          if (ret != 0)
            {
              vtun_syslog(LOG_ERR,"mbedtls_cipher_set_iv() returned -0x%04X", -ret);
              return 0;
            }
        }

      ret = mbedtls_cipher_reset(ctx);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_reset() returned -0x%04X", -ret);
          return 0;
        }
    }

  return 1;
}

int EVP_CIPHER_CTX_set_padding(EVP_CIPHER_CTX *x, int padding)
{
  int ret;

  if (padding == 0)
    {
      ret = mbedtls_cipher_set_padding_mode(x, MBEDTLS_PADDING_NONE);
      if (ret != 0) {
        vtun_syslog(LOG_ERR,"mbedtls_cipher_set_padding_mode() returned -0x%04X", -ret);
      }
      else
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_set_padding_mode() succeeded");
        }
    }
   return 1;
}

int EVP_EncryptUpdate(EVP_CIPHER_CTX *ctx, unsigned char *out,
         int *outl, const unsigned char *in, int inl)
{
  //return 1 for success and 0 for failure.
  int ret;
  mbedtls_cipher_mode_t mode;

  mode = mbedtls_cipher_get_cipher_mode(ctx);
  if (mode == MBEDTLS_MODE_ECB)
    {
      int offset = 0;
      size_t blocksize = 0;

      blocksize = mbedtls_cipher_get_block_size(ctx);

      while (offset < inl)
        {
          size_t out_len = 0;
          ret = mbedtls_cipher_update(ctx, in + offset, blocksize, out + offset, &out_len);
          if (ret != 0)
            {
              vtun_syslog(LOG_ERR,"mbedtls_cipher_update() returned -0x%04X L:%d", -ret, __LINE__);
              return 0;
            }
          offset += blocksize;
        }
    }
  else
    {
      size_t finlen = 0;

      ret = mbedtls_cipher_update(ctx, in, inl, out, (size_t*) outl);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_update() returned -0x%04X L:%d", -ret, __LINE__);
          return 0;
        }

      ret = mbedtls_cipher_finish(ctx, out + *outl, &finlen);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_finish() returned -0x%04X L:%d", -ret, __LINE__);
          return 0;
        }

      ret = mbedtls_cipher_reset(ctx);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR, "mbedtls_cipher_reset() returned -0x%04X L:%d", -ret, __LINE__);
          return 0;
        }

      *outl += finlen;
    }
  return 1;
}

int EVP_DecryptUpdate(EVP_CIPHER_CTX *ctx, unsigned char *out,
         int *outl, const unsigned char *in, int inl)
{
  int ret;
  mbedtls_cipher_mode_t mode;

  mode = mbedtls_cipher_get_cipher_mode(ctx);
  if (mode == MBEDTLS_MODE_ECB)
    {
      int offset = 0;
      size_t blocksize = 0;

      blocksize = mbedtls_cipher_get_block_size(ctx);

      while (offset < inl)
        {
          size_t out_len = 0;
          ret = mbedtls_cipher_update(ctx, in + offset, blocksize, out + offset, &out_len);
          if (ret != 0) {
            vtun_syslog(LOG_ERR,"mbedtls_cipher_update() returned -0x%04X L:%d", -ret, __LINE__);
            return 0;
          }
          offset += blocksize;
        }
    }
  else
    {
      size_t finlen = 0;

      ret = mbedtls_cipher_update(ctx, in, inl, out, (size_t*) outl);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_update() returned -0x%04X L:%d", -ret, __LINE__);
          return 0;
        }

      ret = mbedtls_cipher_finish(ctx, out + *outl, &finlen);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_finish() returned -0x%04X L:%d", -ret, __LINE__);
          return 0;
        }

      ret = mbedtls_cipher_reset(ctx);
      if (ret != 0)
        {
          vtun_syslog(LOG_ERR,"mbedtls_cipher_reset() returned -0x%04X L:%d", -ret, __LINE__);
          return 0;
        }

      *outl += finlen;
    }
  return 1;
}

int EVP_CIPHER_CTX_set_key_length(EVP_CIPHER_CTX *x, int keylen)
{
  return 0;
}

const EVP_CIPHER *EVP_aes_256_ecb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_256_ECB;
  return &cipher;
}

const EVP_CIPHER *EVP_aes_128_ecb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_128_ECB;
  return &cipher;
}

const EVP_CIPHER *EVP_bf_ecb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_BLOWFISH_ECB;
  return &cipher;
}

const EVP_CIPHER *EVP_aes_256_ofb(void)
{
  //static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_256_OFB;
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_NONE;
  vtun_syslog(LOG_ERR,"AES-256-OFB is not supported.");
  return &cipher;
}

const EVP_CIPHER *EVP_aes_256_cfb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_256_CFB128;
  return &cipher;
}

const EVP_CIPHER *EVP_aes_256_cbc(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_256_CBC;
  return &cipher;
}

const EVP_CIPHER *EVP_aes_128_ofb(void)
{
  //static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_128_OFB;
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_NONE;
  vtun_syslog(LOG_ERR,"AES-128-OFB is not supported.");
  return &cipher;
}

const EVP_CIPHER *EVP_aes_128_cfb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_128_CFB128;
  return &cipher;
}

const EVP_CIPHER *EVP_aes_128_cbc(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_AES_128_CBC;
  return &cipher;
}

const EVP_CIPHER *EVP_bf_ofb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_NONE;
  vtun_syslog(LOG_ERR,"Blowfish-256-OFB is not supported.");
  return &cipher;
}

const EVP_CIPHER *EVP_bf_cfb(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_BLOWFISH_CFB64;
  return &cipher;
}

const EVP_CIPHER *EVP_bf_cbc(void)
{
  static const EVP_CIPHER cipher = MBEDTLS_CIPHER_BLOWFISH_CBC;
  return &cipher;
}

pid_t setsid(void)
{
  return (pid_t)-1;
}

ssize_t vtun_udp_readv(int fd, const struct iovec *iov, int iovcnt)
{
  static char buf[sizeof(short) + VTUN_FRAME_SIZE + VTUN_FRAME_OVERHEAD];

  ssize_t buf_len = 0;
  size_t recv_len;
  size_t offset = 0;
  int i;

  recv_len = read(fd, buf, sizeof(buf));
  if (recv_len < 0)
    {
      // read error.
      return recv_len;
    }

  for (i = 0; i < iovcnt; i++)
    {
      if (offset + iov[i].iov_len <= recv_len)
        {
          memcpy(iov[i].iov_base, &buf[offset], iov[i].iov_len);
        }
      else
        {
          memcpy(iov[i].iov_base, &buf[offset], recv_len - offset);
          break;
        }
      offset += iov[i].iov_len;
    }

  return recv_len;
}

int vtun_session_established(void)
{
  return session_established;
}

