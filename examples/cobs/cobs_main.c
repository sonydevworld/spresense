
#include <nuttx/config.h>
#include <stdio.h>
#include "cobs.h"

#define BUF_SZ (512)

static unsigned char srcbuf[BUF_SZ];
static unsigned char encbuf[BUF_SZ];
static unsigned char decbuf[BUF_SZ];

int main(int argc, char **argv)
{
  int i;
  int enclen;
  int len;
  cobs_encode_result eres;
  cobs_decode_result dres;

  if (argc < 2)
    {
      printf("Usage: nsh> %s <src strings>\n", argv[0]);
      printf("  e.x: nsh> %s str1 str2 str3\n", argv[0]);
      return -1;
    }

  printf("== Input Strings :\n");
  for (i = 1; i < argc; i++)
    {
      printf("%s\n", argv[i]);
    }

  printf("== Conbine them with '0' and display as HEX\n");
  for (i = 1, enclen = 0; i < argc; i++)
    {
      len = strlen(argv[i]);
      memcpy(&srcbuf[enclen], argv[i], len);
      enclen += len;
      if (i != (argc - 1))
        {
          srcbuf[enclen] = 0;
          enclen++;
        }
    }

  for (i = 0; i < enclen; i++)
    {
      printf("%02x ", srcbuf[i]);
    }
  printf("\n");

  eres = cobs_encode(encbuf, BUF_SZ, srcbuf, enclen);
  printf("== Encode COBS from %d bytes to %d bytes\n",
         enclen, eres.out_len);
  for (i = 0; i < eres.out_len; i++)
    {
      printf("%02x ", encbuf[i]);
    }
  printf("\n");

  dres = cobs_decode(decbuf, BUF_SZ, encbuf, eres.out_len);
  printf("== Decode COBS from %d bytes to %d bytes\n",
         eres.out_len, dres.out_len);
  for (i = 0; i < dres.out_len; i++)
    {
      printf("%02x ", decbuf[i]);
    }
  printf("\n");

  return 0;
}
