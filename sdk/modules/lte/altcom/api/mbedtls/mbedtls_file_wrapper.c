/****************************************************************************
 * modules/lte/altcom/api/mbedtls/mbedtls_file_wrapper.c
 *
 *   Copyright 2018 Sony Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtls_load_local_file(const char *path, unsigned char **buf, size_t *len)
{
  FILE *fd;
  size_t size;
  fpos_t end_pos = 0;

  /* Check file size */

  if ((fd = fopen( path, "rb" )) == NULL)
    {
      return (MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }

  fseek(fd, 0, SEEK_END);
  if (0 != fgetpos(fd, &end_pos))
    {
      fclose(fd);
      return(MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }
  fseek(fd, 0, SEEK_SET);

  if (end_pos <= 0)
    {
      fclose(fd);
      return(MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }

  size = end_pos;

  /* Malloc buffer */

  *len = (size_t) size;
  if ((*len+1 == 0) || ((*buf = malloc(*len+1)) == NULL ))
    {
      fclose(fd);
      return(MBEDTLS_ERR_PK_ALLOC_FAILED);
    }

  /* Read certificate file */

  if (fread(*buf, 1, *len, fd) != *len)
    {
      fclose(fd);
      free(*buf);
      return(MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }

  fclose(fd);
  (*buf)[*len] = '\0';

  if (strstr((const char *)*buf, "-----BEGIN ") != NULL)
    {
      ++*len;
    }

  return(0);
}

