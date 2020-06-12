/****************************************************************************
 * modules/asmp/rawelf/rawelf_verify.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <nuttx/config.h>

#include <string.h>
#include <debug.h>
#include <errno.h>

#include "rawelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char g_rawelfmagic[EI_MAGIC_SIZE] =
{
    0x7f, 'E', 'L', 'F'
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: checkarch (originally: up_checkarch)
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the ELF file is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the ELF loader must provide this function.
 *
 *   This function is hard copy from libs/libc/machine/arm/armv7-m/arch_elf.c.
 *   We can't enable ELF option, it makes examples to be loadable.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the ELF file.
 *
 * Returned Value:
 *   True if the architecture supports this ELF file.
 *
 ****************************************************************************/

static bool checkarch(FAR const Elf32_Ehdr *ehdr)
{
  /* Make sure it's an ARM executable */

  if (ehdr->e_machine != EM_ARM)
    {
      berr("ERROR: Not for ARM: e_machine=%04x\n", ehdr->e_machine);
      return false;
    }

  /* Make sure that 32-bit objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS32)
    {
      berr("ERROR: Need 32-bit objects: e_ident[EI_CLASS]=%02x\n", ehdr->e_ident[EI_CLASS]);
      return false;
    }

  /* Verify endian-ness */

#ifdef CONFIG_ENDIAN_BIG
  if (ehdr->e_ident[EI_DATA] != ELFDATA2MSB)
#else
  if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
#endif
    {
      berr("ERROR: Wrong endian-ness: e_ident[EI_DATA]=%02x\n", ehdr->e_ident[EI_DATA]);
      return false;
    }

  /* TODO:  Check ABI here. */
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawelf_verifyheader
 *
 * Description:
 *   Given the header from a possible ELF executable, verify that it
 *   is an ELF executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   -ENOEXEC  : Not an ELF file
 *   -EINVAL : Not a relocatable ELF file or not supported by the current,
 *               configured architecture.
 *
 ****************************************************************************/

int rawelf_verifyheader(FAR const Elf32_Ehdr *ehdr)
{
  if (!ehdr)
    {
      berr("NULL ELF header!");
      return -ENOEXEC;
    }

  /* Verify that the magic number indicates an ELF file */

  if (memcmp(ehdr->e_ident, g_rawelfmagic, EI_MAGIC_SIZE) != 0)
    {
      bwarn("Not ELF magic {%02x, %02x, %02x, %02x}\n",
            ehdr->e_ident[0], ehdr->e_ident[1], ehdr->e_ident[2], ehdr->e_ident[3]);
      return -ENOEXEC;
    }

  /* Verify that this is a relocatable file */

  if (ehdr->e_type == ET_REL)
    {
      berr("Relocatable file: e_type=%d\n", ehdr->e_type);
      return -EINVAL;
    }

  /* Verify that this file works with the currently configured architecture */

  if (!checkarch(ehdr))
    {
      berr("Not a supported architecture\n");
      return -ENOEXEC;
    }

  /* Looks good so far... we still might find some problems later. */

  return OK;
}

