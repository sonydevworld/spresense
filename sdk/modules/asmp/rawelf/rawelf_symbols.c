/****************************************************************************
 * modules/asmp/rawelf/rawelf_symbols.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
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

#include <sdk/config.h>

#include <stdlib.h>
#include <string.h>
#include <elf32.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/symtab.h>

#include "rawelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ELF_BUFFERINCR
#  define CONFIG_ELF_BUFFERINCR 32
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_symname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should only
 *            happen if the file is corrupted).
 *   ESRCH - Symbol has no name
 *
 ****************************************************************************/

static int rawelf_symname(FAR struct rawelf_loadinfo_s *loadinfo,
                          FAR const Elf32_Sym *sym)
{
  FAR uint8_t *buffer;
  off_t  offset;
  size_t readlen;
  size_t bytesread;
  int ret;

  /* Get the file offset to the string that is the name of the symbol.  The
   * st_name member holds an offset into the file's symbol string table.
   */

  if (sym->st_name == 0)
    {
      berr("Symbol has no name\n");
      return -ESRCH;
    }

  offset = loadinfo->shdr[loadinfo->strtabidx].sh_offset + sym->st_name;

  /* Loop until we get the entire symbol name into memory */

  bytesread = 0;

  for (; ; )
    {
      /* Get the number of bytes to read */

      readlen = loadinfo->buflen - bytesread;
      if (offset + readlen > loadinfo->filelen)
        {
          if (loadinfo->filelen <= offset)
            {
              berr("At end of file\n");
              return -EINVAL;
            }

          readlen = loadinfo->filelen - offset;
        }

      /* Read that number of bytes into the array */

      buffer = &loadinfo->iobuffer[bytesread];
      ret = rawelf_read(loadinfo, buffer, readlen, offset);
      if (ret < 0)
        {
          berr("elf_read failed: %d\n", ret);
          return ret;
        }

      bytesread += readlen;
      offset += readlen;

      /* Did we read the NUL terminator? */

      if (memchr(buffer, '\0', readlen) != NULL)
        {
          /* Yes, the buffer contains a NUL terminator. */

          return OK;
        }

      /* No.. then we have to read more */

      ret = rawelf_reallocbuffer(loadinfo, CONFIG_ELF_BUFFERINCR);
      if (ret < 0)
        {
          berr("elf_reallocbuffer failed: %d\n", ret);
          return ret;
        }
    }

  /* We will not get here */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_findsymtab(FAR struct rawelf_loadinfo_s *loadinfo)
{
  int i;

  /* Find the symbol table section header and its associated string table */

  for (i = 1; i < loadinfo->ehdr.e_shnum; i++)
    {
      if (loadinfo->shdr[i].sh_type == SHT_SYMTAB)
        {
          loadinfo->symtabidx = i;
          loadinfo->strtabidx = loadinfo->shdr[i].sh_link;
          break;
        }
    }

  /* Verify that there is a symbol and string table */

  if (loadinfo->symtabidx == 0)
    {
      berr("No symbols in ELF file\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rawelf_readsym
 *
 * Description:
 *   Read the ELFT symbol structure at the specfied index into memory.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   index    - Symbol table index
 *   sym      - Location to return the table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_readsym(FAR struct rawelf_loadinfo_s *loadinfo, int index,
                   FAR Elf32_Sym *sym)
{
  FAR Elf32_Shdr *symtab = &loadinfo->shdr[loadinfo->symtabidx];
  off_t offset;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (symtab->sh_size / sizeof(Elf32_Sym)))
    {
      berr("Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = symtab->sh_offset + sizeof(Elf32_Sym) * index;

  /* And, finally, read the symbol table entry into memory */

  return rawelf_read(loadinfo, (FAR uint8_t *)sym, sizeof(Elf32_Sym), offset);
}

/****************************************************************************
 * Name: rawelf_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sym      - Symbol table entry (value might be undefined)
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should only
 *            happen if the file is corrupted).
 *   ENOSYS - Symbol lies in common
 *   ESRCH  - Symbol has no name
 *   ENOENT - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

int rawelf_symvalue(FAR struct rawelf_loadinfo_s *loadinfo, FAR Elf32_Sym *sym)
{
  uintptr_t secbase;

  switch (sym->st_shndx)
    {
    case SHN_COMMON:
      {
        /* NuttX ELF modules should be compiled with -fno-common. */

        berr("SHN_COMMON: Re-compile with -fno-common\n");
        return -ENOSYS;
      }

    case SHN_ABS:
      {
        /* st_value already holds the correct value */

        binfo("SHN_ABS: st_value=%08lx\n", (long)sym->st_value);
        return OK;
      }

    case SHN_UNDEF:
      {
        /* RAWELF modules cannot allow undefined symbols */

        berr("SHN_UNDEF: Undefined symbol\n");
        return -ENOENT;
      }
      break;

    default:
      {
        secbase = loadinfo->shdr[sym->st_shndx].sh_addr;

        binfo("Other: %08lx+%08x=%08lx\n",
              sym->st_value, secbase, sym->st_value + secbase);

        sym->st_value += secbase;
      }
      break;
    }

  return OK;
}

int rawelf_getsymbolbyname(struct rawelf_loadinfo_s *loadinfo,
                           FAR const char *name, size_t namelen,
                           FAR Elf32_Sym *sym)
{
  FAR Elf32_Shdr *symtab = &loadinfo->shdr[loadinfo->symtabidx];
  int nents = symtab->sh_size / symtab->sh_entsize;
  int ret;
  int i;

  for (i = 0; i < nents; i++)
    {
      ret = rawelf_readsym(loadinfo, i, sym);
      if (ret < 0)
        {
          berr("rawelf_readsym failed. %d\n", ret);
          return ret;
        }
      ret = rawelf_symname(loadinfo, sym);
      if (ret < 0)
        {
          berr("Failed to get symbol name or nameless.\n");
          continue;
        }

      if (strncmp(name, (FAR char *)loadinfo->iobuffer, namelen) == 0)
        {
          return OK;
        }
    }

  return -ENOENT;
}
