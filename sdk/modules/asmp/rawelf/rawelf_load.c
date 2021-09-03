/****************************************************************************
 * modules/asmp/rawelf/rawelf_load.c
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

#include <sdk/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <elf32.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <mm/tile.h>

#include "rawelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RAWELF_ALIGN_MASK   ((1 << CONFIG_RAWELF_ALIGN_LOG2) - 1)
#define RAWELF_ALIGNUP(a)   (((unsigned long)(a) + RAWELF_ALIGN_MASK) & ~RAWELF_ALIGN_MASK)
#define RAWELF_ALIGNDOWN(a) ((unsigned long)(a) & ~RAWELF_ALIGN_MASK)

#ifndef MAX
#  define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#  define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawelf_elfsize
 *
 * Description:
 *   Calculate total memory allocation for the RAWELF file.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int rawelf_elfsize(struct rawelf_loadinfo_s *loadinfo)
{
  size_t size;
  uint32_t sp;
  int i;

  /* Accumulate the size each section into memory that is marked SHF_ALLOC */

  size = 0;
  sp = 0;

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR Elf32_Shdr *shdr = &loadinfo->shdr[i];

      /* SHF_ALLOC indicates that the section requires memory during
       * execution.
       */

      if ((shdr->sh_flags & SHF_ALLOC) != 0)
        {
          size += RAWELF_ALIGNUP(shdr->sh_size);

          /* Find section of address 0 for determine stack pointer. */

          if (!sp && shdr->sh_addr == 0)
            {
              /* Read stack pointer. This is ARM M architecture only. */

              (void) rawelf_read(loadinfo, (FAR uint8_t *)&sp, sizeof(uint32_t),
                                 shdr->sh_offset);
              binfo("Stack pointer: %08lx\n", sp);
            }
        }
    }

  /* If stack pointer not found, this ELF format is not supported. */

  if (sp == 0)
    {
      return -1;
    }

  /* Save the allocation size. If sp is above the size, then set sp to size. */

  loadinfo->textsize = MAX(size, sp);

  return OK;
}

/****************************************************************************
 * Name: rawelf_loadfile
 *
 * Description:
 *   Read the section data into memory. Section addresses in the shdr[] are
 *   updated to point to the corresponding position in the memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int rawelf_loadfile(FAR struct rawelf_loadinfo_s *loadinfo)
{
  FAR uint8_t *base, *mem;
  int ret;
  int i;

  /* Read each section into memory that is marked SHF_ALLOC + SHT_NOBITS */

  binfo("Loaded sections:\n");
  base = (FAR uint8_t *)loadinfo->textalloc;

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR Elf32_Shdr *shdr = &loadinfo->shdr[i];

      /* SHF_ALLOC indicates that the section requires memory during
       * execution */

      if ((shdr->sh_flags & SHF_ALLOC) == 0)
        {
          continue;
        }

      mem = base + shdr->sh_addr;

      /* SHT_NOBITS indicates that there is no data in the file for the
       * section.
       */

      if (shdr->sh_type != SHT_NOBITS)
        {
          /* Read the section data from sh_offset to the memory region */

          ret = rawelf_read(loadinfo, mem, shdr->sh_size, shdr->sh_offset);
          if (ret < 0)
            {
              berr("ERROR: Failed to read section %d: %d\n", i, ret);
              return ret;
            }
        }

      /* If there is no data in an allocated section, then the allocated
       * section must be cleared.
       */

      else
        {
          memset(mem, 0, shdr->sh_size);
        }

      /* Update sh_addr to point to copy in memory */

      binfo("%d. %08lx->%08lx\n", i,
            (unsigned long)shdr->sh_addr, (unsigned long)mem);

      shdr->sh_addr = (uintptr_t)mem;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawelf_load
 *
 * Description:
 *   Loads the binary into memory, allocating memory, performing relocations
 *   and initializing the data and bss segments.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_load(FAR struct rawelf_loadinfo_s *loadinfo)
{
#ifdef CONFIG_UCLIBCXX_EXCEPTION
  int exidx;
#endif
  int ret;

  binfo("loadinfo: %p\n", loadinfo);
  DEBUGASSERT(loadinfo && loadinfo->filfd >= 0);

  /* Load section headers into memory */

  ret = rawelf_loadshdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: rawelf_loadshdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Determine total size to allocate */

  ret = rawelf_elfsize(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: stack pointer not found.\n");
      goto errout_with_buffers;
    }

  /* Allocate (and zero) memory for the ELF file. */

  loadinfo->textalloc = (uintptr_t)tile_alloc(loadinfo->textsize + loadinfo->datasize);
  if (!loadinfo->textalloc)
    {
      berr("ERROR: tile_alloc() failed\n");
      ret = -ENOMEM;
      goto errout_with_buffers;
    }
  memset((void *)loadinfo->textalloc, 0, loadinfo->textsize + loadinfo->datasize);

  loadinfo->dataalloc = loadinfo->textalloc + loadinfo->textsize;

  /* Load ELF section data into memory */

  ret = rawelf_loadfile(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: rawelf_loadfile failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Load static constructors and destructors. */

#ifdef CONFIG_UCLIBCXX_EXCEPTION
  exidx = rawelf_findsection(loadinfo, CONFIG_RAWELF_EXIDX_SECTNAME);
  if (exidx < 0)
    {
      binfo("rawelf_findsection: Exception Index section not found: %d\n", exidx);
    }
  else
    {
      up_init_exidx(loadinfo->shdr[exidx].sh_addr, loadinfo->shdr[exidx].sh_size);
    }
#endif

  return OK;

  /* Error exits */

errout_with_addrenv:
errout_with_buffers:
  rawelf_unload(loadinfo);
  return ret;
}

