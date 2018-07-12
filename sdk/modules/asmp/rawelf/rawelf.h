/****************************************************************************
 * modules/asmp/rawelf/rawelf.h
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

#ifndef __ASMP_SUPERVISOR_RAWELF_RAWELF_H
#define __ASMP_SUPERVISOR_RAWELF_RAWELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/types.h>
#include <elf32.h>

#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_RAWELF_ALIGN_LOG2
#  define CONFIG_RAWELF_ALIGN_LOG2 2
#endif

#ifndef CONFIG_RAWELF_STACKSIZE
#  define CONFIG_RAWELF_STACKSIZE 2048
#endif

#ifndef CONFIG_RAWELF_BUFFERSIZE
#  define CONFIG_RAWELF_BUFFERSIZE 128
#endif

#ifndef CONFIG_RAWELF_BUFFERINCR
#  define CONFIG_RAWELF_BUFFERINCR 32
#endif

/* Allocation array size and indices */

#define LIBRAWELF_RAWELF_ALLOC     0
#ifdef CONFIG_BINFMT_CONSTRUCTORS
#  define LIBRAWELF_CTORS_ALLOC 1
#  define LIBRAWELF_CTPRS_ALLOC 2
#  define LIBRAWELF_NALLOC      3
#else
#  define LIBRAWELF_NALLOC      1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a description of the currently loaded instantiation
 * of an ELF binary.
 */

struct rawelf_loadinfo_s
{
  /* elfalloc is the base address of the memory that is allocated to hold the
   * ELF program image.
   *
   * If CONFIG_ARCH_ADDRENV=n, elfalloc will be allocated using kmm_malloc() (or
   * kmm_zalloc()).  If CONFIG_ARCH_ADDRENV-y, then elfalloc will be allocated using
   * up_addrenv_create().  In either case, there will be a unique instance
   * of elfalloc (and stack) for each instance of a process.
   *
   * The alloc[] array in struct binary_s will hold memory that persists after
   * the ELF module has been loaded.
   */

  uintptr_t         textalloc;   /* .text memory allocated when ELF file was loaded */
  uintptr_t         dataalloc;   /* .bss/.data memory allocated when ELF file was loaded */
  size_t            textsize;    /* Size of the ELF .text memory allocation */
  size_t            datasize;    /* Size of the ELF .bss/.data memory allocation */
  off_t             filelen;     /* Length of the entire ELF file */
  Elf32_Ehdr        ehdr;        /* Buffered ELF file header */
  FAR Elf32_Shdr    *shdr;       /* Buffered ELF section headers */
  uint8_t           *iobuffer;   /* File I/O buffer */

  /* Address environment.
   *
   * addrenv - This is the handle created by up_addrenv_create() that can be
   *   used to manage the tasks address space.
   * oldenv  - This is a value returned by up_addrenv_select() that must be
   *   used to restore the current address environment.
   */

  uint16_t           symtabidx;  /* Symbol table section index */
  uint16_t           strtabidx;  /* String table section index */
  uint16_t           buflen;     /* size of iobuffer[] */
  int                filfd;      /* Descriptor for the file being loaded */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: elf_init
 *
 * Description:
 *   This function is called to configure the library to process an ELF
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/
int rawelf_init(FAR struct rawelf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: rawelf_uninit
 *
 * Description:
 *   Releases any resources committed by rawelf_init().  This essentially
 *   undoes the actions of rawelf_init.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_uninit(FAR struct rawelf_loadinfo_s *loadinfo);

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

int rawelf_load(FAR struct rawelf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: rawelf_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of rawelf_load.  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_unload(struct rawelf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: rawelf_verifyheader
 *
 * Description:
 *   Given the header from a possible ELF executable, verify that it is
 *   an ELF executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_verifyheader(FAR const Elf32_Ehdr *header);

/****************************************************************************
 * Name: rawelf_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'.  The data is
 *   read into 'buffer.' If 'buffer' is part of the ELF address environment,
 *   then the caller is responsibile for assuring that that address
 *   environment is in place before calling this function (i.e., that
 *   rawelf_addrenv_select() has been called if CONFIG_ARCH_ADDRENV=y).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_read(FAR struct rawelf_loadinfo_s *loadinfo, FAR uint8_t *buffer,
             size_t readsize, off_t offset);

/****************************************************************************
 * Name: rawelf_loadshdrs
 *
 * Description:
 *   Loads section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_loadshdrs(FAR struct rawelf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: rawelf_findsection
 *
 * Description:
 *   A section by its name.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sectname - Name of the section to find
 *
 * Returned Value:
 *   On success, the index to the section is returned; A negated errno value
 *   is returned on failure.
 *
 ****************************************************************************/

int rawelf_findsection(FAR struct rawelf_loadinfo_s *loadinfo,
                    FAR const char *sectname);

/****************************************************************************
 * Name: rawelf_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_findsymtab(FAR struct rawelf_loadinfo_s *loadinfo);

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
                   FAR Elf32_Sym *sym);

/****************************************************************************
 * Name: elf_symvalue
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
 *            happen if the file is corrupted)
 *   ENOSYS - Symbol lies in common
 *   ESRCH  - Symbol has no name
 *   ENOENT - Symbol undefined
 *
 ****************************************************************************/

int rawelf_symvalue(FAR struct rawelf_loadinfo_s *loadinfo, FAR Elf32_Sym *sym);

/****************************************************************************
 * Name: rawelf_freebuffers
 *
 * Description:
 *  Release all working buffers.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_freebuffers(FAR struct rawelf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: rawelf_allocbuffer
 *
 * Description:
 *   Perform the initial allocation of the I/O buffer, if it has not already
 *   been allocated.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_allocbuffer(FAR struct rawelf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: rawelf_reallocbuffer
 *
 * Description:
 *   Increase the size of I/O buffer by the specified buffer increment.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_reallocbuffer(FAR struct rawelf_loadinfo_s *loadinfo, size_t increment);

/****************************************************************************
 * Name: rawelf_getsymbolbyname
 *
 * Description:
 *   Get number of entries in symtab
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int rawelf_getsymbolbyname(FAR struct rawelf_loadinfo_s *loadinfo,
                           FAR const char *name, size_t namelen,
                           FAR Elf32_Sym *sym);

#endif /* __ASMP_SUPERVISOR_RAWELF_RAWELF_H */
