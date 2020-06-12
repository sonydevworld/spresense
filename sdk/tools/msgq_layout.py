#!/usr/bin/env python3
############################################################################
# tools/msgq_layout.py
#
#   Copyright 2019 Sony Semiconductor Solutions Corporation
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

import sys
import os
import re
import traceback
import datetime

# constants

ALINGMENT_SIZE  = 4             # Alignment size
QUE_BLOCK_SIZE  = 68            # Message queue block size sizeof (MsgQueBlock)
MIN_PACKET_SIZE = 8             # Minimum message packet size
MAX_PACKET_SIZE = 512           # Maximum message packet size
MAX_PACKET_NUM  = 16384         # Maximum number of message packets
INVALID_DRM     = 0xffffffff    # Invalid DRM address

# false:Not Support multi core , ture: Support multi core

USE_MULTI_CORE = False

# Parameter initialization

MsgFillValueAfterPop   = 0x00
MsgParamTypeMatchCheck = False
MsgQuePool             = []
SpinLockPool           = []

# Automatically created buffer name

MsgBufferName = "AutoGenMesgBuff"

# Auto buffer creation flag

IsAutoGenBuff = False

# ---------------------------------------------------------------------------
# Class definition
# ---------------------------------------------------------------------------

#
# Duplicate check
#

class DuplicationCheck:
    def __init__(self):
        self.m_array = []

    def exist(self, obj):
        result = obj in self.m_array
        self.m_array.append(obj)
        return result

# ---------------------------------------------------------------------------
# Sub routines
# ---------------------------------------------------------------------------

#
# Show error message
#

def die(msg):
    sys.stderr.write("\n*** Error! {0}\n\n\n".format(msg))

#
# Header comment templete
#

template = "\
/****************************************************************************\n\
 * {0}\n\
 *\n\
 *   Copyright {1} Sony Semiconductor Solutions Corporation\n\
 *\n\
 * Redistribution and use in source and binary forms, with or without\n\
 * modification, are permitted provided that the following conditions\n\
 * are met:\n\
 *\n\
 * 1. Redistributions of source code must retain the above copyright\n\
 *    notice, this list of conditions and the following disclaimer.\n\
 * 2. Redistributions in binary form must reproduce the above copyright\n\
 *    notice, this list of conditions and the following disclaimer in\n\
 *    the documentation and/or other materials provided with the\n\
 *    distribution.\n\
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor\n\
 *    the names of its contributors may be used to endorse or promote\n\
 *    products derived from this software without specific prior written\n\
 *    permission.\n\
 *\n\
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n\
 * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n\
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS\n\
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n\
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,\n\
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,\n\
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS\n\
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED\n\
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT\n\
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN\n\
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE\n\
 * POSSIBILITY OF SUCH DAMAGE.\n\
 *\n\
 ****************************************************************************/\n\n"

#
# Header comment output
#

def output_header_comment(out, title):
    out.write("/* This file is generated automatically. */\n")
    out.write(template.format(os.path.basename(title), datetime.date.today().year))

#
# Include guard prevention macro creation
#

def make_include_guard_name(filename):
    name = os.path.basename(filename.upper()) + "_INCLUDED"
    return name.replace(".", "_")

#
# Output message pool templete
#

msg_pool_templete = "\
#ifndef {0}\n\
#define {0}\n\
\n\
#include \"{1}\"\n\
\n\
extern const MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] =\n\
{{\n\
  {2}\n\
\n\
{3}\
}};\n\
\n\
#endif /* {0} */\n"

#
# Output message pool templete(--without_memory_layout)
#

msg_pool_templete_auto_gen = "\
#ifndef {0}\n\
#define {0}\n\
\n\
#include \"{1}\"\n\
\n\
uint8_t {4}[{5}];\n\
\n\
extern MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS];\n\
\n\
MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] =\n\
{{\n\
  {2}\n\
\n\
{3}\
}};\n\
\n\
#endif /* {0} */\n"

#
# Output message pool list
#

def make_msgq_pool_header(io, pools, end_addr):
    guard_name = make_include_guard_name(MsgqPoolFile)

    output_header_comment(io, MsgqPoolFile)

    if USE_MULTI_CORE == True:
        comment  = "/* n_drm, n_size, n_num, h_drm, h_size, h_num, owner, spinlock */"
    else:
        comment = "/* n_drm, n_size, n_num, h_drm, h_size, h_num */"

    if not IsAutoGenBuff:
        templete = msg_pool_templete
    else:
        templete = msg_pool_templete_auto_gen

    io.write(templete.format(guard_name,
                             os.path.basename(MsgqIdFile),
                             comment,
                             "".join(pools),
                             MsgBufferName,
                             end_addr))

#
# Address alignment
#

def cache_align(addr):
    return (addr + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)

#
# Message queue size and stage number analysis
#

def getMsgQueParam(line, dup_chk):
    resv_ids = ["MSGQ_NULL", "MSGQ_TOP", "MSGQ_END"]

    id = line[0]                
    if id == "":
        raise ValueError("Empty MSGQ ID found")
    if id.index("MSGQ_") != 0:
        raise ValueError("Bad ID found at {0}".format(id))
    if id in resv_ids:
        raise ValueError("Reserved ID found")
    if dup_chk.exist(id):
        raise ValueError("Duplication ID at {0}".format(id))

    n_size  = line[1]
    if n_size < MIN_PACKET_SIZE or n_size > MAX_PACKET_SIZE or n_size % 4 != 0:
        raise ValueError("Bad n_size at {0}".format(id))
    if MsgParamTypeMatchCheck == True and n_size > MIN_PACKET_SIZE:
        n_size += 4

    n_num   = line[2]
    if n_num == 0 or n_num > MAX_PACKET_NUM:
        raise ValueError("Bad n_num at {0}".format(id))

    h_size = line[3]
    if h_size != 0 and (h_size < MIN_PACKET_SIZE or h_size > MAX_PACKET_SIZE or h_size % 4 != 0):
        raise ValueError("Bad h_size at {0}".format(id))

    if MsgParamTypeMatchCheck == True and h_size > MIN_PACKET_SIZE:
        h_size += 4

    h_num = line[4]
    if h_num > MAX_PACKET_NUM or (h_size > 0 and h_num == 0) or (h_size == 0 and h_num > 0):
        raise ValueError("Bad h_num at {0}".format(id))

    if USE_MULTI_CORE == True:
        owner = line[5]
        if owner in CpuIds.compact is False:
            raise ValueError("Bad owner at {0}".format(id))

    if USE_MULTI_CORE == True:
        spinlock = line[6];

    if USE_MULTI_CORE == True:
        if spinlock == "" or spinlock == "SPL_NULL":
            spinlock = "SPL_NULL"
        else:
            if spinlock in SpinLockNames is False:
                raise ValueError("Bad spinlock at {0}".format(id))
            n_size = (n_size + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)
            h_size = (h_size + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)

    if USE_MULTI_CORE == True:
        return id, spinlock, n_size, n_num, h_size, h_num, owner
    else:
        return id, n_size, n_num, h_size, h_num

#
# Create message pool list
#

def parseMsgQuePool():
    macros = []
    if USE_MULTI_CORE == True:
        pools = ["  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0, 0 }, /* MSGQ_NULL */\n"]
    else:
        pools = ["  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0 }, /* MSGQ_NULL */\n"]

    que_area_drm = START_DRM + QUE_BLOCK_SIZE   # 0 is reserved
    msg_area_drm = START_DRM + len(MsgQuePool) * QUE_BLOCK_SIZE

    dup_chk = DuplicationCheck()

    for line in MsgQuePool:
        if line is None:
            break

        if USE_MULTI_CORE == True:
            id, spinlock, n_size, n_num, h_size, h_num, owner = getMsgQueParam(line, dup_chk)
        else:
            id, n_size, n_num, h_size, h_num = getMsgQueParam(line, dup_chk)

        if (USE_MULTI_CORE == True and TargetCore == owner) or TargetCore is None:
            n_drm = cache_align(msg_area_drm)
            h_drm = INVALID_DRM if h_size == 0 else n_drm + n_size * n_num
            msg_area_drm = n_drm + n_size * n_num + h_size * h_num
            if not msg_area_drm <= LIMIT_DRM:
                raise ValueError("Lack of pool area at " + id)

            # create macros for pool information
            macros.append("/************************************************************************/")
            macros.append("{0}_QUE_BLOCK_DRM 0x{1:x}".format(id, que_area_drm))
            macros.append("{0}_N_QUE_DRM 0x{1:x}".format(id, n_drm))
            macros.append("{0}_N_SIZE {1:d}".format(id, n_size))
            macros.append("{0}_N_NUM {1:d}".format(id, n_num))
            macros.append("{0}_H_QUE_DRM 0x{1:x}".format(id, h_drm))
            macros.append("{0}_H_SIZE {1:d}".format(id, h_size))
            macros.append("{0}_H_NUM {1:d}".format(id, h_num))
            if USE_MULTI_CORE == True:
                macros.append("{0}_OWNER {1}".format(id, owner))
            if USE_MULTI_CORE == True:
                macros.append("{0}_SPINLOCK {1}\n".format(id, spinlock))

            # create pool entry
            if not IsAutoGenBuff:
                if USE_MULTI_CORE == True:
                    pools.append("  { 0x%x, %d, %d, 0x%x, %d, %d, %d, %d }, /* %s */\n" % (n_drm, n_size, n_num, h_drm, h_size, h_num, owner, spinlock, id))
                else:
                    pools.append("  { 0x%x, %d, %d, 0x%x, %d, %d }, /* %s */\n" % (n_drm, n_size, n_num, h_drm, h_size, h_num, id))
            else:
                if USE_MULTI_CORE == True:
                    pools.append("  { (drm_t)%s + 0x%x, %d, %d, 0x%x, %d, %d, %d, %d }, /* %s */\n" % (MsgBufferName, n_drm, n_size, n_num, h_drm, h_size, h_num, owner, spinlock, id))
                else:
                    pools.append("  { (drm_t)%s + 0x%x, %d, %d, 0x%x, %d, %d }, /* %s */\n" % (MsgBufferName, n_drm, n_size, n_num, h_drm, h_size, h_num, id))

            que_area_drm += QUE_BLOCK_SIZE
    return macros, pools, msg_area_drm

#
# Create message ID list
#

def create_msgq_ids():
    ids = ["MSGQ_NULL"]
    for line in MsgQuePool:
        if line is None:
            break
        if USE_MULTI_CORE == True:
            if TargetCore == line[5] or TargetCore is None:
                ids.append(line[0])
        else:
            ids.append(line[0])
    ids.append("NUM_MSGQ_POOLS")
    return ids

#
# Output message ID templete
#

msg_id_templete = "\
#ifndef {0}\n\
#define {0}\n\
\n\
{1}\
{9}\
/* Message area size: {2} bytes */\n\
\n\
#define MSGQ_TOP_DRM {3}\n\
#define MSGQ_END_DRM {4}\n\
\n\
/* Message area fill value after message poped */\n\
\n\
#define MSG_FILL_VALUE_AFTER_POP 0x{5:x}\n\
\n\
/* Message parameter type match check */\n\
\n\
#define MSG_PARAM_TYPE_MATCH_CHECK {6}\n\
\n\
/* Message queue pool IDs */\n\
\n\
{7}\
\n\
/* User defined constants */\n\
\n\
{8}\
\n\
#endif /* {0} */\n"

#
# Message ID header output
#

def make_msgq_id_header(io, macros, end_addr):

    # Output header comment

    output_header_comment(io, MsgqIdFile)

    if USE_MULTI_CORE == True:
        comment = "#include \"spl_id.h\"\n\n"
    else:
        comment = ""

    if not IsAutoGenBuff:
        comment_msg_buf = ""
        top_dram = "0x{:x}".format(START_DRM)
        end_dram = "0x{:x}".format(end_addr)
    else:
        comment_msg_buf = "/* Message area buffer */\n\n"\
                          "extern uint8_t {0}[];\n\n".format(MsgBufferName)
        top_dram = "(uint32_t){}".format(MsgBufferName)
        end_dram = "(uint32_t){} + 0x{:x}".format(MsgBufferName, end_addr)

    # Create message id list

    id_list = ""

    for index, name in enumerate(create_msgq_ids()):
        id_list += "#define {0} {1}\n".format(name, index)

    # Create message id define list

    de_list = ""

    for name in macros:
        if re.match("^\/\*", name):
            de_list += "{0}\n".format(name)
        else:
            de_list += "#define {0}\n".format(name)

    io.write(msg_id_templete.format(make_include_guard_name(MsgqIdFile),
                                    comment,
                                    end_addr - START_DRM,
                                    top_dram,
                                    end_dram,
                                    MsgFillValueAfterPop,
                                    ("true" if MsgParamTypeMatchCheck else "false"),
                                    id_list,
                                    de_list,
                                    comment_msg_buf))

#
# Usage template
#

usage_template = "\
usage: {0} [--help] [-h] [--without_memory_layout] [-n]\n\
           [address | fixed_file] [size | fixed_ID]\n\
           [id_header] [pool_header]\n\
\n\
-n, --without_memory_layout  Allocate message memory in a global variable\n\
-h, --help                   Show this usage and exit\n\
\n\
ex)\n\
  $ python3 msgq_layout.conf\n\
             or\n\
  $ python3 msgq_layout.conf 0x00800000 0x20000\n\
             or\n\
  $ python3 msgq_layout.conf mem_fixed_layout.h\n\
             or\n\
  $ python3 msgq_layout.conf --without_memory_layout\n\
             or\n\
  $ python3 msgq_layout.conf mem_layout.h MSG_QUE_AREA msgq_id.h msgq_pool.h\n\
\n"

#
# Show command usage
#

def usage():
    sys.stderr.write(usage_template.format(os.path.basename(__file__)))
    sys.exit()

#
# Main routine
#

try:
    if USE_MULTI_CORE == True:
        def collectItem(list, index):
            items = []
            for line in list:
                if line is not None:
                    items.append(line[index])
            return items

        SpinLockNames = collectItem(SpinLockPool, 0)

    # Init parameters

    MsgqIdFile    = "msgq_id.h"
    MsgqPoolFile  = "msgq_pool.h"
    MsgQueArea    = "MSG_QUE_AREA"
    MemLayoutFile = "mem_layout.h"
    TargetCore    = None

    # Argument acquisition

    arguments = sys.argv
    arguments.pop(0)

    options = [option for option in arguments if option.startswith('-')]

    # Option analysis

    for option in options:
        if option == "--help" or option == "-h":
            usage()
        elif option.upper() == "--without_memory_layout".upper() or option == "-n":
            IsAutoGenBuff = True
        else:
            print("Unknown option: {}\n".format(option))
            usage()

    params = [option for option in arguments if not option.startswith('-')]

    if len(params):
        MemLayoutFile = params[0]
        params.pop(0)
    if len(params):
        MsgQueArea    = params[0]
        params.pop(0)
    if len(params):
        MsgqIdFile    = params[0]
        params.pop(0)
    if len(params):
        MsgqPoolFile  = params[0]
        params.pop(0)
    if len(params):
        TargetCore    = params[0]
        params.pop(0)

    # If the first character is not 0,
    # it is assumed to be the file name of Fixed memory layout.

    if IsAutoGenBuff:

        # Dummy size setting

        START_DRM = 0x00000
        AREA_SIZE = 0x20000
    elif not re.match("^0", MemLayoutFile):

        # Analyze the file and get the dump area address and size.

        drm_str   = MsgQueArea + "_DRM"
        size_str  = MsgQueArea + "_SIZE"
        msgq_drm  = ""
        msgq_size = ""

        for line in open(MemLayoutFile).readlines():
            if re.match("^#define {0}".format(drm_str), line):
                msgq_drm  = line.split()[2]
            if re.match("^#define {0}".format(size_str), line):
                msgq_size = line.split()[2]

        if msgq_drm  == "":
            raise ValueError("{0} not found".format(drm_str))
        if msgq_size == "":
            raise ValueError("{0} not found".format(size_str))

        START_DRM = int(msgq_drm, 16)
        AREA_SIZE = int(msgq_size, 16)
    else:
        START_DRM = int(MemLayoutFile, 16)
        AREA_SIZE = int(MsgQueArea, 16)

    # Address and size validity check

    if not ((IsAutoGenBuff or START_DRM != 0) and (START_DRM % ALINGMENT_SIZE) == 0):
        raise ValueError("Bad addr. (require 4bytes align) -- 0x%08x" % START_DRM)
    if not (AREA_SIZE != 0 and (AREA_SIZE % ALINGMENT_SIZE) == 0):
        raise ValueError("Bad size. (require 4bytes align) -- 0x%08x" % AREA_SIZE)

    # Set the upper limit of usable addresses

    LIMIT_DRM = START_DRM + AREA_SIZE

    if not MsgFillValueAfterPop <= 0xff:
        raise ValueError("Bad MsgFillValueAfterPop.")
    
    if not MsgParamTypeMatchCheck in [True, False]:
        raise ValueError("Bad MsgParamTypeMatchCheck.")

except Exception as e:
    die(e)
except ValueError as e:
    die(e)

#
# File output
#

def generate_files():
    macros, pools, end_addr = parseMsgQuePool()
    with open(MsgqIdFile, mode='w') as f:
        make_msgq_id_header(f, macros, end_addr)
    with open(MsgqPoolFile, mode='w') as f:
        make_msgq_pool_header(f, pools, end_addr)
