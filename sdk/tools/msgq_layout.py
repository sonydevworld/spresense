############################################################################
# modules/memutils/message/tool/msgq_layout.py
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

#
# constants

ALINGMENT_SIZE = 4          # アラインメントサイズ
QUE_BLOCK_SIZE =  68            # メッセージキューブロックサイズ sizeof(MsgQueBlock)
MIN_PACKET_SIZE = 8         # 最小メッセージパケットサイズ
MAX_PACKET_SIZE = 512           # 最大メッセージパケットサイズ
MAX_PACKET_NUM = 16384          # 最大メッセージパケット数
INVALID_DRM = 0xffffffff        # 不正なDRMアドレス

# false:Not Support multi core , ture: Support multi core
USE_MULTI_CORE = False

MsgFillValueAfterPop   = 0x00
MsgParamTypeMatchCheck = False
MsgQuePool             = []
SpinLockPool           = []

# Sub routines

def die(msg):
    sys.stderr.write("{0}: *** Error! {1} ***\n".format(os.path.basename(sys.argv[0]), msg))

template = "\
/****************************************************************************\n\
 * {0}\n\
 *\n\
 *   Copyright 2019 Sony Semiconductor Solutions Corporation\n\
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

def output_header_comment(out, title):
    out.write(template.format(title))

def make_include_guard_name(filename):
    name = os.path.basename(filename.upper()) + "_INCLUDED"
    return name.replace(".", "_")

class DuplicationCheck:
    def __init__(self):
        self.m_array = []

    def exist(self, obj):
        result = obj in self.m_array
        self.m_array.append(obj)
        return result

def output_msgq_pool(out, pools):
    out.write("extern const MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] = {\n")
    if USE_MULTI_CORE == True:
        out.write("   /* n_drm, n_size, n_num, h_drm, h_size, h_num, owner, spinlock */\n")
    else:
        out.write("   /* n_drm, n_size, n_num, h_drm, h_size, h_num */\n")

    for line in pools:
        out.write(line)
    
    out.write("};\n\n")

def make_msgq_pool_header(pools):
    title = os.path.basename(MsgqPoolFile)
    guard_name = make_include_guard_name(MsgqPoolFile)

    output_header_comment(sys.stdout, title)
    sys.stdout.write("#ifndef {0}\n".format(guard_name))
    sys.stdout.write("#define {0}\n\n".format(guard_name))

    sys.stdout.write("#include \"{0}\"\n\n".format(os.path.basename(MsgqIdFile)))
    output_msgq_pool(sys.stdout, pools)

    sys.stdout.write("#endif /* {0} */\n".format(guard_name))

def cache_align(addr):
    return (addr + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)

def getMsgQueParam(line, dup_chk):
    resv_ids = ["MSGQ_NULL", "MSGQ_TOP", "MSGQ_END"]

    id = line[0]                
    if id == "":
        raise("Empty MSGQ ID found")
    if id.index("MSGQ_") != 0:
        raise("Bad ID found at {0}".format(id))
    if id in resv_ids:
        raise("Reserved ID found")
    if dup_chk.exist(id):
        raise("Duplication ID at {0}".format(id))

    n_size  = line[1]
    if n_size < MIN_PACKET_SIZE or n_size > MAX_PACKET_SIZE or n_size % 4 != 0:
        raise("Bad n_size at {0}".format(id))
    if MsgParamTypeMatchCheck == True and n_size > MIN_PACKET_SIZE:
        n_size += 4

    n_num   = line[2]
    if n_num == 0 or n_num > MAX_PACKET_NUM:
        raise("Bad n_num at {0}".format(id))

    h_size = line[3]
    if h_size != 0 and (h_size < MIN_PACKET_SIZE or h_size > MAX_PACKET_SIZE or h_size % 4 != 0):
        raise("Bad h_size at {0}".format(id))

    if MsgParamTypeMatchCheck == True and h_size > MIN_PACKET_SIZE:
        h_size += 4

    h_num = line[4]
    if h_num > MAX_PACKET_NUM or (h_size > 0 and h_num == 0) or (h_size == 0 and h_num > 0):
        raise("Bad h_num at {0}".format(id))

    if USE_MULTI_CORE == True:
        owner = line[5]
        if owner in CpuIds.compact is False:
            raise("Bad owner at {0}".format(id))

    if USE_MULTI_CORE == True:
        spinlock = line[6];

    if USE_MULTI_CORE == True:
        if spinlock == "" or spinlock == "SPL_NULL":
            spinlock = "SPL_NULL"
        else:
            if spinlock in SpinLockNames is False:
                raise("Bad spinlock at {0}".format(id))
            n_size = (n_size + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)
            h_size = (h_size + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)

    if USE_MULTI_CORE == True:
        return id, spinlock, n_size, n_num, h_size, h_num, owner
    else:
        return id, n_size, n_num, h_size, h_num

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
            macros.append("{0}_QUE_BLOCK_DRM\t0x{1:x}".format(id, que_area_drm))
            macros.append("{0}_N_QUE_DRM\t0x{1:x}".format(id, n_drm))
            macros.append("{0}_N_SIZE\t{1:d}".format(id, n_size))
            macros.append("{0}_N_NUM\t{1:d}".format(id, n_num))
            macros.append("{0}_H_QUE_DRM\t0x{1:x}".format(id, h_drm))
            macros.append("{0}_H_SIZE\t{1:d}".format(id, h_size))
            macros.append("{0}_H_NUM\t{1:d}".format(id, h_num))
            if USE_MULTI_CORE == True:
                macros.append("{0}_OWNER\t{1}".format(id, owner))
            if USE_MULTI_CORE == True:
                macros.append("{0}_SPINLOCK\t{1}\n".format(id, spinlock))

            # create pool entry
            if USE_MULTI_CORE == True:
                pools.append("  { 0x%x, %d, %d, 0x%x, %d, %d, %d, %d }, /* %s */\n" % (n_drm, n_size, n_num, h_drm, h_size, h_num, owner, spinlock, id))
            else:
                pools.append("  { 0x%x, %d, %d, 0x%x, %d, %d }, /* %s */\n" % (n_drm, n_size, n_num, h_drm, h_size, h_num, id))

            que_area_drm += QUE_BLOCK_SIZE
    return macros, pools, msg_area_drm

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

def make_msgq_id_header(io, macros, end_addr):
    title = os.path.basename(MsgqIdFile)
    guard_name = make_include_guard_name(MsgqIdFile)

    output_header_comment(sys.stdout, title)
    sys.stdout.write("#ifndef {0}\n".format(guard_name))
    sys.stdout.write("#define {0}\n\n".format(guard_name))

    if USE_MULTI_CORE == True:
        sys.stdout.write("#include \"spl_id.h\"\n\n")

    sys.stdout.write("/* Message area size: {0} bytes */\n".format(end_addr - START_DRM))
    sys.stdout.write("#define MSGQ_TOP_DRM\t0x%x\n" % START_DRM)
    sys.stdout.write("#define MSGQ_END_DRM\t0x%x\n\n" % end_addr)

    sys.stdout.write("/* Message area fill value after message poped */\n")
    sys.stdout.write("#define MSG_FILL_VALUE_AFTER_POP\t0x%x\n\n" % MsgFillValueAfterPop)

    sys.stdout.write("/* Message parameter type match check */\n")
    sys.stdout.write("#define MSG_PARAM_TYPE_MATCH_CHECK\t{0}\n\n".format("true" if MsgParamTypeMatchCheck else "false"))

    sys.stdout.write("/* Message queue pool IDs */\n")
    for index, name in enumerate(create_msgq_ids()):
        sys.stdout.write("#define {0}\t{1}\n".format(name, index))
    sys.stdout.write("\n")

    sys.stdout.write("/* User defined constants */\n")
#   Object.constants.sort.each{|cnst|
#       str = cnst.to_s  # 格納される定数は、v1.8までは文字列で、v1.9以降はシンボル
#       sys.stdout.write("#define #{str}\t#{eval(str)}\t/* 0x#{eval(str).to_s(16)} */\n") if str =~ /^U_MSGQ_/
#   }
    sys.stdout.write("\n")

    for name in macros:
        if re.match("^\/\*", name):
            sys.stdout.write("{0}\n".format(name))
        else:
            sys.stdout.write("#define {0}\n".format(name))

    sys.stdout.write("#endif /* {0} */\n".format(guard_name))


# Create a message for python later.
usage_template = "\
Usage:\n\
python3 {0} start_drm size id_header pool_header\n\
             or\n\
python3 {0} fixed_file fixed_ID id_header pool_header\n\n\
ex) ruby msgq_layout.conf 0x00800000 0x20000 msgq_id.h msgq_pool.h\n\
             or\n\
python3 msgq_layout.conf mem_fixed_layout.h MSG_QUE_AREA msgq_id.h msgq_pool.h\n\
             or\n\
python3 msgq_layout.conf mem_fixed_layout.h MSG_QUE_AREA msgq_id.h msgq_pool.h core_id\n\n"

def usage():
    sys.stderr.write(usage_template.format(os.path.basename(sys.argv[0])))


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

    if len(sys.argv) < 1:
        usage() 
        sys.exit()
    if len(sys.argv) > 1:
        MemLayoutFile = sys.argv[1]
    if len(sys.argv) > 2:
        MsgQueArea = sys.argv[2]
    if len(sys.argv) > 3:
        MsgqIdFile = sys.argv[3]
    if len(sys.argv) > 4:
        MsgqPoolFile = sys.argv[4]
    if len(sys.argv) > 5:
        TargetCore   = sys.argv[5]

    # If the first character is not 0,
    # it is assumed to be the file name of Fixed memory layout.

    if not re.match("^0", MemLayoutFile):
        # Analyze the file and get the dump area address and size.
        drm_str  = MsgQueArea + "_DRM"
        size_str = MsgQueArea + "_SIZE"
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
        START_DRM   = int(msgq_drm, 16)
        AREA_SIZE   = int(msgq_size, 16)
    else:
        START_DRM   = int(sys.argv[1], 16)
        AREA_SIZE   = int(sys.argv[2], 16)

    if not (START_DRM != 0 and (START_DRM % ALINGMENT_SIZE) == 0):
        raise ValueError("Bad addr. (require 4bytes align) -- 0x%08x" % START_DRM)
    if not (AREA_SIZE != 0 and (AREA_SIZE % ALINGMENT_SIZE) == 0):
        raise ValueError("Bad size. (require 4bytes align) -- 0x%08x" % AREA_SIZE)
    LIMIT_DRM = START_DRM + AREA_SIZE

    if not MsgFillValueAfterPop <= 0xff:
        raise ValueError("Bad MsgFillValueAfterPop.")
    
    if not MsgParamTypeMatchCheck in [True, False]:
        raise ValueError("Bad MsgParamTypeMatchCheck.")

except Exception as e:
    die(e)
except ValueError as e:
    die(e)

def generate_files():
    sys.stdout.write("/* Auto is generated file. */\n")
    macros, pools, end_addr = parseMsgQuePool()
    make_msgq_id_header(sys.stdout, macros, end_addr)
    make_msgq_pool_header(pools)
