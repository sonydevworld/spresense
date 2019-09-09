#!/usr/bin/env python3
############################################################################
# modules/memutils/memory_manager/tool/mem_layout.py
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

import os
import re
import sys
import argparse

#
# Editable parameters
#
UseFence = 1  # Use of a pool fence


#
# Fixed parameters of feature
#
# To use the function, delete it from below and write it
# in the config file with ture.
#

UseDynamicPool      = False
UseOver255Segments  = False
UseCopiedPoolAttr   = False
UseMultiCore        = False
UseSegDeleter       = False
UseSegThreshold     = False
UseRingBufPool      = False
UseRingBufThreshold = False

#
# Fixed parameters of pool layout
#
# If mem_manager do not use fixed parameters for pool layout,
# set it to false.
#

UseFixedPoolLayout = True

#
# Output setting of the configuration definition of selected feature
#
# If you define the header file, set it to true.
#

EnableOutputMacro = False

#
# Constants
#

Basic   = "BasicType"
RingBuf = "RingBufType"

MinNameSize   = 3
FenceSize     = 4
MinAlign      = 4
MaxPoolId     = 255  # Max pool ID including dynamically generated pool
MaxSegs       = 65535 if UseOver255Segments else 255
RemainderSize = -1

AREA_FENCES_IS_KCONFIG = True

#
# class and method
#

class PowerTable:
    def __init__(self, base, exponent):
        self.base, self.exponent = base, exponent
        self.table = [base ** x for x in list(range(exponent + 1))]


# power of 2 table (2**0, 2**1, ... 2**31)
Power2Table = PowerTable(2, 31)

def round_up(n, align):
    return (n + align - 1) & ~(align - 1)


def align_addr(align, addr):
    if align in Power2Table.table:
        aligned_addr = round_up(addr, align)
    else:
        aligned_addr = align
        while aligned_addr < addr:
            aligned_addr += align
    return aligned_addr, (aligned_addr - addr)


def verify_name(name, tail = ""):
    if not isinstance(name, str):
        return None
    if len(name) >= MinNameSize:
        mm = "^[A-Z][A-Z_0-9]*{0}".format(tail)
        return re.match(mm, name)
    return None


class BaseEntry:
    def __init__(self, name, addr, size):
        self.name           = name
        self.begin_addr     = addr
        self.end_addr       = addr + size
        self.size           = size
        self.last_addr      = self.end_addr - 1
        self.alloc_addr     = addr
        self.max_alloc_addr = addr

    def reset_alloc_addr(self):
        self.alloc_addr = self.begin_addr

    def update_alloc_addr(self):
        self.alloc_addr = self.max_alloc_addr

    def use_size(self):
        return self.alloc_addr - self.begin_addr

    def remainder(self):
        return self.end_addr - self.alloc_addr

    def conflict(self, other):
        if other.begin_addr >= self.begin_addr:
            if other.begin_addr < self.end_addr:
                return False
        if other.end_addr >= self.begin_addr:
            if other.end_addr <= self.end_addr:
                return False
        return True

    def alloc(self, fence, align, req_size):
        lower_fence_size = upper_fence_size = FenceSize if fence else 0

        aligned_addr, skip_size = align_addr(align, self.alloc_addr + lower_fence_size)

        if req_size == RemainderSize:
            req_size = end_addr - aligned_addr - upper_fence_size
            if req_size <= 0:
                return None, None, None

        total_size = req_size + skip_size + lower_fence_size + upper_fence_size
        if total_size > self.remainder():
            return None, None, None

        self.alloc_addr += total_size
        if self.max_alloc_addr < self.alloc_addr:
            self.max_alloc_addr = self.alloc_addr

        return aligned_addr, skip_size, req_size


class DevEntry(BaseEntry):
    def __init__(self, name, ram, addr, size):
        super().__init__(name, addr, size)
        self.ram = ram
        if verify_name(name) is None:
            sys.stderr.write("Bad device name found. name={0}".format(name))
            sys.exit()
        if (addr % MinAlign) != 0:  # 0 is valid
            sys.stderr.write("Bad device addr found at {0}".format(name))
            sys.exit()
        if (size % MinAlign) != 0 or size == 0:
            sys.stderr.write("Bad device size found at {0}".format(name))
            sys.exit()


class MemoryDevices:
    devs = []
    def init(self, *args):
        for arg in args:
            if arg:
                self.check_and_set_arg(*arg)
                if len(self.devs) == 0:
                    sys.stderr.write("Memory device not specified.")
                    sys.exit()

    def check_and_set_arg(self, *arg):
        new_dev = DevEntry(*arg)
        for dev in self.devs:
            if dev.name == new_dev.name:
                sys.stderr.write("Duplication name found from MemoryDevices. name={0}".format(dev.name))
                sys.exit()
            if not dev.conflict(new_dev):
                sys.stderr.write("Conflict found, between {0} and {1}.".format(dev.name, new_dev.name))
                sys.exit()
        self.devs.append(new_dev)

    def output_macros(self, io):
        io.write("/*\n * Memory devices\n */\n\n")
        for dev in self.devs:
            io.write("/* {0}: type={1}, use=0x{2:08x}, remainder=0x{3:08x} */\n\n"
                .format(dev.name, "RAM" if (dev.ram) else "ROM", dev.size - dev.remainder(), dev.remainder()))
            io.write("#define {0}_ADDR  0x{1:08x}\n".format(dev.name, dev.begin_addr))
            io.write("#define {0}_SIZE  0x{1:08x}\n".format(dev.name, dev.size))
            io.write("\n")

    def at(self, name_str):
        for dev in self.devs:
            if dev.name == name_str:
                return dev
        return None


class AreaEntry(BaseEntry):
    def __init__(self, name, device, align, size, fence):
        self.dev_entry  = MemoryDevices.at(device)
        self.align      = align
        self.skip_size  = 0
        self.fence_flag = fence
    
        if verify_name(name, "_AREA") is None:
            sys.stderr.write("Bad area name found. name={0}".format(name))
            sys.exit()
        if MemoryDevices.at(name):
            sys.stderr.write("Redefine name found. name={0}".format(name))
            sys.exit()
        if self.dev_entry is None:
            sys.stderr.write("Device not found at {0}".format(name))
            sys.exit()
        if (align % MinAlign) != 0 or align == 0:
            sys.stderr.write("Bad area align found at {0}".format(name))
            sys.exit()
        if align >= self.dev_entry.last_addr:
            sys.stderr.write("Too big area align at {0}".format(name))
            sys.exit()
        if size != RemainderSize:
            if (size % MinAlign) != 0 or size == 0:
                sys.stderr.write("Bad area size found at {0}".format(name))
                sys.exit()
            if size > self.dev_entry.remainder():
                sys.stderr.write("Too big area size at {0}. remainder={1:08x}".format(name, self.dev_entry.remainder())) 
                sys.exit()
    
        if not UseFence:
            fence_flag = False
            if fence_flag and not self.dev_entry.ram:
                sys.stderr.write("Bad area fence found at {0}".format(name))
                sys.exit()

        addr, skip_size, size = self.dev_entry.alloc(self.fence_flag, align, size)
        if addr is None:
            sys.stderr.write("Can't allocate area at {0}".format(name))
            sys.exit()

        # set base class
        super().__init__(name, addr, size)


class FixedAreas:
    areas = []
    def init(self, *args):
        for arg in args:
            if arg:
                self.check_and_set_arg(*arg)
                if len(self.areas) == 0:
                    sys.stderr.write("Fixed area not specified.")
                    sys.exit()

    def check_and_set_arg(self, *arg):
        new_area = AreaEntry(*arg)
        for area in self.areas:
            if area.name == new_area.name:
                sys.stderr.write("Duplication name found from FixedAreas. name={0}".format(area.name))
                sys.exit()
        self.areas.append(new_area)

    def at(self, name_str):
        for area in self.areas:
            if area.name == name_str:
                return area
        return None

    def used_info(self):
        used = []
        for area in self.areas:
            if area.use_size() > 0:
                used.append([area.name, area.remainder()])
        return used

    def reset_areas(self):
        for area in self.areas:
            area.reset_alloc_addr()

    def update_areas(self):
        for area in self.areas:
            area.update_alloc_addr()

    def memmgr_work_size(self):
        work_size = 0
        for area in self.areas:
            if area.name == "MEMMGR_WORK_AREA":
                work_size = area.size
                break
        return work_size

    def output_macros(self, io):
        num_fences = 0
        io.write("/*\n * Fixed areas\n */\n\n")
        for area in self.areas:
            if area.skip_size > 0:
                io.write("/* Skip 0x{:04x} bytes for alignment. */\n".format(area.skip_size))
            io.write("#define {0}_ALIGN   0x{1:08x}\n".format(area.name, area.align))
            if area.fence_flag:
                io.write("#define {0}_L_FENCE 0x{1:08x}\n".format(area.name, area.begin_addr - FenceSize))
            io.write("#define {0}_ADDR    0x{1:08x}\n".format(area.name, area.begin_addr))
            io.write("#define {0}_DRM     0x{1:08x} /* _DRM is obsolete macro. to use _ADDR */\n".format(area.name, area.begin_addr))
            io.write("#define {0}_SIZE    0x{1:08x}\n".format(area.name, area.size))
            if area.fence_flag:
                io.write("#define {0}_U_FENCE 0x{1:08x}\n".format(area.name, area.begin_addr + area.size))
            io.write("\n")
            if area.fence_flag:
                num_fences += 2
        if AREA_FENCES_IS_KCONFIG == False:
            if UseFence:
                io.write("#define NUM_FIXED_AREA_FENCES {0}\n\n".format(num_fences))

    def output_table(self, io):
        if not UseFence:
            return 
        # Compiler internal error of gcc 3.3.6 occurs when FixedAreaFences[0]
        io.write("extern PoolAddr const FixedAreaFences[] = {\n")
        for area in self.areas:
            if area.fence_flag:
                io.write("  /* lower */ 0x{0:08x}, /* upper */ 0x{1:08x},\n".format(area.begin_addr - FenceSize, area.begin_addr + area.size))
        io.write("}; /* end of FixedAreaFences */\n\n")


class PoolEntry(BaseEntry):
    def __init__(self, name, area, type, align, size, seg, fence, spinlock):
        self.area_entry = FixedAreas.at(area)
        type            = type
        align           = align
        num_seg         = seg
        skip_size       = 0
        self.fence_flag = fence
        spinlock        = spinlock

        if not verify_name(name, "_POOL"):
            sys.stderr.write("Bad pool name found. name={0}".format(name))
            sys.exit()
        if not self.area_entry:
            sys.stderr.write("Area not found at {0}".format(name))
            sys.exit()
        if not self.area_entry.dev_entry.ram:
            sys.stderr.write("Not RAM area found at {0}".format(name))
            sys.exit()
        if MemoryDevices.at[name]:
            sys.stderr.write("Redefine name found at {0}".format(name))
            sys.exit()
        if type != Basic and type != RingBuf:
            sys.stderr.write("Bad pool type found at {0}".format(name))
            sys.exit()
        if type == RingBuf and not UseRingBufPool:
            sys.stderr.write("Don't use RingBuf type at {0}".format(name))
            sys.exit()
        if (align % MinAlign) != 0 or align == 0:
            sys.stderr.write("Bad pool align found at {0}".format(name))
            sys.exit()
        if align >= self.area_entry.last_addr:
            sys.stderr.write("Too big pool align at {0}".format(name))
            sys.exit()
        if size != RemainderSize:
            if (size % MinAlign) != 0 or size == 0:
                sys.stderr.write("Bad pool size found at {0}".format(name))
                sys.exit()
            if size > self.area_entry.remainder:
                sys.stderr.write("Too big pool size at {0}. remainder={1:08x}".format(name, self.area_entry.remainder()))
                sys.exit()
        if seg <= 0 or seg > MaxSegs or (size != RemainderSize and size < seg):
            sys.stderr.write("Bad pool seg found at {0}".format(name))
            sys.exit()
        if not UseFence:
            fence_flag = false
        if not UseMultiCore or spinlock == "":
            spinlock = "SPL_NULL"

        addr, skip_size, size = self.area_entry.alloc(fence_flag, align, size)
        if addr is None:
            sys.stderr.write("Can't allocate pool at {0}".format(name))
            sys.exit()

        # set base class
        super().__init__(name, addr, size)


class PoolEntryFixParam(BaseEntry):
    def __init__(self, section, layout_no, name, area, align, size, seg, fence):
        self.area_entry = FixedAreas.at(area)
        self.type       = Basic
        self.align      = align
        self.num_seg    = seg
        self.skip_size  = 0
        self.fence_flag = fence
        self.section    = section
        self.layout     = layout_no
        spinlock        = "SPL_NULL"

        if not verify_name(name, "_POOL"):
            sys.stderr.write("Bad pool name found. name={0}".format(name))
            sys.exit()
        if not self.area_entry:
            sys.stderr.write("Area not found at {0}".format(name))
            sys.exit()
        if not self.area_entry.dev_entry.ram:         
            sys.stderr.write("Not RAM area found at {0}".format(name))
            sys.exit()
        if MemoryDevices.at(name):
            sys.stderr.write("Redefine name found at {0}".format(name))
            sys.exit()
        if (align % MinAlign) != 0 or align == 0:
            sys.stderr.write("Bad pool align found at {0}".format(name))
            sys.exit()
        if align >= self.area_entry.last_addr:
            sys.stderr.write("Too big pool align at {0}".format(name))
            sys.exit()
        if size != RemainderSize:
            if (size % MinAlign) != 0 or size == 0:
                sys.stderr.write("Bad pool size found at {0}".format(name))
                sys.exit()
            if size > self.area_entry.remainder():
                sys.stderr.write("Too big pool size at section={0} layout={1} {2} remainder={3:08x}\n".format(section, layout_no, name, self.area_entry.remainder()))
                sys.exit()
        if seg <= 0 or seg > MaxSegs or (size != RemainderSize and size < seg):
            sys.stderr.write("Bad pool seg found at {0}".format(name))
            sys.exit()
        if not UseFence:
            self.fence_flag = false

        addr, self.skip_size, size = self.area_entry.alloc(self.fence_flag, align, size)
        if addr is None:
            sys.stderr.write("Can't allocate pool at {0}".format(name))
            sys.exit()

        # set base class
        super().__init__(name, addr, size)


# When creating a memory pool, the necessary work area size for each pool
#  - Alignment adjustment of MemPool area         : 0-3
#  - Pool attribute area(Usually in static pool 0): 0, 12 or 16
#  - BasicPool(=MemPool) area                      : 12 + 4 * sizeof(NumSeg)
#  - RingBufPool area                              : To be determined(MemPool Area+alpha)
#  - Data area of the segment number queue         : Number of segments * sizeof(NumSeg)
#  - Reference counter area                        : Number of segments * sizeof(SegRefCnt)
NumSegSize              = 2 if UseOver255Segments else 1
SegRefCntSize           = 1
PoolAttrSize            = round_up(10 + NumSegSize + (1 if UseFence else 0) + (1 if UseMultiCore else 0), 4)
MemPoolDataSize         = 12 + 4 * NumSegSize   # 16 or 20
BasicPoolDataSize       = MemPoolDataSize
RingBufPoolDataSize     = MemPoolDataSize + 32  # Tentative value for details unexamined
RingBufPoolSegDataSize  = 8                     # Tentative value for details unexamined


class PoolLayout:
    section = 0
    def __init__(self, section, layout_no, *args):
        if section == self.section:
            FixedAreas.reset_areas()
        else:
            FixedAreas.update_areas()
        self.section = section
        self.layout  = layout_no
        self.pools   = []
        for arg in args:
            if arg is not None:
                self.check_and_set_arg(section, layout_no, *arg) 

        # Save the remaining size of the fixed area for each layout and reset
        self.used_area_info = FixedAreas.used_info()

    def check_and_set_arg(self, section, layout_no, *arg):
        if UseFixedPoolLayout:
            new_pool = PoolEntryFixParam(section, layout_no, *arg)
        else:
            new_pool = PoolEntry(*arg)
        for pool in self.pools:
            if pool.name == new_pool.name:
                sys.stderr.write("Duplication name found from PoolLayout. name={0}".format(pool.name))
                sys.exit()
        self.pools.append(new_pool)

    # Calculate necessary work area for each layout
    def work_size(self, section):
        layout_work_size = 0
        for pool in self.pools:
            if section == pool.section:
                pool_work_size  = PoolAttrSize if UseCopiedPoolAttr else 0
                pool_work_size += BasicPoolDataSize if pool.type == Basic else RingBufPoolDataSize
                pool_work_size += pool.num_seg * NumSegSize    # Data area of the segment number queue
                pool_work_size += pool.num_seg * SegRefCntSize # Reference counter area
                # Round up to the MinAlign unit and integrate
                layout_work_size += round_up(pool_work_size, MinAlign)
                if layout_work_size > FixedAreas.memmgr_work_size():
                    sys.stderr.write("MMGER_WORK_AREA 0x{0:x} Over! SECTION={1} LAYOUT={2}:{3}\n".format(layout_work_size, pool.section, pool.layout, pool.name))
                    sys.exit()
        return layout_work_size

    def names(self, section):
        list = []
        for pool in self.pools:
            if section == pool.section:
                list.append(pool.name)
        return list


class PoolAreas:
    section      = 0
    layouts      = []
    pool_ids     = []
    pool_id_num  = []
    layout_num   = []
    section_name = []
    def init(self, *args):
        layout_no  = 0
        for arg in args:
            if arg is not None:
                self.layouts.append(PoolLayout(self.section, layout_no, *arg))
                layout_no += 1

        self.layout_num.append(layout_no)

        if len(self.layouts) == 0:
            sys.stderr.write("Pool layout not specified.")
            sys.exit()

        max_pool_id = MaxPoolId
        if UseDynamicPool:
            if NumDynamicPools <= 0 or NumDynamicPools > MaxPoolId:
                sys.stderr.write("Bad NumDynamicPools value.") 
                sys.exit()
            max_pool_id -= NumDynamicPools

        # Create Pool IDs
        t_list = []
        for layout in self.layouts:
            t_list = t_list + layout.names(self.section)

        t_list2 = []
        for id in t_list:
            if id not in t_list2:
                t_list2.append(id)
        self.pool_id_num.append(len(t_list2) + 1)

        for index, id in enumerate(t_list2):
            if index == 0:
                self.pool_ids.append("const MemMgrLite::PoolId S%d_%-24s = {%2d, SECTION_NO%d}" % (self.section, "NULL_POOL", 0, self.section))
            self.pool_ids.append("const MemMgrLite::PoolId S%d_%-24s = {%2d, SECTION_NO%d}" % (self.section, id, index + 1, self.section))

        if len(self.pool_ids) - 1 > max_pool_id:
            sys.stderr.write("Too many pool IDs.")
            sys.exit()

        self.section += 1

    def init_with_section_name(self, section_name, *args):
        self.section_name.append("#define %-17s SECTION_NO%d" % (section_name, self.section))
        self.init(*args)

    def max_work_size(self, section):
        max = 0
        for layout in self.layouts:
            if max < layout.work_size(section):
                max = layout.work_size(section)
        return max

    def max_pool_num(self):
        max = 0
        for layout in self.layouts:
            if max < len(layout.pools):
                max = len(layout.pools)
        return max

    def output_macros(self, io):
        io.write("/*\n * Memory Manager max work area size\n */\n\n")

        for num in range(self.section):
            if num == 0:
                io.write("#define S{0}_MEMMGR_WORK_AREA_ADDR  MEMMGR_WORK_AREA_ADDR\n".format(num))
            else:
                io.write("#define S{0}_MEMMGR_WORK_AREA_ADDR  (S{1}_MEMMGR_WORK_AREA_ADDR + S{2}_MEMMGR_WORK_AREA_SIZE)\n".format(num, num - 1, num - 1))

        for num in range(self.section):
            io.write("#define S{0}_MEMMGR_WORK_AREA_SIZE  0x{1:08x}\n".format(num, self.max_work_size(num)))
        io.write("\n")

        io.write("/*\n * Section IDs\n */\n\n")
        for num in range(self.section):
            io.write("#define SECTION_NO{0}       {0}\n".format(num))
        io.write("\n")

        if len(self.section_name) > 0:
            io.write("/*\n * Supports named section IDs\n */\n\n")
            for nam in self.section_name:
                io.write("{0}\n".format(nam))
            io.write("\n")

        io.write("/*\n * Number of sections\n */\n\n")
        io.write("#define NUM_MEM_SECTIONS  {}\n\n".format(self.section))

        io.write("/*\n * Pool IDs\n */\n\n")
        for index, name in enumerate(self.pool_ids):
            io.write("{0};  /* {1:2d} */\n".format(name, index))
        io.write("\n")

        for num in range(self.section):
            io.write("#define NUM_MEM_S{0}_LAYOUTS  {1:2d}\n".format(num, self.layout_num[num]))
            io.write("#define NUM_MEM_S{0}_POOLS    {1:2d}\n".format(num, self.pool_id_num[num]))
        io.write("\n")

        io.write("#define NUM_MEM_LAYOUTS     {0:2d}\n".format(max(self.layout_num)))
        io.write("#define NUM_MEM_POOLS       {0:2d}\n".format(max(self.pool_id_num)))
        if UseDynamicPool:
            io.write("#define NUM_DYN_POOLS  {0}\n".format(NumDynamicPools))
            io.write("#define DYN_POOL_WORK_SIZE(attr) \\\n")
            io.write(" ROUND_UP(sizeof(MemMgrLite::PoolAttr) + #{} +".format(BasicPoolDataSize))
            io.write(" {0} * (attr).num_segs + {1} * (attr).num_segs, 4)\n".format(NumSegSize, SegRefCntSize))

        io.write("\n/*\n * Pool areas\n */\n\n")
        for layout in self.layouts:
            for index, pool in enumerate(layout.pools):
                if index == 0:
                    io.write("/* Section{0} Layout{1}: */\n\n".format(pool.section, pool.layout))
                    io.write("#define MEMMGR_S{0}_L{1}_WORK_SIZE   0x{2:08x}\n\n".format(pool.section, pool.layout, layout.work_size(pool.section)))
                if pool.skip_size > 0:
                    io.write("/* Skip 0x{0:04x} bytes for alignment. */\n\n".format(pool.skip_size))
                io.write("#define S{0}_L{1}_{2}_ALIGN    0x{3:08x}\n".format(pool.section, pool.layout, pool.name, pool.align))
                if pool.fence_flag:
                    io.write("#define S{0}_L{1}_{2}_L_FENCE  0x{3:08x}\n".format(pool.section, pool.layout, pool.name, pool.begin_addr - FenceSize))
                io.write("#define S{0}_L{1}_{2}_ADDR     0x{3:08x}\n".format(pool.section, pool.layout, pool.name, pool.begin_addr))
                io.write("#define S{0}_L{1}_{2}_SIZE     0x{3:08x}\n".format(pool.section, pool.layout, pool.name, pool.size))
                if pool.fence_flag:
                    io.write("#define S{0}_L{1}_{2}_U_FENCE  0x{3:08x}\n".format(pool.section, pool.layout, pool.name, pool.begin_addr + pool.size))
                io.write("#define S{0}_L{1}_{2}_NUM_SEG  0x{3:08x}\n".format(pool.section, pool.layout, pool.name, pool.num_seg))
                if pool.type == Basic:
                    io.write("#define S{0}_L{1}_{2}_SEG_SIZE 0x{3:08x}\n".format(pool.section, pool.layout, pool.name, int(pool.size / pool.num_seg)))
                io.write("\n")
            for name_remainder in layout.used_area_info:
                io.write("/* Remainder {0}=0x{1:08x} */\n".format(name_remainder[0], name_remainder[1]))
            io.write("\n")

    def output_table(self, io):
        io.write("MemPool*  static_pools_block[NUM_MEM_SECTIONS][NUM_MEM_POOLS];\n")
        io.write("MemPool** static_pools[NUM_MEM_SECTIONS] = {\n")
        for num in range(self.section):
            io.write("  static_pools_block[{0}],\n".format(num))
        io.write("};\n")
        io.write("uint8_t layout_no[NUM_MEM_SECTIONS] = {\n")
        for num in range(self.section):
            io.write("  BadLayoutNo,\n")
        io.write("};\n")
        io.write("uint8_t pool_num[NUM_MEM_SECTIONS] = {\n")
        for num in range(self.section):
            io.write("  NUM_MEM_S{}_POOLS,\n".format(num))
        io.write("};\n")

        io.write("extern const PoolSectionAttr MemoryPoolLayouts[NUM_MEM_SECTIONS][NUM_MEM_LAYOUTS][%d] = {\n" % (self.max_pool_num() + 1))
        for num in range(self.section):
            io.write("  {  /* Section:%d */\n" % num)
            for layout in self.layouts:
                if layout.section == num:
                    for index, pool in enumerate(layout.pools):
                        if index == 0:
                            io.write("    {/* Layout:%d */\n" % layout.layout)
                            io.write("     /* %-31s  %-11s  %-3s" % ("pool_ID", "type", "seg"))
                            if UseFence:
                                io.write("  fence")
                                if UseMultiCore:
                                    io.write(" spinlock")
                                io.write("  addr        size         */\n")
                        io.write("      { S%d_%-28s, %-11s, %3u" % (pool.section, pool.name, pool.type, pool.num_seg))
                        if UseFence:
                            io.write(", {0}".format(" true" if pool.fence_flag else "false"))
                        if UseMultiCore:
                            io.write(", {0}".format(pool.spinlock))
                        io.write(", 0x{0:08x}, 0x{1:08x}".format(pool.begin_addr, pool.size))
                        io.write(" },  /* %s */\n" % (pool.area_entry.name))
                    io.write("      { S%d_NULL_POOL, 0, 0, false, 0, 0 },\n" % (layout.section))
                    io.write("    },\n")
            io.write("  },\n")
        io.write("}; /* end of MemoryPoolLayouts */\n\n")


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
 ****************************************************************************/\n\
\n"


class HeaderFile:
    def create(self, io, filename, add_def):
        self.guard_name = os.path.basename(filename.upper().replace(".", "_")) + "_INCLUDED"
        self.add_def    = add_def
        self.io         = io
        self.io.write("/* This file is generated automatically. */\n")
        self.io.write(template.format(filename))
        self.io.write("#ifndef {0}\n".format(self.guard_name))
        self.io.write("#define {0}\n\n".format(self.guard_name))
        if self.add_def:
            self.io.write("#include \"memutils/memory_manager/MemMgrTypes.h\"\n")
            self.io.write("\nnamespace MemMgrLite {\n\n")

    def close(self):
        if self.add_def:
            self.io.write("}  /* end of namespace MemMgrLite */\n\n")
        self.io.write("#endif /* {0} */\n".format(self.guard_name))


def generate_files():

    # Init parameters
    MemLayout    = "mem_layout.h"
    FixedFence   = "fixed_fence.h"
    PoollLayout  = "pool_layout.h"

    arguments = sys.argv
    arguments.pop(0)

    options = [option for option in arguments if option.startswith('-')]
    if len(options):
        arguments.pop(0)

    if len(arguments):
        MemLayout   = arguments[0]
        arguments.pop(0)
    if len(arguments):
        FixedFence  = arguments[0]
        arguments.pop(0)
    if len(arguments):
        PoollLayout = arguments[0]
        arguments.pop(0)

    with open(MemLayout, mode='w') as f:
        HeaderFile.create(f, MemLayout, False)
        MemoryDevices.output_macros(f)
        FixedAreas.output_macros(f)
        PoolAreas.output_macros(f)
        HeaderFile.close()

    with open(FixedFence, mode='w') as f:
        HeaderFile.create(f, FixedFence, True)
        FixedAreas.output_table(f)
        HeaderFile.close()

    with open(PoollLayout, mode='w') as f:
        HeaderFile.create(f, PoollLayout, True)
        PoolAreas.output_table(f)
        HeaderFile.close()


MemoryDevices = MemoryDevices()
FixedAreas    = FixedAreas()
PoolAreas     = PoolAreas()
HeaderFile    = HeaderFile()
