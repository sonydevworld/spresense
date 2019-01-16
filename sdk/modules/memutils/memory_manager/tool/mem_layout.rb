############################################################################
# modules/memutils/memory_manager/tool/mem_layout.rb
#
#   Copyright 2018 Sony Semiconductor Solutions Corporation
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

###########################################################################
###########################################################################
# Editable parameters

#####################################################################
# Fixed parameters of feature
#
# To use the function, delete it from below and write it
# in the config file with ture.
#

UseDynamicPool      = false
UseOver255Segments  = false
UseCopiedPoolAttr   = false
UseMultiCore        = false
UseSegDeleter       = false
UseSegThreshold     = false
UseRingBufPool      = false
UseRingBufThreshold = false

#####################################################################
# Fixed parameters of pool layout
#
# If mem_manager do not use fixed parameters for pool layout,
# set it to false.
#

UseFixedPoolLayout = true

#####################################################################
# Output setting of the configuration definition of selected feature
#
# If you define the header file, set it to true.
#

EnableOutputMacro = false

###########################################################################
###########################################################################

##############################################################################
# Constants
#

Basic   = "BasicType"
RingBuf = "RingBufType"

MinNameSize   = 3
FenceSize     = 4
MinAlign      = 4
MaxPoolId     = 255  # Max pool ID including dynamically generated pool
MaxSegs       = UseOver255Segments ? 65535 : 255
RemainderSize = -1

AREA_FENCES_IS_KCONFIG = true

##############################################################################
# class and method
#

class PowerTable
  def initialize(base, exponent)
    @base, @exponent = base, exponent
    @table = Array.new(exponent + 1){|n| base ** n}
  end
  attr_reader :base, :exponent, :table

  def [](n)
    @table[n]
  end

  def index(n)
    @table.index(n)
  end

  def include?(n)
    return @table.include?(n)
  end
end

# power of 2 table (2**0, 2**1, ... 2**31)
Power2Table = PowerTable.new(2, 31)

#######################################################################
def round_up(n, align)
  return (n + align - 1) & ~(align - 1)
end

#######################################################################
def align_addr(align, addr)
  if Power2Table.include?(align)
    aligned_addr = round_up(addr, align)
  else
    aligned_addr = align
    while aligned_addr < addr
      aligned_addr += align
    end
  end
  return aligned_addr, (aligned_addr - addr)
end

#######################################################################
def verify_name(name, tail = "")
  name.instance_of?(String) and name.size >= MinNameSize and name =~ /^[A-Z][A-Z_0-9]*#{tail}$/
end

#######################################################################
class BaseEntry
  def initialize(name, addr, size)
    @name = name
    @begin_addr = @alloc_addr = addr
    @size = size
  end
  attr_reader :name, :begin_addr, :alloc_addr, :size

  def reset_alloc_addr
    @alloc_addr = begin_addr
  end

  def end_addr
    begin_addr + size
  end

  def last_addr
    end_addr - 1
  end

  def use_size
    alloc_addr - begin_addr
  end

  def remainder
    end_addr - alloc_addr
  end

  def to_range
    begin_addr...end_addr
  end

  def include?(addr)
    to_range.include?(addr)
  end

  def conflict?(other)
    include?(other.begin_addr) or other.include?(begin_addr)
  end

  def alloc(fence, align, req_size)
    lower_fence_size = upper_fence_size = (fence) ? FenceSize : 0
    aligned_addr, skip_size = align_addr(align, @alloc_addr + lower_fence_size)

    if req_size == RemainderSize
      req_size = end_addr - aligned_addr - upper_fence_size
      return nil, nil, nil if req_size <= 0
    end

    total_size = req_size + skip_size + lower_fence_size + upper_fence_size
    return nil, nil, nil if total_size > remainder

    @alloc_addr += total_size
    return aligned_addr, skip_size, req_size
  end
end

#######################################################################
class DevEntry < BaseEntry
  def initialize(name, ram, addr, size)
    super(name, addr, size)
    @ram = ram
    abort("Bad device name found. name=#{name}") if !verify_name(name)
    abort("Bad device addr found at #{name}") if addr % MinAlign != 0  # 0 is valid
    abort("Bad device size found at #{name}") if size % MinAlign != 0 or size == 0
  end
  attr_reader :ram
end

#######################################################################
class MemoryDevices
  def self.init(*args)
    @@devs = []
    args.each do |arg|
      check_and_set_arg(arg) if arg
    end
    abort("Memory device not specified.") if @@devs.size == 0
  end

  def self.check_and_set_arg(arg)
    new_dev = DevEntry.new(*arg)
    @@devs.each do |dev|
      abort("Duplication name found. name=#{dev.name}") if dev.name == new_dev.name
      abort("Conflict found, between #{dev.name} and #{new_dev.name}.") if dev.conflict?(new_dev)
    end
    @@devs.push(new_dev)
  end

  def self.[](name_str)
    @@devs.find {|dev| dev.name == name_str}
  end

  def self.output_macros(io)
    io.print("/*\n * Memory devices\n */\n")
    @@devs.each do |dev|
      io.printf("/* %s: type=%s, use=0x%08x, remainder=0x%08x */\n",
        dev.name, (dev.ram) ? "RAM" : "ROM", dev.size - dev.remainder, dev.remainder)
      io.printf("#define %s_ADDR  0x%08x\n", dev.name, dev.begin_addr)
      io.printf("#define %s_SIZE  0x%08x\n", dev.name, dev.size)
      io.print("\n")
    end
  end
end

#######################################################################
class AreaEntry < BaseEntry
  def initialize(name, device, align, size, fence)
    @dev_entry = MemoryDevices[device]
    @align = align
    @skip_size = 0
    @fence_flag = fence

    abort("Bad area name found. name=#{name}") if !verify_name(name, "_AREA")
    abort("Redefine name found. name=#{name}") if MemoryDevices[name]
    abort("Device not found at #{name}")       if !dev_entry
    abort("Bad area align found at #{name}")   if align % MinAlign != 0 or align == 0
    abort("Too big area align at #{name}")     if align >= dev_entry.last_addr
    if size != RemainderSize
      abort("Bad area size found at #{name}")  if size % MinAlign != 0 or size == 0
      abort("Too big area size at #{name}. remainder=#{dev_entry.remainder}") if size > dev_entry.remainder
    end
    @fence_flag = false if !UseFence
    abort("Bad area fence found at #{name}")   if fence_flag and !dev_entry.ram

    addr, @skip_size, size = dev_entry.alloc(fence_flag, align, size)
    abort("Can't allocate area at #{name}")    if addr == nil

    # set base class
    super(name, addr, size)
  end
  attr_reader :dev_entry, :align, :skip_size, :fence_flag
end

#######################################################################
class FixedAreas
  def self.init(*args)
    @@areas = []
    args.each do |arg|
      check_and_set_arg(arg) if arg
    end
    abort("Fixed area not specified.") if @@areas.size == 0
  end

  def self.check_and_set_arg(arg)
    new_area = AreaEntry.new(*arg)
    @@areas.each do |area|
      abort("Duplication name found. name=#{area.name}") if area.name == new_area.name
    end
    @@areas.push(new_area)
  end

  def self.[](name_str)
    @@areas.find {|area| area.name == name_str}
  end

  def self.used_info
    used = []
    @@areas.each do |area|
        used.push([area.name, area.remainder]) if area.use_size > 0
    end
    used
  end

  def self.reset_areas
    @@areas.each do |area|
        area.reset_alloc_addr
    end
  end

  def self.output_macros(io)
    num_fences = 0
    io.print("/*\n * Fixed areas\n */\n")
    @@areas.each do |area|
      io.printf("/* Skip 0x%04x bytes for alignment. */\n", area.skip_size) if area.skip_size > 0
      io.printf("#define %s_ALIGN   0x%08x\n", area.name, area.align)
      io.printf("#define %s_L_FENCE 0x%08x\n", area.name, area.begin_addr - FenceSize) if area.fence_flag
      io.printf("#define %s_ADDR    0x%08x\n", area.name, area.begin_addr)
      io.printf("#define %s_DRM     0x%08x /* _DRM is obsolete macro. to use _ADDR */\n", area.name, area.begin_addr)
      io.printf("#define %s_SIZE    0x%08x\n", area.name, area.size)
      io.printf("#define %s_U_FENCE 0x%08x\n", area.name, area.begin_addr + area.size) if area.fence_flag
      io.print("\n")
      num_fences += 2 if area.fence_flag
    end

    if (AREA_FENCES_IS_KCONFIG == false)
      if UseFence
        io.printf("#define NUM_FIXED_AREA_FENCES #{num_fences}\n\n")
      end
    end
  end

  def self.output_table(io)
    return if !UseFence
    # Compiler internal error of gcc 3.3.6 occurs when FixedAreaFences[0]
    io.printf("extern PoolAddr const FixedAreaFences[] = {\n")
    @@areas.each do |area|
      if area.fence_flag
        io.printf("  /* lower */ 0x%08x, /* upper */ 0x%08x,\n",
                  area.begin_addr - FenceSize, area.begin_addr + area.size)
      end
    end
    io.print("}; /* end of FixedAreaFences */\n\n")
  end
end

#######################################################################
class PoolEntry < BaseEntry
  def initialize(name, area, type, align, size, seg, fence, spinlock)
    @area_entry = FixedAreas[area]
    @type = type
    @align = align
    @num_seg = seg
    @skip_size = 0
    @fence_flag = fence
    @spinlock = spinlock

    abort("Bad pool name found. name=#{name}") if !verify_name(name, "_POOL")
    abort("Area not found at #{name}")         if !area_entry
    abort("Not RAM area found at #{name}")     if !area_entry.dev_entry.ram
    abort("Redefine name found at #{name}")    if MemoryDevices[name]
    abort("Bad pool type found at #{name}")    if type != Basic and type != RingBuf
    abort("Don't use RingBuf type at #{name}") if type == RingBuf and !UseRingBufPool
    abort("Bad pool align found at #{name}")   if align % MinAlign != 0 or align == 0
    abort("Too big pool align at #{name}")     if align >= area_entry.last_addr
    if size != RemainderSize
      abort("Bad pool size found at #{name}")  if size % MinAlign != 0 or size == 0
      abort("Too big pool size at #{name}. remainder=#{area_entry.remainder}") if size > area_entry.remainder
    end
    abort("Bad pool seg found at #{name}")     if seg <= 0 or seg > MaxSegs or (size != RemainderSize and size < seg)
    @fence_flag = false if !UseFence
    @spinlock = "SPL_NULL" if !UseMultiCore or spinlock == ""

    addr, @skip_size, size = area_entry.alloc(fence_flag, align, size)
    abort("Can't allocate pool at #{name}")    if addr == nil

    # set base class
    super(name, addr, size)
  end
  attr_reader :area_entry, :type, :align, :num_seg, :skip_size, :fence_flag, :spinlock
end

#######################################################################
class PoolEntryFixParam < BaseEntry
  def initialize(name, area, align, size, seg, fence)
    @area_entry = FixedAreas[area]
    @type = Basic
    @align = align
    @num_seg = seg
    @skip_size = 0
    @fence_flag = fence
    @spinlock = "SPL_NULL"

    abort("Bad pool name found. name=#{name}") if !verify_name(name, "_POOL")
    abort("Area not found at #{name}")         if !area_entry
    abort("Not RAM area found at #{name}")     if !area_entry.dev_entry.ram
    abort("Redefine name found at #{name}")    if MemoryDevices[name]
    abort("Bad pool align found at #{name}")   if align % MinAlign != 0 or align == 0
    abort("Too big pool align at #{name}")     if align >= area_entry.last_addr
    if size != RemainderSize
      abort("Bad pool size found at #{name}")  if size % MinAlign != 0 or size == 0
      abort("Too big pool size at #{name}. remainder=#{area_entry.remainder}") if size > area_entry.remainder
    end
    abort("Bad pool seg found at #{name}")     if seg <= 0 or seg > MaxSegs or (size != RemainderSize and size < seg)
    @fence_flag = false if !UseFence

    addr, @skip_size, size = area_entry.alloc(fence_flag, align, size)
    abort("Can't allocate pool at #{name}")    if addr == nil

    # set base class
    super(name, addr, size)
  end
  attr_reader :area_entry, :type, :align, :num_seg, :skip_size, :fence_flag, :spinlock
end

# When creating a memory pool, the necessary work area size for each pool
#  - Alignment adjustment of MemPool area         : 0-3
#  - Pool attribute area(Usually in static pool 0): 0, 12 or 16
#  - BasicPool(=MemPool) area                      : 12 + 4 * sizeof(NumSeg)
#  - RingBufPool area                              : To be determined(MemPool Area+alpha)
#  - Data area of the segment number queue         : Number of segments * sizeof(NumSeg)
#  - Reference counter area                        : Number of segments * sizeof(SegRefCnt)
NumSegSize              = UseOver255Segments ? 2 : 1
SegRefCntSize           = 1
PoolAttrSize            = round_up(10 + NumSegSize + (UseFence ? 1 : 0) + (UseMultiCore ? 1 : 0), 4)
MemPoolDataSize         = 12 + 4 * NumSegSize   # 16 or 20
BasicPoolDataSize       = MemPoolDataSize
RingBufPoolDataSize     = MemPoolDataSize + 32  # Tentative value for details unexamined
RingBufPoolSegDataSize  = 8                     # Tentative value for details unexamined

#######################################################################
class PoolLayout
  def initialize(*args)
    @pools = []
    args.each do |arg|
      check_and_set_arg(arg) if arg
    end

    # Save the remaining size of the fixed area for each layout and reset
    @used_area_info = FixedAreas.used_info
    FixedAreas.reset_areas
  end
  attr_reader :pools, :used_area_info

  def check_and_set_arg(arg)
    if UseFixedPoolLayout
      new_pool = PoolEntryFixParam.new(*arg)
    else
      new_pool = PoolEntry.new(*arg)
    end
    @pools.each do |pool|
      abort("Duplication name found. name=#{pool.name}") if pool.name == new_pool.name
    end
    @pools.push(new_pool)
  end

  # Calculate necessary work area for each layout
  def work_size
    layout_work_size = 0
    @pools.each do |pool|
      pool_work_size = (UseCopiedPoolAttr) ? PoolAttrSize : 0
      pool_work_size += (pool.type == Basic) ? BasicPoolDataSize : RingBufPoolDataSize
      pool_work_size += pool.num_seg * NumSegSize    # Data area of the segment number queue
      pool_work_size += pool.num_seg * SegRefCntSize # Reference counter area
      # Round up to the MinAlign unit and integrate
      layout_work_size += round_up(pool_work_size, MinAlign)
    end
    return layout_work_size
  end

  def names
    @pools.collect {|pool| pool.name}
  end
end

#######################################################################
class PoolAreas
  def self.init(*args)
    @@layouts = []
    args.each do |arg|
      @@layouts.push(PoolLayout.new(*arg)) if arg
    end
    abort("Pool layout not specified.") if @@layouts.size == 0

    max_pool_id = MaxPoolId
    if UseDynamicPool
      abort("Bad NumDynamicPools value.") if NumDynamicPools <= 0 or NumDynamicPools > MaxPoolId
      max_pool_id -= NumDynamicPools
    end

    # Create Pool IDs
    @@pool_ids = ["NULL_POOL"] + @@layouts.collect{|layout| layout.names}.flatten.uniq
    abort("Too many pool IDs.") if @@pool_ids.size - 1 > max_pool_id
  end

  def self.max_work_size
    @@layouts.collect{|layout| layout.work_size}.max
  end

  def self.output_macros(io)
    io.print("/*\n * Memory Manager max work area size\n */\n")
    io.printf("#define MEMMGR_MAX_WORK_SIZE  0x%08x\n\n", max_work_size)

    io.print("/*\n * Pool IDs\n */\n")
    @@pool_ids.each_with_index do |name, index|
      io.print("#define #{name}  #{index}\n")
    end
    io.print("\n")
    io.print("#define NUM_MEM_LAYOUTS  #{@@layouts.size}\n")
    io.print("#define NUM_MEM_POOLS  #{@@pool_ids.size}\n\n")
    if UseDynamicPool
      io.print("#define NUM_DYN_POOLS  #{NumDynamicPools}\n")
      io.print("#define DYN_POOL_WORK_SIZE(attr) \\\n")
      io.print(" ROUND_UP(sizeof(MemMgrLite::PoolAttr) + #{BasicPoolDataSize} +")
      io.print(" #{NumSegSize} * (attr).num_segs + #{SegRefCntSize} * (attr).num_segs, 4)\n")
    end

    io.print("\n/*\n * Pool areas\n */\n")
    @@layouts.each_with_index do |layout, index|
      io.print("/* Layout#{index}: */\n")
      io.printf("#define MEMMGR_L#{index}_WORK_SIZE   0x%08x\n\n", layout.work_size)
      layout.pools.each do |pool|
        io.printf("/* Skip 0x%04x bytes for alignment. */\n", pool.skip_size) if pool.skip_size > 0
        io.printf("#define L#{index}_#{pool.name}_ALIGN    0x%08x\n", pool.align)
        io.printf("#define L#{index}_#{pool.name}_L_FENCE  0x%08x\n", pool.begin_addr - FenceSize) if pool.fence_flag
        io.printf("#define L#{index}_#{pool.name}_ADDR     0x%08x\n", pool.begin_addr)
        io.printf("#define L#{index}_#{pool.name}_SIZE     0x%08x\n", pool.size)
        io.printf("#define L#{index}_#{pool.name}_U_FENCE  0x%08x\n", pool.begin_addr + pool.size) if pool.fence_flag
        io.printf("#define L#{index}_#{pool.name}_NUM_SEG  0x%08x\n", pool.num_seg)
        io.printf("#define L#{index}_#{pool.name}_SEG_SIZE 0x%08x\n", pool.size / pool.num_seg) if pool.type == Basic
        io.print("\n")
      end
      layout.used_area_info.each do |name_remainder|
        io.printf("/* Remainder #{name_remainder[0]}=0x%08x */\n", name_remainder[1])
      end
      io.print("\n")
    end
  end

  def self.output_table(io)
    io.print("MemPool* static_pools[NUM_MEM_POOLS];\n\n")
    io.print("extern const PoolAttr MemoryPoolLayouts[NUM_MEM_LAYOUTS][NUM_MEM_POOLS] = {\n")
    @@layouts.each_with_index do |layout, index|
      io.print(" {/* Layout:#{index} */\n")
      io.print("  /* pool_ID          type       seg")
      io.print(" fence") if UseFence
      io.print(" spinlock") if UseMultiCore
      io.print("  addr        size         */\n")
      layout.pools.each do |pool|
        io.printf("  { %-16s, %-6s, %3u", pool.name, pool.type, pool.num_seg)
        io.printf(", #{pool.fence_flag}") if UseFence
        io.printf(", #{pool.spinlock}") if UseMultiCore
        io.printf(", 0x%08x, 0x%08x", pool.begin_addr, pool.size)
        io.printf(" },  /* #{pool.area_entry.name} */\n")
      end
      io.print(" },\n")
    end
    io.print("}; /* end of MemoryPoolLayouts */\n\n")
  end
end

#######################################################################
class BaseFile
  def initialize(filename, description)
    @filename = filename
    @basename = File.basename(filename)

    @fh = open(filename, "wb")
    header_comment(description)
  end
  attr_reader :filename, :basename, :fh

  def header_comment(description)
    @fh.print <<-"EOB"
/****************************************************************************
 * #{basename}
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

    EOB
  end

  def close
    @fh.close
    @fh = nil
  end
end

#######################################################################
def output_macros(io)
  if EnableOutputMacro
    io.print("/*\n * Memory Manager Configurations\n */\n")
    io.print("#define USE_MEMMGR_FENCE\n")             if UseFence
    io.print("#define USE_MEMMGR_DYNAMIC_POOL\n")      if UseDynamicPool
    io.print("#define USE_MEMMGR_OVER255_SEGMENTS\n")  if UseOver255Segments
    io.print("#define USE_MEMMGR_COPIED_POOL_ATTR\n")  if UseCopiedPoolAttr
    io.print("#define USE_MEMMGR_SEG_DELETER\n")       if UseSegDeleter
    io.print("#define USE_MEMMGR_SEG_THRESHOLD\n")     if UseSegThreshold
    io.print("#define USE_MEMMGR_MULTI_CORE\n")        if UseMultiCore
    io.print("#define USE_MEMMGR_RINGBUF_POOL\n")      if UseRingBufPool
    io.print("#define USE_MEMMGR_RINGBUF_THRESHOLD\n") if UseRingBufThreshold
    io.print("\n")

    io.print("/*\n * User defined constants\n */\n")
    Object.constants.sort.each do |cnst|
      str = cnst.to_s  # Stored constants are strings up to v1.8, symbols after v1.9 are symbols
      io.print("#define #{str}  #{eval(str)}  /* 0x#{eval(str).to_s(16)} */\n") if str =~ /^U_MEM_/
    end
    io.print("\n")
  end
end

#######################################################################
class HeaderFile
  def self.create(filename, add_def)
    @@file = BaseFile.new(filename, "-- MemMgrLite layout definition.")
    @@guard_name = @@file.basename.upcase.tr(".", "_") + "_INCLUDED"
    self.fh.print("#ifndef #{@@guard_name}\n")
    self.fh.print("#define #{@@guard_name}\n\n")
    if add_def
      self.fh.print("#include \"memutils/memory_manager/MemMgrTypes.h\"\n")
      self.fh.print("\nnamespace MemMgrLite {\n\n")
    end
  end

  def self.basename
    @@file.basename
  end

  def self.fh
    @@file.fh
  end

  def self.close(add_def)
    self.fh.print("}  /* end of namespace MemMgrLite */\n\n") if add_def
    self.fh.print("#endif /* #{@@guard_name} */\n")
    @@file.close
  end
end

#######################################################################
def generate_files
  HeaderFile.create(ARGV[0], false)
  output_macros(HeaderFile.fh)
  MemoryDevices.output_macros(HeaderFile.fh)
  FixedAreas.output_macros(HeaderFile.fh)
  PoolAreas.output_macros(HeaderFile.fh)
  HeaderFile.close(false)

  HeaderFile.create(ARGV[1], true)
  FixedAreas.output_table(HeaderFile.fh)
  HeaderFile.close(true)

  HeaderFile.create(ARGV[2], true)
  PoolAreas.output_table(HeaderFile.fh)
  HeaderFile.close(true)
end

#######################################################################
# Main routine
#
if ARGV.size != 3
  abort("Usage: ruby #{File.basename($0)} layout_header fence_header pool_header\n" +
        "   Ex) ruby #{File.basename($0)} mem_layout.h  fixed_fence.h pool_layout.h\n")
end

if ARGV[0] == File.basename($0) or ARGV[0] == File.basename(__FILE__) or
   ARGV[1] == File.basename($0) or ARGV[1] == File.basename(__FILE__) or
   ARGV[2] == File.basename($0) or ARGV[2] == File.basename(__FILE__) or
   ARGV[0] == ARGV[1] or ARGV[0] == ARGV[2] or ARGV[1] == ARGV[2]
  abort("*** Error! Bad output file name ***")
end
