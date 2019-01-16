############################################################################
# modules/memutils/message/tool/msgq_layout.rb
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

#######################################################################
# constants

ALINGMENT_SIZE = 4			# アラインメントサイズ
QUE_BLOCK_SIZE =  68			# メッセージキューブロックサイズ sizeof(MsgQueBlock)
MIN_PACKET_SIZE = 8			# 最小メッセージパケットサイズ
MAX_PACKET_SIZE = 512			# 最大メッセージパケットサイズ
MAX_PACKET_NUM = 16384			# 最大メッセージパケット数
INVALID_DRM = 0xffffffff		# 不正なDRMアドレス

# false:Not Support multi core , ture: Support multi core
USE_MULTI_CORE = false

#######################################################################
# Sub routines

def delete_file(name)
	File.delete(name) if File.exist?(name)
end

#######################################################################
def die(msg)
	delete_file(MsgqIdFile)
	delete_file(MsgqPoolFile)
	abort("#{$0}: *** Error! #{msg} ***\n#{$@}")
end

#######################################################################
def output_header_comment(out, title)
	out.print <<-"EOB"
/****************************************************************************
 * #{title}
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

#######################################################################
def make_include_guard_name(filename)
	name = File.basename(filename).upcase() + "_INCLUDED"
	return name.tr(".", "_")
end

#######################################################################
class DuplicationCheck
	def initialize
		@m_array = []
	end

	def exist?(obj)
		(result = @m_array.include?(obj)) or  @m_array.push(obj)
		return result
	end
end

#######################################################################
def output_msgq_pool(out, pools)
	out.print("extern const MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] = {\n")
	if (USE_MULTI_CORE == true)
		out.printf("   /* n_drm, n_size, n_num, h_drm, h_size, h_num, owner, spinlock */\n")
	else
		out.printf("   /* n_drm, n_size, n_num, h_drm, h_size, h_num */\n")
	end

	pools.each{|line|
		out.puts(line)
	}
	out.print("};\n\n")
end

#######################################################################
def make_msgq_pool_header(pools)
	title = File.basename(MsgqPoolFile)
	guard_name = make_include_guard_name(MsgqPoolFile)

	open(MsgqPoolFile, "wb"){|fh|
		output_header_comment(fh, title)
		fh.print("#ifndef #{guard_name}\n")
		fh.print("#define #{guard_name}\n\n")

		fh.print("#include \"#{File.basename(MsgqIdFile)}\"\n\n")
		output_msgq_pool(fh, pools)

		fh.print("#endif /* #{guard_name} */\n")
	}
end

#######################################################################
def cache_align(addr)
	return (addr + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)
end

#######################################################################
if (USE_MULTI_CORE == true)
	def collectItem(list, index)
		items = []
		list.each{|line|
			break if line == nil
			items.push(line[index])
		}
		return items
	end

	SpinLockNames = collectItem(SpinLockPool, 0)
end

#######################################################################
def getMsgQueParam(line, dup_chk)
	resv_ids = ["MSGQ_NULL", "MSGQ_TOP", "MSGQ_END"]

	id = line[0];
	raise("Empty MSGQ ID found") if id == ""
	raise("Bad ID found at #{id}") if id.index("MSGQ_") != 0
	raise("Reserved ID found") if resv_ids.include?(id)
	raise("Duplication ID at #{id}") if dup_chk.exist?(id)

	n_size	= line[1];
	raise("Bad n_size at #{id}") if n_size < MIN_PACKET_SIZE or n_size > MAX_PACKET_SIZE or n_size % 4 != 0
	n_size += 4 if MsgParamTypeMatchCheck == true and n_size > MIN_PACKET_SIZE

	n_num	= line[2];
	raise("Bad n_num at #{id}") if n_num == 0 or n_num > MAX_PACKET_NUM

	h_size	= line[3];
	raise("Bad h_size at #{id}") if h_size != 0 and (h_size < MIN_PACKET_SIZE or h_size > MAX_PACKET_SIZE or h_size % 4 != 0)
	h_size += 4 if MsgParamTypeMatchCheck == true and h_size > MIN_PACKET_SIZE

	h_num	= line[4];
	raise("Bad h_num at #{id}") if h_num > MAX_PACKET_NUM or (h_size > 0 and h_num == 0) or (h_size == 0 and h_num > 0)

	if (USE_MULTI_CORE == true)
		owner = line[5];
		raise("Bad owner at #{id}") if CpuIds.compact.include?(owner) == false
	end

	if (USE_MULTI_CORE == true)
		spinlock = line[6];
	end

	if (USE_MULTI_CORE == true)
		if (spinlock == "" or spinlock == "SPL_NULL")
			spinlock = "SPL_NULL"
		else
			raise("Bad spinlock at #{id}") if SpinLockNames.include?(spinlock) == false
			n_size = (n_size + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)
			h_size = (h_size + ALINGMENT_SIZE - 1) & ~(ALINGMENT_SIZE - 1)
		end
	end

	if (USE_MULTI_CORE == true)
		return id, spinlock, n_size, n_num, h_size, h_num, owner
	else
		return id, n_size, n_num, h_size, h_num
	end
end

#######################################################################
def parseMsgQuePool()
	macros = []
	if (USE_MULTI_CORE == true)
		pools = ["  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0, 0 }, /* MSGQ_NULL */\n"]
	else
		pools = ["  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0 }, /* MSGQ_NULL */\n"]
	end

	que_area_drm = START_DRM + QUE_BLOCK_SIZE	# 0番目は予約領域
	msg_area_drm = START_DRM + MsgQuePool.size * QUE_BLOCK_SIZE

	dup_chk = DuplicationCheck.new

	MsgQuePool.each{|line|
		break if line == nil

		if (USE_MULTI_CORE == true)
			id, spinlock, n_size, n_num, h_size, h_num, owner = getMsgQueParam(line, dup_chk)
		else
			id, n_size, n_num, h_size, h_num = getMsgQueParam(line, dup_chk)
		end

		if ((USE_MULTI_CORE == true && TargetCore == owner) || TargetCore == nil)
			n_drm = cache_align(msg_area_drm)
			h_drm = (h_size == 0) ? INVALID_DRM : n_drm + n_size * n_num
			msg_area_drm = n_drm + n_size * n_num + h_size * h_num
			(msg_area_drm <= LIMIT_DRM) or raise("Lack of pool area at " + id)

			# create macros for pool information
			macros.push("/************************************************************************/")
			macros.push("#{id}_QUE_BLOCK_DRM\t0x#{que_area_drm.to_s(16)}")
			macros.push("#{id}_N_QUE_DRM\t0x#{n_drm.to_s(16)}")
			macros.push("#{id}_N_SIZE\t#{n_size.to_s(10)}")
			macros.push("#{id}_N_NUM\t#{n_num.to_s(10)}")
			macros.push("#{id}_H_QUE_DRM\t0x#{h_drm.to_s(16)}")
			macros.push("#{id}_H_SIZE\t#{h_size.to_s(10)}")
			macros.push("#{id}_H_NUM\t#{h_num.to_s(10)}")
			if (USE_MULTI_CORE == true)
				macros.push("#{id}_OWNER\t#{owner}")
			end
			if (USE_MULTI_CORE == true)
				macros.push("#{id}_SPINLOCK\t#{spinlock}\n")
			end

			# create pool entry
			if (USE_MULTI_CORE == true)
				pools.push("  { 0x#{n_drm.to_s(16)}, #{n_size}, #{n_num}, 0x#{h_drm.to_s(16)}, #{h_size}, #{h_num}, #{owner}, #{spinlock} }, /* #{id} */\n")
			else
				pools.push("  { 0x#{n_drm.to_s(16)}, #{n_size}, #{n_num}, 0x#{h_drm.to_s(16)}, #{h_size}, #{h_num} }, /* #{id} */\n")
			end

			que_area_drm += QUE_BLOCK_SIZE
		end
	}
	return macros, pools, msg_area_drm
end

#######################################################################
def create_msgq_ids()
	ids = ["MSGQ_NULL"]
	MsgQuePool.each{|line|
		break if line == nil
		if (USE_MULTI_CORE == true)
			if (TargetCore == line[5] || TargetCore == nil)
				ids.push(line[0])
			end
		else
			ids.push(line[0])
		end
	}
	ids.push("NUM_MSGQ_POOLS")
	return ids
end

#######################################################################
def make_msgq_id_header(macros, end_addr)
	title = File.basename(MsgqIdFile)
	guard_name = make_include_guard_name(MsgqIdFile)

	open(MsgqIdFile, "wb"){|fh|
		output_header_comment(fh, title)
		fh.print("#ifndef #{guard_name}\n")
		fh.print("#define #{guard_name}\n\n")

		if (USE_MULTI_CORE == true)
			fh.print("#include \"spl_id.h\"\n\n")
		end

		fh.print("/* Message area size: #{end_addr - START_DRM} bytes */\n")
		fh.print("#define MSGQ_TOP_DRM\t0x#{START_DRM.to_s(16)}\n")
		fh.print("#define MSGQ_END_DRM\t0x#{end_addr.to_s(16)}\n\n")

		fh.print("/* Message area fill value after message poped */\n")
		fh.print("#define MSG_FILL_VALUE_AFTER_POP\t0x#{MsgFillValueAfterPop.to_s(16)}\n\n")

		fh.print("/* Message parameter type match check */\n")
		fh.print("#define MSG_PARAM_TYPE_MATCH_CHECK\t#{MsgParamTypeMatchCheck}\n\n")

		fh.print("/* Message queue pool IDs */\n")
		create_msgq_ids().each_with_index{|name, index|
			fh.print("#define #{name}\t#{index}\n")
		}
		fh.print("\n")

		fh.print("/* User defined constants */\n")
		Object.constants.sort.each{|cnst|
			str = cnst.to_s  # 格納される定数は、v1.8までは文字列で、v1.9以降はシンボル
			fh.print("#define #{str}\t#{eval(str)}\t/* 0x#{eval(str).to_s(16)} */\n") if str =~ /^U_MSGQ_/
		}
		fh.print("\n")

		macros.each{|name|
			if /^\/\*/ =~ name
				fh.print("#{name}\n")
			else
				fh.print("#define #{name}\n")
			end
		}

		fh.print("#endif /* #{guard_name} */\n")
	}
end

#######################################################################
def usage()
  abort("Usage:\n" +
        "ruby #{File.basename($0)} start_drm size id_header pool_header\n" +
        "             or\n" +
        "ruby #{File.basename($0)} fixed_file fixed_ID id_header pool_header\n\n" +
        "ex) ruby msgq_layout.conf 0x00800000 0x20000 msgq_id.h msgq_pool.h\n" +
        "             or\n" +
        "ruby msgq_layout.conf mem_fixed_layout.h MSG_QUE_AREA msgq_id.h msgq_pool.h\n" +
        "             or\n" +
        "ruby msgq_layout.conf mem_fixed_layout.h MSG_QUE_AREA msgq_id.h msgq_pool.h core_id")
end

#######################################################################
# Main routine
#
begin
	usage() if ARGV.length < 4

	MsgqIdFile	= ARGV[2]
	MsgqPoolFile	= ARGV[3]
	TargetCore	= ARGV[4]

	# 先頭文字が0以外ならば、Fixedメモリレイアウトのファイル名と仮定する
	if /^0/ !~ ARGV[0]
		# ファイルを解析して、ダンプ領域のアドレスとサイズを取得する
		drm_str	 = ARGV[1] + "_DRM"
		size_str = ARGV[1] + "_SIZE"
		msgq_drm  = ""
		msgq_size = ""
		open(ARGV[0], "rb"){|fh|
			fh.each{|line|
				msgq_drm  = line.split[2] if /^#define #{drm_str}/ =~ line
				msgq_size = line.split[2] if /^#define #{size_str}/ =~ line
			}
		}
		(msgq_drm  != "") or raise("#{drm_str} not found")
		(msgq_size != "") or raise("#{size_str} not found")
		START_DRM	= msgq_drm.hex()
		AREA_SIZE	= msgq_size.hex()
	else
		START_DRM	= ARGV[0].hex()
		AREA_SIZE	= ARGV[1].hex()
	end
	(START_DRM != 0 and START_DRM % ALINGMENT_SIZE == 0) or raise("Bad addr. (require 4bytes align) -- #{START_DRM.to_s(16)}")
	(AREA_SIZE != 0 and AREA_SIZE % ALINGMENT_SIZE == 0) or raise("Bad size. (require 4bytes align) -- #{AREA_SIZE.to_s(16)}")
	LIMIT_DRM = START_DRM + AREA_SIZE

	(MsgFillValueAfterPop <= 0xff) or raise("Bad MsgFillValueAfterPop.")
	[true, false].include?(MsgParamTypeMatchCheck) or raise("Bad MsgParamTypeMatchCheck.")

	macros, pools, end_addr = parseMsgQuePool()
	make_msgq_id_header(macros, end_addr)
	make_msgq_pool_header(pools)
rescue => e
	die(e.message)
end
