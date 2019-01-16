############################################################################
# modules/memutils/message/tool/msgq_dump.rb
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

require "analyze_util.rb"		# hex_dump

#######################################################################
# constants

QUE_BLOCK_SIZE = 64		# メッセージキューブロックサイズ
MIN_PACKET_SIZE = 8		# 最小メッセージパケットサイズ
INVALID_DRM = 0xffffffff	# 不正なDRMアドレス

#######################################################################
def die(msg)
	abort("#{$0}: *** Error! #{msg} ***\n#{$@}")
end

#######################################################################
def collect_dump_info(msgq_header)
	ids = []
	params = {}

	open(msgq_header, "rb"){|fh|
		state = :init
		fh.each{|line|
			case state
			when :init
				params[line.split[1]] = line.split[2].hex() if /^#define MSGQ_TOP_DRM/ =~ line
				params[line.split[1]] = line.split[2].hex() if /^#define MSGQ_END_DRM/ =~ line
				params[line.split[1]] = line.split[2] if /^#define MSG_PARAM_TYPE_MATCH_CHECK/ =~ line
				state = :pool_id if /^#define MSGQ_NULL/ =~ line
			when :pool_id
				# IDは、"MSGQ_NULL"と"NUM_MSGQ_POOLS"の間に定義されている
				ids.push(line.split[1]) if /^#define MSGQ_/ =~ line
				state = :pool_attr if /^#define NUM_MSGQ_POOLS/ =~ line
			when :pool_attr
				params[line.split[1]] = line.split[2].hex() if /_DRM\t/ =~ line
				params[line.split[1]] = line.split[2].to_i() if /_SIZE\t/ =~ line
				params[line.split[1]] = line.split[2].to_i() if /_NUM\t/ =~ line
				params[line.split[1]] = line.split[2] if /_OWNER\t/ =~ line
				params[line.split[1]] = line.split[2] if /_SPINLOCK\t/ =~ line
			end
		}
	}
	return ids, params
end

#######################################################################
def dump_elem(seq, elem_data)
	type, reply, src_cpu, flags, param_size = elem_data.unpack((Endian == "BE") ? "n2c2n" : "v2c2v")
	printf("[%3u] type=0x#{type.to_s(16)} reply=0x#{reply.to_s(16)} src_cpu=#{src_cpu} flags=#{flags} param_size=#{param_size}\n", seq)
	hex_dump(elem_data)
	print("\n")
end

#######################################################################
def dump_que(title, data, ctrl_offset)
	drm, elem, capa, put, get, count = data[ctrl_offset, 14].unpack((Endian == "BE") ? "Nn5" : "Vv5")
	print("#{title}: drm=0x#{drm.to_s(16)} elem=#{elem} capa=#{capa} put=#{put} get=#{get} size=#{count}\n")
	if drm != INVALID_DRM
		from_offset = drm - TopDrm
		to_offset = from_offset + elem * capa - 1

		seq = 0
		from_offset.step(to_offset, elem){|offset|
			dump_elem(seq, data.slice(offset, elem))
			seq += 1
		}
	end
end

#######################################################################
def dump_one_id(data, params, id)
	que_block_offset = params["#{id}_QUE_BLOCK_DRM"] - TopDrm
	n_que_offset = params["#{id}_N_QUE_DRM"] - TopDrm
	h_que_offset = params["#{id}_H_QUE_DRM"] - TopDrm

	que_id, init, owner, spinlock, sem, pending = data[que_block_offset, 10].unpack((Endian == "BE") ? "nc2n3" : "vc2v3")
	cur_que, total_pend, max_pend, max_n_que, max_h_que = data[que_block_offset + 44, 14].unpack((Endian == "BE") ? "N2n3" : "V2v3")
	print("/// ID:#{que_id} init=#{init} owner=#{owner} spinlock=#{spinlock} sem=#{sem} pending=#{pending} cur_que=0x#{cur_que.to_s(16)} ///\n")
	print("Tally: total_pending=#{total_pend} max_pending=#{max_pend} max_queuing=#{max_n_que}, #{max_h_que}\n\n")
	dump_que("Normal priority queue", data, que_block_offset + 12)
	dump_que("High priority queue", data, que_block_offset + 28)
end

#######################################################################
def dump_bin(ids, params, bin_file)
	# read all data
	data = IO.read(bin_file)
	raise("Too small data size -- #{bin_file}") if data.size < params["MSGQ_END_DRM"] - TopDrm

	# check header
	raise("Bad MSG_LIB_NAME -- #{bin_file}") if data[0, 6] != "MsgLib"
	raise("Bad MSG_LIB_VER -- #{bin_file}") if data[6, 4] != "2.00"

	ids.each{|id|
		dump_one_id(data, params, id)
	}
end

#######################################################################
def usage()
  abort("Usage: ruby #{File.basename($0)} msgq_header dump_file [BE or LE]\n" +
        "   ex) ruby #{File.basename($0)} msgq_id.h msgq_log.bin BE\n")
end

#######################################################################
# Main routine
#
begin
	usage() if ARGV.length < 2

	msgq_header	= ARGV[0]
	bin_file	= ARGV[1]

	# エンディアン指定なしや"LE"以外ならば、BigEndianとみなす
	Endian = (ARGV[2] == "LE") ? "LE" : "BE"

	ids, params = collect_dump_info(msgq_header)
	raise("Bad format -- #{msgq_header}") if ids.size < 1 or params.size < 12

	TopDrm = params["MSGQ_TOP_DRM"]
	dump_bin(ids, params, bin_file)

rescue => e
	die(e.message)
end
