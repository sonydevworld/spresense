#!/usr/bin/env python
############################################################################
# tools/callstack.py
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

import sys

syms = []

def get_symbol(x):
    try:
        addr = int(x, 16)
    except ValueError, e:
        return
    for num in range(len(syms)-1):
        if syms[num][0] <= addr and addr < syms[num+1][0]:
            return '[%08x] ' % (addr) + syms[num][1] + ' + 0x%x' % (addr - syms[num][0])

def main():
    argv = sys.argv
    argc = len(argv)

    if (argc < 3):
        print 'Usage: python %s <System.map> <stackdump.log>' % argv[0]
        quit()

    for line in open(argv[1], 'r'):
        address, type, symbol = line[:-1].split(' ')
        if type == 'T' or type == 't' or type == 'W' or type == 'w':
            syms.append((int(address, 16), symbol))

    callstack = []
    for line in open(argv[2], 'r'):
        print line[:-1]
        if 'up_stackdump:' in line:
            for item in line.split(' '):
                callstack.append(get_symbol(item))

    print '----------------- callstack -----------------'
    for cs in callstack:
        if cs is not None:
            print cs

if __name__ == '__main__':
    main()
