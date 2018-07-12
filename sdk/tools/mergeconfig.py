#!/usr/bin/env python3
############################################################################
# tools/mergeconfig.py
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

import os
import sys
import argparse
import re

def getconfigname(str):
    m = re.match(r'# (?P<name>CONFIG_[\w]+) is .*', str)
    if m:
        return m.group('name')
    m = re.match(r'(?P<name>CONFIG_[\w]+)=y', str)
    if m:
        return m.group('name')
    return None

parser = argparse.ArgumentParser(description='')
parser.add_argument('kernelconfig', metavar='<kernel .config>', type=str, help='kernel .config file')
parser.add_argument('sdkconfig', metavar='<SDK .config>', type=str, help='SDK .config file')
parser.add_argument('tmpdir', metavar='tmpdir', type=str, nargs='?', default=".config.tmp")

opts = parser.parse_args()

# Check input .config files and setup work directory

if not os.path.exists(opts.kernelconfig):
    print("Kernel config %s not found." % opts.kernelconfig)
    sys.exit(1)

if not os.path.exists(opts.sdkconfig):
    print("SDK config %s not found." % opts.sdkconfig)
    sys.exit(1)

if os.path.exists(opts.tmpdir):
    if not os.path.isdir(opts.tmpdir):
        print("Temporary directory creation failure.")
        sys.exit(1)
else:
    os.makedirs(opts.tmpdir)


# Generate output file path

destconfigfile = os.path.join(opts.tmpdir, ".config")

with open(destconfigfile, 'w') as dest:
    # Read kernel configuration and output first.

    with open(opts.kernelconfig, 'r') as src:
        kernelconfig = src.read()
        dest.write(kernelconfig)

    # Append SDK configs except duplicated configs. (only for enabled)

    with open(opts.sdkconfig, 'r') as src:
        for line in src:
            cn = getconfigname(line)

            if cn and '%s=y' % cn in kernelconfig:
                continue
            dest.write(line)
