#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/mkappsdir.py
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
import re
import shutil

TOOL_DESCRIPTION = '''
Create a new application directory at the outside of sdk repository

This tool must be run on sdk directory.
'''

REFERENCE = 'examples'

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=TOOL_DESCRIPTION)
    parser.add_argument('dirname', metavar='<dir name>', type=str,
                        help='New application directory name')
    parser.add_argument('desc', type=str, nargs="?", help='Menu description')
    parser.add_argument('-f', '--force', action='store_true', default=False,
                        help='force to create application directory')
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='verbose messages')
    opts = parser.parse_args()

    verbose = opts.verbose

    sdkdir = os.path.abspath(os.path.join(sys.argv[0], '..', '..'))
    srcdir = os.path.join(sdkdir, '..', REFERENCE)

    #
    # The target directory strategy is:
    # 1. absolute path or relative path => use them
    # 2. just a name => sibling of sdk directory
    #
    if opts.dirname.startswith('/') or opts.dirname.startswith('.'):
        targetdir = opts.dirname
    else:
        targetdir = os.path.join(sdkdir, '..', opts.dirname)

    # Sanity checks

    if re.search(r'\s', opts.dirname):
        print('Any white spaces not allowed in <dir name>', file=sys.stderr)
        sys.exit(1)

    if not os.path.exists(srcdir):
        print('examples directory not found', file=sys.stderr)
        sys.exit(2)

    try:
        os.mkdir(targetdir)
    except:
        if not opts.force:
            print('%s already exists' % targetdir, file=sys.stderr)
            sys.exit(3)

    # Required files for extending application series to outside of sdk repos.

    filelist = ['Makefile', 'Make.defs', '.gitignore', '.sdksubdir']

    for f in filelist:
        src = os.path.join(srcdir, f)
        dst = os.path.join(targetdir, f)
        if verbose > 0:
            print('Copying file %s -> %s' % (src, dst))
        shutil.copyfile(src, dst)

    # Finally, replace menu description

    makefile = os.path.join(targetdir, 'Makefile')

    if opts.desc:
        desc = opts.desc
    else:
        desc = opts.dirname.capitalize()

    pat = re.compile(r'MENUDESC\s*=\s*".*"')
    with open(makefile, "r") as f:
        buf = f.read()
    buf = pat.sub('MENUDESC = "%s"' % desc, buf)
    with open(makefile, "w") as f:
        f.write(buf)
