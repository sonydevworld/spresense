#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/mkdefconfig.py
#
#   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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
import logging

def make_savedefconfig():
    ''' Run 'make savedefconfig' at specified directory
    '''
    command = 'make savedefconfig'
    if logging.getLogger().getEffectiveLevel() > logging.INFO:
        command += ' 2>&1 >/dev/null'

    logging.debug('command: "%s"', command)

    return os.system(command)

def confirm(msg):
    ''' Tell to user with msg
    '''
    while True:
        try:
            res = input(msg + ' (y/n)? ')
        except KeyboardInterrupt:
            print()
            sys.exit(0)
        if res == 'y':
            return True
        if res == 'n':
            return False

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description='Make default config from current config')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs=1,
                        help='configuration name')
    parser.add_argument('-k', dest='kernel', action='store_true',
                        help='save kernel configuration')
    parser.add_argument('-d', '--dir', type=str, nargs=1,
                        help='change configs directory')
    parser.add_argument('-y', action='store_true',
                        help='overwrite existing defconfig')
    parser.add_argument('--all', action='store_true',
                        help='Save SDK and kernel configuration with same name')
    parser.add_argument('--verbose', '-v', action='count',
                        help='verbose messages')

    opts = parser.parse_args()

    configname = opts.configname[0]

    loglevel = logging.WARNING
    if opts.verbose == 1:
        loglevel = logging.INFO
    if opts.verbose == 2:
        loglevel = logging.DEBUG
    logging.basicConfig(level=loglevel)

    if opts.kernel:
        logging.warning("-k option is deprecated. Ignored.")
    if opts.all:
        logging.warning("--all option is deprecated. Ignored.")

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        path = os.path.realpath(opts.dir[0])
        if path.endswith('configs'):
            configdir = os.path.join(path, configname)
        else:
            configdir = os.path.join(path, 'configs', configname)
    else:
        configdir = os.path.join('configs', configname)

    if os.path.exists(configdir):
        if not os.path.isdir(configdir):
            print("configuration couldn't saved.")
            sys.exit(1)
        else:
            if not opts.y:
                yes = confirm(configname + ' is already exists, overwrite? ')
                if not yes:
                    sys.exit(0)

    # Create defconfig file by savedefconfig, this target moves defconfig
    # from nuttx/ to sdk/.
    
    ret = make_savedefconfig()
    if ret:
        sys.exit(ret)

    # Move generated defconfig file to target directory

    os.makedirs(configdir, exist_ok=True)
    os.replace('defconfig', os.path.join(configdir, 'defconfig'))
