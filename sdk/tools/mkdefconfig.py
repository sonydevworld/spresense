#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/mkdefconfig.py
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
import logging
import re

CONFIG_APPS_DIR = 'CONFIG_APPS_DIR="../sdk/tools/empty_apps"'

def remove_config(opt, config):
    os.system("kconfig-tweak --file %s --undefine %s" % (config, opt))

def make_savedefconfig(d):
    ''' Run 'make savedefconfig' at specified directory
    '''
    command = 'make -C ' + d + ' savedefconfig'
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

def save_default_config(basedir, confpath, kernel):
    ret = make_savedefconfig(basedir)
    if ret != 0:
        print("Create defconfig failed. %d" % (ret), file=sys.stderr)
        sys.exit(ret)

    defconfig = os.path.join(basedir, 'defconfig')
    if os.path.exists(confpath):
        yes = confirm(configname + ' is already exists, overwrite? ')
        if not yes:
            sys.exit(0)

    logging.debug("Output: %s\n" % (confpath))
    if kernel:
        os.replace(defconfig, confpath)
        with open(confpath, 'a') as dest:
            dest.write(CONFIG_APPS_DIR + "\n")

        # Remove host related options

        remove_config('CONFIG_HOST_LINUX', confpath)
        remove_config('CONFIG_HOST_WINDOWS', confpath)
        remove_config('CONFIG_HOST_OSX', confpath)
        remove_config('CONFIG_HOST_MACOS', confpath) # for latest NuttX
        remove_config('TOOLCHAIN_WINDOWS', confpath)
        remove_config('WINDOWS_NATIVE', confpath)
        remove_config('WINDOWS_CYGWIN', confpath)
        remove_config('WINDOWS_UBUNTU', confpath)
        remove_config('WINDOWS_MSYS', confpath)
        remove_config('WINDOWS_OTHER', confpath)
    else:
        with open(confpath, 'w') as dest:
            with open(defconfig, 'r') as src:
                for line in src:
                    if not re.match(r'CONFIG_BOARD_.*', line):
                        dest.write(line)

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description='Make default config from current config')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs=1,
                        help='configuration name')
    parser.add_argument('-k', dest='kernel', action='store_true',
                        help='save kernel configuration')
    parser.add_argument('-d', '--dir', type=str, nargs=1,
                        help='change configs directory')
    parser.add_argument('--all', action='store_true',
                        help='Save SDK and kernel configuration with same name')
    parser.add_argument('--verbose', '-v', action='count',
                        help='verbose messages')

    opts = parser.parse_args()

    configname = opts.configname[0]

    savesdk = True
    savekernel = False
    if opts.kernel:
        savesdk = False
        savekernel = True
    if opts.all:
        savesdk = True
        savekernel = True

    loglevel = logging.WARNING
    if opts.verbose == 1:
        loglevel = logging.INFO
    if opts.verbose == 2:
        loglevel = logging.DEBUG
    logging.basicConfig(level=loglevel)

    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))

    # This script allows run in 'sdk'

    if not os.path.isdir('../nuttx'):
        print('Error: NuttX directory not found.', file=sys.stderr)
        sys.exit(1)

    # Test kernel and sdk already configured

    if savesdk and not os.path.exists(os.path.join(sdkdir, '.config')):
        print('Error: SDK not configured.', file=sys.stderr)
        sys.exit(2)

    if not os.path.exists(os.path.join(topdir, '.config')):
        print('Error: Kernel not configured.', file=sys.stderr)
        sys.exit(3)

    # Setup all of paths

    configdir = os.path.join(sdkdir, 'configs')
    kconfigdir = os.path.join(sdkdir, 'configs', 'kernel')

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        d = opts.dir[0]
        configdir = d
        kconfigdir = os.path.join(d, 'kernel')

    if savekernel:
        if not os.path.isdir(kconfigdir):
            print('Error: kernel configuration directory not found.', file=sys.stderr)
            sys.exit(4)
    elif savesdk:
        if not os.path.isdir(configdir):
            print('Error: configuration directory not found.', file=sys.stderr)
            sys.exit(5)

    logging.debug('Kernel dir: %s', topdir)
    logging.debug('SDK dir   : %s', sdkdir)
    logging.debug('Config dir: %s\n', configdir)

    # Do 'savedefconfig' target

    if savesdk:
        confpath = os.path.join(configdir, configname + '-defconfig')
        save_default_config(sdkdir, confpath, False)

    if savekernel:
        confpath = os.path.join(kconfigdir, configname + '-defconfig')
        save_default_config(topdir, confpath, True)
