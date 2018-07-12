#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/config.py
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
import glob
import shutil
import re

import eula

MODE_MENUCONFIG = "menuconfig"
MODE_QCONFIG = "qconfig"
MODE_GCONFIG = "gconfig"

BOARDDIR = "board"
FEATUREDIR = "feature"
DEVDIR = "device"
EXAMPLESDIR = "examples"

def get_defconfigs(directory):
    return list(map(lambda x: os.path.basename(str(x)),
                    glob.glob(os.path.join(directory, '*-defconfig'))))

def install(src, dest, mode=0o644):
    logging.debug(src)
    logging.debug(dest)
    logging.debug(mode)

    shutil.copy(src, dest)
    os.chmod(dest, mode)
    return

def append(srcfile, destfile):
    # Read contents of destfile first

    with open(destfile, 'r') as dest:
        buf = dest.read()

    with open(srcfile, 'r') as src:
        with open(destfile, 'a') as dest:
            for line in src:
                # Append option if not exists

                if line not in buf:
                    logging.debug('write option: %s', line)
                    dest.write(line)

def apply_defconfig(configname, configlist, topdir, sdkdir, kernel):
    # Convert config names to "*-defconfig" and check it already exists

    defconfigs = list(map(lambda x: x + "-defconfig", configname))
    for c in defconfigs:
        if c not in configlist:
            print('Error: config "%s" not found' % c, file=sys.stderr)
            sys.exit(3)

    # Copy Make.defs file first, because SDK Makefile depends on Make.defs in
    # kernel, but there is nothing if kernel not configured.

    srcmakedefs = os.path.join(sdkdir, 'bsp', 'scripts', 'Make.defs.nuttx')
    destmakedefs = os.path.join(topdir, 'Make.defs')
    if not os.path.exists(destmakedefs):
        install(srcmakedefs, destmakedefs)

    if kernel:
        src = os.path.join(kconfigdir, defconfigs[0])
        dest = os.path.join(topdir, '.config')
        install(src, dest)
        postproc = 'make olddefconfigkernel'
    else:
        dest = os.path.join(sdkdir, '.config')

        # Create new empty .config file, existed file content will be discarded

        f = open(dest, 'w')
        f.close()

        for c in defconfigs:
            src = os.path.join(configdir, c)
            append(src, dest)
        postproc = 'make olddefconfig'

    if logging.getLogger().getEffectiveLevel() > logging.INFO:
        postproc += ' 2>&1 >/dev/null'
    ret = os.system(postproc)
    if ret != 0:
        print('Post process failed. %d' % ret)
        if kernel:
            print('Try \'make distcleankernel\' first.' )
        else:
            print('Try \'make clean\' first.' )
    return ret

def apply_spices(spices, configfile):
    with open(configfile, 'r') as src:
        buf = src.read()

    ENABLER = r'CONFIG_%s=y'
    DISABLER = r'# CONFIG_%s is not set'

    for spice in spices:
        flag, spice = spice[0], spice[1:]
        if flag == '+':
            repl = ENABLER % spice
        else:
            repl = DISABLER % spice

        r = re.compile(r'.*CONFIG_%s[= ].*' % spice, re.M)
        m = r.search(buf)
        if m:
            buf = r.sub(repl, buf)
        else:
            buf += repl

    with open(configfile, 'w') as dest:
        dest.write(buf)

def do_kconfig_conf(mode, sdkdir):
    ret = os.system('make %s' % mode)
    return ret

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Configration tool')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs='*',
                        help='configuration name')
    parser.add_argument('-k', '--kernel', action='store_true',
                        help='kernel config')
    parser.add_argument('-l', '--list', action='store_true',
                        help='list default configurations.\nshow kernel defconfigs with --kernel.')
    parser.add_argument('-m', '--menuconfig', action='store_true',
                        help='run config in "menuconfig"')
    parser.add_argument('-q', '--qconfig', action='store_true',
                        help='run config in "qconfig"')
    parser.add_argument('-g', '--gconfig', action='store_true',
                        help='run config in "gconfig"')
    parser.add_argument('-d', '--dir', type=str, nargs=1,
                        help='change configs directory')
    parser.add_argument('-v', '--verbose', action='count',
                        help='verbose messages')
    opts = parser.parse_args()

    loglevel = logging.WARNING
    if opts.verbose == 1:
        loglevel = logging.INFO
    if opts.verbose == 2:
        loglevel = logging.DEBUG
    logging.basicConfig(level=loglevel)

    menumode = None
    if opts.menuconfig: menumode = MODE_MENUCONFIG
    if opts.qconfig:    menumode = MODE_QCONFIG
    if opts.gconfig:    menumode = MODE_GCONFIG

    # Setup paths

    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))
    configdir = os.path.join(sdkdir, 'configs')
    kconfigdir = os.path.join(sdkdir, 'configs', 'kernel')

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        d = opts.dir[0]
        configdir = d
        kconfigdir = os.path.join(d, 'kernel')

        # Set 'default-defconfig' to configname list when it is not specified.
        # Only for base directory change option.

        if len(opts.configname) == 0:
            opts.configname = ['default']

    if opts.kernel:
        if not os.path.isdir(kconfigdir):
            print('Kernel config directory not found.', file=sys.stderr)
            sys.exit(3)

        configs = get_defconfigs(kconfigdir)
    else:
        if not os.path.isdir(configdir):
            print('Config directory not found.', file=sys.stderr)
            sys.exit(2)

        configs = get_defconfigs(configdir)

        # Get defconfigs from each reserved subdirectories and add
        # their subdirectory name to the prefix.

        l = get_defconfigs(os.path.join(configdir, BOARDDIR))
        bconfigs = list(map(lambda x: os.path.join(BOARDDIR, str(x)), l))
        l = get_defconfigs(os.path.join(configdir, FEATUREDIR))
        fconfigs = list(map(lambda x: os.path.join(FEATUREDIR, str(x)), l))
        l = get_defconfigs(os.path.join(configdir, DEVDIR))
        dconfigs = list(map(lambda x: os.path.join(DEVDIR, str(x)), l))
        l = get_defconfigs(os.path.join(configdir, EXAMPLESDIR))
        econfigs = list(map(lambda x: os.path.join(EXAMPLESDIR, str(x)), l))

        # Sort them before join

        bconfigs.sort()
        fconfigs.sort()
        dconfigs.sort()
        econfigs.sort()

        configs += bconfigs
        configs += fconfigs
        configs += dconfigs
        configs += econfigs

    if opts.list:
        print('Available configurations:')

        for c in configs:
            print('\t%s' % c.replace('-defconfig', ''))

        sys.exit(0)

    defconfigs = []
    spices = []
    if len(opts.configname) > 0:
        for c in opts.configname:
            if c.startswith('-') or c.startswith('+'):
                logging.info("    spice: %s", c)
                spices.append(c)
            else:
                logging.info("defconfig: %s", c)
                defconfigs.append(c)

    if len(defconfigs) > 0:
        ret = apply_defconfig(defconfigs, configs, topdir, sdkdir, opts.kernel)
        if ret != 0:
            sys.exit(ret)

        if not opts.kernel:
            # Check loader version
            eula_handler = eula.EULAhander()
            eula_handler.check()

    if len(spices) > 0:
        if opts.kernel:
            d = topdir
            target = 'olddefconfigkernel'
        else:
            d = sdkdir
            target = 'olddefconfig'

        apply_spices(spices, "%s/.config" % d)
        ret = os.system('make %s 2>&1 >/dev/null' % target)
        if ret != 0:
            sys.exit(ret)

    if menumode:
        if opts.kernel:
            menumode += 'kernel'
        do_kconfig_conf(menumode, sdkdir)

    # This tool needs mode option or config name

    if menumode == None and len(opts.configname) == 0:
        parser.print_usage()
        sys.exit(9)
