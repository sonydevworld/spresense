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

CAT_ROOT     = "configroot"
CAT_KERNEL   = "kernel"
CAT_BOARD    = "board"
CAT_FEATURE  = "feature"
CAT_DEVICE   = "device"
CAT_EXAMPLES = "examples"
CAT_MYAPP    = "myapp"

DEF_CATEGORY = [CAT_ROOT, CAT_BOARD, CAT_FEATURE, CAT_DEVICE, CAT_EXAMPLES, CAT_MYAPP]

#
# Configuration database
#
# Structure: {<category>:{<name>:<defconfig>, ...}, ...}
# e.g. {'default':{'default':'path/to/default-defconfig'},
#       'board':{'spresense':'path/to/spresense-defconfig'},
#       'feature':{'zmodem':'path/to/spresense-defconfig'},
#       ...}
configs = dict()

# Store defconfig information into database
def store_defconfig_db(category, name, defconfig):
    global configs

    if category not in configs:
        configs[category] = dict()

    configs[category][name] = defconfig

# Search all defconfig files from directory
def get_defconfigs(directory, category=CAT_ROOT, prefix=''):
    global configs

    # Search defconfig files

    # Search root defconfig files
    defconfigs = glob.glob(os.path.join(directory, '*-defconfig'))
    for defconfig in defconfigs:
        name = os.path.basename(defconfig).replace('-defconfig', '')
        name = os.path.join(prefix, name)
        if category != CAT_ROOT:
            name = os.path.join(category, name)
        store_defconfig_db(category, name, defconfig)

    # Search subdir defconfig files
    defconfigs = glob.glob(os.path.join(directory, '*', '*-defconfig'))
    for defconfig in defconfigs:
        category = os.path.basename(os.path.dirname(defconfig))
        name = os.path.basename(defconfig).replace('-defconfig', '')
        name = os.path.join(prefix, name)
        if category != CAT_KERNEL:
            name = os.path.join(category, name)
        store_defconfig_db(category, name, defconfig)

def get_all_defconfigs(configdir):
    # Pickup SDK configurations

    get_defconfigs(configdir)

    # Pickup user application configurations

    if 'SPRESENSE_HOME' in os.environ:
        myapp_root = os.environ['SPRESENSE_HOME']
        for app_config in glob.glob(os.path.join(myapp_root, '*', 'configs')):
            appname = os.path.basename(os.path.dirname(app_config))
            get_defconfigs(app_config, category=CAT_MYAPP, prefix=appname)

def get_defconfig_src(defconfig, kernel):
    if kernel:
        key = CAT_KERNEL
    else:
        path = defconfig.split('/')

        if len(path) == 1:
            # root configuration
            key = CAT_ROOT
        else:
            # Subset configuration
            key = path[0]

    if key not in configs or defconfig not in configs[key]:
        return None

    return configs[key][defconfig]

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

def enable_config(opt, config):
    os.system("kconfig-tweak --file %s --enable %s" % (config, opt))

def disable_config(opt, config):
    os.system("kconfig-tweak --file %s --disable %s" % (config, opt))

def tweak_platform(config):
    # Check kconfig-frontend installation

    ret = os.system("which kconfig-tweak > /dev/null")

    # If kconfig-frontend is missing, exit.

    if ret != 0:
        print("Error: kconfig-frontend is missing. Please setup your environment.")
        sys.exit(4)

    # Same as uname -s

    platform = os.uname()[0]

    # Ignore linux/mac because it will not be affected on ARM arch

    if re.match(r'CYGWIN_.*', platform):
        enable_config('HOST_WINDOWS', config)
        enable_config('TOOLCHAIN_WINDOWS', config)
        enable_config('WINDOWS_CYGWIN', config)
    elif re.match(r'MSYS_.*', platform):
        enable_config('HOST_WINDOWS', config)
        enable_config('TOOLCHAIN_WINDOWS', config)
        enable_config('WINDOWS_MSYS', config)
    elif re.match(r'MINGW.*', platform):
        print('Error: MinGW is not supported.')
        sys.exit(4)

def apply_defconfig(defconfigs, topdir, sdkdir, kernel):
    # Convert config names to "*-defconfig" and check it already exists

    for c in defconfigs:
        src = get_defconfig_src(c, kernel)
        if src == None:
            print('Error: config "%s" not found' % c, file=sys.stderr)
            sys.exit(3)

    # Copy Make.defs file first, because SDK Makefile depends on Make.defs in
    # kernel, but there is nothing if kernel not configured.

    srcmakedefs = os.path.join(sdkdir, 'bsp', 'scripts', 'Make.defs.nuttx')
    destmakedefs = os.path.join(topdir, 'Make.defs')
    if not os.path.exists(destmakedefs):
        install(srcmakedefs, destmakedefs)

    if kernel:
        src = get_defconfig_src(defconfigs[0], kernel)
        dest = os.path.join(topdir, '.config')
        install(src, dest)
        tweak_platform(dest)
        postproc = 'make olddefconfigkernel'
    else:
        dest = os.path.join(sdkdir, '.config')

        # Create new empty .config file, existed file content will be discarded

        f = open(dest, 'w')
        f.close()

        for c in defconfigs:
            src = get_defconfig_src(c, kernel)
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

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        d = opts.dir[0]
        configdir = d

        # Set 'default-defconfig' to configname list when it is not specified.
        # Only for base directory change option.

        if len(opts.configname) == 0:
            opts.configname = ['default']

    # pick-up defconfig files from SDK and user application

    get_all_defconfigs(configdir)

    if opts.kernel:
        if CAT_KERNEL not in configs:
            print('Kernel configuration not found.', file=sys.stderr)
            sys.exit(3)

    else:
        if len(configs.keys()) == 0:
            print('Configuration not found.', file=sys.stderr)
            sys.exit(2)

    if opts.list:
        print('Available configurations:')
        if opts.kernel:
            if CAT_KERNEL in configs:
                for defconfig in sorted(configs[CAT_KERNEL]):
                    print('\t%s' % defconfig)
        else:
            for subdir in DEF_CATEGORY:
                if subdir in configs:
                    for defconfig in sorted(configs[subdir]):
                        print('\t%s' % defconfig)

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
        ret = apply_defconfig(defconfigs, topdir, sdkdir, opts.kernel)
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
