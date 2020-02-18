#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/config.py
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
import glob
import shutil
import re

MODE_MENUCONFIG = "menuconfig"
MODE_QCONFIG = "qconfig"
MODE_GCONFIG = "gconfig"

SPRESENSE_HOME = 'SPRESENSE_HOME'

# defconfigs in 'configs' directory are not categorized
NO_CATEGORY = 'configs'

# Path to apps directory from nuttx
APPSDIR = '../sdk/apps'

class Defconfig:

    def __init__(self, path):
        self.path = os.path.realpath(path)
        fn = os.path.basename(path)
        dp = os.path.dirname(path)
        m = re.match(r'(?P<name>.*)-defconfig$', fn)
        if m:
            # Old style defconfigs (configs/*/*-defconfig)
            self.name = m.group("name")
            self.category = os.path.basename(dp)
        else:
            # Parent directory name as a defconfig name, and it's parent
            # treated as category. category name may 'configs' when nuttx configs
            # and user commands.

            self.name = os.path.basename(dp)
            pp = os.path.dirname(dp)
            self.category = os.path.basename(pp)

    def get_configname(self):
        if self.category == None or self.category == "configs":
            return self.name
        return "%s/%s" % (self.category, self.name)

    def __repr__(self):
        return "%s(name: %s, path: %s)" % (self.__class__.__name__,
                                           self.get_configname(), self.path)

class Defconfigs:

    def __init__(self, topdir, output=None, platform=None):
        self.defconfigs = []        # Store Defconfig objects
        self.category = ['configs'] # no category is set as 'configs'
        self.topdir = topdir        # Path to nuttx
        self.applies = []           # defconfigs list would be applied
        self.enables = []           # Enable options tweak list
        self.disables = []          # Disable options tweak list

        if output:
            self.dotconfig = output
        else:
            self.dotconfig = os.path.join(topdir, '.config')

        if platform:
            self.platform = platfrom
        else:
            self.platform = os.uname()[0] # Same as uname -s

    def __add_category(self, category):
        if category in self.category:
            return
        self.category += [category]

    def __get_old_styles(self, path):
        for f in glob.glob(os.path.join(path, "**/*-defconfig"), recursive=True):
            c = Defconfig(f)
            logging.debug(c)
            self.defconfigs += [c]
            self.__add_category(c.category)

    def __get_defconfig(self, path):
        files = glob.glob(os.path.join(path, "**/defconfig"), recursive=True)
        for f in files:
            c = Defconfig(f)
            logging.debug(c)
            self.defconfigs += [c]
            self.__add_category(c.category)

    def add_config_dirs(self, *args):
        '''
        Append directories to managed defconfigs list.
        Supplied directory expects path to "configs" directory name.
        '''
        for arg in args:
            self.__get_defconfig(arg)
            self.__get_old_styles(arg)

    def get_fullpath(self, name):
        for c in self.defconfigs:
            if name == c.get_configname():
                return c.path
        raise RuntimeError('Config "%s" not found' % name)

    def get_configs_by_category(self, category):
        if category == None:
            category = NO_CATEGORY

        l = []
        for c in self.defconfigs:
            if c.category == category:
                l += [c.get_configname()]
        return l

    def append(self, defconfig):
        for c in defconfig:
            if c.startswith('-'):
                self.disables += [c[1:]] # Kill prefix
            elif c.startswith('+'):
                self.enables += [c[1:]]  # Kill prefix
            else:
                self.applies += [c]

    def __tweak_platform(self, opts):
        # We need tweak options related to host environment.
        # This process is needed by NuttX build system.
        # See nuttx/tools/configure.sh.

        platform = self.platform

        if re.match(r'Darwin.*', platform):
            opts.add('CONFIG_HOST_MACOS=y\n')
        elif re.match(r'CYGWIN_.*', platform):
            opts.add('CONFIG_HOST_WINDOWS=y\n')
            opts.add('CONFIG_TOOLCHAIN_WINDOWS=y\n')
            opts.add('CONFIG_WINDOWS_CYGWIN=y\n')
        elif re.match(r'MSYS_.*', platform):
            opts.add('CONFIG_HOST_WINDOWS=y\n')
            opts.add('CONFIG_TOOLCHAIN_WINDOWS=y\n')
            opts.add('CONFIG_WINDOWS_MSYS=y\n')
        elif re.match(r'MINGW.*', platform):
            raise RuntimeError("MinGW currently not supported.")
        else:
            opts.add('CONFIG_HOST_LINUX=y\n')
        
    def apply(self, dest=None):
        if dest is None:
            dest = self.dotconfig

        logging.debug("Output config file at %s" % dest)

        # Use builtin set object for gathering options from multiple
        # defconfig files without duplication.

        opts = set()

        for c in self.applies:
            with open(self.get_fullpath(c), 'r') as f:
                for line in f:
                    opts.add(line)

        for o in self.enables:
            opts.discard('# CONFIG_%s is not set\n' % o)
            opts.add('CONFIG_%s=y\n' % o)
        for o in self.disables:
            opts.discard('CONFIG_%s=y\n' % o)
            opts.add('# CONFIG_%s is not set\n' % o)

        self.__tweak_platform(opts)
        opts.add('CONFIG_APPS_DIR="%s"\n' % APPSDIR)

        with open(dest, "w") as f:
            for o in opts:
                logging.debug(o)
                f.write(o)

def install(src, dest, mode=0o644):
    logging.debug(src)
    logging.debug(dest)
    logging.debug(mode)

    shutil.copy(src, dest)
    os.chmod(dest, mode)
    return

def do_olddefconfig():
    proc = 'make olddefconfig'

    if logging.getLogger().getEffectiveLevel() > logging.INFO:
        proc += ' 2>&1 >/dev/null'
    ret = os.system(proc)
    if ret != 0:
        print('Post process failed. %d' % ret)
        print('Try \'make distclean\' first.' )
    return ret

def prepare_config(topdir):
    # Copy Make.defs file first, because SDK Makefile depends on Make.defs in
    # kernel, but there is nothing if kernel not configured.

    srcmakedefs = os.path.join(topdir, 'boards', 'arm', 'cxd56xx', 'spresense', 'scripts', 'Make.defs')
    destmakedefs = os.path.join(topdir, 'Make.defs')
    if not os.path.exists(destmakedefs):
        install(srcmakedefs, destmakedefs)

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Configration tool')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs='*',
                        help='configuration name')
    parser.add_argument('-k', '--kernel', action='store_true',
                        help='deprecated')
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

    if opts.kernel:
        logging.warning("-k option is deprecated. Ignored.")

    menumode = None
    if opts.menuconfig: menumode = MODE_MENUCONFIG
    if opts.qconfig:    menumode = MODE_QCONFIG
    if opts.gconfig:    menumode = MODE_GCONFIG

    # Setup paths

    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))
    configdir = os.path.join(topdir, 'boards', 'arm', 'cxd56xx', 'spresense', 'configs')

    defconfigs = Defconfigs(topdir)

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        defconfigs.add_config_dirs(opts.dir[0])
    else:
        defconfigs.add_config_dirs(configdir, 'configs')
        if SPRESENSE_HOME in os.environ:
            defconfigs.add_config_dirs(os.environ[SPRESENSE_HOME])

    if opts.list:
        print('Available configurations:')
        for cat in (None, 'feature', 'device', 'examples'):
            configs = defconfigs.get_configs_by_category(cat)
            for c in configs:
                print("\t%s" % c)
        sys.exit(0)

    prepare_config(topdir)

    if len(opts.configname) > 0:
        defconfigs.append(opts.configname)
        defconfigs.apply()

        ret = do_olddefconfig()
        if ret != 0:
            sys.exit(ret)

    if menumode:
        os.system('make %s' % menumode)

    # This tool needs mode option or config name

    if menumode == None and len(opts.configname) == 0:
        parser.print_usage()
        sys.exit(9)
