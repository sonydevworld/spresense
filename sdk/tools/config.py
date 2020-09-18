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

import bootloader

MODE_MENUCONFIG = "menuconfig"
MODE_QCONFIG = "qconfig"
MODE_GCONFIG = "gconfig"

SPRESENSE_HOME = 'SPRESENSE_HOME'

# defconfigs in 'configs' directory are not categorized
NO_CATEGORY = 'configs'

# Path to apps directory from nuttx
APPSDIR = '"../sdk/apps"'

is_not_set = re.compile(r'^# (?P<symbol>.*) is not set')
is_config = re.compile(r'(?P<symbol>^CONFIG_.*)=(?P<value>.*)')

class DefconfigManager:

    def __init__(self, topdir, output=None, platform=None):
        self.defconfigs = []          # Store Defconfig objects
        self.category = [NO_CATEGORY] # no category is set as 'configs'
        self.topdir = topdir          # Path to nuttx
        self.applies = []             # defconfigs list would be applied
        self.enables = []             # Enable options tweak list
        self.disables = []            # Disable options tweak list
        self.base = None

        if output:
            self.dotconfig = output
        else:
            self.dotconfig = os.path.join(topdir, '.config')

    def __add_category(self, category):
        if category in self.category:
            return
        self.category += [category]

    def __get_old_styles(self, path):
        for f in glob.glob(os.path.join(path, "**/*-defconfig"), recursive=True):
            c = DefconfigManager.DefconfigPath(f)
            logging.debug(c)
            self.defconfigs += [c]
            self.__add_category(c.category)

    def __get_defconfig(self, path):
        files = glob.glob(os.path.join(path, "**/defconfig"), recursive=True)
        for f in files:
            c = DefconfigManager.DefconfigPath(f)
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

    class DefconfigPath:

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

class Defconfig:

    def __init__(self, name, manager):
        if name is None:
            name = 'default'
        self.path = manager.get_fullpath(name)
        self.man = manager
        self.load()

    def __is_hostenv(self, string):
        if re.match(r'[# ]*(CONFIG_|)HOST_LINUX', string): return True
        if re.match(r'[# ]*(CONFIG_|)HOST_WINDOWS', string): return True
        if re.match(r'[# ]*(CONFIG_|)HOST_MACOS', string): return True
        if re.match(r'[# ]*(CONFIG_|)HOST_OTHER', string): return True
        if re.match(r'[# ]*(CONFIG_|)WINDOWS_NATIVE', string): return True
        if re.match(r'[# ]*(CONFIG_|)WINDOWS_CYGWIN', string): return True
        if re.match(r'[# ]*(CONFIG_|)WINDOWS_MSYS', string): return True
        if re.match(r'[# ]*(CONFIG_|)WINDOWS_UBUNTU', string): return True
        if re.match(r'[# ]*(CONFIG_|)WINDOWS_OTHER', string): return True
        if re.match(r'[# ]*(CONFIG_|)SIM_X8664_MICROSOFT', string): return True
        if re.match(r'[# ]*(CONFIG_|)SIM_X8664_SYSTEMV', string): return True
        return False

    def load(self):
        self.opts = {}
        with open(self.path, 'r') as f:
            for line in f:
                if self.__is_hostenv(line):
                    continue

                m = is_not_set.match(line.strip())
                if m:
                    sym = m.group('symbol').replace('CONFIG_', '', 1)
                    self.opts[sym] = 'n'
                else:
                    m = is_config.match(line)
                    if m:
                        sym = m.group('symbol').replace('CONFIG_', '', 1)
                        self.opts[sym] = m.group('value')
                    else:
                        logging.debug('[IGNORE]: %s' % line.strip())

    def tweak_platform(self, platform=None):
        # We need tweak options related to host environment.
        # This process is needed by NuttX build system.
        # See nuttx/tools/configure.sh.

        if platform is None:
            platform = os.uname()[0] # Same as uname -s

        if re.match(r'Darwin.*', platform):
            self.opts['HOST_MACOS'] = 'y'
        elif re.match(r'CYGWIN_.*', platform):
            self.opts['HOST_WINDOWS'] = 'y'
            self.opts['TOOLCHAIN_WINDOWS'] = 'y'
            self.opts['WINDOWS_CYGWIN'] = 'y'
        elif re.match(r'MSYS_.*', platform):
            self.opts['HOST_WINDOWS'] = 'y'
            self.opts['TOOLCHAIN_WINDOWS'] = 'y'
            self.opts['WINDOWS_MSYS'] = 'y'
        elif re.match(r'MINGW.*', platform):
            raise RuntimeError("MinGW currently not supported.")
        else:
            self.opts['HOST_LINUX'] = 'y'

    def apply(self, opt):
        if opt.startswith('-') or opt.startswith('+'):
            # Apply single option tweak from command line
            self.__apply_config(opt)
        else:
            path = self.man.get_fullpath(opt)
            logging.debug("Apply defconfig %s" % path)
            with open(path, 'r') as f:
                for line in f:
                    self.__apply_config(line.rstrip())

    def __apply_config(self, config):
        val = ''
        if '=' in config:
            sym, val = config[1:].split('=')
        else:
            sym = config[1:]

        if self.__is_hostenv(sym):
            logging.debug('Ignore host environment option %s' % sym)
            return

        if config.startswith('+'):
            logging.debug("Add CONFIG_%s" % sym)
            if val == '':
                val = 'y'
            if sym in self.opts:
                logging.info("Overwrite CONFIG_%s to %s" % (sym, val))
            self.opts[sym] = val
        elif config.startswith('-'):
            logging.debug("Remove CONFIG_%s" % sym)
            if sym not in self.opts:
                logging.debug("CONFIG_%s is already removed." % sym)
                return
            if self.opts[sym] != val:
                logging.info("CONFIG_%s value mismatch '%s' != '%s'", sym, self.opts[sym], val)
            del self.opts[sym]
        elif config.startswith(' '):
            old, new = val.split('->')
            logging.debug("Change CONFIG_%s %s -> %s" % (sym, old, new))
            if sym not in self.opts:
                raise RuntimeError("Fatal: Applying defconfig not proceed")
            if self.opts[sym] != old:
                logging.info("Overwrite CONFIG_%s %s -> %s" % (sym, self.opts[sym], new))
            self.opts[sym] = new
        else:
            logging.debug('Unsupported config pattern "%s"' % config)

    def saveas(self, path):
        if 'HOST_WINDOWS' not in self.opts and 'HOST_LINUX' not in self.opts and 'HOST_MACOS' not in self.opts:
            self.tweak_platform()
        self.opts['APPS_DIR'] = APPSDIR

        with open(path, 'w') as f:
            for sym, val in self.opts.items():
                if val == 'n':
                    print('# CONFIG_%s is not set' % sym, file=f)
                else:
                    print('CONFIG_%s=%s' % (sym, val), file=f)
            
    def __repr__(self):
        return '%s (%s)' % (self.__class__.__name__, self.path)

def install(src, dest, mode=0o644):
    logging.debug(src)
    logging.debug(dest)
    logging.debug('mode: %o' % mode)

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

def prepare_config(topdir, sdkdir):
    # Copy Make.defs file first, because SDK Makefile depends on Make.defs in
    # kernel, but there is nothing if kernel not configured.

    srcmakedefs = os.path.join(sdkdir, 'tools', 'scripts', 'Make.defs')
    destmakedefs = os.path.join(topdir, 'Make.defs')
    install(srcmakedefs, destmakedefs)

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Configuration tool')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs='*',
                        help='configuration name')
    parser.add_argument('-k', '--kernel', action='store_true',
                        help='DEPRECATED')
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
    parser.add_argument('-i', '--info', metavar='config', type=str, nargs='*',
                        help='show configuration information')
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

    # Check python3 version
    if sys.version_info.minor < 5:
        print("Installed python3 is too old. Please update python3 to 3.5.0 or later.")
        sys.exit(1)

    # Setup paths

    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))

    manager = DefconfigManager(topdir)

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        manager.add_config_dirs(opts.dir[0])
    else:
        manager.add_config_dirs('configs')
        if SPRESENSE_HOME in os.environ and os.environ[SPRESENSE_HOME] != '':
            manager.add_config_dirs(os.environ[SPRESENSE_HOME])

    if opts.info and len(opts.info) > 0:
        for c in opts.info:
            path = manager.get_fullpath(c)
            readme = re.sub(r'defconfig$', 'README.txt', path)
            print('=== %s ===' % c)
            if os.path.exists(readme):
                print('[Description]')
                with open(readme, 'r') as f:
                    print(f.read())

            print('[Differences]')
            with open(path, 'r') as f:
                for line in f:
                    if re.match(r'^#', line) and not re.match(r'#.*not set', line):
                        continue
                    print(line.strip())
            print()
        sys.exit(0)

    if opts.list:
        print('Available configurations:')
        for cat in (None, 'feature', 'device', 'examples'):
            configs = manager.get_configs_by_category(cat)
            configs.sort()
            for c in configs:
                print("\t%s" % c)
        sys.exit(0)

    prepare_config(topdir, sdkdir)

    if len(opts.configname) > 0:
        clist = []
        base = None
        for c in opts.configname:
            if '/' not in c and not c.startswith('+') and not c.startswith('-'):
                if base is not None:
                    print("Error: %s can't be specify with %s\n" % (c, base), file=sys.stderr)
                    sys.exit(1)
                base = c
            else:
                clist.append(c)

        dest = Defconfig(base, manager)
        for c in clist:
            dest.apply(c)

        dest.saveas(os.path.join(topdir, '.config'))

        ret = do_olddefconfig()
        if ret != 0:
            sys.exit(ret)

    if menumode:
        os.system('make %s' % menumode)

    # This tool needs mode option or config name

    if menumode == None and len(opts.configname) == 0:
        parser.print_usage()
        sys.exit(9)

    # Since every developer use this script, to notice a necessity of bootloader update,
    # check the saved bootloader version and if necessary, show warning message.
    bootloader.BootloaderVersion().checkBootloaderVersion()
