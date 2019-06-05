#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
# tools/mkcmd.py
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

TOOL_DESCRIPTION = '''
Create a new application
'''

EPILOG = '''This tool create a new application at examples directory.
You can use '-d' option to change application directory, it is a same level
of sdk and examples.
'''

KCONFIG_TMPL = '''
config {configname}
	bool "{menudesc}"
	default n
	---help---
		Enable the {appname} app

if {configname}

config {configname}_PROGNAME
	string "Program name"
	default "{appname}"
	depends on BUILD_KERNEL
	---help---
		This is the name of the program that will be use when the NSH ELF
		program is installed.

config {configname}_PRIORITY
	int "{appname} task priority"
	default 100

config {configname}_STACKSIZE
	int "{appname} stack size"
	default 2048

endif
'''

MAKEFILE_TMPL = '''
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

CONFIG_{configname}_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_{configname}_STACKSIZE ?= 2048

APPNAME = {appname}
PRIORITY = $(CONFIG_{configname}_PRIORITY)
STACKSIZE = $(CONFIG_{configname}_STACKSIZE)

ASRCS =
CSRCS =
MAINSRC = {appname}_main.c

CONFIG_{configname}_PROGNAME ?= {appname}$(EXEEXT)
PROGNAME = $(CONFIG_{configname}_PROGNAME)

include $(APPDIR)/Application.mk
'''

MAKEDEFS_TMPL = '''
ifeq ($(CONFIG_{configname}),y)
CONFIGURED_APPS += {appname}
endif
'''

MAINCSRC_TMPL = '''
#include <sdk/config.h>
#include <stdio.h>

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int {appname}_main(int argc, char *argv[])
#endif
{{
  return 0;
}}
'''

GITIGNORE = '''/Make.dep
/.depend
/.built
/*.asm
/*.obj
/*.rel
/*.lst
/*.sym
/*.adb
/*.lib
/*.src
'''

def create_from_template(template, filename, appname, configname, menudesc=None):
    with open(filename, "w") as f:
        f.write(template.format(appname=appname, configname=configname,
                                menudesc=menudesc))
def create_defconfig_file(configname):
    os.mkdir('configs')
    os.system('touch configs/default-defconfig')
    os.system("kconfig-tweak --file %s --enable CONFIG_%s" % ('configs/default-defconfig', configname))

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=TOOL_DESCRIPTION,
                                     epilog=EPILOG)
    parser.add_argument('appname', metavar='<app name>', type=str, help='New application name')
    parser.add_argument('desc', type=str, nargs="?", help='Menu description')
    parser.add_argument('-c', '--create_config', action='store_true', default=False,
                        help='create default defconfig file into application directory')
    parser.add_argument('-d', '--basedir', type=str, default='examples',
                        help='Base directory to create new application')
    parser.add_argument('-v', '--verbose', action='count', default=0, help='Verbose messages')
    opts = parser.parse_args()

    verbose = opts.verbose

    appname = opts.appname
    optprefix = os.path.basename(os.path.normpath(opts.basedir)).upper()
    configname = optprefix + '_' + appname.upper()
    maincsrcfile = appname + '_main.c'
    basedir = os.path.join('..', opts.basedir)
    targetdir = appname
    createconf = opts.create_config
    if opts.desc:
        menudesc = opts.desc
    else:
        menudesc = '%s app' % appname

    os.chdir(basedir)

    # Sanity checks

    if re.search(r'\s', opts.appname):
        print('Any white spaces not allowed in <appname>', file=sys.stderr)
        sys.exit(1)

    if not os.path.exists('Application.mk'):
        print("This tool must run on the directory where 'Application.mk' file exists",
              file=sys.stderr)
        sys.exit(2)

    try:
        os.mkdir(targetdir)
    except:
        print('%s already exists' % targetdir, file=sys.stderr)
        sys.exit(3)

    # Create application

    os.chdir(targetdir)

    create_from_template(KCONFIG_TMPL, 'Kconfig', appname, configname, menudesc)
    create_from_template(MAKEFILE_TMPL, 'Makefile', appname, configname)
    create_from_template(MAKEDEFS_TMPL, 'Make.defs', appname, configname)
    create_from_template(MAINCSRC_TMPL, maincsrcfile, appname, configname)

    # Create defconfig file

    if createconf:
        create_defconfig_file(configname)

    with open('.gitignore', "w") as f:
        f.write(GITIGNORE)

    print("New '%s' app successfully created at '%s'." % (appname, os.path.join(basedir, appname)))

    if os.path.isfile(os.path.join('..', 'Kconfig')):
        print("Please 'make clean' from sdk first.")
