#!/usr/bin/env python3
# -*- coding:utf-8 -*-
############################################################################
# tools/bootloader.py
#
#   Copyright 2018,2020 Sony Semiconductor Solutions Corporation
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

# bootloader.py:
# This tool is for checking and update firmware binaries that needed to take EULA.
# Usage: bootloader.py [-h] [-c] [-i spresense-binaries.zip]

import argparse
import json
import os
import re
import zipfile

# This script file path
SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))

# Firmware(loader.espk, gnssfw.espk) path
FIRMWARE_PATH = os.path.abspath(os.path.join(SCRIPT_PATH, "..", "..", "firmware", "spresense"))

# Json file name for describe required version
VERSION_JSON = "version.json"

# Json file name for describe stored version
STORED_VERSION_JSON = "stored_version.json"


# Name       : BootloaderVersion
# Description: For bootloader check version and extract archive
class BootloaderVersion:

	# Name       : __init__
	# Description: Initialize
	def __init__(self):
		pass

	# Get version json file from file path
	def getVersionJson(self, filename):
		version_file = open(filename)
		version_json = json.load(version_file)
		version_file.close()
		return version_json

	# Get bootloader download URL from version.json path
	def getDownloadURL(self, filename):
		if os.path.isfile(filename):
			return self.getVersionJson(filename)["DownloadURL"];
		else:
			return None

	# Get bootloader version from json file path
	def getLoaderVersion(self, filename):
		if os.path.isfile(filename):
			return self.getVersionJson(filename)["LoaderVersion"];
		else:
			return None

	# Check update necessity
	def isNeedToUpdate(self):
		ret = False
		requre_version = self.getLoaderVersion(os.path.join(FIRMWARE_PATH, VERSION_JSON))
		current_version = self.getLoaderVersion(os.path.join(FIRMWARE_PATH, STORED_VERSION_JSON))

		if requre_version != None:
			if requre_version != current_version:
				ret = True
		return ret

	# Name       : check
	# Description: Check version for update and print warning.
	def checkBootloaderVersion(self):
		if self.isNeedToUpdate():
			version = self.getLoaderVersion(os.path.join(FIRMWARE_PATH, VERSION_JSON))
			url = self.getDownloadURL(os.path.join(FIRMWARE_PATH, VERSION_JSON))
			print("WARNING: New loader %s is required, please download and install." % version)
			print("         Download URL   : %s" % url)
			print("         Install command:")
			print("                          1. Extract loader archive into host PC.")
			print("                             ./tools/flash.sh -e <download zip file>")
			print("                          2. Flash loader into Board.")
			print("                             ./tools/flash.sh -l %s -c <port>" % FIRMWARE_PATH)

	# Name       : update
	# Description: Update EULA binaries from zip archive
	def updateBootloaderBinary(self, file_path):
		requre_version = self.getLoaderVersion(os.path.join(FIRMWARE_PATH, VERSION_JSON))
		# Check file is zip archive or not
		if file_path.endswith(".zip"):
			# Open zip archive
			binzip = zipfile.ZipFile(file_path)

			# Check stored_version.json contain or not
			if STORED_VERSION_JSON in binzip.namelist():
				# Check stored_version.json for compare with target version
				update_line = binzip.read(STORED_VERSION_JSON).decode('utf-8')
				update_json = json.loads(update_line)
				update_version = update_json["LoaderVersion"]
				if update_version == requre_version:
					# If same with target version, do u	pdate
					binzip.extractall(FIRMWARE_PATH)
					print("Update succeed.")
				else:
					print("Error: Please use correct loader version (Selected: %s, Required: %s)." % (update_version, requre_version))
			else:
				print("ERROR: Please select correct zip file")
		else:
			print("ERROR: Please select correct zip file")

if __name__ == "__main__":
	# Option
	parser = argparse.ArgumentParser(description=None)
	parser.add_argument('-c', '--check', action='store_true', help='Check loader version')
	parser.add_argument('-i', '--input', metavar='spresense-binaries.zip', type=str, help='Loader update zip file')
	args = parser.parse_args()

	bootloader_version = BootloaderVersion()

	if args.check:
		# for '-c'
		bootloader_version.checkBootloaderVersion()
	else:
		# for '-i'
		bootloader_version.updateBootloaderBinary(args.input)	
