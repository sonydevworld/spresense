#!/usr/bin/env python3
# -*- coding:utf-8 -*-
############################################################################
# tools/eula.py
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

# eula.py:
# This tool is for checking and update firmware binaries that needed to take EULA.
# Usage: eula.py [-h] [-c] [-i spresense-binaries.zip]

import argparse
import json
import os
import re
import zipfile

# This script file path
SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))

# Firmware(loader.espk, gnssfw.espk) path
FIRMWARE_PATH = os.path.abspath(os.path.join(SCRIPT_PATH, "..", "..", "firmware"))

# SDK .config file path
CONFIG_PATH = os.path.abspath(os.path.join(SCRIPT_PATH, "..", ".config"))

# Json file name for describe required version
VERSION_JSON = "version.json"

# Json file name for describe stored version
STORED_VERSION_JSON = "stored_version.json"


# Name       : EULAhander
# Description: For check and extract EULA binaries
class EULAhander:

	# Name       : __init__
	# Description: Initialize
	def __init__(self):
		self.firmware_path = os.path.join(FIRMWARE_PATH, self.getBoardName())
		self.download_url = ""
		self.loader_version = ""
		self.loadVersion()

	# Name       : loadConfiguration
	# Description: Load SDK build configuration to dictionary
	# Return     : Configuratin dictonary
	def loadConfiguration(self):
		ret = {}

		# File open
		file = open(CONFIG_PATH)
		read = file.read()
		file.close()
		lines = read.split("\n")
		for line in lines:
			# Put configuration to dictionary
			if "=" in line:
				splt = line.split("=")
				left = splt[0]
				right = re.sub("\"", "", splt[1])
				ret[left] = right
		return ret

	# Name       : getBoardName
	# Description: Get board name from SDK build configuration
	# Return     : Board name
	def getBoardName(self):
		return self.loadConfiguration()["CONFIG_BOARD_NAME"]

	# Name       : loadVersion
	# Description: Load required version and download URL from version.json
	def loadVersion(self):
		version_file_name = os.path.join(self.firmware_path, VERSION_JSON)
		if os.path.isfile(version_file_name):
			version_file = open(version_file_name)
			version_json = json.load(version_file)
			version_file.close()
			version = version_json["LoaderVersion"]
			self.loader_version = version
			self.download_url = version_json["DownloadURL"]

	def getDownloadURL(self):
		return self.download_url

	def getLoaderVersion(self):
		return self.loader_version

	# Name       : check
	# Description: Check version for update and print warning.
	def check(self):
		is_need_to_update = False
		current_file_name = os.path.join(self.firmware_path, STORED_VERSION_JSON)

		if self.loader_version != "":
			if os.path.isfile(current_file_name):
				current_file = open(current_file_name)
				current_json = json.load(current_file)
				current_file.close()
				current_version = current_json["LoaderVersion"]
				if self.loader_version != current_version:
					is_need_to_update = True
			else:
				is_need_to_update = True

		if is_need_to_update:
			version = self.getLoaderVersion()
			url = self.getDownloadURL()
			print("WARNING: New loader %s is required, please download and install." % version)
			print("         Download URL   : %s" % url)
			print("         Install command:")
			print("                          1. Extract loader archive into host PC.")
			print("                             ./tools/flash.sh -e <download zip file>")
			print("                          2. Flash loader into Board.")
			print("                             ./tools/flash.sh -l %s -c <port>" % self.firmware_path)

	# Name       : update
	# Description: Update EULA binaries from zip archive
	def update(self, file_path):
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
				if update_version == self.loader_version:
					# If same with target version, do u	pdate
					binzip.extractall(self.firmware_path)
					print("Update succeed.")
				else:
					print("Error: Please use correct loader version (Selected: %s, Required: %s)." % (update_version, self.loader_version))
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

	eula_handler = EULAhander()

	if args.check:
		# for '-c'
		eula_handler.check()
	else:
		# for '-i'
		eula_handler.update(args.input)	
