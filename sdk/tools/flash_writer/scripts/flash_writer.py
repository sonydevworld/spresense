#! /usr/bin/env python3

__copyright__ = ['Copyright (C) 2018, 2021 Sony Semiconductor Solutions Corp.']
__license__ = 'LGPL v2.1'

import time
import sys
import os
import struct
import glob
import fnmatch
import errno
import argparse
import shutil
import subprocess
import re
import xmodem

import_serial_module = True

# When SDK release, plase set SDK_RELEASE as True.
SDK_RELEASE = False

if SDK_RELEASE :
	PRINT_RAW_COMMAND = False
	REBOOT_AT_END = True
else :
	PRINT_RAW_COMMAND = True
	REBOOT_AT_END = True

try:
	import serial
except:
	import_serial_module = False

PROTOCOL_SERIAL = 0

MAX_DOT_COUNT = 70

# configure parameters and default value
class ConfigArgs:
	PROTOCOL_TYPE = PROTOCOL_SERIAL
	SERIAL_PORT = "COM1"
	EOL = bytes([10])
	DTR_RESET = False
	XMODEM_BAUD = 0
	NO_SET_BOOTABLE = False
	PACKAGE_NAME = []
	ERASE_NAME = []

ROM_MSG = [b"Welcome to nash"]
XMDM_MSG = "Waiting for XMODEM (CRC or 1K) transfer. Ctrl-X to cancel."

class ConfigArgsLoader():
	def __init__(self):
		self.parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
		self.parser.add_argument("package_name", help="the name of the package to install", nargs='*')
		self.parser.add_argument("-e", "--erase", dest="erase_name", help="erase file", action='append')

		self.parser.add_argument("-d", "--dtr-reset", dest="dtr_reset",
									action="store_true", default=None,
									help="try to auto reset develop board if possible")
		self.parser.add_argument("-n", "--no-set-bootable", dest="no_set_bootable",
									action="store_true", default=None,
									help="not to set bootable")

		group = self.parser.add_argument_group()
		group.add_argument("-c", "--serial-port", dest="serial_port", help="the serial port")
		group.add_argument("-b", "--xmodem-baudrate", dest="xmodem_baud", help="Use the faster baudrate in xmodem")

		mutually_group = self.parser.add_mutually_exclusive_group()
		mutually_group.add_argument("-s", "--serial-protocol", dest="serial_protocol",
									action="store_true", default=None,
									help="use the serial port for binary transmission, default options")

	def update_config(self):
		args = self.parser.parse_args()

		ConfigArgs.PACKAGE_NAME = args.package_name
		ConfigArgs.ERASE_NAME = args.erase_name

		if args.serial_protocol == True:
			ConfigArgs.PROTOCOL_TYPE = PROTOCOL_SERIAL

		if ConfigArgs.PROTOCOL_TYPE == None:
			ConfigArgs.PROTOCOL_TYPE = PROTOCOL_SERIAL

		if ConfigArgs.PROTOCOL_TYPE == PROTOCOL_SERIAL:
			if args.serial_port is not None:
				ConfigArgs.SERIAL_PORT = args.serial_port

		if args.xmodem_baud is not None:
			ConfigArgs.XMODEM_BAUD = args.xmodem_baud

		if args.dtr_reset is not None:
			ConfigArgs.DTR_RESET = args.dtr_reset

		if args.no_set_bootable is not None:
			ConfigArgs.NO_SET_BOOTABLE = args.no_set_bootable

class SerialDev:
	def __init__(self):
		if import_serial_module is False:
			print("Cannot import serial module, maybe it's not install yet.")
			print("\n", end="")
			print("Please install python-setuptool by Cygwin installer.")
			print("After that use easy_intall command to install serial module")
			print("    $ cd tool/")
			print("    $ python3 -m easy_install pyserial-2.7.tar.gz")
			quit()
		else:
			port = ConfigArgs.SERIAL_PORT
			try:
				self.serial = serial.Serial(port, baudrate=115200,
					parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
					bytesize=serial.EIGHTBITS, timeout=0.03)
			except Exception as e:
				print("Cannot open port : " + port)
				sys.exit(e.args[0])

	def readline(self, size=None):
		return self.serial.readline(size)

	def write(self, buffer):
		self.serial.write(buffer)
		self.serial.flush()

	def discard_inputs(self, timeout=1.0):
		time.sleep(timeout)
		self.serial.flushInput()

	def getc(self, size, timeout=1):
		self.serial.timeout = timeout
		c = self.serial.read(size)
		self.serial.timeout = 0.1
		return c

	def putc(self, buffer, timeout=1):
		self.serial.timeout = timeout
		self.serial.write(buffer)
		self.serial.flush()
		self.serial.timeout = 0.1
		self.show_progress(len(buffer))

	# Note: windows platform dependent code
	def putc_win(self, buffer, timeout=1):
		self.serial.write(buffer)
		self.show_progress(len(buffer))
		while True:
			if self.serial.out_waiting == 0:
				break

	def setBaudrate(self, baudrate):
#		self.serial.setBaudrate(baudrate)
		self.serial.baudrate = baudrate

	def reboot(self):
		# Target Reset by DTR
		self.serial.setDTR(False)
		self.serial.setDTR(True)
		self.serial.setDTR(False)

	def set_file_size(self, filesize):
		self.bytes_transfered = 0
		self.filesize = filesize
		self.count = 0

	def show_progress(self, sendsize):
		if PRINT_RAW_COMMAND:
			if self.count < MAX_DOT_COUNT:
				self.bytes_transfered = self.bytes_transfered + sendsize
				cur_count = int(self.bytes_transfered * MAX_DOT_COUNT / self.filesize)
				if MAX_DOT_COUNT < cur_count:
					cur_count = MAX_DOT_COUNT
				for idx in range(cur_count - self.count):
					print('#',end='')
					sys.stdout.flush()
				self.count = cur_count
				if self.count == MAX_DOT_COUNT:
					print("\n")

class FlashWriter:
	def __init__(self, protocol_sel=PROTOCOL_SERIAL):
		if protocol_sel == PROTOCOL_SERIAL:
			self.serial = SerialDev()

	def cancel_autoboot(self) :
		boot_msg = ''
		retry = 0
		self.serial.reboot()  # Target reboot before send 'r'
		while boot_msg == '' :
			if retry > 10:
				# If retry 10 times, reset spresense board
				self.serial.reboot()
				retry = 0
			rx = self.serial.readline().strip()
			self.serial.write(b"r")  # Send "r" key to avoid auto boot
			for msg in ROM_MSG :
				if msg in rx :
					boot_msg = msg
					break
			retry = retry + 1
		while True :
			rx = self.serial.readline().decode(errors="replace").strip()
			if "updater" in rx :
				# Workaround : Sometime first character is dropped.
				# Send line feed as air shot before actual command.
				self.serial.write(b"\n")    # Send line feed
				self.serial.discard_inputs()# Clear input buffer to sync
				return boot_msg.decode(errors="ignore")

	def recv(self):
		rx = self.serial.readline()
		if PRINT_RAW_COMMAND :
			serial_line = rx.decode(errors="replace")
			if serial_line.strip() != "" and not serial_line.startswith(XMDM_MSG):
				print(serial_line, end="")
		return rx

	def wait(self, string):
		while True:
			rx = self.recv()
			if string.encode() in rx:
				time.sleep(0.1)
				break

	def wait_for_prompt(self):
		prompt_pat = re.compile(b"updater")
		while True:
			rx = self.recv()
			if prompt_pat.search(rx):
				time.sleep(0.1)
				break

	def send(self, string):
		self.serial.write(str(string).encode() + b"\n")
		rx = self.serial.readline()
		if PRINT_RAW_COMMAND :
			print(rx.decode(errors="replace"), end="")

	def read_output(self, prompt_text) :
		output = []
		while True :
			rx = self.serial.readline()
			if prompt_text.encode() in rx :
				time.sleep(0.1)
				break
			if rx != "" :
				output.append(rx.decode(errors="ignore").rstrip())
		return output

	def install_files(self, files, command) :
		if ConfigArgs.XMODEM_BAUD:
			command += " -b " + ConfigArgs.XMODEM_BAUD
		if os.name == 'nt':
			modem = xmodem.XMODEM(self.serial.getc, self.serial.putc_win, 'xmodem1k')
		else:
			modem = xmodem.XMODEM(self.serial.getc, self.serial.putc, 'xmodem1k')
		for file in files:
			with open(file, "rb") as bin :
				self.send(command)
				print("Install " + file)
				self.wait(XMDM_MSG)
				print("|0%" + 
						"-" * (int(MAX_DOT_COUNT / 2) - 6) +
						"50%" +
						"-" * (MAX_DOT_COUNT - int(MAX_DOT_COUNT / 2) - 5) +
						"100%|")
				if ConfigArgs.XMODEM_BAUD:
					self.serial.setBaudrate(ConfigArgs.XMODEM_BAUD)
					self.serial.discard_inputs() # Clear input buffer to sync
				self.serial.set_file_size(os.path.getsize(file))
				modem.send(bin)
				if ConfigArgs.XMODEM_BAUD:
					self.serial.setBaudrate(115200)
			self.wait_for_prompt()

	def delete_files(self, files) :
		for file in files :
			self.delete_binary(file)

	def delete_binary(self, bin_name) :
		self.send("rm " + bin_name)
		self.wait_for_prompt()

def main():
	try:
		config_loader = ConfigArgsLoader()
		config_loader.update_config()
	except:
		return errno.EINVAL

	# Wait to reset the board
	writer = FlashWriter(ConfigArgs.PROTOCOL_TYPE)

	do_wait_reset = True

	if ConfigArgs.DTR_RESET:
		do_wait_reset = False
		bootrom_msg = writer.cancel_autoboot()

	if do_wait_reset == True:
		rx = writer.recv()
		time.sleep(1)
		for i in range(3):
			writer.send("")
			rx = writer.recv()
			if "updater".encode() in rx:
				# No need to wait for reset
				do_wait_reset = False
				break
			time.sleep(1)

	if do_wait_reset:
		# Wait to reset the board
		print('Please press RESET button on target board')
		sys.stdout.flush()
		bootrom_msg = writer.cancel_autoboot()

	# Remove files
	if ConfigArgs.ERASE_NAME :
		print(">>> Remove exisiting files ...")
		writer.delete_files(ConfigArgs.ERASE_NAME)

	# Install files
	if ConfigArgs.PACKAGE_NAME:
		print(">>> Install files ...")
	if ConfigArgs.PACKAGE_NAME :
		writer.install_files(ConfigArgs.PACKAGE_NAME, "install")

	# Set auto boot
	if not ConfigArgs.NO_SET_BOOTABLE:
		print(">>> Save Configuration to FlashROM ...")
		writer.send("set bootable M0P")
		writer.wait_for_prompt()

	# Sync all cached data to flash
	writer.send("sync")
	writer.wait_for_prompt()

	if REBOOT_AT_END :
		print("Restarting the board ...")
		writer.send("reboot")

	return 0

if __name__ == "__main__":
	try:
		sys.exit(main())
	except KeyboardInterrupt:
		print("Canceled by keyboard interrupt.")
		pass
