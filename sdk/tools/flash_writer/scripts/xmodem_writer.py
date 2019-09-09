#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# TODO: This script will be merge in flash_writer.py
#
__copyright__ = ['Copyright (C) 2018, 2019 Sony Semiconductor Solutions Corp.']
__license__ = 'LGPL v2.1'

import time
import sys
import os
import errno
import argparse
import re
import xmodem

import_serial_module = True

try:
	import serial
except:
	import_serial_module = False

XMODEM_VERSION = "1.0.0"
MAX_DOT_COUNT = 70
XMODEM_BUFF_SIZE = 128 * 1024
PRINT_RAW_COMMAND = True
REBOOT_AT_END = True
DEST_DIR="/mnt/spif"

# configure parameters and default value
class ConfigArgs:
	SERIAL_PORT = "COM1"
	EOL = bytes([10])
	WAIT_RESET = True
	DTR_RESET = False
	XMODEM_BAUD = 0
	PACKAGE_NAME = []

NSH_MSG = "nsh>"

class ConfigArgsLoader():
	def __init__(self):
		self.parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
		self.parser.add_argument("package_name", help="the name of the package to install", nargs='*')

		self.parser.add_argument("-d", "--dtr-reset", dest="dtr_reset",
									action="store_true", default=None,
									help="try to auto reset develop board if possible")

		group = self.parser.add_argument_group()
		group.add_argument("-c", "--serial-port", dest="serial_port", help="the serial port")
		group.add_argument("-b", "--xmodem-baudrate", dest="xmodem_baud", help="Use the faster baudrate in xmodem")

		mutually_group2 = self.parser.add_mutually_exclusive_group()
		mutually_group2.add_argument("-F", "--force-wait-reset", dest="wait_reset",
									action="store_true", default=None,
									help="force wait for pressing RESET button")
		mutually_group2.add_argument("-N", "--no-wait-reset", dest="wait_reset",
									action="store_false", default=None,
									help="if possible, skip to wait for pressing RESET button")

	def update_config(self):
		args = self.parser.parse_args()

		ConfigArgs.PACKAGE_NAME = args.package_name

		if args.serial_port is not None:
			ConfigArgs.SERIAL_PORT = args.serial_port

		if args.xmodem_baud is not None:
			ConfigArgs.XMODEM_BAUD = args.xmodem_baud

		if args.dtr_reset is not None:
			ConfigArgs.DTR_RESET = args.dtr_reset

		if args.wait_reset is not None:
			ConfigArgs.WAIT_RESET = args.wait_reset

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
					bytesize=serial.EIGHTBITS, timeout=0.1, dsrdtr=False)
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
		self.serial.setBaudrate(baudrate)
		self.serial.baudrate = baudrate

	def reboot(self):
		# Target Reset by DTR
		self.serial.setDTR(False)
		self.serial.setDTR(True)
		self.serial.setDTR(False)
		pass

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

class StreamSplitter:
	def __init__(self, stream):
		self.body = stream.read()

	def pop(self):
		isEof = False
		self.stage = self.body[:XMODEM_BUFF_SIZE-1]

		if len(self.body) <= XMODEM_BUFF_SIZE-1:
			self.stage = self.stage + b'%c' % (0xFF)
			isEof = True
		else:
			self.stage = self.stage + b'%c' % (0xFE)

		self.body = self.body[XMODEM_BUFF_SIZE-1:]

		return isEof

	def read(self, size):
		ret = self.stage[:size]
		self.stage = self.stage[size:]
		return ret

class XmodemWriter:
	def __init__(self):
		self.serial = SerialDev()

	def check_sysutil_sw(self):
		self.serial.write(b"\n")
		self.serial.discard_inputs()
		self.send("xmodem")
		rx = self.serial.readline().decode(errors="replace").strip()
		if not "Version: %s" % XMODEM_VERSION in rx:
			print("Error: Please flash new bootloader.")
			sys.exit(-1)
		

	def cancel_autoboot(self) :
		# Retry while 10 seconds. If expire retry count, will exit
		retry = 100
		self.serial.reboot()  # Target reboot before send '%'
		while retry > 0 :
			rx = self.serial.readline().strip()
			self.serial.write(b"%")  # Send "%" key to avoid auto boot
			if NSH_MSG.encode() in rx :
				self.check_sysutil_sw()
				break
			retry = retry - 1

		if retry == 0:
			print("Error: Please flash new bootloader.");
			sys.exit(-2)

		while True :
			rx = self.serial.readline().decode(errors="replace").strip()
			if NSH_MSG in rx :
				break
			# Workaround : Sometime first character is dropped.
			# Send line feed as air shot before actual command.
			self.serial.write(b"\n")    # Send line feed

	def recv(self):
		rx = self.serial.readline()
		if PRINT_RAW_COMMAND :
			serial_line = rx.decode(errors="replace")
			if serial_line.strip() != "":
				print(serial_line, end="")
		return rx

	def wait(self, string):
		while True:
			rx = self.recv()
			if string.encode() in rx:
				time.sleep(0.1)
				break

	def wait_for_prompt(self):
		prompt_pat = re.compile(NSH_MSG.encode())
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

	def install_files(self, files, command) :
		if ConfigArgs.XMODEM_BAUD:
			command += " -b " + ConfigArgs.XMODEM_BAUD
		if os.name == 'nt':
			modem = xmodem.XMODEM(self.serial.getc, self.serial.putc_win, 'xmodem1k')
		else:
			modem = xmodem.XMODEM(self.serial.getc, self.serial.putc, 'xmodem1k')
		for file in files:
			with open(file, "rb") as bin :
				filename = os.path.basename(file)
				self.send("%s %s/%s" % (command, DEST_DIR, filename))
				print("Install " + file)
				print("|0%" + 
						"-" * (int(MAX_DOT_COUNT / 2) - 6) +
						"50%" +
						"-" * (MAX_DOT_COUNT - int(MAX_DOT_COUNT / 2) - 5) +
						"100%|")
				if ConfigArgs.XMODEM_BAUD:
					self.serial.setBaudrate(ConfigArgs.XMODEM_BAUD)
				self.serial.discard_inputs() # Clear input buffer to sync
				self.serial.set_file_size(os.path.getsize(file))

				streamSplitter = StreamSplitter(bin)

				while True:
					# Pop buffer
					eof = streamSplitter.pop()
					# Send data from buffer via xmodem
					modem.send(streamSplitter)
					if eof:
						break

				if ConfigArgs.XMODEM_BAUD:
					self.serial.setBaudrate(115200)
			self.wait_for_prompt()

def main():
	try:
		config_loader = ConfigArgsLoader()
		config_loader.update_config()
	except:
		return errno.EINVAL

	# Wait to reset the board
	writer = XmodemWriter()

	do_wait_reset = True

	if ConfigArgs.DTR_RESET:
		do_wait_reset = False
		bootrom_msg = writer.cancel_autoboot()

	if ConfigArgs.WAIT_RESET == False and do_wait_reset == True:
		rx = writer.recv()
		time.sleep(1)
		for i in range(3):
			writer.send("")
			rx = writer.recv()
			if NSH_MSG.encode() in rx:
				# No need to wait for reset
				do_wait_reset = False
				break
			time.sleep(1)

	if do_wait_reset:
		# Wait to reset the board
		print('Please press RESET button on target board')
		sys.stdout.flush()
		bootrom_msg = writer.cancel_autoboot()

	# Install files
	if ConfigArgs.PACKAGE_NAME:
		print(">>> Install files ...")
	if ConfigArgs.PACKAGE_NAME :
		writer.install_files(ConfigArgs.PACKAGE_NAME, "xmodem")
	print()

	return 0

if __name__ == "__main__":
	try:
		sys.exit(main())
	except KeyboardInterrupt:
		print("Canceled by keyboard interrupt.")
