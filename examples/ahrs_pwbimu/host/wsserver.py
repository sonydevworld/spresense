#############################################################################
# examples/ahrs/host/wssprserver.py
#
#   Copyright 2025 Sony Semiconductor Solutions Corporation
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
#############################################################################


from websocket_server import WebsocketServer
import logging
import threading
import time
import serial
import json
import struct
import binascii
import argparse

DEV_NAME = "/dev/ttyUSB0"
SVR_PORT = 8080

class SprSerial(serial.Serial):
  def __init__(self, devname):
    super().__init__(devname)
    self.baudrate = 115200
    self.timeout  = None
    self.bytesize = serial.EIGHTBITS
    self.parity   = serial.PARITY_NONE
    self.stopbits = serial.STOPBITS_ONE
    self.xonxoff  = 0
    self.rtscts   = 0

  def connect_dev(self, start_str, exec_cmd):
    print("Connecting device")
    print("  Reset Device")
    time.sleep(0.1)
    self.setDTR(False)
    self.setDTR(True)
    time.sleep(0.1)
    self.setDTR(False)

    if exec_cmd is not None:
      print("  Waiting for booting-up device")
      rdat = b""
      while start_str not in rdat:
        rdat = self.readline()
      time.sleep(0.1)
      print("  Exec AHRS app on Spresense ")
      self.write(exec_cmd)

    print("Done")

class SerialWebsockServer(WebsocketServer):

  def __init__(self, com, listen_url, port):
    super().__init__(port = port, host = listen_url)
    self.set_fn_new_client(self.connected)
    self.set_fn_client_left(self.disconnected)
    self.set_fn_message_received(self.data_received) 
    self.com = com

    self.is_running = True
    self.lock = threading.Lock()
    self.connected = False
    self.thd = threading.Thread(target = self.proc_sendposture)
    # self.thd = threading.Thread(target = self.proc_senddummy)
    self.thd.start()

  def convert_json(self, data):
    data = data.decode('utf-8')
    data = data.rstrip('\n')
    data = data.rstrip('\r')
    rpy = data.split(',')
    if len(rpy) == 4:
      try:
        q_w  = struct.unpack(">f", binascii.unhexlify(rpy[0]))[0]
        q_x  = struct.unpack(">f", binascii.unhexlify(rpy[1]))[0]
        q_y  = struct.unpack(">f", binascii.unhexlify(rpy[2]))[0]
        q_z  = struct.unpack(">f", binascii.unhexlify(rpy[3]))[0]
        return json.dumps({"q_w" : q_w, "q_x" : q_x, "q_y" : q_y, "q_z" : q_z})
      except Exception as e:
        print("   Non hexdigit : >" + str(rpy[0]) + \
              "< >" + str(rpy[1]) + "< >" + str(rpy[2]) +
              "< >" + str(rpy[3]) + "<")
        return None
    else:
      return None

  def proc_sendposture(self):
    while self.is_running:
      data = self.com.readline()
      data.strip()
      data = self.convert_json(data)

      if data is not None:
        with self.lock:
          if self.connected:
            self.send_message_to_all(data)

  def proc_senddummy(self):
    roll  = 0;
    pitch = 0;
    yaw   = 0;
    while self.is_running:
      time.sleep(0.01)
      data = json.dumps({"roll" : roll, "pitch" : pitch, "yaw" : yaw})
      yaw += 3;
      if yaw >= 360:
        yaw = 0

      if data is not None:
        with self.lock:
          if self.connected:
            self.send_message_to_all(data)

  def connected(self, client, server):
    with self.lock:
      self.connected = True

  def disconnected(self, client, server):
    with self.lock:
      self.connected = False

  def data_received(self, client, server, message):
    # Avoid receiving message
    pass
    
if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('-i', '--initialized', action='store_true', help='No Reset')
  parser.add_argument('-c', '--comm_port', dest="serial_port",
                      type=str, default=DEV_NAME, help='Serial Port')

  args = parser.parse_args()

  if args.initialized:
    start_str = b'Connected'
    exec_cmd = None
  else:
    start_str = b'NuttShell'
    exec_cmd = b'ahrs_pwbimu h\n'

  com = SprSerial(args.serial_port)
  com.connect_dev(start_str, exec_cmd)
  server = SerialWebsockServer(com, "0.0.0.0", SVR_PORT)
  server.run_forever()
