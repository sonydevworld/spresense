############################################################################
# examples/multi_webcamera/host/NetImgReceiver.py
#
#   Copyright 2019, 2020 Sony Semiconductor Solutions Corporation
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
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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

import socket
from threading import Thread as Mth
# from multiprocessing import Process as Mth
import struct
import signal
import sys
import traceback

class NetImgReceiver(object):
    INDICATOR_CHAR = r'SZ: '

    def __init__(self, panel, server_ip, port, my_id):
        self.panel = panel
        self.server_ip = server_ip
        self.port = port
        self.is_run = True
        self.my_id = my_id

        # signal.signal(signal.SIGINT, self.sigIntHandler)

    def sigIntHandler(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def startReceiving(self):
        self.thread_hdr = Mth(
            target=self.receiveThread,
        )
        self.thread_hdr.start()

    def receiveThread(self):
        
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            server_sock.connect((self.server_ip, self.port))
        except:
            print self.server_ip + ":" + str(self.port) + " is not respond.."
            server_sock.close()
            return
        print "NetImgReceiver: " + self.server_ip + ":" + str(self.port) + " is connected."

        while self.is_run:

            indicator = server_sock.recv(4)
            if indicator != NetImgReceiver.INDICATOR_CHAR :
                continue

            jpgsize_bytes = server_sock.recv(4)
            jpgsize = struct.unpack('<I', jpgsize_bytes)[0]
            # print "NetImgReceiver " + str(self.my_id) + ": jpgsize = " + str(jpgsize)

            jpg_data = ''
            jpg_data_len = 0

            retry_count = 0
            max_retry_count = 300

            while jpg_data_len < jpgsize:
                # print "NetImgReceiver " + str(self.my_id) + ": Receiving jpeg data"
                try:
                    tmp_jpg_data = server_sock.recv(jpgsize - jpg_data_len)
                    
                    if not self.is_run:
                        print "NetImgReceiver " + str(self.my_id) + ": unset is_run"
                        break

                    if len(tmp_jpg_data) == 0:
                        retry_count -= 1
                        if retry_count <= 0:
                            print "NetImgReceiver " + str(self.my_id) + ": Client did not send data.."
                            self.is_run = False
                            break
                    else:
                        retry_count = max_retry_count
                        jpg_data += tmp_jpg_data

                    jpg_data_len = len(jpg_data)

                except:
                    print "NetImgReceiver " + str(self.my_id) + ": Client socket is gone..."
                    traceback.print_exc()
                    self.is_run = False
                    break

                if jpg_data_len == jpgsize:
                    if self.panel is not None:
                        fname = self.panel.updateJpeg(jpg_data, self.server_ip, self.my_id)

        print "NetImgReceiver " + str(self.my_id) + ": Closing server_sock"
        server_sock.close()

        return

if __name__ == "__main__":
    import test_module
    import time
    import os

    bind_port = 10080

    print "Start Server"
    svr = test_module.TestServer()
    svr.startServer(bind_port)

    time.sleep(0.3)

    print "Connect to Server"
    rcver = NetImgReceiver(None, 'localhost', bind_port, 0)
    rcver.startReceiving()

    time.sleep(10)

    os.kill(os.getpid(), 9)
