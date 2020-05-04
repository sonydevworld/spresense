############################################################################
# examples/multi_webcamera/host/test_module/TestServer.py
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
# from threading import Thread as Mth
from multiprocessing import Process as Mth
import struct
import signal
import sys
import traceback
import time

class TestServer(object):
    def __init__(self):
        self.server = None
        self.client_sock = None

        image_files = ['test_module/001.jpg',
                       'test_module/002.jpg',
                       'test_module/003.jpg',
                       'test_module/004.jpg',
                       'test_module/005.jpg',
                       'test_module/006.jpg',
                       'test_module/007.jpg']
        self.images = [None, None, None, None, None, None, None]
        for i in range(len(image_files)):
            f = open(image_files[i], 'rb')
            self.images[i] = f.read()
            f.close()

        self.imgid=0

        # signal.signal(signal.SIGINT, self.sigIntHandler)

    def sigIntHandler(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def startServer(self, port):
        thread_hdr = Mth(
            target=self.serverThread,
            args=(port,self.images,self.imgid, )
        )
        thread_hdr.start()
        self.imgid += 1

    def serverThread(self, port, images, my_id):
        bind_ip = '0.0.0.0'
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((bind_ip, port))
        self.server.listen(1)

        img_id = 0

        try:
            self.client_sock, address = self.server.accept()
        except:
            print "Exception is occured.."
            return

        print 'Accepted connection from {}:{}'.format(address[0], address[1])

        while True:
            print "SVR " + str(my_id) + " " + str(port) + ": id=" + str(img_id) + " SZ:" + str(len(images[img_id]))
            self.client_sock.send('SZ: ')
            self.client_sock.send(struct.pack('<I', len(images[img_id])))
            self.client_sock.send(images[img_id])
            time.sleep(0.5)
            img_id += 1
            if img_id >= len(images):
                img_id = 0

        return
