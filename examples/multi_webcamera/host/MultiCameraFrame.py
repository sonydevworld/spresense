############################################################################
# examples/multi_webcamera/host/MultiCameraFrame.py
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

import os
import wx
import wx.lib.newevent
from ImgScaler import scaleJpeg2Bitmap
from NetImgReceiver import NetImgReceiver
from threading import Lock

DEBUG_MODE = False
FULLSCREEN = False

def CalcLayout(dispsize, imgsize=(427,320), basic_size=(1440,900)):
    img_ratios = (float(imgsize[0]) / float(basic_size[0]), float(imgsize[1]) / float(basic_size[1]))
    rest_size = ((float(basic_size[0]) / 2.) - float(imgsize[0])) / 2. / float(basic_size[0]), ((float(basic_size[1]) / 2.) - float(imgsize[1])) / 2. / float(basic_size[1])
    
    img_size_x = int( img_ratios[0] * dispsize[0] )
    img_size_y = int( img_ratios[1] * dispsize[1] )

    disp1_x = int( rest_size[0] * dispsize[0] )
    disp1_y = int( rest_size[1] * dispsize[1] )
    disp2_x = disp1_x * 3 + img_size_x
    disp2_y = disp1_y
    disp3_x = disp1_x
    disp3_y = disp1_y * 3 + img_size_y
    disp4_x = disp2_x
    disp4_y = disp3_y

    return ( (disp1_x, disp1_y), (disp2_x, disp2_y), (disp3_x, disp3_y), (disp4_x, disp4_y) ), (img_size_x, img_size_y)
    
def OldCalcLayout(dispsize, imgsize):
    disp1_x = int( (float(dispsize[0]) / 2. - float(imgsize[0])) / 2. )
    disp1_y = int( (float(dispsize[1]) / 2. - float(imgsize[1])) / 2. )
    disp2_x = disp1_x * 3 + imgsize[0]
    disp2_y = disp1_y
    disp3_x = disp1_x
    disp3_y = disp1_y * 3 + imgsize[1]
    disp4_x = disp2_x
    disp4_y = disp3_y
    return ( (disp1_x, disp1_y), (disp2_x, disp2_y), (disp3_x, disp3_y), (disp4_x, disp4_y) )

class CameraServerRapper(object):
    def __init__(self, bmp_panel):
        self.bmp_panel = bmp_panel

    def setReceiver(self, receiver):
        self.receiver = receiver

(PictUpdateEvent, EVT_UPDATE_PICTURE) = wx.lib.newevent.NewEvent()

class MultiCamFrame(wx.Frame):
    def __init__(self, servers, withoutCurs=True):
        wx.Frame.__init__(self, None, wx.ID_ANY, 'SPRESENSE Demo [Multi WebCamera]', style=wx.DEFAULT_FRAME_STYLE)

        if withoutCurs :
            cursor = wx.StockCursor(wx.CURSOR_BLANK)
            self.SetCursor(cursor)

        self.main_field = WebCamPanel(self, servers)

class WebCamPanel(wx.Panel):
    def __init__(self, parent, servers):
        wx.Panel.__init__(self, parent)
        self.SetBackgroundColour('Black')
        self.SetBackgroundStyle(wx.BG_STYLE_PAINT)

        self.mutex = Lock()

        self.servers = servers

        self.max_cameras = 4
        self.calcActualServerNum()

        self.imgpos, self.imgsize = CalcLayout(wx.DisplaySize())
        print wx.DisplaySize()
        print self.imgpos
        print self.imgsize

        self.Bind(wx.EVT_CHAR, self.onKeyDown)
        self.Bind(EVT_UPDATE_PICTURE, self.onPictUpdate)
        self.Bind(wx.EVT_IDLE, self.onIdle)

        self.addEmptyImage()

        self.trial_connection = 0

    def calcActualServerNum(self):
        if self.servers is not None:
            ipnum = len(self.servers)
            self.actual_server_num = self.max_cameras if (ipnum >= self.max_cameras) else ipnum
        else:
            self.actual_server_num = 0

    def onIdle(self, event):
        if self.trial_connection < self.actual_server_num:
            (ip, port) = self.servers[self.trial_connection]
            print "Start Client Connection " + ip + ":" + str(port)
            rcver = NetImgReceiver(self, ip, port, self.trial_connection)
            self.bmppanels[self.trial_connection].setReceiver(rcver)
            rcver.startReceiving()
            self.trial_connection += 1

    def addEmptyImage(self):
        self.bmppanels = [None, None, None, None]
        for i in range(self.max_cameras):
            print "Create bmppanels " + str(i)
            img = wx.EmptyImage(width=self.imgsize[0], height=self.imgsize[1], clear=True)
            self.bmppanels[i] = CameraServerRapper( wx.StaticBitmap(self, -1, wx.BitmapFromImage(img), pos=self.imgpos[i]) )

    def onPictUpdate(self, event):
            # try:
            bmp = scaleJpeg2Bitmap(event.jpeg, self.imgsize[0], self.imgsize[1])
            # print "onPictUpdate:  Server=" + event.jpeg_server + " ID = " + str(event.my_id)
            if bmp is not None:
                self.bmppanels[event.my_id].bmp_panel.SetBitmap(bmp)
                self.Refresh()
            # except:
            # pass

    def updateJpeg(self, jpg_data, server_ip, my_id):
        self.mutex.acquire()
        evt = PictUpdateEvent(jpeg=jpg_data, jpeg_server=server_ip, my_id=my_id)
        wx.PostEvent(self, evt)
        self.mutex.release()

    def onKeyDown(self, event):
        if event.GetKeyCode() == wx.WXK_ESCAPE:
            os.kill(os.getpid(), 9)

if __name__ == "__main__":
    if DEBUG_MODE is True:
        import test_module
        import time
        import os
        import socket

        bind_port = 10080
        servers = ( ('localhost', bind_port),
                    ('localhost', bind_port+1),
                    ('localhost', bind_port+2),
                    ('localhost', bind_port+3) )

        print "Start Server"
        svr = test_module.TestServer()
        for ip, port in servers:
            svr.startServer(port)

        time.sleep(0.3)

    else:
        servers = ( ('192.168.11.1', 10080),
                    ('192.168.11.2', 10080),
                    ('192.168.11.3', 10080),
                    ('192.168.11.4', 10080) )

    app = wx.App()
    frame = MultiCamFrame(servers, False)
    if FULLSCREEN:
        frame.ShowFullScreen(True)
    else:
        frame.Show()
        frame.Maximize()
    app.MainLoop()
