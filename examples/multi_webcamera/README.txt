examples/multi_webcamera
^^^^^^^^^^^^^^^^^^^^^^^^

  This sample code is an example application for using Spresense Camera Board and IDY iS110B Wi-Fi Add-on board.
  This application code has some essential programing contents.

  - How to use Spresense Camera.
  - How to use pthread on NuttX.
  - How to synchronize and handshaking a data between multi threads.
  - How to use socket interface.
  - What is Motion JPEG over HTTP.
  - How to make original protocol to send a data via TCP.

  I hope this application heplps you to understand above things.

  [Board environment]

    This sample assumes to work with below parts.

    Spresense Main Board
    Spresense Camera Board
    IDY iS110B Wi-Fi Add-on board.

  [Overview of this application]

    This application is a simple Wi-Fi camera. But there are 2 options.

    1. Simple Web Camera by using motion JPEG over HTTP.
        In this option, Spresense send a camera JPEG image to a web client.
        So you can access your Spresense board from any WebBrowser.

    2. Multiple Web Camera by using original protocol.
        In this option, Spresense send a camera JPEG image to a client by original protocol on TCP.
        In host/ directory, there is reference host programing by using Python.

  [How to build]

    Configure for this example by using default config.

      $ cd ../../sdk
      $ ./tools/config.py examples/multiwebcam

    In default, the option 1 (motion JPEG over HTTP) is selected.
    If you want to try option 2 (original protocol), you can configure it to select the option2.

      $ make menuconfig

    In the config menu, unset "Http MJPEG is used" it located in below.

      "Application Configuration" ->
        "Spresense SDK" ->
          "Examples" ->
            "Http MJPEG is used" in "Multi Web Camera"
 
    And also you can choose image size to send from below options in menu of "Multi Web Camera Image size".
    VGA is selected as default.

      QQVGA : 160 x 120
      QVGA  : 320 x 240
      VGA   : 640 x 480
      HD    : 1280 x 720

    If you changed any item on configuration menu, save it into .config file.

    After finishing configuration, type make to start build in sdk/ directory.

      $ make

    nuttx.spk is created in sdk/ directory if it succeeded.
    Then flash it into the device.
    Below example is a case of ubuntu CLI command to flash.
    In below example, USB connection with Spresense main board is /dev/ttyUSB0.

      $ ./tools/flash.sh -c /dev/ttyUSB0 nuttx.spk
      >>> Install files ...
      install -b 115200
      Install nuttx.spk
      |0%-----------------------------50%------------------------------100%|
      ######################################################################

      xxxxx bytes loaded.
      Package validation is OK.
      Saving package to "nuttx"
      updater# sync
      updater# Restarting the board ...
      reboot

  [How to work]

    * Motion JPEG camera mode:

      Access to device terminal. The below example is using "minicom" and device serail port is /dev/ttyUSB0.
      It should be read to fit the customer's environment.

        $ minicom -b 115200 -D /dev/ttyUSB0
        nsh> 

      At first, create Wi-Fi AP network you define and set SSID name and password as 10 digits.
      Below is an example of SSID=spresense_net password=0123456789

        nsh> gs2200m -a 1 spresense_net 0123456789 &

      Then you can see your ipaddress by ifconfig

        nsh> ifconfig
        eth0    Link encap:Ethernet HWaddr 3c:95:09:00:84:2a at UP
                inet addr:192.168.11.1 DRaddr:192.168.11.1 Mask:255.255.255.0

      And you execute the example command "multiwebcam" on nsh prompt.

        nsh> multiwebcam &

      That's all on the Spresense side.

      After that, your own device like PC or SmartPhone, attach to the AP network of "spresense_net".
      You can see camera image to http://192.168.11.1 from your browser.

    * Original protocol camera mode:

      In this mode, you can use up to 4 spresense camera at the same time.

      On the 1st device, create Wi-Fi AP network as the same as "Motion JPEG camera mode".

        nsh> gs2200m -a 1 spresense_net 0123456789 &
        nsh> ifconfig
        eth0    Link encap:Ethernet HWaddr 3c:95:09:00:84:2a at UP
                inet addr:192.168.11.1 DRaddr:192.168.11.1 Mask:255.255.255.0
        nsh> multiwebcam &

      On other devices, attach the AP network.

        nsh> gs2200m spresense_net 0123456789 &
        nsh> sleep 5
        nsh> ifconfig
        eth0    Link encap:Ethernet HWaddr 3c:95:09:00:84:2a at UP
                inet addr:192.168.11.X DRaddr:192.168.11.1 Mask:255.255.255.0
        nsh> multiwebcam &

      The "sleep 5" after gs2200m command, it is just waiting for SSID authentication etc..
      You need check each device's IP addresses to access it from PC tool.
      Basically, the IP address is incremented according to the connection order.

        192.168.11.2
        192.168.11.3
        192.168.11.4

      On PC side, sample tool is provided in spresense/examples/multi_webcamera/host.
      It is written in Python2.x with WxPython to display GUI.
      So you need to install smoe required pachages of python at first on you PC.
      Below example is in case of ubuntu 16.04.

        $ sudo  apt  install  python-wxgtk3.0  python-wxtools

      After that, attach the AP network as the same as other spresense boards.
      And then, execute MultiCameraFrame.py on your PC in host/ directory.

        $ python MultiCameraFrame.py

      You can see black full size window on your display and see images from spresense devices on the window.

      TIPs:
        If you want to change spresense IP address to access from PC, line 183 to 186 in MultiCameraFrame.py is setting IP addresses.
        You can change the addresses for your own environment.

