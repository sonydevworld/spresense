examples/camera
^^^^^^^^^^^^^^^

  This sample code gets an image from the image sensor.

  Supported image sensor is ISX012 and will work with the official Spresense camera board.


  This example can be used with the camera example default configuration

  $ ./tools/config.py examples/camera

  This example can also output captured images to an LCD directly.
  If you want to work with LCD devices, configure like this:

  $ ./tools/config.py examples/camera device/lcd

  And enable following option in /sdk/configs/device/camera-defconfig

  CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD=y
  
  Currently supports ili9340 and lpm013m091a based displays.
  
  Default resolution is 320x240.  
  This can be modified in the LCD driver code at /sdk/drivers/lcd/ili9340.c

  The LCD is expected to be connected on SPI4 (available on the Spresense expansion board)
  To change this, modify /sdk/configs/device/lcd-defconfig
  
  The example will capture images to an SD card if one is connected to the expansion board, otherwise it will save to flash memory.
  
  Execute under nsh:

  nsh> camera
  FILENAME:/mnt/spif/VIDEO001.JPG

  * Display to LCD

  nsh> camera
  nximage_initialize: Initializing LCD
  nximage_initialize: Open NX
  nximage_initialize: Screen resolution (320,240)
  FILENAME:/mnt/spif/VIDEO001.JPG
