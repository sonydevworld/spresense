examples/camera
^^^^^^^^^^^^^^^

  This sample code gets an image from the image sensor.

  Supported image sensor is ISX012.


  This example can be used by camera default config.

  $ ./tools/config.py camera

  This example can be output captured images to LCD directly.
  If you want to work with LCD devices, type like this:

  $ ./tools/config.py camera ili9340

  And enable following options.

  CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD -- Show captured image on the LCD
  CONFIG_EXAMPLES_CAMERA_INFINITE   -- Capture infinitely

  Execute under nsh:

  nsh> camera
  FILENAME:/mnt/spif/VIDEO001.JPG

  * Display to LCD

  nsh> camera moni
  nximage_initialize: Initializing LCD
  nximage_initialize: Open NX
  nximage_initialize: Screen resolution (320,240)
  FILENAME:/mnt/spif/VIDEO001.JPG
