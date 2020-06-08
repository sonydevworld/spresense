examples/camera
^^^^^^^^^^^^^^^

  This sample code is an example for using Spresense Camera Board and displaying on LCD.

  [Board environment]

    This sample assumes to work with below patrs.

      Spresense Main Board
      Spresense Extention Board
      Spresense Camera Board
      Arduino UNO LCD Connector Board
      ILI9341 2.2inch LCD

    You can select without LCD. In that case, minimum required parts are below.

      Spresense Main Board
      Spresense Camera Board

  [How to build]

    Configure for this example by using default config.

    $ cd ../../sdk
    $ ./tools/config.py examples/camera

    This configuration is using ILI9341 LCD display.
    If you don't want to use it.
    You can connfigure by using menuconfig.

    $ make menuconfig

    And uncheck "Output LCD" setting located in below.

      "Application Configuration" ->
        "Spresense SDK" ->
          "Examples" ->
            "Output LCD" in "Camera example"

    After finishing configuration, type make to start build.

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

    This sample is implemented as 'camera' command on NuttXShell.
    The synopsys of the command is as below.

    nsh> camera ([-jpg]) ([capture num])

      -jpg        : this option is set for storing JPEG file into a strage.
                  : If this option isn't set capturing raw YUV422 data in a file.
                  : raw YUV422 is default.

      capture num : this option instructs number of taking pictures.
                  : If it sets '0', it runs as just displaying preview image on LCD eternally. 
                  : 10 is default.
    
    Storage will be selected automatically.
    If you put microSD card on Extension board, storage to sotre files is selected microSD card.
    If no microSD card is found, the storage is selected as SPI Flash.
    
    Execution example:

      nsh> camera
      nximage_listener: Connected
      nximage_initialize: Screen resolution (320,240)
      Take 10 pictures as YUV file in /mnt/sd0 after 5000 mili-seconds.
      After finishing taking pictures, this app will be finished after 10000 mili-seconds.
      Expier time is pasted.
      Start captureing...
      FILENAME:/mnt/sd0/VIDEO001.YUV
      FILENAME:/mnt/sd0/VIDEO002.YUV
      FILENAME:/mnt/sd0/VIDEO003.YUV
      FILENAME:/mnt/sd0/VIDEO004.YUV
      FILENAME:/mnt/sd0/VIDEO005.YUV
      FILENAME:/mnt/sd0/VIDEO006.YUV
      FILENAME:/mnt/sd0/VIDEO007.YUV
      FILENAME:/mnt/sd0/VIDEO008.YUV
      FILENAME:/mnt/sd0/VIDEO009.YUV
      FILENAME:/mnt/sd0/VIDEO010.YUV
      Finished captureing...
      Expier time is pasted.
      nximage_listener: Lost server connection: 117

      nsh> camera 0
      nximage_initialize: Screen resolution (320,240)
      Start video this mode is eternal. (Non stop, non save files.)

