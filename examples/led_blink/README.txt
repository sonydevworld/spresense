examples/led_blink
^^^^^^^^^^^^^^^^^^

  This sample code is an example to show how to use LEDs on Spresense Main board.

  [Overview of this application]

    This application shows simple usage for turning on/off LEDs on Spresense Main board.

  [How to build]

    Configure for this example by using default config.

      $ cd ../../sdk
      $ ./tools/config.py examples/led_blink

    Type make to start build in sdk/ directory.

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

    Access to device terminal. The below example is using "minicom" and device serail port is /dev/ttyUSB0.
    It should be read to fit the customer's environment.

      $ sdk/tools/linux/miniterm --raw --eol LF /dev/ttyUSB0
      nsh> 

    Execute the example command "led_blink" on nsh prompt.

      nsh> led_blink

    Then, you can see moving LED lights.
