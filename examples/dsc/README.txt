examples/dsc
^^^^^^^^^^^^^^^^^^^^^^^^

  This sample code is an example application for using Spresense Camera Board.
  This is "Digital Still Camera(DSC)"-like application.

  [Board environment]

    This sample assumes to work with below parts.

    Spresense Main Board
    Spresense Camera Board
    LCD+2SW kit for SPRESENSE (https://www.switch-science.com/catalog/7982/)

  [How to build]

    Configure for this example by using default config.

      $ cd ../../sdk
      $ ./tools/config.py examples/dsc
      $ make

    nuttx.spk is created in sdk/ directory if the "make" succeeded.
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

    Connect to the serial terminal to enter the NuttShell prompt.
    Input "dsc" after nsh> prompt and press enter key.

      nsh> dsc

    Then, the application started.
    You can see the camera image on your LCD.

    The "LCD+2SW kit for SPRESENSE" has 2 buttons.
    On this application, one button is for Shutter or Select button.
    And the other is for menu or go-down button.

    Just after booting up, the application is in a shooter mode.
    In this mode, Shutter/Select button acts as a Shutter button.
    When you pressed this button, picture will be taken and save it
    into microSD card. So, to take a picture, you need microSD card.

    In the shooter mode, menu/go-down button acts as a menu button.
    When you pressed this button, the mode will change to "menu" mode.

    In the menu mode, menu/go-down button acts as a go-down button.
    You can walk around several menu by pressing the button.
    And the Shutter/Select button will act as a Select button.
    The menu item you cursored, can be executed the action when the
    Shutter/Select button pressed.

    To go back the shooter mode, go-down until "ExitMenu", and press
    Shutter/Select button. You can go back to shooter mode, then.

  [Auto booting up]

    This applicatin default configuration includes "startup_script".
    Enter the below command on nsh> prompt, and the auto boot-up is
    enabled.

      nsh> echo dsc > /mnt/spif/init.rc

    If you stop auto boot-up, delete the init.rc file like below command.

      nsh> rm /mnt/spif/init.rc
