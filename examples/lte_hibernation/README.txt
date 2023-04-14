examples/lte_hibernation
^^^^^^^^^^^^^^^^^^^^^
******************************************************************************
* Description
******************************************************************************

  This application is a sample that shows how Spresense works when it is
  put into CodSleep while maintaining an LTE connection.
  Spresense goes into ColdSleep while maintaining an LTE connection and
  periodically starts up and downloads the HTML of a web page with Wget.

  It can be used as a reference to create a program with reduced power
  consumption by changing the program for the part that executes Wget.

  Supported LTE modem is ALT1250.

******************************************************************************
* Build kernel and SDK
******************************************************************************

  $ ./tools/config.py examples/lte_hibernation

    <Setting APN>
    Set the APN of the carrier according to the your environment.

    $ tools/config.py -m
      Application Configuration
        Spresense SDK
          Example
            [*] LTE Hibernation sample application
              Access Point Name
              IP type Selection
              Authentication type Selection
              Username used for authentication
              Password used for authentication

  $ make

******************************************************************************
* Execute Example
******************************************************************************

  This sample application will run automatically without the need to execute
  commands from NuttShell.
  So, it will be executed simply by turning on the Spresense.
