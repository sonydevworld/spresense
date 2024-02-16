examples/lte_hibernation_wake_socket
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
******************************************************************************
* Description
******************************************************************************

  This application is a sample that shows how Spresense works when it is
  put into CodSleep while maintaining an LTE connection.
  Spresense can enter ColdSleep while maintaining the LTE connection and
  also the connection to the server. Spresense is able to wake up and receive
  data from the server when it arrives.

  It can be used as a reference for this sample to make a program that wakes
  up Spresense by notification from the server.

  Supported LTE modem is ALT1250.

******************************************************************************
* Build kernel and SDK
******************************************************************************

  $ ./tools/config.py examples/lte_hibernation_wake_socket

    <Setting APN>
    Set the APN of the carrier according to the your environment.

    $ tools/config.py -m
      Application Configuration
        Spresense SDK
          Example
            [*] Example of waking up from LTE hibernation by socket
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
