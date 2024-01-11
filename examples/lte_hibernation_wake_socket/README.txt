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
  $ make

******************************************************************************
* Install certificate file
******************************************************************************

  This example performs an HTTP GET to https://httpbin.org/delay/10.
  Therefore, please install the server certificate file in /mnt/sd0/CERTS
  beforehand. The installation directory can be changed in
  CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_CERTS_PATH.

******************************************************************************
* Execute Example
******************************************************************************

  Create the file /mnt/spif/init.rc. Refer to the following command to create
  the init.rc file.

  $ echo "lte_hibernation_wake_socket -a <apn_name> -i <ip_type> -t <auth_type> -u <user_name> -p <password>" > /mnt/spif/init.rc

  The usage of lte_hibernation_wake_socket is as follows.

    USAGE: lte_hibernation_wake_socket command
     [-a <apn_name>] [-i <ip_type>] [-t <auth_type>] [-u <user_name>] [-p <password>]
      -a: APN name
      -i: IP type : v4 or v6 or v4v6
      -t: Authenticaion type : none or pap or chap
      -u: User name for authenticaion
      -p: Password for authenticaion
     [-h]: Show this message

  Once /mnt/spif/init.rc is created, this application will start automatically
  when you reset Spresense. Therefore, please use the following command to
  reset Spresense. Or press the reset button on the Spresense main board.

  $ reboot
