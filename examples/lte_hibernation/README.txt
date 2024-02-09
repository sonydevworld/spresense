examples/lte_hibernation
^^^^^^^^^^^^^^^^^^^^^^^^
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
  $ make

******************************************************************************
* Execute Example
******************************************************************************

  Create the file /mnt/spif/init.rc. Refer to the following command to create
  the init.rc file.

  $ echo "lte_hibernation -a <apn_name> -i <ip_type> -t <auth_type> -u <user_name> -p <password> &" > /mnt/spif/init.rc

  The usage of lte_hibernation is as follows.

    USAGE: lte_hibernation command
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
