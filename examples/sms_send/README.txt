examples/sms_send
^^^^^^^^^^^^^^^^^

  This application is a sample for sending SMS.
  Insert an SMS capable SIM into your device. After connecting
  to the LTE network, run this application to send an SMS.

  Build SDK:

  This application can be used by sms_send default config.

  $ ./tools/config.py examples/sms_send
  $ make

  How to operate under nsh:

  nsh> lte_sysctl start
  nsh> ifup eth0
  nsh> sms_send <phone number> <text msg> [<enable status report>]

  For more details on lte_sysctl commands,
  see sdk/system/lte_sysctl/README.txt
