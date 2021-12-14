examples/sms_recv
^^^^^^^^^^^^^^^^^

  This application is a sample for receiving SMS.
  Insert an SMS capable SIM into your device. After connecting
  to the LTE network, run this application and send an SMS to
  the phone number associated with the SIM. The received SMS
  message will be output to the console.

  Build SDK:

  This application can be used by sms_recv default config.

  $ ./tools/config.py examples/sms_recv
  $ make

  How to operate under nsh:

  nsh> lte_sysctl start
  nsh> ifup eth0
  nsh> sms_recv &

  For more details on lte_sysctl commands,
  see sdk/system/lte_sysctl/README.txt
