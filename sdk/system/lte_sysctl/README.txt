system/lte_sysctl
^^^^^^^^^^^^^^^^^
******************************************************************************
* Description
******************************************************************************

  This is a modem control command for using LTE network.
  By using this command, you can control user socket daemon of LTE.

  Supported LTE modem is ALT1250.

******************************************************************************
* Build kernel and SDK
******************************************************************************

  $ ./tools/config.py feature/lte

    The web client also support HTTPS.
    HTTPS can be used by adding the mbed TLS configuration as follows:

    $ ./tools/config.py feature/lte feature/externals_mbedtls
      or                            ^^^^^^^^^^^^^^^^^^^^^^^^^
    $ ./tools/config.py feature/lte feature/lte_stub_mbedtls
                                    ^^^^^^^^^^^^^^^^^^^^^^^^

  <Setting APN>
  Set the APN of the carrier according to the your environment.

  $ tools/config.py -m
    Application Configuration -> Spresense SDK -> System tools
      [*] lte_sysctl
        Access Point Name
        IP type Selection
        Authentication type Selection
        Username used for authentication
        Password used for Authentication

  $ make

  NSH prompt can setting APN.
    type example for...
    nsh> lte_sysctl -a apn -t 202 -i 0 -v 2 -u user -p password start

  After that, if ifup command is issued,
  it connects to the lte network with the set your APN.

******************************************************************************
* Execute Example
******************************************************************************

  Execute under nsh:
    Type 'lte_sysctl <start/stop>' on nsh like this.
    nsh> lte_sysctl start
      or
    nsh> lte_sysctl stop

