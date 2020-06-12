system/lte_daemon
^^^^^^^^^^^^^^^^^
******************************************************************************
* Description
******************************************************************************

  This is a daemon for using LTE network.
  When the daemon is started, it performs the search up to the LTE network.

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
      [*] lte_daemon
        Access Point Name
        IP type Selection
        Authentication type Selection
        Username used for authentication
        Password used for Authentication

  $ make

  NSH prompt can setting APN.
    type exmaple for...
    nsh> lte_daemon -a apn -t 202 -i 0 -v 2 -u user -p passward start

  After that, if ifup command is issued,
  it connects to the lte network with the set your APN.

******************************************************************************
* Execute Example
******************************************************************************

  Execute under nsh:
    Type 'lte_daemon <start/stop>' on nsh like this.
    nsh> lte_daemon start
      or
    nsh> lte_daemon stop

