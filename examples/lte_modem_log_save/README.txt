examples/lte_modem_log_save
^^^^^^^^^^^^^^^^^^^^^^^^^^^

******************************************************************************
* Description
******************************************************************************

  This application is a sample that saves modem FW logs to the modem.

  Supported LTE modem is ALT1250.

******************************************************************************
* Build kernel and SDK
******************************************************************************

  $ tools/config.py examples/lte_modem_log_save

  <When changing the log file saved directory>
    $ tools/config.py -m
      Application Configuration
        Spresense SDK
          Example
            [*] Save modem FW logs example
  $ make

******************************************************************************
* Execute Example
******************************************************************************

  Execute under nsh:

  Type 'lte_modem_log_save' on nsh.

  nsh> lte_modem_log_save

