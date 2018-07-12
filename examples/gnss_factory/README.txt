examples/gnss_factory
^^^^^^^^^^^^^^^^^^^^^

  Example for GNSS FACTORY test.

  Run the factory test.
  In the factory test, a signal from a satellite whose satellite number is known is received
  from a signal simulator etc., and its signal strength cn [dBHz] and doppler [Hz] are output to the debug port. 
  If there is a problem with the equipment, an error will be output.

  The factory test checks the results every second.
  When the signal can be received, the result is displayed and the test is terminated.

  Specify svid (satellite number) and execution method with kconfig (described later).
  These measurement results cn [dBHz] and doppler [Hz] are multiplied by 1000000 and output.
  When run from the console, svid can be set as an argument.

  The following configuration options can be selected:

  CONFIG_EXAMPLES_GNSS_FACTORY -- GNSS positioning example. Default: n
  CONFIG_EXAMPLES_GNSS_FACTORY_PROGNAME -- You can choice other name for
    this example if change this field. Default: "gnss_factory"
  CONFIG_EXAMPLES_GNSS_FACTORY_PRIORITY -- Specified this example's task priority.
      Default: 100
  CONFIG_EXAMPLES_GNSS_FACTORY_STACKSIZE -- Specified this example's stack size.
      Default: 2048
  CONFIG_EXAMPLES_GNSS_FACTORY_SVID -- Specified this example's svid.
      Default: 1
  
  In addition to the above, the following definitions are required:
    CONFIG_CXD56_GNSS
  To run the factory test directly, change 'Application entry point'

