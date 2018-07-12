examplex/gnss
^^^^^^^^^^^^^^^^^^^

  Example for GNSS positioning notification.

  Repeat Hot Start and Stop of positioning five times automatically.
  In the first positioning, positioning is continued for 200 seconds
  after FIX position, accumulating ephemeris. For the second to fifth
  iterations, it will perform positioning for 10 seconds.
  During positioning, notification is made from gnssfw every period.
  Reception of positioning notification can be chosen by poll or by signal.
  In both reception methods, position information is read from
  the GNSS device by read processing.
 
  The following configuration options can be selected:

  CONFIG_EXAMPLES_GNSS -- GNSS positioning example. Default: n
  CONFIG_EXAMPLES_GNSS_PROGNAME -- You can choice other name for
    this program if change this field. Default: "gnss"
  CONFIG_EXAMPLES_GNSS_PRIORITY -- Specified this example's task priority.
	  Default: 100
  CONFIG_EXAMPLES_GNSS_STACKSIZE -- Specified this example's stack size.
	  Default: 2048
  CONFIGE_EXAMPLES_GNSS_USE_POLL -- Choice poll method event notification.
    Default: CONFIGE_EXAMPLES_GNSS_USE_POLL
  CONFIGE_EXAMPLES_GNSS_USE_SIGNAL -- Choice signal method event notification
  
  In addition to the above, the following definitions are required:
    CONFIG_CXD56_GNSS
    CONFIG_EXAMPLES_GNSS
