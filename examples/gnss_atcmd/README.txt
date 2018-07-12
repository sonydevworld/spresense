examplex/gnss_atcmd
^^^^^^^^^^^^^^^^^^^

  Convert serial commands to IOCTL commands of GNSS device and output
  NMEA sentence from positioning data.

  Input serial commands starting with '@' character used in CXD5600 and
  CXD5602 from CDC/USB serial. Output NMEA sentences from CDC/USB serial.

  The following configuration options can be selected:

  CONFIG_EXAMPLES_GNSS_ATCMD -- Enable this GNSS CXD5603 @command emulator
    example. Default: n
  CONFIG_EXAMPLES_GNSS_ATCMD_PROGNAME -- You can choice other name for
    this program if change this field. Default: "gnss_atcmd"
  CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM -- Example supports spectrum output.
    Default: y
  CONFIG_EXAMPLES_GNSS_ATCMD_PRIORITY -- Specified this example's task priority.
	  Default: 100
  CONFIG_EXAMPLES_GNSS_ATCMD_STACKSIZE -- Specified this example's stack size.
	  Default: 2048

  Select command input TTY from the following:
    Default: CONFIG_EXAMPLES_GNSS_ATCMD_USB
  
  CONFIG_EXAMPLES_GNSS_ATCMD_USB -- USB CDC
  CONFIG_EXAMPLES_GNSS_ATCMD_STDINOUT -- stdin / stdout when nsh is enabled
  CONFIG_EXAMPLES_GNSS_ATCMD_TTYS0 -- ttyS0
  CONFIG_EXAMPLES_GNSS_ATCMD_TTYS1 -- ttyS1
  CONFIG_EXAMPLES_GNSS_ATCMD_TTYS2 -- ttyS2

  In addition to the above, the following definitions are required:
    CONFIG_CXD56_GNSS
    CONFIG_LIBM
    CONFIG_GPSUTILS_CXD56NMEA_LIB
    CONFIG_EXAMPLES_GNSS_ATCMD
    CONFIG_SYSTEM_CDCACM
